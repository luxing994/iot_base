/*
 * 设备轮询
 *
 *   plc_poll_task       串口 PLC / 测试仪 / Modbus RTU，每秒轮询一次
 *   PlcPoll_xxxNet()    网口 PLC，由 tcp_client.c 中的连接任务每秒调用一次
 *
 * 读哪些寄存器由点表决定（plc_points.c，按 menuconfig 选择），这里只负责按表逐点执行：
 * 把点表类型翻译成协议层的读函数，读到值后交给 plc_report.c 上报。
 * 串口读函数内部是一次同步事务（见 serial_txn.h）；网口读函数返回 PLC_ERR_LINK 时
 * 结束本轮，由调用方重连。
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "iot_common.h"
#include "plc_poll.h"
#include "plc_points.h"
#include "plc_report.h"
#include "plc_value.h"
#include "host_cmd.h"
#include "serial_txn.h"
#include "s7_area.h"
#include "fx_plc_protocol.h"
#include "hl_plc_protocol.h"
#include "ppi_plc_protocol.h"
#include "ls_plc_load_protocol.h"
#include "t_tester_protocol.h"
#include "ainuo_tester_ascii_protocol.h"
#include "lx_modbus_tcp_master.h"
#include "lx_s7comm.h"
#include "master.h"

static const char *TAG = "PLC_POLL";

#define POLL_INTERVAL_MS    1000

/* 网口每读一个点之后的间隔（沿用原实现） */
#define FX_NET_GAP_MS       200
#define S7_NET_GAP_MS       100
#define MODBUS_NET_GAP_MS   300

/* DBS 真空计只有一个读数，沿用原实现上报为 FR000 */
#define DBS_FRNUM           0

typedef int (*PointReader)(const PlcPoint *p, PlcValue *v);
typedef int (*NetPointReader)(int sock, const PlcPoint *p, PlcValue *v);
typedef void (*PointReporter)(const PlcPoint *p, const PlcValue *v);

static __attribute__((unused)) void LogUnsupported(const char *protocol, const PlcPoint *p)
{
    ESP_LOGW(TAG, "%s: point type %d not supported on this connection (FR%03d %s)",
             protocol, p->type, p->frnum, p->name);
}

/*
 * 串口：逐点读取并上报，读失败的点跳过。
 * 连续 SERIAL_OFFLINE_FAILS 个点无应答时认为设备离线，结束本轮：
 * 否则每个点都要等满超时（如 30 个点 x 500ms = 15s），串口锁长时间被占用，
 * 上位机下发的写命令也要等这么久才能执行。
 */
#define SERIAL_OFFLINE_FAILS    3

static __attribute__((unused)) void PollTable(const char *protocol, const PlcPointTable *table,
                                              PointReader read, PointReporter report)
{
    int fails = 0;

    for (size_t i = 0; i < table->count; i++) {
        const PlcPoint *p = &table->points[i];
        PlcValue v;
        int ret = read(p, &v);

        if (ret == PLC_OK) {
            fails = 0;
            report(p, &v);
        } else if (ret == PLC_ERR_UNSUPPORTED) {
            LogUnsupported(protocol, p);
        } else if (++fails >= SERIAL_OFFLINE_FAILS && i + 1 < table->count) {
            // 设备离线由 serial_txn 统一告警（只在状态变化时打印），这里只是缩短本轮
            ESP_LOGD(TAG, "%s: no response for %d points, skip the rest of this round", protocol, fails);
            return;
        }
    }
}

/* 网口：逐点读取并上报，链路失效时返回 -1 */
static __attribute__((unused)) int PollNetTable(const char *protocol, int sock, const PlcPointTable *table,
                                                NetPointReader read, PointReporter report, int gap_ms)
{
    for (size_t i = 0; i < table->count; i++) {
        const PlcPoint *p = &table->points[i];
        PlcValue v;
        int ret = read(sock, p, &v);

        if (ret == PLC_ERR_LINK) {
            return -1;
        }
        if (ret == PLC_OK) {
            report(p, &v);
        } else if (ret == PLC_ERR_UNSUPPORTED) {
            LogUnsupported(protocol, p);
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(gap_ms));
    }
    return 0;
}

/* ============================================================ 各协议的点表翻译 */

#if defined(CONFIG_PLC_FX)
static void FxReport(const PlcPoint *p, const PlcValue *v)
{
    PlcReport_Fx(p->frnum, v);
}
#endif

#if defined(CONFIG_PLC_FX_CONNECT_RS232) || defined(CONFIG_PLC_FX_CONNECT_RS485)
static int FxSerialRead(const PlcPoint *p, PlcValue *v)
{
    switch (p->type) {
#if defined(CONFIG_PLC_FX_CONNECT_RS232)
    case FX_D16:    return FX_ProgReadD(p->address, v);         // 编程口
#else
    case FX_D16:    return FX_LinkRead(FX_DEV_D, p->address, FX_FMT_U16, v);   // 计算机链接
    case FX_D32:    return FX_LinkRead(FX_DEV_D, p->address, FX_FMT_I32, v);
    case FX_DFLOAT: return FX_LinkRead(FX_DEV_D, p->address, FX_FMT_FLOAT, v);
    case FX_Y:      return FX_LinkRead(FX_DEV_Y, p->address, FX_FMT_BIT, v);
    case FX_M:      return FX_LinkRead(FX_DEV_M, p->address, FX_FMT_BIT, v);
#endif
    default:        return PLC_ERR_UNSUPPORTED;
    }
}
#endif

#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
static int FxNetRead(int sock, const PlcPoint *p, PlcValue *v)
{
    switch (p->type) {
    case FX_D16:    return FX_NetRead(sock, FX_DEV_D, p->address, FX_FMT_U16, v);
    case FX_D32:    return FX_NetRead(sock, FX_DEV_D, p->address, FX_FMT_I32, v);
    case FX_DFLOAT: return FX_NetRead(sock, FX_DEV_D, p->address, FX_FMT_FLOAT, v);
    case FX_Y:      return FX_NetRead(sock, FX_DEV_Y, p->address, FX_FMT_BIT, v);
    case FX_M:      return FX_NetRead(sock, FX_DEV_M, p->address, FX_FMT_BIT, v);
    default:        return PLC_ERR_UNSUPPORTED;
    }
}
#endif

#if defined(CONFIG_PLC_HOSTLINK)
static int HostLinkRead(const PlcPoint *p, PlcValue *v)
{
    switch (p->type) {
    case HL_DM:       return HL_ReadDM(p->address, HL_FMT_U16, v);
    case HL_DM_BCD:   return HL_ReadDM(p->address, HL_FMT_BCD, v);
    case HL_DM_FLOAT: return HL_ReadDM(p->address, HL_FMT_FLOAT, v);
    case HL_CIO_BIT:  return HL_ReadCIOBit(p->address, p->bit, v);
    default:          return PLC_ERR_UNSUPPORTED;
    }
}

static void HostLinkReport(const PlcPoint *p, const PlcValue *v)
{
    PlcReport_HostLink(p->frnum, v, p->type == HL_CIO_BIT);
}
#endif

#if defined(CONFIG_PLC_PPI)
// 点表类型 -> S7 变量（PPI 串口和 S7 网口共用）
static int S7VarFromPoint(const PlcPoint *p, S7Var *var)
{
    var->byte = p->address;
    var->bit = p->bit;
    switch (p->type) {
    case S7_VB:    var->area = S7_AREA_V; var->type = S7_BYTE; break;
    case S7_VW:    var->area = S7_AREA_V; var->type = S7_INT;  break;
    case S7_VD:    var->area = S7_AREA_V; var->type = S7_DINT; break;
    case S7_VREAL: var->area = S7_AREA_V; var->type = S7_REAL; break;
    case S7_V_BIT: var->area = S7_AREA_V; var->type = S7_BIT;  break;
    case S7_Q_BIT: var->area = S7_AREA_Q; var->type = S7_BIT;  break;
    case S7_I_BIT: var->area = S7_AREA_I; var->type = S7_BIT;  break;
    case S7_M_BIT: var->area = S7_AREA_M; var->type = S7_BIT;  break;
    default:       return -1;
    }
    return 0;
}

static void PpiReport(const PlcPoint *p, const PlcValue *v)
{
    PlcReport_Ppi(p->frnum, v);
}
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_RS485)
static int PpiSerialRead(const PlcPoint *p, PlcValue *v)
{
    S7Var var;

    if (S7VarFromPoint(p, &var) != 0) {
        return PLC_ERR_UNSUPPORTED;
    }
    return PPI_Read(&var, v);
}
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
static int S7NetRead(int sock, const PlcPoint *p, PlcValue *v)
{
    S7Var var;

    if (S7VarFromPoint(p, &var) != 0) {
        return PLC_ERR_UNSUPPORTED;
    }
    return S7_NetRead(sock, &var, v);
}
#endif

#if defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
static int ModbusNetRead(int sock, const PlcPoint *p, PlcValue *v)
{
    switch (p->type) {
    case MB_HR16:    return MODBUS_NetRead(sock, p->address, MODBUS_FMT_U16, v);
    case MB_HR32:    return MODBUS_NetRead(sock, p->address, MODBUS_FMT_I32, v);
    case MB_HRFLOAT: return MODBUS_NetRead(sock, p->address, MODBUS_FMT_FLOAT, v);
    default:         return PLC_ERR_UNSUPPORTED;
    }
}

static void ModbusNetReport(const PlcPoint *p, const PlcValue *v)
{
    PlcReport_ModbusNet(p->frnum, v);
}
#endif

/* ============================================================ 串口轮询任务 */

void plc_poll_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(POLL_INTERVAL_MS));
        if (!HostCmd_IsCollectEnabled()) {
            continue;   // 上位机发送 EndFlush 后暂停采集，StartFlush 后恢复
        }

#if defined(CONFIG_PLC_FX_CONNECT_RS232) || defined(CONFIG_PLC_FX_CONNECT_RS485)
        PollTable("FX", &g_fxPoints, FxSerialRead, FxReport);
#endif

#if defined(CONFIG_PLC_HOSTLINK)
        PollTable("HostLink", &g_hostLinkPoints, HostLinkRead, HostLinkReport);
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_RS485)
        PollTable("PPI", &g_ppiPoints, PpiSerialRead, PpiReport);
#endif

#if defined(CONFIG_PLC_LS_LOAD)
        {
            PlcValue v;
            if (DBS_Read(&v) == PLC_OK) {
                PlcReport_Ls(DBS_FRNUM, &v);
            }
        }
#endif

#if defined(CONFIG_TESTER_76T)
        TTester_ReadGroup((uint16_t)HostCmd_GetTesterGroup());   // 0 读当前组，否则读上位机指定组的历史结果
#endif

#if defined(CONFIG_TESTER_AINUO)
        AINUO_ReadGroup();
#endif

#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
        master_operation_func(NULL);    // Modbus RTU 由 esp-modbus 按 master.c 中的参数表读取
#endif
    }
}

#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
// 上位机 PowerOn / PowerOff：Modbus RTU 空开合闸 / 分闸
void send_command_task(void *pvParameters)
{
    while (1) {
        EventBits_t uxBits = xEventGroupWaitBits(xEventGroup3, BIT_0 | BIT_1, pdTRUE, pdFALSE, portMAX_DELAY);
        if (uxBits & BIT_0) {
            master_send_switch_func(1);
        } else if (uxBits & BIT_1) {
            master_send_switch_func(0);
        }
    }
}
#endif

/* ============================================================ 网口 */

#if defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
int PlcPoll_ModbusNet(int sock)
{
    return PollNetTable("Modbus TCP", sock, &g_modbusNetPoints, ModbusNetRead, ModbusNetReport,
                        MODBUS_NET_GAP_MS);
}
#endif

#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
int PlcPoll_FxNet(int sock)
{
    return PollNetTable("FX MC", sock, &g_fxPoints, FxNetRead, FxReport, FX_NET_GAP_MS);
}
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
int PlcPoll_S7Net(int sock)
{
    return PollNetTable("S7", sock, &g_s7Points, S7NetRead, PpiReport, S7_NET_GAP_MS);
}
#endif
