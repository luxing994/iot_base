/*
 * 设备轮询
 *
 *   plc_poll_task       串口 PLC / 测试仪 / Modbus RTU，每秒轮询一次
 *   PlcPoll_xxxNet()    网口 PLC，由 tcp_client.c 中的连接任务每秒调用一次
 *
 * 读哪些寄存器由点表决定（plc_points.c，按 menuconfig 选择），这里只负责按表逐点执行：
 *   串口：每个点一次同步事务（发请求 -> 等完整应答 -> 解码上报，见 serial_txn.h）
 *   网口：每个点读完立即解码上报，链路失效时返回 -1 由调用方重连
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "iot_common.h"
#include "plc_poll.h"
#include "plc_points.h"
#include "plc_report.h"
#include "host_cmd.h"
#include "serial_txn.h"
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

/* FX 计算机链接的站号、PC 号和报文等待时间 */
#define FX_STATION          0
#define FX_PC_NUMBER        255
#define FX_WAIT             10

static __attribute__((unused)) void LogUnsupported(const char *protocol, const PlcPoint *p)
{
    ESP_LOGW(TAG, "%s: point type %d not supported on this connection (FR%03d %s)",
             protocol, p->type, p->frnum, p->name);
}

/* ============================================================ 串口 */

#if defined(CONFIG_PLC_FX_CONNECT_RS232)
#define FX_SERIAL_PARSE   GetDataFromFxPlc          // 编程口
#else
#define FX_SERIAL_PARSE   GetSerialDataFromFxPlc    // 计算机链接
#endif

static __attribute__((unused)) void FxOnResponse(void)
{
    SendAckToPlc();
    PlcReport_Fx();
}

#define FX_POINT(request)      SERIAL_TXN(request, FX_SERIAL_PARSE, SERIAL_TXN_TIMEOUT_MS, FxOnResponse())
#define HL_POINT(request)      SERIAL_TXN(request, GetSerialWordDataFromHlPlc, SERIAL_TXN_TIMEOUT_MS, PlcReport_HostLink())
#define PPI_POINT(request)     SERIAL_TXN(request, GetSerialWordDataFromPpiPlc, SERIAL_TXN_TIMEOUT_MS, PlcReport_Ppi())
#define LS_POINT(request)      SERIAL_TXN(request, DBSGetData, SERIAL_TXN_TIMEOUT_MS, PlcReport_Ls())

#if defined(CONFIG_PLC_FX_CONNECT_RS232) || defined(CONFIG_PLC_FX_CONNECT_RS485)
static void PollFxSerial(const PlcPointTable *table)
{
    for (size_t i = 0; i < table->count; i++) {
        const PlcPoint *p = &table->points[i];

        switch (p->type) {
#if defined(CONFIG_PLC_FX_CONNECT_RS232)
        case FX_D16:    FX_POINT(ReadSingleDataRegister(p->address, p->frnum)); break;
#else
        case FX_D16:    FX_POINT(SerialReadSingleDataRegister(FX_STATION, FX_PC_NUMBER, FX_WAIT, p->address, p->frnum)); break;
        case FX_D32:    FX_POINT(SerialReadDoubleDataRegister(FX_STATION, FX_PC_NUMBER, FX_WAIT, p->address, p->frnum)); break;
        case FX_DFLOAT: FX_POINT(SerialReadSingleFloatDataRegister(FX_STATION, FX_PC_NUMBER, FX_WAIT, p->address, p->frnum)); break;
        case FX_Y:      FX_POINT(SerialReadSingleYDataRegister(FX_STATION, FX_PC_NUMBER, FX_WAIT, p->address, p->frnum)); break;
        case FX_M:      FX_POINT(SerialReadSingleMDataRegister(FX_STATION, FX_PC_NUMBER, FX_WAIT, p->address, p->frnum)); break;
#endif
        default:        LogUnsupported("FX", p); break;
        }
    }
}
#endif

#if defined(CONFIG_PLC_HOSTLINK)
static void PollHostLink(const PlcPointTable *table)
{
    for (size_t i = 0; i < table->count; i++) {
        const PlcPoint *p = &table->points[i];

        switch (p->type) {
        case HL_DM:       HL_POINT(HLReadSingleDataRegister(p->address, p->frnum)); break;
        case HL_DM_BCD:   HL_POINT(HLReadBCDDataRegister(p->address, p->frnum)); break;
        case HL_DM_FLOAT: HL_POINT(HLReadFloatDataRegister(p->address, p->frnum)); break;
        case HL_CIO_BIT:  HL_POINT(HLReadBitCIORegister(p->address, p->bit, p->frnum)); break;
        default:          LogUnsupported("HostLink", p); break;
        }
    }
}
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_RS485)
static void PollPpiSerial(const PlcPointTable *table)
{
    for (size_t i = 0; i < table->count; i++) {
        const PlcPoint *p = &table->points[i];

        switch (p->type) {
        case S7_VB:    PPI_POINT(PPIReadByteDataRegister(p->address, 1, p->frnum)); break;
        case S7_VW:    PPI_POINT(PPIReadShortDataRegister(p->address, 1, p->frnum)); break;
        case S7_VD:    PPI_POINT(PPIReadSIntDataRegister(p->address, 1, p->frnum)); break;
        case S7_VREAL: PPI_POINT(PPIReadFloatDataRegister(p->address, 1, p->frnum)); break;
        case S7_V_BIT: PPI_POINT(PPIReadBitDataRegister(p->address, p->bit, 1, p->frnum)); break;
        case S7_Q_BIT: PPI_POINT(PPIReadBitOutputRegister(p->address, p->bit, 1, p->frnum)); break;
        case S7_I_BIT: PPI_POINT(PPIReadBitInputRegister(p->address, p->bit, 1, p->frnum)); break;
        case S7_M_BIT: PPI_POINT(PPIReadBitMDataRegister(p->address, p->bit, 1, p->frnum)); break;
        default:       LogUnsupported("PPI", p); break;
        }
    }
}
#endif

#if defined(CONFIG_TESTER_76T) || defined(CONFIG_TESTER_AINUO)
static char s_testerJson[2048];

static void PostTesterJson(void (*get_json)(char *str))
{
    get_json(s_testerJson);
    IOT_PostMessage(s_testerJson);
}
#endif

void plc_poll_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(POLL_INTERVAL_MS));
        if (!HostCmd_IsCollectEnabled()) {
            continue;   // 上位机发送 EndFlush 后暂停采集，StartFlush 后恢复
        }

#if defined(CONFIG_PLC_FX_CONNECT_RS232) || defined(CONFIG_PLC_FX_CONNECT_RS485)
        PollFxSerial(&g_fxPoints);
#endif

#if defined(CONFIG_PLC_HOSTLINK)
        PollHostLink(&g_hostLinkPoints);
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_RS485)
        PollPpiSerial(&g_ppiPoints);
#endif

#if defined(CONFIG_PLC_LS_LOAD)
        LS_POINT(DBSReadData());
#endif

#if defined(CONFIG_TESTER_76T)
        // 上位机下发过 group 命令后读该组历史结果，否则读当前组
        if (HostCmd_GetTesterGroup() == 0) {
            SERIAL_TXN(TTesterReadCurrentGroup(), TTesterResolve, SERIAL_TXN_TESTER_TIMEOUT_MS,
                       PostTesterJson(TTesterGetJsonData));
        } else {
            SERIAL_TXN(TTesterReadHistoryGroup(HostCmd_GetTesterGroup()), TTesterResolve,
                       SERIAL_TXN_TESTER_TIMEOUT_MS, PostTesterJson(TTesterGetJsonData));
        }
#endif

#if defined(CONFIG_TESTER_AINUO)
        SERIAL_TXN(AINUO_TTesterReadCurrentGroup(), AINUO_TTesterResolve, SERIAL_TXN_TESTER_TIMEOUT_MS,
                   PostTesterJson(AINUO_TTesterGetJsonData));
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

// 读寄存器返回 <0 表示链路失效，立即结束本轮由调用方重连；读到数据后立即解码上报
#define NET_POINT(call, fetch, report)      \
    do {                                    \
        if ((call) < 0) {                   \
            return -1;                      \
        }                                   \
        if ((fetch) == 0) {                 \
            report;                         \
        }                                   \
    } while (0)

#define MODBUS_NET(call)   NET_POINT(call, GetNetDataFromModbusPlc(), PlcReport_ModbusNet())
#define FX_NET(call)       NET_POINT(call, GetNetDataFromFxPlc(), PlcReport_Fx())
#define S7_NET(call)       NET_POINT(call, GetNetDataFromS7PlcNet(), PlcReport_Ppi())

#if defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
int PlcPoll_ModbusNet(int sock)
{
    for (size_t i = 0; i < g_modbusNetPoints.count; i++) {
        const PlcPoint *p = &g_modbusNetPoints.points[i];

        switch (p->type) {
        case MB_HR16:    MODBUS_NET(MODBUS_NetReadSingleDataRegister(sock, p->address, p->frnum)); break;
        case MB_HR32:    MODBUS_NET(MODBUS_NetReadDoubleDataRegister(sock, p->address, p->frnum)); break;
        case MB_HRFLOAT: MODBUS_NET(MODBUS_NetReadFloatRegister(sock, p->address, p->frnum)); break;
        default:         LogUnsupported("Modbus TCP", p); break;
        }
    }
    return 0;
}
#endif

#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
int PlcPoll_FxNet(int sock)
{
    for (size_t i = 0; i < g_fxPoints.count; i++) {
        const PlcPoint *p = &g_fxPoints.points[i];

        switch (p->type) {
        case FX_D16: FX_NET(FX_NetReadSingleDataRegister(sock, p->address, p->frnum)); break;
        case FX_D32: FX_NET(FX_NetReadDoubleDataRegister(sock, p->address, p->frnum)); break;
        case FX_Y:   FX_NET(FX_NetReadSingleYDataRegister(sock, p->address, p->frnum)); break;
        default:     LogUnsupported("FX MC", p); break;
        }
    }
    return 0;
}
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
int PlcPoll_S7Net(int sock)
{
    for (size_t i = 0; i < g_s7Points.count; i++) {
        const PlcPoint *p = &g_s7Points.points[i];

        switch (p->type) {
        case S7_VW:    S7_NET(S7_NetReadSingleDataRegister(sock, p->address, p->frnum)); break;
        case S7_VD:    S7_NET(S7_NetReadIntDataRegister(sock, p->address, p->frnum)); break;
        case S7_VREAL: S7_NET(S7_NetReadFloatDataRegister(sock, p->address, p->frnum)); break;
        case S7_V_BIT: S7_NET(S7_NetReadBitDataRegister(sock, p->address, p->bit, p->frnum)); break;
        case S7_Q_BIT: S7_NET(S7_NetReadBitQRegister(sock, p->address, p->bit, p->frnum)); break;
        case S7_I_BIT: S7_NET(S7_NetReadBitIRegister(sock, p->address, p->bit, p->frnum)); break;
        case S7_M_BIT: S7_NET(S7_NetReadBitMRegister(sock, p->address, p->bit, p->frnum)); break;
        default:       LogUnsupported("S7", p); break;
        }
    }
    return 0;
}
#endif
