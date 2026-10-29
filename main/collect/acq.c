/*
 * 采集引擎，说明见 acq.h
 *
 * 协议适配层只负责"读一个点的原始数据"（16 位、32 位或一位），
 * 类型转换（符号、BCD、字序、倍率）和上报在这里统一完成，各协议不用各自处理。
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "devcfg.h"
#include "proto_lib.h"
#include "plc_value.h"
#include "plc_report.h"
#include "serial_txn.h"
#include "uart_task.h"
#include "host_cmd.h"
#include "iot_report.h"
#include "event_log.h"
#include "s7_area.h"
#include "fx_plc_protocol.h"
#include "hl_plc_protocol.h"
#include "ppi_plc_protocol.h"
#include "ls_plc_load_protocol.h"
#include "t_tester_protocol.h"
#include "ainuo_tester_ascii_protocol.h"
#include "acq.h"

static const char *TAG = "ACQ";

/* 串口连续多少个点无应答时结束本轮（否则每个点都要等满超时，串口锁长时间被占用） */
#define OFFLINE_FAILS       3
#define DBS_FRNUM           0       // DBS 真空计只有一个读数，沿用原实现上报为 FR000

/* 读出的原始数据 */
typedef struct {
    uint32_t raw;           // 16 位、32 位数据，或已取出的一位
    bool isBit;             // raw 已是 0 / 1
    const char *order;      // 32 位数据：raw 按哪种字序组成（协议的默认字序）
} RawValue;

typedef int (*RawReader)(const DevCfgDevice *d, const DevCfgPoint *p, RawValue *r);
typedef void (*TemplatePoll)(void);

static RawReader s_read;            // 点表协议
static TemplatePoll s_template;     // 内置模板（测试仪等）
static char s_problem[96];

/* 每个点最近一次的结果（网页显示） */
static double *s_values;
static int8_t *s_states;            // 0 未读，1 成功，-1 失败
static int64_t *s_times;            // 最近一次成功的时刻（us），0 表示从未成功
static int64_t s_roundTime;         // 最近一轮采集结束的时刻（us）
static bool s_running;
/* 变化上报：上次上报的值 */
static double *s_reported;
static bool *s_hasReported;

static bool Is32(uint8_t type)
{
    return (PT_DWORDS & PT_MASK(type)) != 0;
}

static const char *AreaName(const DevCfgDevice *d, const DevCfgPoint *p)
{
    return d->proto->areas[p->area].name;
}

/* ============================================================ 协议适配：读一个点的原始数据 */

// 三菱 X / Y 的地址是八进制：计算机链接报文中按八进制数字写，如 X17 写成 "0017"
static uint16_t OctalDigits(uint32_t v)
{
    uint32_t r = 0;
    uint32_t mul = 1;

    while (v > 0) {
        r += (v % 8) * mul;
        mul *= 10;
        v /= 8;
    }
    return (uint16_t)r;
}

static int ReadFxProg(const DevCfgDevice *d, const DevCfgPoint *p, RawValue *r)
{
    PlcValue v;
    int ret;

    (void)d;
    if (Is32(p->type) || p->type == PT_BIT) {
        return PLC_ERR_UNSUPPORTED;     // 编程口只读 D 寄存器 16 位（导入时已校验）
    }
    ret = FX_ProgReadD((uint16_t)p->addr, &v);
    r->raw = (uint32_t)v.i & 0xFFFF;
    return ret;
}

static int ReadFxLink(const DevCfgDevice *d, const DevCfgPoint *p, RawValue *r)
{
    const char *area = AreaName(d, p);
    FxDevice dev = FX_DEV_D;
    uint16_t addr = (uint16_t)p->addr;
    PlcValue v;
    int ret;

    switch (area[0]) {
    case 'M': dev = FX_DEV_M; break;
    case 'X': dev = FX_DEV_X; addr = OctalDigits(p->addr); break;
    case 'Y': dev = FX_DEV_Y; addr = OctalDigits(p->addr); break;
    default:  break;
    }
    if (dev != FX_DEV_D) {
        ret = FX_LinkRead(dev, addr, FX_FMT_BIT, &v);
        r->raw = (uint32_t)v.i & 1;
        r->isBit = true;
    } else if (Is32(p->type)) {
        ret = FX_LinkRead(dev, addr, FX_FMT_I32, &v);       // 低字在前
        r->raw = (uint32_t)v.i;
        r->order = "CDAB";
    } else {
        ret = FX_LinkRead(dev, addr, FX_FMT_U16, &v);
        r->raw = (uint32_t)v.i & 0xFFFF;
    }
    return ret;
}

static int ReadHostLink(const DevCfgDevice *d, const DevCfgPoint *p, RawValue *r)
{
    PlcValue v;
    int ret;

    if (strcmp(AreaName(d, p), "CIO") == 0) {
        ret = HL_ReadCIOBit((uint16_t)p->addr, p->bit, &v);
        r->raw = (uint32_t)v.i & 1;
        r->isBit = true;
    } else if (Is32(p->type)) {
        ret = HL_ReadDM((uint16_t)p->addr, HL_FMT_FLOAT, &v);   // 两个字，低字在前；按位取出再按类型解释
        memcpy(&r->raw, &v.f, sizeof(r->raw));
        r->order = "CDAB";
    } else {
        ret = HL_ReadDM((uint16_t)p->addr, HL_FMT_U16, &v);     // 字区取位由引擎处理
        r->raw = (uint32_t)v.i & 0xFFFF;
    }
    return ret;
}

static int ReadPpi(const DevCfgDevice *d, const DevCfgPoint *p, RawValue *r)
{
    S7Var var = { .byte = p->addr, .bit = p->bit };
    PlcValue v;
    int ret;

    switch (AreaName(d, p)[0]) {
    case 'I': var.area = S7_AREA_I; break;
    case 'Q': var.area = S7_AREA_Q; break;
    case 'M': var.area = S7_AREA_M; break;
    default:  var.area = S7_AREA_V; break;
    }
    if (p->type == PT_BIT) {
        var.type = S7_BIT;
        ret = PPI_Read(&var, &v);
        r->raw = (uint32_t)v.i & 1;
        r->isBit = true;
    } else if (Is32(p->type)) {
        var.type = S7_DINT;                     // 大端，按位取出再按类型解释
        ret = PPI_Read(&var, &v);
        r->raw = (uint32_t)v.i;
        r->order = "ABCD";
    } else {
        var.type = S7_INT;
        ret = PPI_Read(&var, &v);
        r->raw = (uint32_t)v.i & 0xFFFF;
    }
    return ret;
}

/* ============================================================ 内置模板 */

static void PollDbs(void)
{
    PlcValue v;

    if (DBS_Read(&v) == PLC_OK) {
        PlcReport_Ls(DBS_FRNUM, &v);
    }
}

static void PollT76(void)
{
    TTester_ReadGroup((uint16_t)HostCmd_GetTesterGroup());   // 0 读当前组，否则读上位机指定组的历史结果
}

static void PollAinuo(void)
{
    AINUO_ReadGroup();
}

/* ============================================================ 类型转换 */

// 32 位数据按配置的字序重新组合。raw 是协议按 from 字序组成的值
static uint32_t Reorder(uint32_t raw, const char *from, uint8_t to)
{
    uint16_t w0;        // 起始地址处的字
    uint16_t w1;        // 下一个字

    if (from != NULL && strcmp(from, "ABCD") == 0) {
        w0 = (uint16_t)(raw >> 16);
        w1 = (uint16_t)raw;
    } else {            // CDAB：低字在前
        w0 = (uint16_t)raw;
        w1 = (uint16_t)(raw >> 16);
    }
    switch (to) {
    case ORDER_ABCD: return ((uint32_t)w0 << 16) | w1;
    case ORDER_BADC: return ((uint32_t)__builtin_bswap16(w0) << 16) | __builtin_bswap16(w1);
    case ORDER_DCBA: return ((uint32_t)__builtin_bswap16(w1) << 16) | __builtin_bswap16(w0);
    default:         return ((uint32_t)w1 << 16) | w0;      // CDAB
    }
}

static bool FromBcd(uint32_t raw, double *out)
{
    uint32_t v = 0;

    for (int shift = 12; shift >= 0; shift -= 4) {
        uint32_t digit = (raw >> shift) & 0x0F;
        if (digit > 9) {
            return false;
        }
        v = v * 10 + digit;
    }
    *out = v;
    return true;
}

// 原始数据 -> 上报值；数据无效（BCD 非法、浮点 NaN）返回 false
static bool ToValue(const DevCfgPoint *p, const RawValue *r, double *out)
{
    double v;
    uint32_t w;
    float f;

    switch (p->type) {
    case PT_BIT:
        v = r->isBit ? (r->raw & 1) : ((r->raw >> p->bit) & 1);
        break;
    case PT_I16:
        v = (int16_t)(r->raw & 0xFFFF);
        break;
    case PT_U16:
        v = r->raw & 0xFFFF;
        break;
    case PT_BCD16:
        if (!FromBcd(r->raw & 0xFFFF, &v)) {
            return false;
        }
        break;
    case PT_I32:
        v = (int32_t)Reorder(r->raw, r->order, p->order);
        break;
    case PT_U32:
        v = Reorder(r->raw, r->order, p->order);
        break;
    case PT_F32:
        w = Reorder(r->raw, r->order, p->order);
        memcpy(&f, &w, sizeof(f));
        if (!isfinite(f)) {
            return false;
        }
        v = f;
        break;
    default:
        return false;
    }
    *out = v * p->scale;
    return true;
}

/* ============================================================ 上报 */

static void FormatValue(const DevCfgPoint *p, double v, char *buf, size_t size)
{
    snprintf(buf, size, "%.*f", p->dec, v);
    if (strcmp(buf, "-0") == 0 || (buf[0] == '-' && strspn(buf + 1, "0.") == strlen(buf + 1))) {
        memmove(buf, buf + 1, strlen(buf));     // 去掉 "-0.00" 的负号
    }
}

static void ReportPoint(const DevCfg *c, uint16_t i, double v)
{
    const DevCfgPoint *p = &c->points[i];
    char text[32];

    if (p->mode == POINT_CHANGE && s_hasReported[i]) {
        double diff = fabs(v - s_reported[i]);
        if ((p->dead > 0) ? (diff <= p->dead) : (diff == 0)) {
            return;
        }
    }
    s_reported[i] = v;
    s_hasReported[i] = true;
    FormatValue(p, v, text, sizeof(text));
    IotReport_SendPoint(DevCfg_DevTypeId(), c->dev.devTypeName, p->fr, "%s", text);
}

/* 一轮：逐点读取并上报 */
static void PollRound(const DevCfg *c)
{
    const bool batch = (c->dev.report == DEV_REPORT_BATCH);
    IotBatch *b = batch ? IotBatch_Create() : NULL;
    bool complete = true;
    int fails = 0;

    for (uint16_t i = 0; i < c->count; i++) {
        const DevCfgPoint *p = &c->points[i];
        RawValue r = { 0 };
        double v;
        int ret = s_read(&c->dev, p, &r);

        if (ret == PLC_OK && ToValue(p, &r, &v)) {
            fails = 0;
            s_values[i] = v;
            s_states[i] = 1;
            s_times[i] = esp_timer_get_time();
            if (b != NULL) {
                char text[32];
                FormatValue(p, v, text, sizeof(text));
                IotBatch_AddFr(b, p->fr, p->unit[0] != '\0' ? p->unit : "00", "%s", text);
            } else {
                ReportPoint(c, i, v);
            }
            continue;
        }

        complete = false;
        s_states[i] = -1;
        if (ret == PLC_OK) {
            ESP_LOGW(TAG, "FR%03u %s: invalid data", (unsigned)p->fr, p->name);
        } else if (ret == PLC_ERR_RESPONSE && ++fails >= OFFLINE_FAILS && i + 1 < c->count) {
            // 设备离线由 serial_txn 统一告警（只在状态变化时记录），这里只是缩短本轮
            ESP_LOGD(TAG, "no response for %d points, skip the rest of this round", fails);
            break;
        }
    }
    if (b != NULL) {
        // 批量报文要求一轮的点全部读到，缺点时本轮不报（与原批量设备的做法一致）
        if (complete && IotBatch_Count(b) == c->count) {
            IotBatch_Send(b, DevCfg_DevTypeId());
        }
        IotBatch_Destroy(b);
    }
}

static void AcqTask(void *arg)
{
    const DevCfg *c = DevCfg_Get();

    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(c->dev.pollMs));
        if (!HostCmd_IsCollectEnabled()) {
            continue;   // 上位机发送 EndFlush 后暂停采集，StartFlush 后恢复
        }
        if (s_template != NULL) {
            s_template();
        } else {
            PollRound(c);
        }
        s_roundTime = esp_timer_get_time();
    }
}

/* ============================================================ 启动 */

static const struct {
    const char *id;
    RawReader read;
    TemplatePoll poll;
    void (*setStation)(uint8_t station);
} s_drivers[] = {
    { "fx_prog",  ReadFxProg,   NULL,      NULL },
    { "fx_link",  ReadFxLink,   NULL,      FX_SetStation },
    { "hostlink", ReadHostLink, NULL,      HL_SetUnit },
    { "ppi",      ReadPpi,      NULL,      PPI_SetStation },
    { "dbs",      NULL,         PollDbs,   NULL },
    { "t76",      NULL,         PollT76,   NULL },
    { "ainuo",    NULL,         PollAinuo, NULL },
};

bool Acq_Start(UBaseType_t pollPrio, UBaseType_t uartPrio)
{
    const DevCfg *c = DevCfg_Get();
    const DevCfgDevice *d;
    size_t i;

    if (c == NULL) {
        return false;
    }
    d = &c->dev;
    for (i = 0; i < sizeof(s_drivers) / sizeof(s_drivers[0]); i++) {
        if (strcmp(s_drivers[i].id, d->proto->id) == 0) {
            break;
        }
    }
    if (i == sizeof(s_drivers) / sizeof(s_drivers[0]) || !(d->iface & IFACE_SERIAL)) {
        // 网口协议、Modbus RTU 在后续版本中接入
        snprintf(s_problem, sizeof(s_problem), "本固件暂不能按配置采集 %s，请升级固件", d->proto->name);
        ESP_LOGE(TAG, "%s", s_problem);
        EvLog_Add(EV_ERROR, "%s", s_problem);
        return true;
    }

    if (c->count > 0) {
        s_values = calloc(c->count, sizeof(double));
        s_states = calloc(c->count, sizeof(int8_t));
        s_times = calloc(c->count, sizeof(int64_t));
        s_reported = calloc(c->count, sizeof(double));
        s_hasReported = calloc(c->count, sizeof(bool));
        if (s_values == NULL || s_states == NULL || s_times == NULL || s_reported == NULL || s_hasReported == NULL) {
            strlcpy(s_problem, "内存不足，无法启动采集", sizeof(s_problem));
            ESP_LOGE(TAG, "%s", s_problem);
            return true;
        }
    }
    s_read = s_drivers[i].read;
    s_template = s_drivers[i].poll;
    if (s_drivers[i].setStation != NULL) {
        s_drivers[i].setStation(d->station);
    }

    // 拨码开关只有 232 / 485 两档：RS422 设备经外接转换器接在 232 档
    const UartParams up = {
        .baud = d->baud,
        .dataBits = d->dataBits,
        .parity = d->parity,
        .stopBits = d->stopBits,
        .rs485 = (d->iface == IFACE_RS485),
        .name = (d->iface == IFACE_RS485) ? "RS485" : "RS232",
    };
    UART_Start(&up);
    if (xTaskCreate(uart_event_task, "uart_event", 1024 * 4, NULL, uartPrio, NULL) != pdPASS ||
        xTaskCreate(AcqTask, "acq", 1024 * 6, NULL, pollPrio, NULL) != pdPASS) {
        strlcpy(s_problem, "内存不足，无法启动采集任务", sizeof(s_problem));
        ESP_LOGE(TAG, "%s", s_problem);
        return true;
    }
    s_running = true;
    ESP_LOGI(TAG, "%s: %s %s, station %u, %u points every %u ms", d->name, d->proto->id, UART_InterfaceText(),
             (unsigned)d->station, (unsigned)c->count, (unsigned)d->pollMs);
    return true;
}

const char *Acq_ProblemText(void)
{
    return (s_problem[0] != '\0') ? s_problem : NULL;
}

bool Acq_IsRunning(void)
{
    return s_running;
}

int32_t Acq_RoundAgeS(void)
{
    return (s_roundTime == 0) ? -1 : (int32_t)((esp_timer_get_time() - s_roundTime) / 1000000);
}

int Acq_GetPointText(uint16_t index, char *text, size_t size, uint32_t *ageS)
{
    const DevCfg *c = DevCfg_Get();

    text[0] = '\0';
    *ageS = 0;
    if (c == NULL || index >= c->count || s_states == NULL) {
        return 0;
    }
    if (s_times[index] != 0) {
        FormatValue(&c->points[index], s_values[index], text, size);
        *ageS = (uint32_t)((esp_timer_get_time() - s_times[index]) / 1000000);
    }
    return s_states[index];
}
