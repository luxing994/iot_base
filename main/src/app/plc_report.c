#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "iot_common.h"
#include "iot_report.h"
#include "plc_report.h"
#include "hl_plc_protocol.h"

static const char *TAG = "PLC_REPORT";

/* 协议层与本模块之间的"当前数据"描述，由协议层发请求/收应答时设置 */
int g_fxplccount = 0;
int g_fxplcdataformat = 0;   // PLC_DATA_TYPE_xxx
int g_fxplcbitpos = 0;
int g_modbusplccount = 0;
int g_modbusplcdataformat = 0;
int g_ppiplccount = 0;
int g_ppiplcdataformat = 0;
int g_hostlinkplccount = 0;
int g_hostlinkplcdataformat = 0;
int g_hostlinkplcbitpos = 0;
int g_lsplccount = 0;
int g_lsplcdataformat = 0;

typedef enum {
    PLC_VAL_INT,
    PLC_VAL_FLOAT,
    PLC_VAL_STR,
} PlcValKind;

typedef struct {
    PlcValKind kind;
    int32_t i;
    float f;
    char s[8];
} PlcValue;

typedef int (*BufferReader)(uint8_t *data, uint32_t size);

/* ---------------------------------------------------------------- 解码工具 */

static float U32ToFloat(uint32_t raw)
{
    float f;
    memcpy(&f, &raw, sizeof(f));
    return f;
}

// 读取 n 个 ASCII 十六进制字符并转换成数值
static int ReadAsciiHex(BufferReader rd, int nchars, uint32_t *out)
{
    char buf[9] = {0};

    if (nchars > 8 || rd((uint8_t *)buf, nchars) != 0) {
        return -1;
    }
    *out = strtoul(buf, NULL, 16);
    return 0;
}

// 读取 n 个原始字节（n <= 4），little = 1 按小端组合，否则按大端
static int ReadBytes(BufferReader rd, int n, int little, uint32_t *out)
{
    uint8_t b[4] = {0};
    uint32_t v = 0;

    if (n > 4 || rd(b, n) != 0) {
        return -1;
    }
    for (int i = 0; i < n; i++) {
        v |= (uint32_t)b[little ? i : (n - 1 - i)] << (8 * i);
    }
    *out = v;
    return 0;
}

// 读取定长字符串（协议层写入的字符串数据为 5 字节，含结尾 0）
static int ReadString(BufferReader rd, PlcValue *v)
{
    memset(v->s, 0, sizeof(v->s));
    if (rd((uint8_t *)v->s, 5) != 0) {
        return -1;
    }
    v->s[5] = '\0';
    v->kind = PLC_VAL_STR;
    return 0;
}

static void SetInt(PlcValue *v, int32_t i)
{
    v->kind = PLC_VAL_INT;
    v->i = i;
}

static void SetFloat(PlcValue *v, float f)
{
    v->kind = PLC_VAL_FLOAT;
    v->f = f;
}

static __attribute__((unused)) double ValueAsDouble(const PlcValue *v)
{
    return (v->kind == PLC_VAL_FLOAT) ? (double)v->f : (double)v->i;
}

/* 单值上报：整数 "%d"，浮点按 decimals 位小数，字符串原样 */
static void SendValue(const char *devTypeId, const char *devTypeName, int frnum,
                      const PlcValue *v, int decimals)
{
    switch (v->kind) {
    case PLC_VAL_STR:
        IotReport_SendPoint(devTypeId, devTypeName, frnum, "%s", v->s);
        break;
    case PLC_VAL_FLOAT:
        IotReport_SendPoint(devTypeId, devTypeName, frnum, "%.*f", decimals, (double)v->f);
        break;
    default:
        IotReport_SendPoint(devTypeId, devTypeName, frnum, "%d", (int)v->i);
        break;
    }
}

/* ------------------------------------------------- 批量设备：生能冷媒灌注机 */

#if defined(CONFIG_PLC_FX_SN_CACLREFILMAC)
// 生能 2-4 车间冷媒灌注机（FX），10 个数据点
static const IotBatchPointDef s_snBatchPoints[] = {
    { "00", 2, 1.0f },   // FR001 系统真空
    { "CG", 2, 0.1f },   // FR002 冷媒温度（A系统）
    { "CG", 2, 0.1f },   // FR003 冷媒温度（B系统）
    { "PA", 2, 0.1f },   // FR004 系统压力（A系统）
    { "PA", 2, 0.1f },   // FR005 系统压力（B系统）
    { "00", 2, 1.0f },   // FR006 灌注量
    { "00", 0, 1.0f },   // FR007 单班产量
    { "00", 0, 1.0f },   // FR008 总产量
    { "00", 0, 1.0f },   // FR009 测试结果
    { "00", 2, 1.0f },   // FR010 实际设定灌注量
};
#define SN_BATCH_ENABLED 1
#elif defined(CONFIG_PLC_HOSTLINK_SN_CACLREFILMAC)
// 生能 1 车间冷媒灌注机（HostLink），27 个数据点
static const IotBatchPointDef s_snBatchPoints[] = {
    { "00",  0, 1.0f },  // FR001 A模式选择
    { "00",  0, 1.0f },  // FR002 B模式选择
    { "S",   0, 1.0f },  // FR003 A检测时间
    { "S",   0, 1.0f },  // FR004 B检测时间
    { "PA",  2, 1.0f },  // FR005 A抽空上限
    { "PA",  2, 1.0f },  // FR006 B抽空上限
    { "PA",  2, 1.0f },  // FR007 A抽空下限
    { "PA",  2, 1.0f },  // FR008 B抽空下限
    { "MPA", 2, 1.0f },  // FR009 A充注压力
    { "MPA", 2, 1.0f },  // FR010 B充注压力
    { "PA",  2, 1.0f },  // FR011 A真空度
    { "PA",  2, 1.0f },  // FR012 B真空度
    { "G",   2, 1.0f },  // FR013 A设定量
    { "G",   2, 1.0f },  // FR014 B设定量
    { "00",  2, 1.0f },  // FR015 A百分比
    { "00",  2, 1.0f },  // FR016 B百分比
    { "GS",  2, 1.0f },  // FR017 A充注速度
    { "GS",  2, 1.0f },  // FR018 B充注速度
    { "G",   2, 1.0f },  // FR019 A充注量
    { "G",   2, 1.0f },  // FR020 B充注量
    { "00",  0, 1.0f },  // FR021 A工作状态
    { "00",  0, 1.0f },  // FR022 B工作状态
    { "S",   2, 1.0f },  // FR023 A充注时间
    { "S",   2, 1.0f },  // FR024 B充注时间
    { "00",  0, 1.0f },  // FR025 结果判定
    { "00",  2, 1.0f },  // FR026 A充注设定值
    { "00",  2, 1.0f },  // FR027 B充注设定值
};
#define SN_BATCH_ENABLED 1
#endif

#ifdef SN_BATCH_ENABLED
#define SN_BATCH_COUNT (sizeof(s_snBatchPoints) / sizeof(s_snBatchPoints[0]))
static double s_snBatchValues[SN_BATCH_COUNT];
static IotBatchCollector s_snBatch = {
    .devTypeId = FRIGEFILLTYPEID,
    .points = s_snBatchPoints,
    .count = SN_BATCH_COUNT,
    .values = s_snBatchValues,
};
#endif

int PlcReport_GetBatchValue(int frnum, double *value)
{
#ifdef SN_BATCH_ENABLED
    return IotBatchCollector_Get(&s_snBatch, frnum, value);
#else
    return -1;
#endif
}

/* ------------------------------------------------------------- 三菱 FX */

#if defined(CONFIG_PLC_FX_XF_CONTROLLER) || defined(CONFIG_PLC_FX_RY_LINE)
#define FX_LAST_VALUE_COUNT 256
static int32_t s_fxLastValue[FX_LAST_VALUE_COUNT];
static bool s_fxLastValid[FX_LAST_VALUE_COUNT];

// 与上次值不同才上报
static bool FxValueChanged(int frnum, int32_t value)
{
    if (frnum < 0 || frnum >= FX_LAST_VALUE_COUNT) {
        return true;
    }
    if (s_fxLastValid[frnum] && s_fxLastValue[frnum] == value) {
        return false;
    }
    s_fxLastValue[frnum] = value;
    s_fxLastValid[frnum] = true;
    return true;
}
#endif

static int FxDecode(int format, PlcValue *v)
{
    BufferReader rd = FXPLC_ReadBufferBytes;
    uint32_t raw, hi, lo;

    (void)hi;   // 网口模式下不使用
    (void)lo;

    switch (format) {
    case PLC_DATA_TYPE_CHAR_POINT_OR_BYTE_FOR_PPI:
        return ReadString(rd, v);

#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
    // 网口：协议层按小端写入的 16 位字
    case PLC_DATA_TYPE_SHORT:
        if (ReadBytes(rd, 2, 1, &raw) != 0) return -1;
        SetInt(v, (int32_t)raw);
        return 0;
    case PLC_DATA_TYPE_BIT:
        if (ReadBytes(rd, 2, 1, &raw) != 0) return -1;
        SetInt(v, raw & 0x0001);
        return 0;
    case PLC_DATA_TYPE_INT:
        if (ReadBytes(rd, 4, 1, &raw) != 0) return -1;
        SetInt(v, (int32_t)raw);
        return 0;
    case PLC_DATA_TYPE_FLOAT:
        if (ReadBytes(rd, 4, 1, &raw) != 0) return -1;
        SetFloat(v, U32ToFloat(raw));
        return 0;
#else
    // 串口：协议层写入的是 ASCII 十六进制，每个字 4 个字符，低字在前
    case PLC_DATA_TYPE_SHORT:
        if (ReadAsciiHex(rd, 4, &raw) != 0) return -1;
        SetInt(v, (int32_t)raw);
        return 0;
    case PLC_DATA_TYPE_BIT:
        if (ReadAsciiHex(rd, 4, &raw) != 0) return -1;
        SetInt(v, raw & 0x0001);
        return 0;
    case PLC_DATA_TYPE_INT:
        if (ReadAsciiHex(rd, 4, &lo) != 0 || ReadAsciiHex(rd, 4, &hi) != 0) return -1;
        SetInt(v, (int32_t)((hi << 16) | lo));
        return 0;
    case PLC_DATA_TYPE_FLOAT:
        if (ReadAsciiHex(rd, 4, &lo) != 0 || ReadAsciiHex(rd, 4, &hi) != 0) return -1;
        SetFloat(v, U32ToFloat((hi << 16) | lo));
        return 0;
#endif
    default:
        return -1;
    }
}

void PlcReport_Fx(void)
{
    const int frnum = g_fxplccount;
    PlcValue v;

    if (FxDecode(g_fxplcdataformat, &v) != 0) {
        ESP_LOGW(TAG, "fx decode failed, format %d", g_fxplcdataformat);
        return;
    }

    if (v.kind != PLC_VAL_STR) {
#if defined(CONFIG_PLC_FX_SN_CACLREFILMAC)
        IotBatchCollector_Update(&s_snBatch, frnum, ValueAsDouble(&v));
        return;
#elif defined(CONFIG_PLC_FX_XF_CONTROLLER) || defined(CONFIG_PLC_FX_RY_LINE)
        if (v.kind == PLC_VAL_INT && !FxValueChanged(frnum, v.i)) {
            return;
        }
#endif
    }
    SendValue(FRIGEFILLTYPEID, FXPLCDEVTYPEID, frnum, &v, 2);
}

/* --------------------------------------------------------- 欧姆龙 HostLink */

static int HostLinkDecode(int format, PlcValue *v)
{
    BufferReader rd = FXPLC_ReadBufferBytes;
    uint32_t raw;

    switch (format) {
    case PLC_DATA_TYPE_CHAR_POINT_OR_BYTE_FOR_PPI:
        return ReadString(rd, v);
    case PLC_DATA_TYPE_SHORT:
        if (ReadAsciiHex(rd, 4, &raw) != 0) return -1;
        SetInt(v, (int32_t)raw);
        return 0;
    case PLC_DATA_TYPE_BCD:
        if (ReadAsciiHex(rd, 4, &raw) != 0) return -1;
        SetInt(v, BCDToInt((int)raw));
        return 0;
    case PLC_DATA_TYPE_BIT:
        if (ReadAsciiHex(rd, 4, &raw) != 0) return -1;
        HLReadDataCallback((int)raw);
        SetInt(v, (raw >> g_hostlinkplcbitpos) & 0x0001);
        return 0;
    case PLC_DATA_TYPE_FLOAT:
        // 8 个十六进制字符，低字在前，需要交换高低字
        if (ReadAsciiHex(rd, 8, &raw) != 0) return -1;
        SetFloat(v, U32ToFloat((raw << 16) | (raw >> 16)));
        return 0;
    default:
        return -1;
    }
}

void PlcReport_HostLink(void)
{
    const int frnum = g_hostlinkplccount;
    const int format = g_hostlinkplcdataformat;
    PlcValue v;

    if (HostLinkDecode(format, &v) != 0) {
        ESP_LOGW(TAG, "hostlink decode failed, format %d", format);
        return;
    }

#if defined(CONFIG_PLC_HOSTLINK_SN_CACLREFILMAC)
    // 位数据和字符串始终单独上报，其余进入批量报文
    if (format != PLC_DATA_TYPE_BIT && v.kind != PLC_VAL_STR) {
        IotBatchCollector_Update(&s_snBatch, frnum, ValueAsDouble(&v));
        return;
    }
#endif
    SendValue(FRIGEFILLTYPEID, HLPLCDEVTYPEID, frnum, &v, 2);
}

/* ------------------------------------------------------------------ LS / DBS */

void PlcReport_Ls(void)
{
    const int frnum = g_lsplccount;
    char buf[5] = {0};
    PlcValue v;

    if (g_lsplcdataformat == PLC_DATA_TYPE_CHAR_POINT_OR_BYTE_FOR_PPI) {
        if (ReadString(FXPLC_ReadBufferBytes, &v) != 0) {
            return;
        }
        SendValue(VACUUMTYPEID, LSPLCDEVTYPEID, frnum, &v, 2);
    } else if (g_lsplcdataformat == PLC_DATA_TYPE_SHORT) {
        // 4 个十六进制字符，前后两个字节交换
        for (int i = 0; i < 4; i++) {
            if (FXPLC_ReadBufferBytes((uint8_t *)&buf[(i + 2) % 4], 1) != 0) {
                return;
            }
        }
        SetInt(&v, (int32_t)strtoul(buf, NULL, 16));
        SendValue(FRIGEFILLTYPEID, LSPLCDEVTYPEID, frnum, &v, 2);
    }
}

/* ------------------------------------------------------- 西门子 PPI / S7 */

// S7 网口：协议层写入的是本机（小端）内存；PPI 串口：报文原始大端字节
#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
#define PPI_LITTLE_ENDIAN 1
#else
#define PPI_LITTLE_ENDIAN 0
#endif

void PlcReport_Ppi(void)
{
    const int frnum = g_ppiplccount;
    BufferReader rd = PPIPLC_ReadBufferBytes;
    uint32_t raw;
    PlcValue v;

    switch (g_ppiplcdataformat) {
    case PLC_DATA_TYPE_CHAR_POINT_OR_BYTE_FOR_PPI:
    case PLC_DATA_TYPE_BIT:
        if (ReadBytes(rd, 1, 1, &raw) != 0) return;
        SetInt(&v, (int32_t)raw);
        SendValue(VACUUMTYPEID, LSPLCDEVTYPEID, frnum, &v, 2);
        break;
    case PLC_DATA_TYPE_SHORT:
        if (ReadBytes(rd, 2, PPI_LITTLE_ENDIAN, &raw) != 0) return;
        SetInt(&v, (int16_t)raw);          // S7 INT 为有符号 16 位
        SendValue(FRIGEFILLTYPEID, LSPLCDEVTYPEID, frnum, &v, 2);
        break;
    case PLC_DATA_TYPE_INT:
        if (ReadBytes(rd, 4, PPI_LITTLE_ENDIAN, &raw) != 0) return;
        SetInt(&v, (int32_t)raw);
        SendValue(FRIGEFILLTYPEID, LSPLCDEVTYPEID, frnum, &v, 2);
        break;
    case PLC_DATA_TYPE_FLOAT:
        if (ReadBytes(rd, 4, PPI_LITTLE_ENDIAN, &raw) != 0) return;
        SetFloat(&v, U32ToFloat(raw));
        SendValue(FRIGEFILLTYPEID, LSPLCDEVTYPEID, frnum, &v, 2);
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------- Modbus TCP */

void PlcReport_ModbusNet(void)
{
    const int frnum = g_modbusplccount;
    BufferReader rd = MODBUSPLC_ReadBufferBytes;
    uint32_t raw;
    PlcValue v;

    // 协议层写入的是本机（小端）内存
    switch (g_modbusplcdataformat) {
    case PLC_DATA_TYPE_FLOAT:
        if (ReadBytes(rd, 4, 1, &raw) != 0) return;
        SetFloat(&v, U32ToFloat(raw));
        SendValue(VACUUMTYPEID, MDPLCDEVTYPEID, frnum, &v, 3);
        break;
    case PLC_DATA_TYPE_SHORT:
        if (ReadBytes(rd, 2, 1, &raw) != 0) return;
        SetInt(&v, (int32_t)raw);
        SendValue(VACUUMTYPEID, MDPLCDEVTYPEID, frnum, &v, 3);
        break;
    case PLC_DATA_TYPE_INT:
        if (ReadBytes(rd, 4, 1, &raw) != 0) return;
        SetInt(&v, (int32_t)raw);
        SendValue(FRIGEFILLTYPEID, LSPLCDEVTYPEID, frnum, &v, 3);
        break;
    default:
        break;
    }
}
