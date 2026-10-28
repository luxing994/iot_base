#include <stdbool.h>
#include "sdkconfig.h"
#include "iot_common.h"
#include "iot_report.h"
#include "plc_report.h"

/*
 * 各协议上报时使用的设备类型（devTypeId / devTypeName）。
 * 同一协议的所有数据点都使用同一组值，不随数据格式变化。
 */
typedef struct {
    const char *devTypeId;     // 设备类别：LM 冷媒灌注 / ZK 真空 / WK 温控 ...
    const char *devTypeName;   // PLC 类别：FXPLC / HLPLC / LSPLC / PPIPLC / MDPLC
} PlcReportType;

static const PlcReportType s_fxType         = { FRIGEFILLTYPEID, FXPLCDEVTYPEID };
static const PlcReportType s_hostLinkType   = { FRIGEFILLTYPEID, HLPLCDEVTYPEID };
static const PlcReportType s_lsType         = { VACUUMTYPEID,    LSPLCDEVTYPEID };
static const PlcReportType s_ppiType        = { VACUUMTYPEID,    PPIPLCDEVTYPEID };
static const PlcReportType s_modbusNetType  = { VACUUMTYPEID,    MDPLCDEVTYPEID };

/* 单值上报：整数 "%d"，浮点按 decimals 位小数，字符串原样 */
static void SendValue(const PlcReportType *type, int frnum, const PlcValue *v, int decimals)
{
    switch (v->kind) {
    case PLC_VAL_STR:
        IotReport_SendPoint(type->devTypeId, type->devTypeName, frnum, "%s", v->s);
        break;
    case PLC_VAL_FLOAT:
        IotReport_SendPoint(type->devTypeId, type->devTypeName, frnum, "%.*f", decimals, (double)v->f);
        break;
    default:
        IotReport_SendPoint(type->devTypeId, type->devTypeName, frnum, "%d", (int)v->i);
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
    (void)frnum;
    (void)value;
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

void PlcReport_Fx(int frnum, const PlcValue *v)
{
#if defined(CONFIG_PLC_FX_SN_CACLREFILMAC)
    IotBatchCollector_Update(&s_snBatch, frnum, PlcValue_AsDouble(v));
    return;
#elif defined(CONFIG_PLC_FX_XF_CONTROLLER) || defined(CONFIG_PLC_FX_RY_LINE)
    if (v->kind == PLC_VAL_INT && !FxValueChanged(frnum, v->i)) {
        return;
    }
#endif
    SendValue(&s_fxType, frnum, v, 2);
}

/* --------------------------------------------------------- 欧姆龙 HostLink */

void PlcReport_HostLink(int frnum, const PlcValue *v, bool isBit)
{
#if defined(CONFIG_PLC_HOSTLINK_SN_CACLREFILMAC)
    // 位数据始终单独上报，其余进入批量报文
    if (!isBit) {
        IotBatchCollector_Update(&s_snBatch, frnum, PlcValue_AsDouble(v));
        return;
    }
#else
    (void)isBit;
#endif
    SendValue(&s_hostLinkType, frnum, v, 2);
}

/* ------------------------------------------------------------------ LS / DBS */

void PlcReport_Ls(int frnum, const PlcValue *v)
{
    SendValue(&s_lsType, frnum, v, 2);
}

/* ------------------------------------------------------- 西门子 PPI / S7 */

void PlcReport_Ppi(int frnum, const PlcValue *v)
{
    SendValue(&s_ppiType, frnum, v, 2);
}

/* ------------------------------------------------------------- Modbus TCP */

void PlcReport_ModbusNet(int frnum, const PlcValue *v)
{
    SendValue(&s_modbusNetType, frnum, v, 3);
}
