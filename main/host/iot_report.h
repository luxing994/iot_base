#ifndef IOT_REPORT_H
#define IOT_REPORT_H

/*
 * 上位机 JSON 上报模块
 *
 * 所有发往上位机的数据报文都在这里拼装，然后放入上报队列（由 tcp_client_task 发出）。
 * 报文以 IOT_MSG_END 结尾，上位机按它拆包。
 *
 * 1. 单值报文：一个数据点一条报文
 *      IotReport_SendPoint(FRIGEFILLTYPEID, FXPLCDEVTYPEID, 3, "%d", value);
 *    orderId 自动生成为 "FR003"。需要自定义其他字段时用 IotReport_SendValueMsg()。
 *
 * 2. 批量报文：多个数据点合并成一条（ParameterIds / ParameterValues / ParameterUnits 用 "__" 连接）
 *      IotBatch *b = IotBatch_Create();
 *      IotBatch_Add(b, "PA", "%.2f", pressure);   // 自动编号 FR001
 *      IotBatch_Add(b, "00", "%d", count);         // FR002
 *      IotBatch_Send(b, FRIGEFILLTYPEID);
 *      IotBatch_Destroy(b);
 */

#include <stdint.h>
#include <stdarg.h>
#include "sdkconfig.h"
#include "gw_config.h"

#define IOT_MSG_END              ";;**##"
#define IOT_DEV_NUMBER           (GwConfig_Get()->devNumber)   // 报文 devNumber 字段默认值（运行参数，见 gw_config.h）
#define IOT_BATCH_ORDER_ID       "BatchParameters"
#define IOT_BATCH_MAX_POINTS     40

/*
 * 单值报文的全部字段。取值为 NULL 的字段使用默认值：
 *   devNumber -> IOT_DEV_NUMBER
 *   devId / devName / orderName -> 上位机最近一次下发命令中的值
 */
typedef struct {
    const char *devNumber;
    const char *devId;
    const char *devName;
    const char *devTypeId;
    const char *devTypeName;
    const char *orderId;
    const char *orderName;
    const char *value;
} IotValueMsg;

typedef struct IotBatch IotBatch;

/* 发送一条单值报文，返回 0 成功 */
int IotReport_SendValueMsg(const IotValueMsg *msg);

/* 按 printf 格式生成 value，发送一条单值报文；orderId 由调用方给出 */
int IotReport_SendValue(const char *devTypeId, const char *devTypeName, const char *orderId,
                        const char *fmt, ...) __attribute__((format(printf, 4, 5)));

/* 同上，orderId 自动生成为 "FR%03d" */
int IotReport_SendPoint(const char *devTypeId, const char *devTypeName, int frnum,
                        const char *fmt, ...) __attribute__((format(printf, 4, 5)));

/* 批量报文 */
IotBatch *IotBatch_Create(void);
void IotBatch_Destroy(IotBatch *batch);
void IotBatch_Reset(IotBatch *batch);
int  IotBatch_Count(const IotBatch *batch);
int  IotBatch_Add(IotBatch *batch, const char *unit, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
int  IotBatch_Send(const IotBatch *batch, const char *devTypeId);

/*
 * 批量采集器：PLC 每次只返回一个数据点时，按 FR 编号逐个缓存，
 * 收到最后一个点（frnum == count）时自动拼成一条批量报文发送。
 *
 * 新增一种批量格式只需要定义点表：
 *   static const IotBatchPointDef s_points[] = {
 *       { "PA", 2, 0.1f },   // FR001: 原始值 / 10，保留 2 位小数
 *       { "00", 0, 1.0f },   // FR002: 整数
 *   };
 *   static double s_values[2];
 *   static IotBatchCollector s_collector = { FRIGEFILLTYPEID, s_points, 2, s_values };
 *   ...
 *   IotBatchCollector_Update(&s_collector, frnum, value);
 */
typedef struct {
    const char *unit;      // ParameterUnits 中的单位
    uint8_t decimals;      // 0: 按整数输出 "%d"；>0: 按 "%.Nf" 输出
    float scale;           // 输出值 = 原始值 * scale
} IotBatchPointDef;

typedef struct {
    const char *devTypeId;
    const IotBatchPointDef *points;
    int count;
    double *values;        // 长度为 count 的缓存，由使用方提供
} IotBatchCollector;

/* 缓存第 frnum 个点（从 1 开始）；frnum == count 时发送整包，返回 1 表示已发送 */
int IotBatchCollector_Update(IotBatchCollector *collector, int frnum, double value);
/* 读取最近一次缓存的值，frnum 无效时返回 -1 */
int IotBatchCollector_Get(const IotBatchCollector *collector, int frnum, double *value);

/*
 * 安规测试仪一组测试的结果（76T / 艾诺共用）：
 * 8 个测试项，每项 5 个测量值，ParameterValues / ParameterTags / ParameterChecks 用 "__" 连接
 */
#define IOT_TESTER_ITEMS    8
#define IOT_TESTER_VALUES   5
#define IOT_TESTER_NAME     "TTester"       // 报文中 devNumber / devName / devStatus 字段

typedef struct {
    int groupId;
    float values[IOT_TESTER_ITEMS][IOT_TESTER_VALUES];
    int tags[IOT_TESTER_ITEMS];     // 各位置的测试项编号
    int checks[IOT_TESTER_ITEMS];   // 各测试项结果
    int groupCheck;                 // 本组结果
    int totalCheck;                 // 总结果
} IotTesterGroup;

int IotReport_SendTesterGroup(const IotTesterGroup *g);

#endif
