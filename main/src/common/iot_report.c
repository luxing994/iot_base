#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "iot_common.h"
#include "iot_report.h"
#include "iot_time.h"
#include "tcp_client.h"
#include "tcp_server.h"

#define VALUE_MSG_SIZE      768
#define BATCH_MSG_EXTRA     512     // 批量报文中除三个参数串以外的固定字段长度
#define VALUE_STR_SIZE      48
#define BATCH_IDS_SIZE      (IOT_BATCH_MAX_POINTS * 8)
#define BATCH_VALUES_SIZE   (IOT_BATCH_MAX_POINTS * 24)
#define BATCH_UNITS_SIZE    (IOT_BATCH_MAX_POINTS * 8)
#define BATCH_SEPARATOR     "__"

static const char *TAG = "IOT_REPORT";

struct IotBatch {
    int count;
    char ids[BATCH_IDS_SIZE];
    char values[BATCH_VALUES_SIZE];
    char units[BATCH_UNITS_SIZE];
};

// 消息所有权交给队列，由 tcp_client_task 发送后 free；入队失败在这里释放
static int PostOwnedMessage(char *msg)
{
    if (xQueue1 == NULL || xQueueSend(xQueue1, &msg, IOT_MSG_QUEUE_TIMEOUT) != pdPASS) {
        ESP_LOGW(TAG, "report queue full, drop message");
        free(msg);
        return -1;
    }
    return 0;
}

static const char *OrDefault(const char *value, const char *def)
{
    return (value != NULL) ? value : def;
}

int IotReport_SendValueMsg(const IotValueMsg *msg)
{
    const CommandJsonData *cmd = GetCommandJsonData();
    char *buf;
    int len;

    if (msg == NULL || msg->value == NULL) {
        return -1;
    }

    buf = malloc(VALUE_MSG_SIZE);
    if (buf == NULL) {
        ESP_LOGE(TAG, "no memory for report");
        return -1;
    }

    len = snprintf(buf, VALUE_MSG_SIZE, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"
        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
        "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%s\",\n    \"expand\":\"NULL\",\n    \"isAnswer\":\"no\"\n}" IOT_MSG_END,
        OrDefault(msg->devNumber, IOT_DEV_NUMBER), OrDefault(msg->devId, cmd->devId),
        OrDefault(msg->devName, cmd->devName), OrDefault(msg->devTypeId, ""), OrDefault(msg->devTypeName, ""),
        GetStaIp(), OrDefault(msg->orderId, ""), OrDefault(msg->orderName, cmd->orderName),
        DEVTIMEMODE, (long long)GetMilliTimeNow(), msg->value);
    if (len < 0 || len >= VALUE_MSG_SIZE) {
        ESP_LOGE(TAG, "report too long: %d", len);
        free(buf);
        return -1;
    }

    ESP_LOGD(TAG, "%s", buf);
    return PostOwnedMessage(buf);
}

static int SendValueV(const char *devTypeId, const char *devTypeName, const char *orderId,
                      const char *fmt, va_list ap)
{
    char value[VALUE_STR_SIZE];
    IotValueMsg msg = {
        .devTypeId = devTypeId,
        .devTypeName = devTypeName,
        .orderId = orderId,
        .value = value,
    };

    vsnprintf(value, sizeof(value), fmt, ap);
    return IotReport_SendValueMsg(&msg);
}

int IotReport_SendValue(const char *devTypeId, const char *devTypeName, const char *orderId,
                        const char *fmt, ...)
{
    va_list ap;
    int ret;

    va_start(ap, fmt);
    ret = SendValueV(devTypeId, devTypeName, orderId, fmt, ap);
    va_end(ap);
    return ret;
}

int IotReport_SendPoint(const char *devTypeId, const char *devTypeName, int frnum,
                        const char *fmt, ...)
{
    char orderId[16];     // "FR" + 最长 11 位整数 + 结尾 0
    va_list ap;
    int ret;

    snprintf(orderId, sizeof(orderId), "FR%03d", frnum);
    va_start(ap, fmt);
    ret = SendValueV(devTypeId, devTypeName, orderId, fmt, ap);
    va_end(ap);
    return ret;
}

/* ---------------------------------------------------------------- 批量报文 */

IotBatch *IotBatch_Create(void)
{
    IotBatch *batch = calloc(1, sizeof(IotBatch));
    if (batch == NULL) {
        ESP_LOGE(TAG, "no memory for batch");
    }
    return batch;
}

void IotBatch_Destroy(IotBatch *batch)
{
    free(batch);
}

void IotBatch_Reset(IotBatch *batch)
{
    if (batch != NULL) {
        memset(batch, 0, sizeof(*batch));
    }
}

int IotBatch_Count(const IotBatch *batch)
{
    return (batch != NULL) ? batch->count : 0;
}

// 在 dst 末尾追加 "__" + item，空间不足返回 -1
static int AppendItem(char *dst, size_t size, int first, const char *item)
{
    size_t used = strlen(dst);
    int n = snprintf(dst + used, size - used, "%s%s", first ? "" : BATCH_SEPARATOR, item);
    return (n < 0 || (size_t)n >= size - used) ? -1 : 0;
}

int IotBatch_Add(IotBatch *batch, const char *unit, const char *fmt, ...)
{
    char id[16];          // "FR" + 最长 11 位整数 + 结尾 0
    char value[VALUE_STR_SIZE];
    va_list ap;
    int first;

    if (batch == NULL || batch->count >= IOT_BATCH_MAX_POINTS) {
        return -1;
    }

    va_start(ap, fmt);
    vsnprintf(value, sizeof(value), fmt, ap);
    va_end(ap);

    first = (batch->count == 0);
    snprintf(id, sizeof(id), "FR%03d", batch->count + 1);
    if (AppendItem(batch->ids, sizeof(batch->ids), first, id) != 0 ||
        AppendItem(batch->values, sizeof(batch->values), first, value) != 0 ||
        AppendItem(batch->units, sizeof(batch->units), first, (unit != NULL) ? unit : "00") != 0) {
        ESP_LOGE(TAG, "batch buffer overflow");
        return -1;
    }
    batch->count++;

    return 0;
}

int IotBatch_Send(const IotBatch *batch, const char *devTypeId)
{
    const CommandJsonData *cmd = GetCommandJsonData();
    size_t size;
    char *buf;
    int len;

    if (batch == NULL || batch->count == 0) {
        return -1;
    }

    size = strlen(batch->ids) + strlen(batch->values) + strlen(batch->units) + BATCH_MSG_EXTRA;
    buf = malloc(size);
    if (buf == NULL) {
        ESP_LOGE(TAG, "no memory for batch report");
        return -1;
    }

    len = snprintf(buf, size, "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"\",\n    \"devName\":\"\",\n    \"devStatus\":\"\",\n"
        "    \"devTypeId\": \"%s\",\n    \"orderName\":\"%s\",\n    \"orderId\":\"%s\",\n"
        "    \"ParameterIds\":\"%s\",\n"
        "    \"ParameterValues\":\"%s\",\n"
        "    \"ParameterUnits\":\"%s\",\n"
        "    \"value\":\"\",\n    \"devIP\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
        "    \"valueUnit\":\"\",\n    \"expand\":\"\",\n    \"isAnswer\":\"no\"\n}" IOT_MSG_END,
        cmd->devId, devTypeId, cmd->orderName, IOT_BATCH_ORDER_ID,
        batch->ids, batch->values, batch->units,
        GetStaIp(), DEVTIMEMODE, (long long)GetMilliTimeNow());
    if (len < 0 || (size_t)len >= size) {
        ESP_LOGE(TAG, "batch report too long: %d", len);
        free(buf);
        return -1;
    }

    ESP_LOGD(TAG, "%s", buf);
    return PostOwnedMessage(buf);
}

/* ---------------------------------------------------------------- 批量采集器 */

int IotBatchCollector_Update(IotBatchCollector *collector, int frnum, double value)
{
    IotBatch *batch;
    int i;

    if (collector == NULL || frnum < 1 || frnum > collector->count) {
        return -1;
    }

    collector->values[frnum - 1] = value;
    if (frnum != collector->count) {
        return 0;
    }

    batch = IotBatch_Create();
    if (batch == NULL) {
        return -1;
    }
    for (i = 0; i < collector->count; i++) {
        const IotBatchPointDef *def = &collector->points[i];
        double v = collector->values[i] * ((def->scale != 0.0f) ? def->scale : 1.0f);

        if (def->decimals == 0) {
            IotBatch_Add(batch, def->unit, "%d", (int)v);
        } else {
            IotBatch_Add(batch, def->unit, "%.*f", (int)def->decimals, v);
        }
    }
    IotBatch_Send(batch, collector->devTypeId);
    IotBatch_Destroy(batch);

    return 1;
}

int IotBatchCollector_Get(const IotBatchCollector *collector, int frnum, double *value)
{
    if (collector == NULL || value == NULL || frnum < 1 || frnum > collector->count) {
        return -1;
    }
    *value = collector->values[frnum - 1];
    return 0;
}
