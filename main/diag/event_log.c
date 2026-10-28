/*
 * 事件日志：spiffs 分区按环形写入固定长度的记录（不挂载文件系统）
 *
 *   分区布局  [0, 16K)    异常重启前的运行日志（一份，覆盖写）
 *             [16K, 末尾) 事件记录，每条 128 字节，每扇区 32 条；写到扇区开头时先擦除该扇区，
 *                         因此最旧的一个扇区（32 条）会被整体覆盖
 *
 *   启动时扫描一遍找到序号最大的记录，从它后面继续写。
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "iot_time.h"
#include "event_log.h"

static const char *TAG = "EVLOG";

#define SECTOR          4096
#define CRASH_SIZE      (4 * SECTOR)
#define EV_BASE         CRASH_SIZE
#define REC_SIZE        128
#define PER_SECTOR      (SECTOR / REC_SIZE)
#define EV_MAGIC        0x31564545u     // "EEV1"
#define CRASH_MAGIC     0x31485243u     // "CRH1"
#define QUEUE_LEN       16
#define CMD_CLEAR       0xFF            // 队列中的特殊记录：清空事件

typedef struct {
    uint32_t magic;
    uint32_t seq;           // 递增序号
    uint64_t timeMs;        // Unix 毫秒，未校时为 0
    uint32_t uptimeS;
    uint16_t boot;
    uint8_t level;
    uint8_t rsv;
    char text[REC_SIZE - 24];
} EvRecord;

_Static_assert(sizeof(EvRecord) == REC_SIZE, "EvRecord must be 128 bytes");

typedef struct {
    uint32_t magic;
    uint32_t len;
    uint64_t timeMs;
    uint32_t boot;
    char reason[24];
    uint32_t rsv[5];
} CrashHeader;      // 64 字节

static const esp_partition_t *s_part;
static QueueHandle_t s_queue;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_slots;            // 事件槽位数
static uint32_t s_head;             // 下一条写入的槽位（只由写任务修改）
static uint32_t s_nextSeq = 1;      // 下一条记录的序号（只由写任务修改）
static uint32_t s_lastSeq;          // 已写入 Flash 的最新序号
static uint32_t s_boot = 1;

static bool IsBlank(const EvRecord *r)
{
    return r->magic == 0xFFFFFFFFu;
}

static void ReadSlot(uint32_t slot, EvRecord *r)
{
    if (esp_partition_read(s_part, EV_BASE + slot * REC_SIZE, r, sizeof(*r)) != ESP_OK) {
        memset(r, 0, sizeof(*r));
    }
}

/* ------------------------------------------------------------ 写任务 */

// 序号在写任务中分配，保证 Flash 中的顺序与序号一致（多个任务同时记录时入队顺序不确定）
static void WriteRecord(EvRecord *r)
{
    uint32_t off = EV_BASE + s_head * REC_SIZE;

    r->seq = s_nextSeq++;

    if (s_head % PER_SECTOR == 0) {
        esp_partition_erase_range(s_part, off, SECTOR);
    }
    if (esp_partition_write(s_part, off, r, sizeof(*r)) != ESP_OK) {
        ESP_LOGW(TAG, "write failed");
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    s_head = (s_head + 1) % s_slots;
    s_lastSeq = r->seq;
    taskEXIT_CRITICAL(&s_lock);
}

static void ClearAll(void)
{
    esp_partition_erase_range(s_part, EV_BASE, s_slots * REC_SIZE);
    taskENTER_CRITICAL(&s_lock);
    s_head = 0;
    taskEXIT_CRITICAL(&s_lock);
}

static void EvLogTask(void *arg)
{
    EvRecord r;

    while (1) {
        if (xQueueReceive(s_queue, &r, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (r.level == CMD_CLEAR) {
            ClearAll();
        } else {
            WriteRecord(&r);
        }
    }
}

/* ------------------------------------------------------------ 初始化 */

// 找到序号最大的记录，从它后面继续写
static void Scan(void)
{
    EvRecord *buf = malloc(SECTOR);
    uint32_t maxSeq = 0;
    uint32_t maxSlot = 0;
    uint32_t maxBoot = 0;
    bool found = false;

    if (buf == NULL) {
        return;
    }
    for (uint32_t s = 0; s < s_slots / PER_SECTOR; s++) {
        if (esp_partition_read(s_part, EV_BASE + s * SECTOR, buf, SECTOR) != ESP_OK) {
            continue;
        }
        for (uint32_t i = 0; i < PER_SECTOR; i++) {
            const EvRecord *r = &buf[i];

            if (r->magic == EV_MAGIC && (!found || (int32_t)(r->seq - maxSeq) > 0)) {
                found = true;
                maxSeq = r->seq;
                maxSlot = s * PER_SECTOR + i;
                maxBoot = r->boot;
            }
        }
    }
    free(buf);

    if (found) {
        s_head = (maxSlot + 1) % s_slots;
        s_lastSeq = maxSeq;
        s_nextSeq = maxSeq + 1;
        s_boot = maxBoot + 1;
        // 正常情况下当前扇区剩余的槽位是空的；异常（写到一半断电等）时跳到下一扇区开头
        if (s_head % PER_SECTOR != 0) {
            EvRecord r;

            ReadSlot(s_head, &r);
            if (!IsBlank(&r)) {
                s_head = (s_head / PER_SECTOR + 1) * PER_SECTOR % s_slots;
            }
        }
    }
}

void EvLog_Init(void)
{
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
    if (s_part == NULL || s_part->size < CRASH_SIZE + 2 * SECTOR) {
        ESP_LOGE(TAG, "no log partition, events are not saved");
        return;
    }
    s_slots = (s_part->size - EV_BASE) / SECTOR * PER_SECTOR;
    Scan();

    s_queue = xQueueCreate(QUEUE_LEN, sizeof(EvRecord));
    if (s_queue == NULL || xTaskCreate(EvLogTask, "evlog", 3072, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "start failed");
        s_queue = NULL;
        return;
    }
    ESP_LOGD(TAG, "boot %u, %u slots, head %u", (unsigned)s_boot, (unsigned)s_slots, (unsigned)s_head);
}

uint32_t EvLog_BootNumber(void)
{
    return s_boot;
}

/* ------------------------------------------------------------ 记录 / 读取 */

void EvLog_Add(int level, const char *fmt, ...)
{
    EvRecord r;
    va_list args;

    if (s_queue == NULL) {
        return;
    }
    memset(&r, 0, sizeof(r));
    va_start(args, fmt);
    vsnprintf(r.text, sizeof(r.text), fmt, args);
    va_end(args);

    r.magic = EV_MAGIC;
    r.timeMs = IotTime_IsSynced() ? GetMilliTimeNow() : 0;
    r.uptimeS = (uint32_t)(esp_timer_get_time() / 1000000);
    r.boot = (uint16_t)s_boot;
    r.level = (uint8_t)level;

    if (xQueueSend(s_queue, &r, 0) != pdTRUE) {
        ESP_LOGW(TAG, "queue full, event dropped: %s", r.text);
    }
}

cJSON *EvLog_ToJson(int max, bool withDev)
{
    cJSON *arr = cJSON_CreateArray();
    uint32_t slot;
    uint32_t expect;
    EvRecord r;

    if (arr == NULL || s_part == NULL || s_slots == 0) {
        return arr;
    }
    taskENTER_CRITICAL(&s_lock);
    slot = s_head;
    expect = s_lastSeq;
    taskEXIT_CRITICAL(&s_lock);

    // 从最新一条往回读，序号不连续（被覆盖、已清空）时停止
    int n = 0;
    for (uint32_t i = 0; n < max && i < s_slots; i++) {
        slot = (slot + s_slots - 1) % s_slots;
        ReadSlot(slot, &r);
        if (r.magic != EV_MAGIC || r.seq != expect) {
            break;
        }
        expect = r.seq - 1;
        r.text[sizeof(r.text) - 1] = '\0';
        if ((r.level & EV_ACK) && !withDev) {
            break;          // 客户清除过，更早的不再给客户看
        }
        if ((r.level & EV_DEV) && !withDev) {
            continue;
        }
        n++;

        cJSON *o = cJSON_CreateObject();
        cJSON_AddNumberToObject(o, "t", (double)r.timeMs);
        cJSON_AddNumberToObject(o, "up", r.uptimeS);
        cJSON_AddNumberToObject(o, "b", r.boot);
        cJSON_AddNumberToObject(o, "l", r.level & EV_LEVEL_MASK);
        if (r.level & EV_OP) {
            cJSON_AddTrueToObject(o, "op");
        }
        cJSON_AddStringToObject(o, "m", r.text);
        cJSON_AddItemToArray(arr, o);
    }
    return arr;
}

void EvLog_Clear(void)
{
    EvRecord r = { .level = CMD_CLEAR };

    if (s_queue != NULL) {
        xQueueSend(s_queue, &r, pdMS_TO_TICKS(1000));
    }
}

/* ------------------------------------------------------------ 异常重启日志 */

void EvLog_SaveCrash(const char *reason, const char *data, size_t len)
{
    CrashHeader h = { 0 };

    if (s_part == NULL) {
        return;
    }
    if (len > CRASH_SIZE - sizeof(h)) {
        data += len - (CRASH_SIZE - sizeof(h));     // 只保留最后的部分
        len = CRASH_SIZE - sizeof(h);
    }
    h.magic = CRASH_MAGIC;
    h.len = len;
    h.timeMs = 0;
    h.boot = s_boot - 1;        // 异常发生在上一次启动
    strlcpy(h.reason, reason, sizeof(h.reason));

    esp_partition_erase_range(s_part, 0, CRASH_SIZE);
    esp_partition_write(s_part, 0, &h, sizeof(h));
    esp_partition_write(s_part, sizeof(h), data, len);
}

bool EvLog_ReadCrash(CrashInfo *info, char *buf, size_t size)
{
    CrashHeader h;

    if (s_part == NULL || esp_partition_read(s_part, 0, &h, sizeof(h)) != ESP_OK ||
        h.magic != CRASH_MAGIC || h.len > CRASH_SIZE - sizeof(h)) {
        return false;
    }
    if (info != NULL) {
        info->timeMs = h.timeMs;
        info->boot = h.boot;
        info->len = h.len;
        strlcpy(info->reason, h.reason, sizeof(info->reason));
    }
    if (buf != NULL && size > 0) {
        size_t n = (h.len < size - 1) ? h.len : size - 1;

        esp_partition_read(s_part, sizeof(h), buf, n);
        buf[n] = '\0';
    }
    return true;
}

void EvLog_ClearCrash(void)
{
    if (s_part != NULL) {
        esp_partition_erase_range(s_part, 0, SECTOR);   // 擦掉头部即可
    }
}
