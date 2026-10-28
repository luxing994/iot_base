/*
 * 报文监视，说明见 monitor.h
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "iot_time.h"
#include "monitor.h"

static const char *TAG = "MON";

#define MON_SLOTS       128
#define MON_BYTES_MAX   (32 * 1024)
#define MON_REC_MAX     1024
#define MON_MERGE_MS    30              // 同通道同方向间隔小于此值的数据合并为一条
#define MON_SOCKS       16
#define LOCK_WAIT_MS    20              // 拿不到锁时丢弃本次记录，不阻塞收发

typedef struct {
    uint32_t seq;
    uint32_t upMs;
    uint64_t timeMs;
    uint8_t ch;
    uint8_t dir;
    uint16_t len;           // 原始长度（可能大于保存的 n）
    uint16_t n;             // 保存的字节数
    uint8_t *data;
} MonRec;

static SemaphoreHandle_t s_mutex;
static MonRec s_recs[MON_SLOTS];
static uint32_t s_first;                // 最旧一条的下标
static uint32_t s_count;
static uint32_t s_nextSeq = 1;
static size_t s_bytes;
static uint32_t s_lost;

static volatile bool s_on;
static volatile uint32_t s_mask;
static int64_t s_stopAtUs;

static struct {
    int sock;
    uint8_t ch;
} s_socks[MON_SOCKS];
static portMUX_TYPE s_sockLock = portMUX_INITIALIZER_UNLOCKED;

/* ------------------------------------------------------------ 存储（调用方持有 s_mutex） */

static MonRec *Last(void)
{
    return s_count ? &s_recs[(s_first + s_count - 1) % MON_SLOTS] : NULL;
}

static void DropOldest(void)
{
    MonRec *r = &s_recs[s_first];

    s_bytes -= r->n;
    free(r->data);
    r->data = NULL;
    s_first = (s_first + 1) % MON_SLOTS;
    s_count--;
}

static void FreeAll(void)
{
    while (s_count > 0) {
        DropOldest();
    }
    s_lost = 0;
}

/* ------------------------------------------------------------ 接口 */

void Mon_Init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
    for (int i = 0; i < MON_SOCKS; i++) {
        s_socks[i].sock = -1;
    }
}

void Mon_Start(uint32_t mask, uint32_t minutes)
{
    if (s_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    FreeAll();
    s_mask = mask;
    s_stopAtUs = esp_timer_get_time() + (int64_t)minutes * 60 * 1000000;
    s_on = true;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "start, channels 0x%02x, %u min", (unsigned)mask, (unsigned)minutes);
}

void Mon_Stop(void)
{
    if (s_on) {
        s_on = false;
        ESP_LOGI(TAG, "stop");
    }
}

void Mon_Clear(void)
{
    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        FreeAll();
        xSemaphoreGive(s_mutex);
    }
}

void Mon_Capture(MonChannel ch, MonDir dir, const void *data, size_t len)
{
    uint32_t now;
    MonRec *last;
    size_t keep;

    if (!s_on || !(s_mask & (1u << ch)) || len == 0 || s_mutex == NULL) {
        return;
    }
    if (esp_timer_get_time() > s_stopAtUs) {
        Mon_Stop();
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(LOCK_WAIT_MS)) != pdTRUE) {
        return;
    }
    now = (uint32_t)(esp_timer_get_time() / 1000);
    last = Last();

    // 合并：同通道同方向、间隔很短，且上一条没有被截断
    if (last != NULL && last->ch == ch && last->dir == dir && now - last->upMs <= MON_MERGE_MS &&
        last->n == last->len && last->n + len <= MON_REC_MAX && s_bytes + len <= MON_BYTES_MAX) {
        uint8_t *p = realloc(last->data, last->n + len);

        if (p != NULL) {
            memcpy(p + last->n, data, len);
            last->data = p;
            last->n += len;
            last->len += len;
            last->upMs = now;
            s_bytes += len;
            xSemaphoreGive(s_mutex);
            return;
        }
    }

    keep = (len < MON_REC_MAX) ? len : MON_REC_MAX;
    while (s_count > 0 && (s_count == MON_SLOTS || s_bytes + keep > MON_BYTES_MAX)) {
        DropOldest();
        s_lost++;
    }
    MonRec *r = &s_recs[(s_first + s_count) % MON_SLOTS];

    r->data = malloc(keep);
    if (r->data != NULL) {
        memcpy(r->data, data, keep);
        r->seq = s_nextSeq++;
        r->upMs = now;
        r->timeMs = IotTime_IsSynced() ? GetMilliTimeNow() : 0;
        r->ch = (uint8_t)ch;
        r->dir = (uint8_t)dir;
        r->len = (uint16_t)((len > 0xFFFF) ? 0xFFFF : len);
        r->n = (uint16_t)keep;
        s_bytes += keep;
        s_count++;
    }
    xSemaphoreGive(s_mutex);
}

void Mon_BindSocket(int sock, MonChannel ch)
{
    int slot = -1;

    taskENTER_CRITICAL(&s_sockLock);
    for (int i = 0; i < MON_SOCKS; i++) {
        if (s_socks[i].sock == sock) {
            slot = i;
            break;
        }
        if (slot < 0 && s_socks[i].sock < 0) {
            slot = i;
        }
    }
    if (slot < 0) {
        slot = 0;       // 表满（socket 号被反复复用时可能出现），覆盖第一个
    }
    s_socks[slot].sock = sock;
    s_socks[slot].ch = (uint8_t)ch;
    taskEXIT_CRITICAL(&s_sockLock);
}

void Mon_CaptureSock(int sock, MonDir dir, const void *data, size_t len)
{
    int ch = -1;

    if (!s_on) {
        return;
    }
    taskENTER_CRITICAL(&s_sockLock);
    for (int i = 0; i < MON_SOCKS; i++) {
        if (s_socks[i].sock == sock) {
            ch = s_socks[i].ch;
            break;
        }
    }
    taskEXIT_CRITICAL(&s_sockLock);
    if (ch >= 0) {
        Mon_Capture((MonChannel)ch, dir, data, len);
    }
}

cJSON *Mon_ToJson(uint32_t after, size_t maxBytes)
{
    static const char hex[] = "0123456789abcdef";
    cJSON *root = cJSON_CreateObject();
    cJSON *items = cJSON_AddArrayToObject(root, "items");
    uint32_t next = after;
    size_t total = 0;
    int64_t left;

    if (s_mutex == NULL) {
        return root;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

    for (uint32_t i = 0; i < s_count && total < maxBytes; i++) {
        const MonRec *r = &s_recs[(s_first + i) % MON_SLOTS];
        char *x;

        if (r->seq <= after) {
            continue;
        }
        // 最后一条可能还会合并后续数据：返回后网页不会再取它，所以等合并窗口过去再返回
        if (i == s_count - 1 && s_on && now - r->upMs <= MON_MERGE_MS) {
            break;
        }
        x = malloc((size_t)r->n * 2 + 1);
        if (x == NULL) {
            break;
        }
        for (uint16_t k = 0; k < r->n; k++) {
            x[k * 2] = hex[r->data[k] >> 4];
            x[k * 2 + 1] = hex[r->data[k] & 0x0F];
        }
        x[r->n * 2] = '\0';

        cJSON *o = cJSON_CreateObject();
        cJSON_AddNumberToObject(o, "s", r->seq);
        cJSON_AddNumberToObject(o, "t", (double)r->timeMs);
        cJSON_AddNumberToObject(o, "up", r->upMs);
        cJSON_AddNumberToObject(o, "c", r->ch);
        cJSON_AddNumberToObject(o, "d", r->dir);
        cJSON_AddNumberToObject(o, "n", r->len);
        cJSON_AddStringToObject(o, "x", x);
        cJSON_AddItemToArray(items, o);
        free(x);
        next = r->seq;
        total += r->n;
    }
    xSemaphoreGive(s_mutex);

    if (s_on && esp_timer_get_time() > s_stopAtUs) {
        Mon_Stop();
    }
    left = s_on ? (s_stopAtUs - esp_timer_get_time()) / 1000000 : 0;
    cJSON_AddBoolToObject(root, "on", s_on);
    cJSON_AddNumberToObject(root, "left", (double)(left > 0 ? left : 0));
    cJSON_AddNumberToObject(root, "mask", s_mask);
    cJSON_AddNumberToObject(root, "next", next);
    cJSON_AddNumberToObject(root, "lost", s_lost);
    return root;
}
