/*
 * 运行日志缓冲：挂在 esp_log 的输出函数上，环形保存最近 LOG_BUF_SIZE 字节
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "event_log.h"
#include "log_buffer.h"

#define RING_MAGIC  0x31474F4Cu     // "LOG1"
#define LINE_STACK  160             // 单条日志先在栈上格式化
#define LINE_MAX    1024            // 单条日志最长保存长度，超过部分截断（只影响缓冲，不影响串口输出）

typedef struct {
    uint32_t magic;
    uint32_t total;         // 累计写入字节数，写入位置 = total % LOG_BUF_SIZE
    uint32_t check;         // magic ^ total，用于判断复位后内容是否有效
    char data[LOG_BUF_SIZE];
} LogRing;

/* 复位后不清零（上电时内容随机，由 magic / check 判断） */
static __NOINIT_ATTR LogRing s_ring;

/* 开机日志：每次启动后最先的 BOOT_LOG_SIZE 字节单独保存，写满后不再写入（启动信息块不会被后续日志冲掉） */
static char s_bootLog[BOOT_LOG_SIZE];
static size_t s_bootLen;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_orig;
static bool s_inEsc;        // 正在跳过颜色控制符 ESC[...m

// 调用方持有 s_lock
static void Append(const char *s, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        char c = s[i];

        if (s_inEsc) {
            if (c == 'm') {
                s_inEsc = false;
            }
            continue;
        }
        if (c == 0x1B) {
            s_inEsc = true;
            continue;
        }
        s_ring.data[s_ring.total % LOG_BUF_SIZE] = c;
        s_ring.total++;
        if (s_bootLen < BOOT_LOG_SIZE) {
            s_bootLog[s_bootLen++] = c;
        }
    }
    s_ring.check = RING_MAGIC ^ s_ring.total;
}

static int Hook(const char *fmt, va_list args)
{
    char line[LINE_STACK];      // 大多数日志放得下；栈上不能开太大（有的任务栈很小）
    char *text = line;
    va_list copy;
    int n;

    va_copy(copy, args);
    n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);
    if (n >= (int)sizeof(line)) {
        // 长日志（如 HEALTH）在堆上重新格式化，超过 LINE_MAX 的部分截断
        size_t size = (n < LINE_MAX) ? (size_t)n + 1 : LINE_MAX;
        char *big = malloc(size);

        if (big != NULL) {
            va_copy(copy, args);
            vsnprintf(big, size, fmt, copy);
            va_end(copy);
            text = big;
        } else {
            size = sizeof(line);    // 内存不足：保存栈上已格式化的部分
        }
        if (n >= (int)size) {
            n = (int)size - 1;
            text[n - 1] = '\n';     // 被截断时保证以换行结尾
        }
    }
    if (n > 0) {
        taskENTER_CRITICAL(&s_lock);
        Append(text, (size_t)n);
        taskEXIT_CRITICAL(&s_lock);
    }
    if (text != line) {
        free(text);
    }
    return s_orig(fmt, args);
}

static const char *AbnormalReset(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_BROWNOUT:  return "brownout";
    default:                return NULL;
    }
}

void LogBuf_Init(void)
{
    const char *abnormal = AbnormalReset(esp_reset_reason());
    bool valid = (s_ring.magic == RING_MAGIC && s_ring.check == (RING_MAGIC ^ s_ring.total));

    // 异常复位：复位前的日志还在内存里，保存到 Flash
    if (abnormal != NULL && valid && s_ring.total > 0) {
        size_t len = (s_ring.total < LOG_BUF_SIZE) ? s_ring.total : LOG_BUF_SIZE;
        uint32_t start = s_ring.total - len;
        char *copy = malloc(len);

        if (copy != NULL) {
            for (size_t i = 0; i < len; i++) {
                copy[i] = s_ring.data[(start + i) % LOG_BUF_SIZE];
            }
            EvLog_SaveCrash(abnormal, copy, len);
            free(copy);
        }
    }

    s_ring.magic = RING_MAGIC;
    s_ring.total = 0;
    s_ring.check = RING_MAGIC;
    s_orig = esp_log_set_vprintf(Hook);
}

size_t LogBuf_ReadBoot(char *buf, size_t size)
{
    size_t n;

    taskENTER_CRITICAL(&s_lock);
    n = (s_bootLen < size) ? s_bootLen : size;
    memcpy(buf, s_bootLog, n);
    taskEXIT_CRITICAL(&s_lock);
    return n;
}

size_t LogBuf_Read(uint32_t from, char *buf, size_t size, uint32_t *pos)
{
    size_t n;

    taskENTER_CRITICAL(&s_lock);
    uint32_t total = s_ring.total;
    uint32_t oldest = (total > LOG_BUF_SIZE) ? total - LOG_BUF_SIZE : 0;

    if (from < oldest || from > total) {
        from = oldest;
    }
    n = total - from;
    if (n > size) {
        n = size;
    }
    for (size_t i = 0; i < n; i++) {
        buf[i] = s_ring.data[(from + i) % LOG_BUF_SIZE];
    }
    taskEXIT_CRITICAL(&s_lock);

    *pos = from + n;
    return n;
}
