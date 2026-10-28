#include "freertos/FreeRTOS.h"
#include "freertos/task.h"      // taskENTER_CRITICAL / taskEXIT_CRITICAL
#include "esp_timer.h"
#include "iot_time.h"

#define BASE_TIME_TOLERANCE_MS 10

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_offsetMs;      // 绝对时间 - 开机时长，未同步时为 0

// 开机时长（毫秒）。esp_timer 为 64 位，不会像 32 位的 esp_log_timestamp() 那样约 49.7 天回绕
static int64_t UptimeMs(void)
{
    return esp_timer_get_time() / 1000;
}

uint64_t GetMilliTimeNow(void)
{
    int64_t offset;

    // 64 位变量在 32 位 CPU 上不是原子读写，加锁避免读到一半被另一个任务改掉
    taskENTER_CRITICAL(&s_lock);
    offset = s_offsetMs;
    taskEXIT_CRITICAL(&s_lock);

    return (uint64_t)(offset + UptimeMs());
}

void CalBaseTime(uint64_t abstime)
{
    const int64_t uptime = UptimeMs();

    if (abstime == 0) {
        return;     // 解析失败（atoll 返回 0），忽略
    }
    taskENTER_CRITICAL(&s_lock);
    int64_t diff = (int64_t)abstime - (s_offsetMs + uptime);
    if (diff > BASE_TIME_TOLERANCE_MS || diff < -BASE_TIME_TOLERANCE_MS) {
        s_offsetMs = (int64_t)abstime - uptime;
    }
    taskEXIT_CRITICAL(&s_lock);
}
