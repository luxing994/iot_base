#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "iot_time.h"

uint64_t g_baseTime = 0;

#define BASE_TIME_TOLERANCE_MS 10

uint64_t GetMilliTimeNow(void)
{
    return g_baseTime + esp_log_timestamp();
}

void CalBaseTime(uint64_t abstime)
{
    uint64_t timenow = GetMilliTimeNow();
    int64_t diff = (int64_t)(abstime - timenow);

    // abs() 只接受 int，64 位差值会被截断，改为 64 位比较
    if (diff > BASE_TIME_TOLERANCE_MS || diff < -BASE_TIME_TOLERANCE_MS) {
        g_baseTime = abstime - (timenow - g_baseTime);
    }
}
