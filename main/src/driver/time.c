#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "time.h"

uint64_t g_baseTime = 0;

uint64_t GetMilliTimeNow()
{
    return g_baseTime + esp_log_timestamp();
}

void CalBaseTime(uint64_t abstime)
{   
    uint64_t timenow;

    timenow = GetMilliTimeNow();
    if (abs(abstime - timenow) > 10) {
        g_baseTime = abstime - (timenow - g_baseTime);
    }
}
