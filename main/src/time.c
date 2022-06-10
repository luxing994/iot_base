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

void InitBaseTime(uint64_t abstime)
{   
    g_baseTime = abstime - GetMilliTimeNow();
}
