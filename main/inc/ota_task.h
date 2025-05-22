#ifndef OTA_TASK_H
#define OTA_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// OTA 任务入口：pvParameter 传入固件下载 URL 字符串
void ota_task(void *pvParameter);

#endif // OTA_TASK_H
