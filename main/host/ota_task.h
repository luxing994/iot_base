#ifndef OTA_TASK_H
#define OTA_TASK_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * OTA 升级（由 OTA 控制通道的 OTA_UPDATE:<url> 命令触发）
 *
 *   if (OTA_TryBegin()) {
 *       创建 ota_task 任务，参数为 strdup 出来的 URL；创建失败时调用 OTA_Abort()
 *   }
 * 同一时间只允许一个 OTA，OTA_TryBegin() 原子地占用，ota_task 结束时自动释放。
 */
bool OTA_TryBegin(void);
void OTA_Abort(void);

// OTA 任务入口：pvParameter 为 malloc 得到的固件下载 URL，任务结束时释放
void ota_task(void *pvParameter);

#endif // OTA_TASK_H
