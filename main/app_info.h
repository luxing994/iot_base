#ifndef APP_INFO_H
#define APP_INFO_H

/* 固件信息（启动信息块和配置网页共用） */

#define FW_VERSION  "V3"

/* 本固件启用的采集设备，如 "FX(RS485), MAX6675"，没有时为 "none" */
const char *App_CollectText(void);

/* 本次启动的复位原因 */
const char *App_ResetText(void);

/* 各应用任务的历史最小剩余栈，每个任务一行交给 put（诊断报告用） */
void App_WriteTaskStacks(void (*put)(void *ctx, const char *line), void *ctx);

#endif
