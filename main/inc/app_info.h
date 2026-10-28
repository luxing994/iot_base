#ifndef APP_INFO_H
#define APP_INFO_H

/* 固件信息（启动信息块和配置网页共用） */

#define FW_VERSION  "V3"

/* 本固件启用的采集设备，如 "FX(RS485), MAX6675"，没有时为 "none" */
const char *App_CollectText(void);

/* 本次启动的复位原因 */
const char *App_ResetText(void);

#endif
