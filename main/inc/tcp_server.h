#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "iot_common.h"

/*
 * 本机 TCP 服务端：
 *   tcp_server_task   CONFIG_SERVER_PORT  : 接收上位机 JSON 命令
 *   tcp_server1_task  CONFIG_SERVER_PORT1 : 接收文件分包
 */
void tcp_server_task(void *pvParameters);
void tcp_server1_task(void *pvParameters);

/* 上位机最近一次下发的命令（字段为副本，可随时读取） */
const CommandJsonData *GetCommandJsonData(void);
int GetGroupIdFromRecvJsonData(void);

/* 命令解析结果，供采集 / 控制任务使用 */
extern uint32_t g_devStartFlushFlag;                        // 0: 上位机要求暂停采集（EndFlush）
extern int g_snttestercurrentgroup;                         // 安规测试仪要读取的历史组号，0 表示当前组
extern SNSetCaclReFillingMachine g_setsnrefillingmachine;   // 冷媒灌注机充注量设定命令

/* 文件传输与配置项（乐江罗拉车等设备使用） */
int GetFileData(uint8_t *data, int size);
int GetTaskNum(uint16_t *tasknum);
int GetTaskPitch(uint16_t *taskpitch);
int GetTaskSpeed(uint16_t *taskspeed);
int GetTaskCount(uint16_t *taskcount);
int GetTaskTime(uint16_t *tasktime);
int GetMode(uint8_t *mode);

#endif
