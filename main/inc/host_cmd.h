#ifndef HOST_CMD_H
#define HOST_CMD_H

/*
 * 上位机 JSON 命令
 *
 * tcp_server.c 从命令端口收到一条完整 JSON 后调用 HostCmd_HandleMessage()，
 * 这里负责解析、保存命令状态、通知相应任务执行。其他任务只通过本文件的
 * 接口读取命令状态，读取接口加锁，可在任意任务中调用。
 *
 * 命令格式示例（group 模式，安规测试仪组参数）：
 * {
 *     "devId": "1531243721197228032", "devName": "", "devTypeId": "1516606339298758656",
 *     "deviceOrderMode": "group", "groupId": "001", "deviceOrderWay": "write",
 *     "orderDate": "2022-05-30 21:46:21", "orderId": "FR022", "orderName": "",
 *     "parameterType": "01_02_00_00_00_00_00_00",
 *     "parameters": [
 *         [{"type": "", "value": "1500"}, {"type": "", "value": "50.00"}, ...],   // 第 1 个测试项
 *         [{"type": "", "value": "25.00"}, ...]                                     // 第 2 个测试项
 *     ],
 *     "responseType": "", "timeStamp": "1653918381719"
 * }
 *
 * orderId 为 StartFlush / EndFlush / PowerOn / PowerOff 时为控制命令；
 * deviceOrderMode 为 single 时为冷媒灌注机充注量设定（parameters 为数值，parameterType 为 "A"/"B"），
 * 写入 PLC 并回读一致后回复 "true"，写入失败、回读不一致或超时回复 "false"；其他命令不回复。
 */

#include <stdbool.h>
#include "iot_common.h"

/* 上报报文中引用的命令字段（devId 等跟随上位机最近一次命令） */
typedef struct {
    char devId[CMD_STR_LEN];
    char devName[CMD_STR_LEN];
    char devTypeId[CMD_STR_LEN];
    char orderId[CMD_STR_LEN];
    char orderName[CMD_STR_LEN];
} HostCmdIdentity;

/*
 * 处理一条完整的 JSON 命令。
 * 返回需要回复给上位机的字符串（例如 "true"），无需回复时返回 NULL。
 */
const char *HostCmd_HandleMessage(const char *json);

/* 最近一次命令中的设备标识字段（拷贝一份快照） */
void HostCmd_GetIdentity(HostCmdIdentity *out);

/* 是否允许采集：上位机 EndFlush 后为 false，StartFlush 后为 true（默认 true） */
bool HostCmd_IsCollectEnabled(void);

/* 安规测试仪：最近一次 group 命令的组号，0 表示读取当前组 */
int HostCmd_GetTesterGroup(void);

/* 冷媒灌注机：最近一次 single 命令的充注量设定 */
void HostCmd_GetChargeSetting(SNSetCaclReFillingMachine *out);

#endif
