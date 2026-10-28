#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "sdkconfig.h"

/*
 * 本机 TCP 服务端，pvParameters 为地址族（AF_INET / AF_INET6）
 *   tcp_server_task   CONFIG_IOT_CMD_PORT  : 接收上位机 JSON 命令（处理见 host_cmd.h）
 *   tcp_server1_task  CONFIG_IOT_FILE_PORT : 接收文件分包（处理见 file_transfer.h）
 */
void tcp_server_task(void *pvParameters);
#ifdef CONFIG_IOT_FILE_SERVER
void tcp_server1_task(void *pvParameters);
#endif

#endif
