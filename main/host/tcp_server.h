#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "sdkconfig.h"

/*
 * 本机 TCP 服务端，pvParameters 为地址族（AF_INET / AF_INET6）
 *   tcp_server_task   命令端口 cmdPort  : 接收上位机 JSON 命令（处理见 host_cmd.h）
 *   tcp_server1_task  文件端口 filePort : 接收文件分包（处理见 file_transfer.h）
 */
void tcp_server_task(void *pvParameters);
#ifdef CONFIG_IOT_FILE_SERVER
void tcp_server1_task(void *pvParameters);
#endif

#endif
