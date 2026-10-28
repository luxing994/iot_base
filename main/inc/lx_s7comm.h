#ifndef S7COMM_ESP32_H
#define S7COMM_ESP32_H

/*
 * 西门子 S7comm（ISO-on-TCP，端口 102），用于 S7-200 SMART 等网口 PLC
 *
 *   连接：tcp_client.c 建立 TCP 连接后调用 S7_Handshake()（COTP 连接 + 通信设置）
 *   读取：S7_NetRead()，每次读一个变量，变量描述见 s7_area.h
 */

#include <stdint.h>
#include "plc_value.h"
#include "s7_area.h"

#define S7_TCP_PORT 102

/* 成功返回 0，失败返回 -1（调用方关闭 socket 后重连） */
int S7_Handshake(int sock);

/* 返回 PLC_OK、PLC_ERR_RESPONSE 或 PLC_ERR_LINK（需重连） */
int S7_NetRead(int sock, const S7Var *var, PlcValue *out);

#endif // S7COMM_ESP32_H
