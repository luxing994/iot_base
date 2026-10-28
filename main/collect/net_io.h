#ifndef NET_IO_H
#define NET_IO_H

/*
 * 网口 PLC 的 socket 收发工具
 *
 * TCP 是字节流，一次 recv 不保证收到完整应答。协议层先按帧头读出长度，
 * 再用 Net_RecvExact 读够剩余字节。
 */

#include <stddef.h>

/* 设置收发超时，超时后 recv/send 返回失败（视为链路失效，由调用方重连） */
void Net_SetTimeout(int sock, int timeout_ms);

/* 发送全部数据，成功返回 0 */
int Net_SendAll(int sock, const void *data, size_t len);

/* 接收恰好 len 字节，成功返回 0；对端关闭、出错或超时返回 -1 */
int Net_RecvExact(int sock, void *buf, size_t len);

/* 丢弃 len 字节（应答中不需要的部分），成功返回 0 */
int Net_Discard(int sock, size_t len);

#endif
