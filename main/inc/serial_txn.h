#ifndef SERIAL_TXN_H
#define SERIAL_TXN_H

/*
 * 串口事务：一次"发请求 -> 等应答 -> 解析"在同一个任务里同步完成
 *
 * 协议层的读写函数内部这样使用（调用方只看到 FX_LinkRead() 之类的同步接口）：
 *
 *   SerialTxn_Begin();                          // 获取串口锁，清空接收缓冲
 *   SerialTxn_Send(frame, len);                 // 发请求
 *   ret = SerialTxn_Wait(ParseXxx, &ctx, SERIAL_TXN_TIMEOUT_MS);   // 等完整应答并解析
 *   SerialTxn_End();                            // 释放串口锁
 *
 * 解析函数用 SerialRx_xxx() 从接收缓冲读数据，返回值：
 *   SERIAL_PARSE_OK     收到完整且正确的应答，事务成功
 *   SERIAL_PARSE_MORE   数据还没收全：事务层回滚读位置后继续等待，已到的字节不会丢失
 *   SERIAL_PARSE_SKIP   读到的是中间帧（如 PPI 的 E5 短应答），已处理完，继续等后面的应答
 *   SERIAL_PARSE_ERROR  设备返回错误（NAK 等）：立即结束本次事务
 *
 * 所有使用 UART1 的地方（轮询任务、命令任务）都必须在 Begin/End 之间收发，
 * 避免一个任务的请求插进另一个任务的"请求-应答"中间。
 */

#include <stddef.h>
#include <stdint.h>
#include "sdkconfig.h"

#define SERIAL_TXN_TIMEOUT_MS         CONFIG_IOT_SERIAL_TIMEOUT_MS
/* 该选项只在启用测试仪时存在，但测试仪源文件始终参与编译，未启用时给一个默认值 */
#ifdef CONFIG_IOT_TESTER_TIMEOUT_MS
#define SERIAL_TXN_TESTER_TIMEOUT_MS  CONFIG_IOT_TESTER_TIMEOUT_MS
#else
#define SERIAL_TXN_TESTER_TIMEOUT_MS  1000
#endif

#define SERIAL_PARSE_OK      0
#define SERIAL_PARSE_MORE   (-1)
#define SERIAL_PARSE_ERROR  (-2)
#define SERIAL_PARSE_SKIP    1

typedef int (*SerialParseFn)(void *ctx);

void SerialTxn_Init(void);
void SerialTxn_Begin(void);
void SerialTxn_Send(const void *data, size_t len);
int  SerialTxn_Wait(SerialParseFn parse, void *ctx, int timeout_ms);   // 成功返回 0
void SerialTxn_End(void);

/* Begin + Send + Wait + End，成功返回 0 */
int  SerialTxn_Request(const void *req, size_t len, SerialParseFn parse, void *ctx, int timeout_ms);

/*
 * 解析函数使用的读接口：从事务读游标处读取，数据不够时返回 -1
 * （此时解析函数应返回 SERIAL_PARSE_MORE）
 */
int SerialRx_Read(void *buf, size_t len);
int SerialRx_Byte(uint8_t *b);

/* 跳过数据直到读到 a 或 b 中的一个字节，返回读到的字节；数据不够返回 -1 */
int SerialRx_SeekAny(uint8_t a, uint8_t b);

#endif
