#ifndef SERIAL_TXN_H
#define SERIAL_TXN_H

/*
 * 串口事务：一次"发请求 -> 等应答 -> 解析"在同一个任务里同步完成
 *
 *   SerialTxn_Begin();                                  // 获取串口锁，清空接收缓冲
 *   SerialReadSingleDataRegister(0, 255, 10, 214, 4);   // 协议层发送请求（同时记录 FR 编号和数据格式）
 *   if (SerialTxn_Wait(GetSerialDataFromFxPlc, SERIAL_TXN_TIMEOUT_MS) == 0) {
 *       PlcReport_Fx();                                 // 收到完整应答，立即解码上报
 *   }
 *   SerialTxn_End();                                    // 释放串口锁
 *
 * 解析函数在应答没收全时返回非 0，事务层回滚读位置后继续等待，已到的字节不会丢失；
 * 只有解析成功才真正消耗缓冲区中的数据。超时则丢弃本次应答。
 *
 * 所有使用 UART1 的地方（轮询任务、命令任务）都必须在 Begin/End 之间收发，
 * 避免一个任务的请求插进另一个任务的"请求-应答"中间。
 */

#include "sdkconfig.h"

#define SERIAL_TXN_TIMEOUT_MS         CONFIG_IOT_SERIAL_TIMEOUT_MS
#define SERIAL_TXN_TESTER_TIMEOUT_MS  CONFIG_IOT_TESTER_TIMEOUT_MS

typedef int (*SerialParseFn)(void);   // 协议层应答解析函数，0 表示收到完整且正确的应答

void SerialTxn_Init(void);
void SerialTxn_Begin(void);
int  SerialTxn_Wait(SerialParseFn parse, int timeout_ms);
void SerialTxn_End(void);

/* 便捷宏：执行一次完整事务，成功后执行 on_ok */
#define SERIAL_TXN(request, parse, timeout_ms, on_ok)            \
    do {                                                         \
        SerialTxn_Begin();                                       \
        request;                                                 \
        if (SerialTxn_Wait((parse), (timeout_ms)) == 0) {        \
            on_ok;                                               \
        }                                                        \
        SerialTxn_End();                                         \
    } while (0)

#endif
