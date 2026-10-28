#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "iot_common.h"
#include "iot_time.h"

RingBuffer uart2Buffer;

int UART_InitBuffer(void)
{
    if (RING_InitBuffer(&uart2Buffer, UART_BUFF_SIZE) != 0) {
        return -1;
    }
    // 串口接收缓冲使用事务模式：帧未收全时回滚，已读字节不丢失（见 serial_txn.c）
    RING_SetTransactional(&uart2Buffer, 1);

    return 0;
}

int UART_WriteBufferBytes(uint8_t *data, uint32_t size)
{
    if (RING_WriteBufferBytes(&uart2Buffer, data, size) != 0) {
        return -1;
    }
	return 0;
}

int UART_ReadBufferBytes(uint8_t *data, uint32_t size)
{
    if (RING_ReadBufferBytes(&uart2Buffer, data, size) != 0) {
        return -1;
    }
	return 0;
}

/*
 * 上报队列入队，msg 的所有权交给队列（tcp_client_task 发送后 free）。
 * 队列满（上位机断线、发送跟不上）时丢弃最旧的一条再入队，不阻塞调用方：
 * 原实现每条等待 100ms，断线期间会把所有采集任务拖慢；丢旧留新也保证重连后先发的是较新的数据。
 */
int IOT_PostOwnedMessage(char *msg)
{
    char *oldest = NULL;

    if (msg == NULL || xQueue1 == NULL) {
        free(msg);
        return -1;
    }
    if (xQueueSend(xQueue1, &msg, 0) == pdPASS) {
        return 0;
    }
    if (xQueueReceive(xQueue1, &oldest, 0) == pdPASS) {
        free(oldest);
    }
    if (xQueueSend(xQueue1, &msg, 0) != pdPASS) {
        free(msg);
        return -1;
    }
    return 0;
}

int IOT_PostMessage(const char *msg)
{
    if (msg == NULL || msg[0] == '\0') {
        return -1;
    }
    // 入队的是独立副本，避免多个任务共用同一缓冲区被覆盖
    return IOT_PostOwnedMessage(strdup(msg));
}

int get_random_range_freq(int min_val, int max_val)
{
    int current_value = 0;
    static int seeded = 0;

    uint64_t now = GetMilliTimeNow();   // 毫秒

    // 参数保护
    if (min_val > max_val)
    {
        int temp = min_val;
        min_val = max_val;
        max_val = temp;
    }

    // 初始化随机种子（只执行一次）
    if (!seeded)
    {
        srand((unsigned int)now);
        seeded = 1;
    }

    current_value = min_val + (rand() % (max_val - min_val + 1));

    return current_value;
}
/* ----------------------------------------------------- 串口接收缓冲事务接口 */

void UART_CommitBuffer(void)
{
    RING_Commit(&uart2Buffer);
}

void UART_RollbackBuffer(void)
{
    RING_Rollback(&uart2Buffer);
}

void UART_FlushBuffer(void)
{
    RING_Flush(&uart2Buffer);
}

uint32_t UART_BufferedBytes(void)
{
    return RING_Count(&uart2Buffer);
}
