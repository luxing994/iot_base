#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>

typedef struct {
    volatile uint32_t front;     // 已提交的读位置（写端据此判断是否已满）
    volatile uint32_t rear;      // 写位置
    volatile uint32_t rfront;    // 读游标（事务模式下可回滚到 front）
    uint32_t size;
    uint8_t transactional;       // 1: 读取后需显式 RING_Commit()
    uint8_t *recBuffer;
} RingBuffer;

int RING_InitBuffer(RingBuffer *ptr, int size);
int RING_WriteBufferBytes(RingBuffer *ptr, uint8_t *data, uint32_t length);
int RING_ReadBufferBytes(RingBuffer *ptr, uint8_t *data, uint32_t length);

/* 事务模式 */
void RING_SetTransactional(RingBuffer *ptr, int enable);
void RING_Commit(RingBuffer *ptr);       // 释放已读数据
void RING_Rollback(RingBuffer *ptr);     // 读游标回到上次提交的位置
void RING_Flush(RingBuffer *ptr);        // 丢弃所有未读数据（只能由读端调用）
uint32_t RING_Count(const RingBuffer *ptr);   // 读游标之后的字节数

#endif
