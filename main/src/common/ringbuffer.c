#include "ringbuffer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * 单生产者 / 单消费者环形缓冲区。
 *   写端（生产者）只修改 rear，用 front 判断是否已满；
 *   读端（消费者）从读游标 rfront 读取：
 *     - 普通模式：读完自动提交（front = rfront）
 *     - 事务模式：读取不立即释放空间，解析成功后 RING_Commit()，帧不完整时 RING_Rollback()
 *       重新从上次提交的位置读，已读出的字节不会丢失
 */

int RING_InitBuffer(RingBuffer *ptr, int size)
{
    if (ptr == NULL || size == 0) {
        return -1;
    }

    ptr->front = ptr->rear = ptr->rfront = 0;
    ptr->size = size;
    ptr->transactional = 0;
    ptr->recBuffer = (uint8_t *)malloc(size);
    if (ptr->recBuffer == NULL) {
        return -1;
    }
    memset(ptr->recBuffer, 0, size);

    return 0;
}

void RING_SetTransactional(RingBuffer *ptr, int enable)
{
    if (ptr != NULL) {
        ptr->transactional = enable ? 1 : 0;
    }
}

static int RING_WriteBufferByte(RingBuffer *ptr, uint8_t data)
{
    uint32_t rear = ptr->rear;
    uint32_t next = (rear + 1) % ptr->size;

    if (next == ptr->front) {
        return -1;
    }

    ptr->recBuffer[rear] = data;
    __sync_synchronize();          // 先写数据再移动 rear，另一个核上的读端看到 rear 时数据已就绪
    ptr->rear = next;
    return 0;
}

int RING_WriteBufferBytes(RingBuffer *ptr, uint8_t *data, uint32_t length)
{
    if ((ptr == NULL) || (data == NULL) || (ptr->recBuffer == NULL) || (length == 0)) {
        return -1;
    }

    for (uint32_t i = 0; i < length; i++) {
        if (RING_WriteBufferByte(ptr, data[i]) != 0) {
            return -1;
        }
    }
    return 0;
}

static int RING_ReadBufferByte(RingBuffer *ptr, uint8_t *data)
{
    uint32_t pos = ptr->rfront;

    if (pos == ptr->rear) {
        return -1;
    }

    *data = ptr->recBuffer[pos];
    ptr->rfront = (pos + 1) % ptr->size;
    return 0;
}

int RING_ReadBufferBytes(RingBuffer *ptr, uint8_t *data, uint32_t length)
{
    int ret = 0;

    if ((ptr == NULL) || (data == NULL) || (ptr->recBuffer == NULL) || (length == 0)) {
        return -1;
    }

    for (uint32_t i = 0; i < length; i++) {
        if (RING_ReadBufferByte(ptr, &data[i]) != 0) {
            ret = -1;
            break;
        }
    }

    if (!ptr->transactional) {
        ptr->front = ptr->rfront;
    }
    return ret;
}

void RING_Commit(RingBuffer *ptr)
{
    if (ptr != NULL) {
        ptr->front = ptr->rfront;
    }
}

void RING_Rollback(RingBuffer *ptr)
{
    if (ptr != NULL) {
        ptr->rfront = ptr->front;
    }
}

void RING_Flush(RingBuffer *ptr)
{
    if (ptr != NULL) {
        uint32_t rear = ptr->rear;
        ptr->rfront = rear;
        ptr->front = rear;
    }
}

uint32_t RING_Count(const RingBuffer *ptr)
{
    if (ptr == NULL || ptr->size == 0) {
        return 0;
    }
    return (ptr->rear + ptr->size - ptr->rfront) % ptr->size;
}
