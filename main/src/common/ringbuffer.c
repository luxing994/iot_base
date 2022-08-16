#include "ringbuffer.h"
#include <stdio.h>
#include <string.h>

int RING_InitBuffer(RingBuffer *ptr, int size)
{
    if (ptr == NULL || size == 0) {
		return -1;
	}

	ptr->front = ptr->rear = 0;
	ptr->size = size;
	(ptr->recBuffer) = (uint8_t *)malloc(size);
	if ((ptr->recBuffer) == NULL) {
		return -1;
	}
    memset(ptr->recBuffer, 0, size);
	
	return 0;
}

static int RING_WriteBufferByte(RingBuffer *ptr, uint8_t data)
{
    if ((ptr == NULL) || ((ptr->rear + 1) % (ptr->size) == ptr->front)) {
		return -1;
	}

    ptr->recBuffer[ptr->rear] = data;
	ptr->rear = (ptr->rear + 1) % (ptr->size);
	return 0;
}

int RING_WriteBufferBytes(RingBuffer *ptr, uint8_t *data, uint32_t length)
{
    int i;

	if ((ptr == NULL) || (data == NULL) || (ptr->recBuffer == NULL) || (length == 0)) {
		return -1;
	}

    for (i = 0; i < length; i++) {
        if (RING_WriteBufferByte(ptr, data[i]) != 0) {
            return -1;
        }
    }
	return 0;
}

static int RING_ReadBufferByte(RingBuffer *ptr, uint8_t *data)
{
    if ((ptr->front == ptr->rear) || (ptr == NULL) || (data == NULL)) {
        return -1;
    }

    *data = ptr->recBuffer[ptr->front];
    ptr->front = (ptr->front + 1) % (ptr->size);
    return 0;
}

int RING_ReadBufferBytes(RingBuffer *ptr, uint8_t *data, uint32_t length)
{
    int i;

	if ((ptr == NULL) || (data == NULL) || (ptr->recBuffer == NULL) || (length == 0)) {
		return -1;
	}

    for (i = 0; i < length; i++) {
        if (RING_ReadBufferByte(ptr, &data[i]) != 0) {
            return -1;
        }
    }
	return 0;
}