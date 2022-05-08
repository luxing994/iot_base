#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>

typedef struct {
    uint32_t front;
    uint32_t rear;
    uint32_t size;
    uint8_t rdata;
    uint8_t *recBuffer;
} RingBuffer;

int RING_InitBuffer(RingBuffer *ptr, int size);
int RING_WriteBufferBytes(RingBuffer *ptr, uint8_t *data, uint32_t length);
int RING_ReadBufferBytes(RingBuffer *ptr, uint8_t *data, uint32_t length);

#endif