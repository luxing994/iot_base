#ifndef HPROTOCOLS_H
#define HPROTOCOLS_H

#include <stdint.h>

#define HPRO_HEAD_FIRST_BYTE      0x5A
#define HPRO_HEAD_SECOND_BYTE     0xA5

typedef enum {
    READ = 0x01,
    WRITE
} HproFuncCode;

typedef enum {
    HMISTATUS = 0x01,
    BREAK,
    MODEL,
    RATE,
    APARAMETER,
    PARAMETER,
    PAPASTATUS
} HproOpReadCode;

typedef enum {
    UPHMI = 0x01,
    UPMAINCONTROL,
    UPMAINAXIS,
    UPMAINNEEDLE,
    WMODEL,
    WMODE,
    WPARA
} HproOpWriteCode;

#pragma pack(1)
// 通用帧格式
typedef struct {
    uint8_t head[2];
    uint8_t length[2];
    uint8_t func;
    uint8_t operate;
    uint8_t* data;
    uint8_t crc[2];
} HproComFrame;
#pragma pack()

#endif