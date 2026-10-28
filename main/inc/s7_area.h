#ifndef S7_AREA_H
#define S7_AREA_H

/*
 * 西门子 S7 变量描述，PPI（串口）和 S7comm（网口）共用
 *
 * 两种协议的 ReadVar 请求里，变量地址部分（Item）格式相同：
 *   12 0A 10 | 传输尺寸 | 个数(2) | DB 号(2) | 数据区 | 位地址(3)
 * 应答数据均为大端。
 */

#include <stdint.h>
#include "plc_value.h"
#include "proto_util.h"

/* 数据区 */
#define S7_AREA_I   0x81    // 输入
#define S7_AREA_Q   0x82    // 输出
#define S7_AREA_M   0x83    // 位存储器
#define S7_AREA_V   0x84    // V 区（S7-200 中为 DB1）

/* 请求中的传输尺寸 */
#define S7_TS_BIT   0x01
#define S7_TS_BYTE  0x02
#define S7_TS_WORD  0x04
#define S7_TS_DWORD 0x06

typedef enum {
    S7_BIT,     // 1 位，上报 0 / 1
    S7_BYTE,    // 8 位无符号
    S7_INT,     // 16 位有符号
    S7_DINT,    // 32 位有符号
    S7_REAL,    // 32 位浮点
} S7Type;

typedef struct {
    uint8_t area;       // S7_AREA_xxx
    uint8_t type;       // S7Type
    uint32_t byte;      // 字节地址
    uint8_t bit;        // 位号（仅 S7_BIT）
} S7Var;

static inline uint16_t S7_DbNumber(const S7Var *v)
{
    return (v->area == S7_AREA_V) ? 1 : 0;
}

static inline int S7_TypeBytes(uint8_t type)
{
    switch (type) {
    case S7_INT:  return 2;
    case S7_DINT:
    case S7_REAL: return 4;
    default:      return 1;
    }
}

/* 写入请求中的 Item（12 字节） */
static inline void S7_PutItem(uint8_t *p, uint8_t transport, uint16_t count, const S7Var *v)
{
    const uint16_t db = S7_DbNumber(v);
    const uint32_t addr = v->byte * 8 + ((v->type == S7_BIT) ? (v->bit & 0x07) : 0);

    p[0] = 0x12;
    p[1] = 0x0A;
    p[2] = 0x10;
    p[3] = transport;
    p[4] = (uint8_t)(count >> 8);
    p[5] = (uint8_t)count;
    p[6] = (uint8_t)(db >> 8);
    p[7] = (uint8_t)db;
    p[8] = v->area;
    p[9] = (uint8_t)(addr >> 16);
    p[10] = (uint8_t)(addr >> 8);
    p[11] = (uint8_t)addr;
}

/* 应答 Item 中的长度换算成字节数：读位时单位是字节，其余单位是位 */
static inline int S7_RespDataBytes(const S7Var *v, uint16_t len)
{
    return (v->type == S7_BIT) ? len : len / 8;
}

/* 把应答数据（大端）转换成上报值 */
static inline void S7_SetValue(const S7Var *v, const uint8_t *data, PlcValue *out)
{
    switch (v->type) {
    case S7_INT:
        PlcValue_SetInt(out, (int16_t)Proto_Be16(data));
        break;
    case S7_DINT:
        PlcValue_SetInt(out, (int32_t)Proto_Be32(data));
        break;
    case S7_REAL:
        PlcValue_SetFloatBits(out, Proto_Be32(data));
        break;
    default:    // 位、字节
        PlcValue_SetInt(out, data[0]);
        break;
    }
}

#endif
