#ifndef MB_PDU_H
#define MB_PDU_H

/*
 * Modbus 读请求 / 应答 PDU（功能码 + 数据），TCP（lx_modbus_tcp_master.c）和 RTU（mb_rtu.c）共用
 *
 *   01 读线圈  02 读离散输入  03 读保持寄存器  04 读输入寄存器
 *   寄存器结果每个占 out[i] 一个字；线圈 / 离散输入结果每个位占 out[i] 的 0 / 1
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define MB_FC_READ_COILS        0x01
#define MB_FC_READ_DISCRETE     0x02
#define MB_FC_READ_HOLDING      0x03
#define MB_FC_READ_INPUT        0x04

#define MB_MAX_READ_REGS        125
#define MB_MAX_READ_BITS        2000

static inline bool MB_IsBitFc(uint8_t fc)
{
    return fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE;
}

/* 写读请求 PDU（5 字节） */
static inline void MB_PutReadPdu(uint8_t *p, uint8_t fc, uint16_t addr, uint16_t count)
{
    p[0] = fc;
    p[1] = (uint8_t)(addr >> 8);
    p[2] = (uint8_t)addr;
    p[3] = (uint8_t)(count >> 8);
    p[4] = (uint8_t)count;
}

/* 读应答中数据部分的字节数 */
static inline size_t MB_ReadDataBytes(uint8_t fc, uint16_t count)
{
    return MB_IsBitFc(fc) ? (size_t)((count + 7) / 8) : (size_t)count * 2;
}

/*
 * 解析读应答 PDU（功能码 | 字节数 | 数据）：
 *   成功返回 0；异常应答返回异常码（>0）；格式不对返回 -1
 */
static inline int MB_ParseReadPdu(uint8_t fc, uint16_t count, const uint8_t *p, size_t n, uint16_t *out)
{
    const size_t bytes = MB_ReadDataBytes(fc, count);

    if (n >= 2 && p[0] == (fc | 0x80)) {
        return p[1] != 0 ? p[1] : -1;
    }
    if (n < 2 + bytes || p[0] != fc || p[1] != bytes) {
        return -1;
    }
    for (uint16_t i = 0; i < count; i++) {
        if (MB_IsBitFc(fc)) {
            out[i] = (p[2 + i / 8] >> (i % 8)) & 1;
        } else {
            out[i] = (uint16_t)((p[2 + i * 2] << 8) | p[3 + i * 2]);
        }
    }
    return 0;
}

/* 异常码的中文说明（日志、诊断用） */
static inline const char *MB_ExceptionText(int code)
{
    switch (code) {
    case 1:  return "不支持的功能码";
    case 2:  return "地址超出范围";
    case 3:  return "数据值非法";
    case 4:  return "设备故障";
    case 6:  return "设备忙";
    default: return "异常应答";
    }
}

#endif
