#ifndef PROTO_UTIL_H
#define PROTO_UTIL_H

/*
 * 协议层公共工具：ASCII 十六进制编解码、校验和、BCD
 */

#include <stddef.h>
#include <stdint.h>

/* 把 value 写成 width 个大写十六进制字符（不补 '\0'），例如 (0x1A, 4) -> "001A" */
void Proto_PutHex(uint8_t *dst, uint32_t value, int width);

/* 解析 n 个十六进制字符（n <= 8），遇到非法字符返回 -1 */
int Proto_GetHex(const uint8_t *src, int n, uint32_t *out);

/* 字节累加和的低 8 位 */
uint8_t Proto_Sum8(const uint8_t *data, size_t len);

/* 字节累加和低 8 位的两个十六进制字符（FX / LS 协议的和校验） */
void Proto_PutSum8Hex(uint8_t *dst, const uint8_t *data, size_t len);

/* 逐字节异或（HostLink FCS） */
uint8_t Proto_Xor8(const uint8_t *data, size_t len);

/* 4 位 BCD 码转整数，例如 0x1234 -> 1234 */
int Proto_BcdToInt(uint32_t bcd);

/* 大端字节序读取 */
static inline uint16_t Proto_Be16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static inline uint32_t Proto_Be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static inline uint16_t Proto_Le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t Proto_Le32(const uint8_t *p)
{
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

#endif
