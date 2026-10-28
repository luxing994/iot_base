#include "proto_util.h"

static const char s_hex[] = "0123456789ABCDEF";

void Proto_PutHex(uint8_t *dst, uint32_t value, int width)
{
    for (int i = width - 1; i >= 0; i--) {
        dst[i] = (uint8_t)s_hex[value & 0xF];
        value >>= 4;
    }
}

static int HexDigit(uint8_t c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

int Proto_GetHex(const uint8_t *src, int n, uint32_t *out)
{
    uint32_t v = 0;

    if (n <= 0 || n > 8) {
        return -1;
    }
    for (int i = 0; i < n; i++) {
        int d = HexDigit(src[i]);
        if (d < 0) {
            return -1;
        }
        v = (v << 4) | (uint32_t)d;
    }
    *out = v;
    return 0;
}

uint8_t Proto_Sum8(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;

    while (len--) {
        sum += *data++;
    }
    return sum;
}

void Proto_PutSum8Hex(uint8_t *dst, const uint8_t *data, size_t len)
{
    Proto_PutHex(dst, Proto_Sum8(data, len), 2);
}

uint8_t Proto_Xor8(const uint8_t *data, size_t len)
{
    uint8_t x = 0;

    while (len--) {
        x ^= *data++;
    }
    return x;
}

int Proto_BcdToInt(uint32_t bcd)
{
    int sum = 0;
    int scale = 1;

    for (int i = 0; i < 4; i++) {
        sum += (int)(bcd & 0xF) * scale;
        scale *= 10;
        bcd >>= 4;
    }
    return sum;
}
