#include <string.h>
#include "esp_log.h"
#include "mb_rtu.h"
#include "mb_pdu.h"
#include "serial_txn.h"
#include "proto_util.h"

static const char *TAG = "MB_RTU";

#define RTU_RESP_MAX    (3 + MB_MAX_READ_REGS * 2 + 2)

static uint8_t s_slave = 1;

void MB_RtuSetSlave(uint8_t slave)
{
    s_slave = slave;
}

static uint16_t Crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;

    while (n-- > 0) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

typedef struct {
    uint8_t fc;
    uint16_t count;
    uint16_t *out;
    int exception;          // 异常码，0 表示没有
} RtuCtx;

/* 应答：从站 | 功能码 | 字节数 | 数据 | CRC(低字节在前)；异常应答：从站 | 功能码|0x80 | 异常码 | CRC */
static int RtuParse(void *arg)
{
    RtuCtx *ctx = arg;
    uint8_t f[RTU_RESP_MAX];
    size_t n;

    if (SerialRx_SeekAny(s_slave, s_slave) < 0 || SerialRx_Read(&f[1], 2) != 0) {
        return SERIAL_PARSE_MORE;
    }
    f[0] = s_slave;
    if (f[1] == (ctx->fc | 0x80)) {
        n = 5;
    } else if (f[1] == ctx->fc && f[2] == MB_ReadDataBytes(ctx->fc, ctx->count)) {
        n = 3 + (size_t)f[2] + 2;
    } else {
        ESP_LOGE(TAG, "unexpected response %02X %02X %02X", f[0], f[1], f[2]);
        return SERIAL_PARSE_ERROR;
    }
    if (SerialRx_Read(&f[3], n - 3) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (Crc16(f, n - 2) != (uint16_t)(f[n - 2] | (f[n - 1] << 8))) {
        ESP_LOGE(TAG, "CRC mismatch");
        return SERIAL_PARSE_ERROR;
    }
    ctx->exception = MB_ParseReadPdu(ctx->fc, ctx->count, &f[1], n - 3, ctx->out);
    if (ctx->exception != 0) {
        ESP_LOGE(TAG, "function 0x%02X exception %d (%s)", ctx->fc, ctx->exception, MB_ExceptionText(ctx->exception));
        return SERIAL_PARSE_ERROR;
    }
    return SERIAL_PARSE_OK;
}

int MB_RtuRead(uint8_t fc, uint16_t address, uint16_t count, uint16_t *out)
{
    uint8_t req[8];
    RtuCtx ctx = { .fc = fc, .count = count, .out = out };
    uint16_t crc;

    if (count == 0 || MB_ReadDataBytes(fc, count) > MB_MAX_READ_REGS * 2) {
        return PLC_ERR_RESPONSE;
    }
    req[0] = s_slave;
    MB_PutReadPdu(&req[1], fc, address, count);
    crc = Crc16(req, 6);
    req[6] = (uint8_t)crc;
    req[7] = (uint8_t)(crc >> 8);

    for (int tries = 0; tries < 2; tries++) {
        int ret = SerialTxn_Request(req, sizeof(req), RtuParse, &ctx, SERIAL_TXN_TIMEOUT_MS);

        if (ret == 0) {
            return PLC_OK;
        }
        // 只有 CRC 错（线路干扰）才重读；超时、异常应答（地址不对等）重读也没用
        if (ret != SERIAL_TXN_BAD || ctx.exception != 0) {
            break;
        }
    }
    return PLC_ERR_RESPONSE;
}

/* ------------------------------------------------------------ Modbus ASCII */

#define ASCII_START     ':'

// 读 n 个字节（每个 2 个十六进制字符）
static int AsciiBytes(uint8_t *dst, size_t n)
{
    uint8_t hex[2];
    uint32_t b;

    for (size_t i = 0; i < n; i++) {
        if (SerialRx_Read(hex, 2) != 0) {
            return SERIAL_PARSE_MORE;
        }
        if (Proto_GetHex(hex, 2, &b) != 0) {
            ESP_LOGE(TAG, "bad ASCII data: %.2s", (const char *)hex);
            return SERIAL_PARSE_ERROR;
        }
        dst[i] = (uint8_t)b;
    }
    return SERIAL_PARSE_OK;
}

static uint8_t Lrc(const uint8_t *p, size_t n)
{
    uint8_t sum = 0;

    while (n-- > 0) {
        sum += *p++;
    }
    return (uint8_t)(-sum);
}

/* 应答：: 从站 | 功能码 | 字节数 | 数据 | LRC CR LF；异常应答：: 从站 | 功能码|0x80 | 异常码 | LRC CR LF */
static int AsciiParse(void *arg)
{
    RtuCtx *ctx = arg;
    uint8_t f[RTU_RESP_MAX];
    uint8_t crlf[2];
    size_t n;
    int ret;

    if (SerialRx_SeekAny(ASCII_START, ASCII_START) < 0) {
        return SERIAL_PARSE_MORE;
    }
    ret = AsciiBytes(f, 3);
    if (ret != SERIAL_PARSE_OK) {
        return ret;
    }
    if (f[0] != s_slave) {
        ESP_LOGE(TAG, "response from station %u, want %u", f[0], s_slave);
        return SERIAL_PARSE_ERROR;
    }
    if (f[1] == (ctx->fc | 0x80)) {
        n = 4;
    } else if (f[1] == ctx->fc && f[2] == MB_ReadDataBytes(ctx->fc, ctx->count)) {
        n = 3 + (size_t)f[2] + 1;
    } else {
        ESP_LOGE(TAG, "unexpected response %02X %02X %02X", f[0], f[1], f[2]);
        return SERIAL_PARSE_ERROR;
    }
    ret = AsciiBytes(&f[3], n - 3);
    if (ret != SERIAL_PARSE_OK) {
        return ret;
    }
    if (SerialRx_Read(crlf, sizeof(crlf)) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (Lrc(f, n - 1) != f[n - 1]) {
        ESP_LOGE(TAG, "LRC mismatch");
        return SERIAL_PARSE_ERROR;
    }
    ctx->exception = MB_ParseReadPdu(ctx->fc, ctx->count, &f[1], n - 2, ctx->out);
    if (ctx->exception != 0) {
        ESP_LOGE(TAG, "function 0x%02X exception %d (%s)", ctx->fc, ctx->exception, MB_ExceptionText(ctx->exception));
        return SERIAL_PARSE_ERROR;
    }
    return SERIAL_PARSE_OK;
}

int MB_AsciiRead(uint8_t fc, uint16_t address, uint16_t count, uint16_t *out)
{
    uint8_t bin[7];             // 从站 + PDU(5) + LRC
    uint8_t req[1 + sizeof(bin) * 2 + 2];
    RtuCtx ctx = { .fc = fc, .count = count, .out = out };

    if (count == 0 || MB_ReadDataBytes(fc, count) > MB_MAX_READ_REGS * 2) {
        return PLC_ERR_RESPONSE;
    }
    bin[0] = s_slave;
    MB_PutReadPdu(&bin[1], fc, address, count);
    bin[6] = Lrc(bin, 6);
    req[0] = ASCII_START;
    for (size_t i = 0; i < sizeof(bin); i++) {
        Proto_PutHex(&req[1 + i * 2], bin[i], 2);
    }
    req[sizeof(req) - 2] = '\r';
    req[sizeof(req) - 1] = '\n';

    for (int tries = 0; tries < 2; tries++) {
        int ret = SerialTxn_Request(req, sizeof(req), AsciiParse, &ctx, SERIAL_TXN_TIMEOUT_MS);

        if (ret == 0) {
            return PLC_OK;
        }
        // 只有 LRC 错等（线路干扰）才重读
        if (ret != SERIAL_TXN_BAD || ctx.exception != 0) {
            break;
        }
    }
    return PLC_ERR_RESPONSE;
}
