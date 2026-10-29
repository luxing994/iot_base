#include <string.h>
#include "esp_log.h"
#include "hl_plc_protocol.h"
#include "proto_util.h"
#include "serial_txn.h"

static const char *TAG = "HOSTLINK";

#define HL_HEAD             '@'
#define HL_FRAME_MAX        48

#define FINS_MEMORY_READ    0x0101
#define FINS_MEMORY_WRITE   0x0102

#define FINS_AREA_CIO_WORD  0xB0
#define FINS_AREA_DM_WORD   0x82

/* @ 单元号 00 | FA | 响应等待 0 | ICF 00 | DA2 00 | SA2 00 | SID 00 */
static const char s_cmdHead[] = "@00FA000000000";
#define HL_CMD_HEAD_LEN     (sizeof(s_cmdHead) - 1)

static uint8_t s_unit;

void HL_SetUnit(uint8_t unit)
{
    s_unit = unit;
}

/* 应答中 '@' 之后的固定部分：单元号(2) FA(2) 00(2) ICF(2) DA2(2) SA2(2) SID(2) 命令(4) 结束码(4) */
#define HL_RESP_HEAD_LEN    22

typedef struct {
    uint16_t fins;          // 期望应答中的 FINS 命令
    int words;              // 应答数据字数
    uint32_t w[2];
} HlCtx;

static size_t HlBuild(uint8_t *f, uint16_t fins, uint8_t area, uint32_t addr, uint16_t count,
                      const uint8_t *data, size_t dlen)
{
    size_t n = HL_CMD_HEAD_LEN;

    memcpy(f, s_cmdHead, HL_CMD_HEAD_LEN);
    f[1] = (uint8_t)('0' + s_unit / 10);   // 单元号，两位十进制
    f[2] = (uint8_t)('0' + s_unit % 10);
    Proto_PutHex(&f[n], fins, 4);
    n += 4;
    Proto_PutHex(&f[n], area, 2);
    n += 2;
    Proto_PutHex(&f[n], addr << 8, 6);      // 字地址(4) + 位号(2)
    n += 6;
    Proto_PutHex(&f[n], count, 4);
    n += 4;
    if (dlen > 0) {
        memcpy(&f[n], data, dlen);
        n += dlen;
    }
    Proto_PutHex(&f[n], Proto_Xor8(f, n), 2);
    n += 2;
    f[n++] = '*';
    f[n++] = '\r';
    return n;
}

static int HlParse(void *arg)
{
    HlCtx *ctx = arg;
    uint8_t h[HL_RESP_HEAD_LEN];
    uint8_t hex[4];
    uint32_t cmd, end, got;
    uint8_t fcs;

    if (SerialRx_SeekAny(HL_HEAD, HL_HEAD) < 0 || SerialRx_Read(h, sizeof(h)) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (memcmp(&h[2], "FA", 2) != 0 || memcmp(&h[6], "40", 2) != 0 ||
        Proto_GetHex(&h[14], 4, &cmd) != 0 || Proto_GetHex(&h[18], 4, &end) != 0) {
        ESP_LOGE(TAG, "bad response header: %.22s", (const char *)h);
        return SERIAL_PARSE_ERROR;
    }
    if (cmd != ctx->fins) {
        ESP_LOGE(TAG, "unexpected command %04X, want %04X", (unsigned)cmd, ctx->fins);
        return SERIAL_PARSE_ERROR;
    }
    if (end != 0) {
        ESP_LOGE(TAG, "end code %04X", (unsigned)end);
        return SERIAL_PARSE_ERROR;
    }

    /*
     * FCS：'@' 到数据末尾逐字节异或。8N1 没有奇偶校验，线路干扰造成的错位只能靠它发现，
     * 否则错的数据会照常上报（如浮点高位 '0' 错成 '8'，0.00 就成了 -0.00）
     */
    fcs = (uint8_t)(HL_HEAD ^ Proto_Xor8(h, sizeof(h)));
    for (int i = 0; i < ctx->words; i++) {
        if (SerialRx_Read(hex, sizeof(hex)) != 0) {
            return SERIAL_PARSE_MORE;
        }
        fcs ^= Proto_Xor8(hex, sizeof(hex));
        if (Proto_GetHex(hex, 4, &ctx->w[i]) != 0) {
            ESP_LOGE(TAG, "bad data: %.4s", (const char *)hex);
            return SERIAL_PARSE_ERROR;
        }
    }
    if (SerialRx_Read(hex, 2) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (Proto_GetHex(hex, 2, &got) != 0 || got != fcs) {
        ESP_LOGE(TAG, "FCS mismatch: got %.2s, want %02X", (const char *)hex, (unsigned)fcs);
        return SERIAL_PARSE_ERROR;
    }
    return SERIAL_PARSE_OK;
}

static int HlReadWords(uint8_t area, uint16_t address, HlCtx *ctx)
{
    uint8_t f[HL_FRAME_MAX];
    size_t n;

    ctx->fins = FINS_MEMORY_READ;
    n = HlBuild(f, FINS_MEMORY_READ, area, address, (uint16_t)ctx->words, NULL, 0);
    for (int tries = 0; tries < 2; tries++) {
        int ret = SerialTxn_Request(f, n, HlParse, ctx, SERIAL_TXN_TIMEOUT_MS);

        if (ret == 0) {
            return PLC_OK;
        }
        if (ret != SERIAL_TXN_BAD) {
            break;      // 无应答不重试，免得设备离线时一轮拖得太长
        }
        // 应答被干扰（校验和不对等）：读操作没有副作用，重读一次
    }
    return PLC_ERR_RESPONSE;
}

int HL_ReadDM(uint16_t address, HlFormat fmt, PlcValue *out)
{
    HlCtx ctx = { .words = (fmt == HL_FMT_FLOAT) ? 2 : 1 };

    if (HlReadWords(FINS_AREA_DM_WORD, address, &ctx) != PLC_OK) {
        return PLC_ERR_RESPONSE;
    }

    switch (fmt) {
    case HL_FMT_FLOAT:
        PlcValue_SetFloatBits(out, (ctx.w[1] << 16) | ctx.w[0]);
        break;
    case HL_FMT_BCD:
        PlcValue_SetInt(out, Proto_BcdToInt(ctx.w[0]));
        break;
    default:
        PlcValue_SetInt(out, (int32_t)ctx.w[0]);
        break;
    }
    return PLC_OK;
}

int HL_ReadCIOBit(uint16_t address, uint8_t bit, PlcValue *out)
{
    HlCtx ctx = { .words = 1 };

    // 读整个字再取位
    if (HlReadWords(FINS_AREA_CIO_WORD, address, &ctx) != PLC_OK) {
        return PLC_ERR_RESPONSE;
    }
    PlcValue_SetInt(out, (int32_t)((ctx.w[0] >> (bit & 0x0F)) & 0x0001));
    return PLC_OK;
}

int HL_WriteDMFloat(uint16_t address, float value)
{
    uint8_t f[HL_FRAME_MAX];
    uint8_t data[8];
    uint32_t raw;
    HlCtx ctx = { .fins = FINS_MEMORY_WRITE, .words = 0 };
    size_t n;

    memcpy(&raw, &value, sizeof(raw));
    Proto_PutHex(&data[0], raw & 0xFFFF, 4);    // 低字在前
    Proto_PutHex(&data[4], raw >> 16, 4);
    n = HlBuild(f, FINS_MEMORY_WRITE, FINS_AREA_DM_WORD, address, 2, data, sizeof(data));

    if (SerialTxn_Request(f, n, HlParse, &ctx, SERIAL_TXN_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "write D%u failed", address);
        return PLC_ERR_RESPONSE;
    }
    return PLC_OK;
}
