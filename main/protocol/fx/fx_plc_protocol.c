#include <string.h>
#include "esp_log.h"
#include "fx_plc_protocol.h"
#include "proto_util.h"
#include "serial_txn.h"
#include "net_io.h"

static const char *TAG = "FX";

#define FX_STX  0x02
#define FX_ETX  0x03
#define FX_ENQ  0x05
#define FX_ACK  0x06
#define FX_NAK  0x15

static int FxWords(FxFormat fmt)
{
    return (fmt == FX_FMT_I32 || fmt == FX_FMT_FLOAT) ? 2 : 1;
}

// 把读到的字（w[0] 为低字）按格式转换成上报值
static void FxSetValue(FxFormat fmt, const uint32_t *w, PlcValue *out)
{
    switch (fmt) {
    case FX_FMT_I32:
        PlcValue_SetInt(out, (int32_t)((w[1] << 16) | w[0]));
        break;
    case FX_FMT_FLOAT:
        PlcValue_SetFloatBits(out, (w[1] << 16) | w[0]);
        break;
    case FX_FMT_BIT:
        PlcValue_SetInt(out, (int32_t)(w[0] & 0x0001));
        break;
    default:
        PlcValue_SetInt(out, (int32_t)w[0]);
        break;
    }
}

/* ================================================================ 编程口（RS232）
 *
 * 读请求：STX '0' 地址(4) 字节数(2) ETX 和校验(2)       地址 = D 寄存器在 PLC 内存中的字节地址
 * 应答：  STX 数据(字节数 * 2) ETX 和校验(2)             数据低字节在前
 */
#define FX_PROG_CMD_READ        '0'
#define FX_PROG_D_BASE          0x1000
#define FX_PROG_D_SPECIAL_BASE  0x0E00      // D8000 起的特殊寄存器

typedef struct {
    int nbytes;
    uint32_t raw;
} FxProgCtx;

static uint16_t FxProgAddress(uint16_t d)
{
    if (d >= 8000) {
        return (uint16_t)((d - 8000) * 2 + FX_PROG_D_SPECIAL_BASE);
    }
    return (uint16_t)(d * 2 + FX_PROG_D_BASE);
}

static int FxProgParse(void *arg)
{
    FxProgCtx *ctx = arg;
    uint8_t data[8];
    uint8_t etx;
    uint32_t v = 0;
    int c;

    c = SerialRx_SeekAny(FX_STX, FX_NAK);
    if (c < 0) {
        return SERIAL_PARSE_MORE;
    }
    if (c == FX_NAK) {
        ESP_LOGE(TAG, "programming port NAK");
        return SERIAL_PARSE_ERROR;
    }
    if (SerialRx_Read(data, ctx->nbytes * 2) != 0 || SerialRx_Byte(&etx) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (etx != FX_ETX) {
        return SERIAL_PARSE_ERROR;
    }

    // 每个字节 2 个十六进制字符，低字节在前
    for (int i = 0; i < ctx->nbytes; i++) {
        uint32_t b;
        if (Proto_GetHex(&data[i * 2], 2, &b) != 0) {
            return SERIAL_PARSE_ERROR;
        }
        v |= b << (8 * i);
    }
    ctx->raw = v;
    return SERIAL_PARSE_OK;
}

int FX_ProgReadD(uint16_t address, PlcValue *out)
{
    uint8_t f[11];
    FxProgCtx ctx = { .nbytes = 2 };

    f[0] = FX_STX;
    f[1] = FX_PROG_CMD_READ;
    Proto_PutHex(&f[2], FxProgAddress(address), 4);
    Proto_PutHex(&f[6], ctx.nbytes, 2);
    f[8] = FX_ETX;
    Proto_PutSum8Hex(&f[9], &f[1], 8);      // 命令 ~ ETX

    if (SerialTxn_Request(f, sizeof(f), FxProgParse, &ctx, SERIAL_TXN_TIMEOUT_MS) != 0) {
        return PLC_ERR_RESPONSE;
    }
    PlcValue_SetInt(out, (int32_t)ctx.raw);
    return PLC_OK;
}

/* ================================================================ 计算机链接（RS485）
 *
 * 请求：ENQ 站号(2) PC号(2) 命令(2) 等待(1) 软元件(5) 点数(2) [数据] 和校验(2)
 * 读应答：STX 站号(2) PC号(2) 数据(点数 * 4) ETX 和校验(2)，PC 收到后回 ACK 站号 PC号
 * 写应答：ACK 站号(2) PC号(2)
 * 出错：  NAK 站号(2) PC号(2) 错误码(2)
 * 数据每个字 4 个十六进制字符，高位在前；多个字时低字在前
 */
#define FX_LINK_HEADER_LEN   15     // ENQ ~ 点数

typedef struct {
    int words;
    uint8_t addr[4];        // 应答中的站号 + PC 号，回 ACK 时使用
    uint32_t w[2];
} FxLinkCtx;

static void FxLinkHeader(uint8_t *f, const char *cmd, FxDevice dev, uint16_t address, int count)
{
    f[0] = FX_ENQ;
    Proto_PutHex(&f[1], FX_LINK_STATION, 2);
    Proto_PutHex(&f[3], FX_LINK_PC_NUMBER, 2);
    f[5] = (uint8_t)cmd[0];
    f[6] = (uint8_t)cmd[1];
    Proto_PutHex(&f[7], FX_LINK_WAIT_MS / 10, 1);
    f[8] = (uint8_t)dev;
    for (int i = 3; i >= 0; i--) {          // 元件编号 4 位十进制
        f[9 + i] = (uint8_t)('0' + address % 10);
        address /= 10;
    }
    Proto_PutHex(&f[13], (uint32_t)count, 2);
}

// 读到 NAK 时取出错误码并记录
static int FxLinkNak(void)
{
    uint8_t nak[6];

    if (SerialRx_Read(nak, sizeof(nak)) != 0) {
        return SERIAL_PARSE_MORE;
    }
    ESP_LOGE(TAG, "PLC NAK, station %c%c pc %c%c error %c%c",
             nak[0], nak[1], nak[2], nak[3], nak[4], nak[5]);
    return SERIAL_PARSE_ERROR;
}

static int FxLinkReadParse(void *arg)
{
    FxLinkCtx *ctx = arg;
    uint8_t hex[4];
    uint8_t etx;
    int c;

    c = SerialRx_SeekAny(FX_STX, FX_NAK);
    if (c < 0) {
        return SERIAL_PARSE_MORE;
    }
    if (c == FX_NAK) {
        return FxLinkNak();
    }
    if (SerialRx_Read(ctx->addr, sizeof(ctx->addr)) != 0) {
        return SERIAL_PARSE_MORE;
    }
    for (int i = 0; i < ctx->words; i++) {
        if (SerialRx_Read(hex, sizeof(hex)) != 0) {
            return SERIAL_PARSE_MORE;
        }
        if (Proto_GetHex(hex, 4, &ctx->w[i]) != 0) {
            return SERIAL_PARSE_ERROR;
        }
    }
    if (SerialRx_Byte(&etx) != 0) {
        return SERIAL_PARSE_MORE;
    }
    return (etx == FX_ETX) ? SERIAL_PARSE_OK : SERIAL_PARSE_ERROR;
}

static int FxLinkWriteParse(void *arg)
{
    uint8_t addr[4];
    int c;

    (void)arg;
    c = SerialRx_SeekAny(FX_ACK, FX_NAK);
    if (c < 0) {
        return SERIAL_PARSE_MORE;
    }
    if (c == FX_NAK) {
        return FxLinkNak();
    }
    return (SerialRx_Read(addr, sizeof(addr)) == 0) ? SERIAL_PARSE_OK : SERIAL_PARSE_MORE;
}

int FX_LinkRead(FxDevice dev, uint16_t address, FxFormat fmt, PlcValue *out)
{
    uint8_t f[FX_LINK_HEADER_LEN + 2];
    FxLinkCtx ctx = { .words = FxWords(fmt) };
    int ret;

    FxLinkHeader(f, "WR", dev, address, ctx.words);
    Proto_PutSum8Hex(&f[FX_LINK_HEADER_LEN], &f[1], FX_LINK_HEADER_LEN - 1);    // 站号 ~ 点数

    SerialTxn_Begin();
    SerialTxn_Send(f, sizeof(f));
    ret = SerialTxn_Wait(FxLinkReadParse, &ctx, SERIAL_TXN_TIMEOUT_MS);
    if (ret == 0) {
        uint8_t ack[5] = { FX_ACK, ctx.addr[0], ctx.addr[1], ctx.addr[2], ctx.addr[3] };
        SerialTxn_Send(ack, sizeof(ack));
    }
    SerialTxn_End();

    if (ret != 0) {
        return PLC_ERR_RESPONSE;
    }
    FxSetValue(fmt, ctx.w, out);
    return PLC_OK;
}

int FX_LinkWriteFloat(uint16_t address, float value)
{
    uint8_t f[FX_LINK_HEADER_LEN + 8 + 2];
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    FxLinkHeader(f, "WW", FX_DEV_D, address, 2);
    Proto_PutHex(&f[FX_LINK_HEADER_LEN], raw & 0xFFFF, 4);          // 低字
    Proto_PutHex(&f[FX_LINK_HEADER_LEN + 4], raw >> 16, 4);         // 高字
    Proto_PutSum8Hex(&f[FX_LINK_HEADER_LEN + 8], &f[1], FX_LINK_HEADER_LEN - 1 + 8);

    if (SerialTxn_Request(f, sizeof(f), FxLinkWriteParse, NULL, SERIAL_TXN_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "write D%u failed", address);
        return PLC_ERR_RESPONSE;
    }
    return PLC_OK;
}

/* ================================================================ 以太网（MC 协议 3E 帧，二进制）
 *
 * 请求：副头部 50 00 | 网络号 | PC号 | IO号(2) | 站号 | 长度(2) | 监视定时器(2) |
 *       指令 0401(2) | 子指令 0000(2) | 起始地址(3) | 软元件代码 | 点数(2)        多字节均为小端
 * 应答：副头部 D0 00 | 网络号 | PC号 | IO号(2) | 站号 | 长度(2) | 结束代码(2) | 数据
 */
#define FX_NET_RESP_HEADER_LEN  11
#define FX_NET_WATCH_TIMER      0x000A      // 250ms 单位

static int FxNetDeviceCode(FxDevice dev)
{
    switch (dev) {
    case FX_DEV_Y: return 0x9D;
    case FX_DEV_M: return 0x90;
    default:       return 0xA8;     // D
    }
}

int FX_NetRead(int sock, FxDevice dev, uint32_t address, FxFormat fmt, PlcValue *out)
{
    const uint16_t words = (uint16_t)FxWords(fmt);
    const uint8_t req[] = {
        0x50, 0x00,                 // 副头部
        0x00,                       // 网络号
        0xFF,                       // PC 号
        0xFF, 0x03,                 // 请求目标模块 IO 号 0x03FF
        0x00,                       // 请求目标模块站号
        0x0C, 0x00,                 // 后续长度 12
        FX_NET_WATCH_TIMER & 0xFF, FX_NET_WATCH_TIMER >> 8,
        0x01, 0x04,                 // 指令 0401：成批读出
        0x00, 0x00,                 // 子指令：按字
        address & 0xFF, (address >> 8) & 0xFF, (address >> 16) & 0xFF,
        (uint8_t)FxNetDeviceCode(dev),
        words & 0xFF, words >> 8,
    };
    uint8_t hdr[FX_NET_RESP_HEADER_LEN];
    uint8_t data[16];
    uint32_t w[2] = { 0 };
    uint16_t len, endcode, dlen;

    if (Net_SendAll(sock, req, sizeof(req)) != 0 ||
        Net_RecvExact(sock, hdr, sizeof(hdr)) != 0) {
        return PLC_ERR_LINK;
    }
    len = Proto_Le16(&hdr[7]);          // 结束代码 + 数据的长度
    if (hdr[0] != 0xD0 || hdr[1] != 0x00 || len < 2) {
        ESP_LOGE(TAG, "bad response header %02X %02X len %u", hdr[0], hdr[1], len);
        return PLC_ERR_LINK;            // 数据流已错位，重连
    }
    endcode = Proto_Le16(&hdr[9]);
    dlen = len - 2;

    if (dlen > sizeof(data)) {
        Net_Discard(sock, dlen);
        return PLC_ERR_RESPONSE;
    }
    if (Net_RecvExact(sock, data, dlen) != 0) {
        return PLC_ERR_LINK;
    }
    if (endcode != 0) {
        ESP_LOGE(TAG, "read %c%u failed, end code 0x%04X", dev, (unsigned)address, endcode);
        return PLC_ERR_RESPONSE;
    }
    if (dlen != words * 2) {
        ESP_LOGE(TAG, "unexpected data length %u", dlen);
        return PLC_ERR_RESPONSE;
    }

    for (int i = 0; i < words; i++) {
        w[i] = Proto_Le16(&data[i * 2]);
    }
    FxSetValue(fmt, w, out);
    return PLC_OK;
}
