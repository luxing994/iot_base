#include <string.h>
#include "esp_log.h"
#include "ppi_plc_protocol.h"
#include "proto_util.h"
#include "serial_txn.h"

static const char *TAG = "PPI";

#define PPI_SD1             0x10    // 定长帧（确认帧）
#define PPI_SD2             0x68    // 变长帧
#define PPI_ED              0x16
#define PPI_SHORT_ACK       0xE5

#define PPI_DA_DEFAULT      0x02    // PLC 地址默认值（设备配置中可改，见 PPI_SetStation）
#define PPI_SA              0x00    // 本机地址
#define PPI_FC_READ         0x6C
#define PPI_FC_CONFIRM      0x5C

#define PPI_READ_LE         0x1B    // 读请求中 DA 到地址结束的长度
#define PPI_RESP_HEAD_LEN   21      // 应答中 DA 到 Item 长度结束的字节数
#define PPI_DATA_MAX        8

/* 读请求中 DA 之后的固定部分：S7 头 + ReadVar 参数（Item 由 S7_PutItem 填写） */
static uint8_t s_da = PPI_DA_DEFAULT;

void PPI_SetStation(uint8_t station)
{
    s_da = station;
}

static const uint8_t s_readHead[] = {
    0x00, PPI_SA, PPI_FC_READ,              // DA 在组帧时填入
    0x32, 0x01, 0x00, 0x00, 0x00, 0x00,     // 协议 ID、作业请求、冗余、PDU 参考
    0x00, 0x0E,                             // 参数长度
    0x00, 0x00,                             // 数据长度
    0x04, 0x01,                             // ReadVar，1 个 Item
};

typedef struct {
    const S7Var *var;
    uint8_t data[PPI_DATA_MAX];
} PpiCtx;

static uint8_t PpiTransport(uint8_t type)
{
    switch (type) {
    case S7_BIT:  return S7_TS_BIT;
    case S7_BYTE: return S7_TS_BYTE;
    case S7_INT:  return S7_TS_WORD;
    default:      return S7_TS_DWORD;   // DINT / REAL
    }
}

static void PpiSendConfirm(void)
{
    const uint8_t f[] = {
        PPI_SD1, s_da, PPI_SA, PPI_FC_CONFIRM,
        (uint8_t)(s_da + PPI_SA + PPI_FC_CONFIRM), PPI_ED,
    };
    SerialTxn_Send(f, sizeof(f));
}

/*
 * 应答：SD2 LE LEr SD2 | DA SA FC | S7 头(12) | 04 01 | 返回码 类型 长度(2) | 数据 | FCS ED
 * 先收到的 E5 短应答在这里回确认帧，然后继续等数据帧
 */
static int PpiParse(void *arg)
{
    PpiCtx *ctx = arg;
    uint8_t len[3];
    uint8_t r[PPI_RESP_HEAD_LEN];
    uint8_t tail[2];
    int c, dlen, want;

    c = SerialRx_SeekAny(PPI_SHORT_ACK, PPI_SD2);
    if (c < 0) {
        return SERIAL_PARSE_MORE;
    }
    if (c == PPI_SHORT_ACK) {
        PpiSendConfirm();
        return SERIAL_PARSE_SKIP;
    }

    if (SerialRx_Read(len, sizeof(len)) != 0 || SerialRx_Read(r, sizeof(r)) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (len[0] != len[1] || len[2] != PPI_SD2 || len[0] < PPI_RESP_HEAD_LEN) {
        ESP_LOGE(TAG, "bad frame header %02X %02X %02X", len[0], len[1], len[2]);
        return SERIAL_PARSE_ERROR;
    }
    if (r[17] != 0xFF) {
        ESP_LOGE(TAG, "read failed, return code 0x%02X", r[17]);
        return SERIAL_PARSE_ERROR;
    }

    dlen = len[0] - PPI_RESP_HEAD_LEN;
    want = S7_TypeBytes(ctx->var->type);
    if (dlen != S7_RespDataBytes(ctx->var, Proto_Be16(&r[19])) || dlen < want || dlen > PPI_DATA_MAX) {
        ESP_LOGE(TAG, "unexpected data length %d", dlen);
        return SERIAL_PARSE_ERROR;
    }
    if (SerialRx_Read(ctx->data, dlen) != 0 || SerialRx_Read(tail, sizeof(tail)) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (tail[1] != PPI_ED) {
        return SERIAL_PARSE_ERROR;
    }
    if ((uint8_t)(Proto_Sum8(r, sizeof(r)) + Proto_Sum8(ctx->data, dlen)) != tail[0]) {
        ESP_LOGW(TAG, "FCS mismatch");     // 现场尚未验证，暂只告警
    }
    return SERIAL_PARSE_OK;
}

int PPI_Read(const S7Var *var, PlcValue *out)
{
    uint8_t f[4 + sizeof(s_readHead) + 12 + 2];
    PpiCtx ctx = { .var = var };
    size_t n = 0;

    f[n++] = PPI_SD2;
    f[n++] = PPI_READ_LE;
    f[n++] = PPI_READ_LE;
    f[n++] = PPI_SD2;
    memcpy(&f[n], s_readHead, sizeof(s_readHead));
    f[n] = s_da;
    n += sizeof(s_readHead);
    S7_PutItem(&f[n], PpiTransport(var->type), 1, var);
    n += 12;
    f[n] = Proto_Sum8(&f[4], n - 4);    // DA ~ 地址
    n++;
    f[n++] = PPI_ED;

    if (SerialTxn_Request(f, n, PpiParse, &ctx, SERIAL_TXN_TIMEOUT_MS) != 0) {
        return PLC_ERR_RESPONSE;
    }
    S7_SetValue(var, ctx.data, out);
    return PLC_OK;
}
