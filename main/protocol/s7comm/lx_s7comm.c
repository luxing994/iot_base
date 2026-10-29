#include <string.h>
#include "esp_log.h"
#include "lx_s7comm.h"
#include "proto_util.h"
#include "net_io.h"

static const char *TAG = "S7COMM";

#define TPKT_VERSION        0x03
#define TPKT_HEADER_LEN     4
#define COTP_CC_PDU_TYPE    0xD0
#define S7_PROTOCOL_ID      0x32
#define S7_ROSCTR_ACK_DATA  0x03
#define S7_RESP_MAX         256

static uint16_t s_pduRef;

/*
 * 接收一个完整的 TPKT 报文（按报文头中的总长度读取），返回报文总长度。
 * 超出缓冲区的部分读出后丢弃。失败返回 -1。
 */
static int S7_RecvTpkt(int sock, uint8_t *buf, size_t cap)
{
    uint16_t total;
    size_t keep;

    if (Net_RecvExact(sock, buf, TPKT_HEADER_LEN) != 0) {
        return -1;
    }
    total = Proto_Be16(&buf[2]);
    if (buf[0] != TPKT_VERSION || total < TPKT_HEADER_LEN) {
        ESP_LOGE(TAG, "bad TPKT header %02X len %u", buf[0], total);
        return -1;
    }
    keep = (total < cap) ? total : cap;
    if (Net_RecvExact(sock, buf + TPKT_HEADER_LEN, keep - TPKT_HEADER_LEN) != 0) {
        return -1;
    }
    if (total > keep && Net_Discard(sock, total - keep) != 0) {
        return -1;
    }
    return (int)keep;
}

/* ------------------------------------------------------------------ 建立连接 */

// COTP 连接请求（CR）：本地 TSAP 0x0101，远端 TSAP 0x01 + 机架槽号
static uint8_t s_remoteTsap = 0x01;     // 机架 * 32 + 槽号，默认 0 机架 1 槽（S7-200 SMART 现场验证过）

void S7_SetRackSlot(uint8_t rack, uint8_t slot)
{
    s_remoteTsap = (uint8_t)((rack << 5) | (slot & 0x1F));
}

static int S7_CotpConnect(int sock)
{
    uint8_t cr[] = {
        0x03, 0x00, 0x00, 0x16,                 // TPKT
        0x11, 0xE0, 0x00, 0x00, 0x04, 0x30, 0x00,   // COTP CR
        0xC1, 0x02, 0x01, 0x01,                 // 源 TSAP
        0xC2, 0x02, 0x01, 0x01,                 // 目的 TSAP
        0xC0, 0x01, 0x0A,                       // TPDU 大小 1024
    };
    uint8_t rsp[64];
    int len;

    cr[18] = s_remoteTsap;
    if (Net_SendAll(sock, cr, sizeof(cr)) != 0) {
        return -1;
    }
    len = S7_RecvTpkt(sock, rsp, sizeof(rsp));
    if (len < 6 || rsp[5] != COTP_CC_PDU_TYPE) {
        ESP_LOGE(TAG, "COTP connect refused");
        return -1;
    }
    return 0;
}

// S7 通信设置：协商 PDU 长度
static int S7_SetupCommunication(int sock)
{
    static const uint8_t req[] = {
        0x03, 0x00, 0x00, 0x19,                 // TPKT
        0x02, 0xF0, 0x80,                       // COTP DT
        0x32, 0x01, 0x00, 0x00, 0xCC, 0xC1, 0x00, 0x08, 0x00, 0x00,   // S7 头
        0xF0, 0x00, 0x00, 0x01, 0x00, 0x01, 0x03, 0xC0,               // 最大 PDU 960
    };
    uint8_t rsp[64];
    int len;

    if (Net_SendAll(sock, req, sizeof(req)) != 0) {
        return -1;
    }
    len = S7_RecvTpkt(sock, rsp, sizeof(rsp));
    if (len < 27 || rsp[5] != 0xF0 || rsp[7] != S7_PROTOCOL_ID) {
        ESP_LOGE(TAG, "setup communication failed");
        return -1;
    }
    ESP_LOGI(TAG, "PDU size: %u", Proto_Be16(&rsp[25]));
    return 0;
}

int S7_Handshake(int sock)
{
    if (S7_CotpConnect(sock) != 0 || S7_SetupCommunication(sock) != 0) {
        return -1;
    }
    ESP_LOGI(TAG, "S7 connection established");
    return 0;
}

/* ------------------------------------------------------------------ 读变量 */

/*
 * 请求的传输尺寸沿用现场验证过的写法：
 * 位 -> BIT x1，字节 -> BYTE x1，16 位 -> WORD x1，32 位 -> WORD x2
 */
static void S7_NetTransport(uint8_t type, uint8_t *ts, uint16_t *count)
{
    switch (type) {
    case S7_BIT:  *ts = S7_TS_BIT;  *count = 1; break;
    case S7_BYTE: *ts = S7_TS_BYTE; *count = 1; break;
    case S7_INT:  *ts = S7_TS_WORD; *count = 1; break;
    default:      *ts = S7_TS_WORD; *count = 2; break;
    }
}

int S7_NetRead(int sock, const S7Var *var, PlcValue *out)
{
    // 每次请求前发一个空的 COTP 数据帧，与现场验证过的原实现保持一致
    static const uint8_t cotp_dt[] = { 0x03, 0x00, 0x00, 0x07, 0x02, 0xF0, 0x00 };
    uint8_t req[31] = {
        0x03, 0x00, 0x00, 31,                   // TPKT，总长 31
        0x02, 0xF0, 0x80,                       // COTP DT
        S7_PROTOCOL_ID, 0x01, 0x00, 0x00,       // S7 头：作业请求
        0x00, 0x00,                             // PDU 参考（下面填写）
        0x00, 0x0E, 0x00, 0x00,                 // 参数长度 14，数据长度 0
        0x04, 0x01,                             // ReadVar，1 个 Item
    };
    uint8_t rsp[S7_RESP_MAX];
    const uint8_t *h = &rsp[7];     // S7 头
    uint8_t ts;
    uint16_t count;
    int len, dlen;

    s_pduRef++;
    req[11] = (uint8_t)(s_pduRef >> 8);
    req[12] = (uint8_t)s_pduRef;
    S7_NetTransport(var->type, &ts, &count);
    S7_PutItem(&req[19], ts, count, var);

    if (Net_SendAll(sock, cotp_dt, sizeof(cotp_dt)) != 0 ||
        Net_SendAll(sock, req, sizeof(req)) != 0) {
        return PLC_ERR_LINK;
    }
    len = S7_RecvTpkt(sock, rsp, sizeof(rsp));
    if (len < 0) {
        return PLC_ERR_LINK;
    }

    // S7 头(12) | 04 01 | 返回码 类型 长度(2) | 数据
    if (len < 7 + 18 || h[0] != S7_PROTOCOL_ID || h[1] != S7_ROSCTR_ACK_DATA) {
        ESP_LOGE(TAG, "bad response (len %d)", len);
        return PLC_ERR_RESPONSE;
    }
    if (h[10] != 0 || h[11] != 0 || h[14] != 0xFF) {
        ESP_LOGE(TAG, "read area 0x%02X byte %u failed: error %02X%02X, return code %02X",
                 var->area, (unsigned)var->byte, h[10], h[11], h[14]);
        return PLC_ERR_RESPONSE;
    }
    dlen = S7_RespDataBytes(var, Proto_Be16(&h[16]));
    if (dlen != S7_TypeBytes(var->type) || len < 7 + 18 + dlen) {
        ESP_LOGE(TAG, "unexpected data length %d", dlen);
        return PLC_ERR_RESPONSE;
    }

    S7_SetValue(var, &h[18], out);
    return PLC_OK;
}
