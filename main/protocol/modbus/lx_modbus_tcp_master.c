#include <string.h>
#include "esp_log.h"
#include "lx_modbus_tcp_master.h"
#include "proto_util.h"
#include "net_io.h"
#include "mb_pdu.h"

static const char *TAG = "MODBUS_TCP";

#define MBAP_LEN                7
#define MODBUS_PDU_MAX          253

#define MODBUS_FC_READ_HOLDING  0x03
#define MODBUS_FC_WRITE_SINGLE  0x06
#define MODBUS_FC_WRITE_MULTI   0x10

static uint16_t s_transId;
static uint8_t s_unitId = MODBUS_TCP_UNIT_ID;

void MODBUS_SetUnitId(uint8_t unit)
{
    s_unitId = unit;
}

/*
 * 发送一个请求 PDU 并接收应答 PDU（不含 MBAP 头）
 * 成功返回应答 PDU 长度，失败返回 PLC_ERR_RESPONSE / PLC_ERR_LINK
 */
static int MbTransact(int sock, const uint8_t *pdu, size_t plen, uint8_t *resp, size_t cap)
{
    uint8_t adu[MBAP_LEN + MODBUS_PDU_MAX];
    uint8_t mbap[MBAP_LEN];
    const uint16_t tid = ++s_transId;
    uint16_t rlen;
    size_t n;

    if (plen > MODBUS_PDU_MAX) {
        return PLC_ERR_RESPONSE;
    }
    adu[0] = (uint8_t)(tid >> 8);
    adu[1] = (uint8_t)tid;
    adu[2] = 0x00;                      // 协议标识
    adu[3] = 0x00;
    adu[4] = (uint8_t)((plen + 1) >> 8);    // 后续长度：单元标识 + PDU
    adu[5] = (uint8_t)(plen + 1);
    adu[6] = s_unitId;
    memcpy(&adu[MBAP_LEN], pdu, plen);

    if (Net_SendAll(sock, adu, MBAP_LEN + plen) != 0 ||
        Net_RecvExact(sock, mbap, sizeof(mbap)) != 0) {
        return PLC_ERR_LINK;
    }
    rlen = Proto_Be16(&mbap[4]);
    if (Proto_Be16(&mbap[2]) != 0 || rlen < 2 || rlen - 1 > MODBUS_PDU_MAX) {
        ESP_LOGE(TAG, "bad MBAP header, length %u", rlen);
        return PLC_ERR_LINK;            // 数据流已错位，重连
    }
    n = rlen - 1;
    if (n > cap) {
        Net_Discard(sock, n);
        return PLC_ERR_RESPONSE;
    }
    if (Net_RecvExact(sock, resp, n) != 0) {
        return PLC_ERR_LINK;
    }

    if (Proto_Be16(&mbap[0]) != tid) {
        ESP_LOGE(TAG, "transaction id mismatch");
        return PLC_ERR_RESPONSE;
    }
    if (resp[0] == (pdu[0] | 0x80)) {
        ESP_LOGE(TAG, "function 0x%02X exception %u", pdu[0], (n > 1) ? resp[1] : 0);
        return PLC_ERR_RESPONSE;
    }
    if (resp[0] != pdu[0]) {
        return PLC_ERR_RESPONSE;
    }
    return (int)n;
}

int MODBUS_NetReadHolding(int sock, uint16_t address, uint16_t count, uint16_t *regs)
{
    const uint8_t pdu[] = {
        MODBUS_FC_READ_HOLDING,
        (uint8_t)(address >> 8), (uint8_t)address,
        (uint8_t)(count >> 8), (uint8_t)count,
    };
    uint8_t resp[2 + MODBUS_MAX_READ_REGS * 2];
    int n;

    if (count == 0 || count > MODBUS_MAX_READ_REGS) {
        return PLC_ERR_RESPONSE;
    }
    n = MbTransact(sock, pdu, sizeof(pdu), resp, sizeof(resp));
    if (n < 0) {
        return n;
    }
    if (n < 2 || resp[1] != count * 2 || n < 2 + count * 2) {
        ESP_LOGE(TAG, "unexpected byte count %u", (n > 1) ? resp[1] : 0);
        return PLC_ERR_RESPONSE;
    }
    for (int i = 0; i < count; i++) {
        regs[i] = Proto_Be16(&resp[2 + i * 2]);
    }
    return PLC_OK;
}

int MODBUS_NetRead(int sock, uint16_t address, ModbusFormat fmt, PlcValue *out)
{
    uint16_t regs[2];
    const uint16_t count = (fmt == MODBUS_FMT_U16) ? 1 : 2;
    int ret;

    ret = MODBUS_NetReadHolding(sock, address, count, regs);
    if (ret != PLC_OK) {
        return ret;
    }

    switch (fmt) {
    case MODBUS_FMT_I32:
        PlcValue_SetInt(out, (int32_t)(((uint32_t)regs[0] << 16) | regs[1]));
        break;
    case MODBUS_FMT_FLOAT:
        PlcValue_SetFloatBits(out, ((uint32_t)regs[0] << 16) | regs[1]);
        break;
    default:
        PlcValue_SetInt(out, regs[0]);
        break;
    }
    return PLC_OK;
}

int MODBUS_NetWriteSingle(int sock, uint16_t address, uint16_t value)
{
    const uint8_t pdu[] = {
        MODBUS_FC_WRITE_SINGLE,
        (uint8_t)(address >> 8), (uint8_t)address,
        (uint8_t)(value >> 8), (uint8_t)value,
    };
    uint8_t resp[8];
    int n = MbTransact(sock, pdu, sizeof(pdu), resp, sizeof(resp));

    return (n < 0) ? n : PLC_OK;
}

int MODBUS_NetWriteMultiple(int sock, uint16_t address, uint16_t count, const uint16_t *values)
{
    uint8_t pdu[6 + 2 * 123];
    uint8_t resp[8];
    int n;

    if (count == 0 || count > 123) {
        return PLC_ERR_RESPONSE;
    }
    pdu[0] = MODBUS_FC_WRITE_MULTI;
    pdu[1] = (uint8_t)(address >> 8);
    pdu[2] = (uint8_t)address;
    pdu[3] = (uint8_t)(count >> 8);
    pdu[4] = (uint8_t)count;
    pdu[5] = (uint8_t)(count * 2);
    for (int i = 0; i < count; i++) {
        pdu[6 + i * 2] = (uint8_t)(values[i] >> 8);
        pdu[7 + i * 2] = (uint8_t)values[i];
    }

    n = MbTransact(sock, pdu, 6 + count * 2, resp, sizeof(resp));
    return (n < 0) ? n : PLC_OK;
}

int MODBUS_NetReadFc(int sock, uint8_t fc, uint16_t address, uint16_t count, uint16_t *out)
{
    uint8_t pdu[5];
    uint8_t resp[2 + MB_MAX_READ_REGS * 2];
    int n;

    if (count == 0 || MB_ReadDataBytes(fc, count) > sizeof(resp) - 2) {
        return PLC_ERR_RESPONSE;
    }
    MB_PutReadPdu(pdu, fc, address, count);
    n = MbTransact(sock, pdu, sizeof(pdu), resp, sizeof(resp));
    if (n < 0) {
        return n;
    }
    if (MB_ParseReadPdu(fc, count, resp, (size_t)n, out) != 0) {
        ESP_LOGE(TAG, "function 0x%02X: unexpected response length %d", fc, n);
        return PLC_ERR_RESPONSE;
    }
    return PLC_OK;
}
