#include "lx_modbus_tcp_master.h"
#include "iot_common.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

uint32_t mdnetsrdatabuffaddr = 0;
uint32_t mdnetsrdatalen = 0;
int mdnetdatarecvflag = 0;
uint16_t g_trans_id = 0;

static void MODBUS_NetDataRecvNotice(uint32_t dataaddr, uint32_t len, uint16_t frnum, uint8_t datatype)
{
    mdnetdatarecvflag = 1;
	mdnetsrdatabuffaddr = dataaddr;
	mdnetsrdatalen = len;
	g_fxplccount = frnum;
	g_fxplcdataformat = datatype;
}

// 发送并接收，返回接收长度或负值错误
static int send_and_recv(int sock, const uint8_t *req, int req_len,
                         uint8_t *resp, int resp_max_len)
{
    if (send(sock, req, req_len, 0) != req_len)
        return MODBUS_ERR_SEND;

    int len = recv(sock, resp, resp_max_len, 0);
    if (len <= 0)
        return MODBUS_ERR_RECV;

    return len;
}

int modbus_tcp_read_holding_registers(int sock, uint16_t trans_id,
                                      uint8_t unit_id, uint16_t start_addr,
                                      uint16_t quantity, uint16_t *out_data)
{
    uint8_t req[MODBUS_TCP_HEADER_SIZE + 5];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    req[0] = trans_id >> 8;
    req[1] = trans_id & 0xFF;
    req[2] = MODBUS_TCP_PROTOCOL_ID >> 8;
    req[3] = MODBUS_TCP_PROTOCOL_ID & 0xFF;
    req[4] = 0x00;
    req[5] = 0x06; // UnitID(1) + Func(1) + Addr(2) + Qty(2)
    req[6] = unit_id;

    req[7] = MODBUS_FUNC_READ_HOLDING_REGS;
    req[8] = start_addr >> 8;
    req[9] = start_addr & 0xFF;
    req[10] = quantity >> 8;
    req[11] = quantity & 0xFF;

    int len = send_and_recv(sock, req, sizeof(req), resp, sizeof(resp));
    if (len < (MODBUS_TCP_HEADER_SIZE + 2))
        return MODBUS_ERR_RESP_LENGTH;

    if (resp[7] != MODBUS_FUNC_READ_HOLDING_REGS)
        return MODBUS_ERR_FUNC_MISMATCH;

    uint8_t byte_count = resp[8];
    if (byte_count != quantity * 2)
        return MODBUS_ERR_DATA_MISMATCH;

    for (int i = 0; i < quantity; i++) {
        out_data[i] = (resp[9 + i*2] << 8) | resp[10 + i*2];
    }
    return MODBUS_OK;
}

int modbus_tcp_read_holding_registers_32(int sock, uint16_t trans_id,
                                         uint8_t unit_id, uint16_t start_addr,
                                         uint16_t quantity_32, uint32_t *out_data_32,
                                         int high_word_first)
{
    uint16_t quantity_16 = quantity_32 * 2;
    uint16_t *tmp_buffer = malloc(sizeof(uint16_t) * quantity_16);
    if (!tmp_buffer)
        return MODBUS_ERR_SEND;

    int ret = modbus_tcp_read_holding_registers(sock, trans_id, unit_id, start_addr, quantity_16, tmp_buffer);
    if (ret != MODBUS_OK) {
        free(tmp_buffer);
        return ret;
    }

    for (int i = 0; i < quantity_32; i++) {
        uint16_t high_word, low_word;
        if (high_word_first) {
            high_word = tmp_buffer[i*2];
            low_word  = tmp_buffer[i*2 + 1];
        } else {
            low_word  = tmp_buffer[i*2];
            high_word = tmp_buffer[i*2 + 1];
        }
        out_data_32[i] = ((uint32_t)high_word << 16) | low_word;
    }

    free(tmp_buffer);
    return MODBUS_OK;
}

int modbus_tcp_write_single_register(int sock, uint16_t trans_id,
                                     uint8_t unit_id, uint16_t reg_addr,
                                     uint16_t value)
{
    uint8_t req[MODBUS_TCP_HEADER_SIZE + 5];
    uint8_t resp[MODBUS_TCP_HEADER_SIZE + 5];

    req[0] = trans_id >> 8;
    req[1] = trans_id & 0xFF;
    req[2] = MODBUS_TCP_PROTOCOL_ID >> 8;
    req[3] = MODBUS_TCP_PROTOCOL_ID & 0xFF;
    req[4] = 0x00;
    req[5] = 0x06;
    req[6] = unit_id;

    req[7] = MODBUS_FUNC_WRITE_SINGLE_REG;
    req[8] = reg_addr >> 8;
    req[9] = reg_addr & 0xFF;
    req[10] = value >> 8;
    req[11] = value & 0xFF;

    int len = send_and_recv(sock, req, sizeof(req), resp, sizeof(resp));
    if (len < sizeof(resp))
        return MODBUS_ERR_RESP_LENGTH;

    if (resp[7] != MODBUS_FUNC_WRITE_SINGLE_REG)
        return MODBUS_ERR_FUNC_MISMATCH;

    return MODBUS_OK;
}

int modbus_tcp_write_multiple_registers(int sock, uint16_t trans_id,
                                        uint8_t unit_id, uint16_t start_addr,
                                        uint16_t quantity, const uint16_t *values)
{
    uint8_t req[MODBUS_TCP_MAX_ADU_LENGTH];
    uint8_t resp[MODBUS_TCP_MAX_ADU_LENGTH];

    uint16_t byte_count = quantity * 2;
    uint16_t length = 1 + 1 + 2 + 2 + 1 + byte_count;

    req[0] = trans_id >> 8;
    req[1] = trans_id & 0xFF;
    req[2] = MODBUS_TCP_PROTOCOL_ID >> 8;
    req[3] = MODBUS_TCP_PROTOCOL_ID & 0xFF;
    req[4] = length >> 8;
    req[5] = length & 0xFF;
    req[6] = unit_id;

    req[7] = MODBUS_FUNC_WRITE_MULTIPLE_REGS;
    req[8] = start_addr >> 8;
    req[9] = start_addr & 0xFF;
    req[10] = quantity >> 8;
    req[11] = quantity & 0xFF;
    req[12] = byte_count;

    for (int i = 0; i < quantity; i++) {
        req[13 + i*2] = values[i] >> 8;
        req[14 + i*2] = values[i] & 0xFF;
    }

    int len = send_and_recv(sock, req, 13 + byte_count, resp, sizeof(resp));
    if (len < (MODBUS_TCP_HEADER_SIZE + 5))
        return MODBUS_ERR_RESP_LENGTH;

    if (resp[7] != MODBUS_FUNC_WRITE_MULTIPLE_REGS)
        return MODBUS_ERR_FUNC_MISMATCH;

    return MODBUS_OK;
}

int GetNetDataFromModbusPlc(void)
{
	uint16_t datalen;
	int16_t tdata;
	uint8_t* data = NULL;
	static const char *TAG = "MODBUS_GET_NET_DATA";
	int i, ret;

	if (mdnetdatarecvflag != 1 || mdnetsrdatabuffaddr == NULL || mdnetsrdatalen == 0) {
		return -1;
	}

	data = (uint8_t *)mdnetsrdatabuffaddr;
    if (g_fxplcdataformat == PLC_DATA_TYPE_SHORT) {
        for (i = 0; i < mdnetsrdatalen * 2; i++) {
            ret = FXPLC_WriteBufferBytes(&data[i], 1);
            if (ret != 0) {
                ESP_LOGE(TAG, "buffer error: %d", ret);
                return -1;
            }
        }
    } else if (g_fxplcdataformat == PLC_DATA_TYPE_FLOAT || g_fxplcdataformat == PLC_DATA_TYPE_INT) {
        for (i = 0; i < mdnetsrdatalen * 4; i++) {
            ret = FXPLC_WriteBufferBytes(&data[i], 1);
            if (ret != 0) {
                ESP_LOGE(TAG, "buffer error: %d", ret);
                return -1;
            }
        }
    }
	
	mdnetdatarecvflag = 0;

	return 0;
}

void MODBUS_NetReadSingleDataRegister(int sock, uint32_t address, uint16_t frnum)
{
    int i, err;
    uint16_t read_regs;
    static const char *TAG = "MODBUS_READ_SINGLE_DATA";

    err = modbus_tcp_read_holding_registers(sock, g_trans_id, 1, address, 1, &read_regs);
    if (err == MODBUS_OK) {
        ESP_LOGI(TAG, "Read 16-bit regs: %u", read_regs);
    } else {
        ESP_LOGE(TAG, "Modbus 16-bit read failed: %d", err);
        // break;
    }
    g_trans_id = (g_trans_id + 1) & 0xFFFF;
    
    MODBUS_NetDataRecvNotice(&read_regs, 1, frnum, PLC_DATA_TYPE_SHORT);
    vTaskDelay(20);
}

void MODBUS_NetReadDoubleDataRegister(int sock, uint32_t address, uint16_t frnum)
{
    int i, err;
    uint32_t read_regs;
    static const char *TAG = "MODBUS_READ_DOUBLE_DATA";

    err = modbus_tcp_read_holding_registers_32(sock, g_trans_id, 1, address, 1, &read_regs, 1);
    if (err == MODBUS_OK) {
        ESP_LOGI(TAG, "Read 16-bit regs: %u", read_regs);
    } else {
        ESP_LOGE(TAG, "Modbus 16-bit read failed: %d", err);
        // break;
    }
    g_trans_id = (g_trans_id + 1) & 0xFFFF;

    MODBUS_NetDataRecvNotice(&read_regs, 1, frnum, PLC_DATA_TYPE_INT);
    vTaskDelay(20);
}

void MODBUS_NetReadFloatRegister(int sock, uint32_t address, uint16_t frnum)
{
    int i, err;
    uint32_t read_regs;
    static const char *TAG = "MODBUS_READ_FLOAT_DATA";

    err = modbus_tcp_read_holding_registers_32(sock, g_trans_id, 1, address, 1, &read_regs, 1);
    if (err == MODBUS_OK) {
        ESP_LOGI(TAG, "Read 32-bit regs: %f", (float)read_regs);
    } else {
        ESP_LOGE(TAG, "Modbus 32-bit read failed: %d", err);
        // break;
    }

    g_trans_id = (g_trans_id + 1) & 0xFFFF;
    
    MODBUS_NetDataRecvNotice(&read_regs, 1, frnum, PLC_DATA_TYPE_FLOAT);
    vTaskDelay(20);
}
