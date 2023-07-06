#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <ctype.h>  
#include <string.h>
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "fx_plc_protocol.h"
#include "iot_common.h"

RingBuffer dataRegisterBuffer;
FxPlcReadFrameFormat rdatabuff = {0};
FxPlcSerialAskReadFrameFormat srdatabuff = {0};
FxPlcSerialAskReadBackFrameFormat srbdatabuff = {0};
FxPlcSerialAnsNackFrameFormat srbedatabuff = {0};
FxPlcSerialAnsAckFrameFormat mackdatabuff = {0};
FxPlcSerialAnsAckFrameFormat mnackdatabuff = {0};

static int CharToInt(char ch)  
{  
        // 如果是数字，则用数字的ASCII码减去48, 如果ch = '2' ,则 '2' - 48 = 2  
        if(isdigit(ch)) {
			return ch - 48;  
		}
  
        // 如果是字母，但不是A~F,a~f则返回  
        if( ch < 'A' || (ch > 'F' && ch < 'a') || ch > 'z' ) {
			return -1; 
		}
  
        // 如果是大写字母，则用数字的ASCII码减去55, 如果ch = 'A' ,则 'A' - 55 = 10  
        // 如果是小写字母，则用数字的ASCII码减去87, 如果ch = 'a' ,则 'a' - 87 = 10  
        if(isalpha(ch))  {
			return isupper(ch) ? ch - 55 : ch - 87;  
		}
  
        return -1;  
} 

static int MoveLeftArry(uint8_t *data, uint16_t length, uint16_t num)
{
	int temp, i, j;

	if (data == NULL || length <= 0) {
		return -1;
	}

	for (j = 0; j < num; j++) {
		temp = data[length - 1];
		for (i = length - 2; i >= 0; i--) {
			data[i + 1] = data[i];
		}
		data[0] = temp;
	}

	return 0;
}

static void AddZero(uint8_t num, uint8_t *data, uint16_t length)
{
	int i;

	switch (num) {
	case 1: {
		MoveLeftArry(data, length, length - num);
		for (i = 0; i < length - num; i++) {
			data[i] = '0';
		}
		break;
	}
	case 2: {
		MoveLeftArry(data, length, length - num);
		for (i = 0; i < length - num; i++) {
			data[i] = '0';
		}
		break;
	}
	case 3: {
		MoveLeftArry(data, length, length - num);
		for (i = 0; i < length - num; i++) {
			data[i] = '0';
		}
		break;
	}
	default: {

	}
	}
}

static uint32_t CalDataRegisterAddress(uint16_t address)
{
	uint16_t addr, num, i;
	uint32_t ret;
	uint8_t str[8] = { 0 };

	if (address >= 8000) {
		addr = (address - 8000) * 2 + PLC_D_SPECIAL_BASE_ADDRESS;
	}
	else {
		addr = address * 2 + PLC_D_BASE_ADDRESS;
	}

	num = sprintf((char *)str, "%x", addr);
	if ((num > 0) && (num < 4)) {
		AddZero(num, str, 4);
	}
	for (i = 0; i < 4; i++) {
		if (str[i] > 'F') {
			str[i] -= 32;
		}
	}
	memcpy(&ret, &str[0], sizeof(ret));

	return ret;
}

static uint32_t CalSerialDataRegisterAddress(uint16_t address)
{
	uint16_t num, i;
	uint32_t ret;
	uint8_t str[8] = { 0 };

	num = sprintf((char *)str, "%d", address);
	if ((num > 0) && (num < 4)) {
		AddZero(num, str, 4);
	}

	memcpy(&ret, &str[0], sizeof(ret));

	return ret;
}

static uint16_t CalLength(uint16_t length)
{
	uint16_t ret, len, i;
	uint8_t str[8] = { 0 };

	len = sprintf((char *)str, "%x", length);
	if ((len > 0) && (len < 2)) {
		AddZero(len, str, 2);
	}
	for (i = 0; i < 2; i++) {
		if (str[i] > 'F') {
			str[i] -= 32;
		}
	}
	memcpy(&ret, &str[0], sizeof(ret));
	return ret;
}

static uint8_t CalTimeout(uint8_t timeout)      // 0 - 150ms
{
	uint8_t ret = 0;

	ret = timeout / 10;
	if (ret >= 10 && ret <= 15) {
		ret = ret - 10 + 'A';
	} else {
		ret += '0';
	}
	return ret;
}

static uint16_t CalReadDataRegister(uint8_t *data)
{
	uint16_t rdata;

	rdata = CharToInt(data[2]) * 4096 + CharToInt(data[3]) * 256 + CharToInt(data[0]) * 16 + CharToInt(data[1]);
	return rdata;
}

static uint16_t CalSerialReadDataRegister(uint8_t *data)
{
	uint16_t rdata;

	rdata = CharToInt(data[0]) * 4096 + CharToInt(data[1]) * 256 + CharToInt(data[2]) * 16 + CharToInt(data[3]);
	return rdata;
}

int FXPLC_InitBuffer(void)
{
    if (RING_InitBuffer(&dataRegisterBuffer, FXPLC_BUFF_SIZE) != 0) {
        return -1;
    }

    return 0;
}

int FXPLC_WriteBufferBytes(uint8_t *data, uint32_t size)
{
    if (RING_WriteBufferBytes(&dataRegisterBuffer, data, size) != 0) {
        return -1;
    }
	return 0;
}

int FXPLC_ReadBufferBytes(uint8_t *data, uint32_t size)
{
    if (RING_ReadBufferBytes(&dataRegisterBuffer, data, size) != 0) {
        return -1;
    }
	return 0;
}

static int PackReadDataRegisterFrame(uint16_t address, uint16_t length, FxPlcReadFrameFormat* rdata)
{
	uint32_t addr;
	uint16_t len, sumcheck;

	if (rdata == NULL) {
		return -1;
	}

	rdata->stx = PLC_STX;
	rdata->etx = PLC_ETX;
	rdata->cmd = PLC_READ;

	addr = CalDataRegisterAddress(address);
	rdata->address[0] = addr & 0xff;
	rdata->address[1] = (addr >> 8) & 0xff;
	rdata->address[2] = (addr >> 16) & 0xff;
	rdata->address[3] = (addr >> 24) & 0xff;

	len = CalLength(length);
	rdata->length[0] = len & 0xff;
	rdata->length[1] = (len >> 8) & 0xff;

	sumcheck = CalSumCheckData((uint8_t *)(&(rdata->cmd)), PLC_READ_DATA_FRAME_CAL_LEAGTH);
	rdata->sum[0] = sumcheck & 0xff;
	rdata->sum[1] = (sumcheck >> 8) & 0xff;

	return 0;
}

static int PackInputRelayFrame(uint16_t length, FxPlcReadFrameFormat* rdata)
{
	const uint8_t iraddr[4] = { 0x30, 0x30, 0x38, 0x30 };    // address: 0x80 = 0 + PLC_X_GROUP_BASE_ADDRESS
	uint16_t len, sumcheck;

	if (length > FX_PLC_MAX_X_LEN || rdata == NULL) {
		return -1;
	}

	rdata->stx = PLC_STX;
	rdata->etx = PLC_ETX;
	rdata->cmd = PLC_READ;

	rdata->address[0] = iraddr[0];
	rdata->address[1] = iraddr[1];
	rdata->address[0] = iraddr[2];
	rdata->address[1] = iraddr[3];

	len = CalLength(length);
	rdata->length[0] = len & 0xff;
	rdata->length[1] = (len >> 8) & 0xff;

	sumcheck = CalSumCheckData((uint8_t *)(&(rdata->cmd)), PLC_READ_DATA_FRAME_CAL_LEAGTH);
	rdata->sum[0] = sumcheck & 0xff;
	rdata->sum[1] = (sumcheck >> 8) & 0xff;

	return 0;
}

static int PackOutputRelayFrame(uint16_t length, FxPlcReadFrameFormat* rdata)
{
	const uint8_t iraddr[4] = { 0x30, 0x30, 0x41, 0x30 };    // address: 0x80 = 0 + PLC_Y_GROUP_BASE_ADDRESS
	uint16_t len, sumcheck;

	if (length > FX_PLC_MAX_Y_LEN || rdata == NULL) {
		return -1;
	}

	rdata->stx = PLC_STX;
	rdata->etx = PLC_ETX;
	rdata->cmd = PLC_READ;

	rdata->address[0] = iraddr[0];
	rdata->address[1] = iraddr[1];
	rdata->address[0] = iraddr[2];
	rdata->address[1] = iraddr[3];

	len = CalLength(length);
	rdata->length[0] = len & 0xff;
	rdata->length[1] = (len >> 8) & 0xff;

	sumcheck = CalSumCheckData((uint8_t *)(&(rdata->cmd)), PLC_READ_DATA_FRAME_CAL_LEAGTH);
	rdata->sum[0] = sumcheck & 0xff;
	rdata->sum[1] = (sumcheck >> 8) & 0xff;

	return 0;
}

static int PackSerialReadDataRegisterFrame(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address,  
	uint16_t length, FxPlcSerialAskReadFrameFormat* rdata)
{
	uint32_t addr;
	uint16_t plcn, pcn, len, sumcheck;
	const char *cmd = PLC_WR;

	if (rdata == NULL) {
		return -1;
	}

	rdata->enq = PLC_ENQ;

	plcn = CalLength(plcnum);
	rdata->plcnum[0] = plcn & 0xff;
	rdata->plcnum[1] = (plcn >> 8) & 0xff;

	pcn = CalLength(pcnum);
	rdata->pcnum[0] = pcn & 0xff;
	rdata->pcnum[1] = (pcn >> 8) & 0xff;

	rdata->cmd[0] = cmd[0];
	rdata->cmd[1] = cmd[1];

	rdata->timeout = CalTimeout(timeout);

	addr = CalSerialDataRegisterAddress(address);
	rdata->address[0] = 'D';
	rdata->address[1] = addr & 0xff;
	rdata->address[2] = (addr >> 8) & 0xff;
	rdata->address[3] = (addr >> 16) & 0xff;
	rdata->address[4] = (addr >> 24) & 0xff;

	len = CalLength(length);
	rdata->length[0] = len & 0xff;
	rdata->length[1] = (len >> 8) & 0xff;

	sumcheck = CalSumCheckData((uint8_t *)(&(rdata->plcnum[0])), PLC_SERIAL_READ_DATA_FRAME_CAL_LEAGTH);
	rdata->sum[0] = sumcheck & 0xff;
	rdata->sum[1] = (sumcheck >> 8) & 0xff;
	
	return 0;
}

int GetDataFromFxPlc(int *length)
{
	uint8_t curData = 0;
	uint8_t dataArry[4] = {0};
	uint16_t rdata;
	int ret, len = 0, count = 0;
	static const char *TAG = "GET_DATA";

	while (curData != PLC_STX) {
		ret = UART_ReadBufferBytes(&curData, 1);
		if (ret != 0) {
			return -1;
		}
	}

	do {
		ret = UART_ReadBufferBytes(&curData, 1);
		if (ret != 0) {
			return -1;
		}

		if (count < 4) {
			dataArry[count] = curData;
		}
		count++;
		if (count % 4 == 0) {
			count = 0;
			len++;
			rdata = CalReadDataRegister(dataArry);
			ret = FXPLC_WriteBufferBytes(&rdata, sizeof(rdata));
			if (ret != 0) {
				return -1;
			}
		}
	} while (curData != PLC_ETX);

	*length = len;
	return 0;
}

int GetSerialDataFromFxPlc(int *length)
{
	uint8_t curData = 0;
	uint8_t dataArry[4] = {0};
	uint16_t rdata;
	int ret, len = 0, count = 0;
	static const char *TAG = "GET_SERIAL_DATA";

	while ((curData != PLC_STX) && (curData != PLC_NAK)) {
		ret = UART_ReadBufferBytes(&curData, 1);
		if (ret != 0) {
			return -1;
		}
	}

	if (curData == PLC_NAK) {
		srbedatabuff.nak = PLC_NAK;
		ret = UART_ReadBufferBytes(srbedatabuff.plcnum, sizeof(srbedatabuff.plcnum));
		if (ret != 0) {
			return -1;
		}
		ret = UART_ReadBufferBytes(srbedatabuff.pcnum, sizeof(srbedatabuff.pcnum));
		if (ret != 0) {
			return -1;
		}
		ret = UART_ReadBufferBytes(srbedatabuff.errorcode, sizeof(srbedatabuff.errorcode));
		if (ret != 0) {
			return -1;
		}
		*length = 0;
		ESP_LOGE(TAG, "PLC:%c%c, PC:%c%c, errorcode:%c%c", srbedatabuff.plcnum[0], srbedatabuff.plcnum[1], 
			srbedatabuff.pcnum[0], srbedatabuff.pcnum[1], srbedatabuff.errorcode[0], srbedatabuff.errorcode[1]);
	} else {
		srbdatabuff.stx = PLC_STX;
		ret = UART_ReadBufferBytes(srbdatabuff.plcnum, sizeof(srbdatabuff.plcnum));
		if (ret != 0) {
			return -1;
		}
		ret = UART_ReadBufferBytes(srbdatabuff.pcnum, sizeof(srbdatabuff.pcnum));
		if (ret != 0) {
			return -1;
		}
		do {
			ret = UART_ReadBufferBytes(&curData, 1);
			if (ret != 0) {
				return -1;
			}

			if (count < 4) {
				dataArry[count] = curData;
			}
			count++;
			if (count % 4 == 0) {
				count = 0;
				len++;
				rdata = CalSerialReadDataRegister(dataArry);
				ret = FXPLC_WriteBufferBytes(&rdata, sizeof(rdata));
				if (ret != 0) {
					return -1;
				}
			}
		} while (curData != PLC_ETX);
		*length = len;
	}

	return 0;
}

void SendAckToPlc(void)
{
	mackdatabuff.ack = PLC_ACK;
	mackdatabuff.plcnum[0] = srbdatabuff.plcnum[0];
	mackdatabuff.plcnum[1] = srbdatabuff.plcnum[1];
	mackdatabuff.pcnum[0] = srbdatabuff.pcnum[0];
	mackdatabuff.pcnum[1] = srbdatabuff.pcnum[1];

	uart_write_bytes(UART_NUM_1, (uint8_t *)&mackdatabuff, sizeof(FxPlcSerialAnsAckFrameFormat));
}

void SendNackToPlc(void)
{
	mackdatabuff.ack = PLC_NAK;
	mackdatabuff.plcnum[0] = srbedatabuff.plcnum[0];
	mackdatabuff.plcnum[1] = srbedatabuff.plcnum[1];
	mackdatabuff.pcnum[0] = srbedatabuff.pcnum[0];
	mackdatabuff.pcnum[1] = srbedatabuff.pcnum[1];

	uart_write_bytes(UART_NUM_1, (uint8_t *)&mackdatabuff, sizeof(FxPlcSerialAnsAckFrameFormat));
}

void ReadSingleDataRegister(uint16_t address)   // RS232
{
	PackReadDataRegisterFrame(address, 2, &rdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&rdatabuff, sizeof(FxPlcReadFrameFormat));
}

void SerialReadSingleDataRegister(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address)    // RS485
{
	PackSerialReadDataRegisterFrame(plcnum, pcnum, timeout, address, 1, &srdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&srdatabuff, PLC_SERIAL_READ_DATA_FRAME_LEAGTH);
}

void ReadMulDataRegister(uint16_t startaddr, uint16_t length)
{
	int i;

	for (i = 0; i < length; i++) {
		ReadSingleDataRegister(startaddr + i);
		vTaskDelay(10);
	}
}

int ReadInputRelayData()
{
	int ret;

	ret = PackInputRelayFrame(FX_PLC_MAX_X_LEN, &rdatabuff);
	if (ret != 0) {
		return ret;
	}

	uart_write_bytes(UART_NUM_1, (uint8_t *)&rdatabuff, PLC_READ_DATA_FRAME_LEAGTH);
	return 0;
}

int ReadOutputRelayData()
{
	int ret;

	ret = PackOutputRelayFrame(FX_PLC_MAX_Y_LEN, &rdatabuff);
	if (ret != 0) {
		return ret;
	}

	uart_write_bytes(UART_NUM_1, (uint8_t *)&rdatabuff, sizeof(FxPlcReadFrameFormat));
	return 0;
}
