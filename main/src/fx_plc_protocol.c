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
FxPlcReadFrameFormat readDataFrame = {0};
FxPlcReadFrameFormat mptr = {0};

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

static uint16_t CalReadDataRegister(uint8_t *data)
{
	uint16_t rdata;

	rdata = CharToInt(data[2]) * 4096 + CharToInt(data[3]) * 256 + CharToInt(data[0]) * 16 + CharToInt(data[1]);
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

void PackReadDataRegisterFrame(uint16_t address, uint16_t length, FxPlcReadFrameFormat* rdata)
{
	uint32_t addr;
	uint16_t len, sumcheck;

	readDataFrame.stx = PLC_STX;
	readDataFrame.etx = PLC_ETX;
	readDataFrame.cmd = PLC_READ;

	addr = CalDataRegisterAddress(address);
	readDataFrame.address[0] = addr & 0xff;
	readDataFrame.address[1] = (addr >> 8) & 0xff;
	readDataFrame.address[2] = (addr >> 16) & 0xff;
	readDataFrame.address[3] = (addr >> 24) & 0xff;

	len = CalLength(length);
	readDataFrame.length[0] = len & 0xff;
	readDataFrame.length[1] = (len >> 8) & 0xff;

	sumcheck = CalSumCheckData((uint8_t *)(&(readDataFrame.cmd)), PLC_READ_DATA_FRAME_CAL_LEAGTH);
	readDataFrame.sum[0] = sumcheck & 0xff;
	readDataFrame.sum[1] = (sumcheck >> 8) & 0xff;

    memcpy(rdata, &readDataFrame, sizeof(FxPlcReadFrameFormat));
}

void PackInputRelayFrame(uint16_t address, uint16_t length, FxPlcReadFrameFormat* rdata)
{
	
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

void ReadSingleDataRegister(uint16_t address)
{
	PackReadDataRegisterFrame(address, 2, &mptr);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&mptr, sizeof(FxPlcReadFrameFormat));
}

void ReadMulDataRegister(uint16_t startaddr, uint16_t length)
{
	int i;

	for (i = 0; i < length; i++) {
		ReadSingleDataRegister(startaddr + i);
		vTaskDelay(10);
	}
}
