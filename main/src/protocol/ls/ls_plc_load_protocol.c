#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <ctype.h>  
#include <string.h>
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "ls_plc_load_protocol.h"
#include "iot_common.h"

LsLoadCommandFrameFormat lsloadsdatabuff = {0};
LsLoadAckFrameFormat lsloadackrdatabuff = {0};
LsLoadNackFrameFormat lsloadnackrdatabuff = {0};

static int LSLoadPackReadWordDataRegisterFrame(uint32_t address, uint16_t length, LsLoadCommandFrameFormat* rdata)
{
	uint16_t sumcheck;
	char str[10] = {0};

	if (rdata == NULL) {
		return -1;
	}

	rdata->head = LS_START_OF_TX;
	rdata->rw = LS_LOAD_READ;
	rdata->area = LS_LOAD_MEM_ADDR_WORD_D;

	sprintf(str, "%06X", address * 2);
	rdata->addr[0] = str[4];
	rdata->addr[1] = str[5];
	rdata->addr[2] = str[2];
	rdata->addr[3] = str[3];
	rdata->addr[4] = str[0];
	rdata->addr[5] = str[1];
	sprintf(str, "%02X", length * 2);
	memcpy(rdata->num, str, 2);
	sumcheck = CalSumCheckData((uint8_t *)(&(rdata->rw)), sizeof(LsLoadCommandFrameFormat) - 4);
	rdata->sum[0] = sumcheck & 0xff;
	rdata->sum[1] = (sumcheck >> 8) & 0xff;
	rdata->end = LS_END_OF_TX;

	return 0;
}

void LSLoadReadSingleDataRegister(uint32_t address, uint16_t frnum)
{
	LSLoadPackReadWordDataRegisterFrame(address, 1, &lsloadsdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&lsloadsdatabuff, sizeof(LsLoadCommandFrameFormat));
	g_fxplccount = frnum;
    vTaskDelay(30);
}

int LSLoadGetSerialWordDataFromFxPlc(int *length)
{
	uint8_t curData = 0;
	uint8_t dataArry[4] = {0};
	uint16_t rdata;
	int ret, len = 0, count = 0;
	static const char *TAG = "LS_LOAD_GET_SERIAL_DATA";

	while ((curData != LS_START_OF_ACK) && (curData != LS_START_OF_NACK)) {
		ret = UART_ReadBufferBytes(&curData, 1);
		if (ret != 0) {
			return -1;
		}
	}

	if (curData == LS_START_OF_NACK) {
		lsloadnackrdatabuff.head = LS_START_OF_NACK;
		ret = UART_ReadBufferBytes(lsloadnackrdatabuff.code, sizeof(lsloadnackrdatabuff.code));
		if (ret != 0) {
			return -1;
		}
		*length = 0;
		ESP_LOGE(TAG, "LS_PLC errorcode:%s", lsloadnackrdatabuff.code);
	} else {
		lsloadackrdatabuff.head = LS_START_OF_ACK;
		ret = UART_ReadBufferBytes(&lsloadackrdatabuff.rw, 1);
		if (ret != 0) {
			return -1;
		}
		if (lsloadackrdatabuff.rw == LS_LOAD_READ) {
			do {
				ret = UART_ReadBufferBytes(&curData, 1);
				if (ret != 0) {
					return -1;
				}
				dataArry[count] = curData;
				count++;
				if (count % 4 == 0) {
					count = 0;
					len++;
					rdata = LSCalSerialReadDataRegister(dataArry);
					// ESP_LOGI(TAG, "Read bytes length: '%d'", rdata);
					ret = FXPLC_WriteBufferBytes(&rdata, sizeof(rdata));
					if (ret != 0) {
						return -1;
					}
				}
			} while (curData != LS_END_OF_TR);

			lsloadackrdatabuff.sum[0] = dataArry[0];
			lsloadackrdatabuff.sum[1] = dataArry[1];
			*length = len;
		}
		
	}

	return 0;
}
