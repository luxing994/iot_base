#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <ctype.h>  
#include <string.h>
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "hl_plc_protocol.h"
#include "iot_common.h"

HostLinkCommandFrameFormat hlsdatabuff = {0};
HostLinkResponseFrameFormat hlrdatabuff = {0};

static uint8_t CalFCS(uint8_t* pbuff, uint16_t len)
{
	uint8_t ret = 0;

	while (len--) {
		ret ^= *pbuff++;
	}

	return ret;
}

static int HLPackReadWordDataRegisterFrame(uint32_t address, uint16_t length, HostLinkCommandFrameFormat* rdata)
{
	uint32_t addr;
	uint16_t len, sumcheck;
	char str[10] = {0};

	if (rdata == NULL) {
		return -1;
	}

	rdata->head = HOSTLINK_HEAD;
	memcpy(rdata->plcnum, HOSTLINK_PLC_NUM, 2);
    memcpy(rdata->finscomdata.head, FINS_HEAD, 2);
	rdata->finscomdata.resptime = FINS_TIME;
	memcpy(rdata->finscomdata.icf, FINS_ICF_LOCAL, 2);
	memcpy(rdata->finscomdata.da2, FINS_DA2_CPU, 2);
	memcpy(rdata->finscomdata.sa2, FINS_SA2_CPU, 2);
	memcpy(rdata->finscomdata.sid, FINS_SID, 2);
	memcpy(rdata->finscomdata.sid, FINS_SID, 2);

	sprintf(str, "%04x", READIO);
	memcpy(rdata->finscomdata.code, str, 4);

	sprintf(str, "%02x", DMWORD);
	memcpy(rdata->finscomdata.mem, str, 2);

	sprintf(str, "%06d", address);
	memcpy(rdata->finscomdata.text_startaddr, str, strlen(str));

	sprintf(str, "%04d", length);
	memcpy(rdata->finscomdata.text_num, str, 4);


	sprintf(str, "%02x", CalFCS(&hlsdatabuff, sizeof(HostLinkCommandFrameFormat) - 4));
	memcpy(rdata->fcs, str, 2);
	memcpy(rdata->end, HOSTLINK_END, 2);
	return 0;
}

void HLReadSingleDataRegister(uint32_t address)
{
	HLPackReadWordDataRegisterFrame(address, 1, &hlsdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&hlsdatabuff, sizeof(HostLinkCommandFrameFormat));
}

int GetSerialWordDataFromHlPlc(int *length)
{
	uint8_t curData = 0;
	uint8_t dataArry[4] = {0};
	uint16_t rdata;
	int ret, len = 0, count = 0;
	static const char *TAG = "GET_SERIAL_HLDATA";

	while (curData != HOSTLINK_HEAD) {
		ret = UART_ReadBufferBytes(&curData, 1);
		if (ret != 0) {
			return -1;
		}
	}

	hlrdatabuff.head = HOSTLINK_HEAD;
	ret = UART_ReadBufferBytes(hlrdatabuff.plcnum, sizeof(hlrdatabuff.plcnum));
	if (ret != 0) {
		return -1;
	}
	
	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.head, sizeof(hlrdatabuff.finscomdata.head));
	if (ret != 0) {
		return -1;
	}

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.resdata, sizeof(hlrdatabuff.finscomdata.resdata));
	if (ret != 0) {
		return -1;
	}

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.icf, sizeof(hlrdatabuff.finscomdata.icf));
	if (ret != 0) {
		return -1;
	}

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.da2, sizeof(hlrdatabuff.finscomdata.da2));
	if (ret != 0) {
		return -1;
	}

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.sa2, sizeof(hlrdatabuff.finscomdata.sa2));
	if (ret != 0) {
		return -1;
	}

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.sid, sizeof(hlrdatabuff.finscomdata.sid));
	if (ret != 0) {
		return -1;
	}

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.code, sizeof(hlrdatabuff.finscomdata.code));
	if (ret != 0) {
		return -1;
	}

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.respcode, sizeof(hlrdatabuff.finscomdata.respcode));
	if (ret != 0) {
		return -1;
	}

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
			rdata = CalSerialReadDataRegister(dataArry);
			// ESP_LOGI(TAG, "Read bytes length: '%d'", rdata);
			ret = FXPLC_WriteBufferBytes(&rdata, sizeof(rdata));
			if (ret != 0) {
				return -1;
			}
		}
	} while (curData != '*');

	hlrdatabuff.fcs[0] = dataArry[0];
	hlrdatabuff.fcs[1] = dataArry[1];

	*length = len;

	return 0;
}
