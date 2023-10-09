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

static int HLPackReadWordDataRegisterFrame(uint32_t address, uint16_t length, HostLinkCommandFrameFormat* rdata)
{
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

	sprintf(str, "%04X", READIO);
	memcpy(rdata->finscomdata.code, str, 4);

	sprintf(str, "%02X", DMWORD);
	memcpy(rdata->finscomdata.mem, str, 2);

	address *= 256;
	sprintf(str, "%06X", address);
	memcpy(rdata->finscomdata.text_startaddr, str, strlen(str));

	sprintf(str, "%04X", length);
	memcpy(rdata->finscomdata.text_num, str, 4);


	sprintf(str, "%02X", CalFCS(&hlsdatabuff, sizeof(HostLinkCommandFrameFormat) - 4));
	memcpy(rdata->fcs, str, 2);
	memcpy(rdata->end, HOSTLINK_END, 2);
	return 0;
}

void HLReadSingleDataRegister(uint32_t address, uint16_t frnum)
{
	HLPackReadWordDataRegisterFrame(address, 1, &hlsdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&hlsdatabuff, sizeof(HostLinkCommandFrameFormat));
	g_fxplccount = frnum;
	g_fxplcdataformat = 1;
    vTaskDelay(30);
}

void HLReadFloatDataRegister(uint32_t address, uint16_t frnum)
{
	HLPackReadWordDataRegisterFrame(address, 2, &hlsdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&hlsdatabuff, sizeof(HostLinkCommandFrameFormat));
	g_fxplccount = frnum;
	g_fxplcdataformat = 2;
    vTaskDelay(30);
}

void HLReadBCDDataRegister(uint32_t address, uint16_t frnum)
{
	HLPackReadWordDataRegisterFrame(address, 1, &hlsdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&hlsdatabuff, sizeof(HostLinkCommandFrameFormat));
	g_fxplccount = frnum;
	g_fxplcdataformat = 3;
    vTaskDelay(30);
}

int GetSerialWordDataFromHlPlc(void)
{
	uint8_t curData = 0;
	uint8_t dataArry[4] = {0};
	int ret, count = 0;
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
	
	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.head, 2);
	if (ret != 0 || strcmp((char *)hlrdatabuff.finscomdata.head, "FA")) {
		ESP_LOGE(TAG, "fins head error: %s", (char *)hlrdatabuff.finscomdata.head);
		return -1;
	}

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.resdata, 2);
	if (ret != 0 || strcmp((char *)hlrdatabuff.finscomdata.resdata, "00")) {
		ESP_LOGE(TAG, "fins resdata error: %s", (char *)hlrdatabuff.finscomdata.resdata);
		return -1;
	}

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.icf, 2);
	if (ret != 0 || strcmp((char *)hlrdatabuff.finscomdata.icf, "40")) {
		ESP_LOGE(TAG, "fins icf error: %s", (char *)hlrdatabuff.finscomdata.head);
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

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.code, 4);
	if (ret != 0 || strcmp((char *)hlrdatabuff.finscomdata.code, "0101") != 0) {
		ESP_LOGE(TAG, "code error: %s", (char *)hlrdatabuff.finscomdata.code);
		return -1;
	}

	ret = UART_ReadBufferBytes(hlrdatabuff.finscomdata.respcode, 4);
	if (ret != 0 || strcmp((char *)hlrdatabuff.finscomdata.respcode, "0000") != 0) {
		ESP_LOGE(TAG, "respcode error: %s", (char *)hlrdatabuff.finscomdata.code);
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
			ret = FXPLC_WriteBufferBytes(dataArry, sizeof(dataArry));
			if (ret != 0) {
				ESP_LOGE(TAG, "fx buffer error: %d", ret);
				return -1;
			}
		}
	} while (curData != '*');

	hlrdatabuff.fcs[0] = dataArry[0];
	hlrdatabuff.fcs[1] = dataArry[1];

	return 0;
}
