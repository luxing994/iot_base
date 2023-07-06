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

HostLinkCommandFrameFormat hlrdatabuff = {0};

static uint8_t CalFCS(uint8_t* pbuff, uint16_t len)
{
	uint8_t ret = 0;

	while (len--) {
		ret ^= *pbuff++;
	}

	return ret;
}

static int HLPackReadDataRegisterFrame(uint32_t address, uint16_t length, HostLinkCommandFrameFormat* rdata)
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


	sprintf(str, "%02x", CalFCS(&hlrdatabuff, sizeof(HostLinkCommandFrameFormat) - 4));
	memcpy(rdata->fcs, str, 2);
	memcpy(rdata->end, HOSTLINK_END, 2);
	return 0;
}

void HLReadSingleDataRegister(uint32_t address, uint16_t length)
{
	HLPackReadDataRegisterFrame(address, length, &hlrdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&hlrdatabuff, sizeof(HostLinkCommandFrameFormat));
}

// static int PackInputRelayFrame(uint16_t length, FxPlcReadFrameFormat* rdata)
// {
// 	const uint8_t iraddr[4] = { 0x30, 0x30, 0x38, 0x30 };    // address: 0x80 = 0 + PLC_X_GROUP_BASE_ADDRESS
// 	uint16_t len, sumcheck;

// 	if (length > FX_PLC_MAX_X_LEN || rdata == NULL) {
// 		return -1;
// 	}

// 	rdata->stx = PLC_STX;
// 	rdata->etx = PLC_ETX;
// 	rdata->cmd = PLC_READ;

// 	rdata->address[0] = iraddr[0];
// 	rdata->address[1] = iraddr[1];
// 	rdata->address[0] = iraddr[2];
// 	rdata->address[1] = iraddr[3];

// 	len = CalLength(length);
// 	rdata->length[0] = len & 0xff;
// 	rdata->length[1] = (len >> 8) & 0xff;

// 	sumcheck = CalSumCheckData((uint8_t *)(&(rdata->cmd)), PLC_READ_DATA_FRAME_CAL_LEAGTH);
// 	rdata->sum[0] = sumcheck & 0xff;
// 	rdata->sum[1] = (sumcheck >> 8) & 0xff;

// 	return 0;
// }

// static int PackOutputRelayFrame(uint16_t length, FxPlcReadFrameFormat* rdata)
// {
// 	const uint8_t iraddr[4] = { 0x30, 0x30, 0x41, 0x30 };    // address: 0x80 = 0 + PLC_Y_GROUP_BASE_ADDRESS
// 	uint16_t len, sumcheck;

// 	if (length > FX_PLC_MAX_Y_LEN || rdata == NULL) {
// 		return -1;
// 	}

// 	rdata->stx = PLC_STX;
// 	rdata->etx = PLC_ETX;
// 	rdata->cmd = PLC_READ;

// 	rdata->address[0] = iraddr[0];
// 	rdata->address[1] = iraddr[1];
// 	rdata->address[0] = iraddr[2];
// 	rdata->address[1] = iraddr[3];

// 	len = CalLength(length);
// 	rdata->length[0] = len & 0xff;
// 	rdata->length[1] = (len >> 8) & 0xff;

// 	sumcheck = CalSumCheckData((uint8_t *)(&(rdata->cmd)), PLC_READ_DATA_FRAME_CAL_LEAGTH);
// 	rdata->sum[0] = sumcheck & 0xff;
// 	rdata->sum[1] = (sumcheck >> 8) & 0xff;

// 	return 0;
// }

// static int PackSerialReadDataRegisterFrame(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address,  
// 	uint16_t length, FxPlcSerialAskReadFrameFormat* rdata)
// {
// 	uint32_t addr;
// 	uint16_t plcn, pcn, len, sumcheck;
// 	const char *cmd = PLC_WR;

// 	if (rdata == NULL) {
// 		return -1;
// 	}

// 	rdata->enq = PLC_ENQ;

// 	plcn = CalLength(plcnum);
// 	rdata->plcnum[0] = plcn & 0xff;
// 	rdata->plcnum[1] = (plcn >> 8) & 0xff;

// 	pcn = CalLength(pcnum);
// 	rdata->pcnum[0] = pcn & 0xff;
// 	rdata->pcnum[1] = (pcn >> 8) & 0xff;

// 	rdata->cmd[0] = cmd[0];
// 	rdata->cmd[1] = cmd[1];

// 	rdata->timeout = CalTimeout(timeout);

// 	addr = CalSerialDataRegisterAddress(address);
// 	rdata->address[0] = 'D';
// 	rdata->address[1] = addr & 0xff;
// 	rdata->address[2] = (addr >> 8) & 0xff;
// 	rdata->address[3] = (addr >> 16) & 0xff;
// 	rdata->address[4] = (addr >> 24) & 0xff;

// 	len = CalLength(length);
// 	rdata->length[0] = len & 0xff;
// 	rdata->length[1] = (len >> 8) & 0xff;

// 	sumcheck = CalSumCheckData((uint8_t *)(&(rdata->plcnum[0])), PLC_SERIAL_READ_DATA_FRAME_CAL_LEAGTH);
// 	rdata->sum[0] = sumcheck & 0xff;
// 	rdata->sum[1] = (sumcheck >> 8) & 0xff;
	
// 	return 0;
// }

// int GetDataFromFxPlc(int *length)
// {
// 	uint8_t curData = 0;
// 	uint8_t dataArry[4] = {0};
// 	uint16_t rdata;
// 	int ret, len = 0, count = 0;
// 	static const char *TAG = "GET_DATA";

// 	while (curData != PLC_STX) {
// 		ret = UART_ReadBufferBytes(&curData, 1);
// 		if (ret != 0) {
// 			return -1;
// 		}
// 	}

// 	do {
// 		ret = UART_ReadBufferBytes(&curData, 1);
// 		if (ret != 0) {
// 			return -1;
// 		}

// 		if (count < 4) {
// 			dataArry[count] = curData;
// 		}
// 		count++;
// 		if (count % 4 == 0) {
// 			count = 0;
// 			len++;
// 			rdata = CalReadDataRegister(dataArry);
// 			ret = FXPLC_WriteBufferBytes(&rdata, sizeof(rdata));
// 			if (ret != 0) {
// 				return -1;
// 			}
// 		}
// 	} while (curData != PLC_ETX);

// 	*length = len;
// 	return 0;
// }

// int GetSerialDataFromFxPlc(int *length)
// {
// 	uint8_t curData = 0;
// 	uint8_t dataArry[4] = {0};
// 	uint16_t rdata;
// 	int ret, len = 0, count = 0;
// 	static const char *TAG = "GET_SERIAL_DATA";

// 	while ((curData != PLC_STX) && (curData != PLC_NAK)) {
// 		ret = UART_ReadBufferBytes(&curData, 1);
// 		if (ret != 0) {
// 			return -1;
// 		}
// 	}

// 	if (curData == PLC_NAK) {
// 		srbedatabuff.nak = PLC_NAK;
// 		ret = UART_ReadBufferBytes(srbedatabuff.plcnum, sizeof(srbedatabuff.plcnum));
// 		if (ret != 0) {
// 			return -1;
// 		}
// 		ret = UART_ReadBufferBytes(srbedatabuff.pcnum, sizeof(srbedatabuff.pcnum));
// 		if (ret != 0) {
// 			return -1;
// 		}
// 		ret = UART_ReadBufferBytes(srbedatabuff.errorcode, sizeof(srbedatabuff.errorcode));
// 		if (ret != 0) {
// 			return -1;
// 		}
// 		*length = 0;
// 		ESP_LOGE(TAG, "PLC:%c%c, PC:%c%c, errorcode:%c%c", srbedatabuff.plcnum[0], srbedatabuff.plcnum[1], 
// 			srbedatabuff.pcnum[0], srbedatabuff.pcnum[1], srbedatabuff.errorcode[0], srbedatabuff.errorcode[1]);
// 	} else {
// 		srbdatabuff.stx = PLC_STX;
// 		ret = UART_ReadBufferBytes(srbdatabuff.plcnum, sizeof(srbdatabuff.plcnum));
// 		if (ret != 0) {
// 			return -1;
// 		}
// 		ret = UART_ReadBufferBytes(srbdatabuff.pcnum, sizeof(srbdatabuff.pcnum));
// 		if (ret != 0) {
// 			return -1;
// 		}
// 		do {
// 			ret = UART_ReadBufferBytes(&curData, 1);
// 			if (ret != 0) {
// 				return -1;
// 			}

// 			if (count < 4) {
// 				dataArry[count] = curData;
// 			}
// 			count++;
// 			if (count % 4 == 0) {
// 				count = 0;
// 				len++;
// 				rdata = CalSerialReadDataRegister(dataArry);
// 				ret = FXPLC_WriteBufferBytes(&rdata, sizeof(rdata));
// 				if (ret != 0) {
// 					return -1;
// 				}
// 			}
// 		} while (curData != PLC_ETX);
// 		*length = len;
// 	}

// 	return 0;
// }

// void SendAckToPlc(void)
// {
// 	mackdatabuff.ack = PLC_ACK;
// 	mackdatabuff.plcnum[0] = srbdatabuff.plcnum[0];
// 	mackdatabuff.plcnum[1] = srbdatabuff.plcnum[1];
// 	mackdatabuff.pcnum[0] = srbdatabuff.pcnum[0];
// 	mackdatabuff.pcnum[1] = srbdatabuff.pcnum[1];

// 	uart_write_bytes(UART_NUM_1, (uint8_t *)&mackdatabuff, sizeof(FxPlcSerialAnsAckFrameFormat));
// }

// void SendNackToPlc(void)
// {
// 	mackdatabuff.ack = PLC_NAK;
// 	mackdatabuff.plcnum[0] = srbedatabuff.plcnum[0];
// 	mackdatabuff.plcnum[1] = srbedatabuff.plcnum[1];
// 	mackdatabuff.pcnum[0] = srbedatabuff.pcnum[0];
// 	mackdatabuff.pcnum[1] = srbedatabuff.pcnum[1];

// 	uart_write_bytes(UART_NUM_1, (uint8_t *)&mackdatabuff, sizeof(FxPlcSerialAnsAckFrameFormat));
// }



// void SerialReadSingleDataRegister(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address)
// {
// 	PackSerialReadDataRegisterFrame(plcnum, pcnum, timeout, address, 1, &srdatabuff);
// 	uart_write_bytes(UART_NUM_1, (uint8_t *)&srdatabuff, PLC_SERIAL_READ_DATA_FRAME_LEAGTH);
// }

// void ReadMulDataRegister(uint16_t startaddr, uint16_t length)
// {
// 	int i;

// 	for (i = 0; i < length; i++) {
// 		ReadSingleDataRegister(startaddr + i);
// 		vTaskDelay(10);
// 	}
// }

// int ReadInputRelayData()
// {
// 	int ret;

// 	ret = PackInputRelayFrame(FX_PLC_MAX_X_LEN, &rdatabuff);
// 	if (ret != 0) {
// 		return ret;
// 	}

// 	uart_write_bytes(UART_NUM_1, (uint8_t *)&rdatabuff, PLC_READ_DATA_FRAME_LEAGTH);
// 	return 0;
// }

// int ReadOutputRelayData()
// {
// 	int ret;

// 	ret = PackOutputRelayFrame(FX_PLC_MAX_Y_LEN, &rdatabuff);
// 	if (ret != 0) {
// 		return ret;
// 	}

// 	uart_write_bytes(UART_NUM_1, (uint8_t *)&rdatabuff, sizeof(FxPlcReadFrameFormat));
// 	return 0;
// }
