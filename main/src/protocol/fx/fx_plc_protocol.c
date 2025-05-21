#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <ctype.h>  
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "fx_plc_protocol.h"
#include "iot_common.h"

FxPlcReadFrameFormat rdatabuff = {0};
FxPlcSerialAskReadFrameFormat srdatabuff = {0};
FxPlcSerialAskReadBackFrameFormat srbdatabuff = {0};
FxPlcSerialAskWriteFrameFormat swfdatabuff = {0};
FxPlcSerialAnsNackFrameFormat srbedatabuff = {0};
FxPlcSerialAnsAckFrameFormat mackdatabuff = {0};
FxPlcSerialAnsAckFrameFormat mnackdatabuff = {0};
FxPlcInternetAskReadFrameFormat netsdatabuff = {0};
uint32_t netsrdatabuffaddr = 0;
uint32_t netsrdatalen = 0;

static int netdatarecvflag = 0;

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

static int PackSerialReadDataRegisterFrame(uint8_t type, uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address,  
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
	rdata->address[0] = type;
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

static int PackSerialWriteRealDataRegisterFrame(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address,  
	float wdata, FxPlcSerialAskWriteFrameFormat* rdata)
{
	uint32_t addr;
	uint16_t plcn, pcn, len, sumcheck;
	char rfdata[9] = {0};
	int idata;
	const char *cmd = PLC_WW;

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

	len = CalLength(2);
	rdata->length[0] = len & 0xff;
	rdata->length[1] = (len >> 8) & 0xff;

	
	idata = *((int *)&wdata);
	sprintf(rfdata, "%X", idata);
	memcpy(rdata->data, &rfdata[4], 4);
	memcpy(&(rdata->data[4]), rfdata, 4);

	sumcheck = CalSumCheckData((uint8_t *)(&(rdata->plcnum[0])), PLC_SERIAL_WREITE_SINGLE_REAL_DATA_FRAME_CAL_LEAGTH);
	rdata->sum[0] = sumcheck & 0xff;
	rdata->sum[1] = (sumcheck >> 8) & 0xff;
	
	return 0;
}

static int PackNetReadDataRegisterFrame(uint8_t netnum, uint8_t plcnum, uint16_t ionum, uint8_t mudulenum, uint32_t address,  
	uint16_t length, FxPlcInternetAskReadFrameFormat* rdata)
{
	if (rdata == NULL) {
		return -1;
	}

	rdata->sechead[0] = FX_PLC_NET_REQUIRE_START_FIRST;
	rdata->sechead[1] = FX_PLC_NET_REQUIRE_START_SECOND;
	rdata->netnum = netnum;
	rdata->plcnum = plcnum;
	rdata->ionum[0] = ionum & 0xff;
	rdata->ionum[1] = (ionum >> 8) & 0xff;
	rdata->modulenum = mudulenum;
	rdata->length[0] = FX_PLC_NET_FIRST_LEN_MDATA_OB_READ & 0xff;
	rdata->length[1] = (FX_PLC_NET_FIRST_LEN_MDATA_OB_READ >> 8) & 0xff;
	rdata->watchtimer[0] = FX_PLC_NET_WATCHTIMER & 0xff;
	rdata->watchtimer[1] = (FX_PLC_NET_WATCHTIMER >> 8) & 0xff;
	rdata->instruction[0] = FX_PLC_NET_FIRST_INS_MDATA_OB_READ & 0xff;
    rdata->instruction[1] = (FX_PLC_NET_FIRST_INS_MDATA_OB_READ >> 8) & 0xff;
	rdata->secinstruction[0] = FX_PLC_NET_SECOND_INSTRUCTION_WORD & 0xff;
	rdata->secinstruction[1] = (FX_PLC_NET_SECOND_INSTRUCTION_WORD >> 8) & 0xff;
    
	rdata->startregaddr[0] = address & 0xff;
	rdata->startregaddr[1] = (address >> 8) & 0xff;
	rdata->startregaddr[2] = (address >> 16) & 0xff;

	rdata->regcode = FX_PLC_NET_REGISTER_D;
	rdata->regnum[0] = length & 0xff;
	rdata->regnum[1] = (length >> 8) & 0xff;

	return 0;
}

int GetDataFromFxPlc(void)
{
	uint8_t curData = 0;
	int ret;
	static const char *TAG = "GET_DATA";

	while (curData != PLC_STX) {
		ret = UART_ReadBufferBytes(&curData, 1);
		if (ret != 0) {
			return -1;
		}
	}

	ret = UART_ReadBufferBytes(&curData, 1);
	if (ret != 0) {
		return -1;
	}
	while (curData != PLC_ETX) {
		ret = FXPLC_WriteBufferBytes(&curData, sizeof(curData));
		if (ret != 0) {
			return -1;
		}
		ret = UART_ReadBufferBytes(&curData, 1);
		if (ret != 0) {
			return -1;
		}
	}

	return 0;
}

int GetSerialDataFromFxPlc(void)
{
	uint8_t curData = 0;
	int ret, i;
	static const char *TAG = "GET_SERIAL_DATA";
	uint8_t dataArry[4] = {0};

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

		// ret = UART_ReadBufferBytes(&curData, 1);
		// if (ret != 0) {
		// 	return -1;
		// }
		// while (curData != PLC_ETX) {
		// 	ret = FXPLC_WriteBufferBytes(&curData, sizeof(curData));
		// 	if (ret != 0) {
		// 		return -1;
		// 	}
		// 	ret = UART_ReadBufferBytes(&curData, 1);
		// 	if (ret != 0) {
		// 		// return -1;
		// 	}
		// }
		if ((g_fxplcdataformat == 1) || (g_fxplcdataformat == 5)) {
			ret = UART_ReadBufferBytes(dataArry, 4);
			if (ret != 0) {
				return -1;
			}

			ret = FXPLC_WriteBufferBytes(dataArry, sizeof(dataArry));
			if (ret != 0) {
				ESP_LOGE(TAG, "fx buffer error: %d", ret);
				return -1;
			}
		} else if ((g_fxplcdataformat == 2) || (g_fxplcdataformat == 4)) {
			for (i = 0; i < 2; i++) {
				ret = UART_ReadBufferBytes(dataArry, 4);
				if (ret != 0) {
					return -1;
				}

				ret = FXPLC_WriteBufferBytes(dataArry, sizeof(dataArry));
				if (ret != 0) {
					ESP_LOGE(TAG, "fx buffer error: %d", ret);
					return -1;
				}
			}
		}

		ret = UART_ReadBufferBytes(dataArry, 1);
		if (ret != 0 || dataArry[0] != PLC_ETX) {
			return -1;
		}
	}

	return 0;
}

void FX_NetDataRecvNotice(uint32_t dataaddr, uint32_t len, uint16_t frnum, uint8_t datatype)
{
    netdatarecvflag = 1;
	netsrdatabuffaddr = dataaddr;
	netsrdatalen = len;
	g_fxplccount = frnum;
	g_fxplcdataformat = datatype;
}

int GetNetDataFromFxPlc(void)
{
	uint16_t datalen;
	int16_t tdata;
	uint8_t* data = NULL;
	static const char *TAG = "GET_NET_DATA";
	int i, ret;

	if (netdatarecvflag != 1 || netsrdatabuffaddr == NULL || netsrdatalen == 0) {
		return -1;
	}

	data = (uint8_t *)netsrdatabuffaddr;

	if (netsrdatalen < FX_PLC_NET_RESPONSE_FRAME_MIN_SIZE) {
		return -1;
	}

	if ((data[0] != FX_PLC_NET_RESPONSE_START_FIRST) && (data[1] != FX_PLC_NET_RESPONSE_START_SECOND)) {
		return -1;
	}

	datalen = (data[8] << 8 | data[7]) - 2;
	if (netsrdatalen != sizeof(FxPlcInternetAskReadBackFrameFormat) - 4 + datalen) {
		return -1;
	}
	
	for (i = 0; i < datalen / 2; i++) {
		tdata = (data[sizeof(FxPlcInternetAskReadBackFrameFormat) - 4 + i + 1] << 8) | \
		data[sizeof(FxPlcInternetAskReadBackFrameFormat) - 4 + i];
		ret = FXPLC_WriteBufferBytes(&tdata, sizeof(tdata));
		if (ret != 0) {
			ESP_LOGE(TAG, "fx buffer error: %d", ret);
			return -1;
		}
	}

	netdatarecvflag = 0;

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

void ReadSingleDataRegister(uint16_t address, uint16_t frnum)   // RS232
{
	PackReadDataRegisterFrame(address, 2, &rdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&rdatabuff, sizeof(FxPlcReadFrameFormat));
	g_fxplccount = frnum;
	g_fxplcdataformat = 1;
    vTaskDelay(20);
}

void SerialReadSingleDataRegister(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address, uint16_t frnum)    // RS485
{
	PackSerialReadDataRegisterFrame(PLC_TYPE_D, plcnum, pcnum, timeout, address, 1, &srdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&srdatabuff, PLC_SERIAL_READ_DATA_FRAME_LEAGTH);
	g_fxplccount = frnum;
	g_fxplcdataformat = 1;
    vTaskDelay(20);
}

void SerialReadDoubleDataRegister(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address, uint16_t frnum)    // RS485
{
	PackSerialReadDataRegisterFrame(PLC_TYPE_D, plcnum, pcnum, timeout, address, 2, &srdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&srdatabuff, PLC_SERIAL_READ_DATA_FRAME_LEAGTH);
	g_fxplccount = frnum;
	g_fxplcdataformat = 4;
    vTaskDelay(20);
}

void SerialReadSingleFloatDataRegister(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address, uint16_t frnum)    // RS485
{
	PackSerialReadDataRegisterFrame(PLC_TYPE_D, plcnum, pcnum, timeout, address, 2, &srdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&srdatabuff, PLC_SERIAL_READ_DATA_FRAME_LEAGTH);
	g_fxplccount = frnum;
	g_fxplcdataformat = 2;
    vTaskDelay(20);
}

void SerialReadSingleYDataRegister(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address, uint16_t frnum)    // RS485
{
	PackSerialReadDataRegisterFrame(PLC_TYPE_Y, plcnum, pcnum, timeout, address, 1, &srdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&srdatabuff, PLC_SERIAL_READ_DATA_FRAME_LEAGTH);
	g_fxplccount = frnum;
	g_fxplcdataformat = 5;
    vTaskDelay(20);
}

void SerialWriteSingleFloatDataRegister(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address, float wdata)    // RS485
{
	PackSerialWriteRealDataRegisterFrame(plcnum, pcnum, timeout, address, wdata, &swfdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&swfdatabuff, PLC_SERIAL_WREITE_SINGLE_REAL_DATA_FRAME_LEAGTH);
    vTaskDelay(20);
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

void FX_PackNetReadSingleDataRegister(uint32_t address, uint16_t frnum)
{
	PackNetReadDataRegisterFrame(FX_PLC_NET_NET_NUMBER, FX_PLC_NET_PLC_NUMBER, FX_PLC_NET_IO_NUMBER, \
		FX_PLC_NET_MODULE_NUMBER, address, 1, &netsdatabuff);
}
