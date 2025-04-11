#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <ctype.h>  
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "ppi_plc_protocol.h"
#include "iot_common.h"

uint8_t g_confirmflag = 0;
uint8_t ppidataArry[PPI_PLC_READ_DATA_MAX_LENGTH] = {0};
PpiPlcConfirmCommandFrameFormat ppiccdatabuff = {0};
PpiPlcReadCommandFrameFormat ppircdatabuff = {0};
PpiPlcRespondCommandFrameFormat ppirescdatabuff = {0};

static int PPIPackDConfirmCommandDataFrame(PpiPlcConfirmCommandFrameFormat* rdata)
{
	if (rdata == NULL) {
		return -1;
	}

	rdata->start = PPI_PLC_CON_SD;
	rdata->directaddr = PPI_PLC_DA;
	rdata->sourceaddr = PPI_PLC_SA;
    rdata->funccode = PPI_PLC_FC_CONFIRM_COM;
    rdata->checkcode = CalSumCheckDataLow((uint8_t *)(&(rdata->directaddr)), PPI_PLC_CONFIRM_COM_LEN);
    rdata->end = PPI_PLC_EC;

	return 0;
}

static int PPIPackReadByteDataRegisterFrame(uint32_t address, uint8_t length, PpiPlcReadCommandFrameFormat* rdata)
{
	if (rdata == NULL) {
		return -1;
	}

	rdata->fstart = PPI_PLC_SD;
	rdata->length = PPI_PLC_READ_COM_LEN;
	rdata->rlength = PPI_PLC_READ_COM_LEN;
    rdata->sstart = PPI_PLC_SD;
    rdata->directaddr = PPI_PLC_DA;
    rdata->sourceaddr = PPI_PLC_SA;
    rdata->funccode = PPI_PLC_FC_READ_COM;
    rdata->identify = PPI_PLC_IP;
    rdata->remotecontrol = PPI_PLC_RC;
    rdata->identify1 = PPI_PLC_RI;
    rdata->identify2 = PPI_PLC_RI;
    rdata->protocoldata = PPI_PLC_PD;
    rdata->unitpara = PPI_PLC_UP;
    rdata->paralength1 = PPI_PLC_PL1;
    rdata->paralength2 = PPI_PLC_PL2;
    rdata->datalength1 = PPI_PLC_DL1;
    rdata->datalength2 = PPI_PLC_DL2;
    rdata->mode = PPI_PLC_MOD_READ;
    rdata->valueaddrnum = PPI_PLC_AN;
    rdata->pend1 = PPI_PLC_PEND1;
    rdata->pend2 = PPI_PLC_PEND2;
    rdata->defid = PPI_PLC_DI;
    rdata->dataunit = PPI_PLC_DU_BYTE;
    rdata->pend3 = PPI_PLC_PEND3;
    rdata->datalen = length;
    rdata->pend4 = PPI_PLC_PEND4;
    rdata->regtype1 = PPI_PLC_RT1_V;
    rdata->regtype2 = PPI_PLC_RT2_V;

    rdata->dataaddr[2] = (address * 8) & 0xff;
    rdata->dataaddr[1] = ((address * 8) >> 8) & 0xff;
    rdata->dataaddr[0] = ((address * 8) >> 16) & 0xff;
    rdata->checkcode = CalSumCheckDataLow((uint8_t *)(&(rdata->directaddr)), PPI_PLC_READ_COM_LEN);
    rdata->end = PPI_PLC_EC;

	return 0;
}

void PPIReadByteDataRegister(uint32_t address, uint8_t length, uint16_t frnum)
{
	uint8_t rdata;

    PPIPackReadByteDataRegisterFrame(address, length, &ppircdatabuff);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ppircdatabuff, sizeof(PpiPlcReadCommandFrameFormat));
	g_fxplccount = frnum;
	g_fxplcdataformat = 0;  // byte
    vTaskDelay(10);
    if (g_confirmflag == 1) {
        PPIPackDConfirmCommandDataFrame(&ppiccdatabuff);
        uart_write_bytes(UART_NUM_1, (uint8_t *)&ppiccdatabuff, sizeof(PpiPlcConfirmCommandFrameFormat));
        g_confirmflag = 0;
    }
    vTaskDelay(10);
}

int GetSerialWordDataFromPpiPlc(void)
{
	uint8_t curData = 0;
    int datalen = 0;
	int ret = 0, i;
	static const char *TAG = "GET_SERIAL_PPIDATA";

    ret = UART_ReadBufferBytes(&curData, 1);
    if (ret != 0) {
        return -1;
    }

    if (curData == PPI_PLC_CONFIRM_CODE1) {
        g_confirmflag = 1;
    }

	while (curData != PPI_PLC_SD) {
		ret = UART_ReadBufferBytes(&curData, 1);
		if (ret != 0) {
			return -1;
		}
	}

	ppirescdatabuff.fstart = PPI_PLC_SD;
	ret = UART_ReadBufferBytes(&(ppirescdatabuff.length), sizeof(ppirescdatabuff.length));
	if (ret != 0) {
		return -1;
	}
	
	ret = UART_ReadBufferBytes(&(ppirescdatabuff.rlength), sizeof(ppirescdatabuff.rlength));
	if (ret != 0) {
		return -1;
	}

    datalen = ppirescdatabuff.rlength - PPI_PLC_RESPOND_DA_LENGTH;

	ret = UART_ReadBufferBytes(&(ppirescdatabuff.sstart), sizeof(ppirescdatabuff.sstart));
	if ((ret != 0) || (ppirescdatabuff.sstart != PPI_PLC_SD)) {
		return -1;
	}

	ret = UART_ReadBufferBytes(&(ppirescdatabuff.directaddr), PPI_PLC_RESPOND_DA_LENGTH);
	if ((ret != 0) || datalen != ((ppirescdatabuff.datalengthbit[0] << 8 | ppirescdatabuff.datalengthbit[1]) / 8)) {
		ESP_LOGE(TAG, "databitlen error: %d != %d", datalen, (ppirescdatabuff.datalengthbit[0] \
            << 8 | ppirescdatabuff.datalengthbit[1]) / 8);
        return -1;
	}
    
    if (datalen > PPI_PLC_READ_DATA_MAX_LENGTH) {
        return -1;
    } 
    
    ret = UART_ReadBufferBytes(ppidataArry, datalen);
    if (ret != 0) {
        return -1;
    }

    ppirescdatabuff.data = ppidataArry;

    ret = FXPLC_WriteBufferBytes(ppidataArry, datalen);
    if (ret != 0) {
        ESP_LOGE(TAG, "fx buffer error: %d", ret);
        return -1;
    }

    ret = UART_ReadBufferBytes(&ppirescdatabuff.check, 2);
    if (ret != 0 || ppirescdatabuff.end != PPI_PLC_EC) {
        return -1;
    }

	return 0;
}