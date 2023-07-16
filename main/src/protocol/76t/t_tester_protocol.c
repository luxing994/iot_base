#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <ctype.h>  
#include <math.h>
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "t_tester_protocol.h"
#include "iot_common.h"

TTesterNoCommandDataFrameFormat ttesterreadstatussbuff = {0};
TTesterOneCommandDataFrameFormat ttesterreadcuritemsbuff = {0};
TTesterOneCommandDataFrameFormat ttesterreadcurgroupbuff = {0};
TTesterTwoCommandDataFrameFormat ttesterreadhisgroupbuff = {0};
TTesterOneCommandDataFrameFormat ttesterselectgroupbuff = {0};

TTesterPressurizationPara ttesterpredata = {0};
TTesterGroundingPara ttestergrodata = {0};
TTesterRinsulationPara ttesterrindata = {0};
TTesterLeakagePara ttesterleadata = {0};
TTesterPowerPara ttesterpowdata = {0};
TTesterStartupPara ttesterstadata = {0};
TTesterOpenshortPara ttesterosdata = {0};
TTesterDcvoltagePara ttesterdcdata = {0};

TTestestGroupResultPara curgrouptestdata = {0};
uint8_t ttestrdatabuffer[256] = {0};
const char *TAG = "TTest_Resolve";

static float TTesterResTranData(uint16_t data)
{
    if ((data & 0x3f) == 0) {
        return (float)0;
    } else {
        return (float)(data & 0x3fff) * pow(10.0, (float)(-(data >> 14)));
    }
   
}

static void TTesterPackSelectGroupFrame(uint16_t group)
{
    ttesterselectgroupbuff.head = T_TESTER_FRAME_HEAD;
    ttesterselectgroupbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttesterselectgroupbuff.address[0] = 0;
    ttesterselectgroupbuff.address[1] = T_TESTER_ADDRESS;
    ttesterselectgroupbuff.length = sizeof(TTesterOneCommandDataFrameFormat) - 1;
    ttesterselectgroupbuff.reserve = 0;
    ttesterselectgroupbuff.maincode[0] = TTESTERSELECTGROUP;
    ttesterselectgroupbuff.maincode[1] = group;
    ttesterselectgroupbuff.end = T_TESTER_FRAME_END;
    ttesterselectgroupbuff.sum = CalSumCheckDataLow(&ttesterselectgroupbuff.head, \
        sizeof(TTesterOneCommandDataFrameFormat) - 1);
}

static void TTesterPackReadStatusFrame(void)
{
    ttesterreadstatussbuff.head = T_TESTER_FRAME_HEAD;
    ttesterreadstatussbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttesterreadstatussbuff.address[0] = 0;
    ttesterreadstatussbuff.address[1] = T_TESTER_ADDRESS;
    ttesterreadstatussbuff.length = sizeof(TTesterNoCommandDataFrameFormat) - 1;
    ttesterreadstatussbuff.reserve = 0;
    ttesterreadstatussbuff.maincode = TTESTERINQUIRYSTATUS;
    ttesterreadstatussbuff.end = T_TESTER_FRAME_END;
    ttesterreadstatussbuff.sum = CalSumCheckDataLow(&ttesterreadstatussbuff.head, \
        sizeof(TTesterNoCommandDataFrameFormat) - 1);
}

static void TTesterPackReadCurrentItemFrame(void)
{
    ttesterreadcuritemsbuff.head = T_TESTER_FRAME_HEAD;
    ttesterreadcuritemsbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttesterreadcuritemsbuff.address[0] = 0;
    ttesterreadcuritemsbuff.address[1] = T_TESTER_ADDRESS;
    ttesterreadcuritemsbuff.length = sizeof(TTesterOneCommandDataFrameFormat) - 1;
    ttesterreadcuritemsbuff.reserve = 0;
    ttesterreadcuritemsbuff.maincode[0] = TTESTERDATAREAD;
    ttesterreadcuritemsbuff.maincode[1] = T_TESTER_READ_CURRENT_TEST_DATA;
    ttesterreadcuritemsbuff.end = T_TESTER_FRAME_END;
    ttesterreadcuritemsbuff.sum = CalSumCheckDataLow(&ttesterreadcuritemsbuff.head, \
        sizeof(TTesterOneCommandDataFrameFormat) - 1);
}

static void TTesterPackReaCurrentGroupFrame(void)
{
    ttesterreadcurgroupbuff.head = T_TESTER_FRAME_HEAD;
    ttesterreadcurgroupbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttesterreadcurgroupbuff.address[0] = 0;
    ttesterreadcurgroupbuff.address[1] = T_TESTER_ADDRESS;
    ttesterreadcurgroupbuff.length = sizeof(TTesterOneCommandDataFrameFormat) - 1;
    ttesterreadcurgroupbuff.reserve = 0;
    ttesterreadcurgroupbuff.maincode[0] = TTESTERDATAREAD;
    ttesterreadcurgroupbuff.maincode[1] = T_TESTER_READ_CURRENT_GROUP_TEST_DATA;
    ttesterreadcurgroupbuff.end = T_TESTER_FRAME_END;
    ttesterreadcurgroupbuff.sum = CalSumCheckDataLow(&ttesterreadcurgroupbuff.head, \
        sizeof(TTesterOneCommandDataFrameFormat) - 1);
}

static void TTesterPackReadHistoryGroupFrame(uint16_t group)
{
    ttesterreadhisgroupbuff.head = T_TESTER_FRAME_HEAD;
    ttesterreadhisgroupbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttesterreadhisgroupbuff.address[0] = 0;
    ttesterreadhisgroupbuff.address[1] = T_TESTER_ADDRESS;
    ttesterreadhisgroupbuff.length = sizeof(TTesterTwoCommandDataFrameFormat) - 1;
    ttesterreadhisgroupbuff.reserve = 0;
    ttesterreadhisgroupbuff.maincode[0] = TTESTERDATAREAD;
    ttesterreadhisgroupbuff.maincode[1] = T_TESTER_READ_HISTORY_GROUP_TEST_DATA;
    ttesterreadhisgroupbuff.maincode[2] = group;
    ttesterreadhisgroupbuff.end = T_TESTER_FRAME_END;
    ttesterreadhisgroupbuff.sum = CalSumCheckDataLow(&ttesterreadhisgroupbuff.head, \
        sizeof(TTesterTwoCommandDataFrameFormat) - 1);
}

static void TTesterResolveCurrentGroupDataResponse(uint8_t num, uint8_t item, uint8_t* rdata, uint16_t len)
{
    switch (item) {
        case TTESTERPRESSURIZATION: {
            ttesterpredata.voltage = TTesterResTranData((rdata[0] << 8) + rdata[1]);
            ttesterpredata.current = TTesterResTranData((rdata[2] << 8) + rdata[3]);
            ttesterpredata.testtime = TTesterResTranData((rdata[4] << 8) + rdata[5]);
            ttesterpredata.uptime = TTesterResTranData((rdata[6] << 8) + rdata[7]);
            ttesterpredata.downtime = TTesterResTranData((rdata[8] << 8) + rdata[9]);
            if (num < 7) {
                memcpy(curgrouptestdata.itemdata[num].para, &ttesterpredata, len * 2);
            } else {
                memcpy(curgrouptestdata.lastitemdata.para, &ttesterpredata, len * 2);
            }
            break;
        }
        case TTESTERGROUNDING: {
            ttestergrodata.current = TTesterResTranData((rdata[0] << 8) + rdata[1]);
            ttestergrodata.resistance = TTesterResTranData((rdata[2] << 8) + rdata[3]);
            ttestergrodata.testtime = TTesterResTranData((rdata[4] << 8) + rdata[5]);
            ttestergrodata.reserve1 = TTesterResTranData((rdata[6] << 8) + rdata[7]);
            ttestergrodata.reserve2 = TTesterResTranData((rdata[8] << 8) + rdata[9]);
            if (num < 7) {
                memcpy(curgrouptestdata.itemdata[num].para, &ttestergrodata, len * 2);
            } else {
                memcpy(curgrouptestdata.lastitemdata.para, &ttestergrodata, len * 2);
            }
            break;
        }
        case TTESTERINSULATION: {
            ttesterrindata.voltage = TTesterResTranData((rdata[0] << 8) + rdata[1]);
            ttesterrindata.resistance = TTesterResTranData((rdata[2] << 8) + rdata[3]);
            ttesterrindata.testtime = TTesterResTranData((rdata[4] << 8) + rdata[5]);
            ttesterrindata.reserve1 = TTesterResTranData((rdata[6] << 8) + rdata[7]);
            ttesterrindata.reserve2 = TTesterResTranData((rdata[8] << 8) + rdata[9]);
            if (num < 7) {
                memcpy(curgrouptestdata.itemdata[num].para, &ttesterrindata, len * 2);
            } else {
                memcpy(curgrouptestdata.lastitemdata.para, &ttesterrindata, len * 2);
            }
            break;
        }
        case TTESTERLEAKAGE: {
            ttesterleadata.voltage = TTesterResTranData((rdata[0] << 8) + rdata[1]);
            ttesterleadata.firecurrent = TTesterResTranData((rdata[2] << 8) + rdata[3]);
            ttesterleadata.groundcurrent = TTesterResTranData((rdata[4] << 8) + rdata[5]);
            ttesterleadata.testtime = TTesterResTranData((rdata[6] << 8) + rdata[7]);
            ttesterleadata.reserve = TTesterResTranData((rdata[8] << 8) + rdata[9]);
            if (num < 7) {
                memcpy(curgrouptestdata.itemdata[num].para, &ttesterleadata, len * 2);
            } else {
                memcpy(curgrouptestdata.lastitemdata.para, &ttesterleadata, len * 2);
            }
            break;
        }
        case TTESTERPOWER: {
            ttesterpowdata.voltage = TTesterResTranData((rdata[0] << 8) + rdata[1]);
            ttesterpowdata.current = TTesterResTranData((rdata[2] << 8) + rdata[3]);
            ttesterpowdata.power = TTesterResTranData((rdata[4] << 8) + rdata[5]);
            ttesterpowdata.powerf = TTesterResTranData((rdata[6] << 8) + rdata[7]);
            ttesterpowdata.testtime = TTesterResTranData((rdata[8] << 8) + rdata[9]);
            if (num < 7) {
                memcpy(curgrouptestdata.itemdata[num].para, &ttesterpowdata, len * 2);
            } else {
                memcpy(curgrouptestdata.lastitemdata.para, &ttesterpowdata, len * 2);
            }
            break;
        }
        case TTESTERSTARTUP: {
            ttesterstadata.voltage = TTesterResTranData((rdata[0] << 8) + rdata[1]);
            ttesterstadata.current = TTesterResTranData((rdata[2] << 8) + rdata[3]);
            ttesterstadata.testtime = TTesterResTranData((rdata[4] << 8) + rdata[5]);
            ttesterstadata.reserve1 = TTesterResTranData((rdata[6] << 8) + rdata[7]);
            ttesterstadata.reserve2 = TTesterResTranData((rdata[8] << 8) + rdata[9]);
            if (num < 7) {
                memcpy(curgrouptestdata.itemdata[num].para, &ttesterstadata, len * 2);
            } else {
                memcpy(curgrouptestdata.lastitemdata.para, &ttesterstadata, len * 2);
            }
            break;
        }
        case TTESTEROPENSHORT: {
            ttesterosdata.voltage = TTesterResTranData((rdata[0] << 8) + rdata[1]);
            ttesterosdata.current = TTesterResTranData((rdata[2] << 8) + rdata[3]);
            ttesterosdata.testtime = TTesterResTranData((rdata[4] << 8) + rdata[5]);
            ttesterosdata.reserve1 = TTesterResTranData((rdata[6] << 8) + rdata[7]);
            ttesterosdata.reserve2 = TTesterResTranData((rdata[8] << 8) + rdata[9]);
            if (num < 7) {
                memcpy(curgrouptestdata.itemdata[num].para, &ttesterosdata, len * 2);
            } else {
                memcpy(curgrouptestdata.lastitemdata.para, &ttesterosdata, len * 2);
            }
            break;
        }
        case TTESTERDCVOLTAGE: {
            ttesterosdata.voltage = TTesterResTranData((rdata[0] << 8) + rdata[1]);
            ttesterosdata.current = TTesterResTranData((rdata[2] << 8) + rdata[3]);
            ttesterosdata.testtime = TTesterResTranData((rdata[4] << 8) + rdata[5]);
            ttesterosdata.reserve1 = TTesterResTranData((rdata[6] << 8) + rdata[7]);
            ttesterosdata.reserve2 = TTesterResTranData((rdata[8] << 8) + rdata[9]);
            if (num < 7) {
                memcpy(curgrouptestdata.itemdata[num].para, &ttesterosdata, len * 2);
            } else {
                memcpy(curgrouptestdata.lastitemdata.para, &ttesterosdata, len * 2);
            }
            break;
        }
        default: {  
            if (num < 7) {
                memset(curgrouptestdata.itemdata[num].para, 0, len * 2);
            } else {
                memset(curgrouptestdata.lastitemdata.para, 0, len * 2);
            }
            break;
        }
    }
}

static int TTesterResolveCurrentGroupResponse(uint8_t* data, uint16_t len)
{
    TTesterTestItemDataFormat rdata = {0};
    int i, ret;
    
    if (len != (sizeof(TTesterTestItemDataFormat) * 8)) {
        return -1;
    }
    
    for (i = 0; i < 8; i++) {
        memcpy(&rdata, &data[i * sizeof(TTesterTestItemDataFormat)], sizeof(TTesterTestItemDataFormat));
        if (i < 7) {
            curgrouptestdata.itemdata[i].group = rdata.group;
            curgrouptestdata.itemdata[i].term = rdata.term;
            curgrouptestdata.itemdata[i].item = rdata.item;
            curgrouptestdata.itemdata[i].termret = rdata.result;
            TTesterResolveCurrentGroupDataResponse(i, curgrouptestdata.itemdata[i].item, rdata.data, \ 
                sizeof(TTesterTestItemDataFormat) - 4);
        } else {
            curgrouptestdata.lastitemdata.group = rdata.group;
            curgrouptestdata.lastitemdata.term = rdata.term;
            curgrouptestdata.lastitemdata.item = rdata.item;
            curgrouptestdata.lastitemdata.termret = (rdata.result) & 0xf;
            if (((rdata.result) & 0xf0) == (T_TESTER_GROUP_TEST_QUALIFY + T_TESTER_ALL_GROUP_TEST_QUALIFY)) {
                curgrouptestdata.lastitemdata.groupret = TTESTERITEMTESTQUALIFY;
                curgrouptestdata.lastitemdata.allret = TTESTERITEMTESTQUALIFY;
            } else if (((rdata.result) & 0xf0) == (T_TESTER_GROUP_TEST_QUALIFY + T_TESTER_ALL_GROUP_TEST_UNQUALIFY)) {
                curgrouptestdata.lastitemdata.groupret = TTESTERITEMTESTQUALIFY;
                curgrouptestdata.lastitemdata.allret = TTESTERITEMTESTUNQUALIFY;
            } else if (((rdata.result) & 0xf0) == (T_TESTER_GROUP_TEST_UNQUALIFY + T_TESTER_ALL_GROUP_TEST_UNQUALIFY)) {
                curgrouptestdata.lastitemdata.groupret = TTESTERITEMTESTUNQUALIFY;
                curgrouptestdata.lastitemdata.allret = TTESTERITEMTESTUNQUALIFY;
            }
            TTesterResolveCurrentGroupDataResponse(i, curgrouptestdata.lastitemdata.item, rdata.data, \ 
                    sizeof(TTesterTestItemDataFormat) - 4);
        }
        
    }

    return 0;
}


static void TTesterResolveReadDataResponse(uint8_t* data, uint16_t len)
{
    // char *TAG = "TTester resolve command resopnse";
    
    switch (data[0]) {
        case T_TESTER_READ_SELFTEST_DATA: {
            // 获取自检数据
            break;
        }
        case T_TESTER_READ_STATUS_RESPONSE: {
            // 获取状态
            break;
        }
        case T_TESTER_READ_CURRENT_TEST_DATA: {
            // 获取当前测试项的数据
            break;
        }
        case T_TESTER_READ_CURRENT_GROUP_TEST_DATA: {
            TTesterResolveCurrentGroupResponse(&data[1], len - 1);
            break;
        }
        case T_TESTER_READ_HISTORY_GROUP_TEST_DATA: {
            // 获取历史测试数据
            break;
        }
        default: {
            break;
        }
    }
}

static void TTesterResolveCommandResponse(uint8_t* data, uint16_t len)
{
    // char *TAG = "TTester resolve command resopnse";
    
    switch (data[0]) {
        case TTESTERCOMMANDFRAMEERROR: {
            // ESP_LOGE(TAG, "command frame error");
            break;
        }
        case TTESTERCOMMANDRECEIVESUCCESS: {
            // ESP_LOGE(TAG, "command receive success");
            break;
        }
        case TTESTERCOMMANDREFUSE: {
            // ESP_LOGE(TAG, "command refuse");
            break;
        }
        default: {
            break;
        }
    }
}

void TTestSelectGroup(uint16_t group)
{
    TTesterPackSelectGroupFrame(group);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterselectgroupbuff, sizeof(TTesterOneCommandDataFrameFormat));
}

void TTestInquiryStatus(void)
{
    TTesterPackReadStatusFrame();
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadstatussbuff, sizeof(TTesterNoCommandDataFrameFormat));
}

void TTestReadCurrentItem(void)
{
    TTesterPackReadCurrentItemFrame();
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadcuritemsbuff, sizeof(TTesterOneCommandDataFrameFormat));
}

void TTestReadCurrentGroup(void)
{
    TTesterPackReaCurrentGroupFrame();
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadcurgroupbuff, sizeof(TTesterOneCommandDataFrameFormat));
}

void TTestReadHistoryGroup(uint16_t group)
{
    TTesterPackReadHistoryGroupFrame(group);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadhisgroupbuff, sizeof(TTesterTwoCommandDataFrameFormat));
}

int TTesterResolve(void)
{
	uint8_t curData[4] = {0};
    uint8_t length, sum;
	int ret, i;

	while ((curData[0] != T_TESTER_FRAME_HEAD) && (curData[1] != T_TESTER_PROTOCOL_NUM) \
        && (curData[3] != T_TESTER_ADDRESS)) {
        ret = UART_ReadBufferBytes(&curData[0], 4);
        if (ret != 0) {
            return -1;
        }
	}

    ret = UART_ReadBufferBytes(&length, 1);
    if (ret != 0) {
        return -1;
    }

    length -= 6;
    ret = UART_ReadBufferBytes(ttestrdatabuffer, length);
    if (ret != 0) {
        return -1;
    }

    ret = UART_ReadBufferBytes(&curData[0], 2);
    if (ret != 0) {
        return -1;
    }
    sum = curData[1];

    switch (ttestrdatabuffer[1]) {
        case TTESTERDATAREVRESPONSE: {
            TTesterResolveCommandResponse(&ttestrdatabuffer[2], length - 2);
            break;
        }
        case TTESTERDATAREADRESPONSE: {
            TTesterResolveReadDataResponse(&ttestrdatabuffer[2], length - 2);
            break;
        }
        default: {
            break;
        } 
    }

    for (i = 0; i < 7; i++) {
         ESP_LOGI(TAG, "group:%d term:%d item:%d result:%d 1:%f 2:%f 3:%f 4:%f 5:%f", curgrouptestdata.itemdata[i].group, \
            curgrouptestdata.itemdata[i].term, curgrouptestdata.itemdata[i].item, curgrouptestdata.itemdata[i].termret, \
            *((float *)(&(curgrouptestdata.itemdata[i].para[0]))), *((float *)(&(curgrouptestdata.itemdata[i].para[4]))), \
            *((float *)(&(curgrouptestdata.itemdata[i].para[8]))), *((float *)(&(curgrouptestdata.itemdata[i].para[12]))), \
            *((float *)(&(curgrouptestdata.itemdata[i].para[16]))));
    }
        ESP_LOGI(TAG, "group:%d term:%d item:%d termresult:%d groupresult:%d allresult:%d 1:%f 2:%f 3:%f 4:%f 5:%f", \
            curgrouptestdata.lastitemdata.group, curgrouptestdata.lastitemdata.term, curgrouptestdata.lastitemdata.item, \
            curgrouptestdata.lastitemdata.termret, curgrouptestdata.lastitemdata.groupret, curgrouptestdata.lastitemdata.allret, \
            *((float *)(&(curgrouptestdata.itemdata[i].para[0]))), *((float *)(&(curgrouptestdata.itemdata[i].para[4]))), \
            *((float *)(&(curgrouptestdata.itemdata[i].para[8]))), *((float *)(&(curgrouptestdata.itemdata[i].para[12]))), \
            *((float *)(&(curgrouptestdata.itemdata[i].para[16]))));

	return 0;
}
