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
#include "tcp_client.h"
#include "time.h"

/*
{
	"devId": "TTester",
	"devNumber": "TTester",
	"devName": "TTester",
	"devStatus": "TTester",
	"devTypeId": "TTester",
    "deviceOrderWay": "read",
	"orderName": "",
	"orderId": "FR001",
	"orderMode": "group",
	"groupId": "001", 
	"ParameterValues": "10.00__84.00__0.00__0.00__0.00__11.00__7.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00__0.000.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00__0.000.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00__0.00",
	"ParameterUnits": "V__mA__s__s__s__A__mO__s__s__s__V__mO__s__s__s__V__A__A__s__s__V__mA__W__W__sV__mA__s__s__s__V__mA__s__s__s__V__mA__s__s__s",
	"ParameterChecks": "6__6__7__7__7__7__7__7",
	"GroupChecks": "6",
	"TotalChecks": "6",
	"value": "6",
	"devIP": "",
	"timeStamp": "77396"
	"valueUnit": ""
	"expand": ""
	"isAnswer": "yes"
};;**##
*/

TTesterNoCommandDataFrameFormat ttesterreadstatussbuff = {0};
TTesterOneCommandDataFrameFormat ttesterreadcuritemsbuff = {0};
TTesterOneCommandDataFrameFormat ttesterreadcurgroupbuff = {0};
TTesterTwoCommandDataFrameFormat ttesterreadhisgroupbuff = {0};
TTesterOneCommandDataFrameFormat ttesterselectgroupbuff = {0};
TTesterCommonFrameFormat ttestersetgroupbuff = {0};

TTesterPressurizationPara ttesterpredata = {0};
TTesterGroundingPara ttestergrodata = {0};
TTesterInsulationPara ttesterrindata = {0};
TTesterLeakagePara ttesterleadata = {0};
TTesterPowerPara ttesterpowdata = {0};
TTesterStartupPara ttesterstadata = {0};
TTesterOpenshortPara ttesterosdata = {0};
TTesterDcvoltagePara ttesterdcdata = {0};

TTesterSetPressurizationPara setprepara = {0};
TTesterSetGroundingPara setgroundpara = {0};
TTesterSetInsulationPara setinspara = {0};
TTesterSetLeakagePara setleapara = {0};
TTesterSetPowerPara setpowerpara = {0};
TTesterSetStartupPara setstartuppara = {0};
TTesterSetOpenshortPara setopenshortpara = {0};
int ttestparanum[9] = {0, T_TESTER_SET_PRESSURIZATION_PARA_NUM, T_TESTER_SET_GROUNDING_PARA_NUM, \
    T_TESTER_SET_INSULATION_PARA_NUM, T_TESTER_SET_LEAKAGE_PARA_NUM, T_TESTER_SET_POWER_PARA_NUM, \
    T_TESTER_SET_STARTUP_PARA_NUM, T_TESTER_SET_OPENSHORT_PARA_NUM, T_TESTER_SET_DCVOLTAGE_PARA_NUM};
int ttestparabytenum[9] = {1, sizeof(TTesterSetPressurizationPara) + 1, sizeof(TTesterSetGroundingPara) + 1, \
    sizeof(TTesterSetInsulationPara) + 1, sizeof(TTesterSetLeakagePara) + 1, sizeof(TTesterSetPowerPara) + 1, \
    sizeof(TTesterSetStartupPara) + 1, sizeof(TTesterSetOpenshortPara) + 1, 1};
int tterterparasetstatus[8] = {0};
char ttesterjsondatabuff[1024] = {0};

TTestestGroupResultPara curgrouptestdata = {0};
uint8_t ttesterrdatabuffer[256] = {0};
uint16_t ttestersetparabytenum = 0;
int ttesterreadgroupnumber = 0;
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

static void TTesterPackSetGroupParaFrame(uint16_t group)
{
    int i, count;
    uint8_t sum[3] = {0}; 
    ttestersetgroupbuff.data = (uint8_t *)malloc(3 + ttestersetparabytenum);
    if (ttestersetgroupbuff.data == NULL) {
        ESP_LOGE(TAG, "mem error");
        return;
    }
    ttestersetgroupbuff.head = T_TESTER_FRAME_HEAD;
    ttestersetgroupbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttestersetgroupbuff.address[0] = 0;
    ttestersetgroupbuff.address[1] = T_TESTER_ADDRESS;
    ttestersetgroupbuff.length = sizeof(TTesterCommonFrameFormat) -1 - 4 + 3 + ttestersetparabytenum;
    ttestersetgroupbuff.reserve = 0;
    ttestersetgroupbuff.data[0] = TTESTERSETPARA;
    ttestersetgroupbuff.data[1] = T_TESTER_SET_PARA_SECOND_BYTE;
    ttestersetgroupbuff.data[2] = group;
    count = 3;
    for (i = 0; i < 8; i++) {
        ttestersetgroupbuff.data[count] = (uint8_t)(tterterparasetstatus[i]);
        switch (tterterparasetstatus[i]) {
            case TTESTERPRESSURIZATION: {
                memcpy(&ttestersetgroupbuff.data[count + 1], &setprepara, ttestparabytenum[tterterparasetstatus[i]] - 1);
                break;
            }
            case TTESTERGROUNDING: {
                memcpy(&ttestersetgroupbuff.data[count + 1], &setgroundpara, ttestparabytenum[tterterparasetstatus[i]] - 1);
                break;
            }
            case TTESTERINSULATION: {
                memcpy(&ttestersetgroupbuff.data[count + 1], &setinspara, ttestparabytenum[tterterparasetstatus[i]] - 1);
                break;
            }
            case TTESTERLEAKAGE: {
                memcpy(&ttestersetgroupbuff.data[count + 1], &setleapara, ttestparabytenum[tterterparasetstatus[i]] - 1);
                break;
            }
            case TTESTERPOWER: {
                memcpy(&ttestersetgroupbuff.data[count + 1], &setpowerpara, ttestparabytenum[tterterparasetstatus[i]] - 1);
                break;
            }
            case TTESTERSTARTUP: {
                memcpy(&ttestersetgroupbuff.data[count + 1], &setstartuppara, ttestparabytenum[tterterparasetstatus[i]] - 1);
                break;
            }
            case TTESTEROPENSHORT: {
                memcpy(&ttestersetgroupbuff.data[count + 1], &setopenshortpara, ttestparabytenum[tterterparasetstatus[i]] - 1);
                break;
            }
            default: {
                break;
            }
        }
        count += ttestparabytenum[tterterparasetstatus[i]];
    }

    ttestersetgroupbuff.end = T_TESTER_FRAME_END;
    sum[0] = CalSumCheckDataLow(&ttestersetgroupbuff.head, 6);
    sum[1] = CalSumCheckDataLow(&ttestersetgroupbuff.data[0], 3 + ttestersetparabytenum);
    sum[2] = T_TESTER_FRAME_END;
    ttestersetgroupbuff.sum = CalSumCheckDataLow(sum, sizeof(sum));
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
            ttesterdcdata.voltage = TTesterResTranData((rdata[0] << 8) + rdata[1]);
            ttesterdcdata.current = TTesterResTranData((rdata[2] << 8) + rdata[3]);
            ttesterdcdata.testtime = TTesterResTranData((rdata[4] << 8) + rdata[5]);
            ttesterdcdata.reserve1 = TTesterResTranData((rdata[6] << 8) + rdata[7]);
            ttesterdcdata.reserve2 = TTesterResTranData((rdata[8] << 8) + rdata[9]);
            if (num < 7) {
                memcpy(curgrouptestdata.itemdata[num].para, &ttesterdcdata, len * 2);
            } else {
                memcpy(curgrouptestdata.lastitemdata.para, &ttesterdcdata, len * 2);
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


static void TTesterPackJsonFrame(void)
{
    char ttesterparadatajsondatabuff[256] = {0};
    char ttesterparaunitjsondatabuff[128] = {0};
    char ttestertestresultjsondatabuff[64] = {0};
    
    (void)sprintf(ttesterparadatajsondatabuff, "%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f"
                "%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f"
                "%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f",
            ttesterpredata.voltage, ttesterpredata.current, ttesterpredata.testtime, ttesterpredata.uptime, ttesterpredata.downtime, \
            ttestergrodata.current, ttestergrodata.resistance, ttestergrodata.testtime, ttestergrodata.reserve1, ttestergrodata.reserve2, \
            ttesterrindata.voltage, ttesterrindata.resistance, ttesterrindata.testtime, ttesterrindata.reserve1, ttesterrindata.reserve2, \
            ttesterpowdata.voltage, ttesterpowdata.voltage, ttesterpowdata.voltage, ttesterpowdata.voltage, ttesterpowdata.voltage, \
            ttesterleadata.voltage, ttesterleadata.firecurrent, ttesterleadata.groundcurrent, ttesterleadata.testtime, ttesterleadata.reserve, \
            ttesterstadata.voltage, ttesterstadata.current, ttesterstadata.voltage, ttesterstadata.reserve1, ttesterstadata.reserve2, \
            ttesterosdata.voltage, ttesterosdata.current, ttesterosdata.testtime, ttesterosdata.reserve1, ttesterosdata.reserve2, \
            ttesterdcdata.voltage, ttesterdcdata.current, ttesterdcdata.testtime, ttesterdcdata.reserve1, ttesterdcdata.reserve2);
    (void)sprintf(ttesterparaunitjsondatabuff, "V__mA__s__s__s__A__mO__s__s__s__V__mO__s__s__s__V__A__A__s__s__V__mA__W__W__s"
                "V__mA__s__s__s__V__mA__s__s__s__V__mA__s__s__s");
    (void)sprintf(ttestertestresultjsondatabuff, "%d__%d__%d__%d__%d__%d__%d__%d", curgrouptestdata.itemdata[0].termret, curgrouptestdata.itemdata[1].termret, \
    curgrouptestdata.itemdata[2].termret, curgrouptestdata.itemdata[3].termret, curgrouptestdata.itemdata[4].termret, curgrouptestdata.itemdata[5].termret, \
    curgrouptestdata.itemdata[6].termret, curgrouptestdata.lastitemdata.termret);
    (void)sprintf(ttesterjsondatabuff, "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devStatus\": \"%s\",\n    \"devTypeId\": \"%s\",\n    \"deviceOrderWay\": \"%s\",\n    \"orderName\":\"%s\",\n    \"orderId\":\"%s\",\n"
                "    \"orderMode\":\"%s\",\n    \"groupId\":\"%03d\",\n    \"ParameterValues\":\"%s\",\n    \"ParameterUnits\":\"%s\",\n    \"ParameterChecks\":\"%s\",\n"
		        "    \"GroupChecks\":\"%d\",\n    \"TotalChecks\":\"%d\",\n    \"value\":\"\",\n    \"devIP\":\"%s\",\n    \"timeStamp\":\"%lld\"\n    \"valueUnit\":\"\"\n"  
                "    \"expand\":\"\"\n    \"isAnswer\":\"%s\"\n};;**##",
            T_TESTER_DEVID, T_TESTER_DEVNUMBER, T_TESTER_DEVNAME, T_TESTER_DEVSTATUS, T_TESTER_DEVTYPEID, "read", T_TESTER_ORDERNAME, \
            T_TESTER_ORDERID, T_TESTER_ORDERMODE, ttesterreadgroupnumber, ttesterparadatajsondatabuff, ttesterparaunitjsondatabuff, ttestertestresultjsondatabuff, \
            curgrouptestdata.lastitemdata.groupret, curgrouptestdata.lastitemdata.allret, GetStaIp(), GetMilliTimeNow(), T_TESTER_ISANSWER_YES);
}

uint16_t TTesterStrChangeToUint(char *str)
{
    int pos, rdata;
    
    pos = CalFloatStrPointPos(str);
    rdata = (int)(atof(str) * pow(10, pos));
    return ((rdata & 0x3fff) | (pos << 14)); 
}

void TTesterGetSetParaByteNum(uint16_t num)
{
    ttestersetparabytenum = num;
}

int TTesterGetSetParaStatus(char *str)
{
    char *strarry[sizeof(tterterparasetstatus) / sizeof(int)];
    char *token;
    int i = 0;

    if (str == NULL) {
        return -1;
    }

    token = strtok(str, "_");
    while (token != NULL) {
        strarry[i] = token;
        token = strtok(NULL, "_");
        i++;
    }

    for (i = 0; i < sizeof(tterterparasetstatus) / sizeof(int); i++) {
        tterterparasetstatus[i] = atoi(strarry[i]);
        // ESP_LOGI(TAG, "size:%d tterterparasetstatus[%d]:%d strarry[%d]:%s", sizeof(tterterparasetstatus) / sizeof(int), i, tterterparasetstatus[i], i, strarry[i]);
    }

    return 0;
}

void TTesterSelectGroup(uint16_t group)
{
    TTesterPackSelectGroupFrame(group);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterselectgroupbuff, sizeof(TTesterOneCommandDataFrameFormat));
}

void TTesterInquiryStatus(void)
{
    TTesterPackReadStatusFrame();
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadstatussbuff, sizeof(TTesterNoCommandDataFrameFormat));
}

void TTesterReadCurrentItem(void)
{
    TTesterPackReadCurrentItemFrame();
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadcuritemsbuff, sizeof(TTesterOneCommandDataFrameFormat));
}

void TTesterReadCurrentGroup(void)
{
    TTesterPackReaCurrentGroupFrame();
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadcurgroupbuff, sizeof(TTesterOneCommandDataFrameFormat));
}

void TTesterReadHistoryGroup(uint16_t group)
{
    TTesterPackReadHistoryGroupFrame(group);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadhisgroupbuff, sizeof(TTesterTwoCommandDataFrameFormat));
}

void TTesterSetGroupPara(uint16_t group)
{
    TTesterPackSetGroupParaFrame(group);
    uart_write_bytes(UART_NUM_1, (uint8_t *)&ttestersetgroupbuff.head, 6);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&(ttestersetgroupbuff.data[0]), 3 + ttestersetparabytenum);
    uart_write_bytes(UART_NUM_1, (uint8_t *)&ttestersetgroupbuff.end, 2);
}

void TTesterGetJsonData(char *str)
{
    strcpy(str, ttesterjsondatabuff);
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
    ret = UART_ReadBufferBytes(ttesterrdatabuffer, length);
    if (ret != 0) {
        return -1;
    }

    ret = UART_ReadBufferBytes(&curData[0], 2);
    if (ret != 0) {
        return -1;
    }
    sum = curData[1];

    switch (ttesterrdatabuffer[1]) {
        case TTESTERDATAREVRESPONSE: {
            TTesterResolveCommandResponse(&ttesterrdatabuffer[2], length - 2);
            break;
        }
        case TTESTERDATAREADRESPONSE: {
            TTesterResolveReadDataResponse(&ttesterrdatabuffer[2], length - 2);
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

    TTesterPackJsonFrame();
	return 0;
}
