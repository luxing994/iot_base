#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <ctype.h>  
#include <math.h>
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "ainuo_tester_ascii_protocol.h"
#include "t_tester_protocol.h"
#include "iot_common.h"
#include "tcp_client.h"
#include "time.h"

AINUOTTesterGroundingPara ainuogroudingtestdata = {0};
AINUOTTesterInsulationPara ainuoinsulationtestdata = {0};
AINUOTTesterPressurizationPara ainuopressurizationtestdata = {0};
AINUOTTesterLeakagePara ainuoleakagetestdata = {0};
AINUOTTesterPowerPara ainuopowertestdata = {0};
AINUOTTesterStartPara ainuostarttestdata = {0};
AINUOTTesterDCVoltagePara ainuodcvoltagetestdata = {0};
AINUOTTesterShortCircuitPara ainuoshortcircuittestdata = {0};

AINUOTTesterSetGroundingPara ainuosetgroudingtestdata = {0};
int ainuotesterstatus = 0;

char g_ainuosenddatabuff[256] = {0};
char g_ainuoreaddatabuff[256] = {0};
char g_ainuottesterjsondatabuff[2048] = {0};
const char *AINUOTAG = "AINUO_TTest";

static int AINUO_TTesterPackSendCommandData(int address, int command)
{
    int len;
    
    len = sprintf(g_ainuosenddatabuff, "{%03d%1d00}", address, command);
    if (len != 8) {
        return -1;
    }

    return 0;
}

static int AINUO_TTesterPackSendSetCommandData(int address, int group)
{
    uint8_t checksum;
    char addstring[16] = {0};
    
    (void)sprintf(g_ainuosenddatabuff, "{%03d%1d1%04d%04d%04d%04d", address, AINUOTTESTSETPARA, (int)(ainuosetgroudingtestdata.current * 100), \
    (int)ainuosetgroudingtestdata.resistance1, (int)ainuosetgroudingtestdata.resistance2, (int)(ainuosetgroudingtestdata.time * 10));

    checksum = CalSumCheckDataLow((uint8_t *)&g_ainuosenddatabuff[1], strlen(g_ainuosenddatabuff) - 1);
    
    (void)sprintf(addstring, "%02X}", checksum);
    strcat(g_ainuosenddatabuff, addstring);

    return 0;
}

static void AINUO_TTesterResolveGroundingData(uint8_t *data)
{
    char strdata[5] = {0};
    
    memcpy(strdata, &data[1], 4);
    ainuogroudingtestdata.current = atoi(strdata) / 100.0;
    memcpy(strdata, &data[5], 4);
    ainuogroudingtestdata.resistance = atoi(strdata) / 10.0;
    memcpy(strdata, &data[17], 4);
    ainuogroudingtestdata.testtime = atoi(strdata) / 10.0;
    ainuogroudingtestdata.result = data[22] - '0';
}

static void AINUO_TTesterResolveInsulationData(uint8_t *data)
{
    char strdata[5] = {0};
    
    memcpy(strdata, &data[1], 4);
    ainuoinsulationtestdata.voltage = (float)atoi(strdata);
    memcpy(strdata, &data[5], 4);
    ainuoinsulationtestdata.resistance = atoi(strdata) / 10.0;
    memcpy(strdata, &data[17], 4);
    ainuoinsulationtestdata.testtime = atoi(strdata) / 10.0;
    ainuoinsulationtestdata.result = data[22] - '0';
}

static void AINUO_TTesterResolvePressurizationData(uint8_t *data)
{
    char strdata[5] = {0};
    
    memcpy(strdata, &data[1], 4);
    ainuopressurizationtestdata.voltage = (float)atoi(strdata);
    memcpy(strdata, &data[5], 4);
    ainuopressurizationtestdata.current = atoi(strdata) / 100.0;
    memcpy(strdata, &data[17], 4);
    ainuopressurizationtestdata.testtime = atoi(strdata) / 10.0;
    ainuopressurizationtestdata.condition = data[21] - '0';
    ainuopressurizationtestdata.result = data[22] - '0';
}

static void AINUO_TTesterResolveLeakageData(uint8_t *data)
{
    char strdata[5] = {0};
    
    memcpy(strdata, &data[1], 4);
    ainuoleakagetestdata.voltage = atoi(strdata) / 10.0;
    memcpy(strdata, &data[5], 4);
    ainuoleakagetestdata.current = (float)atoi(strdata);
    memcpy(strdata, &data[17], 4);
    ainuoleakagetestdata.testtime = atoi(strdata) / 10.0;
    ainuoleakagetestdata.condition = data[21] - '0';
    ainuoleakagetestdata.result = data[22] - '0';
}

static void AINUO_TTesterResolvePowerData(uint8_t *data)
{
    char strdata[5] = {0};
    
    memcpy(strdata, &data[1], 4);
    ainuopowertestdata.voltage = atoi(strdata) / 10.0;
    memcpy(strdata, &data[5], 4);
    ainuopowertestdata.current = atoi(strdata) / 100.0;
    memcpy(strdata, &data[9], 4);
    ainuopowertestdata.power = atoi(strdata) / 10.0;
    memcpy(strdata, &data[17], 4);
    ainuopowertestdata.testtime = atoi(strdata) / 10.0;
    ainuopowertestdata.condition = data[21] - '0';
    ainuopowertestdata.result = data[22] - '0';
}

static void AINUO_TTesterResolveStartData(uint8_t *data)
{
    char strdata[5] = {0};
    
    memcpy(strdata, &data[1], 4);
    ainuostarttestdata.voltage = atoi(strdata) / 10.0;
    memcpy(strdata, &data[5], 4);
    ainuostarttestdata.current1 = atoi(strdata) / 100.0;
    memcpy(strdata, &data[9], 4);
    ainuostarttestdata.current2 = atoi(strdata) / 100.0;
    memcpy(strdata, &data[13], 4);
    ainuostarttestdata.current3 = atoi(strdata) / 100.0;
    memcpy(strdata, &data[17], 4);
    ainuostarttestdata.testtime = atoi(strdata) / 10.0;
    ainuostarttestdata.condition = data[21] - '0';
    ainuostarttestdata.result = data[22] - '0';
}

static void AINUO_TTesterResolveDCVoltageData(uint8_t *data)
{
    char strdata[5] = {0};
    
    memcpy(strdata, &data[1], 4);
    ainuodcvoltagetestdata.voltage = (float)atoi(strdata);
    memcpy(strdata, &data[5], 4);
    ainuodcvoltagetestdata.current = (float)atoi(strdata);
    memcpy(strdata, &data[17], 4);
    ainuodcvoltagetestdata.testtime = atoi(strdata) / 10.0;
    ainuodcvoltagetestdata.condition = data[21] - '0';
    ainuodcvoltagetestdata.result = data[22] - '0';
}

static void AINUO_TTesterResolveShortCircuitData(uint8_t *data)
{
    char strdata[5] = {0};
    
    memcpy(strdata, &data[1], 4);
    ainuoshortcircuittestdata.voltage = atoi(strdata) / 10.0;
    memcpy(strdata, &data[5], 4);
    ainuoshortcircuittestdata.resistance = atoi(strdata) / 10.0;
    memcpy(strdata, &data[17], 4);
    ainuoshortcircuittestdata.testtime = atoi(strdata) / 10.0;
    ainuoshortcircuittestdata.result = data[22] - '0';
}

static void AINUO_TTesterResolveTestData(void)
{
    AINUO_TTesterResolveGroundingData((uint8_t *)&g_ainuoreaddatabuff[0]);
    AINUO_TTesterResolveInsulationData((uint8_t *)&g_ainuoreaddatabuff[23]);
    AINUO_TTesterResolvePressurizationData((uint8_t *)&g_ainuoreaddatabuff[46]);
    AINUO_TTesterResolveLeakageData((uint8_t *)&g_ainuoreaddatabuff[69]);
    AINUO_TTesterResolvePowerData((uint8_t *)&g_ainuoreaddatabuff[92]);
    AINUO_TTesterResolveStartData((uint8_t *)&g_ainuoreaddatabuff[115]);
    AINUO_TTesterResolveDCVoltageData((uint8_t *)&g_ainuoreaddatabuff[138]);
    AINUO_TTesterResolveShortCircuitData((uint8_t *)&g_ainuoreaddatabuff[161]);

    ainuotesterstatus = g_ainuoreaddatabuff[184] - '0';
}

static void AINUO_TTesterPackJsonFrame(void)
{
    char ttesterparadatajsondatabuff[512] = {0};
    char ttesterparaunitjsondatabuff[128] = {0};
    char ttestertestresultjsondatabuff[128] = {0};
    int groupret, allret;
    
    if ((ainuogroudingtestdata.result == 1) && (ainuoinsulationtestdata.result == 1) && (ainuopressurizationtestdata.result == 1) && (ainuoleakagetestdata.result == 1) \
    && (ainuopowertestdata.result == 1) && (ainuoinsulationtestdata.result == 1) && (ainuodcvoltagetestdata.result == 1) && (ainuoshortcircuittestdata.result == 1)) {
        groupret = 5;
        allret = 5;
    } else {
        groupret = 6;
        allret = 6;
    }

    (void)sprintf(ttesterparadatajsondatabuff, "%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__"
                "%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__"
                "%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f",
            ainuogroudingtestdata.current, ainuogroudingtestdata.resistance, 0.0, 0.0, ainuogroudingtestdata.testtime, \
            ainuoinsulationtestdata.voltage, ainuoinsulationtestdata.resistance, 0.0, 0.0, ainuoinsulationtestdata.testtime, \
            ainuopressurizationtestdata.voltage, ainuopressurizationtestdata.current, 0.0, 0.0, ainuopressurizationtestdata.testtime, \
            ainuoleakagetestdata.voltage, ainuoleakagetestdata.current, 0.0, 0.0, ainuoleakagetestdata.testtime, \
            ainuopowertestdata.voltage, ainuopowertestdata.current, ainuopowertestdata.power, 0.0, ainuopowertestdata.testtime, \
            ainuostarttestdata.voltage, ainuostarttestdata.current1, ainuostarttestdata.current2, ainuostarttestdata.current3, ainuostarttestdata.testtime, \
            ainuodcvoltagetestdata.voltage, ainuodcvoltagetestdata.current, 0.0, 0.0, ainuodcvoltagetestdata.testtime, \
            ainuoshortcircuittestdata.voltage, ainuoshortcircuittestdata.resistance, 0.0, 0.0, ainuoshortcircuittestdata.testtime);
    (void)sprintf(ttesterparaunitjsondatabuff, "2__3__1__4__5__6__8__7");   // 2：接地 3：绝缘 1：耐压 4：泄露 5：功率 6：启动 7：直耐 8：短路
    (void)sprintf(ttestertestresultjsondatabuff, "%d__%d__%d__%d__%d__%d__%d__%d", (ainuogroudingtestdata.result) + 4, (ainuoinsulationtestdata.result) + 4, \
    (ainuopressurizationtestdata.result) + 4, (ainuoleakagetestdata.result) + 4, (ainuopowertestdata.result) + 4, (ainuostarttestdata.result) + 4, \
    (ainuodcvoltagetestdata.result) + 4, (ainuoshortcircuittestdata.result) + 4);   // 0: 未判断 1：合格 其他：不合格
    (void)sprintf(g_ainuottesterjsondatabuff, "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devStatus\": \"%s\",\n    \"devTypeId\": \"%s\",\n    \"deviceOrderWay\": \"%s\",\n    \"orderName\":\"%s\",\n    \"orderId\":\"%s\",\n"
                "    \"orderMode\":\"%s\",\n    \"groupId\":\"%03d\",\n    \"ParameterValues\":\"%s\",\n    \"ParameterTags\":\"%s\",\n    \"ParameterChecks\":\"%s\",\n"
		        "    \"GroupChecks\":\"%d\",\n    \"TotalChecks\":\"%d\",\n    \"value\":\"\",\n    \"devIP\":\"%s\",\n    \"timeStamp\":\"%lld\",\n    \"valueUnit\":\"\",\n"  
                "    \"expand\":\"\",\n    \"isAnswer\":\"%s\"\n};;**##",
            DEVID, T_TESTER_DEVNUMBER, T_TESTER_DEVNAME, T_TESTER_DEVSTATUS, SAFETYTESTERTYPEID, "read", T_TESTER_ORDERNAME, \
            T_TESTER_ORDERID, T_TESTER_ORDERMODE, 0, ttesterparadatajsondatabuff, ttesterparaunitjsondatabuff, ttestertestresultjsondatabuff, \
            groupret, allret, GetStaIp(), GetMilliTimeNow(), T_TESTER_ISANSWER_NO);
}

void AINUO_TTesterReadCurrentGroup(void)
{
    AINUO_TTesterPackSendCommandData(AINUO_TESTER_ADDRESS, AINUOTTESTERREADDATA);
    uart_write_bytes(UART_NUM_1, (uint8_t *)&g_ainuosenddatabuff, strlen(g_ainuosenddatabuff));
}

void AINUO_TTesterGetJsonData(char *str)
{
    strcpy(str, g_ainuottesterjsondatabuff);
}

int AINUO_TTesterResolve(void)
{
	char curData[5] = {0};
    char sum[2] = {0};
    char end, command;
    int address, ret, i;

	while ((curData[0] != AINUO_TESTER_FRAME_HEAD) || (address != AINUO_TESTER_ADDRESS)) {
        ret = UART_ReadBufferBytes((uint8_t *)&curData[0], 4);
        sscanf(&curData[1], "%d", &address);
        if (ret != 0) {
            return -1;
        }
	}

    ret = UART_ReadBufferBytes((uint8_t *)&command, 1);
    if (ret != 0) {
        return -1;
    }

    ret = UART_ReadBufferBytes((uint8_t *)&g_ainuoreaddatabuff, AINUO_TESTER_GROUP_LENTH * AINUO_TESTER_ROW_LENTH + 1);
    if (ret != 0) {
        return -1;
    }

    ret = UART_ReadBufferBytes((uint8_t *)&sum, 2);
    if (ret != 0) {
        return -1;
    }

    ret = UART_ReadBufferBytes((uint8_t *)&end, 1);
    if (ret != 0 || end != AINUO_TESTER_FRAME_END) {
        return -1;
    }

    switch (command - '0') {
        case AINUOTTESTERREADDATA: {
        AINUO_TTesterResolveTestData();     
            break;
        }
        default: {
            break;
        } 
    }

    ESP_LOGI(AINUOTAG, "number:1 item:grounding 1(current):%f 2(resistance):%f 3(void):NaN 4(void):NaN 5(testtime):%f condition:NaN result:%d", 
        ainuogroudingtestdata.current, ainuogroudingtestdata.resistance, ainuogroudingtestdata.testtime, ainuogroudingtestdata.result);
    ESP_LOGI(AINUOTAG, "number:2 item:insulation 1(voltage):%f 2(resistance):%f 3(void):NaN 4(void):NaN 5(testtime):%f condition:NaN result:%d", 
        ainuoinsulationtestdata.voltage, ainuoinsulationtestdata.resistance, ainuoinsulationtestdata.testtime, ainuoinsulationtestdata.result);
    ESP_LOGI(AINUOTAG, "number:3 item:pressurization 1(voltage):%f 2(current):%f 3(void):NaN 4(void):NaN 5(testtime):%f condition:%d result:%d", 
        ainuopressurizationtestdata.voltage, ainuopressurizationtestdata.current, ainuopressurizationtestdata.testtime, \
        ainuopressurizationtestdata.condition, ainuopressurizationtestdata.result);
    ESP_LOGI(AINUOTAG, "number:4 item:leakage 1(voltage):%f 2(current):%f 3(void):NaN 4(void):NaN 5(testtime):%f condition:%d result:%d", 
        ainuoleakagetestdata.voltage, ainuoleakagetestdata.current, ainuoleakagetestdata.testtime, ainuoleakagetestdata.condition, ainuoleakagetestdata.result);
    ESP_LOGI(AINUOTAG, "number:5 item:power 1(voltage):%f 2(current):%f 3(power):%f 4(void):NaN 5(testtime):%f condition:%d result:%d", 
        ainuopowertestdata.voltage, ainuopowertestdata.current, ainuopowertestdata.power, ainuopowertestdata.testtime, ainuopowertestdata.condition, ainuopowertestdata.result);
    ESP_LOGI(AINUOTAG, "number:6 item:start 1(voltage):%f 2(current1):%f 3(current2):%f 4(current3):%f 5(testtime):%f condition:%d result:%d", 
        ainuostarttestdata.voltage, ainuostarttestdata.current1, ainuostarttestdata.current2, ainuostarttestdata.current3, \
        ainuostarttestdata.testtime, ainuostarttestdata.condition, ainuostarttestdata.result);
    ESP_LOGI(AINUOTAG, "number:7 item:DC voltage 1(voltage):%f 2(current):%f 3(void):NaN 4(void):NaN 5(testtime):%f condition:%d result:%d", 
        ainuodcvoltagetestdata.voltage, ainuodcvoltagetestdata.current, ainuodcvoltagetestdata.testtime, ainuodcvoltagetestdata.condition, ainuodcvoltagetestdata.result);
    ESP_LOGI(AINUOTAG, "number:8 item:short circuit 1(voltage):%f 2(resistance):%f 3(void):NaN 4(void):NaN 5(testtime):%f condition:NaN result:%d", 
        ainuoshortcircuittestdata.voltage, ainuoshortcircuittestdata.resistance, ainuoshortcircuittestdata.testtime, ainuoshortcircuittestdata.result);

    AINUO_TTesterPackJsonFrame();
	return 0;
}