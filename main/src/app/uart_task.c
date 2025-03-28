/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "mprotocols.h"
#include "hprotocols.h"
#include "iot_common.h"
#include "ringbuffer.h"
#include "tcp_server.h"
#include "tcp_client.h"
#include "time.h"
#include "sensor.h"
#include "fx_plc_protocol.h"
#include "hl_plc_protocol.h"
#include "ls_plc_load_protocol.h"
#include "t_tester_protocol.h"
#include "ainuo_tester_ascii_protocol.h"

#define PATTERN_CHR_NUM    (3) 
#define RX_BUF_SIZE  (UART_BUFF_SIZE * 2)
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)
#define RTS_PIN (GPIO_NUM_8)

extern SNSetCaclReFillingMachine g_setsnrefillingmachine;

static QueueHandle_t uart1_queue;
HproFuncCode funcCode;
CommandJsonData jsondata;
char controlerStr[2048] = {0};
char g_devId[32] = {"IIIG_1.0"};
RingBuffer uart2Buffer;
uint32_t g_devStartStatus = 0;
int g_rdatalen = 1;
int g_senddata = 1;
int g_fxplccount = 0;
int g_fxplcdataformat = 0;   // (0:char *    1:short     2:float    3:BCD   4:int   5:bit)   
int g_fxplcbitpos = 0; 
uint16_t g_lastdata[32] = {0};
char g_lastrdata[5] = {0};
SNCaclReFillingMachine g_lastrefilldata = {0};
SNHostLinkCaclReFillingMachine g_lasthostlinkrefilldata = {0};
int g_datapos = 0;

void ParseOpCode(char *str, uint8_t op)
{
    char rdata[5] = {0};
    char rdatah[9] = {0};
    char rdatal[5] = {0};
    char rfdata[9] = {0};
    int idata, i;
    float fdata;
    jsondata = GetCommandJsonData();
    char version[5] = {0};
    char frstr[10] = {0};
    switch (op) {
        case SWITCHCOUNT: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                GetSwitchCount());
            break;
        }
        case SWITCHSTATUS: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                GetSwitchLevel());
            break;
        }
        case FXPLCDEMODATA: {
            (void)sprintf(frstr, "FR%03d", g_fxplccount);
            if (g_fxplccount - 1 >= 0) {
                g_fxplccount -= 1;
            }
            if (g_fxplcdataformat == 0) {
                FXPLC_ReadBufferBytes((uint8_t *)rdata, sizeof(rdata));
                (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%s\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, FXPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    rdata);
            } else if ((g_fxplcdataformat == 1) || (g_fxplcdataformat == 4) || (g_fxplcdataformat == 5) ) {
                if (g_fxplcdataformat == 1) {
                    FXPLC_ReadBufferBytes((uint8_t *)rdata, 4);
                    (void)sscanf(rdata, "%x", &idata);
                } else if (g_fxplcdataformat == 4) {
                    FXPLC_ReadBufferBytes((uint8_t *)rdatal, 4);
                    FXPLC_ReadBufferBytes((uint8_t *)rdatah, 4);
                    strcat(rdatah, rdatal);
                    (void)sscanf(rdatah, "%x", &idata);
                } else if (g_fxplcdataformat == 5) {
                    FXPLC_ReadBufferBytes((uint8_t *)rdata, 4);
                    (void)sscanf(rdata, "%x", &idata);
                    idata &= 0x0001;
                }
                
#if defined(CONFIG_XF_CONTROLER) || defined(CONFIG_RY_LINE)
                if (idata != g_lastdata[g_fxplccount]) {
                    g_senddata = 1;
                } else {
                    g_senddata = 0;
                }

                if (g_fxplccount < (sizeof(g_lastdata) / sizeof(uint16_t))) {
                    g_lastdata[g_fxplccount] = idata;
                }

                (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, FXPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    idata);
#endif

#ifdef CONFIG_SN_CACLREFILMAC
                if (idata != *(uint16_t *)(&((uint8_t *)&g_lastrefilldata)[g_fxplccount * 4])) {
                    g_senddata = 1;
                } else {
                    g_senddata = 1;
                }
                memcpy(&((uint8_t *)&g_lastrefilldata)[g_fxplccount * 4], (int* )&idata, sizeof(int));
                if (g_fxplccount * 4 == (sizeof(SNCaclReFillingMachine) - 4)) {
                    (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##\n"
                    "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"\",\n    \"devName\":\"\",\n    \"devStatus\":\"\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"orderName\":\"%s\",\n    \"orderId\":\"%s\",\n"
                    "    \"ParameterIds\":\"FR001__FR002__FR003__FR004__FR005__FR006__FR007__FR008__FR009\",\n"
                    "    \"ParameterValues\":\"%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%d__%d__%d\",\n"
                    "    \"ParameterUnits\":\"00__CG__CG__PA__PA__00__00__00__00\",\n"
                    "    \"value\":\"\",\n    \"devIP\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"\",\n    \"expand\":\"\",\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, FXPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    idata, jsondata.devId, FRIGEFILLTYPEID, jsondata.orderName, "BatchParameters", g_lastrefilldata.sysvacuum, \
                    (g_lastrefilldata.atemperature) / 10.0, (g_lastrefilldata.btemperature) / 10.0, (g_lastrefilldata.asyspressure) / 10.0, \
                    (g_lastrefilldata.bsyspressure) / 10.0, g_lastrefilldata.perfusionvolume, g_lastrefilldata.singleproduction, g_lastrefilldata.totalproduction, \ 
                    g_lastrefilldata.result, GetStaIp(), DEVTIMEMODE, GetMilliTimeNow());
                } else {
                    /*
                    if (g_fxplccount == 2 || g_fxplccount == 3 || g_fxplccount == 4 || g_fxplccount == 5) {    // 温度和压力都除10再输出
                        (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                        "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.2f\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                        g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, FXPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                        idata / 10.0);
                    } else {
                    */
                         (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                        "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%u\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                        g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, FXPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                        idata);
                    // }
                }
#endif
                } else if (g_fxplcdataformat == 2) {
                for (i = 0; i < 8; i++) {
                    FXPLC_ReadBufferBytes((uint8_t *)&rfdata[(i + 4) % 8], 1);
                }
                (void)sscanf(rfdata, "%x", &idata);
                fdata = *((float *)&idata);
#ifdef CONFIG_SN_CACLREFILMAC
                if (fdata != *(float *)(&((uint8_t *)&g_lastrefilldata)[g_fxplccount * 4])) {
                    g_senddata = 1;
                } else {
                    g_senddata = 1;
                }
                memcpy(&((uint8_t *)&g_lastrefilldata)[g_fxplccount * 4], &fdata, sizeof(float));
                if (g_fxplccount * 4 == (sizeof(SNCaclReFillingMachine) - 4)) {
                    (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.2f\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##\n"
                    "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"\",\n    \"devName\":\"\",\n    \"devStatus\":\"\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"orderName\":\"%s\",\n    \"orderId\":\"%s\",\n"
                    "    \"ParameterIds\":\"FR001__FR002__FR003__FR004__FR005__FR006__FR007__FR008__FR009\",\n"
                    "    \"ParameterValues\":\"%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%d__%d__%d\",\n"
                    "    \"ParameterUnits\":\"00__CG__CG__PA__PA__00__00__00__00\",\n"
                    "    \"value\":\"\",\n    \"devIP\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"\",\n    \"expand\":\"\",\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, FXPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    fdata, jsondata.devId, FRIGEFILLTYPEID, jsondata.orderName, "BatchParameters", g_lastrefilldata.sysvacuum, \
                    (g_lastrefilldata.atemperature) / 10.0, (g_lastrefilldata.btemperature) / 10.0, (g_lastrefilldata.asyspressure) / 10.0, \
                    (g_lastrefilldata.bsyspressure) / 10.0, g_lastrefilldata.perfusionvolume, g_lastrefilldata.singleproduction, g_lastrefilldata.totalproduction, \ 
                    g_lastrefilldata.result, GetStaIp(), DEVTIMEMODE, GetMilliTimeNow());
                } else {
                    // if (g_fxplccount == 9) {
                    //     if (g_snrefillingsetdata->setchargeamount == fdata) {
                    //         xEventGroupSetBits(xEventGroup3, BIT_2);
                    //     } else {
                    //         xEventGroupSetBits(xEventGroup3, BIT_3);
                    //     }
                    // }
                    (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.2f\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, FXPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    fdata);
                }
#endif
            }
            break;
        }
        case HLPLCDEMODATA: {
            (void)sprintf(frstr, "FR%03d", g_fxplccount);
            if (g_fxplccount - 1 >= 0) {
                g_fxplccount -= 1;
            }
            if (g_fxplcdataformat == 0) {
                FXPLC_ReadBufferBytes((uint8_t *)rdata, sizeof(rdata));
                (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%s\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, HLPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    rdata);
                
            } else if (g_fxplcdataformat == 1) {
                FXPLC_ReadBufferBytes((uint8_t *)rdata, 4);
                (void)sscanf(rdata, "%x", &idata);
                if (idata != *(int *)(&((uint8_t *)&g_lasthostlinkrefilldata)[g_fxplccount * 4])) {
                    g_senddata = 1;
                } else {
                    g_senddata = 1;
                }

                memcpy(&((uint8_t *)&g_lasthostlinkrefilldata)[g_fxplccount * 4], (int* )&idata, sizeof(int));
                if (g_fxplccount * 4 >= (sizeof(SNHostLinkCaclReFillingMachine) - 4)) {
                    (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##\n"
                    "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"\",\n    \"devName\":\"\",\n    \"devStatus\":\"\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"orderName\":\"%s\",\n    \"orderId\":\"%s\",\n"
                    "    \"ParameterIds\":\"FR001__FR002__FR003__FR004__FR005__FR006__FR007__FR008__FR009__FR010__FR011__FR012__FR013__FR014__FR015__FR016__FR017__FR018__FR019__FR020__FR021__FR022__FR023__FR024__FR025__FR026__FR027\",\n"
                    "    \"ParameterValues\":\"%d__%d__%d__%d__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%d__%d__%.2f__%.2f__%d__%.2f__%.2f\",\n"
                    "    \"ParameterUnits\":\"00__00__S__S__PA__PA__PA__PA__MPA__MPA__PA__PA__G__G__00__00__GS__GS__G__G__00__00__S__S__00__00__00\",\n"
                    "    \"value\":\"\",\n    \"devIP\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"\",\n    \"expand\":\"\",\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, HLPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    idata, jsondata.devId, FRIGEFILLTYPEID, jsondata.orderName, "BatchParameters", g_lasthostlinkrefilldata.amode, g_lasthostlinkrefilldata.bmode, \
                    g_lasthostlinkrefilldata.atesttime, g_lasthostlinkrefilldata.btesttime, g_lasthostlinkrefilldata.aup, g_lasthostlinkrefilldata.bup, \
                    g_lasthostlinkrefilldata.adown, g_lasthostlinkrefilldata.bdown, g_lasthostlinkrefilldata.apressure, g_lasthostlinkrefilldata.bpressure, \
                    g_lasthostlinkrefilldata.avacuumdegree, g_lasthostlinkrefilldata.bvacuumdegree, g_lasthostlinkrefilldata.aset, g_lasthostlinkrefilldata.bset, \
                    g_lasthostlinkrefilldata.apercentage, g_lasthostlinkrefilldata.bpercentage, g_lasthostlinkrefilldata.aspeed, g_lasthostlinkrefilldata.bspeed, \
                    g_lasthostlinkrefilldata.achargeamount, g_lasthostlinkrefilldata.bchargeamount, g_lasthostlinkrefilldata.astatus, g_lasthostlinkrefilldata.bstatus, \
                    g_lasthostlinkrefilldata.achargetime, g_lasthostlinkrefilldata.bchargetime, g_lasthostlinkrefilldata.result, g_lasthostlinkrefilldata.arealsetchargeamount, \ 
                    g_lasthostlinkrefilldata.brealsetchargeamount, GetStaIp(), DEVTIMEMODE, GetMilliTimeNow());
                } else {
                    (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, HLPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    idata);
                }
            } else if (g_fxplcdataformat == 2) {
                FXPLC_ReadBufferBytes((uint8_t *)rfdata, 8);
                (void)sscanf(rfdata, "%x", &idata); 
                idata = ((idata & 0x0000ffff) << 16) | ((idata >> 16) & 0x0000ffff);
                fdata = *((float *)&idata);
                if (fdata != *(float *)(&((uint8_t *)&g_lasthostlinkrefilldata)[g_fxplccount * 4])) {
                    g_senddata = 1;
                } else {
                    g_senddata = 1;
                }

                memcpy(&((uint8_t *)&g_lasthostlinkrefilldata)[g_fxplccount * 4], &fdata, sizeof(float));
                if (g_fxplccount * 4 >= (sizeof(SNHostLinkCaclReFillingMachine) - 4)) {
                    (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.2F\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##\n"
                    "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"\",\n    \"devName\":\"\",\n    \"devStatus\":\"\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"orderName\":\"%s\",\n    \"orderId\":\"%s\",\n"
                    "    \"ParameterIds\":\"FR001__FR002__FR003__FR004__FR005__FR006__FR007__FR008__FR009__FR010__FR011__FR012__FR013__FR014__FR015__FR016__FR017__FR018__FR019__FR020__FR021__FR022__FR023__FR024__FR025__FR026__FR027\",\n"
                    "    \"ParameterValues\":\"%d__%d__%d__%d__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%d__%d__%.2f__%.2f__%d__%.2f__%.2f\",\n"
                    "    \"ParameterUnits\":\"00__00__S__S__PA__PA__PA__PA__MPA__MPA__PA__PA__G__G__00__00__GS__GS__G__G__00__00__S__S__00__00__00\",\n"
                    "    \"value\":\"\",\n    \"devIP\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"\",\n    \"expand\":\"\",\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, HLPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    fdata, jsondata.devId, FRIGEFILLTYPEID, jsondata.orderName, "BatchParameters", g_lasthostlinkrefilldata.amode, g_lasthostlinkrefilldata.bmode, \
                    g_lasthostlinkrefilldata.atesttime, g_lasthostlinkrefilldata.btesttime, g_lasthostlinkrefilldata.aup, g_lasthostlinkrefilldata.bup, \
                    g_lasthostlinkrefilldata.adown, g_lasthostlinkrefilldata.bdown, g_lasthostlinkrefilldata.apressure, g_lasthostlinkrefilldata.bpressure, \
                    g_lasthostlinkrefilldata.avacuumdegree, g_lasthostlinkrefilldata.bvacuumdegree, g_lasthostlinkrefilldata.aset, g_lasthostlinkrefilldata.bset, \
                    g_lasthostlinkrefilldata.apercentage, g_lasthostlinkrefilldata.bpercentage, g_lasthostlinkrefilldata.aspeed, g_lasthostlinkrefilldata.bspeed, \
                    g_lasthostlinkrefilldata.achargeamount, g_lasthostlinkrefilldata.bchargeamount, g_lasthostlinkrefilldata.astatus, g_lasthostlinkrefilldata.bstatus, \
                    g_lasthostlinkrefilldata.achargetime, g_lasthostlinkrefilldata.bchargetime, g_lasthostlinkrefilldata.result, g_lasthostlinkrefilldata.arealsetchargeamount, \
                    g_lasthostlinkrefilldata.brealsetchargeamount, GetStaIp(), DEVTIMEMODE, GetMilliTimeNow());
                } else {
                    (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.2f\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, HLPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    fdata);
                }
            } else if (g_fxplcdataformat == 3) {
                FXPLC_ReadBufferBytes((uint8_t *)rdata, 4);
                (void)sscanf(rdata, "%x", &idata);
                idata = BCDToInt(idata);
                if (idata != *(int *)(&((uint8_t *)&g_lasthostlinkrefilldata)[g_fxplccount * 4])) {
                    g_senddata = 1;
                } else {
                    g_senddata = 1;
                }

                memcpy(&((uint8_t *)&g_lasthostlinkrefilldata)[g_fxplccount * 4], (int* )&idata, sizeof(int));
                if (g_fxplccount * 4 >= (sizeof(SNHostLinkCaclReFillingMachine) - 4)) {
                    (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##\n"
                    "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"\",\n    \"devName\":\"\",\n    \"devStatus\":\"\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"orderName\":\"%s\",\n    \"orderId\":\"%s\",\n"
                    "    \"ParameterIds\":\"FR001__FR002__FR003__FR004__FR005__FR006__FR007__FR009__FR010__FR011__FR012__FR013__FR014__FR015__FR016__FR017__FR018__FR019__FR020__FR021__FR022__FR023__FR024__FR025__FR026__FR027\",\n"
                    "    \"ParameterValues\":\"%d__%d__%d__%d__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%.2f__%d__%d__%.2f__%.2f__%d__%.2f__%.2f\",\n"
                    "    \"ParameterUnits\":\"00__00__S__S__PA__PA__PA__PA__MPA__MPA__PA__PA__G__G__00__00__GS__GS__G__G__00__00__S__S__00__00__00\",\n"
                    "    \"value\":\"\",\n    \"devIP\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"\",\n    \"expand\":\"\",\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, HLPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    idata, jsondata.devId, FRIGEFILLTYPEID, jsondata.orderName, "BatchParameters", g_lasthostlinkrefilldata.amode, g_lasthostlinkrefilldata.bmode, \
                    g_lasthostlinkrefilldata.atesttime, g_lasthostlinkrefilldata.btesttime, g_lasthostlinkrefilldata.aup, g_lasthostlinkrefilldata.bup, \
                    g_lasthostlinkrefilldata.adown, g_lasthostlinkrefilldata.bdown, g_lasthostlinkrefilldata.apressure, g_lasthostlinkrefilldata.bpressure, \
                    g_lasthostlinkrefilldata.avacuumdegree, g_lasthostlinkrefilldata.bvacuumdegree, g_lasthostlinkrefilldata.aset, g_lasthostlinkrefilldata.bset, \
                    g_lasthostlinkrefilldata.apercentage, g_lasthostlinkrefilldata.bpercentage, g_lasthostlinkrefilldata.aspeed, g_lasthostlinkrefilldata.bspeed, \
                    g_lasthostlinkrefilldata.achargeamount, g_lasthostlinkrefilldata.bchargeamount, g_lasthostlinkrefilldata.astatus, g_lasthostlinkrefilldata.bstatus, \
                    g_lasthostlinkrefilldata.achargetime, g_lasthostlinkrefilldata.bchargetime, g_lasthostlinkrefilldata.result, g_lasthostlinkrefilldata.arealsetchargeamount, 
                    g_lasthostlinkrefilldata.brealsetchargeamount, GetStaIp(), DEVTIMEMODE, GetMilliTimeNow());
                } else {
                    (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, HLPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    idata);
                }
            } else if (g_fxplcdataformat == 5) {
                FXPLC_ReadBufferBytes((uint8_t *)rdata, 4);
                (void)sscanf(rdata, "%x", &idata);
                idata >>= g_fxplcbitpos;
                idata &= 0x0001;

                (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, HLPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                idata);
            }
            break;
        }
        case LSPLCDEMODATA: {
            (void)sprintf(frstr, "FR%03d", g_fxplccount);
            if (g_fxplcdataformat == 0) {
                FXPLC_ReadBufferBytes((uint8_t *)rdata, sizeof(rdata));
                (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%s\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, VACUUMTYPEID, LSPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    rdata);
                if (strcmp(rdata, g_lastrdata) != 0) {
                    g_senddata = 1;
                } else {
                    g_senddata = 1;
                }
                strcpy(g_lastrdata, rdata);
            } else if (g_fxplcdataformat == 1) {
                for (i = 0 ; i < 4; i++) {
                    FXPLC_ReadBufferBytes((uint8_t *)&rdata[(i + 2) % 4], 1);
                } 
                (void)sscanf(rdata, "%x", &idata);
                (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, LSPLCDEVTYPEID, GetStaIp(), frstr, jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                    idata);
                g_datapos += 2;
            }
            break;
        }
        case TEMPCONTROLDATA: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, TEMPDEVTYPEID, DEVTYPENAME, GetStaIp(), "FR001", jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                electroData.tempControl.realData);
            break;
        }
        case MOTORDATAVOL: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##"
                "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, MOTORDEVTYPEID, DEVTYPENAME, GetStaIp(), "FR001", jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                electroData.motorData.voltage, g_devId, jsondata.devId, jsondata.devName, MOTORDEVTYPEID, DEVTYPENAME, GetStaIp(), 
                "FR002", jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), electroData.motorData.current);
            break;
        }
        case MOTORDATACUR: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, MOTORDEVTYPEID, DEVTYPENAME, GetStaIp(), "FR002", jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                electroData.motorData.current);
            break;
        }
        case FREEZERDATA: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n    \"isAnswer\":\"no\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, FREEZERDEVTYPEID, DEVTYPENAME, GetStaIp(), "FR001", jsondata.orderName, DEVTIMEMODE, GetMilliTimeNow(), 
                electroData.freezerData.temperature);
            break;
        }
        default: {
            break;
        }
    }
}

#if (defined CONFIG_PLC_FX) || (defined CONFIG_PLC_HOSTLINK) || (defined CONFIG_PLC_LS_LOAD) || (defined CONFIG_TESTER_76T) || (defined CONFIG_TESTER_AINUO)
void uart_init(void) {
    int ret;
    static const char *UART_INIT_TAG = "UART_INIT";

#ifdef CONFIG_PLC_FX
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_7_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#endif

#ifdef CONFIG_PLC_LS_LOAD
    const uart_config_t uart_config = {
        .baud_rate = 9600,   // 115200
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#endif

#ifdef CONFIG_PLC_HOSTLINK
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#endif

#ifdef CONFIG_TESTER_76T
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#endif

#ifdef CONFIG_TESTER_AINUO
    const uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#endif
    
    esp_log_level_set(UART_INIT_TAG, ESP_LOG_ERROR);
    uart_driver_install(UART_NUM_1, UART_BUFF_SIZE * 2, UART_BUFF_SIZE * 2, 20, &uart1_queue, 0);
    uart_param_config(UART_NUM_1, &uart_config);
#ifdef CONFIG_PLC_RS232
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#endif

#ifdef CONFIG_PLC_RS485
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, RTS_PIN, UART_PIN_NO_CHANGE);
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
#endif
    ret = UART_InitBuffer();
    if (ret != 0) {
        ESP_LOGE(UART_INIT_TAG, "uart buffer init failed\n");
    }
}
#endif

// read command
void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    uint32_t sendaddr = (uint32_t)&controlerStr;
    EventBits_t uxBits;

    esp_log_level_set(TX_TASK_TAG, ESP_LOG_ERROR);
    while (1) {
        uxBits = xEventGroupWaitBits(xEventGroup1, BIT_14 | BIT_15 | BIT_16 | BIT_17 | BIT_18 | BIT_19 | BIT_20, pdTRUE, pdFALSE, (TickType_t)10);
        if ((uxBits & BIT_14) != 0) {
#ifdef CONFIG_PLC_FX
            SerialWriteSingleFloatDataRegister(0, 255, 10, 512, g_setsnrefillingmachine.setchargeamount); //  设置加注量
            // SerialReadSingleFloatDataRegister(0, 255, 10, 512, 10);
#endif

#ifdef CONFIG_PLC_HOSTLINK
            if (strcmp(g_setsnrefillingmachine.select, "A") == 0) {
                HLWriteRealDataRegister(8150, g_setsnrefillingmachine.setchargeamount);   // A充注量设定值
                HLReadFloatDataRegister(8150, 26);     // A充注设定值
            } else if ((strcmp(g_setsnrefillingmachine.select, "B")) == 0) {
                HLWriteRealDataRegister(8550, g_setsnrefillingmachine.setchargeamount);   // B充注量设定值
                HLReadFloatDataRegister(8550, 27);     // B充注设定值
            }
           
#endif
        } else if ((uxBits & BIT_15) != 0) {
#ifdef CONFIG_TESTER_76T
            TTesterSetGroupPara(GetGroupIdFromRecvJsonData());
#endif

#ifdef CONFIG_TESTER_AINUO
            AINUO_TTesterSetGroupPara(GetGroupIdFromRecvJsonData());
#endif
        } else if ((uxBits & BIT_16) != 0) {
            ParseOpCode(controlerStr, SWITCHCOUNT);
            if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                ESP_LOGE(TX_TASK_TAG, "Error occurred during sending queue: switch count event");
            }
        } else if ((uxBits & BIT_17) != 0) {
            ParseOpCode(controlerStr, SWITCHSTATUS);
            if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                ESP_LOGE(TX_TASK_TAG, "Error occurred during sending queue: switch static event");
            }
        } else if ((uxBits & BIT_18) != 0) {
            ParseOpCode(controlerStr, TEMPCONTROLDATA);
            if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                ESP_LOGE(TX_TASK_TAG, "Error occurred during sending queue: tempcontroldata event");
            }
        } else if ((uxBits & BIT_19) != 0) {
            ParseOpCode(controlerStr, MOTORDATAVOL);
            if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                ESP_LOGE(TX_TASK_TAG, "Error occurred during sending queue: motordatavol event");
            }
        } else if ((uxBits & BIT_20) != 0) {
            ParseOpCode(controlerStr, FREEZERDATA);
            if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                ESP_LOGE(TX_TASK_TAG, "Error occurred during sending queue: freezerdata event");
            }
        }
    }
    vTaskDelete(NULL);
}

void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    uint32_t sendaddr = (uint32_t)&controlerStr;
    int ret = 0;

    TickType_t xLastWakeTime;
#ifdef CONFIG_TESTER_AINUO
 	const TickType_t xFrequency = 100;
#else
    const TickType_t xFrequency = 10;
#endif
    
    xLastWakeTime = xTaskGetTickCount();
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_ERROR);
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
#ifdef CONFIG_PLC_FX
    #ifdef CONFIG_PLC_RS232
        ret = GetDataFromFxPlc();
    #endif

    #ifdef CONFIG_PLC_RS485
        ret = GetSerialDataFromFxPlc();
    #endif
#endif

#ifdef CONFIG_PLC_HOSTLINK
        ret = GetSerialWordDataFromHlPlc();
#endif

#ifdef CONFIG_PLC_LS_LOAD
    #ifdef CONFIG_PLC_RS232
        // ret = LSLoadGetSerialWordDataFromFxPlc();
        ret = DBSGetData();
    #endif
#endif

#ifdef CONFIG_TESTER_76T
        ret = TTesterResolve();
#endif

#ifdef CONFIG_TESTER_AINUO
        ret = AINUO_TTesterResolve();
#endif

// Parse Data
        if (ret == 0) {
#ifdef CONFIG_PLC_FX
            SendAckToPlc();
            ParseOpCode(controlerStr, FXPLCDEMODATA);
#endif

#ifdef CONFIG_PLC_HOSTLINK
            ParseOpCode(controlerStr, HLPLCDEMODATA);
#endif

#ifdef CONFIG_PLC_LS_LOAD
            ParseOpCode(controlerStr, LSPLCDEMODATA);
#endif

#ifdef CONFIG_TESTER_76T
            TTesterGetJsonData(controlerStr);
#endif

#ifdef CONFIG_TESTER_AINUO
            AINUO_TTesterGetJsonData(controlerStr);
#endif
            if (g_senddata == 1) {
                ESP_LOGI(RX_TASK_TAG, "Read bytes: '%s'", controlerStr);
                if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                    ESP_LOGE(RX_TASK_TAG, "Error occurred during sending queue");
                }
            }
		}
#ifdef CONFIG_PLC_FX
        else if (ret != 0) {
            SendNackToPlc();
        }
#endif
    }
    vTaskDelete(NULL);
}

void uart_event_task(void *pvParameters)
{
    static const char *UART_EVENT_TASK_TAG = "UART_EVENT_TASK";
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);
    int ret;
    // esp_log_level_set(UART_EVENT_TASK_TAG, ESP_LOG_ERROR);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RX_BUF_SIZE);
            ESP_LOGI(UART_EVENT_TASK_TAG, "uart[%d] event:", UART_NUM_1);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(UART_EVENT_TASK_TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM_1, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(UART_EVENT_TASK_TAG, "[DATA EVT]: %s", dtmp);
                    ret = UART_WriteBufferBytes(dtmp, event.size);
                    if (ret != 0) {
                        ESP_LOGE(UART_EVENT_TASK_TAG, "uart buffer error: %d", ret);
                    }
                    // uart_write_bytes(UART_NUM_1, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(UART_EVENT_TASK_TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(UART_EVENT_TASK_TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(UART_EVENT_TASK_TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(UART_EVENT_TASK_TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(UART_EVENT_TASK_TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(UART_NUM_1, &buffered_size);
                    int pos = uart_pattern_pop_pos(UART_NUM_1);
                    ESP_LOGI(UART_EVENT_TASK_TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(UART_NUM_1);
                    } else {
                        uart_read_bytes(UART_NUM_1, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(UART_NUM_1, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(UART_EVENT_TASK_TAG, "read data: %s", dtmp);
                        ESP_LOGI(UART_EVENT_TASK_TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(UART_EVENT_TASK_TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}
