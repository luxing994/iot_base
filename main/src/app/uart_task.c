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

#define CONTROLERTYPE 2
#define PATTERN_CHR_NUM    (3) 
#define RX_BUF_SIZE  (UART_BUFF_SIZE * 2)
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)
#define RTS_PIN (GPIO_NUM_8)
#define DEVIDLENGTH 12

static QueueHandle_t uart1_queue;
static const char *TAG = "uart_events";
HproFuncCode funcCode;
CommandJsonData jsondata;
// HproOpReadCode opCode;
char controlerStr[1024] = {0};
char sendDataBuffer[22][16] = { { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x01 }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x02 },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x03 }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x04 },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x05 }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x06 },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x07 }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x08 },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x09 }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0A },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0B }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0C },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0D }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0E }, 
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0F }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x10 },
                               { 0x5A, 0xA5, 0x00, 0x04, 0x02, 0x07 }, { 0x5A, 0xA5, 0x00, 0x04, 0x02, 0x08 },
                               { 0x5A, 0xA5, 0x00, 0x04, 0x02, 0x09 }, { 0x5A, 0xA5, 0x00, 0x04, 0x02, 0x0A },
                               { 0x5A, 0xA5, 0x00, 0x04, 0x02, 0x0B }, { 0x5A, 0xA5, 0x00, 0x03, 0x02, 0x0C } };
char sendFileDataBuffer[6][256] = { { 0x5A, 0xA5, 0x00, 0xF2, 0x02, 0x01 }, { 0x5A, 0xA5, 0x00, 0xF2, 0x02, 0x02 }, 
                                    { 0x5A, 0xA5, 0x00, 0xF2, 0x02, 0x03 }, { 0x5A, 0xA5, 0x00, 0xF2, 0x02, 0x04 },
                                    { 0x5A, 0xA5, 0x00, 0xF2, 0x02, 0x05 }, { 0x5A, 0xA5, 0x00, 0xF2, 0x02, 0x06 } };
                               
int sendflag = 0;
HproComFrame dataFrame;
char g_devId[32] = {"FX_PLC_001"};
RingBuffer uart2Buffer;
HproComFrame dataFrame = {0};
uint32_t g_devStartStatus = 0;
int g_rdatalen = 1;
int g_senddata = 1;
int g_fxplccount = 0;
int g_fxplcdataformat = 0;   // (0:char *    1:int      2:float)   
uint16_t g_lastdata[11] = {0};
SNCaclReFillingMachine g_lastrefilldata = {0};
int g_datapos = 0;

void ParseOpCode(char *str, uint8_t op)
{
    char rdata[5] = {0};
    char rfdata[9] = {0};
    int idata, i;
    float fdata;
    jsondata = GetCommandJsonData();
    char version[5] = {0};
    char frstr[10] = {0};
    uint8_t temp;
    switch (op) {
        case BREAK: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;
        }
        case HMISTATUS: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;
        }
        case MODE: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0]);
            break;
        }
        case COUNT: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;
        }
        case SCHEDULE: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0] << 8 | dataFrame.data[1]);
            break; 
        }
        case PATTERN: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;  
        }
        case PITCH: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                (dataFrame.data[0] << 8 | dataFrame.data[1]) / 10.0);
            break;  
        }
        case PITCHCOUNT: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;
        }
        case SPINDLERATE: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;
        }
        case BOOTTIME: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d;%d;%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0], dataFrame.data[1], dataFrame.data[2]);
            break;
        }
        case APPVERSION: {
            memcpy(version, (char *)(&(dataFrame.data[0])), VERSION_SIZE);
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%s\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
               version);
            break; 
        }
        case CONTROLVERSION: {
            memcpy(version, (char *)(&(dataFrame.data[0])), VERSION_SIZE);
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%s\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                version);
            break;   
        }
        case MECHANICCALL: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0]);
            break;
        }
        case MATERIALCALL: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0]);
            break;
        }
        case OTHERCALL: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                dataFrame.data[0]);
            break;
        }
        case SYSTEMID: {
            g_devStartStatus = 1;
            memcpy(g_devId, &dataFrame.data[0], DEVIDLENGTH);
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%s\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                g_devId);
            break;
        }
        case SWITCHCOUNT: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                GetSwitchCount());
            break;
        }
        case SWITCHSTATUS: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, jsondata.devTypeId, DEVTYPENAME, GetStaIp(), jsondata.orderId, jsondata.orderName, GetMilliTimeNow(), 
                GetSwitchLevel());
            break;
        }
        case FXPLCDEMODATA: {
            (void)sprintf(frstr, "FR%03d", g_fxplccount);
            if (g_fxplcdataformat == 0) {
                FXPLC_ReadBufferBytes((uint8_t *)rdata, sizeof(rdata));
                    // if (rdata != g_lastdata[g_fxplccount]) {
                    //     g_senddata = 1;
                    // } else {
                    //     g_senddata = 0;
                    // }
                (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%s\",\n    \"expand\":\"NULL\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, DEVTYPENAME, GetStaIp(), frstr, jsondata.orderName, GetMilliTimeNow(), 
                    rdata);
            } else if (g_fxplcdataformat == 1) {
                FXPLC_ReadBufferBytes((uint8_t *)rdata, 4);
                    // if (rdata != g_lastdata[g_fxplccount]) {
                    //     g_senddata = 1;
                    // } else {
                    //     g_senddata = 0;
                    // }
                (void)sscanf(rdata, "%x", &idata);
                if (idata != *(uint16_t *)(&((uint8_t *)&g_lastrefilldata)[g_datapos])) {
                    g_senddata = 1;
                } else {
                    g_senddata = 0;
                }
                memcpy(&((uint8_t *)&g_lastrefilldata)[g_datapos], (uint16_t* )&idata, sizeof(uint16_t));
                (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, DEVTYPENAME, GetStaIp(), frstr, jsondata.orderName, GetMilliTimeNow(), 
                    idata);
                g_datapos += 2;
                if (g_datapos >= sizeof(SNCaclReFillingMachine)) {
                    g_datapos = 0;
                }
            } else if (g_fxplcdataformat == 2) {
                for (i = 0; i < 8; i++) {
                    FXPLC_ReadBufferBytes((uint8_t *)&rfdata[(i + 4) % 8], 1);
                }
                // if (rdata != g_lastdata[g_fxplccount]) {
                //     g_senddata = 1;
                // } else {
                //     g_senddata = 0;
                // }
                (void)sscanf(rfdata, "%x", &idata);
                fdata = *((float *)&idata);
                if (fdata != *(float *)(&((uint8_t *)&g_lastrefilldata)[g_datapos])) {
                    g_senddata = 1;
                } else {
                    g_senddata = 0;
                }
                memcpy(&((uint8_t *)&g_lastrefilldata)[g_datapos], &fdata, sizeof(float));
                (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                    "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                    "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
                    "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.2f\",\n    \"expand\":\"NULL\"\n};;**##", \  
                    g_devId, jsondata.devId, jsondata.devName, FRIGEFILLTYPEID, DEVTYPENAME, GetStaIp(), frstr, jsondata.orderName, GetMilliTimeNow(), 
                    fdata);
                g_datapos += 4;
                if (g_datapos >= sizeof(SNCaclReFillingMachine)) {
                    g_datapos = 0;
                }
            }
            // if (g_fxplccount < (sizeof(g_lastdata) / sizeof(uint16_t))) {
            //       g_lastdata[g_fxplccount] = rdata;
            // }
            break;
        }
        case TEMPCONTROLDATA: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, TEMPDEVTYPEID, DEVTYPENAME, GetStaIp(), "FR001", jsondata.orderName, GetMilliTimeNow(), 
                electroData.tempControl.realData);
            break;
        }
        case MOTORDATAVOL: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n};;**##"
                "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, MOTORDEVTYPEID, DEVTYPENAME, GetStaIp(), "FR001", jsondata.orderName, GetMilliTimeNow(), 
                electroData.motorData.voltage, g_devId, jsondata.devId, jsondata.devName, MOTORDEVTYPEID, DEVTYPENAME, GetStaIp(), 
                "FR002", jsondata.orderName, GetMilliTimeNow(), electroData.motorData.current);
            break;
        }
        case MOTORDATACUR: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, MOTORDEVTYPEID, DEVTYPENAME, GetStaIp(), "FR002", jsondata.orderName, GetMilliTimeNow(), 
                electroData.motorData.current);
            break;
        }
        case FREEZERDATA: {
            (void)sprintf(str, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n};;**##", \ 
            g_devId, jsondata.devId, jsondata.devName, FREEZERDEVTYPEID, DEVTYPENAME, GetStaIp(), "FR001", jsondata.orderName, GetMilliTimeNow(), 
                electroData.freezerData.temperature);
            break;
        }
        default: {
            break;
        }
    }
}

int GetDataFromControler(void)
{
	uint8_t curData = 0;
	uint8_t lastData = 0;
    uint16_t length;
    uint16_t crc;
	int ret;
    static const char *GET_DATA_TAG = "GET_DATA_TASK";
    esp_log_level_set(GET_DATA_TAG, ESP_LOG_INFO);
	while (!((curData == HPRO_HEAD_SECOND_BYTE) && (lastData == HPRO_HEAD_FIRST_BYTE))) {
		lastData = curData;
		ret = UART_ReadBufferBytes(&curData, 1);
		if (ret != 0) {
			return -1;
		}
	}
    dataFrame.head[0] = HPRO_HEAD_FIRST_BYTE;
    dataFrame.head[1] = HPRO_HEAD_SECOND_BYTE;

    ret = UART_ReadBufferBytes(&dataFrame.length[0], 2);
    if (ret != 0) {
        return -1;
    }

    length = (dataFrame.length[0] << 8) | dataFrame.length[1];
    ESP_LOGI(GET_DATA_TAG, "read data length:%d\n", length);

    ret += UART_ReadBufferBytes(&dataFrame.func, 1);
	ret += UART_ReadBufferBytes(&dataFrame.operate, 1);
    ret += UART_ReadBufferBytes(&dataFrame.data[0], length - 2);
    if (ret != 0) {
        return -1;
    }

    ret = UART_ReadBufferBytes(&dataFrame.crc[0], 2);
    if (ret != 0) {
        return -1;
    }
    
    // // CRC16 check
    // if (CheckCRC16((uint8_t *)&dataFrame.head[0], length + 4, (dataFrame.crc[1] << 8) | dataFrame.crc[0]) != 0) {
    //     crc = crc16bitbybit((uint8_t *)&dataFrame.head[0], length + 4);
    //     ESP_LOGI(GET_DATA_TAG, "rec crc:%x%x, cal crc:%x%x", dataFrame.crc[0], dataFrame.crc[1], crc >> 8, crc & 0xff);
    //     return -1;
    // }

	return 0;
}

#if (defined CONFIG_PLC_FX) || (defined CONFIG_PLC_HOSTLINK) || (defined CONFIG_PLC_LS_LOAD) || (defined CONFIG_TESTER_76T)
void uart_init(void) {
    int ret;
    static const char *TAG = "uart_init";

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
        .baud_rate = 9600,
        .data_bits = UART_DATA_7_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
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
        ESP_LOGE(TAG, "uart buffer init failed\n");
    }
}
#endif

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}


// read command
void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    uint32_t sendaddr = (uint32_t)&controlerStr;
    EventBits_t uxBits;
    uint32_t recvp;
    uint16_t crc;

    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        uxBits = xEventGroupWaitBits(xEventGroup1, BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7 | BIT_8 \
            | BIT_9 | BIT_10 | BIT_11 | BIT_12 | BIT_13 | BIT_14 | BIT_15 | BIT_16 | BIT_17 | BIT_18 | BIT_19 | BIT_20, pdTRUE, pdFALSE, (TickType_t)10);
        if ((uxBits & BIT_0) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[0], 6);
            memcpy(&sendDataBuffer[0][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[0], 8);
        } else if ((uxBits & BIT_1) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[1], 6);
            memcpy(&sendDataBuffer[1][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[1], 8);
        } else if ((uxBits & BIT_2) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[2], 6);
            memcpy(&sendDataBuffer[2][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[2], 8);
        } else if ((uxBits & BIT_3) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[3], 6);
            memcpy(&sendDataBuffer[3][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[3], 8);
        } else if ((uxBits & BIT_4) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[4], 6);
            memcpy(&sendDataBuffer[4][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[4], 8);
        } else if ((uxBits & BIT_5) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[5], 6);
            memcpy(&sendDataBuffer[5][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[5], 8);
        } else if ((uxBits & BIT_6) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[6], 6);
            memcpy(&sendDataBuffer[6][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[6], 8);
        } else if ((uxBits & BIT_7) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[7], 6);
            memcpy(&sendDataBuffer[7][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[7], 8);
        } else if ((uxBits & BIT_8) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[8], 6);
            memcpy(&sendDataBuffer[8][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[8], 8);
        } else if ((uxBits & BIT_9) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[9], 6);
            memcpy(&sendDataBuffer[9][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[9], 8);
        } else if ((uxBits & BIT_10) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[10], 6);
            memcpy(&sendDataBuffer[10][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[10], 8);
        } else if ((uxBits & BIT_11) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[11], 6);
            memcpy(&sendDataBuffer[11][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[11], 8);
        } else if ((uxBits & BIT_12) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[12], 6);
            memcpy(&sendDataBuffer[12][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[12], 8);
        } else if ((uxBits & BIT_13) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[13], 6);
            memcpy(&sendDataBuffer[13][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[13], 8);
        } else if ((uxBits & BIT_14) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[14], 6);
            memcpy(&sendDataBuffer[14][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[14], 8);
        } else if ((uxBits & BIT_15) != 0) {
            // crc = crc16bitbybit((uint8_t *)sendDataBuffer[15], 6);
            // memcpy(&sendDataBuffer[15][6], &crc, 2);
            // uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[15], 8);
            TTesterSetGroupPara(1);
        } else if ((uxBits & BIT_16) != 0) {
            ParseOpCode(controlerStr, SWITCHCOUNT);
            if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                //TO DO
            }
        } else if ((uxBits & BIT_17) != 0) {
            ParseOpCode(controlerStr, SWITCHSTATUS);
            if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                //TO DO
            }
        } else if ((uxBits & BIT_18) != 0) {
            ParseOpCode(controlerStr, TEMPCONTROLDATA);
            if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                //TO DO
            }
        } else if ((uxBits & BIT_19) != 0) {
            ParseOpCode(controlerStr, MOTORDATAVOL);
            if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                //TO DO
            }
        } else if ((uxBits & BIT_20) != 0) {
            ParseOpCode(controlerStr, FREEZERDATA);
            if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                //TO DO
            }
        }
    }
    vTaskDelete(NULL);
}

// write command
void tx1_task(void *arg)
{
    static const char *TX1_TASK_TAG = "TX1_TASK";
    EventBits_t uxBits;
    uint32_t recvp;
    uint16_t crc;
    uint16_t data;
    int temp;

    esp_log_level_set(TX1_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        uxBits = xEventGroupWaitBits(xEventGroup2, BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7 | BIT_8 \
            | BIT_9 | BIT_10 | BIT_11, pdTRUE, pdFALSE, (TickType_t)10);
        if ((uxBits & BIT_0) != 0) {
            GetFileData((uint8_t *)&sendFileDataBuffer[0][6], FILETRANSSIZE);
            crc = crc16bitbybit((uint8_t *)sendFileDataBuffer[0], FILETRANSSIZE + 6);
            memcpy(&sendFileDataBuffer[0][FILETRANSSIZE + 6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendFileDataBuffer[0], FILETRANSSIZE + 8);
        } else if ((uxBits & BIT_6) != 0) {
            GetTaskNum(&data);
            ESP_LOGI(TX1_TASK_TAG, "tasknum:%d\n", data);
            GetTaskNum((uint8_t *)&sendDataBuffer[16][6]);
            temp = sendDataBuffer[16][6];
            sendDataBuffer[16][6] = sendDataBuffer[16][7];
            sendDataBuffer[16][7] = temp;
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[16], 2 + 6);
            memcpy(&sendDataBuffer[16][2 + 6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[16], 2 + 8);
        } else if ((uxBits & BIT_7) != 0) {
            GetTaskPitch(&data);
            ESP_LOGI(TX1_TASK_TAG, "taskpitch:%d\n", data);
            GetTaskPitch((uint8_t *)&sendDataBuffer[17][6]);
            temp = sendDataBuffer[17][6];
            sendDataBuffer[17][6] = sendDataBuffer[17][7];
            sendDataBuffer[17][7] = temp;
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[17], 2 + 6);
            memcpy(&sendDataBuffer[17][2 + 6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[17], 2 + 8);
        } else if ((uxBits & BIT_8) != 0) {
            GetTaskSpeed(&data);
            ESP_LOGI(TX1_TASK_TAG, "taskspeed:%d\n", data);
            GetTaskSpeed((uint8_t *)&sendDataBuffer[18][6]);
            temp = sendDataBuffer[18][6];
            sendDataBuffer[18][6] = sendDataBuffer[18][7];
            sendDataBuffer[18][7] = temp;
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[18], 2 + 6);
            memcpy(&sendDataBuffer[18][2 + 6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[18], 2 + 8);
        } else if ((uxBits & BIT_9) != 0) {
            GetTaskCount(&data);
            ESP_LOGI(TX1_TASK_TAG, "taskcount:%d\n", data);
            GetTaskCount((uint8_t *)&sendDataBuffer[19][6]);
            temp = sendDataBuffer[19][6];
            sendDataBuffer[19][6] = sendDataBuffer[19][7];
            sendDataBuffer[19][7] = temp;
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[19], 2 + 6);
            memcpy(&sendDataBuffer[19][2 + 6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[19], 2 + 8);
        } else if ((uxBits & BIT_10) != 0) {
            GetTaskTime(&data);
            ESP_LOGI(TX1_TASK_TAG, "tasktime:%d\n", data);
            GetTaskTime((uint8_t *)&sendDataBuffer[20][6]);
            temp = sendDataBuffer[20][6];
            sendDataBuffer[20][6] = sendDataBuffer[20][7];
            sendDataBuffer[20][7] = temp;
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[20], 2 + 6);
            memcpy(&sendDataBuffer[20][2 + 6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[20], 2 + 8);
        } else if ((uxBits & BIT_11) != 0) {
            GetMode(&data);
            ESP_LOGI(TX1_TASK_TAG, "mode:%d\n", (uint8_t)data);
            GetMode((uint8_t *)&sendDataBuffer[21][6]);
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[21], 1 + 6);
            memcpy(&sendDataBuffer[21][1 + 6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[21], 1 + 8);
        }
    }
    vTaskDelete(NULL);
}

void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    uint32_t sendaddr = (uint32_t)&controlerStr;
    uint16_t parameter = 0;
    uint8_t data;
    int ret = 0, i;

    TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 10;
    
    xLastWakeTime = xTaskGetTickCount();
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // ret = GetDataFromControler();
#ifdef CONFIG_PLC_FX
    #ifdef CONFIG_PLC_RS232
        ret = GetDataFromFxPlc(&g_rdatalen);
    #endif

    #ifdef CONFIG_PLC_RS485
        ret = GetSerialDataFromFxPlc(&g_rdatalen);
    #endif
#endif

#ifdef CONFIG_PLC_LS_LOAD
    #ifdef CONFIG_PLC_RS232
        // ret = LSLoadGetSerialWordDataFromFxPlc(&g_rdatalen);
        ret = DBSGetData();
    #endif
#endif

#ifdef CONFIG_PLC_HOSTLINK
        ret = GetSerialWordDataFromHlPlc(&g_rdatalen);
#endif

#ifdef CONFIG_TESTER_76T
        ret = TTesterResolve();
#endif
		// if (ret == 0 && g_rdatalen != 0) {
        if (ret == 0) {
            // SendAckToPlc();
            // ESP_LOGI(RX_TASK_TAG, "Read bytes length: '%d'", g_rdatalen);
            // for (i = 0; i < g_rdatalen; i++) {
                // ParseOpCode(controlerStr, dataFrame.operate);
#ifdef CONFIG_TESTER_76T
                TTesterGetJsonData(controlerStr);
#else
                ParseOpCode(controlerStr, FXPLCDEMODATA);
#endif
                if (g_senddata == 1) {
                    ESP_LOGI(RX_TASK_TAG, "Read bytes: '%s'", controlerStr);
                    if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                        //TO DO
                    }
                // }
                // vTaskDelay(10);
            }
		} else if (ret == 0 && g_rdatalen == 0) {
            SendAckToPlc();
        }
    }
    vTaskDelete(NULL);
}

void uart_event_task(void *pvParameters)
{
    static const char *UART_EVENT_TASK_TAG = "UART_EVENT_TASK";
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);
    esp_log_level_set(UART_EVENT_TASK_TAG, ESP_LOG_ERROR);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RX_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_NUM_1);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM_1, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]: %s", dtmp);
                    UART_WriteBufferBytes(dtmp, event.size);
                    // uart_write_bytes(UART_NUM_1, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(UART_NUM_1, &buffered_size);
                    int pos = uart_pattern_pop_pos(UART_NUM_1);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
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
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}