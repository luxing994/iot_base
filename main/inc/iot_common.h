#ifndef IOT_COMMON_H
#define IOT_COMMON_H

#include "ringbuffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

extern QueueHandle_t xQueue1;
extern EventGroupHandle_t xEventGroup1;
extern EventGroupHandle_t xEventGroup2;

#define UART_BUFF_SIZE 1024
#define FXPLC_BUFF_SIZE 1024

#define BIT_0	( 1 << 0 )
#define BIT_1	( 1 << 1 )
#define BIT_2	( 1 << 2 )
#define BIT_3	( 1 << 3 )
#define BIT_4	( 1 << 4 )
#define BIT_5	( 1 << 5 )
#define BIT_6	( 1 << 6 )
#define BIT_7	( 1 << 7 )
#define BIT_8	( 1 << 8 )
#define BIT_9	( 1 << 9 )
#define BIT_10	( 1 << 10 )
#define BIT_11	( 1 << 11 )
#define BIT_12	( 1 << 12 )
#define BIT_13	( 1 << 13 )
#define BIT_14	( 1 << 14 )
#define BIT_15	( 1 << 15 )
#define BIT_16	( 1 << 16 )
#define BIT_17	( 1 << 17 )
#define BIT_18	( 1 << 18 )
#define BIT_19	( 1 << 19 )
#define BIT_20	( 1 << 20 )
#define BIT_21	( 1 << 21 )
#define BIT_22	( 1 << 22 )
#define BIT_23	( 1 << 23 )
#define BIT_24	( 1 << 24 )
#define BIT_25	( 1 << 25 )
#define BIT_26	( 1 << 26 )
#define BIT_27	( 1 << 27 )
#define BIT_28	( 1 << 28 )
#define BIT_29	( 1 << 29 )
#define BIT_30	( 1 << 30 )
#define BIT_31	( 1 << 31 )

#define ID           "123"
#define DEVID        "Hello"
#define DEVNAME      "1111"
#define DEVTYPEID    "222"
#define DEVTYPENAME  "Hello"
#define INITORDERID  "FR000"

#define FILETRANSSIZE 240

// 命令接收JSON格式
/*
{
    "devId": "1531243721197228032",
    "devName": "",
    "devTypeId": "1516606339298758656",
    "deviceOrderFile": "",
    "deviceOrderMode": "",
    "deviceOrderWay": "write",
    "orderDate": "2022-05-30 21:46:21",
    "orderId": "FR022",
    "orderName": "",
    "parameterType": "",
    "parameters": [{"type":"","value":"2000"},{"type":"","value":"2000"},
    {"type":"","value":"2000"},{"type":"","value":"2000"},
    {"type":"","value":"2000"},{"type":"","value":"1"}],
    "responseType": "",
    "timeStamp": "1653918381719"
}
*/

// 乐江机械罗拉车配置项
struct ConfigData {
    uint16_t tasknum;
    uint16_t taskpitch;
    uint16_t taskspeed;
    uint16_t taskcount;
    uint16_t tasktime;
    uint8_t mode;
};

typedef struct {
    char *type;
    char *value;
} parametersData;

typedef struct {
    char *devId;
    char *devName;
    char *devTypeId;
    char *deviceOrderFile;
    char *deviceOrderMode;
    char *deviceOrderWay;
    char *orderDate;
    char *orderId;
    char *orderName;
    char *parameterType;
    parametersData paradata[10];
    char *responseType;
    char *timeStamp;
} CommandJsonData;

uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len);
int CheckCRC16(uint8_t *ptr, uint16_t len, uint16_t rcrc);
uint16_t CalSumCheckData(uint8_t *data, uint16_t len);
int CheckSumData(uint8_t *data, uint16_t len, uint16_t checksum);
int UART_InitBuffer(void);
int UART_WriteBufferBytes(uint8_t *data, uint32_t size);
int UART_ReadBufferBytes(uint8_t *data, uint32_t size);

#endif
