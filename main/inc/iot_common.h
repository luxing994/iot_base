#ifndef IOT_COMMON_H
#define IOT_COMMON_H

#include "sdkconfig.h"
#include "ringbuffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

extern QueueHandle_t xQueue1;
extern EventGroupHandle_t xEventGroup1;
extern EventGroupHandle_t xEventGroup2;
extern EventGroupHandle_t xEventGroup3;

#define UART_BUFF_SIZE 1024
#define FXPLC_BUFF_SIZE 1024

// 上报队列：元素为 char *（堆上的 JSON 副本），消费者负责 free
#define IOT_MSG_QUEUE_LEN      32
#define IOT_MSG_QUEUE_TIMEOUT  pdMS_TO_TICKS(100)

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
#define DEVID        CONFIG_IOT_DEVICE_ID   // menuconfig -> Device Identity
#define DEVNAME      "Hello"
#define DEVTYPEID    "Hello"
#define DEVTYPENAME  "Hello"
#define ORDERID      "Hello"
#define ORDERNAME    "Hello"
#define INITORDERID  "FR000"
#define FILETRANSSIZE 240
#define DEVTIMEMODE  "yes"

// device type
#define MOTORDEVTYPEID     "ZL"
#define TEMPDEVTYPEID      "WK"
#define FREEZERDEVTYPEID   "LD"

#define FRIGEFILLTYPEID    "LM"
#define SAFETYTESTERTYPEID "ST"
#define VACUUMTYPEID       "ZK"


// device plc type
#define FXPLCDEVTYPEID    "FXPLC"
#define LSPLCDEVTYPEID    "LSPLC"
#define HLPLCDEVTYPEID    "HLPLC"
#define MDPLCDEVTYPEID    "MDPLC"
#define PPIPLCDEVTYPEID   "PPIPLC"

#define PLC_DATA_TYPE_CHAR_POINT_OR_BYTE_FOR_PPI 0
#define PLC_DATA_TYPE_SHORT                      1
#define PLC_DATA_TYPE_FLOAT                      2
#define PLC_DATA_TYPE_BCD                        3
#define PLC_DATA_TYPE_INT                        4
#define PLC_DATA_TYPE_BIT                        5

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

// 命令字段统一用定长数组保存副本，不再指向 cJSON 树（原实现解析后从不释放 cJSON，
// 且字段缺失时为 NULL，会导致 strcmp/atoll 空指针）
#define CMD_STR_LEN        48
#define CMD_SHORT_STR_LEN  16
#define CMD_PARA_GROUPS    9
#define CMD_PARA_ITEMS     20

typedef struct {
    char type[CMD_SHORT_STR_LEN];
    char value[CMD_SHORT_STR_LEN];
} parametersData;

typedef struct {
    char devId[CMD_STR_LEN];
    char devName[CMD_STR_LEN];
    char devTypeId[CMD_STR_LEN];
    char deviceOrderFile[CMD_STR_LEN];
    char deviceOrderMode[CMD_SHORT_STR_LEN];
    char groupId[CMD_SHORT_STR_LEN];
    char deviceOrderWay[CMD_SHORT_STR_LEN];
    char orderDate[CMD_STR_LEN];
    char orderId[CMD_STR_LEN];
    char orderName[CMD_STR_LEN];
    char parameterType[CMD_STR_LEN];
    parametersData paradata[CMD_PARA_GROUPS][CMD_PARA_ITEMS];
    char responseType[CMD_SHORT_STR_LEN];
    char timeStamp[CMD_STR_LEN];
} CommandJsonData;

// 电镀厂温控仪表、整流机和冷冻机数据
typedef struct {
    int dPt;      // 小数点位置
    int PV;       // 测量值
    int SV;       // 设定值
} TempControlTransData;

typedef struct {
    float setData;
    float realData;
} TempControlData;

typedef struct {
    float current;
    float voltage;
} MotorData;

typedef struct {
    float temperature;
} FreezerData;

typedef struct {
    int status;
} AirSwitchData;

typedef struct {
    TempControlData tempControl;
    MotorData motorData;
    FreezerData freezerData;
    AirSwitchData switchData;
} ElectroFactoryData;

// 生能1-4车间冷媒灌注机设置数据格式
// （采集数据的点表见 plc_report.c 中的 s_snBatchPoints）
typedef struct {
    char select[CMD_SHORT_STR_LEN];
    float setchargeamount;
} SNSetCaclReFillingMachine;

extern int g_fxplccount;
extern int g_fxplcdataformat;
extern int g_fxplcbitpos;
extern int g_modbusplccount;
extern int g_modbusplcdataformat;
extern int g_ppiplccount;
extern int g_ppiplcdataformat;
extern int g_hostlinkplccount;
extern int g_hostlinkplcdataformat;
extern int g_hostlinkplcbitpos;
extern int g_lsplccount;
extern int g_lsplcdataformat;
extern uint64_t g_baseTime;

uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len);
int CalFloatStrPointPos(char *str);
int CheckCRC16(uint8_t *ptr, uint16_t len, uint16_t rcrc);
uint8_t CalFCS(uint8_t* pbuff, uint16_t len);
uint16_t CalSumCheckData(uint8_t *data, uint16_t len);
uint8_t CalSumCheckDataLow(uint8_t *data, uint16_t len);
int CheckSumData(uint8_t *data, uint16_t len, uint16_t checksum);
uint16_t CalReadDataRegister(uint8_t *data);
uint16_t CalSerialReadDataRegister(uint8_t *data);
int BCDToInt(int bcd);
int UART_InitBuffer(void);
int UART_WriteBufferBytes(uint8_t *data, uint32_t size);
int UART_ReadBufferBytes(uint8_t *data, uint32_t size);
int FXPLC_InitBuffer(void);
int FXPLC_WriteBufferBytes(uint8_t *data, uint32_t size);
int FXPLC_ReadBufferBytes(uint8_t *data, uint32_t size);
int MODBUSPLC_InitBuffer(void);
int MODBUSPLC_WriteBufferBytes(uint8_t *data, uint32_t size);
int MODBUSPLC_ReadBufferBytes(uint8_t *data, uint32_t size);
int PPIPLC_InitBuffer(void);
int PPIPLC_WriteBufferBytes(uint8_t *data, uint32_t size);
int PPIPLC_ReadBufferBytes(uint8_t *data, uint32_t size);
int get_random_range_freq(int min_val, int max_val);
int IOT_PostMessage(const char *msg);

#endif
