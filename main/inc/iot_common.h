#ifndef IOT_COMMON_H
#define IOT_COMMON_H

#include "sdkconfig.h"
#include "ringbuffer.h"
#include "gw_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

extern QueueHandle_t xQueue1;
extern EventGroupHandle_t xEventGroup1;
extern EventGroupHandle_t xEventGroup2;
extern EventGroupHandle_t xEventGroup3;

#define UART_BUFF_SIZE 1024

// 上报队列：元素为 char *（堆上的 JSON），由 tcp_client_task 发送后 free；满时丢弃最旧的一条
#define IOT_MSG_QUEUE_LEN      32


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

/* xEventGroup1 中冷媒充注量设定的请求与结果（host_cmd <-> cmd_task） */
#define EVT_CHARGE_REQUEST  BIT_14      // host_cmd -> cmd_task：执行写入
#define EVT_CHARGE_OK       BIT_18      // cmd_task -> host_cmd：写入并回读一致
#define EVT_CHARGE_FAIL     BIT_19      // cmd_task -> host_cmd：写入失败、回读失败或不一致

#define ID           "123"
#define DEVID        (GwConfig_Get()->devId)    // 运行参数（gw_config.h），出厂默认值见 menuconfig
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

// 上位机命令字段长度与 group 命令参数表尺寸（命令处理见 host_cmd.h）
#define CMD_STR_LEN        48
#define CMD_SHORT_STR_LEN  16
#define CMD_PARA_GROUPS    9
#define CMD_PARA_ITEMS     20

typedef struct {
    char type[CMD_SHORT_STR_LEN];
    char value[CMD_SHORT_STR_LEN];
} parametersData;


// 生能1-4车间冷媒灌注机设置数据格式
// （采集数据的点表见 plc_report.c 中的 s_snBatchPoints）
typedef struct {
    char select[CMD_SHORT_STR_LEN];
    float setchargeamount;
} SNSetCaclReFillingMachine;



/* UART1 接收缓冲（事务模式，见 serial_txn.c） */
int UART_InitBuffer(void);
int UART_WriteBufferBytes(uint8_t *data, uint32_t size);
int UART_ReadBufferBytes(uint8_t *data, uint32_t size);
void UART_CommitBuffer(void);
void UART_RollbackBuffer(void);
void UART_FlushBuffer(void);
uint32_t UART_BufferedBytes(void);

int get_random_range_freq(int min_val, int max_val);
int IOT_PostOwnedMessage(char *msg);    // msg 为 malloc 得到的字符串，所有权交给上报队列
/* 上报统计（诊断用）：累计提交条数、因积压丢弃的条数、当前排队条数 */
void IOT_GetReportStats(uint32_t *posted, uint32_t *dropped, uint32_t *queued);
int IOT_PostMessage(const char *msg);   // 复制一份后入队

#endif
