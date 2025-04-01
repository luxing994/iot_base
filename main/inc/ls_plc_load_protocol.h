#ifndef LS_PLC_LOAD_PROTOCOL
#define LS_PLC_LOAD_PROTOCOL

#include <stdint.h>

/*
LS PLC LOAD协议
UART传输格式
通讯方式：RS-232
数据位：8位
停止位：1位
波特率: 115200bps
奇偶: 无
*/
#define LS_START_OF_TX           0x02    // 命令帧帧头
#define LS_END_OF_TX             0x03    // 命令帧帧尾
#define LS_START_OF_ACK          0x06    // 正常响应帧帧头
#define LS_START_OF_NACK         0x15    // 异常响应帧帧头
#define LS_END_OF_TR             0x04    // 响应帧帧尾

#define LS_LOAD_MEM_ADDR_WORD_D    'a'   // D字区域内存地址
#define LS_LOAD_MEM_ADDR_WORD_T    'b'   // T字区域内存地址
#define LS_LOAD_MEM_ADDR_WORD_C    'c'   // C字区域内存地址
#define LS_LOAD_MEM_ADDR_BIT_P     'h'   // P位区域内存地址
#define LS_LOAD_MEM_ADDR_BIT_M     'i'   // M位区域内存地址
#define LS_LOAD_MEM_ADDR_BIT_L     'j'   // L位区域内存地址
#define LS_LOAD_MEM_ADDR_BIT_K     'k'   // K位区域内存地址
#define LS_LOAD_MEM_ADDR_BIT_T     'l'   // T位区域内存地址
#define LS_LOAD_MEM_ADDR_BIT_C     'm'   // C位区域内存地址
#define LS_LOAD_MEM_ADDR_BIT_F     'n'   // F位区域内存地址

#define LS_LOAD_READ   'r'
#define LS_LOAD_WRITE  'w'

#pragma pack(1)
typedef struct {
    uint8_t head;
    uint8_t rw;
    uint8_t area;
    uint8_t addr[6];
    uint8_t num[2];
    uint8_t sum[2];
    uint8_t end;
} LsLoadCommandFrameFormat;

typedef struct {
    uint8_t head;
    uint8_t rw;
    uint8_t* data;
    uint8_t sum[2];
    uint8_t end;
} LsLoadAckFrameFormat;

typedef struct {
    uint8_t head;
    uint8_t code[4];
    uint8_t end;
} LsLoadNackFrameFormat;
#pragma pack()

/*
54DBS热偶真空计
UART传输格式
通讯方式：RS-232
数据位：8位
停止位：1位
波特率: 9600bps
奇偶: 无
*/
#define DBS_ADDRESS          0x01    // 设备地址
#define DBS_START_OF_TR      0xFF    // 响应帧帧头
#define DBS_STATUS_LOW       0x4C    // 小于设定值
#define DBS_STATUS_HIGH      0x48    // 大于设定值     

#pragma pack(1)
typedef struct {
    uint8_t head;
    uint8_t addr;
    uint8_t data[4];
    uint8_t status;
    uint8_t sum[2];
} DbsResponseFrameFormat;
#pragma pack()

/*
TR0906-N真空计
UART传输格式
通讯方式：RS-232
数据位：8位
停止位：1位
波特率: 9600bps
奇偶: 无
*/
#define TR_ADDRESS           0x01    // 设备地址
#define TR_REQUIRE_CODE      0xA2    // 响应帧帧头

#pragma pack(1)
typedef struct {
    uint8_t addr;
    uint8_t commmand;
    uint8_t check;
} TrSendFrameFormat;

typedef struct {
    uint8_t addr;
    uint8_t commmand;
    uint8_t data[4];
    uint8_t check;
} TrResponseFrameFormat;
#pragma pack()

void LSLoadReadSingleDataRegister(uint32_t address, uint16_t frnum);
int LSLoadGetSerialWordDataFromFxPlc(void);
void DBSReadData(void);
int DBSGetData(void);
void TRReadData(void);
int TRGetData(void);

#endif