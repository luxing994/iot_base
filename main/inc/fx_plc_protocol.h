#ifndef FX_PLC_PROTOCOL
#define FX_PLC_PROTOCOL

#include <stdint.h>

/*
UART传输格式
数据位：7位
波特率:9600bps
奇偶:even(偶校验)
停止位：0位
*/

#define PLC_D_BASE_ADDRESS  4096 
#define PLC_D_SPECIAL_BASE_ADDRESS  3584 
#define PLC_Y_GROUP_BASE_ADDRESS  160 
#define PLC_PY_GROUP_BASE_ADDRESS  672 
#define PLC_T_GROUP_BASE_ADDRESS  192 
#define PLC_OT_GROUP_BASE_ADDRESS  704 
#define PLC_RT_GROUP_BASE_ADDRESS  1216 
#define PLC_M_SINGLE_BASE_ADDRESS  2048
#define PLC_M_GROUP_BASE_ADDRESS  256 
#define PLC_PM_GROUP_BASE_ADDRESS  768 
#define PLC_S_GROUP_BASE_ADDRESS  0 
#define PLC_X_GROUP_BASE_ADDRESS  128 
#define PLC_C_GROUP_BASE_ADDRESS  448 
#define PLC_OC_GROUP_BASE_ADDRESS  960 
#define PLC_RC_GROUP_BASE_ADDRESS  1472 
#define PLC_TV_GROUP_BASE_ADDRESS  2048 
#define PLC_CV16_GROUP_BASE_ADDRESS  2560

// FX PLC 控制字符
#define PLC_STX 0x02
#define PLC_ETX 0x03
#define PLC_EOT 0X04
#define PLC_ENQ 0x05
#define PLC_ACK 0x06
#define PLC_LF  0x0A
#define PLC_CL  0x0C
#define PLC_CR  0x0D
#define PLC_NAK 0x15

// FX PLC 串口指令
#define PLC_BR  "BR"   // 位单位成批读出
#define PLC_WR  "WR"   // 字单位成批读出（5字符） 
#define PLC_QR  "QR"   // 字单位成批读取（7字符）
#define PLC_BW  "BW"   // 位单位成批写入
#define PLC_WW  "WW"   // 字单位成批写入（5字符） 
#define PLC_QW  "QW"   // 字单位成批写入（7字符）

// FX PLC 编程口指令
#define PLC_READ 0x30      // '0'
#define PLC_WRITE 0x31     // '1'
#define PLC_ON 0x37        // '7'
#define PLC_OFF 0x38       // '8'

#define PLC_READ_DATA_FRAME_LEAGTH  11
#define PLC_READ_DATA_FRAME_CAL_LEAGTH  8

#define PLC_SERIAL_READ_DATA_FRAME_LEAGTH  17
#define PLC_SERIAL_READ_DATA_FRAME_CAL_LEAGTH  14

#define PLC_SERIAL_QREAD_DATA_FRAME_LEAGTH  19
#define PLC_SERIAL_QREAD_DATA_FRAME_CAL_LEAGTH  16

#define PLC_SERIAL_WREITE_SINGLE_REAL_DATA_FRAME_LEAGTH  25
#define PLC_SERIAL_WREITE_SINGLE_REAL_DATA_FRAME_CAL_LEAGTH  22

#define FX_PLC_MAX_X  32
#define FX_PLC_MAX_X_LEN  (FX_PLC_MAX_X / 8)

#define FX_PLC_MAX_Y  32
#define FX_PLC_MAX_Y_LEN  (FX_PLC_MAX_Y / 8)

// FX PLC 编程口通讯格式
#pragma pack(1)
typedef struct {
    uint8_t stx;
    uint8_t cmd;
    uint8_t address[4];
    uint8_t length[2];
    uint8_t etx;
    uint8_t sum[2];
} FxPlcReadFrameFormat;

// FX PLC 串口通讯格式
typedef struct {
    uint8_t enq;
    uint8_t plcnum[2];
    uint8_t pcnum[2];
    uint8_t cmd[2];
    uint8_t timeout;      // 0-150ms -> 0x0-0xf
    uint8_t address[5];   
    uint8_t length[2];
    uint8_t sum[2];
} FxPlcSerialAskReadFrameFormat;

typedef struct {
    uint8_t enq;
    uint8_t plcnum[2];
    uint8_t pcnum[2];
    uint8_t cmd[2];
    uint8_t timeout;      // 0-150ms -> 0x0-0xf
    uint8_t address[7];
    uint8_t length[2];
    uint8_t sum[2];
} FxPlcSerialAskQReadFrameFormat;

typedef struct {
    uint8_t enq;
    uint8_t plcnum[2];
    uint8_t pcnum[2];
    uint8_t cmd[2];
    uint8_t timeout;      // 0-150ms -> 0x0-0xf
    uint8_t address[5];
    uint8_t length[2];
    uint8_t data[8];
    uint8_t sum[2];
} FxPlcSerialAskWriteFrameFormat;

typedef struct {
    uint8_t stx;
    uint8_t plcnum[2];
    uint8_t pcnum[2];
    uint8_t databuf[256];           // 64 * 4
    uint8_t etx;
    uint8_t sum[2];
} FxPlcSerialAskReadBackFrameFormat;

typedef struct {
    uint8_t ack;
    uint8_t plcnum[2];
    uint8_t pcnum[2];
} FxPlcSerialAnsAckFrameFormat;

typedef struct {
    uint8_t nak;
    uint8_t plcnum[2];
    uint8_t pcnum[2];
    uint8_t errorcode[2];
} FxPlcSerialAnsNackFrameFormat;
#pragma pack()

void ReadSingleDataRegister(uint16_t address, uint16_t frnum);
void SerialReadSingleDataRegister(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address, uint16_t frnum);
void SerialReadSingleFloatDataRegister(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address, uint16_t frnum);
void SerialWriteSingleFloatDataRegister(uint16_t plcnum, uint16_t pcnum, uint8_t timeout, uint16_t address, float wdata);
void SendAckToPlc(void);
void SendNackToPlc(void);
int ReadInputRelayData();
int ReadOutputRelayData();
int GetDataFromFxPlc(void);
int GetSerialDataFromFxPlc(void);
int FXPLC_InitBuffer(void);
int FXPLC_ReadBufferBytes(uint8_t *data, uint32_t size);

#endif