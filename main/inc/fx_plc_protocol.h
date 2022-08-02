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
#define PLC_ENQ 0x05
#define PLC_ACK 0x06
#define PLC_NAK 0x15
#define PLC_STX 0x02
#define PLC_ETX 0x03

// FX PLC 命令字符
#define PLC_READ 0x30      // '0'
#define PLC_WRITE 0x31     // '1'
#define PLC_ON 0x37        // '7'
#define PLC_OFF 0x38       // '8'

#define PLC_READ_DATA_FRAME_LEAGTH  11
#define PLC_READ_DATA_FRAME_CAL_LEAGTH  8

#pragma pack(1)
typedef struct {
    uint8_t stx;
    uint8_t cmd;
    uint8_t address[4];
    uint8_t length[2];
    uint8_t etx;
    uint8_t sum[2];
} FxPlcReadFrameFormat;
#pragma pack()

void PackReadDataRegisterFrame(uint16_t address, uint16_t length, FxPlcReadFrameFormat* rdata);
void ReadSingleDataRegister(uint16_t address);
void ReadMulDataRegister(uint16_t startaddr, uint16_t length);
int GetDataFromFxPlc(int *length);
int FXPLC_InitBuffer(void);
int FXPLC_ReadBufferBytes(uint8_t *data, uint32_t size);

#endif