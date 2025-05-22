#ifndef PPI_PLC_PROTOCOL_H
#define PPI_PLC_PROTOCOL_H

#include <stdint.h>

/*
UART传输格式
数据位：8位
波特率：9600bps
奇偶：偶校验
停止位：1位
*/

// PPI PLC 控制字符
#define PPI_PLC_SD                    0x68
#define PPI_PLC_READ_COM_LEN          0x1B
#define PPI_PLC_CONFIRM_COM_LEN       0x03
#define PPI_PLC_IP                    0x32
#define PPI_PLC_RC                    0x01
#define PPI_PLC_RI                    0x00
#define PPI_PLC_PD                    0x00
#define PPI_PLC_UP                    0x00
#define PPI_PLC_PL1                   0x00
#define PPI_PLC_PL2                   0x0E
#define PPI_PLC_DL1                   0x00
#define PPI_PLC_DL2                   0x00
#define PPI_PLC_MOD_READ              0x04
#define PPI_PLC_MOD_WRITE             0x05
#define PPI_PLC_AN                    0x01
#define PPI_PLC_PEND1                 0x12
#define PPI_PLC_PEND2                 0x0A
#define PPI_PLC_PEND3                 0x00
#define PPI_PLC_PEND4                 0x00
#define PPI_PLC_DI                    0x10
#define PPI_PLC_EC                    0x16

#define PPI_PLC_FC_READ_COM           0x6C
#define PPI_PLC_FC_CONFIRM_COM        0x5C
#define PPI_PLC_FC_RESPOND_COM        0x08

#define PPI_PLC_DU_BIT                0x01
#define PPI_PLC_DU_BYTE               0x02
#define PPI_PLC_DU_WORD               0x04
#define PPI_PLC_DU_DWORD              0x06

#define PPI_PLC_RT1_V                 0x01
#define PPI_PLC_RT1_OTHER             0x00  

#define PPI_PLC_RT2_S                 0x04
#define PPI_PLC_RT2_SM                0x05
#define PPI_PLC_RT2_AI                0x06
#define PPI_PLC_RT2_AQ                0x07 
#define PPI_PLC_RT2_C                 0x1E
#define PPI_PLC_RT2_I                 0x81   
#define PPI_PLC_RT2_Q                 0x82
#define PPI_PLC_RT2_M                 0x83
#define PPI_PLC_RT2_V                 0x84
#define PPI_PLC_RT2_T                 0x1F   

#define PPI_PLC_DA                    0x02
#define PPI_PLC_SA                    0x00

#define PPI_PLC_CONFIRM_CODE1         0xE5
#define PPI_PLC_CONFIRM_CODE2         0xF9

#define PPI_PLC_CON_SD                0x10

#define PPI_PLC_RESPOND_RC            0x03
#define PPI_PLC_RESPOND_DU_ WORD      0x06
#define PPI_PLC_RESPOND_DU_ DWORD     0x08
#define PPI_PLC_RESPOND_DU_ OTHER     0x05
#define PPI_PLC_RESPOND_DA_LENGTH     21

#define PPI_PLC_READ_DATA_MAX_LENGTH  8

#pragma pack(1)
typedef struct {
    uint8_t fstart;
    uint8_t length;
    uint8_t rlength;
    uint8_t sstart;
    uint8_t directaddr;
    uint8_t sourceaddr;   
    uint8_t funccode;
    uint8_t identify;
    uint8_t remotecontrol;
    uint8_t identify1;
    uint8_t identify2;
    uint8_t protocoldata;
    uint8_t unitpara;
    uint8_t paralength1;
    uint8_t paralength2;
    uint8_t datalength1;
    uint8_t datalength2;
    uint8_t mode;
    uint8_t valueaddrnum;
    uint8_t pend1;
    uint8_t pend2;
    uint8_t defid;
    uint8_t dataunit;
    uint8_t pend3;
    uint8_t datalen;
    uint8_t pend4;
    uint8_t regtype1;
    uint8_t regtype2;
    uint8_t dataaddr[3];
    uint8_t checkcode;
    uint8_t end;
} PpiPlcReadCommandFrameFormat;

typedef struct {
    uint8_t fstart;
    uint8_t length;
    uint8_t rlength;
    uint8_t sstart;
    uint8_t directaddr;
    uint8_t sourceaddr;   
    uint8_t funccode;
    uint8_t identify;
    uint8_t remotecontrol;
    uint8_t identify1;
    uint8_t identify2;
    uint8_t paralength1;
    uint8_t paralength2;
    uint8_t datalength1;
    uint8_t datalength2;
    uint8_t pend1;
    uint8_t dataunit;
    uint8_t errortag;
    uint8_t errorcode;
    uint8_t serveid;
    uint8_t valuenum;
    uint8_t result;
    uint8_t datatype;
    uint8_t datalengthbit[2];
    uint8_t* data;
    uint8_t check;
    uint8_t end;
} PpiPlcRespondCommandFrameFormat;

typedef struct {
    uint8_t start;
    uint8_t directaddr;
    uint8_t sourceaddr;
    uint8_t funccode;   
    uint8_t checkcode;
    uint8_t end;
} PpiPlcConfirmCommandFrameFormat;
#pragma pack()

void PPIReadByteDataRegister(uint32_t address, uint8_t length, uint16_t frnum);
void PPIReadBitInputRegister(uint32_t address, uint8_t bit, uint8_t length, uint16_t frnum);
void PPIReadBitOutputRegister(uint32_t address, uint8_t bit, uint8_t length, uint16_t frnum);
int GetSerialWordDataFromPpiPlc(void);

#endif