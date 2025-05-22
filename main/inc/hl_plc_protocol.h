#ifndef HL_PLC_PROTOCOL_H
#define HL_PLC_PROTOCOL_H

#include <stdint.h>

/*
UART传输格式
数据位：7位
停止位：2位
波特率:9600bps
奇偶:even(偶校验)
*/

#define HOSTLINK_HEAD      '@'                 // HostLink 帧头
#define HOSTLINK_PLC_NUM   "00"                // HostLink PLC单元
#define FINS_HEAD          "FA"                // FINS 帧头
#define FINS_TIME          '0'                 // FINS 响应时间 (X * 10ms)
#define FINS_ICF_LOCAL     "00"                // FINS ICF本地模式
#define FINS_ICF_NET       "80"                // FINS ICF跨网模式
#define FINS_DA2_CPU       "00"                // FINS DA2目标单元为CPU
#define FINS_DA2_OTHER     HOSTLINK_PLC_NUM    // FINS DA2目标单元为其他
#define FINS_SA2_CPU       "00"                // FINS SA2源单元为CPU
#define FINS_SA2_OTHER     HOSTLINK_PLC_NUM    // FINS SA2源单元为其他
#define FINS_SID           "00"                // FINS SID常用为"00"
#define HOSTLINK_END       "*\x0D"               // HostLink 帧尾

typedef enum {
    READIO  = 0x0101,
    WRITEIO = 0x0102,
    WRITEIMMIO = 0x0103,
    READDISCONIO = 0x0104,
    READDATA = 0x0201,
    WRITEDATA = 0x0202,
    DELETEDATA = 0x0203
} HosklinkComCode;

typedef enum {
    CIOBIT  = 0x30,
    WRBIT = 0x31,
    HRBIT = 0x32,
    CIOWORD = 0xB0,
    WRWORD = 0xB1,
    HRWORD = 0xB2,
    DMBIT = 0x2,
    DMWORD = 0x82
} HosklinkMemCode;

typedef enum {
    NORMAL = 0000,
    SENDDATATOOLONG = 0104,
    NUMOUTOFRANGE = 0105,
    ADDRESSERROR = 0501
} HosklinkErrorCode;

#pragma pack(1)
typedef struct {
    uint8_t head[2];
    uint8_t resptime;
    uint8_t icf[2];
    uint8_t da2[2];
    uint8_t sa2[2];
    uint8_t sid[2];   
    uint8_t code[4];
    uint8_t mem[2];
    uint8_t text_startaddr[6];
    uint8_t text_num[4];
} FinsCommandFrameFormat;

typedef struct {
    uint8_t head[2];
    uint8_t resptime;
    uint8_t icf[2];
    uint8_t da2[2];
    uint8_t sa2[2];
    uint8_t sid[2];   
    uint8_t code[4];
    uint8_t mem[2];
    uint8_t text_startaddr[6];
    uint8_t text_num[4];
    uint8_t text_fdata[8];
} FinsWriteRealCommandFrameFormat;

typedef struct {
    uint8_t head[2];
    uint8_t resptime;
    uint8_t icf[2];
    uint8_t da2[2];
    uint8_t sa2[2];
    uint8_t sid[2];   
    uint8_t code[4];
    uint8_t mem[2];
    uint8_t text_startaddr[6];
    uint8_t text_num[4];
    uint8_t text_data[2];
} FinsWriteCIOCommandFrameFormat;

// Fins 响应帧格式  最多支持读10个word
typedef struct {
    uint8_t head[3];
    uint8_t resdata[3];
    uint8_t icf[3];
    uint8_t da2[2];
    uint8_t sa2[2];
    uint8_t sid[2];   
    uint8_t code[5];
    uint8_t respcode[5];
    uint8_t *text;
} FinsResponseFrameFormat;

// HostLink 命令帧格式
typedef struct {
    uint8_t head;
    uint8_t plcnum[2];
    FinsCommandFrameFormat finscomdata;
    uint8_t fcs[2];
    uint8_t end[2];
} HostLinkCommandFrameFormat;

// HostLink 写实数命令帧格式
typedef struct {
    uint8_t head;
    uint8_t plcnum[2];
    FinsWriteRealCommandFrameFormat finscomdata;
    uint8_t fcs[2];
    uint8_t end[2];
} HostLinkWriteRealCommandFrameFormat;

// HostLink 写位数据命令帧格式
typedef struct {
    uint8_t head;
    uint8_t plcnum[2];
    FinsWriteCIOCommandFrameFormat finscomdata;
    uint8_t fcs[2];
    uint8_t end[2];
} HostLinkWriteCIOCommandFrameFormat;

// HostLink 响应帧格式
typedef struct {
    uint8_t head;
    uint8_t plcnum[2];
    FinsResponseFrameFormat finscomdata;
    uint8_t fcs[2];
    uint8_t end[2];
} HostLinkResponseFrameFormat;
#pragma pack()

void HLReadSingleDataRegister(uint32_t address, uint16_t frnum);
void HLReadFloatDataRegister(uint32_t address, uint16_t frnum);
void HLReadBCDDataRegister(uint32_t address, uint16_t frnum);
void HLWriteRealDataRegister(uint32_t address, float wdata);
void HLReadBitCIORegister(uint32_t address, uint16_t bitpos, uint16_t frnum);
void HLWriteBitCIORegister(uint32_t address, uint16_t bitpos, uint16_t data, uint16_t frnum);
void HLReadDataCallback(int data);
int GetSerialWordDataFromHlPlc(void);

#endif