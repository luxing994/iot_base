#ifndef FX_PLC_PROTOCOL_H
#define FX_PLC_PROTOCOL_H

/*
 * 三菱 FX 系列 PLC
 *
 *   编程口（RS232）      FX_ProgReadD()          只支持读 D 寄存器 16 位
 *   计算机链接（RS485）  FX_LinkRead() / FX_LinkWriteFloat()   专用协议格式 1，带和校验
 *   以太网（MC 协议）    FX_NetRead()            3E 帧，二进制
 *
 * 串口：9600bps，7 位数据，偶校验，1 位停止位
 * 串口读写函数内部完成一次完整的串口事务（见 serial_txn.h），返回时已拿到应答或超时。
 */

#include <stdint.h>
#include "plc_value.h"

/* 计算机链接参数（与 PLC 中 D8120 / D8121 的设置一致） */
#define FX_LINK_STATION     0       // 站号默认值（设备配置中可改，见 FX_SetStation）
#define FX_LINK_PC_NUMBER   0xFF    // PC 号
#define FX_LINK_WAIT_MS     10      // 报文等待时间，0~150ms，10ms 为单位

/* 软元件 */
typedef enum {
    FX_DEV_D = 'D',
    FX_DEV_X = 'X',
    FX_DEV_Y = 'Y',
    FX_DEV_M = 'M',
} FxDevice;

/* 计算机链接站号 0~15，与 PLC 中 D8121 一致 */
void FX_SetStation(uint8_t station);

/* 读出的数据格式 */
typedef enum {
    FX_FMT_U16,     // 1 个字，无符号
    FX_FMT_I32,     // 2 个字，低字在前
    FX_FMT_FLOAT,   // 2 个字，低字在前，IEEE754
    FX_FMT_BIT,     // 读 1 个字（16 个位元件），取最低位
} FxFormat;

/* 串口：返回 PLC_OK 或 PLC_ERR_RESPONSE */
int FX_ProgReadD(uint16_t address, PlcValue *out);
int FX_LinkRead(FxDevice dev, uint16_t address, FxFormat fmt, PlcValue *out);
int FX_LinkWriteFloat(uint16_t address, float value);

/* 网口：返回 PLC_OK、PLC_ERR_RESPONSE 或 PLC_ERR_LINK（需重连） */
int FX_NetRead(int sock, FxDevice dev, uint32_t address, FxFormat fmt, PlcValue *out);

#endif
