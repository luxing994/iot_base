#ifndef LS_PLC_LOAD_PROTOCOL_H
#define LS_PLC_LOAD_PROTOCOL_H

/*
 * menuconfig 中 "LS / DBS device" 对应的串口设备
 *
 * 54DBS 热偶真空计（RS232，9600bps，8 位数据，无校验，1 位停止位）
 *   请求：地址(1)
 *   应答：FF 地址(1) 读数(4 个 ASCII 字符) 状态(1) 校验(2)
 */

#include "plc_value.h"

#define DBS_ADDRESS     0x01

/* 读取真空度读数（4 个字符原样作为字符串上报），返回 PLC_OK 或 PLC_ERR_RESPONSE */
int DBS_Read(PlcValue *out);

#endif
