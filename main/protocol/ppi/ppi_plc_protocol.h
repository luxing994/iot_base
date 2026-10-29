#ifndef PPI_PLC_PROTOCOL_H
#define PPI_PLC_PROTOCOL_H

/*
 * 西门子 S7-200 PPI 协议（RS485，9600bps，8 位数据，偶校验，1 位停止位）
 *
 * 读数据的过程：
 *   主站发读请求（SD2 帧）-> PLC 回短应答 E5 -> 主站发确认帧（SD1 帧）-> PLC 回数据帧（SD2 帧）
 * PPI_Read() 内部完成整个过程（见 serial_txn.h）。
 *
 * 数据区与 S7 网口共用 s7_area.h 中的定义；V 区即 DB1。
 */

#include <stdint.h>
#include "plc_value.h"
#include "s7_area.h"

/* 返回 PLC_OK 或 PLC_ERR_RESPONSE */
int PPI_Read(const S7Var *var, PlcValue *out);

/* PLC 站地址（默认 2） */
void PPI_SetStation(uint8_t station);

#endif
