#ifndef MB_RTU_H
#define MB_RTU_H

/*
 * Modbus RTU 主站（设备配置采集用，不依赖 esp-modbus）
 *
 * 走串口事务（serial_txn.h）：与其他串口协议共用接收缓冲、CRC 校验失败计入"错码"、
 * 报文页可以监视收发。功能码见 mb_pdu.h。
 */

#include <stdint.h>
#include "plc_value.h"

/* 从站地址（设备配置中的站号） */
void MB_RtuSetSlave(uint8_t slave);

/* 按功能码读：返回 PLC_OK 或 PLC_ERR_RESPONSE（超时、异常应答、CRC 错） */
int MB_RtuRead(uint8_t fc, uint16_t address, uint16_t count, uint16_t *out);

#endif
