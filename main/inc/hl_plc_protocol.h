#ifndef HL_PLC_PROTOCOL_H
#define HL_PLC_PROTOCOL_H

/*
 * 欧姆龙 HostLink（内嵌 FINS 命令，ASCII）
 *
 * 命令：@ 单元号(2) FA 响应等待(1) ICF(2) DA2(2) SA2(2) SID(2) FINS命令(4) 参数... FCS(2) * CR
 * 应答：@ 单元号(2) FA 00 ICF(2) DA2(2) SA2(2) SID(2) FINS命令(4) 结束码(4) 数据... FCS(2) * CR
 *
 * 读写函数内部完成一次完整的串口事务（见 serial_txn.h）。
 * 串口参数见 uart_task.c（标准为 9600 7E2，现场设备改成了 115200 8N1）。
 */

#include <stdint.h>
#include "plc_value.h"

typedef enum {
    HL_FMT_U16,     // 1 个字，无符号
    HL_FMT_BCD,     // 1 个字，BCD 码
    HL_FMT_FLOAT,   // 2 个字，低字在前，IEEE754
} HlFormat;

/* 返回 PLC_OK 或 PLC_ERR_RESPONSE */
int HL_ReadDM(uint16_t address, HlFormat fmt, PlcValue *out);
int HL_ReadCIOBit(uint16_t address, uint8_t bit, PlcValue *out);
int HL_WriteDMFloat(uint16_t address, float value);

#endif
