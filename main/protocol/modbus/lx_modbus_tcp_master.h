#ifndef LX_MODBUS_TCP_MASTER_H
#define LX_MODBUS_TCP_MASTER_H

/*
 * Modbus TCP 主站（保持寄存器读写）
 *
 * 寄存器地址为协议地址，从 0 开始（手册中的 40001 / 1 号寄存器对应地址 0）。
 * 所有函数返回 PLC_OK、PLC_ERR_RESPONSE（异常应答等）或 PLC_ERR_LINK（需重连）。
 */

#include <stdint.h>
#include "plc_value.h"

#define MODBUS_TCP_PORT         502
#define MODBUS_TCP_UNIT_ID      1
#define MODBUS_MAX_READ_REGS    125

typedef enum {
    MODBUS_FMT_U16,     // 1 个寄存器，无符号
    MODBUS_FMT_I32,     // 2 个寄存器，高字在前
    MODBUS_FMT_FLOAT,   // 2 个寄存器，高字在前，IEEE754
} ModbusFormat;

/* 单元标识（设备配置中的站号），默认 1 */
void MODBUS_SetUnitId(uint8_t unit);

/* 按功能码读（01/02/03/04，见 mb_pdu.h）：寄存器每个一个字，线圈 / 离散输入每个 0 / 1 */
int MODBUS_NetReadFc(int sock, uint8_t fc, uint16_t address, uint16_t count, uint16_t *out);

/* 读一个数据点 */
int MODBUS_NetRead(int sock, uint16_t address, ModbusFormat fmt, PlcValue *out);

/* 通用读写 */
int MODBUS_NetReadHolding(int sock, uint16_t address, uint16_t count, uint16_t *regs);
int MODBUS_NetWriteSingle(int sock, uint16_t address, uint16_t value);
int MODBUS_NetWriteMultiple(int sock, uint16_t address, uint16_t count, const uint16_t *values);

#endif
