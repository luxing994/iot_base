#ifndef LX_MODBUS_TCP_MASTER_H
#define LX_MODBUS_TCP_MASTER_H

#include <stdint.h>

// Modbus TCP常量
#define MODBUS_TCP_PORT              502
#define MODBUS_TCP_PROTOCOL_ID       0x0000
#define MODBUS_TCP_HEADER_SIZE       7   // MBAP头长度
#define MODBUS_TCP_MAX_ADU_LENGTH    260

// Modbus功能码
#define MODBUS_FUNC_READ_HOLDING_REGS     0x03
#define MODBUS_FUNC_WRITE_SINGLE_REG      0x06
#define MODBUS_FUNC_WRITE_MULTIPLE_REGS   0x10

// 错误码
#define MODBUS_OK                 0
#define MODBUS_ERR_SEND           -1
#define MODBUS_ERR_RECV           -2
#define MODBUS_ERR_RESP_LENGTH    -3
#define MODBUS_ERR_FUNC_MISMATCH  -4
#define MODBUS_ERR_DATA_MISMATCH  -5

// 16位寄存器读取
int modbus_tcp_read_holding_registers(int sock, uint16_t trans_id,
                                      uint8_t unit_id, uint16_t start_addr,
                                      uint16_t quantity, uint16_t *out_data);

// 32位寄存器读取（高字优先/大端模式）
int modbus_tcp_read_holding_registers_32(int sock, uint16_t trans_id,
                                         uint8_t unit_id, uint16_t start_addr,
                                         uint16_t quantity_32, uint32_t *out_data_32,
                                         int high_word_first);

// 单个寄存器写入
int modbus_tcp_write_single_register(int sock, uint16_t trans_id,
                                     uint8_t unit_id, uint16_t reg_addr,
                                     uint16_t value);

// 多个寄存器写入
int modbus_tcp_write_multiple_registers(int sock, uint16_t trans_id,
                                        uint8_t unit_id, uint16_t start_addr,
                                        uint16_t quantity, const uint16_t *values);
int GetNetDataFromModbusPlc(void);
void MODBUS_NetReadSingleDataRegister(int sock, uint32_t address, uint16_t frnum);
void MODBUS_NetReadDoubleDataRegister(int sock, uint32_t address, uint16_t frnum);
void MODBUS_NetReadFloatRegister(int sock, uint32_t address, uint16_t frnum);

#endif
