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

/* 存储区（FINS 字区代码） */
typedef enum {
    HL_AREA_DM  = 0x82,     // 数据存储区
    HL_AREA_CIO = 0xB0,     // 输入输出区
    HL_AREA_WR  = 0xB1,     // 工作区 W
    HL_AREA_HR  = 0xB2,     // 保持区 H
    HL_AREA_AR  = 0xB3,     // 辅助区 A
} HlArea;

/* 单元号 0~31（默认 0） */
void HL_SetUnit(uint8_t unit);

/* 读连续 1~2 个字，w[0] 为起始地址的字。返回 PLC_OK 或 PLC_ERR_RESPONSE */
int HL_ReadWords(HlArea area, uint16_t address, int words, uint32_t *w);

/* 返回 PLC_OK 或 PLC_ERR_RESPONSE */
int HL_ReadDM(uint16_t address, HlFormat fmt, PlcValue *out);
int HL_ReadCIOBit(uint16_t address, uint8_t bit, PlcValue *out);
int HL_WriteDMFloat(uint16_t address, float value);

#endif
