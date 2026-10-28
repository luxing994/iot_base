#ifndef AINUO_TESTER_ASCII_PROTOCOL_H
#define AINUO_TESTER_ASCII_PROTOCOL_H

/*
 * 艾诺安规测试仪 ASCII 协议（RS232，8 位数据，无校验，1 位停止位，波特率见 uart_task.c）
 *
 * 读结果：  请求 {地址(3) 命令(1) 00}
 *           应答 {地址(3) 命令(1) 8 行测试数据(每行 23 字符) 状态(1) 校验(2)}
 * 下发参数：{地址(3) 7 1 参数... 组号 校验(2)}
 *
 * 测试项编号沿用 76T（t_tester_protocol.h 中的 TTesterTestItem）。
 * 读写函数内部完成一次完整的串口事务（见 serial_txn.h）。
 */

#include <stdint.h>
#include "iot_common.h"   // parametersData

#define AINUO_TESTER_ADDRESS    15

/* 读当前组测试结果并上报，成功返回 0 */
int AINUO_ReadGroup(void);

/* 下发组参数，成功返回 0 */
int AINUO_SetGroupPara(uint16_t group);

/* 把上位机 group 命令中一个测试项的参数保存下来，下发时使用 */
void AINUO_TTesterApplyItemParams(int item, parametersData *para);

#endif
