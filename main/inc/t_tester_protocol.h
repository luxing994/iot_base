#ifndef T_TESTER_PROTOCOL_H
#define T_TESTER_PROTOCOL_H

/*
 * 76T 安规测试仪（RS232 / RS485，8 位数据，无校验，1 位停止位，波特率见 uart_task.c）
 *
 * 帧格式：'I' '1' 地址(2) 长度(1) 保留(1) 主命令 [数据...] 'D' 和校验
 *         长度 = 帧总长 - 1，和校验 = 前面所有字节累加的低 8 位
 *
 *   TTester_ReadGroup()       读当前组 / 历史组的测试结果并上报（plc_poll.c 每秒调用）
 *   TTester_SetGroupPara()    下发上位机 group 命令中的组参数（cmd_task 调用）
 * 两个函数内部都完成一次完整的串口事务（见 serial_txn.h）。
 *
 * 上位机 group 命令的测试项列表也在这里解析，艾诺测试仪沿用同一套测试项编号。
 */

#include <stdint.h>
#include "iot_common.h"   // parametersData

/* 测试项编号（上位机 parameterType 与测试仪协议共用） */
typedef enum {
    TTESTEREMPTY,
    TTESTERPRESSURIZATION,  // 耐压
    TTESTERGROUNDING,       // 接地
    TTESTERINSULATION,      // 绝缘
    TTESTERLEAKAGE,         // 泄漏
    TTESTERPOWER,           // 功率
    TTESTERSTARTUP,         // 启动
    TTESTEROPENSHORT,       // 开短路
    TTESTERDCVOLTAGE,       // 直流耐压
    TTESTERCOMMON           // 通用参数
} TTesterTestItem;

#define TTESTER_GROUP_SLOTS  9      // parameterType 中的位置数：8 个测试项 + 通用参数

/* ------------------------------------------------ 上位机 group 命令 */

/* 解析 parameterType（如 "01_02_00_..._09"），记录本组各位置的测试项，成功返回 0 */
int TTester_SetItemList(const char *list);

/* 第 slot 个位置的测试项编号（TTesterTestItem） */
int TTester_GetItem(int slot);

/* 测试项 item 在 parameters 中的参数个数 */
int TTester_ItemParaNum(int item);

/* 把一个测试项的参数（上位机字符串）转换成测试仪格式保存，下发时使用 */
void TTesterApplyItemParams(int item, parametersData *para);

/* ------------------------------------------------ 串口操作 */

/* group 为 0 读当前组，否则读该组历史结果；收到结果后上报，成功返回 0 */
int TTester_ReadGroup(uint16_t group);

/* 下发通用参数和组参数，成功返回 0 */
int TTester_SetGroupPara(uint16_t group);

#endif
