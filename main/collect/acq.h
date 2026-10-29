#ifndef ACQ_H
#define ACQ_H

/*
 * 采集引擎：按设备配置（devcfg.h）采集并上报
 *
 *   Acq_Start()   有设备配置时按配置设置串口、启动采集任务，返回 true；
 *                 没有设备配置时返回 false，由 app_main 按 menuconfig 启动原来的采集任务（过渡期）
 *
 * 一轮：按点表逐点读取 -> 按类型转换（字序、符号、BCD、倍率）-> 按上报方式上报：
 *   逐点（每轮都报 / 值变化才报）或一轮读完合并成一条批量报文
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

bool Acq_Start(UBaseType_t pollPrio, UBaseType_t uartPrio);

/* 有设备配置但无法采集时的原因（如协议尚未支持），正常时返回 NULL */
const char *Acq_ProblemText(void);

/* 采集任务已启动（上位机仍可暂停采集，见 HostCmd_IsCollectEnabled） */
bool Acq_IsRunning(void);

/* 距最近一轮采集结束的秒数，还没采完一轮返回 -1 */
int32_t Acq_RoundAgeS(void);

/*
 * 第 index 个点的实时数据（供网页显示）：
 *   返回 0 还没读过，1 最近一次读取成功，-1 最近一次读取失败
 *   text 为最近一次成功读到的值（已按小数位格式化，与上报一致），从未成功时为空串；
 *   *ageS 为距该值读到的秒数
 */
int Acq_GetPointText(uint16_t index, char *text, size_t size, uint32_t *ageS);

#endif
