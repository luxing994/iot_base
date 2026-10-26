#ifndef PLC_REPORT_H
#define PLC_REPORT_H

/*
 * PLC 数据解码与上报
 *
 * 协议层（xx_plc_protocol.c）收到应答后把原始数据写入各自的环形缓冲区，并通过
 * g_xxxplccount（数据点编号 FRnnn）和 g_xxxplcdataformat（PLC_DATA_TYPE_xxx）说明
 * 当前数据的含义。接收任务在协议层解析成功后调用这里对应的 PlcReport_xxx()，
 * 由它从缓冲区取出数据、按格式解码，再根据设备类型（Kconfig 中的 device type）选择：
 *   - 单值上报（默认）
 *   - 值变化才上报（FX 日跃产线 / 行车控制器）
 *   - 收齐后批量上报（生能冷媒灌注机）
 */

#include <stdint.h>

void PlcReport_Fx(void);
void PlcReport_HostLink(void);
void PlcReport_Ls(void);
void PlcReport_Ppi(void);         // PPI 串口和 S7 网口共用
void PlcReport_ModbusNet(void);

/*
 * 读取批量设备（冷媒灌注机）最近一次采集到的第 frnum 个数据点。
 * 未启用批量设备或 frnum 无效时返回 -1。
 */
int PlcReport_GetBatchValue(int frnum, double *value);

#endif
