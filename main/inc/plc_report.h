#ifndef PLC_REPORT_H
#define PLC_REPORT_H

/*
 * PLC 数据上报
 *
 * 协议层读到一个数据点后返回解码好的 PlcValue（见 plc_value.h），轮询层调用这里对应的
 * PlcReport_xxx()，再根据设备类型（Kconfig 中的 device type）选择：
 *   - 单值上报（默认）
 *   - 值变化才上报（FX 日跃产线 / 行车控制器）
 *   - 收齐后批量上报（生能冷媒灌注机）
 */

#include <stdbool.h>
#include "plc_value.h"

void PlcReport_Fx(int frnum, const PlcValue *v);
void PlcReport_HostLink(int frnum, const PlcValue *v, bool isBit);   // 位数据始终单独上报
void PlcReport_Ls(int frnum, const PlcValue *v);
void PlcReport_Ppi(int frnum, const PlcValue *v);          // PPI 串口和 S7 网口共用
void PlcReport_ModbusNet(int frnum, const PlcValue *v);

#endif
