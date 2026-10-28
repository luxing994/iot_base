#ifndef PLC_POLL_H
#define PLC_POLL_H

#include "sdkconfig.h"

/* 串口 PLC / 测试仪 / Modbus RTU 周期轮询任务 */
void plc_poll_task(void *pvParameters);

#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
/* 处理上位机 PowerOn / PowerOff 命令（Modbus RTU 空开） */
void send_command_task(void *pvParameters);
#endif

/* 网口 PLC 单轮轮询，返回 <0 表示链路失效需要重连 */
#if defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
int PlcPoll_ModbusNet(int sock);
#endif
#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
int PlcPoll_FxNet(int sock);
#endif
#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
int PlcPoll_S7Net(int sock);
#endif

#endif
