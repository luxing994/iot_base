#ifndef SENSOR_H
#define SENSOR_H

/*
 * 开关量输入：边沿中断置 xEventGroup1 的事件位，由 cmd_task 上报
 *   计数传感器（GPIO38，上升沿） -> BIT_16，上报累计次数
 *   开关状态  （GPIO37，任意边沿）-> BIT_17，上报当前电平
 * InitSensorGpio() 必须在 xEventGroup1 创建之后调用
 */

void InitSensorGpio(void);
int GetSwitchSensorLevel(void);     // 计数传感器当前电平
int GetSwitchCount(void);           // 计数传感器累计上升沿次数
int GetSwitchLevel(void);           // 开关当前电平

#endif
