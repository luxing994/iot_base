#ifndef SENSOR_H
#define SENSOR_H

void InitSensorGpio(void);
void InitSwitchSensor(void);
void InitSwitch(void);
int GetSwitchSensorLevel(void);
int GetSwitchCount(void);
int GetSwitchLevel(void);

#endif