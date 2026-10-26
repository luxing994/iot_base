#ifndef IOT_TIME_H
#define IOT_TIME_H

#include <stdint.h>

extern uint64_t g_baseTime;

void CalBaseTime(uint64_t abstime);
uint64_t GetMilliTimeNow(void);

#endif
