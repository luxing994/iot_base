#ifndef IOT_TIME_H
#define IOT_TIME_H

/*
 * 时间基准：本机不带 RTC，用"上位机下发的绝对时间 - 当时的开机时长"作为偏移量，
 * 之后 GetMilliTimeNow() = 偏移量 + 开机时长。
 * 未同步前返回开机时长。多任务读写由内部加锁保护。
 */

#include <stdint.h>

/* 用上位机下发的绝对时间（Unix 毫秒）校准，与当前时间相差不超过 10ms 时不调整 */
void CalBaseTime(uint64_t abstime);

/* 当前时间（Unix 毫秒） */
uint64_t GetMilliTimeNow(void);

/* 是否已与上位机校时（未校时 GetMilliTimeNow() 返回的是开机时长） */
#include <stdbool.h>
bool IotTime_IsSynced(void);

#endif
