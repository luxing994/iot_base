#ifndef EVENT_LOG_H
#define EVENT_LOG_H

/*
 * 事件日志：用中文记录网关的状态变化（启动、联网、上位机连接、设备离线、配置修改等），
 * 供配置网页"诊断"页显示和导出。
 *
 *   只记录状态变化（连上 / 断开、离线 / 恢复），不记录每次轮询，数量很少；
 *   保存在 Flash 的 spiffs 分区（按环形写入，不挂载文件系统），断电不丢，约可存 2400 条。
 *   同一分区的开头保存"异常重启前的运行日志"（见 log_buffer.h）。
 *
 * EvLog_Add() 可在任意任务中调用（不能在中断中调用），只把记录放入队列，由后台任务写 Flash，不阻塞调用者。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cJSON.h"

typedef enum {
    EV_INFO = 0,
    EV_WARN = 1,
    EV_ERROR = 2,
} EvLevel;

/* 须在 nvs_flash_init() 之后、其他模块记录事件之前调用 */
void EvLog_Init(void);

/* 记录一条事件，文字超过约 100 字节会被截断 */
void EvLog_Add(EvLevel level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/*
 * 最近 max 条事件，最新的在前：[{"t":Unix 毫秒(未校时为 0),"up":开机秒数,"b":第几次启动,"l":级别,"m":"文字"}]
 * 未校时的记录可由网页根据同一次启动中已校时的记录推算时间
 */
cJSON *EvLog_ToJson(int max);

/* 清空全部事件（出厂账号） */
void EvLog_Clear(void);

/* 本次是第几次启动（事件中的 "b"） */
uint32_t EvLog_BootNumber(void);

/* ---------------- 异常重启前的运行日志（由 log_buffer.c 保存） ---------------- */

typedef struct {
    uint64_t timeMs;        // 保存时刻（重启后），未校时为 0
    uint32_t boot;          // 发生异常的那次启动序号
    uint32_t len;
    char reason[24];        // 复位原因，如 "panic"
} CrashInfo;

/* 保存一段日志（覆盖上一次的） */
void EvLog_SaveCrash(const char *reason, const char *data, size_t len);

/* 读取：没有记录返回 false；buf 为 NULL 时只取信息 */
bool EvLog_ReadCrash(CrashInfo *info, char *buf, size_t size);

void EvLog_ClearCrash(void);

#endif
