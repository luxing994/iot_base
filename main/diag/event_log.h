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

/*
 * 可与级别按位或的标记（存在记录的级别字节中，网页据此筛选）：
 *   EV_OP   用户关心的操作：启动、修改参数、重启、恢复出厂、固件升级。客户账号默认只看警告 / 错误和这类事件
 *   EV_DEV  只给厂家看的事件（报文监视、清除崩溃记录等），客户账号读取时直接过滤掉
 */
#define EV_OP           0x10
#define EV_DEV          0x20
#define EV_ACK          0x40    // 客户在网页点了"清除"：客户账号只读到这条之后的事件（记录本身保留，厂家和导出仍可看到）
#define EV_LEVEL_MASK   0x03

/* 须在 nvs_flash_init() 之后、其他模块记录事件之前调用 */
void EvLog_Init(void);

/* 记录一条事件，文字超过约 100 字节会被截断 */
void EvLog_Add(int level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/*
 * 最近 max 条事件，最新的在前：
 *   [{"t":Unix 毫秒(未校时为 0),"up":开机秒数,"b":第几次启动,"l":级别 0~2,"op":true(EV_OP 事件才有),"m":"文字"}]
 * withDev 为 false 时（客户账号）跳过 EV_DEV 事件，读到 EV_ACK 标记即停止
 * 未校时的记录可由网页根据同一次启动中已校时的记录推算时间
 */
cJSON *EvLog_ToJson(int max, bool withDev);

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
