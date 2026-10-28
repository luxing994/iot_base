#ifndef DIAG_H
#define DIAG_H

/*
 * 诊断：把运行状态翻译成现场人员看得懂的检查结果和处理建议，并生成给厂家排查用的诊断报告
 *
 *   Diag_Check()   检查项列表 [{"level":"ok|info|warn|error","title":"","text":"","advice":""}]
 *   Diag_Export()  诊断报告（纯文本）：设备与固件信息、检查结果、配置、网络和各连接的统计、
 *                  崩溃记录（core dump 摘要 + 异常重启前日志）、事件、开机日志、最近日志
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cJSON.h"

cJSON *Diag_Check(int *errors, int *warns);

typedef void (*DiagWriter)(void *ctx, const char *text, size_t len);

void Diag_Export(DiagWriter write, void *ctx);

/* core dump（程序崩溃时由系统保存到 coredump 分区）：没有时返回 false */
bool Diag_CoreDumpInfo(size_t *size);
bool Diag_CoreDumpRead(size_t offset, void *buf, size_t len);
void Diag_CoreDumpErase(void);

/* 本次启动的复位原因（中文），abnormal 返回是否为异常复位（程序异常、看门狗、电压不足） */
const char *Diag_ResetText(bool *abnormal);

/* 把 Unix 毫秒格式化为本地时间 "2026-10-28 14:06:12"，ms 为 0 时输出 "-" */
void Diag_FormatTime(uint64_t ms, char *buf, size_t size);

#endif
