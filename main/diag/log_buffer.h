#ifndef LOG_BUFFER_H
#define LOG_BUFFER_H

/*
 * 运行日志缓冲：ESP_LOGx 的输出除了照常从串口打印，还在内存中保留一份（去掉颜色控制符），
 * 供配置网页查看和导出，现场不用接串口。
 *
 *   开机日志  每次启动最先的 BOOT_LOG_SIZE 字节（含启动信息块），写满后不再写入，不会被后续日志冲掉
 *   最近日志  环形保存最近 LOG_BUF_SIZE 字节
 *
 * 最近日志放在复位后不清零的内存中：看门狗、程序异常、掉电不足等异常复位后，
 * 启动时把复位前的内容保存到 Flash（见 event_log.h 的 EvLog_SaveCrash），用于事后排查。
 * 程序崩溃时的调用栈不经过 ESP_LOG，由 core dump 保存（见 diag.c）。
 */

#include <stddef.h>
#include <stdint.h>

#define LOG_BUF_SIZE    (16 * 1024)
#define BOOT_LOG_SIZE   (4 * 1024)

/* 须在 EvLog_Init() 之后尽早调用 */
void LogBuf_Init(void);

/*
 * 复制 [from, 当前) 之间的最近日志到 buf（from 为之前返回的 *pos；超出缓冲范围时从最早的开始）
 * 返回复制的字节数，*pos 返回当前写入位置（累计字节数）
 */
size_t LogBuf_Read(uint32_t from, char *buf, size_t size, uint32_t *pos);

/* 当前写入位置（累计字节数） */
uint32_t LogBuf_Pos(void);

/* 复制开机日志，返回字节数 */
size_t LogBuf_ReadBoot(char *buf, size_t size);

#endif
