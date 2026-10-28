#ifndef MONITOR_H
#define MONITOR_H

/*
 * 报文监视：记录网关与上位机、设备之间收发的原始数据，供配置网页实时查看（厂家账号）
 *
 *   默认关闭；网页点"开始"后按所选通道记录，到时间自动停止（避免长期占用内存）。
 *   关闭时各收发点只多一次判断，不影响性能。
 *   内存中最多保留 MON_SLOTS 条、共 MON_BYTES_MAX 字节，满了丢弃最旧的；单条超过 MON_REC_MAX 字节只保留前面部分。
 *   同一通道同一方向、间隔很短的数据合并为一条（串口应答、网口分段接收会分几次到达）。
 *
 * Mon_Capture / Mon_CaptureSock 只能在任务中调用（不能在中断中）。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cJSON.h"

typedef enum {
    MON_CH_REPORT = 0,      // 上位机上报通道
    MON_CH_LINK,            // 上位机心跳通道
    MON_CH_OTA,             // 上位机 OTA 通道
    MON_CH_CMD,             // 本机命令端口（上位机下发 JSON 命令）
    MON_CH_FILE,            // 本机文件端口
    MON_CH_SERIAL,          // 串口设备
    MON_CH_PLC,             // 网口 PLC / Modbus TCP 设备
    MON_CH_COUNT,
} MonChannel;

typedef enum {
    MON_TX = 0,             // 网关发出
    MON_RX = 1,             // 网关收到
} MonDir;

void Mon_Init(void);

/* 开始记录 mask 中的通道（bit n 对应 MonChannel n），minutes 分钟后自动停止；会清空之前的记录 */
void Mon_Start(uint32_t mask, uint32_t minutes);
void Mon_Stop(void);
void Mon_Clear(void);

void Mon_Capture(MonChannel ch, MonDir dir, const void *data, size_t len);

/* socket 收发：先用 Mon_BindSocket 登记 socket 属于哪个通道（连接建立时调用） */
void Mon_BindSocket(int sock, MonChannel ch);
void Mon_CaptureSock(int sock, MonDir dir, const void *data, size_t len);

/*
 * 序号大于 after 的记录（最多约 maxBytes 字节数据）：
 * {"on":是否记录中,"left":剩余秒数,"mask":通道,"next":下次请求的 after,"lost":因超出容量丢弃的条数,
 *  "items":[{"s":序号,"t":Unix 毫秒(未校时为 0),"up":开机毫秒,"c":通道,"d":方向,"n":原始长度,"x":"十六进制数据"}]}
 */
cJSON *Mon_ToJson(uint32_t after, size_t maxBytes);

#endif
