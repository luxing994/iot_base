#ifndef DEVCFG_H
#define DEVCFG_H

/*
 * 设备配置：采集什么设备、怎么通信、读哪些点、怎么上报（格式规范见 docs/devcfg.md）
 *
 *   保存在 Flash 分区 devcfg 中（两个槽轮流写，断电安全），保存的是导入时的 JSON Lines 原文。
 *   启动时 DevCfg_Init() 读取并解析一次，之后只读；导入新配置后重启生效。
 *
 * 导入（配置网页 / 以后的上位机）：
 *   DevCfgImport *imp = DevCfg_ImportBegin(save);
 *   while (收到数据) DevCfg_ImportFeed(imp, data, len);      // 边收边按行解析、校验，save 时同时写 Flash
 *   err = DevCfg_ImportEnd(imp, &errors, &warnings, &summary); // 全部通过才提交
 *
 * 标识：devId 标识网关（出厂参数，换网关时变）；devNumber 标识被采集的设备（随设备配置，换网关时不变），
 * 上位机按 devNumber 归档数据。devNumber / devName / devTypeId / orderName 取自设备配置，
 * 没有导入配置时使用 menuconfig 中的默认值（DevCfg_DevNumber() 等）。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "cJSON.h"
#include "proto_lib.h"
#include "gw_config.h"

#define DEVCFG_MAX_POINTS   256
#define DEVCFG_MAX_ERRORS   20

typedef enum { DEV_REPORT_EACH, DEV_REPORT_BATCH } DevReport;
typedef enum { POINT_CYCLE, POINT_CHANGE } PointMode;
typedef enum { ORDER_ABCD, ORDER_CDAB, ORDER_BADC, ORDER_DCBA } WordOrder;

typedef struct {
    char name[32];              // 显示名称（可用中文，不进报文）
    char devName[32];           // 报文 devName（只含英文字符），缺省为 number
    char number[32];            // devNumber：设备编号，上位机据此识别设备
    char orderName[32];
    char brand[16];
    char model[24];
    uint8_t category;           // 见 DevCfg_CategoryName()
    const ProtoInfo *proto;
    uint8_t iface;              // IFACE_xxx
    uint32_t baud;              // 以下串口参数，网口设备为 0
    uint8_t dataBits;
    char parity;
    uint8_t stopBits;
    uint8_t station;
    char ip[16];                // 以下网口参数，串口设备为空
    uint16_t port;
    uint8_t rack;
    uint8_t slot;
    uint16_t pollMs;
    uint8_t report;             // DevReport
    char devTypeId[32];         // 空为 menuconfig 默认值
    char devTypeName[12];
} DevCfgDevice;

typedef struct {
    char name[32];
    char unit[8];
    float scale;
    float dead;
    uint32_t addr;
    uint16_t fr;
    uint8_t area;               // dev.proto->areas 的下标
    uint8_t type;               // PointType
    uint8_t bit;
    uint8_t order;              // WordOrder
    uint8_t dec;
    uint8_t mode;               // PointMode
    uint8_t cat;                // 见 DevCfg_PointCatName()
    bool writable;
} DevCfgPoint;

typedef struct {
    char rev[24];
    char source[48];
    char created[20];
    char library[16];
    char note[64];
    DevCfgDevice dev;
    DevCfgPoint *points;
    uint16_t count;
} DevCfg;

/* 启动时调用一次（nvs 初始化之后、采集任务启动之前） */
void DevCfg_Init(void);

/* 当前生效的配置，没有导入过或配置无效时返回 NULL */
const DevCfg *DevCfg_Get(void);

/* 设备标识：有设备配置时取自配置，否则为 menuconfig 默认值 */
const char *DevCfg_DevNumber(void);
const char *DevCfg_DevName(void);
const char *DevCfg_DevTypeId(void);
const char *DevCfg_OrderName(void);

/* 没有生效配置时的原因（"未导入采集配置" / 具体错误），有配置时返回 NULL */
const char *DevCfg_ProblemText(void);

/* 串口参数文字，如 "RS485 9600 8E1 站号 2"；网口设备如 "以太网 192.168.1.10:102" */
void DevCfg_LinkText(const DevCfgDevice *dev, char *buf, size_t size);

const char *DevCfg_CategoryName(uint8_t cat);      // "plc" ...
const char *DevCfg_CategoryText(uint8_t cat);      // "PLC" / "机床" ...
const char *DevCfg_PointCatName(uint8_t cat);      // "state" ...

/* 点地址文字，如 "D200"、"X17"、"V100.3"、"HR40.2" */
void DevCfg_PointAddrText(const DevCfgDevice *dev, const DevCfgPoint *p, char *buf, size_t size);

/* 当前配置的摘要 JSON（含全部点），分段输出，避免一次构造大 JSON 占用内存 */
void DevCfg_WriteJson(void (*put)(void *ctx, const char *text, size_t len), void *ctx);

/* 导出当前配置原文（JSON Lines）；没有配置返回 ESP_ERR_NOT_FOUND */
esp_err_t DevCfg_WriteFile(void (*put)(void *ctx, const char *text, size_t len), void *ctx);

/* 删除配置，重启后停止采集 */
esp_err_t DevCfg_Clear(void);

/* ---------------- 导入 ---------------- */

typedef struct DevCfgImport DevCfgImport;

/* save 为 false 时只校验不写 Flash；内存不足返回 NULL */
DevCfgImport *DevCfg_ImportBegin(bool save);

void DevCfg_ImportFeed(DevCfgImport *imp, const char *data, size_t len);

/*
 * 结束导入并释放 imp。全部校验通过返回 ESP_OK（save 时已提交，gateway 行的参数同时写入运行参数，重启生效），
 * *summary 为配置概况；否则返回 ESP_FAIL，*summary 为第一条错误的文字。
 * *errors / *warnings 为 [{"line":行号,"src":"...","msg":"..."}]（调用方释放）；警告不阻止导入（如网关编号不一致）
 */
esp_err_t DevCfg_ImportEnd(DevCfgImport *imp, cJSON **errors, cJSON **warnings, const char **summary);

#endif
