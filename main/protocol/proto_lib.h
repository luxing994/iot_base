#ifndef PROTO_LIB_H
#define PROTO_LIB_H

/*
 * 协议库：固件支持哪些协议，以及每种协议允许的接口、默认通信参数、存储区和数据类型
 *
 * 设备配置（devcfg.h）按这里的定义校验；配置网页、Excel 调查表通过 GET /api/devcfg/protocols 读取，
 * 保证表格里能选的就是固件能做的。格式说明见 docs/devcfg.md。
 *
 * 新增一种协议：在 proto_lib.c 的表中加一行（存储区表按需新增），再实现采集驱动。
 */

#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"

/* 接口（位掩码） */
#define IFACE_RS485     0x01
#define IFACE_RS232     0x02
#define IFACE_RS422     0x04
#define IFACE_ETH       0x08
#define IFACE_SERIAL    (IFACE_RS485 | IFACE_RS232 | IFACE_RS422)

/* 数据类型 */
typedef enum {
    PT_BIT,
    PT_I16,
    PT_U16,
    PT_I32,
    PT_U32,
    PT_F32,
    PT_BCD16,
    PT_COUNT
} PointType;

#define PT_MASK(t)      (1u << (t))
#define PT_WORDS        (PT_MASK(PT_I16) | PT_MASK(PT_U16) | PT_MASK(PT_BCD16))
#define PT_DWORDS       (PT_MASK(PT_I32) | PT_MASK(PT_U32) | PT_MASK(PT_F32))

/* 存储区 */
typedef struct {
    const char *name;           // 配置中的 area，如 "D"
    const char *text;           // 说明，如 "数据寄存器"
    uint32_t types;             // 允许的类型（PT_MASK 组合）
    uint32_t maxAddr;           // 最大地址
    uint8_t base;               // 地址进制：8 或 10
    uint8_t maxBit;             // 字区中取位时的最大位号（0 表示该区不能按位取；位区本身不需要 bit）
    bool bitArea;               // 位区：只能是 bit 类型，地址即位地址
} ProtoArea;

typedef struct {
    const char *id;             // 配置中的 protocol，如 "fx_link"
    const char *name;           // 显示名称
    uint8_t ifaces;             // 允许的接口
    bool needPoints;            // 需要点表；false 为内置模板（数据项由固件决定）
    /* 串口默认参数 */
    uint32_t baud;
    uint8_t dataBits;
    char parity;                // 'N' / 'E' / 'O'
    uint8_t stopBits;
    /* 默认站号、网口端口 */
    uint8_t station;
    uint8_t maxStation;
    uint16_t port;
    const char *order;          // 32 位数据默认字序
    const char *devTypeName;    // 上报报文默认 devTypeName
    const ProtoArea *areas;
    uint8_t areaCount;
} ProtoInfo;

/* 按 ID 查找，找不到返回 NULL */
const ProtoInfo *ProtoLib_Find(const char *id);

/* 已在真机上按设备配置采集验证过（未验证的协议导入时给出警告，测通后加入 proto_lib.c 的列表） */
bool ProtoLib_IsVerified(const ProtoInfo *p);

/* 在协议中按名称查找存储区，返回下标，找不到返回 -1 */
int ProtoLib_FindArea(const ProtoInfo *p, const char *name);

/* 数据类型名称 <-> 枚举，找不到返回 -1 / "?" */
int ProtoLib_TypeFromName(const char *name);
const char *ProtoLib_TypeName(int type);

/* 接口名称 <-> 位，找不到返回 0 / "?" */
uint8_t ProtoLib_IfaceFromName(const char *name);
const char *ProtoLib_IfaceName(uint8_t iface);

/* 全部协议的描述，供网页和调查表使用 */
cJSON *ProtoLib_ToJson(void);

#endif
