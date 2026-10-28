/*
 * 协议库，说明见 proto_lib.h 和 docs/devcfg.md
 */
#include <string.h>
#include "proto_lib.h"

#define COUNT(a)        (sizeof(a) / sizeof((a)[0]))
#define T_ALLNUM        (PT_WORDS | PT_DWORDS)

/* ------------------------------------------------------------ 存储区 */

static const ProtoArea s_fxProgAreas[] = {
    { "D",  "数据寄存器",   PT_MASK(PT_I16) | PT_MASK(PT_U16), 7999, 10, 0, false },
};

static const ProtoArea s_fxAreas[] = {
    { "D",  "数据寄存器",   T_ALLNUM,        7999, 10, 0, false },
    { "M",  "辅助继电器",   PT_MASK(PT_BIT), 7679, 10, 0, true },
    { "X",  "输入",         PT_MASK(PT_BIT), 0377, 8,  0, true },
    { "Y",  "输出",         PT_MASK(PT_BIT), 0377, 8,  0, true },
};

static const ProtoArea s_hostLinkAreas[] = {
    { "DM",  "数据存储区",  T_ALLNUM | PT_MASK(PT_BIT), 32767, 10, 15, false },
    { "CIO", "输入输出区",  PT_MASK(PT_BIT),            6143,  10, 15, false },
};

static const ProtoArea s_s7Areas[] = {
    { "V",  "变量存储区",   (T_ALLNUM & ~PT_MASK(PT_BCD16)) | PT_MASK(PT_BIT), 65535, 10, 7, false },
    { "I",  "输入",         PT_MASK(PT_BIT), 255, 10, 7, false },
    { "Q",  "输出",         PT_MASK(PT_BIT), 255, 10, 7, false },
    { "M",  "位存储区",     PT_MASK(PT_BIT), 255, 10, 7, false },
};

static const ProtoArea s_modbusAreas[] = {
    { "HR",   "保持寄存器（4x）", T_ALLNUM | PT_MASK(PT_BIT), 65535, 10, 15, false },
    { "IR",   "输入寄存器（3x）", T_ALLNUM | PT_MASK(PT_BIT), 65535, 10, 15, false },
    { "COIL", "线圈（0x）",       PT_MASK(PT_BIT),            65535, 10, 0,  true },
    { "DI",   "离散输入（1x）",   PT_MASK(PT_BIT),            65535, 10, 0,  true },
};

/* ------------------------------------------------------------ 协议 */

#define AREAS(a)        (a), COUNT(a)
#define NO_AREAS        NULL, 0

static const ProtoInfo s_protocols[] = {
    /* id            名称                          接口                       点表   波特率  数据 校验 停止 站号 最大站号 端口  字序    devTypeName */
    { "fx_prog",    "三菱 FX 编程口",              IFACE_RS232 | IFACE_RS422, true,  9600,   7, 'E', 1,   0,   0,    0,    "CDAB", "FXPLC",  AREAS(s_fxProgAreas) },
    { "fx_link",    "三菱 FX 计算机链接",          IFACE_RS485 | IFACE_RS232, true,  9600,   7, 'E', 1,   0,   15,   0,    "CDAB", "FXPLC",  AREAS(s_fxAreas) },
    { "fx_mc",      "三菱 MC 协议（以太网）",      IFACE_ETH,                 true,  0,      0, 'N', 0,   0,   0,    5000, "CDAB", "FXPLC",  AREAS(s_fxAreas) },
    { "hostlink",   "欧姆龙 HostLink",             IFACE_RS232 | IFACE_RS485, true,  115200, 8, 'N', 1,   0,   31,   0,    "CDAB", "HLPLC",  AREAS(s_hostLinkAreas) },
    { "ppi",        "西门子 PPI（S7-200）",        IFACE_RS485,               true,  9600,   8, 'E', 1,   2,   126,  0,    "ABCD", "PPIPLC", AREAS(s_s7Areas) },
    { "s7",         "西门子 S7comm（以太网）",     IFACE_ETH,                 true,  0,      0, 'N', 0,   0,   0,    102,  "ABCD", "PPIPLC", AREAS(s_s7Areas) },
    { "modbus_rtu", "Modbus RTU",                  IFACE_RS485 | IFACE_RS232, true,  9600,   8, 'N', 1,   1,   247,  0,    "ABCD", "MDPLC",  AREAS(s_modbusAreas) },
    { "modbus_tcp", "Modbus TCP",                  IFACE_ETH,                 true,  0,      0, 'N', 0,   1,   255,  502,  "ABCD", "MDPLC",  AREAS(s_modbusAreas) },
    { "dbs",        "DBS 真空计",                  IFACE_RS232,               false, 9600,   8, 'N', 1,   0,   0,    0,    "ABCD", "LSPLC",  NO_AREAS },
    { "t76",        "76T 安规测试仪",              IFACE_RS232 | IFACE_RS485, false, 9600,   8, 'N', 1,   1,   99,   0,    "ABCD", "",       NO_AREAS },
    { "ainuo",      "艾诺安规测试仪",              IFACE_RS232 | IFACE_RS485, false, 19200,  8, 'N', 1,   1,   255,  0,    "ABCD", "",       NO_AREAS },
};

static const char *const s_typeNames[PT_COUNT] = { "bit", "i16", "u16", "i32", "u32", "f32", "bcd16" };

static const struct {
    uint8_t bit;
    const char *name;
} s_ifaces[] = {
    { IFACE_RS485, "rs485" },
    { IFACE_RS232, "rs232" },
    { IFACE_RS422, "rs422" },
    { IFACE_ETH,   "eth" },
};

/* ------------------------------------------------------------ 查询 */

const ProtoInfo *ProtoLib_Find(const char *id)
{
    if (id == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < COUNT(s_protocols); i++) {
        if (strcmp(s_protocols[i].id, id) == 0) {
            return &s_protocols[i];
        }
    }
    return NULL;
}

int ProtoLib_FindArea(const ProtoInfo *p, const char *name)
{
    if (p == NULL || name == NULL) {
        return -1;
    }
    for (int i = 0; i < p->areaCount; i++) {
        if (strcmp(p->areas[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

int ProtoLib_TypeFromName(const char *name)
{
    for (int i = 0; name != NULL && i < PT_COUNT; i++) {
        if (strcmp(s_typeNames[i], name) == 0) {
            return i;
        }
    }
    return -1;
}

const char *ProtoLib_TypeName(int type)
{
    return (type >= 0 && type < PT_COUNT) ? s_typeNames[type] : "?";
}

uint8_t ProtoLib_IfaceFromName(const char *name)
{
    for (size_t i = 0; name != NULL && i < COUNT(s_ifaces); i++) {
        if (strcmp(s_ifaces[i].name, name) == 0) {
            return s_ifaces[i].bit;
        }
    }
    return 0;
}

const char *ProtoLib_IfaceName(uint8_t iface)
{
    for (size_t i = 0; i < COUNT(s_ifaces); i++) {
        if (s_ifaces[i].bit == iface) {
            return s_ifaces[i].name;
        }
    }
    return "?";
}

/* ------------------------------------------------------------ 导出 */

static cJSON *TypesToJson(uint32_t mask)
{
    cJSON *arr = cJSON_CreateArray();

    for (int t = 0; t < PT_COUNT; t++) {
        if (mask & PT_MASK(t)) {
            cJSON_AddItemToArray(arr, cJSON_CreateString(s_typeNames[t]));
        }
    }
    return arr;
}

cJSON *ProtoLib_ToJson(void)
{
    cJSON *arr = cJSON_CreateArray();

    for (size_t i = 0; i < COUNT(s_protocols); i++) {
        const ProtoInfo *p = &s_protocols[i];
        cJSON *o = cJSON_CreateObject();
        cJSON *ifs = cJSON_AddArrayToObject(o, "ifaces");
        char parity[2] = { p->parity, '\0' };

        cJSON_AddStringToObject(o, "id", p->id);
        cJSON_AddStringToObject(o, "name", p->name);
        for (size_t k = 0; k < COUNT(s_ifaces); k++) {
            if (p->ifaces & s_ifaces[k].bit) {
                cJSON_AddItemToArray(ifs, cJSON_CreateString(s_ifaces[k].name));
            }
        }
        cJSON_AddBoolToObject(o, "needPoints", p->needPoints);
        if (p->ifaces & IFACE_SERIAL) {
            cJSON_AddNumberToObject(o, "baud", p->baud);
            cJSON_AddNumberToObject(o, "dataBits", p->dataBits);
            cJSON_AddStringToObject(o, "parity", parity);
            cJSON_AddNumberToObject(o, "stopBits", p->stopBits);
        }
        if (p->ifaces & IFACE_ETH) {
            cJSON_AddNumberToObject(o, "port", p->port);
        }
        cJSON_AddNumberToObject(o, "station", p->station);
        cJSON_AddNumberToObject(o, "maxStation", p->maxStation);
        cJSON_AddStringToObject(o, "order", p->order);
        cJSON *areas = cJSON_AddArrayToObject(o, "areas");
        for (int a = 0; a < p->areaCount; a++) {
            const ProtoArea *ar = &p->areas[a];
            cJSON *ao = cJSON_CreateObject();
            cJSON_AddStringToObject(ao, "name", ar->name);
            cJSON_AddStringToObject(ao, "text", ar->text);
            cJSON_AddItemToObject(ao, "types", TypesToJson(ar->types));
            cJSON_AddNumberToObject(ao, "maxAddr", ar->maxAddr);
            cJSON_AddNumberToObject(ao, "base", ar->base);
            cJSON_AddNumberToObject(ao, "maxBit", ar->maxBit);
            cJSON_AddBoolToObject(ao, "bitArea", ar->bitArea);
            cJSON_AddItemToArray(areas, ao);
        }
        cJSON_AddItemToArray(arr, o);
    }
    return arr;
}
