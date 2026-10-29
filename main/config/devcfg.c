/*
 * 设备配置，说明见 devcfg.h 和 docs/devcfg.md
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_crc.h"
#include "lwip/inet.h"
#include "event_log.h"
#include "gw_config.h"
#include "devcfg.h"

static const char *TAG = "DEVCFG";

#define PART_SUBTYPE    0x40            // partitions.csv 中 devcfg 的子类型
#define SLOT_MAGIC      0x31464344u     // "DCF1"
#define HDR_SIZE        32
#define TEXT_MAX        (60 * 1024)
#define LINE_MAX        1000
#define SCHEMA_VERSION  1

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint32_t len;
    uint32_t crc;
    uint32_t rsv[4];
} SlotHeader;

_Static_assert(sizeof(SlotHeader) == HDR_SIZE, "SlotHeader must be 32 bytes");

struct DevCfgImport {
    bool save;
    bool atBoot;                // 启动时加载：gateway 行不再应用（之后网页上的修改优先）
    int slot;                   // 写入的槽（save 时）
    uint32_t written;
    uint32_t crc;
    bool writeFailed;
    bool tooBig;

    char line[LINE_MAX + 1];
    size_t lineLen;
    bool lineOverflow;
    int lineNo;
    char src[32];               // 当前行的 "src"

    DevCfg cfg;
    bool haveMeta;
    bool haveDevice;
    bool haveGateway;
    GwConfig *gw;               // gateway 行应用到运行参数后的结果（save 时提交）
    uint8_t gwAreas;            // gateway 行涉及的参数区
    bool pointsRefused;         // 已报过"不需要点位 / 点位太多"，不再逐行重复报
    uint8_t frUsed[1000 / 8 + 1];

    cJSON *errors;
    int errorCount;
    cJSON *warnings;
    char first[200];            // 第一条错误（含行号和来源，最长约 186 字节）
};

static const esp_partition_t *s_part;
static uint32_t s_slotSize;
static int s_latestSlot = -1;           // 最近一次提交的槽（导出读它）
static uint32_t s_latestSeq;
static DevCfg *s_cfg;                   // 启动时加载的配置（运行期间只读）
static char s_problem[240] = "未导入采集配置";     // 可能含 first，留足空间

static const char *const s_categories[] = { "plc", "cnc", "imm", "meter", "board", "legacy" };
static const char *const s_categoryText[] = { "PLC", "机床", "注塑机", "仪表", "自研控制板", "传统设备" };
static const char *const s_pointCats[] = { "other", "state", "alarm", "count", "param", "energy" };
static const char *const s_reports[] = { "each", "batch" };
static const char *const s_modes[] = { "cycle", "change" };
static const char *const s_orders[] = { "ABCD", "CDAB", "BADC", "DCBA" };
static const char *const s_rws[] = { "r", "rw" };
static const char *const s_parities[] = { "N", "E", "O" };
static const uint32_t s_bauds[] = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };

#define COUNT(a)        (sizeof(a) / sizeof((a)[0]))

/* ------------------------------------------------------------ 名称 */

const char *DevCfg_CategoryName(uint8_t cat)
{
    return cat < COUNT(s_categories) ? s_categories[cat] : "?";
}

const char *DevCfg_CategoryText(uint8_t cat)
{
    return cat < COUNT(s_categoryText) ? s_categoryText[cat] : "?";
}

const char *DevCfg_PointCatName(uint8_t cat)
{
    return cat < COUNT(s_pointCats) ? s_pointCats[cat] : "?";
}

static const char *IfaceText(uint8_t iface)
{
    switch (iface) {
    case IFACE_RS485: return "RS485";
    case IFACE_RS232: return "RS232";
    case IFACE_RS422: return "RS422";
    case IFACE_ETH:   return "以太网";
    default:          return "?";
    }
}

void DevCfg_LinkText(const DevCfgDevice *dev, char *buf, size_t size)
{
    if (dev->iface == IFACE_ETH) {
        snprintf(buf, size, "以太网 %s:%u", dev->ip, (unsigned)dev->port);
    } else {
        snprintf(buf, size, "%s %u %u%c%u 站号 %u", IfaceText(dev->iface), (unsigned)dev->baud,
                 (unsigned)dev->dataBits, dev->parity, (unsigned)dev->stopBits, (unsigned)dev->station);
    }
}

void DevCfg_PointAddrText(const DevCfgDevice *dev, const DevCfgPoint *p, char *buf, size_t size)
{
    const ProtoArea *a = &dev->proto->areas[p->area];

    if (p->type == PT_BIT && !a->bitArea) {
        snprintf(buf, size, a->base == 8 ? "%s%lo.%u" : "%s%lu.%u", a->name, (unsigned long)p->addr, (unsigned)p->bit);
    } else {
        snprintf(buf, size, a->base == 8 ? "%s%lo" : "%s%lu", a->name, (unsigned long)p->addr);
    }
}

/* ------------------------------------------------------------ 错误记录 */

static void Err(DevCfgImport *imp, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static void Err(DevCfgImport *imp, const char *fmt, ...)
{
    char msg[128];
    va_list args;

    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    if (imp->errorCount == 0) {
        if (imp->lineNo > 0) {
            snprintf(imp->first, sizeof(imp->first), "第 %d 行%s%s%s：%s", imp->lineNo,
                     imp->src[0] ? "（" : "", imp->src, imp->src[0] ? "）" : "", msg);
        } else {
            strlcpy(imp->first, msg, sizeof(imp->first));
        }
    }
    imp->errorCount++;
    if (imp->errors != NULL && imp->errorCount <= DEVCFG_MAX_ERRORS) {
        cJSON *o = cJSON_CreateObject();
        if (imp->lineNo > 0) {
            cJSON_AddNumberToObject(o, "line", imp->lineNo);
        }
        if (imp->src[0] != '\0') {
            cJSON_AddStringToObject(o, "src", imp->src);
        }
        cJSON_AddStringToObject(o, "msg", msg);
        cJSON_AddItemToArray(imp->errors, o);
    }
}

// 警告：不阻止导入，由导入的人确认
static void Warn(DevCfgImport *imp, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static void Warn(DevCfgImport *imp, const char *fmt, ...)
{
    char msg[160];
    va_list args;

    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
    if (imp->warnings != NULL && cJSON_GetArraySize(imp->warnings) < DEVCFG_MAX_ERRORS) {
        cJSON *o = cJSON_CreateObject();
        if (imp->lineNo > 0) {
            cJSON_AddNumberToObject(o, "line", imp->lineNo);
        }
        if (imp->src[0] != '\0') {
            cJSON_AddStringToObject(o, "src", imp->src);
        }
        cJSON_AddStringToObject(o, "msg", msg);
        cJSON_AddItemToArray(imp->warnings, o);
    }
}

/* ------------------------------------------------------------ 字段读取（出错时记录并返回 false） */

static const cJSON *Item(const cJSON *o, const char *key)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(o, key);
    return (it == NULL || cJSON_IsNull(it)) ? NULL : it;
}

static bool GetStr(DevCfgImport *imp, const cJSON *o, const char *key, char *out, size_t size, bool required)
{
    const cJSON *it = Item(o, key);

    out[0] = '\0';
    if (it == NULL) {
        if (required) {
            Err(imp, "缺少 %s", key);
        }
        return !required;
    }
    if (!cJSON_IsString(it)) {
        Err(imp, "%s 应为文字", key);
        return false;
    }
    if (required && it->valuestring[0] == '\0') {
        Err(imp, "%s 不能为空", key);
        return false;
    }
    if (strlen(it->valuestring) >= size) {
        Err(imp, "%s 太长（最多 %u 字节，约 %u 个汉字）", key, (unsigned)(size - 1), (unsigned)((size - 1) / 3));
        return false;
    }
    strlcpy(out, it->valuestring, size);
    return true;
}

// 整数，也接受数字文字（Excel 导出时数字可能是文字）
static bool GetInt(DevCfgImport *imp, const cJSON *o, const char *key, long def, long min, long max, long *out)
{
    const cJSON *it = Item(o, key);
    double v;

    *out = def;
    if (it == NULL || (cJSON_IsString(it) && it->valuestring[0] == '\0')) {
        return true;
    }
    if (cJSON_IsNumber(it)) {
        v = it->valuedouble;
    } else if (cJSON_IsString(it)) {
        char *end;
        v = strtod(it->valuestring, &end);
        if (*end != '\0') {
            Err(imp, "%s 应为整数：%s", key, it->valuestring);
            return false;
        }
    } else {
        Err(imp, "%s 应为整数", key);
        return false;
    }
    if (v != (double)(long)v || v < min || v > max) {
        Err(imp, "%s 应为 %ld~%ld 的整数", key, min, max);
        return false;
    }
    *out = (long)v;
    return true;
}

static bool GetNum(DevCfgImport *imp, const cJSON *o, const char *key, double def, double *out)
{
    const cJSON *it = Item(o, key);

    *out = def;
    if (it == NULL || (cJSON_IsString(it) && it->valuestring[0] == '\0')) {
        return true;
    }
    if (cJSON_IsNumber(it)) {
        *out = it->valuedouble;
        return true;
    }
    if (cJSON_IsString(it)) {
        char *end;
        *out = strtod(it->valuestring, &end);
        if (*end == '\0') {
            return true;
        }
    }
    Err(imp, "%s 应为数字", key);
    return false;
}

// 枚举：返回下标；缺省返回 def（def 为 -1 表示必填）；出错返回 -2
static int GetEnum(DevCfgImport *imp, const cJSON *o, const char *key, const char *const *names, size_t n, int def)
{
    const cJSON *it = Item(o, key);

    if (it == NULL || (cJSON_IsString(it) && it->valuestring[0] == '\0')) {
        if (def < 0) {
            Err(imp, "缺少 %s", key);
            return -2;
        }
        return def;
    }
    if (cJSON_IsString(it)) {
        for (size_t i = 0; i < n; i++) {
            if (strcmp(names[i], it->valuestring) == 0) {
                return (int)i;
            }
        }
    }
    char allowed[96] = "";
    for (size_t i = 0; i < n; i++) {
        strlcat(allowed, names[i], sizeof(allowed));
        if (i + 1 < n) {
            strlcat(allowed, " / ", sizeof(allowed));
        }
    }
    Err(imp, "%s 应为 %s", key, allowed);
    return -2;
}

static bool GetMsgStr(DevCfgImport *imp, const cJSON *o, const char *key, char *out, size_t size, bool required)
{
    if (!GetStr(imp, o, key, out, size, required)) {
        return false;
    }
    // 只允许可见的英文字符（0x20~0x7E），且不含 " 和 \：上位机按什么编码解析都不会乱码，也不会破坏 JSON
    for (const unsigned char *s = (const unsigned char *)out; *s != '\0'; s++) {
        if (*s < 0x20 || *s > 0x7E || *s == '"' || *s == '\\') {
            Err(imp, "%s 会写入上报报文，只能使用英文字母、数字和英文符号（不能有中文、双引号、反斜杠）", key);
            out[0] = '\0';
            return false;
        }
    }
    return true;
}

static bool IsIp4(const char *s)
{
    struct in_addr a;
    return s[0] != '\0' && inet_aton(s, &a) != 0;
}

/* ------------------------------------------------------------ 各种行 */

static void ParseMeta(DevCfgImport *imp, const cJSON *o)
{
    DevCfg *c = &imp->cfg;
    long schema;

    if (imp->haveMeta) {
        Err(imp, "meta 只能有一行");
        return;
    }
    imp->haveMeta = true;
    if (Item(o, "schema") == NULL) {
        Err(imp, "缺少 schema");
    } else if (GetInt(imp, o, "schema", 0, 0, 1000, &schema) && schema != SCHEMA_VERSION) {
        Err(imp, "格式版本 %ld 不受支持（本固件支持 %d），请升级固件", schema, SCHEMA_VERSION);
    }
    GetStr(imp, o, "rev", c->rev, sizeof(c->rev), true);
    GetStr(imp, o, "source", c->source, sizeof(c->source), false);
    GetStr(imp, o, "created", c->created, sizeof(c->created), false);
    GetStr(imp, o, "library", c->library, sizeof(c->library), false);
    GetStr(imp, o, "note", c->note, sizeof(c->note), false);
}

static void ParseDevice(DevCfgImport *imp, const cJSON *o)
{
    DevCfgDevice *d = &imp->cfg.dev;
    char text[24];
    long v;
    int e;

    if (imp->haveDevice) {
        Err(imp, "device 只能有一行（一台网关对应一台设备）");
        return;
    }
    imp->haveDevice = true;

    GetStr(imp, o, "name", d->name, sizeof(d->name), true);             // 显示名称，可用中文，不进报文
    GetMsgStr(imp, o, "devName", d->devName, sizeof(d->devName), false);   // 报文 devName，缺省为设备编号
    if (GetStr(imp, o, "number", d->number, sizeof(d->number), true)) {
        for (const char *s = d->number; *s != '\0'; s++) {
            if (!((*s >= '0' && *s <= '9') || (*s >= 'A' && *s <= 'Z') || (*s >= 'a' && *s <= 'z') ||
                  *s == '-' || *s == '_' || *s == '.')) {
                Err(imp, "number 只能包含字母、数字和 - _ .");
                break;
            }
        }
    }
    GetMsgStr(imp, o, "orderName", d->orderName, sizeof(d->orderName), false);
    if (d->devName[0] == '\0') {
        strlcpy(d->devName, d->number, sizeof(d->devName));
    }
    GetStr(imp, o, "brand", d->brand, sizeof(d->brand), false);
    GetStr(imp, o, "model", d->model, sizeof(d->model), false);
    e = GetEnum(imp, o, "category", s_categories, COUNT(s_categories), -1);
    d->category = (e >= 0) ? (uint8_t)e : 0;

    if (GetStr(imp, o, "protocol", text, sizeof(text), true) && text[0] != '\0') {
        d->proto = ProtoLib_Find(text);
        if (d->proto == NULL) {
            Err(imp, "本固件不支持协议 %s（协议库中的协议须先开发并升级固件）", text);
        } else if (!ProtoLib_IsVerified(d->proto)) {
            Warn(imp, "协议 %s 尚未在真机上按配置采集验证过，导入后请在采集页核对数据", d->proto->name);
        }
    }
    if (GetStr(imp, o, "iface", text, sizeof(text), true) && text[0] != '\0') {
        d->iface = ProtoLib_IfaceFromName(text);
        if (d->iface == 0) {
            Err(imp, "iface 应为 rs485 / rs232 / rs422 / eth");
        } else if (d->proto != NULL && !(d->proto->ifaces & d->iface)) {
            Err(imp, "%s 不支持接口 %s", d->proto->name, text);
            d->iface = 0;
        }
    }

    const ProtoInfo *p = d->proto;
    if (p != NULL && (d->iface & IFACE_SERIAL)) {
        if (GetInt(imp, o, "baud", p->baud, 1200, 115200, &v)) {
            d->baud = (uint32_t)v;
            bool ok = false;
            for (size_t i = 0; i < COUNT(s_bauds); i++) {
                ok |= (s_bauds[i] == d->baud);
            }
            if (!ok) {
                Err(imp, "baud 应为 1200 / 2400 / 4800 / 9600 / 19200 / 38400 / 57600 / 115200");
            }
        }
        if (GetInt(imp, o, "dataBits", p->dataBits, 7, 8, &v)) {
            d->dataBits = (uint8_t)v;
        }
        int defParity = (p->parity == 'E') ? 1 : (p->parity == 'O') ? 2 : 0;
        e = GetEnum(imp, o, "parity", s_parities, COUNT(s_parities), defParity);
        d->parity = (e >= 0) ? s_parities[e][0] : 'N';
        if (GetInt(imp, o, "stopBits", p->stopBits, 1, 2, &v)) {
            d->stopBits = (uint8_t)v;
        }
    }
    if (p != NULL && d->iface == IFACE_ETH) {
        if (GetStr(imp, o, "ip", d->ip, sizeof(d->ip), true) && d->ip[0] != '\0' && !IsIp4(d->ip)) {
            Err(imp, "ip 格式不正确：%s", d->ip);
        }
        if (GetInt(imp, o, "port", p->port, 1, 65535, &v)) {
            d->port = (uint16_t)v;
        }
        if (GetInt(imp, o, "rack", 0, 0, 7, &v)) {
            d->rack = (uint8_t)v;
        }
        if (GetInt(imp, o, "slot", 1, 0, 31, &v)) {
            d->slot = (uint8_t)v;
        }
    }
    if (p != NULL && GetInt(imp, o, "station", p->station, 0, p->maxStation, &v)) {
        d->station = (uint8_t)v;
    }
    if (GetInt(imp, o, "pollMs", 1000, 100, 60000, &v)) {
        d->pollMs = (uint16_t)v;
    }
    e = GetEnum(imp, o, "report", s_reports, COUNT(s_reports), DEV_REPORT_EACH);
    d->report = (e >= 0) ? (uint8_t)e : DEV_REPORT_EACH;
    GetMsgStr(imp, o, "devTypeId", d->devTypeId, sizeof(d->devTypeId), false);
    GetMsgStr(imp, o, "devTypeName", d->devTypeName, sizeof(d->devTypeName), false);
    if (d->devTypeName[0] == '\0' && p != NULL) {
        strlcpy(d->devTypeName, p->devTypeName, sizeof(d->devTypeName));
    }
}

/*
 * gateway 行：上位机地址和端口、以太网参数，导入时写入运行参数（gw_config），之后仍可在网页修改。
 * devId 只用于核对：与本机不一致时给出警告（更换网关时可以继续导入）。
 */
static const char *const s_gwFactoryKeys[] = { "hostReportPort", "hostLinkPort", "hostOtaPort", "cmdPort", "filePort" };
static const char *const s_gwUserKeys[] = { "hostAddr", "ethDhcp", "ethIp", "ethMask", "ethGateway", "ethDns" };

static bool InList(const char *key, const char *const *list, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (strcmp(key, list[i]) == 0) {
            return true;
        }
    }
    return false;
}

static void ParseGateway(DevCfgImport *imp, const cJSON *o)
{
    char devId[32];
    const char *why = NULL;

    if (imp->haveGateway) {
        Err(imp, "gateway 只能有一行");
        return;
    }
    imp->haveGateway = true;
    if (GetStr(imp, o, "devId", devId, sizeof(devId), false) && devId[0] != '\0' && !imp->atBoot &&
        strcmp(devId, GwConfig_Get()->devId) != 0) {
        Warn(imp, "表中的网关编号 %s 与本机 %s 不一致：如果是更换网关，可以继续导入", devId, GwConfig_Get()->devId);
    }

    // 按参数区分开，分别交给 gw_config 校验
    cJSON *fac = cJSON_CreateObject();
    cJSON *usr = cJSON_CreateObject();
    const cJSON *it;
    cJSON_ArrayForEach(it, o) {
        if (strcmp(it->string, "t") == 0 || strcmp(it->string, "src") == 0 || strcmp(it->string, "devId") == 0) {
            continue;
        }
        if (InList(it->string, s_gwFactoryKeys, COUNT(s_gwFactoryKeys))) {
            cJSON_AddItemReferenceToObject(fac, it->string, (cJSON *)it);
        } else if (InList(it->string, s_gwUserKeys, COUNT(s_gwUserKeys))) {
            cJSON_AddItemReferenceToObject(usr, it->string, (cJSON *)it);
        } else {
            Err(imp, "gateway 行不支持字段 %s", it->string);
        }
    }
    if (!imp->atBoot && (fac->child != NULL || usr->child != NULL)) {
        if (imp->gw == NULL) {
            imp->gw = malloc(sizeof(GwConfig));
        }
        if (imp->gw == NULL) {
            Err(imp, "内存不足");
        } else {
            *imp->gw = *GwConfig_Get();
            if (fac->child != NULL && GwConfig_FromJson(imp->gw, fac, GW_AREA_FACTORY, &why) != ESP_OK) {
                Err(imp, "%s", why);
            } else if (usr->child != NULL && GwConfig_FromJson(imp->gw, usr, GW_AREA_USER, &why) != ESP_OK) {
                Err(imp, "%s", why);
            } else if (GwConfig_Validate(imp->gw, &why) != ESP_OK) {
                Err(imp, "%s", why);
            } else {
                imp->gwAreas = (fac->child != NULL ? GW_AREA_FACTORY : 0) | (usr->child != NULL ? GW_AREA_USER : 0);
            }
        }
    }
    cJSON_Delete(fac);
    cJSON_Delete(usr);
}

// 地址：数字或文字，按区的进制解析
static bool GetAddr(DevCfgImport *imp, const cJSON *o, const ProtoArea *a, uint32_t *out)
{
    const cJSON *it = Item(o, "addr");
    char text[16];

    if (it == NULL) {
        Err(imp, "缺少 addr");
        return false;
    }
    if (cJSON_IsNumber(it)) {
        if (it->valuedouble < 0 || it->valuedouble != (double)(long)it->valuedouble) {
            Err(imp, "addr 应为非负整数");
            return false;
        }
        snprintf(text, sizeof(text), "%ld", (long)it->valuedouble);    // 按文字再解析，八进制区也能检查数字
    } else if (cJSON_IsString(it) && it->valuestring[0] != '\0' && strlen(it->valuestring) < sizeof(text)) {
        strlcpy(text, it->valuestring, sizeof(text));
    } else {
        Err(imp, "addr 格式不正确");
        return false;
    }
    char *end;
    unsigned long v = strtoul(text, &end, a->base);
    if (*end != '\0' || text[0] == '-') {
        Err(imp, a->base == 8 ? "%s 区地址为八进制（只含 0~7）：%s" : "%s 区地址应为数字：%s", a->name, text);
        return false;
    }
    if (v > a->maxAddr) {
        Err(imp, a->base == 8 ? "%s 区地址超出范围（最大 %lo）" : "%s 区地址超出范围（最大 %lu）",
            a->name, (unsigned long)a->maxAddr);
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

static void ParsePoint(DevCfgImport *imp, const cJSON *o)
{
    DevCfg *c = &imp->cfg;
    const ProtoInfo *p = c->dev.proto;
    DevCfgPoint pt;
    char text[16];
    long v;
    double f;
    int e;

    if (!imp->haveDevice) {
        Err(imp, "point 必须写在 device 行之后");
        return;
    }
    if (p == NULL) {
        return;             // 协议无效已报错，点位无法校验
    }
    if (!p->needPoints) {
        if (!imp->pointsRefused) {
            Err(imp, "%s 为内置模板，数据项由固件决定，不需要点位", p->name);
            imp->pointsRefused = true;
        }
        return;
    }
    if (c->count >= DEVCFG_MAX_POINTS) {
        if (!imp->pointsRefused) {
            Err(imp, "数据点超过 %d 个", DEVCFG_MAX_POINTS);
            imp->pointsRefused = true;
        }
        return;
    }

    memset(&pt, 0, sizeof(pt));
    int before = imp->errorCount;

    if (Item(o, "fr") == NULL) {
        Err(imp, "缺少 fr");
    } else if (GetInt(imp, o, "fr", 0, 1, 999, &v)) {
        pt.fr = (uint16_t)v;
        if (imp->frUsed[v / 8] & (1u << (v % 8))) {
            Err(imp, "FR%03ld 重复", v);
        }
        imp->frUsed[v / 8] |= (uint8_t)(1u << (v % 8));
    }
    GetStr(imp, o, "name", pt.name, sizeof(pt.name), true);
    e = GetEnum(imp, o, "cat", s_pointCats, COUNT(s_pointCats), 0);
    pt.cat = (e >= 0) ? (uint8_t)e : 0;

    const ProtoArea *a = NULL;
    if (GetStr(imp, o, "area", text, sizeof(text), true) && text[0] != '\0') {
        int ai = ProtoLib_FindArea(p, text);
        if (ai < 0) {
            char allowed[64] = "";
            for (int i = 0; i < p->areaCount; i++) {
                strlcat(allowed, p->areas[i].name, sizeof(allowed));
                strlcat(allowed, i + 1 < p->areaCount ? " / " : "", sizeof(allowed));
            }
            Err(imp, "%s 没有 %s 区（可用：%s）", p->name, text, allowed);
        } else {
            pt.area = (uint8_t)ai;
            a = &p->areas[ai];
        }
    }
    int type = -1;
    if (GetStr(imp, o, "type", text, sizeof(text), true) && text[0] != '\0') {
        type = ProtoLib_TypeFromName(text);
        if (type < 0) {
            Err(imp, "type 应为 bit / i16 / u16 / i32 / u32 / f32 / bcd16");
        } else if (a != NULL && !(a->types & PT_MASK(type))) {
            Err(imp, "%s 区不支持 %s 类型", a->name, text);
            type = -1;
        }
    }
    pt.type = (type >= 0) ? (uint8_t)type : 0;

    if (a != NULL && GetAddr(imp, o, a, &pt.addr)) {
        if ((PT_DWORDS & PT_MASK(pt.type)) && pt.addr + 1 > a->maxAddr) {
            Err(imp, "32 位数据占两个地址，%s%lu 已是最后一个", a->name, (unsigned long)pt.addr);
        }
    }
    if (a != NULL && type == PT_BIT && !a->bitArea) {
        // 字区中取一位，必须给位号
        if (Item(o, "bit") == NULL) {
            Err(imp, "%s 区的位数据须填写 bit（0~%u）", a->name, (unsigned)a->maxBit);
        } else if (GetInt(imp, o, "bit", 0, 0, a->maxBit, &v)) {
            pt.bit = (uint8_t)v;
        }
    }

    int defOrder = ORDER_ABCD;
    for (size_t i = 0; i < COUNT(s_orders); i++) {
        if (strcmp(s_orders[i], p->order) == 0) {
            defOrder = (int)i;
        }
    }
    e = GetEnum(imp, o, "order", s_orders, COUNT(s_orders), defOrder);
    pt.order = (e >= 0) ? (uint8_t)e : ORDER_ABCD;

    if (GetNum(imp, o, "scale", 1.0, &f)) {
        if (f == 0) {
            Err(imp, "scale 不能为 0");
        }
        pt.scale = (float)f;
    }
    long defDec = (pt.type == PT_F32 || pt.scale != 1.0f) ? 2 : 0;
    if (GetInt(imp, o, "dec", defDec, 0, 6, &v)) {
        pt.dec = (uint8_t)v;
    }
    GetMsgStr(imp, o, "unit", pt.unit, sizeof(pt.unit), false);
    e = GetEnum(imp, o, "mode", s_modes, COUNT(s_modes), POINT_CYCLE);
    pt.mode = (e >= 0) ? (uint8_t)e : POINT_CYCLE;
    if (GetNum(imp, o, "dead", 0, &f)) {
        if (f < 0) {
            Err(imp, "dead 不能为负数");
        }
        pt.dead = (float)f;
    }
    e = GetEnum(imp, o, "rw", s_rws, COUNT(s_rws), 0);
    pt.writable = (e == 1);

    if (imp->errorCount == before) {
        c->points[c->count++] = pt;
    }
}

static void ParseLine(DevCfgImport *imp, char *s)
{
    // 去掉首尾空白和 UTF-8 BOM
    if (imp->lineNo == 1 && (uint8_t)s[0] == 0xEF && (uint8_t)s[1] == 0xBB && (uint8_t)s[2] == 0xBF) {
        s += 3;
    }
    while (*s == ' ' || *s == '\t') {
        s++;
    }
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\r' || s[n - 1] == ' ' || s[n - 1] == '\t')) {
        s[--n] = '\0';
    }
    if (n == 0) {
        return;
    }

    imp->src[0] = '\0';
    cJSON *o = cJSON_Parse(s);
    if (o == NULL || !cJSON_IsObject(o)) {
        Err(imp, "不是有效的 JSON 对象");
        cJSON_Delete(o);
        return;
    }
    const cJSON *src = Item(o, "src");
    if (cJSON_IsString(src)) {
        strlcpy(imp->src, src->valuestring, sizeof(imp->src));
    }
    const cJSON *t = Item(o, "t");
    const char *kind = cJSON_IsString(t) ? t->valuestring : "";

    if (!imp->haveMeta && strcmp(kind, "meta") != 0) {
        Err(imp, "第一行必须是 meta");
        imp->haveMeta = true;       // 只报一次
    } else if (strcmp(kind, "meta") == 0) {
        ParseMeta(imp, o);
    } else if (strcmp(kind, "device") == 0) {
        ParseDevice(imp, o);
    } else if (strcmp(kind, "gateway") == 0) {
        ParseGateway(imp, o);
    } else if (strcmp(kind, "point") == 0) {
        ParsePoint(imp, o);
    } else {
        Err(imp, "未知的行类型 t=\"%s\"（应为 meta / device / gateway / point）", kind);
    }
    cJSON_Delete(o);
}

static void EndLine(DevCfgImport *imp)
{
    imp->lineNo++;
    if (imp->lineOverflow) {
        imp->src[0] = '\0';
        Err(imp, "该行超过 %d 字节", LINE_MAX);
    } else {
        imp->line[imp->lineLen] = '\0';
        ParseLine(imp, imp->line);
    }
    imp->lineLen = 0;
    imp->lineOverflow = false;
}

/* ------------------------------------------------------------ 导入 */

static uint32_t SlotOffset(int slot)
{
    return (uint32_t)slot * s_slotSize;
}

DevCfgImport *DevCfg_ImportBegin(bool save)
{
    DevCfgImport *imp = calloc(1, sizeof(*imp));

    if (imp == NULL) {
        return NULL;
    }
    imp->cfg.points = calloc(DEVCFG_MAX_POINTS, sizeof(DevCfgPoint));
    imp->errors = cJSON_CreateArray();
    imp->warnings = cJSON_CreateArray();
    if (imp->cfg.points == NULL || imp->errors == NULL || imp->warnings == NULL) {
        free(imp->cfg.points);
        cJSON_Delete(imp->errors);
        cJSON_Delete(imp->warnings);
        free(imp);
        return NULL;
    }
    imp->save = save && s_part != NULL;
    if (save && s_part == NULL) {
        Err(imp, "没有 devcfg 分区，请用新的分区表重新烧写固件");
    }
    if (imp->save) {
        imp->slot = (s_latestSlot == 0) ? 1 : 0;
        if (esp_partition_erase_range(s_part, SlotOffset(imp->slot), s_slotSize) != ESP_OK) {
            imp->writeFailed = true;
        }
    }
    return imp;
}

void DevCfg_ImportFeed(DevCfgImport *imp, const char *data, size_t len)
{
    if (imp->save && !imp->writeFailed && !imp->tooBig) {
        if (imp->written + len > TEXT_MAX) {
            imp->tooBig = true;
        } else if (esp_partition_write(s_part, SlotOffset(imp->slot) + HDR_SIZE + imp->written, data, len) != ESP_OK) {
            imp->writeFailed = true;
        } else {
            imp->crc = esp_rom_crc32_le(imp->crc, (const uint8_t *)data, len);
            imp->written += len;
        }
    } else if (!imp->save) {
        imp->written += len;
        imp->tooBig |= (imp->written > TEXT_MAX);
    }

    for (size_t i = 0; i < len; i++) {
        if (data[i] == '\n') {
            EndLine(imp);
        } else if (imp->lineLen < LINE_MAX) {
            imp->line[imp->lineLen++] = data[i];
        } else {
            imp->lineOverflow = true;
        }
    }
}

// 结束解析，做整体检查；返回是否全部通过
static bool Finish(DevCfgImport *imp)
{
    if (imp->lineLen > 0 || imp->lineOverflow) {
        EndLine(imp);           // 最后一行没有换行符
    }
    imp->lineNo = 0;
    imp->src[0] = '\0';
    if (!imp->haveMeta) {
        Err(imp, "配置为空");
    } else if (!imp->haveDevice) {
        Err(imp, "缺少 device 行");
    } else if (imp->cfg.dev.proto != NULL && imp->cfg.dev.proto->needPoints && imp->cfg.count == 0 && imp->errorCount == 0) {
        Err(imp, "没有数据点");
    }
    if (imp->tooBig) {
        Err(imp, "配置超过 %d KB", TEXT_MAX / 1024);
    }
    if (imp->writeFailed) {
        Err(imp, "写入 Flash 失败");
    }
    return imp->errorCount == 0;
}

static void FreeImport(DevCfgImport *imp)
{
    free(imp->cfg.points);
    free(imp->gw);
    cJSON_Delete(imp->errors);
    cJSON_Delete(imp->warnings);
    free(imp);
}

esp_err_t DevCfg_ImportEnd(DevCfgImport *imp, cJSON **errors, cJSON **warnings, const char **summary)
{
    static char s_summary[200];
    bool ok = Finish(imp);

    if (ok && imp->save) {
        SlotHeader h = {
            .magic = SLOT_MAGIC,
            .seq = s_latestSeq + 1,
            .len = imp->written,
            .crc = imp->crc,
        };
        // 槽头最后写：写到一半断电时这个槽没有槽头，仍使用原来的配置
        if (esp_partition_write(s_part, SlotOffset(imp->slot), &h, sizeof(h)) != ESP_OK) {
            Err(imp, "写入 Flash 失败");
            ok = false;
        } else {
            s_latestSlot = imp->slot;
            s_latestSeq = h.seq;
            // gateway 行的参数写入运行参数（设备配置已提交；这里失败只记录，不影响设备配置）
            if (imp->gw != NULL && (imp->gwAreas & GW_AREA_FACTORY) && GwConfig_Save(imp->gw, GW_AREA_FACTORY) != ESP_OK) {
                ESP_LOGE(TAG, "save gateway factory params failed");
            }
            if (imp->gw != NULL && (imp->gwAreas & GW_AREA_USER) && GwConfig_Save(imp->gw, GW_AREA_USER) != ESP_OK) {
                ESP_LOGE(TAG, "save gateway user params failed");
            }
            ESP_LOGI(TAG, "saved rev %s, %s, %u points, slot %d seq %u", imp->cfg.rev,
                     imp->cfg.dev.proto->id, (unsigned)imp->cfg.count, imp->slot, (unsigned)h.seq);
        }
    }
    if (ok) {
        snprintf(s_summary, sizeof(s_summary), "版本 %s：%s，%s，%u 个数据点", imp->cfg.rev, imp->cfg.dev.name,
                 imp->cfg.dev.proto->name, (unsigned)imp->cfg.count);
    } else {
        strlcpy(s_summary, imp->first, sizeof(s_summary));
    }
    if (summary != NULL) {
        *summary = s_summary;
    }
    if (errors != NULL) {
        *errors = imp->errors;
        imp->errors = NULL;
    }
    if (warnings != NULL) {
        *warnings = imp->warnings;
        imp->warnings = NULL;
    }
    FreeImport(imp);
    return ok ? ESP_OK : ESP_FAIL;
}

/* ------------------------------------------------------------ 存储 */

static bool ReadHeader(int slot, SlotHeader *h)
{
    return esp_partition_read(s_part, SlotOffset(slot), h, sizeof(*h)) == ESP_OK &&
           h->magic == SLOT_MAGIC && h->len > 0 && h->len <= TEXT_MAX;
}

// 按块读出槽中的原文，交给 put；CRC 不符返回 false
static bool ReadText(int slot, const SlotHeader *h, void (*put)(void *ctx, const char *text, size_t len), void *ctx)
{
    char *buf = malloc(1024);
    uint32_t crc = 0;
    bool ok = (buf != NULL);

    for (uint32_t off = 0; ok && off < h->len; off += 1024) {
        size_t n = (h->len - off < 1024) ? h->len - off : 1024;
        ok = esp_partition_read(s_part, SlotOffset(slot) + HDR_SIZE + off, buf, n) == ESP_OK;
        if (ok) {
            crc = esp_rom_crc32_le(crc, (const uint8_t *)buf, n);
            if (put != NULL) {
                put(ctx, buf, n);
            }
        }
    }
    free(buf);
    return ok && crc == h->crc;
}

static void FeedImport(void *ctx, const char *text, size_t len)
{
    DevCfg_ImportFeed((DevCfgImport *)ctx, text, len);
}

void DevCfg_Init(void)
{
    SlotHeader h[2];
    bool valid[2];

    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)PART_SUBTYPE, "devcfg");
    if (s_part == NULL) {
        ESP_LOGE(TAG, "no devcfg partition");
        strlcpy(s_problem, "没有 devcfg 分区，请用新的分区表重新烧写固件", sizeof(s_problem));
        return;
    }
    s_slotSize = (uint32_t)(s_part->size / 2);

    for (int i = 0; i < 2; i++) {
        valid[i] = ReadHeader(i, &h[i]);
    }
    if (valid[0] && valid[1]) {
        s_latestSlot = ((int32_t)(h[1].seq - h[0].seq) > 0) ? 1 : 0;
    } else {
        s_latestSlot = valid[0] ? 0 : valid[1] ? 1 : -1;
    }
    if (s_latestSlot < 0) {
        ESP_LOGI(TAG, "no device config");
        return;
    }
    s_latestSeq = h[s_latestSlot].seq;

    // 先校验 CRC，再解析
    if (!ReadText(s_latestSlot, &h[s_latestSlot], NULL, NULL)) {
        snprintf(s_problem, sizeof(s_problem), "采集配置已损坏（校验失败），请重新导入");
        EvLog_Add(EV_ERROR, "%s", s_problem);
        return;
    }
    DevCfgImport *imp = DevCfg_ImportBegin(false);
    if (imp == NULL) {
        strlcpy(s_problem, "内存不足，无法加载采集配置", sizeof(s_problem));
        return;
    }
    imp->atBoot = true;
    ReadText(s_latestSlot, &h[s_latestSlot], FeedImport, imp);
    if (!Finish(imp)) {
        // 固件升级后协议库变化等原因：配置不再有效
        snprintf(s_problem, sizeof(s_problem), "采集配置无效：%s", imp->first);
        EvLog_Add(EV_ERROR, "%s", s_problem);
        FreeImport(imp);
        return;
    }

    s_cfg = malloc(sizeof(DevCfg));
    if (s_cfg == NULL) {
        FreeImport(imp);
        strlcpy(s_problem, "内存不足，无法加载采集配置", sizeof(s_problem));
        return;
    }
    *s_cfg = imp->cfg;
    // 点表缩到实际大小
    if (s_cfg->count == 0) {
        free(s_cfg->points);
        s_cfg->points = NULL;
    } else {
        DevCfgPoint *shrunk = realloc(s_cfg->points, s_cfg->count * sizeof(DevCfgPoint));
        if (shrunk != NULL) {
            s_cfg->points = shrunk;
        }
    }
    imp->cfg.points = NULL;
    FreeImport(imp);
    s_problem[0] = '\0';

    char link[64];
    DevCfg_LinkText(&s_cfg->dev, link, sizeof(link));
    ESP_LOGI(TAG, "rev %s: %s, %s, %s, %u points", s_cfg->rev, s_cfg->dev.name, s_cfg->dev.proto->name,
             link, (unsigned)s_cfg->count);
}

const DevCfg *DevCfg_Get(void)
{
    return s_cfg;
}

const char *DevCfg_DevNumber(void)
{
    return (s_cfg != NULL) ? s_cfg->dev.number : GwConfig_Get()->devNumber;
}

const char *DevCfg_DevName(void)
{
    return (s_cfg != NULL) ? s_cfg->dev.devName : GwConfig_Get()->devName;
}

const char *DevCfg_DevTypeId(void)
{
    return (s_cfg != NULL && s_cfg->dev.devTypeId[0] != '\0') ? s_cfg->dev.devTypeId : GwConfig_Get()->devTypeId;
}

const char *DevCfg_OrderName(void)
{
    return (s_cfg != NULL) ? s_cfg->dev.orderName : GwConfig_Get()->orderName;
}

const char *DevCfg_ProblemText(void)
{
    return s_cfg != NULL ? NULL : s_problem;
}

esp_err_t DevCfg_WriteFile(void (*put)(void *ctx, const char *text, size_t len), void *ctx)
{
    SlotHeader h;

    if (s_part == NULL || s_latestSlot < 0 || !ReadHeader(s_latestSlot, &h)) {
        return ESP_ERR_NOT_FOUND;
    }
    return ReadText(s_latestSlot, &h, put, ctx) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

esp_err_t DevCfg_Clear(void)
{
    esp_err_t err = ESP_OK;

    if (s_part == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    // 只擦槽头所在的扇区
    for (int i = 0; i < 2; i++) {
        if (esp_partition_erase_range(s_part, SlotOffset(i), 4096) != ESP_OK) {
            err = ESP_FAIL;
        }
    }
    s_latestSlot = -1;
    ESP_LOGI(TAG, "cleared");
    return err;
}

/* ------------------------------------------------------------ 摘要 JSON */

static void PutJson(void (*put)(void *ctx, const char *text, size_t len), void *ctx, cJSON *o)
{
    char *text = cJSON_PrintUnformatted(o);

    if (text != NULL) {
        put(ctx, text, strlen(text));
        free(text);
    }
    cJSON_Delete(o);
}

static cJSON *DeviceJson(const DevCfgDevice *d)
{
    cJSON *o = cJSON_CreateObject();
    char link[64];

    DevCfg_LinkText(d, link, sizeof(link));
    cJSON_AddStringToObject(o, "name", d->name);
    cJSON_AddStringToObject(o, "number", d->number);
    cJSON_AddStringToObject(o, "devName", d->devName);
    cJSON_AddStringToObject(o, "orderName", d->orderName);
    cJSON_AddStringToObject(o, "brand", d->brand);
    cJSON_AddStringToObject(o, "model", d->model);
    cJSON_AddStringToObject(o, "category", DevCfg_CategoryName(d->category));
    cJSON_AddStringToObject(o, "categoryText", DevCfg_CategoryText(d->category));
    cJSON_AddStringToObject(o, "protocol", d->proto->id);
    cJSON_AddStringToObject(o, "protocolName", d->proto->name);
    cJSON_AddBoolToObject(o, "needPoints", d->proto->needPoints);
    cJSON_AddStringToObject(o, "iface", ProtoLib_IfaceName(d->iface));
    cJSON_AddStringToObject(o, "ifaceText", IfaceText(d->iface));
    cJSON_AddStringToObject(o, "link", link);
    cJSON_AddNumberToObject(o, "pollMs", d->pollMs);
    cJSON_AddStringToObject(o, "report", s_reports[d->report]);
    cJSON_AddStringToObject(o, "devTypeId", d->devTypeId);
    cJSON_AddStringToObject(o, "devTypeName", d->devTypeName);
    return o;
}

static cJSON *PointJson(const DevCfgDevice *d, const DevCfgPoint *p)
{
    cJSON *o = cJSON_CreateObject();
    char addr[24];

    DevCfg_PointAddrText(d, p, addr, sizeof(addr));
    cJSON_AddNumberToObject(o, "fr", p->fr);
    cJSON_AddStringToObject(o, "name", p->name);
    cJSON_AddStringToObject(o, "cat", DevCfg_PointCatName(p->cat));
    cJSON_AddStringToObject(o, "addr", addr);
    cJSON_AddStringToObject(o, "type", ProtoLib_TypeName(p->type));
    if (PT_DWORDS & PT_MASK(p->type)) {
        cJSON_AddStringToObject(o, "order", s_orders[p->order]);
    }
    if (p->scale != 1.0f) {
        cJSON_AddNumberToObject(o, "scale", p->scale);
    }
    cJSON_AddNumberToObject(o, "dec", p->dec);
    cJSON_AddStringToObject(o, "unit", p->unit);
    cJSON_AddStringToObject(o, "mode", s_modes[p->mode]);
    if (p->mode == POINT_CHANGE) {
        cJSON_AddNumberToObject(o, "dead", p->dead);
    }
    if (p->writable) {
        cJSON_AddBoolToObject(o, "rw", true);
    }
    return o;
}

void DevCfg_WriteJson(void (*put)(void *ctx, const char *text, size_t len), void *ctx)
{
    const DevCfg *c = s_cfg;

    if (c == NULL) {
        cJSON *o = cJSON_CreateObject();
        cJSON_AddBoolToObject(o, "loaded", false);
        cJSON_AddStringToObject(o, "problem", s_problem);
        PutJson(put, ctx, o);
        return;
    }

    cJSON *meta = cJSON_CreateObject();
    cJSON_AddStringToObject(meta, "rev", c->rev);
    cJSON_AddStringToObject(meta, "source", c->source);
    cJSON_AddStringToObject(meta, "created", c->created);
    cJSON_AddStringToObject(meta, "library", c->library);
    cJSON_AddStringToObject(meta, "note", c->note);

    put(ctx, "{\"loaded\":true,\"meta\":", 22);
    PutJson(put, ctx, meta);
    put(ctx, ",\"device\":", 10);
    PutJson(put, ctx, DeviceJson(&c->dev));
    put(ctx, ",\"points\":[", 11);
    for (uint16_t i = 0; i < c->count; i++) {
        if (i > 0) {
            put(ctx, ",", 1);
        }
        PutJson(put, ctx, PointJson(&c->dev, &c->points[i]));
    }
    put(ctx, "]}", 2);
}
