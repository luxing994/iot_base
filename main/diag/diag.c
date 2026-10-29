/*
 * 诊断：检查项 + 诊断报告，说明见 diag.h
 *
 * 检查项的文字面向现场人员（中文、给出处理建议）；诊断报告面向厂家排查，信息尽量全：
 * 拿到报告应能回答"什么版本、什么配置、出了什么事、何时发生、当时系统处于什么状态"。
 */
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
#include "esp_core_dump.h"
#endif
#include "app_info.h"
#include "event_log.h"
#include "gw_config.h"
#include "devcfg.h"
#include "acq.h"
#include "iot_common.h"
#include "iot_time.h"
#include "log_buffer.h"
#include "net_manager.h"
#include "serial_txn.h"
#include "tcp_client.h"
#include "uart_task.h"
#include "diag.h"

#define HEAP_LOW_BYTES      (40 * 1024)
#define RSSI_WEAK           (-75)
#define LINK_DROPS_UNSTABLE 5
#define EXPORT_EVENTS       300

/* ============================================================ 工具 */

void Diag_FormatTime(uint64_t ms, char *buf, size_t size)
{
    time_t sec = (time_t)(ms / 1000);
    struct tm tm;

    if (ms == 0) {
        strlcpy(buf, "-", size);
        return;
    }
    localtime_r(&sec, &tm);
    strftime(buf, size, "%Y-%m-%d %H:%M:%S", &tm);
}

static void FormatDuration(uint32_t s, char *buf, size_t size)
{
    if (s >= 86400) {
        snprintf(buf, size, "%u 天 %u 小时", (unsigned)(s / 86400), (unsigned)(s % 86400 / 3600));
    } else if (s >= 3600) {
        snprintf(buf, size, "%u 小时 %u 分", (unsigned)(s / 3600), (unsigned)(s % 3600 / 60));
    } else if (s >= 60) {
        snprintf(buf, size, "%u 分 %u 秒", (unsigned)(s / 60), (unsigned)(s % 60));
    } else {
        snprintf(buf, size, "%u 秒", (unsigned)s);
    }
}

static uint32_t UptimeS(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

const char *Diag_ResetText(bool *abnormal)
{
    *abnormal = true;
    switch (esp_reset_reason()) {
    case ESP_RST_PANIC:     return "程序异常";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:       return "看门狗（程序卡住）";
    case ESP_RST_BROWNOUT:  return "电源电压不足";
    default:                break;
    }
    *abnormal = false;
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "上电 / 复位键";
    case ESP_RST_SW:        return "软件重启（保存配置、网页重启、升级等）";
    case ESP_RST_EXT:       return "外部复位";
    default:                return "其他";
    }
}

/* ============================================================ 检查项 */

typedef struct {
    cJSON *arr;
    int errors;
    int warns;
} Checks;

static void Add(Checks *c, const char *level, const char *title, const char *advice, const char *fmt, ...)
    __attribute__((format(printf, 5, 6)));

static void Add(Checks *c, const char *level, const char *title, const char *advice, const char *fmt, ...)
{
    char text[160];
    va_list args;
    cJSON *o = cJSON_CreateObject();

    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    cJSON_AddStringToObject(o, "level", level);
    cJSON_AddStringToObject(o, "title", title);
    cJSON_AddStringToObject(o, "text", text);
    cJSON_AddStringToObject(o, "advice", advice != NULL ? advice : "");
    cJSON_AddItemToArray(c->arr, o);
    if (strcmp(level, "error") == 0) {
        c->errors++;
    } else if (strcmp(level, "warn") == 0) {
        c->warns++;
    }
}

static void CheckNetwork(Checks *c, const NetStatus *st)
{
    const GwConfig *cfg = GwConfig_Get();

#ifdef CONFIG_IOT_ETH_W5500
    if (!cfg->ethEnabled) {
        Add(c, "info", "以太网", NULL, "未启用");
    } else if (!st->ethPresent) {
        Add(c, "error", "以太网", "以太网模块故障，请联系厂家。可暂时改用 Wi-Fi。", "以太网模块无响应");
    } else if (!st->ethLink) {
        Add(c, st->wifiUp ? "warn" : "error", "以太网",
            "检查网线两端是否插好、交换机 / 路由器网口指示灯是否亮。如果不使用以太网，请在“设置”页关闭以太网。",
            "未插网线");
    } else if (!st->ethUp) {
        Add(c, "error", "以太网",
            cfg->ethDhcp ? "已插网线但没有分配到 IP：确认路由器开启了 DHCP，或在“设置”页改用固定 IP。"
                         : "检查“设置”页中的固定 IP、子网掩码、网关是否与现场网络一致。",
            "已插网线，但还没有 IP 地址");
    } else {
        Add(c, "ok", "以太网", NULL, "已连接，IP %s（%s）", st->ethIp, cfg->ethDhcp ? "自动获取" : "静态");
    }
#endif

    if (!cfg->wifiEnabled) {
        Add(c, "info", "Wi-Fi", NULL, "未启用");
    } else if (st->wifiUp) {
        if (st->wifiRssi < RSSI_WEAK) {
            Add(c, "warn", "Wi-Fi", "信号弱容易断线：调整天线方向、把网关挪近路由器，或改用以太网。",
                "已连接“%s”，IP %s，信号弱（%d dBm）", cfg->wifiSsid, st->wifiIp, st->wifiRssi);
        } else {
            Add(c, "ok", "Wi-Fi", NULL, "已连接“%s”，IP %s，信号 %d dBm", cfg->wifiSsid, st->wifiIp, st->wifiRssi);
        }
    } else {
        Add(c, "error", "Wi-Fi",
            "确认 Wi-Fi 名称和密码正确（区分大小写）、路由器已开启 2.4G 频段（网关不支持 5G）、网关在信号范围内。",
            "连不上“%s”%s%s", cfg->wifiSsid, st->wifiReason ? "：" : "", NetMgr_WifiReasonText(st->wifiReason));
    }
    if (st->wifiDrops >= LINK_DROPS_UNSTABLE) {
        Add(c, "warn", "Wi-Fi 稳定性", "频繁断线多为信号弱或路由器负载高，建议改善信号或改用以太网。",
            "本次开机后已断开 %u 次", (unsigned)st->wifiDrops);
    }
    if (!st->ethUp && !st->wifiUp) {
        Add(c, "error", "联网", "网关没有联网，无法与上位机通信，请按上面以太网 / Wi-Fi 的提示处理。", "网关未联网");
    }
}

static const char *LinkAdvice(const TcpLinkState *l, bool host)
{
    switch (l->lastErr) {
    case ECONNREFUSED:
    case ECONNRESET:
        return host ? "上位机电脑在线，但这个端口没有程序在接收：确认上位机软件已启动、端口号与上位机设置一致。"
                    : "设备在线但该端口未开放：确认设备已开启以太网通信、端口号正确。";
    case EHOSTUNREACH:
    case ENETUNREACH:
        return "网络不通：确认 IP 地址正确，网关与对方在同一网段（或网关地址设置正确）。";
    default:
        return host ? "上位机没有回应：确认上位机 IP 正确、电脑已开机，Windows 防火墙已放行该端口，网关与上位机网络互通。"
                    : "设备没有回应：确认设备 IP 正确、设备已上电，网线连接正常。";
    }
}

static void CheckLinks(Checks *c, bool netUp)
{
    TcpLinkState links[8];
    int n = TcpClient_GetLinks(links, sizeof(links) / sizeof(links[0]));

    for (int i = 0; i < n; i++) {
        const TcpLinkState *l = &links[i];
        const char *name = TcpClient_LinkName(l->peer);
        bool host = (strcmp(l->tag, "HOST") == 0);

        if (l->off) {
            Add(c, "info", name, NULL, "未启用（端口设为 0）");
        } else if (l->up) {
            if (l->drops >= LINK_DROPS_UNSTABLE) {
                Add(c, "warn", name, "连接不稳定，多为网络质量差或对方程序重启，请检查网络。",
                    "已连接 %s，本次开机后断开过 %u 次", l->addr, (unsigned)l->drops);
            } else {
                Add(c, "ok", name, NULL, "已连接 %s", l->addr);
            }
        } else if (!netUp) {
            Add(c, "warn", name, "网关联网后会自动连接。", "等待联网");
        } else if (l->fails == 0) {
            Add(c, "info", name, NULL, "正在连接");
        } else {
            Add(c, "error", name, LinkAdvice(l, host), "连不上 %s：%s（已重试 %d 次）",
                l->addr, TcpClient_ErrText(l->lastErr), l->fails);
        }
    }
}

static void CheckSerial(Checks *c)
{
    SerialTxnStats s;
    const char *name = UART_InterfaceName() != NULL ? UART_InterfaceName() : "串口";
    const char *params = UART_InterfaceText() != NULL ? UART_InterfaceText() : name;
    bool rs485 = (strcmp(name, "RS485") == 0);
    char advice[320];

    SerialTxn_GetStats(&s);
    if (s.txns == 0) {
        return;     // 没有串口设备，或还没开始轮询
    }
    if (s.offline) {
        // 现场最常见的原因是网关上的 232/485 拨码开关拨错，放在第一条
        if (s.garbled) {
            snprintf(advice, sizeof(advice), "①确认网关上的 232/485 拨码开关拨在 %s 位置；"
                     "②确认设备的波特率、数据位、校验位、站号与网关一致（网关：%s）。", name, params);
        } else {
            snprintf(advice, sizeof(advice), "①确认网关上的 232/485 拨码开关拨在 %s 位置（本设备使用 %s）；%s"
                     "③确认设备已上电、通信口已启用。", name, name,
                     rs485 ? "②检查 A、B 线是否接反、是否接牢；" : "②检查 TX、RX 是否交叉连接，GND 是否接上；");
        }
        Add(c, "error", "串口设备", advice, "%s 设备无应答（已持续 %u 秒）", name, (unsigned)s.offlineS);
    } else if (s.errors > 10 && s.errors > s.ok) {
        Add(c, "warn", "串口设备", "设备有应答但内容有误：确认站号和读取的寄存器地址正确。",
            "应答异常较多（成功 %u，异常 %u）", (unsigned)s.ok, (unsigned)s.errors);
    } else if (s.errors > 10 && s.errors * 100 > s.txns) {
        // 错码超过 1%：多为线路干扰（错码已被校验拦下并重读，不会上报错误数据）
        Add(c, "warn", "串口设备", rs485 ? "线路有干扰：检查 A、B 线是否用双绞屏蔽线、是否接牢，线长时在末端加 120Ω 终端电阻。"
                                         : "线路有干扰：GND 接牢，线尽量短并用屏蔽线；可将设备和网关的波特率一起调低，或改用带校验位的格式（如 7E2）。",
            "应答偶有错码（%s，成功 %u 次，错码 %u 次，超时 %u 次）", params,
            (unsigned)s.ok, (unsigned)s.errors, (unsigned)s.timeouts);
    } else {
        Add(c, "ok", "串口设备", NULL, "通信正常（%s，成功 %u 次，错码 %u 次，超时 %u 次）", params,
            (unsigned)s.ok, (unsigned)s.errors, (unsigned)s.timeouts);
    }
}

// 设备配置（过渡期：没有设备配置时按 menuconfig 采集，只作提示）
static void CheckDevCfg(Checks *c)
{
    const DevCfg *cfg = DevCfg_Get();

    if (cfg == NULL) {
        Add(c, "info", "采集配置", "厂家账号在“采集”页导入设备配置表。", "%s", DevCfg_ProblemText());
    } else if (Acq_ProblemText() != NULL) {
        Add(c, "error", "采集配置", "请联系厂家升级固件，或导入本固件支持的设备配置。", "%s", Acq_ProblemText());
    }
}

static void CheckSystem(Checks *c, const NetStatus *st)
{
    uint32_t posted, dropped, queued;
    bool abnormal;
    const char *reset = Diag_ResetText(&abnormal);

    if (GwConfig_Get()->hostReportPort == 0) {
        Add(c, "info", "数据上报", NULL, "上报通道未启用，采集的数据不会上报");
    } else {
        IOT_GetReportStats(&posted, &dropped, &queued);
        if (dropped > 0) {
            Add(c, "warn", "数据上报", "上报通道断开期间数据会积压，超过 32 条后丢弃最旧的。请保证上报通道连接正常。",
                "已因积压丢弃 %u 条数据（共 %u 条）", (unsigned)dropped, (unsigned)posted);
        }
    }
    if (!IotTime_IsSynced()) {
        Add(c, "warn", "时间", "心跳通道连上上位机后会自动校时。", "尚未与上位机校时，上报数据的时间可能不准");
    }
    if (!GwConfig_IsSaved()) {
        Add(c, "warn", "现场配置", "在“设置”页选好联网方式、填好上位机地址并保存，之后开机不再自动打开热点。",
            "尚未完成现场配置，每次开机都会打开配置热点");
    }
    if (abnormal) {
        Add(c, "warn", "上次重启", esp_reset_reason() == ESP_RST_BROWNOUT
            ? "检查电源适配器和供电线路，电压不稳会导致网关反复重启。"
            : "偶尔一次可以忽略；如果反复出现，请点“导出诊断信息”发给厂家。",
            "上次为异常重启：%s", reset);
    }
    if (esp_get_free_heap_size() < HEAP_LOW_BYTES) {
        Add(c, "warn", "内存", "内存偏低，请导出诊断信息发给厂家。", "剩余内存 %u KB",
            (unsigned)(esp_get_free_heap_size() / 1024));
    }
    if (st->apOn) {
        Add(c, "info", "配置热点", NULL, "已打开：%s，%d 台设备已连接", st->apSsid, st->apClients);
    }
}

cJSON *Diag_Check(int *errors, int *warns)
{
    Checks c = { .arr = cJSON_CreateArray() };
    NetStatus st;

    NetMgr_GetStatus(&st);
    CheckNetwork(&c, &st);
    CheckLinks(&c, st.ethUp || st.wifiUp);
    CheckSerial(&c);
    CheckDevCfg(&c);
    CheckSystem(&c, &st);
    if (errors != NULL) {
        *errors = c.errors;
    }
    if (warns != NULL) {
        *warns = c.warns;
    }
    return c.arr;
}

/* ============================================================ core dump */

bool Diag_CoreDumpInfo(size_t *size)
{
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    size_t addr;

    return esp_core_dump_image_get(&addr, size) == ESP_OK;
#else
    (void)size;
    return false;
#endif
}

bool Diag_CoreDumpRead(size_t offset, void *buf, size_t len)
{
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    size_t addr, size;

    if (esp_core_dump_image_get(&addr, &size) != ESP_OK || offset + len > size) {
        return false;
    }
    return esp_flash_read(NULL, buf, addr + offset, len) == ESP_OK;
#else
    (void)offset; (void)buf; (void)len;
    return false;
#endif
}

void Diag_CoreDumpErase(void)
{
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    esp_core_dump_image_erase();
#endif
}

/* ============================================================ 诊断报告 */

typedef struct {
    DiagWriter write;
    void *ctx;
} Out;

static void Put(Out *o, const char *s)
{
    o->write(o->ctx, s, strlen(s));
}

static void PutCb(void *ctx, const char *s)
{
    Put(ctx, s);
}

static void Printf(Out *o, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static void Printf(Out *o, const char *fmt, ...)
{
    char line[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    Put(o, line);
}

static void ExportHeader(Out *o)
{
    const esp_app_desc_t *app = esp_ota_get_app_description();
    const GwConfig *cfg = GwConfig_Get();
    esp_chip_info_t chip;
    char now[24], up[32], elf[17], mac[GW_STR_MAC], ethMac[GW_STR_MAC];
    bool abnormal;
    const char *reset = Diag_ResetText(&abnormal);

    Diag_FormatTime(IotTime_IsSynced() ? GetMilliTimeNow() : 0, now, sizeof(now));
    FormatDuration(UptimeS(), up, sizeof(up));
    esp_ota_get_app_elf_sha256(elf, sizeof(elf));
    esp_chip_info(&chip);
    GwConfig_GetMacStr(GW_MAC_WIFI, mac, sizeof(mac));
    GwConfig_GetMacStr(GW_MAC_ETH, ethMac, sizeof(ethMac));

    Put(o, "==================== IoT 网关诊断信息 ====================\n");
    Printf(o, "导出时间    : %s%s\n", now, IotTime_IsSynced() ? "" : "（未校时）");
    Printf(o, "设备        : devId %s, devNumber %s, http://%s.local\n", cfg->devId, DevCfg_DevNumber(), NetMgr_GetHostname());
    Printf(o, "固件        : %s, 版本 %s, 编译 %s %s, IDF %s\n", FW_VERSION, app->version, app->date, app->time, app->idf_ver);
    Printf(o, "ELF SHA256  : %s（排查崩溃时用于找到对应的 elf 文件）\n", elf);
    Printf(o, "芯片        : ESP32-S3 rev %d, Wi-Fi MAC %s, 以太网 MAC %s\n", chip.revision, mac, ethMac);
    Printf(o, "运行        : 第 %u 次启动, 已运行 %s, 本次启动原因：%s\n", (unsigned)EvLog_BootNumber(), up, reset);
    Printf(o, "内存        : 剩余 %u, 历史最低 %u, 最大连续块 %u 字节\n", (unsigned)esp_get_free_heap_size(),
           (unsigned)esp_get_minimum_free_heap_size(), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    Printf(o, "采集设备    : %s\n", App_CollectText());
}

static void ExportChecks(Out *o)
{
    static const struct { const char *level; const char *text; } names[] = {
        { "ok", "正常" }, { "info", "提示" }, { "warn", "警告" }, { "error", "错误" },
    };
    cJSON *arr = Diag_Check(NULL, NULL);
    cJSON *it;

    Put(o, "\n---------------- 检查结果 ----------------\n");
    cJSON_ArrayForEach(it, arr) {
        const char *level = cJSON_GetObjectItem(it, "level")->valuestring;
        const char *advice = cJSON_GetObjectItem(it, "advice")->valuestring;
        const char *ln = level;

        for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
            if (strcmp(level, names[i].level) == 0) {
                ln = names[i].text;
            }
        }
        Printf(o, "[%s] %s：%s\n", ln, cJSON_GetObjectItem(it, "title")->valuestring,
               cJSON_GetObjectItem(it, "text")->valuestring);
        if (advice[0] != '\0') {
            Printf(o, "       建议：%s\n", advice);
        }
    }
    cJSON_Delete(arr);
}

static void ExportConfig(Out *o)
{
    cJSON *cfg = GwConfig_ToJson(GwConfig_Get(), GW_AREA_ALL);
    char *text = cJSON_PrintUnformatted(cfg);
    uint32_t baud = 0;
    uart_word_length_t bits;
    uart_parity_t parity;
    uart_stop_bits_t stop;

    Put(o, "\n---------------- 配置（不含密码） ----------------\n");
    Printf(o, "参数来源    : 出厂参数 %s, 现场参数 %s\n", GwConfig_IsFactorySaved() ? "网页设置" : "menuconfig",
           GwConfig_IsSaved() ? "网页设置" : "menuconfig（未配置）");
    if (text != NULL) {
        Put(o, text);
        Put(o, "\n");
        free(text);
    }
    cJSON_Delete(cfg);
    if (uart_is_driver_installed(UART_NUM_1) && uart_get_baudrate(UART_NUM_1, &baud) == ESP_OK &&
        uart_get_word_length(UART_NUM_1, &bits) == ESP_OK &&
        uart_get_parity(UART_NUM_1, &parity) == ESP_OK && uart_get_stop_bits(UART_NUM_1, &stop) == ESP_OK) {
        Printf(o, "串口        : %u %d%c%s\n", (unsigned)baud, 5 + (int)bits,
               parity == UART_PARITY_EVEN ? 'E' : parity == UART_PARITY_ODD ? 'O' : 'N',
               stop == UART_STOP_BITS_2 ? "2" : stop == UART_STOP_BITS_1_5 ? "1.5" : "1");
    } else {
        Put(o, "串口        : 未使用\n");
    }
}

static void ExportStats(Out *o)
{
    TcpLinkState links[8];
    SerialTxnStats s;
    NetStatus st;
    uint32_t posted, dropped, queued;
    int n;

    NetMgr_GetStatus(&st);
    Put(o, "\n---------------- 网络与连接 ----------------\n");
    Printf(o, "以太网      : %s, 网线 %s, IP %s, 网线断开 %u 次\n", st.ethPresent ? "已启动" : "未启动",
           st.ethLink ? "已插" : "未插", st.ethUp ? st.ethIp : "-", (unsigned)st.ethDowns);
    Printf(o, "Wi-Fi       : %s, IP %s, 信号 %d dBm, 断开 %u 次, 最近断开原因 %u（%s）\n",
           st.wifiEnabled ? (st.wifiUp ? "已连接" : "未连接") : "未启用", st.wifiUp ? st.wifiIp : "-",
           st.wifiRssi, (unsigned)st.wifiDrops, st.wifiReason, NetMgr_WifiReasonText(st.wifiReason));
    Printf(o, "配置热点    : %s, %s, 终端 %d\n", st.apOn ? "开" : "关", st.apSsid, st.apClients);

    n = TcpClient_GetLinks(links, sizeof(links) / sizeof(links[0]));
    for (int i = 0; i < n; i++) {
        const TcpLinkState *l = &links[i];

        Printf(o, "%-12s: %s %s, 连上 %u 次, 断开 %u 次, 连续失败 %d 次, 最近错误 errno %d, 状态变化于开机后 %u 秒\n",
               l->peer, l->off ? "停用" : l->up ? "已连接" : "未连接", l->addr[0] ? l->addr : "-",
               (unsigned)l->connects, (unsigned)l->drops, l->fails, l->lastErr, (unsigned)l->sinceS);
    }

    SerialTxn_GetStats(&s);
    Printf(o, "串口事务    : 共 %u, 成功 %u, 超时 %u, 应答有误 %u, %s\n", (unsigned)s.txns, (unsigned)s.ok,
           (unsigned)s.timeouts, (unsigned)s.errors, s.offline ? (s.garbled ? "离线（收到乱码）" : "离线（无数据）") : "在线");
    IOT_GetReportStats(&posted, &dropped, &queued);
    Printf(o, "数据上报    : 提交 %u, 积压丢弃 %u, 当前排队 %u\n", (unsigned)posted, (unsigned)dropped, (unsigned)queued);

    Put(o, "任务剩余栈  :\n");
    App_WriteTaskStacks(PutCb, o);
}

static void ExportCrash(Out *o)
{
    CrashInfo info;
    size_t size;

    Put(o, "\n---------------- 崩溃记录 ----------------\n");
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    esp_core_dump_summary_t *sum = malloc(sizeof(*sum));

    if (sum != NULL && Diag_CoreDumpInfo(&size) && esp_core_dump_get_summary(sum) == ESP_OK) {
        Printf(o, "core dump   : %u 字节, 任务 %s, PC 0x%08x, ELF %.16s\n", (unsigned)size, sum->exc_task,
               (unsigned)sum->exc_pc, (const char *)sum->app_elf_sha256);
        Put(o, "调用栈      :");
        for (uint32_t i = 0; i < sum->exc_bt_info.depth && i < 16; i++) {
            Printf(o, " 0x%08x", (unsigned)sum->exc_bt_info.bt[i]);
        }
        Printf(o, "%s\n", sum->exc_bt_info.corrupted ? "（调用栈已损坏）" : "");
        Put(o, "解析方法    : xtensa-esp32s3-elf-addr2line -pfiaC -e iot_base.elf <上面的地址>\n");
    } else {
        Put(o, "core dump   : 无\n");
    }
    free(sum);
#else
    (void)size;
    Put(o, "core dump   : 本固件未启用\n");
#endif

    if (EvLog_ReadCrash(&info, NULL, 0)) {
        char *buf = malloc(info.len + 1);

        Printf(o, "异常重启前日志（第 %u 次启动，原因 %s，%u 字节）：\n", (unsigned)info.boot, info.reason, (unsigned)info.len);
        if (buf != NULL && EvLog_ReadCrash(&info, buf, info.len + 1)) {
            Put(o, buf);
        }
        free(buf);
        Put(o, "\n");
    } else {
        Put(o, "异常重启前日志：无\n");
    }
}

static void ExportEvents(Out *o)
{
    static const char *const levels[] = { "信息", "警告", "错误" };
    cJSON *arr = EvLog_ToJson(EXPORT_EVENTS, true);
    int n = cJSON_GetArraySize(arr);

    Printf(o, "\n---------------- 事件（最近 %d 条，旧的在前） ----------------\n", n);
    for (int i = n - 1; i >= 0; i--) {
        cJSON *e = cJSON_GetArrayItem(arr, i);
        uint64_t t = (uint64_t)cJSON_GetObjectItem(e, "t")->valuedouble;
        int level = cJSON_GetObjectItem(e, "l")->valueint;
        char ts[24];

        Diag_FormatTime(t, ts, sizeof(ts));
        Printf(o, "#%-4d %-19s +%-7us [%s] %s\n", cJSON_GetObjectItem(e, "b")->valueint, ts,
               (unsigned)cJSON_GetObjectItem(e, "up")->valuedouble, levels[level > 2 ? 2 : level],
               cJSON_GetObjectItem(e, "m")->valuestring);
    }
    cJSON_Delete(arr);
}

static void ExportLogs(Out *o)
{
    char *buf = malloc(LOG_BUF_SIZE);
    uint32_t pos;
    size_t n;

    if (buf == NULL) {
        Put(o, "\n（内存不足，无法导出日志）\n");
        return;
    }
    Put(o, "\n---------------- 开机日志 ----------------\n");
    n = LogBuf_ReadBoot(buf, LOG_BUF_SIZE);
    o->write(o->ctx, buf, n);

    Put(o, "\n---------------- 最近日志 ----------------\n");
    n = LogBuf_Read(0, buf, LOG_BUF_SIZE, &pos);
    o->write(o->ctx, buf, n);
    free(buf);
}

void Diag_Export(DiagWriter write, void *ctx)
{
    Out o = { .write = write, .ctx = ctx };

    ExportHeader(&o);
    ExportChecks(&o);
    ExportConfig(&o);
    ExportStats(&o);
    ExportCrash(&o);
    ExportEvents(&o);
    ExportLogs(&o);
    Put(&o, "\n==================== 结束 ====================\n");
}
