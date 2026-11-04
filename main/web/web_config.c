/*
 * 配置网页：静态页面 + JSON 接口，接口说明见 web_config.h
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "cJSON.h"
#include "sdkconfig.h"
#include "app_info.h"
#include "gw_config.h"
#include "devcfg.h"
#include "proto_lib.h"
#include "net_manager.h"
#include "tcp_client.h"
#include "web_config.h"
#include "event_log.h"
#include "log_buffer.h"
#include "diag.h"
#include "monitor.h"
#include "uart_task.h"
#include "acq.h"
#include "host_cmd.h"
#include "iot_time.h"
#include <time.h>

static const char *TAG = "WEB";

#define BODY_MAX            2048
#define SESSION_IDLE_MS     (30 * 60 * 1000)
#define LOGIN_MAX_FAILS     5
#define LOGIN_LOCK_MS       60000
#define SCAN_MAX            20
#define RESTART_DELAY_MS    1500        // 先把应答发出去再重启

/* 出厂密码（menuconfig，所有设备相同，只给生产 / 售后人员）：登录后可修改出厂参数 */
#ifndef CONFIG_IOT_FACTORY_PASSWORD
// 新增 Kconfig 选项后 build/config/sdkconfig.h 没有重新生成，静默当作空密码会导致出厂登录被禁用
#error "CONFIG_IOT_FACTORY_PASSWORD missing: run idf.py reconfigure, then build again"
#endif
#define FACTORY_PASS        CONFIG_IOT_FACTORY_PASSWORD

#define USER_ADMIN          "admin"         // 客户账号，密码为运行参数 webPass
#define USER_FACTORY        "factory"       // 厂家账号，密码为出厂密码

/* main/web/index.html，CMakeLists.txt 中以 EMBED_TXTFILES 嵌入（末尾带 '\0'） */
extern const char s_indexStart[] asm("_binary_index_html_start");
extern const char s_indexEnd[] asm("_binary_index_html_end");

/* 会话：同一时间只有一个有效令牌，所有请求都在 httpd 任务中处理，无需加锁 */
static char s_token[33];
static bool s_tokenFactory;      // 用出厂密码登录：可修改出厂参数
static int64_t s_tokenUsedMs;
static int s_loginFails;
static int64_t s_lockUntilMs;

static int64_t NowMs(void)
{
    return esp_timer_get_time() / 1000;
}

/* ------------------------------------------------------------ 应答工具 */

// 申请内存失败：记下当时的内存情况便于排查，应答不再申请内存
static esp_err_t NoMemory(httpd_req_t *req)
{
    ESP_LOGW(TAG, "%s: no memory (free %u, largest block %u, min free %u)", req->uri,
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
             (unsigned)esp_get_minimum_free_heap_size());
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"网关内存不足，请稍后重试\"}");
}


// 发送并释放 root
static esp_err_t SendJson(httpd_req_t *req, cJSON *root)
{
    char *text = (root != NULL) ? cJSON_PrintUnformatted(root) : NULL;
    esp_err_t err;

    cJSON_Delete(root);
    if (text == NULL) {
        return NoMemory(req);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    err = httpd_resp_sendstr(req, text);
    free(text);
    return err;
}

static esp_err_t SendError(httpd_req_t *req, const char *status, const char *msg)
{
    cJSON *root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "error", msg);
    httpd_resp_set_status(req, status);
    return SendJson(req, root);
}

static esp_err_t SendOk(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();

    cJSON_AddBoolToObject(root, "ok", true);
    return SendJson(req, root);
}

// 读取请求体并解析为 JSON，失败时已发送错误应答并返回 NULL
static cJSON *ReadJson(httpd_req_t *req)
{
    char *buf;
    size_t got = 0;
    cJSON *json;

    if (req->content_len == 0 || req->content_len > BODY_MAX) {
        SendError(req, "400 Bad Request", "请求内容为空或过长");
        return NULL;
    }
    buf = malloc(req->content_len + 1);
    if (buf == NULL) {
        NoMemory(req);
        return NULL;
    }
    while (got < req->content_len) {
        int n = httpd_req_recv(req, buf + got, req->content_len - got);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (n <= 0) {
            free(buf);
            return NULL;    // 连接已断开，无法应答
        }
        got += n;
    }
    buf[got] = '\0';
    json = cJSON_Parse(buf);
    free(buf);
    if (!cJSON_IsObject(json)) {
        cJSON_Delete(json);
        SendError(req, "400 Bad Request", "请求格式不正确");
        return NULL;
    }
    return json;
}

/* ------------------------------------------------------------ 访问控制 */

// 请求是否从配置热点进来：看本端地址是不是热点 IP（httpd 用 IPv6 双栈 socket，IPv4 以映射地址出现）
static bool FromConfigAp(httpd_req_t *req)
{
    struct sockaddr_storage addr;
    socklen_t len = sizeof(addr);
    uint32_t ip4 = 0;

    if (getsockname(httpd_req_to_sockfd(req), (struct sockaddr *)&addr, &len) != 0) {
        return false;
    }
    if (addr.ss_family == AF_INET) {
        ip4 = ((const struct sockaddr_in *)&addr)->sin_addr.s_addr;
    }
#if LWIP_IPV6
    else if (addr.ss_family == AF_INET6) {
        const struct sockaddr_in6 *a6 = (const struct sockaddr_in6 *)&addr;

        if (a6->sin6_addr.un.u32_addr[0] == 0 && a6->sin6_addr.un.u32_addr[1] == 0 &&
            a6->sin6_addr.un.u32_addr[2] == htonl(0xFFFF)) {
            ip4 = a6->sin6_addr.un.u32_addr[3];
        }
    }
#endif
    return ip4 == ipaddr_addr(NET_AP_IP);
}

// 打开了"仅允许热点访问"时拒绝局域网来的请求，已发送应答时返回 false
static bool CheckSource(httpd_req_t *req)
{
    static const char *const why = "配置网页已设置为仅允许通过配置热点访问：请在 5 秒内连续按 3 次复位键打开热点 IoTGW-xxxx，"
                                   "连接后访问 http://" NET_AP_IP;

    if (!GwConfig_Get()->webApOnly || FromConfigAp(req)) {
        return true;
    }
    if (strncmp(req->uri, "/api/", 5) == 0) {
        SendError(req, "403 Forbidden", why);
    } else {
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        httpd_resp_sendstr(req, why);
    }
    return false;
}

/* ------------------------------------------------------------ 登录 */

static bool CheckAuth(httpd_req_t *req)
{
    char token[sizeof(s_token)];

    if (!CheckSource(req)) {
        return false;
    }
    if (s_token[0] != '\0' && NowMs() - s_tokenUsedMs > SESSION_IDLE_MS) {
        s_token[0] = '\0';      // 长时间无操作，令牌失效
    }
    if (s_token[0] == '\0' ||
        httpd_req_get_hdr_value_str(req, "X-Token", token, sizeof(token)) != ESP_OK ||
        strcmp(token, s_token) != 0) {
        SendError(req, "401 Unauthorized", "未登录或登录已过期");
        return false;
    }
    s_tokenUsedMs = NowMs();
    return true;
}

static esp_err_t LoginHandler(httpd_req_t *req)
{
    const cJSON *pass;
    cJSON *json;
    cJSON *root;
    uint8_t rnd[16];
    const cJSON *user;
    bool factory;
    bool ok = false;

    if (!CheckSource(req)) {
        return ESP_OK;
    }
    if (NowMs() < s_lockUntilMs) {
        return SendError(req, "429 Too Many Requests", "密码错误次数过多，请 1 分钟后再试");
    }
    json = ReadJson(req);
    if (json == NULL) {
        return ESP_OK;
    }
    // 两个固定账号：admin（客户，密码 webPass）、factory（厂家，密码 menuconfig 出厂密码）
    user = cJSON_GetObjectItemCaseSensitive(json, "username");
    pass = cJSON_GetObjectItemCaseSensitive(json, "password");
    factory = cJSON_IsString(user) && strcmp(user->valuestring, USER_FACTORY) == 0;
    if (cJSON_IsString(user) && cJSON_IsString(pass)) {
        if (factory) {
            ok = FACTORY_PASS[0] != '\0' && strcmp(pass->valuestring, FACTORY_PASS) == 0;
        } else if (strcmp(user->valuestring, USER_ADMIN) == 0) {
            ok = strcmp(pass->valuestring, GwConfig_Get()->webPass) == 0;
        }
    }
    if (!ok) {
        // 用户名可能是任意内容，日志里只区分是否为已知账号
        ESP_LOGW(TAG, "login failed: %s", factory ? USER_FACTORY :
                 (cJSON_IsString(user) && strcmp(user->valuestring, USER_ADMIN) == 0) ? USER_ADMIN : "unknown user");
        cJSON_Delete(json);
        if (++s_loginFails >= LOGIN_MAX_FAILS) {
            s_loginFails = 0;
            s_lockUntilMs = NowMs() + LOGIN_LOCK_MS;
            EvLog_Add(EV_WARN, "网页登录密码连续错误 %d 次，锁定 1 分钟", LOGIN_MAX_FAILS);
        }
        return SendError(req, "401 Unauthorized", "用户名或密码错误");
    }
    cJSON_Delete(json);

    s_loginFails = 0;
    esp_fill_random(rnd, sizeof(rnd));
    for (int i = 0; i < 16; i++) {
        snprintf(&s_token[i * 2], 3, "%02x", rnd[i]);
    }
    s_tokenFactory = factory;
    s_tokenUsedMs = NowMs();
    ESP_LOGI(TAG, "login ok: %s", factory ? USER_FACTORY : USER_ADMIN);
    EvLog_Add(EV_INFO, "网页登录：%s%s", factory ? USER_FACTORY : USER_ADMIN, FromConfigAp(req) ? "（经配置热点）" : "");

    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "token", s_token);
    cJSON_AddStringToObject(root, "role", factory ? "factory" : "user");
    return SendJson(req, root);
}

/* ------------------------------------------------------------ 状态 / 配置 */

static esp_err_t StatusHandler(httpd_req_t *req)
{
    const GwConfig *cfg = GwConfig_Get();
    TcpLinkState links[8];
    NetStatus st;
    char mac[GW_STR_MAC];
    cJSON *root;
    cJSON *obj;
    cJSON *arr;
    int n;

    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    NetMgr_GetStatus(&st);

    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "fw", FW_VERSION);
    cJSON_AddStringToObject(root, "build", __DATE__ " " __TIME__);
    cJSON_AddStringToObject(root, "idf", esp_get_idf_version());
    cJSON_AddNumberToObject(root, "uptime", (double)(esp_timer_get_time() / 1000000));
    cJSON_AddNumberToObject(root, "heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "heapMin", esp_get_minimum_free_heap_size());
    cJSON_AddNumberToObject(root, "heapBlock", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    cJSON_AddStringToObject(root, "reset", App_ResetText());
    cJSON_AddStringToObject(root, "collect", App_CollectText());
    if (UART_InterfaceText() != NULL) {
        cJSON_AddStringToObject(root, "serial", UART_InterfaceText());
        cJSON_AddStringToObject(root, "serialIf", UART_InterfaceName() != NULL ? UART_InterfaceName() : "");
    }
    cJSON_AddBoolToObject(root, "saved", GwConfig_IsSaved());
    cJSON_AddBoolToObject(root, "factorySaved", GwConfig_IsFactorySaved());
    {
        int errors = 0;
        int warns = 0;

        cJSON_Delete(Diag_Check(&errors, &warns));     // 状态页顶部显示"有几个问题"
        cJSON_AddNumberToObject(root, "diagErrors", errors);
        cJSON_AddNumberToObject(root, "diagWarns", warns);
    }
    cJSON_AddStringToObject(root, "role", s_tokenFactory ? "factory" : "user");
    cJSON_AddStringToObject(root, "devId", cfg->devId);
    cJSON_AddStringToObject(root, "devNumber", DevCfg_DevNumber());
    cJSON_AddStringToObject(root, "hostAddr", cfg->hostAddr);
    cJSON_AddStringToObject(root, "hostname", NetMgr_GetHostname());
    cJSON_AddBoolToObject(root, "viaAp", FromConfigAp(req));     // 当前是否经配置热点访问

    obj = cJSON_AddObjectToObject(root, "eth");
    GwConfig_GetMacStr(GW_MAC_ETH, mac, sizeof(mac));
    cJSON_AddBoolToObject(obj, "enabled", cfg->ethEnabled);
    cJSON_AddBoolToObject(obj, "present", st.ethPresent);
    cJSON_AddBoolToObject(obj, "link", st.ethLink);
    cJSON_AddBoolToObject(obj, "up", st.ethUp);
    cJSON_AddStringToObject(obj, "ip", st.ethUp ? st.ethIp : "");
    cJSON_AddStringToObject(obj, "mac", mac);
    cJSON_AddBoolToObject(obj, "dhcp", cfg->ethDhcp);

    obj = cJSON_AddObjectToObject(root, "wifi");
    GwConfig_GetMacStr(GW_MAC_WIFI, mac, sizeof(mac));
    cJSON_AddBoolToObject(obj, "enabled", cfg->wifiEnabled);
    cJSON_AddStringToObject(obj, "ssid", cfg->wifiSsid);
    cJSON_AddBoolToObject(obj, "up", st.wifiUp);
    cJSON_AddStringToObject(obj, "ip", st.wifiUp ? st.wifiIp : "");
    cJSON_AddNumberToObject(obj, "rssi", st.wifiRssi);
    cJSON_AddStringToObject(obj, "mac", mac);

    obj = cJSON_AddObjectToObject(root, "ap");
    cJSON_AddBoolToObject(obj, "on", st.apOn);
    cJSON_AddNumberToObject(obj, "clients", st.apClients);
    cJSON_AddStringToObject(obj, "ssid", st.apSsid);
    cJSON_AddStringToObject(obj, "reason", NetMgr_ApReasonText(st.apReason));

    arr = cJSON_AddArrayToObject(root, "links");
    n = TcpClient_GetLinks(links, sizeof(links) / sizeof(links[0]));
    for (int i = 0; i < n; i++) {
        obj = cJSON_CreateObject();
        cJSON_AddStringToObject(obj, "tag", links[i].tag);
        cJSON_AddStringToObject(obj, "peer", links[i].peer);
        cJSON_AddBoolToObject(obj, "up", links[i].up);
        cJSON_AddBoolToObject(obj, "off", links[i].off);
        cJSON_AddItemToArray(arr, obj);
    }
    return SendJson(req, root);
}

static esp_err_t ConfigGetHandler(httpd_req_t *req)
{
    cJSON *root;
    cJSON *obj;

    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    root = cJSON_CreateObject();
    // 出厂参数也返回（客户只能查看），修改权限由 POST 接口控制
    cJSON_AddItemToObject(root, "config", GwConfig_ToJson(GwConfig_Get(), GW_AREA_ALL));
    {
        uint8_t mac[6];
        char chip[GW_STR_MAC];

        esp_read_mac(mac, ESP_MAC_ETH);     // ethMac 留空时使用的芯片 MAC
        snprintf(chip, sizeof(chip), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        cJSON_AddStringToObject(root, "ethMacChip", chip);
    }
    cJSON_AddStringToObject(root, "role", s_tokenFactory ? "factory" : "user");

    // 本固件编译进来的功能，网页据此显示 / 隐藏对应的设置项
    obj = cJSON_AddObjectToObject(root, "features");
#ifdef CONFIG_IOT_ETH_W5500
    cJSON_AddBoolToObject(obj, "eth", true);
#endif
#ifdef CONFIG_PLC_FX_CONNECT_NETWORK
    cJSON_AddBoolToObject(obj, "fx", true);
#endif
#ifdef CONFIG_PLC_PPI_CONNECT_NETWORK
    cJSON_AddBoolToObject(obj, "s7", true);
#endif
#ifdef CONFIG_PLC_MODBUS_CONNECT_NETWORK
    cJSON_AddBoolToObject(obj, "modbus", true);
#endif
#ifdef CONFIG_IOT_FILE_PORT
    cJSON_AddBoolToObject(obj, "filePort", true);
#endif
#ifdef CONFIG_IOT_HOST_IPV6
    cJSON_AddBoolToObject(obj, "ipv6", true);
#endif
    return SendJson(req, root);
}

static void RestartTask(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
    esp_restart();
}

static void ScheduleRestart(const char *why)
{
    ESP_LOGW(TAG, "%s, restarting", why);
    xTaskCreate(RestartTask, "web_restart", 2048, NULL, 5, NULL);
}

// 用请求中的字段修改 area 区参数，校验后保存并重启
static esp_err_t SaveArea(httpd_req_t *req, GwArea area)
{
    GwConfig *cfg;
    const char *why = NULL;
    cJSON *json;
    esp_err_t err;

    json = ReadJson(req);
    if (json == NULL) {
        return ESP_OK;
    }
    cfg = malloc(sizeof(*cfg));     // 结构体约 600 字节，不放 httpd 任务栈上
    if (cfg == NULL) {
        cJSON_Delete(json);
        return NoMemory(req);
    }
    *cfg = *GwConfig_Get();
    err = GwConfig_FromJson(cfg, json, area, &why);
    cJSON_Delete(json);
    if (err == ESP_OK) {
        err = GwConfig_Validate(cfg, &why);
    }
    if (err == ESP_OK) {
        err = GwConfig_Save(cfg, area);
        if (err != ESP_OK) {
            why = "写入存储失败";
        }
    }
    free(cfg);
    if (err != ESP_OK) {
        return SendError(req, "400 Bad Request", why != NULL ? why : "保存失败");
    }
    EvLog_Add(EV_INFO | EV_OP, "%s已修改（%s），重启生效", (area == GW_AREA_FACTORY) ? "出厂参数" : "现场参数",
              s_tokenFactory ? USER_FACTORY : USER_ADMIN);
    SendOk(req);
    ScheduleRestart((area == GW_AREA_FACTORY) ? "factory config saved from web" : "config saved from web");
    return ESP_OK;
}

static esp_err_t ConfigPostHandler(httpd_req_t *req)
{
    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    return SaveArea(req, GW_AREA_USER);
}

static esp_err_t FactoryConfigPostHandler(httpd_req_t *req)
{
    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    if (!s_tokenFactory) {
        return SendError(req, "403 Forbidden", "需要用出厂密码登录");
    }
    return SaveArea(req, GW_AREA_FACTORY);
}

static esp_err_t ScanHandler(httpd_req_t *req)
{
    NetWifiAp *aps;
    cJSON *arr;
    int n;

    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    aps = calloc(SCAN_MAX, sizeof(*aps));
    if (aps == NULL) {
        return NoMemory(req);
    }
    n = NetMgr_ScanWifi(aps, SCAN_MAX);
    if (n < 0) {
        free(aps);
        return SendError(req, "503 Service Unavailable", "扫描失败，请稍后再试");
    }
    arr = cJSON_CreateArray();
    for (int i = 0; i < n; i++) {
        cJSON *obj;

        if (aps[i].ssid[0] == '\0') {
            continue;
        }
        obj = cJSON_CreateObject();
        cJSON_AddStringToObject(obj, "ssid", aps[i].ssid);
        cJSON_AddNumberToObject(obj, "rssi", aps[i].rssi);
        cJSON_AddBoolToObject(obj, "secure", aps[i].secure);
        cJSON_AddItemToArray(arr, obj);
    }
    free(aps);
    return SendJson(req, arr);
}

static esp_err_t RebootHandler(httpd_req_t *req)
{
    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    EvLog_Add(EV_INFO | EV_OP, "网页操作重启（%s）", s_tokenFactory ? USER_FACTORY : USER_ADMIN);
    SendOk(req);
    ScheduleRestart("restart requested from web");
    return ESP_OK;
}

static esp_err_t FactoryHandler(httpd_req_t *req)
{
    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    if (GwConfig_Reset() != ESP_OK) {
        return SendError(req, "500 Internal Server Error", "清除配置失败");
    }
    EvLog_Add(EV_WARN | EV_OP, "恢复出厂设置（%s），现场参数已清除", s_tokenFactory ? USER_FACTORY : USER_ADMIN);
    SendOk(req);
    ScheduleRestart("factory reset from web");
    return ESP_OK;
}

/* ------------------------------------------------------------ 诊断 / 日志 */

static bool RequireFactory(httpd_req_t *req)
{
    if (!s_tokenFactory) {
        SendError(req, "403 Forbidden", "需要用厂家账号登录");
        return false;
    }
    return true;
}

static esp_err_t DiagHandler(httpd_req_t *req)
{
    int errors = 0;
    int warns = 0;
    cJSON *root;

    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "items", Diag_Check(&errors, &warns));
    cJSON_AddNumberToObject(root, "errors", errors);
    cJSON_AddNumberToObject(root, "warns", warns);
    cJSON_AddNumberToObject(root, "boot", EvLog_BootNumber());
    cJSON_AddNumberToObject(root, "uptime", (double)(esp_timer_get_time() / 1000000));
    cJSON_AddNumberToObject(root, "now", IotTime_IsSynced() ? (double)GetMilliTimeNow() : 0);
    {
        CrashInfo info;
        size_t size = 0;

        // 崩溃记录：有才返回对应对象，网页据此决定显示哪些操作
        if (EvLog_ReadCrash(&info, NULL, 0)) {
            cJSON *c = cJSON_AddObjectToObject(root, "crashLog");

            cJSON_AddNumberToObject(c, "boot", info.boot);
            cJSON_AddStringToObject(c, "reason", info.reason);
            cJSON_AddNumberToObject(c, "len", info.len);
        }
        if (Diag_CoreDumpInfo(&size)) {
            cJSON_AddNumberToObject(cJSON_AddObjectToObject(root, "coreDump"), "size", size);
        }
    }
    return SendJson(req, root);
}

static esp_err_t EventsHandler(httpd_req_t *req)
{
    char query[16];
    char val[8];
    int n = 200;

    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "n", val, sizeof(val)) == ESP_OK) {
        n = atoi(val);
        if (n <= 0 || n > 500) {
            n = 200;
        }
    }
    return SendJson(req, EvLog_ToJson(n, s_tokenFactory));     // 客户账号不返回 EV_DEV 事件
}

static esp_err_t EventsClearHandler(httpd_req_t *req)
{
    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    if (s_tokenFactory) {
        EvLog_Clear();      // 厂家：真正清空
        EvLog_Add(EV_INFO | EV_OP, "事件记录已清空（factory）");
    } else {
        // 客户：只写一条标记，之前的记录对客户不再显示；厂家排查和导出仍能看到
        EvLog_Add(EV_INFO | EV_DEV | EV_ACK, "客户在网页清除了事件记录（admin）");
    }
    return SendOk(req);
}

/*
 * 分块发送，攒满 1 KB 发一次。httpd 只有一个任务、逐个处理请求，缓冲用静态的一份即可，
 * 不在每次请求时申请（采集页每 2 秒刷新，内存紧张时申请失败会返回 500）
 */
typedef struct {
    httpd_req_t *req;
    size_t n;
    char buf[1024];
} ChunkOut;

static ChunkOut s_chunk;

static void ChunkWrite(void *ctx, const char *text, size_t len)
{
    ChunkOut *o = ctx;

    while (len > 0) {
        size_t room = sizeof(o->buf) - o->n;
        size_t k = (len < room) ? len : room;

        memcpy(o->buf + o->n, text, k);
        o->n += k;
        text += k;
        len -= k;
        if (o->n == sizeof(o->buf)) {
            httpd_resp_send_chunk(o->req, o->buf, o->n);
            o->n = 0;
        }
    }
}

static ChunkOut *BeginChunks(httpd_req_t *req, const char *type)
{
    ChunkOut *o = &s_chunk;

    o->req = req;
    o->n = 0;
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return o;
}

static esp_err_t EndChunks(ChunkOut *o)
{
    if (o->n > 0) {
        httpd_resp_send_chunk(o->req, o->buf, o->n);
    }
    httpd_resp_send_chunk(o->req, NULL, 0);
    return ESP_OK;
}

// 实时日志：返回 from 之后新增的日志，响应头 X-Log-Pos 为下次请求的 from
static esp_err_t LogHandler(httpd_req_t *req)
{
    static char posText[12];
    char query[24];
    char val[12];
    uint32_t from = 0;
    uint32_t end = LogBuf_Pos();        // 只发到此刻为止，发送期间新增的留给下次
    uint32_t pos;
    ChunkOut *o;

    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "from", val, sizeof(val)) == ESP_OK) {
        from = (uint32_t)strtoul(val, NULL, 10);
    }
    o = BeginChunks(req, "text/plain; charset=utf-8");
    snprintf(posText, sizeof(posText), "%u", (unsigned)end);
    httpd_resp_set_hdr(req, "X-Log-Pos", posText);
    while (from < end) {
        size_t n = LogBuf_Read(from, o->buf, sizeof(o->buf), &pos);

        if (n == 0) {
            break;
        }
        if (pos > end) {                // 读的过程中有新日志，截到 end
            n -= pos - end;
            pos = end;
        }
        httpd_resp_send_chunk(req, o->buf, n);
        from = pos;
    }
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t ExportHandler(httpd_req_t *req)
{
    static char disposition[96];
    ChunkOut *o;
    char ts[24] = "";

    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    if (IotTime_IsSynced()) {
        time_t sec = (time_t)(GetMilliTimeNow() / 1000);
        struct tm tm;

        localtime_r(&sec, &tm);
        strftime(ts, sizeof(ts), "-%Y%m%d-%H%M", &tm);
    }
    snprintf(disposition, sizeof(disposition), "attachment; filename=\"iotgw-%s%s.txt\"", NetMgr_GetHostname(), ts);
    o = BeginChunks(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);

    ESP_LOGI(TAG, "export diagnostics");
    Diag_Export(ChunkWrite, o);
    return EndChunks(o);
}

// 异常重启前日志（厂家账号）
static esp_err_t CrashHandler(httpd_req_t *req)
{
    CrashInfo info;
    char *buf;
    esp_err_t err;

    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    if (!EvLog_ReadCrash(&info, NULL, 0)) {
        return SendError(req, "404 Not Found", "没有异常重启记录");
    }
    buf = malloc(info.len + 128);
    if (buf == NULL) {
        return NoMemory(req);
    }
    int head = snprintf(buf, 128, "第 %u 次启动异常重启（%s），重启前日志：\n\n", (unsigned)info.boot, info.reason);
    EvLog_ReadCrash(&info, buf + head, info.len + 1);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    err = httpd_resp_send(req, buf, head + info.len);
    free(buf);
    return err;
}

static esp_err_t CrashClearHandler(httpd_req_t *req)
{
    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    EvLog_ClearCrash();
    Diag_CoreDumpErase();
    EvLog_Add(EV_INFO | EV_DEV, "崩溃记录已清除（factory）");
    return SendOk(req);
}

// core dump 原始文件（厂家账号），用 espcoredump.py info_corefile -t raw -c <文件> iot_base.elf 解析
static esp_err_t CoreDumpHandler(httpd_req_t *req)
{
    char *buf;
    size_t size;

    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    if (!Diag_CoreDumpInfo(&size)) {
        return SendError(req, "404 Not Found", "没有 core dump");
    }
    buf = malloc(1024);
    if (buf == NULL) {
        return NoMemory(req);
    }
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"coredump.bin\"");
    for (size_t off = 0; off < size; off += 1024) {
        size_t n = (size - off < 1024) ? size - off : 1024;

        if (!Diag_CoreDumpRead(off, buf, n) || httpd_resp_send_chunk(req, buf, n) != ESP_OK) {
            break;
        }
    }
    httpd_resp_send_chunk(req, NULL, 0);
    free(buf);
    return ESP_OK;
}

/* ------------------------------------------------------------ 报文监视（厂家账号） */

static esp_err_t MonitorGetHandler(httpd_req_t *req)
{
    char query[24];
    char val[12];
    uint32_t after = 0;

    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "after", val, sizeof(val)) == ESP_OK) {
        after = (uint32_t)strtoul(val, NULL, 10);
    }
    return SendJson(req, Mon_ToJson(after, 12 * 1024));
}

static esp_err_t MonitorStartHandler(httpd_req_t *req)
{
    const cJSON *mask;
    const cJSON *minutes;
    cJSON *json;
    uint32_t m;

    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    json = ReadJson(req);
    if (json == NULL) {
        return ESP_OK;
    }
    mask = cJSON_GetObjectItemCaseSensitive(json, "mask");
    minutes = cJSON_GetObjectItemCaseSensitive(json, "minutes");
    m = cJSON_IsNumber(minutes) ? (uint32_t)minutes->valueint : 10;
    if (m < 1 || m > 60) {
        m = 10;
    }
    if (!cJSON_IsNumber(mask) || mask->valueint <= 0) {
        cJSON_Delete(json);
        return SendError(req, "400 Bad Request", "请至少选择一个通道");
    }
    Mon_Start((uint32_t)mask->valueint, m);
    cJSON_Delete(json);
    EvLog_Add(EV_INFO | EV_DEV, "开始报文监视（%u 分钟）", (unsigned)m);
    return SendOk(req);
}

static esp_err_t MonitorStopHandler(httpd_req_t *req)
{
    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    Mon_Stop();
    return SendOk(req);
}

/* ------------------------------------------------------------ 设备配置（格式见 docs/devcfg.md） */

// 当前配置摘要（客户账号只读查看）
static esp_err_t DevCfgGetHandler(httpd_req_t *req)
{
    ChunkOut *o;

    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    o = BeginChunks(req, "application/json; charset=utf-8");
    DevCfg_WriteJson(ChunkWrite, o);
    return EndChunks(o);
}

// 下载配置原文（厂家账号）
static esp_err_t DevCfgFileHandler(httpd_req_t *req)
{
    static char disposition[80];
    ChunkOut *o;

    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    o = BeginChunks(req, "application/x-ndjson; charset=utf-8");
    snprintf(disposition, sizeof(disposition), "attachment; filename=\"devcfg-%s.jsonl\"", NetMgr_GetHostname());
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);
    esp_err_t err = DevCfg_WriteFile(ChunkWrite, o);
    if (err == ESP_ERR_NOT_FOUND) {
        // 还没有发送任何内容，可以改发错误应答
        httpd_resp_set_hdr(req, "Content-Disposition", "inline");
        return SendError(req, "404 Not Found", "没有采集配置");
    }
    return EndChunks(o);
}

// 导入配置（厂家账号）：?check=1 只校验；否则校验通过后保存并重启
static esp_err_t DevCfgPostHandler(httpd_req_t *req)
{
    char query[16];
    char val[4];
    bool check = false;
    char *buf;
    size_t got = 0;

    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "check", val, sizeof(val)) == ESP_OK) {
        check = (strcmp(val, "1") == 0);
    }
    if (req->content_len == 0 || req->content_len > 60 * 1024) {
        return SendError(req, "400 Bad Request", "配置文件为空或超过 60 KB");
    }
    buf = malloc(1024);
    DevCfgImport *imp = (buf != NULL) ? DevCfg_ImportBegin(!check) : NULL;
    if (imp == NULL) {
        free(buf);
        return NoMemory(req);
    }
    while (got < req->content_len) {
        size_t want = req->content_len - got;
        int n = httpd_req_recv(req, buf, want < 1024 ? want : 1024);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (n <= 0) {
            break;      // 连接已断开：按不完整处理，不会提交
        }
        DevCfg_ImportFeed(imp, buf, (size_t)n);
        got += (size_t)n;
    }
    free(buf);
    if (got < req->content_len) {
        DevCfg_ImportFeed(imp, "\n{\"t\":\"?\"}", 10);     // 收到的不完整：加一行无效内容使校验失败，不会提交
    }

    cJSON *errors = NULL;
    cJSON *warnings = NULL;
    const char *summary = NULL;
    esp_err_t err = DevCfg_ImportEnd(imp, &errors, &warnings, &summary);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "warnings", warnings);
    if (err != ESP_OK) {
        cJSON_AddStringToObject(root, "error", summary);
        cJSON_AddItemToObject(root, "errors", errors);
        httpd_resp_set_status(req, "400 Bad Request");
        return SendJson(req, root);
    }
    cJSON_Delete(errors);
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddBoolToObject(root, "check", check);
    cJSON_AddStringToObject(root, "summary", summary);
    if (check) {
        return SendJson(req, root);
    }
    EvLog_Add(EV_INFO | EV_OP, "采集配置已导入（%s），重启生效", summary);
    SendJson(req, root);
    ScheduleRestart("device config imported");
    return ESP_OK;
}

static esp_err_t DevCfgClearHandler(httpd_req_t *req)
{
    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    if (DevCfg_Clear() != ESP_OK) {
        return SendError(req, "500 Internal Server Error", "删除失败");
    }
    EvLog_Add(EV_WARN | EV_OP, "采集配置已删除（factory），重启后停止采集");
    SendOk(req);
    ScheduleRestart("device config cleared");
    return ESP_OK;
}

static esp_err_t ProtocolsHandler(httpd_req_t *req)
{
    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "schema", 1);
    cJSON_AddItemToObject(root, "protocols", ProtoLib_ToJson());
    return SendJson(req, root);
}

/*
 * 实时数据（客户、厂家账号都可查看），points 与 /api/devcfg 的点表一一对应：
 *   [状态, 值, 秒前]  状态 0 未读，1 正常，-1 最近一次读取失败；值为最近一次读到的值
 */
static esp_err_t AcqHandler(httpd_req_t *req)
{
    const DevCfg *c = DevCfg_Get();
    const char *problem = Acq_ProblemText();
    char line[128];
    char text[32];
    uint32_t age;
    ChunkOut *o;
    int n;

    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    o = BeginChunks(req, "application/json; charset=utf-8");
    n = snprintf(line, sizeof(line), "{\"running\":%s,\"paused\":%s,\"linkDown\":%s,\"roundAge\":%ld,\"problem\":",
                 Acq_IsRunning() ? "true" : "false", HostCmd_IsCollectEnabled() ? "false" : "true",
                 Acq_IsLinkDown() ? "true" : "false",
                 (long)Acq_RoundAgeS());
    ChunkWrite(o, line, (size_t)n);
    if (problem != NULL) {
        cJSON *item = cJSON_CreateString(problem);     // 借 cJSON 转义
        char *s = (item != NULL) ? cJSON_PrintUnformatted(item) : NULL;

        ChunkWrite(o, (s != NULL) ? s : "null", strlen((s != NULL) ? s : "null"));
        cJSON_free(s);
        cJSON_Delete(item);
    } else {
        ChunkWrite(o, "null", 4);
    }
    ChunkWrite(o, ",\"points\":[", 11);
    for (uint16_t i = 0; c != NULL && i < c->count; i++) {
        int st = Acq_GetPointText(i, text, sizeof(text), &age);
        n = snprintf(line, sizeof(line), "%s[%d,\"%s\",%lu]", (i > 0) ? "," : "", st, text, (unsigned long)age);
        ChunkWrite(o, line, (size_t)n);
    }
    ChunkWrite(o, "]}", 2);
    return EndChunks(o);
}

/*
 * 试读（厂家账号）：请求为一个点（同 point 行，fr、name 可省略），按当前设备读一次。
 * 应答 {"ok":true/false,"addr":"D100","value":"12.5","raw":"0x007D","error":"..."}，读取失败也返回 200
 */
static esp_err_t AcqTryHandler(httpd_req_t *req)
{
    DevCfgPoint pt;
    char err[200];
    char value[32];
    char raw[16] = "";
    char addr[24] = "";
    const char *fail;
    cJSON *json;
    cJSON *root;

    if (!CheckAuth(req) || !RequireFactory(req)) {
        return ESP_OK;
    }
    json = ReadJson(req);
    if (json == NULL) {
        return ESP_OK;
    }
    memset(&pt, 0, sizeof(pt));
    if (!DevCfg_ParseTryPoint(json, &pt, err, sizeof(err))) {
        fail = err;
    } else {
        DevCfg_PointAddrText(&DevCfg_Get()->dev, &pt, addr, sizeof(addr));
        fail = Acq_TryRead(&pt, value, sizeof(value), raw, sizeof(raw));
        ESP_LOGI(TAG, "try read %s: %s", addr, (fail != NULL) ? "failed" : value);
    }
    cJSON_Delete(json);

    root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", fail == NULL);
    cJSON_AddStringToObject(root, "addr", addr);
    if (fail == NULL) {
        cJSON_AddStringToObject(root, "value", value);
    } else {
        cJSON_AddStringToObject(root, "error", fail);
    }
    cJSON_AddStringToObject(root, "raw", raw);
    return SendJson(req, root);
}

static esp_err_t IndexHandler(httpd_req_t *req)
{
    if (!CheckSource(req)) {
        return ESP_OK;
    }
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, s_indexStart, s_indexEnd - s_indexStart - 1);
}

/* ------------------------------------------------------------ 启动 */

void WebConfig_Start(void)
{
    static const httpd_uri_t uris[] = {
        { .uri = "/",            .method = HTTP_GET,  .handler = IndexHandler },
        { .uri = "/api/login",   .method = HTTP_POST, .handler = LoginHandler },
        { .uri = "/api/status",  .method = HTTP_GET,  .handler = StatusHandler },
        { .uri = "/api/config",  .method = HTTP_GET,  .handler = ConfigGetHandler },
        { .uri = "/api/config",  .method = HTTP_POST, .handler = ConfigPostHandler },
        { .uri = "/api/scan",    .method = HTTP_GET,  .handler = ScanHandler },
        { .uri = "/api/reboot",  .method = HTTP_POST, .handler = RebootHandler },
        { .uri = "/api/factory", .method = HTTP_POST, .handler = FactoryHandler },
        { .uri = "/api/factory-config", .method = HTTP_POST, .handler = FactoryConfigPostHandler },
        { .uri = "/api/diag",          .method = HTTP_GET,  .handler = DiagHandler },
        { .uri = "/api/events",        .method = HTTP_GET,  .handler = EventsHandler },
        { .uri = "/api/events/clear",  .method = HTTP_POST, .handler = EventsClearHandler },
        { .uri = "/api/log",           .method = HTTP_GET,  .handler = LogHandler },
        { .uri = "/api/export",        .method = HTTP_GET,  .handler = ExportHandler },
        { .uri = "/api/crash",         .method = HTTP_GET,  .handler = CrashHandler },
        { .uri = "/api/crash/clear",   .method = HTTP_POST, .handler = CrashClearHandler },
        { .uri = "/api/coredump",      .method = HTTP_GET,  .handler = CoreDumpHandler },
        { .uri = "/api/monitor",       .method = HTTP_GET,  .handler = MonitorGetHandler },
        { .uri = "/api/monitor/start", .method = HTTP_POST, .handler = MonitorStartHandler },
        { .uri = "/api/monitor/stop",  .method = HTTP_POST, .handler = MonitorStopHandler },
        { .uri = "/api/devcfg",           .method = HTTP_GET,  .handler = DevCfgGetHandler },
        { .uri = "/api/devcfg",           .method = HTTP_POST, .handler = DevCfgPostHandler },
        { .uri = "/api/devcfg/file",      .method = HTTP_GET,  .handler = DevCfgFileHandler },
        { .uri = "/api/devcfg/clear",     .method = HTTP_POST, .handler = DevCfgClearHandler },
        { .uri = "/api/devcfg/protocols", .method = HTTP_GET,  .handler = ProtocolsHandler },
        { .uri = "/api/acq",              .method = HTTP_GET,  .handler = AcqHandler },
        { .uri = "/api/acq/try",          .method = HTTP_POST, .handler = AcqTryHandler },
    };
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    esp_err_t err;

    config.stack_size = 8192;     // 导出诊断报告时调用层次较深
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;     // 连接数满时关闭最久未用的连接（手机浏览器常保持多个空闲连接）
    config.max_uri_handlers = sizeof(uris) / sizeof(uris[0]);

    err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start http server failed: %s", esp_err_to_name(err));
        return;
    }
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
    ESP_LOGI(TAG, "config page on port %u%s", config.server_port,
             GwConfig_Get()->webApOnly ? " (hotspot only)" : "");
}
