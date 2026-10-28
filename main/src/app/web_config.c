/*
 * 配置网页：静态页面 + JSON 接口，接口说明见 web_config.h
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "cJSON.h"
#include "sdkconfig.h"
#include "app_info.h"
#include "gw_config.h"
#include "net_manager.h"
#include "tcp_client.h"
#include "web_config.h"

static const char *TAG = "WEB";

#define BODY_MAX            2048
#define SESSION_IDLE_MS     (30 * 60 * 1000)
#define LOGIN_MAX_FAILS     5
#define LOGIN_LOCK_MS       60000
#define SCAN_MAX            20
#define RESTART_DELAY_MS    1500        // 先把应答发出去再重启

/* main/web/index.html，CMakeLists.txt 中以 EMBED_TXTFILES 嵌入（末尾带 '\0'） */
extern const char s_indexStart[] asm("_binary_index_html_start");
extern const char s_indexEnd[] asm("_binary_index_html_end");

/* 会话：同一时间只有一个有效令牌，所有请求都在 httpd 任务中处理，无需加锁 */
static char s_token[33];
static int64_t s_tokenUsedMs;
static int s_loginFails;
static int64_t s_lockUntilMs;

static int64_t NowMs(void)
{
    return esp_timer_get_time() / 1000;
}

/* ------------------------------------------------------------ 应答工具 */

// 发送并释放 root
static esp_err_t SendJson(httpd_req_t *req, cJSON *root)
{
    char *text = (root != NULL) ? cJSON_PrintUnformatted(root) : NULL;
    esp_err_t err;

    cJSON_Delete(root);
    if (text == NULL) {
        return httpd_resp_send_500(req);
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
        httpd_resp_send_500(req);
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
    pass = cJSON_GetObjectItemCaseSensitive(json, "password");
    if (!cJSON_IsString(pass) || strcmp(pass->valuestring, GwConfig_Get()->webPass) != 0) {
        cJSON_Delete(json);
        ESP_LOGW(TAG, "login failed");
        if (++s_loginFails >= LOGIN_MAX_FAILS) {
            s_loginFails = 0;
            s_lockUntilMs = NowMs() + LOGIN_LOCK_MS;
        }
        return SendError(req, "401 Unauthorized", "密码错误");
    }
    cJSON_Delete(json);

    s_loginFails = 0;
    esp_fill_random(rnd, sizeof(rnd));
    for (int i = 0; i < 16; i++) {
        snprintf(&s_token[i * 2], 3, "%02x", rnd[i]);
    }
    s_tokenUsedMs = NowMs();
    ESP_LOGI(TAG, "login ok");

    root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "token", s_token);
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
    cJSON_AddStringToObject(root, "reset", App_ResetText());
    cJSON_AddStringToObject(root, "collect", App_CollectText());
    cJSON_AddBoolToObject(root, "saved", GwConfig_IsSaved());
    cJSON_AddStringToObject(root, "devId", cfg->devId);
    cJSON_AddStringToObject(root, "devNumber", cfg->devNumber);
    cJSON_AddStringToObject(root, "hostAddr", cfg->hostAddr);
    cJSON_AddBoolToObject(root, "viaAp", FromConfigAp(req));     // 当前是否经配置热点访问

    obj = cJSON_AddObjectToObject(root, "eth");
    GwConfig_GetMacStr(GW_MAC_ETH, mac, sizeof(mac));
    cJSON_AddBoolToObject(obj, "present", st.ethPresent);
    cJSON_AddBoolToObject(obj, "link", st.ethLink);
    cJSON_AddBoolToObject(obj, "up", st.ethUp);
    cJSON_AddStringToObject(obj, "ip", st.ethUp ? st.ethIp : "");
    cJSON_AddStringToObject(obj, "mac", mac);
    cJSON_AddBoolToObject(obj, "dhcp", cfg->ethDhcp);

    obj = cJSON_AddObjectToObject(root, "wifi");
    GwConfig_GetMacStr(GW_MAC_WIFI, mac, sizeof(mac));
    cJSON_AddBoolToObject(obj, "enabled", st.wifiEnabled);
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
    cJSON_AddItemToObject(root, "config", GwConfig_ToJson(GwConfig_Get()));

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

static esp_err_t ConfigPostHandler(httpd_req_t *req)
{
    GwConfig *cfg;
    const char *why = NULL;
    cJSON *json;
    esp_err_t err;

    if (!CheckAuth(req)) {
        return ESP_OK;
    }
    json = ReadJson(req);
    if (json == NULL) {
        return ESP_OK;
    }
    cfg = malloc(sizeof(*cfg));     // 结构体约 600 字节，不放 httpd 任务栈上
    if (cfg == NULL) {
        cJSON_Delete(json);
        return httpd_resp_send_500(req);
    }
    *cfg = *GwConfig_Get();
    err = GwConfig_FromJson(cfg, json, &why);
    cJSON_Delete(json);
    if (err == ESP_OK) {
        err = GwConfig_Validate(cfg, &why);
    }
    if (err == ESP_OK) {
        err = GwConfig_Save(cfg);
        if (err != ESP_OK) {
            why = "写入存储失败";
        }
    }
    free(cfg);
    if (err != ESP_OK) {
        return SendError(req, "400 Bad Request", why != NULL ? why : "保存失败");
    }
    SendOk(req);
    ScheduleRestart("config saved from web");
    return ESP_OK;
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
        return httpd_resp_send_500(req);
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
    SendOk(req);
    ScheduleRestart("factory reset from web");
    return ESP_OK;
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
    };
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    esp_err_t err;

    config.stack_size = 6144;
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
