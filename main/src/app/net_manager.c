#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "nvs.h"
#include "lwip/inet.h"
#include "sdkconfig.h"
#ifdef CONFIG_IOT_ETH_W5500
#include "esp_eth.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#endif
#include "gw_config.h"
#include "net_manager.h"

static const char *TAG = "NET";

#define TICK_MS                 1000
#define BOOT_RESET_COUNT        3           // 连按复位次数
#define BOOT_RESET_WINDOW_MS    5000        // 开机后运行满这么久才清零复位计数
#define AP_IDLE_CLOSE_MS        (10 * 60 * 1000)
#define WIFI_RETRY_MS           5000
#define WIFI_LOG_EVERY          12          // 连续重连失败时每 12 次（约 1 分钟）打印一次

#define BOOT_NVS_NAMESPACE      "gwboot"
#define BOOT_NVS_KEY            "resets"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static esp_netif_t *s_ethNetif;
static esp_netif_t *s_staNetif;
static esp_netif_t *s_apNetif;

static bool s_ethPresent;
static volatile bool s_ethLink;
static volatile bool s_ethUp;
static char s_ethIp[16];

static bool s_wifiInited;
static bool s_wifiStarted;
static volatile bool s_scanning;        // 网页正在扫描 Wi-Fi，暂停重连
static bool s_staWanted;                // 配置了 Wi-Fi SSID
static volatile bool s_staConnected;
static volatile bool s_staUp;
static char s_staIp[16];
static int s_staFails;
static TickType_t s_staRetryAt;         // 0：不需要重连

static volatile bool s_apOn;
static volatile int s_apClients;
static NetApReason s_apReason;
static int s_bootResets;            // NetMgr_CountBoot() 记下的连续复位次数
static char s_apSsid[33];
static TickType_t s_apIdleSince;

static TickType_t Now(void)
{
    return xTaskGetTickCount();
}

static bool Elapsed(TickType_t since, uint32_t ms)
{
    return (Now() - since) >= pdMS_TO_TICKS(ms);
}

const char *NetMgr_ApReasonText(NetApReason reason)
{
    switch (reason) {
    case NET_AP_FACTORY:    return "factory state (no saved config)";
    case NET_AP_RESET_X3:   return "reset button pressed 3 times";
    case NET_AP_MANUAL:     return "requested";
    default:                return "off";
    }
}

/* ================================================================ 复位计数 */

// 开机计数 +1 并返回新值；运行满 BOOT_RESET_WINDOW_MS 后清零，因此只有短时间内连续复位才会累加
static int BootCounterIncrement(void)
{
    nvs_handle_t h;
    uint8_t count = 0;

    if (nvs_open(BOOT_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return 0;
    }
    nvs_get_u8(h, BOOT_NVS_KEY, &count);
    count++;
    nvs_set_u8(h, BOOT_NVS_KEY, count);
    nvs_commit(h);
    nvs_close(h);
    return count;
}

static void BootCounterClear(void)
{
    nvs_handle_t h;

    if (nvs_open(BOOT_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, BOOT_NVS_KEY, 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

void NetMgr_CountBoot(void)
{
    s_bootResets = BootCounterIncrement();
}

/* ================================================================ Wi-Fi / 热点 */

static const char *WifiReasonText(uint8_t reason)
{
    switch (reason) {
    case WIFI_REASON_NO_AP_FOUND:            return "AP not found";
    case WIFI_REASON_AUTH_FAIL:              return "auth failed";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:      return "handshake timeout, wrong password?";
    case WIFI_REASON_BEACON_TIMEOUT:         return "signal lost";
    case WIFI_REASON_ASSOC_LEAVE:            return "left";
    default:                                 return "";
    }
}

static void WifiApplyConfig(void)
{
    static bool staConfigured = false;
    const GwConfig *cfg = GwConfig_Get();
    wifi_mode_t mode;
    esp_err_t err;

    if (s_staWanted && s_apOn) {
        mode = WIFI_MODE_APSTA;
    } else if (s_staWanted) {
        mode = WIFI_MODE_STA;
    } else if (s_apOn) {
        mode = WIFI_MODE_AP;
    } else {
        if (s_wifiInited) {
            esp_wifi_stop();
            s_wifiStarted = false;
        }
        return;
    }

    // 运行中切换模式（打开 / 关闭热点）时 Wi-Fi 可能正在连接，出错只记录，不能用 ESP_ERROR_CHECK 重启
    err = esp_wifi_set_mode(mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi: set mode %d failed: %s", mode, esp_err_to_name(err));
        return;
    }
    if (s_staWanted && !staConfigured) {
        wifi_config_t sta = { 0 };

        strlcpy((char *)sta.sta.ssid, cfg->wifiSsid, sizeof(sta.sta.ssid));
        strlcpy((char *)sta.sta.password, cfg->wifiPass, sizeof(sta.sta.password));
        sta.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        sta.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        sta.sta.threshold.authmode = WIFI_AUTH_OPEN;
        err = esp_wifi_set_config(WIFI_IF_STA, &sta);
        staConfigured = (err == ESP_OK);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "wifi: set station config failed: %s", esp_err_to_name(err));
        }
    }
    if (s_apOn) {
        wifi_config_t ap = { 0 };

        strlcpy((char *)ap.ap.ssid, s_apSsid, sizeof(ap.ap.ssid));
        ap.ap.ssid_len = strlen(s_apSsid);
        strlcpy((char *)ap.ap.password, cfg->apPass, sizeof(ap.ap.password));
        ap.ap.authmode = (strlen(cfg->apPass) >= 8) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        ap.ap.max_connection = 3;
        ap.ap.channel = 1;      // 同时连着路由器时会自动改为路由器的信道
        err = esp_wifi_set_config(WIFI_IF_AP, &ap);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "wifi: set hotspot config failed: %s", esp_err_to_name(err));
        }
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi: start failed: %s", esp_err_to_name(err));
        return;
    }
    s_wifiStarted = true;
    esp_wifi_set_ps(WIFI_PS_NONE);      // 网关常供电，关闭省电以降低通信延迟
}

static void OnWifiEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case WIFI_EVENT_STA_START:
        if (s_staWanted) {
            ESP_LOGI(TAG, "wifi: connecting to \"%s\"", GwConfig_Get()->wifiSsid);
            esp_wifi_connect();
        }
        break;
    case WIFI_EVENT_STA_CONNECTED:
        s_staConnected = true;
#ifdef CONFIG_IOT_HOST_IPV6
        esp_netif_create_ip6_linklocal(s_staNetif);
#endif
        break;
    case WIFI_EVENT_STA_DISCONNECTED: {
        const wifi_event_sta_disconnected_t *e = data;

        if (s_staUp || s_staFails % WIFI_LOG_EVERY == 0) {
            ESP_LOGW(TAG, "wifi: disconnected (reason %d %s), retry every %d s",
                     e->reason, WifiReasonText(e->reason), WIFI_RETRY_MS / 1000);
        }
        s_staFails++;
        s_staConnected = false;
        s_staUp = false;
        s_staRetryAt = Now() + pdMS_TO_TICKS(WIFI_RETRY_MS);   // 由管理任务重连，避免在事件回调里连续重试
        break;
    }
    case WIFI_EVENT_AP_START:
        ESP_LOGW(TAG, "config hotspot ON: \"%s\", http://%s  (%s)", s_apSsid, NET_AP_IP,
                 NetMgr_ApReasonText(s_apReason));
        break;
    case WIFI_EVENT_AP_STOP:
        ESP_LOGI(TAG, "config hotspot OFF");
        break;
    case WIFI_EVENT_AP_STACONNECTED:
        s_apClients++;
        ESP_LOGI(TAG, "config hotspot: client joined (%d connected)", s_apClients);
        break;
    case WIFI_EVENT_AP_STADISCONNECTED:
        if (s_apClients > 0) {
            s_apClients--;
        }
        ESP_LOGI(TAG, "config hotspot: client left (%d connected)", s_apClients);
        break;
    default:
        break;
    }
}

static void WifiInit(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (s_wifiInited) {
        return;
    }
    s_staNetif = esp_netif_create_default_wifi_sta();
    s_apNetif = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));    // 账号保存在本项目的配置中
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, OnWifiEvent, NULL));
    s_wifiInited = true;
}

void NetMgr_OpenConfigAp(NetApReason reason)
{
    if (s_apOn) {
        return;
    }
    s_apReason = reason;
    s_apOn = true;
    s_apClients = 0;
    s_apIdleSince = Now();
    WifiInit();
    WifiApplyConfig();
}

static void CloseConfigAp(void)
{
    s_apOn = false;
    s_apReason = NET_AP_OFF;
    s_apClients = 0;
    WifiApplyConfig();
}

/* ================================================================ 以太网 */

#ifdef CONFIG_IOT_ETH_W5500

static void EthSetStaticIp(const GwConfig *cfg)
{
    esp_netif_ip_info_t ip = { 0 };
    esp_netif_dns_info_t dns = { 0 };

    esp_netif_dhcpc_stop(s_ethNetif);
    ip.ip.addr = ipaddr_addr(cfg->ethIp);
    ip.netmask.addr = ipaddr_addr(cfg->ethMask);
    ip.gw.addr = ipaddr_addr(cfg->ethGateway);
    if (esp_netif_set_ip_info(s_ethNetif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "eth: set static IP failed");
        return;
    }
    if (cfg->ethDns[0] != '\0') {
        dns.ip.u_addr.ip4.addr = ipaddr_addr(cfg->ethDns);
        dns.ip.type = ESP_IPADDR_TYPE_V4;
        esp_netif_set_dns_info(s_ethNetif, ESP_NETIF_DNS_MAIN, &dns);
    }
    strlcpy(s_ethIp, cfg->ethIp, sizeof(s_ethIp));
    ESP_LOGI(TAG, "eth: static IP %s, mask %s, gw %s", cfg->ethIp, cfg->ethMask, cfg->ethGateway);
}

static void OnEthEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case ETHERNET_EVENT_CONNECTED:
        s_ethLink = true;
        ESP_LOGI(TAG, "eth: link up");
#ifdef CONFIG_IOT_HOST_IPV6
        esp_netif_create_ip6_linklocal(s_ethNetif);
#endif
        if (!GwConfig_Get()->ethDhcp) {
            // 静态地址：在链路连通后停止 DHCP 并设置地址（与 ESP-IDF static_ip 示例的做法一致）
            EthSetStaticIp(GwConfig_Get());
            s_ethUp = true;
        }
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        s_ethLink = false;
        s_ethUp = false;
        ESP_LOGW(TAG, "eth: link down (cable unplugged?)");
        break;
    default:
        break;
    }
}

// 初始化 W5500；模块不存在或初始化失败时返回 false，网关继续使用 Wi-Fi
static bool EthStart(void)
{
    esp_netif_config_t netifCfg = ESP_NETIF_DEFAULT_ETH();
    spi_bus_config_t bus = {
        .miso_io_num = CONFIG_IOT_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_IOT_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_IOT_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t dev = {
        .command_bits = 16,     // W5500 帧的地址段
        .address_bits = 8,      // W5500 帧的控制段
        .mode = 0,
        .clock_speed_hz = CONFIG_IOT_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = CONFIG_IOT_ETH_SPI_CS_GPIO,
        .queue_size = 20,
    };
    spi_device_handle_t spi = NULL;
    esp_eth_handle_t eth = NULL;
    uint8_t mac[6];
    esp_err_t err;

    s_ethNetif = esp_netif_new(&netifCfg);

    // W5500 中断依赖的 GPIO ISR 服务已在 app_main 中安装
    err = spi_bus_initialize(CONFIG_IOT_ETH_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err == ESP_OK) {
        err = spi_bus_add_device(CONFIG_IOT_ETH_SPI_HOST, &dev, &spi);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "eth: spi init failed: %s", esp_err_to_name(err));
        return false;
    }

    eth_w5500_config_t w5500 = ETH_W5500_DEFAULT_CONFIG(spi);
    eth_mac_config_t macCfg = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phyCfg = ETH_PHY_DEFAULT_CONFIG();

    w5500.int_gpio_num = CONFIG_IOT_ETH_SPI_INT_GPIO;
    phyCfg.reset_gpio_num = CONFIG_IOT_ETH_RST_GPIO;
    esp_eth_mac_t *ethMac = esp_eth_mac_new_w5500(&w5500, &macCfg);
    esp_eth_phy_t *ethPhy = esp_eth_phy_new_w5500(&phyCfg);
    esp_eth_config_t ethCfg = ETH_DEFAULT_CONFIG(ethMac, ethPhy);

    err = esp_eth_driver_install(&ethCfg, &eth);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "eth: W5500 not responding (%s), ethernet disabled", esp_err_to_name(err));
        return false;
    }

    // W5500 没有出厂 MAC：使用配置值，未配置时按芯片 MAC 推算
    GwConfig_GetEthMac(mac);
    esp_eth_ioctl(eth, ETH_CMD_S_MAC_ADDR, mac);

    esp_netif_attach(s_ethNetif, esp_eth_new_netif_glue(eth));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, OnEthEvent, NULL));
    esp_eth_start(eth);
    ESP_LOGI(TAG, "eth: W5500 ready, waiting for cable");     // 区分"没插网线"和"模块没初始化"
    return true;
}

#endif /* CONFIG_IOT_ETH_W5500 */

/* ================================================================ IP 事件 */

static void OnIpEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case IP_EVENT_ETH_GOT_IP: {
        const ip_event_got_ip_t *e = data;

        taskENTER_CRITICAL(&s_lock);
        snprintf(s_ethIp, sizeof(s_ethIp), IPSTR, IP2STR(&e->ip_info.ip));
        taskEXIT_CRITICAL(&s_lock);
        s_ethUp = true;
        ESP_LOGI(TAG, "eth: got IP " IPSTR ", mask " IPSTR ", gw " IPSTR,
                 IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.netmask), IP2STR(&e->ip_info.gw));
        break;
    }
    case IP_EVENT_ETH_LOST_IP:
        s_ethUp = false;
        ESP_LOGW(TAG, "eth: lost IP");
        break;
    case IP_EVENT_STA_GOT_IP: {
        const ip_event_got_ip_t *e = data;

        taskENTER_CRITICAL(&s_lock);
        snprintf(s_staIp, sizeof(s_staIp), IPSTR, IP2STR(&e->ip_info.ip));
        taskEXIT_CRITICAL(&s_lock);
        s_staUp = true;
        s_staFails = 0;
        ESP_LOGI(TAG, "wifi: got IP " IPSTR ", mask " IPSTR ", gw " IPSTR,
                 IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.netmask), IP2STR(&e->ip_info.gw));
        break;
    }
    case IP_EVENT_STA_LOST_IP:
        s_staUp = false;
        break;
    default:
        break;
    }
}

/* ================================================================ 管理任务 */

static void NetMgrTask(void *arg)
{
    bool bootCleared = false;
    bool retryPausedLogged = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));

        // 正常运行满 5 秒，说明不是"连按复位"，清零计数
        if (!bootCleared && Elapsed(0, BOOT_RESET_WINDOW_MS)) {
            BootCounterClear();
            bootCleared = true;
        }

        // Wi-Fi 重连；有终端连着配置热点时暂停（重连扫描会切换信道，导致手机掉线）
        if (s_staWanted && !s_staConnected && !s_scanning && s_staRetryAt != 0 &&
            (int32_t)(Now() - s_staRetryAt) >= 0) {
            if (s_apClients > 0) {
                if (!retryPausedLogged) {
                    ESP_LOGI(TAG, "wifi: reconnect paused while a client is on the config hotspot");
                    retryPausedLogged = true;
                }
            } else {
                retryPausedLogged = false;
                s_staRetryAt = 0;
                esp_wifi_connect();
            }
        }

        // 热点 10 分钟没有终端连接时关闭
        if (s_apOn) {
            if (s_apClients > 0) {
                s_apIdleSince = Now();
            } else if (Elapsed(s_apIdleSince, AP_IDLE_CLOSE_MS)) {
                ESP_LOGI(TAG, "config hotspot: idle for %d minutes", AP_IDLE_CLOSE_MS / 60000);
                CloseConfigAp();
            }
        }
    }
}

/* ================================================================ 接口 */

void NetMgr_Start(void)
{
    const GwConfig *cfg = GwConfig_Get();
    uint8_t mac[6];

    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_apSsid, sizeof(s_apSsid), "IoTGW-%02X%02X", mac[4], mac[5]);

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, OnIpEvent, NULL));

#ifdef CONFIG_IOT_ETH_W5500
    if (cfg->ethEnabled) {
        s_ethPresent = EthStart();
    } else {
        ESP_LOGI(TAG, "eth: disabled in config");
    }
#endif

    s_staWanted = cfg->wifiEnabled && cfg->wifiSsid[0] != '\0';
    if (s_staWanted) {
        WifiInit();
    }

    // 是否进入配置模式
    if (s_bootResets >= BOOT_RESET_COUNT) {
        BootCounterClear();
        s_apReason = NET_AP_RESET_X3;
    } else if (!GwConfig_IsSaved()) {
        s_apReason = NET_AP_FACTORY;
    }
    if (s_apReason != NET_AP_OFF) {
        NetMgr_OpenConfigAp(s_apReason);
    } else if (s_staWanted) {
        WifiApplyConfig();
    }

    xTaskCreate(NetMgrTask, "net_mgr", 3072, NULL, 4, NULL);
}

bool NetMgr_IsConnected(void)
{
    return s_ethUp || s_staUp;
}

void NetMgr_GetStatus(NetStatus *out)
{
    wifi_ap_record_t ap;

    memset(out, 0, sizeof(*out));
    taskENTER_CRITICAL(&s_lock);
    out->ethPresent = s_ethPresent;
    out->ethLink = s_ethLink;
    out->ethUp = s_ethUp;
    strlcpy(out->ethIp, s_ethIp, sizeof(out->ethIp));
    out->wifiEnabled = s_staWanted;
    out->wifiUp = s_staUp;
    strlcpy(out->wifiIp, s_staIp, sizeof(out->wifiIp));
    out->apOn = s_apOn;
    out->apClients = s_apClients;
    out->apReason = s_apReason;
    strlcpy(out->apSsid, s_apSsid, sizeof(out->apSsid));
    taskEXIT_CRITICAL(&s_lock);

    if (out->wifiUp && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        out->wifiRssi = ap.rssi;
    }
}

void NetMgr_GetLocalIp(const char *peer, char *buf, size_t size)
{
    esp_netif_t *candidates[2] = { s_ethUp ? s_ethNetif : NULL, s_staUp ? s_staNetif : NULL };
    esp_netif_ip_info_t info;
    uint32_t peerAddr = ipaddr_addr(peer);

    buf[0] = '\0';
    // 优先与上位机同网段的网口
    for (int pass = 0; pass < 2 && buf[0] == '\0'; pass++) {
        for (int i = 0; i < 2; i++) {
            if (candidates[i] == NULL || esp_netif_get_ip_info(candidates[i], &info) != ESP_OK ||
                info.ip.addr == 0) {
                continue;
            }
            if (pass == 1 || ((info.ip.addr ^ peerAddr) & info.netmask.addr) == 0) {
                snprintf(buf, size, IPSTR, IP2STR(&info.ip));
                break;
            }
        }
    }
}

int NetMgr_ScanWifi(NetWifiAp *out, int max)
{
    wifi_scan_config_t scan = { .show_hidden = false };
    wifi_ap_record_t *recs;
    wifi_mode_t mode = WIFI_MODE_NULL;
    uint16_t n = (uint16_t)max;
    esp_err_t err;

    // 扫描需要 STA 接口：只开着热点时临时切到 AP+STA，Wi-Fi 未启动时以 STA 启动（不会去连接）
    WifiInit();
    esp_wifi_get_mode(&mode);
    if (!s_wifiStarted) {
        esp_wifi_set_mode(WIFI_MODE_STA);
        if (esp_wifi_start() == ESP_OK) {
            s_wifiStarted = true;
        }
    } else if (mode == WIFI_MODE_AP) {
        esp_wifi_set_mode(WIFI_MODE_APSTA);
    }

    s_scanning = true;
    err = esp_wifi_scan_start(&scan, true);
    if (err == ESP_ERR_WIFI_STATE) {
        // STA 正在连接路由器时不能扫描，停止本次连接后重试（扫描完由管理任务按计划重连）
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
        err = esp_wifi_scan_start(&scan, true);
    }
    s_scanning = false;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "wifi: scan failed: %s", esp_err_to_name(err));
        return -1;
    }

    recs = calloc(n, sizeof(*recs));
    if (recs == NULL) {
        return -1;
    }
    if (esp_wifi_scan_get_ap_records(&n, recs) != ESP_OK) {
        n = 0;
    }
    for (int i = 0; i < n; i++) {
        strlcpy(out[i].ssid, (const char *)recs[i].ssid, sizeof(out[i].ssid));
        out[i].rssi = recs[i].rssi;
        out[i].secure = (recs[i].authmode != WIFI_AUTH_OPEN);
    }
    free(recs);
    if (s_staWanted && !s_staConnected && s_staRetryAt == 0) {
        s_staRetryAt = Now();       // 扫描打断了正在进行的连接，尽快重连
    }
    return n;
}

int NetMgr_GetIfIndex(void)
{
    esp_netif_t *netif = s_ethUp ? s_ethNetif : s_staNetif;
    return (netif != NULL) ? esp_netif_get_netif_impl_index(netif) : 0;
}
