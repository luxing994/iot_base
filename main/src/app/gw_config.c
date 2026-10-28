#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs.h"
#include "esp_netif.h"
#include "lwip/inet.h"
#include "sdkconfig.h"
#include "gw_config.h"

static const char *TAG = "GW_CONFIG";

#define GW_NVS_NAMESPACE    "gwcfg"

/* ------------------------------------------------------------ 出厂默认值 */

#if defined(CONFIG_IOT_HOST_IPV6)
#define DEF_HOST_ADDR       CONFIG_IOT_HOST_IPV6_ADDR
#else
#define DEF_HOST_ADDR       CONFIG_IOT_HOST_IPV4_ADDR
#endif

#ifdef CONFIG_IOT_FILE_PORT
#define DEF_FILE_PORT       CONFIG_IOT_FILE_PORT
#else
#define DEF_FILE_PORT       8767
#endif

#ifdef CONFIG_IOT_ETH_DHCP
#define DEF_ETH_DHCP        true
#define DEF_ETH_IP          "192.168.1.200"
#define DEF_ETH_MASK        "255.255.255.0"
#define DEF_ETH_GATEWAY     "192.168.1.1"
#define DEF_ETH_DNS         "192.168.1.1"
#elif defined(CONFIG_IOT_ETH_STATIC_IP)
#define DEF_ETH_DHCP        false
#define DEF_ETH_IP          CONFIG_IOT_ETH_STATIC_IP
#define DEF_ETH_MASK        CONFIG_IOT_ETH_STATIC_MASK
#define DEF_ETH_GATEWAY     CONFIG_IOT_ETH_STATIC_GATEWAY
#define DEF_ETH_DNS         CONFIG_IOT_ETH_STATIC_DNS
#else   /* 未启用以太网 */
#define DEF_ETH_DHCP        true
#define DEF_ETH_IP          ""
#define DEF_ETH_MASK        ""
#define DEF_ETH_GATEWAY     ""
#define DEF_ETH_DNS         ""
#endif

#ifdef CONFIG_IOT_ETH_MAC
#define DEF_ETH_MAC         CONFIG_IOT_ETH_MAC
#else
#define DEF_ETH_MAC         ""
#endif

#define DEF_AP_PASS         CONFIG_IOT_AP_PASSWORD
#define DEF_WEB_PASS        CONFIG_IOT_WEB_PASSWORD

/* 网口 PLC：未启用该连接方式时 menuconfig 中没有对应选项 */
#ifdef CONFIG_PLC_FX_IPV4_ADDR
#define DEF_FX_IP           CONFIG_PLC_FX_IPV4_ADDR
#define DEF_FX_PORT         CONFIG_PLC_FX_TCP_PORT
#else
#define DEF_FX_IP           ""
#define DEF_FX_PORT         0
#endif

#ifdef CONFIG_PLC_PPI_IPV4_ADDR
#define DEF_S7_IP           CONFIG_PLC_PPI_IPV4_ADDR
#define DEF_S7_PORT         CONFIG_PLC_PPI_TCP_PORT
#else
#define DEF_S7_IP           ""
#define DEF_S7_PORT         0
#endif

#ifdef CONFIG_PLC_MODBUS_IPV4_ADDR
#define DEF_MODBUS_IP       CONFIG_PLC_MODBUS_IPV4_ADDR
#define DEF_MODBUS_PORT     CONFIG_PLC_MODBUS_TCP_PORT
#else
#define DEF_MODBUS_IP       ""
#define DEF_MODBUS_PORT     0
#endif

static const GwConfig s_defaults = {
    .devId = CONFIG_IOT_DEVICE_ID,
    .devNumber = CONFIG_IOT_DEVICE_NUMBER,

    .hostAddr = DEF_HOST_ADDR,
    .hostReportPort = CONFIG_IOT_HOST_REPORT_PORT,
    .hostLinkPort = CONFIG_IOT_HOST_LINK_PORT,
    .hostOtaPort = CONFIG_IOT_HOST_OTA_PORT,

    .cmdPort = CONFIG_IOT_CMD_PORT,
    .advertisedPort = CONFIG_IOT_ADVERTISED_PORT,
    .filePort = DEF_FILE_PORT,

    .wifiSsid = CONFIG_IOT_WIFI_SSID,
    .wifiPass = CONFIG_IOT_WIFI_PASSWORD,
    .ethMac = DEF_ETH_MAC,
    .ethDhcp = DEF_ETH_DHCP,
    .ethIp = DEF_ETH_IP,
    .ethMask = DEF_ETH_MASK,
    .ethGateway = DEF_ETH_GATEWAY,
    .ethDns = DEF_ETH_DNS,

    .fxIp = DEF_FX_IP,
    .fxPort = DEF_FX_PORT,
    .s7Ip = DEF_S7_IP,
    .s7Port = DEF_S7_PORT,
    .modbusIp = DEF_MODBUS_IP,
    .modbusPort = DEF_MODBUS_PORT,

    .apPass = DEF_AP_PASS,
    .webPass = DEF_WEB_PASS,
};

/* ------------------------------------------------------------ NVS 字段表 */

typedef enum {
    FIELD_STR,
    FIELD_U16,
    FIELD_BOOL,
} FieldType;

typedef struct {
    const char *key;        // NVS 键名，最长 15 个字符
    uint8_t type;
    uint16_t offset;
    uint16_t size;
} GwField;

#define FIELD(k, t, m)  { k, t, offsetof(GwConfig, m), sizeof(((GwConfig *)0)->m) }

static const GwField s_fields[] = {
    FIELD("dev_id",      FIELD_STR,  devId),
    FIELD("dev_number",  FIELD_STR,  devNumber),
    FIELD("host_addr",   FIELD_STR,  hostAddr),
    FIELD("host_report", FIELD_U16,  hostReportPort),
    FIELD("host_link",   FIELD_U16,  hostLinkPort),
    FIELD("host_ota",    FIELD_U16,  hostOtaPort),
    FIELD("cmd_port",    FIELD_U16,  cmdPort),
    FIELD("adv_port",    FIELD_U16,  advertisedPort),
    FIELD("file_port",   FIELD_U16,  filePort),
    FIELD("wifi_ssid",   FIELD_STR,  wifiSsid),
    FIELD("wifi_pass",   FIELD_STR,  wifiPass),
    FIELD("eth_mac",     FIELD_STR,  ethMac),
    FIELD("eth_dhcp",    FIELD_BOOL, ethDhcp),
    FIELD("eth_ip",      FIELD_STR,  ethIp),
    FIELD("eth_mask",    FIELD_STR,  ethMask),
    FIELD("eth_gw",      FIELD_STR,  ethGateway),
    FIELD("eth_dns",     FIELD_STR,  ethDns),
    FIELD("fx_ip",       FIELD_STR,  fxIp),
    FIELD("fx_port",     FIELD_U16,  fxPort),
    FIELD("s7_ip",       FIELD_STR,  s7Ip),
    FIELD("s7_port",     FIELD_U16,  s7Port),
    FIELD("mb_ip",       FIELD_STR,  modbusIp),
    FIELD("mb_port",     FIELD_U16,  modbusPort),
    FIELD("ap_pass",     FIELD_STR,  apPass),
    FIELD("web_pass",    FIELD_STR,  webPass),
};

#define FIELD_COUNT (sizeof(s_fields) / sizeof(s_fields[0]))

static GwConfig s_config;
static bool s_loaded;
static bool s_saved;

/* ------------------------------------------------------------ 加载 */

// 读取一个字段；NVS 中没有或读取失败时保持默认值
static void LoadField(nvs_handle_t h, const GwField *f, GwConfig *cfg)
{
    uint8_t *p = (uint8_t *)cfg + f->offset;
    esp_err_t err = ESP_OK;

    switch (f->type) {
    case FIELD_STR: {
        char tmp[GW_STR_WIFIPW];        // 不小于最长的字符串字段
        size_t len = sizeof(tmp);

        err = nvs_get_str(h, f->key, tmp, &len);
        if (err == ESP_OK && len <= f->size) {
            memcpy(p, tmp, len);
        }
        break;
    }
    case FIELD_U16:
        err = nvs_get_u16(h, f->key, (uint16_t *)p);
        break;
    case FIELD_BOOL: {
        uint8_t v;
        err = nvs_get_u8(h, f->key, &v);
        if (err == ESP_OK) {
            *(bool *)p = (v != 0);
        }
        break;
    }
    default:
        break;
    }

    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "read %s failed: %s, use default", f->key, esp_err_to_name(err));
    }
}

void GwConfig_Init(void)
{
    nvs_handle_t h;
    esp_err_t err;

    s_config = s_defaults;
    s_saved = false;

    err = nvs_open(GW_NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        for (size_t i = 0; i < FIELD_COUNT; i++) {
            LoadField(h, &s_fields[i], &s_config);
        }
        nvs_close(h);
        s_saved = true;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "open nvs failed: %s, use defaults", esp_err_to_name(err));
    }
    s_loaded = true;

    ESP_LOGD(TAG, "%s config: devId %s, host %s:%u/%u/%u, wifi \"%s\", eth %s",
             s_saved ? "saved" : "factory", s_config.devId, s_config.hostAddr,
             s_config.hostReportPort, s_config.hostLinkPort, s_config.hostOtaPort,
             s_config.wifiSsid, s_config.ethDhcp ? "dhcp" : s_config.ethIp);
}

bool GwConfig_ParseMac(const char *str, uint8_t mac[6])
{
    unsigned int b[6];
    char sep[5];
    char tail;

    if (str == NULL || sscanf(str, "%2x%c%2x%c%2x%c%2x%c%2x%c%2x%c", &b[0], &sep[0], &b[1], &sep[1],
                              &b[2], &sep[2], &b[3], &sep[3], &b[4], &sep[4], &b[5], &tail) != 11) {
        return false;   // 恰好 11 项：6 个字节 + 5 个分隔符，后面不能再有字符
    }
    for (int i = 0; i < 5; i++) {
        if (sep[i] != ':' && sep[i] != '-') {
            return false;
        }
    }
    for (int i = 0; i < 6; i++) {
        mac[i] = (uint8_t)b[i];
    }
    return true;
}

void GwConfig_GetEthMac(uint8_t mac[6])
{
    if (GwConfig_ParseMac(GwConfig_Get()->ethMac, mac)) {
        return;
    }
    // 未配置：由芯片出厂 MAC 推算，每台设备不同
    esp_read_mac(mac, ESP_MAC_ETH);
}

void GwConfig_GetMacStr(GwMacType type, char *buf, size_t size)
{
    const char *ifkey = (type == GW_MAC_ETH) ? "ETH_DEF" : "WIFI_STA_DEF";
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey(ifkey);
    uint8_t mac[6] = { 0 };

    // 网口已创建时读取实际使用的 MAC（例如以太网初始化时设置的值）
    if (netif == NULL || esp_netif_get_mac(netif, mac) != ESP_OK) {
        if (type == GW_MAC_ETH) {
            GwConfig_GetEthMac(mac);
        } else {
            esp_read_mac(mac, ESP_MAC_WIFI_STA);
        }
    }
    snprintf(buf, size, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

const GwConfig *GwConfig_Get(void)
{
    if (!s_loaded) {
        // 正常流程在 app_main 中先调用 GwConfig_Init()，这里只是兜底
        return &s_defaults;
    }
    return &s_config;
}

bool GwConfig_IsSaved(void)
{
    return s_saved;
}

void GwConfig_GetDefaults(GwConfig *out)
{
    *out = s_defaults;
}

/* ------------------------------------------------------------ 校验 */

static bool IsIp4(const char *s)
{
    struct in_addr addr;
    return s[0] != '\0' && inet_aton(s, &addr) != 0;
}

static bool IsUnicastMac(const char *s)
{
    uint8_t mac[6];
    return GwConfig_ParseMac(s, mac) && (mac[0] & 0x01) == 0;
}

esp_err_t GwConfig_Validate(const GwConfig *cfg, const char **why)
{
    const char *reason = NULL;

    if (cfg->devId[0] == '\0') {
        reason = "设备号不能为空";
    } else if (cfg->hostAddr[0] == '\0') {
        reason = "上位机地址不能为空";
    } else if (cfg->hostReportPort == 0 || cfg->hostLinkPort == 0 || cfg->hostOtaPort == 0 ||
               cfg->cmdPort == 0) {
        reason = "端口不能为 0";
    } else if (cfg->wifiSsid[0] != '\0' && cfg->wifiPass[0] != '\0' && strlen(cfg->wifiPass) < 8) {
        reason = "Wi-Fi 密码至少 8 位（开放网络留空）";
    } else if (cfg->ethMac[0] != '\0' && !IsUnicastMac(cfg->ethMac)) {
        reason = "以太网 MAC 格式应为 AA:BB:CC:DD:EE:FF，且第一个字节须为偶数（单播地址）";
    } else if (!cfg->ethDhcp && (!IsIp4(cfg->ethIp) || !IsIp4(cfg->ethMask) || !IsIp4(cfg->ethGateway))) {
        reason = "以太网静态 IP、子网掩码、网关格式不正确";
    } else if (!cfg->ethDhcp && cfg->ethDns[0] != '\0' && !IsIp4(cfg->ethDns)) {
        reason = "DNS 地址格式不正确";
    } else if (strlen(cfg->apPass) < 8) {
        reason = "配置热点密码至少 8 位";
    } else if (cfg->webPass[0] == '\0') {
        reason = "网页登录密码不能为空";
    }
#ifdef CONFIG_PLC_FX_CONNECT_NETWORK
    else if (!IsIp4(cfg->fxIp) || cfg->fxPort == 0) {
        reason = "FX PLC 地址或端口不正确";
    }
#endif
#ifdef CONFIG_PLC_PPI_CONNECT_NETWORK
    else if (!IsIp4(cfg->s7Ip) || cfg->s7Port == 0) {
        reason = "S7 PLC 地址或端口不正确";
    }
#endif
#ifdef CONFIG_PLC_MODBUS_CONNECT_NETWORK
    else if (!IsIp4(cfg->modbusIp) || cfg->modbusPort == 0) {
        reason = "Modbus TCP 设备地址或端口不正确";
    }
#endif

    if (why != NULL) {
        *why = reason;
    }
    return (reason == NULL) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

/* ------------------------------------------------------------ 保存 / 恢复出厂 */

static esp_err_t SaveField(nvs_handle_t h, const GwField *f, const GwConfig *cfg)
{
    const uint8_t *p = (const uint8_t *)cfg + f->offset;

    switch (f->type) {
    case FIELD_STR: {
        char tmp[GW_STR_WIFIPW];

        // 保证以 0 结尾
        memcpy(tmp, p, f->size);
        tmp[f->size - 1] = '\0';
        return nvs_set_str(h, f->key, tmp);
    }
    case FIELD_U16:
        return nvs_set_u16(h, f->key, *(const uint16_t *)p);
    case FIELD_BOOL:
        return nvs_set_u8(h, f->key, *(const bool *)p ? 1 : 0);
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t GwConfig_Save(const GwConfig *cfg)
{
    const char *why = NULL;
    nvs_handle_t h;
    esp_err_t err;

    err = GwConfig_Validate(cfg, &why);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "invalid config: %s", why);
        return err;
    }

    err = nvs_open(GW_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "open nvs failed: %s", esp_err_to_name(err));
        return err;
    }
    for (size_t i = 0; i < FIELD_COUNT && err == ESP_OK; i++) {
        err = SaveField(h, &s_fields[i], cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "write %s failed: %s", s_fields[i].key, esp_err_to_name(err));
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "config saved, takes effect after restart");
    }
    return err;
}

esp_err_t GwConfig_Reset(void)
{
    nvs_handle_t h;
    esp_err_t err;

    err = nvs_open(GW_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_all(h);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    ESP_LOGW(TAG, "config reset to factory defaults: %s", esp_err_to_name(err));
    return err;
}
