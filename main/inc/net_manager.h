#ifndef NET_MANAGER_H
#define NET_MANAGER_H

/*
 * 联网管理：以太网（W5500）+ Wi-Fi + 配置热点
 *
 *   - 以太网和 Wi-Fi 按运行参数（gw_config.h）启动，任一路拿到 IP 即可通信，
 *     断开后自动重连；开机不会因为某一路连不上而卡住
 *   - 配置热点 IoTGW-xxxx（xxxx 为 MAC 后 4 位）只在以下情况打开：
 *       出厂状态（从未保存过配置）、5 秒内连按 3 次复位键（或快速断电上电 3 次）
 *     10 分钟内没有终端连接则自动关闭；热点打开期间采集照常进行
 *   - 网络断开不会进入配置模式（现场网络临时中断很常见），只在后台持续重连
 *
 * NetMgr_CountBoot() 在 nvs_flash_init() 之后立即调用；
 * NetMgr_Start() 不阻塞，须在 esp_netif_init()、esp_event_loop_create_default()、
 * GwConfig_Init() 之后调用。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NET_AP_IP   "192.168.4.1"

typedef enum {
    NET_AP_OFF = 0,
    NET_AP_FACTORY,         // 出厂状态
    NET_AP_RESET_X3,        // 连按 3 次复位
    NET_AP_MANUAL,          // 网页 / 命令要求打开
} NetApReason;

typedef struct {
    bool ethPresent;        // 以太网模块已初始化
    bool ethLink;           // 网线已连接
    bool ethUp;             // 以太网已有 IP
    char ethIp[16];
    bool wifiEnabled;       // 配置了 Wi-Fi
    bool wifiUp;            // Wi-Fi 已有 IP
    char wifiIp[16];
    int8_t wifiRssi;
    bool apOn;
    int apClients;
    char apSsid[33];
    NetApReason apReason;
} NetStatus;

/* 开机复位计数 +1，须在 nvs_flash_init() 之后尽早调用（越早越不容易漏计快速连按的复位） */
void NetMgr_CountBoot(void);

void NetMgr_Start(void);

/* 任一网口已有 IP */
bool NetMgr_IsConnected(void);

void NetMgr_GetStatus(NetStatus *out);

/* 与 peer 同网段的本机 IP（用于上报 devIP），没有同网段时返回任一已连接网口的 IP，都没有时为空串 */
void NetMgr_GetLocalIp(const char *peer, char *buf, size_t size);

/* 打开配置热点 */
void NetMgr_OpenConfigAp(NetApReason reason);

/* 扫描附近的 Wi-Fi（阻塞约 2~3 秒），返回个数，失败返回 -1 */
typedef struct {
    char ssid[33];
    int8_t rssi;
    bool secure;
} NetWifiAp;

int NetMgr_ScanWifi(NetWifiAp *out, int max);

/* IPv6 链路本地地址需要的网口编号（优先以太网） */
int NetMgr_GetIfIndex(void);

/* 热点名称和热点打开原因的说明文字（用于日志和网页） */
const char *NetMgr_ApReasonText(NetApReason reason);

#endif
