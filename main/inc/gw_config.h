#ifndef GW_CONFIG_H
#define GW_CONFIG_H

/*
 * 网关运行参数（现场安装时会改的参数，保存在 NVS 中）
 *
 *   启动时 GwConfig_Init() 从 NVS 读取一次；NVS 中没有的项使用 menuconfig 中的值（出厂默认值）。
 *   运行期间配置只读，GwConfig_Get() 可在任意任务中调用，无需加锁。
 *   修改配置：GwConfig_Save() 写入 NVS，重启后生效。
 *
 * 设备类型、协议、串口参数等仍在 menuconfig 中选择（编译期决定）。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "cJSON.h"

#define GW_STR_ID       32
#define GW_STR_HOST     48      // IPv4 / IPv6 地址
#define GW_STR_IP4      16
#define GW_STR_SSID     33      // Wi-Fi SSID 最长 32 字节
#define GW_STR_WIFIPW   65      // Wi-Fi 密码最长 64 字节
#define GW_STR_PASS     33
#define GW_STR_MAC      18      // "AA:BB:CC:DD:EE:FF"

typedef struct {
    /* 设备标识 */
    char devId[GW_STR_ID];              // 上报报文 devId，每台网关唯一
    char devNumber[GW_STR_ID];          // 上报报文 devNumber

    /* 上位机 */
    char hostAddr[GW_STR_HOST];
    uint16_t hostReportPort;            // 数据上报
    uint16_t hostLinkPort;              // 上线包 + 心跳
    uint16_t hostOtaPort;               // OTA 控制通道

    /* 本机服务 */
    uint16_t cmdPort;                   // 接收上位机 JSON 命令
    uint16_t advertisedPort;            // 上线包中的 connectPort
    uint16_t filePort;                  // 文件分包（menuconfig 中开启文件端口时使用）

    /* 网络 */
    char wifiSsid[GW_STR_SSID];         // 为空表示不使用 Wi-Fi
    char wifiPass[GW_STR_WIFIPW];
    char ethMac[GW_STR_MAC];            // 以太网 MAC（出厂参数，只来自 menuconfig，不存 NVS），空表示按芯片出厂 MAC 自动生成
    bool ethDhcp;                       // false：使用下面的静态地址
    char ethIp[GW_STR_IP4];
    char ethMask[GW_STR_IP4];
    char ethGateway[GW_STR_IP4];
    char ethDns[GW_STR_IP4];

    /* 网口 PLC */
    char fxIp[GW_STR_IP4];
    uint16_t fxPort;
    char s7Ip[GW_STR_IP4];
    uint16_t s7Port;
    char modbusIp[GW_STR_IP4];
    uint16_t modbusPort;

    /* 配置模式 */
    char apPass[GW_STR_PASS];           // 配置热点密码（至少 8 位，出厂参数）
    char webPass[GW_STR_PASS];          // 配置网页登录密码
    bool webApOnly;                     // 配置网页只允许通过配置热点访问
} GwConfig;

/* 从 NVS 加载配置，须在 nvs_flash_init() 之后、创建任务之前调用 */
void GwConfig_Init(void);

/* 当前生效的配置（只读） */
const GwConfig *GwConfig_Get(void);

/*
 * 参数分两区：
 *   出厂参数（devId、devNumber、上位机端口、本机命令 / 文件端口、以太网 MAC、配置热点密码）：出货时用出厂密码在
 *     配置网页设置，客户不能修改，恢复出厂不清除
 *   客户参数（其余）：现场安装时设置，恢复出厂时清除
 */
typedef enum {
    GW_AREA_USER = 1,
    GW_AREA_FACTORY = 2,
    GW_AREA_ALL = 3,
} GwArea;

/* 客户参数是否保存过（false 表示出厂状态：开机打开配置热点） */
bool GwConfig_IsSaved(void);

/* 出厂参数是否在网页中设置过（false 表示使用 menuconfig 中的值） */
bool GwConfig_IsFactorySaved(void);

/*
 * 本机 MAC 地址，格式 "AA:BB:CC:DD:EE:FF"（buf 至少 GW_STR_MAC 字节）
 * 网口已创建时返回网口当前实际使用的 MAC，否则返回将要使用的 MAC
 */
typedef enum {
    GW_MAC_WIFI,
    GW_MAC_ETH,
} GwMacType;

void GwConfig_GetMacStr(GwMacType type, char *buf, size_t size);

/*
 * 以太网应使用的 MAC：配置了 ethMac 时用配置值，否则由芯片出厂 MAC 推算（每台设备不同）。
 * W5500 没有出厂 MAC，初始化以太网时用这个值设置。
 */
void GwConfig_GetEthMac(uint8_t mac[6]);

/* 解析 "AA:BB:CC:DD:EE:FF"（也接受 '-' 分隔），成功返回 true */
bool GwConfig_ParseMac(const char *str, uint8_t mac[6]);

/* 出厂默认值（menuconfig 中的值） */
void GwConfig_GetDefaults(GwConfig *out);

/* 检查配置是否合法，非法时返回 ESP_ERR_INVALID_ARG，并在 why 中给出原因 */
esp_err_t GwConfig_Validate(const GwConfig *cfg, const char **why);

/* 校验后把 area 区的字段保存到 NVS，重启后生效 */
esp_err_t GwConfig_Save(const GwConfig *cfg, GwArea area);

/* 恢复出厂：清除客户参数（出厂参数保留），重启后生效 */
esp_err_t GwConfig_Reset(void);

/*
 * 与 JSON 互转（配置网页用），键名与结构体成员同名，如 "devId"、"hostReportPort"。
 *   ToJson   输出 areas 区的字段，不输出密码类字段（wifiPass、apPass、webPass）
 *   FromJson 只覆盖 JSON 中出现的字段；出现 areas 以外的字段、类型或长度不对时返回
 *            ESP_ERR_INVALID_ARG 并给出原因
 */
cJSON *GwConfig_ToJson(const GwConfig *cfg, uint8_t areas);
esp_err_t GwConfig_FromJson(GwConfig *cfg, const cJSON *json, uint8_t areas, const char **why);

#endif
