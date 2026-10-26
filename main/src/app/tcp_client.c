/*
 * 与上位机 / 网口 PLC 的 TCP 客户端任务
 *
 *   tcp_client_task        上位机 PORT     : 发送 xQueue1 中的上报数据
 *   tcp_client1_task       上位机 PORT1    : 上线包 + 1s 心跳，接收时间戳
 *   tcp_client3_task       上位机 OTA_PORT : 上线包 + 1s 心跳，接收 OTA_UPDATE / TIMESTAMP 命令
 *   tcp_xxx_client_task    网口 PLC        : 周期轮询寄存器，链路失效自动重连
 */
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "iot_common.h"
#include "protocol_examples_common.h"
#include "iot_time.h"
#include "tcp_client.h"
#include "fx_plc_protocol.h"
#include "lx_modbus_tcp_master.h"
#include "lx_s7comm.h"
#include "ota_task.h"

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT     CONFIG_EXAMPLE_PORT     // 数据上报
#define PORT1    CONFIG_EXAMPLE_PORT1    // 上线 / 心跳
#define OTA_PORT 8768                    // OTA 控制通道

#define CONNECT_PORT_STR        "8766"  // 上线包中告知上位机的本机命令端口
#define CLIENT_RECONNECT_MS     5000
#define HEART_BEAT_MS           1000
#define PLC_POLL_INTERVAL_MS    1000
#define JSON_BUF_SIZE           1024

char mcu_ip[32] = {0};
int ota_tcp_sock = -1;

char* GetStaIp(void)
{
    return mcu_ip;
}

/* ---------------------------------------------------------------- 通用工具 */

static bool is_our_netif(const char *prefix, esp_netif_t *netif)
{
    return strncmp(prefix, esp_netif_get_desc(netif), strlen(prefix) - 1) == 0;
}

// 把 "a.b.c.d" 拆成整数数组（会修改 ip）
static void GetIpArry(char *ip, int *data, uint16_t len)
{
    char *token = strtok(ip, ".");
    for (int i = 0; i < len && token != NULL; i++) {
        data[i] = atoi(token);
        token = strtok(NULL, ".");
    }
}

// 在多个网卡中找出与上位机同网段的那个，记录其 IP 作为本机上报 IP
static void UpdateMcuIp(const char *hostip)
{
    const char *TAG = "example_connect";
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = NULL;
    int addr[4] = {0};
    char ptr[32] = {0};

    strlcpy(ptr, hostip, sizeof(ptr));
    GetIpArry(ptr, addr, sizeof(addr) / sizeof(int));
    for (int i = 0; i < esp_netif_get_nr_of_ifs(); ++i) {
        netif = esp_netif_next(netif);
        if (is_our_netif(TAG, netif)) {
            esp_netif_get_ip_info(netif, &ip_info);
            if ((esp_ip4_addr1(&ip_info.ip) == addr[0]) && (esp_ip4_addr2(&ip_info.ip) == addr[1]) &&
                (esp_ip4_addr3(&ip_info.ip) == addr[2])) {
                snprintf(mcu_ip, sizeof(mcu_ip), IPSTR, IP2STR(&ip_info.ip));
                break;
            }
        }
    }
}

// 上线包（orderId=initDev, isAnswer=yes）和心跳包（orderId=heartBeat, isAnswer=no）格式相同
static void PackLinkData(char *buf, size_t size, const char *orderId, const char *isAnswer)
{
    (void)snprintf(buf, size, "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"%s\",\n    \"devName\":\"%s\",\n"
            "    \"devStatus\":\"\",\n    \"devTypeId\":\"%s\",\n    \"orderName\":\"%s\",\n"
            "    \"orderId\":\"%s\",\n    \"devIP\":\"%s\",\n    \"connectPort\":\"%s\",\n"
            "    \"value\":\"NULL\",\n    \"timeStamp\":\"%lld\",\n    \"isAnswer\":\"%s\",\n"
            "    \"expand\":\"NULL\"};;**##",
            DEVID, "Hello", DEVNAME, DEVTYPEID, ORDERNAME, orderId, mcu_ip, CONNECT_PORT_STR,
            GetMilliTimeNow(), isAnswer);
}

// 发送全部数据（send 可能只发出一部分）
static int SendAll(int sock, const char *data, size_t len)
{
    while (len > 0) {
        int n = send(sock, data, len, 0);
        if (n < 0) {
            return -1;
        }
        data += n;
        len -= n;
    }
    return 0;
}

// 建立 TCP 连接，成功返回 socket，失败返回 -1
static int TcpConnect(const char *ip, uint16_t port, const char *tag)
{
    int addr_family;
    int ip_protocol;
    int sock;

#if defined(CONFIG_EXAMPLE_IPV4)
    struct sockaddr_in dest_addr = { 0 };
    dest_addr.sin_addr.s_addr = inet_addr(ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
#else
    struct sockaddr_in6 dest_addr = { 0 };
    inet6_aton(ip, &dest_addr.sin6_addr);
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(port);
    dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
#endif

    sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(tag, "Unable to create socket: errno %d", errno);
        return -1;
    }
    ESP_LOGI(tag, "Socket created, connecting to %s:%d", ip, port);

    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(tag, "Socket unable to connect: errno %d", errno);
        close(sock);
        return -1;
    }
    ESP_LOGI(tag, "Successfully connected");

    return sock;
}

static void TcpClose(int sock, const char *tag)
{
    if (sock >= 0) {
        ESP_LOGE(tag, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
}

/* ---------------------------------------------------------- 上位机数据上报 */

void tcp_client_task(void *pvParameters)
{
    const char *TAG = "tcp client";
    char *msg = NULL;

    while (1) {
        int sock = TcpConnect(HOST_IP_ADDR, PORT, TAG);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
            continue;
        }

        while (1) {
            if (xQueueReceive(xQueue1, &msg, pdMS_TO_TICKS(1000)) != pdPASS) {
                continue;
            }
            ESP_LOGI(TAG, "Read data %s\n", msg);
            int err = SendAll(sock, msg, strlen(msg));
            free(msg);   // 队列元素由 IOT_PostMessage 分配，发送后在这里释放
            msg = NULL;
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
        }

        TcpClose(sock, TAG);
        vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
    }
}

/* ------------------------------------------------------- 上线 / 心跳通道 */

typedef void (*LinkRecvHandler)(char *data, int len);

// 端口 PORT1 收到的是纯时间戳
static void HandleTimestampData(char *data, int len)
{
    g_baseTime = atoll(data);
    ESP_LOGI("TIMESTAMP_CMD", "time:%lld  timenow:%lld\n", g_baseTime, GetMilliTimeNow());
}

static void HandleTcpCommand(const char *cmd_line)
{
    const char *prefixota = "OTA_UPDATE:";
    const char *prefixtime = "TIMESTAMP:";

    while (*cmd_line == ' ' || *cmd_line == '\t') {
        ++cmd_line;
    }

    if (strncmp(cmd_line, prefixota, strlen(prefixota)) == 0) {
        const char *url = cmd_line + strlen(prefixota);
        ESP_LOGI("OTA_CMD", "OTA UPDATE URL=%s", url);
        char *u = strdup(url);
        if (u != NULL && xTaskCreate(ota_task, "ota_task", 8192, u, 5, NULL) != pdPASS) {
            free(u);
        }
    } else if (strncmp(cmd_line, prefixtime, strlen(prefixtime)) == 0) {
        g_baseTime = atoll(cmd_line + strlen(prefixtime));
        ESP_LOGI("TIMESTAMP_CMD", "time:%lld  timenow:%lld\n", g_baseTime, GetMilliTimeNow());
    } else {
        ESP_LOGI("OTA_CMD", "UNKNOW COMMAND:%s", cmd_line);
    }
}

// OTA 通道按行解析命令
static void HandleOtaCommandData(char *data, int len)
{
    char *saveptr = NULL;
    char *line = strtok_r(data, "\r\n", &saveptr);

    while (line != NULL) {
        HandleTcpCommand(line);
        line = strtok_r(NULL, "\r\n", &saveptr);
    }
}

// 连接上位机 -> 发上线包 -> 每秒发心跳并非阻塞接收；断线后重连
// sockout 非空时把当前 socket 暴露给外部（OTA 任务用它回传进度）
static void LinkClientLoop(uint16_t port, const char *tag, LinkRecvHandler handler, int *sockout)
{
    char *txbuf = malloc(JSON_BUF_SIZE);   // 每个任务独立缓冲区（原实现两个任务共用 initdata）
    char *rxbuf = malloc(JSON_BUF_SIZE);

    if (txbuf == NULL || rxbuf == NULL) {
        ESP_LOGE(tag, "no memory");
        free(txbuf);
        free(rxbuf);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        int sock = TcpConnect(HOST_IP_ADDR, port, tag);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
            continue;
        }
        if (sockout != NULL) {
            *sockout = sock;
        }

        UpdateMcuIp(HOST_IP_ADDR);
        PackLinkData(txbuf, JSON_BUF_SIZE, "initDev", "yes");
        if (SendAll(sock, txbuf, strlen(txbuf)) < 0) {
            ESP_LOGE(tag, "Error occurred during sending: errno %d", errno);
        }

        TickType_t lastWake = xTaskGetTickCount();
        while (1) {
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(HEART_BEAT_MS));
            PackLinkData(txbuf, JSON_BUF_SIZE, "heartBeat", "no");
            if (SendAll(sock, txbuf, strlen(txbuf)) < 0) {
                ESP_LOGE(tag, "Error occurred during sending: errno %d", errno);
                break;
            }

            int len = recv(sock, rxbuf, JSON_BUF_SIZE - 1, MSG_DONTWAIT);
            if (len > 0) {
                rxbuf[len] = '\0';
                handler(rxbuf, len);
            } else if (len == 0) {
                ESP_LOGW(tag, "Connection closed by peer");
                break;
            }
        }

        if (sockout != NULL) {
            *sockout = -1;
        }
        TcpClose(sock, tag);
        vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
    }
}

void tcp_client1_task(void *pvParameters)
{
    LinkClientLoop(PORT1, "tcp client1", HandleTimestampData, NULL);
}

void tcp_client3_task(void *pvParameters)
{
    ESP_LOGI("tcp client3", "Firmware version: V3");
    LinkClientLoop(OTA_PORT, "tcp client3", HandleOtaCommandData, &ota_tcp_sock);
}

/* ---------------------------------------------------------- 网口 PLC 轮询 */

typedef int (*PlcConnectFunc)(void);
typedef int (*PlcPollFunc)(int sock);

// 读寄存器返回 <0 表示链路失效，立即结束本轮并重连
#define NET_READ(call)  do { if ((call) < 0) { return -1; } } while (0)

static void PlcNetClientLoop(const char *tag, PlcConnectFunc connect_fn, PlcPollFunc poll_fn)
{
    while (1) {
        int sock = connect_fn();
        if (sock < 0) {
            ESP_LOGE(tag, "connect to plc failed");
            vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
            continue;
        }

        TickType_t lastWake = xTaskGetTickCount();
        while (1) {
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PLC_POLL_INTERVAL_MS));
            if (poll_fn(sock) < 0) {
                ESP_LOGE(tag, "plc link lost");
                break;
            }
        }

        TcpClose(sock, tag);
        vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
    }
}

#if defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
static int ModbusConnect(void)
{
    return TcpConnect(CONFIG_PLC_MODBUS_IPV4_ADDR, CONFIG_PLC_MODBUS_TCP_PORT, "tcp modbus client");
}

static int ModbusPoll(int sock)
{
    // 电解炉控制柜
    NET_READ(MODBUS_NetReadFloatRegister(sock, 5, 23));       // 11#工作室真空度
    NET_READ(MODBUS_NetReadFloatRegister(sock, 7, 24));       // 11#收集室真空度
    NET_READ(MODBUS_NetReadFloatRegister(sock, 13, 25));      // 11#工作室压力值
    NET_READ(MODBUS_NetReadFloatRegister(sock, 15, 26));      // 11#收集室压力值
    NET_READ(MODBUS_NetReadFloatRegister(sock, 19, 27));      // 11#炉体水温

    NET_READ(MODBUS_NetReadFloatRegister(sock, 1, 28));       // 12#工作室真空度
    NET_READ(MODBUS_NetReadFloatRegister(sock, 3, 29));       // 12#收集室真空度
    NET_READ(MODBUS_NetReadFloatRegister(sock, 9, 30));       // 12#工作室压力值
    NET_READ(MODBUS_NetReadFloatRegister(sock, 11, 31));      // 12#收集室压力值
    NET_READ(MODBUS_NetReadFloatRegister(sock, 17, 32));      // 12#炉体水温

    // 无纸记录仪
    // NET_READ(MODBUS_NetReadFloatRegister(sock, 0x2000, 1));       // 1温区温度
    // NET_READ(MODBUS_NetReadFloatRegister(sock, 0x2002, 2));       // 2温区温度
    // NET_READ(MODBUS_NetReadFloatRegister(sock, 0x2004, 3));       // 3温区温度
    // NET_READ(MODBUS_NetReadFloatRegister(sock, 0x2006, 4));       // 4温区温度
    // NET_READ(MODBUS_NetReadFloatRegister(sock, 0x2008, 5));       // 5温区温度
    // NET_READ(MODBUS_NetReadFloatRegister(sock, 0x200A, 6));       // 6温区温度
    return 0;
}

void tcp_modbus_client_task(void *pvParameters)
{
    PlcNetClientLoop("tcp modbus client", ModbusConnect, ModbusPoll);
}
#endif

#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
static int FxConnect(void)
{
    return TcpConnect(CONFIG_PLC_FX_IPV4_ADDR, CONFIG_PLC_FX_TCP_PORT, "tcp fx client");
}

static int FxPoll(int sock)
{
    NET_READ(FX_NetReadSingleYDataRegister(sock, 94, 1));        // 正常运行状态
    NET_READ(FX_NetReadSingleYDataRegister(sock, 93, 2));        // 暂停状态
    NET_READ(FX_NetReadSingleYDataRegister(sock, 92, 3));        // 异常停止状态
    NET_READ(FX_NetReadSingleDataRegister(sock, 214, 4));        // 白班生产时间
    NET_READ(FX_NetReadSingleDataRegister(sock, 215, 5));        // 晚班生产时间
    NET_READ(FX_NetReadDoubleDataRegister(sock, 30, 6));         // 总累计冲次
    NET_READ(FX_NetReadSingleDataRegister(sock, 10, 7));         // 实时冲速
    NET_READ(FX_NetReadSingleDataRegister(sock, 101, 8));        // 当前累积生产数量
    return 0;
}

void tcp_fx_client_task(void *pvParameters)
{
    PlcNetClientLoop("tcp fx client", FxConnect, FxPoll);
}
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
static int S7Connect(void)
{
    uint16_t pdu_len = 0;
    return s7_connect(CONFIG_PLC_PPI_IPV4_ADDR, CONFIG_PLC_PPI_TCP_PORT, &pdu_len);
}

static int S7Poll(int sock)
{
#ifdef CONFIG_PLC_PPI_ELECTROLYSIS_DEVICE
    // 电解炉电解柜
    NET_READ(S7_NetReadBitQRegister(sock, 0, 2, 1));         // 运行显示
    NET_READ(S7_NetReadBitMRegister(sock, 2, 0, 2));         // 启动
    NET_READ(S7_NetReadBitMRegister(sock, 2, 3, 3));         // 故障复位
    NET_READ(S7_NetReadBitDataRegister(sock, 3315, 1, 4));   // 电源输入故障
    NET_READ(S7_NetReadBitDataRegister(sock, 3315, 2, 5));   // 设备超温
    NET_READ(S7_NetReadBitDataRegister(sock, 3315, 3, 6));   // 输出过电压
    NET_READ(S7_NetReadBitDataRegister(sock, 3315, 4, 7));   // 输出过电流
    NET_READ(S7_NetReadBitDataRegister(sock, 3315, 5, 8));   // IGBT保护
    NET_READ(S7_NetReadBitDataRegister(sock, 3315, 6, 9));   // 输出低电压
    NET_READ(S7_NetReadBitDataRegister(sock, 3315, 7, 10));  // 风扇变频故障
    NET_READ(S7_NetReadBitDataRegister(sock, 3316, 1, 11));  // 故障显示
    NET_READ(S7_NetReadBitMRegister(sock, 0, 7, 12));        // 急停
    NET_READ(S7_NetReadFloatDataRegister(sock, 210, 13));    // 实时电压
    NET_READ(S7_NetReadFloatDataRegister(sock, 214, 14));    // 实时电流
    NET_READ(S7_NetReadFloatDataRegister(sock, 280, 15));    // 设备温度
    NET_READ(S7_NetReadFloatDataRegister(sock, 534, 16));    // 电解AH
    NET_READ(S7_NetReadFloatDataRegister(sock, 1648, 17));   // 电压给定
    NET_READ(S7_NetReadFloatDataRegister(sock, 1848, 18));   // 电流给定
    NET_READ(S7_NetReadIntDataRegister(sock, 900, 19));      // 批号
    NET_READ(S7_NetReadSingleDataRegister(sock, 520, 20));   // 电解时间H
    NET_READ(S7_NetReadSingleDataRegister(sock, 522, 21));   // 电解时间M
#endif

#ifdef CONFIG_PLC_PPI_CONTROLLER_DEVICE
    // 电解炉控制柜PLC
    NET_READ(S7_NetReadBitQRegister(sock, 8, 3, 1));         // 前置阀1开到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 8, 4, 2));         // 前置阀1关到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 8, 5, 3));         // 前置阀2开到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 8, 6, 4));         // 前置阀2关到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 0, 7, 5));         // 旁通阀输出
    NET_READ(S7_NetReadBitQRegister(sock, 8, 7, 6));         // 粗抽阀1（左）开到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 9, 0, 7));         // 粗抽阀1（左）关到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 9, 1, 8));         // 粗抽阀2（左）开到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 9, 2, 9));         // 粗抽阀2（左）关到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 9, 3, 10));        // 粗抽阀1（右）开到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 9, 4, 11));        // 粗抽阀1（右）关到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 9, 5, 12));        // 粗抽阀2（右）开到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 9, 6, 13));        // 粗抽阀2（右）关到位指示
    NET_READ(S7_NetReadBitQRegister(sock, 0, 0, 14));        // H-150滑阀泵输出
    NET_READ(S7_NetReadBitQRegister(sock, 0, 1, 15));        // ZJP-1200罗茨泵输出
    NET_READ(S7_NetReadBitIRegister(sock, 9, 0, 17));        // 插板阀开启到位(左)
    NET_READ(S7_NetReadBitIRegister(sock, 9, 1, 18));        // 插板阀关闭到位(左)
    NET_READ(S7_NetReadBitIRegister(sock, 9, 2, 19));        // 插板阀开启到位(右)
    NET_READ(S7_NetReadBitIRegister(sock, 9, 3, 20));        // 插板阀关闭到位(右)
    NET_READ(S7_NetReadFloatDataRegister(sock, 100, 22));    // 冷却水压力
#endif
    return 0;
}

void tcp_ppi_client_task(void *pvParameters)
{
    PlcNetClientLoop("tcp ppi client", S7Connect, S7Poll);
}
#endif
