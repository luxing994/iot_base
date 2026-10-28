/*
 * 与上位机 / 网口 PLC 的 TCP 客户端任务
 *
 *   tcp_client_task        上位机 PORT     : 发送 xQueue1 中的上报数据
 *   tcp_client1_task       上位机 PORT1    : 上线包 + 1s 心跳，接收时间戳
 *   tcp_client3_task       上位机 OTA_PORT : 上线包 + 1s 心跳，接收 OTA_UPDATE / TIMESTAMP 命令
 *   tcp_xxx_client_task    网口 PLC        : 周期轮询寄存器，链路失效自动重连
 */
#include <stdio.h>
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
#include "net_io.h"
#include "lx_s7comm.h"
#include "ota_task.h"
#include "plc_poll.h"
#include "host_cmd.h"

#if defined(CONFIG_IOT_HOST_IPV4)
#define HOST_IP_ADDR CONFIG_IOT_HOST_IPV4_ADDR
#elif defined(CONFIG_IOT_HOST_IPV6)
#define HOST_IP_ADDR CONFIG_IOT_HOST_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT     CONFIG_IOT_HOST_REPORT_PORT     // 数据上报
#define PORT1    CONFIG_IOT_HOST_LINK_PORT    // 上线 / 心跳
#define OTA_PORT CONFIG_IOT_HOST_OTA_PORT     // OTA 控制通道

#define CONNECT_PORT            CONFIG_IOT_ADVERTISED_PORT   // 上线包 connectPort 字段
#define CLIENT_RECONNECT_MS     CONFIG_IOT_RECONNECT_INTERVAL_MS
#define HEART_BEAT_MS           CONFIG_IOT_HEARTBEAT_INTERVAL_MS
#define PLC_POLL_INTERVAL_MS    1000
#define JSON_BUF_SIZE           1024
#define HOST_SEND_TIMEOUT_MS    5000    // 与上位机的连接发送超时

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

// 在多个网卡中找出与上位机同网段的那个，记录其 IP 作为本机上报 IP
// （两个连接任务会同时调用，原实现用 strtok 拆 IP，不可重入）
static void UpdateMcuIp(const char *hostip)
{
    const char *TAG = "example_connect";
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = NULL;
    int addr[4] = {0};

    if (sscanf(hostip, "%d.%d.%d.%d", &addr[0], &addr[1], &addr[2], &addr[3]) != 4) {
        return;     // 非 IPv4 地址
    }
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
            "    \"orderId\":\"%s\",\n    \"devIP\":\"%s\",\n    \"connectPort\":\"%d\",\n"
            "    \"value\":\"NULL\",\n    \"timeStamp\":\"%lld\",\n    \"isAnswer\":\"%s\",\n"
            "    \"expand\":\"NULL\"};;**##",
            DEVID, "Hello", DEVNAME, DEVTYPEID, ORDERNAME, orderId, mcu_ip, CONNECT_PORT,
            (long long)GetMilliTimeNow(), isAnswer);
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

#if defined(CONFIG_IOT_HOST_IPV4)
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

    // 上位机不再读取数据时 send 会一直阻塞，设置超时后按断线处理并重连
    Net_SetTimeout(sock, HOST_SEND_TIMEOUT_MS);
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
            if (xQueueReceive(xQueue1, &msg, portMAX_DELAY) != pdPASS) {
                continue;
            }
            ESP_LOGD(TAG, "send %s", msg);     // 每条上报都打 INFO 日志会占满串口输出
            if (SendAll(sock, msg, strlen(msg)) < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                // 放回队首，重连后重发（原实现直接丢弃这一条）
                if (xQueueSendToFront(xQueue1, &msg, 0) != pdPASS) {
                    free(msg);
                }
                msg = NULL;
                break;
            }
            free(msg);   // 队列元素由上报模块分配，发送后在这里释放
            msg = NULL;
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
    CalBaseTime((uint64_t)atoll(data));
    ESP_LOGI("TIMESTAMP_CMD", "timenow:%lld", (long long)GetMilliTimeNow());
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
        if (!OTA_TryBegin()) {
            ESP_LOGW("OTA_CMD", "OTA already running, ignore");
            return;
        }
        char *u = strdup(url);
        if (u == NULL || xTaskCreate(ota_task, "ota_task", 8192, u, 5, NULL) != pdPASS) {
            free(u);
            OTA_Abort();
        }
    } else if (strncmp(cmd_line, prefixtime, strlen(prefixtime)) == 0) {
        CalBaseTime((uint64_t)atoll(cmd_line + strlen(prefixtime)));
        ESP_LOGI("TIMESTAMP_CMD", "timenow:%lld", (long long)GetMilliTimeNow());
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

#define PLC_NET_TIMEOUT_MS      3000    // 网口 PLC 收发超时，超时视为链路失效并重连

typedef int (*PlcConnectFunc)(void);
typedef int (*PlcPollFunc)(int sock);

// 网口 PLC 地址都是 IPv4（与上位机使用 IPv4 还是 IPv6 无关）
static __attribute__((unused)) int PlcTcpConnect(const char *ip, uint16_t port, const char *tag)
{
    struct sockaddr_in dest_addr = { 0 };
    int sock;

    dest_addr.sin_addr.s_addr = inet_addr(ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(tag, "Unable to create socket: errno %d", errno);
        return -1;
    }
    ESP_LOGI(tag, "connecting to plc %s:%d", ip, port);
    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(tag, "Socket unable to connect: errno %d", errno);
        close(sock);
        return -1;
    }
    Net_SetTimeout(sock, PLC_NET_TIMEOUT_MS);
    return sock;
}

static __attribute__((unused)) void PlcNetClientLoop(const char *tag, PlcConnectFunc connect_fn,
                                                     PlcPollFunc poll_fn)
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
            if (!HostCmd_IsCollectEnabled()) {
                continue;   // 与串口轮询一致：上位机 EndFlush 后暂停采集（原实现网口 PLC 不受控制）
            }
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
    return PlcTcpConnect(CONFIG_PLC_MODBUS_IPV4_ADDR, CONFIG_PLC_MODBUS_TCP_PORT, "tcp modbus client");
}

void tcp_modbus_client_task(void *pvParameters)
{
    PlcNetClientLoop("tcp modbus client", ModbusConnect, PlcPoll_ModbusNet);
}
#endif

#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
static int FxConnect(void)
{
    return PlcTcpConnect(CONFIG_PLC_FX_IPV4_ADDR, CONFIG_PLC_FX_TCP_PORT, "tcp fx client");
}

void tcp_fx_client_task(void *pvParameters)
{
    PlcNetClientLoop("tcp fx client", FxConnect, PlcPoll_FxNet);
}
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
static int S7Connect(void)
{
    int sock = PlcTcpConnect(CONFIG_PLC_PPI_IPV4_ADDR, CONFIG_PLC_PPI_TCP_PORT, "tcp ppi client");

    if (sock >= 0 && S7_Handshake(sock) != 0) {
        close(sock);
        return -1;
    }
    return sock;
}

void tcp_ppi_client_task(void *pvParameters)
{
    PlcNetClientLoop("tcp ppi client", S7Connect, PlcPoll_S7Net);
}
#endif
