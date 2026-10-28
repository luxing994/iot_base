/*
 * 与上位机 / 网口 PLC 的 TCP 客户端任务
 *
 *   tcp_client_task        上位机上报端口 : 发送 xQueue1 中的上报数据
 *   tcp_client1_task       上位机心跳端口 : 上线包 + 心跳，接收时间戳
 *   tcp_client3_task       上位机 OTA 端口: 上线包 + 心跳，接收 OTA_UPDATE / TIMESTAMP 命令
 *   tcp_xxx_client_task    网口 PLC       : 周期轮询寄存器，链路失效自动重连
 *
 * 网络未就绪时各任务等待，不发起连接；连接失败的日志限频打印，避免上位机离线时刷屏。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "iot_common.h"
#include "iot_time.h"
#include "tcp_client.h"
#include "net_io.h"
#include "net_manager.h"
#include "lx_s7comm.h"
#include "ota_task.h"
#include "plc_poll.h"
#include "host_cmd.h"
#include "event_log.h"
#include "monitor.h"
#include "esp_timer.h"

/* 上位机地址和端口为运行参数（gw_config.h），可在配置网页修改 */
#define HOST_IP_ADDR    (GwConfig_Get()->hostAddr)
#define PORT            (GwConfig_Get()->hostReportPort)    // 数据上报
#define PORT1           (GwConfig_Get()->hostLinkPort)      // 上线 / 心跳
#define OTA_PORT        (GwConfig_Get()->hostOtaPort)       // OTA 控制通道
// 上线包 / 心跳包的 connectPort 字段：上位机目前不使用，沿用原始代码的固定值
#define CONNECT_PORT    8766

#define CLIENT_RECONNECT_MS     CONFIG_IOT_RECONNECT_INTERVAL_MS
#define HEART_BEAT_MS           CONFIG_IOT_HEARTBEAT_INTERVAL_MS
#define PLC_POLL_INTERVAL_MS    1000
#define JSON_BUF_SIZE           1024
#define HOST_SEND_TIMEOUT_MS    5000    // 与上位机的连接发送超时
#define PLC_NET_TIMEOUT_MS      3000    // 网口 PLC 收发超时，超时视为链路失效并重连
#define CONNECT_LOG_EVERY       12      // 连续连接失败时每 12 次打印一次（重连间隔 5s 时约 1 分钟）

char mcu_ip[32] = {0};
int ota_tcp_sock = -1;

char* GetStaIp(void)
{
    return mcu_ip;
}

/* ---------------------------------------------------------------- 连接工具 */

// 每个连接一份，用于日志限频和状态查询
typedef struct {
    const char *tag;
    const char *peer;       // 日志中的对端名称，如 "report channel"
    int fails;              // 连续失败次数（连上后清零）
    volatile bool up;
    volatile bool off;      // 端口为 0，不使用这条通道
    int lastErr;            // 最近一次连接失败的 errno
    uint32_t connects;      // 本次启动累计连上次数
    uint32_t drops;         // 连上后又断开的次数
    uint32_t sinceS;        // 最近一次状态变化的开机秒数
    char addr[56];          // 最近一次连接的 "ip:port"
} ConnLog;

#define MAX_LINKS   8

/* 静态存放（任务可能结束，不能放在任务栈上） */
static ConnLog s_links[MAX_LINKS];
static ConnLog s_linkSpare;         // 槽位用完时使用，不参与状态查询（正常不会发生）
static int s_linkCount;
static portMUX_TYPE s_linkLock = portMUX_INITIALIZER_UNLOCKED;

static ConnLog *NewLink(const char *tag, const char *peer)
{
    ConnLog *log = &s_linkSpare;

    taskENTER_CRITICAL(&s_linkLock);
    if (s_linkCount < MAX_LINKS) {
        log = &s_links[s_linkCount++];
    }
    log->tag = tag;
    log->peer = peer;
    taskEXIT_CRITICAL(&s_linkLock);
    return log;
}

// 上位机端口为 0 表示不使用这条通道：app_main 不创建该任务，只在这里登记状态
void TcpClient_RegisterOff(const char *peer)
{
    ConnLog *log = NewLink("HOST", peer);

    log->off = true;
    ESP_LOGI(log->tag, "%s disabled (port 0)", peer);
}

int TcpClient_GetLinks(TcpLinkState *out, int max)
{
    int n = 0;

    taskENTER_CRITICAL(&s_linkLock);
    for (int i = 0; i < s_linkCount && n < max; i++, n++) {
        out[n].tag = s_links[i].tag;
        out[n].peer = s_links[i].peer;
        out[n].up = s_links[i].up;
        out[n].off = s_links[i].off;
        out[n].lastErr = s_links[i].lastErr;
        out[n].fails = s_links[i].fails;
        out[n].connects = s_links[i].connects;
        out[n].drops = s_links[i].drops;
        out[n].sinceS = s_links[i].sinceS;
        strlcpy(out[n].addr, s_links[i].addr, sizeof(out[n].addr));
    }
    taskEXIT_CRITICAL(&s_linkLock);
    return n;
}

static void WaitNetwork(void)
{
    while (!NetMgr_IsConnected()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static uint32_t UptimeS(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

const char *TcpClient_LinkName(const char *peer)
{
    static const char *const names[][2] = {
        { "report channel", "上位机上报通道" },
        { "link channel",   "上位机心跳通道" },
        { "ota channel",    "上位机 OTA 通道" },
        { "modbus tcp",     "Modbus TCP 设备" },
        { "fx mc",          "FX PLC（网口）" },
        { "s7",             "S7 PLC（网口）" },
    };

    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        if (strcmp(peer, names[i][0]) == 0) {
            return names[i][1];
        }
    }
    return peer;
}

const char *TcpClient_ErrText(int err)
{
    switch (err) {
    case ECONNREFUSED:
    case ECONNRESET:    return "对方拒绝连接（端口未开放）";
    case ECONNABORTED:
    case ETIMEDOUT:
    case EAGAIN:        return "对方无回应";
    case EHOSTUNREACH:
    case ENETUNREACH:   return "网络不通";
    case 0:             return "";
    default:            return "连接失败";
    }
}

static void LogConnectFail(ConnLog *log, const char *ip, uint16_t port, int err)
{
    snprintf(log->addr, sizeof(log->addr), "%s:%u", ip, port);
    log->lastErr = err;
    if (log->fails == 0) {
        // 只记第一次失败（连上之后再失败才会再记）
        log->sinceS = UptimeS();
        EvLog_Add(EV_WARN, "%s连接失败（%s）：%s", TcpClient_LinkName(log->peer), log->addr, TcpClient_ErrText(err));
    }
    if (log->fails % CONNECT_LOG_EVERY == 0) {
        ESP_LOGW(log->tag, "%s %s:%u unreachable (errno %d), retry every %d s",
                 log->peer, ip, port, err, CLIENT_RECONNECT_MS / 1000);
    }
    log->fails++;
}

static void LogConnected(ConnLog *log, const char *ip, uint16_t port)
{
    ESP_LOGI(log->tag, "%s %s:%u connected", log->peer, ip, port);
    snprintf(log->addr, sizeof(log->addr), "%s:%u", ip, port);
    log->fails = 0;
    log->lastErr = 0;
    log->connects++;
    log->sinceS = UptimeS();
    log->up = true;
    EvLog_Add(EV_INFO, "%s已连接（%s）", TcpClient_LinkName(log->peer), log->addr);
}

static void TcpClose(int sock, ConnLog *log)
{
    if (log->up) {
        log->drops++;
        log->sinceS = UptimeS();
        EvLog_Add(EV_WARN, "%s已断开", TcpClient_LinkName(log->peer));
    }
    log->up = false;
    if (sock >= 0) {
        ESP_LOGW(log->tag, "%s disconnected, reconnect in %d s", log->peer, CLIENT_RECONNECT_MS / 1000);
        shutdown(sock, 0);
        close(sock);
    }
}

// 记录与上位机同网段的本机 IP，作为上报报文中的 devIP
static void UpdateMcuIp(const char *hostip)
{
    char ip[sizeof(mcu_ip)];

    NetMgr_GetLocalIp(hostip, ip, sizeof(ip));
    if (ip[0] != '\0') {
        strlcpy(mcu_ip, ip, sizeof(mcu_ip));
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
            DEVID, DevCfg_DevNumber(), DEVNAME, DEVTYPEID, ORDERNAME, orderId, mcu_ip, CONNECT_PORT,
            (long long)GetMilliTimeNow(), isAnswer);
}

// 发送全部数据（send 可能只发出一部分）
static int SendAll(int sock, const char *data, size_t len)
{
    Mon_CaptureSock(sock, MON_TX, data, len);
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

// 连接上位机，成功返回 socket，失败返回 -1
static int TcpConnect(const char *ip, uint16_t port, ConnLog *log)
{
    int addr_family;
    int ip_protocol;
    int sock;

#if defined(CONFIG_IOT_HOST_IPV6)
    struct sockaddr_in6 dest_addr = { 0 };
    inet6_aton(ip, &dest_addr.sin6_addr);
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(port);
    dest_addr.sin6_scope_id = NetMgr_GetIfIndex();
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
#else
    struct sockaddr_in dest_addr = { 0 };
    dest_addr.sin_addr.s_addr = inet_addr(ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
#endif

    sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(log->tag, "unable to create socket: errno %d", errno);
        return -1;
    }
    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        LogConnectFail(log, ip, port, errno);
        close(sock);
        return -1;
    }
    LogConnected(log, ip, port);
    Mon_BindSocket(sock, strcmp(log->peer, "report channel") == 0 ? MON_CH_REPORT :
                         strcmp(log->peer, "ota channel") == 0 ? MON_CH_OTA : MON_CH_LINK);

    // 上位机不再读取数据时 send 会一直阻塞，设置超时后按断线处理并重连
    Net_SetTimeout(sock, HOST_SEND_TIMEOUT_MS);
    return sock;
}

/* ---------------------------------------------------------- 上位机数据上报 */

void tcp_client_task(void *pvParameters)
{
    ConnLog *log = NewLink("HOST", "report channel");
    char *msg = NULL;

    while (1) {
        WaitNetwork();
        int sock = TcpConnect(HOST_IP_ADDR, PORT, log);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
            continue;
        }
        UpdateMcuIp(HOST_IP_ADDR);     // 心跳通道停用时也要有 devIP

        while (1) {
            if (xQueueReceive(xQueue1, &msg, portMAX_DELAY) != pdPASS) {
                continue;
            }
            ESP_LOGD(log->tag, "send %s", msg);     // 每条上报都打 INFO 日志会占满串口输出
            if (SendAll(sock, msg, strlen(msg)) < 0) {
                ESP_LOGW(log->tag, "report send failed: errno %d", errno);
                // 放回队首，重连后重发
                if (xQueueSendToFront(xQueue1, &msg, 0) != pdPASS) {
                    free(msg);
                }
                msg = NULL;
                break;
            }
            free(msg);   // 队列元素由上报模块分配，发送后在这里释放
            msg = NULL;
        }

        TcpClose(sock, log);
        vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
    }
}

/* ------------------------------------------------------- 上线 / 心跳通道 */

typedef void (*LinkRecvHandler)(char *data, int len);

// 心跳端口收到的是纯时间戳
static void HandleTimestampData(char *data, int len)
{
    CalBaseTime((uint64_t)atoll(data));
    ESP_LOGD("HOST", "time sync: %lld", (long long)GetMilliTimeNow());
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
        ESP_LOGI("OTA", "update requested: %s", url);
        if (!OTA_TryBegin()) {
            ESP_LOGW("OTA", "already running, ignore");
            return;
        }
        char *u = strdup(url);
        if (u == NULL || xTaskCreate(ota_task, "ota_task", 8192, u, 5, NULL) != pdPASS) {
            free(u);
            OTA_Abort();
        }
    } else if (strncmp(cmd_line, prefixtime, strlen(prefixtime)) == 0) {
        CalBaseTime((uint64_t)atoll(cmd_line + strlen(prefixtime)));
        ESP_LOGD("HOST", "time sync: %lld", (long long)GetMilliTimeNow());
    } else {
        ESP_LOGW("HOST", "unknown command on OTA channel: %s", cmd_line);
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

// 连接上位机 -> 发上线包 -> 定时发心跳并非阻塞接收；断线后重连
// sockout 非空时把当前 socket 暴露给外部（OTA 任务用它回传进度）
static void LinkClientLoop(uint16_t port, const char *peer, LinkRecvHandler handler, int *sockout)
{
    ConnLog *log = NewLink("HOST", peer);
    char *txbuf = malloc(JSON_BUF_SIZE);   // 每个任务独立缓冲区
    char *rxbuf = malloc(JSON_BUF_SIZE);

    if (txbuf == NULL || rxbuf == NULL) {
        ESP_LOGE(log->tag, "%s: no memory", peer);
        free(txbuf);
        free(rxbuf);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        WaitNetwork();
        int sock = TcpConnect(HOST_IP_ADDR, port, log);
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
            ESP_LOGW(log->tag, "%s: send online packet failed: errno %d", peer, errno);
        }

        TickType_t lastWake = xTaskGetTickCount();
        while (1) {
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(HEART_BEAT_MS));
            PackLinkData(txbuf, JSON_BUF_SIZE, "heartBeat", "no");
            if (SendAll(sock, txbuf, strlen(txbuf)) < 0) {
                ESP_LOGW(log->tag, "%s: send heartbeat failed: errno %d", peer, errno);
                break;
            }

            int len = recv(sock, rxbuf, JSON_BUF_SIZE - 1, MSG_DONTWAIT);
            if (len > 0) {
                Mon_CaptureSock(sock, MON_RX, rxbuf, len);
                rxbuf[len] = '\0';
                handler(rxbuf, len);
            } else if (len == 0) {
                break;      // 上位机关闭了连接
            }
        }

        if (sockout != NULL) {
            *sockout = -1;
        }
        TcpClose(sock, log);
        vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
    }
}

void tcp_client1_task(void *pvParameters)
{
    LinkClientLoop(PORT1, "link channel", HandleTimestampData, NULL);
}

void tcp_client3_task(void *pvParameters)
{
    LinkClientLoop(OTA_PORT, "ota channel", HandleOtaCommandData, &ota_tcp_sock);
}

/* ---------------------------------------------------------- 网口 PLC 轮询 */

typedef int (*PlcConnectFunc)(ConnLog *log);
typedef int (*PlcPollFunc)(int sock);

// 网口 PLC 地址都是 IPv4（与上位机使用 IPv4 还是 IPv6 无关）
static __attribute__((unused)) int PlcTcpConnect(const char *ip, uint16_t port, ConnLog *log)
{
    struct sockaddr_in dest_addr = { 0 };
    int sock;

    dest_addr.sin_addr.s_addr = inet_addr(ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(log->tag, "unable to create socket: errno %d", errno);
        return -1;
    }
    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        LogConnectFail(log, ip, port, errno);
        close(sock);
        return -1;
    }
    LogConnected(log, ip, port);
    Mon_BindSocket(sock, MON_CH_PLC);     // 收发由 net_io 记录
    Net_SetTimeout(sock, PLC_NET_TIMEOUT_MS);
    return sock;
}

static __attribute__((unused)) void PlcNetClientLoop(const char *peer, PlcConnectFunc connect_fn,
                                                     PlcPollFunc poll_fn)
{
    ConnLog *log = NewLink("PLC", peer);
    while (1) {
        WaitNetwork();
        int sock = connect_fn(log);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
            continue;
        }

        TickType_t lastWake = xTaskGetTickCount();
        while (1) {
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PLC_POLL_INTERVAL_MS));
            if (!HostCmd_IsCollectEnabled()) {
                continue;   // 与串口轮询一致：上位机 EndFlush 后暂停采集
            }
            if (poll_fn(sock) < 0) {
                break;      // 链路失效
            }
        }

        TcpClose(sock, log);
        vTaskDelay(pdMS_TO_TICKS(CLIENT_RECONNECT_MS));
    }
}

#if defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
static int ModbusConnect(ConnLog *log)
{
    return PlcTcpConnect(GwConfig_Get()->modbusIp, GwConfig_Get()->modbusPort, log);
}

void tcp_modbus_client_task(void *pvParameters)
{
    PlcNetClientLoop("modbus tcp", ModbusConnect, PlcPoll_ModbusNet);
}
#endif

#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
static int FxConnect(ConnLog *log)
{
    return PlcTcpConnect(GwConfig_Get()->fxIp, GwConfig_Get()->fxPort, log);
}

void tcp_fx_client_task(void *pvParameters)
{
    PlcNetClientLoop("fx mc", FxConnect, PlcPoll_FxNet);
}
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
static int S7Connect(ConnLog *log)
{
    int sock = PlcTcpConnect(GwConfig_Get()->s7Ip, GwConfig_Get()->s7Port, log);

    if (sock >= 0 && S7_Handshake(sock) != 0) {
        log->up = false;
        close(sock);
        return -1;
    }
    return sock;
}

void tcp_ppi_client_task(void *pvParameters)
{
    PlcNetClientLoop("s7", S7Connect, PlcPoll_S7Net);
}
#endif
