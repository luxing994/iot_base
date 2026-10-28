/*
 * 本机 TCP 服务端（只负责传输，业务处理见 host_cmd.c / file_transfer.c）
 *
 *   tcp_server_task   CONFIG_IOT_CMD_PORT  : 上位机 JSON 命令。TCP 是字节流，一次 recv 可能
 *                                           只有半条命令，也可能有多条，这里按花括号配对切分出
 *                                           完整的 JSON 对象后再交给 HostCmd_HandleMessage()
 *   tcp_server1_task  CONFIG_IOT_FILE_PORT : 文件分包，交给 FileTransfer_HandlePacket()
 */
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "iot_common.h"
#include "tcp_server.h"
#include "host_cmd.h"
#include "file_transfer.h"

#define PORT                CONFIG_IOT_CMD_PORT      // 命令端口
#ifdef CONFIG_IOT_FILE_SERVER
#define PORT1               CONFIG_IOT_FILE_PORT     // 文件端口
#endif
#define KEEPALIVE_IDLE      CONFIG_IOT_KEEPALIVE_IDLE
#define KEEPALIVE_INTERVAL  CONFIG_IOT_KEEPALIVE_INTERVAL
#define KEEPALIVE_COUNT     CONFIG_IOT_KEEPALIVE_COUNT

#define SERVER_RETRY_MS     5000
#define CMD_RX_BUF_SIZE     10240                   // 单条命令最大长度
#define FILE_RX_BUF_SIZE    FILETRANSSIZE

typedef void (*ConnectionHandler)(int sock, const char *tag);

/* ------------------------------------------------------------- 通用监听循环 */

static int CreateListenSocket(int addr_family, uint16_t port, const char *tag)
{
    struct sockaddr_storage dest_addr = { 0 };
    int ip_protocol = IPPROTO_IP;
    int opt = 1;
    int sock;

    if (addr_family == AF_INET) {
        struct sockaddr_in *addr4 = (struct sockaddr_in *)&dest_addr;
        addr4->sin_addr.s_addr = htonl(INADDR_ANY);
        addr4->sin_family = AF_INET;
        addr4->sin_port = htons(port);
    }
#ifdef CONFIG_IOT_HOST_IPV6
    else if (addr_family == AF_INET6) {
        struct sockaddr_in6 *addr6 = (struct sockaddr_in6 *)&dest_addr;
        addr6->sin6_family = AF_INET6;
        addr6->sin6_port = htons(port);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(tag, "Unable to create socket: errno %d", errno);
        return -1;
    }
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(tag, "Socket unable to bind port %d: errno %d", port, errno);
        close(sock);
        return -1;
    }
    if (listen(sock, 1) != 0) {
        ESP_LOGE(tag, "Error occurred during listen: errno %d", errno);
        close(sock);
        return -1;
    }
    ESP_LOGI(tag, "Socket listening on port %d", port);

    return sock;
}

static void SetKeepAlive(int sock)
{
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;

    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
}

static void LogPeer(const struct sockaddr_storage *addr, const char *tag)
{
    char addr_str[64] = {0};

    if (addr->ss_family == PF_INET) {
        inet_ntoa_r(((const struct sockaddr_in *)addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
    }
#ifdef CONFIG_IOT_HOST_IPV6
    else if (addr->ss_family == PF_INET6) {
        inet6_ntoa_r(((const struct sockaddr_in6 *)addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
    }
#endif
    ESP_LOGI(tag, "Socket accepted ip address: %s", addr_str);
}

// 监听 -> 逐个接受连接并交给 handler；出错时重新建立监听（原实现 accept 失败后任务直接退出）
static void TcpServerLoop(int addr_family, uint16_t port, const char *tag, ConnectionHandler handler)
{
    while (1) {
        int listen_sock = CreateListenSocket(addr_family, port, tag);
        if (listen_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(SERVER_RETRY_MS));
            continue;
        }

        while (1) {
            struct sockaddr_storage source_addr;
            socklen_t addr_len = sizeof(source_addr);
            int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
            if (sock < 0) {
                ESP_LOGE(tag, "Unable to accept connection: errno %d", errno);
                break;
            }

            SetKeepAlive(sock);
            LogPeer(&source_addr, tag);
            handler(sock, tag);
            shutdown(sock, 0);
            close(sock);
        }

        close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(SERVER_RETRY_MS));
    }
}

/* ------------------------------------------------------------- 命令端口 */

static char s_cmdBuf[CMD_RX_BUF_SIZE + 1];

// 从 start（指向 '{'）开始查找配对的 '}'，忽略字符串中的括号；找到返回其下标，否则返回 -1
static int FindJsonEnd(const char *buf, size_t start, size_t len)
{
    int depth = 0;
    bool inString = false;
    bool escape = false;

    for (size_t i = start; i < len; i++) {
        char c = buf[i];

        if (inString) {
            if (escape) {
                escape = false;
            } else if (c == '\\') {
                escape = true;
            } else if (c == '"') {
                inString = false;
            }
        } else if (c == '"') {
            inString = true;
        } else if (c == '{') {
            depth++;
        } else if (c == '}' && --depth == 0) {
            return (int)i;
        }
    }
    return -1;
}

// 处理缓冲区中所有完整的命令，返回剩余（不完整）数据移到开头后的长度
static size_t ProcessCommands(int sock, size_t len, const char *tag)
{
    size_t pos = 0;

    while (pos < len) {
        size_t start = pos;
        while (start < len && s_cmdBuf[start] != '{') {
            start++;                       // 跳过命令之间的空白或杂数据
        }
        if (start >= len) {
            return 0;
        }

        int end = FindJsonEnd(s_cmdBuf, start, len);
        if (end < 0) {
            // 命令还没收全，保留到下次 recv
            memmove(s_cmdBuf, s_cmdBuf + start, len - start);
            return len - start;
        }

        char saved = s_cmdBuf[end + 1];
        s_cmdBuf[end + 1] = '\0';
        const char *reply = HostCmd_HandleMessage(&s_cmdBuf[start]);
        s_cmdBuf[end + 1] = saved;

        if (reply != NULL && send(sock, reply, strlen(reply), 0) < 0) {
            ESP_LOGE(tag, "Error occurred during sending: errno %d", errno);
        }
        pos = end + 1;
    }
    return 0;
}

static void HandleCommandConnection(int sock, const char *tag)
{
    size_t len = 0;

    while (1) {
        int n = recv(sock, s_cmdBuf + len, CMD_RX_BUF_SIZE - len, 0);
        if (n < 0) {
            ESP_LOGE(tag, "Error occurred during receiving: errno %d", errno);
            return;
        }
        if (n == 0) {
            ESP_LOGW(tag, "Connection closed");
            return;
        }

        len = ProcessCommands(sock, len + n, tag);
        if (len >= CMD_RX_BUF_SIZE) {
            ESP_LOGE(tag, "command too long, dropped");
            len = 0;
        }
    }
}

void tcp_server_task(void *pvParameters)
{
    TcpServerLoop((int)pvParameters, PORT, "tcp_server", HandleCommandConnection);
}

/* ------------------------------------------------------------- 文件端口 */

#ifdef CONFIG_IOT_FILE_SERVER

static void HandleFileConnection(int sock, const char *tag)
{
    char buf[FILE_RX_BUF_SIZE + 1];

    FileTransfer_Reset();   // 上一个连接中途断开时，未传完的文件作废
    while (1) {
        int len = recv(sock, buf, FILE_RX_BUF_SIZE, 0);
        if (len < 0) {
            ESP_LOGE(tag, "Error occurred during receiving: errno %d", errno);
            return;
        }
        if (len == 0) {
            ESP_LOGW(tag, "Connection closed");
            return;
        }
        buf[len] = '\0';
        FileTransfer_HandlePacket(buf, len);
    }
}

void tcp_server1_task(void *pvParameters)
{
    TcpServerLoop((int)pvParameters, PORT1, "tcp_server1", HandleFileConnection);
}
#endif /* CONFIG_IOT_FILE_SERVER */
