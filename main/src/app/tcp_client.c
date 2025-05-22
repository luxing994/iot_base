/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdint.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "addr_from_stdin.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "iot_common.h"
#include "protocol_examples_common.h"
#include "time.h"
#include "tcp_server.h"
#include "fx_plc_protocol.h"
#include "ota_task.h"


#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT     // read data com
#define PORT1 CONFIG_EXAMPLE_PORT1   // init data com
#define PORT2 8767   // init data com

#define CLIENT_RECONNECT_INTERVAL  5 // second
#define HEART_BEAT_INTERVAL        1 // second

static char initdata[1024] = {0};
static char initrevdata[1024] = {0};
char mcu_ip[32] = {0};
int ota_tcp_sock;

static bool is_our_netif(const char *prefix, esp_netif_t *netif)
{
    return strncmp(prefix, esp_netif_get_desc(netif), strlen(prefix) - 1) == 0;
}

char* GetStaIp(void)
{
    return mcu_ip;
}

static void GetIpArry(char *ip, int *data, uint16_t len)
{
    char *token = NULL;
    int i = 0;

    token = strtok(ip, ".");
    for (i = 0; i < len && token != NULL; i++) {
        data[i] = atoi(token);
        token = strtok(NULL, ".");
    }
}

static void handle_tcp_ota_command(const char *cmd_line) {
    // 跳过前导空白
    while (*cmd_line == ' ' || *cmd_line == '\t') ++cmd_line;

    const char *prefix = "OTA_UPDATE:";
    if (strstr(cmd_line, prefix) == cmd_line) {
        const char *url = cmd_line + strlen(prefix);
        ESP_LOGI("OTA_CMD", "解析到 OTA 更新，URL=%s", url);
        char *u = strdup(url);
        if (u) {
            xTaskCreate(ota_task, "ota_task", 8192, u, 5, NULL);
        }
    } else {
        ESP_LOGI("OTA_CMD", "未知命令：%s", cmd_line);
    }
}

void PackInitData(char *strip)
{
    const char *TAG = "example_connect";
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = NULL;
    int addr[4] = {0};
    char ptr[32] = {0};
    int i;

    strcpy(ptr, strip);
    GetIpArry(ptr, addr, sizeof(addr) / sizeof(int));
    for (int i = 0; i < esp_netif_get_nr_of_ifs(); ++i) {
        netif = esp_netif_next(netif);
        if (is_our_netif(TAG, netif)) {
            esp_netif_get_ip_info(netif, &ip_info);
            if ((esp_ip4_addr1(&ip_info.ip) == addr[0]) && (esp_ip4_addr2(&ip_info.ip) == addr[1]) && \
                (esp_ip4_addr3(&ip_info.ip) == addr[2])) {
                sprintf(mcu_ip, "" IPSTR, IP2STR(&ip_info.ip));
                break;
            }
        }
    }
     
    (void)sprintf(initdata, "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"%s\",\n    \"devName\":\"%s\",\n"
            "    \"devStatus\":\"\",\n    \"devTypeId\":\"%s\",\n    \"orderName\":\"%s\",\n"
            "    \"orderId\":\"%s\",\n    \"devIP\":\"%s\",\n    \"connectPort\":\"%s\",\n"
            "    \"value\":\"NULL\",\n    \"timeStamp\":\"%lld\",\n    \"isAnswer\":\"yes\",\n"
            "    \"expand\":\"NULL\"};;**##",  \ 
            DEVID, "Hello", DEVNAME, DEVTYPEID, ORDERNAME, "initDev", mcu_ip, "8766", GetMilliTimeNow());
}

void PackHeartBeatData()
{
    (void)sprintf(initdata, "{\n    \"devId\":\"%s\",\n    \"devNumber\":\"%s\",\n    \"devName\":\"%s\",\n"
            "    \"devStatus\":\"\",\n    \"devTypeId\":\"%s\",\n    \"orderName\":\"%s\",\n"
            "    \"orderId\":\"%s\",\n    \"devIP\":\"%s\",\n    \"connectPort\":\"%s\",\n"
            "    \"value\":\"NULL\",\n    \"timeStamp\":\"%lld\",\n    \"isAnswer\":\"no\",\n"
            "    \"expand\":\"NULL\"};;**##",  \ 
            DEVID, "Hello", DEVNAME, DEVTYPEID, ORDERNAME, "heartBeat", mcu_ip, "8766", GetMilliTimeNow());
}

void tcp_client_task(void *pvParameters)
{
    int addr_family = 0;
    int ip_protocol = 0;
    int sock;
    uint32_t recvp;
    char host_ip[] = HOST_IP_ADDR;
    const char *TAG = "tcp client";
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CLIENT_RECONNECT_INTERVAL * 100;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(host_ip, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif
        sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            goto end;
        }
        ESP_LOGI(TAG, "Successfully connected");

        while (1) {
            if(xQueueReceive(xQueue1, &recvp, (TickType_t)10) == pdPASS) {
                ESP_LOGI(TAG, "Read data %s\n", (uint8_t *)recvp);
                int err = send(sock, (uint8_t *)recvp, strlen(recvp), 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        end:
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void tcp_client1_task(void *pvParameters)
{
    int addr_family = 0;
    int ip_protocol = 0;
    uint32_t recvp;
    char host_ip[] = HOST_IP_ADDR;
    const char *TAG = "tcp client1";
    TickType_t xLastWakeTime1, xLastWakeTime2;
 	const TickType_t xFrequency1 = CLIENT_RECONNECT_INTERVAL * 100;
    const TickType_t xFrequency2 = HEART_BEAT_INTERVAL * 100;
    

    while (1) {
        vTaskDelayUntil(&xLastWakeTime1, xFrequency1);
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT1);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(host_ip, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT1);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT1, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif
        ota_tcp_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (ota_tcp_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        // ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT1);
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT1);

        int err = connect(ota_tcp_sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            goto end;
        }
        ESP_LOGI(TAG, "Successfully connected");
        ESP_LOGI(TAG, "Firmware version: V3");

        PackInitData(host_ip);
        err = send(ota_tcp_sock, (uint8_t *)initdata, strlen(initdata), 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        }

        int len = recv(ota_tcp_sock, initrevdata, sizeof(initrevdata) - 1, 0);
        // Error occurred during receiving
        if (len < 0) {
            ESP_LOGE(TAG, "recv failed: errno %d", errno);
            // break;
        }
        // Data received
        else {
            initrevdata[len] = 0; // Null-terminate whatever we received and treat like a string
            ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
            ESP_LOGI(TAG, "%s", initrevdata);
            g_baseTime = atoll(initrevdata);
            ESP_LOGI(TAG, "time:%lld  timenow:%lld\n", g_baseTime, GetMilliTimeNow());
        }
        
        while (1) {
            vTaskDelayUntil(&xLastWakeTime2, xFrequency2);
            PackHeartBeatData();
            err = send(ota_tcp_sock, (uint8_t *)initdata, strlen(initdata), 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            // 非阻塞接收命令
            len = recv(ota_tcp_sock, initrevdata, sizeof(initrevdata) - 1, MSG_DONTWAIT);
            if (len > 0) {
                initrevdata[len] = '\0';
                // 按行拆分并逐行解析
                char *saveptr = NULL;
                char *line = strtok_r(initrevdata, "\r\n", &saveptr);
                while (line) {
                    handle_tcp_ota_command(line);
                    line = strtok_r(NULL, "\r\n", &saveptr);
                }
            }
        }

        end:
        if (ota_tcp_sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(ota_tcp_sock, 0);
            close(ota_tcp_sock);
        }
    }
    vTaskDelete(NULL);
}

#if (defined CONFIG_PLC_NETWORK)
static void NetReadSingleDataRegister(int sock, uint32_t address, uint16_t frnum)
{
    int i, err;
    const char *TAG = "tcp client2";
    
    FX_PackNetReadSingleDataRegister(address, frnum);
    // for (i = 0; i < sizeof(netsdatabuff); i++) {
    //     ESP_LOGI(TAG, "send%d:0x%02x", i + 1, ((uint8_t *)&netsdatabuff)[i]);
    // }
    
    err = send(sock, (uint8_t *)&netsdatabuff, sizeof(netsdatabuff), 0);
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }

    int len = recv(sock, initrevdata, sizeof(initrevdata) - 1, 0);
    // Error occurred during receiving
    if (len < 0) {
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        // break;
    }
    // Data received
    else {
        initrevdata[len] = 0; // Null-terminate whatever we received and treat like a string
        // ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
        // for (i = 0; i < len; i++) {
        //     ESP_LOGI(TAG, "recv%d:0x%02x", i + 1, initrevdata[i]);
        // }
        FX_NetDataRecvNotice(&initrevdata, len, frnum, 1);
    }
}


void tcp_client2_task(void *pvParameters)
{
    int addr_family = 0;
    int ip_protocol = 0;
    int sock, i = 0;
    uint32_t recvp;
    char host_ip[] = CONFIG_EXAMPLE_IPV4_ADDR1;
    const char *TAG = "tcp client2";
    TickType_t xLastWakeTime1, xLastWakeTime2;
 	const TickType_t xFrequency1 = CLIENT_RECONNECT_INTERVAL * 100;
    const TickType_t xFrequency2 = HEART_BEAT_INTERVAL * 100;
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime1, xFrequency1);
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(host_ip, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif
        sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, CONFIG_EXAMPLE_PORT2);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            goto end;
        }
        ESP_LOGI(TAG, "Successfully connected");
    
        while (1) {
            vTaskDelayUntil(&xLastWakeTime2, xFrequency2);
            
            NetReadSingleDataRegister(sock, i, i);
            i++;
            if (i > 5) {
                i = 0;
            }
        }

        end:
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}
#endif
