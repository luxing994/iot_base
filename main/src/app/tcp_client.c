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

#define PORT CONFIG_EXAMPLE_PORT     // read data com
#define PORT1 CONFIG_EXAMPLE_PORT1   // init data com
#define PORT2 CONFIG_EXAMPLE_PORT2   // init data com
#define PORT3 8768   // OTA

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

static void handle_tcp_command(const char *cmd_line) {
    // 跳过前导空白
    while (*cmd_line == ' ' || *cmd_line == '\t') ++cmd_line;

    const char *prefixota = "OTA_UPDATE:";
    const char *prefixtime = "TIMESTAMP:";
    if (strstr(cmd_line, prefixota) == cmd_line) {
        const char *url = cmd_line + strlen(prefixota);
        ESP_LOGI("OTA_CMD", "OTA UPDATE URL=%s", url);
        char *u = strdup(url);
        if (u) {
            xTaskCreate(ota_task, "ota_task", 8192, u, 5, NULL);
        }
    } else if (strstr(cmd_line, prefixtime) == cmd_line) {
        const char *timestamp = cmd_line + strlen(prefixtime);
        ESP_LOGI("TIMESTAMP_CMD", "Received timestamp: %s", timestamp);
        g_baseTime = atoll(timestamp);
        ESP_LOGI("TIMESTAMP_CMD", "time:%lld  timenow:%lld\n", g_baseTime, GetMilliTimeNow());
    } else {
        ESP_LOGI("OTA_CMD", "UNKNOW COMMAND:%s", cmd_line);
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
    int sock;
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
        sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        // ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT1);
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT1);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            goto end;
        }
        ESP_LOGI(TAG, "Successfully connected");

        PackInitData(host_ip);
        err = send(sock, (uint8_t *)initdata, strlen(initdata), 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        }
        
        while (1) {
            vTaskDelayUntil(&xLastWakeTime2, xFrequency2);
            PackHeartBeatData();
            err = send(sock, (uint8_t *)initdata, strlen(initdata), 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            // 非阻塞接收命令
            int len = recv(sock, initrevdata, sizeof(initrevdata) - 1, MSG_DONTWAIT);
            if (len > 0) {
                initrevdata[len] = 0;
                g_baseTime = atoll(initrevdata);
                ESP_LOGI("TIMESTAMP_CMD", "time:%lld  timenow:%lld\n", g_baseTime, GetMilliTimeNow());
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

#if defined(CONFIG_PLC_NETWORK)
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
    uint16_t pdu_len = 0; // 定义并初始化 pdu_len
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime1, xFrequency1);
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT2);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(host_ip, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT2);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT2, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif

#if (defined CONFIG_PLC_PPI)
        // 连接到 PLC 并初始化 S7 通信
        int sock = s7_connect(host_ip, &pdu_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Failed to setup communication with S7 PLC");
            goto end;  // 如果连接失败，重新尝试
        }

        ESP_LOGI(TAG, "Successfully connected to S7 PLC");
#else
        sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT2);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            goto end;
        }
        ESP_LOGI(TAG, "Successfully connected");
#endif
        while (1) {
            vTaskDelayUntil(&xLastWakeTime2, xFrequency2);
#if defined(CONFIG_PLC_FX)
            FX_NetReadSingleYDataRegister(sock, 94, 1);        // 正常运行状态
            FX_NetReadSingleYDataRegister(sock, 93, 2);        // 暂停状态
            FX_NetReadSingleYDataRegister(sock, 92, 3);        // 异常停止状态
            FX_NetReadSingleDataRegister(sock, 214, 4);        // 白班生产时间
            FX_NetReadSingleDataRegister(sock, 215, 5);        // 晚班生产时间
            FX_NetReadDoubleDataRegister(sock, 30, 6);         // 总累计冲次
            FX_NetReadSingleDataRegister(sock, 10, 7);         // 实时冲速 
            FX_NetReadSingleDataRegister(sock, 101, 8);        // 当前累积生产数量
#endif

#if defined(CONFIG_PLC_MODBUS)
            MODBUS_NetReadFloatRegister(sock, 1, 0);
#endif

#if (defined CONFIG_PLC_PPI)
            S7_NetReadBitQRegister(sock, 0, 2, 1);         // 运行显示
            S7_NetReadBitMRegister(sock, 2, 0, 2);         // 启动
            S7_NetReadBitMRegister(sock, 2, 3, 3);         // 故障复位
            S7_NetReadBitDataRegister(sock, 3315, 1, 4);   // 电源输入故障
            S7_NetReadBitDataRegister(sock, 3315, 2, 5);   // 设备超温
            S7_NetReadBitDataRegister(sock, 3315, 3, 6);   // 输出过电压
            S7_NetReadBitDataRegister(sock, 3315, 4, 7);   // 输出过电流
            S7_NetReadBitDataRegister(sock, 3315, 5, 8);   // IGBT保护
            S7_NetReadBitDataRegister(sock, 3315, 6, 9);   // 输出低电压
            S7_NetReadBitDataRegister(sock, 3315, 7, 10);   // 风扇变频故障
            S7_NetReadBitDataRegister(sock, 3316, 1, 11);   // 故障显示
            S7_NetReadBitMRegister(sock, 0, 7, 12);         // 急停
            S7_NetReadFloatDataRegister(sock, 210, 13);     // 实时电压
            S7_NetReadFloatDataRegister(sock, 214, 14);     // 实时电流
            S7_NetReadFloatDataRegister(sock, 280, 15);     // 设备温度
            S7_NetReadFloatDataRegister(sock, 534, 16);     // 点解AH
            S7_NetReadFloatDataRegister(sock, 1648, 17);     // 电压给定
            S7_NetReadFloatDataRegister(sock, 1848, 18);     // 电流给定
            S7_NetReadIntDataRegister(sock, 900, 19);        // 批号
            S7_NetReadSingleDataRegister(sock, 520, 20);     // 电解时间H
            S7_NetReadSingleDataRegister(sock, 522, 21);     // 电解时间M
#endif
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

// void tcp_client2_task(void *pvParameters)
// {
//     const char *TAG = "tcp client2";
//     char host_ip[] = CONFIG_EXAMPLE_IPV4_ADDR1;
//     TickType_t xLastWakeTime1, xLastWakeTime2;
//     const TickType_t xFrequency1 = CLIENT_RECONNECT_INTERVAL * 100;
//     const TickType_t xFrequency2 = HEART_BEAT_INTERVAL * 100;

//     uint16_t pdu_len = 0; // 定义并初始化 pdu_len

//     while (1) {
//         vTaskDelayUntil(&xLastWakeTime1, xFrequency1);

//         // 连接到 PLC 并初始化 S7 通信
//         int sock = s7_connect(host_ip, &pdu_len);
//         if (sock < 0) {
//             ESP_LOGE(TAG, "Failed to setup communication with S7 PLC");
//             continue;  // 如果连接失败，重新尝试
//         }

//         ESP_LOGI(TAG, "Successfully connected to S7 PLC");

//         // ----------- 循环读取数据 -----------
//         while (1) {
//             vTaskDelayUntil(&xLastWakeTime2, xFrequency2);

//             // // 通过 S7comm 读取数据
//             // uint8_t bit_value = 0;
//             // if (s7_read_bit(sock, pdu_len, 0x83, 0, 0x0000, 0, &bit_value) == 0) {
//             //     ESP_LOGI(TAG, "M0.0 = %d", bit_value);
//             // }
//             S7_NetReadBitQRegister(sock, 0, 2, 1);         // 运行显示
//             S7_NetReadBitMRegister(sock, 2, 0, 2);         // 启动
//             S7_NetReadBitMRegister(sock, 2, 3, 3);         // 故障复位
//             S7_NetReadBitDataRegister(sock, 3315, 1, 4);   // 电源输入故障
//             S7_NetReadBitDataRegister(sock, 3315, 2, 5);   // 设备超温
//             S7_NetReadBitDataRegister(sock, 3315, 3, 6);   // 输出过电压
//             S7_NetReadBitDataRegister(sock, 3315, 4, 7);   // 输出过电流
//             S7_NetReadBitDataRegister(sock, 3315, 5, 8);   // IGBT保护
//             S7_NetReadBitDataRegister(sock, 3315, 6, 9);   // 输出低电压
//             S7_NetReadBitDataRegister(sock, 3315, 7, 10);   // 风扇变频故障
//             S7_NetReadBitDataRegister(sock, 3316, 1, 11);   // 故障显示
//             S7_NetReadBitMRegister(sock, 0, 7, 12);         // 急停
//             S7_NetReadFloatDataRegister(sock, 210, 13);     // 实时电压
//             S7_NetReadFloatDataRegister(sock, 214, 14);     // 实时电流
//             S7_NetReadFloatDataRegister(sock, 280, 15);     // 设备温度
//             S7_NetReadFloatDataRegister(sock, 534, 16);     // 点解AH
//             S7_NetReadFloatDataRegister(sock, 1648, 17);     // 电压给定
//             S7_NetReadFloatDataRegister(sock, 1848, 18);     // 电流给定
//             S7_NetReadIntDataRegister(sock, 900, 19);        // 批号
//             S7_NetReadSingleDataRegister(sock, 520, 20);     // 电解时间H
//             S7_NetReadSingleDataRegister(sock, 522, 21);     // 电解时间M
//         }

//         // 连接关闭
//         if (sock != -1) {
//             ESP_LOGE(TAG, "Shutting down socket...");
//             shutdown(sock, 0);
//             close(sock);  // 关闭 TCP 连接
//         }
//     }

//     vTaskDelete(NULL);
// }
#endif


void tcp_client3_task(void *pvParameters)
{
    int addr_family = 0;
    int ip_protocol = 0;
    int sock;
    uint32_t recvp;
    char host_ip[] = HOST_IP_ADDR;
    const char *TAG = "tcp client3";
    TickType_t xLastWakeTime1, xLastWakeTime2;
 	const TickType_t xFrequency1 = CLIENT_RECONNECT_INTERVAL * 100;
    const TickType_t xFrequency2 = HEART_BEAT_INTERVAL * 100;
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime1, xFrequency1);
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT3);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(host_ip, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT3);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT3, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif
        ota_tcp_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (ota_tcp_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        // ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT3);
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT3);

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

        while (1) {
            vTaskDelayUntil(&xLastWakeTime2, xFrequency2);
            PackHeartBeatData();
            err = send(ota_tcp_sock, (uint8_t *)initdata, strlen(initdata), 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            // 非阻塞接收命令
            int len = recv(ota_tcp_sock, initrevdata, sizeof(initrevdata) - 1, MSG_DONTWAIT);
            if (len > 0) {
                initrevdata[len] = '\0';
                // 按行拆分并逐行解析
                char *saveptr = NULL;
                char *line = strtok_r(initrevdata, "\r\n", &saveptr);
                while (line) {
                    handle_tcp_command(line);
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