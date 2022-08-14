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


#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT     // read data com
#define PORT1 CONFIG_EXAMPLE_PORT1   // init data com

#define CLIENT_RECONNECT_INTERVAL  5 // second
#define HEART_BEAT_INTERVAL        1 // second

static char initdata[1024] = {0};
char mcu_ip[32] = {0};

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

    token = strtok(ip, ".");mcu_ip
    for (i = 0; i < len && token != NULL; i++) {
        data[i] = atoi(token);
        token = strtok(NULL, ".");
    }
}

void PackInitData(char *strip)
{
    const char *TAG = "example_connect";
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = NULL;
    char *token = NULL;
    int flag;
    int addr[4] = {0};

    GetIpArry(strip, addr, sizeof(addr) / sizeof(int));
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

    (void)sprintf(initdata, "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"   
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"NULL\",\n    \"expand\":\"NULL\"\n};;**##",  \ 
                "-1", DEVID, DEVNAME, DEVTYPEID, DEVTYPENAME, mcu_ip, "initDev", ORDERNAME, GetMilliTimeNow());
}

void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
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

        err = send(sock, (uint8_t *)initdata, strlen(initdata), 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        }

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
    char rx_buffer[128];
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
        memset(initdata, 0, sizeof(initdata));
        strcpy(initdata, "Hello;;**##");
        while (1) {
            vTaskDelayUntil(&xLastWakeTime2, xFrequency2);
            err = send(sock, (uint8_t *)initdata, strlen(initdata), 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
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
