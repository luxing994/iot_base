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

static char initdata[1024] = {0};
char sta_ip[32] = {0};

char* GetStaIp(void)
{
    return sta_ip;
}

void PackInitData(void)
{
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = get_example_netif();
    esp_netif_get_ip_info(netif, &ip_info);

    (void)sprintf(sta_ip, "" IPSTR, IP2STR(&ip_info.ip));
    (void)sprintf(initdata, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"   
                "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"NULL\",\n    \"expand\":\"NULL\"\n};;**##",  \ 
                "-1", DEVID, DEVNAME, DEVTYPEID, DEVTYPENAME, sta_ip, "FR000", ORDERNAME, GetMilliTimeNow());
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

    while (1) {
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

        PackInitData();
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

    while (1) {
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

        PackInitData();
        err = send(sock, (uint8_t *)initdata, strlen(initdata), 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        }
        break;

        end:
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

