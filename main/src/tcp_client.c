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
#include "cJSON.h"


#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT
#define BIT_0	( 1 << 0 )

extern EventGroupHandle_t xEventGroup1;
extern QueueHandle_t xQueue1;
static const char *TAG = "tcp client";
static const char *payload = "Message from ESP32\n";
char host_ip[] = HOST_IP_ADDR;
int sock, flag = 0;

void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    int addr_family = 0;
    int ip_protocol = 0;
    uint32_t recvp;

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
            continue;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            continue;
        }
        ESP_LOGI(TAG, "Successfully connected");
        flag = 1;

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

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
            flag = 0;
        }
    }
    vTaskDelete(NULL);
}

void tcp_client_recv_task(void *pvParameters)
{
    static const char *TCP_RECV_TASK_TAG = "TCP_RECV_TASK";
    TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 10;
    char rx_buffer[128];
    cJSON *root = NULL;
    cJSON *token = NULL;
    char *devId = NULL;

    xLastWakeTime = xTaskGetTickCount();
    esp_log_level_set(TCP_RECV_TASK_TAG, ESP_LOG_INFO);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (flag == 1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TCP_RECV_TASK_TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TCP_RECV_TASK_TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TCP_RECV_TASK_TAG, "%s", rx_buffer);
                root = cJSON_Parse(rx_buffer);
                if (root == NULL) {
                    ESP_LOGE(TCP_RECV_TASK_TAG, "JSON parse error\n");
                }
                token = cJSON_GetObjectItem(root, "respnoseInfo");
                if (token != NULL) {
                    xEventGroupSetBits(xEventGroup1, BIT_0);
                }
                // ESP_LOGI(TCP_RECV_TASK_TAG, "respnoseInfo:%s", token->valuestring);
                
                // token = cJSON_GetObjectItem(root, "devType");
                // ESP_LOGI(TCP_RECV_TASK_TAG, "devType:%s", token->valuestring);
                // if (strcmp(token->valuestring, "dev status") == 0) {
                //     xEventGroupSetBits(xEventGroup1, BIT_0);
                // }
            }
        }
    }
}
