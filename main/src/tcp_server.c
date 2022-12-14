/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "tcp_server.h"
#include "hprotocols.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "cJSON.h"
#include "iot_common.h"
#include "ringbuffer.h"

#define PORT1                       8764
#define PORT                        CONFIG_EXAMPLE_PORT
#define KEEPALIVE_IDLE              CONFIG_EXAMPLE_KEEPALIVE_IDLE
#define KEEPALIVE_INTERVAL          CONFIG_EXAMPLE_KEEPALIVE_INTERVAL
#define KEEPALIVE_COUNT             CONFIG_EXAMPLE_KEEPALIVE_COUNT

struct FileData {
    char name[16];
    int size;
    int count;
    int countsize;
    int flag;
    char buffer[FILETRANSSIZE];
};

struct ConfigData {
    uint16_t tasknum;
    uint16_t taskpitch;
    uint16_t taskspeed;
    uint16_t taskcount;
    uint16_t tasktime;
    uint8_t mode;
};

struct FileData file = {0};
struct ConfigData config = {0};
uint16_t configdata[10] = {0};

int GetFileCount(uint8_t *count)
{
    if (count == NULL) {
        return -1;
    }

    *count = file.count;

    return 0;
}

int GetFileData(uint8_t *data, int size)
{
    if (data == NULL || size < FILETRANSSIZE) {
        return -1;
    }

    memcpy(data, file.buffer, FILETRANSSIZE);

    return 0;
}

int GetTaskNum(uint16_t *tasknum)
{
    if (tasknum == NULL) {
        return -1;
    }

    *tasknum = config.tasknum;

    return 0;
}

int GetTaskPitch(uint16_t *taskpitch)
{
    if (taskpitch == NULL) {
        return -1;
    }

    *taskpitch = config.taskpitch;

    return 0;
}

int GetTaskSpeed(uint16_t *taskspeed)
{
    if (taskspeed == NULL) {
        return -1;
    }

    *taskspeed = config.taskspeed;

    return 0;
}

int GetTaskCount(uint16_t *taskcount)
{
    if (taskcount == NULL) {
        return -1;
    }

    *taskcount = config.taskcount;

    return 0;
}

int GetTaskTime(uint16_t *tasktime)
{
    if (tasktime == NULL) {
        return -1;
    }

    *tasktime = config.tasktime;

    return 0;
}

int GetMode(uint8_t *mode)
{
    if (mode == NULL) {
        return -1;
    }

    *mode = config.mode;

    return 0;
}

int ParseFile(char *buffer, int length)
{
    char *delim = "::!!@@$$**";
    char *token= NULL;

    token = strtok(buffer, delim);
    // ???????????????????????????
    if ((token != NULL) && (file.flag == 0)) {        
        ESP_LOGI("ParseFile", "File Name: %s\n", token);
        memcpy(file.name, token, strlen(token));
        token = strtok(NULL, delim);
        if ((token != NULL) && (file.flag == 0)) {
            ESP_LOGI("ParseFile", "File Size: %d\n", atoi(token));
            file.size = atoi(token);
            file.count = 0;
            file.countsize = 0;
            file.flag = 1;
        }
        // ???????????????240?????????
        // token = strtok(NULL, delim);
        // if (token != NULL) {
            
        // }
    } else {
        if (file.countsize < file.size) {
            memset(file.buffer, 0, FILETRANSSIZE);
            memcpy(file.buffer, buffer, length);
            file.countsize += length;
            if (file.countsize == file.size) {
                file.flag = 0;
            }
            file.count++;
            switch (atoi(&((file.name)[2]))) {
                ESP_LOGI("ParseFile", "Number %d\n", atoi(&((file.name)[2])));
                case APPUPDATE: {
                    xEventGroupSetBits(xEventGroup2, BIT_0);
                    break;
                }
                default: {
                    break;
                }
            }
        }
    }

    return 0;
}

static void do_retransmit(const int sock)
{
    int len, i;
    int orderId;
    int arraysize;
    char rx_buffer[1024] = {0};
    cJSON *root = NULL;
    cJSON *token = NULL;
    cJSON *item = NULL;
    const char *TAG = "tcp_server";

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);
            root = cJSON_Parse(&rx_buffer);
            if (root != NULL) {
                token = cJSON_GetObjectItem(root, "orderId");
                if (token != NULL) {
                    orderId = atoi(&(token->valuestring)[2]);
                    ESP_LOGI(TAG, "orderId:%d", orderId);
                    token = cJSON_GetObjectItem(root, "parameters");
                    if (token != NULL) {
                        arraysize = cJSON_GetArraySize(token);
                        ESP_LOGI(TAG, "arraysize %d", arraysize);
                        item = token->child;
                        
                        for (i = 0; i < arraysize; i++) {
                            configdata[i] = atoi(cJSON_GetObjectItem(item, "value")->valuestring);
                            ESP_LOGI(TAG, "configdata[%d]: %d", i, configdata[i]);
                            item = item->next;
                        }
                        config.tasknum = configdata[0];
                        config.taskpitch = configdata[1];
                        config.taskspeed = configdata[2];
                        config.taskcount = configdata[3];
                        config.tasktime = configdata[4];
                        config.mode = (uint8_t)configdata[5];
                    }
                    switch (orderId) {
                        case BREAK: {
                            xEventGroupSetBits(xEventGroup1, BIT_0);
                            break;
                        }
                        case HMISTATUS: {
                            xEventGroupSetBits(xEventGroup1, BIT_1);
                            break;
                        }
                        case MODE: {
                            xEventGroupSetBits(xEventGroup1, BIT_2);
                            break;
                        }
                        case COUNT: {
                            xEventGroupSetBits(xEventGroup1, BIT_3);
                            break;
                        }
                        case SCHEDULE: {
                            xEventGroupSetBits(xEventGroup1, BIT_4);
                            break;
                        }
                        case PATTERN: {
                            xEventGroupSetBits(xEventGroup1, BIT_5);
                            break;
                        }
                        case PITCH: {
                            xEventGroupSetBits(xEventGroup1, BIT_6);
                            break;
                        }
                        case PITCHCOUNT: {
                            xEventGroupSetBits(xEventGroup1, BIT_7);
                            break;
                        }
                        case SPINDLERATE: {
                            xEventGroupSetBits(xEventGroup1, BIT_8);
                            break;
                        }
                        case BOOTTIME: {
                            xEventGroupSetBits(xEventGroup1, BIT_9);
                            break;
                        }
                        case APPVERSION: {
                            xEventGroupSetBits(xEventGroup1, BIT_10);
                            break;
                        }
                        case CONTROLVERSION: {
                            xEventGroupSetBits(xEventGroup1, BIT_11);
                            break;
                        }
                        case MECHANICCALL: {
                            xEventGroupSetBits(xEventGroup1, BIT_12);
                            break;
                        }
                        case MATERIALCALL: {
                            xEventGroupSetBits(xEventGroup1, BIT_13);
                            break;
                        }
                        case OTHERCALL: {
                            xEventGroupSetBits(xEventGroup1, BIT_14);
                            break;
                        }
                        case TASKNUMBER: {
                            xEventGroupSetBits(xEventGroup2, BIT_6);
                            break;
                        }
                        case TASKPITCH: {
                            xEventGroupSetBits(xEventGroup2, BIT_7);
                            break;
                        }
                        case TASKSPEED: {
                            xEventGroupSetBits(xEventGroup2, BIT_8);
                            break;
                        }
                        case TASKCOUNT: {
                            xEventGroupSetBits(xEventGroup2, BIT_9);
                            break;
                        }
                        case TASKTIME: {
                            xEventGroupSetBits(xEventGroup2, BIT_10);
                            break;
                        }
                        case SETMODE: {
                            xEventGroupSetBits(xEventGroup2, BIT_11);
                            break;
                        }
                        default: {
                            break;
                        }
                    }
                }
            }
        }
    } while (len > 0);
}

static void do_retransmit1(const int sock)
{
    int len;
    char rx_buffer[241] = {0};
    const char *TAG = "tcp_server1";

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);
            ParseFile(rx_buffer, len);
        }
    } while (len > 0);
}

void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;
    const char *TAG = "tcp_server";

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#ifdef CONFIG_EXAMPLE_IPV6
    else if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#ifdef CONFIG_EXAMPLE_IPV6
        else if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void tcp_server1_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;
    const char *TAG = "tcp_server1";

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT1);
        ip_protocol = IPPROTO_IP;
    }
#ifdef CONFIG_EXAMPLE_IPV6
    else if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT1);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT1);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#ifdef CONFIG_EXAMPLE_IPV6
        else if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        do_retransmit1(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}
