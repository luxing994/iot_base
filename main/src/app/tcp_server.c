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
#include "driver/uart.h"
#include "protocol_examples_common.h"
#include "tcp_server.h"
#include "hprotocols.h"
#include "iot_common.h"
#include "fx_plc_protocol.h"
#include "hl_plc_protocol.h"
#include "ppi_plc_protocol.h"
#include "ls_plc_load_protocol.h"
#include "t_tester_protocol.h"
#include "ainuo_tester_ascii_protocol.h"
#include "master.h"
#include "tcp_master.h"
#include "plc_report.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "cJSON.h"
#include "ringbuffer.h"
#include "iot_time.h"

#define PORT1                       CONFIG_SERVER_PORT1     // file data com
#define PORT                        CONFIG_SERVER_PORT      // command com
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

struct FileData file = {0};
struct ConfigData config = {0};
CommandJsonData comdata = {
    .devId = DEVID,
    .devName = DEVID,
    .devTypeId = DEVTYPEID,
    .orderId = ORDERID,
    .orderName = ORDERNAME,
};

// 把 JSON 字符串字段拷贝到定长缓冲区；字段不存在或不是字符串时保持原值
static void JsonCopyString(const cJSON *obj, const char *key, char *dst, size_t size)
{
    const cJSON *item = cJSON_GetObjectItem(obj, key);

    if (cJSON_IsString(item) && item->valuestring != NULL) {
        strlcpy(dst, item->valuestring, size);
    }
}

// 读取数值字段，兼容 "12.5" 字符串和 12.5 数字两种写法
static int JsonGetNumber(const cJSON *obj, const char *key, double *out)
{
    const cJSON *item = cJSON_GetObjectItem(obj, key);

    if (cJSON_IsString(item) && item->valuestring != NULL) {
        *out = atof(item->valuestring);
        return 0;
    }
    if (cJSON_IsNumber(item)) {
        *out = item->valuedouble;
        return 0;
    }
    return -1;
}


char g_rxbuffer[10240] = {0};
uint32_t g_devStartFlushFlag = 1;
int g_snttestercurrentgroup = 0;
SNSetCaclReFillingMachine g_setsnrefillingmachine;

/*
{
	"devId": "1531243721197228032",
	"devName": "",
	"devTypeId": "1516606339298758656",
	"deviceOrderFile": "",
	"deviceOrderMode": "group",
	"groupId": "001",
	"deviceOrderWay": "write",
	"orderDate": "2022-05-30 21:46:21",
	"orderId": "FR022",
	"orderName": "",
	"parameterType": "01_02_00_00_00_00_00_00",
	"parameters": [
		[{
				"type": "",
				"value": "1500"
			}, {
				"type": "",
				"value": "50.00"
			}, {
				"type": "",
				"value": "0.200"
			}, {
				"type": "",
				"value": "0.0"
			},
			{
				"type": "",
				"value": "0.0"
			}, {
				"type": "",
				"value": "1.0"
			}, {
				"type": "",
				"value": "0"
			}, {
				"type": "",
				"value": "0"
			},
			{
				"type": "",
				"value": "0"
			}, {
				"type": "",
				"value": "220.0"
			}, {
				"type": "",
				"value": "1.0"
			}
		],
		[{
				"type": "",
				"value": "25.00"
			},
			{
				"type": "",
				"value": "100"
			}, {
				"type": "",
				"value": "1"
			}, {
				"type": "",
				"value": "1.0"
			}, {
				"type": "",
				"value": "0"
			},
			{
				"type": "",
				"value": "1.0"
			}, {
				"type": "",
				"value": "0"
			}
		]
	],
	"responseType": "",
	"timeStamp": "1653918381719"
}
*/

void ParseCommandJsonData(cJSON *root)
{
    cJSON *token = NULL;
    cJSON *item = NULL;
    cJSON *row = NULL;
    const char *TAG = "parse json data";
    int arraysizerow, arraysize = 0, i, j, k;
    int ttestparacount = 0;
    int ttestparabytecount = 0;

    double number;

    // 每条命令都重新解析模式，避免沿用上一条命令的 deviceOrderMode
    comdata.deviceOrderMode[0] = '\0';
    comdata.timeStamp[0] = '\0';

    JsonCopyString(root, "devId", comdata.devId, sizeof(comdata.devId));
    JsonCopyString(root, "devName", comdata.devName, sizeof(comdata.devName));
    JsonCopyString(root, "devTypeId", comdata.devTypeId, sizeof(comdata.devTypeId));
    JsonCopyString(root, "deviceOrderFile", comdata.deviceOrderFile, sizeof(comdata.deviceOrderFile));
    JsonCopyString(root, "deviceOrderMode", comdata.deviceOrderMode, sizeof(comdata.deviceOrderMode));
    JsonCopyString(root, "groupId", comdata.groupId, sizeof(comdata.groupId));
    JsonCopyString(root, "deviceOrderWay", comdata.deviceOrderWay, sizeof(comdata.deviceOrderWay));
    JsonCopyString(root, "orderDate", comdata.orderDate, sizeof(comdata.orderDate));
    JsonCopyString(root, "orderId", comdata.orderId, sizeof(comdata.orderId));
    JsonCopyString(root, "orderName", comdata.orderName, sizeof(comdata.orderName));
    ESP_LOGI(TAG, "groupId: %s", comdata.groupId);

    if (strcmp(comdata.deviceOrderMode, "single") == 0) {
        if (JsonGetNumber(root, "parameters", &number) == 0) {
            g_setsnrefillingmachine.setchargeamount = (float)number;
            ESP_LOGI(TAG, "paradatatype: %.2f", g_setsnrefillingmachine.setchargeamount);
        }

        JsonCopyString(root, "parameterType", g_setsnrefillingmachine.select, sizeof(g_setsnrefillingmachine.select));
    } else if (strcmp(comdata.deviceOrderMode, "group") == 0) {
        token = cJSON_GetObjectItem(root, "parameterType");
        if (cJSON_IsString(token) && token->valuestring != NULL) {
            strlcpy(comdata.parameterType, token->valuestring, sizeof(comdata.parameterType));
            ESP_LOGI(TAG, "paradatatype: %s", comdata.parameterType);
            TTesterGetSetParaStatus(comdata.parameterType);
            for (i = 0; i < sizeof(tterterparasetstatus) / sizeof(int); i++) {
                ttestparacount += ttestparanum[tterterparasetstatus[i]];
                ttestparabytecount += ttestparabytenum[tterterparasetstatus[i]];
            }
            TTesterGetSetParaByteNum(ttestparabytecount);
        }

        token = cJSON_GetObjectItem(root, "parameters");
        if (cJSON_IsArray(token)) {
            arraysizerow = cJSON_GetArraySize(token);
            for (i = 0; i < arraysizerow; i++) {
                arraysize += cJSON_GetArraySize(cJSON_GetArrayItem(token, i));
            }
            if (arraysize != ttestparacount) {
                ESP_LOGE(TAG, "arraysize error: correct size %d, error size %d ", ttestparacount, arraysize);
                return;
            }
            
            // JSON一维数组解析
            // item = token->child;
            // for (i = 0; i < 8; i++) {
            //     for (j = 0; j < ttestparanum[tterterparasetstatus[i]]; j++) {
            //         comdata.paradata[i][j].type = cJSON_GetObjectItem(item, "type")->valuestring;
            //         comdata.paradata[i][j].value = cJSON_GetObjectItem(item, "value")->valuestring;
            //         ESP_LOGI(TAG, "paradata[%d][%d] {type: %s value: %f}", i, j, comdata.paradata[i][j].type, atof(comdata.paradata[i][j].value));
            //         item = item->next;
            //     }
            // }

            // JSON二维数组解析
            for (i = 0,  k = 0; i < sizeof(tterterparasetstatus) / sizeof(int); i++) {
                if (tterterparasetstatus[i] == 0) {
                    continue;
                } else {
                    row = cJSON_GetArrayItem(token, k);
                    k++;
                    for (j = 0; j < ttestparanum[tterterparasetstatus[i]] && j < CMD_PARA_ITEMS; j++) {
                        item = cJSON_GetArrayItem(row, j);
                        comdata.paradata[i][j].type[0] = '\0';
                        strlcpy(comdata.paradata[i][j].value, "0", sizeof(comdata.paradata[i][j].value));
                        JsonCopyString(item, "type", comdata.paradata[i][j].type, sizeof(comdata.paradata[i][j].type));
                        JsonCopyString(item, "value", comdata.paradata[i][j].value, sizeof(comdata.paradata[i][j].value));
                        ESP_LOGI(TAG, "paradata[%d][%d] {type: %s value: %f}", i, j, comdata.paradata[i][j].type, atof(comdata.paradata[i][j].value));
                    }
                }
                
            }

            // 各测试项参数的字段映射由测试仪模块负责
            for (i = 0; i < CMD_PARA_GROUPS; i++) {
#ifdef CONFIG_TESTER_76T
                TTesterApplyItemParams(tterterparasetstatus[i], comdata.paradata[i]);
#endif
#ifdef CONFIG_TESTER_AINUO
                AINUO_TTesterApplyItemParams(tterterparasetstatus[i], comdata.paradata[i]);
#endif
            }
        }
    }   

    JsonCopyString(root, "responseType", comdata.responseType, sizeof(comdata.responseType));
    JsonCopyString(root, "timeStamp", comdata.timeStamp, sizeof(comdata.timeStamp));
    ESP_LOGI(TAG, "timenow: %lld", atoll(comdata.timeStamp));
}

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

// 结构体约 6KB，返回指针，避免调用方在任务栈上整体拷贝
const CommandJsonData *GetCommandJsonData(void)
{
    return &comdata;
}

int GetGroupIdFromRecvJsonData(void)
{
    if (sscanf(comdata.groupId, "%d", &g_snttestercurrentgroup) != 1) {
        g_snttestercurrentgroup = 0;
    }
    return g_snttestercurrentgroup;
}

int ParseFile(char *buffer, int length)
{
    char *delim = "::!!@@$$**";
    char *token= NULL;

    token = strtok(buffer, delim);
    // 先判断是否为控制包
    if ((token != NULL) && (file.flag == 0)) {        
        ESP_LOGI("ParseFile", "File Name: %s\n", token);
        strlcpy(file.name, token, sizeof(file.name));   // 原 memcpy 不检查长度，文件名超过 15 字节会越界
        token = strtok(NULL, delim);
        if ((token != NULL) && (file.flag == 0)) {
            ESP_LOGI("ParseFile", "File Size: %d\n", atoi(token));
            file.size = atoi(token);
            file.count = 0;
            file.countsize = 0;
            file.flag = 1;
        }
        // 服务端补齐240个字节
        // token = strtok(NULL, delim);
        // if (token != NULL) {
            
        // }
    } else {
        if (file.countsize < file.size) {
            if (length > FILETRANSSIZE) {
                length = FILETRANSSIZE;
            }
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

// 冷媒灌注机：PLC 回读到的设定充注量与上位机下发值一致时，回复 "true"
static void AckChargeAmountIfApplied(int sock)
{
    int frnum = 0;
    double applied;

#if defined(CONFIG_PLC_FX_SN_CACLREFILMAC)
    frnum = 10;                                                      // FR010 实际设定灌注量
#elif defined(CONFIG_PLC_HOSTLINK_SN_CACLREFILMAC)
    if (strcmp(g_setsnrefillingmachine.select, "A") == 0) {
        frnum = 26;                                                  // FR026 A充注设定值
    } else if (strcmp(g_setsnrefillingmachine.select, "B") == 0) {
        frnum = 27;                                                  // FR027 B充注设定值
    }
#endif

    if (frnum == 0 || PlcReport_GetBatchValue(frnum, &applied) != 0) {
        return;
    }
    if ((float)applied == g_setsnrefillingmachine.setchargeamount) {
        if (send(sock, "true", strlen("true"), 0) < 0) {
            ESP_LOGE("tcp_server", "Error occurred during sending: errno %d", errno);
        }
    }
}

static void do_retransmit(const int sock)
{
    int len, i, err;
    int orderId;
    int arraysize;
    EventBits_t uxBits;
    uint64_t timestart;
    
    cJSON *root = NULL;
    cJSON *token = NULL;
    cJSON *item = NULL;
    const char *TAG = "tcp_server";
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;

    do {
        len = recv(sock, g_rxbuffer, sizeof(g_rxbuffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            g_rxbuffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, g_rxbuffer);
            root = cJSON_Parse(g_rxbuffer);
            if (root != NULL) {
                ParseCommandJsonData(root);
                // 字段已全部拷贝进 comdata，可以立即释放（原实现每条命令泄漏一棵 cJSON 树）
                cJSON_Delete(root);
                root = NULL;
                if (atoll(comdata.timeStamp) > 0) {
                    CalBaseTime(atoll(comdata.timeStamp));
                }
                if (strcmp(comdata.orderId, "StartFlush") == 0) {
                    g_devStartFlushFlag = 1;
                } else if (strcmp(comdata.orderId, "EndFlush") == 0) {
                    g_devStartFlushFlag = 0;
                } else if (strcmp(comdata.orderId, "PowerOn") == 0) {
                    xEventGroupSetBits(xEventGroup3, BIT_0);
                } else if (strcmp(comdata.orderId, "PowerOff") == 0) {
                    xEventGroupSetBits(xEventGroup3, BIT_1);
                } else if (strcmp(comdata.deviceOrderMode, "single") == 0) {
                    xEventGroupSetBits(xEventGroup1, BIT_14);
                } else if (strcmp(comdata.deviceOrderMode, "group") == 0) {
                    xEventGroupSetBits(xEventGroup1, BIT_15);
                }
            }

            AckChargeAmountIfApplied(sock);
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
