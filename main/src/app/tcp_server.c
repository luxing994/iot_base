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
#include "ls_plc_load_protocol.h"
#include "t_tester_protocol.h"
#include "ainuo_tester_ascii_protocol.h"
#include "master.h"
#include "tcp_master.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "cJSON.h"
#include "ringbuffer.h"
#include "time.h"

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
uint16_t configdata[10] = {0};
CommandJsonData comdata = {
    .devId = DEVID,
    .devName = DEVID,
    .devTypeId = DEVTYPEID,
    .orderId = ORDERID,
    .orderName = ORDERNAME,
};

extern TTesterSetPressurizationPara setprepara;
extern TTesterSetGroundingPara setgroundpara;
extern TTesterSetInsulationPara setinspara;
extern TTesterSetLeakagePara setleapara;
extern TTesterSetPowerPara setpowerpara;
extern TTesterSetStartupPara setstartuppara;
extern TTesterSetOpenshortPara setopenshortpara;
extern int ttestparanum[9];
extern int ttestparabytenum[9];
extern int tterterparasetstatus[8];
extern SNCaclReFillingMachine g_lastrefilldata;
extern SNHostLinkCaclReFillingMachine g_lasthostlinkrefilldata;

char g_rxbuffer[4096] = {0};
uint32_t g_devStartFlushFlag = 1;
uint32_t g_switchPowerOn = 0;
int g_snttestercurrentgroup = 0;
extern uint32_t g_devStartStatus;
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
    int arraysizerow, arraysize = 0, i, j;
    int ttestparacount = 0;
    int ttestparabytecount = 0;

    token = cJSON_GetObjectItem(root, "devId");
    if (token != NULL) {
        comdata.devId = (token->valuestring);
    }

    token = cJSON_GetObjectItem(root, "devName");
    if (token != NULL) {
        comdata.devName = (token->valuestring);
    }

    token = cJSON_GetObjectItem(root, "devTypeId");
    if (token != NULL) {
        comdata.devTypeId = (token->valuestring);
    }

    token = cJSON_GetObjectItem(root, "deviceOrderFile");
    if (token != NULL) {
        comdata.deviceOrderFile = (token->valuestring);
    }

    token = cJSON_GetObjectItem(root, "deviceOrderMode");
    if (token != NULL) {
        comdata.deviceOrderMode = (token->valuestring);
    }

    token = cJSON_GetObjectItem(root, "groupId");
    if (token != NULL) {
        comdata.groupId = (token->valuestring);
        ESP_LOGI(TAG, "groupId: %s", comdata.groupId);
    }

    token = cJSON_GetObjectItem(root, "deviceOrderWay");
    if (token != NULL) {
        comdata.deviceOrderWay = (token->valuestring);
    }

    token = cJSON_GetObjectItem(root, "orderDate");
    if (token != NULL) {
        comdata.orderDate = (token->valuestring);
    }

    token = cJSON_GetObjectItem(root, "orderId");
    if (token != NULL) {
        comdata.orderId = (token->valuestring);
    }

    token = cJSON_GetObjectItem(root, "orderName");
    if (token != NULL) {
        comdata.orderName = (token->valuestring);
    }

    
    if (strcmp(comdata.deviceOrderMode, "single") == 0) {
        token = cJSON_GetObjectItem(root, "parameters");
        if (token != NULL) {
            g_setsnrefillingmachine.setchargeamount = atof((token->valuestring));
        }
    } else if (strcmp(comdata.deviceOrderMode, "group") == 0) {
        token = cJSON_GetObjectItem(root, "parameterType");
        if (token != NULL) {
            comdata.parameterType = (token->valuestring);
            ESP_LOGI(TAG, "paradatatype: %s", comdata.parameterType);
            TTesterGetSetParaStatus(comdata.parameterType);
            for (i = 0; i < sizeof(tterterparasetstatus) / sizeof(int); i++) {
                ttestparacount += ttestparanum[tterterparasetstatus[i]];
                ttestparabytecount += ttestparabytenum[tterterparasetstatus[i]];
            }
            TTesterGetSetParaByteNum(ttestparabytecount);
        }

        token = cJSON_GetObjectItem(root, "parameters");
        if (token != NULL) {
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
            for (i = 0; i < arraysizerow; i++) {
                row = cJSON_GetArrayItem(token, i);
                for (j = 0; j < ttestparanum[tterterparasetstatus[i]]; j++) {
                    item =  cJSON_GetArrayItem(row, j);
                    comdata.paradata[i][j].type = cJSON_GetObjectItem(item, "type")->valuestring;
                    comdata.paradata[i][j].value = cJSON_GetObjectItem(item, "value")->valuestring;
                    ESP_LOGI(TAG, "paradata[%d][%d] {type: %s value: %f}", i, j, comdata.paradata[i][j].type, atof(comdata.paradata[i][j].value));
                }
            }

            for (i = 0; i < 8; i++) {
                switch (tterterparasetstatus[i]) {
                    case TTESTERPRESSURIZATION: {
                        setprepara.voltage[0] = TTesterStrChangeToUint(comdata.paradata[i][0].value) >> 8;
                        setprepara.voltage[1] = TTesterStrChangeToUint(comdata.paradata[i][0].value) & 0xff;
                        setprepara.curupperlim[0] = TTesterStrChangeToUint(comdata.paradata[i][1].value) >> 8;
                        setprepara.curupperlim[1] = TTesterStrChangeToUint(comdata.paradata[i][1].value) & 0xff;
                        setprepara.curlowerlim[0] = TTesterStrChangeToUint(comdata.paradata[i][2].value) >> 8;
                        setprepara.curlowerlim[1] = TTesterStrChangeToUint(comdata.paradata[i][2].value) & 0xff;
                        setprepara.uptime[0] = TTesterStrChangeToUint(comdata.paradata[i][3].value) >> 8;
                        setprepara.uptime[1] = TTesterStrChangeToUint(comdata.paradata[i][3].value) & 0xff;
                        setprepara.downtime[0] = TTesterStrChangeToUint(comdata.paradata[i][4].value) >> 8;
                        setprepara.downtime[1] = TTesterStrChangeToUint(comdata.paradata[i][4].value) & 0xff;
                        setprepara.testtime[0] = TTesterStrChangeToUint(comdata.paradata[i][5].value) >> 8;
                        setprepara.testtime[1] = TTesterStrChangeToUint(comdata.paradata[i][5].value) & 0xff;
                        setprepara.curtozero[0] = TTesterStrChangeToUint(comdata.paradata[i][6].value) >> 8;
                        setprepara.curtozero[1] = TTesterStrChangeToUint(comdata.paradata[i][6].value) & 0xff;
                        setprepara.teststatus = atoi(comdata.paradata[i][7].value);
                        setprepara.testmode = atoi(comdata.paradata[i][8].value);
                        setprepara.curset[0] = TTesterStrChangeToUint(comdata.paradata[i][9].value) >> 8;
                        setprepara.curset[1] = TTesterStrChangeToUint(comdata.paradata[i][9].value) & 0xff;
                        setprepara.suspendtime[0] = TTesterStrChangeToUint(comdata.paradata[i][10].value) >> 8;
                        setprepara.suspendtime[1] = TTesterStrChangeToUint(comdata.paradata[i][10].value) & 0xff;
                        break;
                    }
                    case TTESTERGROUNDING: {
#ifdef CONFIG_TESTER_76T
                        setgroundpara.curset[0] = TTesterStrChangeToUint(comdata.paradata[i][0].value) >> 8;
                        setgroundpara.curset[1] = TTesterStrChangeToUint(comdata.paradata[i][0].value) & 0xff;
                        setgroundpara.resupperlim[0] = TTesterStrChangeToUint(comdata.paradata[i][1].value) >> 8;
                        setgroundpara.resupperlim[1] = TTesterStrChangeToUint(comdata.paradata[i][1].value) & 0xff;
                        setgroundpara.reslowerlim[0] = TTesterStrChangeToUint(comdata.paradata[i][2].value) >> 8;
                        setgroundpara.reslowerlim[1] = TTesterStrChangeToUint(comdata.paradata[i][2].value) & 0xff;
                        setgroundpara.testtime[0] = TTesterStrChangeToUint(comdata.paradata[i][3].value) >> 8;
                        setgroundpara.testtime[1] = TTesterStrChangeToUint(comdata.paradata[i][3].value) & 0xff;
                        setgroundpara.testmode = atoi(comdata.paradata[i][4].value);
                        setgroundpara.suspendtime[0] = TTesterStrChangeToUint(comdata.paradata[i][5].value) >> 8;
                        setgroundpara.suspendtime[1] = TTesterStrChangeToUint(comdata.paradata[i][5].value) & 0xff;
                        setgroundpara.res0tozero[0] = TTesterStrChangeToUint(comdata.paradata[i][6].value) >> 8;
                        setgroundpara.res0tozero[1] = TTesterStrChangeToUint(comdata.paradata[i][6].value) & 0xff;
#endif

#ifdef CONFIG_TESTER_AINUO
                        ainuosetgroudingtestdata.current = atof(comdata.paradata[i][0].value);
                        ainuosetgroudingtestdata.resistance1 = atof(comdata.paradata[i][1].value);
                        ainuosetgroudingtestdata.resistance2 = atof(comdata.paradata[i][2].value);
                        ainuosetgroudingtestdata.time = atof(comdata.paradata[i][3].value);
#endif
                        break;
                    }
                    case TTESTERINSULATION: {
                        setinspara.voltage[0] = TTesterStrChangeToUint(comdata.paradata[i][0].value) >> 8;
                        setinspara.voltage[1] = TTesterStrChangeToUint(comdata.paradata[i][0].value) & 0xff;
                        setinspara.resupperlim[0] = TTesterStrChangeToUint(comdata.paradata[i][1].value) >> 8;
                        setinspara.resupperlim[1] = TTesterStrChangeToUint(comdata.paradata[i][1].value) & 0xff;
                        setinspara.reslowerlim[0] = TTesterStrChangeToUint(comdata.paradata[i][2].value) >> 8;
                        setinspara.reslowerlim[1] = TTesterStrChangeToUint(comdata.paradata[i][2].value) & 0xff;
                        setinspara.testtime[0] = TTesterStrChangeToUint(comdata.paradata[i][3].value) >> 8;
                        setinspara.testtime[1] = TTesterStrChangeToUint(comdata.paradata[i][3].value) & 0xff;
                        setinspara.delaytime[0] = TTesterStrChangeToUint(comdata.paradata[i][4].value) >> 8;
                        setinspara.delaytime[1] = TTesterStrChangeToUint(comdata.paradata[i][4].value) & 0xff;
                        setinspara.testmode = atoi(comdata.paradata[i][5].value);
                        setinspara.suspendtime[0] = TTesterStrChangeToUint(comdata.paradata[i][6].value) >> 8;
                        setinspara.suspendtime[1] = TTesterStrChangeToUint(comdata.paradata[i][6].value) & 0xff;
                        setinspara.res0tozero[0] = TTesterStrChangeToUint(comdata.paradata[i][7].value) >> 8;
                        setinspara.res0tozero[1] = TTesterStrChangeToUint(comdata.paradata[i][7].value) & 0xff;
                        setinspara.res1tozero[0] = TTesterStrChangeToUint(comdata.paradata[i][8].value) >> 8;
                        setinspara.res1tozero[1] = TTesterStrChangeToUint(comdata.paradata[i][8].value) & 0xff;
                        setinspara.res2tozero[0] = TTesterStrChangeToUint(comdata.paradata[i][9].value) >> 8;
                        setinspara.res2tozero[1] = TTesterStrChangeToUint(comdata.paradata[i][9].value) & 0xff;
                        break;
                    }
                    case TTESTERLEAKAGE: {
                        setleapara.voltage[0] = TTesterStrChangeToUint(comdata.paradata[i][0].value) >> 8;
                        setleapara.voltage[1] = TTesterStrChangeToUint(comdata.paradata[i][0].value) & 0xff;
                        setleapara.curupperlim[0] = TTesterStrChangeToUint(comdata.paradata[i][1].value) >> 8;
                        setleapara.curupperlim[1] = TTesterStrChangeToUint(comdata.paradata[i][1].value) & 0xff;
                        setleapara.curlowerlim[0] = TTesterStrChangeToUint(comdata.paradata[i][2].value) >> 8;
                        setleapara.curlowerlim[1] = TTesterStrChangeToUint(comdata.paradata[i][2].value) & 0xff;
                        setleapara.testtime[0] = TTesterStrChangeToUint(comdata.paradata[i][3].value) >> 8;
                        setleapara.testtime[1] = TTesterStrChangeToUint(comdata.paradata[i][3].value) & 0xff;
                        setleapara.teststatus = atoi(comdata.paradata[i][4].value);
                        setleapara.testmode = atoi(comdata.paradata[i][5].value);
                        setleapara.suspendtime[0] = TTesterStrChangeToUint(comdata.paradata[i][6].value) >> 8;
                        setleapara.suspendtime[1] = TTesterStrChangeToUint(comdata.paradata[i][6].value) & 0xff;
                        setleapara.res0tozero[0] = TTesterStrChangeToUint(comdata.paradata[i][7].value) >> 8;
                        setleapara.res0tozero[1] = TTesterStrChangeToUint(comdata.paradata[i][7].value) & 0xff;
                        setleapara.res1tozero[0] = TTesterStrChangeToUint(comdata.paradata[i][8].value) >> 8;
                        setleapara.res1tozero[1] = TTesterStrChangeToUint(comdata.paradata[i][8].value) & 0xff;
                        setleapara.res2tozero[0] = TTesterStrChangeToUint(comdata.paradata[i][9].value) >> 8;
                        setleapara.res2tozero[1] = TTesterStrChangeToUint(comdata.paradata[i][9].value) & 0xff;
                        setleapara.curtozero[0] = TTesterStrChangeToUint(comdata.paradata[i][10].value) >> 8;
                        setleapara.curtozero[1] = TTesterStrChangeToUint(comdata.paradata[i][10].value) & 0xff;
                        break;
                    }
                    case TTESTERPOWER: {
                        setpowerpara.voltage[0] = TTesterStrChangeToUint(comdata.paradata[i][0].value) >> 8;
                        setpowerpara.voltage[1] = TTesterStrChangeToUint(comdata.paradata[i][0].value) & 0xff;
                        setpowerpara.curupperlim[0] = TTesterStrChangeToUint(comdata.paradata[i][1].value) >> 8;
                        setpowerpara.curupperlim[1] = TTesterStrChangeToUint(comdata.paradata[i][1].value) & 0xff;
                        setpowerpara.curlowerlim[0] = TTesterStrChangeToUint(comdata.paradata[i][2].value) >> 8;
                        setpowerpara.curlowerlim[1] = TTesterStrChangeToUint(comdata.paradata[i][2].value) & 0xff;
                        setpowerpara.powerupperlim[0] = TTesterStrChangeToUint(comdata.paradata[i][3].value) >> 8;
                        setpowerpara.powerupperlim[1] = TTesterStrChangeToUint(comdata.paradata[i][3].value) & 0xff;
                        setpowerpara.powerlowerlim[0] = TTesterStrChangeToUint(comdata.paradata[i][4].value) >> 8;
                        setpowerpara.powerlowerlim[1] = TTesterStrChangeToUint(comdata.paradata[i][4].value) & 0xff;
                        setpowerpara.delaytime[0] = TTesterStrChangeToUint(comdata.paradata[i][5].value) >> 8;
                        setpowerpara.delaytime[1] = TTesterStrChangeToUint(comdata.paradata[i][5].value) & 0xff;
                        setpowerpara.testtime[0] = TTesterStrChangeToUint(comdata.paradata[i][6].value) >> 8;
                        setpowerpara.testtime[1] = TTesterStrChangeToUint(comdata.paradata[i][6].value) & 0xff;
                        setpowerpara.testmode = atoi(comdata.paradata[i][7].value);
                        setpowerpara.suspendtime[0] = TTesterStrChangeToUint(comdata.paradata[i][8].value) >> 8;
                        setpowerpara.suspendtime[1] = TTesterStrChangeToUint(comdata.paradata[i][8].value) & 0xff;
                        setpowerpara.voltype = atoi(comdata.paradata[i][9].value);
                        break;
                    }
                    case TTESTERSTARTUP: {
                        setstartuppara.voltage[0] = TTesterStrChangeToUint(comdata.paradata[i][0].value) >> 8;
                        setstartuppara.voltage[1] = TTesterStrChangeToUint(comdata.paradata[i][0].value) & 0xff;
                        setstartuppara.volupperlim[0] = TTesterStrChangeToUint(comdata.paradata[i][1].value) >> 8;
                        setstartuppara.volupperlim[1] = TTesterStrChangeToUint(comdata.paradata[i][1].value) & 0xff;
                        setstartuppara.vollowerlim[0] = TTesterStrChangeToUint(comdata.paradata[i][2].value) >> 8;
                        setstartuppara.vollowerlim[1] = TTesterStrChangeToUint(comdata.paradata[i][2].value) & 0xff;
                        setstartuppara.curupperlim[0] = TTesterStrChangeToUint(comdata.paradata[i][3].value) >> 8;
                        setstartuppara.curupperlim[1] = TTesterStrChangeToUint(comdata.paradata[i][3].value) & 0xff;
                        setstartuppara.curlowerlim[0] = TTesterStrChangeToUint(comdata.paradata[i][4].value) >> 8;
                        setstartuppara.curlowerlim[1] = TTesterStrChangeToUint(comdata.paradata[i][4].value) & 0xff;
                        setstartuppara.delaytime[0] = TTesterStrChangeToUint(comdata.paradata[i][5].value) >> 8;
                        setstartuppara.delaytime[1] = TTesterStrChangeToUint(comdata.paradata[i][5].value) & 0xff;
                        setstartuppara.testtime[0] = TTesterStrChangeToUint(comdata.paradata[i][6].value) >> 8;
                        setstartuppara.testtime[1] = TTesterStrChangeToUint(comdata.paradata[i][6].value) & 0xff;
                        setstartuppara.testmode = atoi(comdata.paradata[i][7].value);
                        setstartuppara.suspendtime[0] = TTesterStrChangeToUint(comdata.paradata[i][8].value) >> 8;
                        setstartuppara.suspendtime[1] = TTesterStrChangeToUint(comdata.paradata[i][8].value) & 0xff;
                        setstartuppara.voltype = atoi(comdata.paradata[i][9].value);
                        break;
                    }
                    case TTESTEROPENSHORT: {
                        setopenshortpara.curupperlim[0] = TTesterStrChangeToUint(comdata.paradata[i][0].value) >> 8;
                        setopenshortpara.curupperlim[1] = TTesterStrChangeToUint(comdata.paradata[i][0].value) & 0xff;
                        setopenshortpara.curlowerlim[0] = TTesterStrChangeToUint(comdata.paradata[i][1].value) >> 8;
                        setopenshortpara.curlowerlim[1] = TTesterStrChangeToUint(comdata.paradata[i][1].value) & 0xff;
                        setopenshortpara.testtime[0] = TTesterStrChangeToUint(comdata.paradata[i][2].value) >> 8;
                        setopenshortpara.testtime[1] = TTesterStrChangeToUint(comdata.paradata[i][2].value) & 0xff;
                        setopenshortpara.testmode = atoi(comdata.paradata[i][3].value);
                        setopenshortpara.suspendtime[0] = TTesterStrChangeToUint(comdata.paradata[i][4].value) >> 8;
                        setopenshortpara.suspendtime[1] = TTesterStrChangeToUint(comdata.paradata[i][4].value) & 0xff;
                        break;
                    }
                    case TTESTERDCVOLTAGE: {
                        break;
                    }
                    default: {
                        break;
                    }
                }
            }
        }
    }   

    token = cJSON_GetObjectItem(root, "responseType");
    if (token != NULL) {
        comdata.responseType = (token->valuestring);
    }

    token = cJSON_GetObjectItem(root, "timeStamp");
    if (token != NULL) {
        comdata.timeStamp = (token->valuestring);
        ESP_LOGI(TAG, "timenow: %lld", atoll(comdata.timeStamp));
    }
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

CommandJsonData GetCommandJsonData()
{
    return comdata;
}

int GetGroupIdFromRecvJsonData()
{
    (void)sscanf(comdata.groupId, "%d", &g_snttestercurrentgroup);
    return g_snttestercurrentgroup;
}

void ServerParseOpCode(int op)
{
     switch (op) {
        case BREAK: {
            strcpy(comdata.orderId, "FR001");
            xEventGroupSetBits(xEventGroup1, BIT_0);
            break;
        }
        case HMISTATUS: {
            strcpy(comdata.orderId, "FR002");
            xEventGroupSetBits(xEventGroup1, BIT_1);
            break;
        }
        case MODE: {
            strcpy(comdata.orderId, "FR003");
            xEventGroupSetBits(xEventGroup1, BIT_2);
            break;
        }
        case COUNT: {
            strcpy(comdata.orderId, "FR004");
            xEventGroupSetBits(xEventGroup1, BIT_3);
            break;
        }
        case SCHEDULE: {
            strcpy(comdata.orderId, "FR005");
            xEventGroupSetBits(xEventGroup1, BIT_4);
            break;
        }
        case PATTERN: {
            strcpy(comdata.orderId, "FR006");
            xEventGroupSetBits(xEventGroup1, BIT_5);
            break;
        }
        case PITCH: {
            strcpy(comdata.orderId, "FR007");
            xEventGroupSetBits(xEventGroup1, BIT_6);
            break;
        }
        case PITCHCOUNT: {
            strcpy(comdata.orderId, "FR008");
            xEventGroupSetBits(xEventGroup1, BIT_7);
            break;
        }
        case SPINDLERATE: {
            strcpy(comdata.orderId, "FR009");
            xEventGroupSetBits(xEventGroup1, BIT_8);
            break;
        }
        case BOOTTIME: {
            strcpy(comdata.orderId, "FR010");
            xEventGroupSetBits(xEventGroup1, BIT_9);
            break;
        }
        case APPVERSION: {
            strcpy(comdata.orderId, "FR011");
            xEventGroupSetBits(xEventGroup1, BIT_10);
            break;
        }
        case CONTROLVERSION: {
            strcpy(comdata.orderId, "FR012");
            xEventGroupSetBits(xEventGroup1, BIT_11);
            break;
        }
        case MECHANICCALL: {
            strcpy(comdata.orderId, "FR013");
            xEventGroupSetBits(xEventGroup1, BIT_12);
            break;
        }
        case MATERIALCALL: {
            strcpy(comdata.orderId, "FR014");
            xEventGroupSetBits(xEventGroup1, BIT_13);
            break;
        }
        case OTHERCALL: {
            strcpy(comdata.orderId, "FR015");
            xEventGroupSetBits(xEventGroup1, BIT_14);
            break;
        }
        case SYSTEMID: {
            strcpy(comdata.orderId, "FR016");
            xEventGroupSetBits(xEventGroup1, BIT_15);
            break;
        }
        case SWITCHCOUNT: {
            strcpy(comdata.orderId, "FR030");
            xEventGroupSetBits(xEventGroup1, BIT_16);
            break;
        }
        case SWITCHSTATUS: {
            strcpy(comdata.orderId, "FR031");
            xEventGroupSetBits(xEventGroup1, BIT_17);
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

int ParseFile(char *buffer, int length)
{
    char *delim = "::!!@@$$**";
    char *token= NULL;

    token = strtok(buffer, delim);
    // 先判断是否为控制包
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
        // 服务端补齐240个字节
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
    int len, i, err;
    int orderId;
    int arraysize;
    EventBits_t uxBits;
    uint64_t timestart;
    
    cJSON *root = NULL;
    cJSON *token = NULL;
    cJSON *item = NULL;
    const char *TAG = "tcp_server";

    do {
        len = recv(sock, g_rxbuffer, sizeof(g_rxbuffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            g_rxbuffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, g_rxbuffer);
            root = cJSON_Parse(&g_rxbuffer);
            if (root != NULL) {
                ParseCommandJsonData(root);
                // if (strcmp(comdata.devId, DEVID) == 0) {
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
                    } else {
                        // orderId = atoi(&((comdata.orderId)[2]));
                        // ESP_LOGI(TAG, "OrderId: %d\n", orderId);
                        // ServerParseOpCode(orderId);
                    }
                // }
            } 
#ifdef CONFIG_PLC_FX
            if (g_setsnrefillingmachine.setchargeamount == g_lastrefilldata.realsetchargeamount) {
                err = send(sock, "true", strlen("true"), 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
            }
#endif

#ifdef CONFIG_PLC_HOSTLINK
            // while (GetMilliTimeNow() - timestart < 10000) {
                if (g_setsnrefillingmachine.setchargeamount == g_lasthostlinkrefilldata.realsetchargeamount) {
                    err = send(sock, "true", strlen("true"), 0);
                    if (err < 0) {
                        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    }
                }
            // }
#endif   
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

void send_data_task(void *pvParameters)
{
    const char *TAG = "SEND_DATA_TASK";
    int i, rem = 0;
    
    TickType_t xLastWakeTime;
 	const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    
    
    xLastWakeTime = xTaskGetTickCount();
    esp_log_level_set(TAG, ESP_LOG_INFO);
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (g_devStartFlushFlag == 0) {
        } else {
#ifdef CONFIG_PLC_FX
    #ifdef CONFIG_PLC_RS232
            ReadSingleDataRegister(8000, 1);
    #endif

    #ifdef CONFIG_PLC_RS485
        #ifdef CONFIG_RY_LINE
            SerialReadSingleDataRegister(0, 255, 10, 232, 1); // 日跃PLC产量累加值寄存器
        #endif

        #ifdef CONFIG_XF_CONTROLER
            SerialReadSingleDataRegister(0, 255, 10, 180, 1);    //  行车总时间
            SerialReadSingleDataRegister(0, 255, 10, 181, 2);    //  行车设定时间
            SerialReadSingleDataRegister(0, 255, 10, 171, 3);    //  一车运行时间
            SerialReadSingleDataRegister(0, 255, 10, 172, 4);    //  二车运行时间
            SerialReadSingleDataRegister(0, 255, 10, 173, 5);    //  三车运行时间
            SerialReadSingleDataRegister(0, 255, 10, 174, 6);    //  四车运行时间
            SerialReadSingleDataRegister(0, 255, 10, 500, 7);    //  一车工位数值
            SerialReadSingleDataRegister(0, 255, 10, 550, 8);    //  二车工位数值
            SerialReadSingleDataRegister(0, 255, 10, 600, 9);    //  三车工位数值
            SerialReadSingleDataRegister(0, 255, 10, 650, 10);    //  四车工位数值
        #endif

        #ifdef CONFIG_SN_CACLREFILMAC
            SerialReadSingleFloatDataRegister(0, 255, 10, 164, 1);   //  系统真空
            SerialReadSingleDataRegister(0, 255, 10, 31, 2);         //  冷媒温度（A系统）
            SerialReadSingleDataRegister(0, 255, 10, 32, 3);         //  冷媒温度（B系统）
            SerialReadSingleDataRegister(0, 255, 10, 200, 4);        //  系统压力（A系统）
            SerialReadSingleDataRegister(0, 255, 10, 210, 5);        //  系统压力（B系统）
            SerialReadSingleFloatDataRegister(0, 255, 10, 512, 6);   //  灌注量
            SerialReadSingleDataRegister(0, 255, 10, 7974, 7);       //  单班产量
            SerialReadSingleDataRegister(0, 255, 10, 7982, 8);       //  总产量
            SerialReadSingleDataRegister(0, 255, 10, 112, 9);        //  测试结果
            SerialReadSingleFloatDataRegister(0, 255, 10, 16, 10);   //  系统真空
        #endif
    #endif
#endif

#ifdef CONFIG_PLC_LS_LOAD
    #ifdef CONFIG_PLC_RS232
            // for (i = 0; i < 5120; i++) {
            //     LSLoadReadSingleDataRegister(i, i);
            // }
            // DBSReadData();
    #endif

    #ifdef CONFIG_PLC_RS485
            //  LS PLC 485 use modbus RTU protocol
    #endif
#endif

#ifdef CONFIG_PLC_HOSTLINK
            HLReadSingleDataRegister(8101, 1);   // A模式选择
            HLReadSingleDataRegister(8501, 2);   // B模式选择
            HLReadBCDDataRegister(8824, 3);      // A检测时间
            HLReadBCDDataRegister(8844, 4);      // B检测时间
            HLReadFloatDataRegister(8820, 5);    // A抽空上限
            HLReadFloatDataRegister(8840, 6);    // B抽空上限
            HLReadFloatDataRegister(8822, 7);    // A抽空下限
            HLReadFloatDataRegister(8842, 8);    // B抽空下限
            HLReadFloatDataRegister(19704, 9);    // A充注压力
            HLReadFloatDataRegister(19724, 10);    // B充注压力
            HLReadFloatDataRegister(19746, 11);    // A真空度
            HLReadFloatDataRegister(19766, 12);   // B真空度
            HLReadFloatDataRegister(8828, 13);    // A设定量
            HLReadFloatDataRegister(8848, 14);    // B设定量
            HLReadFloatDataRegister(8182, 15);    // A百分比
            HLReadFloatDataRegister(8582, 16);    // B百分比
            HLReadFloatDataRegister(8194, 17);    // A充注速度
            HLReadFloatDataRegister(8594, 18);    // B充注速度
            HLReadFloatDataRegister(8180, 19);    // A充注量
            HLReadFloatDataRegister(8580, 20);    // B充注量
            HLReadSingleDataRegister(8102, 21);    // A工作状态
            HLReadSingleDataRegister(8502, 22);    // B工作状态
            HLReadFloatDataRegister(8104, 23);    // A充注时间
            HLReadFloatDataRegister(8504, 24);    // B充注时间
            HLReadSingleDataRegister(8940, 25);    // 结果判定
            HLReadFloatDataRegister(8920, 26);     // 充注设定值
#endif 

#ifdef CONFIG_TESTER_76T
            if (g_snttestercurrentgroup == 0) {
                TTesterReadCurrentGroup();
            } else {
                TTesterReadHistoryGroup(g_snttestercurrentgroup);
            }
            
#endif

#ifdef CONFIG_TESTER_AINUO
            AINUO_TTesterReadCurrentGroup();
#endif

#ifdef CONFIG_PLC_MUDBUS
    #ifdef CONFIG_MB_COMM_MODE_TCP
            tcp_master_operation_func(NULL);
    #else   
            master_operation_func(NULL);
    #endif
#endif
        }
    }
    vTaskDelete(NULL);
}

void send_command_task(void *pvParameters)
{
    const char *TAG = "SEND_COMMAND_TASK";
    EventBits_t uxBits;
#ifdef CONFIG_PLC_MUDBUS
    while (1) {
    #ifdef CONFIG_MB_COMM_MODE_TCP  
    #else   
        uxBits = xEventGroupWaitBits(xEventGroup3, BIT_0 | BIT_1, pdTRUE, pdFALSE, (TickType_t)10);
        if ((uxBits & BIT_0) != 0) {
            master_send_switch_func(1);     // 1:ON   0:OFF
        } else if ((uxBits & BIT_1) != 0) {
            master_send_switch_func(0);
        }      
    #endif
    }
#endif
    vTaskDelete(NULL);
}
