/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "mprotocols.h"
#include "hprotocols.h"
#include "iot_common.h"

#define CONTROLERTYPE 2
#define PATTERN_CHR_NUM    (3) 
#define BUF_SIZE 1024
#define RX_BUF_SIZE  (BUF_SIZE * 2)
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)

static QueueHandle_t uart1_queue;
static const char *TAG = "uart_events";
HproFuncCode funcCode;
// HproOpReadCode opCode;
char controlerStr[256] = {0};
char sendDataBuffer[15][16] = { { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x01 }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x02 },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x03 }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x04 },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x05 }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x06 },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x07 }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x08 },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x09 }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0A },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0B }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0C },
                               { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0D }, { 0x5A, 0xA5, 0x00, 0x02, 0x01, 0x0E } };
                               
const uint16_t polynom = 0xA001;
int sendflag = 0;

typedef struct {
    uint32_t front;
    uint32_t rear;
    uint8_t rdata;
    uint8_t recBuffer[RX_BUF_SIZE];
} UartBuffer;
UartBuffer uart1Buffer;
HproComFrame dataFrame;
HproComFrame sendDataFrame;

uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len)
{
	uint8_t i;
	uint16_t crc = 0xffff;
 
	if (len == 0) {
		len = 1;
	}
	while (len--) {
		crc ^= *ptr;
		for (i = 0; i<8; i++)
		{
			if (crc & 1) {
				crc >>= 1;
				crc ^= polynom;
			}
			else {
				crc >>= 1;
			}
		}
		ptr++;
	}
	return(crc);
}

int CheckCRC16(uint8_t *ptr, uint16_t len, uint16_t rcrc)
{
    uint16_t crc;

    crc = crc16bitbybit(ptr, len);
    if (rcrc != crc) {
        return -1; 
    }

    return 0;
}

static void UART_InitBuffer(void)
{
    uart1Buffer.front = uart1Buffer.rear = 0;
    memset(uart1Buffer.recBuffer, 0, RX_BUF_SIZE);
}

static int UART_WriteBufferByte(uint8_t data)
{
    if ((uart1Buffer.rear + 1) % (RX_BUF_SIZE) == uart1Buffer.front) {
		return -1;
	}

    uart1Buffer.recBuffer[uart1Buffer.rear] = data;
	uart1Buffer.rear = (uart1Buffer.rear + 1) % RX_BUF_SIZE;
	return 0;
}

static int UART_WriteBufferBytes(uint8_t *data, uint32_t size)
{
    int i;

    for (i = 0; i < size; i++) {
        if (UART_WriteBufferByte(data[i]) != 0) {
            return -1;
        }
    }
	return 0;
}

int UART_ReadBufferByte(uint8_t *data)
{
    if (uart1Buffer.front == uart1Buffer.rear) {
        return -1;
    }

    *data = uart1Buffer.recBuffer[uart1Buffer.front];
    uart1Buffer.front = (uart1Buffer.front + 1) % RX_BUF_SIZE;
    return 0;
}

int UART_ReadBufferBytes(uint8_t *data, uint32_t size)
{
    int i;

    for (i = 0; i < size; i++) {
        if (UART_ReadBufferByte(&data[i]) != 0) {
            return -1;
        }
    }
	return 0;
}

void ParseOpCode(char *str, uint8_t op)
{
    switch (op) {
        case BREAK: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR001", DEVTYPENAME, 0, dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;
        }
        case HMISTATUS: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
		        "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
		        "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR002", DEVTYPENAME, 0, dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;
        }
        case MODE: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR003", DEVTYPENAME, 0, dataFrame.data[0]);
            break;    
        }
        case COUNT: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"jian\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR004", DEVTYPENAME, 0, dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;    
        }
        case SCHEDULE: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR005", DEVTYPENAME, 0, dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;    
        }
        case PATTERN: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR006", DEVTYPENAME, 0, dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;    
        }
        case PITCH: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.1f\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR007", DEVTYPENAME, 0, (dataFrame.data[0] << 8 | dataFrame.data[1]) / 10.0);
            break;    
        }
        case PITCHCOUNT: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR008", DEVTYPENAME, 0, dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;    
        }
        case SPINDLERATE: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR009", DEVTYPENAME, 0, dataFrame.data[0] << 8 | dataFrame.data[1]);
            break;    
        }
        case BOOTTIME: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d;%d;%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR010", DEVTYPENAME, 0, dataFrame.data[0] << 8 | dataFrame.data[1],
                dataFrame.data[2] << 8 | dataFrame.data[3], dataFrame.data[4] << 8 | dataFrame.data[5]);
            break;    
        }
        case APPVERSION: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%s\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR011", DEVTYPENAME, 0, (char *)(&(dataFrame.data[0])));
            break;    
        }
        case CONTROLVERSION: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%s\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR012", DEVTYPENAME, 0, (char *)(&(dataFrame.data[0])));
            break;    
        }
        case MECHANICCALL: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR013", DEVTYPENAME, 0, dataFrame.data[0]);
            break;    
        }
        case MATERIALCALL: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR014", DEVTYPENAME, 0, dataFrame.data[0]);
            break;    
        }
        case OTHERCALL: {
            (void)sprintf(str, "{\n    \"id\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"  
                "    \"devTypeId\": \"%s\",\n    \"devTypeName\":\"%s\",\n    \"timeStamp\":\"%d\",\n"
                "    \"valueUnit\":\"NULL\",\n    \"value\":\"%d\",\n    \"expand\":\"NULL\"\n};\n", \ 
                ID, DEVID, DEVNAME, "FR015", DEVTYPENAME, 0, dataFrame.data[0]);
            break;    
        }
        default: {
            break;
        }
    }
}

int GetDataFromControler(void)
{
	uint8_t curData = 0;
	uint8_t lastData = 0;
    uint16_t length;
    uint16_t crc;
	int ret;
    static const char *GET_DATA_TAG = "GET_DATA_TASK";
    esp_log_level_set(GET_DATA_TAG, ESP_LOG_INFO);
	while (!((curData == HPRO_HEAD_SECOND_BYTE) && (lastData == HPRO_HEAD_FIRST_BYTE))) {
		lastData = curData;
		ret = UART_ReadBufferByte(&curData);
		if (ret != 0) {
			return -1;
		}
	}
    dataFrame.head[0] = HPRO_HEAD_FIRST_BYTE;
    dataFrame.head[1] = HPRO_HEAD_SECOND_BYTE;

    ret = UART_ReadBufferBytes(&dataFrame.length[0], 2);
    if (ret != 0) {
        return -1;
    }

    length = (dataFrame.length[0] << 8) | dataFrame.length[1];
    ESP_LOGI(GET_DATA_TAG, "read data length:%d\n", length);

    ret += UART_ReadBufferByte(&dataFrame.func);
	ret += UART_ReadBufferByte(&dataFrame.operate);
    ret += UART_ReadBufferBytes(&dataFrame.data[0], length - 2);
    if (ret != 0) {
        return -1;
    }

    ret = UART_ReadBufferBytes(&dataFrame.crc[0], 2);
    if (ret != 0) {
        return -1;
    }
    
    // // CRC16 check
    // if (CheckCRC16((uint8_t *)&dataFrame.head[0], length + 4, (dataFrame.crc[1] << 8) | dataFrame.crc[0]) != 0) {
    //     crc = crc16bitbybit((uint8_t *)&dataFrame.head[0], length + 4);
    //     ESP_LOGI(GET_DATA_TAG, "rec crc:%x%x, cal crc:%x%x", dataFrame.crc[0], dataFrame.crc[1], crc >> 8, crc & 0xff);
    //     return -1;
    // }

	return 0;
}

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart1_queue, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    UART_InitBuffer();
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    EventBits_t uxBits;
    uint32_t recvp;
    uint16_t crc;

    esp_log_level_set(TX_TASK_TAG, ESP_LOG_ERROR);
    while (1) {
        uxBits = xEventGroupWaitBits(xEventGroup1, BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7 | BIT_8 \
            | BIT_9 | BIT_10 | BIT_11 | BIT_12 | BIT_13 | BIT_14, pdTRUE, pdFALSE, (TickType_t)10);
        if ((uxBits & BIT_0) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[0], 6);
            memcpy(&sendDataBuffer[0][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[0], 8);
        } else if ((uxBits & BIT_1) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[1], 6);
            memcpy(&sendDataBuffer[1][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[1], 8);
        } else if ((uxBits & BIT_2) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[2], 6);
            memcpy(&sendDataBuffer[2][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[2], 8);
        } else if ((uxBits & BIT_3) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[3], 6);
            memcpy(&sendDataBuffer[3][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[3], 8);
        } else if ((uxBits & BIT_4) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[4], 6);
            memcpy(&sendDataBuffer[4][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[4], 8);
        } else if ((uxBits & BIT_5) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[5], 6);
            memcpy(&sendDataBuffer[5][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[5], 8);
        } else if ((uxBits & BIT_6) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[6], 6);
            memcpy(&sendDataBuffer[6][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[6], 8);
        } else if ((uxBits & BIT_7) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[7], 6);
            memcpy(&sendDataBuffer[7][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[7], 8);
        } else if ((uxBits & BIT_8) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[8], 6);
            memcpy(&sendDataBuffer[8][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[8], 8);
        } else if ((uxBits & BIT_9) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[9], 6);
            memcpy(&sendDataBuffer[9][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[9], 8);
        } else if ((uxBits & BIT_10) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[10], 6);
            memcpy(&sendDataBuffer[10][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[10], 8);
        } else if ((uxBits & BIT_11) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[11], 6);
            memcpy(&sendDataBuffer[11][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[11], 8);
        } else if ((uxBits & BIT_12) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[12], 6);
            memcpy(&sendDataBuffer[12][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[12], 8);
        } else if ((uxBits & BIT_13) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[13], 6);
            memcpy(&sendDataBuffer[13][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[13], 8);
        } else if ((uxBits & BIT_14) != 0) {
            crc = crc16bitbybit((uint8_t *)sendDataBuffer[14], 6);
            memcpy(&sendDataBuffer[14][6], &crc, 2);
            uart_write_bytes(UART_NUM_1, (uint8_t *)sendDataBuffer[14], 8);
        }
    }
}

void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    uint32_t sendaddr = (uint32_t)&controlerStr;
    uint16_t parameter = 0;
    uint8_t data;
    int ret;

    TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 10;
    
    xLastWakeTime = xTaskGetTickCount();
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        ret = GetDataFromControler();
		if (ret == 0) {
            ParseOpCode(controlerStr, dataFrame.operate);
            ESP_LOGI(RX_TASK_TAG, "Read bytes: '%s'", controlerStr);
			if (xQueueSend(xQueue1, (void *)&sendaddr, (TickType_t)10) != pdPASS) {
                //TO DO
            }
		}
    }
}

void uart_event_task(void *pvParameters)
{
    static const char *UART_EVENT_TASK_TAG = "UART_EVENT_TASK";
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);
    esp_log_level_set(UART_EVENT_TASK_TAG, ESP_LOG_ERROR);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RX_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_NUM_1);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM_1, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    UART_WriteBufferBytes(dtmp, event.size);
                    // uart_write_bytes(UART_NUM_1, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(UART_NUM_1, &buffered_size);
                    int pos = uart_pattern_pop_pos(UART_NUM_1);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(UART_NUM_1);
                    } else {
                        uart_read_bytes(UART_NUM_1, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(UART_NUM_1, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}