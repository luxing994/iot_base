#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/sockets.h"   // AF_INET
#include "sdkconfig.h"
#include "iot_common.h"
#include "tcp_client.h"
#include "tcp_server.h"
#include "uart_task.h"
#include "plc_poll.h"
#include "sensor.h"
#include "max6675.h"
#include "master.h"

static const char *TAG = "APP_MAIN";

QueueHandle_t xQueue1;
EventGroupHandle_t xEventGroup1;
EventGroupHandle_t xEventGroup2;
EventGroupHandle_t xEventGroup3;

/*
 * 任务优先级（数值越大越高）。lwIP tcpip 任务为 18、Wi-Fi 任务为 23，
 * 应用任务都放在它们之下，避免串口解析等工作影响网络协议栈。
 */
#define PRIO_UART_EVENT     12      // 及时把 UART 驱动数据搬进环形缓冲区
#define PRIO_UART_RX        10      // 解析串口应答
#define PRIO_COMMAND        9       // 执行上位机命令
#define PRIO_POLL           6       // 设备轮询（串口 / 网口 / 传感器）
#define PRIO_NETWORK        5       // 与上位机的连接

#define APP_MAX_TASKS       20

static TaskHandle_t s_tasks[APP_MAX_TASKS];
static int s_taskCount;

// 创建任务并记录句柄（供栈监视使用）
static void StartTask(TaskFunction_t fn, const char *name, uint32_t stack, void *arg, UBaseType_t prio)
{
    TaskHandle_t handle = NULL;

    if (xTaskCreate(fn, name, stack, arg, prio, &handle) != pdPASS) {
        ESP_LOGE(TAG, "create task %s failed", name);
        return;
    }
    if (s_taskCount < APP_MAX_TASKS) {
        s_tasks[s_taskCount++] = handle;
    }
}

#ifdef CONFIG_IOT_DEBUG_STACK_MONITOR
// 周期打印各任务历史最小剩余栈（字节），用于调整栈大小
static void stack_monitor_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_IOT_DEBUG_STACK_MONITOR_INTERVAL_S * 1000));
        for (int i = 0; i < s_taskCount; i++) {
            ESP_LOGI(TAG, "stack free %-20s %5u", pcTaskGetName(s_tasks[i]),
                     (unsigned)uxTaskGetStackHighWaterMark(s_tasks[i]));
        }
        ESP_LOGI(TAG, "heap free %u, min %u", (unsigned)esp_get_free_heap_size(),
                 (unsigned)esp_get_minimum_free_heap_size());
    }
}
#endif

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 按 menuconfig 配置连接 Wi-Fi 或以太网
    ESP_ERROR_CHECK(example_connect());

    // 队列和事件组必须先于 GPIO 中断和各任务创建：中断回调里会直接置 xEventGroup1 的位
    xQueue1 = xQueueCreate(IOT_MSG_QUEUE_LEN, sizeof(char *));
    xEventGroup1 = xEventGroupCreate();
    xEventGroup2 = xEventGroupCreate();
    xEventGroup3 = xEventGroupCreate();

    /* 外设初始化 */
#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
    master_init();
#endif
#ifdef APP_UART_ENABLED
    uart_init();
#endif
    InitSensorGpio();
#ifdef CONFIG_SENSOR_MAX6675
    MAX6675_Init();
#endif
    FXPLC_InitBuffer();
    MODBUSPLC_InitBuffer();
    PPIPLC_InitBuffer();

    /* 串口收发与命令执行 */
#ifdef APP_UART_ENABLED
    StartTask(uart_event_task, "uart_event", 1024 * 4, NULL, PRIO_UART_EVENT);
#endif
#ifdef APP_RX_TASK_ENABLED
    StartTask(rx_task, "uart_rx", 1024 * 8, NULL, PRIO_UART_RX);
#endif
    StartTask(cmd_task, "cmd", 1024 * 8, NULL, PRIO_COMMAND);

    /* 设备轮询（Modbus RTU 设备会在轮询任务里直接打包上报，栈适当加大） */
    StartTask(plc_poll_task, "plc_poll", 1024 * 6, NULL, PRIO_POLL);
#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
    StartTask(send_command_task, "modbus_cmd", 1024 * 4, NULL, PRIO_COMMAND);
#endif
#if defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
    StartTask(tcp_modbus_client_task, "modbus_net", 1024 * 8, NULL, PRIO_POLL);
    StartTask(rx_modbus_task, "modbus_net_rx", 1024 * 8, NULL, PRIO_UART_RX);
#endif
#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
    StartTask(tcp_fx_client_task, "fx_net", 1024 * 8, NULL, PRIO_POLL);
#endif
#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
    StartTask(tcp_ppi_client_task, "s7_net", 1024 * 8, NULL, PRIO_POLL);
    StartTask(rx_ppi_task, "s7_net_rx", 1024 * 8, NULL, PRIO_UART_RX);
#endif
#ifdef CONFIG_SENSOR_MAX6675
    StartTask(max6675_task, "max6675", 1024 * 4, NULL, PRIO_POLL);
#endif

    /* 与上位机通信 */
    StartTask(tcp_client_task, "host_report", 1024 * 8, NULL, PRIO_NETWORK);
    StartTask(tcp_client1_task, "host_link", 1024 * 8, NULL, PRIO_NETWORK);
    StartTask(tcp_client3_task, "host_ota", 1024 * 8, NULL, PRIO_NETWORK);
    StartTask(tcp_server_task, "cmd_server", 1024 * 4, (void *)AF_INET, PRIO_NETWORK);
#ifdef CONFIG_IOT_FILE_SERVER
    StartTask(tcp_server1_task, "file_server", 1024 * 4, (void *)AF_INET, PRIO_NETWORK);
#endif

#ifdef CONFIG_IOT_DEBUG_STACK_MONITOR
    xTaskCreate(stack_monitor_task, "stack_monitor", 1024 * 3, NULL, 1, NULL);
#endif
}
