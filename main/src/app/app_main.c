#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/sockets.h"   // AF_INET
#include "tcp_client.h"
#include "tcp_server.h"
#include "uart_task.h"
#include "sensor.h"
#include "max6675.h"
#include "fx_plc_protocol.h"
#include "master.h"
#include "tcp_master.h"
#include "iot_common.h"
#include "plc_poll.h"

QueueHandle_t xQueue1;
EventGroupHandle_t xEventGroup1;
EventGroupHandle_t xEventGroup2;
EventGroupHandle_t xEventGroup3;

void app_main(void)
{    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // 队列和事件组必须先于 GPIO 中断和各任务创建：中断回调里会直接置 xEventGroup1 的位
    xQueue1 = xQueueCreate(IOT_MSG_QUEUE_LEN, sizeof(char *));
    xEventGroup1 = xEventGroupCreate();
    xEventGroup2 = xEventGroupCreate();
    xEventGroup3 = xEventGroupCreate();

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

    /* 串口收发 */
#ifdef APP_UART_ENABLED
    xTaskCreate(uart_event_task, "uart_event_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL);
#endif
#ifdef APP_RX_TASK_ENABLED
    xTaskCreate(rx_task, "uart_rx_task", 1024*8, NULL, configMAX_PRIORITIES - 2, NULL);
#endif
    xTaskCreate(tx_task, "uart_tx_task", 1024*8, NULL, configMAX_PRIORITIES - 3, NULL);

    /* 网口 PLC */
#if defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
    xTaskCreate(tcp_modbus_client_task, "tcp_modbus_client", 1024*8, NULL, 5, NULL);
    xTaskCreate(rx_modbus_task, "tcp_modbus_rx_task", 1024*8, NULL, configMAX_PRIORITIES - 2, NULL);
#endif
#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
    xTaskCreate(tcp_fx_client_task, "tcp_fx_client", 1024*8, NULL, 5, NULL);
#endif
#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
    xTaskCreate(tcp_ppi_client_task, "tcp_ppi_client", 1024*8, NULL, 5, NULL);
    xTaskCreate(rx_ppi_task, "tcp_ppi_rx_task", 1024*8, NULL, configMAX_PRIORITIES - 2, NULL);
#endif

    /* 传感器 */
#ifdef CONFIG_SENSOR_MAX6675
    xTaskCreate(max6675_task, "max6675_task", 1024*4, NULL, 5, NULL);
#endif

    /* 与上位机通信 */
    xTaskCreate(tcp_client_task, "tcp_client", 1024*8, NULL, 5, NULL);
    xTaskCreate(tcp_client1_task, "tcp_client1", 1024*8, NULL, 5, NULL);
    xTaskCreate(tcp_client3_task, "tcp_client3", 1024*8, NULL, 5, NULL);
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);
    xTaskCreate(tcp_server1_task, "tcp_server1", 4096, (void*)AF_INET, 5, NULL);

    /* 串口/RTU 轮询（Modbus RTU 设备会在此任务里直接打包上报，栈适当加大） */
    xTaskCreate(send_data_task, "send_data_task", 1024*6, NULL, 5, NULL);
#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
    xTaskCreate(send_command_task, "send_command_task", 4096, NULL, 5, NULL);
#endif
}
