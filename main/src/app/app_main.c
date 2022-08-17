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
#include "addr_from_stdin.h"
#include "tcp_client.h"
#include "tcp_server.h"
#include "uart_task.h"
#include "sensor.h"
#include "fx_plc_protocol.h"
#include "master.h"
#include "tcp_master.h"

QueueHandle_t xQueue1;
EventGroupHandle_t xEventGroup1;
EventGroupHandle_t xEventGroup2;

void app_main(void)
{    
#if defined(CONFIG_MB_COMM_MODE_RTU) || defined(CONFIG_PLC_FX)
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
#endif

#ifdef CONFIG_PLC_MUDBUS
    #ifdef CONFIG_MB_COMM_MODE_TCP
    tcp_master_init();
    #else
    master_init();
    #endif
#endif

#ifdef CONFIG_PLC_FX
    uart_init();
#endif
    InitSensorGpio();
    FXPLC_InitBuffer();

    xQueue1 = xQueueCreate(32, sizeof(char *));
    xEventGroup1 = xEventGroupCreate();
    xEventGroup2 = xEventGroupCreate();

#if CONFIG_PLC_FX
    xTaskCreate(rx_task, "uart_rx_task", 1024*8, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*8, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(tx1_task, "uart_tx1_task", 1024*8, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(uart_event_task, "uart_event_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL);
#endif
    xTaskCreate(tcp_client_task, "tcp_client", 1024*8, NULL, 5, NULL);
    xTaskCreate(tcp_client1_task, "tcp_client1", 1024*8, NULL, 5, NULL);
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);
    xTaskCreate(tcp_server1_task, "tcp_server1", 4096, (void*)AF_INET, 5, NULL);
    xTaskCreate(send_data_task, "send_data_task", 4096, (void*)AF_INET, 5, NULL);
}