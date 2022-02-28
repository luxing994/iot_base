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
#include "uart_task.h"

QueueHandle_t xQueue1;
EventGroupHandle_t xEventGroup1;

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

    uart_init();

    xQueue1 = xQueueCreate(32, sizeof(char *));
    xEventGroup1 = xEventGroupCreate();
    xTaskCreate(rx_task, "uart_rx_task", 1024*8, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
}