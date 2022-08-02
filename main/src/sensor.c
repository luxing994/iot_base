#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "iot_common.h"
#include "sensor.h"

#define SWITCH_SENSOR_GPIO   (GPIO_NUM_38)
#define SWITCH_GPIO          (GPIO_NUM_37)

uint32_t g_count = 0;

int GetSwitchCount(void)
{
    return g_count;
}

void switch_sensor_gpio_isr_callback(void *arg)
{
    g_count++;
    xEventGroupSetBitsFromISR(xEventGroup1, BIT_15, NULL);
}

void switch_gpio_isr_callback(void *arg)
{
    xEventGroupSetBitsFromISR(xEventGroup1, BIT_16, NULL);
}

void InitSensorGpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SWITCH_SENSOR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << SWITCH_GPIO),
    io_conf.mode = GPIO_MODE_INPUT,
    io_conf.pull_up_en = 1,
    io_conf.pull_down_en = 0,
    io_conf.intr_type = GPIO_INTR_ANYEDGE,
    
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SWITCH_SENSOR_GPIO, switch_sensor_gpio_isr_callback, NULL);
    gpio_isr_handler_add(SWITCH_GPIO, switch_gpio_isr_callback, NULL);
}

void InitSwitchSensor(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SWITCH_SENSOR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SWITCH_SENSOR_GPIO, switch_sensor_gpio_isr_callback, NULL);
}

void InitSwitch(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SWITCH_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SWITCH_GPIO, switch_gpio_isr_callback, NULL);
}

int GetSwitchSensorLevel(void)
{
    return gpio_get_level(SWITCH_SENSOR_GPIO);
}

int GetSwitchLevel(void)
{
    return gpio_get_level(SWITCH_GPIO);
}