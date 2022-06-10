#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sensor.h"

#define SWITCH_SENSOR_GPIO   (GPIO_NUM_38)

uint32_t g_count = 0;

int GetSwitchCount(void)
{
    return g_count;
}

void switch_sensor_gpio_isr_callback(void *arg)
{
    g_count++;
}

void InitSwitchSensor(void)
{
    gpio_reset_pin(SWITCH_SENSOR_GPIO);
    gpio_set_direction(SWITCH_SENSOR_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(SWITCH_SENSOR_GPIO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SWITCH_SENSOR_GPIO, switch_sensor_gpio_isr_callback, NULL);
}

int GetSwitchSensorLevel(void)
{
    return gpio_get_level(SWITCH_SENSOR_GPIO);
}