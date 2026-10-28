/*
 * 开关量输入
 *
 *   GPIO38  计数传感器：上升沿计数，置 xEventGroup1 的 BIT_16，cmd_task 上报累计次数
 *   GPIO37  开关状态：  任意边沿，置 xEventGroup1 的 BIT_17，cmd_task 上报当前电平
 *
 * 现场接的是接近开关，输出无触点抖动，因此不做软件消抖；
 * 若改接机械开关，需要在中断里按时间间隔过滤抖动。
 */
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "iot_common.h"
#include "sensor.h"

#define SWITCH_SENSOR_GPIO   (GPIO_NUM_38)
#define SWITCH_GPIO          (GPIO_NUM_37)

static volatile uint32_t s_count;

int GetSwitchCount(void)
{
    return (int)s_count;
}

static void IRAM_ATTR SwitchSensorIsr(void *arg)
{
    BaseType_t woken = pdFALSE;

    s_count++;
    xEventGroupSetBitsFromISR(xEventGroup1, BIT_16, &woken);
    portYIELD_FROM_ISR(woken);
}

static void IRAM_ATTR SwitchIsr(void *arg)
{
    BaseType_t woken = pdFALSE;

    xEventGroupSetBitsFromISR(xEventGroup1, BIT_17, &woken);
    portYIELD_FROM_ISR(woken);
}

void InitSensorGpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SWITCH_SENSOR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << SWITCH_GPIO);
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_conf);

    // GPIO ISR 服务由 app_main 统一安装（W5500 中断也要用），这里只挂回调
    gpio_isr_handler_add(SWITCH_SENSOR_GPIO, SwitchSensorIsr, NULL);
    gpio_isr_handler_add(SWITCH_GPIO, SwitchIsr, NULL);
}

int GetSwitchSensorLevel(void)
{
    return gpio_get_level(SWITCH_SENSOR_GPIO);
}

int GetSwitchLevel(void)
{
    return gpio_get_level(SWITCH_GPIO);
}
