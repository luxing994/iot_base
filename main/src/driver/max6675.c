#include "sdkconfig.h"

#ifdef CONFIG_SENSOR_MAX6675

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "iot_common.h"
#include "iot_time.h"
#include "tcp_client.h"
#include "max6675.h"

#define TAG "MAX6675"

#define MAX6675_CS_GPIO    CONFIG_MAX6675_CS_GPIO
#define MAX6675_SCK_GPIO   CONFIG_MAX6675_SCK_GPIO
#define MAX6675_SO_GPIO    CONFIG_MAX6675_SO_GPIO

void MAX6675_Init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MAX6675_CS_GPIO) | (1ULL << MAX6675_SCK_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << MAX6675_SO_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(MAX6675_CS_GPIO, 1);
    gpio_set_level(MAX6675_SCK_GPIO, 0);
}

static portMUX_TYPE s_spi_mux = portMUX_INITIALIZER_UNLOCKED;

/* Returns temperature in Celsius, or -1.0 if thermocouple is open */
float MAX6675_ReadTemp(void)
{
    uint16_t raw = 0;

    // taskENTER_CRITICAL(&s_spi_mux);

    gpio_set_level(MAX6675_CS_GPIO, 0);
    esp_rom_delay_us(1);  /* tCSS: CS↓ 到首个 SCK 最小 100ns */

    for (int i = 15; i >= 0; i--) {
        raw |= ((uint16_t)gpio_get_level(MAX6675_SO_GPIO) << i);
        gpio_set_level(MAX6675_SCK_GPIO, 1);
        esp_rom_delay_us(1);  /* tDO: SCK↓ 到 SO 数据有效最大 100ns */
        gpio_set_level(MAX6675_SCK_GPIO, 0);
        esp_rom_delay_us(1);
    }

    gpio_set_level(MAX6675_CS_GPIO, 1);

    // taskEXIT_CRITICAL(&s_spi_mux);

    if (raw & 0x04) {
        ESP_LOGW(TAG, "thermocouple open circuit");
        return -1.0f;
    }

    return (float)(raw >> 3) * 0.25f;
}

/* 卡尔曼滤波参数：
 *   Q：过程噪声，温度变化较快时调大（如 0.5），变化缓慢时调小（如 0.01）
 *   R：测量噪声，传感器噪声越大调大（如 8.0），越小调小（如 1.0） */
#define KALMAN_Q  0.1f
#define KALMAN_R  4.0f

/* 返回卡尔曼滤波后的温度；读取失败时保持上次估计值不变，
 * 尚无任何有效读数时返回 -1.0 */
static float TempFilter(float new_val)
{
    static float x = -1.0f;  /* 状态估计值 */
    static float p =  1.0f;  /* 估计误差协方差 */

    if (new_val < 0.0f) {
        ESP_LOGW(TAG, "invalid reading, hold last value: %.2f", x);
        return x;
    }

    if (x < 0.0f) {
        x = new_val;
        return x;
    }

    p = p + KALMAN_Q;                  /* 预测：不确定度增加 */
    float k = p / (p + KALMAN_R);      /* 卡尔曼增益 */
    x = x + k * (new_val - x);         /* 更新：融合测量值 */
    p = (1.0f - k) * p;

    return x;
}

static char max6675_txbuf[512];

void max6675_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(250);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        float temp = TempFilter(MAX6675_ReadTemp());
        // float temp = MAX6675_ReadTemp();

        if (temp < 0.0f) {
            continue;
        }

        ESP_LOGI(TAG, "temperature: %.2f C", temp);

        (void)snprintf(max6675_txbuf, sizeof(max6675_txbuf),
            "{\n    \"devNumber\":\"%s\",\n    \"devId\":\"%s\",\n    \"devName\":\"%s\",\n"
            "    \"devTypeId\":\"%s\",\n    \"devTypeName\":\"%s\",\n    \"devIP\":\"%s\",\n"
            "    \"orderId\":\"%s\",\n    \"orderName\":\"%s\",\n    \"timeStampNeed\":\"%s\",\n    \"timeStamp\":\"%lld\",\n"
            "    \"valueUnit\":\"NULL\",\n    \"value\":\"%.2f\",\n    \"expand\":\"NULL\",\n    \"isAnswer\":\"no\"\n};;**##",
            DEVID, DEVID, DEVNAME, TEMPDEVTYPEID, MDPLCDEVTYPEID, GetStaIp(),
            INITORDERID, "temperature", DEVTIMEMODE, GetMilliTimeNow(), temp);

        if (IOT_PostMessage(max6675_txbuf) != 0) {
            ESP_LOGW(TAG, "queue full, drop data");
        }
    }
}

#endif /* CONFIG_SENSOR_MAX6675 */
