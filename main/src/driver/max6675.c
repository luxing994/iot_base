#include "sdkconfig.h"

#ifdef CONFIG_SENSOR_MAX6675

/*
 * MAX6675 热电偶温度采集（GPIO 模拟 SPI，只读）
 *
 * 每 250ms 读一次（芯片一次转换约 220ms），卡尔曼滤波后上报 FR000。
 * 热电偶断开时不上报，恢复后继续从上次的估计值开始滤波。
 */

#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "event_log.h"
#include "esp_rom_sys.h"
#include "iot_common.h"
#include "iot_report.h"
#include "max6675.h"

static const char *TAG = "MAX6675";

#define MAX6675_CS_GPIO     CONFIG_MAX6675_CS_GPIO
#define MAX6675_SCK_GPIO    CONFIG_MAX6675_SCK_GPIO
#define MAX6675_SO_GPIO     CONFIG_MAX6675_SO_GPIO

#define MAX6675_PERIOD_MS   250
#define MAX6675_OPEN_BIT    0x0004      // D2：热电偶开路

void MAX6675_Init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MAX6675_CS_GPIO) | (1ULL << MAX6675_SCK_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << MAX6675_SO_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;    // 未接芯片时读到全 1，会被判为开路
    gpio_config(&io_conf);

    gpio_set_level(MAX6675_CS_GPIO, 1);
    gpio_set_level(MAX6675_SCK_GPIO, 0);
}

/*
 * 读一次温度（摄氏度），热电偶开路返回 -1.0
 * 时钟由主机产生，读取过程被任务切换打断只会拉长时钟，不影响数据，因此不关中断
 */
float MAX6675_ReadTemp(void)
{
    uint16_t raw = 0;

    gpio_set_level(MAX6675_CS_GPIO, 0);
    esp_rom_delay_us(1);    // tCSS：CS 下降沿到第一个 SCK 至少 100ns

    for (int i = 15; i >= 0; i--) {
        raw |= (uint16_t)(gpio_get_level(MAX6675_SO_GPIO) << i);
        gpio_set_level(MAX6675_SCK_GPIO, 1);
        esp_rom_delay_us(1);    // tDO：SCK 下降沿到 SO 数据有效最多 100ns
        gpio_set_level(MAX6675_SCK_GPIO, 0);
        esp_rom_delay_us(1);
    }

    gpio_set_level(MAX6675_CS_GPIO, 1);

    if (raw & MAX6675_OPEN_BIT) {
        return -1.0f;
    }
    return (float)(raw >> 3) * 0.25f;   // D14~D3，0.25℃ / LSB
}

/*
 * 卡尔曼滤波参数：
 *   Q：过程噪声，温度变化较快时调大（如 0.5），变化缓慢时调小（如 0.01）
 *   R：测量噪声，传感器噪声越大调大（如 8.0），越小调小（如 1.0）
 */
#define KALMAN_Q  0.1f
#define KALMAN_R  4.0f

// 输入有效读数，返回滤波后的温度；第一个读数直接作为初始估计
static float TempFilter(float measured)
{
    static float x = -1.0f;     // 状态估计值，< 0 表示尚未初始化
    static float p = 1.0f;      // 估计误差协方差

    if (x < 0.0f) {
        x = measured;
        return x;
    }

    p = p + KALMAN_Q;                   // 预测：不确定度增加
    float k = p / (p + KALMAN_R);       // 卡尔曼增益
    x = x + k * (measured - x);         // 更新：融合测量值
    p = (1.0f - k) * p;

    return x;
}

void max6675_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool open = false;
    char value[48];         // 足够容纳任意 float 的 "%.2f" 输出
    // 温度报文使用固定的设备标识，不跟随上位机命令
    const IotValueMsg msg = {
        .devNumber = DEVID,
        .devId = DEVID,
        .devName = DEVNAME,
        .devTypeId = TEMPDEVTYPEID,
        .devTypeName = MDPLCDEVTYPEID,
        .orderId = INITORDERID,
        .orderName = "temperature",
        .value = value,
    };

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MAX6675_PERIOD_MS));

        float measured = MAX6675_ReadTemp();
        if (measured < 0.0f) {
            // 原实现开路时继续上报上一次的温度，上位机无法发现故障；改为不上报，只在状态变化时打印
            if (!open) {
                ESP_LOGW(TAG, "thermocouple open circuit, stop reporting");
                EvLog_Add(EV_WARN, "热电偶开路（断线或未接），停止上报温度");
                open = true;
            }
            continue;
        }
        if (open) {
            ESP_LOGI(TAG, "thermocouple reconnected");
            EvLog_Add(EV_INFO, "热电偶已恢复");
            open = false;
        }

        float temp = TempFilter(measured);
        ESP_LOGD(TAG, "temperature: %.2f C", temp);
        snprintf(value, sizeof(value), "%.2f", temp);
        if (IotReport_SendValueMsg(&msg) != 0) {
            ESP_LOGW(TAG, "report failed, drop data");
        }
    }
}

#endif /* CONFIG_SENSOR_MAX6675 */
