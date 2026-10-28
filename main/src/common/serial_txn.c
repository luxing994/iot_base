#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "iot_common.h"
#include "serial_txn.h"

#define SERIAL_TXN_UART      UART_NUM_1
#define SERIAL_TXN_POLL_MS   10     // 应答未收全时的重试间隔
#define SERIAL_TXN_GAP_MS    10     // 两次事务之间的最小间隔（RS485 收发切换、设备处理时间）

/*
 * 设备离线判断：连续 SERIAL_OFFLINE_TIMEOUTS 次无应答记为离线。
 * 日志只在状态变化时打印（离线 / 恢复），离线期间每 SERIAL_REMIND_MS 提醒一次，
 * 单次超时只打 DEBUG，避免设备没接时每次轮询都刷屏。
 */
#define SERIAL_OFFLINE_TIMEOUTS  3
#define SERIAL_REMIND_MS         60000

static const char *TAG = "SERIAL";
static SemaphoreHandle_t s_lock;

/* 以下状态只在持有 s_lock 的事务内访问 */
static int s_timeouts;              // 连续超时次数
static bool s_offline;
static TickType_t s_offlineSince;
static TickType_t s_lastRemind;

static void OnResponse(void)
{
    if (s_offline) {
        ESP_LOGI(TAG, "device responding again (was offline %u s)",
                 (unsigned)((xTaskGetTickCount() - s_offlineSince) * portTICK_PERIOD_MS / 1000));
    }
    s_offline = false;
    s_timeouts = 0;
}

static void OnTimeout(int buffered)
{
    const TickType_t now = xTaskGetTickCount();

    ESP_LOGD(TAG, "response timeout (%d bytes buffered)", buffered);
    if (!s_offline) {
        if (++s_timeouts < SERIAL_OFFLINE_TIMEOUTS) {
            return;
        }
        s_offline = true;
        s_offlineSince = now;
        s_lastRemind = now;
        // 收到数据却解析不出应答，多半是波特率 / 校验位 / 站号不对；完全没数据则检查接线和设备电源
        ESP_LOGW(TAG, "device not responding, keep polling (%s)",
                 buffered > 0 ? "garbled data: check baud rate / parity / station" : "no data: check wiring / power");
        return;
    }
    if (now - s_lastRemind >= pdMS_TO_TICKS(SERIAL_REMIND_MS)) {
        s_lastRemind = now;
        ESP_LOGW(TAG, "device still not responding (%u min)",
                 (unsigned)((now - s_offlineSince) * portTICK_PERIOD_MS / 60000));
    }
}

void SerialTxn_Init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
    }
}

void SerialTxn_Begin(void)
{
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    // 丢弃上一次事务之后到达的残留数据（迟到的应答、干扰等）
    UART_FlushBuffer();
}

void SerialTxn_Send(const void *data, size_t len)
{
    uart_write_bytes(SERIAL_TXN_UART, data, len);
}

int SerialTxn_Wait(SerialParseFn parse, void *ctx, int timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while (1) {
        if (UART_BufferedBytes() > 0) {
            int ret = parse(ctx);

            if (ret == SERIAL_PARSE_OK) {
                UART_CommitBuffer();
                OnResponse();
                return 0;
            }
            if (ret == SERIAL_PARSE_SKIP) {
                UART_CommitBuffer();
                continue;       // 后面可能已经有数据，立即再解析一次
            }
            if (ret == SERIAL_PARSE_ERROR) {
                UART_FlushBuffer();
                OnResponse();       // 应答内容有误（错误码等），但设备在线
                return -1;
            }
            UART_RollbackBuffer();
        }

        if (xTaskGetTickCount() - start >= timeout) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(SERIAL_TXN_POLL_MS));
    }

    OnTimeout((int)UART_BufferedBytes());
    UART_FlushBuffer();
    return -1;
}

void SerialTxn_End(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
    vTaskDelay(pdMS_TO_TICKS(SERIAL_TXN_GAP_MS));
}

int SerialTxn_Request(const void *req, size_t len, SerialParseFn parse, void *ctx, int timeout_ms)
{
    int ret;

    SerialTxn_Begin();
    SerialTxn_Send(req, len);
    ret = SerialTxn_Wait(parse, ctx, timeout_ms);
    SerialTxn_End();
    return ret;
}

/* ------------------------------------------------------------ 解析读接口 */

int SerialRx_Read(void *buf, size_t len)
{
    return UART_ReadBufferBytes((uint8_t *)buf, len) == 0 ? 0 : -1;
}

int SerialRx_Byte(uint8_t *b)
{
    return SerialRx_Read(b, 1);
}

int SerialRx_SeekAny(uint8_t a, uint8_t b)
{
    uint8_t c;

    do {
        if (SerialRx_Byte(&c) != 0) {
            return -1;
        }
    } while (c != a && c != b);
    return c;
}
