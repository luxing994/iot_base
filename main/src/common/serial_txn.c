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

static const char *TAG = "SERIAL_TXN";
static SemaphoreHandle_t s_lock;

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
                return 0;
            }
            if (ret == SERIAL_PARSE_SKIP) {
                UART_CommitBuffer();
                continue;       // 后面可能已经有数据，立即再解析一次
            }
            if (ret == SERIAL_PARSE_ERROR) {
                UART_FlushBuffer();
                return -1;
            }
            UART_RollbackBuffer();
        }

        if (xTaskGetTickCount() - start >= timeout) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(SERIAL_TXN_POLL_MS));
    }

    ESP_LOGW(TAG, "response timeout (%d bytes buffered)", (int)UART_BufferedBytes());
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
