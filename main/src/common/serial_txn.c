#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "iot_common.h"
#include "serial_txn.h"
#include "ppi_plc_protocol.h"

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

int SerialTxn_Wait(SerialParseFn parse, int timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while (1) {
        if (UART_BufferedBytes() > 0) {
            // 解析函数会把数据写入解码缓冲，先清空，避免上一次未完成的尝试留下残片
            FXPLC_FlushBuffer();
            PPIPLC_FlushBuffer();

            if (parse() == 0) {
                UART_CommitBuffer();
                return 0;
            }
            UART_RollbackBuffer();

#if defined(CONFIG_PLC_PPI_CONNECT_RS485)
            // PPI：收到短应答 E5 后发确认帧，PLC 才会返回数据
            if (PPI_TakeConfirmRequest()) {
                UART_FlushBuffer();
                PPISendConfirm();
            }
#endif
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
