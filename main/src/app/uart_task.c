/*
 * UART1 驱动与串口相关任务
 *
 *   uart_event_task : UART 驱动事件 -> 把收到的字节写入 uart2Buffer
 *   rx_task         : 周期调用协议层解析应答，解析成功后交给 plc_report 解码上报
 *   rx_modbus_task  : Modbus TCP 读到数据后解码上报
 *   rx_ppi_task     : S7 网口读到数据后解码上报
 *   tx_task         : 处理上位机命令 / 开关量中断事件
 */
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "iot_common.h"
#include "iot_report.h"
#include "plc_report.h"
#include "uart_task.h"
#include "host_cmd.h"
#include "sensor.h"
#include "fx_plc_protocol.h"
#include "lx_modbus_tcp_master.h"
#include "lx_s7comm.h"
#include "hl_plc_protocol.h"
#include "ls_plc_load_protocol.h"
#include "ppi_plc_protocol.h"
#include "t_tester_protocol.h"
#include "ainuo_tester_ascii_protocol.h"

#define PATTERN_CHR_NUM    (3)
#define RX_BUF_SIZE        (UART_BUFF_SIZE * 2)
#define TXD_PIN            (GPIO_NUM_17)
#define RXD_PIN            (GPIO_NUM_18)
#define RTS_PIN            (GPIO_NUM_8)
#define TESTER_JSON_SIZE   2048



/* ------------------------------------------------------------------ UART */

#ifdef APP_UART_ENABLED

/*
 * UART1 同一时间只能服务一种串口协议，按优先级取第一个启用的协议的串口参数。
 * （原实现每个协议各定义一个 uart_config，同时启用两个协议时会编译失败）
 */
#if defined(CONFIG_PLC_FX)
#define APP_UART_BAUD    9600
#define APP_UART_DATA    UART_DATA_7_BITS
#define APP_UART_PARITY  UART_PARITY_EVEN
#define APP_UART_STOP    UART_STOP_BITS_1
#elif defined(CONFIG_PLC_LS_LOAD)
#define APP_UART_BAUD    9600
#define APP_UART_DATA    UART_DATA_8_BITS
#define APP_UART_PARITY  UART_PARITY_DISABLE
#define APP_UART_STOP    UART_STOP_BITS_1
#elif defined(CONFIG_PLC_HOSTLINK)
// 标准 HostLink 为 9600 7E2，现场设备改成了 115200 8N1
#define APP_UART_BAUD    115200
#define APP_UART_DATA    UART_DATA_8_BITS
#define APP_UART_PARITY  UART_PARITY_DISABLE
#define APP_UART_STOP    UART_STOP_BITS_1
#elif defined(CONFIG_TESTER_76T)
#define APP_UART_BAUD    9600
#define APP_UART_DATA    UART_DATA_8_BITS
#define APP_UART_PARITY  UART_PARITY_DISABLE
#define APP_UART_STOP    UART_STOP_BITS_1
#elif defined(CONFIG_TESTER_AINUO)
#define APP_UART_BAUD    19200
#define APP_UART_DATA    UART_DATA_8_BITS
#define APP_UART_PARITY  UART_PARITY_DISABLE
#define APP_UART_STOP    UART_STOP_BITS_1
#elif defined(CONFIG_PLC_PPI)
#define APP_UART_BAUD    9600
#define APP_UART_DATA    UART_DATA_8_BITS
#define APP_UART_PARITY  UART_PARITY_EVEN
#define APP_UART_STOP    UART_STOP_BITS_1
#endif

static QueueHandle_t uart1_queue;

void uart_init(void)
{
    static const char *UART_INIT_TAG = "UART_INIT";
    const uart_config_t uart_config = {
        .baud_rate = APP_UART_BAUD,
        .data_bits = APP_UART_DATA,
        .parity = APP_UART_PARITY,
        .stop_bits = APP_UART_STOP,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_log_level_set(UART_INIT_TAG, ESP_LOG_ERROR);
    uart_driver_install(UART_NUM_1, UART_BUFF_SIZE * 2, UART_BUFF_SIZE * 2, 20, &uart1_queue, 0);
    uart_param_config(UART_NUM_1, &uart_config);
#ifdef CONFIG_APP_USE_RS232
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#endif

#ifdef CONFIG_APP_USE_RS485
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, RTS_PIN, UART_PIN_NO_CHANGE);
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
#endif
    if (UART_InitBuffer() != 0) {
        ESP_LOGE(UART_INIT_TAG, "uart buffer init failed\n");
    }
}

void uart_event_task(void *pvParameters)
{
    static const char *UART_EVENT_TASK_TAG = "UART_EVENT_TASK";
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(RX_BUF_SIZE + 1);   // 多 1 字节保证日志打印时以 0 结尾

    if (dtmp == NULL) {
        ESP_LOGE(UART_EVENT_TASK_TAG, "no memory for uart rx buffer");
        vTaskDelete(NULL);
        return;
    }
    esp_log_level_set(UART_EVENT_TASK_TAG, ESP_LOG_ERROR);

    for (;;) {
        if (xQueueReceive(uart1_queue, (void *)&event, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        bzero(dtmp, RX_BUF_SIZE + 1);
        switch (event.type) {
        case UART_DATA:
            uart_read_bytes(UART_NUM_1, dtmp, event.size, portMAX_DELAY);
            ESP_LOGI(UART_EVENT_TASK_TAG, "[DATA EVT]: %d bytes", event.size);
            if (UART_WriteBufferBytes(dtmp, event.size) != 0) {
                ESP_LOGE(UART_EVENT_TASK_TAG, "uart buffer full");
            }
            break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            // 驱动缓冲区溢出，丢弃已缓存数据重新开始接收
            ESP_LOGI(UART_EVENT_TASK_TAG, "uart overflow: %d", event.type);
            uart_flush_input(UART_NUM_1);
            xQueueReset(uart1_queue);
            break;
        case UART_PATTERN_DET: {
            uart_get_buffered_data_len(UART_NUM_1, &buffered_size);
            int pos = uart_pattern_pop_pos(UART_NUM_1);
            if (pos == -1) {
                uart_flush_input(UART_NUM_1);
            } else {
                uint8_t pat[PATTERN_CHR_NUM + 1] = {0};
                uart_read_bytes(UART_NUM_1, dtmp, pos, pdMS_TO_TICKS(100));
                uart_read_bytes(UART_NUM_1, pat, PATTERN_CHR_NUM, pdMS_TO_TICKS(100));
            }
            break;
        }
        default:
            ESP_LOGI(UART_EVENT_TASK_TAG, "uart event type: %d", event.type);
            break;
        }
    }
}

#endif /* APP_UART_ENABLED */

/* ------------------------------------------------------------- 接收任务 */

#ifdef APP_RX_TASK_ENABLED

#if defined(CONFIG_TESTER_76T) || defined(CONFIG_TESTER_AINUO)
static char s_testerJson[TESTER_JSON_SIZE];
#endif

// 解析一次各协议的应答，成功则解码上报
static void RxPollOnce(void)
{
    int ret;

#if defined(CONFIG_PLC_FX)
#if defined(CONFIG_PLC_FX_CONNECT_RS232)
    ret = GetDataFromFxPlc();
#elif defined(CONFIG_PLC_FX_CONNECT_RS485)
    ret = GetSerialDataFromFxPlc();
#else
    ret = GetNetDataFromFxPlc();
#endif
    if (ret == 0) {
#if defined(CONFIG_PLC_FX_CONNECT_RS232) || defined(CONFIG_PLC_FX_CONNECT_RS485)
        SendAckToPlc();
#endif
        PlcReport_Fx();
    }
#if defined(CONFIG_PLC_FX_CONNECT_RS232) || defined(CONFIG_PLC_FX_CONNECT_RS485)
    else {
        // 注意：缓冲区为空时也会走到这里，与原实现一致，每个周期都会回 NAK
        SendNackToPlc();
    }
#endif
#endif

#if defined(CONFIG_PLC_HOSTLINK_CONNECT_RS232) || defined(CONFIG_PLC_HOSTLINK_CONNECT_RS485)
    if (GetSerialWordDataFromHlPlc() == 0) {
        PlcReport_HostLink();
    }
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_RS485)
    if (GetSerialWordDataFromPpiPlc() == 0) {
        PlcReport_Ppi();
    }
#endif

#if defined(CONFIG_PLC_LS_LOAD_CONNECT_RS232)
    if (DBSGetData() == 0) {
        PlcReport_Ls();
    }
#endif

#if defined(CONFIG_TESTER_76T)
    if (TTesterResolve() == 0) {
        TTesterGetJsonData(s_testerJson);
        IOT_PostMessage(s_testerJson);
    }
#endif

#if defined(CONFIG_TESTER_AINUO)
    if (AINUO_TTesterResolve() == 0) {
        AINUO_TTesterGetJsonData(s_testerJson);
        IOT_PostMessage(s_testerJson);
    }
#endif

    (void)ret;
}

void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
#ifdef CONFIG_TESTER_AINUO
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
#else
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
#endif
    TickType_t xLastWakeTime = xTaskGetTickCount();

    esp_log_level_set(RX_TASK_TAG, ESP_LOG_ERROR);
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        RxPollOnce();
    }
}

#endif /* APP_RX_TASK_ENABLED */

#ifdef CONFIG_PLC_MODBUS_CONNECT_NETWORK
void rx_modbus_task(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
        if (GetNetDataFromModbusPlc() == 0) {
            PlcReport_ModbusNet();
        }
    }
}
#endif

#ifdef CONFIG_PLC_PPI_CONNECT_NETWORK
void rx_ppi_task(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
        if (GetNetDataFromS7PlcNet() == 0) {
            PlcReport_Ppi();
        }
    }
}
#endif

/* ------------------------------------------------------------- 命令任务 */

// 开关量计数 / 状态：沿用上位机最近一次命令中的 devTypeId 和 orderId
static void ReportSwitch(int value)
{
    HostCmdIdentity id;
    char str[16];
    IotValueMsg msg = {
        .devTypeId = id.devTypeId,
        .devTypeName = DEVTYPENAME,
        .orderId = id.orderId,
        .value = str,
    };

    HostCmd_GetIdentity(&id);
    snprintf(str, sizeof(str), "%d", value);
    IotReport_SendValueMsg(&msg);
}

// 上位机 single 模式命令：写冷媒灌注机充注量
static void ApplyChargeSetting(void)
{
    SNSetCaclReFillingMachine charge;

    HostCmd_GetChargeSetting(&charge);
#ifdef CONFIG_PLC_FX_SN_CACLREFILMAC
    SerialWriteSingleFloatDataRegister(0, 255, 10, 16, charge.setchargeamount);
#endif

#ifdef CONFIG_PLC_HOSTLINK_SN_CACLREFILMAC
    if (strcmp(charge.select, "A") == 0) {
        HLWriteRealDataRegister(8150, charge.setchargeamount);   // A充注量设定值
        HLReadFloatDataRegister(8150, 26);                       // A充注设定值
    } else if (strcmp(charge.select, "B") == 0) {
        HLWriteRealDataRegister(8550, charge.setchargeamount);   // B充注量设定值
        HLReadFloatDataRegister(8550, 27);                       // B充注设定值
    }
#endif
    (void)charge;
}

// 上位机 group 模式命令：下发安规测试仪组参数
static void ApplyTesterGroup(void)
{
#ifdef CONFIG_TESTER_76T
    TTesterSetGroupPara(HostCmd_GetTesterGroup());
#endif

#ifdef CONFIG_TESTER_AINUO
    AINUO_TTesterSetGroupPara(HostCmd_GetTesterGroup());
#endif
}

void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    EventBits_t uxBits;

    esp_log_level_set(TX_TASK_TAG, ESP_LOG_ERROR);
    while (1) {
        // 返回时所有等待位都会被清除，因此每一位都要独立处理（原实现用 else if，同时到达的事件会丢失）
        uxBits = xEventGroupWaitBits(xEventGroup1, BIT_14 | BIT_15 | BIT_16 | BIT_17,
                                     pdTRUE, pdFALSE, pdMS_TO_TICKS(100));
        if (uxBits & BIT_14) {
            ApplyChargeSetting();
        }
        if (uxBits & BIT_15) {
            ApplyTesterGroup();
        }
        if (uxBits & BIT_16) {
            ReportSwitch(GetSwitchCount());
        }
        if (uxBits & BIT_17) {
            ReportSwitch(GetSwitchLevel());
        }
    }
}
