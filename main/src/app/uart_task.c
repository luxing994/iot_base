/*
 * UART1 驱动与串口相关任务
 *
 *   uart_event_task : UART 驱动事件 -> 把收到的字节写入 uart2Buffer
 *   cmd_task        : 执行上位机命令（写 PLC / 测试仪参数），上报开关量中断事件
 *
 * 串口请求与应答的配对由 serial_txn.c 的事务完成，轮询见 plc_poll.c
 */
#include <stdio.h>
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
#include "serial_txn.h"
#include "monitor.h"
#include "fx_plc_protocol.h"
#include "hl_plc_protocol.h"
#include "t_tester_protocol.h"
#include "ainuo_tester_ascii_protocol.h"

#define PATTERN_CHR_NUM    (3)
#define RX_BUF_SIZE        (UART_BUFF_SIZE * 2)
#define TXD_PIN            (GPIO_NUM_17)
#define RXD_PIN            (GPIO_NUM_18)
#define RTS_PIN            (GPIO_NUM_8)



/* ------------------------------------------------------------------ UART */

#ifdef APP_UART_ENABLED

/*
 * UART1 同一时间只能服务一种串口协议（uart_task.h 中有编译期检查），按该协议设置串口参数。
 */
#if defined(CONFIG_PLC_FX_CONNECT_RS232) || defined(CONFIG_PLC_FX_CONNECT_RS485)
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
#elif defined(CONFIG_PLC_PPI_CONNECT_RS485)
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
            ESP_LOGD(UART_EVENT_TASK_TAG, "[DATA EVT]: %d bytes", event.size);   // 每次收到数据都打 INFO 会刷屏
            Mon_Capture(MON_CH_SERIAL, MON_RX, dtmp, event.size);
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

const char *UART_InterfaceName(void)
{
#if defined(CONFIG_APP_USE_RS485)
    return "RS485";
#elif defined(CONFIG_APP_USE_RS232)
    return "RS232";
#else
    return NULL;
#endif
}

const char *UART_InterfaceText(void)
{
#ifdef APP_UART_ENABLED
    static char text[32];

    if (text[0] == '\0') {
        snprintf(text, sizeof(text), "%s %d %d%c%s", UART_InterfaceName() ? UART_InterfaceName() : "UART",
                 APP_UART_BAUD, 5 + (int)APP_UART_DATA,
                 APP_UART_PARITY == UART_PARITY_EVEN ? 'E' : APP_UART_PARITY == UART_PARITY_ODD ? 'O' : 'N',
                 APP_UART_STOP == UART_STOP_BITS_2 ? "2" : APP_UART_STOP == UART_STOP_BITS_1_5 ? "1.5" : "1");
    }
    return text;
#else
    return UART_InterfaceName();     // Modbus RTU 由 esp-modbus 管理，参数见 master.c
#endif
}

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

/*
 * 上位机 single 模式命令：写冷媒灌注机充注量，写完立即回读同一寄存器。
 * 写入成功且回读值与下发值一致时返回 0，否则返回 -1；结果由 host_cmd 回复上位机 true / false。
 */
static int ApplyChargeSetting(void)
{
    SNSetCaclReFillingMachine charge;
    PlcValue v;

    HostCmd_GetChargeSetting(&charge);
    (void)v;

#if defined(CONFIG_PLC_FX_SN_CACLREFILMAC)
    const uint16_t address = 16;                        // D16：实际设定灌注量（FR010）

    if (FX_LinkWriteFloat(address, charge.setchargeamount) != PLC_OK ||
        FX_LinkRead(FX_DEV_D, address, FX_FMT_FLOAT, &v) != PLC_OK) {
        return -1;
    }
#elif defined(CONFIG_PLC_HOSTLINK_SN_CACLREFILMAC)
    uint16_t address;
    int frnum;

    if (strcmp(charge.select, "A") == 0) {
        address = 8150;     // A充注量设定值 -> FR026
        frnum = 26;
    } else if (strcmp(charge.select, "B") == 0) {
        address = 8550;     // B充注量设定值 -> FR027
        frnum = 27;
    } else {
        ESP_LOGE("CHARGE", "unknown parameterType \"%s\", expect A or B", charge.select);
        return -1;
    }
    if (HL_WriteDMFloat(address, charge.setchargeamount) != PLC_OK ||
        HL_ReadDM(address, HL_FMT_FLOAT, &v) != PLC_OK) {
        return -1;
    }
    PlcReport_HostLink(frnum, &v, false);
#else
    ESP_LOGE("CHARGE", "charge setting not supported by current device type");
    return -1;
#endif

#if defined(CONFIG_PLC_FX_SN_CACLREFILMAC) || defined(CONFIG_PLC_HOSTLINK_SN_CACLREFILMAC)
    // PLC 中保存的就是下发的 32 位浮点，回读值应逐位相等
    if (v.f != charge.setchargeamount) {
        ESP_LOGE("CHARGE", "read back %.3f, expect %.3f", v.f, charge.setchargeamount);
        return -1;
    }
    ESP_LOGI("CHARGE", "charge amount %.3f applied", v.f);
    return 0;
#endif
}

// 上位机 group 模式命令：下发安规测试仪组参数（应答只用于确认，不上报）
static void ApplyTesterGroup(void)
{
#ifdef CONFIG_TESTER_76T
    TTester_SetGroupPara((uint16_t)HostCmd_GetTesterGroup());
#endif

#ifdef CONFIG_TESTER_AINUO
    AINUO_SetGroupPara((uint16_t)HostCmd_GetTesterGroup());
#endif
}

void cmd_task(void *arg)
{
    static const char *TAG = "CMD_TASK";
    EventBits_t uxBits;

    esp_log_level_set(TAG, ESP_LOG_ERROR);
    while (1) {
        // 返回时所有等待位都会被清除，因此每一位都要独立处理（原实现用 else if，同时到达的事件会丢失）
        uxBits = xEventGroupWaitBits(xEventGroup1, EVT_CHARGE_REQUEST | BIT_15 | BIT_16 | BIT_17,
                                     pdTRUE, pdFALSE, portMAX_DELAY);
        if (uxBits & EVT_CHARGE_REQUEST) {
            // 结果通知 host_cmd，由它回复上位机 true / false
            xEventGroupSetBits(xEventGroup1, (ApplyChargeSetting() == 0) ? EVT_CHARGE_OK : EVT_CHARGE_FAIL);
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
