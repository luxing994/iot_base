/*
 * SPDX-FileCopyrightText: 2016-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Modbus RTU 主站（esp-modbus，UART1，RS485 半双工）
 *
 * 设备型号在 menuconfig 中选择（IoT Gateway Configuration -> Data acquisition -> Modbus -> Device）。
 * 每种设备在下面的"设备定义"中占一个 #if 块，集中定义：
 *   串口参数 / 寄存器参数表（CID）/ 读到数据后的处理 OnValue()
 * plc_poll_task 每秒调用 master_operation_func() 读一遍所有 CID。
 *
 * 新增一种设备：在 Kconfig 的 PLC_MODBUS_RTU_DEVICE 中加选项，在这里加一个 #elif 块。
 */

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "modbus_params.h"  // for modbus parameters structures
#include "mbcontroller.h"
#include "sdkconfig.h"
#include "iot_common.h"
#include "iot_report.h"
#include "master.h"

#define MASTER_TAG "MODBUS_RTU"

#define MB_PORT_NUM       1     // UART1
#define MB_UART_TXD_PIN   17
#define MB_UART_RXD_PIN   18
#define MB_UART_RTS_PIN   8
#define MB_DEVICE_ADDR    1     // 从站地址

#define MASTER_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(MASTER_TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

// 参数在 holding_reg_params 中的偏移（esp-modbus 要求非 0，因此 +1）
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define STR(fieldname) ((const char*)( fieldname ))
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

/* 参数表一行：{ CID, 名称, 单位, 从站地址, 寄存器类型, 起始地址, 寄存器个数, 存储偏移, 数据类型, 字节数, 选项, 权限 } */
#define HOLD_U16(cid, name, unit, reg, field) \
    { cid, STR(name), STR(unit), MB_DEVICE_ADDR, MB_PARAM_HOLDING, reg, 1, \
      HOLD_OFFSET(field), PARAM_TYPE_U16, 2, OPTS(-40, 100, 1), PAR_PERMS_READ_WRITE_TRIGGER }
#define HOLD_FLOAT(cid, name, unit, reg, field) \
    { cid, STR(name), STR(unit), MB_DEVICE_ADDR, MB_PARAM_HOLDING, reg, 2, \
      HOLD_OFFSET(field), PARAM_TYPE_FLOAT, 4, OPTS(-40, 100, 1), PAR_PERMS_READ }

/* ================================================================ 设备定义 */

#if defined(CONFIG_PLC_MODBUS_RTU_AIR_SWITCH)
/*
 * 智能空开：上位机 PowerOn / PowerOff 时写 13 号寄存器合闸 / 分闸（master_send_switch_func），
 * 轮询只读取状态，不上报
 */
#define MB_BAUD     9600
#define MB_PARITY   UART_PARITY_DISABLE
#define MB_STOP     UART_STOP_BITS_1
#define MB_HAS_SWITCH

enum { CID_SWITCH };

static const mb_parameter_descriptor_t s_params[] = {
    { CID_SWITCH, STR("SWITCH"), STR("__"), MB_DEVICE_ADDR, MB_PARAM_HOLDING, 13, 1,
      HOLD_OFFSET(holding_data0), PARAM_TYPE_ASCII, 2, OPTS(0, 100, 1), PAR_PERMS_READ_WRITE_TRIGGER },
};

static void OnValue(uint16_t cid, uint8_t type, uint32_t raw)
{
    (void)cid;
    (void)type;
    (void)raw;
}

#elif defined(CONFIG_PLC_MODBUS_RTU_TEMP_CONTROLLER)
/* 电镀厂温控仪表：测量值、设定值、小数点位置都读到后，上报测量值 FR001 */
#define MB_BAUD     9600
#define MB_PARITY   UART_PARITY_DISABLE
#define MB_STOP     UART_STOP_BITS_2

enum { CID_PV, CID_SV, CID_DPT };

static const mb_parameter_descriptor_t s_params[] = {
    HOLD_U16(CID_PV,  "Data_channel_0", "V",  74, holding_data0),   // 测量值
    HOLD_U16(CID_SV,  "Data_channel_1", "A",  75, holding_data1),   // 设定值
    HOLD_U16(CID_DPT, "Data_channel_2", "__", 12, holding_data2),   // 小数点位置
};

static int s_pv = 0, s_sv = 0, s_dpt = -1;

static void OnValue(uint16_t cid, uint8_t type, uint32_t raw)
{
    float real, set;
    int dpt;

    (void)type;
    switch (cid) {
    case CID_PV:  s_pv = (int)raw;  break;
    case CID_SV:  s_sv = (int)raw;  break;
    case CID_DPT: s_dpt = (int)raw; break;
    default: break;
    }
    if (s_pv == 0 || s_sv == 0 || s_dpt == -1) {
        return;
    }

    // 小数点位置 >= 128 时实际位数为 (dpt - 128) + 1
    dpt = (s_dpt >= 128) ? (s_dpt - 128 + 1) : s_dpt;
    real = (float)s_pv / pow(10, dpt);
    set = (float)s_sv / pow(10, dpt);
    s_pv = 0;
    s_sv = 0;
    s_dpt = -1;
    if (real != 0 && set != 0) {
        IotReport_SendValue(TEMPDEVTYPEID, DEVTYPENAME, "FR001", "%.1f", real);
    }
}

#elif defined(CONFIG_PLC_MODBUS_RTU_RECTIFIER)
/* 电镀厂整流机：电压 FR001、电流 FR002，每轮读完电流后、两者都不为 0 时上报 */
#define MB_BAUD     19200
#define MB_PARITY   UART_PARITY_EVEN
#define MB_STOP     UART_STOP_BITS_1

enum { CID_VOLTAGE, CID_CURRENT };

static const mb_parameter_descriptor_t s_params[] = {
    HOLD_U16(CID_VOLTAGE, "Data_channel_0", "V", 0, holding_data0),
    HOLD_U16(CID_CURRENT, "Data_channel_1", "A", 1, holding_data1),
};

static float s_voltage, s_current;

static void OnValue(uint16_t cid, uint8_t type, uint32_t raw)
{
    (void)type;
    if (cid == CID_VOLTAGE) {
        s_voltage = (float)raw;
        return;
    }
    s_current = (float)raw;
    // 原实现在读到电压和电流后各上报一次（每轮重复两遍），改为每轮只上报一次
    if (s_voltage != 0 && s_current != 0) {
        IotReport_SendValue(MOTORDEVTYPEID, DEVTYPENAME, "FR001", "%.1f", s_voltage);
        IotReport_SendValue(MOTORDEVTYPEID, DEVTYPENAME, "FR002", "%.1f", s_current);
    }
}

#elif defined(CONFIG_PLC_MODBUS_RTU_FREEZER)
/* 电镀厂冷冻机：温度（原始值 / 10）FR001，不为 0 时上报 */
#define MB_BAUD     4800
#define MB_PARITY   UART_PARITY_DISABLE
#define MB_STOP     UART_STOP_BITS_1

enum { CID_TEMPERATURE };

static const mb_parameter_descriptor_t s_params[] = {
    HOLD_U16(CID_TEMPERATURE, "Data_channel_0", "C", 7, holding_data0),
};

static float s_temperature;

static void OnValue(uint16_t cid, uint8_t type, uint32_t raw)
{
    (void)type;
    if (cid == CID_TEMPERATURE) {
        s_temperature = (float)raw / 10;
    }
    if (s_temperature != 0) {
        IotReport_SendValue(FREEZERDEVTYPEID, DEVTYPENAME, "FR001", "%.1f", s_temperature);
    }
}

#else
/* 无纸记录仪（默认）：6 个温区温度，浮点，第 n 个温区上报为 FR00n，值为 0 时不上报 */
#define MB_BAUD     9600
#define MB_PARITY   UART_PARITY_DISABLE
#define MB_STOP     UART_STOP_BITS_1

static const mb_parameter_descriptor_t s_params[] = {
    HOLD_FLOAT(0, "Data_channel_0", "C", 0x2000, holding_data0),
    HOLD_FLOAT(1, "Data_channel_1", "C", 0x2002, holding_data1),
    HOLD_FLOAT(2, "Data_channel_2", "C", 0x2004, holding_data2),
    HOLD_FLOAT(3, "Data_channel_3", "C", 0x2006, holding_data3),
    HOLD_FLOAT(4, "Data_channel_4", "C", 0x2008, holding_data4),
    HOLD_FLOAT(5, "Data_channel_5", "C", 0x200A, holding_data5),
};

static void OnValue(uint16_t cid, uint8_t type, uint32_t raw)
{
    float value;

    if (type != PARAM_TYPE_FLOAT) {
        return;
    }
    memcpy(&value, &raw, sizeof(value));
    if (value != 0) {
        IotReport_SendPoint(TEMPDEVTYPEID, DEVTYPENAME, cid + 1, "%.2f", value);   // 温度记录 -> WK
    }
}
#endif

#define MB_PARAM_COUNT  (sizeof(s_params) / sizeof(s_params[0]))

/* ================================================================ 轮询 */

// 读一遍所有 CID，周期由调用方 plc_poll_task 控制
void master_operation_func(void *arg)
{
    (void)arg;

    for (uint16_t cid = 0; cid < MB_PARAM_COUNT; cid++) {
        const mb_parameter_descriptor_t *desc = NULL;
        uint32_t raw = 0;
        uint8_t type = 0;
        esp_err_t err;

        err = mbc_master_get_cid_info(cid, &desc);
        if (err == ESP_ERR_NOT_FOUND || desc == NULL) {
            break;
        }
        err = mbc_master_get_parameter(cid, (char *)desc->param_key, (uint8_t *)&raw, &type);
        if (err != ESP_OK) {
            ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                     desc->cid, (char *)desc->param_key, (int)err, (char *)esp_err_to_name(err));
            continue;
        }

        if (type == PARAM_TYPE_FLOAT) {
            raw = (raw << 16) | (raw >> 16);    // 设备按高字在前发送，交换高低字
        }
        ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) = 0x%08x", desc->cid,
                 (char *)desc->param_key, (char *)desc->param_units, (unsigned)raw);
        OnValue(cid, type, raw);
    }
}

// 上位机 PowerOn / PowerOff：智能空开合闸 / 分闸，其他设备不处理
void master_send_switch_func(int status)
{
#ifdef MB_HAS_SWITCH
    const mb_parameter_descriptor_t *desc = NULL;
    uint8_t value[4] = { 0x00, (status == 1) ? 0xFF : 0x00, 0x00, 0x00 };
    uint8_t type = 0;
    esp_err_t err;

    err = mbc_master_get_cid_info(CID_SWITCH, &desc);
    if (err == ESP_ERR_NOT_FOUND || desc == NULL) {
        return;
    }
    err = mbc_master_set_parameter(CID_SWITCH, (char *)desc->param_key, value, &type);
    if (err == ESP_OK) {
        ESP_LOGI(MASTER_TAG, "switch %s written", status == 1 ? "on" : "off");
    } else {
        ESP_LOGE(MASTER_TAG, "switch write fail, err = 0x%x (%s).", (int)err, (char *)esp_err_to_name(err));
    }
#else
    (void)status;
#endif
}

/* ================================================================ 初始化 */

esp_err_t master_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = MB_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = MB_PARITY,
        .stop_bits = MB_STOP,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    mb_communication_info_t comm = {
        .port = MB_PORT_NUM,
        .mode = MB_MODE_RTU,
        .baudrate = uart_config.baud_rate,
        .parity = MB_PARITY_NONE,           // 校验位和停止位在下面 uart_param_config 中按设备设置
    };
    void *master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MASTER_CHECK((master_handler != NULL), ESP_ERR_INVALID_STATE,
                 "mb controller initialization fail.");
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                 "mb controller initialization fail, returns(0x%x).", (uint32_t)err);
    err = mbc_master_setup((void *)&comm);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                 "mb controller setup fail, returns(0x%x).", (uint32_t)err);

    err = uart_set_pin(MB_PORT_NUM, MB_UART_TXD_PIN, MB_UART_RXD_PIN, MB_UART_RTS_PIN, UART_PIN_NO_CHANGE);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                 "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);

    err = mbc_master_start();
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                 "mb controller start fail, returns(0x%x).", (uint32_t)err);

    uart_param_config(MB_PORT_NUM, &uart_config);
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                 "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&s_params[0], MB_PARAM_COUNT);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                 "mb controller set descriptor fail, returns(0x%x).", (uint32_t)err);
    ESP_LOGI(MASTER_TAG, "Modbus master stack initialized, %d parameters", (int)MB_PARAM_COUNT);
    return err;
}
