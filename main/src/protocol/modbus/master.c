/*
 * SPDX-FileCopyrightText: 2016-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include "string.h"
#include "esp_log.h"
#include "modbus_params.h"  // for modbus parameters structures
#include "mbcontroller.h"
#include "sdkconfig.h"
#include "iot_common.h"

#define AIR_SWITCH        0
#define TEMP_CONTROLER    1
#define MOTOR             2
#define FREEZER           3

#define AIR_SWITCH_SPEED      9600
#define TEMP_CONTROLER_SPEED  9600
#define MOTOR_SPEED           19200
#define FREEZER_SPEED         4800

#define DEVICE_VERSION    MOTOR

#define MB_PORT_NUM     (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_DEV_SPEED    (CONFIG_MB_UART_BAUD_RATE)  // The communication speed of the UART

// Note: Some pins on target chip cannot be assigned for UART communication.
// See UART documentation for selected board and target to configure pins using Kconfig.

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 30

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS          (1000)
#define UPDATE_CIDS_TIMEOUT_TICS        (UPDATE_CIDS_TIMEOUT_MS / portTICK_RATE_MS)

// Timeout between polls
#define POLL_TIMEOUT_MS                 (1)
#define POLL_TIMEOUT_TICS               (POLL_TIMEOUT_MS / portTICK_RATE_MS)

#define MASTER_TAG "MASTER_TEST"

#define MASTER_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(MASTER_TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char*)( fieldname ))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

// Enumeration of modbus device addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 2 // Only one slave device used for the test (add other slave addresses here)
};
#if DEVICE_VERSION == AIR_SWITCH
// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_HOLD_WRITE_REG_1,
    CID_COUNT
};
#elif DEVICE_VERSION == TEMP_CONTROLER
// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_HOLD_DATA_0 = 0,
    CID_HOLD_DATA_1,
    CID_HOLD_DATA_2,
    CID_COUNT
};
#elif DEVICE_VERSION == MOTOR
// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_HOLD_DATA_0 = 0,
    CID_HOLD_DATA_1,
    CID_COUNT
};
#else
// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_HOLD_DATA_0 = 0,
    CID_COUNT
};
#endif

// Example Data (Object) Dictionary for Modbus parameters:
// The CID field in the table must be unique.
// Modbus Slave Addr field defines slave address of the device with correspond parameter.
// Modbus Reg Type - Type of Modbus register area (Holding register, Input Register and such).
// Reg Start field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type, Data Size specify type of the characteristic and its data size.
// Parameter Options field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode - can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
#if DEVICE_VERSION == AIR_SWITCH
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
    { CID_HOLD_WRITE_REG_1, STR("SWITCH"), STR("__"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 13, 1,
            HOLD_OFFSET(holding_data0), PARAM_TYPE_ASCII, 2, OPTS( 0, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
};
#elif DEVICE_VERSION == TEMP_CONTROLER
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
    { CID_HOLD_DATA_0, STR("Data_channel_0"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 74, 1,
            HOLD_OFFSET(holding_data0), PARAM_TYPE_FLOAT, 2, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_HOLD_DATA_1, STR("Data_channel_1"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 75, 1,
            HOLD_OFFSET(holding_data1), PARAM_TYPE_FLOAT, 2, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_HOLD_DATA_2, STR("Data_channel_2"), STR("__"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 12, 1,
            HOLD_OFFSET(holding_data2), PARAM_TYPE_FLOAT, 2, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
};
#elif DEVICE_VERSION == MOTOR
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
    { CID_HOLD_DATA_0, STR("Data_channel_0"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0, 1,
            HOLD_OFFSET(holding_data0), PARAM_TYPE_FLOAT, 2, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_HOLD_DATA_1, STR("Data_channel_1"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 1, 1,
            HOLD_OFFSET(holding_data1), PARAM_TYPE_FLOAT, 2, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
};
#else
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
    { CID_HOLD_DATA_0, STR("Data_channel_0"), STR("C"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 7, 1,
            HOLD_OFFSET(holding_data0), PARAM_TYPE_FLOAT, 2, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
};
#endif

TempControlTransData tempTransData = {0, 0, -1};
ElectroFactoryData electroData = {0};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

// The function to get pointer to parameter storage (instance) according to parameter description table
static void* master_get_param_data(const mb_parameter_descriptor_t* param_descriptor)
{
    assert(param_descriptor != NULL);
    void* instance_ptr = NULL;
    if (param_descriptor->param_offset != 0) {
       switch(param_descriptor->mb_param_type)
       {
           case MB_PARAM_HOLDING:
               instance_ptr = ((void*)&holding_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_INPUT:
               instance_ptr = ((void*)&input_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_COIL:
               instance_ptr = ((void*)&coil_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_DISCRETE:
               instance_ptr = ((void*)&discrete_reg_params + param_descriptor->param_offset - 1);
               break;
           default:
               instance_ptr = NULL;
               break;
       }
    } else {
        ESP_LOGE(MASTER_TAG, "Wrong parameter offset for CID #%d", param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}


#if DEVICE_VERSION == TEMP_CONTROLER
void ParseTemperatureData(uint16_t cid, int data)
{
    switch (cid) {
    case CID_HOLD_DATA_0:
        tempTransData.PV = data;
        break;
    case CID_HOLD_DATA_1:
        tempTransData.SV = data;
        break;
    case CID_HOLD_DATA_2:
        tempTransData.dPt = data;
        break;
    default:
        break;
    }
    if (tempTransData.PV != 0 && tempTransData.SV != 0 && tempTransData.dPt != -1) {
        if (tempTransData.dPt >= 128) {
            tempTransData.dPt -= 128;
            electroData.tempControl.realData = (float)tempTransData.PV / pow(10, (tempTransData.dPt) + 1);
            electroData.tempControl.setData = (float)tempTransData.SV / pow(10, (tempTransData.dPt) + 1);
        } else {
            electroData.tempControl.realData = (float)tempTransData.PV / pow(10, tempTransData.dPt);
            electroData.tempControl.setData = (float)tempTransData.SV / pow(10, tempTransData.dPt);
        }
        tempTransData.PV = 0;
        tempTransData.SV = 0;
        tempTransData.dPt = -1;
        if (electroData.tempControl.realData != 0 && electroData.tempControl.setData != 0) {
            xEventGroupSetBits(xEventGroup1, BIT_18);
        }
    }
}
#elif DEVICE_VERSION == MOTOR
void ParseMotorData(uint16_t cid, int data)
{
    switch (cid) {
    case CID_HOLD_DATA_0:
        electroData.motorData.voltage = (float)data;
        break;
    case CID_HOLD_DATA_1:
        electroData.motorData.current = (float)data;
        break;
    default:
        break;
    }
    if (electroData.motorData.voltage != 0 && electroData.motorData.current != 0) {
        xEventGroupSetBits(xEventGroup1, BIT_19);
    }
}
#else
void ParseFreezerData(uint16_t cid, int data)
{
    switch (cid) {
    case CID_HOLD_DATA_0:
        electroData.freezerData.temperature = (float)data / 10;
        break;
    default:
        break;
    }
    if (electroData.freezerData.temperature != 0) {
        xEventGroupSetBits(xEventGroup1, BIT_20);
    }
}
#endif
// User operation function to read slave values and check alarm
void master_operation_func(void *arg)
{
    esp_err_t err = ESP_OK;
    float value = 0;
    int dvalue = 0;
    bool alarm_state = false;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    ESP_LOGI(MASTER_TAG, "Start modbus test...");
    
    for (;;) {
    // for(uint16_t retry = 0; retry <= MASTER_MAX_RETRY && (!alarm_state); retry++) {
        // Read all found characteristics from slave(s)
        for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++)
        {
            // Get data from parameters description table
            // and use this information to fill the characteristics description table
            // and having all required fields in just one table
            err = mbc_master_get_cid_info(cid, &param_descriptor);
            
            if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                void* temp_data_ptr = master_get_param_data(param_descriptor);
                assert(temp_data_ptr);
                uint8_t type = 0;
    
                err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                    (uint8_t*)&dvalue, &type);
                if (err == ESP_OK) {
                    *(uint16_t* )temp_data_ptr = dvalue;
                    if ((param_descriptor->mb_param_type == MB_PARAM_HOLDING) ||
                        (param_descriptor->mb_param_type == MB_PARAM_INPUT)) {
                        ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = %d (0x%x) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        dvalue,
                                        *(uint32_t*)temp_data_ptr);
#if DEVICE_VERSION == TEMP_CONTROLER
                        ParseTemperatureData(param_descriptor->cid, dvalue);
#elif DEVICE_VERSION == MOTOR
                        ParseMotorData(param_descriptor->cid, dvalue);
#else
                        ParseFreezerData(param_descriptor->cid, dvalue);
#endif
                    } else {
                        uint16_t state = *(uint16_t*)temp_data_ptr;
                        const char* rw_str = (state & param_descriptor->param_opts.opt1) ? "ON" : "OFF";
                        ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = %s (0x%x) read successful.",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (char*)param_descriptor->param_units,
                                        (const char*)rw_str,
                                        *(uint16_t*)temp_data_ptr);
                        if (state & param_descriptor->param_opts.opt1) {
                            alarm_state = true;
                            ESP_LOGI(MASTER_TAG, "Alarm triggered by cid #%d.",
                                    param_descriptor->cid);
                        }
                    }
                } else {
                    ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                }
                vTaskDelay(POLL_TIMEOUT_TICS); // timeout between polls
            }
        }
        vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS); //
    }
}

void master_send_switch_func(int status)
{
#if DEVICE_VERSION == AIR_SWITCH
    esp_err_t err = ESP_OK;
    float value = 0;
    bool alarm_state = false;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    err = mbc_master_get_cid_info(CID_HOLD_WRITE_REG_1, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
        void* temp_data_ptr = master_get_param_data(param_descriptor);
        assert(temp_data_ptr);
        uint8_t type = 0;
        if ((param_descriptor->param_type == PARAM_TYPE_ASCII) &&
            (param_descriptor->cid == CID_HOLD_WRITE_REG_1)) {
            if (status == 1) {
                ((uint8_t* )temp_data_ptr)[0] = 0x00;
                ((uint8_t* )temp_data_ptr)[1] = 0xFF;
            } else {
                ((uint8_t* )temp_data_ptr)[0] = 0x00;
                ((uint8_t* )temp_data_ptr)[1] = 0x00;
            }
            
            err = mbc_master_set_parameter(CID_HOLD_WRITE_REG_1, (char*)param_descriptor->param_key,
                                                (uint8_t*)temp_data_ptr, &type);
            if (err == ESP_OK) {
                ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = (0x%08x), write successful.",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (char*)param_descriptor->param_units,
                                            *(uint32_t*)temp_data_ptr);
            } else {
                ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) write fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
            }
        }
    }
#endif
}

// Modbus master initialization
esp_err_t master_init(void)
{
#if DEVICE_VERSION == AIR_SWITCH
    const uart_config_t uart_config = {
        .baud_rate = AIR_SWITCH_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#elif DEVICE_VERSION == TEMP_CONTROLER
    const uart_config_t uart_config = {
        .baud_rate = TEMP_CONTROLER_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#elif DEVICE_VERSION == MOTOR
    const uart_config_t uart_config = {
        .baud_rate = MOTOR_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#else 
    const uart_config_t uart_config = {
        .baud_rate = FREEZER_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#endif
    
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
            .port = MB_PORT_NUM,
#if CONFIG_MB_COMM_MODE_ASCII
            .mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
            .mode = MB_MODE_RTU,
#endif
            .baudrate = MB_DEV_SPEED,
            .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MASTER_CHECK((master_handler != NULL), ESP_ERR_INVALID_STATE,
                                "mb controller initialization fail.");
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller initialization fail, returns(0x%x).",
                            (uint32_t)err);
    err = mbc_master_setup((void*)&comm);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller setup fail, returns(0x%x).",
                            (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                              CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);

    err = mbc_master_start();
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller start fail, returns(0x%x).",
                            (uint32_t)err);

    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);
    uart_param_config(MB_PORT_NUM, &uart_config);
    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                "mb controller set descriptor fail, returns(0x%x).",
                                (uint32_t)err);
    ESP_LOGI(MASTER_TAG, "Modbus master stack initialized...");
    return err;
}
