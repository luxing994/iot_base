#ifndef UART_TASK_H
#define UART_TASK_H

#include "sdkconfig.h"

/*
 * 使用 UART1 的串口协议（Modbus RTU 由 esp-modbus 自行管理串口，不在此列）。
 * UART1 同一时间只能服务一种串口协议，选了多个时在这里直接报错。
 */
#if ((defined(CONFIG_PLC_FX_CONNECT_RS232) || defined(CONFIG_PLC_FX_CONNECT_RS485)) + \
     defined(CONFIG_PLC_HOSTLINK) + defined(CONFIG_PLC_LS_LOAD) + defined(CONFIG_PLC_PPI_CONNECT_RS485) + \
     (defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)) + \
     defined(CONFIG_TESTER_76T) + defined(CONFIG_TESTER_AINUO)) > 1
#error "menuconfig: UART1 can serve only one serial protocol. Keep one serial device under " \
       "IoT Gateway Configuration -> Data acquisition (network connections can be combined)."
#endif

#if defined(CONFIG_PLC_FX_CONNECT_RS232) || defined(CONFIG_PLC_FX_CONNECT_RS485) || \
    defined(CONFIG_PLC_HOSTLINK) || defined(CONFIG_PLC_LS_LOAD) || defined(CONFIG_PLC_PPI_CONNECT_RS485) || \
    defined(CONFIG_TESTER_76T) || defined(CONFIG_TESTER_AINUO)
#define APP_UART_ENABLED 1
#endif

#include <stdbool.h>
#include <stdint.h>

/* 串口参数（设备配置导入后由采集模块按配置设置） */
typedef struct {
    uint32_t baud;
    uint8_t dataBits;       // 7 / 8
    char parity;            // 'N' / 'E' / 'O'
    uint8_t stopBits;       // 1 / 2
    bool rs485;             // RS485 半双工（RTS 控制方向）
    const char *name;       // "RS485" / "RS232" / "RS422"，用于提示拨码开关位置
} UartParams;

/* 安装 UART1 驱动并按参数配置；之后创建 uart_event_task 开始接收 */
void UART_Start(const UartParams *p);
void uart_event_task(void *pvParameters);

#ifdef APP_UART_ENABLED
void uart_init(void);       // menuconfig 选择设备时：按所选协议调用 UART_Start()
#endif

/*
 * 串口接口说明，如 "RS485 115200 8N1"（用于启动信息、网页和诊断，提醒现场核对拨码开关）；
 * 没有串口设备时返回 NULL。UART_InterfaceName() 只返回 "RS485" / "RS232"。
 */
const char *UART_InterfaceText(void);
const char *UART_InterfaceName(void);

// 执行上位机命令（host_cmd 置事件位）并上报开关量中断事件
void cmd_task(void *arg);

#endif
