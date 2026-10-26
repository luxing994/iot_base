#ifndef UART_TASK_H
#define UART_TASK_H

#include "sdkconfig.h"

/* 需要初始化 UART1 的协议（串口协议；PPI 选网口时仍沿用原有的 UART 初始化） */
#if defined(CONFIG_PLC_FX) || defined(CONFIG_PLC_HOSTLINK) || defined(CONFIG_PLC_LS_LOAD) || \
    defined(CONFIG_PLC_PPI) || defined(CONFIG_TESTER_76T) || defined(CONFIG_TESTER_AINUO)
#define APP_UART_ENABLED 1
#endif

/* 需要 rx_task 解析应答的协议（FX 网口也经由 rx_task 解码） */
#if defined(CONFIG_PLC_FX) || defined(CONFIG_PLC_HOSTLINK) || defined(CONFIG_PLC_LS_LOAD) || \
    defined(CONFIG_PLC_PPI_CONNECT_RS485) || defined(CONFIG_TESTER_76T) || defined(CONFIG_TESTER_AINUO)
#define APP_RX_TASK_ENABLED 1
#endif

#ifdef APP_UART_ENABLED
void uart_init(void);
void uart_event_task(void *pvParameters);
#endif

#ifdef APP_RX_TASK_ENABLED
void rx_task(void *arg);
#endif

#ifdef CONFIG_PLC_MODBUS_CONNECT_NETWORK
void rx_modbus_task(void *arg);
#endif

#ifdef CONFIG_PLC_PPI_CONNECT_NETWORK
void rx_ppi_task(void *arg);
#endif

void tx_task(void *arg);

#endif
