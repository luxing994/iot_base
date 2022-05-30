#ifndef UART_TASK_H
#define UART_TASK_H

void uart_init(void);
void tx_task(void *arg);
void tx1_task(void *arg);
void rx_task(void *arg);
void uart_event_task(void *pvParameters);

#endif