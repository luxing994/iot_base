#ifndef MASTER_H

esp_err_t master_init(void);
void master_operation_func(void *arg);
void master_send_switch_func(int status);

#endif