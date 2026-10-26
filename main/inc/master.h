#ifndef MASTER_H
#define MASTER_H

#include "esp_err.h"

esp_err_t master_init(void);
void master_operation_func(void *arg);
void master_send_switch_func(int status);

#endif
