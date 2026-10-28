#ifndef MAX6675_H
#define MAX6675_H

#include "sdkconfig.h"      // 下面的条件依赖 CONFIG_SENSOR_MAX6675，不能依赖包含顺序

#ifdef CONFIG_SENSOR_MAX6675

void MAX6675_Init(void);
float MAX6675_ReadTemp(void);
void max6675_task(void *pvParameters);

#endif /* CONFIG_SENSOR_MAX6675 */

#endif
