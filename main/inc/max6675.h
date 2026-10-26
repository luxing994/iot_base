#ifndef MAX6675_H
#define MAX6675_H

#ifdef CONFIG_SENSOR_MAX6675

void MAX6675_Init(void);
float MAX6675_ReadTemp(void);
void max6675_task(void *pvParameters);

#endif /* CONFIG_SENSOR_MAX6675 */

#endif
