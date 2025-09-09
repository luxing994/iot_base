#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

void tcp_client_task(void *pvParameters);
void tcp_client1_task(void *pvParameters);
#if (defined CONFIG_PLC_MODBUS_CONNECT_NETWORK)
void tcp_modbus_client_task(void *pvParameters);
#endif

#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
void tcp_fx_client_task(void *pvParameters);
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
void tcp_ppi_client_task(void *pvParameters);
#endif

void tcp_client3_task(void *pvParameters);
void heart_beat_task(void *pvParameters);
char* GetStaIp(void);

#endif