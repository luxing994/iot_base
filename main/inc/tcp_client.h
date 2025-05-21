#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

void tcp_client_task(void *pvParameters);
void tcp_client1_task(void *pvParameters);
#if (defined CONFIG_PLC_NETWORK)
void tcp_client2_task(void *pvParameters);
#endif
void heart_beat_task(void *pvParameters);
char* GetStaIp(void);

#endif