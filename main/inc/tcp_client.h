#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

void tcp_client_task(void *pvParameters);
void tcp_client1_task(void *pvParameters);
void heart_beat_task(void *pvParameters);
char* GetStaIp(void);

#endif