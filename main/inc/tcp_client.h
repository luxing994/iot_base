#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

void tcp_client_task(void *pvParameters);
void tcp_client_recv_task(void *pvParameters);

#endif