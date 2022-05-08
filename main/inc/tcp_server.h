#ifndef TCP_SERVER_H
#define TCP_SERVER_H

void tcp_server_task(void *pvParameters);
void tcp_server1_task(void *pvParameters);
int GetFileData(uint8_t *data, int size);

#endif