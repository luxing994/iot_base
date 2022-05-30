#ifndef TCP_SERVER_H
#define TCP_SERVER_H

void tcp_server_task(void *pvParameters);
void tcp_server1_task(void *pvParameters);
int GetFileData(uint8_t *data, int size);
int GetTaskNum(uint16_t *tasknum);
int GetTaskPitch(uint16_t *taskpitch);
int GetTaskSpeed(uint16_t *taskspeed);
int GetTaskCount(uint16_t *taskcount);
int GetTaskTime(uint16_t *tasktime);
int GetMode(uint8_t *mode);

#endif