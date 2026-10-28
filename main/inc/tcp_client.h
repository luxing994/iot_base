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
char* GetStaIp(void);

/* 各连接的当前状态（配置网页的状态页用） */
#include <stdbool.h>

typedef struct {
    const char *tag;        // "HOST" / "PLC"
    const char *peer;       // "report channel" 等
    bool up;
    bool off;               // 端口为 0，不使用
} TcpLinkState;

int TcpClient_GetLinks(TcpLinkState *out, int max);

/* 上位机端口为 0 时 app_main 不创建该连接任务，调用此函数登记为停用（状态显示 off） */
void TcpClient_RegisterOff(const char *peer);

// OTA 控制通道 socket（tcp_client3_task 维护，断线时为 -1），OTA 任务用它回传进度
extern int ota_tcp_sock;

#endif