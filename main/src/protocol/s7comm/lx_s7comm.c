#include "lx_s7comm.h"
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "esp_log.h"
#include "iot_common.h"

// -------------------------- 定义常量和宏 --------------------------

#define S7_TCP_PORT 102                      // S7comm 使用的 TCP 端口
#define TPKT_VERSION 0x03                    // TPKT 协议版本
#define COTP_CR_PDU_TYPE 0xE0                // COTP Connection Request (CR) 类型
#define COTP_CC_PDU_TYPE 0xD0                // COTP Confirm (CC) 类型
#define S7_PROTOCOL_ID 0x32                  // S7 协议 ID
#define S7_COMMUNICATION_FUNCTION 0x01       // S7 Setup Communication 功能码
#define S7_READ_VAR_FUNCTION 0x04            // S7 ReadVar 功能码

#define S7_JOB_TYPE 0x01                     // ROSCTR：Job (0x01)
#define S7_REDUNDANCY 0x00                   // 冗余字段，通常为 0
#define S7_PDU_REF 0x00                      // PDU 引用字段
#define S7_PARAM_LENGTH 0x0E                 // 参数长度（14 字节）
#define S7_DATA_LENGTH 0x00                  // 数据长度（初始化时为 0）
#define COTP_TPDU_SIZE 0x0A                  // COTP TPDU size code (1024字节)
#define MAX_PDU_SIZE 1024                   // 最大 PDU 大小

static const char *TAG = "S7COMM";
uint32_t s7dataunitreference = 0;
uint32_t s7netsrdatabuffaddr = 0;
uint32_t s7netsrdatalen = 0;
int s7netdatarecvflag = 0;

static void S7_NetDataRecvNotice(uint32_t dataaddr, uint32_t len, uint16_t frnum, uint8_t datatype)
{
    s7netdatarecvflag = 1;
	s7netsrdatabuffaddr = dataaddr;
	s7netsrdatalen = len;
	g_fxplccount = frnum;
	g_fxplcdataformat = datatype;
}

// -------------------------- 发送和接收数据的工具函数 --------------------------

static int send_all(int fd, const uint8_t *buf, size_t len) {
    int err = send(fd, buf, len, 0);
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        return errno;
    }
    return 0;
}

static int recv_some(int fd, void *buf, size_t cap) {
    int len = recv(fd, buf, cap, 0);
     ESP_LOGI(TAG, "read data size :%d", len);
    if (len <= 0) {
        ESP_LOGE(TAG, "Error occurred during recving: errno %d", errno);
        return errno;
    }
    return 0;
}

// 构建 TPKT 头部
static void tpkt(uint8_t *p, uint16_t len_without_tpkt) {
    p[0] = TPKT_VERSION;
    p[1] = 0x00;
    uint16_t tot = htons((uint16_t)(len_without_tpkt + 4));
    memcpy(p + 2, &tot, 2);
}

// -------------------------- COTP 连接请求和确认 --------------------------

/**
 * @brief COTP Connection Request (CR)，期望接收 Confirm (CC)
 * 
 * @param fd 已连接的 TCP socket
 * @param local_tsap 本地 TSAP 地址
 * @param remote_tsap 远程 TSAP 地址
 * @return int 0 成功，负值失败
 */
static int cotp_cr_cc(int fd) 
{
    uint8_t pkt[22] = { 
        0x03, 0x0, 0x0, 0x16,   // TPKT Header (4 bytes)
        0x11, 0xe0, 0x00, 0x00, // COTP Header (4 bytes)
        0x04, 0x30, 0x0, 
        0xc1, 0x02, 0x01, 0x01,  // Source TSAP
        0xc2, 0x02, 0x01, 0x01,  // Destination TSAP
        0xc0, 0x01, 0x0a
    };

    // 发送 COTP CR 请求
    if (send_all(fd, pkt, sizeof(pkt)) != 0) return -1;
     
    uint8_t rsp[128];
    int got = recv_some(fd, rsp, sizeof(rsp));
    if (got != 0) {
        return -1;
    }

    if (rsp[0] != TPKT_VERSION) {
        return -1;
    }

    if (rsp[5] != COTP_CC_PDU_TYPE) {
        return -1;
    }  // 确认 COTP CC 类型
    return 0;
}

/**
 * @brief 设置 S7 通信（协商 PDU 长度）
 * 
 * @param fd 已连接的 TCP socket
 * @param out_pdu_len 输出协商的 PDU 长度
 * @return int 0 成功，负值失败
 */
static int s7_setup_comm(int fd, uint16_t *out_pdu_len) 
{
    uint8_t pkt[25] = { 
        0x03, 0x0, 0x0, 0x19,  // TPKT Header (4 bytes)
        0x02, 0xf0, 0x80,      // COTP Header (3 bytes)
        0x32, 0x01, 0x00, 0x00, 0xcc, 0xc1, 0x00, 0x08, 0x00, 0x00, 
        0xf0, 0x00, 0x00, 0x01, 0x00, 0x01, 0x03, 0xc0
    };

    if (send_all(fd, pkt, sizeof(pkt)) != 0) {
        ESP_LOGE(TAG, "send failed, errno=%d", errno);
        return -1;
    }   

    uint8_t rsp[256];
    int got = recv_some(fd, rsp, sizeof(rsp));
    if (got != 0) {
        return -1;
    }
    if (rsp[0] != TPKT_VERSION) return -1;
    if (rsp[5] != 0xf0) return -1;
    uint16_t pdu = ((uint16_t)rsp[25] << 8) | rsp[26];
    if (out_pdu_len) *out_pdu_len = pdu;
    ESP_LOGI(TAG, "true pdu size:%d", pdu);
    return fd;
}

/**
 * @brief 设置 S7 通信（协商 PDU 长度）
 * 
 * @param fd 已连接的 TCP socket
 * @param out_pdu_len 输出协商的 PDU 长度
 * @return int 0 成功，负值失败
 */
static int s7_begin_data_trans(int fd) 
{
    uint8_t pkt[7] = { 
        0x03, 0x0, 0x0, 0x07,  // TPKT Header (4 bytes)
        0x02, 0xf0, 0x00     // COTP Header (3 bytes)
    };

    if (send_all(fd, pkt, sizeof(pkt)) != 0) {
        ESP_LOGE(TAG, "send failed, errno=%d", errno);
        return -1;
    }  

    return 0;
}

static int s7_read_bit(int sock, uint8_t area, uint32_t address, uint8_t bitoffset, uint8_t *out_word) 
{
    // 构造 S7 请求数据包
    uint8_t s7p[31];  // 包括 S7 header 和 ReadVar 请求
    // TPKT Header (4 bytes)
    s7p[0] = 0x03;
    s7p[1] = 0x0;
    s7p[2] = 0x0;
    s7p[3] = 0x1f;
    // COTP Header (3 bytes)
    s7p[4] = 0x02;
    s7p[5] = 0xf0;
    s7p[6] = 0x80;
    // S7 Header
    s7p[7] = 0x32;
    s7p[8] = 0x01; 
    s7p[9] = 0x00;
    s7p[10] = 0x00;
    s7p[11] = (s7dataunitreference >> 8) & 0xff;   //data unit reference high
    s7p[12] = s7dataunitreference  & 0xff;    //data unit reference low
    s7p[13] = 0x00;  
    s7p[14] = 0x0e;
    s7p[15] = 0x00;
    s7p[16] = 0x00;
    // Parameter
    s7p[17] = 0x04;   // Read Var 
    s7p[18] = 0x01;
    // Item
    s7p[19] = 0x12;
    s7p[20] = 0x0a;
    s7p[21] = 0x10;
    s7p[22] = 0x01;   // Data size: bit
    s7p[23] = 0x00;
    s7p[24] = 0x01;   // Data length
    s7p[25] = 0x00;
    s7p[26] = 0x00;
    s7p[27] = area;   // Data Area

    address <<= 3;
    address += bitoffset;
    s7p[28] = (address >> 16) & 0xff;    // Data Address
    s7p[29] = (address >> 8) & 0xff; 
    s7p[30] = address & 0xff; 

    s7_begin_data_trans(sock);
    if (send_all(sock, s7p, sizeof(s7p)) != 0) {
        ESP_LOGE(TAG, "send failed, errno=%d", errno);
        return -1;
    }
    s7dataunitreference++;

    uint8_t rsp[1024];  // 接收响应
    int got = recv_some(sock, rsp, sizeof(rsp));  // 接收响应数据
    if (got != 0) {
        return -2;  // 数据长度错误
    }

    const uint8_t *h = rsp + 7;
    if (h[0] != 0x32) {
        return -3;  // 错误的响应
    }
    
    if (h[1] != 0x03) {
        return -4;  // 错误的响应类型   
    }

    uint16_t data_len = (h[16] << 8) | h[17];

    if (data_len != 0x01) {
        ESP_LOGE(TAG, "data length = %d", data_len);
        return -5;  // 数据长度小于预期
    }
    
    *out_word = h[18];

    return 0;  // 成功
}

static int s7_read_bit_for_v(int sock, uint8_t area, uint32_t address, uint8_t bitoffset, uint8_t *out_word) 
{
    // 构造 S7 请求数据包
    uint8_t s7p[31];  // 包括 S7 header 和 ReadVar 请求
    // TPKT Header (4 bytes)
    s7p[0] = 0x03;
    s7p[1] = 0x0;
    s7p[2] = 0x0;
    s7p[3] = 0x1f;
    // COTP Header (3 bytes)
    s7p[4] = 0x02;
    s7p[5] = 0xf0;
    s7p[6] = 0x80;
    // S7 Header
    s7p[7] = 0x32;
    s7p[8] = 0x01; 
    s7p[9] = 0x00;
    s7p[10] = 0x00;
    s7p[11] = (s7dataunitreference >> 8) & 0xff;   //data unit reference high
    s7p[12] = s7dataunitreference  & 0xff;    //data unit reference low
    s7p[13] = 0x00;  
    s7p[14] = 0x0e;
    s7p[15] = 0x00;
    s7p[16] = 0x00;
    // Parameter
    s7p[17] = 0x04;   // Read Var 
    s7p[18] = 0x01;
    // Item
    s7p[19] = 0x12;
    s7p[20] = 0x0a;
    s7p[21] = 0x10;
    s7p[22] = 0x01;   // Data size: bit
    s7p[23] = 0x00;
    s7p[24] = 0x01;   // Data length
    s7p[25] = 0x00;
    s7p[26] = 0x01;
    s7p[27] = area;   // Data Area

    address <<= 3;
    address += bitoffset;
    s7p[28] = (address >> 16) & 0xff;    // Data Address
    s7p[29] = (address >> 8) & 0xff; 
    s7p[30] = address & 0xff; 

    s7_begin_data_trans(sock);
    if (send_all(sock, s7p, sizeof(s7p)) != 0) {
        ESP_LOGE(TAG, "send failed, errno=%d", errno);
        return -1;
    }
    s7dataunitreference++;

    uint8_t rsp[1024];  // 接收响应
    int got = recv_some(sock, rsp, sizeof(rsp));  // 接收响应数据
    if (got != 0) {
        return -2;  // 数据长度错误
    }

    const uint8_t *h = rsp + 7;
    if (h[0] != 0x32) {
        return -3;  // 错误的响应
    }
    
    if (h[1] != 0x03) {
        return -4;  // 错误的响应类型   
    }

    uint16_t data_len = (h[16] << 8) | h[17];

    if (data_len != 0x01) {
        ESP_LOGE(TAG, "data length = %d", data_len);
        return -5;  // 数据长度小于预期
    }
    
    *out_word = h[18];

    return 0;  // 成功
}

static int s7_read_word(int sock, uint8_t area, uint32_t address, uint16_t *out_word) 
{
    // 构造 S7 请求数据包
    uint8_t s7p[31];  // 包括 S7 header 和 ReadVar 请求
    // TPKT Header (4 bytes)
    s7p[0] = 0x03;
    s7p[1] = 0x0;
    s7p[2] = 0x0;
    s7p[3] = 0x1f;
    // COTP Header (3 bytes)
    s7p[4] = 0x02;
    s7p[5] = 0xf0;
    s7p[6] = 0x80;
    // S7 Header
    s7p[7] = 0x32;
    s7p[8] = 0x01; 
    s7p[9] = 0x00;
    s7p[10] = 0x00;
    s7p[11] = (s7dataunitreference >> 8) & 0xff;   //data unit reference high
    s7p[12] = s7dataunitreference  & 0xff;    //data unit reference low
    s7p[13] = 0x00;  
    s7p[14] = 0x0e;
    s7p[15] = 0x00;
    s7p[16] = 0x00;
    // Parameter
    s7p[17] = 0x04;   // Read Var 
    s7p[18] = 0x01;
    // Item
    s7p[19] = 0x12;
    s7p[20] = 0x0a;
    s7p[21] = 0x10;
    s7p[22] = 0x04;   // Data size: word
    s7p[23] = 0x00;
    s7p[24] = 0x01;   // Data length
    s7p[25] = 0x00;
    s7p[26] = 0x01;
    s7p[27] = area;   // Data Area

    address <<= 3;
    s7p[28] = (address >> 16) & 0xff;    // Data Address
    s7p[29] = (address >> 8) & 0xff; 
    s7p[30] = address & 0xff; 

    s7_begin_data_trans(sock);
    if (send_all(sock, s7p, sizeof(s7p)) != 0) {
        ESP_LOGE(TAG, "send failed, errno=%d", errno);
        return -1;
    }
    s7dataunitreference++;

    uint8_t rsp[1024];  // 接收响应
    int got = recv_some(sock, rsp, sizeof(rsp));  // 接收响应数据
    if (got != 0) {
        return -2;  // 数据长度错误
    }

    const uint8_t *h = rsp + 7;
    if (h[0] != 0x32) {
        return -3;  // 错误的响应
    }
    
    if (h[1] != 0x03) {
        return -4;  // 错误的响应类型   
    }

    uint16_t data_len = ((h[16] << 8) | h[17]) / 8;

    if (data_len != 0x02) {
        return -5;  // 数据长度小于预期
    }
    
    *out_word = (h[18] << 8) | h[19];

    return 0;  // 成功
}

static int s7_read_dword(int sock, uint8_t area, uint32_t address, uint32_t *out_word) 
{
    // 构造 S7 请求数据包
    uint8_t s7p[31];  // 包括 S7 header 和 ReadVar 请求
    // TPKT Header (4 bytes)
    s7p[0] = 0x03;
    s7p[1] = 0x0;
    s7p[2] = 0x0;
    s7p[3] = 0x1f;
    // COTP Header (3 bytes)
    s7p[4] = 0x02;
    s7p[5] = 0xf0;
    s7p[6] = 0x80;
    // S7 Header
    s7p[7] = 0x32;
    s7p[8] = 0x01; 
    s7p[9] = 0x00;
    s7p[10] = 0x00;
    s7p[11] = (s7dataunitreference >> 8) & 0xff;   //data unit reference high
    s7p[12] = s7dataunitreference  & 0xff;    //data unit reference low
    s7p[13] = 0x00;  
    s7p[14] = 0x0e;
    s7p[15] = 0x00;
    s7p[16] = 0x00;
    // Parameter
    s7p[17] = 0x04;   // Read Var 
    s7p[18] = 0x01;
    // Item
    s7p[19] = 0x12;
    s7p[20] = 0x0a;
    s7p[21] = 0x10;
    s7p[22] = 0x04;   // Data size: word
    s7p[23] = 0x00;
    s7p[24] = 0x02;   // Data length
    s7p[25] = 0x00;
    s7p[26] = 0x01;
    s7p[27] = area;   // Data Area

    address <<= 3;
    s7p[28] = (address >> 16) & 0xff;    // Data Address
    s7p[29] = (address >> 8) & 0xff; 
    s7p[30] = address & 0xff; 

    s7_begin_data_trans(sock);
    if (send_all(sock, s7p, sizeof(s7p)) != 0) {
        ESP_LOGE(TAG, "send failed, errno=%d", errno);
        return -1;
    }
    s7dataunitreference++;

    uint8_t rsp[1024];  // 接收响应
    int got = recv_some(sock, rsp, sizeof(rsp));  // 接收响应数据
    if (got != 0) {
        return -2;  // 数据长度错误
    }

    const uint8_t *h = rsp + 7;
    if (h[0] != 0x32) {
        return -3;  // 错误的响应
    }
    
    if (h[1] != 0x03) {
        return -4;  // 错误的响应类型   
    }

    uint16_t data_len = ((h[16] << 8) | h[17]) / 8;

    if (data_len != 0x04) {
        return -5;  // 数据长度小于预期
    }
    
    *out_word = (h[18] << 24) | (h[19] << 16) | (h[20] << 8) | h[21];

    return 0;  // 成功
}

// -------------------------- S7 通信函数 --------------------------

/**
 * @brief 创建与 PLC 的 TCP 连接，并初始化 S7 通信
 * 
 * @param ip 目标 PLC 的 IP 地址
 * @param local_tsap 本地 TSAP 地址
 * @param remote_tsap 远程 TSAP 地址
 * @param pdu_len 协商的 PDU 长度
 * @return int 成功时返回连接的 socket，失败时返回 -1
 */
int s7_connect(const char *ip, uint16_t *pdu_len) 
{
    struct sockaddr_in dest_addr;
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        return -1;
    }

    dest_addr.sin_addr.s_addr = inet_addr(ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(S7_TCP_PORT);

    // 尝试建立 TCP 连接
    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "TCP connection established");

    // 发送 COTP 连接请求（CR）并等待确认（CC）
    if (cotp_cr_cc(sock) != 0) {
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "COTP connection successful");

    // 设置 S7 通信参数
    return s7_setup_comm(sock, pdu_len);
}

int GetNetDataFromS7PlcNet(void)
{
	uint16_t datalen;
	int16_t tdata;
	uint8_t* data = NULL;
	static const char *TAG = "S7_GET_NET_DATA";
	int i, ret;

	if (s7netdatarecvflag != 1 || s7netsrdatabuffaddr == NULL || s7netsrdatalen == 0) {
		return -1;
	}

	data = (uint8_t *)s7netsrdatabuffaddr;
    if (g_fxplcdataformat == PLC_DATA_TYPE_BIT || g_fxplcdataformat == PLC_DATA_TYPE_CHAR_POINT_OR_BYTE_FOR_PPI) {
        ret = FXPLC_WriteBufferBytes(&data[0], 1);
        if (ret != 0) {
            ESP_LOGE(TAG, "buffer error: %d", ret);
            return -1;
        }
    } else if (g_fxplcdataformat == PLC_DATA_TYPE_SHORT) {
        for (i = 0; i < s7netsrdatalen * 2; i++) {
            ret = FXPLC_WriteBufferBytes(&data[i], 1);
            if (ret != 0) {
                ESP_LOGE(TAG, "buffer error: %d", ret);
                return -1;
            }
        }
    } else if (g_fxplcdataformat == PLC_DATA_TYPE_FLOAT || g_fxplcdataformat == PLC_DATA_TYPE_INT) {
        for (i = 0; i < s7netsrdatalen * 4; i++) {
            ret = FXPLC_WriteBufferBytes(&data[i], 1);
            if (ret != 0) {
                ESP_LOGE(TAG, "buffer error: %d", ret);
                return -1;
            }
        }
    }
	
	s7netdatarecvflag = 0;

	return 0;
}

void S7_NetReadBitDataRegister(int sock, uint32_t address, uint16_t bitoffset, uint16_t frnum)
{
    int i, err;
    uint8_t read_regs;
    static const char *TAG = "S7_READ_SINGLE_DATA";

    err = s7_read_bit_for_v(sock, S7_DATA_AREA_DB, address, bitoffset, &read_regs);
    if (err == 0) {
        ESP_LOGI(TAG, "Read bit regs: %u", read_regs);
    } else {
        ESP_LOGE(TAG, "Read bit regs failed: %d", err);
        // break;
    }
    
    S7_NetDataRecvNotice(&read_regs, 1, frnum, PLC_DATA_TYPE_BIT);
    vTaskDelay(S7_READ_DATA_TIME);
}

void S7_NetReadSingleDataRegister(int sock, uint32_t address, uint16_t frnum)
{
    int i, err;
    uint16_t read_regs;
    static const char *TAG = "S7_READ_SINGLE_DATA";

    err = s7_read_word(sock, S7_DATA_AREA_DB, address, &read_regs);
    if (err == 0) {
        ESP_LOGI(TAG, "Read 16-bit regs: %u", read_regs);
    } else {
        ESP_LOGE(TAG, "Read 16-bit regs failed: %d", err);
        // break;
    }
    
    S7_NetDataRecvNotice(&read_regs, 1, frnum, PLC_DATA_TYPE_SHORT);
    vTaskDelay(S7_READ_DATA_TIME);
}

void S7_NetReadIntDataRegister(int sock, uint32_t address, uint16_t frnum)
{
    int i, err;
    uint16_t read_regs;
    static const char *TAG = "S7_READ_SINGLE_DATA";

    err = s7_read_dword(sock, S7_DATA_AREA_DB, address, &read_regs);
    if (err == 0) {
        ESP_LOGI(TAG, "Read 16-bit regs: %u", read_regs);
    } else {
        ESP_LOGE(TAG, "Read 16-bit regs failed: %d", err);
        // break;
    }
    
    S7_NetDataRecvNotice(&read_regs, 1, frnum, PLC_DATA_TYPE_INT);
    vTaskDelay(S7_READ_DATA_TIME);
}

void S7_NetReadFloatDataRegister(int sock, uint32_t address, uint16_t frnum)
{
    int i, err;
    uint16_t read_regs;
    static const char *TAG = "S7_READ_SINGLE_DATA";

    err = s7_read_dword(sock, S7_DATA_AREA_DB, address, &read_regs);
    if (err == 0) {
        ESP_LOGI(TAG, "Read 16-bit regs: %u", read_regs);
    } else {
        ESP_LOGE(TAG, "Read 16-bit regs failed: %d", err);
        // break;
    }
    
    S7_NetDataRecvNotice(&read_regs, 1, frnum, PLC_DATA_TYPE_FLOAT);
    vTaskDelay(S7_READ_DATA_TIME);
}

void S7_NetReadBitMRegister(int sock, uint32_t address, uint16_t bitoffset, uint16_t frnum)
{
    int i, err;
    uint8_t read_regs;
    static const char *TAG = "S7_READ_SINGLE_DATA";

    err = s7_read_bit(sock, S7_DATA_AREA_MERKER, address, bitoffset, &read_regs);
    if (err == 0) {
        ESP_LOGI(TAG, "Read bit regs: %u", read_regs);
    } else {
        ESP_LOGE(TAG, "Read bit regs failed: %d", err);
        // break;
    }
    
    S7_NetDataRecvNotice(&read_regs, 1, frnum, PLC_DATA_TYPE_BIT);
    vTaskDelay(S7_READ_DATA_TIME);
}

void S7_NetReadBitQRegister(int sock, uint32_t address, uint16_t bitoffset, uint16_t frnum)
{
    int i, err;
    uint8_t read_regs;
    static const char *TAG = "S7_READ_SINGLE_DATA";

    err = s7_read_bit(sock, S7_DATA_AREA_OUTPUT, address, bitoffset, &read_regs);
    if (err == 0) {
        ESP_LOGI(TAG, "Read bit regs: %u", read_regs);
    } else {
        ESP_LOGE(TAG, "Read bit regs failed: %d", err);
        // break;
    }
    
    S7_NetDataRecvNotice(&read_regs, 1, frnum, PLC_DATA_TYPE_BIT);
    vTaskDelay(S7_READ_DATA_TIME);
}

