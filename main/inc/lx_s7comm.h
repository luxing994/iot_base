#ifndef S7COMM_ESP32_H
#define S7COMM_ESP32_H

#include <stdint.h>

// -------------------------- 常量定义 --------------------------
#define S7_TCP_PORT 102                      // S7comm 使用的 TCP 端口
#define TPKT_VERSION 0x03                    // TPKT 协议版本
#define COTP_CR_PDU_TYPE 0xE0                // COTP Connection Request (CR) 类型
#define COTP_CC_PDU_TYPE 0xD0                // COTP Confirm (CC) 类型
#define S7_PROTOCOL_ID 0x32                  // S7 协议 ID
#define S7_COMMUNICATION_FUNCTION 0x07       // S7 Setup Communication 功能码
#define S7_READ_VAR_FUNCTION 0x04            // S7 ReadVar 功能码

// S7 数据区类型编号
#define S7_DATA_AREA_INPUT    0x81    // I 区（Input）：用于输入信号（通常是数字量输入）
#define S7_DATA_AREA_OUTPUT   0x82    // Q 区（Output）：用于输出信号（通常是数字量输出）
#define S7_DATA_AREA_MERKER   0x83    // M 区（Merker）：用于内存标志（通常用于状态标志）
#define S7_DATA_AREA_DB       0x84    // DB 区（Data Block）：用于数据块（DB1、DB2 等）
#define S7_DATA_AREA_TDB      0x85    // TDB 区（Temporary Data Block）：用于临时数据块
#define S7_DATA_AREA_COUNTER  0x86    // C 区（Counter）：用于计数器（TC）
#define S7_DATA_AREA_TIMER    0x87    // T 区（Timer）：用于定时器（TON、TOF）

#define S7_READ_DATA_TIME     10

// -------------------------- 函数声明 --------------------------

/**
 * @brief 创建与 PLC 的 TCP 连接，并初始化 S7 通信
 * 
 * @param ip 目标 PLC 的 IP 地址
 * @param local_tsap 本地 TSAP 地址
 * @param remote_tsap 远程 TSAP 地址
 * @param pdu_len 协商的 PDU 长度
 * @return int 成功时返回连接的 socket，失败时返回 -1
 */
int s7_connect(const char *ip, uint16_t port, uint16_t *pdu_len);
int GetNetDataFromS7PlcNet(void);
int S7_NetReadBitDataRegister(int sock, uint32_t address, uint16_t bitoffset, uint16_t frnum);
int S7_NetReadBitMRegister(int sock, uint32_t address, uint16_t bitoffset, uint16_t frnum);
int S7_NetReadBitIRegister(int sock, uint32_t address, uint16_t bitoffset, uint16_t frnum);
int S7_NetReadBitQRegister(int sock, uint32_t address, uint16_t bitoffset, uint16_t frnum);
int S7_NetReadSingleDataRegister(int sock, uint32_t address, uint16_t frnum);
int S7_NetReadFloatDataRegister(int sock, uint32_t address, uint16_t frnum);
int S7_NetReadIntDataRegister(int sock, uint32_t address, uint16_t frnum);

#endif // S7COMM_ESP32_H
