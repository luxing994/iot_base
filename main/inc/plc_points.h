#ifndef PLC_POINTS_H
#define PLC_POINTS_H

/*
 * 设备点表：每个数据点 = 读哪个寄存器 + 上报为第几个数据点（FR 编号）
 *
 * 点表定义在 plc_points.c，按 menuconfig 中选择的设备类型和连接方式选出当前使用的表；
 * plc_poll.c 按表逐点读取，plc_report.c 解码上报。
 */

#include <stddef.h>
#include <stdint.h>

typedef enum {
    /* 三菱 FX */
    FX_D16,         // D 寄存器，16 位整数
    FX_D32,         // D 寄存器，32 位整数（两个字）
    FX_DFLOAT,      // D 寄存器，32 位浮点
    FX_Y,           // Y 输出位
    FX_M,           // M 辅助继电器位

    /* 欧姆龙 HostLink */
    HL_DM,          // DM 区，16 位整数
    HL_DM_BCD,      // DM 区，BCD 码
    HL_DM_FLOAT,    // DM 区，32 位浮点
    HL_CIO_BIT,     // CIO 区的某一位（bit 字段指定位号）

    /* 西门子 S7-200（PPI 串口 / S7 网口），V 区即 DB1 */
    S7_VB,          // V 区字节（仅 PPI 串口）
    S7_VW,          // V 区字，16 位整数
    S7_VD,          // V 区双字，32 位整数
    S7_VREAL,       // V 区双字，32 位浮点
    S7_V_BIT,       // V 区位
    S7_Q_BIT,       // Q 输出位
    S7_I_BIT,       // I 输入位
    S7_M_BIT,       // M 位

    /* Modbus TCP 保持寄存器（地址从 0 开始） */
    MB_HR16,        // 16 位整数
    MB_HR32,        // 32 位整数（高字在前）
    MB_HRFLOAT,     // 32 位浮点（高字在前）
} PlcPointType;

typedef struct {
    uint8_t type;           // PlcPointType
    uint32_t address;       // 寄存器 / 字节地址
    uint8_t bit;            // 位类型使用的位号，其余类型填 0
    uint16_t frnum;         // 上报的数据点编号 FRnnn
    const char *name;       // 说明，仅用于日志和阅读
} PlcPoint;

typedef struct {
    const PlcPoint *points;
    size_t count;
} PlcPointTable;

/* 当前配置选中的点表（未启用的协议为空表） */
extern const PlcPointTable g_fxPoints;          // FX：串口或网口
extern const PlcPointTable g_hostLinkPoints;    // HostLink 串口
extern const PlcPointTable g_ppiPoints;         // PPI 串口
extern const PlcPointTable g_s7Points;          // S7 网口
extern const PlcPointTable g_modbusNetPoints;   // Modbus TCP

#endif
