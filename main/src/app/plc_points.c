/*
 * 设备点表
 *
 * 每行：{ 类型, 地址, 位号, FR 编号, "说明" }。类型说明见 plc_points.h。
 *
 * - Common 点表：对应 menuconfig 中的 "Common xxx device"，用于其他尚未单独适配的标准协议设备，
 *   接入新设备时直接修改这里（当前内容为测试数据）。
 * - 设备点表：已适配的具体设备，对应 menuconfig 中的设备类型。
 * - 参考点表：以往现场使用过的点表，未接入菜单；需要时把内容复制到 Common 点表。
 *
 * 新增一种设备：在这里加一张点表，在 Kconfig.projbuild 对应协议的 Device 中加一个选项，
 * 再在文件末尾的选择逻辑里加一行。
 */
#include "sdkconfig.h"
#include "plc_points.h"

#define POINT_TABLE     static const PlcPoint __attribute__((unused))
#define TABLE(t)        { (t), sizeof(t) / sizeof((t)[0]) }
#define EMPTY_TABLE     { NULL, 0 }

/* ====================================================================== 三菱 FX */

/* Common（RS485 计算机链接）—— 当前为测试数据：弯管机 */
POINT_TABLE s_fxCommonRs485[] = {
    { FX_Y,   30,  0, 1, "报警状态" },
    { FX_Y,   31,  0, 2, "自动运行" },
    { FX_Y,   32,  0, 3, "停止" },
    { FX_D32, 214, 0, 4, "当前产量" },
    { FX_D32, 216, 0, 5, "产量设定" },
    { FX_D32, 218, 0, 6, "总产量" },
    { FX_D32, 94,  0, 7, "每分钟循环次数" },
};

/* Common（RS232 编程口，只支持读 D 寄存器）—— 当前为测试数据 */
POINT_TABLE s_fxCommonRs232[] = {
    { FX_D16, 8000, 0, 1, "D8000" },
};

/* Common（网口 MC 协议，支持 D16 / D32 / Y）—— 当前为测试数据：高速冲床 */
POINT_TABLE s_fxCommonNet[] = {
    { FX_Y,   94,  0, 1, "正常运行状态" },
    { FX_Y,   93,  0, 2, "暂停状态" },
    { FX_Y,   92,  0, 3, "异常停止状态" },
    { FX_D16, 214, 0, 4, "白班生产时间" },
    { FX_D16, 215, 0, 5, "晚班生产时间" },
    { FX_D32, 30,  0, 6, "总累计冲次" },
    { FX_D16, 10,  0, 7, "实时冲速" },
    { FX_D16, 101, 0, 8, "当前累积生产数量" },
};

/* 日跃产线（RS485，值变化才上报） */
POINT_TABLE s_fxRiYueLine[] = {
    { FX_D16, 232, 0, 1, "产量累加值" },
};

/* 行车控制器（RS485，值变化才上报） */
POINT_TABLE s_fxCraneController[] = {
    { FX_D16, 180, 0, 1,  "行车总时间" },
    { FX_D16, 181, 0, 2,  "行车设定时间" },
    { FX_D16, 171, 0, 3,  "一车运行时间" },
    { FX_D16, 172, 0, 4,  "二车运行时间" },
    { FX_D16, 173, 0, 5,  "三车运行时间" },
    { FX_D16, 174, 0, 6,  "四车运行时间" },
    { FX_D16, 500, 0, 7,  "一车工位数值" },
    { FX_D16, 550, 0, 8,  "二车工位数值" },
    { FX_D16, 600, 0, 9,  "三车工位数值" },
    { FX_D16, 650, 0, 10, "四车工位数值" },
};

/* 生能 2-4 车间冷媒灌注机（RS485，10 个点收齐后批量上报，点表顺序与 plc_report.c 中批量格式对应） */
POINT_TABLE s_fxSnRefillMachine[] = {
    { FX_DFLOAT, 164,  0, 1,  "系统真空" },
    { FX_D16,    31,   0, 2,  "冷媒温度（A系统）" },
    { FX_D16,    32,   0, 3,  "冷媒温度（B系统）" },
    { FX_D16,    200,  0, 4,  "系统压力（A系统）" },
    { FX_D16,    210,  0, 5,  "系统压力（B系统）" },
    { FX_DFLOAT, 512,  0, 6,  "灌注量" },
    { FX_D16,    7974, 0, 7,  "单班产量" },
    { FX_D16,    7982, 0, 8,  "总产量" },
    { FX_D16,    112,  0, 9,  "测试结果：4=真空合格 10=真空不合格 52=充注合格" },
    { FX_DFLOAT, 16,   0, 10, "实际设定灌注量" },
};

/* ---- 参考点表（RS485） ---- */

/* 高速冲床（换热车间） */
POINT_TABLE s_fxRefPunchPress[] = {
    { FX_Y,   94,  0, 1, "正常运行状态" },
    { FX_Y,   93,  0, 2, "暂停状态" },
    { FX_Y,   92,  0, 3, "异常停止状态" },
    { FX_D16, 214, 0, 4, "白班生产时间" },
    { FX_D16, 215, 0, 5, "晚班生产时间" },
    { FX_D32, 30,  0, 6, "总累计冲次" },
    { FX_D16, 10,  0, 7, "实时冲速" },
    { FX_D16, 101, 0, 8, "当前累积生产数量" },
};

/* 双两管无削开料机 */
POINT_TABLE s_fxRefTubeCutter[] = {
    { FX_D16, 8340, 0, 1,  "位置（自动1）" },
    { FX_D16, 132,  0, 2,  "长度（自动1）" },
    { FX_D16, 130,  0, 3,  "次数（自动1）" },
    { FX_D16, 134,  0, 4,  "切割（自动1）" },
    { FX_D16, 144,  0, 5,  "总计数值（自动1）" },
    { FX_D16, 140,  0, 6,  "计数值（自动1）" },
    { FX_D16, 138,  0, 7,  "设定值（自动1）" },
    { FX_D16, 8350, 0, 8,  "位置（自动2）" },
    { FX_D16, 182,  0, 9,  "长度（自动2）" },
    { FX_D16, 180,  0, 10, "次数（自动2）" },
    { FX_D16, 184,  0, 11, "切割（自动2）" },
    { FX_D16, 194,  0, 12, "总计数值（自动2）" },
    { FX_D16, 190,  0, 13, "计数值（自动2）" },
    { FX_D16, 188,  0, 14, "设定值（自动2）" },
};

/* 毛细管下料机（AMAJQ0205） */
POINT_TABLE s_fxRefCapillaryCutter0205[] = {
    { FX_M,   222,  0, 1,  "运行/停止（A1组）" },
    { FX_D16, 1030, 0, 2,  "设定（A1组）" },
    { FX_D16, 224,  0, 3,  "送料（A1组）" },
    { FX_D16, 610,  0, 4,  "检测（A1组）" },
    { FX_D16, 290,  0, 5,  "记数值（A1组）" },
    { FX_D16, 294,  0, 6,  "设定值（A1组）" },
    { FX_M,   223,  0, 7,  "运行/停止（A2组）" },
    { FX_D16, 1040, 0, 8,  "设定（A2组）" },
    { FX_D16, 274,  0, 9,  "送料（A2组）" },
    { FX_D16, 660,  0, 10, "检测（A2组）" },
    { FX_D16, 292,  0, 11, "记数值（A2组）" },
    { FX_D16, 296,  0, 12, "设定值（A2组）" },
    { FX_M,   224,  0, 13, "运行/停止（B1组）" },
    { FX_D16, 530,  0, 14, "设定（B1组）" },
    { FX_D16, 320,  0, 15, "送料（B1组）" },
    { FX_D16, 640,  0, 16, "检测（B1组）" },
    { FX_D16, 390,  0, 17, "记数值（B1组）" },
    { FX_D16, 394,  0, 18, "设定值（B1组）" },
    { FX_M,   225,  0, 19, "运行/停止（B2组）" },
    { FX_D16, 540,  0, 20, "设定（B2组）" },
    { FX_D16, 130,  0, 21, "送料（B2组）" },
    { FX_D16, 644,  0, 22, "检测（B2组）" },
    { FX_D16, 392,  0, 23, "记数值（B2组）" },
    { FX_D16, 396,  0, 24, "设定值（B2组）" },
};

/* 毛细管下料机（AMAJQ0252） */
POINT_TABLE s_fxRefCapillaryCutter0252[] = {
    { FX_Y,   34,  0, 1,  "运行状态" },
    { FX_Y,   35,  0, 2,  "停止状态" },
    { FX_Y,   36,  0, 3,  "故障状态" },
    { FX_D16, 908, 0, 4,  "总计数（A1组）" },
    { FX_D16, 916, 0, 5,  "设定值（A1组）" },
    { FX_D16, 900, 0, 6,  "计数值（A1组）" },
    { FX_D16, 300, 0, 7,  "开料总长（A1组）" },
    { FX_D16, 306, 0, 8,  "打凸长度补偿（A1组）" },
    { FX_D16, 312, 0, 9,  "长度修正校验（A1组）" },
    { FX_D16, 314, 0, 10, "长度修正不校验（A1组）" },
    { FX_M,   770, 0, 11, "打凸设定（A1组）" },
    { FX_D16, 910, 0, 12, "总计数（A2组）" },
    { FX_D16, 918, 0, 13, "设定值（A2组）" },
    { FX_D16, 902, 0, 14, "计数值（A2组）" },
    { FX_D16, 350, 0, 15, "开料总长（A2组）" },
    { FX_D16, 356, 0, 16, "打凸长度补偿（A2组）" },
    { FX_D16, 362, 0, 17, "长度修正校验（A2组）" },
    { FX_D16, 354, 0, 18, "长度修正不校验（A2组）" },
    { FX_M,   771, 0, 19, "打凸设定（A2组）" },
    { FX_D16, 912, 0, 20, "总计数（B1组）" },
    { FX_D16, 920, 0, 21, "设定值（B1组）" },
    { FX_D16, 904, 0, 22, "计数值（B1组）" },
    { FX_D16, 600, 0, 23, "开料总长（B1组）" },
    { FX_D16, 606, 0, 24, "打凸长度补偿（B1组）" },
    { FX_D16, 612, 0, 25, "长度修正校验（B1组）" },
    { FX_D16, 614, 0, 26, "长度修正不校验（B1组）" },
    { FX_M,   772, 0, 27, "打凸设定（B1组）" },
    { FX_D16, 914, 0, 28, "总计数（B2组）" },
    { FX_D16, 922, 0, 29, "设定值（B2组）" },
    { FX_D16, 906, 0, 30, "计数值（B2组）" },
    { FX_D16, 650, 0, 31, "开料总长（B2组）" },
    { FX_D16, 656, 0, 32, "打凸长度补偿（B2组）" },
    { FX_D16, 662, 0, 33, "长度修正校验（B2组）" },
    { FX_D16, 664, 0, 34, "长度修正不校验（B2组）" },
    { FX_M,   773, 0, 35, "打凸设定（B2组）" },
};

/* 开料机 */
POINT_TABLE s_fxRefCuttingMachine[] = {
    { FX_M,      40,   0, 1,  "工作状态（1号）" },
    { FX_M,      0,    0, 2,  "故障状态（1号）" },
    { FX_D16,    324,  0, 3,  "设定量（1号）" },
    { FX_D16,    340,  0, 4,  "计件量（1号）" },
    { FX_DFLOAT, 350,  0, 5,  "产品加工长度（1号）" },
    { FX_DFLOAT, 8360, 0, 6,  "扩口当前位置（1号）" },
    { FX_DFLOAT, 8340, 0, 7,  "送料当前位置（1号）" },
    { FX_M,      90,   0, 8,  "工作状态（2号）" },
    { FX_M,      50,   0, 9,  "故障状态（2号）" },
    { FX_D16,    524,  0, 10, "设定量（2号）" },
    { FX_D16,    540,  0, 11, "计件量（2号）" },
    { FX_DFLOAT, 352,  0, 12, "产品加工长度（2号）" },
    { FX_DFLOAT, 2520, 0, 13, "扩口当前位置（2号）" },
    { FX_DFLOAT, 8350, 0, 14, "送料当前位置（2号）" },
};

/* ============================================================== 欧姆龙 HostLink */

/* Common —— 当前为测试数据：高速冲床 CIO 位 */
POINT_TABLE s_hostLinkCommon[] = {
    // { HL_DM, 12,  0, 1, "总累计冲次" },
    // { HL_DM, 101, 0, 2, "当班累计生产数量" },
    { HL_CIO_BIT, 101, 5, 1, "CIO101.05" },
    { HL_CIO_BIT, 101, 4, 2, "CIO101.04" },
    { HL_CIO_BIT, 102, 2, 3, "CIO102.02" },
    { HL_CIO_BIT, 102, 3, 4, "CIO102.03" },
    { HL_CIO_BIT, 101, 2, 5, "CIO101.02" },
    { HL_CIO_BIT, 101, 3, 6, "CIO101.03" },
    { HL_CIO_BIT, 100, 0, 7, "CIO100.00" },
    { HL_CIO_BIT, 100, 1, 8, "CIO100.01" },
};

/* 生能 1 车间冷媒灌注机（27 个点收齐后批量上报，点表顺序与 plc_report.c 中批量格式对应） */
POINT_TABLE s_hostLinkSnRefillMachine[] = {
    { HL_DM,       8101,  0, 1,  "A模式选择" },
    { HL_DM,       8501,  0, 2,  "B模式选择" },
    { HL_DM_BCD,   8824,  0, 3,  "A检测时间" },
    { HL_DM_BCD,   8844,  0, 4,  "B检测时间" },
    { HL_DM_FLOAT, 8820,  0, 5,  "A抽空上限" },
    { HL_DM_FLOAT, 8840,  0, 6,  "B抽空上限" },
    { HL_DM_FLOAT, 8822,  0, 7,  "A抽空下限" },
    { HL_DM_FLOAT, 8842,  0, 8,  "B抽空下限" },
    { HL_DM_FLOAT, 19704, 0, 9,  "A充注压力" },
    { HL_DM_FLOAT, 19724, 0, 10, "B充注压力" },
    { HL_DM_FLOAT, 19746, 0, 11, "A真空度" },
    { HL_DM_FLOAT, 19766, 0, 12, "B真空度" },
    { HL_DM_FLOAT, 8828,  0, 13, "A设定量" },
    { HL_DM_FLOAT, 8848,  0, 14, "B设定量" },
    { HL_DM_FLOAT, 8182,  0, 15, "A百分比" },
    { HL_DM_FLOAT, 8582,  0, 16, "B百分比" },
    { HL_DM_FLOAT, 8194,  0, 17, "A充注速度" },
    { HL_DM_FLOAT, 8594,  0, 18, "B充注速度" },
    { HL_DM_FLOAT, 8180,  0, 19, "A充注量" },
    { HL_DM_FLOAT, 8580,  0, 20, "B充注量" },
    { HL_DM,       8102,  0, 21, "A工作状态" },
    { HL_DM,       8502,  0, 22, "B工作状态" },
    { HL_DM_FLOAT, 8104,  0, 23, "A充注时间" },
    { HL_DM_FLOAT, 8504,  0, 24, "B充注时间" },
    { HL_DM,       8940,  0, 25, "结果判定" },
    { HL_DM_FLOAT, 8150,  0, 26, "A充注设定值" },
    { HL_DM_FLOAT, 8550,  0, 27, "B充注设定值" },
};

/* ========================================================= 西门子 S7-200（PPI 串口） */

/* Common —— 当前为测试数据 */
POINT_TABLE s_ppiCommon[] = {
    { S7_VB,    120,  0, 1, "VB120" },
    // { S7_VB,    101,  0, 2, "VB101" },
    // { S7_VB,    102,  0, 3, "VB102" },
    // { S7_I_BIT, 1,    7, 1, "I1.7" },
    { S7_I_BIT, 2,    3, 2, "I2.3" },
    // { S7_Q_BIT, 1,    6, 3, "Q1.6" },
    // { S7_Q_BIT, 2,    0, 4, "Q2.0" },
    { S7_M_BIT, 2,    3, 3, "M2.3" },
    { S7_V_BIT, 3315, 1, 4, "V3315.1" },
    { S7_VREAL, 100,  0, 5, "VD100" },
    { S7_VW,    520,  0, 6, "VW520" },
};

/* ---- 参考点表 ---- */

/* 电解炉电解柜（PPI 串口版本） */
POINT_TABLE s_ppiRefElectrolysisCabinet[] = {
    { S7_Q_BIT, 0,    2, 1,  "运行显示" },
    { S7_M_BIT, 2,    0, 2,  "启动" },
    { S7_M_BIT, 2,    3, 3,  "故障复位" },
    { S7_V_BIT, 3315, 1, 4,  "电源输入故障" },
    { S7_V_BIT, 3315, 2, 5,  "设备超温" },
    { S7_V_BIT, 3315, 3, 6,  "输出过电压" },
    { S7_V_BIT, 3315, 4, 7,  "输出过电流" },
    { S7_V_BIT, 3315, 5, 8,  "IGBT保护" },
    { S7_V_BIT, 3315, 6, 9,  "输出低电压" },
    { S7_V_BIT, 3315, 7, 10, "风扇变频故障" },
    { S7_V_BIT, 3316, 1, 11, "故障显示" },
    { S7_M_BIT, 0,    7, 12, "急停" },
    { S7_VREAL, 210,  0, 13, "实时电压" },
    { S7_VREAL, 214,  0, 14, "实时电流" },
    { S7_VREAL, 280,  0, 15, "设备温度" },
    { S7_VREAL, 534,  0, 16, "电解AH" },
    { S7_VREAL, 1648, 0, 17, "电压给定" },
    { S7_VREAL, 1848, 0, 18, "电流给定" },
    { S7_VREAL, 900,  0, 19, "批号" },
    { S7_VREAL, 100,  0, 20, "VD100" },
};

/* ============================================================ 西门子 S7-200（S7 网口） */

/* 电解炉电解柜 */
POINT_TABLE s_s7ElectrolysisCabinet[] = {
    { S7_Q_BIT, 0,    2, 1,  "运行显示" },
    { S7_M_BIT, 2,    0, 2,  "启动" },
    { S7_M_BIT, 2,    3, 3,  "故障复位" },
    { S7_V_BIT, 3315, 1, 4,  "电源输入故障" },
    { S7_V_BIT, 3315, 2, 5,  "设备超温" },
    { S7_V_BIT, 3315, 3, 6,  "输出过电压" },
    { S7_V_BIT, 3315, 4, 7,  "输出过电流" },
    { S7_V_BIT, 3315, 5, 8,  "IGBT保护" },
    { S7_V_BIT, 3315, 6, 9,  "输出低电压" },
    { S7_V_BIT, 3315, 7, 10, "风扇变频故障" },
    { S7_V_BIT, 3316, 1, 11, "故障显示" },
    { S7_M_BIT, 0,    7, 12, "急停" },
    { S7_VREAL, 210,  0, 13, "实时电压" },
    { S7_VREAL, 214,  0, 14, "实时电流" },
    { S7_VREAL, 280,  0, 15, "设备温度" },
    { S7_VREAL, 534,  0, 16, "电解AH" },
    { S7_VREAL, 1648, 0, 17, "电压给定" },
    { S7_VREAL, 1848, 0, 18, "电流给定" },
    { S7_VD,    900,  0, 19, "批号" },
    { S7_VW,    520,  0, 20, "电解时间H" },
    { S7_VW,    522,  0, 21, "电解时间M" },
};

/* 电解炉控制柜（FR016、FR021 未使用；FR023 起为同一现场 Modbus TCP 的数据） */
POINT_TABLE s_s7FurnaceController[] = {
    { S7_Q_BIT, 8,   3, 1,  "前置阀1开到位指示" },
    { S7_Q_BIT, 8,   4, 2,  "前置阀1关到位指示" },
    { S7_Q_BIT, 8,   5, 3,  "前置阀2开到位指示" },
    { S7_Q_BIT, 8,   6, 4,  "前置阀2关到位指示" },
    { S7_Q_BIT, 0,   7, 5,  "旁通阀输出" },
    { S7_Q_BIT, 8,   7, 6,  "粗抽阀1（左）开到位指示" },
    { S7_Q_BIT, 9,   0, 7,  "粗抽阀1（左）关到位指示" },
    { S7_Q_BIT, 9,   1, 8,  "粗抽阀2（左）开到位指示" },
    { S7_Q_BIT, 9,   2, 9,  "粗抽阀2（左）关到位指示" },
    { S7_Q_BIT, 9,   3, 10, "粗抽阀1（右）开到位指示" },
    { S7_Q_BIT, 9,   4, 11, "粗抽阀1（右）关到位指示" },
    { S7_Q_BIT, 9,   5, 12, "粗抽阀2（右）开到位指示" },
    { S7_Q_BIT, 9,   6, 13, "粗抽阀2（右）关到位指示" },
    { S7_Q_BIT, 0,   0, 14, "H-150滑阀泵输出" },
    { S7_Q_BIT, 0,   1, 15, "ZJP-1200罗茨泵输出" },
    { S7_I_BIT, 9,   0, 17, "插板阀开启到位(左)" },
    { S7_I_BIT, 9,   1, 18, "插板阀关闭到位(左)" },
    { S7_I_BIT, 9,   2, 19, "插板阀开启到位(右)" },
    { S7_I_BIT, 9,   3, 20, "插板阀关闭到位(右)" },
    { S7_VREAL, 100, 0, 22, "冷却水压力" },
};

/* ================================================================== Modbus TCP */

/* 电解炉真空 / 压力 / 水温（与 S7 电解炉控制柜同一现场，FR 编号接在其后） */
POINT_TABLE s_modbusFurnaceVacuum[] = {
    { MB_HRFLOAT, 5,  0, 23, "11#工作室真空度" },
    { MB_HRFLOAT, 7,  0, 24, "11#收集室真空度" },
    { MB_HRFLOAT, 13, 0, 25, "11#工作室压力值" },
    { MB_HRFLOAT, 15, 0, 26, "11#收集室压力值" },
    { MB_HRFLOAT, 19, 0, 27, "11#炉体水温" },
    { MB_HRFLOAT, 1,  0, 28, "12#工作室真空度" },
    { MB_HRFLOAT, 3,  0, 29, "12#收集室真空度" },
    { MB_HRFLOAT, 9,  0, 30, "12#工作室压力值" },
    { MB_HRFLOAT, 11, 0, 31, "12#收集室压力值" },
    { MB_HRFLOAT, 17, 0, 32, "12#炉体水温" },
};

/* ---- 参考点表 ---- */

/* 无纸记录仪（Modbus TCP 版本） */
POINT_TABLE s_modbusRefPaperlessRecorder[] = {
    { MB_HRFLOAT, 0x2000, 0, 1, "1温区温度" },
    { MB_HRFLOAT, 0x2002, 0, 2, "2温区温度" },
    { MB_HRFLOAT, 0x2004, 0, 3, "3温区温度" },
    { MB_HRFLOAT, 0x2006, 0, 4, "4温区温度" },
    { MB_HRFLOAT, 0x2008, 0, 5, "5温区温度" },
    { MB_HRFLOAT, 0x200A, 0, 6, "6温区温度" },
};

/* ======================================================= 按 menuconfig 选择点表 */

const PlcPointTable g_fxPoints =
#if defined(CONFIG_PLC_FX_CONNECT_RS485)
  #if defined(CONFIG_PLC_FX_RY_LINE)
    TABLE(s_fxRiYueLine);
  #elif defined(CONFIG_PLC_FX_XF_CONTROLLER)
    TABLE(s_fxCraneController);
  #elif defined(CONFIG_PLC_FX_SN_CACLREFILMAC)
    TABLE(s_fxSnRefillMachine);
  #else
    TABLE(s_fxCommonRs485);
  #endif
#elif defined(CONFIG_PLC_FX_CONNECT_RS232)
    TABLE(s_fxCommonRs232);
#elif defined(CONFIG_PLC_FX_CONNECT_NETWORK)
    TABLE(s_fxCommonNet);
#else
    EMPTY_TABLE;
#endif

const PlcPointTable g_hostLinkPoints =
#if defined(CONFIG_PLC_HOSTLINK_SN_CACLREFILMAC)
    TABLE(s_hostLinkSnRefillMachine);
#elif defined(CONFIG_PLC_HOSTLINK)
    TABLE(s_hostLinkCommon);
#else
    EMPTY_TABLE;
#endif

const PlcPointTable g_ppiPoints =
#if defined(CONFIG_PLC_PPI_CONNECT_RS485)
    TABLE(s_ppiCommon);
#else
    EMPTY_TABLE;
#endif

const PlcPointTable g_s7Points =
#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK) && defined(CONFIG_PLC_PPI_ELECTROLYSIS_DEVICE)
    TABLE(s_s7ElectrolysisCabinet);
#elif defined(CONFIG_PLC_PPI_CONNECT_NETWORK) && defined(CONFIG_PLC_PPI_CONTROLLER_DEVICE)
    TABLE(s_s7FurnaceController);
#else
    EMPTY_TABLE;    // Common S7 设备：按现场在上方新增点表后在这里选用
#endif

const PlcPointTable g_modbusNetPoints =
#if defined(CONFIG_PLC_MODBUS_TCP_FURNACE_VACUUM)
    TABLE(s_modbusFurnaceVacuum);
#else
    EMPTY_TABLE;    // Common Modbus TCP 设备：按现场在上方新增点表后在这里选用
#endif
