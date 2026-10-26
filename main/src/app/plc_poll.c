/*
 * 设备寄存器点表与轮询任务
 *
 * 每种现场设备要读哪些寄存器、对应哪个数据点编号（FRnnn）都集中在这里：
 *   plc_poll_task       串口 PLC / 测试仪 / Modbus RTU，每秒轮询一次
 *   PlcPoll_xxxNet()    网口 PLC，由 tcp_client.c 中的连接任务每秒调用一次
 *
 * 新增一台设备：在 Kconfig 中增加 device type，然后在对应位置按
 *   读寄存器函数(地址, ..., FR编号);   // 注释说明含义
 * 的格式追加即可，数据会由 plc_report.c 解码后自动上报。
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "iot_common.h"
#include "plc_poll.h"
#include "host_cmd.h"
#include "fx_plc_protocol.h"
#include "hl_plc_protocol.h"
#include "ppi_plc_protocol.h"
#include "ls_plc_load_protocol.h"
#include "t_tester_protocol.h"
#include "ainuo_tester_ascii_protocol.h"
#include "lx_modbus_tcp_master.h"
#include "lx_s7comm.h"
#include "master.h"
#include "serial_txn.h"
#include "plc_report.h"

/* ============================================================ 串口 / Modbus RTU */

/*
 * 每个串口数据点都是一次同步事务：发请求 -> 等完整应答 -> 解码上报（见 serial_txn.h）。
 * 点表中的读请求函数只负责发送并记录 FR 编号 / 数据格式，用下面的宏包装。
 */
#if defined(CONFIG_PLC_FX_CONNECT_RS232)
#define FX_SERIAL_PARSE   GetDataFromFxPlc          // 编程口
#else
#define FX_SERIAL_PARSE   GetSerialDataFromFxPlc    // 485 计算机链接
#endif

static __attribute__((unused)) void FxOnResponse(void)
{
    SendAckToPlc();
    PlcReport_Fx();
}

#if defined(CONFIG_TESTER_76T) || defined(CONFIG_TESTER_AINUO)
static char s_testerJson[2048];

static void PostTesterJson(void (*get_json)(char *str))
{
    get_json(s_testerJson);
    IOT_PostMessage(s_testerJson);
}
#endif

#define FX_POINT(request)      SERIAL_TXN(request, FX_SERIAL_PARSE, SERIAL_TXN_TIMEOUT_MS, FxOnResponse())
#define HL_POINT(request)      SERIAL_TXN(request, GetSerialWordDataFromHlPlc, SERIAL_TXN_TIMEOUT_MS, PlcReport_HostLink())
#define PPI_POINT(request)     SERIAL_TXN(request, GetSerialWordDataFromPpiPlc, SERIAL_TXN_TIMEOUT_MS, PlcReport_Ppi())
#define LS_POINT(request)      SERIAL_TXN(request, DBSGetData, SERIAL_TXN_TIMEOUT_MS, PlcReport_Ls())
#define TESTER_76T_POINT(request) \
    SERIAL_TXN(request, TTesterResolve, SERIAL_TXN_TESTER_TIMEOUT_MS, PostTesterJson(TTesterGetJsonData))
#define TESTER_AINUO_POINT(request) \
    SERIAL_TXN(request, AINUO_TTesterResolve, SERIAL_TXN_TESTER_TIMEOUT_MS, PostTesterJson(AINUO_TTesterGetJsonData))

void plc_poll_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (!HostCmd_IsCollectEnabled()) {
            // 上位机发送 EndFlush 后暂停采集，StartFlush 后恢复
        } else {
#ifdef CONFIG_PLC_FX_CONNECT_RS232
            FX_POINT(ReadSingleDataRegister(8000, 1));
#endif

#ifdef CONFIG_PLC_FX_CONNECT_RS485
        #ifdef CONFIG_PLC_FX_RY_LINE
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 232, 1)); // 日跃PLC产量累加值寄存器
        #endif

        #ifdef CONFIG_PLC_FX_XF_CONTROLLER
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 180, 1));    //  行车总时间
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 181, 2));    //  行车设定时间
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 171, 3));    //  一车运行时间
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 172, 4));    //  二车运行时间
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 173, 5));    //  三车运行时间
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 174, 6));    //  四车运行时间
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 500, 7));    //  一车工位数值
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 550, 8));    //  二车工位数值
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 600, 9));    //  三车工位数值
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 650, 10));    //  四车工位数值
        #endif

        #ifdef CONFIG_PLC_FX_SN_CACLREFILMAC
            FX_POINT(SerialReadSingleFloatDataRegister(0, 255, 10, 164, 1));   //  系统真空
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 31, 2));         //  冷媒温度（A系统）
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 32, 3));         //  冷媒温度（B系统）
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 200, 4));        //  系统压力（A系统）
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 210, 5));        //  系统压力（B系统）
            FX_POINT(SerialReadSingleFloatDataRegister(0, 255, 10, 512, 6));   //  灌注量
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 7974, 7));       //  单班产量
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 7982, 8));       //  总产量
            FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 112, 9));        //  测试结果:4=真空合格 10=真空不合格 52：充注合格
            FX_POINT(SerialReadSingleFloatDataRegister(0, 255, 10, 16, 10));   //  实际设定灌注量
        #endif

        #ifdef CONFIG_PLC_FX_OTHER_DEVICE
            // 弯管机
            FX_POINT(SerialReadSingleYDataRegister(0, 255, 10, 30, 1));        // 报警状态
            FX_POINT(SerialReadSingleYDataRegister(0, 255, 10, 31, 2));        // 自动运行
            FX_POINT(SerialReadSingleYDataRegister(0, 255, 10, 32, 3));        // 停止
            FX_POINT(SerialReadDoubleDataRegister(0, 255, 10, 214, 4));        // 当前产量
            FX_POINT(SerialReadDoubleDataRegister(0, 255, 10, 216, 5));        // 产量设定
            FX_POINT(SerialReadDoubleDataRegister(0, 255, 10, 218, 6));        // 总产量
            FX_POINT(SerialReadDoubleDataRegister(0, 255, 10, 94, 7));         // 每分钟循环次数

            // 高速冲床（换热车间）
            // FX_POINT(SerialReadSingleYDataRegister(0, 255, 10, 94, 1));        // 正常运行状态
            // FX_POINT(SerialReadSingleYDataRegister(0, 255, 10, 93, 2));        // 暂停状态
            // FX_POINT(SerialReadSingleYDataRegister(0, 255, 10, 92, 3));        // 异常停止状态
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 214, 4));        // 白班生产时间
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 215, 5));        // 晚班生产时间
            // FX_POINT(SerialReadDoubleDataRegister(0, 255, 10, 30, 6));         // 总累计冲次
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 10, 7));         // 实时冲速 
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 101, 8));        // 当前累积生产数量

            // 双两管无削开料机
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 8340, 1));        // 位置（自动1）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 132, 2));         // 长度（自动1）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 130, 3));         // 次数（自动1）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 134, 4));         // 切割（自动1）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 144, 5));         // 总计数值（自动1）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 140, 6));         // 计数值（自动1）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 138, 7));         // 设定值（自动1）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 8350, 8));        // 位置（自动2）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 182, 9));         // 长度（自动2）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 180, 10));        // 次数（自动2）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 184, 11));        // 切割（自动2）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 194, 12));        // 总计数值（自动2）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 190, 13));        // 计数值（自动2）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 188, 14));        // 设定值（自动2）
            
            // 毛细管下料机（AMAJQ0205）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 222, 1));           // 运行/停止（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 1030, 2));           // 设定（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 224, 3));            // 送料（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 610, 4));            // 检测（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 290, 5));            // 记数值（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 294, 6));            // 设定值（A1组）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 223, 7));           // 运行/停止（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 1040, 8));           // 设定（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 274, 9));            // 送料（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 660, 10));           // 检测（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 292, 11));           // 记数值（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 296, 12));           // 设定值（A2组）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 224, 13));          // 运行/停止（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 530, 14));           // 设定（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 320, 15));           // 送料（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 640, 16));           // 检测（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 390, 17));           // 记数值（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 394, 18));           // 设定值（B1组）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 225, 19));          // 运行/停止（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 540, 20));           // 设定（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 130, 21));           // 送料（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 644, 22));           // 检测（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 392, 23));           // 记数值（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 396, 24));           // 设定值（B2组）

            // 毛细管下料机（AMAJQ0252）
            // FX_POINT(SerialReadSingleYDataRegister(0, 255, 10, 34, 1));            // 运行状态
            // FX_POINT(SerialReadSingleYDataRegister(0, 255, 10, 35, 2));            // 停止状态
            // FX_POINT(SerialReadSingleYDataRegister(0, 255, 10, 36, 3));            // 故障状态
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 908, 4));            // 总计数（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 916, 5));            // 设定值（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 900, 6));            // 计数值（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 300, 7));            // 开料总长（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 306, 8));            // 打凸长度补偿（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 312, 9));            // 长度修正校验（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 314, 10));            // 长度修正不校验（A1组）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 770, 11));           // 打凸设定（A1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 910, 12));            // 总计数（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 918, 13));            // 设定值（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 902, 14));            // 计数值（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 350, 15));            // 开料总长（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 356, 16));            // 打凸长度补偿（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 362, 17));            // 长度修正校验（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 354, 18));            // 长度修正不校验（A2组）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 771, 19));           // 打凸设定（A2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 912, 20));            // 总计数（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 920, 21));            // 设定值（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 904, 22));            // 计数值（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 600, 23));            // 开料总长（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 606, 24));            // 打凸长度补偿（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 612, 25));            // 长度修正校验（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 614, 26));            // 长度修正不校验（B1组）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 772, 27));           // 打凸设定（B1组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 914, 28));            // 总计数（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 922, 29));            // 设定值（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 906, 30));            // 计数值（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 650, 31));            // 开料总长（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 656, 32));            // 打凸长度补偿（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 662, 33));            // 长度修正校验（B2组）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 664, 34));            // 长度修正不校验（B2组）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 773, 35));           // 打凸设定（B2组）
            
            // 开料机
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 40, 1));             // 工作状态（1号）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 0, 2));              // 故障状态（1号）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 324, 3));             // 设定量（1号）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 340, 4));             // 计件量（1号）
            // FX_POINT(SerialReadSingleFloatDataRegister(0, 255, 10, 350, 5));        // 产品加工长度（1号）
            // FX_POINT(SerialReadSingleFloatDataRegister(0, 255, 10, 8360, 6));       // 扩口当前位置（1号）
            // FX_POINT(SerialReadSingleFloatDataRegister(0, 255, 10, 8340, 7));       // 送料当前位置（1号）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 90, 8));             // 工作状态（2号）
            // FX_POINT(SerialReadSingleMDataRegister(0, 255, 10, 50, 9));              // 故障状态（2号）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 524, 10));             // 设定量（2号）
            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 540, 11));             // 计件量（2号）
            // FX_POINT(SerialReadSingleFloatDataRegister(0, 255, 10, 352, 12));        // 产品加工长度（2号）
            // FX_POINT(SerialReadSingleFloatDataRegister(0, 255, 10, 2520, 13));       // 扩口当前位置（2号）
            // FX_POINT(SerialReadSingleFloatDataRegister(0, 255, 10, 8350, 14));       // 送料当前位置（2号）

            // FX_POINT(SerialReadSingleDataRegister(0, 255, 10, 8340, 1));        // 嘉兴假数据
        #endif
#endif


#ifdef CONFIG_PLC_LS_LOAD_CONNECT_RS232
            // for (i = 0; i < 5120; i++) {
            //     LSLoadReadSingleDataRegister(i, i);
            // }
            LS_POINT(DBSReadData());
            // TRReadData();
#endif

#ifdef CONFIG_PLC_LS_LOAD_CONNECT_RS485
            //  LS PLC 485 use modbus RTU protocol
#endif

#if (defined CONFIG_PLC_HOSTLINK_CONNECT_RS485 || CONFIG_PLC_HOSTLINK_CONNECT_RS232)
    #ifdef CONFIG_PLC_HOSTLINK_SN_CACLREFILMAC
            // 冷媒灌注机
            HL_POINT(HLReadSingleDataRegister(8101, 1));    // A模式选择
            HL_POINT(HLReadSingleDataRegister(8501, 2));    // B模式选择
            HL_POINT(HLReadBCDDataRegister(8824, 3));       // A检测时间
            HL_POINT(HLReadBCDDataRegister(8844, 4));       // B检测时间
            HL_POINT(HLReadFloatDataRegister(8820, 5));     // A抽空上限
            HL_POINT(HLReadFloatDataRegister(8840, 6));     // B抽空上限
            HL_POINT(HLReadFloatDataRegister(8822, 7));     // A抽空下限
            HL_POINT(HLReadFloatDataRegister(8842, 8));     // B抽空下限
            HL_POINT(HLReadFloatDataRegister(19704, 9));    // A充注压力
            HL_POINT(HLReadFloatDataRegister(19724, 10));   // B充注压力
            HL_POINT(HLReadFloatDataRegister(19746, 11));   // A真空度
            HL_POINT(HLReadFloatDataRegister(19766, 12));   // B真空度
            HL_POINT(HLReadFloatDataRegister(8828, 13));    // A设定量
            HL_POINT(HLReadFloatDataRegister(8848, 14));    // B设定量
            HL_POINT(HLReadFloatDataRegister(8182, 15));    // A百分比
            HL_POINT(HLReadFloatDataRegister(8582, 16));    // B百分比
            HL_POINT(HLReadFloatDataRegister(8194, 17));    // A充注速度
            HL_POINT(HLReadFloatDataRegister(8594, 18));    // B充注速度
            HL_POINT(HLReadFloatDataRegister(8180, 19));    // A充注量
            HL_POINT(HLReadFloatDataRegister(8580, 20));    // B充注量
            HL_POINT(HLReadSingleDataRegister(8102, 21));   // A工作状态
            HL_POINT(HLReadSingleDataRegister(8502, 22));   // B工作状态
            HL_POINT(HLReadFloatDataRegister(8104, 23));    // A充注时间
            HL_POINT(HLReadFloatDataRegister(8504, 24));    // B充注时间
            HL_POINT(HLReadSingleDataRegister(8940, 25));   // 结果判定
            HL_POINT(HLReadFloatDataRegister(8150, 26));    // A充注设定值
            HL_POINT(HLReadFloatDataRegister(8550, 27));    // B充注设定值
    #endif
            
    #ifdef CONFIG_PLC_HOSTLINK_OTHER_DEVICE
            // 高速冲床
            // HL_POINT(HLReadSingleDataRegister(12, 1));         // 总累计冲次
            // HL_POINT(HLReadSingleDataRegister(101, 2));        // 当班累计生产数量
            // HLWriteBitCIORegister(2, 0, 0, 1);
            // HLWriteBitCIORegister(2, 0, 1, 1);
            HL_POINT(HLReadBitCIORegister(101, 5, 1));
            HL_POINT(HLReadBitCIORegister(101, 4, 2));
            HL_POINT(HLReadBitCIORegister(102, 2, 3));
            HL_POINT(HLReadBitCIORegister(102, 3, 4));
            HL_POINT(HLReadBitCIORegister(101, 2, 5));
            HL_POINT(HLReadBitCIORegister(101, 3, 6));
            HL_POINT(HLReadBitCIORegister(100, 0, 7));
            HL_POINT(HLReadBitCIORegister(100, 1, 8));
    #endif
#endif 

#ifdef CONFIG_PLC_PPI_CONNECT_RS485
            // PPI_POINT(PPIReadBitOutputRegister(0, 2, 1, 1));
            // PPI_POINT(PPIReadBitMDataRegister(2, 0, 1, 2));
            // PPI_POINT(PPIReadBitMDataRegister(2, 3, 1, 3));
            // PPI_POINT(PPIReadBitDataRegister(3315, 1, 1, 4));
            // PPI_POINT(PPIReadBitDataRegister(3315, 2, 1, 5));
            // PPI_POINT(PPIReadBitDataRegister(3315, 3, 1, 6));
            // PPI_POINT(PPIReadBitDataRegister(3315, 4, 1, 7));
            // PPI_POINT(PPIReadBitDataRegister(3315, 5, 1, 8));
            // PPI_POINT(PPIReadBitDataRegister(3315, 6, 1, 9));
            // PPI_POINT(PPIReadBitDataRegister(3315, 7, 1, 10));
            // PPI_POINT(PPIReadBitDataRegister(3316, 1, 1, 11));
            // PPI_POINT(PPIReadBitMDataRegister(0, 7, 1, 12));
            // PPI_POINT(PPIReadFloatDataRegister(210, 1, 13));
            // PPI_POINT(PPIReadFloatDataRegister(214, 1, 14));
            // PPI_POINT(PPIReadFloatDataRegister(280, 1, 15));
            // PPI_POINT(PPIReadFloatDataRegister(534, 1, 16));
            // PPI_POINT(PPIReadFloatDataRegister(1648, 1, 17));
            // PPI_POINT(PPIReadFloatDataRegister(1848, 1, 18));
            // PPI_POINT(PPIReadFloatDataRegister(900, 1, 19));
            // PPI_POINT(PPIReadFloatDataRegister(100, 1, 20));

            PPI_POINT(PPIReadByteDataRegister(120, 1, 1));
            // PPI_POINT(PPIReadByteDataRegister(101, 1, 2));
            // PPI_POINT(PPIReadByteDataRegister(102, 1, 3));
            // PPI_POINT(PPIReadBitInputRegister(1, 7, 1, 1));
            PPI_POINT(PPIReadBitInputRegister(2, 3, 1, 2));
            // PPI_POINT(PPIReadBitOutputRegister(1, 6, 1, 3));
            // PPI_POINT(PPIReadBitOutputRegister(2, 0, 1, 4));
            PPI_POINT(PPIReadBitMDataRegister(2, 3, 1, 3));

            PPI_POINT(PPIReadBitDataRegister(3315, 1, 1, 4));

            PPI_POINT(PPIReadFloatDataRegister(100, 1, 5));
            PPI_POINT(PPIReadShortDataRegister(520, 1, 6));
#endif

#ifdef CONFIG_TESTER_76T
            if (HostCmd_GetTesterGroup() == 0) {
                TESTER_76T_POINT(TTesterReadCurrentGroup());
            } else {
                TESTER_76T_POINT(TTesterReadHistoryGroup(HostCmd_GetTesterGroup()));
            }
#endif

#ifdef CONFIG_TESTER_AINUO
            TESTER_AINUO_POINT(AINUO_TTesterReadCurrentGroup());
#endif

#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
            master_operation_func(NULL);
#endif
        }
    }
    vTaskDelete(NULL);
}

#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
void send_command_task(void *pvParameters)
{
    EventBits_t uxBits;
    while (1) {
    
        uxBits = xEventGroupWaitBits(xEventGroup3, BIT_0 | BIT_1, pdTRUE, pdFALSE, portMAX_DELAY);
        if ((uxBits & BIT_0) != 0) {
            master_send_switch_func(1);     // 1:ON   0:OFF
        } else if ((uxBits & BIT_1) != 0) {
            master_send_switch_func(0);
        }      
    
    }
    vTaskDelete(NULL);
}
#endif

/* ===================================================================== 网口 PLC */

// 读寄存器返回 <0 表示链路失效，立即结束本轮由调用方重连；读到数据后立即解码上报
#define NET_POINT(call, fetch, report)      \
    do {                                    \
        if ((call) < 0) {                   \
            return -1;                      \
        }                                   \
        if ((fetch) == 0) {                 \
            report;                         \
        }                                   \
    } while (0)

#define MODBUS_NET(call)   NET_POINT(call, GetNetDataFromModbusPlc(), PlcReport_ModbusNet())
#define FX_NET(call)       NET_POINT(call, GetNetDataFromFxPlc(), PlcReport_Fx())
#define S7_NET(call)       NET_POINT(call, GetNetDataFromS7PlcNet(), PlcReport_Ppi())

#if defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
int PlcPoll_ModbusNet(int sock)
{
    // 电解炉控制柜
    MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 5, 23));       // 11#工作室真空度
    MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 7, 24));       // 11#收集室真空度
    MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 13, 25));      // 11#工作室压力值
    MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 15, 26));      // 11#收集室压力值
    MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 19, 27));      // 11#炉体水温

    MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 1, 28));       // 12#工作室真空度
    MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 3, 29));       // 12#收集室真空度
    MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 9, 30));       // 12#工作室压力值
    MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 11, 31));      // 12#收集室压力值
    MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 17, 32));      // 12#炉体水温

    // 无纸记录仪
    // MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 0x2000, 1));       // 1温区温度
    // MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 0x2002, 2));       // 2温区温度
    // MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 0x2004, 3));       // 3温区温度
    // MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 0x2006, 4));       // 4温区温度
    // MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 0x2008, 5));       // 5温区温度
    // MODBUS_NET(MODBUS_NetReadFloatRegister(sock, 0x200A, 6));       // 6温区温度
    return 0;
}
#endif

#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
int PlcPoll_FxNet(int sock)
{
    FX_NET(FX_NetReadSingleYDataRegister(sock, 94, 1));        // 正常运行状态
    FX_NET(FX_NetReadSingleYDataRegister(sock, 93, 2));        // 暂停状态
    FX_NET(FX_NetReadSingleYDataRegister(sock, 92, 3));        // 异常停止状态
    FX_NET(FX_NetReadSingleDataRegister(sock, 214, 4));        // 白班生产时间
    FX_NET(FX_NetReadSingleDataRegister(sock, 215, 5));        // 晚班生产时间
    FX_NET(FX_NetReadDoubleDataRegister(sock, 30, 6));         // 总累计冲次
    FX_NET(FX_NetReadSingleDataRegister(sock, 10, 7));         // 实时冲速
    FX_NET(FX_NetReadSingleDataRegister(sock, 101, 8));        // 当前累积生产数量
    return 0;
}
#endif

#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
int PlcPoll_S7Net(int sock)
{
#ifdef CONFIG_PLC_PPI_ELECTROLYSIS_DEVICE
    // 电解炉电解柜
    S7_NET(S7_NetReadBitQRegister(sock, 0, 2, 1));         // 运行显示
    S7_NET(S7_NetReadBitMRegister(sock, 2, 0, 2));         // 启动
    S7_NET(S7_NetReadBitMRegister(sock, 2, 3, 3));         // 故障复位
    S7_NET(S7_NetReadBitDataRegister(sock, 3315, 1, 4));   // 电源输入故障
    S7_NET(S7_NetReadBitDataRegister(sock, 3315, 2, 5));   // 设备超温
    S7_NET(S7_NetReadBitDataRegister(sock, 3315, 3, 6));   // 输出过电压
    S7_NET(S7_NetReadBitDataRegister(sock, 3315, 4, 7));   // 输出过电流
    S7_NET(S7_NetReadBitDataRegister(sock, 3315, 5, 8));   // IGBT保护
    S7_NET(S7_NetReadBitDataRegister(sock, 3315, 6, 9));   // 输出低电压
    S7_NET(S7_NetReadBitDataRegister(sock, 3315, 7, 10));  // 风扇变频故障
    S7_NET(S7_NetReadBitDataRegister(sock, 3316, 1, 11));  // 故障显示
    S7_NET(S7_NetReadBitMRegister(sock, 0, 7, 12));        // 急停
    S7_NET(S7_NetReadFloatDataRegister(sock, 210, 13));    // 实时电压
    S7_NET(S7_NetReadFloatDataRegister(sock, 214, 14));    // 实时电流
    S7_NET(S7_NetReadFloatDataRegister(sock, 280, 15));    // 设备温度
    S7_NET(S7_NetReadFloatDataRegister(sock, 534, 16));    // 电解AH
    S7_NET(S7_NetReadFloatDataRegister(sock, 1648, 17));   // 电压给定
    S7_NET(S7_NetReadFloatDataRegister(sock, 1848, 18));   // 电流给定
    S7_NET(S7_NetReadIntDataRegister(sock, 900, 19));      // 批号
    S7_NET(S7_NetReadSingleDataRegister(sock, 520, 20));   // 电解时间H
    S7_NET(S7_NetReadSingleDataRegister(sock, 522, 21));   // 电解时间M
#endif

#ifdef CONFIG_PLC_PPI_CONTROLLER_DEVICE
    // 电解炉控制柜PLC
    S7_NET(S7_NetReadBitQRegister(sock, 8, 3, 1));         // 前置阀1开到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 8, 4, 2));         // 前置阀1关到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 8, 5, 3));         // 前置阀2开到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 8, 6, 4));         // 前置阀2关到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 0, 7, 5));         // 旁通阀输出
    S7_NET(S7_NetReadBitQRegister(sock, 8, 7, 6));         // 粗抽阀1（左）开到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 9, 0, 7));         // 粗抽阀1（左）关到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 9, 1, 8));         // 粗抽阀2（左）开到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 9, 2, 9));         // 粗抽阀2（左）关到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 9, 3, 10));        // 粗抽阀1（右）开到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 9, 4, 11));        // 粗抽阀1（右）关到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 9, 5, 12));        // 粗抽阀2（右）开到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 9, 6, 13));        // 粗抽阀2（右）关到位指示
    S7_NET(S7_NetReadBitQRegister(sock, 0, 0, 14));        // H-150滑阀泵输出
    S7_NET(S7_NetReadBitQRegister(sock, 0, 1, 15));        // ZJP-1200罗茨泵输出
    S7_NET(S7_NetReadBitIRegister(sock, 9, 0, 17));        // 插板阀开启到位(左)
    S7_NET(S7_NetReadBitIRegister(sock, 9, 1, 18));        // 插板阀关闭到位(左)
    S7_NET(S7_NetReadBitIRegister(sock, 9, 2, 19));        // 插板阀开启到位(右)
    S7_NET(S7_NetReadBitIRegister(sock, 9, 3, 20));        // 插板阀关闭到位(右)
    S7_NET(S7_NetReadFloatDataRegister(sock, 100, 22));    // 冷却水压力
#endif
    return 0;
}
#endif
