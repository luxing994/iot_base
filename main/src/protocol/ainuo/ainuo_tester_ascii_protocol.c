#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "ainuo_tester_ascii_protocol.h"
#include "t_tester_protocol.h"
#include "iot_report.h"
#include "proto_util.h"
#include "serial_txn.h"

static const char *TAG = "AINUO";

#define AINUO_FRAME_HEAD        '{'
#define AINUO_FRAME_END         '}'
#define AINUO_ROW_LEN           23      // 每个测试项一行
#define AINUO_DATA_LEN          (IOT_TESTER_ITEMS * AINUO_ROW_LEN + 1)     // 8 行 + 状态
#define AINUO_SEND_MAX          512

/* 命令 */
#define AINUOTTESTERREADDATA    0
#define AINUOTTESTSETPARA       7

/* ================================================================ 组参数（上位机下发） */

typedef struct {
    float current;
    float resistance1;
    float resistance2;
    float testtime;
} AINUOTTesterSetGroundingPara;

typedef struct {
    float voltage;
    float resistance1;
    float resistance2;
    float testtime;
    float delaytime;
    float uptime;
} AINUOTTesterSetInsulationPara;

typedef struct {
    float voltage;
    float current1;
    float current2;
    float testtime;
    int condition;
    float uptime;
    float downtime;
    int startpoint;
} AINUOTTesterSetPressurizationPara;

typedef struct {
    float voltage;
    float current1;
    float current2;
    float testtime;
    int condition;
} AINUOTTesterSetLeakagePara;

typedef struct {
    float voltage;
    float power1;
    float power2;
    float testtime;
} AINUOTTesterSetPowerPara;

typedef struct {
    float voltage;
    float current1;
    float current2;
    float testtime;
} AINUOTTesterSetStartPara;

typedef struct {
    float voltage;
    float current;
    float testtime;
    int condition;
    float uptime;
    int startpoint;
    float delaytime;
} AINUOTTesterSetDCVoltagePara;

typedef struct {
    float voltage;
    float resistance;
    float testtime;
} AINUOTTesterSetShortCircuitPara;

typedef struct {
    int compensate;
    int failitem;
    int ofrequence;
    int loadground;
    int utensiltype;
} AINUOTTesterSetCommonPara;

static AINUOTTesterSetGroundingPara ainuosetgroudingtestdata;
static AINUOTTesterSetPressurizationPara ainuosetpressurizationtestdata;
static AINUOTTesterSetInsulationPara ainuosetinsulationtestdata;
static AINUOTTesterSetLeakagePara ainuosetleakagetestdata;
static AINUOTTesterSetPowerPara ainuosetpowertestdata;
static AINUOTTesterSetStartPara ainuosetstarttestdata;
static AINUOTTesterSetDCVoltagePara ainuosetdcvoltagetestdata;
static AINUOTTesterSetShortCircuitPara ainuosetshortcircuittestdata;
static AINUOTTesterSetCommonPara ainuosetcommontestdata;

// 末尾追加 "校验(2)}"：校验为 '{' 之后所有字符累加的低 8 位
static int AinuoFinish(char *buf, size_t size, int len)
{
    uint8_t sum;

    if (len < 1 || (size_t)len + 3 >= size) {
        return -1;
    }
    sum = Proto_Sum8((const uint8_t *)&buf[1], (size_t)len - 1);
    Proto_PutHex((uint8_t *)&buf[len], sum, 2);
    buf[len + 2] = AINUO_FRAME_END;
    buf[len + 3] = '\0';
    return len + 3;
}

static int AinuoPackSetCommand(char *buf, size_t size, int address, int group)
{
    int len;

    len = snprintf(buf, size, "{%03d%1d1%04d%04d%04d%04d01%04d%04d%04d%04d01%04d%04d%04d%04d%01d1%04d%04d%04d%04d%01d1%04d%04d%04d%04d0"
    "1%04d%04d%04d%04d01%04d%04d0000%04d%01d1%04d%04d0000%04d0%01d%01d%01d%01d0000%08d00000000%04d%04d%04d%04d%04d%04d%04d%04d%01d",
    address, AINUOTTESTSETPARA, (int)(ainuosetgroudingtestdata.current * 100), (int)ainuosetgroudingtestdata.resistance1, \
    (int)ainuosetgroudingtestdata.resistance2, (int)(ainuosetgroudingtestdata.testtime * 10), \
    (int)ainuosetinsulationtestdata.voltage, (int)(ainuosetinsulationtestdata.resistance1 * 10), \
    (int)(ainuosetinsulationtestdata.resistance2 * 10), (int)(ainuosetinsulationtestdata.testtime * 10), \
    (int)ainuosetpressurizationtestdata.voltage, (int)(ainuosetpressurizationtestdata.current1 * 100), \
    (int)(ainuosetpressurizationtestdata.current2 * 100), (int)(ainuosetpressurizationtestdata.testtime * 10), \
    ainuosetpressurizationtestdata.condition, (int)(ainuosetleakagetestdata.voltage * 10), (int)ainuosetleakagetestdata.current1, \
    (int)ainuosetleakagetestdata.current2, (int)(ainuosetleakagetestdata.testtime * 10), ainuosetleakagetestdata.condition, \
    (int)(ainuosetpowertestdata.voltage * 10), (int)(ainuosetpowertestdata.power1 * 10), (int)(ainuosetpowertestdata.power2 * 10), \
    (int)(ainuosetpowertestdata.testtime * 10), (int)(ainuosetstarttestdata.voltage * 10), (int)(ainuosetstarttestdata.current1 * 100), \
    (int)(ainuosetstarttestdata.current2 * 100), (int)(ainuosetstarttestdata.testtime * 10), (int)ainuosetdcvoltagetestdata.voltage, \
    (int)ainuosetdcvoltagetestdata.current, (int)(ainuosetdcvoltagetestdata.testtime * 10), ainuosetdcvoltagetestdata.condition, \
    (int)(ainuosetshortcircuittestdata.voltage * 10), (int)(ainuosetshortcircuittestdata.resistance * 10), (int)(ainuosetshortcircuittestdata.testtime * 10), 
    ainuosetcommontestdata.failitem, ainuosetcommontestdata.ofrequence, ainuosetcommontestdata.loadground, ainuosetcommontestdata.utensiltype, \
    ainuosetcommontestdata.compensate, (int)(ainuosetinsulationtestdata.uptime * 10), (int)(ainuosetinsulationtestdata.delaytime * 10), \
    (int)(ainuosetpressurizationtestdata.uptime * 10), ainuosetpressurizationtestdata.startpoint, (int)(ainuosetpressurizationtestdata.downtime * 10), \
    (int)(ainuosetdcvoltagetestdata.uptime * 10), ainuosetdcvoltagetestdata.startpoint, (int)(ainuosetdcvoltagetestdata.delaytime * 10), group);
    if (len < 0 || (size_t)len >= size) {
        return -1;
    }
    return AinuoFinish(buf, size, len);
}

/* ================================================================ 应答 */

typedef struct {
    char cmd;
    char data[AINUO_DATA_LEN];
} AinuoCtx;

static int AinuoParse(void *arg)
{
    AinuoCtx *ctx = arg;
    char addr[4] = { 0 };
    char tail[3];       // 校验(2) '}'

    if (SerialRx_SeekAny(AINUO_FRAME_HEAD, AINUO_FRAME_HEAD) < 0 || SerialRx_Read(addr, 3) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (atoi(addr) != AINUO_TESTER_ADDRESS) {
        return SERIAL_PARSE_SKIP;       // 不是本机地址的帧头，跳过继续找
    }
    if (SerialRx_Read(&ctx->cmd, 1) != 0 ||
        SerialRx_Read(ctx->data, sizeof(ctx->data)) != 0 ||
        SerialRx_Read(tail, sizeof(tail)) != 0) {
        return SERIAL_PARSE_MORE;
    }
    return (tail[2] == AINUO_FRAME_END) ? SERIAL_PARSE_OK : SERIAL_PARSE_ERROR;
}

/* ================================================================ 读组结果 */

/*
 * 每行：[0] 序号 [1..4] 值1 [5..8] 值2 [9..12] 值3 [13..16] 值4 [17..20] 测试时间 [21] 条件 [22] 结果
 * 上报的 5 个值 = 值1~值4 按下表换算（0 表示该项不上报此值，填 0），第 5 个为测试时间 / 10
 * 行顺序：接地 绝缘 耐压 泄漏 功率 启动 直流耐压 短路
 */
static const float s_rowDivisor[IOT_TESTER_ITEMS][4] = {
    { 100, 10,  0,   0   },     // 接地：电流 电阻
    { 1,   10,  0,   0   },     // 绝缘：电压 电阻
    { 1,   100, 0,   0   },     // 耐压：电压 电流
    { 10,  1,   0,   0   },     // 泄漏：电压 电流
    { 10,  100, 10,  0   },     // 功率：电压 电流 功率
    { 10,  100, 100, 100 },     // 启动：电压 电流1 电流2 电流3
    { 1,   1,   0,   0   },     // 直流耐压：电压 电流
    { 10,  10,  0,   0   },     // 短路：电压 电阻
};

/* ParameterTags：各行对应的测试项编号（与 76T 编号一致） */
static const int s_rowTags[IOT_TESTER_ITEMS] = {
    TTESTERGROUNDING, TTESTERINSULATION, TTESTERPRESSURIZATION, TTESTERLEAKAGE,
    TTESTERPOWER, TTESTERSTARTUP, TTESTERDCVOLTAGE, TTESTEROPENSHORT,
};

static int AinuoField(const char *p)
{
    char s[5];

    memcpy(s, p, 4);
    s[4] = '\0';
    return atoi(s);
}

static void AinuoDecode(const char *data, IotTesterGroup *g)
{
    int result[IOT_TESTER_ITEMS];
    int qualified;

    memset(g, 0, sizeof(*g));
    for (int i = 0; i < IOT_TESTER_ITEMS; i++) {
        const char *row = &data[i * AINUO_ROW_LEN];

        for (int k = 0; k < 4; k++) {
            const float div = s_rowDivisor[i][k];
            g->values[i][k] = (div != 0) ? (float)AinuoField(&row[1 + k * 4]) / div : 0.0f;
        }
        g->values[i][4] = (float)AinuoField(&row[17]) / 10;
        result[i] = row[22] - '0';          // 1 合格
        g->tags[i] = s_rowTags[i];
        g->checks[i] = result[i] + 4;       // 上报：5 合格，其他不合格
        ESP_LOGI(TAG, "item %d: %.2f %.2f %.2f %.2f time %.1f result %d", s_rowTags[i],
                 g->values[i][0], g->values[i][1], g->values[i][2], g->values[i][3], g->values[i][4], result[i]);
    }

    // 沿用原实现的判断：启动（第 6 行）未参与，第 2 行绝缘判断了两次
    qualified = result[0] == 1 && result[1] == 1 && result[2] == 1 && result[3] == 1 &&
                result[4] == 1 && result[1] == 1 && result[6] == 1 && result[7] == 1;
    g->groupCheck = qualified ? 5 : 6;
    g->totalCheck = g->groupCheck;
}

int AINUO_ReadGroup(void)
{
    char req[16];
    AinuoCtx ctx;
    IotTesterGroup g;
    int len;

    len = snprintf(req, sizeof(req), "{%03d%1d00}", AINUO_TESTER_ADDRESS, AINUOTTESTERREADDATA);
    if (SerialTxn_Request(req, (size_t)len, AinuoParse, &ctx, SERIAL_TXN_TESTER_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ctx.cmd - '0' != AINUOTTESTERREADDATA) {
        ESP_LOGW(TAG, "unexpected command %c", ctx.cmd);
        return -1;
    }
    AinuoDecode(ctx.data, &g);
    return IotReport_SendTesterGroup(&g);
}

int AINUO_SetGroupPara(uint16_t group)
{
    char buf[AINUO_SEND_MAX];
    AinuoCtx ctx;
    int len;

    len = AinuoPackSetCommand(buf, sizeof(buf), AINUO_TESTER_ADDRESS, group);
    if (len < 0) {
        ESP_LOGE(TAG, "set command too long");
        return -1;
    }
    if (SerialTxn_Request(buf, (size_t)len, AinuoParse, &ctx, SERIAL_TXN_TESTER_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "set group %u: no response", group);
        return -1;
    }
    return 0;
}

/* ================================================================ 上位机参数 */

// 把上位机 group 命令中一个测试项的参数写入对应的设置结构体
void AINUO_TTesterApplyItemParams(int item, parametersData *para)
{
    switch (item) {
    case TTESTERPRESSURIZATION: {

        ainuosetpressurizationtestdata.voltage = atof(para[0].value);
        ainuosetpressurizationtestdata.current1 = atof(para[1].value);
        ainuosetpressurizationtestdata.current2 = atof(para[2].value);
        ainuosetpressurizationtestdata.uptime = atof(para[3].value);
        ainuosetpressurizationtestdata.downtime = atof(para[4].value);
        ainuosetpressurizationtestdata.testtime = atof(para[5].value);
        ainuosetpressurizationtestdata.startpoint = atoi(para[6].value);
        ainuosetpressurizationtestdata.condition = atoi(para[7].value);
        
        break;
    }
    case TTESTERGROUNDING: {

        ainuosetgroudingtestdata.current = atof(para[0].value);
        ainuosetgroudingtestdata.resistance1 = atof(para[1].value);
        ainuosetgroudingtestdata.resistance2 = atof(para[2].value);
        ainuosetgroudingtestdata.testtime = atof(para[3].value);
        break;
    }
    case TTESTERINSULATION: {

        ainuosetinsulationtestdata.voltage = atof(para[0].value);
        ainuosetinsulationtestdata.resistance1 = atof(para[1].value);
        ainuosetinsulationtestdata.resistance2 = atof(para[2].value);
        ainuosetinsulationtestdata.testtime = atof(para[3].value);
        ainuosetinsulationtestdata.delaytime = atof(para[4].value);  // 绝缘延判定时间
        ainuosetinsulationtestdata.uptime = atof(para[6].value);   //绝缘缓升时间

        break;
    }
    case TTESTERLEAKAGE: {

        ainuosetleakagetestdata.voltage = atof(para[0].value);
        ainuosetleakagetestdata.current1 = atof(para[1].value);
        ainuosetleakagetestdata.current2 = atof(para[2].value);
        ainuosetleakagetestdata.testtime = atof(para[3].value);
        ainuosetleakagetestdata.condition = atoi(para[4].value);

        break;
    }
    case TTESTERPOWER: {

        ainuosetpowertestdata.voltage = atof(para[0].value);
        ainuosetpowertestdata.power1 = atof(para[3].value);
        ainuosetpowertestdata.power2 = atof(para[4].value);
        ainuosetpowertestdata.testtime = atof(para[6].value);
        break;
    }
    case TTESTERSTARTUP: {

        ainuosetstarttestdata.voltage = atof(para[0].value);
        ainuosetstarttestdata.current1 = atof(para[3].value);
        ainuosetstarttestdata.current2 = atof(para[4].value);
        ainuosetstarttestdata.testtime = atof(para[6].value);
        break;
    }
    case TTESTEROPENSHORT: {

        ainuosetshortcircuittestdata.voltage = atof(para[0].value);
        ainuosetshortcircuittestdata.resistance = atof(para[1].value);
        ainuosetshortcircuittestdata.testtime = atof(para[2].value);
        break;
    }
    case TTESTERDCVOLTAGE: {
        ainuosetdcvoltagetestdata.voltage = atof(para[0].value);
        ainuosetdcvoltagetestdata.current = atof(para[1].value);
        ainuosetdcvoltagetestdata.testtime = atof(para[2].value);
        ainuosetdcvoltagetestdata.condition = atoi(para[3].value);
        ainuosetdcvoltagetestdata.uptime = atof(para[4].value);
        ainuosetdcvoltagetestdata.startpoint = atoi(para[5].value);
        ainuosetdcvoltagetestdata.delaytime = atof(para[6].value);
        break;
    }
    case TTESTERCOMMON: {
        ainuosetcommontestdata.compensate = atoi(para[0].value);
        ainuosetcommontestdata.failitem = atoi(para[1].value);
        ainuosetcommontestdata.ofrequence = atoi(para[2].value);
        ainuosetcommontestdata.loadground = atoi(para[3].value);
        ainuosetcommontestdata.utensiltype = atoi(para[4].value);
        break;
    }
    default: {
        break;
    }
    }
}
