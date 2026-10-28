#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "t_tester_protocol.h"
#include "iot_report.h"
#include "proto_util.h"
#include "serial_txn.h"

static const char *TAG = "TTESTER";

#define T_HEAD              'I'
#define T_END               'D'
#define T_PROTOCOL          '1'
#define T_ADDRESS           1
#define T_HEADER_LEN        6       // 'I' '1' 地址(2) 长度 保留
#define T_FRAME_MAX         200

/* 主命令 */
#define T_CMD_RESPONSE      2       // 测试仪对命令的应答
#define T_CMD_SET_PARA      3
#define T_CMD_READ          4
#define T_CMD_READ_RESPONSE 7       // 测试仪返回读取的数据

/* 读命令的子命令 */
#define T_READ_CURRENT_GROUP    199
#define T_READ_HISTORY_GROUP    150

/* 设置命令的子命令 */
#define T_SET_GROUP_PARA        0x63
#define T_SET_COMMON_PARA       0x01

/* 组结果：每个测试项 组号 序号 测试项 结果 数据(10) */
#define T_ITEM_RESULT_LEN       14

/* 最后一项结果字节的高 4 位：组结果 + 总结果 */
#define T_GROUP_QUALIFY         0x10
#define T_GROUP_UNQUALIFY       0x20
#define T_ALL_QUALIFY           0x40
#define T_ALL_UNQUALIFY         0x80

#define T_RESULT_QUALIFY        5
#define T_RESULT_UNQUALIFY      6

/* ================================================================ 组参数（下发格式） */

#pragma pack(1)
typedef struct {
    uint8_t voltage[2];
    uint8_t curupperlim[2];
    uint8_t curlowerlim[2];
    uint8_t uptime[2];
    uint8_t downtime[2];
    uint8_t testtime[2];
    uint8_t curtozero[2];
    uint8_t teststatus;
    uint8_t testmode;
    uint8_t curset[2];
    uint8_t suspendtime[2];
} TTesterSetPressurizationPara;

typedef struct {
    uint8_t curset[2];
    uint8_t resupperlim[2];
    uint8_t reslowerlim[2];
    uint8_t testtime[2];
    uint8_t testmode;
    uint8_t suspendtime[2];
    uint8_t res0tozero[2];
} TTesterSetGroundingPara;

typedef struct {
    uint8_t voltage[2];
    uint8_t resupperlim[2];
    uint8_t reslowerlim[2];
    uint8_t testtime[2];
    uint8_t delaytime[2];
    uint8_t testmode;
    uint8_t suspendtime[2];
    uint8_t res0tozero[2];
    uint8_t res1tozero[2];
    uint8_t res2tozero[2];
} TTesterSetInsulationPara;

typedef struct {
    uint8_t voltage[2];
    uint8_t curupperlim[2];
    uint8_t curlowerlim[2];
    uint8_t testtime[2];
    uint8_t teststatus;
    uint8_t testmode;
    uint8_t suspendtime[2];
    uint8_t res0tozero[2];
    uint8_t res1tozero[2];
    uint8_t res2tozero[2];
    uint8_t curtozero[2];
} TTesterSetLeakagePara;

typedef struct {
    uint8_t voltage[2];
    uint8_t curupperlim[2];
    uint8_t curlowerlim[2];
    uint8_t powerupperlim[2];
    uint8_t powerlowerlim[2];
    uint8_t delaytime[2];
    uint8_t testtime[2];
    uint8_t testmode;
    uint8_t suspendtime[2];
    uint8_t voltype;
} TTesterSetPowerPara;

typedef struct {
    uint8_t voltage[2];
    uint8_t volupperlim[2];
    uint8_t vollowerlim[2];
    uint8_t curupperlim[2];
    uint8_t curlowerlim[2];
    uint8_t delaytime[2];
    uint8_t testtime[2];
    uint8_t testmode;
    uint8_t suspendtime[2];
    uint8_t voltype;
} TTesterSetStartupPara;

typedef struct {
    uint8_t curupperlim[2];
    uint8_t curlowerlim[2];
    uint8_t testtime[2];
    uint8_t testmode;
    uint8_t suspendtime[2];
} TTesterSetOpenshortPara;

typedef struct {
    uint8_t frequence;
    uint8_t failitem;
    uint8_t grouplink;
    uint8_t utensiltype;
} TTesterSetCommonPara;
#pragma pack()

static TTesterSetPressurizationPara s_press;
static TTesterSetGroundingPara s_ground;
static TTesterSetInsulationPara s_insul;
static TTesterSetLeakagePara s_leak;
static TTesterSetPowerPara s_power;
static TTesterSetStartupPara s_startup;
static TTesterSetOpenshortPara s_openshort;
static TTesterSetCommonPara s_common;

/* 各测试项：上位机参数个数、下发时的参数区（直流耐压 76T 不支持，只发测试项编号 0） */
typedef struct {
    uint8_t paraNum;
    const void *para;
    uint8_t paraSize;
} TItemDef;

static const TItemDef s_itemDefs[TTESTERCOMMON + 1] = {
    [TTESTEREMPTY]          = { 0,  NULL,         0 },
    [TTESTERPRESSURIZATION] = { 11, &s_press,     sizeof(s_press) },
    [TTESTERGROUNDING]      = { 7,  &s_ground,    sizeof(s_ground) },
    [TTESTERINSULATION]     = { 10, &s_insul,     sizeof(s_insul) },
    [TTESTERLEAKAGE]        = { 11, &s_leak,      sizeof(s_leak) },
    [TTESTERPOWER]          = { 10, &s_power,     sizeof(s_power) },
    [TTESTERSTARTUP]        = { 10, &s_startup,   sizeof(s_startup) },
    [TTESTEROPENSHORT]      = { 5,  &s_openshort, sizeof(s_openshort) },
    [TTESTERDCVOLTAGE]      = { 7,  NULL,         0 },
    [TTESTERCOMMON]         = { 5,  NULL,         0 },
};

static int s_items[TTESTER_GROUP_SLOTS];

int TTester_SetItemList(const char *list)
{
    int items[TTESTER_GROUP_SLOTS] = { 0 };
    const char *p = list;

    if (list == NULL) {
        return -1;
    }
    // 以 '_' 分隔，缺少的位置按空测试项处理
    for (int i = 0; i < TTESTER_GROUP_SLOTS && *p != '\0'; i++) {
        char *end;
        long v = strtol(p, &end, 10);

        if (end == p || v < TTESTEREMPTY || v > TTESTERCOMMON) {
            ESP_LOGE(TAG, "bad item list: %s", list);
            return -1;
        }
        items[i] = (int)v;
        p = end;
        while (*p == '_') {
            p++;
        }
    }
    memcpy(s_items, items, sizeof(s_items));
    return 0;
}

int TTester_GetItem(int slot)
{
    return (slot >= 0 && slot < TTESTER_GROUP_SLOTS) ? s_items[slot] : TTESTEREMPTY;
}

int TTester_ItemParaNum(int item)
{
    return (item >= TTESTEREMPTY && item <= TTESTERCOMMON) ? s_itemDefs[item].paraNum : 0;
}

/*
 * 上位机参数字符串 -> 测试仪 16 位格式：低 14 位为去掉小数点后的整数，高 2 位为小数位数
 * 例如 "1.25" -> 125 | (2 << 14)
 */
static uint16_t TTesterStrToCode(const char *str)
{
    const char *dot = strchr(str, '.');
    int decimals = (dot != NULL) ? (int)strlen(dot + 1) : 0;
    int v = (int)(atof(str) * pow(10, decimals));

    return (uint16_t)((v & 0x3FFF) | (decimals << 14));
}

static void PutCode(uint8_t dst[2], const parametersData *para)
{
    uint16_t code = TTesterStrToCode(para->value);

    dst[0] = (uint8_t)(code >> 8);
    dst[1] = (uint8_t)code;
}

void TTesterApplyItemParams(int item, parametersData *p)
{
    switch (item) {
    case TTESTERPRESSURIZATION:
        PutCode(s_press.voltage, &p[0]);
        PutCode(s_press.curupperlim, &p[1]);
        PutCode(s_press.curlowerlim, &p[2]);
        PutCode(s_press.uptime, &p[3]);
        PutCode(s_press.downtime, &p[4]);
        PutCode(s_press.testtime, &p[5]);
        PutCode(s_press.curtozero, &p[6]);
        s_press.teststatus = atoi(p[7].value);
        s_press.testmode = atoi(p[8].value);
        PutCode(s_press.curset, &p[9]);
        PutCode(s_press.suspendtime, &p[10]);
        break;
    case TTESTERGROUNDING:
        PutCode(s_ground.curset, &p[0]);
        PutCode(s_ground.resupperlim, &p[1]);
        PutCode(s_ground.reslowerlim, &p[2]);
        PutCode(s_ground.testtime, &p[3]);
        s_ground.testmode = atoi(p[4].value);
        PutCode(s_ground.suspendtime, &p[5]);
        PutCode(s_ground.res0tozero, &p[6]);
        break;
    case TTESTERINSULATION:
        PutCode(s_insul.voltage, &p[0]);
        PutCode(s_insul.resupperlim, &p[1]);
        PutCode(s_insul.reslowerlim, &p[2]);
        PutCode(s_insul.testtime, &p[3]);
        PutCode(s_insul.delaytime, &p[4]);
        s_insul.testmode = atoi(p[5].value);
        PutCode(s_insul.suspendtime, &p[6]);
        PutCode(s_insul.res0tozero, &p[7]);
        PutCode(s_insul.res1tozero, &p[8]);
        PutCode(s_insul.res2tozero, &p[9]);
        break;
    case TTESTERLEAKAGE:
        PutCode(s_leak.voltage, &p[0]);
        PutCode(s_leak.curupperlim, &p[1]);
        PutCode(s_leak.curlowerlim, &p[2]);
        PutCode(s_leak.testtime, &p[3]);
        s_leak.teststatus = atoi(p[4].value);
        s_leak.testmode = atoi(p[5].value);
        PutCode(s_leak.suspendtime, &p[6]);
        PutCode(s_leak.res0tozero, &p[7]);
        PutCode(s_leak.res1tozero, &p[8]);
        PutCode(s_leak.res2tozero, &p[9]);
        PutCode(s_leak.curtozero, &p[10]);
        break;
    case TTESTERPOWER:
        PutCode(s_power.voltage, &p[0]);
        PutCode(s_power.curupperlim, &p[1]);
        PutCode(s_power.curlowerlim, &p[2]);
        PutCode(s_power.powerupperlim, &p[3]);
        PutCode(s_power.powerlowerlim, &p[4]);
        PutCode(s_power.delaytime, &p[5]);
        PutCode(s_power.testtime, &p[6]);
        s_power.testmode = atoi(p[7].value);
        PutCode(s_power.suspendtime, &p[8]);
        s_power.voltype = atoi(p[9].value);
        break;
    case TTESTERSTARTUP:
        PutCode(s_startup.voltage, &p[0]);
        PutCode(s_startup.volupperlim, &p[1]);
        PutCode(s_startup.vollowerlim, &p[2]);
        PutCode(s_startup.curupperlim, &p[3]);
        PutCode(s_startup.curlowerlim, &p[4]);
        PutCode(s_startup.delaytime, &p[5]);
        PutCode(s_startup.testtime, &p[6]);
        s_startup.testmode = atoi(p[7].value);
        PutCode(s_startup.suspendtime, &p[8]);
        s_startup.voltype = atoi(p[9].value);
        break;
    case TTESTEROPENSHORT:
        PutCode(s_openshort.curupperlim, &p[0]);
        PutCode(s_openshort.curlowerlim, &p[1]);
        PutCode(s_openshort.testtime, &p[2]);
        s_openshort.testmode = atoi(p[3].value);
        PutCode(s_openshort.suspendtime, &p[4]);
        break;
    case TTESTERCOMMON:
        s_common.failitem = atoi(p[1].value);
        s_common.frequence = atoi(p[2].value);
        s_common.grouplink = atoi(p[3].value);
        s_common.utensiltype = atoi(p[4].value);
        break;
    default:    // 空项、直流耐压：无参数
        break;
    }
}

/* ================================================================ 帧收发 */

static size_t TBuild(uint8_t *f, const uint8_t *payload, size_t plen)
{
    f[0] = T_HEAD;
    f[1] = T_PROTOCOL;
    f[2] = 0;
    f[3] = T_ADDRESS;
    f[4] = (uint8_t)(T_HEADER_LEN + plen + 1);     // 帧总长 - 1
    f[5] = 0;
    memcpy(&f[T_HEADER_LEN], payload, plen);
    f[T_HEADER_LEN + plen] = T_END;
    f[T_HEADER_LEN + plen + 1] = Proto_Sum8(f, T_HEADER_LEN + plen + 1);
    return T_HEADER_LEN + plen + 2;
}

typedef struct {
    uint8_t body[256];      // 保留 主命令 数据...
    int dlen;               // 主命令之后的数据长度
} TCtx;

static int TParse(void *arg)
{
    TCtx *ctx = arg;
    uint8_t h[4];           // 协议号 地址(2) 长度
    uint8_t tail[2];        // 'D' 和校验
    int blen;

    if (SerialRx_SeekAny(T_HEAD, T_HEAD) < 0 || SerialRx_Read(h, sizeof(h)) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (h[0] != T_PROTOCOL || h[2] != T_ADDRESS || h[3] < T_HEADER_LEN + 2) {
        return SERIAL_PARSE_SKIP;       // 不是帧头（数据中的 'I'），跳过继续找
    }
    blen = h[3] - (T_HEADER_LEN - 1) - 1;      // 保留 + 主命令 + 数据
    if (SerialRx_Read(ctx->body, blen) != 0 || SerialRx_Read(tail, sizeof(tail)) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (tail[0] != T_END) {
        return SERIAL_PARSE_ERROR;
    }
    ctx->dlen = blen - 2;
    return SERIAL_PARSE_OK;
}

/* ================================================================ 读组结果 */

static IotTesterGroup s_group;

// 测量值：低 14 位为数值，高 2 位为小数位数
static float TTesterValue(uint16_t raw)
{
    static const float div[4] = { 1.0f, 10.0f, 100.0f, 1000.0f };
    return (float)(raw & 0x3FFF) / div[raw >> 14];
}

static int TParseGroupResult(const uint8_t *d, int len)
{
    if (len != IOT_TESTER_ITEMS * T_ITEM_RESULT_LEN) {
        ESP_LOGE(TAG, "group result length %d", len);
        return -1;
    }

    for (int i = 0; i < IOT_TESTER_ITEMS; i++) {
        const uint8_t *r = &d[i * T_ITEM_RESULT_LEN];
        const uint8_t item = r[2];
        const uint8_t result = r[3];
        const int measured = (item >= TTESTERPRESSURIZATION && item <= TTESTERDCVOLTAGE);

        s_group.tags[i] = item;
        s_group.checks[i] = (i < IOT_TESTER_ITEMS - 1) ? result : (result & 0x0F);
        for (int k = 0; k < IOT_TESTER_VALUES; k++) {
            s_group.values[i][k] = measured ? TTesterValue(Proto_Be16(&r[4 + k * 2])) : 0.0f;
        }
        ESP_LOGI(TAG, "item %d: type %d result 0x%02X %.2f %.2f %.2f %.2f %.2f", i, item, result,
                 s_group.values[i][0], s_group.values[i][1], s_group.values[i][2],
                 s_group.values[i][3], s_group.values[i][4]);

        if (i == IOT_TESTER_ITEMS - 1) {
            // 组结果和总结果在最后一项结果字节的高 4 位，其他组合保持上次的值
            switch (result & 0xF0) {
            case T_GROUP_QUALIFY + T_ALL_QUALIFY:
                s_group.groupCheck = T_RESULT_QUALIFY;
                s_group.totalCheck = T_RESULT_QUALIFY;
                break;
            case T_GROUP_QUALIFY + T_ALL_UNQUALIFY:
                s_group.groupCheck = T_RESULT_QUALIFY;
                s_group.totalCheck = T_RESULT_UNQUALIFY;
                break;
            case T_GROUP_UNQUALIFY + T_ALL_UNQUALIFY:
                s_group.groupCheck = T_RESULT_UNQUALIFY;
                s_group.totalCheck = T_RESULT_UNQUALIFY;
                break;
            default:
                break;
            }
        }
    }
    return 0;
}

int TTester_ReadGroup(uint16_t group)
{
    uint8_t payload[3] = { T_CMD_READ, T_READ_CURRENT_GROUP, 0 };
    size_t plen = 2;
    uint8_t f[T_FRAME_MAX];
    size_t n;
    TCtx ctx;

    if (group != 0) {
        payload[1] = T_READ_HISTORY_GROUP;
        payload[2] = (uint8_t)group;
        plen = 3;
    }
    n = TBuild(f, payload, plen);
    if (SerialTxn_Request(f, n, TParse, &ctx, SERIAL_TXN_TESTER_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ctx.body[1] != T_CMD_READ_RESPONSE || ctx.dlen < 1) {
        ESP_LOGW(TAG, "unexpected response, command %d", ctx.body[1]);
        return -1;
    }

    // 目前只解析当前组结果；历史组应答沿用原实现，上报最近一次的组结果
    if (ctx.body[2] == T_READ_CURRENT_GROUP &&
        TParseGroupResult(&ctx.body[3], ctx.dlen - 1) != 0) {
        return -1;
    }
    s_group.groupId = group;
    return IotReport_SendTesterGroup(&s_group);
}

/* ================================================================ 下发组参数 */

int TTester_SetGroupPara(uint16_t group)
{
    uint8_t payload[T_FRAME_MAX - T_HEADER_LEN - 2];
    uint8_t f[T_FRAME_MAX];
    size_t n, plen;
    TCtx ctx;
    int ret;

    // 通用参数帧
    payload[0] = T_CMD_SET_PARA;
    payload[1] = T_SET_COMMON_PARA;
    payload[2] = (uint8_t)group;
    memcpy(&payload[3], &s_common, sizeof(s_common));
    plen = 3 + sizeof(s_common);
    n = TBuild(f, payload, plen);

    SerialTxn_Begin();
    SerialTxn_Send(f, n);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 组参数帧：前 8 个位置依次为 测试项编号 + 该项参数
    payload[1] = T_SET_GROUP_PARA;
    plen = 3;
    for (int i = 0; i < IOT_TESTER_ITEMS; i++) {
        const int item = s_items[i];
        const TItemDef *def = &s_itemDefs[item];

        if (item == TTESTERCOMMON) {
            continue;
        }
        payload[plen++] = (item == TTESTERDCVOLTAGE) ? 0 : (uint8_t)item;
        if (def->para != NULL) {
            memcpy(&payload[plen], def->para, def->paraSize);
            plen += def->paraSize;
        }
    }
    n = TBuild(f, payload, plen);
    SerialTxn_Send(f, n);

    ret = SerialTxn_Wait(TParse, &ctx, SERIAL_TXN_TESTER_TIMEOUT_MS);
    SerialTxn_End();

    if (ret != 0) {
        ESP_LOGE(TAG, "set group %u: no response", group);
        return -1;
    }
    if (ctx.body[1] == T_CMD_RESPONSE && ctx.dlen >= 1) {
        ESP_LOGI(TAG, "set group %u: response code %d", group, ctx.body[2]);   // 1 成功，0 帧错误，2 拒绝
    }
    return 0;
}
