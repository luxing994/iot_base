#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "cJSON.h"
#include "sdkconfig.h"
#include "iot_common.h"
#include "iot_time.h"
#include "host_cmd.h"
#include "plc_report.h"
#include "t_tester_protocol.h"
#include "ainuo_tester_ascii_protocol.h"

static const char *TAG = "HOST_CMD";

/* ------------------------------------------------------------- 命令状态 */

// 命令状态由命令端口任务写入、多个任务读取，读写都在临界区内拷贝
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static HostCmdIdentity s_identity = {
    .devId = DEVID,
    .devName = DEVID,
    .devTypeId = DEVTYPEID,
    .orderId = ORDERID,
    .orderName = ORDERNAME,
};
static volatile bool s_collectEnabled = true;
static volatile int s_testerGroup = 0;
static SNSetCaclReFillingMachine s_charge;

// group 命令的测试项参数，只在命令端口任务中使用
static parametersData s_groupParams[CMD_PARA_GROUPS][CMD_PARA_ITEMS];

void HostCmd_GetIdentity(HostCmdIdentity *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_identity;
    taskEXIT_CRITICAL(&s_lock);
}

bool HostCmd_IsCollectEnabled(void)
{
    return s_collectEnabled;
}

int HostCmd_GetTesterGroup(void)
{
    return s_testerGroup;
}

void HostCmd_GetChargeSetting(SNSetCaclReFillingMachine *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_charge;
    taskEXIT_CRITICAL(&s_lock);
}

/* ------------------------------------------------------------- JSON 工具 */

// 字符串字段拷贝到定长缓冲区；字段不存在或不是字符串时保持原值
static void JsonCopyString(const cJSON *obj, const char *key, char *dst, size_t size)
{
    const cJSON *item = cJSON_GetObjectItem(obj, key);

    if (cJSON_IsString(item) && item->valuestring != NULL) {
        strlcpy(dst, item->valuestring, size);
    }
}

// 数值字段，兼容 "12.5" 字符串和 12.5 数字两种写法
static int JsonGetNumber(const cJSON *obj, const char *key, double *out)
{
    const cJSON *item = cJSON_GetObjectItem(obj, key);

    if (cJSON_IsString(item) && item->valuestring != NULL) {
        *out = atof(item->valuestring);
        return 0;
    }
    if (cJSON_IsNumber(item)) {
        *out = item->valuedouble;
        return 0;
    }
    return -1;
}

/* ------------------------------------------------------------- 命令解析 */

// single 模式：冷媒灌注机充注量设定
static void ParseSingleCommand(const cJSON *root)
{
    SNSetCaclReFillingMachine charge;
    double number;

    HostCmd_GetChargeSetting(&charge);
    if (JsonGetNumber(root, "parameters", &number) == 0) {
        charge.setchargeamount = (float)number;
    }
    JsonCopyString(root, "parameterType", charge.select, sizeof(charge.select));
    ESP_LOGI(TAG, "charge amount: %.2f, select: %s", charge.setchargeamount, charge.select);

    taskENTER_CRITICAL(&s_lock);
    s_charge = charge;
    taskEXIT_CRITICAL(&s_lock);
}

/*
 * group 模式：安规测试仪组参数
 * parameterType 形如 "01_02_00_..."，指出本组包含哪些测试项；
 * parameters 为二维数组，每个测试项一行，行内参数个数必须与测试项定义一致。
 * 返回 0 表示参数完整可以下发。
 */
static int ParseGroupCommand(const cJSON *root)
{
    const cJSON *type = cJSON_GetObjectItem(root, "parameterType");
    const cJSON *params = cJSON_GetObjectItem(root, "parameters");
    int expected = 0, bytes = 0, actual = 0;
    int i, j, row;

    if (cJSON_IsString(type) && type->valuestring != NULL) {
        ESP_LOGI(TAG, "parameterType: %s", type->valuestring);
        TTesterGetSetParaStatus(type->valuestring);
        for (i = 0; i < CMD_PARA_GROUPS; i++) {
            expected += ttestparanum[tterterparasetstatus[i]];
            bytes += ttestparabytenum[tterterparasetstatus[i]];
        }
        TTesterGetSetParaByteNum(bytes);
    }

    // 没有 parameters 时沿用上一次的参数（只切换组号）
    if (!cJSON_IsArray(params)) {
        return 0;
    }

    for (i = 0; i < cJSON_GetArraySize(params); i++) {
        actual += cJSON_GetArraySize(cJSON_GetArrayItem(params, i));
    }
    if (actual != expected) {
        // 原实现在这里仍会触发下发，导致用新字节数发送旧参数
        ESP_LOGE(TAG, "parameter count error: expect %d, got %d", expected, actual);
        return -1;
    }

    memset(s_groupParams, 0, sizeof(s_groupParams));
    for (i = 0, row = 0; i < CMD_PARA_GROUPS; i++) {
        if (tterterparasetstatus[i] == 0) {
            continue;
        }
        const cJSON *items = cJSON_GetArrayItem(params, row++);
        for (j = 0; j < ttestparanum[tterterparasetstatus[i]] && j < CMD_PARA_ITEMS; j++) {
            const cJSON *item = cJSON_GetArrayItem(items, j);
            parametersData *p = &s_groupParams[i][j];

            strlcpy(p->value, "0", sizeof(p->value));
            JsonCopyString(item, "type", p->type, sizeof(p->type));
            JsonCopyString(item, "value", p->value, sizeof(p->value));
            ESP_LOGI(TAG, "para[%d][%d] {type: %s value: %s}", i, j, p->type, p->value);
        }
    }

    // 各测试项参数的字段映射由测试仪模块负责
    for (i = 0; i < CMD_PARA_GROUPS; i++) {
#ifdef CONFIG_TESTER_76T
        TTesterApplyItemParams(tterterparasetstatus[i], s_groupParams[i]);
#endif
#ifdef CONFIG_TESTER_AINUO
        AINUO_TTesterApplyItemParams(tterterparasetstatus[i], s_groupParams[i]);
#endif
    }

    return 0;
}

/* ------------------------------------------------------------- 命令分发 */

static void CmdStartFlush(void)
{
    s_collectEnabled = true;
}

static void CmdEndFlush(void)
{
    s_collectEnabled = false;
}

static void CmdPowerOn(void)
{
    xEventGroupSetBits(xEventGroup3, BIT_0);   // send_command_task：Modbus RTU 空开合闸
}

static void CmdPowerOff(void)
{
    xEventGroupSetBits(xEventGroup3, BIT_1);   // send_command_task：Modbus RTU 空开分闸
}

// 按 orderId 识别的控制命令
static const struct {
    const char *orderId;
    void (*handler)(void);
} s_controlCmds[] = {
    { "StartFlush", CmdStartFlush },
    { "EndFlush",   CmdEndFlush },
    { "PowerOn",    CmdPowerOn },
    { "PowerOff",   CmdPowerOff },
};

// 按 deviceOrderMode 识别的参数命令，解析成功后通知 tx_task 执行
static void DispatchModeCommand(const cJSON *root, const char *mode, const char *groupId)
{
    if (strcmp(mode, "single") == 0) {
        ParseSingleCommand(root);
        xEventGroupSetBits(xEventGroup1, BIT_14);
    } else if (strcmp(mode, "group") == 0) {
        if (ParseGroupCommand(root) != 0) {
            return;
        }
        if (groupId[0] != '\0') {
            s_testerGroup = atoi(groupId);
        }
        xEventGroupSetBits(xEventGroup1, BIT_15);
    }
}

// 冷媒灌注机：PLC 回读到的设定充注量与上位机下发值一致时，回复 "true"
static const char *ChargeAmountAck(void)
{
    SNSetCaclReFillingMachine charge;
    int frnum = 0;
    double applied;

    HostCmd_GetChargeSetting(&charge);
#if defined(CONFIG_PLC_FX_SN_CACLREFILMAC)
    frnum = 10;                                     // FR010 实际设定灌注量
#elif defined(CONFIG_PLC_HOSTLINK_SN_CACLREFILMAC)
    if (strcmp(charge.select, "A") == 0) {
        frnum = 26;                                 // FR026 A充注设定值
    } else if (strcmp(charge.select, "B") == 0) {
        frnum = 27;                                 // FR027 B充注设定值
    }
#endif

    if (frnum == 0 || PlcReport_GetBatchValue(frnum, &applied) != 0) {
        return NULL;
    }
    return ((float)applied == charge.setchargeamount) ? "true" : NULL;
}

const char *HostCmd_HandleMessage(const char *json)
{
    HostCmdIdentity identity;
    char mode[CMD_SHORT_STR_LEN] = {0};
    char groupId[CMD_SHORT_STR_LEN] = {0};
    char timeStamp[CMD_STR_LEN] = {0};
    cJSON *root;
    size_t i;

    ESP_LOGI(TAG, "command: %s", json);
    root = cJSON_Parse(json);
    if (root == NULL) {
        ESP_LOGW(TAG, "invalid json");
        return ChargeAmountAck();
    }

    // 标识字段缺失时沿用上一次命令的值
    HostCmd_GetIdentity(&identity);
    JsonCopyString(root, "devId", identity.devId, sizeof(identity.devId));
    JsonCopyString(root, "devName", identity.devName, sizeof(identity.devName));
    JsonCopyString(root, "devTypeId", identity.devTypeId, sizeof(identity.devTypeId));
    JsonCopyString(root, "orderId", identity.orderId, sizeof(identity.orderId));
    JsonCopyString(root, "orderName", identity.orderName, sizeof(identity.orderName));
    JsonCopyString(root, "deviceOrderMode", mode, sizeof(mode));
    JsonCopyString(root, "groupId", groupId, sizeof(groupId));
    JsonCopyString(root, "timeStamp", timeStamp, sizeof(timeStamp));

    taskENTER_CRITICAL(&s_lock);
    s_identity = identity;
    taskEXIT_CRITICAL(&s_lock);

    if (atoll(timeStamp) > 0) {
        CalBaseTime(atoll(timeStamp));
    }

    for (i = 0; i < sizeof(s_controlCmds) / sizeof(s_controlCmds[0]); i++) {
        if (strcmp(identity.orderId, s_controlCmds[i].orderId) == 0) {
            s_controlCmds[i].handler();
            break;
        }
    }
    if (i == sizeof(s_controlCmds) / sizeof(s_controlCmds[0])) {
        DispatchModeCommand(root, mode, groupId);
    }

    cJSON_Delete(root);
    return ChargeAmountAck();
}
