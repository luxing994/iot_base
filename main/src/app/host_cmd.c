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

// single 模式：冷媒灌注机充注量设定；没有充注量时返回 -1（回复 false，不写 PLC）
static int ParseSingleCommand(const cJSON *root)
{
    SNSetCaclReFillingMachine charge;
    double number;

    HostCmd_GetChargeSetting(&charge);
    if (JsonGetNumber(root, "parameters", &number) != 0) {
        ESP_LOGE(TAG, "single command without parameters");
        return -1;
    }
    charge.setchargeamount = (float)number;
    JsonCopyString(root, "parameterType", charge.select, sizeof(charge.select));
    ESP_LOGI(TAG, "charge amount: %.2f, select: %s", charge.setchargeamount, charge.select);

    taskENTER_CRITICAL(&s_lock);
    s_charge = charge;
    taskEXIT_CRITICAL(&s_lock);
    return 0;
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
    int expected = 0, actual = 0;
    int i, j, row;

    if (cJSON_IsString(type) && type->valuestring != NULL) {
        ESP_LOGI(TAG, "parameterType: %s", type->valuestring);
        if (TTester_SetItemList(type->valuestring) != 0) {
            return -1;
        }
        for (i = 0; i < CMD_PARA_GROUPS; i++) {
            expected += TTester_ItemParaNum(TTester_GetItem(i));
        }
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
        const int item = TTester_GetItem(i);

        if (item == TTESTEREMPTY) {
            continue;
        }
        const cJSON *items = cJSON_GetArrayItem(params, row++);
        for (j = 0; j < TTester_ItemParaNum(item) && j < CMD_PARA_ITEMS; j++) {
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
        TTesterApplyItemParams(TTester_GetItem(i), s_groupParams[i]);
#endif
#ifdef CONFIG_TESTER_AINUO
        AINUO_TTesterApplyItemParams(TTester_GetItem(i), s_groupParams[i]);
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

/*
 * single 命令：通知 cmd_task 写入充注量，等待它写入并回读校验后再回复上位机。
 * 在命令端口任务中同步等待，期间该端口不处理下一条命令。
 */
#define CHARGE_ACK_TIMEOUT_MS   5000    // 需要排在正在进行的串口事务之后，留足余量

static const char *ApplyChargeAndWait(void)
{
    EventBits_t bits;

    xEventGroupClearBits(xEventGroup1, EVT_CHARGE_OK | EVT_CHARGE_FAIL);   // 丢弃上一次超时后才到的结果
    xEventGroupSetBits(xEventGroup1, EVT_CHARGE_REQUEST);
    bits = xEventGroupWaitBits(xEventGroup1, EVT_CHARGE_OK | EVT_CHARGE_FAIL, pdTRUE, pdFALSE,
                               pdMS_TO_TICKS(CHARGE_ACK_TIMEOUT_MS));
    if (bits & EVT_CHARGE_OK) {
        return "true";
    }
    ESP_LOGE(TAG, "charge setting %s", (bits & EVT_CHARGE_FAIL) ? "failed" : "timeout");
    return "false";
}

// 按 deviceOrderMode 识别的参数命令，返回需要回复上位机的内容（无需回复时为 NULL）
static const char *DispatchModeCommand(const cJSON *root, const char *mode, const char *groupId)
{
    if (strcmp(mode, "single") == 0) {
        if (ParseSingleCommand(root) != 0) {
            return "false";
        }
        return ApplyChargeAndWait();
    }
    if (strcmp(mode, "group") == 0) {
        if (ParseGroupCommand(root) != 0) {
            return NULL;
        }
        if (groupId[0] != '\0') {
            s_testerGroup = atoi(groupId);
        }
        xEventGroupSetBits(xEventGroup1, BIT_15);
    }
    return NULL;
}

const char *HostCmd_HandleMessage(const char *json)
{
    HostCmdIdentity identity;
    char mode[CMD_SHORT_STR_LEN] = {0};
    char groupId[CMD_SHORT_STR_LEN] = {0};
    char timeStamp[CMD_STR_LEN] = {0};
    cJSON *root;
    const char *reply = NULL;
    size_t i;

    ESP_LOGI(TAG, "command: %s", json);
    root = cJSON_Parse(json);
    if (root == NULL) {
        ESP_LOGW(TAG, "invalid json");
        return NULL;
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
        reply = DispatchModeCommand(root, mode, groupId);
    }

    cJSON_Delete(root);
    return reply;
}
