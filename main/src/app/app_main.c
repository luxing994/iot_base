#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"   // AF_INET
#include "sdkconfig.h"
#include "iot_common.h"
#include "tcp_client.h"
#include "tcp_server.h"
#include "uart_task.h"
#include "plc_poll.h"
#include "sensor.h"
#include "max6675.h"
#include "master.h"
#include "serial_txn.h"
#include "gw_config.h"
#include "host_cmd.h"
#include "net_manager.h"

static const char *TAG = "BOOT";

QueueHandle_t xQueue1;
EventGroupHandle_t xEventGroup1;
EventGroupHandle_t xEventGroup2;
EventGroupHandle_t xEventGroup3;

/*
 * 任务优先级（数值越大越高）。lwIP tcpip 任务为 18、Wi-Fi 任务为 23，
 * 应用任务都放在它们之下，避免串口解析等工作影响网络协议栈。
 */
#define PRIO_UART_EVENT     12      // 及时把 UART 驱动数据搬进环形缓冲区
#define PRIO_COMMAND        9       // 执行上位机命令
#define PRIO_POLL           6       // 设备轮询（串口 / 网口 / 传感器）
#define PRIO_NETWORK        5       // 与上位机的连接

#define APP_MAX_TASKS       20

static TaskHandle_t s_tasks[APP_MAX_TASKS];
static int s_taskCount;

// 创建任务并记录句柄（供栈监视使用）
static void StartTask(TaskFunction_t fn, const char *name, uint32_t stack, void *arg, UBaseType_t prio)
{
    TaskHandle_t handle = NULL;

    if (xTaskCreate(fn, name, stack, arg, prio, &handle) != pdPASS) {
        ESP_LOGE(TAG, "create task %s failed", name);
        return;
    }
    if (s_taskCount < APP_MAX_TASKS) {
        s_tasks[s_taskCount++] = handle;
    }
}

#ifdef CONFIG_IOT_DEBUG_STACK_MONITOR
// 周期打印各任务历史最小剩余栈（字节），用于调整栈大小
static void stack_monitor_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_IOT_DEBUG_STACK_MONITOR_INTERVAL_S * 1000));
        for (int i = 0; i < s_taskCount; i++) {
            ESP_LOGI("STACK", "free %-20s %5u", pcTaskGetName(s_tasks[i]),
                     (unsigned)uxTaskGetStackHighWaterMark(s_tasks[i]));
        }
        ESP_LOGI("STACK", "heap free %u, min %u", (unsigned)esp_get_free_heap_size(),
                 (unsigned)esp_get_minimum_free_heap_size());
    }
}
#endif


/* ------------------------------------------------------------------ 启动信息 */

#define FW_VERSION  "V3"

/* 本固件启用的采集设备（编译期在 menuconfig 中选择） */
static const char *const s_devices[] = {
#if defined(CONFIG_PLC_FX_CONNECT_RS232)
    "FX(RS232)",
#elif defined(CONFIG_PLC_FX_CONNECT_RS485)
    "FX(RS485)",
#elif defined(CONFIG_PLC_FX_CONNECT_NETWORK)
    "FX(net)",
#endif
#if defined(CONFIG_PLC_HOSTLINK)
    "HostLink",
#endif
#if defined(CONFIG_PLC_PPI_CONNECT_RS485)
    "PPI(RS485)",
#elif defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
    "S7(net)",
#endif
#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
    "Modbus RTU",
#elif defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
    "Modbus TCP",
#endif
#if defined(CONFIG_PLC_LS_LOAD)
    "DBS",
#endif
#if defined(CONFIG_TESTER_76T)
    "76T",
#endif
#if defined(CONFIG_TESTER_AINUO)
    "AINUO",
#endif
#if defined(CONFIG_SENSOR_MAX6675)
    "MAX6675",
#endif
    NULL,
};

static const char *ResetReasonText(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "power on / reset button";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "crash (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    default:                return "other";
    }
}

// 开机时把版本、身份、网络参数集中打印一次，现场看日志时一目了然
static void PrintBanner(void)
{
    const GwConfig *c = GwConfig_Get();
    char devices[96] = "";
    uint8_t mac[6];

    for (int i = 0; s_devices[i] != NULL; i++) {
        strlcat(devices, (i == 0) ? "" : ", ", sizeof(devices));
        strlcat(devices, s_devices[i], sizeof(devices));
    }

    ESP_LOGI(TAG, "==================================================================");
    ESP_LOGI(TAG, " IoT Gateway %s   build %s %s   IDF %s", FW_VERSION, __DATE__, __TIME__, esp_get_idf_version());
    ESP_LOGI(TAG, " reset     : %s", ResetReasonText(esp_reset_reason()));
    ESP_LOGI(TAG, " config    : %s", GwConfig_IsSaved() ? "saved (web config)" : "factory defaults (menuconfig)");
    ESP_LOGI(TAG, " device    : devId %s, devNumber %s", c->devId, c->devNumber);
    ESP_LOGI(TAG, " collect   : %s", devices[0] != '\0' ? devices : "none");
    ESP_LOGI(TAG, " host      : %s  report %u, link %u, ota %u", c->hostAddr,
             c->hostReportPort, c->hostLinkPort, c->hostOtaPort);
    ESP_LOGI(TAG, " local     : cmd port %u, connectPort %u", c->cmdPort, c->advertisedPort);
#ifdef CONFIG_IOT_ETH_W5500
    GwConfig_GetEthMac(mac);
    ESP_LOGI(TAG, " ethernet  : MAC %02X:%02X:%02X:%02X:%02X:%02X%s, %s%s", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
             c->ethMac[0] != '\0' ? "" : " (auto)", c->ethDhcp ? "DHCP" : "static ", c->ethDhcp ? "" : c->ethIp);
#else
    ESP_LOGI(TAG, " ethernet  : not fitted");
#endif
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (c->wifiSsid[0] != '\0') {
        ESP_LOGI(TAG, " wifi      : MAC %02X:%02X:%02X:%02X:%02X:%02X, SSID \"%s\"",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], c->wifiSsid);
    } else {
        ESP_LOGI(TAG, " wifi      : MAC %02X:%02X:%02X:%02X:%02X:%02X, not configured",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    ESP_LOGI(TAG, "==================================================================");
}

// ESP-IDF 组件自带的联网过程日志较多，只保留警告以上，联网状态由 NET 模块统一打印
static void QuietSystemLogs(void)
{
    static const char *const tags[] = {
        "wifi", "wifi_init", "phy_init", "esp_netif_handlers", "esp_eth.netif.netif_glue",
        "w5500.mac", "w5500.phy", "system_api", "esp_eth",
        "gpio", "uart", "pp", "net80211",    // 外设初始化的配置回显
    };

    for (size_t i = 0; i < sizeof(tags) / sizeof(tags[0]); i++) {
        esp_log_level_set(tags[i], ESP_LOG_WARN);
    }
}
void app_main(void)
{
    // NVS 分区已满或由新版本 IDF 格式化过时需要擦除重建，否则 ESP_ERROR_CHECK 会反复重启
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "nvs partition needs erase: %s", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 连按复位检测：NVS 一就绪就计数，避免后面的初始化耗时导致快速连按时漏计
    NetMgr_CountBoot();

    // 运行参数（设备号、上位机地址、PLC 地址等）从 NVS 加载，之后各模块只读
    GwConfig_Init();
    HostCmd_Init();

    QuietSystemLogs();
    PrintBanner();

    // 联网：以太网 / Wi-Fi / 配置热点，不阻塞；各通信任务在网络就绪后自行连接
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // GPIO ISR 服务全局只安装一次（W5500 中断和开关量输入共用），重复安装 IDF 会打印错误
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    NetMgr_Start();

    // 队列和事件组必须先于 GPIO 中断和各任务创建：中断回调里会直接置 xEventGroup1 的位
    xQueue1 = xQueueCreate(IOT_MSG_QUEUE_LEN, sizeof(char *));
    xEventGroup1 = xEventGroupCreate();
    xEventGroup2 = xEventGroupCreate();
    xEventGroup3 = xEventGroupCreate();

    /* 外设初始化 */
    SerialTxn_Init();
#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
    master_init();
#endif
#ifdef APP_UART_ENABLED
    uart_init();
#endif
    InitSensorGpio();
#ifdef CONFIG_SENSOR_MAX6675
    MAX6675_Init();
#endif

    /* 串口收发与命令执行 */
#ifdef APP_UART_ENABLED
    StartTask(uart_event_task, "uart_event", 1024 * 4, NULL, PRIO_UART_EVENT);
#endif
    StartTask(cmd_task, "cmd", 1024 * 8, NULL, PRIO_COMMAND);

    /* 设备轮询：串口为同步事务（发请求-等应答-解码上报），网口读到即上报 */
    StartTask(plc_poll_task, "plc_poll", 1024 * 6, NULL, PRIO_POLL);
#if defined(CONFIG_PLC_MODBUS_CONNECT_RS232) || defined(CONFIG_PLC_MODBUS_CONNECT_RS485)
    StartTask(send_command_task, "modbus_cmd", 1024 * 4, NULL, PRIO_COMMAND);
#endif
#if defined(CONFIG_PLC_MODBUS_CONNECT_NETWORK)
    StartTask(tcp_modbus_client_task, "modbus_net", 1024 * 8, NULL, PRIO_POLL);
#endif
#if defined(CONFIG_PLC_FX_CONNECT_NETWORK)
    StartTask(tcp_fx_client_task, "fx_net", 1024 * 8, NULL, PRIO_POLL);
#endif
#if defined(CONFIG_PLC_PPI_CONNECT_NETWORK)
    StartTask(tcp_ppi_client_task, "s7_net", 1024 * 8, NULL, PRIO_POLL);
#endif
#ifdef CONFIG_SENSOR_MAX6675
    StartTask(max6675_task, "max6675", 1024 * 4, NULL, PRIO_POLL);
#endif

    /* 与上位机通信 */
    StartTask(tcp_client_task, "host_report", 1024 * 8, NULL, PRIO_NETWORK);
    StartTask(tcp_client1_task, "host_link", 1024 * 8, NULL, PRIO_NETWORK);
    StartTask(tcp_client3_task, "host_ota", 1024 * 8, NULL, PRIO_NETWORK);
    StartTask(tcp_server_task, "cmd_server", 1024 * 4, (void *)AF_INET, PRIO_NETWORK);
#ifdef CONFIG_IOT_FILE_SERVER
    StartTask(tcp_server1_task, "file_server", 1024 * 4, (void *)AF_INET, PRIO_NETWORK);
#endif

#ifdef CONFIG_IOT_DEBUG_STACK_MONITOR
    xTaskCreate(stack_monitor_task, "stack_monitor", 1024 * 3, NULL, 1, NULL);
#endif
}
