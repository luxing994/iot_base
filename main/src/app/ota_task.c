#include <string.h>
#include <stdlib.h>
#include <strings.h>
#include "ota_task.h"
#include "tcp_client.h"
#include "esp_log.h"
#include "event_log.h"
#include "monitor.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "lwip/sockets.h"
#include "root_cert.h"


static const char *TAG = "OTA_TASK";
static int64_t total_len = 0;
static int64_t written = 0;
static uint8_t errbuf[120] = {0};
static bool s_ota_running = false;
static portMUX_TYPE s_ota_lock = portMUX_INITIALIZER_UNLOCKED;

bool OTA_TryBegin(void)
{
    bool ok;

    taskENTER_CRITICAL(&s_ota_lock);
    ok = !s_ota_running;
    s_ota_running = true;
    taskEXIT_CRITICAL(&s_ota_lock);
    return ok;
}

void OTA_Abort(void)
{
    taskENTER_CRITICAL(&s_ota_lock);
    s_ota_running = false;
    taskEXIT_CRITICAL(&s_ota_lock);
}

// 控制通道断开时不发送
static void OtaReport(const void *data, size_t len)
{
    int sock = ota_tcp_sock;
    if (sock >= 0) {
        Mon_CaptureSock(sock, MON_TX, data, len);
        send(sock, data, len, 0);
    }
}

static esp_err_t http_event_cb(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ON_HEADER: {
        // 服务器返回 Content-Length 时保存总大小
        if (strcasecmp(evt->header_key, "Content-Length") == 0) {
            total_len = atoll(evt->header_value);
        }
        break;
    }
    case HTTP_EVENT_ON_DATA: {
        // 每当底层读到一块数据，就累加并发送进度
        written += evt->data_len;
        if (total_len > 0) {
            int percent = (int)(written * 100 / total_len);
            char buf[32];
            int len = snprintf(buf, sizeof(buf), "OTA_PROGRESS:%d\r\n", percent);
            OtaReport(buf, len);
        }
        break;
    }
    default:
        break;
    }
    return ESP_OK;
}

void ota_task(void *pvParameter)
{
    const char *ota_url = (const char *)pvParameter;

    // 调用方已通过 OTA_TryBegin() 占用，这里不再检查
    total_len = 0;   // 原实现不复位，第二次 OTA 的进度会错误
    written = 0;

    ESP_LOGI(TAG, "STARTING OTA DOWNLOAD: %s", ota_url);
    EvLog_Add(EV_INFO, "开始固件升级");

    // 配置 HTTPS OTA，若使用 HTTP 则用 esp_http_client_config_t + esp_ota_*
    esp_http_client_config_t config = {
        .url = ota_url,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,  // <-- 关键：HTTP
        .event_handler  = http_event_cb,   // ← 关键：注册你的回调
        .timeout_ms = 10000,
        // .cert_pem = (const char*)_etc__nginx_ssl_firmware_crt, // 如果是 HTTPS 并需要校验证书
        // .transport_type = HTTP_TRANSPORT_OVER_SSL,  // 启用 SSL/TLS
        // .skip_cert_common_name_check = true,
    };

    // 调用高层封装接口完成下载、写入、校验和重启
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA UPDATE SUCCEEDED, SYSTEM REBOOT");
        EvLog_Add(EV_INFO, "固件升级成功，重启");
        OtaReport("OTA_COMPLETE:OK\r\n", strlen("OTA_COMPLETE:OK\r\n"));
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();  // 切换到新固件
    } else {
        ESP_LOGE(TAG, "OTA UPDATE FAILED: %s", esp_err_to_name(ret));
        EvLog_Add(EV_ERROR, "固件升级失败：%s", esp_err_to_name(ret));
        int l = snprintf((char *)errbuf, sizeof(errbuf), "OTA_ERROR:%s\r\n", esp_err_to_name(ret));
        OtaReport(errbuf, l);
    }

    // 清理：如果不重启，需要删除任务并释放 URL 内存
    free((void*)ota_url);
    OTA_Abort();
    vTaskDelete(NULL);
}
