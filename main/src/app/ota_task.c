#include "ota_task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "root_cert.h"

extern int ota_tcp_sock;  // 你跟上位机的 TCP 控制通道

static const char *TAG = "OTA_TASK";
static int64_t total_len = 0;
static int64_t written = 0;
uint8_t errbuf[120] = {0};

esp_err_t http_event_cb(esp_http_client_event_t *evt)
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
            send(ota_tcp_sock, buf, len, 0);
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
    ESP_LOGI(TAG, "STARTING OTA DOWNLOAD: %s", ota_url);

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
        send(ota_tcp_sock, "OTA_COMPLETE:OK\r\n", strlen("OTA_COMPLETE:OK\r\n"), 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();  // 切换到新固件
    } else {
        ESP_LOGE(TAG, "OTA UPDATE FAILED: %s", esp_err_to_name(ret));
        int l = snprintf((char *)errbuf, sizeof(errbuf), "OTA_ERROR:%s\r\n", esp_err_to_name(ret));
        send(ota_tcp_sock, errbuf, l, 0);
    }

    // 清理：如果不重启，需要删除任务并释放 URL 内存
    free((void*)ota_url);
    vTaskDelete(NULL);
}
