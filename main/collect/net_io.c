#include <errno.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/time.h>
#include "esp_log.h"
#include "monitor.h"
#include "net_io.h"

static const char *TAG = "NET_IO";

void Net_SetTimeout(int sock, int timeout_ms)
{
    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };

    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

int Net_SendAll(int sock, const void *data, size_t len)
{
    const uint8_t *p = data;

    Mon_CaptureSock(sock, MON_TX, data, len);
    while (len > 0) {
        int n = send(sock, p, len, 0);
        if (n <= 0) {
            ESP_LOGE(TAG, "send failed: errno %d", errno);
            return -1;
        }
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

int Net_RecvExact(int sock, void *buf, size_t len)
{
    uint8_t *p = buf;
    const size_t want = len;

    while (len > 0) {
        int n = recv(sock, p, len, 0);
        if (n <= 0) {
            ESP_LOGE(TAG, "recv failed: ret %d errno %d", n, errno);
            return -1;
        }
        p += n;
        len -= (size_t)n;
    }
    Mon_CaptureSock(sock, MON_RX, buf, want);     // 同一应答分几次读取时由监视模块合并
    return 0;
}

int Net_Discard(int sock, size_t len)
{
    uint8_t tmp[32];

    while (len > 0) {
        size_t n = len < sizeof(tmp) ? len : sizeof(tmp);
        if (Net_RecvExact(sock, tmp, n) != 0) {
            return -1;
        }
        len -= n;
    }
    return 0;
}
