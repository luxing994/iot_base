#include "esp_log.h"
#include "ls_plc_load_protocol.h"
#include "serial_txn.h"

static const char *TAG = "DBS";

#define DBS_START_OF_TR     0xFF

typedef struct {
    char reading[4];
} DbsCtx;

static int DbsParse(void *arg)
{
    DbsCtx *ctx = arg;
    uint8_t addr;
    uint8_t tail[3];        // 状态(1) + 校验(2)

    if (SerialRx_SeekAny(DBS_START_OF_TR, DBS_START_OF_TR) < 0 || SerialRx_Byte(&addr) != 0) {
        return SERIAL_PARSE_MORE;
    }
    if (addr != DBS_ADDRESS) {
        return SERIAL_PARSE_ERROR;
    }
    if (SerialRx_Read(ctx->reading, sizeof(ctx->reading)) != 0 ||
        SerialRx_Read(tail, sizeof(tail)) != 0) {
        return SERIAL_PARSE_MORE;
    }
    return SERIAL_PARSE_OK;
}

int DBS_Read(PlcValue *out)
{
    const uint8_t req = DBS_ADDRESS;
    DbsCtx ctx;

    if (SerialTxn_Request(&req, 1, DbsParse, &ctx, SERIAL_TXN_TIMEOUT_MS) != 0) {
        return PLC_ERR_RESPONSE;
    }
    ESP_LOGI(TAG, "reading: %.4s", ctx.reading);
    PlcValue_SetStr(out, ctx.reading, sizeof(ctx.reading));
    return PLC_OK;
}
