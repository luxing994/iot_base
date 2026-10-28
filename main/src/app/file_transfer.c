#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "iot_common.h"
#include "hprotocols.h"
#include "file_transfer.h"

static const char *TAG = "FILE_TRANSFER";

#define FILE_HEADER_DELIM   "::!!@@$$**"

static struct {
    char name[16];
    int size;
    int count;          // 已收到的数据包个数
    int countsize;      // 已收到的字节数
    int flag;           // 1: 已收到控制包，正在接收数据
    char buffer[FILETRANSSIZE];
} s_file;

static struct ConfigData s_config;

// 控制包：解析文件名和文件大小
static void HandleHeader(char *data)
{
    char *name = strtok(data, FILE_HEADER_DELIM);
    char *size;

    if (name == NULL) {
        return;
    }
    strlcpy(s_file.name, name, sizeof(s_file.name));
    ESP_LOGI(TAG, "file name: %s", s_file.name);

    size = strtok(NULL, FILE_HEADER_DELIM);
    if (size != NULL && atoi(size) > 0) {      // 大小为 0 时原实现会一直停在"接收数据"状态
        s_file.size = atoi(size);
        s_file.count = 0;
        s_file.countsize = 0;
        s_file.flag = 1;
        ESP_LOGI(TAG, "file size: %d", s_file.size);
    }
}

// 数据包：保存当前包并通知使用方
static void HandleData(const char *data, int len)
{
    if (s_file.countsize >= s_file.size) {
        return;
    }
    if (len > FILETRANSSIZE) {
        len = FILETRANSSIZE;
    }

    memset(s_file.buffer, 0, FILETRANSSIZE);
    memcpy(s_file.buffer, data, len);
    s_file.countsize += len;
    s_file.count++;
    if (s_file.countsize >= s_file.size) {
        s_file.flag = 0;
    }

    // 文件名形如 "FRxxx"，xxx 为 HproOpReadCode
    int code = atoi(&s_file.name[2]);
    ESP_LOGI(TAG, "packet %d of file code %d", s_file.count, code);
    if (code == APPUPDATE) {
        xEventGroupSetBits(xEventGroup2, BIT_0);
    }
}

void FileTransfer_Reset(void)
{
    s_file.flag = 0;
    s_file.count = 0;
    s_file.countsize = 0;
}

void FileTransfer_HandlePacket(char *data, int len)
{
    // 原实现无论是否在接收数据都先对缓冲区 strtok，文件内容中出现分隔符字符时会被改写
    if (s_file.flag == 0) {
        HandleHeader(data);
    } else {
        HandleData(data, len);
    }
}

int GetFileCount(uint8_t *count)
{
    if (count == NULL) {
        return -1;
    }
    *count = s_file.count;
    return 0;
}

int GetFileData(uint8_t *data, int size)
{
    if (data == NULL || size < FILETRANSSIZE) {
        return -1;
    }
    memcpy(data, s_file.buffer, FILETRANSSIZE);
    return 0;
}

/* ------------------------------------------------------- 乐江罗拉车配置项 */

#define CONFIG_GETTER(func, type, field)    \
    int func(type *out)                     \
    {                                       \
        if (out == NULL) {                  \
            return -1;                      \
        }                                   \
        *out = s_config.field;              \
        return 0;                           \
    }

CONFIG_GETTER(GetTaskNum,   uint16_t, tasknum)
CONFIG_GETTER(GetTaskPitch, uint16_t, taskpitch)
CONFIG_GETTER(GetTaskSpeed, uint16_t, taskspeed)
CONFIG_GETTER(GetTaskCount, uint16_t, taskcount)
CONFIG_GETTER(GetTaskTime,  uint16_t, tasktime)
CONFIG_GETTER(GetMode,      uint8_t,  mode)
