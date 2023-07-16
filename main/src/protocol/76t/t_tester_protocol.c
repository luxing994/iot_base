#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <ctype.h>  
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "t_tester_protocol.h"
#include "iot_common.h"

TTesterNoCommandDataFrameFormat ttesterreadstatussbuff = {0};
TTesterOneCommandDataFrameFormat ttesterreadcuritemsbuff = {0};
TTesterOneCommandDataFrameFormat ttesterreadcurgroupbuff = {0};
TTesterTwoCommandDataFrameFormat ttesterreadhisgroupbuff = {0};
TTesterOneCommandDataFrameFormat ttesterselectgroupbuff = {0};

uint8_t ttestrdatabuffer[256] = {0};

static void TTesterPackSelectGroupFrame(uint16_t group)
{
    ttesterselectgroupbuff.head = T_TESTER_FRAME_HEAD;
    ttesterselectgroupbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttesterselectgroupbuff.address[0] = 0;
    ttesterselectgroupbuff.address[1] = T_TESTER_ADDRESS;
    ttesterselectgroupbuff.length = sizeof(TTesterOneCommandDataFrameFormat) - 1;
    ttesterselectgroupbuff.reserve = 0;
    ttesterselectgroupbuff.maincode[0] = TTESTERSELECTGROUP;
    ttesterselectgroupbuff.maincode[1] = group;
    ttesterselectgroupbuff.end = T_TESTER_FRAME_END;
    ttesterselectgroupbuff.sum = CalSumCheckDataLow(&ttesterselectgroupbuff.head, \
        sizeof(TTesterOneCommandDataFrameFormat) - 1);
}

static void TTesterPackReadStatusFrame(void)
{
    ttesterreadstatussbuff.head = T_TESTER_FRAME_HEAD;
    ttesterreadstatussbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttesterreadstatussbuff.address[0] = 0;
    ttesterreadstatussbuff.address[1] = T_TESTER_ADDRESS;
    ttesterreadstatussbuff.length = sizeof(TTesterNoCommandDataFrameFormat) - 1;
    ttesterreadstatussbuff.reserve = 0;
    ttesterreadstatussbuff.maincode = TTESTERINQUIRYSTATUS;
    ttesterreadstatussbuff.end = T_TESTER_FRAME_END;
    ttesterreadstatussbuff.sum = CalSumCheckDataLow(&ttesterreadstatussbuff.head, \
        sizeof(TTesterNoCommandDataFrameFormat) - 1);
}

static void TTesterPackReadCurrentItemFrame(void)
{
    ttesterreadcuritemsbuff.head = T_TESTER_FRAME_HEAD;
    ttesterreadcuritemsbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttesterreadcuritemsbuff.address[0] = 0;
    ttesterreadcuritemsbuff.address[1] = T_TESTER_ADDRESS;
    ttesterreadcuritemsbuff.length = sizeof(TTesterOneCommandDataFrameFormat) - 1;
    ttesterreadcuritemsbuff.reserve = 0;
    ttesterreadcuritemsbuff.maincode[0] = TTESTERDATAREAD;
    ttesterreadcuritemsbuff.maincode[1] = T_TESTER_READ_CURRENT_TEST_DATA;
    ttesterreadcuritemsbuff.end = T_TESTER_FRAME_END;
    ttesterreadcuritemsbuff.sum = CalSumCheckDataLow(&ttesterreadcuritemsbuff.head, \
        sizeof(TTesterOneCommandDataFrameFormat) - 1);
}

static void TTesterPackReaCurrentGroupFrame(void)
{
    ttesterreadcurgroupbuff.head = T_TESTER_FRAME_HEAD;
    ttesterreadcurgroupbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttesterreadcurgroupbuff.address[0] = 0;
    ttesterreadcurgroupbuff.address[1] = T_TESTER_ADDRESS;
    ttesterreadcurgroupbuff.length = sizeof(TTesterOneCommandDataFrameFormat) - 1;
    ttesterreadcurgroupbuff.reserve = 0;
    ttesterreadcurgroupbuff.maincode[0] = TTESTERDATAREAD;
    ttesterreadcurgroupbuff.maincode[1] = T_TESTER_READ_CURRENT_GROUP_TEST_DATA;
    ttesterreadcurgroupbuff.end = T_TESTER_FRAME_END;
    ttesterreadcurgroupbuff.sum = CalSumCheckDataLow(&ttesterreadcurgroupbuff.head, \
        sizeof(TTesterOneCommandDataFrameFormat) - 1);
}

static void TTesterPackReadHistoryGroupFrame(uint16_t group)
{
    ttesterreadhisgroupbuff.head = T_TESTER_FRAME_HEAD;
    ttesterreadhisgroupbuff.pronum = T_TESTER_PROTOCOL_NUM;
    ttesterreadhisgroupbuff.address[0] = 0;
    ttesterreadhisgroupbuff.address[1] = T_TESTER_ADDRESS;
    ttesterreadhisgroupbuff.length = sizeof(TTesterTwoCommandDataFrameFormat) - 1;
    ttesterreadhisgroupbuff.reserve = 0;
    ttesterreadhisgroupbuff.maincode[0] = TTESTERDATAREAD;
    ttesterreadhisgroupbuff.maincode[1] = T_TESTER_READ_HISTORY_GROUP_TEST_DATA;
    ttesterreadhisgroupbuff.maincode[2] = group;
    ttesterreadhisgroupbuff.end = T_TESTER_FRAME_END;
    ttesterreadhisgroupbuff.sum = CalSumCheckDataLow(&ttesterreadhisgroupbuff.head, \
        sizeof(TTesterTwoCommandDataFrameFormat) - 1);
}

static void TTesterResolveReadDataResponse(uint8_t* data, uint16_t len)
{
    // char *TAG = "TTester resolve command resopnse";
    
    switch (data[0]) {
        case T_TESTER_READ_SELFTEST_DATA: {
            // 获取自检数据
            break;
        }
        case T_TESTER_READ_STATUS_RESPONSE: {
            // 获取状态
            break;
        }
        case T_TESTER_READ_CURRENT_TEST_DATA: {
            // 获取当前测试项的数据
            break;
        }
        case T_TESTER_READ_CURRENT_GROUP_TEST_DATA: {
            // 获取当前测试组的数据
            break;
        }
        case T_TESTER_READ_HISTORY_GROUP_TEST_DATA: {
            // 获取历史测试数据
            break;
        }
        default: {
            break;
        }
    }
}

static void TTesterResolveCommandResponse(uint8_t* data, uint16_t len)
{
    // char *TAG = "TTester resolve command resopnse";
    
    switch (data[0]) {
        case TTESTERCOMMANDFRAMEERROR: {
            // ESP_LOGE(TAG, "command frame error");
            break;
        }
        case TTESTERCOMMANDRECEIVESUCCESS: {
            // ESP_LOGE(TAG, "command receive success");
            break;
        }
        case TTESTERCOMMANDREFUSE: {
            // ESP_LOGE(TAG, "command refuse");
            break;
        }
        default: {
            break;
        }
    }
}

void TTestSelectGroup(uint16_t group)
{
    TTesterPackSelectGroupFrame(group);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterselectgroupbuff, sizeof(TTesterOneCommandDataFrameFormat));
}

void TTestInquiryStatus(void)
{
    TTesterPackReadStatusFrame();
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadstatussbuff, sizeof(TTesterNoCommandDataFrameFormat));
}

void TTestReadCurrentItem(void)
{
    TTesterPackReadCurrentItemFrame();
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadcuritemsbuff, sizeof(TTesterOneCommandDataFrameFormat));
}

void TTestReadCurrentGroup(void)
{
    TTesterPackReaCurrentGroupFrame();
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadcurgroupbuff, sizeof(TTesterOneCommandDataFrameFormat));
}

void TTestReadHistoryGroup(uint16_t group)
{
    TTesterPackReadHistoryGroupFrame(group);
	uart_write_bytes(UART_NUM_1, (uint8_t *)&ttesterreadhisgroupbuff, sizeof(TTesterTwoCommandDataFrameFormat));
}

int TTesterResolve(void)
{
	uint8_t curData[4] = {0};
    uint8_t length, sum;
	int ret;
	static const char *TAG = "TTest_Resolve";

	while ((curData[0] != T_TESTER_FRAME_HEAD) && (curData[1] != T_TESTER_PROTOCOL_NUM) \
        && (curData[3] != T_TESTER_ADDRESS)) {
        ret = UART_ReadBufferBytes(&curData[0], 4);
        if (ret != 0) {
            return -1;
        }
	}

    ret = UART_ReadBufferBytes(&length, 1);
    if (ret != 0) {
        return -1;
    }

    length -= 6;
    ret = UART_ReadBufferBytes(ttestrdatabuffer, length);
    if (ret != 0) {
        return -1;
    }

    ret = UART_ReadBufferBytes(&curData[0], 2);
    if (ret != 0) {
        return -1;
    }
    sum = curData[1];

    switch (ttestrdatabuffer[1]) {
        case TTESTERDATAREVRESPONSE: {
            TTesterResolveCommandResponse(&ttestrdatabuffer[2], length - 2);
            break;
        }
        case TTESTERDATAREADRESPONSE: {
            TTesterResolveReadDataResponse(&ttestrdatabuffer[2], length - 2);
            break;
        }
        default: {
            break;
        } 
    }

	return 0;
}