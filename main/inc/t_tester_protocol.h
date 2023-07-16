#ifndef T_TESTER_PROTOCOL_H
#define T_TESTER_PROTOCOL_H

/*
安全测试仪 76T协议
UART传输格式
通讯方式：RS-232 or RS-485
数据位：8位
停止位：1位
波特率: 1200、2400、4800、9600可选
奇偶: 无
*/

#define T_TESTER_FRAME_HEAD                      'I'
#define T_TESTER_FRAME_END                       'D'
#define T_TESTER_PROTOCOL_NUM                    '1'
#define T_TESTER_ADDRESS                          1

#define T_TESTER_READ_SELFTEST_DATA               6      // 04 or 07 06
#define T_TESTER_READ_STATUS_RESPONSE             20     // 07 14 xx:状态码
#define T_TESTER_READ_CURRENT_TEST_DATA           130    // 04 or 07 82
#define T_TESTER_READ_CURRENT_GROUP_TEST_DATA     199    // 04 or 07 C7
#define T_TESTER_READ_HISTORY_GROUP_TEST_DATA     150    // 04 or 07 96 xx:组号


#define T_TESTER_GROUP_TEST_QUALIFY               0x10
#define T_TESTER_GROUP_TEST_UNQUALIFY             0x20

#define T_TESTER_ALL_GROUP_TEST_QUALIFY           0x40
#define T_TESTER_ALL_GROUP_TEST_UNQUALIFY         0x80

// 安全测试仪主命令编号
typedef enum {
    TTESTERSTOP = 0,
    TTESTERSTART,
    TTESTERDATAREVRESPONSE,
    TTESTERSETPARA,
    TTESTERDATAREAD,
    TTESTERSELECTGROUP,
    TTESTERLINECOMPEN,
    TTESTERDATAREADRESPONSE,
    TTESTERINQUIRYSTATUS = 20,
    TTESTERRESTORE
} TTesterMainCommandCode;

// 安全测试仪接受命令应答码
typedef enum {
    TTESTERCOMMANDFRAMEERROR = 0,
    TTESTERCOMMANDRECEIVESUCCESS,
    TTESTERCOMMANDREFUSE
} TTesterCommandResponseCode;

// 安全测试仪状态码
typedef enum {
    TTESTERSTANDBY = 1,
    TTESTERTESTING,
    TTESTERTESTCOMPLETE,
    TTESTERSELFTESTING,
    TTESTERSTEPWAITING,
    TTESTERSUSPEND
} TTesterStatus;

// 安全测试仪项结果码
typedef enum {
    TTESTERITEMTESTING = 4,
    TTESTERITEMTESTQUALIFY,
    TTESTERITEMTESTUNQUALIFY,
    TTESTERITEMEMPTY,
    TTESTERITEMTESTSTOP
} TTesterItemResultCode;

// 安全测试仪测试项
typedef enum {
    TTESTERPRESSURIZATION = 1,
    TTESTERGROUNDING,
    TTESTERINSULATION,
    TTESTERLEAKAGE,
    TTESTERPOWER,
    TTESTERSTARTUP,
    TTESTEROPENSHORT,
    TTESTERDCVOLTAGE
} TTesterTestItem;

#pragma pack(1)
typedef struct {
    uint8_t head;
    uint8_t pronum;
    uint8_t address[2];
    uint8_t length;
    uint8_t reserve;
    uint8_t* data;
    uint8_t end;
    uint8_t sum;
} TTesterCommonFrameFormat;

// start, stop, restore, read status, self test
typedef struct {
    uint8_t head;
    uint8_t pronum;
    uint8_t address[2];
    uint8_t length;
    uint8_t reserve;
    uint8_t maincode;
    uint8_t end;
    uint8_t sum;
} TTesterNoCommandDataFrameFormat;

// read selftest data, select group, receive command response
typedef struct {
    uint8_t head;
    uint8_t pronum;
    uint8_t address[2];
    uint8_t length;
    uint8_t reserve;
    uint8_t maincode[2];
    uint8_t end;
    uint8_t sum;
} TTesterOneCommandDataFrameFormat;

// read stutus response, read history group test data
typedef struct {
    uint8_t head;
    uint8_t pronum;
    uint8_t address[2];
    uint8_t length;
    uint8_t reserve;
    uint8_t maincode[3];
    uint8_t end;
    uint8_t sum;
} TTesterTwoCommandDataFrameFormat;
#pragma pack()

void TTestSelectGroup(uint16_t group);
void TTestInquiryStatus(void);
void TTestReadCurrentItem(void);
void TTestReadCurrentGroup(void);
void TTestReadHistoryGroup(uint16_t group);
int TTesterResolve(void);

#endif