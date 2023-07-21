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
#define T_TESTER_SET_PARA_SECOND_BYTE             0x63   // 03 63


#define T_TESTER_GROUP_TEST_QUALIFY               0x10
#define T_TESTER_GROUP_TEST_UNQUALIFY             0x20

#define T_TESTER_ALL_GROUP_TEST_QUALIFY           0x40
#define T_TESTER_ALL_GROUP_TEST_UNQUALIFY         0x80

#define T_TESTER_SET_PRESSURIZATION_PARA_NUM      11
#define T_TESTER_SET_GROUNDING_PARA_NUM           7
#define T_TESTER_SET_INSULATION_PARA_NUM          10
#define T_TESTER_SET_LEAKAGE_PARA_NUM             11
#define T_TESTER_SET_POWER_PARA_NUM               10
#define T_TESTER_SET_STARTUP_PARA_NUM             10
#define T_TESTER_SET_OPENSHORT_PARA_NUM           5
#define T_TESTER_SET_DCVOLTAGE_PARA_NUM           0

#define T_TESTER_FLOAT_POINT_MAX                  3

#define T_TESTER_DEVID                           "TTester"
#define T_TESTER_DEVNUMBER                       "TTester"
#define T_TESTER_DEVNAME                         "TTester"
#define T_TESTER_DEVSTATUS                       "TTester"
#define T_TESTER_DEVTYPEID                       "TTester"
#define T_TESTER_DEVTYPENAME                     "TTester"
#define T_TESTER_ORDERNAME                       ""
#define T_TESTER_ORDERID                         "FR001"
#define T_TESTER_ORDERMODE                       "group"

#define T_TESTER_ISANSWER_YES                    "yes"
#define T_TESTER_ISANSWER_NO                     "no"


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
    TTESTEREMPTY,
    TTESTERPRESSURIZATION,
    TTESTERGROUNDING,
    TTESTERINSULATION,
    TTESTERLEAKAGE,
    TTESTERPOWER,
    TTESTERSTARTUP,
    TTESTEROPENSHORT,
    TTESTERDCVOLTAGE
} TTesterTestItem;

// 测试状态
typedef enum {
    coldtestortrendstest,
    hottestorstatictest
} TTesterTestStatus;

// 测试模式
typedef enum {
    continuetest,
    steptest,
    stoptest,
    suspendtest
} TTesterMode;

// 电压类型
typedef enum {
    phasevoltage,
    linevoltage
} TTesterVoltageType;

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

typedef struct {
    uint8_t group;
    uint8_t term;
    uint8_t item;
    uint8_t result;
    uint8_t data[10];
} TTesterTestItemDataFormat;

typedef struct {
    float voltage;
    float current;
    float testtime;
    float uptime;
    float downtime;
} TTesterPressurizationPara;

typedef struct {
    float current;
    float resistance;
    float testtime;
    float reserve1;
    float reserve2;
} TTesterGroundingPara;

typedef struct {
    float voltage;
    float resistance;
    float testtime;
    float reserve1;
    float reserve2;
} TTesterInsulationPara;

typedef struct {
    float voltage;
    float firecurrent;
    float groundcurrent;
    float testtime;
    float reserve;
} TTesterLeakagePara;

typedef struct {
    float voltage;
    float current;
    float power;
    float powerf;
    float testtime;
} TTesterPowerPara;

typedef struct {
    float voltage;
    float current;
    float testtime;
    float reserve1;
    float reserve2;
} TTesterStartupPara;

typedef struct {
    float voltage;
    float current;
    float testtime;
    float reserve1;
    float reserve2;
} TTesterOpenshortPara;

typedef struct {
    float voltage;
    float current;
    float testtime;
    float reserve1;
    float reserve2;
} TTesterDcvoltagePara;

typedef struct {
    uint8_t group;
    uint8_t term;
    uint8_t item;
    uint8_t termret;
    uint8_t para[20];
} TTestestItemResultPara;

typedef struct {
    uint8_t group;
    uint8_t term;
    uint8_t item;
    uint8_t termret;
    uint8_t groupret;
    uint8_t allret;
    uint8_t para[20];
} TTestestLastItemResultPara;

typedef struct {
    TTestestItemResultPara itemdata[7];
    TTestestLastItemResultPara lastitemdata;
} TTestestGroupResultPara;

typedef struct {
    uint8_t voltage[2];
    uint8_t curupperlim[2];
    uint8_t curlowerlim[2];
    uint8_t uptime[2];
    uint8_t downtime[2];
    uint8_t testtime[2];
    uint8_t curtozero[2];
    uint8_t teststatus;
    uint8_t testmode;
    uint8_t curset[2];
    uint8_t suspendtime[2];
} TTesterSetPressurizationPara;

typedef struct {
    uint8_t curset[2];
    uint8_t resupperlim[2];
    uint8_t reslowerlim[2];
    uint8_t testtime[2];
    uint8_t testmode;
    uint8_t suspendtime[2];
    uint8_t res0tozero[2];
} TTesterSetGroundingPara;

typedef struct {
    uint8_t voltage[2];
    uint8_t resupperlim[2];
    uint8_t reslowerlim[2];
    uint8_t testtime[2];
    uint8_t delaytime[2];
    uint8_t testmode;
    uint8_t suspendtime[2];
    uint8_t res0tozero[2];
    uint8_t res1tozero[2];
    uint8_t res2tozero[2];
} TTesterSetInsulationPara;

typedef struct {
    uint8_t voltage[2];
    uint8_t curupperlim[2];
    uint8_t curlowerlim[2];
    uint8_t testtime[2];
    uint8_t teststatus;
    uint8_t testmode;
    uint8_t suspendtime[2];
    uint8_t res0tozero[2];
    uint8_t res1tozero[2];
    uint8_t res2tozero[2];
    uint8_t curtozero[2];
} TTesterSetLeakagePara;

typedef struct {
    uint8_t voltage[2];
    uint8_t curupperlim[2];
    uint8_t curlowerlim[2];
    uint8_t powerupperlim[2];
    uint8_t powerlowerlim[2];
    uint8_t delaytime[2];
    uint8_t testtime[2];
    uint8_t testmode;
    uint8_t suspendtime[2];
    uint8_t voltype;
} TTesterSetPowerPara;

typedef struct {
    uint8_t voltage[2];
    uint8_t volupperlim[2];
    uint8_t vollowerlim[2];
    uint8_t curupperlim[2];
    uint8_t curlowerlim[2];
    uint8_t delaytime[2];
    uint8_t testtime[2];
    uint8_t testmode;
    uint8_t suspendtime[2];
    uint8_t voltype;
} TTesterSetStartupPara;

typedef struct {
    uint8_t curupperlim[2];
    uint8_t curlowerlim[2];
    uint8_t testtime[2];
    uint8_t testmode;
    uint8_t suspendtime[2];
} TTesterSetOpenshortPara;
#pragma pack()

void TTesterGetSetParaByteNum(uint16_t num);
void TTesterSelectGroup(uint16_t group);
void TTesterInquiryStatus(void);
void TTesterReadCurrentItem(void);
void TTesterReadCurrentGroup(void);
void TTesterReadHistoryGroup(uint16_t group);
void TTesterSetGroupPara(uint16_t group);
void TTesterGetJsonData(char *str);
uint16_t TTesterStrChangeToUint(char *str);
int TTesterGetSetParaStatus(char *str);
int TTesterResolve(void);

#endif