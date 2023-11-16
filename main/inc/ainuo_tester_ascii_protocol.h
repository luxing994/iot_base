#ifndef AINUO_TESTER_ASCII_PROTOCOL_H
#define AINUO_TESTER_ASCII_PROTOCOL_H

#include <stdint.h>

/*
AINUO安全测试仪 ASCII协议
UART传输格式
通讯方式：RS-232
数据位：8位
停止位：1位
波特率: 300、1200、2400、4800、9600、19200可选
奇偶: 无
*/

#define AINUO_TESTER_ADDRESS           0x0F
#define AINUO_TESTER_FRAME_HEAD        '{'
#define AINUO_TESTER_FRAME_END         '}'

#define AINUO_TESTER_GROUP_LENTH       8
#define AINUO_TESTER_ROW_LENTH         23

#define AINUO_TESTER_STATUS            2   // 测试完成，其他测试进行

// AINUO安全测试仪命令编号
typedef enum {
    AINUOTTESTERREADDATA = 0,
    AINUOTTESTERSTART,
    AINUOTTESTERSTOP,
    AINUOTTESTSETPARA = 7
} AINUOTTesterMainCommandCode;

typedef struct {
    float current;
    float resistance;
    float testtime;
    int result;
} AINUOTTesterGroundingPara;

typedef struct {
    float voltage;
    float resistance;
    float testtime;
    int result;
} AINUOTTesterInsulationPara;

typedef struct {
    float voltage;
    float current;
    float testtime;
    int condition;
    int result;
} AINUOTTesterPressurizationPara;

typedef struct {
    float voltage;
    float current;
    float testtime;
    int condition;
    int result;
} AINUOTTesterLeakagePara;

typedef struct {
    float voltage;
    float current;
    float power;
    float testtime;
    int condition;
    int result;
} AINUOTTesterPowerPara;

typedef struct {
    float voltage;
    float current1;
    float current2;
    float current3;
    float testtime;
    int condition;
    int result;
} AINUOTTesterStartPara;

typedef struct {
    float voltage;
    float current;
    float testtime;
    int condition;
    int result;
} AINUOTTesterDCVoltagePara;

typedef struct {
    float voltage;
    float resistance;
    float testtime;
    int result;
} AINUOTTesterShortCircuitPara;

typedef struct {
    float current;
    float resistance1;
    float resistance2;
    float testtime;
} AINUOTTesterSetGroundingPara;

typedef struct {
    float voltage;
    float resistance1;
    float resistance2;
    float testtime;
    float delaytime;
    float uptime;
} AINUOTTesterSetInsulationPara;

typedef struct {
    float voltage;
    float current1;
    float current2;
    float testtime;
    int condition;
    float uptime;
    float downtime;
    int startpoint;
} AINUOTTesterSetPressurizationPara;

typedef struct {
    float voltage;
    float current1;
    float current2;
    float testtime;
    int condition;
} AINUOTTesterSetLeakagePara;

typedef struct {
    float voltage;
    float power1;
    float power2;
    float testtime;
} AINUOTTesterSetPowerPara;

typedef struct {
    float voltage;
    float current1;
    float current2;
    float testtime;
} AINUOTTesterSetStartPara;

typedef struct {
    float voltage;
    float current;
    float testtime;
    int condition;
    float uptime;
    int startpoint;
    float delaytime;
} AINUOTTesterSetDCVoltagePara;

typedef struct {
    float voltage;
    float resistance;
    float testtime;
} AINUOTTesterSetShortCircuitPara;

typedef struct {
    int compensate;
    int failitem;
    int ofrequence;
    int loadground;
    int utensiltype;
} AINUOTTesterSetCommonPara;

extern AINUOTTesterSetGroundingPara ainuosetgroudingtestdata;
extern AINUOTTesterSetPressurizationPara ainuosetpressurizationtestdata;
extern AINUOTTesterSetInsulationPara ainuosetinsulationtestdata;
extern AINUOTTesterSetLeakagePara ainuosetleakagetestdata;
extern AINUOTTesterSetPowerPara ainuosetpowertestdata;
extern AINUOTTesterSetStartPara ainuosetstarttestdata;
extern AINUOTTesterSetDCVoltagePara ainuosetdcvoltagetestdata;
extern AINUOTTesterSetShortCircuitPara ainuosetshortcircuittestdata;
extern AINUOTTesterSetCommonPara ainuosetcommontestdata;

void AINUO_TTesterReadCurrentGroup(void);
void AINUO_TTesterSetGroupPara(uint16_t group);
void AINUO_TTesterGetJsonData(char *str);
int AINUO_TTesterResolve(void);

#endif