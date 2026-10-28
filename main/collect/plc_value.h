#ifndef PLC_VALUE_H
#define PLC_VALUE_H

/*
 * 协议层读一个数据点的结果
 *
 * 各协议的读函数（FX_LinkRead、S7_NetRead ...）把应答解码成 PlcValue 直接返回，
 * 轮询层再交给 plc_report.c 上报，中间不再经过全局变量和环形缓冲区。
 */

#include <stdint.h>
#include <string.h>

/* 读函数返回值 */
#define PLC_OK                0
#define PLC_ERR_RESPONSE     (-1)   // 超时、设备返回错误或应答格式不对：跳过本点
#define PLC_ERR_LINK         (-2)   // 网口链路失效（发送失败、对端关闭、接收超时）：需要重连
#define PLC_ERR_UNSUPPORTED  (-3)   // 当前连接方式不支持该数据类型

typedef enum {
    PLC_VAL_INT,
    PLC_VAL_FLOAT,
    PLC_VAL_STR,
} PlcValKind;

typedef struct {
    PlcValKind kind;
    int32_t i;
    float f;
    char s[8];
} PlcValue;

static inline void PlcValue_SetInt(PlcValue *v, int32_t i)
{
    v->kind = PLC_VAL_INT;
    v->i = i;
}

static inline void PlcValue_SetFloat(PlcValue *v, float f)
{
    v->kind = PLC_VAL_FLOAT;
    v->f = f;
}

static inline void PlcValue_SetFloatBits(PlcValue *v, uint32_t raw)
{
    float f;
    memcpy(&f, &raw, sizeof(f));
    PlcValue_SetFloat(v, f);
}

static inline void PlcValue_SetStr(PlcValue *v, const char *s, size_t len)
{
    if (len >= sizeof(v->s)) {
        len = sizeof(v->s) - 1;
    }
    memcpy(v->s, s, len);
    v->s[len] = '\0';
    v->kind = PLC_VAL_STR;
}

static inline double PlcValue_AsDouble(const PlcValue *v)
{
    return (v->kind == PLC_VAL_FLOAT) ? (double)v->f : (double)v->i;
}

#endif
