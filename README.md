# iot_base —— ESP32-S3 工业数据采集网关

通过串口（RS232 / RS485）或网口采集 PLC、安规测试仪、Modbus 仪表和传感器的数据，
打包成 JSON 后经 TCP 上报到上位机；同时接收上位机下发的命令（参数设定、开关控制、
时间同步、OTA 升级）。

- 芯片：ESP32-S3，ESP-IDF v5.0
- 网络：Wi-Fi 或 W5500 以太网（`protocol_examples_common`）
- 设备、协议、连接方式全部在 `idf.py menuconfig` 中选择

## 目录结构

```
main/
├── src/app/
│   ├── app_main.c        初始化与任务创建
│   ├── tcp_client.c      连接上位机（数据上报 / 心跳 / OTA 通道）、网口 PLC 连接与重连
│   ├── tcp_server.c      接收上位机 JSON 命令、文件分包
│   ├── uart_task.c       UART1 初始化、串口收发任务、命令执行
│   ├── plc_poll.c        ★ 设备寄存器点表：每种设备读哪些寄存器、对应哪个 FR 编号
│   ├── plc_report.c      ★ 各协议数据解码，选择单值 / 变化 / 批量上报
│   └── ota_task.c        HTTP OTA
├── src/common/
│   ├── iot_report.c      ★ 统一拼装上报 JSON（单值报文、批量报文、批量采集器）
│   ├── iot_common.c      校验、BCD、环形缓冲区封装、上报队列
│   └── ringbuffer.c
├── src/driver/           GPIO 开关量、MAX6675 热电偶、时间基准
└── src/protocol/         三菱 FX、欧姆龙 HostLink、西门子 PPI / S7、LS、
                          Modbus RTU / TCP、76T 与艾诺安规测试仪
```

## 数据流

```
send_data_task / PlcPoll_xxxNet (plc_poll.c)   按点表发读请求，记录当前 FR 编号和数据格式
        │
        ▼
协议层 Get*()  解析应答，原始数据写入环形缓冲区
        │
        ▼
plc_report.c   按协议 / 格式 / 字节序解码，按设备类型选择上报方式
        │
        ▼
iot_report.c   拼装 JSON（以 ";;**##" 结尾）→ 上报队列 → tcp_client_task → 上位机
```

Modbus RTU 设备（`master.c`）、开关量中断、MAX6675 直接调用 `iot_report`。

## 与上位机的端口

| 方向 | 端口（menuconfig） | 用途 |
|---|---|---|
| 本机 → 上位机 | `EXAMPLE_PORT` | 数据上报 |
| 本机 → 上位机 | `EXAMPLE_PORT1` | 上线包 + 1s 心跳，接收时间戳 |
| 本机 → 上位机 | 8768 | 上线包 + 心跳，接收 `OTA_UPDATE:<url>` / `TIMESTAMP:<ms>` |
| 上位机 → 本机 | `SERVER_PORT` | JSON 命令（StartFlush / EndFlush / PowerOn / PowerOff / single / group） |
| 上位机 → 本机 | `SERVER_PORT1` | 文件分包 |

## 常见修改

**新增一个数据点**：在 `plc_poll.c` 对应设备的点表里加一行，例如

```c
SerialReadSingleDataRegister(0, 255, 10, 214, 4);   // 当前产量 -> FR004
```

**新增一种 JSON 报文**：调用 `iot_report.h` 中的接口，不要再手写 `sprintf`：

```c
IotReport_SendPoint(FRIGEFILLTYPEID, FXPLCDEVTYPEID, 5, "%.2f", value);   // 单值，orderId = FR005

IotBatch *b = IotBatch_Create();                                          // 多值一次打包
IotBatch_Add(b, "PA", "%.2f", pressure);
IotBatch_Add(b, "00", "%d", count);
IotBatch_Send(b, FRIGEFILLTYPEID);
IotBatch_Destroy(b);
```

PLC 每次只返回一个点、需要收齐后一起发送时，用 `IotBatchCollector` 定义点表即可，
示例见 `plc_report.c` 中的冷媒灌注机 `s_snBatchPoints`。

**新增一种设备**：在 `Kconfig.projbuild` 对应协议下增加 device type，
在 `plc_poll.c` 增加该设备的点表；若上报方式特殊，在 `plc_report.c` 中处理。

## 配置与编译

```bash
idf.py menuconfig     # Device Identity / TCP Server IP / PLC Communication / Sensor
idf.py build
idf.py -p PORT flash monitor
```

- `Device Identity`：设备号 `devId`（每台网关必须唯一）和 `devNumber`
- `PLC Communication Configuration`：选择协议、设备类型、连接方式（RS232 / RS485 / 网口）。
  UART1 同一时间只服务一种串口协议
- 当前代码已适配的设备类型以 menuconfig 中的选项为准

## 注意事项

- `sdkconfig` 中含 Wi-Fi 账号密码，提交前注意
- OTA 使用 HTTP 下载且不校验固件签名，只应在受信任的内网中使用
