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
│   ├── tcp_server.c      本机 TCP 服务端（命令端口 JSON 粘包/拆包、文件端口）
│   ├── host_cmd.c        上位机命令解析、命令状态（线程安全读取接口）、分发
│   ├── file_transfer.c   文件分包接收
│   ├── uart_task.c       UART1 初始化与事件任务、命令执行任务 cmd_task
│   ├── plc_points.c      ★ 设备点表：每种设备读哪些寄存器、对应哪个 FR 编号（按 menuconfig 选择）
│   ├── plc_poll.c        按点表逐点轮询（串口事务 / 网口）
│   ├── plc_report.c      ★ 各协议数据解码，选择单值 / 变化 / 批量上报
│   └── ota_task.c        HTTP OTA
├── src/common/
│   ├── iot_report.c      ★ 统一拼装上报 JSON（单值报文、批量报文、批量采集器）
│   ├── iot_common.c      校验、BCD、环形缓冲区封装、上报队列
│   ├── serial_txn.c      串口同步事务（加锁、等待完整应答、回滚 / 提交）
│   └── ringbuffer.c
├── src/driver/           GPIO 开关量、MAX6675 热电偶、时间基准
└── src/protocol/         三菱 FX、欧姆龙 HostLink、西门子 PPI / S7、LS、
                          Modbus RTU / TCP、76T 与艾诺安规测试仪
```

## 数据流

```
点表（plc_poll.c）逐点执行，每个点是一次同步事务：
   │
   │  串口：SerialTxn_Begin → 发请求 → SerialTxn_Wait(协议解析函数) → 解码上报 → SerialTxn_End
   │        （serial_txn.c：串口加锁；应答未收全时回滚读位置继续等，超时放弃）
   │  网口：由各连接任务调用 PlcPoll_xxxNet()，收到应答立即解码上报（链路失效则重连）
   ▼
plc_report.c   按协议 / 格式 / 字节序解码，按设备类型选择上报方式
   │
   ▼
iot_report.c   拼装 JSON（以 ";;**##" 结尾）→ 上报队列 → tcp_client_task → 上位机
```

上位机写命令（冷媒充注量、测试仪组参数）由 `cmd_task` 执行，同样在串口事务内收发，
不会插进轮询的"请求-应答"中间。

Modbus RTU 设备（`master.c`）、开关量中断、MAX6675 直接调用 `iot_report`。

## 与上位机的端口

| 方向 | 端口（menuconfig） | 用途 |
|---|---|---|
| 本机 → 上位机 | `IOT_HOST_REPORT_PORT`（8761） | 数据上报 |
| 本机 → 上位机 | `IOT_HOST_LINK_PORT`（8766） | 上线包 + 心跳，接收时间戳 |
| 本机 → 上位机 | `IOT_HOST_OTA_PORT`（8768） | 上线包 + 心跳，接收 `OTA_UPDATE:<url>` / `TIMESTAMP:<ms>` |
| 上位机 → 本机 | `IOT_CMD_PORT`（8765） | JSON 命令（StartFlush / EndFlush / PowerOn / PowerOff / single / group） |
| 上位机 → 本机 | `IOT_FILE_PORT`（8766，可关闭） | 文件分包 |

## 常见修改

**接入一台未单独适配的设备**：在 menuconfig 中选该协议的 `Common xxx device`，
然后修改 `plc_points.c` 里对应连接方式的 Common 点表，每行一个数据点：

```c
{ FX_D32, 214, 0, 4, "当前产量" },     // { 类型, 地址, 位号, FR 编号, 说明 }
```

类型说明见 `plc_points.h`（如 `FX_D16`、`HL_CIO_BIT`、`S7_VREAL`、`MB_HRFLOAT`）。
以往现场用过的点表作为"参考点表"保留在 `plc_points.c` 中，可直接复制。

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

**新增一种固定设备类型**：在 `plc_points.c` 增加该设备的点表并在文件末尾的选择逻辑中加一行，
在 `Kconfig.projbuild` 对应协议的 `Device` 下增加一个选项；若上报方式特殊（变化上报、批量上报），
在 `plc_report.c` 中处理。Modbus RTU 设备的参数表在 `master.c` 中。

## 配置与编译

```bash
idf.py menuconfig     # 本项目配置都在 IoT Gateway Configuration 菜单下
idf.py build
idf.py -p PORT flash monitor
```

`IoT Gateway Configuration` 菜单结构（各选项按 `?` 可查看中文说明）：

| 子菜单 | 内容 |
|---|---|
| Device identity | 设备号 `devId`（每台网关必须唯一）、`devNumber` |
| Host connection | 上位机 IP、上报 / 心跳 / OTA 端口、心跳和重连间隔 |
| Local server | 命令端口、上线包中的 connectPort、文件端口开关、TCP keepalive |
| Data acquisition | 应答超时；各协议的设备类型、连接方式、网口 IP / 端口 |
| Sensors | MAX6675 热电偶 |
| Debug | 任务栈余量监视 |

- UART1 同一时间只能服务**一种串口协议**，选了多个时编译会直接报错；网口协议之间、网口与一种串口协议可以组合使用
- 旧版本的 sdkconfig 可以直接使用：配置项改名记录在 `main/sdkconfig.rename`，编译时自动迁移

## 注意事项

- `sdkconfig` 中含 Wi-Fi 账号密码，提交前注意
- OTA 使用 HTTP 下载且不校验固件签名，只应在受信任的内网中使用
