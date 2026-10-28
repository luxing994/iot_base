# iot_base —— ESP32-S3 工业数据采集网关

通过串口（RS232 / RS485）或网口采集 PLC、安规测试仪、Modbus 仪表和传感器的数据，
打包成 JSON 后经 TCP 上报到上位机；同时接收上位机下发的命令（参数设定、开关控制、
时间同步、OTA 升级）。

- 芯片：ESP32-S3，ESP-IDF v5.0
- 网络：W5500 以太网 + Wi-Fi，任一路有 IP 即可通信
- 设备类型、协议、串口参数在 `idf.py menuconfig` 中选择（编译期）；
  网络、上位机地址、设备号等现场参数在**配置网页**中修改，无需重新烧录（见“现场配置”）

## 目录结构

按功能模块划分，每个模块的 `.c` 和 `.h` 放在同一目录（模块目录都在 include 路径中，直接 `#include "xxx.h"`）。

```
main/
├── app_main.c / app_info.h   启动顺序、任务创建、启动信息、HEALTH 日志
├── core/                     基础设施
│   ├── iot_common.c          串口接收缓冲封装、上报队列
│   ├── iot_time.c            时间基准（上位机校时）
│   └── ringbuffer.c
├── config/
│   └── gw_config.c           运行参数（用户区 / 出厂区，NVS 存储，默认值来自 menuconfig）
├── net/
│   └── net_manager.c         以太网 / Wi-Fi / 配置热点 / mDNS、连按复位检测
├── host/                     与上位机通信
│   ├── tcp_client.c          连接上位机（上报 / 心跳 / OTA 通道）、网口 PLC 连接与重连
│   ├── tcp_server.c          本机 TCP 服务端（命令端口 JSON 粘包/拆包、文件端口）
│   ├── host_cmd.c            上位机命令解析与分发
│   ├── iot_report.c          ★ 统一拼装上报 JSON（单值、批量、测试仪组结果）
│   ├── file_transfer.c       文件分包接收
│   └── ota_task.c            HTTP OTA（root_cert.h 为证书）
├── collect/                  设备数据采集
│   ├── plc_points.c          ★ 设备点表：每种设备读哪些寄存器、对应哪个 FR 编号
│   ├── plc_poll.c            按点表逐点轮询（串口事务 / 网口）
│   ├── plc_report.c          ★ 选择单值 / 变化 / 批量上报
│   ├── uart_task.c           UART1 初始化与事件任务、命令执行任务 cmd_task
│   ├── serial_txn.c          串口同步事务（加锁、等待完整应答、回滚 / 提交）
│   └── net_io.c              网口 PLC 收发（按长度收全、超时）
├── protocol/                 设备通信协议，每种协议一个子目录
│   ├── proto_util.c          公共编解码工具；s7_area.h 为 PPI / S7comm 共用
│   ├── fx/ hostlink/ ppi/ s7comm/ ls/
│   ├── 76t/ ainuo/           安规测试仪
│   └── modbus/               Modbus RTU（esp-modbus）/ Modbus TCP
├── driver/                   板载外设：GPIO 开关量（sensor.c）、MAX6675 热电偶
├── diag/                     诊断与日志
│   ├── diag.c                诊断检查项、诊断报告导出、core dump
│   ├── event_log.c           事件日志与崩溃记录（存 flash）
│   ├── log_buffer.c          运行日志内存缓冲
│   └── monitor.c             报文监视
└── web/
    ├── web_config.c          配置网页 HTTP 接口
    └── index.html            页面（编译时嵌入固件）
```

新增源文件后在 `main/CMakeLists.txt` 的 SRCS 中登记；新增目录还要加入 INCLUDE_DIRS。

## 数据流

```
plc_poll.c     按点表（plc_points.c）逐点调用协议层的读函数
   │
   │  协议层（protocol/）：组帧 → 收发 → 校验 → 解码，直接返回 PlcValue（plc_value.h）
   │    串口：FX_LinkRead() 等内部是一次同步事务（serial_txn.c：串口加锁，
   │          应答未收全时回滚读位置继续等，设备回 NAK 或超时则放弃该点）
   │    网口：FX_NetRead() / S7_NetRead() / MODBUS_NetRead()，按报文头长度收全应答（net_io.c），
   │          返回 PLC_ERR_LINK 时由连接任务重连
   ▼
plc_report.c   按设备类型选择单值 / 变化 / 批量上报
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
| 上位机 → 本机 | `IOT_CMD_PORT`（8765） | JSON 命令（StartFlush / EndFlush / PowerOn / PowerOff / single / group）；single 命令写入 PLC 并回读一致后回复 `true`，否则回复 `false` |
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

**新增一种 PLC 协议**：在 `protocol/` 下新建子目录实现读函数（并登记到 CMakeLists.txt），返回 `PlcValue`：
串口协议写一个解析函数，用 `SerialTxn_Request()` 发请求并等应答（参考 `ls_plc_load_protocol.c`，最简单）；
网口协议用 `net_io.h` 收发（参考 `lx_modbus_tcp_master.c`）。
然后在 `plc_points.h` 增加点类型，在 `plc_poll.c` 增加"点类型 → 读函数"的翻译。

## 现场配置（配网）

1. **进入配置模式**：新烧录的网关（出厂状态）开机即打开配置热点；已配置过的网关在 5 秒内连续按 3 次复位键
   （或快速断电上电 3 次）打开热点。热点 10 分钟无人连接自动关闭；网络断开**不会**自动打开热点。
2. 手机 / 电脑连接热点 `IoTGW-xxxx`（xxxx 为 Wi-Fi MAC 后 4 位，密码默认 `12345678`），浏览器访问 `http://192.168.4.1`。
   网关已接入局域网时，也可直接访问 `http://<devId>.local`（如 `http://jxgs001.local`，devId 转小写，
   字母数字以外的字符换成 `-`；需电脑与网关在同一网段，Windows 10/11、macOS、iOS 支持）或 `http://<网关 IP>`。
   路由器的设备列表中网关也以这个名称显示。
   电脑开了代理软件时，需把 `*.local` 加入代理的直连（绕过）列表，否则会提示“无法处理此请求”。
3. 用账号 `admin` 登录（密码默认 `admin`）。页面分为：
   - **状态**：运行概况、网络、上位机连接；有问题时点顶部概况进入诊断
   - **诊断**：检查结果和处理建议、最近事件、导出诊断信息
   - **设置**：以太网 / Wi-Fi 开关（至少一个）、自动获取 / 固定 IP、Wi-Fi 扫描、上位机地址、网口设备地址
   - **系统**：修改登录密码、“只允许通过配置热点打开本页面”、重启、恢复出厂

   修改后底部出现保存栏（可撤销），保存后自动重启生效。厂家账号另有“报文”“出厂”页，诊断页另有运行日志和崩溃记录。

参数保存在 NVS 中，升级固件不会丢失；网页中没有保存过的参数使用 menuconfig 中的值（出厂默认值）。

**出厂参数**（devId、devNumber、上位机端口、本机命令 / 文件端口、以太网 MAC、配置热点密码）客户不能修改：出货时用厂家账号 `factory`
（密码为 menuconfig `IOT_FACTORY_PASSWORD`，所有设备相同，只给生产 / 售后）登录网页，在“出厂”页逐台设置，
所有盒子烧同一个固件即可。客户在网页中恢复出厂只清除现场参数，出厂参数保留。

## 诊断与日志

现场不用接串口，打开配置网页的“诊断”页即可：

| 内容 | admin | factory | 说明 |
|---|---|---|---|
| 检查结果 | ✓ | ✓ | 网络、上位机各通道、串口设备、校时、上次重启等，中文给出原因和处理建议；状态页顶部显示问题数 |
| 最近事件 | ✓ | ✓ | 中文记录状态变化（启动、联网、连上 / 断开、设备离线、改配置、升级），存 Flash，断电不丢，约 2400 条 |
| 导出诊断信息 | ✓ | ✓ | 一个文本文件，现场发给厂家即可，内容见下 |
| 实时运行日志 | | ✓ | 与串口输出相同（开机日志单独保存，不会被冲掉） |
| 崩溃记录 | | ✓ | 异常重启前的日志、core dump 下载、清除 |

**诊断报告内容**（排查问题先看这个）：设备与固件身份（版本、git 描述、ELF SHA256）、复位原因、内存、检查结果、
全部配置（不含密码）与串口参数、网络与各连接的统计（连上 / 断开次数、最近错误码）、串口事务统计、上报积压丢弃数、
各任务剩余栈、崩溃记录（core dump 摘要：任务、PC、调用栈）、事件、开机日志、最近日志。

**崩溃排查**：报告中的调用栈地址用对应版本的 elf 解析：
`xtensa-esp32s3-elf-addr2line -pfiaC -e build/iot_base.elf 0x4200xxxx ...`；
完整分析可在“诊断”页下载 core dump 后执行 `espcoredump.py info_corefile -t raw -c coredump.bin build/iot_base.elf`。
**每次发布固件请保存对应的 `build/iot_base.elf`**（按报告中的 ELF SHA256 对应），否则崩溃地址无法解析。

Flash 分区（4 MB）：`ota_0` / `ota_1` 两个 1.5 MB 固件槽（已取消 factory 槽）；`spiffs` 分区（未挂载文件系统）存事件和
异常重启前日志；`coredump` 分区存崩溃现场；`devcfg` 分区存设备配置。
分区表改动后，**设备需用 `idf.py flash` 完整烧录一次**（OTA 只更新程序，不更新分区表）；nvs 等分区位置未变，网页参数和事件记录保留。

## 设备配置（采集）

采集什么设备、怎么通信、读哪些点由**设备配置**描述，厂家账号在配置网页“采集”页导入，无需 menuconfig 和重新编译。
格式规范见 [docs/devcfg.md](docs/devcfg.md)，示例见 [docs/devcfg-example.jsonl](docs/devcfg-example.jsonl)。
固件支持的协议、存储区、数据类型见 `main/protocol/proto_lib.c`（网页接口 `GET /api/devcfg/protocols`）。

> 过渡阶段（P0）：设备配置可以导入、校验、查看和导出，但采集仍按 menuconfig 运行；P1 起改为按设备配置采集。

## 配置与编译

```bash
idf.py menuconfig     # 本项目配置都在 IoT Gateway Configuration 菜单下
idf.py build
idf.py -p PORT flash monitor
```

`IoT Gateway Configuration` 菜单结构（各选项按 `?` 可查看中文说明）：

| 子菜单 | 内容 |
|---|---|
| Data acquisition | 采集什么设备：各协议的设备类型、连接方式、网口设备 IP / 端口默认值；应答超时；MAX6675 热电偶 |
| Factory parameters | 出厂参数默认值（网页“出厂”页逐台修改，客户不能改）：devId 等设备标识、上位机端口、本机端口、热点密码、厂家账号密码、以太网 MAC |
| Field parameters | 现场参数默认值（客户在网页修改，恢复出厂回到这里的值）：上位机地址、以太网 DHCP / 固定 IP、Wi-Fi、admin 密码、仅热点访问 |
| Hardware | W5500 以太网模块开关与引脚 |
| Communication | 心跳间隔、重连间隔、TCP keep-alive |
| Debug | HEALTH 日志间隔、任务栈余量监视 |

- UART1 同一时间只能服务**一种串口协议**，选了多个时编译会直接报错；网口协议之间、网口与一种串口协议可以组合使用
- 旧版本的 sdkconfig 可以直接使用：配置项改名记录在 `main/sdkconfig.rename`，编译时自动迁移

## 注意事项

- `sdkconfig` 中含 Wi-Fi 账号密码（出厂默认值），提交前注意
- 配置网页使用 HTTP，密码明文传输，网页密码务必改掉默认值；局域网不可信时打开“仅允许通过配置热点访问”（热点本身有 WPA2 加密）
- OTA 使用 HTTP 下载且不校验固件签名，只应在受信任的内网中使用
