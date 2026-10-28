# 目录按功能模块划分，每个模块的 .c 和 .h 放在一起（说明见 README.md "目录结构"）
idf_component_register(
    SRCS
        "app_main.c"
        # core：上报队列、串口接收缓冲、时间基准
        "core/iot_common.c"
        "core/ringbuffer.c"
        "core/iot_time.c"
        # config：运行参数（NVS）
        "config/gw_config.c"
        # net：以太网 / Wi-Fi / 配置热点 / mDNS
        "net/net_manager.c"
        # host：与上位机通信
        "host/tcp_client.c"
        "host/tcp_server.c"
        "host/host_cmd.c"
        "host/iot_report.c"
        "host/file_transfer.c"
        "host/ota_task.c"
        # collect：设备数据采集（点表、轮询、上报、串口 / 网口收发）
        "collect/plc_points.c"
        "collect/plc_poll.c"
        "collect/plc_report.c"
        "collect/uart_task.c"
        "collect/serial_txn.c"
        "collect/net_io.c"
        # protocol：设备通信协议
        "protocol/proto_util.c"
        "protocol/fx/fx_plc_protocol.c"
        "protocol/hostlink/hl_plc_protocol.c"
        "protocol/ppi/ppi_plc_protocol.c"
        "protocol/s7comm/lx_s7comm.c"
        "protocol/ls/ls_plc_load_protocol.c"
        "protocol/76t/t_tester_protocol.c"
        "protocol/ainuo/ainuo_tester_ascii_protocol.c"
        "protocol/modbus/master.c"
        "protocol/modbus/modbus_params.c"
        "protocol/modbus/lx_modbus_tcp_master.c"
        # driver：板载外设
        "driver/sensor.c"
        "driver/max6675.c"
        # diag：诊断、事件日志、运行日志、报文监视
        "diag/diag.c"
        "diag/event_log.c"
        "diag/log_buffer.c"
        "diag/monitor.c"
        # web：配置网页
        "web/web_config.c"
    INCLUDE_DIRS
        "."
        "core"
        "config"
        "net"
        "host"
        "collect"
        "protocol"
        "protocol/fx"
        "protocol/hostlink"
        "protocol/ppi"
        "protocol/s7comm"
        "protocol/ls"
        "protocol/76t"
        "protocol/ainuo"
        "protocol/modbus"
        "driver"
        "diag"
        "web"
    EMBED_TXTFILES "web/index.html")   # 配置网页，见 web/web_config.c
