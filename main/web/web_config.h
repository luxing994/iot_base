#ifndef WEB_CONFIG_H
#define WEB_CONFIG_H

/*
 * 配置网页（HTTP 80 端口）
 *
 *   连配置热点时访问 http://192.168.4.1，在局域网中访问 http://<网关 IP>
 *   两个固定账号：admin（客户，密码为运行参数 webPass，可在网页中修改）、
 *   factory（厂家，密码为 menuconfig IOT_FACTORY_PASSWORD），factory 还可修改出厂参数
 *
 *   页面        GET  /                 单页应用（main/web/index.html，编译时嵌入固件）
 *   接口        POST /api/login        {"username":"admin|factory","password":""} -> {"token":"","role":""}
 *   （需登录）  GET  /api/status       运行状态
 *               GET  /api/config       当前配置（不含密码）和本固件支持的功能
 *               POST /api/config       保存配置（只需包含要修改的字段），成功后自动重启
 *               GET  /api/scan         扫描附近 Wi-Fi
 *               POST /api/reboot       重启
 *               POST /api/factory      恢复出厂设置（清除客户参数，出厂参数保留）并重启
 *   （factory） POST /api/factory-config 保存出厂参数（devId、端口、MAC 等）并重启
 *   登录后请求头带 X-Token；令牌 30 分钟无操作失效，重启后失效
 *   运行参数 webApOnly 为 true 时只接受经配置热点（192.168.4.1）来的请求，局域网访问返回 403
 */

void WebConfig_Start(void);

#endif
