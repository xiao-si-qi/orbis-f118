#pragma once

/*
 * 网页配置后台（HTTP :80）。
 *
 * 页面：
 *   /            配网表单（Wi-Fi / HA / MQTT / 设备名 / 登录口令）
 *   /save        POST，写 NVS 后重启
 *   /ota         网页 OTA 页面
 *   /ota/upload  POST，固件直接写进下一个 OTA 分区
 *
 * 全部需要 HTTP Basic Auth，账号来自 app_settings（默认 secrets.h 的 APP_SECRET_WEB_*）。
 */

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 启动配置页服务（80 端口）。幂等：重复调用只返回当前状态。
 * WiFi 连上后调用一次即可；失败时内部 5 秒内不会重复尝试，主循环可以放心反复调用。
 */
esp_err_t web_start(void);

/** 配置页是否已经在跑。 */
bool web_running(void);

#ifdef __cplusplus
}
#endif
