#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 启动 WiFi（STA 模式，自动重连） */
esp_err_t net_wifi_start(void);

/** WiFi 是否已连接 */
bool net_wifi_connected(void);

/** 当前 IP 字符串 */
const char *net_ip_string(void);

/** 当前 RSSI（未连接返回 0） */
int net_wifi_rssi(void);

/** 启动 MQTT（连 switchyard） */
esp_err_t net_mqtt_start(void);

/** 连接后的 meta/订阅（主循环调用） */
void net_mqtt_tick(void);

/** 发布 state（保留消息），由主循环定期调用 */
void net_mqtt_publish_state(void);

/** MQTT 是否已连接 */
bool net_mqtt_connected(void);

/** 设备 ID（用于主题） */
const char *net_device_id(void);

#ifdef __cplusplus
}
#endif
