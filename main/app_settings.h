#pragma once

/*
 * 设备配置（NVS 持久化，从 orbis-w518 移植）。
 *
 * 取值优先级：NVS > secrets.h 默认值。
 * 第一次开机（NVS 里没有这些键）时把 secrets.h 的值写进 NVS，所以刚烧完的板子
 * 行为和以前完全一样；之后以配网页面（web.c）里保存的为准。
 */

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 各字段缓冲长度（含结尾 '\0'），与配网页面 input 的 maxlength 对应 */
#define APP_WIFI_SSID_MAX   33
#define APP_WIFI_PASS_MAX   65
#define APP_HA_URL_MAX      128
#define APP_HA_TOKEN_MAX    256
#define APP_MQTT_HOST_MAX   64
#define APP_MQTT_USER_MAX   32
#define APP_MQTT_PASS_MAX   64
#define APP_DEVICE_ID_MAX   32
#define APP_DEVICE_NAME_MAX 32
#define APP_WEB_USER_MAX    32
#define APP_WEB_PASS_MAX    64

typedef struct {
    /* 原有字段：ha_client.c / ota.c 在用，名字不要改 */
    char ha_url[APP_HA_URL_MAX];
    char ha_token[APP_HA_TOKEN_MAX];

    /* 配网页面新增的字段 */
    char wifi_ssid[APP_WIFI_SSID_MAX];
    char wifi_pass[APP_WIFI_PASS_MAX];
    char mqtt_host[APP_MQTT_HOST_MAX];
    int  mqtt_port;
    char mqtt_user[APP_MQTT_USER_MAX];
    char mqtt_pass[APP_MQTT_PASS_MAX];
    char device_id[APP_DEVICE_ID_MAX];     /* MQTT 主题里的名字 */
    char device_name[APP_DEVICE_NAME_MAX]; /* 中文显示名 */
    char web_user[APP_WEB_USER_MAX];       /* 设置页 Basic Auth */
    char web_pass[APP_WEB_PASS_MAX];
} app_settings_t;

/**
 * 初始化 NVS 并读取配置；NVS 为空时用 secrets.h 的值播种。
 * 幂等；NVS 不可用时也返回成功（只在内存里用 secrets.h 的值，改配置存不下来）。
 */
esp_err_t app_settings_init(void);

/** 只读访问当前配置（进程内单例，首次调用会自动 init）。 */
const app_settings_t *app_settings_get(void);

/** 写入 WiFi / HA 配置并落盘（参数为 NULL 表示不修改）。 */
esp_err_t app_settings_update(const char *ssid, const char *pass, const char *ha_url, const char *ha_token);

/** 设备显示名（NULL 或空串 = 不修改）。 */
esp_err_t app_settings_update_name(const char *name);

/** 设置页登录口令（NULL = 不修改）。 */
esp_err_t app_settings_update_web(const char *user, const char *pass);

/** MQTT / 设备 ID（任一为 NULL 表示不修改；port <= 0 表示不修改）。 */
esp_err_t app_settings_update_mqtt(const char *host, int port, const char *user, const char *pass,
                                   const char *device_id);

/** Wi-Fi 凭据是否已配置。 */
bool app_settings_has_wifi(void);

/** HA 地址与令牌是否都已配置。 */
bool app_settings_has_ha(void);

#ifdef __cplusplus
}
#endif
