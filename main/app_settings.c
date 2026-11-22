/*
 * 设备配置：NVS 持久化（从 orbis-w518/main/app_settings.c 移植）。
 *
 * 首次开机把 secrets.h 的值写进 NVS；之后配网页面（web.c）写入的配置优先。
 * 所有写操作都要能容忍 NVS 写满/写坏：日志里报警，内存里的值照常生效，
 * 不能让设备因为存不下配置就开不了机。
 */

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "app_settings.h"
#include "secrets.h"

static const char *TAG = "settings";

#define NVS_NAMESPACE "app"

#define KEY_SSID        "ssid"
#define KEY_PASS        "pass"
#define KEY_URL         "url"
#define KEY_TOKEN       "token"
#define KEY_MQTT_HOST   "mqtt_host"
#define KEY_MQTT_PORT   "mqtt_port"
#define KEY_MQTT_USER   "mqtt_user"
#define KEY_MQTT_PASS   "mqtt_pass"
#define KEY_DEVICE_ID   "device_id"
#define KEY_DEVICE_NAME "device_name"
#define KEY_WEB_USER    "web_user"
#define KEY_WEB_PASS    "web_pass"

/* secrets.h 里 APP_SECRET_DEVICE_ID 可能留空：兜底值必须和 net.c 原来的
   "orbis-f118" 一致，否则升级固件后 MQTT 主题会从 switchyard/orbis-f118
   变成别的名字，一直等的 cfg 就收不到了。 */
#define FALLBACK_DEVICE_ID "orbis-f118"

static app_settings_t s_cfg;
static bool s_ready;
static bool s_nvs_ok;      /* NVS 真能用才写回，否则只在内存里用默认值 */

static void copy_str(char *dst, size_t dst_len, const char *src)
{
    if (dst == NULL || dst_len == 0) {
        return;
    }
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, src, dst_len - 1);
    dst[dst_len - 1] = '\0';
}

static void read_str(nvs_handle_t h, const char *key, char *out, size_t out_len)
{
    size_t len = out_len;
    const esp_err_t err = nvs_get_str(h, key, out, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        out[0] = '\0';
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "读取 %s 失败（%s），用默认值", key, esp_err_to_name(err));
        out[0] = '\0';
        return;
    }
    out[out_len - 1] = '\0';
}

static esp_err_t config_save(void)
{
    if (!s_nvs_ok) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "打开 NVS 失败（%s），配置未保存", esp_err_to_name(err));
        return err;
    }

    if (err == ESP_OK) err = nvs_set_str(h, KEY_SSID, s_cfg.wifi_ssid);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_PASS, s_cfg.wifi_pass);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_URL, s_cfg.ha_url);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_TOKEN, s_cfg.ha_token);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_MQTT_HOST, s_cfg.mqtt_host);
    if (err == ESP_OK) err = nvs_set_u16(h, KEY_MQTT_PORT, (uint16_t)s_cfg.mqtt_port);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_MQTT_USER, s_cfg.mqtt_user);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_MQTT_PASS, s_cfg.mqtt_pass);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_DEVICE_ID, s_cfg.device_id);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_DEVICE_NAME, s_cfg.device_name);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_WEB_USER, s_cfg.web_user);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_WEB_PASS, s_cfg.web_pass);
    if (err == ESP_OK) err = nvs_commit(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "写 NVS 失败（%s）", esp_err_to_name(err));
    }
    nvs_close(h);
    return err;
}

static void load_from_nvs(void)
{
    nvs_handle_t h;
    /* 全新设备命名空间还不存在：READONLY 打开失败就是"没存过配置"，正常 */
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    read_str(h, KEY_SSID, s_cfg.wifi_ssid, sizeof(s_cfg.wifi_ssid));
    read_str(h, KEY_PASS, s_cfg.wifi_pass, sizeof(s_cfg.wifi_pass));
    read_str(h, KEY_URL, s_cfg.ha_url, sizeof(s_cfg.ha_url));
    read_str(h, KEY_TOKEN, s_cfg.ha_token, sizeof(s_cfg.ha_token));
    read_str(h, KEY_MQTT_HOST, s_cfg.mqtt_host, sizeof(s_cfg.mqtt_host));
    read_str(h, KEY_MQTT_USER, s_cfg.mqtt_user, sizeof(s_cfg.mqtt_user));
    read_str(h, KEY_MQTT_PASS, s_cfg.mqtt_pass, sizeof(s_cfg.mqtt_pass));
    read_str(h, KEY_DEVICE_ID, s_cfg.device_id, sizeof(s_cfg.device_id));
    read_str(h, KEY_DEVICE_NAME, s_cfg.device_name, sizeof(s_cfg.device_name));
    read_str(h, KEY_WEB_USER, s_cfg.web_user, sizeof(s_cfg.web_user));
    read_str(h, KEY_WEB_PASS, s_cfg.web_pass, sizeof(s_cfg.web_pass));

    uint16_t port = 0;
    if (nvs_get_u16(h, KEY_MQTT_PORT, &port) == ESP_OK) {
        s_cfg.mqtt_port = port;
    }
    nvs_close(h);
}

/* NVS 为空（或某一项缺失）时用 secrets.h 兜底 */
static void seed_defaults(void)
{
    bool seeded = false;

    if (s_cfg.wifi_ssid[0] == '\0' && APP_SECRET_WIFI_SSID[0] != '\0') {
        copy_str(s_cfg.wifi_ssid, sizeof(s_cfg.wifi_ssid), APP_SECRET_WIFI_SSID);
        copy_str(s_cfg.wifi_pass, sizeof(s_cfg.wifi_pass), APP_SECRET_WIFI_PASSWORD);
        seeded = true;
    }
    if (s_cfg.ha_url[0] == '\0' && APP_SECRET_HA_URL[0] != '\0') {
        copy_str(s_cfg.ha_url, sizeof(s_cfg.ha_url), APP_SECRET_HA_URL);
        copy_str(s_cfg.ha_token, sizeof(s_cfg.ha_token), APP_SECRET_HA_TOKEN);
        seeded = true;
    }
    if (s_cfg.mqtt_host[0] == '\0' && APP_SECRET_MQTT_HOST[0] != '\0') {
        copy_str(s_cfg.mqtt_host, sizeof(s_cfg.mqtt_host), APP_SECRET_MQTT_HOST);
        copy_str(s_cfg.mqtt_user, sizeof(s_cfg.mqtt_user), APP_SECRET_MQTT_USER);
        copy_str(s_cfg.mqtt_pass, sizeof(s_cfg.mqtt_pass), APP_SECRET_MQTT_PASS);
        seeded = true;
    }
    if (s_cfg.web_user[0] == '\0') {
        copy_str(s_cfg.web_user, sizeof(s_cfg.web_user), APP_SECRET_WEB_USER);
        copy_str(s_cfg.web_pass, sizeof(s_cfg.web_pass), APP_SECRET_WEB_PASS);
        ESP_LOGW(TAG, "设置页登录：%s / %s（默认口令，请尽快在网页里改掉）",
                 s_cfg.web_user, s_cfg.web_pass);
        seeded = true;
    }
    if (s_cfg.device_name[0] == '\0' && APP_SECRET_DEVICE_NAME[0] != '\0') {
        copy_str(s_cfg.device_name, sizeof(s_cfg.device_name), APP_SECRET_DEVICE_NAME);
        seeded = true;
    }
    if (s_cfg.device_id[0] == '\0') {
        copy_str(s_cfg.device_id, sizeof(s_cfg.device_id),
                 APP_SECRET_DEVICE_ID[0] ? APP_SECRET_DEVICE_ID : FALLBACK_DEVICE_ID);
        seeded = true;
    }
    if (s_cfg.mqtt_port <= 0) {
        s_cfg.mqtt_port = (APP_SECRET_MQTT_PORT > 0) ? APP_SECRET_MQTT_PORT : 1883;
    }

    if (seeded && s_nvs_ok) {
        ESP_LOGI(TAG, "首次启动：把 secrets.h 的默认值写入 NVS");
        (void)config_save();    /* 失败已由 config_save 打日志，不阻塞开机 */
    }
}

esp_err_t app_settings_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    memset(&s_cfg, 0, sizeof(s_cfg));

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 需要格式化（%s），擦除后重试", esp_err_to_name(err));
        if (nvs_flash_erase() == ESP_OK) {
            err = nvs_flash_init();
        }
    }

    if (err == ESP_OK) {
        s_nvs_ok = true;
        load_from_nvs();
    } else {
        ESP_LOGE(TAG, "NVS 初始化失败（%s），配置改不了也存不下，先用 secrets.h 的值",
                 esp_err_to_name(err));
    }

    seed_defaults();

    /* 日志里不要把口令打出来，只说有没有 */
    ESP_LOGI(TAG, "WiFi '%s'（%s）| HA '%s'（%s）| MQTT '%s:%d'（%s）| 设备 id '%s' | %s",
             s_cfg.wifi_ssid, s_cfg.wifi_pass[0] ? "有密码" : "无密码",
             s_cfg.ha_url, s_cfg.ha_token[0] ? "有令牌" : "无令牌",
             s_cfg.mqtt_host, s_cfg.mqtt_port,
             s_cfg.mqtt_host[0] ? "启用" : "未启用",
             s_cfg.device_id, s_nvs_ok ? "NVS 可用" : "NVS 不可用");

    s_ready = true;
    return ESP_OK;
}

const app_settings_t *app_settings_get(void)
{
    /* 懒加载：任何模块第一次取配置时自动初始化，不必依赖 main.c 的调用顺序。
       并发进来时最多就是两边都播种一次（写进去的值完全一样），无副作用。 */
    if (!s_ready) {
        (void)app_settings_init();
    }
    return &s_cfg;
}

esp_err_t app_settings_update(const char *ssid, const char *pass, const char *ha_url, const char *ha_token)
{
    if (ssid) {
        copy_str(s_cfg.wifi_ssid, sizeof(s_cfg.wifi_ssid), ssid);
    }
    if (pass) {
        copy_str(s_cfg.wifi_pass, sizeof(s_cfg.wifi_pass), pass);
    }
    if (ha_url) {
        copy_str(s_cfg.ha_url, sizeof(s_cfg.ha_url), ha_url);
        /* 容忍结尾斜杠：拼 URL 的地方都按不带斜杠处理 */
        size_t n = strlen(s_cfg.ha_url);
        while (n > 0 && s_cfg.ha_url[n - 1] == '/') {
            s_cfg.ha_url[--n] = '\0';
        }
    }
    if (ha_token) {
        copy_str(s_cfg.ha_token, sizeof(s_cfg.ha_token), ha_token);
    }
    return config_save();
}

esp_err_t app_settings_update_name(const char *name)
{
    if (name != NULL && name[0] != '\0') {
        copy_str(s_cfg.device_name, sizeof(s_cfg.device_name), name);
    }
    return config_save();
}

esp_err_t app_settings_update_web(const char *user, const char *pass)
{
    if (user) {
        copy_str(s_cfg.web_user, sizeof(s_cfg.web_user), user);
    }
    if (pass) {
        copy_str(s_cfg.web_pass, sizeof(s_cfg.web_pass), pass);
    }
    return config_save();
}

esp_err_t app_settings_update_mqtt(const char *host, int port, const char *user, const char *pass,
                                   const char *device_id)
{
    if (host) {
        copy_str(s_cfg.mqtt_host, sizeof(s_cfg.mqtt_host), host);
    }
    if (port > 0) {
        s_cfg.mqtt_port = port;
    }
    if (user) {
        copy_str(s_cfg.mqtt_user, sizeof(s_cfg.mqtt_user), user);
    }
    if (pass) {
        copy_str(s_cfg.mqtt_pass, sizeof(s_cfg.mqtt_pass), pass);
    }
    if (device_id) {
        copy_str(s_cfg.device_id, sizeof(s_cfg.device_id), device_id);
    }
    return config_save();
}

bool app_settings_has_wifi(void)
{
    return app_settings_get()->wifi_ssid[0] != '\0';
}

bool app_settings_has_ha(void)
{
    const app_settings_t *cfg = app_settings_get();
    return cfg->ha_url[0] != '\0' && cfg->ha_token[0] != '\0';
}
