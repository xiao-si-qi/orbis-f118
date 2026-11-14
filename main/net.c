#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "freertos/task.h"

#include "net.h"
#include "secrets.h"

#ifndef APP_FW_VERSION
#define APP_FW_VERSION "0.1.0"
#endif

static const char *TAG = "net";

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_eg;
static bool s_wifi_up;
static bool s_mqtt_up;
static bool s_need_meta;
static esp_mqtt_client_handle_t s_mqtt;
static char s_dev_id[48];
static char s_will_topic[96];

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_up = false;
        ESP_LOGW(TAG, "WiFi 断开，重连中…");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = (const ip_event_got_ip_t *)data;
        s_wifi_up = true;
        xEventGroupSetBits(s_eg, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi 已连接: " IPSTR, IP2STR(&ev->ip_info.ip));
    }
}

static void mqtt_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_up = true;
        s_need_meta = true;    /* 真正的发布放到主循环，避免在回调里调用 publish */
        ESP_LOGI(TAG, "MQTT 已连接 %s:%d", APP_SECRET_MQTT_HOST, APP_SECRET_MQTT_PORT);
        break;    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_up = false;
        ESP_LOGW(TAG, "MQTT 断开");
        break;
    default:
        break;
    }
}

esp_err_t net_wifi_start(void)
{
    s_eg = xEventGroupCreate();

    /* WiFi 驱动依赖 NVS，必须先初始化，否则 esp_wifi_init 会 abort 并进入重启循环 */
    esp_err_t nerr = nvs_flash_init();
    if (nerr == ESP_ERR_NVS_NO_FREE_PAGES || nerr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 需要擦除，重新初始化");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nerr = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nerr);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, APP_SECRET_WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, APP_SECRET_WIFI_PASSWORD, sizeof(wc.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi 启动，SSID=%s", APP_SECRET_WIFI_SSID);
    return ESP_OK;
}

bool net_wifi_connected(void) { return s_wifi_up; }

int net_wifi_rssi(void)
{
    wifi_ap_record_t ap;
    if (!s_wifi_up || esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return 0;
    return ap.rssi;
}

esp_err_t net_mqtt_start(void)
{
    snprintf(s_dev_id, sizeof(s_dev_id), "%s", APP_SECRET_DEVICE_ID[0] ? APP_SECRET_DEVICE_ID : "orbis-f118");

    char uri[96];
    snprintf(uri, sizeof(uri), "mqtt://%s:%d", APP_SECRET_MQTT_HOST, APP_SECRET_MQTT_PORT);
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = uri,
        .credentials.username = APP_SECRET_MQTT_USER,
        .credentials.authentication.password = APP_SECRET_MQTT_PASS,
        .session.keepalive = 30,
        .session.last_will.topic = s_will_topic,
        .session.last_will.msg = "offline",
        .session.last_will.msg_len = 7,
        .session.last_will.retain = 1,
        .session.last_will.qos = 1,
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event, NULL);
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt));
    ESP_LOGI(TAG, "MQTT 启动: %s (设备 id '%s')", uri, s_dev_id);
    return ESP_OK;
}

void net_mqtt_tick(void)
{
    if (!s_mqtt_up || !s_need_meta) return;
    s_need_meta = false;

    char t[96], payload[384];
    snprintf(t, sizeof(t), "switchyard/%s/availability", s_dev_id);
    int r1 = esp_mqtt_client_publish(s_mqtt, t, "online", 0, 1, 1);

    snprintf(t, sizeof(t), "switchyard/%s/meta", s_dev_id);
    snprintf(payload, sizeof(payload),
             "{\"type\":\"screen\",\"name\":\"ESP32-S31 旋钮屏\",\"fw\":\"" APP_FW_VERSION "\","
             "\"caps\":[\"fw\",\"rssi\",\"uptime\",\"online\",\"log\",\"reboot\"],"
             "\"display\":{\"w\":360,\"h\":360,\"formats\":[\"jpeg\"],\"fit\":[\"cover\",\"contain\"]}}");
    int r2 = esp_mqtt_client_publish(s_mqtt, t, payload, 0, 1, 1);

    snprintf(t, sizeof(t), "switchyard/%s/cmd", s_dev_id);
    esp_mqtt_client_subscribe(s_mqtt, t, 1);
    snprintf(t, sizeof(t), "switchyard/%s/cfg", s_dev_id);
    esp_mqtt_client_subscribe(s_mqtt, t, 1);

    ESP_LOGI(TAG, "meta 发布结果: availability=%d meta=%d", r1, r2);
}

void net_mqtt_publish_state(void)
{
    if (!s_mqtt_up) return;
    char t[96], payload[256];
    snprintf(t, sizeof(t), "switchyard/%s/state", s_dev_id);
    snprintf(payload, sizeof(payload),
             "{\"uptime\":%lu,\"rssi\":%d,\"fw\":\"" APP_FW_VERSION "\",\"online\":true,\"heap\":%u}",
             (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ),
             net_wifi_rssi(),
             (unsigned)(esp_get_free_heap_size() / 1024));
    esp_mqtt_client_publish(s_mqtt, t, payload, 0, 1, 1);
}

bool net_mqtt_connected(void) { return s_mqtt_up; }
const char *net_device_id(void) { return s_dev_id; }
