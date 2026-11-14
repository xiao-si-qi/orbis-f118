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

#include "net.h"
#include "secrets.h"

static const char *TAG = "net";

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_eg;
static bool s_wifi_up;
static bool s_mqtt_up;
static esp_mqtt_client_handle_t s_mqtt;
static char s_dev_id[48];

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
        ESP_LOGI(TAG, "MQTT 已连接 %s:%d", APP_SECRET_MQTT_HOST, APP_SECRET_MQTT_PORT);
        break;
    case MQTT_EVENT_DISCONNECTED:
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
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event, NULL);
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt));
    ESP_LOGI(TAG, "MQTT 启动: %s (设备 id '%s')", uri, s_dev_id);
    return ESP_OK;
}

bool net_mqtt_connected(void) { return s_mqtt_up; }
const char *net_device_id(void) { return s_dev_id; }
