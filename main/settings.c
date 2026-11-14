#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_wifi.h"
#include "net.h"
#include "ha_client.h"
#include "settings.h"

LV_FONT_DECLARE(lv_font_cjk16);

static const char *TAG = "settings";

static lv_obj_t *s_vals[6];

static void reboot_cb(lv_event_t *e)
{
    (void)e;
    ESP_LOGW(TAG, "用户请求重启");
    esp_restart();
}

void settings_build(lv_obj_t *parent)
{
    lv_obj_t *scr = parent;

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "参数");
    lv_obj_set_style_text_font(title, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 52);

    static const char *names[6] = { "WiFi", "IP", "信号", "HA", "固件", "运行" };
    for (int i = 0; i < 6; i++) {
        lv_obj_t *row = lv_obj_create(scr);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, 250, 30);
        lv_obj_align(row, LV_ALIGN_TOP_MID, 0, 92 + i * 32);
        lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *n = lv_label_create(row);
        lv_label_set_text(n, names[i]);
        lv_obj_set_style_text_font(n, &lv_font_cjk16, 0);
        lv_obj_set_style_text_color(n, lv_color_hex(0x8E8E93), 0);
        lv_obj_align(n, LV_ALIGN_LEFT_MID, 0, 0);

        s_vals[i] = lv_label_create(row);
        lv_label_set_text(s_vals[i], "--");
        lv_obj_set_style_text_font(s_vals[i], &lv_font_cjk16, 0);
        lv_obj_set_style_text_color(s_vals[i], lv_color_white(), 0);
        lv_obj_align(s_vals[i], LV_ALIGN_RIGHT_MID, 0, 0);
    }

    lv_obj_t *btn = lv_button_create(scr);
    lv_obj_set_size(btn, 140, 46);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -52);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF453A), 0);
    lv_obj_add_event_cb(btn, reboot_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, "重启设备");
    lv_obj_set_style_text_font(bl, &lv_font_cjk16, 0);
    lv_obj_center(bl);
}

void settings_refresh(void)
{
    if (s_vals[0] == NULL) return;

    lv_label_set_text(s_vals[0], net_wifi_connected() ? "已连接" : "断开");
    lv_obj_set_style_text_color(s_vals[0],
        net_wifi_connected() ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);

    char buf[48];
    lv_label_set_text(s_vals[1], net_ip_string());
    snprintf(buf, sizeof(buf), "%d dBm", net_wifi_rssi());
    lv_label_set_text(s_vals[2], buf);

    lv_label_set_text(s_vals[3], ha_client_online() ? "在线" : "离线");
    lv_obj_set_style_text_color(s_vals[3],
        ha_client_online() ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);

    lv_label_set_text(s_vals[4], "0.1.0");

    const uint32_t up = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    snprintf(buf, sizeof(buf), "%luh%02lum", (unsigned long)(up / 3600), (unsigned long)((up / 60) % 60));
    lv_label_set_text(s_vals[5], buf);
}
