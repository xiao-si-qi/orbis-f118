#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lvgl.h"
#include "ha_client.h"
#include "detail.h"

LV_FONT_DECLARE(lv_font_cjk16);

static const char *TAG = "detail";

static lv_obj_t *s_page;
static lv_obj_t *s_title;
static lv_obj_t *s_switch;
static lv_obj_t *s_br_row, *s_br_slider, *s_br_val;
static lv_obj_t *s_ct_row, *s_ct_slider, *s_ct_val;
static char s_entity[96];
static bool s_open;
static bool s_pressing;

/* 本地覆盖：拖动后 6 秒内不被轮询覆盖 */
static int s_br_override, s_ct_override;
static uint32_t s_br_until, s_ct_until;

static void slider_event_cb(lv_event_t *e)
{
    lv_obj_t *sl = lv_event_get_target(e);
    const lv_event_code_t code = lv_event_get_code(e);
    const int v = (int)lv_slider_get_value(sl);

    if (code == LV_EVENT_PRESSED) {
        s_pressing = true;
        return;
    }
    if (code != LV_EVENT_RELEASED) {
        /* 拖动过程中只更新显示 */
        if (sl == s_br_slider) lv_label_set_text_fmt(s_br_val, "%d%%", v);
        else lv_label_set_text_fmt(s_ct_val, "%dK", v);
        return;
    }
    s_pressing = false;

    char attrs[64];
    if (sl == s_br_slider) {
        s_br_override = v;
        s_br_until = lv_tick_get() + 6000;
        lv_label_set_text_fmt(s_br_val, "%d%%", v);
        snprintf(attrs, sizeof(attrs), "{\"brightness_pct\":%d}", v);
        ESP_LOGI(TAG, "亮度 -> %d%%", v);
    } else {
        s_ct_override = v;
        s_ct_until = lv_tick_get() + 6000;
        lv_label_set_text_fmt(s_ct_val, "%dK", v);
        snprintf(attrs, sizeof(attrs), "{\"color_temp_kelvin\":%d}", v);
        ESP_LOGI(TAG, "色温 -> %dK", v);
    }
    ha_client_action_attrs(s_entity, "turn_on", attrs);
}

static void switch_event_cb(lv_event_t *e)
{
    const bool on = lv_obj_has_state(s_switch, LV_STATE_CHECKED);
    ESP_LOGI(TAG, "开关 -> %s", on ? "开" : "关");
    ha_client_action(s_entity, on ? "turn_on" : "turn_off");
}

static void back_event_cb(lv_event_t *e)
{
    (void)e;
    detail_close();
}

static void build_once(void)
{
    if (s_page != NULL) return;
    lv_obj_t *scr = lv_screen_active();

    s_page = lv_obj_create(scr);
    lv_obj_remove_style_all(s_page);
    lv_obj_set_size(s_page, 360, 360);
    lv_obj_set_pos(s_page, 0, 0);
    lv_obj_set_style_bg_color(s_page, lv_color_hex(0x0B0B0D), 0);
    lv_obj_set_style_bg_opa(s_page, LV_OPA_COVER, 0);
    lv_obj_remove_flag(s_page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_page, LV_OBJ_FLAG_HIDDEN);

    /* 返回键：圆形屏安全区（y≈30 处半宽约 106px） */
    lv_obj_t *back = lv_button_create(s_page);
    lv_obj_set_size(back, 40, 40);
    lv_obj_align(back, LV_ALIGN_TOP_MID, -96, 34);
    lv_obj_set_style_bg_opa(back, LV_OPA_20, 0);
    lv_obj_set_style_radius(back, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_event_cb(back, back_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT);
    lv_obj_center(bl);

    s_title = lv_label_create(s_page);
    lv_label_set_text(s_title, "--");
    lv_obj_set_style_text_font(s_title, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_title, lv_color_white(), 0);
    lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 60);

    s_switch = lv_switch_create(s_page);
    lv_obj_set_size(s_switch, 84, 44);
    lv_obj_align(s_switch, LV_ALIGN_TOP_MID, 0, 92);
    lv_obj_add_event_cb(s_switch, switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* 亮度行 */
    s_br_row = lv_obj_create(s_page);
    lv_obj_remove_style_all(s_br_row);
    lv_obj_set_size(s_br_row, 300, 70);
    lv_obj_align(s_br_row, LV_ALIGN_TOP_MID, 0, 150);

    lv_obj_t *br_lab = lv_label_create(s_br_row);
    lv_label_set_text(br_lab, "亮度");
    lv_obj_set_style_text_font(br_lab, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(br_lab, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(br_lab, LV_ALIGN_TOP_LEFT, 0, 0);

    s_br_val = lv_label_create(s_br_row);
    lv_label_set_text(s_br_val, "--");
    lv_obj_set_style_text_font(s_br_val, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_br_val, lv_color_white(), 0);
    lv_obj_align(s_br_val, LV_ALIGN_TOP_RIGHT, 0, 0);

    s_br_slider = lv_slider_create(s_br_row);
    lv_obj_set_width(s_br_slider, 300);
    lv_obj_set_height(s_br_slider, 12);
    lv_obj_align(s_br_slider, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_slider_set_range(s_br_slider, 1, 100);
    lv_obj_add_event_cb(s_br_slider, slider_event_cb, LV_EVENT_ALL, NULL);

    /* 色温行 */
    s_ct_row = lv_obj_create(s_page);
    lv_obj_remove_style_all(s_ct_row);
    lv_obj_set_size(s_ct_row, 300, 70);
    lv_obj_align(s_ct_row, LV_ALIGN_TOP_MID, 0, 232);

    lv_obj_t *ct_lab = lv_label_create(s_ct_row);
    lv_label_set_text(ct_lab, "色温");
    lv_obj_set_style_text_font(ct_lab, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(ct_lab, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(ct_lab, LV_ALIGN_TOP_LEFT, 0, 0);

    s_ct_val = lv_label_create(s_ct_row);
    lv_label_set_text(s_ct_val, "--");
    lv_obj_set_style_text_font(s_ct_val, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_ct_val, lv_color_white(), 0);
    lv_obj_align(s_ct_val, LV_ALIGN_TOP_RIGHT, 0, 0);

    s_ct_slider = lv_slider_create(s_ct_row);
    lv_obj_set_width(s_ct_slider, 300);
    lv_obj_set_height(s_ct_slider, 12);
    lv_obj_align(s_ct_slider, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_slider_set_range(s_ct_slider, 2700, 6500);
    lv_obj_add_event_cb(s_ct_slider, slider_event_cb, LV_EVENT_ALL, NULL);
}

void detail_open(const char *entity_id)
{
    build_once();
    snprintf(s_entity, sizeof(s_entity), "%s", entity_id);
    s_open = true;
    s_pressing = false;
    lv_obj_remove_flag(s_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_page);
    detail_refresh();
    ESP_LOGI(TAG, "打开详情: %s", s_entity);
}

void detail_close(void)
{
    if (!s_open) return;
    s_open = false;
    lv_obj_add_flag(s_page, LV_OBJ_FLAG_HIDDEN);
    ESP_LOGI(TAG, "关闭详情");
}

bool detail_is_open(void) { return s_open; }

void detail_refresh(void)
{
    if (!s_open || s_page == NULL) return;
    const ha_entity_t *e = ha_client_find(s_entity);
    if (e == NULL) return;

    lv_label_set_text(s_title, e->name[0] ? e->name : s_entity);

    if (!s_pressing) {
        if (e->on) lv_obj_add_state(s_switch, LV_STATE_CHECKED);
        else lv_obj_remove_state(s_switch, LV_STATE_CHECKED);
    }

    /* 亮度 */
    if (e->has_brightness) {
        lv_obj_remove_flag(s_br_row, LV_OBJ_FLAG_HIDDEN);
        if (!s_pressing || lv_tick_get() > s_br_until) {
            int pct = e->brightness * 100 / 255;
            if (pct < 1) pct = 1;
            lv_slider_set_value(s_br_slider, pct, LV_ANIM_OFF);
            lv_label_set_text_fmt(s_br_val, "%d%%", pct);
        }
    } else {
        lv_obj_add_flag(s_br_row, LV_OBJ_FLAG_HIDDEN);
    }

    /* 色温 */
    if (e->has_tint) {
        lv_obj_remove_flag(s_ct_row, LV_OBJ_FLAG_HIDDEN);
        int lo = e->min_k > 0 ? e->min_k : 2700;
        int hi = e->max_k > 0 ? e->max_k : 6500;
        lv_slider_set_range(s_ct_slider, lo, hi);
        if (lv_tick_get() > s_ct_until) {
            int k = e->color_temp_k > 0 ? e->color_temp_k : lo;
            lv_slider_set_value(s_ct_slider, k, LV_ANIM_OFF);
            lv_label_set_text_fmt(s_ct_val, "%dK", k);
        }
    } else {
        lv_obj_add_flag(s_ct_row, LV_OBJ_FLAG_HIDDEN);
    }
}
