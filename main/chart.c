#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lvgl.h"
#include "ha_client.h"
#include "chart.h"

LV_FONT_DECLARE(lv_font_cjk16);

static const char *TAG = "chart";

#define MAX_PTS 32

static lv_obj_t *s_page;
static lv_obj_t *s_title;
static lv_obj_t *s_canvas;
static lv_obj_t *s_span;
static lv_obj_t *s_min;
static lv_obj_t *s_avg;
static lv_obj_t *s_max;
static bool s_open;
static char s_entity[96];

static float s_pts[MAX_PTS];
static int s_n;
static float s_lo, s_hi;

/* 后台结果 */
static volatile bool s_ready;
static volatile bool s_busy;
static float s_r[MAX_PTS];
static int s_rn;
static float s_rmin, s_rmax, s_ravg;

static void draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    if (s_n < 2 || layer == NULL) return;

    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    const int32_t w = lv_area_get_width(&a);
    const int32_t h = lv_area_get_height(&a);
    if (w < 2 || h < 2) return;

    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.color = lv_color_hex(0x5AC8FA);
    d.width = 3;
    d.round_start = true;
    d.round_end = true;

    const float span = (s_hi - s_lo) > 0.0001f ? (s_hi - s_lo) : 1.0f;
    int32_t px = 0, py = 0;
    for (int i = 0; i < s_n; i++) {
        float t = (s_pts[i] - s_lo) / span;
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        const int32_t x = a.x1 + (int32_t)((int64_t)(w - 1) * i / (s_n - 1));
        const int32_t y = a.y2 - (int32_t)(t * (h - 1));
        if (i > 0) {
            d.p1.x = px; d.p1.y = py;
            d.p2.x = x;  d.p2.y = y;
            lv_draw_line(layer, &d);
        }
        px = x; py = y;
    }
}

static void back_cb(lv_event_t *e)
{
    (void)e;
    chart_close();
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

    lv_obj_t *back = lv_button_create(s_page);
    lv_obj_set_size(back, 40, 40);
    lv_obj_align(back, LV_ALIGN_TOP_MID, -96, 34);
    lv_obj_set_style_bg_opa(back, LV_OPA_20, 0);
    lv_obj_set_style_radius(back, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_event_cb(back, back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT);
    lv_obj_center(bl);

    s_title = lv_label_create(s_page);
    lv_label_set_text(s_title, "--");
    lv_obj_set_style_text_font(s_title, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_title, lv_color_white(), 0);
    lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 58);

    s_span = lv_label_create(s_page);
    lv_label_set_text(s_span, "加载中…");
    lv_obj_set_style_text_font(s_span, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_span, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(s_span, LV_ALIGN_TOP_MID, 0, 84);

    s_canvas = lv_obj_create(s_page);
    lv_obj_remove_style_all(s_canvas);
    lv_obj_set_size(s_canvas, 270, 130);
    lv_obj_align(s_canvas, LV_ALIGN_TOP_MID, 0, 112);
    lv_obj_remove_flag(s_canvas, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_canvas, draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_min = lv_label_create(s_page);
    lv_obj_set_style_text_font(s_min, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_min, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(s_min, LV_ALIGN_TOP_MID, -92, 254);

    s_avg = lv_label_create(s_page);
    lv_obj_set_style_text_font(s_avg, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_avg, lv_color_white(), 0);
    lv_obj_align(s_avg, LV_ALIGN_TOP_MID, 0, 254);

    s_max = lv_label_create(s_page);
    lv_obj_set_style_text_font(s_max, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_max, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(s_max, LV_ALIGN_TOP_MID, 92, 254);
}

static void chart_worker(void *arg)
{
    char entity[96];
    snprintf(entity, sizeof(entity), "%s", (const char *)arg);
    free(arg);

    int n = 0;
    float vmin = 0, vmax = 0, vavg = 0;
    const esp_err_t e = ha_client_history(entity, 6, s_r, MAX_PTS, &n, &vmin, &vmax, &vavg);
    if (e == ESP_OK && n > 0) {
        s_rn = n;
        s_rmin = vmin;
        s_rmax = vmax;
        s_ravg = vavg;
        s_ready = true;
    } else {
        ESP_LOGW(TAG, "取历史失败: %s", entity);
        s_rn = 0;
        s_ready = true;
    }
    s_busy = false;
    vTaskDelete(NULL);
}

void chart_open(const char *entity_id)
{
    build_once();
    snprintf(s_entity, sizeof(s_entity), "%s", entity_id);
    s_open = true;
    s_n = 0;
    s_ready = false;

    const ha_entity_t *ent = ha_client_find(s_entity);
    lv_label_set_text(s_title, ent ? ent->name : s_entity);
    lv_label_set_text(s_span, "加载中…");
    lv_label_set_text(s_min, "");
    lv_label_set_text(s_avg, "");
    lv_label_set_text(s_max, "");
    lv_obj_invalidate(s_canvas);

    lv_obj_remove_flag(s_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_page);
    ESP_LOGI(TAG, "打开折线: %s", s_entity);

    if (!s_busy) {
        s_busy = true;
        char *arg = strdup(s_entity);
        if (arg) xTaskCreate(chart_worker, "chart", 8192, arg, 4, NULL);
    }
}

void chart_close(void)
{
    if (!s_open) return;
    s_open = false;
    lv_obj_add_flag(s_page, LV_OBJ_FLAG_HIDDEN);
    ESP_LOGI(TAG, "关闭折线");
}

bool chart_is_open(void) { return s_open; }

void chart_poll(void)
{
    if (!s_open || !s_ready) return;
    s_ready = false;

    if (s_rn <= 0) {
        lv_label_set_text(s_span, "取不到历史数据");
        return;
    }
    s_n = s_rn;
    memcpy(s_pts, s_r, sizeof(float) * s_n);

    float lo = s_rmin, hi = s_rmax;
    if (hi - lo < 0.01f) { lo -= 1; hi += 1; }
    const float pad = (hi - lo) * 0.1f;
    s_lo = lo - pad;
    s_hi = hi + pad;
    lv_obj_invalidate(s_canvas);

    const ha_entity_t *ent = ha_client_find(s_entity);
    const char *unit = ent ? ent->unit : "";
    lv_label_set_text(s_span, "近 6 小时");
    lv_label_set_text_fmt(s_min, "最低 %.1f%s", s_rmin, unit);
    lv_label_set_text_fmt(s_avg, "平均 %.1f%s", s_ravg, unit);
    lv_label_set_text_fmt(s_max, "最高 %.1f%s", s_rmax, unit);
}
