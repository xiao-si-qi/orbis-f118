#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st77916.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lvgl.h"

LV_FONT_DECLARE(lv_font_cjk16);
#include "net.h"
#include "detail.h"
#include "bg_image.h"
#include "settings.h"
#include "chart.h"


static const char *TAG = "orbis-f118";

/* ---- 引脚（实测确认）---- */
#define PIN_SCLK 12
#define PIN_D0   14
#define PIN_D1   15
#define PIN_D2   16
#define PIN_D3   17
#define PIN_CS   13
#define PIN_RST  18
#define PIN_BL   5
#define PIN_TP_SDA 11
#define PIN_TP_SCL 10
#define LCD_W 360
#define LCD_H 360

static esp_lcd_panel_handle_t s_panel;
static lv_display_t *s_disp;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_touch;

/* 厂商初始化表（类型由 esp_lcd_st77916.h 提供）*/
#include "lcd_init_commands.h"

static volatile bool s_flush_done;

static bool on_color_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *ed, void *ctx)
{
    s_flush_done = true;      /* 由 lv_flush_cb 等待后统一通知 LVGL */
    return false;
}

static void lv_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    /* 同步等待本块传输完成再通知 LVGL：
       异步回调一旦丢一次，LVGL 会死等 flushing 标志（实测卡在 wait_for_flushing）。 */
    s_flush_done = false;
    esp_lcd_panel_draw_bitmap(s_panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);

    const int64_t deadline = esp_timer_get_time() + 200 * 1000;   /* 200 ms 兜底 */
    while (!s_flush_done && esp_timer_get_time() < deadline) {
        vTaskDelay(1);
    }
    s_flush_done = false;
    lv_display_flush_ready(disp);
}

static uint32_t tick_cb(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void backlight_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);
    ledc_channel_config_t c = {
        .gpio_num = PIN_BL, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0,
    };
    ledc_channel_config(&c);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1023 * 40 / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void lcd_init(void)
{
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_SCLK, .mosi_io_num = PIN_D0, .miso_io_num = PIN_D1,
        .quadwp_io_num = PIN_D2, .quadhd_io_num = PIN_D3,
        .max_transfer_sz = LCD_W * 80 * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_CS, .dc_gpio_num = -1, .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000, .trans_queue_depth = 10,
        .lcd_cmd_bits = 32, .lcd_param_bits = 8,
        .on_color_trans_done = on_color_done,
        .flags.quad_mode = 1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));

    st77916_vendor_config_t vendor = {
        .init_cmds = guition_lcd_init_cmds,
        .init_cmds_size = sizeof(guition_lcd_init_cmds) / sizeof(guition_lcd_init_cmds[0]),
        .flags.use_qspi_interface = 1,
    };
    esp_lcd_panel_dev_config_t pdc = {
        .reset_gpio_num = PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(io, &pdc, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    ESP_LOGI(TAG, "LCD 初始化完成");
}

static void lvgl_init(void)
{
    lv_init();
    lv_tick_set_cb(tick_cb);

    const size_t buf_bytes = LCD_W * 40 * sizeof(lv_color_t);
    void *b1 = heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM);
    void *b2 = heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM);
    s_disp = lv_display_create(LCD_W, LCD_H);
    lv_display_set_flush_cb(s_disp, lv_flush_cb);
    /* 面板要 RGB565 大端，LVGL 默认小端：显式声明字节交换 */
    lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
    lv_display_set_buffers(s_disp, b1, b2, buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    ESP_LOGI(TAG, "LVGL 初始化完成, 缓冲 %u KB x2", (unsigned)(buf_bytes / 1024));
}

static void touch_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint8_t reg = 0x02, buf[6] = {0};
    static lv_coord_t last_x, last_y;
    static int last_pressed = 0;
    if (i2c_master_transmit_receive(s_touch, &reg, 1, buf, 6, 50) == ESP_OK && (buf[0] & 0x01)) {
        last_x = ((buf[1] & 0x0F) << 8) | buf[2];
        last_y = ((buf[3] & 0x0F) << 8) | buf[4];
        data->state = LV_INDEV_STATE_PRESSED;
        if (!last_pressed) {
            ESP_LOGI(TAG, "触摸按下 x=%d y=%d", last_x, last_y);
            last_pressed = 1;
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        if (last_pressed) { ESP_LOGI(TAG, "触摸松开"); last_pressed = 0; }
    }
    data->point.x = last_x;
    data->point.y = last_y;
}

/* ---------- 编码器（GPIO6=B, GPIO7=A，轮询解码，不用中断）---------- */
static int s_enc_last;
static int s_enc_acc;

/*
 * 这个编码器每格只输出一个"半周期"脉冲（实测波形）：
 *   顺时针：11 -> 01 -> 11
 *   逆时针：11 -> 10 -> 11
 * 所以直接对 01/10 计数，回到 11 仅复位。
 */
static void encoder_task(void *arg)
{
    int last = (gpio_get_level(7) << 1) | gpio_get_level(6);
    while (1) {
        const int cur = (gpio_get_level(7) << 1) | gpio_get_level(6);
        if (cur != last) {
            if (cur == 0x2) {          /* A=0 B=1：顺时针（实测应减小） */
                s_enc_acc -= 1;
            } else if (cur == 0x1) {   /* A=1 B=0：逆时针（实测应增大） */
                s_enc_acc += 1;
            }
            last = cur;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
static void encoder_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << 6) | (1ULL << 7),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    s_enc_last = (gpio_get_level(7) << 1) | gpio_get_level(6);
    xTaskCreate(encoder_task, "enc", 3072, NULL, 6, NULL);
    ESP_LOGI(TAG, "编码器初始化完成 (A=7 B=6，1ms 采样任务)");
}

static int8_t encoder_read_diff(void)
{
    const int acc = s_enc_acc;
    if (acc == 0) return 0;
    s_enc_acc = 0;
    if (acc > 4) return 4;
    if (acc < -4) return -4;
    return (int8_t)acc;
}
/* ---------- 触觉 DRV2605L @0x5A ---------- */
static i2c_master_dev_handle_t s_haptic;

static void haptic_init(void)
{
    i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = 0x5A, .scl_speed_hz = 100000 };
    if (i2c_master_bus_add_device(s_i2c_bus, &dc, &s_haptic) != ESP_OK) {
        ESP_LOGW(TAG, "触觉芯片添加失败");
        return;
    }
    uint8_t mode = 0x00;   /* 内部触发 */
    i2c_master_transmit(s_haptic, (uint8_t[]){ 0x01, mode }, 2, 50);
    uint8_t seq = 0x01;    /* 强点击 */
    i2c_master_transmit(s_haptic, (uint8_t[]){ 0x04, seq }, 2, 50);
    ESP_LOGI(TAG, "触觉初始化完成 (DRV2605L @0x5A)");
}

static void haptic_click(void)
{
    if (s_haptic == NULL) return;
    i2c_master_transmit(s_haptic, (uint8_t[]){ 0x0C, 0x01 }, 2, 50);   /* GO */
}
static void touch_init(void)
{
    i2c_master_bus_config_t bc = {
        .i2c_port = -1, .sda_io_num = PIN_TP_SDA, .scl_io_num = PIN_TP_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &s_i2c_bus));
    i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = 0x15, .scl_speed_hz = 100000 };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &dc, &s_touch));
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read);
    ESP_LOGI(TAG, "触摸初始化完成 (CST816S @0x15)");
}

#include "ha_client.h"

/* ---------------- 电池 ADC（GPIO55，15k/10k 分压）---------------- */
#include "esp_adc/adc_oneshot.h"
static volatile int s_bat_mv;

static void battery_task(void *arg)
{
    adc_unit_t unit; adc_channel_t chan;
    if (adc_oneshot_io_to_channel(55, &unit, &chan) != ESP_OK) {
        ESP_LOGW(TAG, "GPIO55 不是 ADC 引脚");
        vTaskDelete(NULL); return;
    }
    adc_oneshot_unit_init_cfg_t uic = { .unit_id = unit };
    adc_oneshot_unit_handle_t au = NULL;
    if (adc_oneshot_new_unit(&uic, &au) != ESP_OK) { vTaskDelete(NULL); return; }

    adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    esp_err_t e = adc_oneshot_config_channel(au, chan, &cc);
    if (e != ESP_OK) { cc.atten = ADC_ATTEN_DB_6;  e = adc_oneshot_config_channel(au, chan, &cc); }
    if (e != ESP_OK) { cc.atten = ADC_ATTEN_DB_2_5; e = adc_oneshot_config_channel(au, chan, &cc); }
    if (e != ESP_OK) { cc.atten = ADC_ATTEN_DB_0;   e = adc_oneshot_config_channel(au, chan, &cc); }
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "电池 ADC 配置失败: %s", esp_err_to_name(e));
        vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "电池 ADC 就绪 (unit=%d chan=%d)", (int)unit, (int)chan);

    while (1) {
        int raw = 0, sum = 0, n = 0;
        for (int i = 0; i < 8; i++) {
            if (adc_oneshot_read(au, chan, &raw) == ESP_OK) { sum += raw; n++; }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (n > 0) {
            const float pin_v = (float)(sum / n) * 3.3f / 4095.0f;
            s_bat_mv = (int)(pin_v * 2.5f * 1000.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ---------------- 卡片界面 ---------------- */

typedef struct {
    char entity[96];
    char name[64];
    lv_obj_t *card;
    lv_obj_t *value;
    bool toggle;
} card_t;

static lv_group_t *s_card_group;
static lv_indev_t *s_enc_indev;
static lv_obj_t *s_page_main;
static lv_obj_t *s_page_set;
static int s_cur_page;

static const char *EID_LIGHT   = "light.xiaomi_cn_752752842_ceil05_s_2_light";
static const char *EID_POWER   = "sensor.zong_gong_lu";
static const char *EID_WATER   = "sensor.wm_001_temp";
static const char *EID_WEATHER = "weather.he_feng_tian_qi";

static card_t s_cards[] = {
    { "light.xiaomi_cn_752752842_ceil05_s_2_light", "客厅主灯", NULL, NULL, true  },
    { "sensor.zong_gong_lu",                        "总功率",   NULL, NULL, false },
    { "sensor.wm_001_temp",                         "水温",     NULL, NULL, false },
    { "weather.he_feng_tian_qi",                    "天气",     NULL, NULL, false },
};
static int s_card_count = 4;

static void card_clicked_cb(lv_event_t *e)
{
    card_t *c = (card_t *)lv_event_get_user_data(e);
    if (c == NULL) return;
    haptic_click();
    if (c->toggle) {
        detail_open(c->entity);          /* 灯：打开详情页（开关/亮度/色温） */
    } else if (strncmp(c->entity, "sensor.", 7) == 0 || strncmp(c->entity, "weather.", 8) == 0) {
        chart_open(c->entity);           /* 传感器/天气：历史折线 */
    } else {
        ESP_LOGI(TAG, "点击 %s（仅显示）", c->entity);
    }
}

static void show_page(int idx)
{
    if (idx < 0) idx = 0;
    if (idx > 1) idx = 1;
    s_cur_page = idx;
    if (s_page_main) idx == 0 ? lv_obj_remove_flag(s_page_main, LV_OBJ_FLAG_HIDDEN)
                              : lv_obj_add_flag(s_page_main, LV_OBJ_FLAG_HIDDEN);
    if (s_page_set)  idx == 1 ? lv_obj_remove_flag(s_page_set, LV_OBJ_FLAG_HIDDEN)
                              : lv_obj_add_flag(s_page_set, LV_OBJ_FLAG_HIDDEN);
}

static void gesture_cb(lv_event_t *e)
{
    const lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
    if (detail_is_open() || chart_is_open()) return;
    if (dir == LV_DIR_LEFT) show_page(s_cur_page + 1);
    else if (dir == LV_DIR_RIGHT) show_page(s_cur_page - 1);
}

static void card_focus_cb(lv_event_t *e)
{
    (void)e;
    haptic_click();
}

static lv_obj_t *make_card(lv_obj_t *scr, int x, int y, const char *name, card_t *ctx)
{
    lv_obj_t *card = lv_obj_create(scr);
    lv_obj_set_size(card, 150, 100);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_style_bg_color(card, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(card, (lv_opa_t)(255 * 62 / 100), 0);   /* 黑色半透明，透出背景图 */
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_set_style_pad_all(card, 12, 0);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(card, card_clicked_cb, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(card, card_focus_cb, LV_EVENT_FOCUSED, NULL);
    /* 旋钮选中时的描边 */
    lv_obj_set_style_outline_width(card, 3, LV_STATE_FOCUSED);
    lv_obj_set_style_outline_color(card, lv_color_hex(0x0A84FF), LV_STATE_FOCUSED);
    lv_obj_set_style_outline_pad(card, 3, LV_STATE_FOCUSED);

    lv_obj_t *lab = lv_label_create(card);
    lv_label_set_text(lab, name);
    lv_obj_set_style_text_font(lab, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(lab, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(lab, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *val = lv_label_create(card);
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_font(val, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(val, lv_color_white(), 0);
    lv_obj_align(val, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    return val;
}

static void update_cards(void)
{
    for (int i = 0; i < s_card_count; i++) {
        card_t *c = &s_cards[i];
        const ha_entity_t *e = ha_client_find(c->entity);
        if (c->value == NULL) continue;
        if (e == NULL || !e->fresh) {
            lv_label_set_text(c->value, "--");
        } else if (e->unit[0]) {
            lv_label_set_text_fmt(c->value, "%s %s", e->state, e->unit);
        } else {
            lv_label_set_text(c->value, e->state);
        }
        if (c->card) {
            lv_obj_set_style_bg_color(c->card,
                (e && e->on) ? lv_color_hex(0x1F2A44) : lv_color_hex(0x141418), 0);
        }
    }
}

static lv_obj_t *s_bat_label;
static lv_obj_t *s_rssi_bars[4];
static lv_obj_t *s_ha_dot;

static void update_statusbar(void)
{
    if (s_bat_label) {
        const int mv = s_bat_mv;
        /* 只显示合理范围（3.0–4.3 V）——GPIO55 目前读到的是饱和值，不要误导 */
        if (mv >= 3000 && mv <= 4300) {
            int pct = (mv - 3200) * 100 / (4200 - 3200);
            if (pct < 0) pct = 0;
            if (pct > 100) pct = 100;
            lv_label_set_text_fmt(s_bat_label, "%d.%02dV %d%%", mv / 1000, (mv % 1000) / 10, pct);
            lv_obj_set_style_text_color(s_bat_label,
                pct > 30 ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);
        } else {
            lv_label_set_text(s_bat_label, "");
        }
    }
    int level = 0;
    if (net_wifi_connected()) {
        const int r = net_wifi_rssi();
        level = (r >= -55) ? 4 : (r >= -67) ? 3 : (r >= -78) ? 2 : 1;
    }
    for (int i = 0; i < 4; i++) {
        if (s_rssi_bars[i]) {
            lv_obj_set_style_bg_color(s_rssi_bars[i],
                (i < level) ? lv_color_hex(0x30D158) : lv_color_hex(0x3A3A3C), 0);
        }
    }
    if (s_ha_dot) {
        lv_obj_set_style_bg_color(s_ha_dot,
            ha_client_online() ? lv_color_hex(0x30D158) : lv_color_hex(0xFF453A), 0);
    }
}

static void ui_timer_cb(lv_timer_t *t)
{
    (void)t;
    update_cards();
    update_statusbar();
    detail_refresh();
    if (s_cur_page == 1) settings_refresh();
    chart_poll();
}

static void encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    /* 震动由卡片焦点回调负责，这里只上报转动量 */
    data->enc_diff = encoder_read_diff();
    data->state = LV_INDEV_STATE_RELEASED;
}

#include "layout.h"

static void build_cards_ui(void);

static void apply_cfg(const char *json)
{
    bg_image_apply_json(json);

    /* layout_t 约 11 KB，必须放堆上：app_main 的栈只有 3.5 KB */
    layout_t *lay = calloc(1, sizeof(layout_t));
    if (lay == NULL) {
        ESP_LOGE(TAG, "布局内存不足");
        return;
    }
    const esp_err_t perr = layout_parse(json, lay);
    if (perr != ESP_OK || lay->page_count <= 0) {
        ESP_LOGW(TAG, "cfg 里没有可用布局 (%s)", esp_err_to_name(perr));
        free(lay);
        return;
    }

    const layout_page_t *pg = &lay->pages[0];
    int n = pg->tile_count;
    if (n > 4) n = 4;
    if (n <= 0) { free(lay); return; }

    /* 清空旧卡片与旧的按键组 */
    if (s_card_group) { lv_group_delete(s_card_group); s_card_group = NULL; }
    lv_obj_clean(s_page_main);

    for (int i = 0; i < n; i++) {
        snprintf(s_cards[i].entity, sizeof(s_cards[i].entity), "%s", pg->tiles[i].entity);
        snprintf(s_cards[i].name, sizeof(s_cards[i].name), "%s", pg->tiles[i].label[0] ? pg->tiles[i].label : "实体");
        s_cards[i].toggle = (pg->tiles[i].type == TILE_TOGGLE);
        s_cards[i].value = NULL;
        s_cards[i].card = NULL;
    }
    s_card_count = n;

    for (int i = 0; i < n; i++) {
        ha_client_register(s_cards[i].entity, s_cards[i].name);
    }
    build_cards_ui();
    ESP_LOGI(TAG, "cfg 已应用：%d 张卡片", n);
}

static void build_cards_ui(void)
{
    lv_obj_t *scr = s_page_main;
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    /* ---- 顶部状态栏：HA 点 / WiFi 信号格 / 电池 ---- */
    lv_obj_t *bar = lv_obj_create(scr);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, 190, 24);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 44);
    lv_obj_remove_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    s_ha_dot = lv_obj_create(bar);
    lv_obj_remove_style_all(s_ha_dot);
    lv_obj_set_size(s_ha_dot, 10, 10);
    lv_obj_set_style_radius(s_ha_dot, 5, 0);
    lv_obj_set_style_bg_color(s_ha_dot, lv_color_hex(0x8E8E93), 0);
    lv_obj_set_style_bg_opa(s_ha_dot, LV_OPA_COVER, 0);
    lv_obj_align(s_ha_dot, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *ha_lab = lv_label_create(bar);
    lv_label_set_text(ha_lab, "HA");
    lv_obj_set_style_text_font(ha_lab, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ha_lab, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(ha_lab, LV_ALIGN_LEFT_MID, 15, 0);

    for (int i = 0; i < 4; i++) {
        s_rssi_bars[i] = lv_obj_create(bar);
        lv_obj_remove_style_all(s_rssi_bars[i]);
        lv_obj_set_size(s_rssi_bars[i], 5, 6 + i * 4);
        lv_obj_set_style_bg_color(s_rssi_bars[i], lv_color_hex(0x3A3A3C), 0);
        lv_obj_set_style_bg_opa(s_rssi_bars[i], LV_OPA_COVER, 0);
        lv_obj_set_style_radius(s_rssi_bars[i], 1, 0);
        lv_obj_align(s_rssi_bars[i], LV_ALIGN_LEFT_MID, 78 + i * 8, 4);
    }

    s_bat_label = lv_label_create(bar);
    lv_label_set_text(s_bat_label, "---");
    lv_obj_set_style_text_font(s_bat_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_bat_label, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(s_bat_label, LV_ALIGN_RIGHT_MID, -2, 0);

    const int x0 = 22, y0 = 84, gap = 14;
    for (int i = 0; i < s_card_count; i++) {
        const int col = i % 2, row = i / 2;
        s_cards[i].value = make_card(scr, x0 + col * (150 + gap), y0 + row * (100 + gap),
                                     s_cards[i].name, &s_cards[i]);
        s_cards[i].card = lv_obj_get_parent(s_cards[i].value);
        s_cards[i].card = lv_obj_get_parent(s_cards[i].value);
        if (s_card_group == NULL) s_card_group = lv_group_create();
        lv_group_add_obj(s_card_group, s_cards[i].card);
    }
    lv_timer_create(ui_timer_cb, 500, NULL);
}
void app_main(void)
{
    ESP_LOGI(TAG, "=========== orbis-f118 启动 ===========");
    ESP_LOGI(TAG, "PSRAM %u KB / 内部 RAM %u KB",
             (unsigned)(heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    backlight_init();
    lcd_init();
    lvgl_init();
    touch_init();
    encoder_init();
    haptic_init();
    s_enc_indev = lv_indev_create();
    lv_indev_set_type(s_enc_indev, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(s_enc_indev, encoder_read_cb);
    if (s_card_group == NULL) s_card_group = lv_group_create();
    lv_indev_set_group(s_enc_indev, s_card_group);
    xTaskCreate(battery_task, "bat", 4096, NULL, 3, NULL);
    net_wifi_start();
    net_mqtt_start();
    bg_image_init();

    /* 两个页面容器：0=卡片页，1=参数页 */
    s_page_main = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(s_page_main);
    lv_obj_set_size(s_page_main, 360, 360);
    lv_obj_set_pos(s_page_main, 0, 0);
    lv_obj_remove_flag(s_page_main, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_page_main, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(s_page_main, gesture_cb, LV_EVENT_GESTURE, NULL);

    s_page_set = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(s_page_set);
    lv_obj_set_size(s_page_set, 360, 360);
    lv_obj_set_pos(s_page_set, 0, 0);
    lv_obj_remove_flag(s_page_set, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_page_set, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(s_page_set, gesture_cb, LV_EVENT_GESTURE, NULL);

    build_cards_ui();
    settings_build(s_page_set);
    show_page(0);
    bg_image_bind(lv_screen_active());
    {
        static const char *bg_json =
            "{\"bg\":{\"images\":["
            "\"https://ha.xiaosiqi.cn/local/switchyard/orbis/bg/1.jpg\","
            "\"https://ha.xiaosiqi.cn/local/switchyard/orbis/bg/2.jpg\","
            "\"https://ha.xiaosiqi.cn/local/switchyard/orbis/bg/3.jpg\"],"
            "\"interval\":20,\"dim\":65,\"card\":60}}";
        bg_image_apply_json(bg_json);
        ESP_LOGI(TAG, "背景图配置已应用");
    }

    /* 等 WiFi 连上再初始化 HA */
    for (int i = 0; i < 60 && !net_wifi_connected(); i++) vTaskDelay(pdMS_TO_TICKS(500));
    if (net_wifi_connected()) {
        ha_client_init();
        for (int i = 0; i < s_card_count; i++) ha_client_register(s_cards[i].entity, s_cards[i].name);
        ha_client_fetch_all();
        ha_client_start_poll();
        ESP_LOGI(TAG, "HA 已启动，注册 %d 个实体", s_card_count);
    } else {
        ESP_LOGW(TAG, "WiFi 未连接，跳过 HA 初始化");
    }
    ESP_LOGI(TAG, "进入 LVGL 主循环");

    uint32_t last_beat = 0;
    while (1) {
        uint32_t d = lv_timer_handler();
        if (esp_timer_get_time() / 1000000ULL != last_beat) {
            last_beat = (uint32_t)(esp_timer_get_time() / 1000000ULL);
            if (last_beat % 5 == 0) {
                net_mqtt_tick();
                {
                    static char cfg_json[4096];
                    if (net_take_cfg(cfg_json, sizeof(cfg_json))) apply_cfg(cfg_json);
                }
                net_mqtt_publish_state();
                ESP_LOGI(TAG, "电池: %d mV | 网络: wifi=%d rssi=%d mqtt=%d", s_bat_mv, net_wifi_connected(), net_wifi_rssi(), net_mqtt_connected());
                ESP_LOGI(TAG, "心跳 %u s，内部 RAM %u KB，PSRAM %u KB",
                         last_beat,
                         (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                         (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
            }
        }
        if (d > 10) d = 10;
        if (d == 0) d = 1;
        vTaskDelay(pdMS_TO_TICKS(d));
    }
}
