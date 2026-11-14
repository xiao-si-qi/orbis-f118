#include <stdio.h>
#include <string.h>
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
#include "net.h"

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

static bool on_color_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *ed, void *ctx)
{
    lv_display_flush_ready(s_disp);
    return false;
}

static void lv_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_draw_bitmap(s_panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
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

static lv_obj_t *s_tap_label;
static lv_obj_t *s_net_label;
static int s_tap_count = 0;

static void tap_cb(lv_event_t *e)
{
    s_tap_count++;
    if (s_tap_label) {
        lv_label_set_text_fmt(s_tap_label, "taps: %d", s_tap_count);
    }
    ESP_LOGI("orbis-f118", "按钮被点击，第 %d 次", s_tap_count);
}

static lv_obj_t *s_enc_label;
static int s_enc_value = 50;

static void encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    const int8_t diff = encoder_read_diff();
    if (diff != 0) {
        s_enc_value += diff * 5;
        if (s_enc_value < 0) s_enc_value = 0;
        if (s_enc_value > 100) s_enc_value = 100;
        if (s_enc_label) lv_label_set_text_fmt(s_enc_label, "value: %d", s_enc_value);
        ESP_LOGI(TAG, "旋钮: value=%d", s_enc_value);
        haptic_click();
        data->enc_diff = diff;
    } else {
        data->enc_diff = 0;
    }
    data->state = LV_INDEV_STATE_RELEASED;
}

static void build_test_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Orbis on F118");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 50);

    lv_obj_t *sub = lv_label_create(scr);
    lv_label_set_text(sub, "ESP32-S31 / LVGL 9.6.0");
    lv_obj_set_style_text_color(sub, lv_color_hex(0x8E8E93), 0);
    lv_obj_align(sub, LV_ALIGN_TOP_MID, 0, 92);

    lv_obj_t *btn = lv_button_create(scr);
    lv_obj_set_size(btn, 170, 64);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 10);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0A84FF), 0);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "TAP ME");
    lv_obj_center(lbl);

    s_net_label = lv_label_create(scr);
    lv_label_set_text(s_net_label, "WiFi --  MQTT --");
    lv_obj_set_style_text_color(s_net_label, lv_color_hex(0x64D2FF), 0);
    lv_obj_align(s_net_label, LV_ALIGN_TOP_MID, 0, 130);

    s_enc_label = lv_label_create(scr);
    lv_label_set_text_fmt(s_enc_label, "value: %d", s_enc_value);
    lv_obj_set_style_text_font(s_enc_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_enc_label, lv_color_hex(0xFFD60A), 0);
    lv_obj_align(s_enc_label, LV_ALIGN_BOTTOM_MID, 0, -110);

    s_tap_label = lv_label_create(scr);
    lv_label_set_text(s_tap_label, "taps: 0");
    lv_obj_set_style_text_color(s_tap_label, lv_color_hex(0x30D158), 0);
    lv_obj_align(s_tap_label, LV_ALIGN_BOTTOM_MID, 0, -70);

    lv_obj_add_event_cb(btn, tap_cb, LV_EVENT_CLICKED, NULL);
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
    lv_indev_t *enc = lv_indev_create();
    lv_indev_set_type(enc, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(enc, encoder_read_cb);
    net_wifi_start();
    net_mqtt_start();
    build_test_ui();
    ESP_LOGI(TAG, "进入 LVGL 主循环");

    uint32_t last_beat = 0;
    while (1) {
        uint32_t d = lv_timer_handler();
        if (esp_timer_get_time() / 1000000ULL != last_beat) {
            last_beat = (uint32_t)(esp_timer_get_time() / 1000000ULL);
            if (last_beat % 5 == 0) {
                ESP_LOGI(TAG, "网络: wifi=%d rssi=%d mqtt=%d", net_wifi_connected(), net_wifi_rssi(), net_mqtt_connected());
                if (s_net_label) {
                    lv_label_set_text_fmt(s_net_label, "WiFi %s  MQTT %s",
                                          net_wifi_connected() ? "OK" : "--",
                                          net_mqtt_connected() ? "OK" : "--");
                }
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
