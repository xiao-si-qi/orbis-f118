/*
 * orbis-f118 · 360×360 圆屏（ESP32-S31 / JC3636F118）
 *
 * 硬件初始化沿用原来的写法（背光 / LCD / LVGL / 触摸 / 编码器 / 触觉 / 电池 / 网络）。
 * 界面改用从 orbis-panel 移植过来的 ui_home：
 *   - JSON 布局页（HA 下发）+ 固件内置 System 页
 *   - 灯详情（开关/亮度/色温）、传感器折线、空调页都在 ui_home.c 里
 *
 * main.c 只负责：起硬件 → 起 UI → 恢复 NVS 里的布局 → 主循环里跑 LVGL、
 * 应用 MQTT 下发的 cfg/cmd、周期上报 state。
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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
#include "esp_memory_utils.h"   /* esp_ptr_dma_capable */
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "lvgl.h"

#include "bsp_shim.h"       /* bsp_display_set_backlight()（旧工程 bsp_display_* 的替代） */
#include "assistant_ui.h"  /* 小智助手页（MQTT assistant 命令） */
#include "net.h"
#include "battery.h"
#include "bt_link.h"       /* 蓝牙链路抽象层（本版是桩：协议栈未编译，恒「未启用」） */
#include "web.h"           /* 网页配置后台（配网 / 网页 OTA），WiFi 连上后起 */
#include "app_settings.h"  /* NVS 配置：WiFi / HA / MQTT / 设置页口令 */
#include "ha_client.h"
#include "ha_ws.h"
#include "layout.h"
#include "bg_image.h"
#include "ui_home.h"
#include "log_buffer.h"
#include "ota.h"
#include "voice_audio.h"      /* 音频自检（MQTT 的 audio_test 命令 / 小智页按钮） */
#include "xiaozhi_client.h"   /* 小智 AI（布局里的 voice 段驱动） */
#include "cJSON.h"

static const char *TAG = "orbis-f118";

/* 官方小智服务端（OTA 发现地址）：voice_test 默认用它试整条语音链路 */
#define XZ_OFFICIAL_ENDPOINT "https://api.tenclass.net/xiaozhi/ota/"

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

/* ---- 显示刷新：同步等待完成 ----
   曾经试过异步（完成回调里直接调 lv_display_flush_ready）：花屏，
   因为 LVGL 不是 ISR 安全的，且上一块没传完就提交下一块会错位。
   现在回到同步：提交后等完成标志（1 ms 粒度），另有 200 ms 兜底。 */
static volatile bool s_flush_done;
static volatile uint32_t s_flush_count;      /* 每秒 flush 块数，用来算 FPS */

static bool on_color_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *ed, void *ctx)
{
    (void)io; (void)ed; (void)ctx;
    s_flush_done = true;
    return false;
}

static void lv_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    s_flush_done = false;
    esp_lcd_panel_draw_bitmap(s_panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);


    /* 不要用 vTaskDelay(1)：整屏 23 块 × 1 ms ≈ 23 ms，比传输本身还慢。
       一块 16 行在 80 MHz 四线约 0.29 ms，用 20 µs 粒度自旋即可。 */
    const int64_t deadline = esp_timer_get_time() + 200 * 1000;
    while (!s_flush_done && esp_timer_get_time() < deadline) {
        esp_rom_delay_us(20);
    }
    s_flush_done = false;
    s_flush_count++;
    lv_display_flush_ready(disp);
}

static void flush_watchdog(void) { }   /* 同步模式下不需要 */
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
        .max_transfer_sz = LCD_W * 32 * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_CS, .dc_gpio_num = -1, .spi_mode = 0,
        .pclk_hz = 80 * 1000 * 1000,   /* 厂家 BSP 值：四线 40 MB/s；花屏真因是无显示对象，与时钟无关 */ .trans_queue_depth = 10,
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
    /* LVGL 9.6 的堆由 main/lv_mem_psram.c 从 PSRAM 分配（CONFIG_LV_USE_CUSTOM_MALLOC），
       内部 RAM 里不再保留 64 KB 的静态池。 */
    ESP_LOGI(TAG, "LVGL 内存：PSRAM 池（自定义分配器）");
    lv_tick_set_cb(tick_cb);

    /* 必须先建「显示」：lv_screen_active() 取的是默认显示的活动屏幕，
       没有 display 时它就是 NULL，之后每一次 lv_obj_create(scr) 都会在
       "父对象为 NULL → 找不到默认显示" 这一步失败并返回 NULL。
       这时 LVGL 池其实几乎全空，看起来却像"内存不足"，很容易误判。 */
    s_disp = lv_display_create(LCD_W, LCD_H);
    if (s_disp == NULL) {
        ESP_LOGE(TAG, "LVGL display 创建失败（内存不足），界面无法显示");
        return;
    }

    /* 绘制缓冲放在【内部 RAM + DMA 可用 + 缓存对齐】：
       SPI 驱动只有当缓冲不能直接 DMA 时才申请中转缓冲并 memcpy（每块 11.5 KB 纯浪费）。
       放内部 RAM 后 need_malloc=false → 零拷贝，整屏刷新时间从 ~29 ms 降到 ~7 ms。
       16 行/块：两块共 23 KB 内部 RAM（内存紧张，优先保 RAM）。 */
    const size_t buf_bytes = LCD_W * 16 * sizeof(lv_color_t);
    const size_t align = 64;   /* SPI DMA 缓冲的缓存行对齐（本芯片 64 字节） */
    void *b1 = heap_caps_aligned_alloc(align, buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    void *b2 = heap_caps_aligned_alloc(align, buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (b1 == NULL || b2 == NULL) {
        ESP_LOGE(TAG, "绘制缓冲分配失败（内部 RAM 不足），回退到 PSRAM（会有中转拷贝）");
        if (b1) heap_caps_free(b1);
        if (b2) heap_caps_free(b2);
        b1 = heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM);
        b2 = heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM);
    }
    lv_display_set_flush_cb(s_disp, lv_flush_cb);
    /* 面板要 RGB565 大端，LVGL 默认小端：显式声明字节交换 */
    lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
    lv_display_set_buffers(s_disp, b1, b2, buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    ESP_LOGI(TAG, "LVGL 初始化完成：绘制缓冲 %u KB x2（%s，DMA 直发零拷贝）",
             (unsigned)(buf_bytes / 1024),
             esp_ptr_dma_capable(b1) ? "内部 RAM" : "PSRAM 回退");
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
    (void)indev;
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
    (void)arg;
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

/* ---------------- 电池 ADC（GPIO55，15k/10k 分压）---------------- */
static volatile int s_bat_raw = -1;      /* 电池 ADC 原始值（未标定） */
static volatile int s_bat_raw_max;

static void battery_task(void *arg)
{
    (void)arg;
    adc_unit_t unit; adc_channel_t chan;
    if (adc_oneshot_io_to_channel(55, &unit, &chan) != ESP_OK) {
        ESP_LOGW(TAG, "GPIO55 不是 ADC 引脚");
        vTaskDelete(NULL); return;
    }
    adc_oneshot_unit_init_cfg_t uic = { .unit_id = unit };
    adc_oneshot_unit_handle_t au = NULL;
    if (adc_oneshot_new_unit(&uic, &au) != ESP_OK) { vTaskDelete(NULL); return; }

    /* ESP32-S31 的 ADC 与常见 S3 不同：
       - SOC_ADC_ATTEN_NUM = 1，只有一档衰减，设置其它档位会返回 invalid attenuation
       - 位宽最高 17 位
       - 没有可用的校准方案（ADC_CALI_SCHEME_*_SUPPORTED 未定义）
       因此没有可靠的 raw → mV 公式，先把原始值打出来，等实测标定后再换算。 */
    adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_0, .bitwidth = ADC_BITWIDTH_DEFAULT };
    esp_err_t e = adc_oneshot_config_channel(au, chan, &cc);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "电池 ADC 配置失败: %s", esp_err_to_name(e));
        vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "电池 ADC 就绪 (unit=%d chan=%d, 单档衰减, 无校准方案，仅上报原始值)",
             (int)unit, (int)chan);

    while (1) {
        int raw = 0, sum = 0, n = 0, raw_min = 999999, raw_max = 0;
        for (int i = 0; i < 8; i++) {
            if (adc_oneshot_read(au, chan, &raw) == ESP_OK) {
                sum += raw; n++;
                if (raw < raw_min) raw_min = raw;
                if (raw > raw_max) raw_max = raw;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (n > 0) {
            s_bat_raw = sum / n;
            s_bat_raw_max = raw_max;
            static int tick;
            if (++tick % 15 == 0) {
                ESP_LOGI(TAG, "电池原始值: avg=%d min=%d max=%d（分压 15K/10K）",
                         s_bat_raw, raw_min, raw_max);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

int battery_raw_value(void) { return s_bat_raw; }
int battery_raw_peak(void) { return s_bat_raw_max; }

/* ---------------- 界面：ui_home（从 orbis-panel 移植） ---------------- */

static lv_group_t *s_group;          /* 编码器用的按键组（ui_home 未使用，留空表） */
static volatile bool s_log_on;       /* 串口日志是否同时推 MQTT */
static volatile bool s_audio_test_busy;  /* 音频自检任务是否在跑 */
static int64_t s_last_diag_us;       /* 错误摘要限流（60s） */

static void encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    /* 小智助手页现在只是主界面的最后一页（不再是悬浮二级页）：
       旋钮在它上面照常翻页，所以这里不再做避让。 */
    const int8_t diff = encoder_read_diff();
    /* 方向只在这里定义一次：顺时针 = +1（调大 / 下一页），逆时针 = -1。
       硬件实测顺时针时 diff < 0；若换成反相编码器，只改这一行的 ? 分支即可。 */
    if (diff != 0) {
        const int dir = (diff < 0) ? 1 : -1;
        const bool adjusting = ui_home_detail_knob(dir);
        if (ui_home_detail_open()) {
            ESP_LOGD(TAG, "旋钮: diff=%d 顺时针方向=%d 调值=%d", diff, dir, (int)adjusting);
        }
        if (adjusting) {
            haptic_click();
        } else if (!ui_home_detail_open()) {
            ui_home_page_step(dir);
            haptic_click();
        }
    }
    data->enc_diff = 0;              /* 不交给 LVGL 组导航（ui_home 没有键盘组） */
    data->state = LV_INDEV_STATE_RELEASED;
}

static void haptic_click_cb(lv_event_t *e)
{
    (void)e;
    /* 旧界面在卡片获得焦点时震动；新界面没有按键组，改成"点一下就轻震" */
    haptic_click();
}

static void register_layout_entities(const layout_t *layout)
{
    for (int p = 0; p < layout->page_count; p++) {
        for (int t = 0; t < layout->pages[p].tile_count; t++) {
            const layout_tile_t *tile = &layout->pages[p].tiles[t];
            ha_client_register(tile->entity, tile->label[0] ? tile->label : NULL);
        }
    }
    ESP_LOGI(TAG, "从布局注册了 %d 个实体", ha_client_count());
}

/** MQTT 下发的 cfg：解析布局 → 存 NVS → 注册实体 → 交给 ui_home 重建界面。 */
static void apply_cfg(const char *json)
{
    /* retained cfg 每次重连都会重发：内容没变就别重建界面 */
    static char s_last_cfg[4096];
    if (s_last_cfg[0] != '\0' && strcmp(s_last_cfg, json) == 0) {
        ESP_LOGI(TAG, "cfg 未变化，跳过重建");
        return;
    }
    snprintf(s_last_cfg, sizeof(s_last_cfg), "%s", json);

    bg_image_apply_json(json);       /* 背景图/轮播 */

    /* layout_t 约 11 KB，放堆上：主任务栈不大，ui_home_apply() 内部会拷一份 */
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

    layout_save_nvs(json);
    register_layout_entities(lay);

    /* 小智语音：开关和服务端地址都来自布局的 voice 段（switchyard 下发），
       这是唯一正式来源；MQTT 的 {"cmd":"voice"} 只是调试覆盖。 */
    xiaozhi_client_configure(lay->voice_enabled, lay->voice_endpoint);
    ESP_LOGI(TAG, "小智语音：%s%s%s", lay->voice_enabled ? "启用" : "未启用",
             lay->voice_enabled ? "，endpoint=" : "",
             lay->voice_enabled ? lay->voice_endpoint : "");

    const int pages = lay->page_count;
    ui_home_apply(lay);
    free(lay);
    ESP_LOGI(TAG, "cfg 已应用：共 %d 页", pages);
}

/** MQTT 的 {"cmd":"audio_test"}：录 1.5s → 日志里出峰值/RMS → 回放。
    必须放独立任务：整段要 3 秒多，绝不能在 LVGL 主循环里同步跑。 */
static void audio_test_task(void *arg)
{
    (void)arg;
    s_audio_test_busy = true;
    int peak = 0, rms = 0;
    const int rc = voice_audio_selftest(&peak, &rms);
    if (rc == 0) {
        ESP_LOGI(TAG, "音频自检完成：峰值 %d，均方 %d（已回放，线路输出插耳机可听）", peak, rms);
    } else {
        ESP_LOGE(TAG, "音频自检失败（录音或播放通路有问题，看上面日志）");
    }
    s_audio_test_busy = false;
    vTaskDelete(NULL);
}

static void apply_cmd(const char *json)
{
    cJSON *root = cJSON_Parse(json);
    if (root == NULL) { ESP_LOGW(TAG, "cmd 解析失败"); return; }

    const cJSON *jcmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    const char *cmd = cJSON_IsString(jcmd) ? jcmd->valuestring : NULL;
    if (cmd == NULL) { cJSON_Delete(root); return; }
    ESP_LOGI(TAG, "执行 cmd: %s", cmd);

    if (strcmp(cmd, "reboot") == 0) {
        cJSON_Delete(root);
        ESP_LOGW(TAG, "收到重启命令");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();

    } else if (strcmp(cmd, "ota") == 0) {
        const cJSON *ju = cJSON_GetObjectItemCaseSensitive(root, "url");
        const cJSON *jv = cJSON_GetObjectItemCaseSensitive(root, "version");
        if (cJSON_IsString(ju)) {
            ota_start_url(ju->valuestring, cJSON_IsString(jv) ? jv->valuestring : NULL);
        } else {
            ota_start_check();      /* 没带 url：走 switchyard 的 firmware.json 检查更新 */
        }

    } else if (strcmp(cmd, "state") == 0) {
        net_mqtt_publish_state();

    } else if (strcmp(cmd, "audio_test") == 0) {
        /* 调试：{cmd:"audio_test"} 录 1.5 秒 → 打印峰值/均方 → 回放。独立任务，别卡 LVGL。 */
        if (s_audio_test_busy) {
            ESP_LOGW(TAG, "音频自检已在跑，忽略本次命令");
        } else if (xTaskCreate(audio_test_task, "audio_test", 6144, NULL, 4, NULL) != pdPASS) {
            ESP_LOGE(TAG, "音频自检任务创建失败");
        } else {
            ESP_LOGI(TAG, "音频自检已启动（录 1500ms → 峰值/RMS → 回放）");
        }

    } else if (strcmp(cmd, "voice_test") == 0) {
        /* 一条命令验证整条语音链路（人不在设备边，全靠 MQTT 日志看结果）：
           开日志上报 → 启用官方服务端 → 等 hello → 自动 listen start → 说话 → 听回复。
           可选 {"cmd":"voice_test","endpoint":"https://..."} 换自建服务端。 */
        const cJSON *jep = cJSON_GetObjectItemCaseSensitive(root, "endpoint");
        const char *ep = (cJSON_IsString(jep) && jep->valuestring[0] != '\0')
                             ? jep->valuestring : XZ_OFFICIAL_ENDPOINT;
        s_log_on = true;                    /* 后面所有日志同步发到 MQTT log 主题 */
        ESP_LOGI(TAG, "voice_test 1/4：日志上报已开启，启用小智语音 endpoint=%s", ep);
        xiaozhi_client_configure(true, ep);
        ESP_LOGI(TAG, "voice_test 2/4：等 OTA 发现 → WebSocket hello（约 3-10 秒；失败会打 发现失败/连接出错）");
        ESP_LOGI(TAG, "voice_test 3/4：收到 hello 后自动发 listen start，请对着麦克风说话 3-5 秒");
        ESP_LOGI(TAG, "voice_test 4/4：状态文字 已连接→正在聆听→正在说话→播放回复；每秒一条 上行/下行 摘要");

    } else if (strcmp(cmd, "voice") == 0) {
        /* 调试覆盖：{cmd:"voice",enabled:true,endpoint:"https://..."}
           正常配置走布局的 voice 段（apply_cfg），这里只用于临时验证服务端地址。 */
        const cJSON *jen = cJSON_GetObjectItemCaseSensitive(root, "enabled");
        const cJSON *jep = cJSON_GetObjectItemCaseSensitive(root, "endpoint");
        const bool en = cJSON_IsBool(jen) ? cJSON_IsTrue(jen)
                                         : (cJSON_IsNumber(jen) ? (jen->valueint != 0) : true);
        const char *ep = cJSON_IsString(jep) ? jep->valuestring : NULL;
        xiaozhi_client_configure(en, ep);
        ESP_LOGI(TAG, "小智（调试覆盖）：enabled=%d endpoint=%s", (int)en, ep ? ep : "-");

    } else if (strcmp(cmd, "ha_ws") == 0) {
        /* 调试：{cmd:"ha_ws"} 打一行 WebSocket 状态（事件/命令/往返/最后错误/内部 RAM），
           同时发到 MQTT log 主题，人不在设备边也能看。 */
        char buf[256];
        ha_ws_status(buf, sizeof(buf));
        ESP_LOGI(TAG, "HA WebSocket：%s", buf);
        net_publish_log(buf);

    } else if (strcmp(cmd, "log") == 0) {
        /* 按需抓日志：先把环形缓冲里的最近 40 行发一次，之后按 s_log_on 继续推 */
        char *dump = malloc(4096);
        if (dump != NULL) {
            const size_t n = log_buffer_dump(dump, 4096, 40);
            if (n > 0) {
                net_publish_log(dump);
                ESP_LOGI(TAG, "已发送 %u 字节日志", (unsigned)n);
            }
            free(dump);
        }
        s_log_on = true;
        ESP_LOGI(TAG, "日志上报已开启");

    } else if (strcmp(cmd, "set") == 0) {
        const cJSON *jk = cJSON_GetObjectItemCaseSensitive(root, "key");
        const cJSON *jv = cJSON_GetObjectItemCaseSensitive(root, "value");
        if (cJSON_IsString(jk) && strcmp(jk->valuestring, "brightness") == 0 && cJSON_IsNumber(jv)) {
            const int pct = (int)jv->valuedouble;
            bsp_display_set_backlight((uint8_t)pct);
            ESP_LOGI(TAG, "屏幕亮度 -> %d%%", pct);
        }

    } else if (strcmp(cmd, "detail") == 0) {
        const cJSON *je = cJSON_GetObjectItemCaseSensitive(root, "entity");
        if (cJSON_IsString(je)) {
            ui_home_open_detail(je->valuestring);
        }

    } else if (strcmp(cmd, "assistant") == 0) {
        /* 小智助手页开关（它现在是主界面最后一页，这里只是翻页）：
           {cmd:"assistant"} / open=true 翻到最后一页，
           {"cmd":"assistant","open":false} 翻回第一页（如果还在聆听会顺手关麦）。
           不碰音频通路；语音会话仍由 layout 的 voice 段 / voice 命令控制。 */
        const cJSON *jopen = cJSON_GetObjectItemCaseSensitive(root, "open");
        const bool want_open = cJSON_IsBool(jopen) ? cJSON_IsTrue(jopen) : true;
        if (want_open) {
            assistant_ui_open();
        } else {
            assistant_ui_close();
        }
        ESP_LOGI(TAG, "小智助手页：%s", want_open ? "翻到最后一页" : "翻回第一页");

    } else if (strcmp(cmd, "hist") == 0) {
        /* 调试命令：同步拉 6 小时历史后直接显示折线（会短暂阻塞主循环，正常交互走点卡片） */
        const cJSON *je = cJSON_GetObjectItemCaseSensitive(root, "entity");
        if (cJSON_IsString(je)) {
            static float vals[64];
            int cnt = 0;
            float vmin = 0, vmax = 0, vavg = 0;
            const esp_err_t he = ha_client_history(je->valuestring, 6, vals, 64, &cnt, &vmin, &vmax, &vavg);
            ESP_LOGI(TAG, "hist %s -> %s, %d 点 min=%.2f max=%.2f avg=%.2f",
                     je->valuestring, esp_err_to_name(he), cnt, vmin, vmax, vavg);
            if (he == ESP_OK && cnt > 0) {
                ui_home_show_chart(je->valuestring, vals, cnt, vmin, vmax, vavg);
            }
        }

    } else if (strcmp(cmd, "get") == 0) {
        /* TODO(port): 旧工程用 mqtt_link 把 NVS 里的布局原样发回 cfg 主题；
           net.c 目前只提供 state 发布，没有"发到 cfg 主题"的接口，这里先不支持。 */
        ESP_LOGW(TAG, "cmd get：net.c 暂不支持回读布局（TODO(port)）");

    } else {
        ESP_LOGW(TAG, "未知 cmd: %s", cmd);
    }
    cJSON_Delete(root);
}

static int log_forward(const char *fmt, va_list args)
{
    char buf[160];
    const int n = vsnprintf(buf, sizeof(buf), fmt, args);
    if (n > 0) {
        fputs(buf, stdout);                 /* 仍然输出到串口 */
        fflush(stdout);
        if (s_log_on) net_publish_log(buf); /* 需要时才上报 */
    }
    return n;
}

void app_main(void)
{
    esp_log_set_vprintf(log_forward);
    log_buffer_init();                      /* 环形缓冲挂在 log_forward 前面，用于按需回传 */

    ESP_LOGI(TAG, "=========== orbis-f118 启动 ===========");
    ESP_LOGI(TAG, "PSRAM %u KB / 内部 RAM %u KB",
             (unsigned)(heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    /* 最先读 NVS 配置：后面 net_wifi_start / net_mqtt_start / 网页都要用 */
    app_settings_init();
    /* 蓝牙链路抽象层：本版是桩（协议栈未编译），只读 NVS 里的音频输出偏好、建互斥量。
       放在 NVS 初始化之后、任何界面构建之前：设置页第一次画出来就能拿到正确的偏好，
       而且不启动任何蓝牙栈、不改开机顺序。 */
    bt_link_init();
    backlight_init();
    lcd_init();
    lvgl_init();
    touch_init();
    encoder_init();
    haptic_init();

    lv_indev_t *enc = lv_indev_create();
    lv_indev_set_type(enc, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(enc, encoder_read_cb);
    s_group = lv_group_create();
    lv_indev_set_group(enc, s_group);

    xTaskCreate(battery_task, "bat", 4096, NULL, 3, NULL);

    bg_image_init();                        /* 背景图/卡片蒙版默认值要在首屏之前就位 */
    net_wifi_start();
    net_mqtt_start();

    /* 启动页：先出 "Orbis / connecting..."，再等网络 */
    ui_home_boot();
    lv_obj_add_event_cb(lv_screen_active(), haptic_click_cb, LV_EVENT_CLICKED, NULL);
    ESP_LOGI(TAG, "启动页已显示，等待 WiFi");

    /* 默认背景（HA 的 cfg 里带 bg 时会覆盖它） */
    {
        static const char *bg_json =
            "{\"bg\":{\"images\":["
            "\"https://ha.xiaosiqi.cn/local/switchyard/orbis/bg/1.jpg\","
            "\"https://ha.xiaosiqi.cn/local/switchyard/orbis/bg/2.jpg\","
            "\"https://ha.xiaosiqi.cn/local/switchyard/orbis/bg/3.jpg\"],"
            "\"interval\":20,\"dim\":65,\"card\":60}}";
        bg_image_apply_json(bg_json);
    }

    /* 等 WiFi 连上（期间把启动页刷出来，别黑屏干等） */
    for (int i = 0; i < 60 && !net_wifi_connected(); i++) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* 配置页（80 端口）：一次启动后一直在跑，断线重连不需要重启它 */
    web_start();
    if (net_wifi_connected()) {
        ESP_LOGI(TAG, "配置页: http://%s/ （手机浏览器打开，可改 WiFi / HA / 网页升级）",
                 net_ip_string());
    } else {
        ESP_LOGW(TAG, "WiFi 未连接，配置页等有 IP 后再访问（主循环会持续重试）");
    }

    /* 上一次 MQTT 下发的布局（存在 NVS 里）：开机先按它建界面 */
    layout_t *lay = calloc(1, sizeof(layout_t));
    const bool has_layout = (lay != NULL) &&
                            (layout_load_nvs(lay) == ESP_OK) && lay->page_count > 0;
    if (has_layout) {
        register_layout_entities(lay);
        /* 开机就按上次下发的布局恢复小智语音（没启用则什么都不做） */
        xiaozhi_client_configure(lay->voice_enabled, lay->voice_endpoint);
        ESP_LOGI(TAG, "已从 NVS 恢复布局：%d 页（语音 %s）", lay->page_count,
                 lay->voice_enabled ? "启用" : "未启用");
    } else {
        ESP_LOGW(TAG, "还没有布局，等 switchyard/%s/cfg", net_device_id());
    }

    if (net_wifi_connected()) {
        if (ha_client_init() == ESP_OK) {
            ha_client_fetch_all();
            ha_client_start_poll();
            ESP_LOGI(TAG, "HA 已启动，注册 %d 个实体", ha_client_count());
        } else {
            ESP_LOGE(TAG, "HA client 初始化失败");
        }
    } else {
        ESP_LOGW(TAG, "WiFi 未连接，跳过 HA 初始化");
    }

    /* WebSocket 长连接：下行 state_changed（卡片即时刷新），上行 service call（快路径）。
       地址/协议由 app_settings 里的 HA URL 推导，WiFi 晚上线也没关系（任务内部等网络）。
       放在 HA REST 初始化之后：开机先出首屏状态，再由 WS 接管实时更新。 */
    /* 隔离测试：临时不启动 WS，确认开机崩溃是否由它引起 */
    if (0 && !ha_ws_start()) {
        ESP_LOGW(TAG, "HA WebSocket 任务启动失败，继续只用 REST 轮询");
    }

    ui_home_apply(has_layout ? lay : NULL);   /* 没有布局就只显示 System 页 */
    free(lay);
    ESP_LOGI(TAG, "进入 LVGL 主循环");

    uint32_t last_beat = 0;
    while (1) {
        flush_watchdog();
        uint32_t d = lv_timer_handler();
        net_mqtt_tick();          /* 连上后立刻发布 meta 并订阅，不等心跳 */

        /* 配置页：断线重连 / 启动时失败的话在这里补起（已在跑就是空操作） */
        if (!web_running() && net_wifi_connected()) {
            web_start();
        }

        /* cfg / cmd 每轮都处理，避免排队时被后来的覆盖 */
        {
            static char cfg_json[4096];
            static char cmd_json[1024];
            if (net_take_cfg(cfg_json, sizeof(cfg_json))) apply_cfg(cfg_json);
            if (net_take_cmd(cmd_json, sizeof(cmd_json))) apply_cmd(cmd_json);
        }

        /* 错误摘要：有就发，但 1 分钟内只发一条
           TODO(port): 旧工程发到 /diag（retained），net.c 只有 /log 主题，先混用 */
        if (esp_timer_get_time() - s_last_diag_us >= 60LL * 1000 * 1000) {
            char diag[512];
            if (log_buffer_take_diag(diag, sizeof(diag))) {
                net_publish_log(diag);
                s_last_diag_us = esp_timer_get_time();
            }
        }

        if (esp_timer_get_time() / 1000000ULL != last_beat) {
            last_beat = (uint32_t)(esp_timer_get_time() / 1000000ULL);
            if (last_beat % 5 == 0) {
                net_mqtt_publish_state();
                ESP_LOGI(TAG, "电池原始值: %d | 网络: wifi=%d rssi=%d mqtt=%d", s_bat_raw, net_wifi_connected(), net_wifi_rssi(), net_mqtt_connected());
                {
                    lv_mem_monitor_t mon;
                    lv_mem_monitor(&mon);
                    /* 自定义分配器没有固定池：free/total 反映 PSRAM 堆，已用/峰值才是 LVGL 自己。 */
                    ESP_LOGI(TAG, "LVGL 池(PSRAM): 空闲 %u/%u 字节, 已用 %u, 峰值 %u, 最大块 %u, 碎片 %u%%",
                             (unsigned)mon.free_size, (unsigned)mon.total_size,
                             (unsigned)(mon.total_size - mon.free_size), (unsigned)mon.max_used,
                             (unsigned)mon.free_biggest_size, (unsigned)mon.frag_pct);
                }
                {
                    static uint32_t last_cnt, last_us;
                    const uint32_t cnt = s_flush_count;
                    const uint32_t now_us = (uint32_t)esp_timer_get_time();
                    const uint32_t flush_blocks = cnt - last_cnt;
                    const uint32_t dt_us = (last_us ? (now_us - last_us) : 5000000U);
                    last_cnt = cnt; last_us = now_us;
                    /* 每块 16 行 → 整屏 23 块；块/秒 ÷ 23 = 整屏帧率 */
                    const uint32_t blocks_per_s = dt_us ? (uint32_t)((uint64_t)flush_blocks * 1000000ULL / dt_us) : 0;
                    ESP_LOGI(TAG, "刷新: %u 块/秒 ≈ %u fps（16 行/块，80 MHz）",
                             (unsigned)blocks_per_s, (unsigned)(blocks_per_s * 10 / 23 / 10));
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
