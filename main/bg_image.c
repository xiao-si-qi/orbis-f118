/**
 * @file bg_image.c
 * @brief 背景图轮播实现：HTTP 下载 JPEG → 硬件 JPEG 解码成 RGB565 → LVGL 贴图。
 *
 * 解码策略（一张图只打一行日志说明走的哪条路）：
 *   - 首选芯片内置的硬件解码器（esp_driver_jpeg）：解到 PSRAM 的 RGB565 缓冲，
 *     引擎懒创建、跨图片复用；
 *   - 引擎创建失败 / 这张图解不出来 / 太大超出 PSRAM 预算，回退 esp_jpeg（tjpgd
 *     软件解码），背景图不会因为解码器的问题消失；
 *   - 硬件解码不做缩放，非 360×360 的图仍交给 LVGL 缩放（和软件解码时一模一样）。
 *
 * 线程约定：
 *   - 下载/解码在独立任务里做（会阻塞几百毫秒）；
 *   - 碰 LVGL 对象的动作通过 lv_async_call 切回 LVGL 线程；
 *   - 硬件解码引擎由 bg_task 独占（创建/使用/释放都在这个任务里），
 *     bg_image_stop() 只置停止标志，真正释放由任务退出时完成，避免 use-after-free。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/jpeg_decode.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "jpeg_decoder.h"

#include "app_settings.h"
#include "bg_cache.h"
#include "bg_image.h"

static const char *TAG = "bg";

#define BG_MAX_IMAGES   8
#define BG_URL_MAX      256
#define BG_W            360
#define BG_H            360
#define BG_JPEG_MAX     (512 * 1024)

/* 硬件解码单张图的超时（只防死等；360×360 实测几毫秒）。 */
#define BG_HW_TIMEOUT_MS   500
/* 硬件解码不做缩小：源图多大就解多大。超过这个预算就交给软件解码
   （tjpgd 会做 1/2、1/4 整数缩小），避免一张 1080p 壁纸吃掉 2 MB 以上 PSRAM
   ——LVGL 的堆也在 PSRAM 里。1.5 MB 够 886×886，远大于面板需要的 360×360。 */
#define BG_HW_MAX_BYTES    (1536 * 1024)
/* 2D-DMA 访问的缓冲必须 16 字节对齐。 */
#define BG_DMA_ALIGN       16

typedef struct {
    char urls[BG_MAX_IMAGES][BG_URL_MAX];
    int count;
    int interval_s;
    int dim_pct;
    int card_pct;      /* 卡片黑色蒙版（0-95） */
} bg_cfg_t;

static bg_cfg_t s_cfg;
static volatile bool s_running = false;
static volatile int s_gen;          /* 配置代际：旧任务看到代际变了就退出 */
static volatile bool s_task_alive;
static volatile bool s_paused;
static volatile int s_index = 0;

static bool s_initialized = false;
static uint8_t *s_frame = NULL;          /* RGB565 输出缓冲（PSRAM，按硬件解码要求对齐，可复用） */
static size_t s_frame_len = 0;           /* 实际容量（按 cache line 对齐，>= 申请的字节数） */
static lv_image_dsc_t s_dsc;
static lv_obj_t *s_img = NULL;
static lv_obj_t *s_dim = NULL;
static volatile bool s_frame_ready = false;
static int s_shown_w = 0;
static int s_shown_h = 0;

/* 硬件解码引擎：懒创建、跨图片复用，只由 bg_task 使用/释放。 */
static jpeg_decoder_handle_t s_hw_dec = NULL;
static bool s_hw_broken = false;         /* 引擎创建失败过：本次配置不再尝试 */
static bool s_bench_done = false;        /* 首帧的软件解码基准只测一次 */
static int64_t s_bench_us = 0;           /* 首帧实测的软件解码耗时（微秒） */
static int s_bench_w = 0;
static int s_bench_h = 0;

/* ---------------------------- 图像下载/解码 ---------------------------- */

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t len;
    bool overflow;
} fetch_ctx_t;

static esp_err_t fetch_event(esp_http_client_event_t *ev)
{
    fetch_ctx_t *ctx = (fetch_ctx_t *)ev->user_data;
    if (ev->event_id == HTTP_EVENT_ON_DATA && ctx != NULL) {
        const size_t n = (size_t)ev->data_len;
        if (ctx->len + n > ctx->cap) {
            ctx->overflow = true;
            return ESP_OK;
        }
        memcpy(ctx->buf + ctx->len, ev->data, n);
        ctx->len += n;
    }
    return ESP_OK;
}

/** 下载到 PSRAM 缓冲（走 perform，TLS/重定向都交给 esp_http_client 处理）。
    缓冲按 16 字节对齐分配：硬件解码直接拿它喂 2D-DMA。 */
static uint8_t *http_fetch(const char *url, size_t *out_len, int *out_status)
{
    if (out_status) {
        *out_status = 0;
    }
    uint8_t *buf = heap_caps_aligned_alloc(BG_DMA_ALIGN, BG_JPEG_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        ESP_LOGE(TAG, "no PSRAM for jpeg buffer");
        return NULL;
    }

    fetch_ctx_t ctx = {.buf = buf, .cap = BG_JPEG_MAX, .len = 0, .overflow = false};
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 15000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 4096,
        .event_handler = fetch_event,
        .user_data = &ctx,
        .keep_alive_enable = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        free(buf);
        return NULL;
    }

    const esp_err_t err = esp_http_client_perform(client);
    const int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (out_status) {
        *out_status = status;
    }
    if (err != ESP_OK || status != 200 || ctx.len == 0 || ctx.overflow) {
        ESP_LOGW(TAG, "%s -> %s, HTTP %d, %d bytes%s", url, esp_err_to_name(err), status,
                 (int)ctx.len, ctx.overflow ? " (too large)" : "");
        free(buf);
        return NULL;
    }
    *out_len = ctx.len;
    return buf;
}

/** 当前可用内存（KB），只用于日志。 */
static int mem_kb(int caps)
{
    return (int)(heap_caps_get_free_size((uint32_t)caps) / 1024);
}

/** 输出缓冲：硬件解码要求地址按 cache line 对齐、长度也是对齐的整数倍，
    jpeg_alloc_decoder_mem() 正好按这个规则分配（软件解码也能直接用）。 */
static bool frame_reserve(size_t need)
{
    if (s_frame != NULL && s_frame_len >= need) {
        return true;
    }
    jpeg_decode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    size_t got = 0;
    uint8_t *buf = jpeg_alloc_decoder_mem(need, &mem_cfg, &got);
    if (buf == NULL) {
        ESP_LOGE(TAG, "背景：PSRAM 不足，申请 %u KB 输出缓冲失败", (unsigned)(need / 1024));
        return false;
    }
    if (s_frame != NULL) {
        free(s_frame);
    }
    s_frame = buf;
    s_frame_len = got;
    return true;
}

/** 懒创建硬件解码引擎；创建失败只记一次，后面直接走软件解码。 */
static bool hw_engine_ensure(void)
{
    if (s_hw_dec != NULL) {
        return true;
    }
    if (s_hw_broken) {
        return false;
    }
    const jpeg_decode_engine_cfg_t cfg = {
        .timeout_ms = BG_HW_TIMEOUT_MS,
    };
    const esp_err_t err = jpeg_new_decoder_engine(&cfg, &s_hw_dec);
    if (err != ESP_OK) {
        s_hw_dec = NULL;
        s_hw_broken = true;
        ESP_LOGW(TAG, "背景：硬件 JPEG 解码器创建失败（%s），本次配置只用软件解码",
                 esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "背景：硬件 JPEG 解码引擎已创建（内部内存 %d KB，PSRAM %d KB）",
             mem_kb(MALLOC_CAP_INTERNAL), mem_kb(MALLOC_CAP_SPIRAM));
    return true;
}

/** 释放硬件解码引擎（只在没有解码在飞的时候调用：任务退出 / 兜底）。 */
static void hw_engine_release(void)
{
    if (s_hw_dec == NULL) {
        return;
    }
    const esp_err_t err = jpeg_del_decoder_engine(s_hw_dec);
    s_hw_dec = NULL;
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "背景：硬件解码引擎已释放（内部内存 %d KB，PSRAM %d KB）",
                 mem_kb(MALLOC_CAP_INTERNAL), mem_kb(MALLOC_CAP_SPIRAM));
    } else {
        ESP_LOGW(TAG, "背景：释放硬件解码引擎失败: %s", esp_err_to_name(err));
    }
}

/** 硬件解码的输出尺寸规划。 */
typedef struct {
    uint32_t w;        /* 可见宽（= JPEG 头里的宽） */
    uint32_t h;        /* 可见高 */
    uint32_t stride;   /* 每行字节数：MCU 补齐后的宽度 × 2 */
    size_t need;       /* 输出缓冲字节数 */
} hw_plan_t;

/** 读 JPEG 头，算解码输出尺寸。硬件解码不做缩放，但会把宽高补齐到 MCU
    （8 或 16 像素）边界，所以行跨距是补齐后的宽度，不是图里的宽度。 */
static bool hw_plan(const uint8_t *jpg, uint32_t len, hw_plan_t *out)
{
    jpeg_decode_picture_info_t info = {0};
    const esp_err_t err = jpeg_decoder_get_info(jpg, len, &info);
    if (err != ESP_OK || info.width == 0 || info.height == 0) {
        ESP_LOGW(TAG, "背景：硬件读不到 JPEG 头（%s）", esp_err_to_name(err));
        return false;
    }

    uint32_t mcux = 8;
    uint32_t mcuy = 8;
    switch (info.sample_method) {
    case JPEG_DOWN_SAMPLING_YUV422:
        mcux = 16;
        break;
    case JPEG_DOWN_SAMPLING_YUV420:
        mcux = 16;
        mcuy = 16;
        break;
    default:                        /* YUV444 / 灰度：8×8 */
        break;
    }
    const uint32_t pw = (info.width + mcux - 1) / mcux * mcux;
    const uint32_t ph = (info.height + mcuy - 1) / mcuy * mcuy;
    const size_t need = (size_t)pw * (size_t)ph * 2;
    if (need > BG_HW_MAX_BYTES) {
        ESP_LOGI(TAG, "背景：%ux%u 硬件解码要 %u KB，超出 PSRAM 预算，改走软件解码",
                 (unsigned)info.width, (unsigned)info.height, (unsigned)(need / 1024));
        return false;
    }

    out->w = info.width;
    out->h = info.height;
    out->stride = pw * 2;
    out->need = need;
    return true;
}

/** 硬件解码一张图到 s_frame；成功返回 true，耗时写进 us。 */
static bool hw_run(const uint8_t *jpg, uint32_t len, const hw_plan_t *plan, int64_t *us)
{
    /* RGB565 + BGR（小端）正好是 LVGL 要的 uint16 排列；
       YUV→RGB 用 BT.601（JPEG 的默认标准）。 */
    const jpeg_decode_cfg_t cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    uint32_t written = 0;
    const int64_t t0 = esp_timer_get_time();
    const esp_err_t err = jpeg_decoder_process(s_hw_dec, &cfg, jpg, len, s_frame,
                                               (uint32_t)s_frame_len, &written);
    *us = esp_timer_get_time() - t0;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "背景：硬件解码失败（%s）", esp_err_to_name(err));
        return false;
    }
    if (written != plan->need) {
        /* 补齐尺寸算错了 → 行跨距不可信，宁可回退软件解码也不显示错位的图 */
        ESP_LOGW(TAG, "背景：硬件解码输出 %u 字节（预期 %u），行跨距不可信，回退",
                 (unsigned)written, (unsigned)plan->need);
        return false;
    }
    return true;
}

/** 软件解码（esp_jpeg/tjpgd）到 s_frame；成功返回 true，耗时写进 us。
    大图让解码器做整数缩小，避免缓冲过大 / 内存碎片：
    目标是解码后宽度 >= 360（后面交给 LVGL 缩放到屏幕），但不超过 720。 */
static bool sw_decode(const uint8_t *jpg, uint32_t jpg_len, int64_t *us)
{
    esp_jpeg_image_cfg_t info_cfg = {
        .indata = (uint8_t *)jpg,
        .indata_size = jpg_len,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
    };
    esp_jpeg_image_output_t info = {0};
    if (esp_jpeg_get_image_info(&info_cfg, &info) != ESP_OK || info.width == 0 || info.height == 0) {
        ESP_LOGW(TAG, "背景：软件解码读不到 JPEG 头");
        return false;
    }

    esp_jpeg_image_scale_t scale = JPEG_IMAGE_SCALE_0;
    int divisor = 1;
    if (info.width > 720 && info.width / 2 >= 360) {
        scale = JPEG_IMAGE_SCALE_1_2;
        divisor = 2;
    }
    if (info.width / divisor > 1080 && info.width / 4 >= 360) {
        scale = JPEG_IMAGE_SCALE_1_4;
        divisor = 4;
    }

    const size_t need = (size_t)(info.width / divisor) * (size_t)(info.height / divisor) * 2;
    if (need == 0 || !frame_reserve(need)) {
        return false;
    }

    esp_jpeg_image_cfg_t cfg = {
        .indata = (uint8_t *)jpg,
        .indata_size = jpg_len,
        .outbuf = s_frame,
        .outbuf_size = (uint32_t)need,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = scale,
        .flags = {.swap_color_bytes = 0},   /* tjpgd 直接写 uint16_t，已是 LVGL 需要的小端 */
    };
    esp_jpeg_image_output_t out = {0};
    const int64_t t0 = esp_timer_get_time();
    const esp_err_t err = esp_jpeg_decode(&cfg, &out);
    *us = esp_timer_get_time() - t0;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "背景：软件解码失败: %s (%dx%d, scale 1/%d)", esp_err_to_name(err),
                 info.width, info.height, divisor);
        return false;
    }
    s_shown_w = out.width;
    s_shown_h = out.height;
    s_dsc.data = s_frame;
    s_dsc.data_size = out.output_len;
    s_dsc.header.stride = (uint32_t)out.width * 2;
    return true;
}

/** 解码到 s_frame：先试硬件解码，不行再回退软件解码；成功返回 true。 */
static bool decode_to_frame(const uint8_t *jpg, size_t jpg_len)
{
    const int int_before = mem_kb(MALLOC_CAP_INTERNAL);
    const int ps_before = mem_kb(MALLOC_CAP_SPIRAM);

    /* 2D-DMA 要 16 字节对齐的输入缓冲；缓存/下载缓冲不一定满足，必要时复制一份。 */
    uint8_t *aligned = NULL;
    const bool in_aligned = (((uintptr_t)jpg & (BG_DMA_ALIGN - 1)) == 0);
    const uint8_t *in = jpg;
    if (!in_aligned) {
        aligned = heap_caps_aligned_alloc(BG_DMA_ALIGN, jpg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (aligned != NULL) {
            memcpy(aligned, jpg, jpg_len);
            in = aligned;
        }
    }
    const bool dma_input_ok = in_aligned || (aligned != NULL);

    /* 先按硬件路径把输出缓冲准备好（比软件路径大一点，免得来回 realloc）。 */
    hw_plan_t plan;
    const bool hw_ready = dma_input_ok && hw_engine_ensure() &&
                          hw_plan(in, (uint32_t)jpg_len, &plan) && frame_reserve(plan.need);

    /* 首帧：先跑一遍软件解码，量一个真实的软件耗时作为基准（只做一次）。 */
    if (!s_bench_done) {
        int64_t bench_us = 0;
        s_bench_done = true;
        if (sw_decode(in, (uint32_t)jpg_len, &bench_us)) {
            s_bench_us = bench_us;
            s_bench_w = s_shown_w;
            s_bench_h = s_shown_h;
        }
    }

    int64_t used_us = 0;
    bool hw_used = false;
    if (hw_ready && hw_run(in, (uint32_t)jpg_len, &plan, &used_us)) {
        s_shown_w = (int)plan.w;
        s_shown_h = (int)plan.h;
        s_dsc.data = s_frame;
        s_dsc.data_size = (uint32_t)plan.need;
        s_dsc.header.stride = plan.stride;
        hw_used = true;
    } else if (!sw_decode(in, (uint32_t)jpg_len, &used_us)) {
        ESP_LOGW(TAG, "背景：这张图解不出来（%d 字节），跳过", (int)jpg_len);
        free(aligned);
        return false;
    }
    s_frame_ready = true;

    const int ms = (int)((used_us + 500) / 1000);
    if (hw_used) {
        const int bench_ms = (int)((s_bench_us + 500) / 1000);
        if (s_bench_us > 0 && s_bench_w == s_shown_w && s_bench_h == s_shown_h) {
            const int64_t ratio10 = (used_us > 0) ? (s_bench_us * 10 + used_us / 2) / used_us : 0;
            ESP_LOGI(TAG, "背景：硬件解码 %dx%d 用时 %d ms（软件解码实测 %d ms，提速 %d.%d 倍）",
                     s_shown_w, s_shown_h, ms, bench_ms, (int)(ratio10 / 10), (int)(ratio10 % 10));
        } else if (s_bench_us > 0) {
            ESP_LOGI(TAG, "背景：硬件解码 %dx%d 用时 %d ms（软件解码 %dx%d 实测 %d ms，尺寸不同仅供参考）",
                     s_shown_w, s_shown_h, ms, s_bench_w, s_bench_h, bench_ms);
        } else {
            ESP_LOGI(TAG, "背景：硬件解码 %dx%d 用时 %d ms（软件基准不可用）", s_shown_w, s_shown_h, ms);
        }
    } else {
        ESP_LOGI(TAG, "背景：软件解码 %dx%d 用时 %d ms（本次没走硬件路径）", s_shown_w, s_shown_h, ms);
    }
    ESP_LOGI(TAG, "背景：解码前后 内部内存 %d→%d KB，PSRAM %d→%d KB，输出缓冲 %d KB，行跨距 %d 字节",
             int_before, mem_kb(MALLOC_CAP_INTERNAL), ps_before, mem_kb(MALLOC_CAP_SPIRAM),
             (int)(s_frame_len / 1024), (int)s_dsc.header.stride);

    free(aligned);
    return true;
}

/* ------------------------------ LVGL 侧 ------------------------------ */

/** LVGL 线程：显示当前缓冲区并淡入。 */
/** 目标视觉状态：由任何线程设置，实际改 LVGL 对象必须在 LVGL 线程做。 */
static volatile bool s_want_dim_visible;
static volatile bool s_want_image_hidden;

/** 在 LVGL 线程里应用目标状态（lv_async_call 回调）。 */
static void apply_visual_cb(void *arg)
{
    (void)arg;
    if (s_dim != NULL) {
        lv_obj_set_style_bg_opa(s_dim,
                                s_want_dim_visible ? (lv_opa_t)(s_cfg.dim_pct * 255 / 100) : LV_OPA_TRANSP, 0);
        if (s_want_dim_visible) {
            lv_obj_remove_flag(s_dim, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_dim, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_img != NULL && s_want_image_hidden) {
        lv_obj_add_flag(s_img, LV_OBJ_FLAG_HIDDEN);
    }
}

/** 请求在 LVGL 线程里刷新视觉状态（可以从任意任务调用）。 */
static void request_visual_update(bool dim_visible, bool image_hidden)
{
    s_want_dim_visible = dim_visible;
    s_want_image_hidden = image_hidden;
    lv_lock();
    lv_async_call(apply_visual_cb, NULL);
    lv_unlock();
}

/** 淡入动画回调：lv_anim 的签名是 (void*, int32_t)，不能直接把
    lv_obj_set_style_opa(lv_obj_t*, lv_opa_t, selector) 强转过去。 */
static void fade_opa_cb(void *obj, int32_t opa)
{
    lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)opa, 0);
}

static void show_cb(void *arg)
{
    (void)arg;
    if (s_img == NULL) {
        /* UI 还没建好（或刚被重建）：留个标记，等 bg_image_bind 时补显示 */
        return;
    }
    s_dsc.header.w = s_shown_w;
    s_dsc.header.h = s_shown_h;
    lv_image_set_src(s_img, &s_dsc);

    /* 非 360×360 的图按比例缩放（面板会先裁好，这里只是兜底） */
    if (s_shown_w > 0 && s_shown_w != BG_W) {
        lv_image_set_scale(s_img, (uint32_t)(BG_W * 256 / s_shown_w));
    } else {
        lv_image_set_scale(s_img, 256);
    }

    lv_obj_remove_flag(s_img, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_opa(s_img, LV_OPA_TRANSP, 0);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_img);
    lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_anim_set_time(&a, 500);
    lv_anim_set_exec_cb(&a, fade_opa_cb);
    lv_anim_start(&a);
}

static void bg_task(void *arg)
{
    const int my_gen = (int)(intptr_t)arg;
    s_task_alive = true;

    /* 开机阶段 HA 实体拉取也在跑 TLS，两条握手同时开会把内部 RAM 顶爆
       （表现为 esp-tls 读错误 -0x7F00）。先等网络/首轮请求稳定下来。 */
    vTaskDelay(pdMS_TO_TICKS(4000));

    while (s_running && my_gen == s_gen) {
        if (s_cfg.count <= 0) {
            break;
        }
        while (s_paused && s_running && my_gen == s_gen) {
            vTaskDelay(pdMS_TO_TICKS(200));   /* 拉图表时先让路 */
        }
        if (s_index >= s_cfg.count) {
            s_index = 0;
        }
        const char *url = s_cfg.urls[s_index];

        size_t jpg_len = 0;
        uint8_t *jpg = NULL;
        if (bg_cache_get(s_index, url, &jpg, &jpg_len)) {
            ESP_LOGI(TAG, "background %d/%d 来自缓存", s_index + 1, s_cfg.count);
        } else {
            for (int attempt = 0; attempt < 3 && s_running; attempt++) {
                int st = 0;
                jpg = http_fetch(url, &jpg_len, &st);
                if (jpg != NULL) {
                    bg_cache_put(s_index, url, jpg, jpg_len);
                    break;
                }
                if (st == 404) {
                    ESP_LOGW(TAG, "图片不存在 (404)，跳过这张");
                    break;      /* 404 重试也没用 */
                }
                ESP_LOGW(TAG, "下载失败，%d 秒后重试 (%d/3)", 3, attempt + 1);
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
        }
        if (jpg != NULL) {
            if (decode_to_frame(jpg, jpg_len)) {
                lv_lock();
                lv_async_call(show_cb, NULL);
                lv_unlock();
                ESP_LOGI(TAG, "background %d/%d shown (%dx%d)", s_index + 1, s_cfg.count, s_shown_w, s_shown_h);
            }
            free(jpg);
        }

        /* 单张图不用轮播，停在原地等下一次配置 */
        if (s_cfg.count == 1) {
            while (s_running && s_cfg.count == 1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            continue;
        }

        for (int waited = 0; waited < s_cfg.interval_s * 10 && s_running && my_gen == s_gen; waited++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        s_index++;
    }
    if (my_gen == s_gen) {
        s_running = false;
    }
    /* 引擎由本任务创建/使用：只有「当前这一代」的任务退出时才释放。
       配置换代时旧任务（my_gen != s_gen）可能还没退出，新任务会直接复用
       同一个引擎——这时候绝不能删，否则新任务正在解码的引擎就没了。
       真正的释放点因此是：bg_image_stop() 让当前任务退出，或 bg_image_stop()
       在没有任务时兜底释放。 */
    if (my_gen == s_gen) {
        hw_engine_release();
    }
    s_task_alive = false;
    vTaskDelete(NULL);
}

/* ------------------------------- 配置解析 ------------------------------- */

void bg_image_init(void)
{
    if (s_initialized) {
        return;
    }
    s_initialized = true;
    s_cfg.interval_s = 60;
    s_cfg.dim_pct = 35;
    s_cfg.card_pct = 60;
    memset(&s_dsc, 0, sizeof(s_dsc));
    s_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    s_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
}

void bg_image_bind(lv_obj_t *parent)
{
    if (parent == NULL) {
        return;
    }

    /* 层级（从底到顶）：图片 → 暗化层 → 卡片。
       顺序必须是"先建暗化层放到底，再建图片放到底"，
       这样图片落在索引 0、暗化层紧随其后。 */
    s_dim = lv_obj_create(parent);
    lv_obj_remove_style_all(s_dim);
    lv_obj_set_size(s_dim, BG_W, BG_H);
    lv_obj_set_pos(s_dim, 0, 0);
    lv_obj_set_style_bg_color(s_dim, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_dim, s_cfg.dim_pct > 0 ? (lv_opa_t)(s_cfg.dim_pct * 255 / 100) : LV_OPA_TRANSP, 0);
    lv_obj_remove_flag(s_dim, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_dim, s_cfg.dim_pct > 0 ? 0 : LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_background(s_dim);

    s_img = lv_image_create(parent);
    lv_obj_set_size(s_img, BG_W, BG_H);
    lv_obj_set_pos(s_img, 0, 0);
    lv_obj_remove_flag(s_img, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_img, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_background(s_img);       /* 图片沉到最底，暗化层在它上面 */

    s_want_dim_visible = s_cfg.dim_pct > 0;
    s_want_image_hidden = !s_frame_ready;
    apply_visual_cb(NULL);      /* bind 本身就在 LVGL 线程 */

    ESP_LOGI(TAG, "bind: dim %d%% (opa %d), image behind", s_cfg.dim_pct,
             (int)(s_cfg.dim_pct * 255 / 100));

    /* 如果下载早就完成了，绑定之后立刻补显示 */
    if (s_frame_ready) {
        show_cb(NULL);
    }
}

uint8_t bg_image_card_opa_pct(void)
{
    return (uint8_t)s_cfg.card_pct;
}

void bg_image_set_paused(bool paused)
{
    s_paused = paused;
}

void bg_image_stop(void)
{
    s_running = false;
    for (int i = 0; i < s_cfg.count; i++) {
        bg_cache_drop(i);
    }
    s_cfg.count = 0;
    /* 硬件解码引擎归 bg_task 独占：任务还在跑就让它退出时自己释放，
       这里不能直接删（可能正有一次解码在飞，会 use-after-free）。
       任务已经退出/从没起来过，这里兜底释放。 */
    if (!s_task_alive) {
        hw_engine_release();
    }
    request_visual_update(false, true);   /* 隐藏图片也交给 LVGL 线程 */
}

void bg_image_apply_json(const char *json)
{
    bg_image_init();
    bg_cache_init();
    if (json == NULL) {
        bg_image_stop();
        return;
    }

    cJSON *root = cJSON_Parse(json);
    if (root == NULL) {
        return;
    }
    const cJSON *bg = cJSON_GetObjectItemCaseSensitive(root, "bg");
    if (!cJSON_IsObject(bg)) {
        cJSON_Delete(root);
        bg_image_stop();
        return;
    }

    /* 关键：必须等旧任务真的退出，否则每次重连都会多出一个任务在跑 TLS */
    s_running = false;
    s_gen++;
    for (int i = 0; i < 30 && s_task_alive; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (s_task_alive) {
        ESP_LOGW(TAG, "上一个背景任务 3 秒内没退出，继续");
    }

    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.interval_s = 60;
    s_cfg.dim_pct = 35;
    s_cfg.card_pct = 60;

    const cJSON *interval = cJSON_GetObjectItemCaseSensitive(bg, "interval");
    if (cJSON_IsNumber(interval) && interval->valueint >= 5) {
        s_cfg.interval_s = interval->valueint;
    }
    const cJSON *dim = cJSON_GetObjectItemCaseSensitive(bg, "dim");
    if (cJSON_IsNumber(dim)) {
        s_cfg.dim_pct = dim->valueint < 0 ? 0 : (dim->valueint > 80 ? 80 : dim->valueint);
    }

    const cJSON *card = cJSON_GetObjectItemCaseSensitive(bg, "card");
    if (cJSON_IsNumber(card)) {
        s_cfg.card_pct = card->valueint < 0 ? 0 : (card->valueint > 95 ? 95 : card->valueint);
    }

    const cJSON *images = cJSON_GetObjectItemCaseSensitive(bg, "images");
    if (cJSON_IsArray(images)) {
        const cJSON *item = NULL;
        cJSON_ArrayForEach(item, images)
        {
            if (!cJSON_IsString(item) || s_cfg.count >= BG_MAX_IMAGES) {
                continue;
            }
            snprintf(s_cfg.urls[s_cfg.count], BG_URL_MAX, "%s", item->valuestring);
            s_cfg.count++;
        }
    }
    cJSON_Delete(root);

    if (s_cfg.count == 0) {
        bg_image_stop();
        return;
    }

    /* 注意：这里跑在 MQTT 任务里，**不能**直接改 LVGL 对象，必须切回 LVGL 线程 */
    request_visual_update(s_cfg.dim_pct > 0, false);

    for (int i = s_cfg.count; i < BG_MAX_IMAGES; i++) {
        bg_cache_drop(i);      /* 数量变少时清掉多余缓存 */
    }
    ESP_LOGI(TAG, "background: %d image(s), %ds, dim %d%%", s_cfg.count, s_cfg.interval_s, s_cfg.dim_pct);

    s_index = 0;
    s_running = true;
    s_hw_broken = false;        /* 新配置：硬件解码再试一次（上次可能只是开机内存紧） */
    ESP_LOGI(TAG, "starting bg task, internal heap %d KB (largest block %d KB)",
             (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (int)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
    if (xTaskCreate(bg_task, "bg", 10240, (void *)(intptr_t)s_gen, 3, NULL) != pdPASS) {
        s_running = false;
        ESP_LOGE(TAG, "failed to start background task");
    }
}
