/*
 * screenshot.c — 远程调试：整屏截图 + 翻页（MQTT 命令，不用碰设备）
 *
 * ── 抓屏路线：为什么不是 lv_snapshot ────────────────────────────────
 * 本工程 vendored 的 LVGL 9.6 没有开 CONFIG_LV_USE_SNAPSHOT（sdkconfig 里是
 * "# CONFIG_LV_USE_SNAPSHOT is not set"，改 sdkconfig 不在本功能的范围内），
 * 所以 lv_snapshot_take() 根本编不进来。改用"复用现有 flush 通路"：
 *
 *   1. jpeg_alloc_encoder_mem() 在 PSRAM 里要一块 360×360×2 = 259,200 B 的
 *      整屏 RGB565 缓冲（这个 helper 内部就是 heap_caps_aligned_calloc
 *      MALLOC_CAP_SPIRAM，地址按 cache line / 2D-DMA 要求对齐，硬件 JPEG
 *      编码器正好能直接吃它）；
 *   2. 置 s_capture_on，再 lv_obj_invalidate(lv_screen_active()) +
 *      lv_refr_now(NULL) 强制整屏重绘；
 *   3. 重绘过程中 main.c 的 lv_flush_cb() 每刷一块（本工程绘制缓冲 = 360×16，
 *      整屏 23 块）就调 screenshot_flush_hook()，按 flush 传下来的 area 坐标
 *      把这块拷进整屏缓冲（顺手把 RGB565 的字节序从 LVGL 的 SWAPPED 换成
 *      原生小端 —— 面板要大端，JPEG 编码器要小端）；
 *   4. 清 s_capture_on，把整屏缓冲交给后台任务：硬件 JPEG 编码 → MQTT/串口。
 *
 * 抓屏这一步只花一次整屏刷新的时间（本工程实测整屏 ~7-10 ms + LVGL 渲染），
 * 远低于"LVGL 任务不许卡 300 ms"的红线；编码（几十 ms）+ 上传（几十片）都在
 * 独立任务 "shot" 里做。
 *
 * ── 内存 ──────────────────────────────────────────────────────────
 * 全部来自 PSRAM：整屏 259,200 B + JPEG 位流 96 KB + base64 分片 4 KB
 * ≈ 349 KB 峰值，传完立刻释放。内部 RAM 只多一个 5 KB 栈的任务（跑完就退）。
 *
 * ── 传输 ──────────────────────────────────────────────────────────
 * 主路：MQTT（另一条独立客户端，见下面 shot_mqtt_ensure 的注释）
 *   主题 switchyard/<dev>/shot            ← 一帧 JSON 头 {id,size,chunks,w,h,format,...}
 *   主题 switchyard/<dev>/shot/<id>/<i>   ← 第 i 片，payload 就是纯 base64 文本
 *   每片原始 3072 B → base64 4096 字符 → 报文约 4.1 KB（out_size 4608）
 * 备用：串口 base64（{"cmd":"shot_serial"}，默认关着，免得刷屏）
 *   ===IMG BEGIN w=360 h=360 size=N===
 *   <seq>,<base64 512 字符>
 *   ...
 *   ===IMG END===
 *
 * ── 翻页 / 开页 ────────────────────────────────────────────────────
 * {"cmd":"page","value":N} → ui_home_goto_first() + N × ui_home_page_step(+1)。
 * ui_home 里没有"跳到第 N 页"的 API，而且翻页有 220 ms 滑动动画，动画期间
 * ui_home_page_step() 会被 s_animating 丢掉，所以这一步必须跨轮次做：
 * screenshot_tick()（LVGL 主循环每轮调用）里的状态机每 PG_STEP_MS 走一步。
 *
 * {"cmd":"open","entity":"light.xxx"} 走点卡片那条公开入口 ui_home_open_detail()
 * （灯 / 空调 / 风扇共用一个详情页，按 entity 前缀决定显示哪组控件）；
 * weather.* 走 ui_home_weather_show()。{"cmd":"close"} 走左边缘右滑那条路径：
 * 天气页 → 详情页依次关掉。截图对"哪一屏"没有要求，详情页 / 天气页 / 设置页
 * 都是同一个活动屏上的对象，抓屏路线完全一样。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "lvgl.h"
#include "mqtt_client.h"
#include "driver/jpeg_encode.h"

#include "screenshot.h"
#include "app_settings.h"
#include "net.h"
#include "ui_home.h"

static const char *TAG = "shot";

/* ---- 屏幕参数：必须和 main.c 的 LCD_W / LCD_H 一致 ---- */
#define SHOT_W 360
#define SHOT_H 360
#define SHOT_FRAME_BYTES ((size_t)SHOT_W * (size_t)SHOT_H * 2)   /* 259,200 B */

/* JPEG 位流缓冲：360×360 / q80 / 4:4:4 大概 25-45 KB，96 KB 留足余量 */
#define SHOT_JPEG_MAX (96 * 1024)

/* 每片原始字节：3072 → base64 4096 字符，一个报文约 4.1 KB（客户端 out_size 4608） */
#define SHOT_CHUNK_RAW 3072

/* 串口 base64 每行原始字节：384 → 512 字符/行，115200 下约 45 ms/行 */
#define SHOT_SERIAL_LINE 384

#define SHOT_DEFAULT_QUALITY 80

/* 截图默认等 250 ms 再抓：翻页 / 开详情页的滑动动画是 220 ms，立刻抓会拍到半路 */
#define SHOT_DEFAULT_DELAY_MS 250

/* 翻页：ui_home 的 PAGE_ANIM_MS = 220，这里留 320 ms 让动画走完再下一步 */
#define PG_STEP_MS 320

/* ======================================================================
 *  状态
 * ====================================================================== */

/* 抓屏（只在 LVGL 任务里读写） */
static volatile bool s_capture_on;
static uint8_t *s_frame;              /* PSRAM：整屏 RGB565（原生小端） */
static bool s_swap = true;            /* 抓屏时是否对调 RGB565 高低字节（order 的 bit0） */
static bool s_row_got[SHOT_H];
static int s_row_count;

/* 一次截图任务：同一时刻只允许一个 */
static volatile bool s_busy;
static uint8_t *s_jpeg;               /* PSRAM：JPEG 位流输出 */
static uint8_t *s_b64;                /* PSRAM：一片的 base64 文本 */
static struct {
    bool serial;
    int quality;
    int order;                        /* 0-3，见 screenshot_capture() 的注释 */
    bool flip;                        /* order bit1：JPEG pixel_reverse */
} s_job;

/* 延迟抓屏（等动画走完）：screenshot_tick() 里到点再真抓 */
static struct {
    bool pending;
    int64_t at_us;
} s_pending;

/* MQTT 上传用的独立客户端（见 shot_mqtt_ensure） */
static esp_mqtt_client_handle_t s_mqtt;
static volatile bool s_mqtt_up;
static int s_shot_id;                 /* 递增的截图序号 */

/* 翻页状态机 */
typedef enum {
    PG_IDLE = 0,
    PG_WAIT_FIRST,                    /* 等子页面关掉再 goto_first */
    PG_WAIT_STEP,                     /* 等上一页动画走完再下一步 */
} pg_phase_t;

static struct {
    pg_phase_t phase;
    int target;                       /* 目标页（0 基） */
    int remaining;                    /* 还要走几步 */
    int64_t next_us;
} s_pg = { PG_IDLE, 0, 0, 0 };

/* ======================================================================
 *  抓屏：flush 钩子 + 整屏重绘
 * ====================================================================== */

/**
 * main.c 的 lv_flush_cb() 每次都调这里。没在抓屏时只是一次 bool 判断。
 *
 * LVGL 9 的 partial 渲染模式保证：px_map 里就是 area 这一块、按行紧密排列的像素
 * （宽度 = area->x2-x1+1），所以按 area 的宽高拷就行。
 * 字节序：本工程 lv_display 用 LV_COLOR_FORMAT_RGB565_SWAPPED（面板要大端），
 * 而硬件 JPEG 编码器（JPEG_ENCODE_IN_FORMAT_RGB565 / pixel_reverse=false）要的是
 * 原生小端 RGB565 —— 也就是和 LVGL 内部 uint16 值一致。所以这里逐像素对调高低字节。
 */
void screenshot_flush_hook(const lv_area_t *area, const uint8_t *px_map)
{
    if (!s_capture_on || s_frame == NULL || area == NULL || px_map == NULL) {
        return;
    }

    const int w = (int)(area->x2 - area->x1 + 1);
    const int h = (int)(area->y2 - area->y1 + 1);
    if (w <= 0 || h <= 0) {
        return;
    }

    for (int row = 0; row < h; row++) {
        const int y = (int)area->y1 + row;
        if (y < 0 || y >= SHOT_H) {
            continue;
        }
        const uint16_t *src = (const uint16_t *)(px_map + (size_t)row * (size_t)w * 2);
        uint16_t *dst = (uint16_t *)s_frame + (size_t)y * SHOT_W;
        for (int col = 0; col < w; col++) {
            const int x = (int)area->x1 + col;
            if (x < 0 || x >= SHOT_W) {
                continue;
            }
            /* 默认（order=0）：SWAPPED（内存里高字节在前）→ 原生小端，
               按 uint16 读出来再对调就是 LVGL 内部的 uint16 值。 */
            dst[x] = s_swap ? __builtin_bswap16(src[col]) : src[col];
        }
        if (!s_row_got[y]) {
            s_row_got[y] = true;
            s_row_count++;
        }
    }
}

/** 强制整屏重绘一遍，把画面搬进 s_frame。返回 true 表示至少抓到了部分像素。 */
static bool shot_grab_screen(void)
{
    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL) {
        ESP_LOGE(TAG, "没有活动屏幕（LVGL 未初始化？），放弃截图");
        return false;
    }

    memset(s_row_got, 0, sizeof(s_row_got));
    s_row_count = 0;

    const int64_t t0 = esp_timer_get_time();
    int attempts = 0;
    s_capture_on = true;
    /* 正常情况下第一次就覆盖满 360 行；万一 LVGL 只重绘了一部分（比如有对象
       正在被删除），再补一次整屏重绘。两次都缺行就照实报出来。 */
    while (attempts < 2 && s_row_count < SHOT_H) {
        lv_obj_invalidate(lv_screen_active());
        lv_refr_now(NULL);        /* LVGL 9.6 签名：void lv_refr_now(lv_display_t *disp)，NULL = 所有显示 */
        attempts++;
    }
    s_capture_on = false;
    const int64_t us = esp_timer_get_time() - t0;

    if (s_row_count < SHOT_H) {
        ESP_LOGW(TAG, "只抓到 %d/%d 行（第 %d 次整屏重绘后仍不全），缺的行是黑的",
                 s_row_count, SHOT_H, attempts);
    }
    ESP_LOGI(TAG, "整屏重绘 %d 次，覆盖 %d/%d 行，耗时 %d ms（%u 块 flush）",
             attempts, s_row_count, SHOT_H, (int)(us / 1000),
             (unsigned)((s_row_count + 15) / 16));
    return s_row_count > 0;
}

/* ======================================================================
 *  JPEG 编码（硬件编码器）
 * ====================================================================== */

static esp_err_t shot_encode(int quality, bool flip, uint32_t *out_size)
{
    /* 引擎每次按需创建/释放：本工程内部 RAM 紧张，不常驻。
       bg_image.c 的硬件解码器也常驻着，两者共用同一套 JPEG codec 句柄 + 互斥量
       （驱动内部引用计数 + codec_mutex 串行化），可以同时存在。 */
    const jpeg_encode_engine_cfg_t eng_cfg = {
        .intr_priority = 0,          /* 0 = 驱动选默认优先级 */
        .timeout_ms = 3000,          /* 360×360 编码实测几十 ms，3 s 兜底 */
    };
    jpeg_encoder_handle_t enc = NULL;
    esp_err_t err = jpeg_new_encoder_engine(&eng_cfg, &enc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "jpeg_new_encoder_engine 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 4:4:4（MCU 8×8）：360 正好是 8 的整数倍，不会出现最后一个 MCU 跨行；
       4:2:2/4:2:0 的 MCU 宽 16，360 不是 16 的倍数，末列要靠硬件补齐，不划算。
       pixel_reverse 由 order 的 bit1 决定：默认 false（输入是原生小端的 RGB565，
       见 flush 钩子）；万一硬件认的是另一种字节序，MQTT 发 "order":2 就能试出来。 */
    const jpeg_encode_cfg_t cfg = {
        .width = SHOT_W,
        .height = SHOT_H,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV444,
        .image_quality = (uint32_t)quality,
        .pixel_reverse = flip,
    };

    uint32_t written = 0;
    err = jpeg_encoder_process(enc, &cfg, s_frame, (uint32_t)SHOT_FRAME_BYTES,
                               s_jpeg, (uint32_t)SHOT_JPEG_MAX, &written);
    jpeg_del_encoder_engine(enc);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "jpeg_encoder_process 失败: %s", esp_err_to_name(err));
        return err;
    }
    if (written == 0 || written > SHOT_JPEG_MAX) {
        ESP_LOGE(TAG, "JPEG 输出长度异常: %u", (unsigned)written);
        return ESP_FAIL;
    }
    *out_size = written;
    return ESP_OK;
}

/* ======================================================================
 *  base64
 * ====================================================================== */

static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/** 把 n 字节编成 base64 写进 out（调用方保证 out 够大：n*4/3+4）。返回字符数。 */
static int shot_base64(const uint8_t *in, uint32_t n, char *out)
{
    int o = 0;
    uint32_t i = 0;
    for (; i + 2 < n; i += 3) {
        const uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1] << 8) | in[i + 2];
        out[o++] = B64[(v >> 18) & 0x3F];
        out[o++] = B64[(v >> 12) & 0x3F];
        out[o++] = B64[(v >> 6) & 0x3F];
        out[o++] = B64[v & 0x3F];
    }
    if (i < n) {
        uint32_t v = (uint32_t)in[i] << 16;
        out[o++] = B64[(v >> 18) & 0x3F];
        if (i + 1 < n) {
            v |= (uint32_t)in[i + 1] << 8;
            out[o++] = B64[(v >> 12) & 0x3F];
            out[o++] = B64[(v >> 6) & 0x3F];
            out[o++] = '=';
        } else {
            out[o++] = B64[(v >> 12) & 0x3F];
            out[o++] = '=';
            out[o++] = '=';
        }
    }
    out[o] = '\0';
    return o;
}

/* ======================================================================
 *  MQTT 上传
 * ====================================================================== */

static void shot_mqtt_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_up = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_up = false;
        break;
    default:
        break;
    }
}

/**
 * 截图用的 MQTT 客户端（懒创建，建好就一直留着）。
 *
 * 为什么要单独一条连接：net.c 里的 s_mqtt 是 static，net.h 只暴露 state/log 两个
 * 发布接口，没有"往任意主题发一条"的 API；而 net.c 不在本功能的修改范围里。
 * 所以这里自己起一个客户端：client_id 加 "-shot" 后缀（绝不能用默认的 ESP32_<MAC>，
 * 否则会把主客户端顶下线），只发不订，QoS0 + retain=false。
 * 内存：内部 RAM 约 2 KB + 4.6 KB 缓冲 + 4 KB 栈 ≈ 11 KB，且只在第一次截图时创建。
 */
static bool shot_mqtt_ensure(void)
{
    if (s_mqtt == NULL) {
        const app_settings_t *sc = app_settings_get();
        if (sc->mqtt_host[0] == '\0') {
            ESP_LOGE(TAG, "没配 MQTT 服务器，无法上传截图（可改用 {\"cmd\":\"shot_serial\"}）");
            return false;
        }

        /* uri / client_id 必须是静态的：配置指针在客户端生命周期内都要有效 */
        static char uri[96];
        static char cid[64];
        snprintf(uri, sizeof(uri), "mqtt://%s:%d", sc->mqtt_host,
                 sc->mqtt_port > 0 ? sc->mqtt_port : 1883);
        snprintf(cid, sizeof(cid), "%s-shot", net_device_id());

        const esp_mqtt_client_config_t cfg = {
            .broker.address.uri = uri,
            .credentials.username = sc->mqtt_user,
            .credentials.authentication.password = sc->mqtt_pass,
            .credentials.client_id = cid,
            .session.keepalive = 30,
            .buffer.size = 2048,      /* 收：只有 CONNACK / PINGRESP */
            .buffer.out_size = 4608,  /* 发：一片 base64 4096 + 主题 + 报文头 */
            .task.stack_size = 4096,
            .task.priority = 3,
        };

        s_mqtt = esp_mqtt_client_init(&cfg);
        if (s_mqtt == NULL) {
            ESP_LOGE(TAG, "截图 MQTT 客户端创建失败（内部 RAM 不足）");
            return false;
        }
        esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, shot_mqtt_event, NULL);
        const esp_err_t err = esp_mqtt_client_start(s_mqtt);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "截图 MQTT 启动失败: %s", esp_err_to_name(err));
            esp_mqtt_client_destroy(s_mqtt);
            s_mqtt = NULL;
            return false;
        }
        ESP_LOGI(TAG, "截图 MQTT 客户端已创建（%s，client_id=%s，内部 RAM %u KB）",
                 uri, cid, (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    }

    /* 等连接（最多 5 s；断线重连由 esp-mqtt 自己负责） */
    for (int i = 0; i < 100 && !s_mqtt_up; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!s_mqtt_up) {
        ESP_LOGE(TAG, "截图 MQTT 5 s 内没连上");
        return false;
    }
    return true;
}

static bool shot_send_mqtt(uint32_t size, int quality)
{
    if (!shot_mqtt_ensure()) {
        return false;
    }

    const char *dev = net_device_id();
    const int chunks = (int)((size + SHOT_CHUNK_RAW - 1) / SHOT_CHUNK_RAW);
    const int id = ++s_shot_id;

    char topic[96];
    char hdr[224];

    /* 1) 先发一帧头：PC 端拿到 size/chunks 才好按 index 拼回来 */
    snprintf(hdr, sizeof(hdr),
             "{\"id\":%d,\"size\":%u,\"chunks\":%d,\"chunk_raw\":%d,\"w\":%d,\"h\":%d,"
             "\"format\":\"jpeg\",\"quality\":%d,\"order\":%d}",
             id, (unsigned)size, chunks, SHOT_CHUNK_RAW, SHOT_W, SHOT_H, quality, s_job.order);
    snprintf(topic, sizeof(topic), "switchyard/%s/shot", dev);
    int r = esp_mqtt_client_publish(s_mqtt, topic, hdr, 0, 0, 0);
    if (r < 0) {
        ESP_LOGE(TAG, "截图头发布失败（MQTT 掉线？）");
        return false;
    }
    ESP_LOGI(TAG, "截图头已发: %s %s", topic, hdr);

    /* 2) 逐片发（publish 是同步写 socket，天然限速；每 8 片让一下 CPU 给 WiFi/LVGL） */
    for (int i = 0; i < chunks; i++) {
        const uint32_t off = (uint32_t)i * SHOT_CHUNK_RAW;
        uint32_t n = size - off;
        if (n > SHOT_CHUNK_RAW) {
            n = SHOT_CHUNK_RAW;
        }
        const int blen = shot_base64(s_jpeg + off, n, (char *)s_b64);
        snprintf(topic, sizeof(topic), "switchyard/%s/shot/%d/%d", dev, id, i);
        r = esp_mqtt_client_publish(s_mqtt, topic, (const char *)s_b64, blen, 0, 0);
        if (r < 0) {
            ESP_LOGE(TAG, "第 %d/%d 片发布失败", i + 1, chunks);
            return false;
        }
        if ((i & 7) == 7) {
            vTaskDelay(1);
        }
    }

    /* 3) 收尾：publish 已经把数据写进 socket，这里再等一会儿让 WiFi 把尾巴送完，
          避免紧接着的深睡/重启把最后几片丢掉。 */
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "截图上传完成: id=%d %u 字节 / %d 片（每片原始 %d B → base64 %d B，报文约 %d B）",
             id, (unsigned)size, chunks, SHOT_CHUNK_RAW,
             (SHOT_CHUNK_RAW * 4 + 2) / 3, (SHOT_CHUNK_RAW * 4 + 2) / 3 + 40);
    return true;
}

/* ======================================================================
 *  串口上传（备用，必须显式 {"cmd":"shot_serial"} 才会走）
 * ====================================================================== */

static void shot_send_serial(uint32_t size)
{
    /* 115200 8N1 ≈ 11.5 KB/s：size 30 KB → base64 40 KB → 约 3.5 s。
       为了少让日志插进 base64 行里，先把日志级别抬到 WARN，打完再恢复原级别。
       （ESP_LOG 走的是同一个 stdout/UART，同一行里混进日志会让这一行 base64 作废。） */
    const esp_log_level_t lvl_saved = esp_log_level_get("*");
    esp_log_level_set("*", ESP_LOG_WARN);

    printf("===IMG BEGIN w=%d h=%d size=%u===\n", SHOT_W, SHOT_H, (unsigned)size);
    fflush(stdout);

    int seq = 0;
    for (uint32_t off = 0; off < size; off += SHOT_SERIAL_LINE) {
        uint32_t n = size - off;
        if (n > SHOT_SERIAL_LINE) {
            n = SHOT_SERIAL_LINE;
        }
        shot_base64(s_jpeg + off, n, (char *)s_b64);
        printf("%d,%s\n", seq++, (char *)s_b64);
        fflush(stdout);
    }
    printf("===IMG END===\n");
    fflush(stdout);

    esp_log_level_set("*", lvl_saved);
    ESP_LOGI(TAG, "串口截图已打印：%u 字节 / %d 行 base64（115200 下约 %.1f s）",
             (unsigned)size, seq, (double)(size * 4 / 3 + 24 * seq) / 11520.0);
}

/* ======================================================================
 *  截图任务
 * ====================================================================== */

static void shot_free_buffers(void)
{
    if (s_frame != NULL) {
        free(s_frame);
        s_frame = NULL;
    }
    if (s_jpeg != NULL) {
        free(s_jpeg);
        s_jpeg = NULL;
    }
    if (s_b64 != NULL) {
        free(s_b64);
        s_b64 = NULL;
    }
}

static void shot_task(void *arg)
{
    (void)arg;
    /* 任务参数一次性拷出来：之后 s_job 不会再被改（s_busy 保证） */
    const bool serial = s_job.serial;
    const int quality = s_job.quality;
    const bool flip = s_job.flip;

    uint32_t size = 0;
    const int64_t t0 = esp_timer_get_time();
    const esp_err_t err = shot_encode(quality, flip, &size);
    const int64_t enc_ms = (esp_timer_get_time() - t0) / 1000;

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "JPEG 编码完成: %u 字节（%u KB），质量 %d，4:4:4，耗时 %d ms",
                 (unsigned)size, (unsigned)(size / 1024), quality, (int)enc_ms);
        if (serial) {
            shot_send_serial(size);
        } else if (!shot_send_mqtt(size, quality)) {
            ESP_LOGW(TAG, "MQTT 上传失败；改用 {\"cmd\":\"shot_serial\"} 可以从串口取同一张图");
        }
    } else {
        ESP_LOGE(TAG, "截图失败（编码阶段）: %s", esp_err_to_name(err));
    }

    shot_free_buffers();
    ESP_LOGI(TAG, "截图结束，PSRAM 已释放（当前 PSRAM %u KB，内部 RAM %u KB）",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    s_busy = false;
    vTaskDelete(NULL);
}

/* ======================================================================
 *  对外：截图
 * ====================================================================== */

static bool shot_start(void);

bool screenshot_capture(bool serial, int quality, int delay_ms, int order)
{
    if (s_busy) {
        ESP_LOGW(TAG, "上一次截图还在排队/编码/上传，忽略本次请求");
        return false;
    }
    if (quality <= 0 || quality > 100) {
        quality = SHOT_DEFAULT_QUALITY;
    }
    if (order < 0 || order > 3) {
        order = 0;
    }
    if (delay_ms < 0) {
        delay_ms = SHOT_DEFAULT_DELAY_MS;
    }
    if (delay_ms > 10000) {
        delay_ms = 10000;
    }

    /* 任务参数只有一个来源：s_job（s_busy 保证同一时刻只有一份） */
    s_job.serial = serial;
    s_job.quality = quality;
    s_job.order = order;
    s_job.flip = (order & 2) != 0;

    if (delay_ms > 0) {
        s_busy = true;
        s_pending.pending = true;
        s_pending.at_us = esp_timer_get_time() + (int64_t)delay_ms * 1000;
        ESP_LOGI(TAG, "截图已排队：%d ms 后抓（等滑动动画走完；要立刻抓就发 \"delay\":0）",
                 delay_ms);
        return true;
    }

    return shot_start();
}

static bool shot_start(void)
{
    const bool serial = s_job.serial;
    const int quality = s_job.quality;
    /* order 的 bit0 决定抓屏时要不要对调字节（必须在这里定，抓屏在 LVGL 任务里做） */
    s_swap = (s_job.order & 1) == 0;

    s_busy = true;

    /* 1) 整屏 RGB565 缓冲（PSRAM，按 2D-DMA / cache line 对齐） */
    const jpeg_encode_memory_alloc_cfg_t in_mem = {
        .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER,
    };
    size_t got = 0;
    const uint32_t ps_before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
    s_frame = (uint8_t *)jpeg_alloc_encoder_mem(SHOT_FRAME_BYTES, &in_mem, &got);
    if (s_frame == NULL) {
        ESP_LOGE(TAG, "PSRAM 不足：整屏缓冲 %u KB 申请失败，放弃截图",
                 (unsigned)(SHOT_FRAME_BYTES / 1024));
        s_busy = false;
        return false;
    }

    /* 2) 强制整屏重绘，flush 钩子把画面搬进 s_frame */
    const bool ok = shot_grab_screen();
    if (!ok) {
        shot_free_buffers();
        s_busy = false;
        return false;
    }

    /* 3) JPEG 位流 + 一片 base64 的缓冲，也都在 PSRAM */
    const jpeg_encode_memory_alloc_cfg_t out_mem = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    s_jpeg = (uint8_t *)jpeg_alloc_encoder_mem(SHOT_JPEG_MAX, &out_mem, &got);
    s_b64 = (uint8_t *)heap_caps_malloc((SHOT_CHUNK_RAW * 4 + 2) / 3 + 8, MALLOC_CAP_SPIRAM);
    if (s_jpeg == NULL || s_b64 == NULL) {
        ESP_LOGE(TAG, "PSRAM 不足：JPEG 输出 %u KB / base64 %u B 申请失败，放弃截图",
                 (unsigned)(SHOT_JPEG_MAX / 1024), (unsigned)((SHOT_CHUNK_RAW * 4 + 2) / 3 + 8));
        shot_free_buffers();
        s_busy = false;
        return false;
    }

    const uint32_t ps_after = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
    ESP_LOGI(TAG, "截图缓冲已就绪：整屏 %u KB + JPEG %u KB + base64 %u B，"
                  "PSRAM %u → %u KB（峰值占用约 %u KB）",
             (unsigned)(SHOT_FRAME_BYTES / 1024), (unsigned)(SHOT_JPEG_MAX / 1024),
             (unsigned)((SHOT_CHUNK_RAW * 4 + 2) / 3 + 8),
             (unsigned)ps_before, (unsigned)ps_after, (unsigned)(ps_before - ps_after));

    /* 4) 编码 + 上传丢给独立任务（LVGL 任务到此为止只花了一次整屏刷新） */
    if (xTaskCreate(shot_task, "shot", 5120, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "截图任务创建失败（内部 RAM 不足）");
        shot_free_buffers();
        s_busy = false;
        return false;
    }

    ESP_LOGI(TAG, "截图开始：%s，质量 %d，order=%d（%s字节序，pixel_reverse=%d）",
             serial ? "串口 base64" : "MQTT 分片", quality, s_job.order,
             s_swap ? "对调" : "原样", (int)s_job.flip);
    return true;
}

/* ======================================================================
 *  对外：导航（翻页 / 开详情页 / 关覆盖页）
 * ====================================================================== */

bool screenshot_close_overlay(void)
{
    /* 顺序照 ui_home.c 的左边缘右滑来：天气页 → 折线页 → 详情页。
       ui_home_chart_open()/close 目前是空实现（那一页已经删了），留着不影响。 */
    bool closed = false;
    if (ui_home_weather_open()) {
        ui_home_weather_hide();
        closed = true;
        ESP_LOGI(TAG, "关闭天气页");
    }
    if (ui_home_chart_open()) {
        ui_home_close_chart();
        closed = true;
        ESP_LOGI(TAG, "关闭折线页");
    }
    if (ui_home_detail_open()) {
        ui_home_close_detail();
        closed = true;
        ESP_LOGI(TAG, "关闭详情页");
    }
    if (!closed) {
        ESP_LOGI(TAG, "没有打开的覆盖页（已经在卡片页上）");
    }
    return closed;
}

bool screenshot_open_entity(const char *entity)
{
    if (entity == NULL || entity[0] == '\0') {
        ESP_LOGW(TAG, "open 的 entity 为空");
        return false;
    }

    /* 两层覆盖页会叠在一起：天气页先关掉（详情页之间切换 ui_home 自己会处理） */
    if (ui_home_weather_open() && strncmp(entity, "weather.", 8) != 0) {
        ui_home_weather_hide();
        ESP_LOGI(TAG, "打开详情页前先关掉天气页");
    }

    if (strncmp(entity, "weather.", 8) == 0) {
        ui_home_weather_show(entity);
        ESP_LOGI(TAG, "打开天气页: %s", entity);
        return true;
    }

    if (strncmp(entity, "light.", 6) == 0) {
        ui_home_open_detail(entity);
        ESP_LOGI(TAG, "打开详情页: %s（灯：亮度/色温）", entity);
    } else if (strncmp(entity, "climate.", 8) == 0) {
        ui_home_open_detail(entity);
        ESP_LOGI(TAG, "打开详情页: %s（空调：温度/模式/风速）", entity);
    } else if (strncmp(entity, "fan.", 4) == 0) {
        ui_home_open_detail(entity);
        ESP_LOGI(TAG, "打开详情页: %s（风扇：风速/预设/摇头）", entity);
    } else {
        /* ui_home 按前缀兜底：不认识的前缀会当灯页打开（显示 --）。照实提醒。 */
        ESP_LOGW(TAG, "entity 不是 light./climate./fan./weather.，仍按灯详情页打开: %s", entity);
        ui_home_open_detail(entity);
    }
    return true;
}

/** 翻页前先把覆盖页关掉（ui_home_page_step() 在子页面开着时会直接 return）。 */
static bool shot_close_overlays(void)
{
    return screenshot_close_overlay();
}

bool screenshot_goto_page(int page)
{
    if (page < 0) {
        page = 0;
    }
    if (page > 15) {
        page = 15;
    }
    if (s_pg.phase != PG_IDLE) {
        ESP_LOGW(TAG, "上一个翻页命令还没走完（剩余 %d 步），忽略本次", s_pg.remaining);
        return false;
    }

    /* ui_home 没有 "go to page N" 的 API：只能先回第一页，再 N 次 page_step(+1)。
       每步之间要等 220 ms 的滑动动画，所以交给 screenshot_tick() 的
       状态机分轮次走（一次性连调 N 次只会动一页，会被 s_animating 丢掉）。 */
    const bool closed = shot_close_overlays();
    s_pg.target = page;
    s_pg.remaining = page;

    if (page == 0) {
        ui_home_goto_first();
        s_pg.phase = PG_IDLE;
        ESP_LOGI(TAG, "翻页完成：目标第 0 页（第一页）");
        return true;
    }

    s_pg.phase = PG_WAIT_FIRST;
    /* 关子页面本身也有 ~200 ms 滑出动画，此时 goto_first 会被 s_animating 丢掉 */
    s_pg.next_us = esp_timer_get_time() + (closed ? PG_STEP_MS : 30) * 1000LL;
    ESP_LOGI(TAG, "翻页开始：目标第 %d 页（0 基），共 %d 步，每步 %d ms",
             page, page, PG_STEP_MS);
    return true;
}

void screenshot_page_step(int dir)
{
    if (dir == 0) {
        return;
    }
    if (ui_home_weather_open() || ui_home_detail_open() || ui_home_chart_open()) {
        ESP_LOGW(TAG, "子页面开着，翻页被忽略（先发 {\"cmd\":\"page\",\"value\":N}，它会先关子页面）");
        return;
    }
    ui_home_page_step(dir > 0 ? 1 : -1);
    ESP_LOGI(TAG, "翻页：%s%s", dir > 0 ? "下一页" : "上一页",
             ui_home_assistant_current() ? "（当前是小智助手页）" : "");
}

void screenshot_tick(void)
{
    /* 1) 延迟抓屏到点了：就在 LVGL 任务里抓（tick 本来跑在主循环里） */
    if (s_pending.pending && esp_timer_get_time() >= s_pending.at_us) {
        s_pending.pending = false;
        s_busy = false;                    /* 让 shot_start 自己重新置上 */
        if (!shot_start()) {
            ESP_LOGE(TAG, "延迟截图启动失败");
        }
    }

    /* 2) 翻页状态机 */
    if (s_pg.phase == PG_IDLE) {
        return;
    }
    if (esp_timer_get_time() < s_pg.next_us) {
        return;
    }

    if (s_pg.phase == PG_WAIT_FIRST) {
        ui_home_goto_first();
        s_pg.phase = PG_WAIT_STEP;
        s_pg.next_us = esp_timer_get_time() + PG_STEP_MS * 1000LL;
        return;
    }

    /* PG_WAIT_STEP */
    if (s_pg.remaining > 0) {
        ui_home_page_step(1);
        s_pg.remaining--;
        s_pg.next_us = esp_timer_get_time() + PG_STEP_MS * 1000LL;
        if (s_pg.remaining == 0) {
            s_pg.phase = PG_IDLE;
            /* ui_home 没有 "当前第几页" 的取值接口，只能确认是不是小智页。
               目标页数超过实际页数时 page_step 会绕回（show_page_dir 取模）。 */
            ESP_LOGI(TAG, "翻页完成：目标第 %d 页（0 基）%s", s_pg.target,
                     ui_home_assistant_current() ? "，当前是小智助手页（最后一页）" : "");
        }
        return;
    }

    s_pg.phase = PG_IDLE;
}
