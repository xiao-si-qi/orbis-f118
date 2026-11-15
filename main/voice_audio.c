/**
 * @file voice_audio.c
 * @brief PDM 麦克风录音 + PCM5100A（I2S 标准模式 TX）播放（小智语音的音频底层）。
 *
 * 说明：
 *   - 麦克风是 PDM 数字麦（SCK/DATA），用 I2S 外设的 PDM RX + 硬件 PDM2PCM 滤波取 PCM，
 *     单声道 16bit，默认 16 kHz；PDM2PCM 只有 I2S0 支持，所以录音固定 I2S_NUM_0；
 *   - 喇叭是 PCM5100A（纯 DAC，只有 BCK/DIN/LRCK，无 I2C 控制、无静音脚），
 *     I2S 标准模式 TX only，固定 I2S_NUM_1；
 *   - 采样率跟着调用方走（录音 16k、播放 24k 与小智协议一致），变了就重建通道；
 *   - 本板无静音脚：voice_audio_mute() 是空实现，只为保留上层符号。
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2s_pdm.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"

#include "voice_audio.h"

static const char *TAG = "voice_audio";

/* ---- 引脚（厂商原理图）---- */
#define DAC_BCLK_GPIO   53   /* I2S_DAC_BCK  */
#define DAC_DIN_GPIO    52   /* I2S_DAC_DIN  */
#define DAC_LRCK_GPIO   51   /* I2S_DAC_LRCK/WS */

#define PDM_DATA_GPIO   37   /* PDM_MIC_DATA */
#define PDM_SCK_GPIO    39   /* PDM_MIC_SCK  */

/* PDM RX（PDM2PCM 只挂在 I2S0）与 I2S STD TX 各占一个控制器 */
#define VOICE_RX_PORT   I2S_NUM_0
#define VOICE_TX_PORT   I2S_NUM_1

#define VOICE_RX_RATE_DEFAULT 16000
#define VOICE_TX_RATE_DEFAULT 24000

/* ---- 阻塞上限（看门狗防线）------------------------------------------------
 * 以前录音和播放共用一把 s_lock，而且等待用的是 portMAX_DELAY：播放（24 kHz TX）
 * 一堵，录音（16 kHz RX）就跟着无限期等——上行任务就是这样被拖死的。
 * 现在两条通路各有自己的控制锁，锁只护"建/删通道"，绝不跨 i2s_channel_read /
 * i2s_channel_write 持有；所有等待都有上限，到点就返回短读。
 */
#define VOICE_CHUNK_TIMEOUT_MS 200   /* 单次 i2s_channel_read 的上限 */
#define VOICE_REC_MARGIN_MS    100   /* 一帧之外最多再等这么久，到点就返回短读 */
#define VOICE_TX_MARGIN_MS     200   /* 播放写完一帧之外允许的余量 */
#define VOICE_CTRL_TIMEOUT_MS   50   /* 控制锁的有界等待：绝不用 portMAX_DELAY */
#define VOICE_SLOW_LOG_US  1000000   /* "录音/播放慢"日志最多每秒一条，别刷屏 */

static i2s_chan_handle_t s_rx;
static i2s_chan_handle_t s_tx;
static int s_rx_rate;
static int s_tx_rate;
static bool s_rx_enabled;
static bool s_tx_enabled;
static bool s_ready;
/* 录音、播放各一把：谁也不用等谁（共用一个锁会互相拖死） */
static SemaphoreHandle_t s_rx_ctrl;
static SemaphoreHandle_t s_tx_ctrl;
/* 正在通道上读写的人数（受对应控制锁保护）：有人在用就绝不重建通道 */
static volatile int s_rx_users;
static volatile int s_tx_users;
static int64_t s_slow_log_at;      /* 上一条"慢调用"日志的时间戳（us） */

/**
 * 使能一条通道。**必须**在控制锁外面调用：i2s_channel_enable() 要等 DMA 转起来，
 * 耗时不定，拿着锁做会让另一条通路（播放）白等一截。
 */
static esp_err_t chan_start(i2s_chan_handle_t ch, bool *flag, const char *who)
{
    if (ch == NULL || (flag != NULL && *flag)) {
        return ESP_OK;
    }
    const esp_err_t err = i2s_channel_enable(ch);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        /* INVALID_STATE = 已经开过（重复使能）：当成成功，别当错误处理 */
        if (flag != NULL) {
            *flag = true;
        }
        return ESP_OK;
    }
    ESP_LOGE(TAG, "%s使能失败: %s", who, esp_err_to_name(err));
    return err;
}

/* ------------------------------ 麦克风（PDM RX） ------------------------------ */

/** 只建通道、不使能（使能交给 chan_start 在锁外做）。调用方必须持有 s_rx_ctrl。 */
static esp_err_t mic_create(int sample_rate)
{
    if (s_rx != NULL) {
        return ESP_OK;
    }
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(VOICE_RX_PORT, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = 240;
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "麦克风通道创建失败: %s", esp_err_to_name(err));
        return err;
    }

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = PDM_SCK_GPIO,
            .din = PDM_DATA_GPIO,
            .invert_flags = {false},
        },
    };
    /* 16 kHz 用 16 倍降采样（PDM 时钟 2.048 MHz），硬件 PDM2PCM 直接出 16bit PCM */
    pdm_cfg.clk_cfg.dn_sample_mode = I2S_PDM_DSR_16S;

    err = i2s_channel_init_pdm_rx_mode(s_rx, &pdm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "麦克风初始化失败: %s", esp_err_to_name(err));
        i2s_del_channel(s_rx);      /* 失败就别留半成品句柄，否则下次 ensure 会误判已就绪 */
        s_rx = NULL;
        return err;
    }
    s_rx_rate = sample_rate;
    s_rx_enabled = false;           /* 通道建好了但还没开：第一次录音时在锁外使能 */
    ESP_LOGI(TAG, "麦克风通道就绪（PDM SCK=%d DATA=%d, %d Hz, DSR_16S）",
             PDM_SCK_GPIO, PDM_DATA_GPIO, sample_rate);
    return ESP_OK;
}

static void mic_release(void)
{
    if (s_rx == NULL) {
        return;
    }
    if (s_rx_enabled) {
        i2s_channel_disable(s_rx);
        s_rx_enabled = false;
    }
    i2s_del_channel(s_rx);
    s_rx = NULL;
    s_rx_rate = 0;
}

/** 归还录音通道（只减在册计数，立刻返回）。 */
static void rx_release(void)
{
    if (s_rx_ctrl == NULL) {
        return;
    }
    if (xSemaphoreTake(s_rx_ctrl, pdMS_TO_TICKS(VOICE_CTRL_TIMEOUT_MS)) != pdTRUE) {
        return;
    }
    if (s_rx_users > 0) {
        s_rx_users--;
    }
    xSemaphoreGive(s_rx_ctrl);
}

/**
 * 领用录音通道：有界等待，绝不死等。
 *
 * 控制锁只护"建/删通道"和在册计数，**不跨 i2s_channel_read 持有**；使能也放在
 * 锁外做。领不到（50 ms 内抢不到，或有人在读时采样率对不上）就返回错误，
 * 调用方直接丢这一帧——宁可丢帧，也别在这儿把上行任务堵成看门狗。
 */
static esp_err_t rx_acquire(int sample_rate)
{
    if (s_rx_ctrl == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_rx_ctrl, pdMS_TO_TICKS(VOICE_CTRL_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = ESP_OK;
    if (s_rx == NULL || s_rx_rate != sample_rate) {
        if (s_rx_users > 0) {
            err = ESP_ERR_INVALID_STATE;   /* 有人正在读：绝不在他脚下拆通道 */
        } else {
            mic_release();
            err = mic_create(sample_rate);
        }
    }
    if (err == ESP_OK) {
        s_rx_users++;
    }
    xSemaphoreGive(s_rx_ctrl);
    if (err != ESP_OK) {
        return err;
    }
    err = chan_start(s_rx, &s_rx_enabled, "麦克风");   /* 锁外使能 */
    if (err != ESP_OK) {
        rx_release();
    }
    return err;
}

/* ------------------------------ 喇叭（I2S STD TX） ------------------------------ */

/** 只建通道、不使能（使能交给 chan_start 在锁外做）。调用方必须持有 s_tx_ctrl。 */
static esp_err_t spk_create(int sample_rate)
{
    if (s_tx != NULL) {
        return ESP_OK;
    }
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(VOICE_TX_PORT, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 240;
    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "喇叭通道创建失败: %s", esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = 16,
            .ws_pol = false,
            .bit_shift = true,
#if SOC_I2S_HW_VERSION_1
            /* 老 S3 那套字段叫 msb_right */
            .msb_right = false,
#else
            /* S31 是 I2S HW v2：同一含义的字段改成了 left_align/big_endian/bit_order_lsb，
               这里按老代码 msb_right=false 的意图（右对齐、大端、MSB 先出）显式写一遍 */
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
#endif
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = DAC_BCLK_GPIO,
            .ws = DAC_LRCK_GPIO,
            .dout = DAC_DIN_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {false, false, false},
        },
    };

    err = i2s_channel_init_std_mode(s_tx, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "喇叭初始化失败: %s", esp_err_to_name(err));
        i2s_del_channel(s_tx);      /* 失败时别留半成品句柄 */
        s_tx = NULL;
        return err;
    }
    s_tx_rate = sample_rate;
    s_tx_enabled = false;           /* 通道建好了但还没开：第一次播放时在锁外使能 */
    ESP_LOGI(TAG, "喇叭通道就绪（PCM5100A BCK=%d DIN=%d LRCK=%d, %d Hz）",
             DAC_BCLK_GPIO, DAC_DIN_GPIO, DAC_LRCK_GPIO, sample_rate);
    return ESP_OK;
}

static void spk_release(void)
{
    if (s_tx == NULL) {
        return;
    }
    if (s_tx_enabled) {
        i2s_channel_disable(s_tx);
        s_tx_enabled = false;
    }
    i2s_del_channel(s_tx);
    s_tx = NULL;
    s_tx_rate = 0;
}

/** 归还播放通道（只减在册计数，立刻返回）。 */
static void tx_release(void)
{
    if (s_tx_ctrl == NULL) {
        return;
    }
    if (xSemaphoreTake(s_tx_ctrl, pdMS_TO_TICKS(VOICE_CTRL_TIMEOUT_MS)) != pdTRUE) {
        return;
    }
    if (s_tx_users > 0) {
        s_tx_users--;
    }
    xSemaphoreGive(s_tx_ctrl);
}

/** 领用播放通道：和录音分开锁、分开通道，播放再堵也不会拖住上行。 */
static esp_err_t tx_acquire(int sample_rate)
{
    if (s_tx_ctrl == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_tx_ctrl, pdMS_TO_TICKS(VOICE_CTRL_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = ESP_OK;
    if (s_tx == NULL || s_tx_rate != sample_rate) {
        if (s_tx_users > 0) {
            err = ESP_ERR_INVALID_STATE;   /* 有人正在写：绝不在他脚下拆通道 */
        } else {
            spk_release();
            err = spk_create(sample_rate);
        }
    }
    if (err == ESP_OK) {
        s_tx_users++;
    }
    xSemaphoreGive(s_tx_ctrl);
    if (err != ESP_OK) {
        return err;
    }
    err = chan_start(s_tx, &s_tx_enabled, "喇叭");     /* 锁外使能 */
    if (err != ESP_OK) {
        tx_release();
    }
    return err;
}

/* -------------------------------- 对外接口 -------------------------------- */

int voice_audio_init(void)
{
    if (s_ready) {
        return 0;
    }
    if (s_rx_ctrl == NULL) {
        s_rx_ctrl = xSemaphoreCreateMutex();
    }
    if (s_tx_ctrl == NULL) {
        s_tx_ctrl = xSemaphoreCreateMutex();
    }
    if (s_rx_ctrl == NULL || s_tx_ctrl == NULL) {
        ESP_LOGE(TAG, "音频控制锁创建失败");
        return -1;
    }

    /* 建通道都在各自控制锁里做（很短），使能一律在锁外做 */
    esp_err_t err = ESP_OK;
    if (xSemaphoreTake(s_rx_ctrl, pdMS_TO_TICKS(VOICE_CTRL_TIMEOUT_MS)) != pdTRUE) {
        return -1;
    }
    err = mic_create(VOICE_RX_RATE_DEFAULT);
    xSemaphoreGive(s_rx_ctrl);
    if (err != ESP_OK) {
        return -1;
    }

    if (xSemaphoreTake(s_tx_ctrl, pdMS_TO_TICKS(VOICE_CTRL_TIMEOUT_MS)) != pdTRUE) {
        mic_release();
        return -1;
    }
    err = spk_create(VOICE_TX_RATE_DEFAULT);
    xSemaphoreGive(s_tx_ctrl);
    if (err != ESP_OK) {
        mic_release();
        return -1;
    }

    if (chan_start(s_rx, &s_rx_enabled, "麦克风") != ESP_OK ||
            chan_start(s_tx, &s_tx_enabled, "喇叭") != ESP_OK) {
        mic_release();
        spk_release();
        return -1;
    }
    s_ready = true;
    ESP_LOGI(TAG, "音频通路就绪（录音 %d Hz / 播放 %d Hz）", VOICE_RX_RATE_DEFAULT, VOICE_TX_RATE_DEFAULT);
    return 0;
}

int voice_audio_ready(void)
{
    return s_ready ? 1 : 0;
}

void voice_audio_mute(int on)
{
    /* 本板 PCM5100A 既没有静音脚也没有 I2C 控制：空实现，保留符号给上层。 */
    (void)on;
}

/**
 * 录音耗时上报：超过帧长就记一条（每秒最多一条），超过 1 秒单独用
 * 「录音卡住 N ms（看门狗风险）」的措辞——现场就靠这条判断是不是看门狗。
 */
static void note_record_slow(int64_t t0, int frame_ms, int got, int want)
{
    const int64_t now = esp_timer_get_time();
    const int elapsed_ms = (int)((now - t0) / 1000);
    if (elapsed_ms <= frame_ms) {
        return;                    /* 正常：60 ms 的帧 60 ms 内取完，不刷日志 */
    }
    if (now - s_slow_log_at < VOICE_SLOW_LOG_US) {
        return;                    /* 每秒最多一条 */
    }
    s_slow_log_at = now;
    const unsigned free_kb = (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
    if (elapsed_ms > 1000) {
        ESP_LOGE(TAG, "录音卡住 %d ms（看门狗风险）：帧长 %d ms，取到 %d/%d，内部 RAM %u KB",
                 elapsed_ms, frame_ms, got, want, free_kb);
    } else {
        ESP_LOGW(TAG, "录音耗时 %d ms（帧长 %d ms，取到 %d/%d），内部 RAM %u KB",
                 elapsed_ms, frame_ms, got, want, free_kb);
    }
}

int voice_audio_record(int16_t *buf, int max_samples, int sample_rate, int ms)
{
    if (!s_ready || buf == NULL || max_samples <= 0 || ms <= 0) {
        return -1;
    }
    if (sample_rate <= 0) {
        sample_rate = VOICE_RX_RATE_DEFAULT;
    }

    int want = (int)((int64_t)sample_rate * ms / 1000);
    if (want > max_samples) {
        want = max_samples;
    }
    if (want <= 0) {
        return -1;
    }

    /* 整个调用（含领通道、丢旧数据、读一帧）共用一个起点：总耗时有硬上限 */
    const int64_t t0 = esp_timer_get_time();

    /* 领通道：最多等 VOICE_CTRL_TIMEOUT_MS，领不到就返回 -1，让上层丢这一帧。
       绝不在这儿死等——上行任务一停，看门狗就来了。 */
    if (rx_acquire(sample_rate) != ESP_OK) {
        note_record_slow(t0, ms, 0, want);
        return -1;
    }

    /* 丢掉 DMA 里上一次残留的几十毫秒旧数据（0 超时 = 非阻塞，立刻返回） */
    {
        uint8_t trash[256];
        size_t got = 0;
        for (int i = 0; i < 8; i++) {
            if (i2s_channel_read(s_rx, trash, sizeof(trash), &got, 0) != ESP_OK || got == 0) {
                break;
            }
        }
    }

    /* 读一帧：期限 = 帧长 + 100 ms（以前是 帧长 + 2000 ms，单片又等 200 ms，
       麦克风一没数据就能把上行任务按在这儿 2 秒多）。到点就返回短读，
       短读由上层按"录音丢帧"统计，比卡死好。 */
    const size_t want_bytes = (size_t)want * sizeof(int16_t);
    size_t total = 0;
    const int64_t deadline = t0 + (int64_t)(ms + VOICE_REC_MARGIN_MS) * 1000;
    while (total < want_bytes) {
        const int64_t left_us = deadline - esp_timer_get_time();
        if (left_us <= 0) {
            break;                  /* 到点收手：宁可短读，也不堵住任务 */
        }
        int chunk_ms = (int)(left_us / 1000);
        if (chunk_ms > VOICE_CHUNK_TIMEOUT_MS) {
            chunk_ms = VOICE_CHUNK_TIMEOUT_MS;
        }
        if (chunk_ms < 1) {
            chunk_ms = 1;
        }
        size_t got = 0;
        const size_t left = want_bytes - total;
        /* 注意：IDF 6.x 的 i2s_channel_read() 最后一个参数是**毫秒**（uint32_t timeout_ms），
           不是 tick —— 这里直接给毫秒，别再套 pdMS_TO_TICKS()。 */
        const esp_err_t err = i2s_channel_read(s_rx, (uint8_t *)buf + total, left, &got,
                                               (uint32_t)chunk_ms);
        if (got > 0) {
            total += got;
        }
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "录音失败: %s", esp_err_to_name(err));
            break;
        }
    }
    rx_release();                   /* 通道还回去：锁没有跨上面任何一次读 */

    const int ret = (int)(total / sizeof(int16_t));
    note_record_slow(t0, ms, ret, want);
    return ret;
}

int voice_audio_play(const int16_t *buf, int samples, int sample_rate)
{
    if (!s_ready || buf == NULL || samples <= 0) {
        return -1;
    }
    if (sample_rate <= 0) {
        sample_rate = VOICE_TX_RATE_DEFAULT;
    }

    const int64_t t0 = esp_timer_get_time();
    if (tx_acquire(sample_rate) != ESP_OK) {
        return -1;
    }

    int ret = -1;
    int play_ms = (int)((int64_t)samples * 1000 / sample_rate);
    if (play_ms <= 0) {
        play_ms = 1;
    }
    voice_audio_mute(0);            /* 本板无静音脚，仅保留调用点 */
    const size_t want = (size_t)samples * sizeof(int16_t);
    size_t written = 0;
    /* 写超时 = 本帧播放时长 + 200 ms（以前写死 +2000 ms：TX 一堵，等锁的人全跟着堵）。
       同上：i2s_channel_write() 这个参数是毫秒，不是 tick。 */
    const esp_err_t err = i2s_channel_write(s_tx, buf, want, &written,
                                            (uint32_t)(play_ms + VOICE_TX_MARGIN_MS));
    voice_audio_mute(1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "播放失败: %s（本帧 %d 采样 / %d ms）", esp_err_to_name(err), samples, play_ms);
    } else {
        ret = (int)(written / sizeof(int16_t));
        if (written < want) {
            ESP_LOGW(TAG, "播放只写入 %d/%d 采样点", ret, samples);
        }
    }
    tx_release();

    /* 播放慢也记一条（和录音共用每秒一条的限流）：TX 堵会听得见，也能看出是下行在拖 */
    const int64_t now = esp_timer_get_time();
    const int elapsed_ms = (int)((now - t0) / 1000);
    if (elapsed_ms > play_ms + VOICE_TX_MARGIN_MS / 2 && (now - s_slow_log_at) >= VOICE_SLOW_LOG_US) {
        s_slow_log_at = now;
        ESP_LOGW(TAG, "播放耗时 %d ms（本帧 %d ms，写入 %d/%d），内部 RAM %u KB",
                 elapsed_ms, play_ms, (int)(written / sizeof(int16_t)), samples,
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    }
    return ret;
}

int voice_audio_selftest(int *peak, int *rms)
{
    /* 界面按钮和 MQTT 的 audio_test 可能同时点：只允许一路自检，避免并发初始化 I2S */
    static volatile bool s_selftest_running;
    if (s_selftest_running) {
        ESP_LOGW(TAG, "自检已在跑，忽略本次请求");
        return -1;
    }
    s_selftest_running = true;

    const int seconds_ms = 1500;
    const int rate = VOICE_RX_RATE_DEFAULT;
    const int max_samples = rate * seconds_ms / 1000;
    int16_t *buf = heap_caps_malloc((size_t)max_samples * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        ESP_LOGE(TAG, "自检缓冲分配失败");
        s_selftest_running = false;
        return -1;
    }
    if (voice_audio_init() != 0) {
        free(buf);
        s_selftest_running = false;
        return -1;
    }

    ESP_LOGI(TAG, "自检：录音 %d ms …", seconds_ms);
    const int n = voice_audio_record(buf, max_samples, rate, seconds_ms);
    if (n <= 0) {
        ESP_LOGE(TAG, "自检失败：没有录到数据");
        free(buf);
        s_selftest_running = false;
        return -1;
    }

    int32_t pk = 0;
    int64_t sum = 0;
    for (int i = 0; i < n; i++) {
        const int32_t v = buf[i] < 0 ? -buf[i] : buf[i];
        if (v > pk) {
            pk = v;
        }
        sum += (int64_t)buf[i] * buf[i];
    }
    const int r = (int)(sum / n);
    if (peak) {
        *peak = (int)pk;
    }
    if (rms) {
        *rms = r;
    }
    ESP_LOGI(TAG, "自检结果：%d 采样，峰值 %d，均方 %d %s", n, (int)pk, r,
             pk < 200 ? "（几乎没有声音，检查麦克风接线）" : "");

    ESP_LOGI(TAG, "自检：回放刚才的录音（线路输出，插耳机听）…");
    const int played = voice_audio_play(buf, n, rate);
    ESP_LOGI(TAG, "自检：回放写入 %d 采样点", played);
    free(buf);
    s_selftest_running = false;
    return 0;
}
