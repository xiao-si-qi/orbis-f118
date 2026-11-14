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

/* 每次 read/write 的切片，避免一次阻塞太久 */
#define VOICE_CHUNK_TIMEOUT_MS 200

static i2s_chan_handle_t s_rx;
static i2s_chan_handle_t s_tx;
static int s_rx_rate;
static int s_tx_rate;
static bool s_ready;
static SemaphoreHandle_t s_lock;   /* 串行化 record/play，避免两个任务抢同一条通道 */

/* ------------------------------ 麦克风（PDM RX） ------------------------------ */

static esp_err_t mic_init(int sample_rate)
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
    err = i2s_channel_enable(s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "麦克风使能失败: %s", esp_err_to_name(err));
        i2s_del_channel(s_rx);
        s_rx = NULL;
        return err;
    }
    s_rx_rate = sample_rate;
    ESP_LOGI(TAG, "麦克风就绪（PDM SCK=%d DATA=%d, %d Hz, DSR_16S）",
             PDM_SCK_GPIO, PDM_DATA_GPIO, sample_rate);
    return ESP_OK;
}

static void mic_release(void)
{
    if (s_rx == NULL) {
        return;
    }
    i2s_channel_disable(s_rx);
    i2s_del_channel(s_rx);
    s_rx = NULL;
    s_rx_rate = 0;
}

/** 保证录音通道按 rate 就绪（采样率变了就重建）。 */
static esp_err_t mic_ensure(int sample_rate)
{
    if (s_rx != NULL && s_rx_rate == sample_rate) {
        return ESP_OK;
    }
    mic_release();
    return mic_init(sample_rate);
}

/* ------------------------------ 喇叭（I2S STD TX） ------------------------------ */

static esp_err_t spk_init(int sample_rate)
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
    err = i2s_channel_enable(s_tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "喇叭使能失败: %s", esp_err_to_name(err));
        i2s_del_channel(s_tx);
        s_tx = NULL;
        return err;
    }
    s_tx_rate = sample_rate;
    ESP_LOGI(TAG, "喇叭就绪（PCM5100A BCK=%d DIN=%d LRCK=%d, %d Hz）",
             DAC_BCLK_GPIO, DAC_DIN_GPIO, DAC_LRCK_GPIO, sample_rate);
    return ESP_OK;
}

static void spk_release(void)
{
    if (s_tx == NULL) {
        return;
    }
    i2s_channel_disable(s_tx);
    i2s_del_channel(s_tx);
    s_tx = NULL;
    s_tx_rate = 0;
}

/** 保证播放通道按 rate 就绪（采样率变了就重建）。 */
static esp_err_t spk_ensure(int sample_rate)
{
    if (s_tx != NULL && s_tx_rate == sample_rate) {
        return ESP_OK;
    }
    spk_release();
    return spk_init(sample_rate);
}

/* -------------------------------- 对外接口 -------------------------------- */

int voice_audio_init(void)
{
    if (s_ready) {
        return 0;
    }
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            ESP_LOGE(TAG, "音频互斥锁创建失败");
            return -1;
        }
    }
    if (mic_init(VOICE_RX_RATE_DEFAULT) != ESP_OK) {
        return -1;
    }
    if (spk_init(VOICE_TX_RATE_DEFAULT) != ESP_OK) {
        mic_release();
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

int voice_audio_record(int16_t *buf, int max_samples, int sample_rate, int ms)
{
    if (!s_ready || buf == NULL || max_samples <= 0 || ms <= 0) {
        return -1;
    }
    if (sample_rate <= 0) {
        sample_rate = VOICE_RX_RATE_DEFAULT;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int ret = -1;
    do {
        if (mic_ensure(sample_rate) != ESP_OK) {
            break;
        }

        int want = (int)((int64_t)sample_rate * ms / 1000);
        if (want > max_samples) {
            want = max_samples;
        }
        if (want <= 0) {
            break;
        }

        /* 丢掉 DMA 里上一次残留的几十毫秒旧数据 */
        {
            uint8_t trash[256];
            size_t got = 0;
            for (int i = 0; i < 8; i++) {
                if (i2s_channel_read(s_rx, trash, sizeof(trash), &got, 0) != ESP_OK || got == 0) {
                    break;
                }
            }
        }

        const size_t want_bytes = (size_t)want * sizeof(int16_t);
        size_t total = 0;
        const int64_t deadline = esp_timer_get_time() + (int64_t)(ms + 2000) * 1000;
        while (total < want_bytes && esp_timer_get_time() < deadline) {
            size_t got = 0;
            const size_t left = want_bytes - total;
            const esp_err_t err = i2s_channel_read(s_rx, (uint8_t *)buf + total, left, &got,
                                                   pdMS_TO_TICKS(VOICE_CHUNK_TIMEOUT_MS));
            if (got > 0) {
                total += got;
            }
            if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "录音失败: %s", esp_err_to_name(err));
                break;
            }
        }
        ret = (int)(total / sizeof(int16_t));
        if (ret < want) {
            ESP_LOGW(TAG, "录音只取到 %d/%d 采样点", ret, want);
        }
    } while (0);
    xSemaphoreGive(s_lock);
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

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int ret = -1;
    if (spk_ensure(sample_rate) == ESP_OK) {
        voice_audio_mute(0);   /* 本板无静音脚，仅保留调用点 */
        const size_t want = (size_t)samples * sizeof(int16_t);
        size_t written = 0;
        const TickType_t timeout = pdMS_TO_TICKS(samples * 1000 / sample_rate + 2000);
        const esp_err_t err = i2s_channel_write(s_tx, buf, want, &written, timeout);
        voice_audio_mute(1);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "播放失败: %s", esp_err_to_name(err));
        } else {
            ret = (int)(written / sizeof(int16_t));
            if (written < want) {
                ESP_LOGW(TAG, "播放只写入 %d/%d 采样点", ret, samples);
            }
        }
    }
    xSemaphoreGive(s_lock);
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
