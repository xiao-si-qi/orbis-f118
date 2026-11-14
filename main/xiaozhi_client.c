/**
 * @file xiaozhi_client.c
 * @brief 小智 AI 服务端接入：OTA 发现 + WebSocket hello + Opus 音频双向流。
 *
 * 线程约定：
 *   - 发现/连接在独立任务（xz_task）里做，不阻塞 LVGL 和 MQTT；
 *   - 音频流另起两个任务：xz_send（麦克风→opus→二进制帧）和 xz_play（二进制帧→opus→喇叭），
 *     都 >= 8192 字节栈，且不碰 LVGL（状态只写静态字符串，界面自己轮询）；
 *   - WebSocket 回调里只做"拷一帧进播放队列"这种短活，解码和播放都在 xz_play 里做。
 *
 * 内存约定：
 *   - 发现用一次 HTTPS（esp_http_client），完成后立刻释放；响应缓冲放 PSRAM；
 *   - 码流缓冲（PCM、编码输出、解码输出、播放队列）全部 heap_caps_malloc(MALLOC_CAP_SPIRAM)，
 *     任务栈上只放几个指针和几十字节的结构体。
 *
 * 协议（与 xiaozhi-esp32/main/protocols/websocket_protocol.cc 对齐）：
 *   - 文本帧：JSON 控制消息（hello / listen / tts / stt / llm / abort）；
 *   - 二进制帧：16 字节头 + opus 负载，网络字节序（大端）：
 *       uint16 version(=2) | uint16 type(0=opus,1=json) | uint32 reserved
 *       uint32 timestamp(ms) | uint32 payload_size | payload[]
 *   - 上行 16 kHz 单声道 60 ms（960 采样）；下行 24 kHz 单声道 60 ms（1440 采样）；
 *   - 半双工：说话（上行）和放音（下行）互斥，绝不在放 TTS 时开麦。
 *
 * 本板（ESP32-S31 / JC3636F118）与旧板的差异：
 *   - 默认板型/型号改成 jc3636f118；
 *   - sdkconfig 里 CONFIG_MBEDTLS_HAVE_TIME_DATE 未打开（证书有效期不校验），
 *     所以不用像旧板那样先等 SNTP 同步时间，直接连即可。
 *     TODO(port): 哪天打开证书有效期校验，要先在 net.c 里起 SNTP。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"

#include "esp_opus_dec.h"
#include "esp_opus_enc.h"

#include "bg_image.h"
#include "voice_audio.h"
#include "xiaozhi_client.h"
#include "net.h"

static const char *TAG = "xiaozhi";

#define XZ_ENDPOINT_MAX 128
#define XZ_URL_MAX      192
#define XZ_TOKEN_MAX    96
#define XZ_STATUS_MAX   72
#define XZ_NOTE_MAX     40
#define XZ_TEXT_MAX     96      /* 界面显示的"最近一句"上限（字节）*/
#define XZ_DISCOVER_MAX (24 * 1024)
#define XZ_WS_BUF       768     /* 文本帧累积缓冲：hello/stt/llm 的 JSON 可能上百字节 */

/* ---- 音频流参数（与 xiaozhi-esp32 audio_service.h / 实测服务端一致）---- */
#define XZ_IN_RATE       16000
#define XZ_OUT_RATE      24000
#define XZ_FRAME_MS      60
#define XZ_IN_SAMPLES    (XZ_IN_RATE / 1000 * XZ_FRAME_MS)    /* 960 */
#define XZ_OUT_SAMPLES   (XZ_OUT_RATE / 1000 * XZ_FRAME_MS)   /* 1440 */
#define XZ_HDR_SIZE      16                                   /* BinaryProtocol2 */
#define XZ_PLAY_SLOTS    8                                    /* 播放队列槽数（约 480ms）*/
#define XZ_PLAY_MAX      640                                  /* 单帧 opus 上限（60ms 远小于此）*/
#define XZ_BIN_MAX       1024                                 /* 二进制帧累积上限 */
#define XZ_TASK_STACK 24576   /* Opus CELT 编码器很吃栈；实际分配到 PSRAM */
#define XZ_PLAY_STACK 16384                                 /* 编/解码任务栈 */
#define XZ_WS_TASK_STACK 8192                                 /* WebSocket 任务栈（TLS 在这条任务上）*/
#define XZ_PROTO_VER     2                                    /* 声明的二进制协议版本 */

/* 播放队列槽。整块放 PSRAM，任务栈上只放索引。 */
typedef struct {
    uint16_t len;
    uint8_t  data[XZ_PLAY_MAX];
} xz_slot_t;

static bool s_enabled;
static char s_endpoint[XZ_ENDPOINT_MAX];
static char s_ws_url[XZ_URL_MAX];
static char s_token[XZ_TOKEN_MAX];
static char s_client_id[40];
static char s_device_id[24];
static char s_status[XZ_STATUS_MAX];
static char s_note[XZ_NOTE_MAX];
static char s_activation[16];      /* 官方平台激活码（显示给用户）*/
static char s_session[40];

static volatile bool s_ready;
static volatile int s_gen;
static volatile bool s_task_alive;
static esp_websocket_client_handle_t s_ws;

/* ---- 给界面看的高阶状态（不参与音频流程，只读派生）---- */
static volatile bool s_error;          /* 发现/连接出错（界面画 ERROR 脸）*/
static volatile bool s_thinking;       /* 上行停了、下行还没开始 */
static volatile int64_t s_thinking_at; /* thinking 起点（8 秒兜底，避免一直转圈）*/
static volatile int s_listen_req;      /* 界面手动请求：1=start 2=stop，上行任务消费 */

/* 最近一次 stt 文字：WS 回调写、LVGL 线程读，用临界区护一下，
   读侧拷进快照再返回，界面不会读到半句中文。 */
static char s_last_text[XZ_TEXT_MAX + 1];
static char s_text_snap[XZ_TEXT_MAX + 1];
static portMUX_TYPE s_text_mux = portMUX_INITIALIZER_UNLOCKED;

/* hello 可能被分片，这里做个小累积缓冲 */
static char s_rx[XZ_WS_BUF];
static size_t s_rx_len;

/* 二进制帧累积（分片时用），同样放 PSRAM */
static uint8_t *s_bin;
static size_t s_bin_len;
static bool s_first_frame_logged;

/* ---- 音频流状态 ---- */
enum { XZ_ST_IDLE = 0, XZ_ST_LISTEN, XZ_ST_THINK, XZ_ST_SPEAK };

static volatile bool s_uplink;          /* 允许上行（麦克风）*/
static volatile bool s_pending_listen;  /* 连上后要发一次 listen start */
static volatile bool s_tts_active;      /* 服务端说 tts 还没结束 */
static volatile bool s_stream_stop = true;
static volatile bool s_sender_alive;
static volatile bool s_player_alive;
static volatile int  s_state;
static volatile int  s_wire_ver = XZ_PROTO_VER;  /* 实际在用的二进制协议版本 */
static volatile unsigned s_play_gen;             /* 打断计数：变了就让播放任务重置解码器 */

static SemaphoreHandle_t s_stream_lock;   /* 起停流任务用（不跨阻塞操作持有）*/
static SemaphoreHandle_t s_send_lock;     /* 发送 WS 与销毁 WS 互斥 */
static SemaphoreHandle_t s_q_lock;        /* 播放队列 */
static xz_slot_t *s_q;
static int s_q_head, s_q_tail, s_q_count;

static void handle_text_message(const char *json);

/* ------------------------------ 小工具 ------------------------------ */

static void set_status(const char *text)
{
    snprintf(s_status, sizeof(s_status), "%s", text);
    ESP_LOGI(TAG, "%s", text);
}

/** 音频缓冲一律优先 PSRAM；实在没有才退回内部 RAM。 */
static void *xz_psram_alloc(size_t n)
{
    void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == NULL) {
        p = malloc(n);
    }
    if (p != NULL) {
        memset(p, 0, n);
    }
    return p;
}

/** 状态文字只在这四个里挑（界面直接显示）。 */
static void xz_set_state(int st)
{
    if (s_state == st) {
        return;
    }
    s_state = st;
    switch (st) {
    case XZ_ST_LISTEN: set_status("正在聆听"); break;
    case XZ_ST_THINK:  set_status("正在说话"); break;
    case XZ_ST_SPEAK:  set_status("播放回复"); break;
    default:           set_status("已连接");   break;
    }
}

/** 上行停了、回复还没来：界面显示"思考中"，一直不来就 8 秒后自己收回。 */
static void xz_think_start(void)
{
    s_thinking_at = esp_timer_get_time();
    s_thinking = true;
}

static void xz_think_clear(void)
{
    s_thinking = false;
}

/** UTF-8 安全截断：最多 cap-1 字节，不把一个汉字劈成半个（界面会画出乱码）。 */
static void utf8_copy(char *dst, size_t cap, const char *src)
{
    if (dst == NULL || cap == 0) {
        return;
    }
    dst[0] = '\0';
    if (src == NULL) {
        return;
    }
    size_t n = strlen(src);
    if (n > cap - 1) {
        n = cap - 1;
        while (n > 0 && ((unsigned char)src[n] & 0xC0) == 0x80) {
            n--;                       /* 退到字符首字节 */
        }
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
}

/** 记下最近一次识别到的文字（96 字节封顶，界面拿去显示）。 */
static void set_last_text(const char *text)
{
    if (text == NULL) {
        return;
    }
    taskENTER_CRITICAL(&s_text_mux);
    utf8_copy(s_last_text, sizeof(s_last_text), text);
    taskEXIT_CRITICAL(&s_text_mux);
}

/** 用 MAC 生成稳定的、UUID 形态的 Client-Id（小智服务端要求合法 UUID）。 */
static void init_ids(void)
{
    if (s_client_id[0] != '\0') {
        return;
    }
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_device_id, sizeof(s_device_id), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(s_client_id, sizeof(s_client_id),
             "%02x%02x%02x%02x-%02x%02x-4%02x%01x-8%02x%01x-%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
             mac[0], mac[1] >> 4, mac[2], mac[3] >> 4,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* --------------------------- 1) OTA 发现 --------------------------- */

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
} http_buf_t;

static esp_err_t discover_event(esp_http_client_event_t *ev)
{
    http_buf_t *ctx = (http_buf_t *)ev->user_data;
    if (ev->event_id == HTTP_EVENT_ON_DATA && ctx != NULL && ev->data_len > 0) {
        const size_t n = (size_t)ev->data_len;
        if (ctx->len + n + 1 < ctx->cap) {
            memcpy(ctx->buf + ctx->len, ev->data, n);
            ctx->len += n;
            ctx->buf[ctx->len] = '\0';
        }
    }
    return ESP_OK;
}

/** 24 KB 的响应缓冲优先放 PSRAM，别挤内部 RAM（TLS 还要用）。 */
static char *discover_buf_alloc(void)
{
    char *buf = heap_caps_malloc(XZ_DISCOVER_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        buf = malloc(XZ_DISCOVER_MAX);
    }
    if (buf != NULL) {
        memset(buf, 0, XZ_DISCOVER_MAX);
    }
    return buf;
}

static bool discover_ws_url(void)
{
    char *buf = discover_buf_alloc();
    if (buf == NULL) {
        set_status("内存不足");
        return false;
    }

    char body[320];
    snprintf(body, sizeof(body),
             "{\"version\":2,\"mac_address\":\"%s\",\"uuid\":\"%s\","
             "\"application\":{\"name\":\"orbis-f118\",\"version\":\"1\"},"
             "\"board\":{\"type\":\"jc3636f118\",\"name\":\"orbis\"}}",
             s_device_id, s_client_id);

    http_buf_t ctx = {.buf = buf, .len = 0, .cap = XZ_DISCOVER_MAX};
    esp_http_client_config_t cfg = {
        .url = s_endpoint,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 12000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = discover_event,
        .user_data = &ctx,
        .buffer_size = 4096,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        free(buf);
        set_status("HTTP 初始化失败");
        return false;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Device-Id", s_device_id);
    esp_http_client_set_header(client, "Client-Id", s_client_id);
    esp_http_client_set_header(client, "Activation-Version", "1");
    esp_http_client_set_header(client, "User-Agent", "orbis-f118/1");
    esp_http_client_set_post_field(client, body, (int)strlen(body));

    const esp_err_t err = esp_http_client_perform(client);
    const int http_status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || http_status >= 400 || ctx.len == 0) {
        ESP_LOGW(TAG, "发现失败: %s HTTP %d %d 字节", esp_err_to_name(err), http_status, (int)ctx.len);
        snprintf(s_note, sizeof(s_note), "发现失败 HTTP %d", http_status);
        free(buf);
        set_status("服务端不可达");
        return false;
    }

    bool ok = false;
    cJSON *root = cJSON_Parse(buf);
    if (root != NULL) {
        /* 首次连接官方平台时不会直接给 websocket，而是给激活码：
           {"activation":{"code":"123456","message":"...","challenge":"..."}}
           设备要把 code 显示出来，用户在 xiaozhi.me 控制台绑定后才会下发 websocket。 */
        const cJSON *act = cJSON_GetObjectItemCaseSensitive(root, "activation");
        if (cJSON_IsObject(act)) {
            const cJSON *acode = cJSON_GetObjectItemCaseSensitive(act, "code");
            const cJSON *amsg = cJSON_GetObjectItemCaseSensitive(act, "message");
            const char *code = cJSON_IsString(acode) ? acode->valuestring : NULL;
            if (code != NULL && code[0] != '\0') {
                snprintf(s_activation, sizeof(s_activation), "%s", code);
                ESP_LOGW(TAG, "需要激活，激活码 %s", s_activation);
                if (cJSON_IsString(amsg)) {
                    ESP_LOGW(TAG, "服务端提示: %s", amsg->valuestring);
                }
                {
                    char hint[32];
                    snprintf(hint, sizeof(hint), "待激活 %s", s_activation);
                    set_status(hint);
                }
                cJSON_Delete(root);
                free(buf);
                return false;      /* 交给重试循环：绑定后再取 websocket */
            }
        }

        const cJSON *ws = cJSON_GetObjectItemCaseSensitive(root, "websocket");
        const cJSON *url = cJSON_IsObject(ws) ? cJSON_GetObjectItemCaseSensitive(ws, "url") : NULL;
        if (cJSON_IsString(url) && url->valuestring[0] != '\0') {
            snprintf(s_ws_url, sizeof(s_ws_url), "%s", url->valuestring);
            const cJSON *tok = cJSON_GetObjectItemCaseSensitive(ws, "token");
            if (cJSON_IsString(tok)) {
                snprintf(s_token, sizeof(s_token), "%s", tok->valuestring);
            }
            ok = true;
        }
        snprintf(s_note, sizeof(s_note), "%s",
                 strstr(s_ws_url, "api.tenclass.net") ? "官方服务端" : "自建服务端");
        cJSON_Delete(root);
    }
    free(buf);

    if (!ok) {
        set_status("返回里没有 websocket 地址");
        return false;
    }
    ESP_LOGI(TAG, "发现成功: %s (%s)", s_ws_url, s_note);
    return true;
}

/* --------------------------- 2) 播放队列 --------------------------- */

/** 一次性把队列/锁建好（PSRAM，5 KB 左右，之后不再释放）。 */
static bool ensure_stream_buffers(void)
{
    if (s_q_lock == NULL) {
        s_q_lock = xSemaphoreCreateMutex();
    }
    if (s_send_lock == NULL) {
        s_send_lock = xSemaphoreCreateMutex();
    }
    if (s_stream_lock == NULL) {
        s_stream_lock = xSemaphoreCreateMutex();
    }
    if (s_q == NULL) {
        s_q = xz_psram_alloc(sizeof(xz_slot_t) * XZ_PLAY_SLOTS);
    }
    if (s_q == NULL || s_q_lock == NULL || s_send_lock == NULL || s_stream_lock == NULL) {
        ESP_LOGE(TAG, "音频流缓冲/锁创建失败（内存不足）");
        return false;
    }
    return true;
}

static void play_flush(bool reset_decoder)
{
    if (s_q_lock != NULL && xSemaphoreTake(s_q_lock, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_q_head = s_q_tail = s_q_count = 0;
        xSemaphoreGive(s_q_lock);
    }
    if (reset_decoder) {
        s_play_gen++;          /* 播放任务看到代数变了会 reset 解码器 */
    }
}

/** 从 WS 回调里调用：只做一次 memcpy，绝不解码/播放。 */
static void play_push(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0 || s_q == NULL || s_q_lock == NULL) {
        return;
    }
    if (len > XZ_PLAY_MAX) {
        return;
    }
    if (xSemaphoreTake(s_q_lock, 0) != pdTRUE) {
        return;                /* 抢不到就算了，实时音频丢一帧无所谓 */
    }
    if (s_q_count >= XZ_PLAY_SLOTS) {
        /* 队列满了丢最旧的：宁可少半秒，也别让延迟越滚越大 */
        s_q_tail = (s_q_tail + 1) % XZ_PLAY_SLOTS;
        s_q_count--;
    }
    xz_slot_t *slot = &s_q[s_q_head];
    slot->len = (uint16_t)len;
    memcpy(slot->data, data, len);
    s_q_head = (s_q_head + 1) % XZ_PLAY_SLOTS;
    s_q_count++;
    xSemaphoreGive(s_q_lock);
}

static bool play_pop(uint8_t *dst, size_t cap, uint16_t *out_len)
{
    if (dst == NULL || s_q == NULL || s_q_lock == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_q_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        return false;
    }
    bool ok = false;
    if (s_q_count > 0) {
        const xz_slot_t *slot = &s_q[s_q_tail];
        if (slot->len <= cap) {
            memcpy(dst, slot->data, slot->len);
            *out_len = slot->len;
            ok = true;
        }
        s_q_tail = (s_q_tail + 1) % XZ_PLAY_SLOTS;
        s_q_count--;
    }
    xSemaphoreGive(s_q_lock);
    return ok;
}

/* ------------------------- 3) 帧头（网络字节序） ------------------------- */

static uint16_t xz_rd16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t xz_rd32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static void xz_wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

static void xz_wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)((v >> 16) & 0xFF);
    p[2] = (uint8_t)((v >> 8) & 0xFF);
    p[3] = (uint8_t)(v & 0xFF);
}

/** BinaryProtocol2：version | type | reserved | timestamp | payload_size | payload */
static void xz_write_header(uint8_t *p, uint16_t version, uint16_t type, uint32_t ts, uint32_t size)
{
    xz_wr16(p + 0, version);
    xz_wr16(p + 2, type);
    xz_wr32(p + 4, 0);
    xz_wr32(p + 8, ts);
    xz_wr32(p + 12, size);
}

/* ---------------------------- 4) 流任务 ---------------------------- */

static void xz_sender_task(void *arg);
static void xz_player_task(void *arg);

/** 发送互斥：teardown 会先拿这把锁再摘 s_ws，保证不会"边发边销毁"。 */
static int stream_send_bin(const uint8_t *data, size_t len)
{
    if (s_send_lock == NULL) {
        return -1;
    }
    if (xSemaphoreTake(s_send_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return -1;
    }
    int sent = -1;
    esp_websocket_client_handle_t ws = s_ws;
    if (ws != NULL && s_ready) {
        sent = esp_websocket_client_send_bin(ws, (const char *)data, (int)len, pdMS_TO_TICKS(500));
    }
    xSemaphoreGive(s_send_lock);
    return sent;
}

static int stream_send_text(const char *json)
{
    if (s_send_lock == NULL) {
        return -1;
    }
    if (xSemaphoreTake(s_send_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return -1;
    }
    int sent = -1;
    esp_websocket_client_handle_t ws = s_ws;
    if (ws != NULL && s_ready) {
        sent = esp_websocket_client_send_text(ws, json, (int)strlen(json), pdMS_TO_TICKS(500));
    }
    xSemaphoreGive(s_send_lock);
    return sent;
}

/**
 * 发一条 listen start/stop。
 * start 带 mode=auto（和原来的自动流程一字不差，服务端自己 VAD 收尾）；
 * stop 不带 mode：手动关麦时服务端只要一个 state。
 * 只在音频任务里调用（会阻塞等网络），界面线程绝不能直接调。
 */
static bool send_listen(const char *state, bool with_mode)
{
    char msg[144];
    const char *mode = with_mode ? ",\"mode\":\"auto\"" : "";
    if (s_session[0] != '\0') {
        snprintf(msg, sizeof(msg),
                 "{\"session_id\":\"%.32s\",\"type\":\"listen\",\"state\":\"%s\"%s}",
                 s_session, state, mode);
    } else {
        snprintf(msg, sizeof(msg),
                 "{\"type\":\"listen\",\"state\":\"%s\"%s}", state, mode);
    }
    return stream_send_text(msg) > 0;
}

/** 每帧判一次"是否该退出"：和 stream_start 在同一把锁里，避免抢跑。 */
static bool stream_keep_running(volatile bool *alive)
{
    if (s_stream_lock == NULL) {
        return false;
    }
    xSemaphoreTake(s_stream_lock, portMAX_DELAY);
    const bool stop = s_stream_stop;
    if (stop && alive != NULL) {
        *alive = false;
    }
    xSemaphoreGive(s_stream_lock);
    return !stop;
}

/** 服务端 hello 到了：起流任务（重复调用安全）。 */
static void stream_start(void)
{
    if (!ensure_stream_buffers()) {
        return;
    }
    xSemaphoreTake(s_stream_lock, portMAX_DELAY);
    s_stream_stop = false;
    s_uplink = false;
    s_pending_listen = true;
    s_tts_active = false;
    if (!s_sender_alive) {
        s_sender_alive = true;      /* 先占位，任务里不再重复置位 */
        if (xTaskCreateWithCaps(xz_sender_task, "xz_send", XZ_TASK_STACK, NULL, 5, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
            s_sender_alive = false;
            ESP_LOGE(TAG, "上行任务创建失败");
        }
    }
    if (!s_player_alive) {
        s_player_alive = true;
        if (xTaskCreateWithCaps(xz_player_task, "xz_play", XZ_PLAY_STACK, NULL, 5, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
            s_player_alive = false;
            ESP_LOGE(TAG, "下行任务创建失败");
        }
    }
    xSemaphoreGive(s_stream_lock);
    ESP_LOGI(TAG, "音频流已启动（上行 %d Hz/%d ms，下行 %d Hz/%d ms）",
             XZ_IN_RATE, XZ_FRAME_MS, XZ_OUT_RATE, XZ_FRAME_MS);
}

/** 只置标志，不等待（可以在 WS 回调/LVGL 线程里调用）。 */
static void stream_set_stop(void)
{
    if (s_stream_lock != NULL) {
        xSemaphoreTake(s_stream_lock, portMAX_DELAY);
        s_stream_stop = true;
        xSemaphoreGive(s_stream_lock);
    } else {
        s_stream_stop = true;
    }
    s_uplink = false;
    s_pending_listen = false;
    s_tts_active = false;
    s_listen_req = 0;           /* 连接断了：丢掉界面还没被消费的开/关麦请求 */
    xz_think_clear();
    s_state = XZ_ST_IDLE;       /* 退出路径不刷状态文字，避免覆盖"未启用" */
}

/** 停流并等任务退出（只在 xz_task 里调用；LVGL 线程不要调）。 */
static void stream_stop_wait(int ms)
{
    stream_set_stop();
    for (int i = 0; i < ms / 20 && (s_sender_alive || s_player_alive); i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_sender_alive || s_player_alive) {
        ESP_LOGW(TAG, "音频任务还在收尾（上行 %d 下行 %d）", (int)s_sender_alive, (int)s_player_alive);
    }
}

/* ---- 上行：麦克风 → opus → 二进制帧 ---- */

static void xz_sender_task(void *arg)
{
    (void)arg;
    void *enc = NULL;
    int16_t *pcm = NULL;
    uint8_t *frame = NULL;
    int in_size = 0;
    int out_size = 0;

    /* 与 xiaozhi-esp32 的 AS_OPUS_ENC_CONFIG() 保持一致：16k / 单声道 / 60ms / 码率自适应 */
    esp_opus_enc_config_t ecfg = {
        .sample_rate      = ESP_AUDIO_SAMPLE_RATE_16K,
        .channel          = ESP_AUDIO_MONO,
        .bits_per_sample  = ESP_AUDIO_BIT16,
        .bitrate          = ESP_OPUS_BITRATE_AUTO,
        .frame_duration   = ESP_OPUS_ENC_FRAME_DURATION_60_MS,
        .application_mode = ESP_OPUS_ENC_APPLICATION_AUDIO,
        .complexity       = 0,
        .enable_fec       = false,
        .enable_dtx       = true,
        .enable_vbr       = true,
    };

    const esp_audio_err_t oerr = esp_opus_enc_open(&ecfg, sizeof(ecfg), &enc);
    if (oerr != ESP_AUDIO_ERR_OK || enc == NULL) {
        enc = NULL;
        ESP_LOGE(TAG, "Opus 编码器打开失败（%d），上行不可用", (int)oerr);
        goto out;
    }
    if (esp_opus_enc_get_frame_size(enc, &in_size, &out_size) != ESP_AUDIO_ERR_OK ||
            in_size <= 0 || out_size <= 0) {
        ESP_LOGE(TAG, "Opus 编码器查询帧长失败");
        goto out;
    }
    ESP_LOGI(TAG, "Opus 编码器就绪：16k 单声道 60ms（in=%d 字节 / out=%d 字节，内部 RAM %u KB / PSRAM %u KB）",
             in_size, out_size,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    pcm = xz_psram_alloc(XZ_IN_SAMPLES * sizeof(int16_t));
    frame = xz_psram_alloc(XZ_HDR_SIZE + (size_t)out_size + 64);
    if (pcm == NULL || frame == NULL) {
        ESP_LOGE(TAG, "上行缓冲分配失败");
        goto out;
    }

    uint32_t ts_ms = 0;
    uint32_t stat_frames = 0;
    uint32_t stat_bytes = 0;
    uint32_t stat_err = 0;
    int64_t stat_at = esp_timer_get_time();

    while (stream_keep_running(&s_sender_alive)) {
        /* 连上后先替服务端开麦（小智协议：listen start 由设备发起，auto 由服务端 VAD 收尾） */
        if (s_pending_listen && s_ready) {
            s_pending_listen = false;
            if (send_listen("start", true)) {
                ESP_LOGI(TAG, "已发送 listen start（auto），开始上行");
                s_uplink = true;
                xz_think_clear();
                xz_set_state(XZ_ST_LISTEN);
            } else {
                ESP_LOGW(TAG, "listen start 发送失败，1 秒后重试");
                s_pending_listen = true;
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

        /* 界面「轻触说话」的手动开/关麦：和自动流程走同一条 listen 消息。
           发送放在音频任务里做，LVGL 线程只置 s_listen_req，绝不在界面线程里等网络。 */
        const int req = s_listen_req;
        if (req != 0 && s_ready) {
            s_listen_req = 0;
            if (req == 1) {
                if (send_listen("start", true)) {
                    ESP_LOGI(TAG, "手动：已发送 listen start，开始上行");
                    s_pending_listen = false;       /* 别让自动流程再发一次 */
                    s_uplink = true;
                    xz_think_clear();
                    xz_set_state(XZ_ST_LISTEN);
                } else {
                    ESP_LOGW(TAG, "手动 listen start 发送失败");
                }
            } else {
                if (send_listen("stop", false)) {
                    ESP_LOGI(TAG, "手动：已发送 listen stop，停止上行");
                } else {
                    ESP_LOGW(TAG, "手动 listen stop 发送失败");
                }
                s_uplink = false;
                xz_think_start();                   /* 等服务端把这次说话处理成 tts */
                if (s_state == XZ_ST_LISTEN) {
                    xz_set_state(XZ_ST_IDLE);
                }
            }
        }

        if (s_uplink && s_ready && s_ws != NULL) {
            const int n = voice_audio_record(pcm, XZ_IN_SAMPLES, XZ_IN_RATE, XZ_FRAME_MS);
            if (n < XZ_IN_SAMPLES) {
                stat_err++;
                if (stat_err == 1) {
                    ESP_LOGW(TAG, "录音不足（%d/%d），本帧丢弃", n, XZ_IN_SAMPLES);
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            } else {
                uint8_t *payload = frame + XZ_HDR_SIZE;
                esp_audio_enc_in_frame_t in = {
                    .buffer = (uint8_t *)pcm,
                    .len = (uint32_t)(XZ_IN_SAMPLES * sizeof(int16_t)),
                };
                esp_audio_enc_out_frame_t out = {
                    .buffer = payload,
                    .len = (uint32_t)out_size,
                    .encoded_bytes = 0,
                };
                const esp_audio_err_t e = esp_opus_enc_process(enc, &in, &out);
                if (e != ESP_AUDIO_ERR_OK || out.encoded_bytes == 0) {
                    stat_err++;
                    if (stat_err == 1) {
                        ESP_LOGW(TAG, "Opus 编码失败（%d）", (int)e);
                    }
                } else {
                    int sent;
                    if (s_wire_ver <= 1) {
                        sent = stream_send_bin(payload, out.encoded_bytes);
                    } else {
                        xz_write_header(frame, (uint16_t)s_wire_ver, 0, ts_ms, out.encoded_bytes);
                        sent = stream_send_bin(frame, XZ_HDR_SIZE + out.encoded_bytes);
                    }
                    if (sent < 0) {
                        stat_err++;
                    } else {
                        stat_frames++;
                        stat_bytes += out.encoded_bytes;
                    }
                    ts_ms += XZ_FRAME_MS;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));    /* 空闲/半双工等待：不录不编 */
        }

        /* 每秒一条摘要，不刷屏 */
        const int64_t now = esp_timer_get_time();
        if (now - stat_at >= 1000000) {
            if (s_uplink && (stat_frames > 0 || stat_err > 0)) {
                ESP_LOGI(TAG, "上行：%u 帧 / %u 字节 / 编码错误 %u（已发 %.1f 秒）",
                         (unsigned)stat_frames, (unsigned)stat_bytes, (unsigned)stat_err,
                         (double)ts_ms / 1000.0);
            }
            stat_frames = 0;
            stat_bytes = 0;
            stat_err = 0;
            stat_at = now;
        }
    }

out:
    if (enc != NULL) {
        esp_opus_enc_close(enc);
        enc = NULL;
    }
    free(pcm);
    free(frame);
    ESP_LOGI(TAG, "上行任务退出");
    s_sender_alive = false;
    vTaskDelete(NULL);
}

/* ---- 下行：二进制帧 → opus → 喇叭 ---- */

static void xz_player_task(void *arg)
{
    (void)arg;
    void *dec = NULL;
    uint8_t *raw = NULL;          /* 一帧 opus */
    uint8_t *pcm = NULL;          /* 解码输出（留 2 帧余量）*/
    const uint32_t pcm_cap = (uint32_t)(XZ_OUT_SAMPLES * sizeof(int16_t) * 2);

    esp_opus_dec_cfg_t dcfg = {
        .sample_rate    = XZ_OUT_RATE,
        .channel        = ESP_AUDIO_MONO,
        .frame_duration = ESP_OPUS_DEC_FRAME_DURATION_60_MS,
        .self_delimited = false,
    };
    const esp_audio_err_t oerr = esp_opus_dec_open(&dcfg, sizeof(dcfg), &dec);
    if (oerr != ESP_AUDIO_ERR_OK || dec == NULL) {
        dec = NULL;
        ESP_LOGE(TAG, "Opus 解码器打开失败（%d），下行不可用", (int)oerr);
        goto out;
    }
    ESP_LOGI(TAG, "Opus 解码器就绪：24k 单声道 60ms（内部 RAM %u KB / PSRAM %u KB）",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    raw = xz_psram_alloc(XZ_PLAY_MAX);
    pcm = xz_psram_alloc(pcm_cap);
    if (raw == NULL || pcm == NULL) {
        ESP_LOGE(TAG, "下行缓冲分配失败");
        goto out;
    }

    unsigned gen = s_play_gen;
    uint32_t played_samples = 0;
    uint32_t stat_err = 0;
    int idle_hits = 0;
    bool active = false;
    int64_t stat_at = esp_timer_get_time();

    while (stream_keep_running(&s_player_alive)) {
        uint16_t len = 0;
        bool did = false;
        while (stream_keep_running(&s_player_alive) && play_pop(raw, XZ_PLAY_MAX, &len)) {
            did = true;
            if (gen != s_play_gen) {
                gen = s_play_gen;
                esp_opus_dec_reset(dec);       /* 被打断过：解码状态清一下 */
            }
            esp_audio_dec_in_raw_t in = {
                .buffer = raw,
                .len = len,
                .consumed = 0,
                .frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE,
            };
            esp_audio_dec_out_frame_t of = {
                .buffer = pcm,
                .len = pcm_cap,
                .needed_size = 0,
                .decoded_size = 0,
            };
            esp_audio_dec_info_t info = {0};
            const esp_audio_err_t e = esp_opus_dec_decode(dec, &in, &of, &info);
            if (e == ESP_AUDIO_ERR_OK && of.decoded_size >= sizeof(int16_t)) {
                const int samples = (int)(of.decoded_size / sizeof(int16_t));
                if (s_state != XZ_ST_SPEAK) {
                    xz_set_state(XZ_ST_SPEAK);
                }
                const int played = voice_audio_play((const int16_t *)pcm, samples, XZ_OUT_RATE);
                if (played > 0) {
                    played_samples += (uint32_t)played;
                    active = true;
                } else {
                    stat_err++;
                }
            } else {
                stat_err++;
                if (stat_err == 1) {
                    ESP_LOGW(TAG, "Opus 解码失败（%d，需要 %u 字节）", (int)e, (unsigned)of.needed_size);
                }
            }
        }
        if (!did) {
            vTaskDelay(pdMS_TO_TICKS(15));     /* 队列空：短等，别空转 */
            /* tts 已结束且连续 300ms 没有新帧：把状态从"播放回复"收回"已连接" */
            if (s_state == XZ_ST_SPEAK && !s_tts_active && s_ready) {
                if (++idle_hits >= 20) {
                    idle_hits = 0;
                    xz_set_state(XZ_ST_IDLE);
                }
            } else {
                idle_hits = 0;
            }
        } else {
            idle_hits = 0;
        }

        const int64_t now = esp_timer_get_time();
        if (now - stat_at >= 1000000) {
            if (active || stat_err > 0) {
                ESP_LOGI(TAG, "下行：已播放 %.1f 秒 / 解码错误 %u",
                         (double)played_samples / (double)XZ_OUT_RATE, (unsigned)stat_err);
                active = false;
            }
            stat_err = 0;
            stat_at = now;
        }
    }

out:
    if (dec != NULL) {
        esp_opus_dec_close(dec);
        dec = NULL;
    }
    if (s_q_lock != NULL && xSemaphoreTake(s_q_lock, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_q_head = s_q_tail = s_q_count = 0;      /* 退出前把残留音频丢掉 */
        xSemaphoreGive(s_q_lock);
    }
    free(raw);
    free(pcm);
    ESP_LOGI(TAG, "下行任务退出");
    s_player_alive = false;
    vTaskDelete(NULL);
}

/* --------------------------- 5) 收帧/控制消息 --------------------------- */

/** 二进制帧：剥掉 v2 头后交给播放队列（回调里只做 memcpy）。 */
static void handle_binary(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return;
    }
    const uint8_t *payload = data;
    size_t plen = len;

    if (len >= XZ_HDR_SIZE) {
        const uint16_t ver = xz_rd16(data);
        const uint16_t type = xz_rd16(data + 2);
        const uint32_t psz = xz_rd32(data + 12);
        if (ver == 2 && type <= 1 && psz <= (uint32_t)(len - XZ_HDR_SIZE)) {
            payload = data + XZ_HDR_SIZE;
            plen = psz;
            if (type == 1) {                 /* 协议允许把 JSON 装进二进制头 */
                if (plen < sizeof(s_rx)) {
                    memcpy(s_rx, payload, plen);
                    s_rx[plen] = '\0';
                    handle_text_message(s_rx);
                }
                return;
            }
        } else if (ver != 2 && s_wire_ver != 1) {
            s_wire_ver = 1;                  /* 服务端发裸 opus：上行也切成裸帧 */
            ESP_LOGW(TAG, "服务端二进制帧没有 v2 头，改用协议 v1（裸 opus）");
        }
    } else if (s_wire_ver != 1) {
        s_wire_ver = 1;
        ESP_LOGW(TAG, "服务端二进制帧太短（%u 字节），改用协议 v1（裸 opus）", (unsigned)len);
    }

    if (!s_first_frame_logged) {
        s_first_frame_logged = true;
        ESP_LOGI(TAG, "收到首帧音频：%u 字节（协议 v%d）", (unsigned)plen, (int)s_wire_ver);
    }
    play_push(payload, plen);
}

static void handle_server_hello(const cJSON *root)
{
    const cJSON *sid = cJSON_GetObjectItemCaseSensitive(root, "session_id");
    if (cJSON_IsString(sid) && sid->valuestring[0] != '\0') {
        snprintf(s_session, sizeof(s_session), "%s", sid->valuestring);
    }
    const cJSON *params = cJSON_GetObjectItemCaseSensitive(root, "audio_params");
    const cJSON *sver = cJSON_GetObjectItemCaseSensitive(root, "version");
    if (cJSON_IsNumber(sver)) {
        if (sver->valueint == XZ_PROTO_VER) {
            ESP_LOGI(TAG, "服务端 hello 版本 %d（与上行 v%d 一致）", sver->valueint, XZ_PROTO_VER);
        } else {
            /* 实测这个服务端 hello 里会回 version=1，但二进制帧仍按 v2 打包；
               下行首帧的真假由 handle_binary() 自动识别，这里只提醒一句。 */
            ESP_LOGW(TAG, "服务端 hello 版本 %d（本机按 v%d 收发；若一直只有上行没有下行音频，"
                     "把 XZ_PROTO_VER 改成 %d 再试）",
                     sver->valueint, XZ_PROTO_VER, sver->valueint > 0 ? sver->valueint : 1);
        }
    }
    if (cJSON_IsObject(params)) {
        const cJSON *fmt = cJSON_GetObjectItemCaseSensitive(params, "format");
        const cJSON *rate = cJSON_GetObjectItemCaseSensitive(params, "sample_rate");
        const cJSON *dur = cJSON_GetObjectItemCaseSensitive(params, "frame_duration");
        const cJSON *ch = cJSON_GetObjectItemCaseSensitive(params, "channels");
        ESP_LOGI(TAG, "服务端音频参数：%s / %d Hz / %d 声道 / %d ms",
                 cJSON_IsString(fmt) ? fmt->valuestring : "?",
                 cJSON_IsNumber(rate) ? rate->valueint : 0,
                 cJSON_IsNumber(ch) ? ch->valueint : 0,
                 cJSON_IsNumber(dur) ? dur->valueint : 0);
        if (cJSON_IsNumber(rate) && rate->valueint != XZ_OUT_RATE) {
            ESP_LOGW(TAG, "服务端下行采样率 %d != %d，播放会变调（TODO(stream)）",
                     rate->valueint, XZ_OUT_RATE);
        }
    }
    s_ready = true;
    s_wire_ver = XZ_PROTO_VER;
    s_state = XZ_ST_IDLE;
    s_error = false;
    xz_think_clear();
    set_status("已连接");
    stream_start();          /* 起编解码任务；listen start 由上行任务发 */
}

static void handle_text_message(const char *json)
{
    if (json == NULL) {
        return;
    }
    cJSON *root = cJSON_Parse(json);
    if (root == NULL) {
        return;
    }
    const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
    if (!cJSON_IsString(type)) {
        cJSON_Delete(root);
        return;
    }
    const char *t = type->valuestring;
    const cJSON *state = cJSON_GetObjectItemCaseSensitive(root, "state");
    const char *st = cJSON_IsString(state) ? state->valuestring : "";

    if (strcmp(t, "hello") == 0) {
        handle_server_hello(root);

    } else if (strcmp(t, "listen") == 0) {
        if (strcmp(st, "start") == 0) {
            /* 开麦：先把没放完的回复丢掉，避免麦克风听见自己的喇叭声 */
            play_flush(false);
            s_tts_active = false;
            s_uplink = true;
            xz_think_clear();
            xz_set_state(XZ_ST_LISTEN);
            ESP_LOGI(TAG, "服务端：开始聆听");
        } else {
            s_uplink = false;
            if (s_ready) {
                xz_think_start();      /* 上行停了、回复还没来：界面显示"思考中" */
            }
            if (s_state == XZ_ST_LISTEN) {
                xz_set_state(XZ_ST_IDLE);
            }
            ESP_LOGI(TAG, "服务端：停止聆听（%s）", st);
        }

    } else if (strcmp(t, "tts") == 0) {
        if (strcmp(st, "start") == 0) {
            s_uplink = false;              /* 半双工：放音时不发麦克风 */
            s_tts_active = true;
            xz_think_clear();
            xz_set_state(XZ_ST_THINK);
        } else if (strcmp(st, "stop") == 0) {
            s_tts_active = false;
            xz_think_clear();
            /* 队列里可能还有几百毫秒没放完：播放任务放完最后一帧再把状态收回"已连接" */
            if (s_q_count == 0 && s_state != XZ_ST_IDLE) {
                xz_set_state(XZ_ST_IDLE);
            }
            ESP_LOGI(TAG, "服务端：TTS 结束");
        } else if (strcmp(st, "sentence_start") == 0) {
            /* 只是字幕，忽略 */
        }

    } else if (strcmp(t, "abort") == 0) {
        s_uplink = false;
        s_tts_active = false;
        xz_think_clear();
        play_flush(true);                  /* 丢队列 + 让播放任务重置解码器 */
        xz_set_state(XZ_ST_IDLE);
        ESP_LOGW(TAG, "服务端：打断（停止双向、清空待播音频）");

    } else if (strcmp(t, "stt") == 0) {
        const cJSON *text = cJSON_GetObjectItemCaseSensitive(root, "text");
        if (cJSON_IsString(text)) {
            ESP_LOGI(TAG, "识别到：%.120s", text->valuestring);
            set_last_text(text->valuestring);   /* 界面显示"刚才说了什么" */
        }

    } else if (strcmp(t, "llm") == 0) {
        const cJSON *text = cJSON_GetObjectItemCaseSensitive(root, "text");
        if (cJSON_IsString(text)) {
            ESP_LOGI(TAG, "回复文字：%.120s", text->valuestring);
        }
    }
    cJSON_Delete(root);
}

/* --------------------------- 6) WebSocket --------------------------- */

static void ws_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    esp_websocket_event_data_t *ev = (esp_websocket_event_data_t *)data;

    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED: {
        ESP_LOGI(TAG, "WebSocket 已连接，发送 hello");
        s_wire_ver = XZ_PROTO_VER;
        s_session[0] = '\0';
        s_error = false;
        /* v2：声明的版本要和二进制帧头一致（服务端按这个决定怎么打包下行音频） */
        const char hello[] =
            "{\"type\":\"hello\",\"version\":2,\"transport\":\"websocket\","
            "\"features\":{\"mcp\":false,\"aec\":false},"
            "\"audio_params\":{\"format\":\"opus\",\"sample_rate\":16000,"
            "\"channels\":1,\"frame_duration\":60}}";
        if (s_ws != NULL) {
            esp_websocket_client_send_text(s_ws, hello, (int)strlen(hello), pdMS_TO_TICKS(5000));
        }
        set_status("握手 …");
        break;
    }
    case WEBSOCKET_EVENT_DATA: {
        if (ev == NULL || ev->data_len <= 0) {
            break;
        }

        /* 二进制帧：opus 音频（或装进二进制头的 JSON） */
        if (ev->op_code == 0x02) {
            const size_t n = (size_t)ev->data_len;
            if (ev->payload_offset == 0) {
                s_bin_len = 0;
            }
            if (s_bin == NULL) {
                s_bin = xz_psram_alloc(XZ_BIN_MAX);
            }
            if (s_bin != NULL && s_bin_len + n <= XZ_BIN_MAX) {
                memcpy(s_bin + s_bin_len, ev->data_ptr, n);
                s_bin_len += n;
            }
            const size_t total = (size_t)ev->payload_offset + n;
            if (ev->payload_len > 0 && total >= (size_t)ev->payload_len) {
                if (s_bin != NULL && s_bin_len == (size_t)ev->payload_len) {
                    handle_binary(s_bin, s_bin_len);
                } else {
                    ESP_LOGW(TAG, "二进制帧过大（%d 字节），丢弃", ev->payload_len);
                }
                s_bin_len = 0;
            }
            break;
        }

        /* 文本帧：JSON 控制消息 */
        if (ev->op_code != 0x01) {
            break;
        }
        if (ev->payload_offset == 0) {
            s_rx_len = 0;
        }
        const size_t n = (size_t)ev->data_len;
        if (s_rx_len + n < sizeof(s_rx)) {
            memcpy(s_rx + s_rx_len, ev->data_ptr, n);
            s_rx_len += n;
            s_rx[s_rx_len] = '\0';
        }
        const size_t total = (size_t)ev->payload_offset + n;
        if (ev->payload_len > 0 && total >= (size_t)ev->payload_len) {
            ESP_LOGI(TAG, "收到文本: %.160s", s_rx);
            handle_text_message(s_rx);
            s_rx_len = 0;
        }
        break;
    }
    case WEBSOCKET_EVENT_DISCONNECTED:
        s_ready = false;
        s_error = true;             /* 界面先画 ERROR 脸，重连成功后由 CONNECTED 清掉 */
        xz_think_clear();
        stream_set_stop();          /* 只置标志：编码/播放任务自己收尾并释放句柄 */
        set_status("连接断开");
        break;
    case WEBSOCKET_EVENT_ERROR:
        s_ready = false;
        s_error = true;
        xz_think_clear();
        stream_set_stop();
        set_status("连接出错");
        break;
    default:
        break;
    }
}

/* ----------------------------- 工作线程 ----------------------------- */

static void xz_task(void *arg)
{
    const int my_gen = (int)(intptr_t)arg;
    esp_websocket_client_handle_t my_ws = NULL;
    s_task_alive = true;

    if (my_gen != s_gen) {
        goto out;
    }

    /* 语音要长期占一条 TLS：先把背景图下载停掉，避免两条 TLS 抢内部 RAM */
    bg_image_set_paused(true);

    set_status("发现服务端 …");
    /* 开机时语音配置来自 NVS，可能比 WiFi 还早就绪；发现失败就等网络好了重试 */
    while (my_gen == s_gen && s_enabled) {
        if (!net_wifi_connected()) {
            s_error = false;
            set_status("等待网络 …");
        } else {
            s_error = false;            /* 这次尝试先按"连接中"显示 */
            if (discover_ws_url()) {
                break;
            }
            if (my_gen != s_gen || !s_enabled) {
                goto out;
            }
            s_error = true;             /* 发现失败：界面画 ERROR 脸 + 原因 */
            set_status("服务端不可达，稍后重试");
        }
        for (int i = 0; i < 10; i++) {
            if (my_gen != s_gen || !s_enabled) goto out;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (my_gen != s_gen || !s_enabled || s_ws_url[0] == '\0') {
        goto out;
    }

    set_status("连接服务端 …");
    esp_websocket_client_config_t ws_cfg = {
        .uri = s_ws_url,
        .reconnect_timeout_ms = 5000,
        .network_timeout_ms = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 2048,
        .task_stack = XZ_WS_TASK_STACK,
    };
    my_ws = esp_websocket_client_init(&ws_cfg);
    s_ws = my_ws;                      /* 事件回调/发送任务靠它发数据 */
    if (my_ws == NULL) {
        s_error = true;
        set_status("WebSocket 初始化失败");
        goto out;
    }
    if (s_token[0] != '\0') {
        esp_websocket_client_append_header(my_ws, "Authorization", s_token);
    }
    esp_websocket_client_append_header(my_ws, "Protocol-Version", "2");
    esp_websocket_client_append_header(my_ws, "Device-Id", s_device_id);
    esp_websocket_client_append_header(my_ws, "Client-Id", s_client_id);

    esp_websocket_register_events(my_ws, WEBSOCKET_EVENT_ANY, ws_event, NULL);
    if (esp_websocket_client_start(my_ws) != ESP_OK) {
        s_error = true;
        set_status("WebSocket 启动失败");
        goto out;
    }

    while (my_gen == s_gen) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

out:
    /* 先把音频流停干净（编解码句柄在各自任务里关闭），再拆 WebSocket；
       否则发送任务可能拿着已经销毁的 WS 句柄发数据。 */
    if (!s_enabled || my_gen == s_gen) {
        stream_stop_wait(1500);
    }
    if (s_send_lock != NULL) {
        xSemaphoreTake(s_send_lock, portMAX_DELAY);
    }
    /* 只用本任务自己建的句柄：重启配置时旧任务可能比新任务晚退出，
       若按全局 s_ws 去 destroy，会把新任务的连接拆掉。 */
    if (my_ws != NULL) {
        if (s_ws == my_ws) {
            s_ws = NULL;               /* 先摘掉：发送任务看到 NULL 就不会再发 */
        }
        esp_websocket_client_stop(my_ws);
        esp_websocket_client_destroy(my_ws);
    }
    if (s_send_lock != NULL) {
        xSemaphoreGive(s_send_lock);
    }
    s_ready = false;
    if (my_gen == s_gen) {
        set_status("未连接");
    }
    bg_image_set_paused(false);      /* 退出时把背景图还回去 */
    s_task_alive = false;
    vTaskDelete(NULL);
}

/* ------------------------------ 对外接口 ------------------------------ */

void xiaozhi_client_configure(bool enabled, const char *endpoint)
{
    init_ids();

    const bool want_on = enabled && endpoint != NULL && endpoint[0] != '\0';
    if (!want_on) {
        if (!s_enabled) {
            return;
        }
        s_enabled = false;
        s_gen++;                 /* 让旧任务退出（任务里会停流 + stop/destroy WS） */
        s_ready = false;
        s_error = false;
        s_listen_req = 0;
        xz_think_clear();
        stream_set_stop();       /* 立刻停流，不等任务（这里常在 LVGL 线程里跑） */
        snprintf(s_status, sizeof(s_status), "未启用");
        ESP_LOGI(TAG, "小智已停止");
        return;
    }

    if (s_enabled && strcmp(endpoint, s_endpoint) == 0) {
        return;                  /* 配置没变 */
    }

    s_enabled = true;
    s_error = false;
    s_listen_req = 0;
    xz_think_clear();
    snprintf(s_endpoint, sizeof(s_endpoint), "%s", endpoint);
    s_gen++;
    const int gen = s_gen;

    /* 音频通路先就绪（幂等）：这里在主任务，别放到 WS 回调里做 I2S 初始化 */
    if (voice_audio_init() != 0) {
        ESP_LOGW(TAG, "音频通路初始化失败，语音可能没声音（看上面 voice_audio 日志）");
    }
    if (!ensure_stream_buffers()) {
        ESP_LOGE(TAG, "音频流缓冲创建失败");
    }

    /* 等旧任务退出，避免两个 WebSocket 同时在线。
       调用方是 LVGL 主循环（apply_cfg），所以最多等 1 秒，别把界面卡太久；
       旧任务晚退出也没关系——它只拆自己建的句柄。 */
    for (int i = 0; i < 10 && s_task_alive; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    snprintf(s_status, sizeof(s_status), "准备连接 …");
    if (xTaskCreate(xz_task, "xiaozhi", 8192, (void *)(intptr_t)gen, 4, NULL) != pdPASS) {
        s_enabled = false;
        s_error = true;
        snprintf(s_status, sizeof(s_status), "任务创建失败");
    }
}

const char *xiaozhi_activation_code(void)
{
    return s_activation;
}

const char *xiaozhi_client_status_text(void)
{
    return s_status[0] != '\0' ? s_status : "未启用";
}

bool xiaozhi_client_ready(void)
{
    return s_ready;
}

const char *xiaozhi_client_server_note(void)
{
    return s_note;
}

/* ------------------------- 界面用的状态 / 控制 API ------------------------- */

xz_state_t xiaozhi_state(void)
{
    if (!s_enabled) {
        return XZ_IDLE;                 /* 没开语音：不管连没连上都算空闲 */
    }
    if (s_state == XZ_ST_LISTEN) {
        return XZ_LISTENING;
    }
    if (s_state == XZ_ST_SPEAK) {
        return XZ_SPEAKING;
    }
    if (s_state == XZ_ST_THINK || s_thinking) {
        /* 上行停了、下行还没来 = 服务端在思考。
           说了句空话时服务端可能不回 tts，8 秒兜底收回，别让界面一直转圈。 */
        if (s_thinking && (esp_timer_get_time() - s_thinking_at) > 8LL * 1000 * 1000) {
            s_thinking = false;
        } else {
            return XZ_THINKING;
        }
    }
    if (s_error) {
        return XZ_ERROR;
    }
    if (!s_ready) {
        return XZ_CONNECTING;
    }
    return XZ_IDLE;
}

bool xiaozhi_listen_start(void)
{
    if (!s_enabled) {
        ESP_LOGW(TAG, "手动开麦被拒：小智未启用");
        return false;
    }
    if (!s_ready) {
        ESP_LOGW(TAG, "手动开麦被拒：还没连上服务端（%s）", xiaozhi_client_status_text());
        return false;
    }
    s_listen_req = 1;       /* 上行任务看到就发 listen start，界面线程不等网络 */
    ESP_LOGI(TAG, "手动开麦已排队");
    return true;
}

void xiaozhi_listen_stop(void)
{
    if (!s_enabled) {
        return;
    }
    s_listen_req = 2;       /* 上行任务看到就发 listen stop */
    ESP_LOGI(TAG, "手动关麦已排队");
}

const char *xiaozhi_last_text(void)
{
    taskENTER_CRITICAL(&s_text_mux);
    memcpy(s_text_snap, s_last_text, sizeof(s_text_snap));
    taskEXIT_CRITICAL(&s_text_mux);
    return s_text_snap;
}
