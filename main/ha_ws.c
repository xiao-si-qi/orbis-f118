/**
 * Home Assistant WebSocket 客户端（/api/websocket）。
 *
 * 设计要点：
 *  - 一个 supervisor 任务负责建连/退避/重建，WS 客户端自己的任务只跑收发；
 *  - 认证成功后订阅 state_changed，事件里只挑 entity_id / new_state.state /
 *    new_state.attributes，交给 ha_client_apply_state() 更新实体表
 *    （ui_home 的 500ms 刷新定时器下一拍自然显示，WS 任务从不碰 LVGL）；
 *  - service call 用自增 id 发送，收到 result 才记"成功/失败 + 往返毫秒"，
 *    绝不阻塞等待；
 *  - 地址从 app_settings 的 HA URL 推导：https→wss（带证书包），http→ws
 *    （明文，仅限可信内网）。wss 认证后让 ha_client 暂停并释放 REST 的
 *    HTTP/TLS，全程只保留一条 TLS；ws 无 TLS 开销，REST 降到 60 秒兜底。
 *  - 接收缓冲固定 4KB，超过上限的整条事件直接跳过并打警告，不做无界拼接。
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "sdkconfig.h"

#include "cJSON.h"

#include "app_settings.h"
#include "ha_client.h"
#include "ha_ws.h"
#include "net.h"

static const char *TAG = "ha_ws";

#define HA_WS_URL_MAX            (APP_HA_URL_MAX + 32)
#define HA_WS_JSON_MAX           (4 * 1024)      /* 单条 WS 文本消息上限 */
#define HA_WS_ATTRS_MAX          1536            /* attributes 序列化缓冲 */
#define HA_WS_TX_MAX             512             /* 单条 service call JSON 上限 */
#define HA_WS_TASK_STACK         6144            /* supervisor 任务栈 */
#define HA_WS_CLIENT_STACK       8192            /* WS 任务栈（TLS 握手在这条任务上） */
#define HA_WS_BUFFER_SIZE        2048            /* WS 收包分片缓冲 */
#define HA_WS_AUTH_TIMEOUT_MS    15000           /* 连上后等 auth_ok 的上限 */
#define HA_WS_TLS_MIN_FREE       (60 * 1024)     /* 起 wss 前要求的最小内部 RAM */
#define HA_WS_REST_RESUME_FAILS  3               /* 连续失败几次后让 REST 接管 */
#define HA_WS_BACKOFF_MIN_S      5
#define HA_WS_BACKOFF_MAX_S      60
#define HA_WS_PENDING_MAX        8
#define HA_WS_ID_SUBSCRIBE       1
#define HA_WS_SUMMARY_US         (60LL * 1000 * 1000)

typedef struct {
    int id;
    int64_t t_us;
    char what[72];
} ha_ws_pending_t;

static TaskHandle_t s_task;
static SemaphoreHandle_t s_tx_lock;
static volatile bool s_stop_req;

static esp_websocket_client_handle_t s_client;   /* 受 s_tx_lock 保护 */
static volatile int s_tx_busy;                   /* 正在发送的调用数，销毁前要等它归零 */

static volatile bool s_connected;                /* TCP/WS 已连上 */
static volatile bool s_authed;                   /* auth_ok 且尚未断开 */
static volatile bool s_subscribed;
static volatile bool s_link_down;
static volatile bool s_auth_bad;
static volatile bool s_using_tls;

static volatile uint32_t s_connects;
static volatile uint32_t s_events;
static volatile uint32_t s_cmds;
static volatile uint32_t s_fail;
static volatile int64_t s_last_rtt_us;
static char s_last_err[64];

static char s_rx[HA_WS_JSON_MAX + 1];
static size_t s_rx_len;
static int s_rx_total;
static bool s_rx_skip;

static int s_next_id = 2;                        /* 1 留给 subscribe_events */
static char s_conn_url[HA_WS_URL_MAX];
static char s_conn_token[APP_HA_TOKEN_MAX];

static ha_ws_pending_t s_pending[HA_WS_PENDING_MAX];

/* ------------------------------------------------------------------------- */
/* 小工具                                                                     */
/* ------------------------------------------------------------------------- */

static int64_t now_us(void)
{
    return esp_timer_get_time();
}

static void set_last_err(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_last_err, sizeof(s_last_err), fmt, ap);
    va_end(ap);
}

static void pending_add(int id, const char *what, int64_t t_us)
{
    if (s_tx_lock == NULL) {
        return;
    }
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    ha_ws_pending_t *slot = NULL;
    for (int i = 0; i < HA_WS_PENDING_MAX; i++) {
        if (s_pending[i].id == 0) {
            slot = &s_pending[i];
            break;
        }
    }
    if (slot == NULL) {
        slot = &s_pending[0];              /* 满了就覆盖最旧的一条，不影响功能 */
        for (int i = 1; i < HA_WS_PENDING_MAX; i++) {
            if (s_pending[i].t_us < slot->t_us) {
                slot = &s_pending[i];
            }
        }
    }
    slot->id = id;
    slot->t_us = t_us;
    snprintf(slot->what, sizeof(slot->what), "%s", what ? what : "-");
    xSemaphoreGive(s_tx_lock);
}

static int64_t pending_take(int id, char *what, size_t what_len)
{
    int64_t t_us = 0;
    if (what != NULL && what_len > 0) {
        what[0] = '\0';
    }
    if (s_tx_lock == NULL) {
        return 0;
    }
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    for (int i = 0; i < HA_WS_PENDING_MAX; i++) {
        if (s_pending[i].id == id) {
            t_us = s_pending[i].t_us;
            if (what != NULL && what_len > 0) {
                snprintf(what, what_len, "%s", s_pending[i].what);
            }
            s_pending[i].id = 0;
            s_pending[i].t_us = 0;
            break;
        }
    }
    xSemaphoreGive(s_tx_lock);
    return t_us;
}

/** 从 app_settings 的 HA URL 推导 WebSocket 地址；失败返回 false。 */
static bool build_ws_url(char *out, size_t out_len, bool *tls)
{
    const app_settings_t *cfg = app_settings_get();
    const char *rest = NULL;
    if (strncmp(cfg->ha_url, "https://", 8) == 0) {
        rest = cfg->ha_url + 8;
        *tls = true;
    } else if (strncmp(cfg->ha_url, "http://", 7) == 0) {
        rest = cfg->ha_url + 7;
        *tls = false;
    } else {
        return false;
    }

    char host[APP_HA_URL_MAX];
    snprintf(host, sizeof(host), "%s", rest);
    size_t n = strlen(host);
    while (n > 0 && host[n - 1] == '/') {
        host[--n] = '\0';                  /* 末尾斜杠去掉，避免 //api */
    }
    if (n == 0) {
        return false;
    }
    /* host 里允许带路径前缀（如 https://ha.example.com/ha），直接接 /api/websocket */
    const int written = snprintf(out, out_len, "%s://%s/api/websocket",
                                 *tls ? "wss" : "ws", host);
    return written > 0 && (size_t)written < out_len;
}

/* ------------------------------------------------------------------------- */
/* 发送                                                                       */
/* ------------------------------------------------------------------------- */

/**
 * 取一次发送引用：成功后必须配对调用 ws_release()。
 * 这样 supervisor 销毁句柄前能等在途发送结束，而发送期间不用一直抱着锁
 * （detail.c 会在 LVGL 线程里直接调 service call，不能长时间阻塞）。
 */
static esp_websocket_client_handle_t ws_acquire(void)
{
    if (s_tx_lock == NULL) {
        return NULL;
    }
    if (xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "发送锁忙，放弃本次发送");
        return NULL;
    }
    esp_websocket_client_handle_t cli = NULL;
    if (s_client != NULL && ha_ws_connected()) {
        cli = s_client;
        s_tx_busy++;
    }
    xSemaphoreGive(s_tx_lock);
    return cli;
}

static void ws_release(void)
{
    if (s_tx_lock == NULL) {
        return;
    }
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    if (s_tx_busy > 0) {
        s_tx_busy--;
    }
    xSemaphoreGive(s_tx_lock);
}

/** 事件回调里用：client 就是当前 WS 任务自己的句柄，不会被销毁。 */
static bool ws_send_raw(esp_websocket_client_handle_t client, const char *data, size_t len)
{
    if (client == NULL) {
        return false;
    }
    const int sent = esp_websocket_client_send_text(client, data, (int)len,
                                                    pdMS_TO_TICKS(1000));
    if (sent < 0) {
        s_fail++;
        set_last_err("WebSocket 发送失败");
        ESP_LOGW(TAG, "WebSocket 发送失败（%u 字节）", (unsigned)len);
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* 协议处理                                                                   */
/* ------------------------------------------------------------------------- */

static void handle_auth_required(esp_websocket_client_handle_t client)
{
    const app_settings_t *cfg = app_settings_get();
    char auth[APP_HA_TOKEN_MAX + 48];
    const int n = snprintf(auth, sizeof(auth),
                           "{\"type\":\"auth\",\"access_token\":\"%s\"}", cfg->ha_token);
    if (n <= 0 || (size_t)n >= sizeof(auth)) {
        set_last_err("认证令牌过长");
        ESP_LOGE(TAG, "认证令牌过长，无法发送 auth");
        return;
    }
    if (ws_send_raw(client, auth, (size_t)n)) {
        ESP_LOGI(TAG, "收到 auth_required：已发送访问令牌（长度 %u）", (unsigned)strlen(cfg->ha_token));
    }
}

static void handle_auth_ok(esp_websocket_client_handle_t client)
{
    if (s_authed) {
        return;
    }
    s_authed = true;
    s_auth_bad = false;
    s_subscribed = false;
    s_connects++;
    set_last_err("");
    ha_client_set_online(true);
    s_next_id = 2;
    ESP_LOGI(TAG, "认证成功（auth_ok），内部 RAM %d KB",
             (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    const char sub[] = "{\"id\":1,\"type\":\"subscribe_events\",\"event_type\":\"state_changed\"}";
    if (ws_send_raw(client, sub, sizeof(sub) - 1)) {
        ESP_LOGI(TAG, "已订阅 state_changed（id=%d）", HA_WS_ID_SUBSCRIBE);
    } else {
        s_link_down = true;          /* 交给 supervisor 重连 */
    }
}

static void handle_result(cJSON *root)
{
    const cJSON *jid = cJSON_GetObjectItemCaseSensitive(root, "id");
    if (!cJSON_IsNumber(jid)) {
        return;
    }
    const int id = jid->valueint;
    const bool ok = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(root, "success"));

    if (id == HA_WS_ID_SUBSCRIBE) {
        s_subscribed = ok;
        if (ok) {
            ESP_LOGI(TAG, "订阅 state_changed 成功");
        } else {
            s_fail++;
            const cJSON *err = cJSON_GetObjectItemCaseSensitive(root, "error");
            const cJSON *msg = cJSON_GetObjectItemCaseSensitive(err, "message");
            set_last_err("订阅 state_changed 失败");
            ESP_LOGE(TAG, "订阅 state_changed 失败：%s",
                     cJSON_IsString(msg) ? msg->valuestring : "未知原因");
            s_link_down = true;              /* 收不到事件就没法实时刷新：重连 */
        }
        return;
    }

    char what[72];
    const int64_t t_us = pending_take(id, what, sizeof(what));
    const int64_t rtt_ms = (t_us > 0) ? (now_us() - t_us) / 1000 : -1;
    if (rtt_ms >= 0) {
        s_last_rtt_us = (now_us() - t_us);
    }

    if (ok) {
        ESP_LOGI(TAG, "服务调用 #%d %s 成功，往返 %lld ms",
                 id, what[0] ? what : "?", (long long)rtt_ms);
        return;
    }

    s_fail++;
    const cJSON *err = cJSON_GetObjectItemCaseSensitive(root, "error");
    const cJSON *code = cJSON_GetObjectItemCaseSensitive(err, "code");
    const cJSON *msg = cJSON_GetObjectItemCaseSensitive(err, "message");
    set_last_err("服务调用失败：%s", cJSON_IsString(msg) ? msg->valuestring : "未知错误");
    ESP_LOGW(TAG, "服务调用 #%d %s 失败：%s %s（往返 %lld ms）",
             id, what[0] ? what : "?",
             cJSON_IsString(code) ? code->valuestring : "-",
             cJSON_IsString(msg) ? msg->valuestring : "未知错误",
             (long long)rtt_ms);
}

static void handle_state_changed(cJSON *root)
{
    const cJSON *event = cJSON_GetObjectItemCaseSensitive(root, "event");
    const cJSON *etype = cJSON_GetObjectItemCaseSensitive(event, "event_type");
    if (cJSON_IsString(etype) && strcmp(etype->valuestring, "state_changed") != 0) {
        return;
    }
    const cJSON *data = cJSON_GetObjectItemCaseSensitive(event, "data");
    const cJSON *eid = cJSON_GetObjectItemCaseSensitive(data, "entity_id");
    if (!cJSON_IsString(eid)) {
        return;
    }
    s_events++;
    if (s_events % 100 == 0) {
        ESP_LOGD(TAG, "已收到 %u 条 state_changed", (unsigned)s_events);
    }

    const cJSON *new_state = cJSON_GetObjectItemCaseSensitive(data, "new_state");
    if (new_state == NULL || cJSON_IsNull(new_state)) {
        ESP_LOGD(TAG, "state_changed: %s 已删除", eid->valuestring);
        return;
    }
    const cJSON *state = cJSON_GetObjectItemCaseSensitive(new_state, "state");
    if (!cJSON_IsString(state)) {
        return;
    }

    char attrs_buf[HA_WS_ATTRS_MAX];
    const char *attrs_json = NULL;
    const cJSON *attrs = cJSON_GetObjectItemCaseSensitive(new_state, "attributes");
    if (cJSON_IsObject(attrs) && cJSON_GetArraySize(attrs) > 0) {
        if (cJSON_PrintPreallocated((cJSON *)attrs, attrs_buf, sizeof(attrs_buf), 0)) {
            attrs_json = attrs_buf;
        } else {
            ESP_LOGW(TAG, "state_changed: %s 的属性超过 %d 字节，只更新 state",
                     eid->valuestring, HA_WS_ATTRS_MAX);
        }
    }

    if (ha_client_apply_state(eid->valuestring, state->valuestring, attrs_json)) {
        ESP_LOGD(TAG, "state_changed: %s = %s", eid->valuestring, state->valuestring);
    } else {
        ESP_LOGD(TAG, "state_changed: %s 不在关注列表，忽略", eid->valuestring);
    }
}

static void handle_message(esp_websocket_client_handle_t client, const char *json, size_t len)
{
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (root == NULL) {
        s_fail++;
        set_last_err("JSON 解析失败");
        ESP_LOGW(TAG, "收到无法解析的 JSON（%u 字节），丢弃", (unsigned)len);
        return;
    }

    const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
    const char *t = cJSON_GetStringValue(type);
    if (t != NULL) {
        if (strcmp(t, "auth_required") == 0) {
            handle_auth_required(client);
        } else if (strcmp(t, "auth_ok") == 0) {
            handle_auth_ok(client);
        } else if (strcmp(t, "auth_invalid") == 0) {
            s_authed = false;
            s_auth_bad = true;
            s_fail++;
            set_last_err("认证失败：令牌无效或已过期");
            ESP_LOGE(TAG, "认证失败（auth_invalid）：令牌无效或已过期，稍后按退避重连");
        } else if (strcmp(t, "event") == 0) {
            handle_state_changed(root);
        } else if (strcmp(t, "result") == 0) {
            handle_result(root);
        } else if (strcmp(t, "pong") == 0) {
            /* 心跳回复，静默 */
        } else {
            ESP_LOGD(TAG, "未处理的 WS 消息 type=%s", t);
        }
    }
    cJSON_Delete(root);
}

/** 把分片的文本帧拼进固定缓冲；超限的整条消息跳过。 */
static void handle_text_frame(esp_websocket_event_data_t *ev)
{
    if (ev->data_ptr == NULL || ev->data_len <= 0) {
        return;
    }
    const int total = (ev->payload_len > 0) ? ev->payload_len
                                            : (ev->payload_offset + ev->data_len);
    if (ev->payload_offset == 0) {
        s_rx_len = 0;
        s_rx_skip = false;
        s_rx_total = total;
    }

    if (total > HA_WS_JSON_MAX) {
        if (!s_rx_skip) {
            s_rx_skip = true;
            s_fail++;
            set_last_err("WS 事件过大");
            ESP_LOGW(TAG, "WS 事件过大（%d 字节 > %d），整条跳过", total, HA_WS_JSON_MAX);
        }
    } else if (!s_rx_skip) {
        const size_t n = (size_t)ev->data_len;
        if (s_rx_len + n <= HA_WS_JSON_MAX) {
            memcpy(s_rx + s_rx_len, ev->data_ptr, n);
            s_rx_len += n;
            s_rx[s_rx_len] = '\0';
        } else {
            s_rx_skip = true;
            s_fail++;
            set_last_err("WS 事件过大");
            ESP_LOGW(TAG, "WS 事件累计超过 %d 字节，整条跳过", HA_WS_JSON_MAX);
        }
    }

    if (s_rx_total > 0 && ev->payload_offset + ev->data_len >= s_rx_total) {
        if (!s_rx_skip && s_rx_len > 0) {
            handle_message(ev->client, s_rx, s_rx_len);
        }
        s_rx_len = 0;
        s_rx_skip = false;
        s_rx_total = 0;
    }
}

static void ha_ws_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    esp_websocket_event_data_t *ev = (esp_websocket_event_data_t *)data;

    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
        s_connected = true;
        s_link_down = false;
        s_rx_len = 0;
        s_rx_skip = false;
        s_rx_total = 0;
        ESP_LOGI(TAG, "WebSocket 已连接，等待 auth_required（%s）",
                 s_using_tls ? "TLS" : "明文");
        break;

    case WEBSOCKET_EVENT_DATA:
        if (ev != NULL && ev->op_code == 0x01) {
            handle_text_frame(ev);
        }
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
    case WEBSOCKET_EVENT_CLOSED:
        if (!s_link_down) {
            s_link_down = true;
            s_connected = false;
            s_authed = false;
            s_subscribed = false;
            s_fail++;
            set_last_err(id == WEBSOCKET_EVENT_CLOSED ? "连接被关闭" : "连接断开");
            ESP_LOGW(TAG, "%s", id == WEBSOCKET_EVENT_CLOSED ? "WebSocket 被服务端关闭"
                                                             : "WebSocket 断开");
            ha_client_set_online(false);
        }
        break;

    case WEBSOCKET_EVENT_ERROR:
        if (!s_link_down) {
            s_link_down = true;
            s_connected = false;
            s_authed = false;
            s_fail++;
            set_last_err("连接错误");
            if (ev != NULL) {
                ESP_LOGW(TAG, "WebSocket 错误：type=%d tls=0x%x http=%d sock_errno=%d",
                         (int)ev->error_handle.error_type,
                         (unsigned)ev->error_handle.esp_tls_last_esp_err,
                         ev->error_handle.esp_ws_handshake_status_code,
                         ev->error_handle.esp_transport_sock_errno);
            } else {
                ESP_LOGW(TAG, "WebSocket 错误");
            }
            ha_client_set_online(false);
        }
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* 状态摘要                                                                   */
/* ------------------------------------------------------------------------- */

void ha_ws_status(char *out, size_t len)
{
    if (out == NULL || len == 0) {
        return;
    }
    const char *state;
    if (s_task == NULL) {
        state = "已停止";
    } else if (s_authed) {
        state = "已认证";
    } else if (s_connected) {
        state = "已连接/认证中";
    } else {
        state = "未连接";
    }
    const int64_t rtt_ms = (s_last_rtt_us > 0) ? s_last_rtt_us / 1000 : -1;
    char url[HA_WS_URL_MAX];
    bool cfg_tls = false;
    const bool configured = build_ws_url(url, sizeof(url), &cfg_tls);
    snprintf(out, len,
             "状态=%s 方案=%s 连接成功=%u 事件=%u 命令=%u 失败=%u 最近往返=%lldms "
             "最后错误=%s 内部RAM=%uKB",
             state, configured ? (cfg_tls ? "wss(TLS)" : "ws(明文)") : "未配置",
             (unsigned)s_connects, (unsigned)s_events, (unsigned)s_cmds, (unsigned)s_fail,
             (long long)rtt_ms, s_last_err[0] ? s_last_err : "无",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
}

static void log_summary(void)
{
    char buf[256];
    ha_ws_status(buf, sizeof(buf));
    ESP_LOGI(TAG, "周期摘要：%s", buf);
}

static void maybe_summary(int64_t *last_us)
{
    const int64_t now = now_us();
    if (now - *last_us >= HA_WS_SUMMARY_US) {
        *last_us = now;
        log_summary();
    }
}

static void wait_with_summary(uint32_t ms, int64_t *last_us)
{
    const int64_t end = now_us() + (int64_t)ms * 1000;
    while (!s_stop_req && now_us() < end) {
        vTaskDelay(pdMS_TO_TICKS(250));
        maybe_summary(last_us);
    }
}

/* ------------------------------------------------------------------------- */
/* 连接生命周期                                                               */
/* ------------------------------------------------------------------------- */

/** 只销毁本次任务自己建的句柄；先摘掉全局引用，再等在途发送结束。 */
static void cleanup_client(esp_websocket_client_handle_t cli)
{
    if (cli == NULL) {
        return;
    }
    if (s_tx_lock != NULL) {
        xSemaphoreTake(s_tx_lock, portMAX_DELAY);
        s_connected = false;
        s_authed = false;
        s_subscribed = false;
        if (s_client == cli) {
            s_client = NULL;
        }
        xSemaphoreGive(s_tx_lock);
    } else {
        s_client = NULL;
    }

    for (int i = 0; i < 100 && s_tx_busy > 0; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));      /* 等在途发送把 client 还回来才能 destroy */
    }
    /* destroy 内部会在任务还没停时先 stop_wait_task，对"启动失败"的句柄也安全，
       比先 stop 再 destroy 少一次无意义的 "Client was not started" 警告。 */
    esp_websocket_client_destroy(cli);
}

/** supervisor：负责建连、认证等待、退避与重建。 */
static void ha_ws_task(void *arg)
{
    (void)arg;
    int backoff_s = HA_WS_BACKOFF_MIN_S;
    int fail_streak = 0;
    bool ever_authed = false;
    bool waiting_wifi = false;
    int64_t last_summary = now_us();

    ESP_LOGI(TAG, "WebSocket 任务启动（内部 RAM %d KB）",
             (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    while (!s_stop_req) {
        if (!net_wifi_connected()) {
            ha_client_set_online(false);
            if (ha_client_poll_mode() != HA_POLL_NORMAL) {
                ha_client_set_poll_mode(HA_POLL_NORMAL);
            }
            if (!waiting_wifi) {
                waiting_wifi = true;
                ESP_LOGW(TAG, "WiFi 未连接：WebSocket 等待网络就绪");
            }
            wait_with_summary(1000, &last_summary);
            continue;
        }
        waiting_wifi = false;

        if (!app_settings_has_ha()) {
            ESP_LOGW(TAG, "HA 地址/令牌未配置：WebSocket 不启动");
            wait_with_summary(5000, &last_summary);
            continue;
        }

        /* WiFi 晚于开机连上时 main 会跳过 HA 初始化：这里补一次，保证 REST 兜底可用。 */
        if (!ha_client_is_ready()) {
            if (ha_client_init() != ESP_OK) {
                ESP_LOGW(TAG, "HA 客户端初始化失败，5 秒后重试");
                wait_with_summary(5000, &last_summary);
                continue;
            }
            ha_client_start_poll();
            /* 开机时 WiFi 没连上会跳过 main 里的首轮拉取：这里补一次，
               免得 WS 只有"变化事件"、卡片到第一次变化前一直显示 --。 */
            if (ha_client_count() > 0) {
                ha_client_fetch_all();
            }
        }

        char url[HA_WS_URL_MAX];
        bool tls = false;
        if (!build_ws_url(url, sizeof(url), &tls)) {
            ESP_LOGW(TAG, "HA 地址不是 http(s)://，无法推导 WebSocket 地址：%s",
                     app_settings_get()->ha_url);
            wait_with_summary(10000, &last_summary);
            continue;
        }

        bool rest_hold = false;
        if (tls) {
            /* 握手期间先冻结 REST 轮询：否则刚释放的 HTTP 客户端 5 秒后又被重建，
               两条 TLS 会同时抢内部 RAM。认证成功就一直保持暂停；失败则恢复
               （WS 从没连上过时，REST 仍是设备的主状态来源）。 */
            if (ha_client_poll_mode() == HA_POLL_NORMAL) {
                ha_client_set_poll_mode(HA_POLL_PAUSED);
                rest_hold = true;
            }
            ha_client_release_http();
            if (ha_client_http_alive()) {
                /* 轮询/折线图正占着 REST 的 TLS：这轮先不握手，等它结束再试，
                   保证任何时刻只有一条 TLS 活着。 */
                ESP_LOGW(TAG, "REST 客户端仍在使用中，本轮先不连 WebSocket（5 秒后重试）");
                if (rest_hold) {
                    ha_client_set_poll_mode(HA_POLL_NORMAL);
                }
                wait_with_summary(5000, &last_summary);
                continue;
            }
            const uint32_t free_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            if (free_int < HA_WS_TLS_MIN_FREE) {
                ESP_LOGW(TAG, "内部 RAM 不足（%u KB < %u KB），稍后重试",
                         (unsigned)(free_int / 1024), (unsigned)(HA_WS_TLS_MIN_FREE / 1024));
                ha_client_set_poll_mode(HA_POLL_NORMAL);
                wait_with_summary(5000, &last_summary);
                continue;
            }
        }

        s_connected = false;
        s_authed = false;
        s_subscribed = false;
        s_link_down = false;
        s_auth_bad = false;
        s_rx_len = 0;
        s_rx_skip = false;
        s_rx_total = 0;
        s_next_id = 2;
        memset(s_pending, 0, sizeof(s_pending));

        esp_websocket_client_config_t cfg = {
            .uri = url,
            /* 只有 wss 才挂证书包；ws 保持 NULL，省掉 TLS 的全部开销。 */
            .crt_bundle_attach = tls ? esp_crt_bundle_attach : NULL,
            .buffer_size = HA_WS_BUFFER_SIZE,
            .task_stack = HA_WS_CLIENT_STACK,
            .network_timeout_ms = 10000,
            .disable_auto_reconnect = true,      /* 退避由本任务统一管理 */
            .ping_interval_sec = 30,
        };

        esp_websocket_client_handle_t cli = esp_websocket_client_init(&cfg);
        bool authed_this = false;
        bool settings_changed = false;

        if (cli == NULL) {
            ESP_LOGE(TAG, "WebSocket 初始化失败（内部 RAM %u KB）",
                     (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
        } else {
            esp_websocket_register_events(cli, WEBSOCKET_EVENT_ANY, ha_ws_event, NULL);
            if (s_tx_lock != NULL) {
                xSemaphoreTake(s_tx_lock, portMAX_DELAY);
            }
            s_client = cli;
            if (s_tx_lock != NULL) {
                xSemaphoreGive(s_tx_lock);
            }
            s_using_tls = tls;

            if (esp_websocket_client_start(cli) != ESP_OK) {
                ESP_LOGE(TAG, "WebSocket 启动失败");
            } else {
                ESP_LOGI(TAG, "HA WS: %s（%s，内部 RAM %u KB）", url,
                         tls ? "TLS" : "明文，仅限可信内网",
                         (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
                snprintf(s_conn_url, sizeof(s_conn_url), "%s", url);
                snprintf(s_conn_token, sizeof(s_conn_token), "%s", app_settings_get()->ha_token);

                const int64_t t0 = now_us();
                bool policy_applied = false;
                int tick = 0;
                while (!s_stop_req && !s_link_down && !s_auth_bad) {
                    vTaskDelay(pdMS_TO_TICKS(200));

                    if (!net_wifi_connected()) {
                        set_last_err("WiFi 断开");
                        ESP_LOGW(TAG, "WiFi 断开，WebSocket 稍后重连");
                        break;
                    }
                    if (!s_authed && now_us() - t0 > (int64_t)HA_WS_AUTH_TIMEOUT_MS * 1000) {
                        set_last_err("等待 auth_ok 超时");
                        ESP_LOGW(TAG, "等待 auth_ok 超时（%d 秒），重连",
                                 HA_WS_AUTH_TIMEOUT_MS / 1000);
                        break;
                    }
                    if (s_authed && !s_subscribed &&
                        now_us() - t0 > (int64_t)HA_WS_AUTH_TIMEOUT_MS * 1000) {
                        set_last_err("等待订阅结果超时");
                        ESP_LOGW(TAG, "认证成功但等不到 subscribe 结果（%d 秒），重连",
                                 HA_WS_AUTH_TIMEOUT_MS / 1000);
                        break;
                    }
                    if (s_authed && s_subscribed && !policy_applied) {
                        policy_applied = true;
                        backoff_s = HA_WS_BACKOFF_MIN_S;
                        if (tls) {
                            ha_client_set_poll_mode(HA_POLL_PAUSED);
                        } else {
                            ha_client_set_poll_mode(HA_POLL_SLOW);
                        }
                        ESP_LOGI(TAG, "认证并订阅完成：内部 RAM %u KB，REST %s",
                                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                                 tls ? "已释放（全程只保留这一条 TLS）"
                                     : "保留并放慢到 60 秒兜底");
                    }
                    /* 认证期间/之后如果有界面操作，REST 客户端会被按需重建：
                       每 2 秒检查一次并还回去（不用每 200ms 去抢 REST 的锁）。 */
                    tick++;
                    if (s_authed && tls && (tick % 10 == 0) && ha_client_http_alive()) {
                        ha_client_release_http();
                    }
                    if (tick % 10 == 0) {
                        char cur[HA_WS_URL_MAX];
                        bool cur_tls = false;
                        if (!build_ws_url(cur, sizeof(cur), &cur_tls) || cur_tls != tls ||
                            strcmp(cur, s_conn_url) != 0 ||
                            strcmp(app_settings_get()->ha_token, s_conn_token) != 0) {
                            ESP_LOGI(TAG, "HA 地址/令牌已变化：重建 WebSocket（无需重启）");
                            settings_changed = true;
                            break;
                        }
                    }
                    maybe_summary(&last_summary);
                }
                authed_this = s_authed && s_subscribed;
            }
        }

        cleanup_client(cli);

        if (authed_this) {
            fail_streak = 0;
            backoff_s = HA_WS_BACKOFF_MIN_S;
            ever_authed = true;
            ha_client_set_online(false);
            ESP_LOGW(TAG, "WebSocket 断开（已认证过），%d 秒后重连", backoff_s);
        } else {
            fail_streak++;
            s_fail++;
            if (tls && (rest_hold || fail_streak >= HA_WS_REST_RESUME_FAILS)) {
                if (ha_client_poll_mode() != HA_POLL_NORMAL) {
                    if (rest_hold) {
                        ESP_LOGW(TAG, "WS 首次连接未成功：REST 轮询继续（%d 秒），WS 后台重试",
                                 CONFIG_APP_POLL_INTERVAL_S);
                    } else {
                        ESP_LOGW(TAG, "WS 连续 %d 次未连上：REST 轮询接管（%d 秒），WS 继续后台重试",
                                 fail_streak, CONFIG_APP_POLL_INTERVAL_S);
                    }
                    ha_client_set_poll_mode(HA_POLL_NORMAL);
                }
            }
            ESP_LOGW(TAG, "WebSocket 第 %d 次连接失败：%s，%d 秒后重试",
                     fail_streak, s_last_err[0] ? s_last_err : "未知错误", backoff_s);
            backoff_s = (backoff_s * 2 > HA_WS_BACKOFF_MAX_S)
                            ? HA_WS_BACKOFF_MAX_S : backoff_s * 2;
        }

        if (settings_changed && !s_stop_req) {
            backoff_s = HA_WS_BACKOFF_MIN_S;
            continue;                              /* 配置变了：立刻用新地址重连 */
        }
        wait_with_summary((uint32_t)backoff_s * 1000, &last_summary);
    }

    cleanup_client(s_client);
    s_task = NULL;
    ESP_LOGI(TAG, "WebSocket 任务退出（%s）", ever_authed ? "曾经连上过" : "从未连上");
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------------- */
/* 对外接口                                                                   */
/* ------------------------------------------------------------------------- */

bool ha_ws_start(void)
{
    if (s_task != NULL) {
        return true;
    }
    if (s_tx_lock == NULL) {
        s_tx_lock = xSemaphoreCreateMutex();
        if (s_tx_lock == NULL) {
            ESP_LOGE(TAG, "WebSocket 互斥量创建失败");
            return false;
        }
    }
    s_stop_req = false;
    if (xTaskCreate(ha_ws_task, "ha_ws", HA_WS_TASK_STACK, NULL, 4, &s_task) != pdPASS) {
        s_task = NULL;
        ESP_LOGE(TAG, "WebSocket 任务创建失败");
        return false;
    }
    return true;
}

void ha_ws_stop(void)
{
    if (s_task == NULL) {
        return;
    }
    s_stop_req = true;
    for (int i = 0; i < 60 && s_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (s_task != NULL) {
        ESP_LOGW(TAG, "WebSocket 任务未在 6 秒内退出");
    }
    /* 停止后别再让 REST 轮询保持暂停状态。 */
    if (ha_client_poll_mode() != HA_POLL_NORMAL) {
        ha_client_set_poll_mode(HA_POLL_NORMAL);
    }
}

bool ha_ws_connected(void)
{
    /* 订阅成功才算"可用"：没有 state_changed 就只剩半条链路，
       此时应让 ha_client 继续用 REST 刷新，而不是把界面卡在旧状态。 */
    return s_authed && s_subscribed && s_connected && !s_stop_req;
}

bool ha_ws_call_service(const char *domain, const char *service,
                        const char *entity_id, const char *service_data_json)
{
    if (domain == NULL || domain[0] == '\0' || service == NULL || service[0] == '\0' ||
        entity_id == NULL || entity_id[0] == '\0') {
        return false;
    }

    esp_websocket_client_handle_t cli = ws_acquire();
    if (cli == NULL) {
        return false;                      /* 没认证：调用者会退回 REST */
    }

    int id = 0;
    if (s_tx_lock != NULL) {
        xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    }
    id = s_next_id++;
    if (s_next_id < 2) {
        s_next_id = 2;
    }
    if (s_tx_lock != NULL) {
        xSemaphoreGive(s_tx_lock);
    }

    char msg[HA_WS_TX_MAX];
    int n = 0;
    const bool has_data = (service_data_json != NULL && service_data_json[0] == '{' &&
                           service_data_json[1] != '}' && service_data_json[1] != '\0');
    if (has_data) {
        /* attrs 形如 {"brightness_pct":42}：去掉开头的 '{'，它自带的 '}' 正好关 service_data。 */
        n = snprintf(msg, sizeof(msg),
                     "{\"id\":%d,\"type\":\"call_service\",\"domain\":\"%.23s\","
                     "\"service\":\"%.31s\",\"service_data\":{\"entity_id\":\"%.95s\",%.200s}",
                     id, domain, service, entity_id, service_data_json + 1);
    } else {
        n = snprintf(msg, sizeof(msg),
                     "{\"id\":%d,\"type\":\"call_service\",\"domain\":\"%.23s\","
                     "\"service\":\"%.31s\",\"service_data\":{\"entity_id\":\"%.95s\"}}",
                     id, domain, service, entity_id);
    }
    if (n <= 0 || (size_t)n >= sizeof(msg)) {
        ws_release();
        s_fail++;
        set_last_err("service call 参数过长");
        ESP_LOGW(TAG, "service call 参数过长，未发送：%s/%s %s", domain, service, entity_id);
        return false;
    }

    const int64_t t_us = now_us();
    const int sent = esp_websocket_client_send_text(cli, msg, n, pdMS_TO_TICKS(100));
    ws_release();

    if (sent < 0) {
        s_fail++;
        set_last_err("service call 发送失败");
        ESP_LOGW(TAG, "WS 发送失败：%s/%s %s", domain, service, entity_id);
        return false;
    }

    s_cmds++;
    char what[72];
    snprintf(what, sizeof(what), "%.60s/%.10s", entity_id, service);
    pending_add(id, what, t_us);
    /* 拖动滑杆会连续发很多条：发送日志用 DEBUG，结果日志才用 INFO。 */
    ESP_LOGD(TAG, "服务调用 #%d %s/%s %s（WS 已发送，等待 result）",
             id, domain, service, entity_id);
    return true;
}
