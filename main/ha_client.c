/**
 * Home Assistant REST 客户端（通用实体关注列表）。
 *
 *  - 复用同一个 esp_http_client + keep-alive，避免每次 TLS 握手；
 *  - 启动时一次拉 /api/states，把关注列表的名字/状态/单位全部填好；
 *  - 之后按 CONFIG_APP_POLL_INTERVAL_S 逐个刷新；
 *  - 不做完整 JSON 解析：只扫描 entity_id / state / friendly_name / unit 字段。
 */

#include <string.h>
#include <strings.h>    /* strncasecmp：预报 condition 的大小写不敏感匹配 */
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_heap_caps.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "cJSON.h"
#include "app_settings.h"
#include "ha_client.h"
#include "ha_ws.h"

static const char *TAG = "ha";

#define HA_HTTP_TIMEOUT_MS  10000
#define HA_BODY_MAX  (4 * 1024)
#define HA_POLL_WAKE_BIT BIT0
#define HA_POLL_SLOW_INTERVAL_S 60

static esp_http_client_handle_t s_client;
static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_table_lock;   /* entity 表写入互斥（轮询任务 vs WS 任务） */
static char s_base[APP_HA_URL_MAX];
static char s_token[APP_HA_TOKEN_MAX];
static char *s_buf;
static size_t s_buf_len;

static ha_entity_t s_entities[HA_MAX_ENTITIES];
static volatile bool s_online;
static volatile ha_client_poll_mode_t s_poll_mode;
static EventGroupHandle_t s_poll_ev;
static TaskHandle_t s_poll_task;
static int s_fail_count;
static int s_count;

/* ------------------------------------------------------------------------- */
/* HTTP                                                                       */
/* ------------------------------------------------------------------------- */

/* 历史请求临时把响应写到 PSRAM 大缓冲里（复用同一条 TLS 连接，避免再开一条） */
static char *s_sink;
static size_t s_sink_len;
static size_t s_sink_cap;

static esp_err_t http_event_handler(esp_http_client_event_t *ev)
{
    if (ev->event_id == HTTP_EVENT_ON_DATA && s_sink != NULL && ev->data_len > 0) {
        const size_t n = (size_t)ev->data_len;
        if (s_sink_len + n < s_sink_cap) {
            memcpy(s_sink + s_sink_len, ev->data, n);
            s_sink_len += n;
            s_sink[s_sink_len] = '\0';
        }
        return ESP_OK;
    }

    if (ev->event_id == HTTP_EVENT_ON_DATA && ev->data && ev->data_len > 0) {
        size_t room = HA_BODY_MAX - 1 - s_buf_len;
        size_t n = (size_t)ev->data_len < room ? (size_t)ev->data_len : room;
        if (n > 0) {
            memcpy(s_buf + s_buf_len, ev->data, n);
            s_buf_len += n;
            s_buf[s_buf_len] = '\0';
        }
    }
    return ESP_OK;
}

static esp_err_t client_create(void)
{
    if (s_client) {
        return ESP_OK;
    }
    const esp_http_client_config_t cfg = {
        .url = s_base,
        .timeout_ms = HA_HTTP_TIMEOUT_MS,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 2048,
        .buffer_size_tx = 512,
        .keep_alive_enable = true,
        .event_handler = http_event_handler,
    };
    s_client = esp_http_client_init(&cfg);
    return s_client ? ESP_OK : ESP_ERR_NO_MEM;
}

static void set_auth_header(void)
{
    static char auth[APP_HA_TOKEN_MAX + 16];
    snprintf(auth, sizeof(auth), "Bearer %s", s_token);
    esp_http_client_set_header(s_client, "Authorization", auth);
    esp_http_client_set_header(s_client, "Accept", "application/json");
}

/** 调用者必须已持有 s_lock：关掉长连接并释放客户端（连同 TLS/socket 内存）。 */
static void release_http_locked(void)
{
    if (s_client == NULL) {
        return;
    }
    /* IDF 只有 cleanup()，没有 destroy()；先 close 再 cleanup，句柄立刻置 NULL，
       绝不能对同一个句柄 cleanup 两次（undefined）。 */
    esp_http_client_close(s_client);
    esp_http_client_cleanup(s_client);
    s_client = NULL;
}

void ha_client_release_http(void)
{
    if (s_lock == NULL) {
        return;
    }
    /* 轮询/历史请求可能正握着锁跑一次 HTTPS（最长 10 秒）：等它结束再释放，
       宁可晚一点起 WebSocket，也不让两条 TLS 同时抢内部 RAM。 */
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGW(TAG, "释放 HTTP 客户端超时（有请求在跑），本次跳过");
        return;
    }
    const bool had = (s_client != NULL);
    release_http_locked();
    xSemaphoreGive(s_lock);
    if (had) {
        ESP_LOGI(TAG, "已释放常驻 HTTP/TLS 客户端，内部 RAM %d KB",
                 (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    }
}

bool ha_client_http_alive(void)
{
    if (s_lock == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return true;       /* 有请求在跑，先按"还在"处理 */
    }
    const bool alive = (s_client != NULL);
    xSemaphoreGive(s_lock);
    return alive;
}

static int http_get(const char *url)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (client_create() != ESP_OK) {
        xSemaphoreGive(s_lock);
        return -1;
    }
    set_auth_header();
    esp_http_client_set_method(s_client, HTTP_METHOD_GET);
    esp_http_client_set_url(s_client, url);
    s_buf_len = 0;
    s_buf[0] = '\0';

    int status = -1;
    esp_err_t err = esp_http_client_perform(s_client);
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(s_client);
    } else {
        ESP_LOGW(TAG, "GET %s failed: %s (internal heap %d KB)", url, esp_err_to_name(err),
                 (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
        /* 长连接被服务端/NAT 掐断后复用会一直失败：关掉，下次重新握手 */
        esp_http_client_close(s_client);
    }
    xSemaphoreGive(s_lock);
    if (status > 0) {
        s_online = true;
        s_fail_count = 0;
    } else if (++s_fail_count >= 5) {
        s_online = false;
    }
    if (status != 200) {
        ESP_LOGW(TAG, "GET %s -> %d", url, status);
    }
    return status;
}

static int http_post_service_ext(const char *domain, const char *service, const char *entity_id,
                                 const char *attrs_json)
{
    char url[APP_HA_URL_MAX + 64];
    snprintf(url, sizeof(url), "%s/api/services/%s/%s", s_base, domain, service);
    char body[320];
    if (attrs_json != NULL && attrs_json[0] == '{') {
        /* attrs_json 形如 {"brightness_pct":60}：跳过开头的 '{'，它自带结尾的 '}' */
        snprintf(body, sizeof(body), "{\"entity_id\":\"%.95s\",", entity_id);
        strncat(body, attrs_json + 1, sizeof(body) - strlen(body) - 1);
    } else if (attrs_json != NULL && attrs_json[0] != '\0') {
        snprintf(body, sizeof(body), "{\"entity_id\":\"%.95s\",%.200s}", entity_id, attrs_json + 1);
    } else {
        snprintf(body, sizeof(body), "{\"entity_id\":\"%.95s\"}", entity_id);
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (client_create() != ESP_OK) {
        xSemaphoreGive(s_lock);
        return -1;
    }
    set_auth_header();
    esp_http_client_set_header(s_client, "Content-Type", "application/json");
    esp_http_client_set_method(s_client, HTTP_METHOD_POST);
    esp_http_client_set_url(s_client, url);
    esp_http_client_set_post_field(s_client, body, (int)strlen(body));
    ESP_LOGD(TAG, "POST body(%d): %s", (int)strlen(body), body);

    int status = -1;
    esp_err_t err = esp_http_client_perform(s_client);
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(s_client);
    } else {
        ESP_LOGW(TAG, "GET %s failed: %s (internal heap %d KB)", url, esp_err_to_name(err),
                 (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    }
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "%s/%s %s -> %d", domain, service, entity_id, status);
    return status;
}

static int http_post_service(const char *domain, const char *service, const char *entity_id)
{
    return http_post_service_ext(domain, service, entity_id, NULL);
}

/* ------------------------------------------------------------------------- */
/* 解析工具                                                                    */
/* ------------------------------------------------------------------------- */

static void str_copy(char *dst, size_t dst_len, const char *src)
{
    size_t n = strlen(src);
    if (n >= dst_len) {
        n = dst_len - 1;
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
}

/** 在 [start,end) 范围内找子串 needle。 */
static bool range_contains(const char *start, const char *end, const char *needle)
{
    if (start == NULL) {
        return false;
    }
    if (end == NULL) {
        end = start + strlen(start);
    }
    const size_t nlen = strlen(needle);
    for (const char *p = start; p + nlen <= end; p++) {
        if (memcmp(p, needle, nlen) == 0) {
            return true;
        }
    }
    return false;
}

/** 在 [start,end) 里找 "key": number，返回是否找到。 */
static bool json_int_in(const char *start, const char *end, const char *key, int *out)
{
    if (start == NULL) {
        return false;
    }
    if (end == NULL) {
        end = start + strlen(start);
    }
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(start, pattern);
    if (p == NULL || p >= end) {
        return false;
    }
    p += strlen(pattern);
    while (p < end && (*p == ' ' || *p == '\t')) {
        p++;
    }
    char *endp = NULL;
    const long v = strtol(p, &endp, 10);
    if (endp == p) {
        return false;
    }
    *out = (int)v;
    return true;
}

/** 找 "key": true / false（JSON 布尔）。*/
static bool json_bool_in(const char *start, const char *end, const char *key, bool *out)
{
    if (start == NULL || out == NULL) {
        return false;
    }
    if (end == NULL) {
        end = start + strlen(start);
    }
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(start, pattern);
    if (p == NULL || p >= end) {
        return false;
    }
    p += strlen(pattern);
    while (p < end && (*p == ' ' || *p == '\t')) {
        p++;
    }
    if (p + 4 <= end && memcmp(p, "true", 4) == 0) {
        *out = true;
        return true;
    }
    if (p + 5 <= end && memcmp(p, "false", 5) == 0) {
        *out = false;
        return true;
    }
    return false;
}

/** 找 "key": 1.23（浮点） */
static bool json_float_in(const char *start, const char *end, const char *key, float *out)
{
    if (start == NULL) {
        return false;
    }
    if (end == NULL) {
        end = start + strlen(start);
    }
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(start, pattern);
    if (p == NULL || p >= end) {
        return false;
    }
    p += strlen(pattern);
    while (p < end && (*p == ' ' || *p == '\t')) {
        p++;
    }
    char *endp = NULL;
    const double v = strtod(p, &endp);
    if (endp == p) {
        return false;
    }
    *out = (float)v;
    return true;
}

/**
 * 找 "key": ["a","b",...]，把每一项塞进定长二维数组（每项最多 HA_MODE_LEN-1 字符）。
 * 返回个数。空调的 hvac_modes / fan_modes 用它：按"HA 报上来的顺序"存，
 * 界面自己再按固定顺序过滤，不需要字符串拼接，也不用二次 strtok。
 */
static int json_str_list_in(const char *start, const char *end, const char *key,
                            char (*out)[HA_MODE_LEN], int max_n)
{
    if (start == NULL || out == NULL || max_n <= 0) {
        return 0;
    }
    if (end == NULL) {
        end = start + strlen(start);
    }
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":[", key);
    const char *p = strstr(start, pattern);
    if (p == NULL || p >= end) {
        return 0;
    }
    p += strlen(pattern);
    int n = 0;
    while (p < end && *p != ']' && n < max_n) {
        if (*p == '"') {
            const char *q = strchr(p + 1, '"');
            if (q == NULL || q >= end) {
                break;
            }
            size_t len = (size_t)(q - p - 1);
            if (len > HA_MODE_LEN - 1) {
                len = HA_MODE_LEN - 1;
            }
            memcpy(out[n], p + 1, len);
            out[n][len] = '\0';
            n++;
            p = q + 1;
        } else {
            p++;
        }
    }
    return n;
}

static bool json_str_in(const char *start, const char *end, const char *key, char *out, size_t out_len)
{
    if (start == NULL) {
        return false;
    }
    if (end == NULL) {
        end = start + strlen(start);
    }
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char *p = strstr(start, pattern);
    if (p == NULL || (end != NULL && p >= end)) {
        out[0] = '\0';
        return false;
    }
    p += strlen(pattern);
    size_t n = 0;
    while (*p != '\0' && *p != '"' && (end == NULL || p < end) && n + 1 < out_len) {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return n > 0;
}

/* ------------------------------------------------------------------------- */
/* 关注列表                                                                    */
/* ------------------------------------------------------------------------- */

esp_err_t ha_client_register(const char *entity_id, const char *label)
{
    if (entity_id == NULL || entity_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < s_count; i++) {
        if (strcmp(s_entities[i].entity_id, entity_id) == 0) {
            if (label && label[0] && !s_entities[i].name[0]) {
                str_copy(s_entities[i].name, sizeof(s_entities[i].name), label);
            }
            return ESP_OK;
        }
    }
    if (s_count >= HA_MAX_ENTITIES) {
        ESP_LOGW(TAG, "entity list full (%d)", HA_MAX_ENTITIES);
        return ESP_ERR_NO_MEM;
    }

    ha_entity_t *e = &s_entities[s_count];
    memset(e, 0, sizeof(*e));
    str_copy(e->entity_id, sizeof(e->entity_id), entity_id);
    if (label && label[0] != '\0') {
        str_copy(e->name, sizeof(e->name), label);
    }
    e->valid = true;
    e->fresh = false;
    e->temp_step = 1.0f;      /* 空调每格默认 1°C；HA 报 target_temp_step 后按实体改 */
    strcpy(e->state, "?");
    s_count++;
    return ESP_OK;
}

int ha_client_count(void)
{
    return s_count;
}

const ha_entity_t *ha_client_at(int idx)
{
    if (idx < 0 || idx >= s_count) {
        return NULL;
    }
    return &s_entities[idx];
}

const ha_entity_t *ha_client_find(const char *entity_id)
{
    if (entity_id == NULL) {
        return NULL;
    }
    for (int i = 0; i < s_count; i++) {
        if (strcmp(s_entities[i].entity_id, entity_id) == 0) {
            return &s_entities[i];
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* ------------------------------------------------------------------------- */

esp_err_t ha_client_init(void)
{
    const app_settings_t *cfg = app_settings_get();
    if (!app_settings_has_ha()) {
        ESP_LOGE(TAG, "HA url/token missing");
        return ESP_ERR_INVALID_STATE;
    }
    snprintf(s_base, sizeof(s_base), "%s", cfg->ha_url);
    snprintf(s_token, sizeof(s_token), "%s", cfg->ha_token);

    if (s_buf == NULL) {
        s_buf = malloc(HA_BODY_MAX);
        if (s_buf == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
        s_poll_mode = HA_POLL_NORMAL;
    }
    if (s_table_lock == NULL) {
        s_table_lock = xSemaphoreCreateMutex();
        if (s_table_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    ESP_LOGI(TAG, "HA client ready: %s", s_base);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const esp_err_t err = client_create();
    xSemaphoreGive(s_lock);
    return err;
}

/* ------------------------------------------------------------------------- */
/* 全量刷新                                                                    */
/* ------------------------------------------------------------------------- */

/** preset_modes 的固定清单：位图下标 i 对应 k_preset_names[i]（和 ha_client.h 的 HA_PRESET_* 对齐）。*/
static const char *const k_preset_names[HA_PRESET_COUNT] = {
    "eco", "away", "boost", "none", "sleep",
};

/** 把 attributes（name/unit/灯/空调字段）合并进实体；state 本身由调用者处理。 */
static void apply_attributes(ha_entity_t *e, const char *obj_start, const char *obj_end)
{
    if (e->name[0] == '\0') {
        char fn[48] = {0};
        if (json_str_in(obj_start, obj_end, "friendly_name", fn, sizeof(fn))) {
            str_copy(e->name, sizeof(e->name), fn);
        } else {
            const char *dot = strchr(e->entity_id, '.');
            const char *obj = dot ? dot + 1 : e->entity_id;
            str_copy(e->name, sizeof(e->name), obj);
        }
    }
    char unit[12] = {0};
    if (json_str_in(obj_start, obj_end, "unit_of_measurement", unit, sizeof(unit))) {
        str_copy(e->unit, sizeof(e->unit), unit);
    }

    /* 灯的属性（没有就是 0 / false） */
    int v = 0;
    const bool is_light = (strncmp(e->entity_id, "light.", 6) == 0);
    const bool has_br_attr = json_int_in(obj_start, obj_end, "brightness", &v);
    if (has_br_attr && v > 0) {
        e->brightness = v;
        e->seen_brightness = true;
    } else if (is_light && !e->seen_brightness) {
        e->brightness = 255;      /* 从没拿到过真实值，先按满亮度显示 */
    }
    /* 灯关着时 HA 不返回 brightness：沿用上一次的值，别重置成 100% */
    e->has_brightness = has_br_attr || is_light;

    const bool has_ct_attr = json_int_in(obj_start, obj_end, "color_temp_kelvin", &v);
    if (has_ct_attr) {
        e->color_temp_k = v;
        e->seen_tint = true;
    }
    /* 灯关着时 color_temp_kelvin 也会缺失，但 supported_color_modes 能说明它支持色温 */
    e->has_tint = has_ct_attr ||
                  range_contains(obj_start, obj_end, "\"color_temp\"") ||
                  range_contains(obj_start, obj_end, "min_color_temp_kelvin");
    if (json_int_in(obj_start, obj_end, "min_color_temp_kelvin", &v)) {
        e->min_k = v;
    }
    if (json_int_in(obj_start, obj_end, "max_color_temp_kelvin", &v)) {
        e->max_k = v;
    }
    if (e->min_k == 0) {
        e->min_k = 2700;
    }
    if (e->max_k == 0) {
        e->max_k = 6500;
    }

    /* 空调 */
    if (strncmp(e->entity_id, "climate.", 8) == 0) {
        float fv = 0;
        if (json_float_in(obj_start, obj_end, "current_temperature", &fv)) {
            e->cur_temp = fv;
        }
        if (json_float_in(obj_start, obj_end, "temperature", &fv)) {
            e->target_temp = fv;
        }
        if (json_float_in(obj_start, obj_end, "min_temp", &fv)) {
            e->min_temp = fv;
        }
        if (json_float_in(obj_start, obj_end, "max_temp", &fv)) {
            e->max_temp = fv;
        }
        /* target_temp_step：这台机器每格多少（0.5 还是 1）。只在 HA 给出合法值时改，
           否则保留上一次的值——有的模式下 HA 会漏掉这个字段，不能因此把 0.5 抹成 1。 */
        if (json_float_in(obj_start, obj_end, "target_temp_step", &fv) &&
            fv >= 0.05f && fv <= 5.0f) {
            e->temp_step = fv;
        }
        if (e->temp_step < 0.05f) {
            e->temp_step = 1.0f;              /* 从没报过：按 1 */
        }
        if (e->min_temp < 5.0f) {
            e->min_temp = 16.0f;
        }
        if (e->max_temp < 10.0f) {
            e->max_temp = 30.0f;
        }
        if (e->max_temp <= e->min_temp) {
            e->max_temp = e->min_temp + 1.0f; /* 脏数据兜底：至少留一格 */
        }
        json_str_in(obj_start, obj_end, "fan_mode", e->fan_mode, sizeof(e->fan_mode));
        json_str_in(obj_start, obj_end, "swing_mode", e->swing_mode, sizeof(e->swing_mode));
        /* 支持列表里有没有 full_swing：决定空调页显不显示"摆风"胶囊 */
        if (range_contains(obj_start, obj_end, "\"full_swing\"")) {
            e->has_full_swing = true;
        }
        const int nh = json_str_list_in(obj_start, obj_end, "hvac_modes",
                                        e->hvac_modes, HA_MODE_MAX);
        if (nh > 0) {
            e->hvac_mode_count = (uint8_t)nh;
        }
        const int nf = json_str_list_in(obj_start, obj_end, "fan_modes",
                                        e->fan_modes, HA_MODE_MAX);
        if (nf > 0) {
            e->fan_mode_count = (uint8_t)nf;
        }
        /* preset_modes 界面暂时不显示，只记能力位（eco/away/boost/none/sleep）。 */
        char pm[HA_MODE_MAX][HA_MODE_LEN];
        const int np = json_str_list_in(obj_start, obj_end, "preset_modes", pm, HA_MODE_MAX);
        uint8_t mask = 0;
        for (int i = 0; i < np; i++) {
            for (int k = 0; k < HA_PRESET_COUNT; k++) {
                if (strcmp(pm[i], k_preset_names[k]) == 0) {
                    mask = (uint8_t)(mask | (1u << k));
                }
            }
        }
        e->preset_mask = mask;
    }

    /* 风扇：percentage / preset_mode / preset_modes / oscillating。
       HA 在风扇关着时可能不报 percentage / preset_mode，所以只在新值合法时覆盖，
       别在关机那一帧把页面上的数字清成 0 / 空。 */
    if (strncmp(e->entity_id, "fan.", 4) == 0) {
        int pv = 0;
        if (json_int_in(obj_start, obj_end, "percentage", &pv) && pv >= 0 && pv <= 100) {
            e->percentage = pv;
            e->has_percentage = true;
        }
        char pm[16] = {0};
        if (json_str_in(obj_start, obj_end, "preset_mode", pm, sizeof(pm))) {
            str_copy(e->preset_mode, sizeof(e->preset_mode), pm);
        }
        bool osc = false;
        if (json_bool_in(obj_start, obj_end, "oscillating", &osc)) {
            e->oscillating = osc;
        }
        /* 列表按 HA 报上来的顺序存（和空调的 hvac_modes 同一套定长数组），
           界面按固定顺序挑自己认识的那 4 个，命令里回传这里的原样字符串。 */
        const int np = json_str_list_in(obj_start, obj_end, "preset_modes",
                                        e->preset_modes, HA_FAN_PRESET_MAX);
        if (np > 0) {
            e->preset_mode_count = (uint8_t)np;
        }
    }

    /* 天气：实况属性（预报不在这里，见 ha_client_weather_forecast）。
       温度类存 ×10 整数，湿度/云量/风速/能见度按整数存，浮点只在这里出现一次。 */
    if (strncmp(e->entity_id, "weather.", 8) == 0) {
        float fv = 0;
        if (json_float_in(obj_start, obj_end, "temperature", &fv)) {
            e->wx_temp10 = (int16_t)lroundf(fv * 10.0f);
            e->wx_valid = 1;
        }
        if (json_float_in(obj_start, obj_end, "apparent_temperature", &fv)) {
            e->wx_feel10 = (int16_t)lroundf(fv * 10.0f);
        }
        if (json_float_in(obj_start, obj_end, "dew_point", &fv)) {
            e->wx_dew10 = (int16_t)lroundf(fv * 10.0f);
        }
        if (json_float_in(obj_start, obj_end, "humidity", &fv)) {
            e->wx_humidity = (uint8_t)(fv < 0 ? 0 : (fv > 100 ? 100 : (int)lroundf(fv)));
        }
        if (json_float_in(obj_start, obj_end, "cloud_coverage", &fv)) {
            e->wx_cloud = (uint8_t)(fv < 0 ? 0 : (fv > 100 ? 100 : (int)lroundf(fv)));
        }
        if (json_float_in(obj_start, obj_end, "pressure", &fv)) {
            e->wx_press = (int16_t)lroundf(fv);
        }
        if (json_float_in(obj_start, obj_end, "wind_speed", &fv)) {
            e->wx_wind_kmh = (uint8_t)(fv < 0 ? 0 : (fv > 255 ? 255 : (int)lroundf(fv)));
        }
        if (json_float_in(obj_start, obj_end, "visibility", &fv)) {
            e->wx_vis_km = (uint8_t)(fv < 0 ? 0 : (fv > 255 ? 255 : (int)lroundf(fv)));
        }
        json_str_in(obj_start, obj_end, "wind_bearing", e->wx_wind_bearing,
                    sizeof(e->wx_wind_bearing));
        /* 实况的 condition 就是 state（weather 实体的 state 本身是机器码），
           界面用 ha_wx_kind_of(e->state) 取图标，不在这里重复存一份。 */
    }
}

static void parse_entity_object(ha_entity_t *e, const char *obj_start, const char *obj_end)
{
    char state[24] = {0};
    if (json_str_in(obj_start, obj_end, "state", state, sizeof(state))) {
        str_copy(e->state, sizeof(e->state), state);
        e->on = (strcmp(state, "on") == 0);
        e->fresh = true;
    }
    apply_attributes(e, obj_start, obj_end);
}

esp_err_t ha_client_fetch_all(void)
{
    /* 只取关注的那几个实体：/api/states 在实体多的 HA 上可能有近 1MB，
       没必要为 5 个实体拉一整包（以前就是这么被截断才导致 HA 红点）。 */
    int ok = 0;
    for (int i = 0; i < s_count; i++) {
        if (ha_client_refresh_at(i) == ESP_OK) {
            ok++;
        }
    }
    ESP_LOGI(TAG, "refreshed %d/%d entity(ies)", ok, s_count);
    return (s_count == 0 || ok > 0) ? ESP_OK : ESP_FAIL;
}

static esp_err_t refresh_one(int idx)
{
    if (idx < 0 || idx >= s_count) {
        return ESP_ERR_INVALID_ARG;
    }
    ha_entity_t *e = &s_entities[idx];

    char url[APP_HA_URL_MAX + 160];
    snprintf(url, sizeof(url), "%s/api/states/%.95s", s_base, e->entity_id);
    if (http_get(url) != 200) {
        e->fresh = false;
        return ESP_FAIL;
    }
    /* 轮询任务和 WS 任务都可能写表：解析这一段用表锁串起来（很短，纯 CPU）。 */
    if (s_table_lock) {
        xSemaphoreTake(s_table_lock, portMAX_DELAY);
    }
    parse_entity_object(e, s_buf, NULL);
    if (s_table_lock) {
        xSemaphoreGive(s_table_lock);
    }
    return ESP_OK;
}

esp_err_t ha_client_refresh_at(int idx)
{
    if (idx < 0 || idx >= s_count) {
        return ESP_ERR_INVALID_ARG;
    }
    /* WS 已认证时状态由事件推送，界面上"操作后补一次 REST"就没有意义了：
       直接返回，省掉一次 TCP/TLS 往返（wss 场景下还会多占一份 TLS 内存）。 */
    if (ha_ws_connected()) {
        return ESP_OK;
    }
    return refresh_one(idx);
}

bool ha_client_apply_state(const char *entity_id, const char *state, const char *attrs_json)
{
    if (entity_id == NULL || entity_id[0] == '\0' || state == NULL) {
        return false;
    }
    ha_entity_t *e = NULL;
    for (int i = 0; i < s_count; i++) {
        if (strcmp(s_entities[i].entity_id, entity_id) == 0) {
            e = &s_entities[i];
            break;
        }
    }
    if (e == NULL) {
        return false;          /* 不在关注列表里，不占内存也不刷界面 */
    }

    if (s_table_lock) {
        xSemaphoreTake(s_table_lock, portMAX_DELAY);
    }
    str_copy(e->state, sizeof(e->state), state);
    e->on = (strcmp(state, "on") == 0);
    e->fresh = true;
    if (attrs_json != NULL && attrs_json[0] == '{') {
        apply_attributes(e, attrs_json, NULL);
    }
    if (s_table_lock) {
        xSemaphoreGive(s_table_lock);
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* 动作                                                                       */
/* ------------------------------------------------------------------------- */

/** toggle/turn_on/turn_off 成功后先改本地状态，界面不必等下一帧事件/轮询。 */
static void optimistic_update(const char *entity_id, const char *service)
{
    const ha_entity_t *e = ha_client_find(entity_id);
    if (e == NULL) {
        return;
    }
    if (s_table_lock) {
        xSemaphoreTake(s_table_lock, portMAX_DELAY);
    }
    ha_entity_t *m = (ha_entity_t *)e;
    if (strcmp(service, "toggle") == 0) {
        m->on = !m->on;
        str_copy(m->state, sizeof(m->state), m->on ? "on" : "off");
    } else if (strcmp(service, "turn_on") == 0) {
        m->on = true;
        str_copy(m->state, sizeof(m->state), "on");
    } else if (strcmp(service, "turn_off") == 0) {
        m->on = false;
        str_copy(m->state, sizeof(m->state), "off");
    }
    if (s_table_lock) {
        xSemaphoreGive(s_table_lock);
    }
}

esp_err_t ha_client_action_attrs(const char *entity_id, const char *service, const char *attrs_json)
{
    if (entity_id == NULL || service == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    char domain[24] = {0};
    const char *dot = strchr(entity_id, '.');
    size_t dlen = dot ? (size_t)(dot - entity_id) : 0;
    if (dlen == 0 || dlen >= sizeof(domain)) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(domain, entity_id, dlen);

    /* WebSocket 已认证：走快路径（一次 WS 文本帧，约 20-25ms 往返）。
       发送失败时也不补 REST：无法确认服务端是否已经执行，补发会让 toggle 类命令执行两次。 */
    if (ha_ws_connected()) {
        if (ha_ws_call_service(domain, service, entity_id, attrs_json)) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "WS 发送失败：%s/%s %s %s（不补 REST，避免重复执行）",
                 domain, service, entity_id, attrs_json ? attrs_json : "");
        return ESP_FAIL;
    }

    const int status = http_post_service_ext(domain, service, entity_id, attrs_json);
    ESP_LOGI(TAG, "%s/%s %s %s -> %d", domain, service, entity_id,
             attrs_json ? attrs_json : "", status);
    return (status >= 200 && status < 300) ? ESP_OK : ESP_FAIL;
}

esp_err_t ha_client_action(const char *entity_id, const char *service)
{
    if (entity_id == NULL || service == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    char domain[24] = {0};
    const char *dot = strchr(entity_id, '.');
    size_t dlen = dot ? (size_t)(dot - entity_id) : 0;
    if (dlen == 0 || dlen >= sizeof(domain)) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(domain, entity_id, dlen);

    if (ha_ws_connected()) {
        if (!ha_ws_call_service(domain, service, entity_id, NULL)) {
            ESP_LOGW(TAG, "WS 发送失败：%s/%s %s（不补 REST，避免 toggle 执行两次）",
                     domain, service, entity_id);
            return ESP_FAIL;
        }
        optimistic_update(entity_id, service);
        return ESP_OK;
    }

    if (http_post_service(domain, service, entity_id) >= 300) {
        return ESP_FAIL;
    }
    optimistic_update(entity_id, service);
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */
/* 轮询                                                                       */
/* ------------------------------------------------------------------------- */

/* ---------------- 历史数据（折线图） ---------------- */

#define HIST_MAX_BYTES   (32 * 1024)

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
} hist_ctx_t;

static esp_err_t hist_event(esp_http_client_event_t *ev)
{
    hist_ctx_t *ctx = (hist_ctx_t *)ev->user_data;
    if (ev->event_id == HTTP_EVENT_ON_DATA && ctx != NULL && ev->data_len > 0) {
        const size_t n = (size_t)ev->data_len;
        if (ctx->len + n < ctx->cap) {
            memcpy(ctx->buf + ctx->len, ev->data, n);
            ctx->len += n;
            ctx->buf[ctx->len] = '\0';
        }
    }
    return ESP_OK;
}

static esp_err_t history_impl(const char *entity_id, int hours, float *values, int max_points,
                              int *out_count, float *out_min, float *out_max, float *out_avg)
{
    if (entity_id == NULL || values == NULL || max_points <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_base[0] == '\0') {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "history start: %s", entity_id);

    char *buf = heap_caps_malloc(HIST_MAX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    buf[0] = '\0';

    const time_t now = time(NULL);
    const time_t start = now - (time_t)hours * 3600;
    struct tm tm_start;
    gmtime_r(&start, &tm_start);
    char iso[40];
    snprintf(iso, sizeof(iso), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             tm_start.tm_year + 1900, tm_start.tm_mon + 1, tm_start.tm_mday,
             tm_start.tm_hour, tm_start.tm_min, tm_start.tm_sec);

    char url[APP_HA_URL_MAX + 320];
    snprintf(url, sizeof(url),
             "%s/api/history/period/%s?filter_entity_id=%.95s&minimal_response&no_attributes"
             "&significant_changes_only",
             s_base, iso, entity_id);

    /* 复用常驻客户端（同一条 TLS 连接），只把响应写进 PSRAM 大缓冲：
       再开一条 TLS 会多占 16KB 内部 RAM，之前就是这里失败的 */
    if (s_poll_mode == HA_POLL_PAUSED &&
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < 50 * 1024) {
        ESP_LOGW(TAG, "history: 内部 RAM 不足（%d KB），跳过折线图以保护 WS 连接",
                 (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
        free(buf);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t herr = ESP_FAIL;
    int hstatus = -1;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGW(TAG, "history: 等锁超时，放弃");
            free(buf);
            return ESP_ERR_TIMEOUT;
        }
        if (client_create() != ESP_OK) {
            xSemaphoreGive(s_lock);
            free(buf);
            return ESP_FAIL;
        }
        set_auth_header();
        esp_http_client_set_method(s_client, HTTP_METHOD_GET);
        esp_http_client_set_url(s_client, url);
        s_sink = buf;
        s_sink_len = 0;
        s_sink_cap = HIST_MAX_BYTES - 1;
        buf[0] = '\0';
        herr = esp_http_client_perform(s_client);
        hstatus = esp_http_client_get_status_code(s_client);
        s_sink = NULL;
        if (herr != ESP_OK) {
            esp_http_client_close(s_client);   /* 长连接坏了：关掉重来 */
        }
        xSemaphoreGive(s_lock);
        if (herr == ESP_OK && hstatus == 200) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    if (herr != ESP_OK || hstatus != 200) {
        ESP_LOGW(TAG, "history %s -> %s, HTTP %d", entity_id, esp_err_to_name(herr), hstatus);
        if (herr != ESP_OK) {
            esp_http_client_close(s_client);
        }
        free(buf);
        return ESP_FAIL;
    }

    /* 不解析成对象树：直接在文本里扫 "state":"..."，避免 cJSON 把内部 RAM 撑爆 */
    float *raw = heap_caps_malloc(sizeof(float) * 1024, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (raw == NULL) {
        free(buf);
        return ESP_ERR_NO_MEM;
    }
    int raw_n = 0;
    const char *p = buf;
    while (raw_n < 1024) {
        p = strstr(p, "\"state\":\"");
        if (p == NULL) {
            break;
        }
        p += strlen("\"state\":\"");
        char *endp = NULL;
        const double v = strtod(p, &endp);
        const char *quote = strchr(p, '"');
        if (endp != p && quote != NULL && endp <= quote && isfinite(v) &&
            v > -1.0e6 && v < 1.0e6) {
            raw[raw_n++] = (float)v;      /* 只收数字型状态，unknown/on/off 自动跳过 */
        }
        p = quote ? quote : p;
    }

    int out_n = 0;
    float vmin = 0, vmax = 0, vsum = 0;
    if (raw_n > 0) {
        const int buckets = (raw_n < max_points) ? raw_n : max_points;
        for (int b = 0; b < buckets; b++) {
            const int from = (int)((long)b * raw_n / buckets);
            const int to = (int)((long)(b + 1) * raw_n / buckets);
            float acc = 0;
            int cnt = 0;
            for (int i = from; i < to; i++) {
                acc += raw[i];
                cnt++;
            }
            if (cnt == 0) {
                continue;
            }
            const float avg = acc / cnt;
            values[out_n++] = avg;
            vsum += avg;
            if (out_n == 1 || avg < vmin) vmin = avg;
            if (out_n == 1 || avg > vmax) vmax = avg;
        }
    }
    free(raw);
    free(buf);

    *out_count = out_n;
    if (out_n > 0) {
        *out_min = vmin;
        *out_max = vmax;
        *out_avg = vsum / out_n;
    }
    ESP_LOGI(TAG, "history %s: %d 点 (原始 %d)", entity_id, out_n, raw_n);
    return out_n > 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t ha_client_history(const char *entity_id, int hours, float *values, int max_points,
                            int *out_count, float *out_min, float *out_max, float *out_avg)
{
    const esp_err_t r = history_impl(entity_id, hours, values, max_points,
                                     out_count, out_min, out_max, out_avg);
    /* wss 已接管时，折线图是少见路径：用完立刻把 REST/TLS 内存还回去，
       否则两条 TLS 会一起常驻，正是要避免的情况。 */
    if (s_poll_mode == HA_POLL_PAUSED) {
        ha_client_release_http();
    }
    return r;
}

/* ------------------------------------------------------------------------- */
/* 天气预报                                                                    */
/* ------------------------------------------------------------------------- */

/* 预报响应体：24 条逐小时约 4~5 KB，放 PSRAM（内部 RAM 还要留给 TLS / LVGL）。 */
#define WX_FORECAST_BUF        (8 * 1024)
#define WX_FORECAST_REFRESH_S  (30 * 60)
#define WX_FORECAST_RETRY_S    (5 * 60)

static ha_wx_forecast_t s_wx_fc;         /* 预报缓存（表锁保护）*/
static int64_t s_wx_next_us;             /* 下次抓取时刻（esp_timer）*/

ha_wx_kind_t ha_wx_kind_of(const char *condition)
{
    static const struct {
        const char *ha;
        ha_wx_kind_t kind;
    } map[] = {
        {"sunny",            HA_WX_SUNNY},
        {"clear-night",      HA_WX_CLEAR_NIGHT},
        {"partlycloudy",     HA_WX_PARTLY},
        {"cloudy",           HA_WX_CLOUDY},
        {"rainy",            HA_WX_RAIN},
        {"pouring",          HA_WX_POURING},
        {"lightning",        HA_WX_LIGHTNING},
        {"lightning-rainy",  HA_WX_LIGHTNING},
        {"snowy",            HA_WX_SNOW},
        {"snowy-rainy",      HA_WX_SNOWY_RAIN},
        {"fog",              HA_WX_FOG},
        {"windy",            HA_WX_WINDY},
        {"windy-variant",    HA_WX_WINDY},
        {"hail",             HA_WX_HAIL},
        {"exceptional",      HA_WX_CLOUDY},
    };
    if (condition == NULL) {
        return HA_WX_CLOUDY;
    }
    while (*condition == ' ' || *condition == '\t') {
        condition++;
    }
    for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
        const size_t n = strlen(map[i].ha);
        if (strncasecmp(condition, map[i].ha, n) == 0 &&
            (condition[n] == '\0' || condition[n] == '"' || condition[n] == ' ')) {
            return map[i].kind;
        }
    }
    return HA_WX_CLOUDY;
}

/** 找 "forecast": [ 里那个 '['，返回 '[' 后面的第一个字符。 */
static const char *wx_forecast_array(const char *buf)
{
    if (buf == NULL) {
        return NULL;
    }
    const char *p = strstr(buf, "\"forecast\"");
    if (p == NULL) {
        return NULL;
    }
    p = strchr(p, '[');
    return (p != NULL) ? p + 1 : NULL;
}

/** 从 "2026-11-15T21:00:00+08:00" 里取小时；失败返回 -1。 */
static int wx_hour_of(const char *iso)
{
    if (iso == NULL) {
        return -1;
    }
    const char *t = strchr(iso, 'T');
    if (t == NULL || t[1] < '0' || t[1] > '9' || t[2] < '0' || t[2] > '9') {
        return -1;
    }
    return (t[1] - '0') * 10 + (t[2] - '0');
}

/** 公历 → 距 1970-01-01 的天数（Howard Hinnant 的 days_from_civil，纯整数）。*/
static int32_t wx_days_from_civil(int y, int m, int d)
{
    y -= (m <= 2);
    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const uint32_t yoe = (uint32_t)(y - era * 400);
    const uint32_t doy = (uint32_t)((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1);
    const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int32_t)doe - 719468;
}

/** 解析一条逐小时预报（start..end 是这个对象的范围）。*/
static void wx_parse_hour(const char *start, const char *end, ha_wx_hour_t *out)
{
    char iso[40] = {0};
    json_str_in(start, end, "datetime", iso, sizeof(iso));
    const int h = wx_hour_of(iso);
    out->hour = (uint8_t)(h < 0 ? 0 : h);
    float fv = 0;
    out->temp = json_float_in(start, end, "temperature", &fv) ? (int8_t)lroundf(fv) : 0;
    out->pop = json_float_in(start, end, "precipitation_probability", &fv)
               ? (uint8_t)(fv < 0 ? 0 : (fv > 100 ? 100 : (int)lroundf(fv))) : 0;
    char cond[24] = {0};
    json_str_in(start, end, "condition", cond, sizeof(cond));
    out->kind = (uint8_t)ha_wx_kind_of(cond);
}

/** 解析一条逐日预报：datetime → 距今天几天 + 星期几，高低温 + condition。*/
static void wx_parse_day(const char *start, const char *end, ha_wx_day_t *out)
{
    char iso[40] = {0};
    json_str_in(start, end, "datetime", iso, sizeof(iso));
    int y = 0, m = 0, d = 0;
    if (sscanf(iso, "%4d-%2d-%2d", &y, &m, &d) == 3) {
        const time_t now = time(NULL);
        struct tm tmv;
        localtime_r(&now, &tmv);
        const int32_t today = wx_days_from_civil(tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday);
        const int32_t that = wx_days_from_civil(y, m, d);
        int32_t off = that - today;
        if (off < 0) {
            off = 0;
        }
        if (off > 9) {
            off = 9;
        }
        out->day_off = (uint8_t)off;
        int32_t wd = (that + 4) % 7;         /* 1970-01-01 是周四 → 4（0=周日）*/
        if (wd < 0) {
            wd += 7;
        }
        out->weekday = (uint8_t)wd;
    } else {
        out->day_off = 0;
        out->weekday = 0;
    }
    float fv = 0;
    out->tmax = json_float_in(start, end, "temperature", &fv) ? (int8_t)lroundf(fv) : 0;
    out->tmin = json_float_in(start, end, "templow", &fv) ? (int8_t)lroundf(fv) : 0;
    char cond[24] = {0};
    json_str_in(start, end, "condition", cond, sizeof(cond));
    out->kind = (uint8_t)ha_wx_kind_of(cond);
}

/** 从数组起点按顺序取最多 max_n 个顶层对象，逐个交给 parse。 */
static int wx_walk_array(const char *arr, int max_n, bool daily, ha_wx_forecast_t *out)
{
    const char *p = arr;
    int n = 0;
    while (p != NULL && *p != '\0' && n < max_n) {
        const char *open = strchr(p, '{');
        if (open == NULL) {
            break;
        }
        const char *close = strchr(open, '}');
        if (close == NULL) {
            break;
        }
        if (daily) {
            wx_parse_day(open, close, &out->day[n]);
        } else {
            wx_parse_hour(open, close, &out->hour[n]);
        }
        n++;
        p = close + 1;
        if (*p == ']') {
            break;
        }
    }
    return n;
}

/** 一条 get_forecasts POST：响应写进 PSRAM 缓冲（复用常驻 TLS 连接）。 */
static esp_err_t wx_post_forecast(const char *entity_id, const char *type, char *buf, size_t cap)
{
    char url[APP_HA_URL_MAX + 96];
    snprintf(url, sizeof(url), "%s/api/services/weather/get_forecasts?return_response=true", s_base);
    char body[160];
    snprintf(body, sizeof(body), "{\"entity_id\":\"%.95s\",\"type\":\"%s\"}", entity_id, type);

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(8000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (client_create() != ESP_OK) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_NO_MEM;
    }
    set_auth_header();
    esp_http_client_set_header(s_client, "Content-Type", "application/json");
    esp_http_client_set_method(s_client, HTTP_METHOD_POST);
    esp_http_client_set_url(s_client, url);
    esp_http_client_set_post_field(s_client, body, (int)strlen(body));
    s_sink = buf;
    s_sink_len = 0;
    s_sink_cap = cap - 1;
    buf[0] = '\0';
    const esp_err_t err = esp_http_client_perform(s_client);
    const int status = esp_http_client_get_status_code(s_client);
    s_sink = NULL;
    if (err != ESP_OK) {
        esp_http_client_close(s_client);       /* 长连接坏了：关掉，下次重新握手 */
    }
    xSemaphoreGive(s_lock);
    if (err != ESP_OK || status != 200) {
        ESP_LOGW(TAG, "预报 %s (%s) -> %s, HTTP %d", entity_id, type, esp_err_to_name(err), status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ha_client_weather_forecast(const char *entity_id)
{
    if (entity_id == NULL || s_base[0] == '\0') {
        return ESP_ERR_INVALID_STATE;
    }
    char *buf = heap_caps_malloc(WX_FORECAST_BUF, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ha_wx_forecast_t fc;
    memset(&fc, 0, sizeof(fc));

    esp_err_t r = wx_post_forecast(entity_id, "hourly", buf, WX_FORECAST_BUF);
    if (r == ESP_OK) {
        fc.hour_n = wx_walk_array(wx_forecast_array(buf), HA_WX_HOURLY, false, &fc);
    }
    r = wx_post_forecast(entity_id, "daily", buf, WX_FORECAST_BUF);
    if (r == ESP_OK) {
        fc.day_n = wx_walk_array(wx_forecast_array(buf), HA_WX_DAILY, true, &fc);
    }
    free(buf);

    if (fc.hour_n <= 0 && fc.day_n <= 0) {
        ESP_LOGW(TAG, "预报解析失败（%s）", entity_id);
        return ESP_FAIL;
    }
    fc.ok = true;
    fc.stamp_s = (uint32_t)time(NULL);

    if (s_table_lock) {
        xSemaphoreTake(s_table_lock, portMAX_DELAY);
    }
    s_wx_fc = fc;
    if (s_table_lock) {
        xSemaphoreGive(s_table_lock);
    }
    ESP_LOGI(TAG, "预报已更新：%d 条逐小时 / %d 条逐日（%s）", fc.hour_n, fc.day_n, entity_id);
    return ESP_OK;
}

bool ha_client_weather_forecast_copy(ha_wx_forecast_t *out)
{
    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (s_table_lock == NULL) {
        return false;
    }
    xSemaphoreTake(s_table_lock, portMAX_DELAY);
    *out = s_wx_fc;
    xSemaphoreGive(s_table_lock);
    return out->ok;
}

/** 轮询任务里每圈调一次：成功 30 分钟一次，失败 5 分钟重试；WS 接管时也不耽误。 */
static void wx_forecast_tick(void)
{
    const int64_t now = esp_timer_get_time();
    if (now < s_wx_next_us) {
        return;
    }
    const char *eid = NULL;
    for (int i = 0; i < s_count; i++) {
        if (strncmp(s_entities[i].entity_id, "weather.", 8) == 0) {
            eid = s_entities[i].entity_id;
            break;
        }
    }
    if (eid == NULL) {
        s_wx_next_us = now + 15LL * 1000000;     /* 布局里还没注册天气实体：15 秒后再看 */
        return;
    }
    /* wss 接管时只剩一条 TLS，内部 RAM 不够就别抢（和折线图同一条规矩）*/
    if (s_poll_mode == HA_POLL_PAUSED &&
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < 50 * 1024) {
        s_wx_next_us = now + 60LL * 1000000;
        return;
    }
    const esp_err_t r = ha_client_weather_forecast(eid);
    s_wx_next_us = now +
                   (int64_t)((r == ESP_OK ? WX_FORECAST_REFRESH_S : WX_FORECAST_RETRY_S)) * 1000000;
    if (s_poll_mode == HA_POLL_PAUSED) {
        ha_client_release_http();                /* 用完立刻把 TLS 内存还回去 */
    }
}

bool ha_client_online(void)
{
    return s_online;
}

void ha_client_set_online(bool online)
{
    s_online = online;
    if (online) {
        s_fail_count = 0;
    }
}

bool ha_client_is_ready(void)
{
    return s_lock != NULL && s_base[0] != '\0';
}

ha_client_poll_mode_t ha_client_poll_mode(void)
{
    return s_poll_mode;
}

static void poll_wake(void)
{
    if (s_poll_ev != NULL) {
        xEventGroupSetBits(s_poll_ev, HA_POLL_WAKE_BIT);
    }
}

/** 轮询任务的可中断等待：模式变化（暂停/恢复）会立刻唤醒它。 */
static void poll_delay(uint32_t ms)
{
    if (s_poll_ev == NULL) {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }
    xEventGroupWaitBits(s_poll_ev, HA_POLL_WAKE_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(ms));
}

void ha_client_set_poll_mode(ha_client_poll_mode_t mode)
{
    if (mode == s_poll_mode) {
        return;
    }
    s_poll_mode = mode;

    /* 每次真正切换只打一条：现场看日志就能确认是 WS 接管还是回落 REST。 */
    switch (mode) {
    case HA_POLL_PAUSED:
        ha_client_release_http();
        ESP_LOGI(TAG, "REST 轮询暂停：WebSocket 接管后只保留一条 TLS，"
                      "内部可用 %d KB",
                 (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
        break;
    case HA_POLL_SLOW:
        ESP_LOGI(TAG, "WS 已接管（ws 明文）：REST 轮询放慢到 %d 秒做一致性检查",
                 HA_POLL_SLOW_INTERVAL_S);
        break;
    case HA_POLL_NORMAL:
    default:
        ESP_LOGI(TAG, "WS 不可用：恢复 REST 轮询（%d 秒）", CONFIG_APP_POLL_INTERVAL_S);
        break;
    }
    poll_wake();
}

static void poll_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1500));
    while (true) {
        const ha_client_poll_mode_t mode = s_poll_mode;
        if (mode == HA_POLL_PAUSED) {
            /* WS 事件已经负责状态刷新：这里只等模式变化，不再发 HTTP。
               天气预报是 30 分钟一次的独立请求，WS 接管时也照常刷（内部 RAM
               不够就自己跳过，见 wx_forecast_tick）。 */
            wx_forecast_tick();
            poll_delay(HA_POLL_SLOW_INTERVAL_S * 1000);
            continue;
        }
        if (s_count == 0) {
            /* 还没收到布局，没有实体可刷：只探测 HA 是否可达 */
            char url[APP_HA_URL_MAX + 16];
            snprintf(url, sizeof(url), "%s/api/", s_base);
            int st = http_get(url);
            ESP_LOGI(TAG, "HA probe -> %d (internal heap %d KB)", st,
                     (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
            wx_forecast_tick();          /* 布局里没有关注列表时也不会抓到东西，自己会跳过 */
            poll_delay(30000);
            continue;
        }
        for (int i = 0; i < s_count; i++) {
            /* 兜底轮询用 refresh_one：即使 WS 在线也要真的走一次 REST，
               这样 ws:// 场景下 60 秒一致性检查才有意义。 */
            if (s_poll_mode == HA_POLL_PAUSED) {
                break;          /* 轮询途中被 WS 接管：立刻停手，别再建 TLS */
            }
            refresh_one(i);
        }
        wx_forecast_tick();              /* 30 分钟一次的预报（和实况同一个任务）*/
        poll_delay((mode == HA_POLL_SLOW ? HA_POLL_SLOW_INTERVAL_S
                                         : CONFIG_APP_POLL_INTERVAL_S) * 1000);
    }
}

void ha_client_start_poll(void)
{
    if (s_poll_task != NULL) {
        return;
    }
    if (s_poll_ev == NULL) {
        s_poll_ev = xEventGroupCreate();
    }
    if (xTaskCreate(poll_task, "ha_poll", 6144, NULL, 4, &s_poll_task) != pdPASS) {
        s_poll_task = NULL;
        ESP_LOGE(TAG, "轮询任务创建失败");
    }
}
