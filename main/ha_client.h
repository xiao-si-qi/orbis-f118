#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <string.h>   /* ha_entity_has_hvac_mode 里的 strcmp */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HA_MAX_ENTITIES 24

/** REST 轮询节奏：由 ha_ws 根据 WebSocket 状态切换（见 ha_client_set_poll_mode）。 */
typedef enum {
    HA_POLL_NORMAL = 0,   /* 5 秒 REST 轮询（默认；WS 不可用时的兜底） */
    HA_POLL_SLOW,         /* 60 秒兜底一致性检查（明文 ws:// 已接管，无 TLS 开销） */
    HA_POLL_PAUSED,       /* 暂停并释放常驻 HTTP/TLS（wss:// 已接管，只留一条 TLS） */
} ha_client_poll_mode_t;

/* -------------------------------------------------------------------------
 * 空调实体的模式表
 *
 * 每台空调的 hvac_modes / fan_modes 都是“定长小集合”（HA 最多给 6 个），
 * 所以按 6 × 12 的定长数组存，而不是以前那两条 80 字节的逗号串：
 * 12 刚好放得下最长的 "medium high"（11 + '\0'）。
 * preset_modes 界面不显示，只记“这台机器支持哪些预设”，用 1 字节位图，
 * 不占定长数组（24 个实体 × 60 字节 = 1.4KB 内部 RAM，不值得）。
 * ------------------------------------------------------------------------- */
#define HA_MODE_MAX   6      /* hvac_modes / fan_modes 最多记 6 个 */
#define HA_MODE_LEN   12     /* 单个模式名最长 11 字符 + 结尾 0 */
#define HA_FAN_LEN    16     /* 当前 fan_mode（原样字符串，直接回传给 HA） */
#define HA_FAN_PRESET_MAX  4 /* 风扇 preset_modes 最多记 4 个（直吹/自然/循环/睡眠）*/

#define HA_PRESET_ECO    0
#define HA_PRESET_AWAY   1
#define HA_PRESET_BOOST  2
#define HA_PRESET_NONE   3
#define HA_PRESET_SLEEP  4
#define HA_PRESET_COUNT  5

typedef struct {
    char entity_id[96];
    char name[48];
    char state[24];
    char unit[12];
    bool on;
    bool valid;
    bool fresh;
    /* 灯类实体的属性（其它实体会是 0） */
    int brightness;        /* 0-255 */
    int color_temp_k;      /* 当前色温（开尔文） */
    int min_k, max_k;      /* 该实体支持的范围 */
    bool has_brightness;
    bool has_tint;         /* 支持色温调节 */
    bool seen_brightness;  /* 是否拿到过真实亮度（灯关闭时 HA 不返回） */
    bool seen_tint;
    /* 空调类实体 */
    float cur_temp;        /* 当前温度 */
    float target_temp;     /* 目标温度 */
    float min_temp, max_temp;
    /* target_temp_step：每台机器都不一样（这台 1，另一台可能 0.5），
       旋钮/圆环/小数位全按它来；缺省 1。 */
    float temp_step;
    char fan_mode[HA_FAN_LEN];   /* 当前风速（原样回传，如 "medium low"） */
    /* ---- 空调 / 风扇共用一块「模式名」存储 ----
     * climate.* 用 hvac_modes / fan_modes（外加两个计数和 preset_mask）；
     * fan.*     用 preset_modes（这台风扇支持的预设，原样字符串，直接回传 HA，
     *            如 "直吹风"/"自然风"/"循环风"/"睡眠"）。
     * 一台实体不可能同时是 climate 和 fan，所以这两个视图叠在同一个 union 上：
     * 风扇的列表借用空调那 6×12 的槽位，0 字节额外 RAM（照原先各开一份要 1.1KB）。
     * 只有 fan.* 的解析路径会碰 preset_modes，climate.* 的路径只碰 hvac_modes。*/
    union {
        struct {
            char hvac_modes[HA_MODE_MAX][HA_MODE_LEN];  /* 只存 HA 报上来的，如 auto/cool/heat/dry/fan_only/off */
            char fan_modes[HA_MODE_MAX][HA_MODE_LEN];
            uint8_t hvac_mode_count;
            uint8_t fan_mode_count;
            uint8_t preset_mask;     /* bit i = 支持 HA_PRESET_*，只占 1 字节（界面不显示）*/
        };
        struct {
            char preset_modes[HA_FAN_PRESET_MAX][HA_MODE_LEN];  /* 原样，如 "自然风" */
            uint8_t preset_mode_count;
        };
    };
    /* ---- 风扇类实体（其它实体保持 0）---- */
    int percentage;              /* 0..100（HA 的 percentage；关着时 HA 可能不报，沿用上次）*/
    char preset_mode[16];        /* 当前预设（原样，如 "自然风"）*/
    bool oscillating;            /* 摇头（HA 的 oscillating）*/
    bool has_percentage;         /* 是否真的拿到过 percentage（没拿到就别显示 0%）*/
} ha_entity_t;

/** 这台空调是否支持某个预设模式（下标见 HA_PRESET_*）。*/
static inline bool ha_entity_has_preset(const ha_entity_t *e, int preset)
{
    return e != NULL && preset >= 0 && preset < HA_PRESET_COUNT &&
           (e->preset_mask & (1u << preset)) != 0;
}

/** 这台风扇的 preset_modes 第 idx 个预设（原样中文串）；没有就 NULL。*/
static inline const char *ha_entity_fan_preset(const ha_entity_t *e, int idx)
{
    if (e == NULL || idx < 0 || idx >= HA_FAN_PRESET_MAX || idx >= e->preset_mode_count) {
        return NULL;
    }
    return e->preset_modes[idx];
}

/** 这台空调的 hvac_modes 里有没有 mode（"cool" / "fan_only" …）。*/
static inline bool ha_entity_has_hvac_mode(const ha_entity_t *e, const char *mode)
{
    if (e == NULL || mode == NULL) {
        return false;
    }
    for (int i = 0; i < e->hvac_mode_count && i < HA_MODE_MAX; i++) {
        if (strcmp(e->hvac_modes[i], mode) == 0) {
            return true;
        }
    }
    return false;
}

/** 读取 settings 里的 HA 地址/令牌，创建 HTTP 客户端。 */
esp_err_t ha_client_init(void);

/** 加入关注列表（去重）。label 为 NULL 时用 friendly_name / object_id 兜底。 */
esp_err_t ha_client_register(const char *entity_id, const char *label);

/** 一次拉全量 /api/states，把关注列表里的名字/状态填好（启动时用，最快）。 */
esp_err_t ha_client_fetch_all(void);

int ha_client_count(void);
const ha_entity_t *ha_client_at(int idx);
const ha_entity_t *ha_client_find(const char *entity_id);

/** 单独刷新一个实体。 */
esp_err_t ha_client_refresh_at(int idx);

/** 执行动作：service 传 "toggle" / "turn_on" / "turn_off"。 */
esp_err_t ha_client_action(const char *entity_id, const char *service);

/** 带参数的动作用：attrs_json 是额外字段（不含 entity_id），例如 {"brightness_pct":60}。 */
esp_err_t ha_client_action_attrs(const char *entity_id, const char *service, const char *attrs_json);

/**
 * @brief 拉某个实体最近 hours 小时的历史，降采样到 max_points 个点。
 *
 * @param values     输出数组（float，长度至少 max_points）
 * @param out_count  实际点数
 * @param out_min    最小值
 * @param out_max    最大值
 * @param out_avg    平均值
 */
esp_err_t ha_client_history(const char *entity_id, int hours, float *values, int max_points,
                            int *out_count, float *out_min, float *out_max, float *out_avg);

/** HA REST 是否可达（最近一次请求成功）。界面上的 HA 圆点用它。 */
bool ha_client_online(void);

/** 启动后台轮询（幂等：重复调用不会创建第二个轮询任务）。 */
void ha_client_start_poll(void);

/* -------------------------------------------------------------------------
 * WebSocket（ha_ws.c）用的追加接口。都是线程安全的短操作，可从任意任务调用。
 * ------------------------------------------------------------------------- */

/**
 * @brief 用一条外部状态更新实体（WebSocket 的 state_changed 事件）。
 *
 * @param entity_id  实体 id（必须在关注列表里，否则返回 false）
 * @param state      新状态字符串，如 "on" / "off" / "23.5"
 * @param attrs_json 新状态的 attributes 对象（可为 NULL；解析方式与 REST 全量刷新一致）
 */
bool ha_client_apply_state(const char *entity_id, const char *state, const char *attrs_json);

/** 切换 REST 轮询节奏；只在状态真正改变时打一条日志，并唤醒轮询任务。 */
void ha_client_set_poll_mode(ha_client_poll_mode_t mode);
ha_client_poll_mode_t ha_client_poll_mode(void);

/** 关闭并销毁常驻 HTTP 客户端（释放它占的 TLS/socket 内存，下次用时按需重建）。 */
void ha_client_release_http(void);

/** 常驻 HTTP 客户端当前是否存在（ha_ws 用来确认 TLS 内存是否真的还回去了）。 */
bool ha_client_http_alive(void);

/** HA 客户端是否初始化成功（ha_ws 在 WiFi 晚连上时用它补初始化）。 */
bool ha_client_is_ready(void);

/** WebSocket 认证状态同步给界面上的 HA 圆点（REST 成功/失败也会改它）。 */
void ha_client_set_online(bool online);

#ifdef __cplusplus
}
#endif
