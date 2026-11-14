#pragma once

#include <stdbool.h>
#include <stddef.h>

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
    char hvac_modes[80];   /* 逗号分隔，如 "off,cool,heat" */
    char fan_modes[80];
} ha_entity_t;

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
