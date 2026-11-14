#pragma once

/*
 * Home Assistant WebSocket 客户端（/api/websocket）。
 *
 * 一条长连接同时承担两件事：
 *   - 下行：订阅 state_changed，事件到达后直接更新 ha_client 的实体表，
 *           ui_home 的 500ms 刷新定时器下一拍就能看到新状态（不再等 5 秒轮询）；
 *   - 上行：service call 走快路径（约 20-25ms 往返），WS 没连上时才退回 REST。
 *
 * 连接地址由 app_settings 里的 HA 地址推导：https→wss（带 crt_bundle），
 * http→ws（明文，仅限可信内网）。重连退避 5s→60s；wss 只保留一条 TLS：
 * 认证成功后由 ha_client 暂停并释放 REST 的常驻 HTTP/TLS。
 */

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 启动连接任务（幂等）。返回 false 仅表示任务/互斥量创建失败。 */
bool ha_ws_start(void);

/** 停止连接任务（会等它把手上的 WS 句柄 stop/destroy 掉，最多约 5 秒）。 */
void ha_ws_stop(void);

/** WS 是否已通过认证、可以发 service call（仅"TCP 连上"不算）。 */
bool ha_ws_connected(void);

/**
 * @brief 通过 WebSocket 执行一次 service call（非阻塞，不等待 result）。
 *
 * @param domain            如 "light"
 * @param service           如 "turn_on"
 * @param entity_id         如 "light.bedroom"
 * @param service_data_json 额外字段，形如 {"brightness_pct":42}；NULL 表示只带 entity_id
 *
 * @return true 表示文本帧已交给 WebSocket；false 表示未连接或发送失败。
 */
bool ha_ws_call_service(const char *domain, const char *service,
                        const char *entity_id, const char *service_data_json);

/** 一行状态摘要（MQTT 的 {"cmd":"ha_ws"} 调试命令用）。 */
void ha_ws_status(char *out, size_t len);

#ifdef __cplusplus
}
#endif
