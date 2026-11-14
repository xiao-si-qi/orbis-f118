/**
 * @file xiaozhi_client.h
 * @brief 小智 AI 服务端接入（OTA 发现 + WebSocket 握手 + Opus 音频双向流）。
 *
 * 流程（协议见 xiaozhi-esp32/docs/websocket.md）：
 *   1. POST <endpoint>（OTA/发现接口）→ 拿到 {"websocket":{"url":"wss://..."}}
 *   2. 连 WebSocket，带 Authorization / Protocol-Version / Device-Id / Client-Id
 *   3. 发 hello（transport=websocket, audio_params=opus/16000/60ms）
 *   4. 收服务器 hello（含 session_id）→ 标记已连接，起编解码任务，发 listen start
 *   5. 二进制帧 = 16 字节 v2 头 + opus；上行 16k/60ms，下行 24k/60ms，半双工
 *
 * 服务端地址与开关由 switchyard 下发的布局 JSON（voice.enabled / voice.endpoint）决定，
 * 见 main.c 的 apply_cfg()。
 */
#ifndef XIAOZHI_CLIENT_H
#define XIAOZHI_CLIENT_H

#include <stdbool.h>

/** 按布局配置启动/停止。endpoint 为空或 enabled=false 时停止。 */
void xiaozhi_client_configure(bool enabled, const char *endpoint);

/** 给小智助手页显示的一行状态文字（永远非 NULL）。 */
const char *xiaozhi_client_status_text(void);

/** 官方平台的激活码（空字符串表示无需激活）*/
const char *xiaozhi_activation_code(void);

/** 是否已连上服务端（hello 完成）。 */
bool xiaozhi_client_ready(void);

/** 服务端类型：官方 / 自建（用于页面显示）。 */
const char *xiaozhi_client_server_note(void);

/* ------------------------- 界面用的状态 / 控制 API ------------------------- */

/**
 * 界面用的高阶状态，全部由内部标志推导：
 *   LISTENING = 上行（麦克风）在跑；THINKING = 上行已停、回复音频还没到；
 *   SPEAKING  = TTS 正在播放；ERROR = 发现/连接失败；CONNECTING = 发现或握手进行中。
 */
typedef enum {
    XZ_IDLE = 0,    /**< 未启用，或已连上但空闲 */
    XZ_CONNECTING,  /**< 正在发现服务端 / WebSocket 握手中 */
    XZ_LISTENING,   /**< 麦克风上行中（服务端在听）*/
    XZ_THINKING,    /**< 上行已停、还没有下行音频（服务端在思考）*/
    XZ_SPEAKING,    /**< TTS 正在播放 */
    XZ_ERROR,       /**< 发现/连接出错 */
} xz_state_t;

/** 当前状态（读几个 volatile 标志，不做任何阻塞/分配，可在 LVGL 线程调用）。 */
xz_state_t xiaozhi_state(void);

/**
 * 手动开麦：给上行任务排一次 listen start（和自动流程发的是同一条消息）。
 * 还没连上服务端时返回 false，界面据此提示。
 */
bool xiaozhi_listen_start(void);

/** 手动关麦：给上行任务排一次 listen stop（LVGL 线程里只置标志，不等网络）。 */
void xiaozhi_listen_stop(void);

/**
 * 最近一次识别到的文字（服务端 stt 消息），最多 96 字节，永远非 NULL。
 * 返回的是内部快照，界面直接显示即可，别长期持有。
 */
const char *xiaozhi_last_text(void);

/**
 * 最近一次服务端"要说的话"（tts 消息里的 text，最多 120 字节），永远非 NULL。
 * 界面只在 XZ_SPEAKING 时显示它；下次开麦（listen start）会清空，
 * 避免把上一轮的回复当成这一轮的。和 xiaozhi_last_text() 一样是内部快照。
 */
const char *xiaozhi_last_tts_text(void);

#endif /* XIAOZHI_CLIENT_H */
