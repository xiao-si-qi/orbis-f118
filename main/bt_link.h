#pragma once

/**
 * 蓝牙链路抽象层：界面只跟这个模块打交道，不直接碰蓝牙协议栈。
 *
 * 本版是「经典蓝牙 A2DP source」的真实实现（ESP32-S31，BR/EDR + Bluedroid）：
 *   - 协议栈按需启动：bt_link_init() 只读 NVS、建互斥量/任务，不碰控制器和 host；
 *     第一次 bt_link_scan_start() / bt_link_connect() / bt_link_enable() 才真正拉起协议栈；
 *   - 空闲 60 秒（或扫描结束没连上后 30 秒）自动释放协议栈，把内存还给系统；
 *   - 扫描结果最多存 8 条（名字 + MAC），bt_link_get_scan_result() 给界面用；
 *   - 连接成功后状态 BT_CONNECTED，音箱名给 bt_link_device_name()；
 *   - 音频输出「偏好」照旧存 NVS（耳机孔 / 蓝牙音箱），本版只提供数据通路钩子
 *     （bt_link_audio_is_ready / bt_link_audio_write），还没有接进 voice_audio 的播放路径。
 *
 * 线程安全：状态是单字 volatile，蓝牙任务写、LVGL 线程读；
 * 名字 / 扫描结果 / 音频输出偏好由模块内部的小互斥量保护。
 * bt_link_audio_write() 只在「就绪」时由音频线程调用，内部是无锁环形缓冲（单生产者单消费者）。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 蓝牙链路状态。 */
typedef enum {
    BT_OFF = 0,     /* 未启用（还没启动）/ 已释放（省内存） */
    BT_INIT,        /* 协议栈启动中 */
    BT_IDLE,        /* 协议栈已就绪、未连接 */
    BT_SCANNING,    /* 扫描中 */
    BT_CONNECTING,  /* 连接中 */
    BT_CONNECTED,   /* 已连接（A2DP 音频通路已建好） */
    BT_ERROR,       /* 出错 */
} bt_state_t;

/**
 * 开机初始化一次：读 NVS 偏好 / 上次连接的音箱、建互斥量 + 蓝牙任务。
 * 幂等，不启动蓝牙协议栈（不占 ~80-100 KB 内部内存），不会拖慢开机。
 */
void bt_link_init(void);

/** 蓝牙协议栈是否编译进来了（不代表正在运行）。界面用它决定扫描/连接按钮是否可点。 */
bool bt_link_is_enabled(void);

/** 按需启动协议栈（异步；第一次扫描/连接会自动调用，也可以由界面显式调用）。 */
bool bt_link_enable(void);

/** 立即关停协议栈（断开连接 → A2DP deinit → Bluedroid → 控制器），释放内存；已关闭时是空操作。 */
void bt_link_disable(void);

/** 当前状态（单字 volatile 快照，可从任意任务读）。 */
bt_state_t bt_link_state(void);

/** 状态中文描述：未启用（待启用） / 正在启动蓝牙… / 已就绪 / 扫描中… / 已连接 音箱名 / 蓝牙已释放（省内存）。 */
const char *bt_link_state_text(void);

/** 已连接音箱名；没连上返回空串（不是 NULL）。 */
const char *bt_link_device_name(void);

/** 开始扫描（约 15 秒，自动停止）；需要时会先启动协议栈。返回 false=请求没排上。 */
bool bt_link_scan_start(void);

/** 停止扫描；没在扫描时是空操作。 */
void bt_link_scan_stop(void);

/** 取第 index 条扫描结果（名字 + "aa:bb:cc:dd:ee:ff"）；越界返回 false。 */
bool bt_link_get_scan_result(int index, char *name, size_t name_len,
                             char *addr, size_t addr_len);

/**
 * 连接指定 MAC 地址的音箱；需要时会先启动协议栈。
 * addr 为空/NULL 时连接「上次连过的音箱」（NVS 里存的地址）。
 */
bool bt_link_connect(const char *addr);

/** 断开当前连接（协议栈继续留着，空闲 60 秒后自动释放）。 */
void bt_link_disconnect(void);

/**
 * 音频输出「有效值」：true=蓝牙音箱，false=耳机孔（默认耳机孔）。
 * 只反映用户偏好（协议栈编译进来了就按偏好算），不代表现在能出声。
 */
bool bt_link_audio_to_bt(void);

/** 记下音频输出选择并落盘 NVS（值没变不写 flash）。切到蓝牙音箱会按需启动协议栈并尝试重连上次的音箱。 */
void bt_link_set_audio_to_bt(bool to_bt);

/**
 * A2DP 音频通路是否真的可以收 PCM：已连接 + A2DP 已启动 + 用户选了「蓝牙音箱」。
 * 下一轮 voice_audio.c 的路由就按这个开关走。
 */
bool bt_link_audio_is_ready(void);

/**
 * 把 PCM（单声道 int16，任意采样率）送进 A2DP source 的数据通路。
 * 内部做「线性插值重采样到 44.1 kHz + 单声道复制成双声道 + 8 KB 环形缓冲」，
 * 缓冲满就丢（不会阻塞调用方）；没就绪时直接丢弃。环形缓冲在协议栈启动时从
 * PSRAM 借 9 KB、关栈时归还，平时不占内部 RAM。
 * 注意：本版只是钩子，voice_audio.c / xiaozhi_client.c 还没有调用它。
 */
void bt_link_audio_write(const int16_t *pcm, size_t samples, int sample_rate);

#ifdef __cplusplus
}
#endif
