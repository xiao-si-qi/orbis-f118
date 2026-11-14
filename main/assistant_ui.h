/**
 * @file assistant_ui.h
 * @brief 小智助手界面：纯白全屏页 + 会动的脸（眼睛/嘴）+ 中文状态文字 + 触摸交互。
 *
 * 入口有三处（都很轻）：
 *   - MQTT：{"cmd":"assistant"} 打开，{"cmd":"assistant","open":false} 关闭（main.c）；
 *   - 主界面「小智 AI」语音页上的「小智」按钮（ui_home.c）；
 *   - 本模块内部：右滑 / 顶部关闭按钮回主界面。
 *
 * 约定：
 *   - 页面第一次 assistant_ui_open() 时才创建，之后只隐藏，不反复占内存；
 *   - 所有函数必须在 LVGL 线程调用（main.c 的 MQTT 回调在主循环里，满足）；
 *   - assistant_ui_tick() 由 ui_home 的 500ms 定时器调用，负责状态/文字；
 *     表情动画在本模块自己的 50ms 定时器里跑（关页面时暂停，不空转）。
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 打开小智界面（首次调用时创建，之后只做显示 + 置顶 + 动画）。 */
void assistant_ui_open(void);

/** 关闭：滑出后隐藏；如果还在聆听，顺手把手动开麦停掉，别把麦克风留在后台。 */
void assistant_ui_close(void);

/** 界面当前是否打开（旋钮/翻页要避让）。 */
bool assistant_ui_is_open(void);

/** 由 ui_home 的 500ms 定时器调用：刷新状态文字/表情（关着时立即返回）。 */
void assistant_ui_tick(void);

#ifdef __cplusplus
}
#endif
