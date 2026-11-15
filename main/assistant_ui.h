/**
 * @file assistant_ui.h
 * @brief 小智助手页：纯白底 + 会动的脸（眼睛/嘴）+ 中文状态文字 + 触摸说话。
 *
 * 它现在是主界面（ui_home）的**最后一页**，不是二级悬浮页：
 *   - ui_home 先建好页面对象，再调 assistant_ui_build(page)，本模块把内容画进去
 *     （白底、360x360、整页接触摸，和卡片页共用同一套翻页/圆点/手势）；
 *   - MQTT {"cmd":"assistant"} / {"cmd":"assistant","open":false} 只是翻页：
 *     assistant_ui_open() → ui_home_goto_assistant()，assistant_ui_close() →
 *     ui_home_goto_first()，不再叠第二层界面；
 *   - 旋钮在这一页照常翻页（它只是普通一页）；
 *   - assistant_ui_tick() 由 ui_home 的 500ms 定时器调用：页面可见时刷新状态/文字，
 *     表情动画在本模块自己的 50ms 定时器里跑（页面不可见时 pause，不空转）。
 *
 * 所有函数必须在 LVGL 线程调用（main.c 的 MQTT 回调在主循环里，满足）。
 *
 * 崩溃排查用的裸串口标记（esp_rom_printf，只写 UART0，先于任何 ESP_LOG/格式化）：
 *   [as] raw: page   page_event_cb() 收到事件（按下/抬起/点击各一条）
 *   [as] raw: tap    handle_tap() 第一行——轻触确实走到了本模块
 *   [as] raw: tick   assistant_ui_tick() 第一行——500ms 刷新路径
 * 看到 tap 却看不到 xiaozhi 客户端的 [xz] raw: listen_start，问题在两者之间。
 */
#pragma once

#include <stdbool.h>

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 把助手页内容画进调用方给的父对象（ui_home 的最后一页）。可重复调用（同一对象）。 */
void assistant_ui_build(lv_obj_t *parent);

/**
 * 页面可见性通知（ui_home 翻页时调用）。
 * 只改内部标志 + 起停 50ms 表情定时器，**不碰任何 LVGL 对象**，
 * 所以可以安全地在翻页事件回调里调用；真正的文字刷新留到定时器里做。
 * 页面已被销毁（指针失效）时只把自己置为不可见，绝不碰野指针。
 * 离开页面时如果还在聆听，会顺手关麦。
 */
void assistant_ui_set_visible(bool visible);

/** 翻到助手页（MQTT 打开命令走这里；页面不存在/已销毁时只记日志）。 */
void assistant_ui_open(void);

/** 翻回第一页（MQTT 关闭命令走这里；如果还在聆听，顺手关麦）。 */
void assistant_ui_close(void);

/** 助手页当前是否可见（它只是普通页，旋钮不再避让）。页面已销毁时返回 false。 */
bool assistant_ui_is_open(void);

/**
 * 由 ui_home 的 500ms 定时器调用：同步可见性 + 刷新状态文字/表情。
 * **页面被销毁或不可见时也可以安全调用**（内部先确认页面还挂在活动屏幕上，
 * 不在就只清状态并返回，不解引用任何已释放的控件指针）。
 */
void assistant_ui_tick(void);

#ifdef __cplusplus
}
#endif
