#pragma once

#include "layout.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 启动页：在拿到布局之前先显示"connecting..."。 */
void ui_home_boot(void);

/** 旋钮翻页：delta>0 下一页，delta<0 上一页（子页面打开时忽略） */
void ui_home_page_step(int delta);

/** 翻到最后一页（小智助手页）。页面不存在时只记日志。MQTT assistant 命令用。 */
void ui_home_goto_assistant(void);

/** 翻回第一页（MQTT 关闭小智页用）。 */
void ui_home_goto_first(void);

/** 当前是否停在小智助手页（小智模块做可见性兜底时用；翻页动画期间也以目标页为准）*/
bool ui_home_assistant_current(void);

/** 用布局重建界面（可在任意任务调用，内部切回 LVGL 上下文）。 */
void ui_home_apply(const layout_t *layout);

/** 打开子页面（灯：亮度/色温；空调：目标温度/模式/风速；风扇：风速/预设/摇头）。
 *  都走同一个 s_detail，按实体 id 前缀（light./climate./fan.）决定显示哪一组控件。 */
void ui_home_open_detail(const char *entity_id);

/** 关闭子页面（带滑出动画）。 */
void ui_home_close_detail(void);

/**
 * 旋钮在详情页里调值（灯：亮度/色温；空调：目标温度，整格 = 该实体的 target_temp_step；
 * 风扇：风速，每格 5%）。
 * 返回 true 表示这次旋转被详情页消费了（调用方响一下触觉、不要翻页）；
 * 返回 false 表示没消费（详情页没开 / 设备是关的 / 手指正按着滑条），由调用方走翻页。
 * delta = +1 顺时针（值变大），-1 逆时针。
 */
bool ui_home_detail_knob(int delta);

/** 子页面是否打开（滑动翻页要避让）。 */
bool ui_home_detail_open(void);

/** 【已废弃 / ABI 桩】历史曲线已从设计稿移除：这个函数只打一条警告、什么都不做。
 *  main.c 的 MQTT 调试命令还在调用它，所以符号必须保留。 */
void ui_home_show_chart(const char *entity_id, const float *vals, int n,
                        float vmin, float vmax, float vavg);

/** 【已废弃 / ABI 桩】没有历史页可关：空实现（screenshot.c / assistant_ui.c 还在调）。 */
void ui_home_close_chart(void);

/** 【已废弃 / ABI 桩】永远返回 false（历史曲线已取消）。 */
bool ui_home_chart_open(void);

/** 打开天气页（实况 + 逐小时 + 逐日，整页纵向滚动）。点天气卡片时内部调用；
 *  也留给 MQTT / 调试用。打开后旋钮归它：页内滚动，不再翻页。 */
void ui_home_weather_show(const char *entity_id);

/** 关闭天气页（左边缘右滑也会走这里）。 */
void ui_home_weather_hide(void);

/** 天气页是否打开（旋钮 / 翻页 / 手势都要避让）。 */
bool ui_home_weather_open(void);

/** 打开空调页（温度弧 / 模式键 / 风速 / 电源）。和灯共用一个详情页。 */
void ui_home_open_climate(const char *entity_id);

/** 关闭空调页。 */
void ui_home_close_climate(void);

/** 空调页是否打开（= 详情页开着，且当前实体是 climate.*）。
 *  风扇页也是同一个 s_detail 的第三种形态（fan.*），没有单独的开 / 关函数：
 *  打开走 ui_home_open_detail()，关闭共用 ui_home_close_detail()。 */
bool ui_home_climate_open(void);

#ifdef __cplusplus
}
#endif
