#pragma once

/*
 * screenshot.h — 远程调试用的两个功能（不用碰设备就能看 UI）：
 *
 *   1) 截图：{"cmd":"screenshot"}       → 抓当前整屏 360×360，硬件 JPEG 编码，
 *                                        base64 分片发到 switchyard/<dev>/shot[/<id>/<i>]
 *             {"cmd":"shot_serial"}     → 同一张图改成串口 base64 打印（USB 独用时用，
 *                                        平时绝不开，免得刷屏）。可选 "quality":1-100。
 *
 *   2) 导航：{"cmd":"page","value":N}   → 翻到第 N 页（0 基），N 步 × ui_home_page_step(+1)
 *             {"cmd":"page","value":"next"} / "prev"
 *             {"cmd":"open","entity":"light.xxx"} → 打开灯/空调/风扇详情页（= 点卡片）
 *             {"cmd":"close"}          → 关掉详情页 / 天气页，回到卡片页（= 左边缘右滑）
 *
 * 抓屏路线（见 .c 里的详细说明）：本工程的 LVGL 没开 LV_USE_SNAPSHOT（sdkconfig 里是 n，
 * 不许改 sdkconfig），所以走"复用现有 flush 通路"：打一个标志，让 main.c 的 flush 回调
 * 在每次 SPI 刷屏时把这一块拷进整屏 RGB565 缓冲（PSRAM），然后主动整屏重绘一次。
 *
 * 线程模型：截图请求、抓屏（lv_refr_now）必须在 LVGL 任务里做；编码 + 上传在独立任务里做。
 * screenshot_tick() 由 LVGL 主循环每轮调用，用来驱动"翻页命令"的状态机（翻页有 220 ms 动画，
 * 连着调 ui_home_page_step() 会被 ui_home 的 s_animating 丢掉，所以得分轮次走）。
 */

#include <stdbool.h>

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 抓一张截图。serial=true 走串口 base64（不联网），false 走 MQTT 分片上传。
 * quality 0 表示用默认值（80），否则 1-100。
 * delay_ms < 0 表示用默认值（250 ms）：截图前等一下，让翻页 / 开详情页的 220 ms
 * 滑动动画走完，否则会拍到"滑到一半"的画面；0 = 立刻抓。
 * order 是 RGB565 字节序 / 通道序的调法（默认 0，硬件实测不对时不用重烧固件）：
 *   0 = LVGL SWAPPED 缓冲对调字节 + JPEG pixel_reverse=false（原生长度序，默认）
 *   1 = 原样喂进去（不对调字节）+ pixel_reverse=false
 *   2 = 对调字节 + pixel_reverse=true
 *   3 = 原样 + pixel_reverse=true
 * 返回 true 表示任务已排队/已启动（编码、上传在后台任务里跑），false 表示没起来
 * （PSRAM 不够 / 上一次还没传完 / 任务创建失败），原因已经打在日志里。
 * 必须在 LVGL 任务上下文调用（会调 lv_refr_now）。
 */
bool screenshot_capture(bool serial, int quality, int delay_ms, int order);

/**
 * flush 回调钩子：main.c 的 lv_flush_cb() 每次刷屏都调它。
 * 没在抓屏时是一句 bool 判断，零开销。
 * area/px_map 就是 LVGL 传下来的那一块；本函数把这一块拷进整屏缓冲并对调字节序
 * （LVGL 的绘制缓冲是 RGB565_SWAPPED，JPEG 编码器要的是原生长度序）。
 */
void screenshot_flush_hook(const lv_area_t *area, const uint8_t *px_map);

/** 翻到第 page 页（0 基）。会先关掉详情/天气/折线这些子页面。
 *  返回 false 表示上一个翻页命令还在走（已忽略本次）。 */
bool screenshot_goto_page(int page);

/** 上一页 / 下一页（只翻一页，走 ui_home_page_step）。 */
void screenshot_page_step(int dir);

/**
 * 打开实体的详情页（灯 / 空调 / 风扇 / 天气），走的就是"点卡片"那条公开入口：
 *   light.* / climate.* / fan.*  → ui_home_open_detail(entity)
 *   weather.*                    → ui_home_weather_show(entity)
 * 打开前会先关掉天气页，避免两层覆盖页叠在一起。
 * 返回 true 表示已调用；false 表示参数不合法。
 */
bool screenshot_open_entity(const char *entity);

/**
 * 关掉当前覆盖页（详情 / 天气），回到卡片页 —— 走的就是"左边缘右滑"那条路径。
 * 返回 true 表示确实关掉了什么。
 */
bool screenshot_close_overlay(void);

/** 由 LVGL 主循环每轮调用：驱动翻页状态机。 */
void screenshot_tick(void);

#ifdef __cplusplus
}
#endif
