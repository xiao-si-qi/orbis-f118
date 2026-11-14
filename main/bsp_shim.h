#pragma once

/*
 * 从 orbis-panel（ESP32-S3）移植到 orbis-f118（ESP32-S31）时加的一层兼容 shim。
 *
 * 旧工程 ui_home.c 通过 bsp_display_set_backlight() 调背光；新工程的背光仍然由
 * main.c 的 backlight_init() 用 LEDC（LOW_SPEED / channel 0 / 10bit / PIN_BL）
 * 初始化，这里只补"设置占空比"那一半，避免两边各配一次 LEDC。
 *
 * 另外：新工程 LV_USE_OS=NONE，LVGL 9.6 自带的 lv_lock()/lv_unlock() 在编译时
 * 就是空实现，所以移植过来的 lv_lock()/lv_unlock() 调用不需要再包一层宏。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 设置背光亮度（1..100，越小越暗）。LEDC 的 timer/channel 由 main.c 配好。 */
void bsp_display_set_backlight(uint8_t pct);

#ifdef __cplusplus
}
#endif
