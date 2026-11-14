#pragma once

/*
 * 旧工程 bsp_board.h 的替代：JC3636F118（ESP32-S31 + 360x360 ST77916 + CST816S）。
 *
 * ui_home.c 只 #include 了它，并没有真正用到里面的引脚宏（旧代码里背光/触摸都走
 * bsp_display_* / bsp_touch_*），所以这里保留成一张纯查表用的引脚表，避免移植时
 * 还要改 ui_home.c 的 include。真正生效的引脚定义以 main.c 为准。
 */

#include "bsp_shim.h"

/* ---------------- 显示：ST77916 over 4-line QSPI ---------------- */
#define BSP_LCD_H_RES        360
#define BSP_LCD_V_RES        360

#define BSP_LCD_BL_GPIO      5     /* 背光，LEDC PWM（GPIO_ONLY） */
#define BSP_LCD_RST_GPIO     18
#define BSP_LCD_CS_GPIO      13
#define BSP_LCD_SCK_GPIO     12
#define BSP_LCD_D0_GPIO      14
#define BSP_LCD_D1_GPIO      15
#define BSP_LCD_D2_GPIO      16
#define BSP_LCD_D3_GPIO      17

/* ---------------- 触摸：CST816S over I2C ---------------- */
#define BSP_TOUCH_SDA_GPIO   11
#define BSP_TOUCH_SCL_GPIO   10

/* ---------------- 编码器 / 触觉 ---------------- */
#define BSP_ENCODER_A_GPIO   7
#define BSP_ENCODER_B_GPIO   6
#define BSP_HAPTIC_I2C_ADDR  0x5A

/* ---------------- 电池 ADC ---------------- */
#define BSP_BAT_ADC_GPIO     55
