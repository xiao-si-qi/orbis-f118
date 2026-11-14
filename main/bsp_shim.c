/**
 * @file bsp_shim.c
 * @brief 旧工程 bsp_display_set_backlight() 的新板实现（LEDC 通道 0，10bit）。
 *
 * 只做占空比设置：LEDC timer / channel / GPIO 的初始化仍在 main.c 的
 * backlight_init() 里（GPIO_ONLY 方案，PIN_BL=5，1023 满量程），
 * 这样硬件初始化只有一处，shim 不会和它抢配置。
 */
#include "bsp_shim.h"

#include "driver/ledc.h"

/* 与 main.c backlight_init() 必须一致 */
#define BSP_BL_LEDC_MODE LEDC_LOW_SPEED_MODE
#define BSP_BL_LEDC_CH   LEDC_CHANNEL_0
#define BSP_BL_DUTY_MAX  1023

void bsp_display_set_backlight(uint8_t pct)
{
    if (pct < 1) {
        pct = 1;
    } else if (pct > 100) {
        pct = 100;
    }
    ledc_set_duty(BSP_BL_LEDC_MODE, BSP_BL_LEDC_CH, (uint32_t)BSP_BL_DUTY_MAX * pct / 100);
    ledc_update_duty(BSP_BL_LEDC_MODE, BSP_BL_LEDC_CH);
}
