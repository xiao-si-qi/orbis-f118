#pragma once

/** 电池 ADC 原始值（ESP32-S31 只有一档衰减、无校准方案，未换算成电压），无效返回 -1。 */
int battery_raw_value(void);

/** 最近一次采样中的原始峰值 */
int battery_raw_peak(void);
