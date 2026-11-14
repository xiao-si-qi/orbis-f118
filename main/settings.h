#pragma once
#include "lvgl.h"

/** 在给定容器里构建参数页 */
void settings_build(lv_obj_t *parent);

/** 周期刷新参数页内容（仅在显示时调用） */
void settings_refresh(void);
