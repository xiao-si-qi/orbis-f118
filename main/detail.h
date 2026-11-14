#pragma once
#include <stdbool.h>
#include "lvgl.h"

/** 打开某个灯实体的详情页 */
void detail_open(const char *entity_id);

/** 关闭详情页 */
void detail_close(void);

/** 详情页是否打开 */
bool detail_is_open(void);

/** 周期刷新数值（主循环调用） */
void detail_refresh(void);
