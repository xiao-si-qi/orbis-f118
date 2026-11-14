#pragma once
#include <stdbool.h>
#include "lvgl.h"

/** 打开展示某实体的历史折线（后台取数） */
void chart_open(const char *entity_id);

/** 关闭折线页 */
void chart_close(void);

/** 折线页是否打开 */
bool chart_is_open(void);

/** 主线程轮询：取数完成后更新界面 */
void chart_poll(void);
