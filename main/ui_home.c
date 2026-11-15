/**
 * 家庭控制屏界面：JSON 布局驱动 + 固件内置的「设置」页 + 「小智 AI」页。
 *
 * 页 0..N-1 来自 HA 的 /local/orbis/layout.json（每页最多 4 个 tile）；
 * 第 N 页是固件自己画的「设置」页：一张可纵向滚动的设置列表，从上到下依次是
 * 系统状态（固件 / MQTT / Wi-Fi / 运行时长 / 内存 / 电池 + 检查更新）、蓝牙
 * （状态 / 扫描 / 连接 / 结果）、音频输出（耳机孔 / 蓝牙音箱）。检查更新拉 HA 上的
 * firmware.json，有新版本就下载并重启；
 * 最后一页（N+1）是小智助手页（纯白底 + 会动的脸，内容由 assistant_ui.c 画进来，
 * 布局里 voice.enabled=true 时才建）。翻页、旋钮、手势、圆点和其它页完全一样。
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>    /* strcasecmp：天气状态大小写不敏感匹配 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "lvgl.h"

#include "app_settings.h"
#include "assistant_ui.h"   /* 小智助手页的内容 + 500ms 定时器里的 tick */
#include "bsp_board.h"      /* 新板 shim（引脚表 + bsp_display_set_backlight） */
#include "bt_link.h"        /* 蓝牙状态抽象层（本版是桩：未启用） */
#include "ha_client.h"
#include "layout.h"
#include "net.h"            /* net_wifi_connected / net_wifi_rssi / net_ip_string / net_mqtt_connected */
#include "ota.h"
#include "battery.h"
#include "bg_image.h"
#include "ui_home.h"
#include "voice_audio.h"    /* 小智页顶部「音频自检」按钮 */

/* 旧工程用的是 lv_font_noto_18_cjk（LVGL 9.2 专用）；新工程只有 lv_font_cjk16
   （16px / 4bpp，GB2312 + ASCII，自带 montserrat_20 兜底）。 */
LV_FONT_DECLARE(lv_font_cjk16);

static const char *TAG = "ui";

/* LVGL 9.6 把 lv_obj_add_flag()/lv_obj_remove_flag() 标成 deprecated
   （ESP-IDF 6.1 默认 -Werror，虽然 deprecation 被降级成警告，但会刷一屏，
    换个 IDF/编译器配置就可能变成硬错误）。这里用 9.6 的 per-flag setter 包一层
   同名宏，移植过来的调用点（约 60 处）保持原样。 */
static inline void ui_obj_set_flags(lv_obj_t *obj, lv_obj_flag_t f, bool en)
{
    /* 防御：构建失败（LVGL 池耗尽）或重建窗口期里，宁可不设也不要踩空。
       新工程 LV_USE_ASSERT_* 全关，LVGL 不会替我们挡 NULL。 */
    if (obj == NULL) {
        return;
    }
    if (f & LV_OBJ_FLAG_HIDDEN)         lv_obj_set_hidden(obj, en);
    if (f & LV_OBJ_FLAG_CLICKABLE)      lv_obj_set_clickable(obj, en);
    if (f & LV_OBJ_FLAG_SCROLLABLE)     lv_obj_set_scrollable(obj, en);
    if (f & LV_OBJ_FLAG_EVENT_BUBBLE)   lv_obj_set_event_bubble(obj, en);
    if (f & LV_OBJ_FLAG_GESTURE_BUBBLE) lv_obj_set_gesture_bubble(obj, en);
}
#define lv_obj_add_flag(o, f)    ui_obj_set_flags((o), (f), true)
#define lv_obj_remove_flag(o, f) ui_obj_set_flags((o), (f), false)

#define MAX_PAGES (LAYOUT_MAX_PAGES + 2) /* JSON 页 + 设置页（合并系统状态/蓝牙/音频）+ 小智 AI 页 */

#define CARD_W    140
#define CARD_H    90
#define CARD_GAP  12
#define GRID_TOP  84
#define GRID_LEFT 34

#define COLOR_BG        lv_color_hex(0x000000)
#define COLOR_CARD      lv_color_hex(0x1C1C1E)
#define COLOR_CARD_ON   lv_color_hex(0x1D4ED8)
#define COLOR_BORDER    lv_color_hex(0x2C2C2E)
#define COLOR_TEXT      lv_color_hex(0xF2F2F7)
#define COLOR_MUTED     lv_color_hex(0x8E8E93)
#define COLOR_OK        lv_color_hex(0x30D158)
#define COLOR_BAD       lv_color_hex(0xFF453A)
#define COLOR_ACCENT    lv_color_hex(0x2563EB)

typedef struct {
    lv_obj_t *card;
    lv_obj_t *label;
    lv_obj_t *value;
    lv_obj_t *dot;
    char entity[96];
    tile_type_t type;
} tile_widget_t;

static layout_t s_layout;
static bool s_layout_valid;
static int s_json_pages;

static tile_widget_t s_tiles[LAYOUT_MAX_PAGES][LAYOUT_TILES_PER_PAGE];
static int s_page_tiles[MAX_PAGES];

static lv_obj_t *s_pages[MAX_PAGES];
static lv_obj_t *s_dots[MAX_PAGES];
static int s_page_count;

static lv_obj_t *s_title;
static lv_obj_t *s_wifi_icon;      /* 状态栏：WiFi 信号弧（自绘） */
static lv_color_t s_wifi_color;
static int s_wifi_level;      /* 0..3 格 */
static lv_obj_t *s_ha_icon;        /* 状态栏：HA 房子图标（在线蓝 / 离线灰） */
static lv_obj_t *s_hint;
static lv_timer_t *s_refresh_timer;

/* 设置页「系统状态」段每行右侧的值文字：固件 / MQTT / WiFi / 运行 / 内存 / 电池 */
static lv_obj_t *s_sys_lines[6];
static lv_obj_t *s_mem_bar;      /* 内存行底部的占用百分比进度条 */
static lv_obj_t *s_ota_btn;
static lv_obj_t *s_ota_btn_label;
static lv_obj_t *s_ota_bar;
static lv_obj_t *s_ota_status;

/* 顶部状态行的蓝牙图标 + 设置页（第 N 页，原 System 页的位置） */
static lv_obj_t *s_bt_icon;
static lv_obj_t *s_set_page;          /* 设置页容器 */
static lv_obj_t *s_set_scroll;        /* 设置页的纵向滚动容器（进出页要复位到顶部） */
static lv_obj_t *s_set_bt_state;      /* 「蓝牙  未启用…」状态行 */
static lv_obj_t *s_set_scan_btn;      /* 扫描按钮（协议栈没启用时置灰） */
static lv_obj_t *s_set_scan_label;
static lv_obj_t *s_set_conn_btn;      /* 连接/断开按钮（同上） */
static lv_obj_t *s_set_conn_label;
static lv_obj_t *s_set_empty;         /* 扫描结果区的空状态文字 */
static lv_obj_t *s_set_phone_btn;     /* 音频输出：耳机孔 */
static lv_obj_t *s_set_phone_label;
static lv_obj_t *s_set_bt_btn;        /* 音频输出：蓝牙音箱 */
static lv_obj_t *s_set_bt_label;
static lv_obj_t *s_set_audio_hint;    /* 「蓝牙未启用，仍走耳机孔」提示 */
static char s_set_state_shown[128];   /* 状态行当前文字（避免 500ms 重复重排） */
static uint32_t s_set_hint_until;     /* 提示保留到什么时候（lv_tick） */
static char s_set_sel_addr[20];       /* 选中的扫描结果 MAC（本版恒为空） */

static void refresh_bt_ui(void);

static int s_page;
static bool s_press_on_slider;   /* 本次按压是否落在滑条上 */

/* ---- 灯 / 传感器 的子页面 ---- */
static lv_obj_t *s_detail;                       /* 全屏子页容器 */
static lv_obj_t *s_detail_title;
static lv_obj_t *s_back_label;
static lv_obj_t *s_br_lab;
static lv_obj_t *s_ct_lab;
static lv_obj_t *s_detail_power_sw;
static lv_obj_t *s_detail_br_row, *s_detail_br_slider, *s_detail_br_value;
static lv_obj_t *s_detail_ct_row, *s_detail_ct_slider, *s_detail_ct_value;
static bool s_detail_visible;

/* ---- 传感器折线图页 ---- */
static lv_obj_t *s_chart_page;
static lv_obj_t *s_chart_title;
static lv_obj_t *s_chart_span;
static lv_obj_t *s_chart;
static float s_chart_pts[32];
static int s_chart_pts_n;
static float s_chart_lo, s_chart_hi;
static lv_obj_t *s_chart_min;
static lv_obj_t *s_chart_max;
static lv_obj_t *s_chart_avg;
static lv_obj_t *s_chart_back_label;
static bool s_chart_visible;

/* ---- 空调页 ---- */
static lv_obj_t *s_clim_page;
static lv_obj_t *s_clim_title;
static lv_obj_t *s_clim_temp;     /* 当前/目标大号文字 */
static lv_obj_t *s_clim_span;     /* 当前模式说明 */
static lv_obj_t *s_clim_slider;
static lv_obj_t *s_clim_modes_row;
static lv_obj_t *s_clim_fans_row;
static bool s_clim_visible;
static char s_clim_entity[96];
static volatile bool s_chart_wanted;
static volatile int s_chart_gen;
static char s_chart_entity[96];
/* 最近一次成功的折线图数据：60 秒内同一实体直接复用，避免每次点击都走网络 */
static char s_chart_cache_entity[96];
static float s_chart_cache_vals[32];
static int s_chart_cache_n;
static float s_chart_cache_min, s_chart_cache_max, s_chart_cache_avg;
static uint32_t s_chart_cache_at;
static char s_detail_entity[96];
/* 旋钮在灯详情页里调什么：默认亮度，点过色温滑条就切色温（换一只灯时复位） */
#define DETAIL_KNOB_BR        0
#define DETAIL_KNOB_CT        1
#define DETAIL_CT_STEP_K      100      /* 色温每格 100K */
#define DETAIL_KNOB_SEND_MS   100      /* 转动时网络命令最快 100ms 一条（更快的格子合并）*/
#define DETAIL_KNOB_IDLE_MS    80      /* 停转这么久就把最后的值补发出去 */
#define DETAIL_KNOB_TICK_MS    50      /* 补发定时器周期：只在有脏值时运行 */
static int s_detail_knob_target = DETAIL_KNOB_BR;
static bool s_knob_dirty;              /* 有还没发出去的值（只在 LVGL 线程读写）*/
static int s_knob_pending_target = DETAIL_KNOB_BR;  /* 最新待发值：多格只留最后一格 */
static int s_knob_pending_value;
static uint32_t s_knob_last_send;
static uint32_t s_knob_last_change;
static lv_timer_t *s_knob_timer;       /* 50ms 补发定时器：有脏值才跑，发完就停 */
static bool s_knob_timer_active;
/* worker 发完一条旋钮命令后清零；LVGL 线程据此决定"再等一等"还是"补最新值" */
static volatile bool s_knob_inflight;
static void detail_knob_flush(void);
static void detail_knob_timer_park(void);
static void detail_knob_timer_cb(lv_timer_t *t);
/* 用户刚拖过滑条时，短时间内以本地值为准（HA 的轮询会慢半拍，灯关着时甚至没有 brightness） */
static bool s_sw_override_on;
static uint32_t s_sw_override_until;
static int s_br_override;
static uint32_t s_br_override_until;
static int s_ct_override;
static uint32_t s_ct_override_until;
#define OVERRIDE_MS 6000

/* 子页函数的前置声明（refresh_timer_cb 里会用到） */
static void detail_apply_entity(void);
static void detail_apply_foreground(bool dark_text);
static uint32_t s_last_gesture_ms;   /* 最近一次手势时间，用来吃掉误触 */
static QueueHandle_t s_cmd_q;

typedef struct {
    char entity[96];
    char service[24];
    char attrs[64];      /* 非空则带参数调用（亮度/色温/空调） */
    int gen;             /* 折线图请求代际：过期结果直接丢掉 */
    bool knob;           /* 旋钮命令：worker 走快路径，发完不各自等 300ms 刷新 */
} cmd_t;

/* ------------------------------------------------------------------------- */
/* 翻页                                                                       */
/* ------------------------------------------------------------------------- */

/* 标题用哪套字体：含非 ASCII（中文）就用内置 CJK 字体，纯英文用 Montserrat */
static const lv_font_t *title_font_for(const char *text)
{
    for (const unsigned char *p = (const unsigned char *)text; *p; p++) {
        if (*p >= 0x80) {
            return &lv_font_cjk16;
        }
    }
    return &lv_font_montserrat_20;
}

/* 小智助手页：只有布局里 voice.enabled=true（见 layout.c 的 voice 段）时才会建这一页，
   并且永远是最后一页（第 N+1 页）。服务端地址同样来自布局，由 main.c 的 apply_cfg()
   交给 xiaozhi_client_configure()。 */
/* 语音功能条件编译：CONFIG_APP_VOICE_ENABLED 关闭时整个小智页不建 */
#ifdef CONFIG_APP_VOICE_ENABLED
#define UI_ASSISTANT_PAGE 1
#else
#define UI_ASSISTANT_PAGE 0
#endif

static bool is_assistant_page(int page)
{
    return UI_ASSISTANT_PAGE && s_layout_valid && s_layout.voice_enabled && page == s_json_pages + 1;
}

/* 设置页：固件自己的第 N 页（原 System 页的位置），合并了系统状态、蓝牙和
   音频输出，融合后是「布局页 + 设置页 + 小智页」。 */
static bool is_settings_page(int page)
{
    return page == s_json_pages;
}

static const char *page_title(int page)
{
    if (is_settings_page(page)) {
        return "设置";
    }
    if (is_assistant_page(page)) {
        return "小智 AI";
    }
    return s_layout.pages[page].name;
}

/* 小智页是纯白底：圆点要换成深色（当前页墨色、其它页浅灰），否则白底上看不见 */
static lv_color_t dot_color(int cur_page, int dot)
{
    if (dot == cur_page) {
        return is_assistant_page(cur_page) ? lv_color_hex(0x1C1C1E) : COLOR_TEXT;
    }
    return is_assistant_page(cur_page) ? lv_color_hex(0xC7C7CC) : lv_color_hex(0x3A3A3C);
}

/* ---------------- 翻页动画 ---------------- */

#define PAGE_ANIM_MS 220

static bool s_animating;

static void anim_x_cb(void *var, int32_t v)
{
    lv_obj_set_x((lv_obj_t *)var, (lv_coord_t)v);
}

/* 位置固定、只动渲染偏移：不会被布局刷新覆盖，滑动效果也更平滑 */
static void anim_translate_cb(void *var, int32_t v)
{
    lv_obj_set_style_translate_x((lv_obj_t *)var, (lv_coord_t)v, 0);
}

static void anim_done_cb(lv_anim_t *a)
{
    /* 动画变量就是"滑出去的那一页"：动画结束必须把它真正隐藏掉。
       以前这里写的是 a->var != out（两者本来就相等），条件永远为假，
       于是翻走的页一直处于"可见"状态、只是停在 x=±360：
       小智页因此永远骗过 lv_obj_is_hidden()，表情定时器/文字刷新一直跑，
       子页面/翻页的可见性判断也跟着失真。 */
    lv_obj_t *out = (lv_obj_t *)a->var;
    if (out != NULL) {
        lv_obj_add_flag(out, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_x(out, 0);
    }
    s_animating = false;
}

/** 小智助手页在 s_pages[] 里的下标；没有这一页时返回 -1。 */
static int assistant_page_index(void)
{
    return is_assistant_page(s_json_pages + 1) ? (s_json_pages + 1) : -1;
}

/** 设置页的滚动位置复位到最上面（第一段「系统状态」）。 */
static void settings_scroll_reset(void)
{
    if (s_set_scroll != NULL) {
        lv_obj_scroll_to_y(s_set_scroll, 0, LV_ANIM_OFF);
    }
}

/** 把所有页的可见性/位置/圆点/标题刷成"只有 s_page 可见"。构建首帧用。 */
static void pages_apply_visibility(void)
{
    for (int i = 0; i < s_page_count; i++) {
        if (s_pages[i] == NULL) {          /* 这一页没建出来（LVGL 池不够）：跳过 */
            continue;
        }
        lv_obj_set_x(s_pages[i], 0);
        if (i == s_page) {
            lv_obj_remove_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
        }
        if (s_dots[i]) {
            lv_obj_set_style_bg_color(s_dots[i], dot_color(s_page, i), 0);
        }
    }
    if (s_title) {
        const char *title = page_title(s_page);
        lv_label_set_text(s_title, title);
        lv_obj_set_style_text_font(s_title, title_font_for(title), 0);
        lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 30);
    }
    int visible = 0;
    for (int i = 0; i < s_page_count; i++) {
        if (s_pages[i] != NULL && !lv_obj_is_hidden(s_pages[i])) {
            visible++;
        }
    }
    assistant_ui_set_visible(s_page == assistant_page_index());
    ESP_LOGI(TAG, "showing page %d/%d (visible=%d)", s_page, s_page_count, visible);
}

/** 切换页面：dir > 0 表示"下一页"（新页从右侧滑入），< 0 反之。 */
static void show_page_dir(int page, int dir)
{
    if (s_page_count <= 0) {
        return;
    }
    if (page < 0) {
        page = s_page_count - 1;
    }
    page %= s_page_count;

    const int prev = s_page;
    if (page == prev) {
        return;
    }
    if (s_animating) {
        return;   /* 动画中忽略，避免页面叠在一起 */
    }

    /* 防御：页面没建出来（比如 LVGL 池耗尽）时绝不碰 NULL。
       新工程 LV_USE_ASSERT_* 全关，LVGL 不会替我们挡空指针。 */
    lv_obj_t *in = s_pages[page];
    lv_obj_t *out = (prev >= 0 && prev < s_page_count) ? s_pages[prev] : NULL;
    if (in == NULL) {
        ESP_LOGW(TAG, "页面 %d 还没建好，忽略翻页", page);
        return;
    }
    s_page = page;

    /* 进设置页：滚动位置回到最上面（系统状态），蓝牙状态立刻刷一次不等 500ms */
    if (is_settings_page(page)) {
        settings_scroll_reset();
        refresh_bt_ui();
    }

    /* 圆点 + 标题立刻更新，不跟着动画走 */
    for (int i = 0; i < s_page_count; i++) {
        if (s_dots[i]) {
            lv_obj_set_style_bg_color(s_dots[i], dot_color(page, i), 0);
        }
    }
    if (s_title) {
        const char *title = page_title(page);
        lv_label_set_text(s_title, title);
        lv_obj_set_style_text_font(s_title, title_font_for(title), 0);
        lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 30);
    }

    /* 小智页的可见性：这里只改标志 + 起停定时器，不碰任何 LVGL 对象
       （本函数会在事件回调里被调用，避免在翻页过程中重排/改标签）。 */
    assistant_ui_set_visible(page == assistant_page_index());

    const lv_coord_t dist = (dir >= 0) ? 360 : -360;

    /* 新页先放到屏幕外，再滑到 0 */
    lv_obj_remove_flag(in, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_x(in, dist);

    s_animating = true;

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, in);
    lv_anim_set_values(&a, dist, 0);
    lv_anim_set_time(&a, PAGE_ANIM_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, anim_x_cb);
    lv_anim_start(&a);

    if (out != NULL && out != in) {
        lv_anim_t b;
        lv_anim_init(&b);
        lv_anim_set_var(&b, out);
        lv_anim_set_values(&b, 0, -dist);
        lv_anim_set_time(&b, PAGE_ANIM_MS);
        lv_anim_set_path_cb(&b, lv_anim_path_ease_out);
        lv_anim_set_exec_cb(&b, anim_x_cb);
        lv_anim_set_completed_cb(&b, anim_done_cb);
        lv_anim_set_user_data(&b, out);
        lv_anim_start(&b);
    } else {
        s_animating = false;
    }

    /* 注意：这里不再调用 assistant_ui_tick()（那会在这个事件回调里改标签/重排）。
       小智页的表情动画起停已经由上面的 assistant_ui_set_visible() 完成。 */
}

static void show_page(int page)
{
    /* 没有方向信息时按索引大小推断（点圆点翻页走这里） */
    const int dir = (page >= s_page) ? 1 : -1;
    show_page_dir(page, dir);
}

static void dot_clicked_cb(lv_event_t *e)
{
    show_page((int)(intptr_t)lv_event_get_user_data(e));
}

/* 旋钮翻页：delta>0 下一页，delta<0 上一页。
   子页面（详情/折线/空调）开着时不翻页，避免误触。 */
void ui_home_page_step(int delta)
{
    if (delta == 0 || s_page_count <= 0) return;
    if (ui_home_detail_open() || ui_home_chart_open() || ui_home_climate_open()) return;
    if (delta > 0) {
        show_page_dir(s_page + 1, 1);
    } else {
        show_page_dir(s_page - 1, -1);
    }
}

/* 让手势从任意子对象冒泡到页面/屏幕（否则滑在卡片上就没反应） */
static void ui_enable_gesture_bubble(lv_obj_t *obj)
{
    lv_obj_add_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_EVENT_BUBBLE);
    const uint32_t n = lv_obj_get_child_count(obj);
    for (uint32_t i = 0; i < n; i++) {
        ui_enable_gesture_bubble(lv_obj_get_child(obj, i));
    }
}

/* 原生手势在触摸驱动上不稳定，这里直接按"按下 → 抬起"的位移判断 */
static void swipe_cb(lv_event_t *e)
{
    static lv_point_t start;
    static bool pressed;

    const lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_active();
    if (indev == NULL) {
        return;
    }

    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (code == LV_EVENT_PRESSED) {
        start = p;
        pressed = true;
        /* 记下起手点是否落在滑条上（拖滑条不能被当成滑动） */
        lv_obj_t *target = lv_event_get_target(e);
        s_press_on_slider = (target != NULL && lv_obj_check_type(target, &lv_slider_class));
        return;
    }
    if (code != LV_EVENT_RELEASED || !pressed) {
        return;
    }
    pressed = false;
    if (s_press_on_slider) {
        s_press_on_slider = false;
        return;                 /* 这一下是在滑条上，不算翻页/返回 */
    }

    const int dx = p.x - start.x;
    const int dy = p.y - start.y;
    ESP_LOGD(TAG, "swipe dx=%d dy=%d", dx, dy);

    if (abs(dx) < 40 || abs(dx) < abs(dy)) {
        return;
    }
    if (ui_home_climate_open()) {
        if (dx > 40 && start.x < 70) {
            ui_home_close_climate();
        }
        return;
    }
    if (ui_home_chart_open()) {
        if (dx > 40 && start.x < 70) {
            ui_home_close_chart();
        }
        return;
    }
    if (ui_home_detail_open()) {
        /* 只有从屏幕左边缘起手的右滑才当"返回"，否则拖滑条会被误判 */
        if (dx > 40 && start.x < 70) {
            ui_home_close_detail();
        }
        return;
    }
    const uint32_t now = lv_tick_get();
    if (now - s_last_gesture_ms < 400) {
        return;
    }
    s_last_gesture_ms = now;
    ESP_LOGI(TAG, "swipe dx=%d -> page %d", dx, (dx < 0 ? s_page + 1 : s_page - 1));
    if (dx < 0) {
        show_page_dir(s_page + 1, 1);
    } else {
        show_page_dir(s_page - 1, -1);
    }
}

static void gesture_cb(lv_event_t *e)
{
    (void)e;
    lv_indev_t *indev = lv_indev_active();
    if (indev == NULL) {
        return;
    }
    /* 去抖：一次滑动可能冒泡出多个 GESTURE，避免连翻两页 */
    const uint32_t now = lv_tick_get();
    if (now - s_last_gesture_ms < 400) {
        return;
    }
    s_last_gesture_ms = now;
    switch (lv_indev_get_gesture_dir(indev)) {
    case LV_DIR_LEFT:
        show_page_dir(s_page + 1, 1);
        break;
    case LV_DIR_RIGHT:
        show_page_dir(s_page - 1, -1);
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* tile                                                                       */
/* ------------------------------------------------------------------------- */

static void tile_clicked_cb(lv_event_t *e)
{
    tile_widget_t *w = (tile_widget_t *)lv_event_get_user_data(e);
    if (w == NULL || s_cmd_q == NULL) {
        return;
    }
    if (lv_tick_get() - s_last_gesture_ms < 400) {
        return;   /* 刚滑动过，忽略这次"点击" */
    }
    ESP_LOGI(TAG, "tile 点击: %s type=%d (detail=%p)", w->entity, (int)w->type, (void *)s_detail);

    /* 传感器：点开折线图（历史数据由后台任务拉取） */
    if (w->type == TILE_SENSOR) {
        /* 已经在看同一张图：不重复请求 */
        if (s_chart_visible && strcmp(s_chart_entity, w->entity) == 0) {
            return;
        }
        /* 缓存命中：直接画，不发请求 */
        if (strcmp(s_chart_cache_entity, w->entity) == 0 && s_chart_cache_n > 0 &&
            (lv_tick_get() - s_chart_cache_at) < 60000) {
            ui_home_show_chart(w->entity, s_chart_cache_vals, s_chart_cache_n,
                               s_chart_cache_min, s_chart_cache_max, s_chart_cache_avg);
            ESP_LOGI(TAG, "折线图命中缓存: %s", w->entity);
            return;
        }

        cmd_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        snprintf(cmd.entity, sizeof(cmd.entity), "%s", w->entity);
        snprintf(cmd.service, sizeof(cmd.service), "chart");
        s_chart_gen++;
        cmd.gen = s_chart_gen;
        s_chart_wanted = true;
        bg_image_set_paused(true);      /* 让出 TLS/内存给历史请求 */
        xQueueSend(s_cmd_q, &cmd, 0);
        ESP_LOGI(TAG, "请求折线图: %s (gen %d)", w->entity, cmd.gen);
        if (s_chart_page != NULL && !s_chart_visible) {
            snprintf(s_chart_entity, sizeof(s_chart_entity), "%s", w->entity);
            const ha_entity_t *ent = ha_client_find(w->entity);
            lv_label_set_text(s_chart_title, ent ? ent->name : w->entity);
            lv_label_set_text(s_chart_span, "加载中…");
            lv_label_set_text(s_chart_min, "");
            lv_label_set_text(s_chart_avg, "");
            lv_label_set_text(s_chart_max, "");
            s_chart_pts_n = 0;               /* 清掉上一张图的线 */
            lv_obj_invalidate(s_chart);
            s_chart_visible = true;
            lv_obj_set_pos(s_chart_page, 0, 0);
            lv_obj_set_style_translate_x(s_chart_page, 360, 0);
            lv_obj_remove_flag(s_chart_page, LV_OBJ_FLAG_HIDDEN);

            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, s_chart_page);
            lv_anim_set_values(&a, 360, 0);
            lv_anim_set_time(&a, 220);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
            lv_anim_set_exec_cb(&a, anim_translate_cb);
            lv_anim_start(&a);
        }
        return;
    }

    /* 空调：点开专属页（温度/模式/风速） */
    if (strncmp(w->entity, "climate.", 8) == 0) {
        ui_home_open_climate(w->entity);
        return;
    }

    /* 灯：点开详情页（亮度/色温）；其它实体保持直接动作 */
    if (strncmp(w->entity, "light.", 6) == 0) {
        ui_home_open_detail(w->entity);
        return;
    }

    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", w->entity);
    snprintf(cmd.service, sizeof(cmd.service), "%s", (w->type == TILE_SCENE) ? "turn_on" : "toggle");
    xQueueSend(s_cmd_q, &cmd, 0);
}

static void make_tile(lv_obj_t *parent, int page, int slot, int col, int row)
{
    const layout_tile_t *lt = &s_layout.pages[page].tiles[slot];
    tile_widget_t *w = &s_tiles[page][slot];

    memset(w, 0, sizeof(*w));
    snprintf(w->entity, sizeof(w->entity), "%s", lt->entity);
    w->type = lt->type;

    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, CARD_W, CARD_H);
    /* 坐标相对滚动容器：左右各留 22px，行距 = 卡片高 + 间隙 */
    lv_obj_set_pos(card, 22 + col * (CARD_W + CARD_GAP), row * (CARD_H + CARD_GAP));
    /* 卡片是黑色半透明：叠在"背景图 + 暗化层"之上会更暗，形成玻璃质感 */
    lv_obj_set_style_bg_color(card, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(card, (lv_opa_t)(bg_image_card_opa_pct() * 255 / 100), 0);
    lv_obj_set_style_radius(card, 22, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, COLOR_BORDER, 0);
    lv_obj_set_style_pad_all(card, 12, 0);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(card, tile_clicked_cb, LV_EVENT_CLICKED, w);

    lv_obj_t *label = lv_label_create(card);
    lv_label_set_text(label, lt->label[0] ? lt->label : lt->entity);
    lv_obj_set_style_text_font(label, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(label, COLOR_TEXT, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_set_size(label, CARD_W - 24, 42); /* 最多两行，超出打点 */
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *value = lv_label_create(card);
    lv_label_set_text(value, "--");
    lv_obj_set_style_text_font(value, &lv_font_montserrat_20, 0);
    if (lt->type == TILE_SENSOR) {
        lv_obj_set_style_text_color(value, COLOR_TEXT, 0);
        lv_obj_align(value, LV_ALIGN_LEFT_MID, 0, 8);
        lv_obj_set_width(value, CARD_W - 24);
    } else {
        lv_obj_set_style_text_color(value, COLOR_MUTED, 0);
        lv_obj_align(value, LV_ALIGN_BOTTOM_LEFT, 0, 0);

        lv_obj_t *dot = lv_obj_create(card);
        lv_obj_set_size(dot, 10, 10);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, COLOR_BAD, 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_align(dot, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
        w->dot = dot;
    }

    w->card = card;
    w->label = label;
    w->value = value;
}

/** WiFi 图标：三道弧 + 一个点，点亮的弧数量表示信号格数（1~3 格）。 */
static void wifi_icon_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    if (layer == NULL) {
        return;
    }
    lv_area_t a;
    lv_obj_get_coords(obj, &a);

    const int radii[3] = {9, 6, 3};      /* 外 -> 内 */
    const int cx = (a.x1 + a.x2) / 2;
    const int cy = a.y2 - 2;

    for (int i = 0; i < 3; i++) {
        /* 外弧对应第 1 格：level>=1 时点亮最外圈，依此类推 */
        const int need = 3 - i;          /* 外弧需要 3 格中的最高档 */
        const bool lit = (s_wifi_level >= need) && (s_wifi_level > 0);

        lv_draw_arc_dsc_t d;
        lv_draw_arc_dsc_init(&d);
        d.color = lit ? s_wifi_color : lv_color_hex(0x3A3A3C);
        d.width = 2;
        d.rounded = 1;
        d.start_angle = 205;
        d.end_angle = 335;
        d.center.x = cx;
        d.center.y = cy;
        d.radius = radii[i];
        lv_draw_arc(layer, &d);
    }

    lv_draw_rect_dsc_t rd;
    lv_draw_rect_dsc_init(&rd);
    rd.bg_color = s_wifi_level > 0 ? s_wifi_color : lv_color_hex(0x3A3A3C);
    rd.bg_opa = LV_OPA_COVER;
    rd.radius = 1;
    lv_area_t dot = {cx - 1, cy - 1, cx + 1, cy + 1};
    lv_draw_rect(layer, &rd, &dot);
}

/* ------------------------------------------------------------------------- */
/* 设置页的公共部分：OTA 回调 / 建页失败报告                                   */
/* ------------------------------------------------------------------------- */

static void ota_button_cb(lv_event_t *e)
{
    (void)e;
    if (!ota_busy()) {
        ota_start_check();
    }
}

/**
 * 页面对象没建出来时，先把原因查清楚再报。
 * lv_obj_create() 返回 NULL 不一定是内存不足：父对象无效（最常见的
 * lv_screen_active() 为 NULL——LVGL 根本没建 display）时它同样返回 NULL，
 * 而这时 LVGL 池可能几乎全空。以前这里一律写"内存不足"，把人带偏过。
 */
static void report_page_create_failed(const char *name, lv_obj_t *parent)
{
    lv_mem_monitor_t mon;
    lv_mem_monitor(&mon);

    if (parent == NULL) {
        ESP_LOGE(TAG, "%s创建失败：没有活动屏幕（lv_screen_active() 返回 NULL，"
                      "先查 lv_display_create() 是否成功），本页跳过", name);
        return;
    }
    if (!lv_obj_is_in_widget_tree(parent)) {
        ESP_LOGE(TAG, "%s创建失败：父对象 %p 不在控件树里（显示/屏幕已失效），本页跳过",
                 name, (void *)parent);
        return;
    }
    if (mon.free_size < 1024 || mon.free_biggest_size < 1024) {
        ESP_LOGE(TAG, "%s创建失败（LVGL 内存不足：空闲 %u/%u 字节，最大块 %u，碎片 %u%%），本页跳过",
                 name, (unsigned)mon.free_size, (unsigned)mon.total_size,
                 (unsigned)mon.free_biggest_size, (unsigned)mon.frag_pct);
        return;
    }
    ESP_LOGE(TAG, "%s创建失败，但 LVGL 池还空闲 %u/%u 字节（最大块 %u，碎片 %u%%）"
                  "——不是内存不足，父对象 %p 在控件树里，需查 LVGL 内部状态",
             name, (unsigned)mon.free_size, (unsigned)mon.total_size,
             (unsigned)mon.free_biggest_size, (unsigned)mon.frag_pct, (void *)parent);
}

/* ---- 小智助手页（最后一页）的附加控件：音频自检 ---- */
static lv_obj_t *s_selftest_label;

/* 自检任务跑在独立任务里（录音 1.5s + 回放，不能阻塞 LVGL）。
   约定和 net.c 一样：任务只写静态变量，LVGL 线程在 refresh_selftest_label() 里读。 */
static volatile int s_voice_test_state;   /* 0=空闲 1=请求中 2=完成 3=失败 */
static int s_voice_test_peak;
static uint32_t s_voice_test_until;       /* 结果在按钮上保留到什么时候（lv_tick） */
static char s_selftest_shown[24];         /* 按钮当前文字（避免 500ms 重复重排） */

#if UI_ASSISTANT_PAGE
static void voice_test_task(void *arg)
{
    (void)arg;
    int peak = 0, rms = 0;
    const int rc = voice_audio_selftest(&peak, &rms);
    s_voice_test_peak = peak;
    s_voice_test_state = (rc == 0) ? 2 : 3;
    /* 按钮上只放峰值（宽度有限），峰值 + RMS 完整结果在日志里 */
    ESP_LOGI(TAG, "音频自检结果：峰值 %d / RMS %d（%s）", peak, rms, rc == 0 ? "成功" : "失败");
    vTaskDelete(NULL);
}

static void selftest_set_label(const char *text)
{
    if (s_selftest_label == NULL || strcmp(s_selftest_shown, text) == 0) {
        return;
    }
    snprintf(s_selftest_shown, sizeof(s_selftest_shown), "%s", text);
    lv_label_set_text(s_selftest_label, text);
}

static void voice_test_btn_cb(lv_event_t *e)
{
    (void)e;
    if (s_voice_test_state == 1) {
        return;                 /* 已经在跑了 */
    }
    s_voice_test_state = 1;
    s_voice_test_until = 0;
    selftest_set_label("录音中…");
    ESP_LOGI(TAG, "音频自检：开始录音");
    /* 音频只占一路 I2S + 少量 PSRAM，任务栈给 6 KB 足够 */
    if (xTaskCreate(voice_test_task, "audio_test", 6144, NULL, 4, NULL) != pdPASS) {
        s_voice_test_state = 3;
        ESP_LOGW(TAG, "音频自检任务创建失败");
    }
}

/** 按钮文字跟着自检状态走：录音中… → 峰值 xxx（保留 5 秒）→ 音频自检。 */
static void refresh_selftest_label(void)
{
    const int st = s_voice_test_state;
    if (st == 1) {
        selftest_set_label("录音中…");
        return;
    }
    if (st == 2 || st == 3) {
        if (s_voice_test_until == 0) {
            s_voice_test_until = lv_tick_get() + 5000;     /* 500ms/轮 → 约 5 秒 */
        }
        if ((int32_t)(lv_tick_get() - s_voice_test_until) < 0) {
            char buf[24];
            if (st == 2) {
                snprintf(buf, sizeof(buf), "峰值 %d", s_voice_test_peak);
            } else {
                snprintf(buf, sizeof(buf), "自检失败");
            }
            selftest_set_label(buf);
            return;
        }
        s_voice_test_state = 0;         /* 5 秒到了：复位，下次还能点 */
    }
    selftest_set_label("音频自检");
}

/**
 * 最后一页：小智助手页。
 * 页面对象交给 assistant_ui_build() 布置（纯白底、360x360、整页接触摸/轻触说话），
 * 这里只补一个「音频自检」小按钮——原来语音页底部的那个，挪到白页顶部（脸的上面）。
 */
static void build_assistant_page(lv_obj_t *scr)
{
    lv_obj_t *page = lv_obj_create(scr);
    if (page == NULL) {
        report_page_create_failed("小智页", scr);
        return;
    }
    lv_obj_set_size(page, 360, 360);
    lv_obj_set_pos(page, 0, 0);
    s_pages[s_json_pages + 1] = page;

    /* 内容全部由 assistant_ui 画进来；失败时它会把内部指针清空，这一页就空着 */
    assistant_ui_build(page);


    lv_obj_t *btn = lv_button_create(page);
    lv_obj_set_size(btn, 110, 30);
    /* 水平居中（x 偏移 0）；y=26 时芯片左右各占 125..235，
       圆屏顶部 y=26 可见宽度约 87..273，整块都在圆内。 */
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 26);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xF2F2F7), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn, 15, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_add_event_cb(btn, voice_test_btn_cb, LV_EVENT_CLICKED, NULL);

    s_selftest_label = lv_label_create(btn);
    lv_obj_set_style_text_font(s_selftest_label, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_selftest_label, lv_color_hex(0x3A3A3C), 0);
    lv_obj_center(s_selftest_label);
    s_selftest_shown[0] = '\0';
    selftest_set_label("音频自检");
}
#endif /* UI_ASSISTANT_PAGE */

/* ------------------------------------------------------------------------- */
/* 设置页（第 N 页：系统状态 + 蓝牙 + 音频输出）与蓝牙状态图标                */
/* ------------------------------------------------------------------------- */

/**
 * 顶部状态行的蓝牙图标：协议栈未启用/关闭 = 灰，扫描中/已连接 = 蓝。
 * 颜色每 500ms 由 refresh_timer_cb -> refresh_bt_ui() 刷一次。
 */
static void refresh_bt_icon(void)
{
    if (s_bt_icon == NULL) {
        return;
    }
    const bt_state_t st = bt_link_state();
    const bool active = bt_link_is_enabled() && (st == BT_SCANNING || st == BT_CONNECTED);
    lv_obj_set_style_text_color(s_bt_icon, active ? COLOR_ACCENT : COLOR_MUTED, 0);
}

/** 设置页标签：统一 CJK 字体；x<0 或 y<0 表示在父对象里居中。失败返回 NULL。 */
static lv_obj_t *settings_label(lv_obj_t *parent, const char *text, int x, int y, int w,
                                lv_color_t color)
{
    if (parent == NULL) {
        return NULL;
    }
    lv_obj_t *lab = lv_label_create(parent);
    if (lab == NULL) {
        return NULL;
    }
    lv_label_set_text(lab, text);
    lv_obj_set_style_text_font(lab, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(lab, color, 0);
    if (w > 0) {
        lv_obj_set_width(lab, w);
    }
    if (x < 0 || y < 0) {
        lv_obj_center(lab);
    } else {
        lv_obj_set_pos(lab, x, y);
    }
    return lab;
}

/* 设置页的列表尺寸：容器 256 宽，右边留 8px 给 4px 滚动条 */
#define SET_ROW_W      248
#define SET_ROW_H       44    /* 标准设置行高（参考常见设置列表） */
#define SET_ROW_GAP      8
#define SET_ROW_PAD_X   14
#define SET_HEAD_H      30    /* 分组标题占位：16px 字 + 下方留白 */
#define SET_SEC_GAP     14    /* 分组之间额外留白 */
#define SET_END_PAD     24    /* 滚动内容末尾留白，最后一行不贴圆边 */

/**
 * 设置页的一条设置行：整行一张小卡片（深灰半透明 + 圆角 12），
 * 背景图能透出来一点，和 tile 卡的玻璃质感一致。失败返回 NULL。
 */
static lv_obj_t *settings_row(lv_obj_t *parent, int y, int h)
{
    if (parent == NULL) {
        return NULL;
    }
    lv_obj_t *row = lv_obj_create(parent);
    if (row == NULL) {
        return NULL;
    }
    lv_obj_remove_style_all(row);
    lv_obj_set_pos(row, 0, y);
    lv_obj_set_size(row, SET_ROW_W, h);
    lv_obj_set_style_bg_color(row, COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(row, (lv_opa_t)140, 0);      /* 约 55% */
    lv_obj_set_style_radius(row, 12, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, COLOR_BORDER, 0);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    return row;
}

/** 行左侧标题（16px CJK 白字，垂直居中）。 */
static lv_obj_t *settings_row_title(lv_obj_t *row, const char *text)
{
    lv_obj_t *lab = settings_label(row, text, 0, -1, 0, COLOR_TEXT);
    if (lab != NULL) {
        lv_obj_align(lab, LV_ALIGN_LEFT_MID, SET_ROW_PAD_X, 0);
    }
    return lab;
}

/** 行右侧的值文字（灰字、右对齐、垂直居中）；w 是最大宽度。 */
static lv_obj_t *settings_row_value(lv_obj_t *row, int w)
{
    lv_obj_t *lab = settings_label(row, "--", 0, -1, w, COLOR_MUTED);
    if (lab != NULL) {
        lv_obj_set_style_text_align(lab, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(lab, LV_ALIGN_RIGHT_MID, -SET_ROW_PAD_X, 0);
    }
    return lab;
}

/** 分组标题：同样的 16px 中文字体，用灰色区分层级；返回下一行的 y。 */
static int settings_heading(lv_obj_t *parent, int y, const char *text)
{
    if (parent != NULL) {
        settings_label(parent, text, SET_ROW_PAD_X, y + 4, 0, COLOR_MUTED);
    }
    return y + SET_HEAD_H;
}

/** 扫描 / 连接按钮的可用态：可用=蓝底白字，禁用=深灰底灰字 + LV_STATE_DISABLED。 */
static void settings_apply_btn_enabled(lv_obj_t *btn, lv_obj_t *label, bool en)
{
    if (btn == NULL) {
        return;
    }
    lv_obj_set_state(btn, LV_STATE_DISABLED, !en);
    lv_obj_set_style_bg_color(btn, en ? COLOR_ACCENT : lv_color_hex(0x2C2C2E), 0);
    if (label != NULL) {
        lv_obj_set_style_text_color(label, en ? COLOR_TEXT : COLOR_MUTED, 0);
    }
}

/** 音频输出两个选项的高亮 + 「仍走耳机孔」提示的显隐。 */
static void settings_apply_audio(void)
{
    const bool bt = bt_link_audio_to_bt();
    const lv_color_t on_bg = COLOR_ACCENT;
    const lv_color_t off_bg = lv_color_hex(0x2C2C2E);

    if (s_set_phone_btn != NULL) {
        lv_obj_set_style_bg_color(s_set_phone_btn, bt ? off_bg : on_bg, 0);
    }
    if (s_set_phone_label != NULL) {
        lv_obj_set_style_text_color(s_set_phone_label, bt ? COLOR_MUTED : COLOR_TEXT, 0);
    }
    if (s_set_bt_btn != NULL) {
        lv_obj_set_style_bg_color(s_set_bt_btn, bt ? on_bg : off_bg, 0);
    }
    if (s_set_bt_label != NULL) {
        lv_obj_set_style_text_color(s_set_bt_label, bt ? COLOR_TEXT : COLOR_MUTED, 0);
    }
    if (s_set_audio_hint != NULL) {
        if ((int32_t)(lv_tick_get() - s_set_hint_until) < 0) {
            lv_obj_remove_flag(s_set_audio_hint, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_set_audio_hint, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void settings_scan_cb(lv_event_t *e)
{
    (void)e;
    if (!bt_link_is_enabled()) {
        /* 按钮本来就置灰，这里再挡一道（触摸/旋钮都可能绕过样式） */
        ESP_LOGI(TAG, "蓝牙未启用（蓝牙栈未编译），扫描不可用");
        return;
    }
    if (!bt_link_scan_start()) {
        ESP_LOGW(TAG, "蓝牙扫描启动失败");
    }
    refresh_bt_ui();
}

static void settings_conn_cb(lv_event_t *e)
{
    (void)e;
    if (!bt_link_is_enabled()) {
        ESP_LOGI(TAG, "蓝牙未启用（蓝牙栈未编译），连接不可用");
        return;
    }
    if (bt_link_state() == BT_CONNECTED) {
        bt_link_disconnect();
        ESP_LOGI(TAG, "蓝牙：断开连接");
    } else if (s_set_sel_addr[0] != '\0') {
        if (!bt_link_connect(s_set_sel_addr)) {
            ESP_LOGW(TAG, "蓝牙连接失败：%s", s_set_sel_addr);
        }
    } else {
        ESP_LOGW(TAG, "蓝牙：还没有选中的音箱，先扫描");
    }
    refresh_bt_ui();
}

static void settings_audio_phone_cb(lv_event_t *e)
{
    (void)e;
    bt_link_set_audio_to_bt(false);       /* 耳机孔：幂等，值没变不写 NVS */
    s_set_hint_until = 0;
    settings_apply_audio();
}

static void settings_audio_bt_cb(lv_event_t *e)
{
    (void)e;
    bt_link_set_audio_to_bt(true);        /* 协议栈没启用时会被模块拒绝 */
    if (!bt_link_audio_to_bt()) {
        /* 选择不生效：有效输出仍是耳机孔，提示 5 秒 */
        s_set_hint_until = lv_tick_get() + 5000;
        ESP_LOGI(TAG, "设置页：蓝牙未启用，音频输出保持耳机孔");
    } else {
        s_set_hint_until = 0;
    }
    settings_apply_audio();
}

/**
 * 蓝牙相关界面刷新（500ms 定时器 + 构建完成 + 进设置页时调用）。
 * 所有控件都做 NULL 检查：页面/某个控件没建出来时只跳过对应部分。
 */
static void refresh_bt_ui(void)
{
    refresh_bt_icon();

    if (s_set_page == NULL) {
        return;
    }

    if (s_set_bt_state != NULL) {
        /* 行首已经有蓝牙图标，状态行只放状态文字（"未启用（蓝牙栈未编译）"等） */
        const char *text = bt_link_state_text();
        if (strcmp(text, s_set_state_shown) != 0) {
            snprintf(s_set_state_shown, sizeof(s_set_state_shown), "%s", text);
            lv_label_set_text(s_set_bt_state, text);
        }
    }

    /* 协议栈没启用（本版恒如此）：扫描/连接一律置灰 */
    const bool on = bt_link_is_enabled();
    settings_apply_btn_enabled(s_set_scan_btn, s_set_scan_label, on);
    settings_apply_btn_enabled(s_set_conn_btn, s_set_conn_label, on);

    const bool connected = (bt_link_state() == BT_CONNECTED);
    if (s_set_conn_label != NULL) {
        const char *text = connected ? "断开" : "连接";
        if (strcmp(lv_label_get_text(s_set_conn_label), text) != 0) {
            lv_label_set_text(s_set_conn_label, text);
        }
    }

    if (s_set_empty != NULL) {
        const char *text = on ? "点「扫描」搜索音箱" : "未启用，无法扫描";
        if (strcmp(lv_label_get_text(s_set_empty), text) != 0) {
            lv_label_set_text(s_set_empty, text);
        }
    }

    settings_apply_audio();
}

/**
 * 设置页（第 N 页，取代原来的 System 页和独立的设置页）。
 *
 * 标准设置列表：一个纵向滚动容器，里面是分组标题 + 一行行设置项。
 * 从上到下：系统状态（固件 / MQTT / WiFi / 运行 / 内存 / 电池 / 检查更新）、
 * 蓝牙（状态 / 扫描 / 连接 / 扫描结果）、音频输出（耳机孔 / 蓝牙音箱）。
 *
 * 容器在圆屏中间的安全带里：x=52..308、y=64..300（滚动条在右侧 4px）。
 * 行宽 248（右侧留 8px 给滚动条）、行高 44、行距 8。滚动位置在进页/重建时
 * 复位到顶部。滚动方向只有 VER，所以左右滑动仍然翻页；旋钮仍然是翻页
 * （见 ui_home_page_step），不在这里驱动滚动。
 */
static void build_settings_page(lv_obj_t *scr)
{
    lv_obj_t *page = lv_obj_create(scr);
    if (page == NULL) {
        report_page_create_failed("设置页", scr);
        return;
    }
    lv_obj_set_size(page, 360, 360);
    lv_obj_set_pos(page, 0, 0);
    lv_obj_set_style_bg_opa(page, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(page, 0, 0);
    lv_obj_set_style_pad_all(page, 0, 0);
    lv_obj_remove_flag(page, LV_OBJ_FLAG_SCROLLABLE);

    s_set_page = page;
    s_pages[s_json_pages] = page;

    /* ---- 纵向滚动容器：圆屏中间 x=52..308、y=64..300 的安全带 ---- */
    lv_obj_t *cont = lv_obj_create(page);
    if (cont == NULL) {
        report_page_create_failed("设置页滚动区", page);
        return;
    }
    lv_obj_remove_style_all(cont);
    lv_obj_set_pos(cont, 52, 64);
    lv_obj_set_size(cont, 256, 236);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 0, 0);
    lv_obj_add_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);      /* 只上下滚，左右留给翻页 */
    lv_obj_set_scroll_chain(cont, false);         /* 滚到头不再把滚动传给父对象 */
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_AUTO);
    /* 4px 细滚动条：圆角、半透明灰、贴右缘 */
    lv_obj_set_style_width(cont, 4, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_color(cont, COLOR_MUTED, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(cont, LV_OPA_50, LV_PART_SCROLLBAR);
    lv_obj_set_style_radius(cont, LV_RADIUS_CIRCLE, LV_PART_SCROLLBAR);
    lv_obj_set_style_border_width(cont, 0, LV_PART_SCROLLBAR);
    lv_obj_set_style_pad_right(cont, 2, LV_PART_SCROLLBAR);
    s_set_scroll = cont;

    int y = 0;

    /* ---------------- 系统状态 ---------------- */
    y = settings_heading(cont, y, "系统状态");

    static const char *const sys_names[6] = {"固件", "MQTT", "WiFi", "运行", "内存", "电池"};
    for (int i = 0; i < 6; i++) {
        lv_obj_t *row = settings_row(cont, y, SET_ROW_H);
        if (row != NULL) {
            settings_row_title(row, sys_names[i]);
            /* WiFi 行要放「-55 dBm / 192.168.1.23」两行，右值区留宽一点 */
            s_sys_lines[i] = settings_row_value(row, (i == 2) ? 176 : 150);
            if (i == 4) {
                /* 内存：百分比在行右侧，进度条压在行底部，颜色随占用变 */
                s_mem_bar = lv_bar_create(row);
                if (s_mem_bar != NULL) {
                    lv_obj_set_pos(s_mem_bar, SET_ROW_PAD_X, 34);
                    lv_obj_set_size(s_mem_bar, SET_ROW_W - SET_ROW_PAD_X * 2, 6);
                    lv_bar_set_range(s_mem_bar, 0, 100);
                    lv_obj_set_style_bg_color(s_mem_bar, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
                    lv_obj_set_style_bg_opa(s_mem_bar, LV_OPA_COVER, LV_PART_MAIN);
                    lv_obj_set_style_radius(s_mem_bar, 3, LV_PART_MAIN);
                    lv_obj_set_style_bg_color(s_mem_bar, COLOR_ACCENT, LV_PART_INDICATOR);
                    lv_obj_set_style_bg_opa(s_mem_bar, LV_OPA_COVER, LV_PART_INDICATOR);
                    lv_obj_set_style_radius(s_mem_bar, 3, LV_PART_INDICATOR);
                }
            }
        }
        y += SET_ROW_H + SET_ROW_GAP;
    }

    /* 检查更新：系统状态段末尾的整行按钮，进度条/状态文字压在行下方 */
    s_ota_btn = lv_button_create(cont);
    if (s_ota_btn != NULL) {
        lv_obj_set_pos(s_ota_btn, 0, y);
        lv_obj_set_size(s_ota_btn, SET_ROW_W, SET_ROW_H);
        lv_obj_set_style_bg_color(s_ota_btn, COLOR_ACCENT, 0);
        lv_obj_set_style_bg_opa(s_ota_btn, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(s_ota_btn, 12, 0);
        lv_obj_set_style_shadow_width(s_ota_btn, 0, 0);
        lv_obj_add_event_cb(s_ota_btn, ota_button_cb, LV_EVENT_CLICKED, NULL);

        s_ota_btn_label = settings_label(s_ota_btn, "检查更新", 0, -1, 0, COLOR_TEXT);
        if (s_ota_btn_label != NULL) {
            lv_obj_align(s_ota_btn_label, LV_ALIGN_LEFT_MID, SET_ROW_PAD_X, 0);
        }
        lv_obj_t *ota_icon = lv_label_create(s_ota_btn);
        if (ota_icon != NULL) {
            lv_label_set_text(ota_icon, LV_SYMBOL_DOWNLOAD);
            lv_obj_set_style_text_font(ota_icon, &lv_font_montserrat_20, 0);
            lv_obj_set_style_text_color(ota_icon, COLOR_TEXT, 0);
            lv_obj_align(ota_icon, LV_ALIGN_RIGHT_MID, -SET_ROW_PAD_X, 0);
        }
    } else {
        ESP_LOGW(TAG, "设置页「检查更新」行创建失败（LVGL 池不够？）");
    }
    y += SET_ROW_H + 6;

    s_ota_bar = lv_bar_create(cont);
    if (s_ota_bar != NULL) {
        lv_obj_set_pos(s_ota_bar, 0, y);
        lv_obj_set_size(s_ota_bar, SET_ROW_W, 8);
        lv_bar_set_range(s_ota_bar, 0, 100);
        lv_obj_set_style_bg_color(s_ota_bar, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(s_ota_bar, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(s_ota_bar, 4, LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_ota_bar, COLOR_ACCENT, LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(s_ota_bar, LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_radius(s_ota_bar, 4, LV_PART_INDICATOR);
        lv_obj_add_flag(s_ota_bar, LV_OBJ_FLAG_HIDDEN);
    }
    y += 8 + 6;

    s_ota_status = settings_label(cont, "", SET_ROW_PAD_X, y, SET_ROW_W - SET_ROW_PAD_X * 2,
                                  COLOR_MUTED);
    if (s_ota_status != NULL) {
        lv_obj_set_style_text_align(s_ota_status, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_flag(s_ota_status, LV_OBJ_FLAG_HIDDEN);
    }
    y += 21 + SET_SEC_GAP;

    /* ---------------- 蓝牙 ---------------- */
    y = settings_heading(cont, y, "蓝牙");

    lv_obj_t *bt_row = settings_row(cont, y, SET_ROW_H);
    if (bt_row != NULL) {
        lv_obj_t *icon = lv_label_create(bt_row);
        if (icon != NULL) {
            lv_label_set_text(icon, LV_SYMBOL_BLUETOOTH);
            lv_obj_set_style_text_font(icon, &lv_font_montserrat_20, 0);
            lv_obj_set_style_text_color(icon, COLOR_MUTED, 0);
            lv_obj_align(icon, LV_ALIGN_LEFT_MID, SET_ROW_PAD_X - 2, 0);
        }
        s_set_bt_state = settings_label(bt_row, "未启用（蓝牙栈未编译）", 0, -1,
                                        SET_ROW_W - 40 - SET_ROW_PAD_X, COLOR_TEXT);
        if (s_set_bt_state != NULL) {
            lv_label_set_long_mode(s_set_bt_state, LV_LABEL_LONG_DOT);
            lv_obj_align(s_set_bt_state, LV_ALIGN_LEFT_MID, 40, 0);
        }
    }
    y += SET_ROW_H + SET_ROW_GAP;

    lv_obj_t *bt_btns = settings_row(cont, y, SET_ROW_H);
    if (bt_btns != NULL) {
        s_set_scan_btn = lv_button_create(bt_btns);
        if (s_set_scan_btn != NULL) {
            lv_obj_set_size(s_set_scan_btn, 102, 30);
            lv_obj_set_pos(s_set_scan_btn, SET_ROW_PAD_X, 7);
            lv_obj_set_style_radius(s_set_scan_btn, 10, 0);
            lv_obj_set_style_shadow_width(s_set_scan_btn, 0, 0);
            lv_obj_add_event_cb(s_set_scan_btn, settings_scan_cb, LV_EVENT_CLICKED, NULL);
            s_set_scan_label = settings_label(s_set_scan_btn, "扫描", -1, -1, 0, COLOR_TEXT);
        }

        s_set_conn_btn = lv_button_create(bt_btns);
        if (s_set_conn_btn != NULL) {
            lv_obj_set_size(s_set_conn_btn, 102, 30);
            lv_obj_set_pos(s_set_conn_btn, 132, 7);
            lv_obj_set_style_radius(s_set_conn_btn, 10, 0);
            lv_obj_set_style_shadow_width(s_set_conn_btn, 0, 0);
            lv_obj_add_event_cb(s_set_conn_btn, settings_conn_cb, LV_EVENT_CLICKED, NULL);
            s_set_conn_label = settings_label(s_set_conn_btn, "连接", -1, -1, 0, COLOR_TEXT);
        }
    }
    y += SET_ROW_H + SET_ROW_GAP;

    /* 扫描结果区：协议栈没启用时只有空状态文字（本版永远如此）。
       下一轮：TODO(bt) 扫描到音箱后在这里建可点选的行，选中的 MAC 存 s_set_sel_addr。 */
    lv_obj_t *res_row = settings_row(cont, y, 36);
    if (res_row != NULL) {
        s_set_empty = settings_label(res_row, "未启用，无法扫描", 0, -1, 0, COLOR_MUTED);
        if (s_set_empty != NULL) {
            lv_obj_align(s_set_empty, LV_ALIGN_LEFT_MID, SET_ROW_PAD_X, 0);
        }
    }
    y += 36 + SET_SEC_GAP;

    /* ---------------- 音频输出 ---------------- */
    y = settings_heading(cont, y, "音频输出");

    lv_obj_t *audio_row = settings_row(cont, y, SET_ROW_H);
    if (audio_row != NULL) {
        s_set_phone_btn = lv_button_create(audio_row);
        if (s_set_phone_btn != NULL) {
            lv_obj_set_size(s_set_phone_btn, 94, 30);
            lv_obj_set_pos(s_set_phone_btn, SET_ROW_PAD_X, 7);
            lv_obj_set_style_radius(s_set_phone_btn, 15, 0);
            lv_obj_set_style_shadow_width(s_set_phone_btn, 0, 0);
            lv_obj_add_event_cb(s_set_phone_btn, settings_audio_phone_cb, LV_EVENT_CLICKED, NULL);
            s_set_phone_label = settings_label(s_set_phone_btn, "耳机孔", -1, -1, 0, COLOR_TEXT);
        }

        s_set_bt_btn = lv_button_create(audio_row);
        if (s_set_bt_btn != NULL) {
            lv_obj_set_size(s_set_bt_btn, 108, 30);
            lv_obj_set_pos(s_set_bt_btn, 124, 7);
            lv_obj_set_style_radius(s_set_bt_btn, 15, 0);
            lv_obj_set_style_shadow_width(s_set_bt_btn, 0, 0);
            lv_obj_add_event_cb(s_set_bt_btn, settings_audio_bt_cb, LV_EVENT_CLICKED, NULL);
            s_set_bt_label = settings_label(s_set_bt_btn, "蓝牙音箱", -1, -1, 0, COLOR_TEXT);
        }
    }
    y += SET_ROW_H + 6;

    /* 只有「选了蓝牙音箱但协议栈没启用」时才出现，5 秒后自动隐藏 */
    s_set_audio_hint = settings_label(cont, "蓝牙未启用，仍走耳机孔", SET_ROW_PAD_X, y,
                                      SET_ROW_W - SET_ROW_PAD_X * 2, lv_color_hex(0xFF9F0A));
    y += 21;

    /* 末尾垫 24px：最后一行能滚到圆屏中间，不被下弧线切掉 */
    lv_obj_t *tail = lv_obj_create(cont);
    if (tail != NULL) {
        lv_obj_remove_style_all(tail);
        lv_obj_set_pos(tail, 0, y);
        lv_obj_set_size(tail, SET_ROW_W, SET_END_PAD);
        lv_obj_remove_flag(tail, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    }

    /* 进页永远从「系统状态」开始 */
    lv_obj_scroll_to_y(cont, 0, LV_ANIM_OFF);

    /* 按真实状态刷一遍：协议栈没启用时扫描/连接就是灰的、音频高亮在耳机孔 */
    s_set_state_shown[0] = '\0';
    refresh_bt_ui();
}

/* ------------------------------------------------------------------------- */
/* 小智页导航（MQTT assistant 命令用）                                        */
/* ------------------------------------------------------------------------- */

/** 翻到小智页（最后一页）。页面不存在（布局未就绪 / voice.enabled=false）时只记日志。 */
void ui_home_goto_assistant(void)
{
    if (s_page_count <= 0 || !is_assistant_page(s_json_pages + 1)) {
        ESP_LOGW(TAG, "小智页不存在（布局未就绪或 voice.enabled=false），忽略翻页");
        return;
    }
    ESP_LOGI(TAG, "翻到小智页：第 %d/%d 页", s_json_pages + 1, s_page_count);
    show_page_dir(s_json_pages + 1, 1);
}

/** 回到第一页（MQTT 关闭小智页用）。 */
void ui_home_goto_first(void)
{
    if (s_page_count <= 0) {
        return;
    }
    ESP_LOGI(TAG, "回到第一页");
    show_page_dir(0, -1);
}

bool ui_home_assistant_current(void)
{
    return is_assistant_page(s_page);
}

/** 设置页系统状态行的值文字：NULL 安全（某个标签没建出来时只跳过）。 */
static void sys_value_set(int i, const char *text)
{
    if (i >= 0 && i < 6 && s_sys_lines[i] != NULL) {
        lv_label_set_text(s_sys_lines[i], text);
    }
}

static void refresh_status_page(void)
{
    if (s_sys_lines[0] == NULL) {
        return;
    }
    char buf[64];

    snprintf(buf, sizeof(buf), "v%s", ota_current_version());
    sys_value_set(0, buf);

    sys_value_set(1, net_mqtt_connected() ? "已连接" : "未连接");

    char ip[24] = "-";
    snprintf(ip, sizeof(ip), "%s", net_ip_string());
    /* 两行右对齐：信号强度 / IP（行高 44 里正好放得下 2×21px） */
    snprintf(buf, sizeof(buf), "%d dBm\n%s", net_wifi_rssi(), ip);
    sys_value_set(2, buf);

    const int64_t up_s = esp_timer_get_time() / 1000000;
    snprintf(buf, sizeof(buf), "%dh%02dm", (int)(up_s / 3600), (int)((up_s % 3600) / 60));
    sys_value_set(3, buf);

    {
        /* 内存：行右侧只放百分比，进度条在行底部，颜色随占用变化 */
        const size_t mem_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
        const size_t mem_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        const int used_pct = mem_total ? (int)((mem_total - mem_free) * 100 / mem_total) : 0;
        snprintf(buf, sizeof(buf), "%d%%", used_pct);
        sys_value_set(4, buf);
        if (s_mem_bar) {
            lv_bar_set_value(s_mem_bar, used_pct, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(s_mem_bar,
                used_pct > 85 ? lv_color_hex(0xFF453A)
                : (used_pct > 70 ? lv_color_hex(0xFFD60A) : COLOR_ACCENT), LV_PART_INDICATOR);
        }
    }

    /* 电池：ESP32-S31 只有单档衰减且无校准方案，先显示原始值（未标定） */
    {
        const int raw = battery_raw_value();
        if (raw >= 0) {
            snprintf(buf, sizeof(buf), "raw %d", raw);
        } else {
            snprintf(buf, sizeof(buf), "--");
        }
        sys_value_set(5, buf);
    }

    /* OTA 行/进度条/状态文字：任一没建出来就只跳过这一段 */
    if (s_ota_bar == NULL || s_ota_btn_label == NULL || s_ota_status == NULL) {
        return;
    }
    if (ota_busy()) {
        lv_obj_remove_flag(s_ota_bar, LV_OBJ_FLAG_HIDDEN);
        const int p = ota_percent();
        lv_bar_set_value(s_ota_bar, p < 0 ? 0 : p, LV_ANIM_OFF);
        lv_label_set_text(s_ota_btn_label, p < 0 ? "检查中" : "升级中");
    } else {
        lv_obj_add_flag(s_ota_bar, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(s_ota_btn_label, "检查更新");
    }
    const char *ota_text = ota_status_text();
    lv_label_set_text(s_ota_status, ota_text);
    if (ota_text[0] != '\0') {
        lv_obj_remove_flag(s_ota_status, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_ota_status, LV_OBJ_FLAG_HIDDEN);
    }
}

/* ------------------------------------------------------------------------- */
/* 刷新                                                                       */
/* ------------------------------------------------------------------------- */

/** 状态文本可能含中文（天气"晴"、模式名等）：非 ASCII 就用 CJK 字体的较小字号。 */
static const lv_font_t *value_font_for(const char *text)
{
    if (text == NULL) {
        return &lv_font_cjk16;
    }
    /* 统一用同一个字体（这版 CJK 字库自带 ASCII 数字/字母），
       这样"晴"和"56.0 W"的字号完全一致，不会一大一小 */
    (void)text;
    return &lv_font_cjk16;
}

/**
 * HA 的天气状态是英文机器码（sunny / partlycloudy / rainy…），卡片上要显示中文。
 * 只对 weather.* 实体做映射；大小写不敏感、忽略首尾空白，未知状态原样返回
 * （HA 以后新增的天气类型仍然能显示，只是还是英文，也不打日志）。
 * 纯静态表 + 栈上小缓冲，不分配内存。
 */
static const char *weather_state_zh(const char *entity_id, const char *state)
{
    static const struct {
        const char *ha;
        const char *zh;
    } map[] = {
        {"clear-night", "晴（夜）"},
        {"cloudy", "多云"},
        {"exceptional", "异常天气"},
        {"fog", "雾"},
        {"hail", "冰雹"},
        {"lightning", "闪电"},
        {"lightning-rainy", "雷阵雨"},
        {"partlycloudy", "局部多云"},
        {"pouring", "暴雨"},
        {"rainy", "雨"},
        {"snowy", "雪"},
        {"snowy-rainy", "雨夹雪"},
        {"sunny", "晴"},
        {"windy", "大风"},
        {"windy-variant", "大风"},
    };

    if (entity_id == NULL || state == NULL || strncmp(entity_id, "weather.", 8) != 0) {
        return state;
    }
    /* 去掉首尾空白（HA 正常不带，这里防御一下），拷进小缓冲后再 strcasecmp */
    while (*state == ' ' || *state == '\t') {
        state++;
    }
    char tok[24];
    size_t n = 0;
    while (state[n] != '\0' && state[n] != ' ' && state[n] != '\t' && n + 1 < sizeof(tok)) {
        tok[n] = state[n];
        n++;
    }
    tok[n] = '\0';
    for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
        if (strcasecmp(tok, map[i].ha) == 0) {
            return map[i].zh;
        }
    }
    return state;
}

static void refresh_values(void)
{
    for (int p = 0; p < s_json_pages; p++) {
        for (int i = 0; i < s_page_tiles[p]; i++) {
            tile_widget_t *w = &s_tiles[p][i];
            const ha_entity_t *e = ha_client_find(w->entity);
            if (w->value == NULL) {
                continue;
            }
            if (e == NULL) {
                lv_label_set_text(w->value, "--");
                continue;
            }
            if (w->type == TILE_SENSOR) {
                const char *state_txt = weather_state_zh(e->entity_id, e->state);
                if (!e->fresh || strcmp(e->state, "unavailable") == 0) {
                    lv_label_set_text(w->value, "--");
                } else if (e->unit[0]) {
                    char buf[64];
                    snprintf(buf, sizeof(buf), "%s %s", state_txt, e->unit);
                    lv_label_set_text(w->value, buf);
                    lv_obj_set_style_text_font(w->value, value_font_for(buf), 0);
                } else {
                    lv_label_set_text(w->value, state_txt);
                    lv_obj_set_style_text_font(w->value, value_font_for(state_txt), 0);
                }
                continue;
            }

            const char *txt;
            if (!e->fresh || strcmp(e->state, "unavailable") == 0 || strcmp(e->state, "unknown") == 0) {
                txt = "--";
            } else if (strcmp(e->state, "on") == 0) {
                txt = "ON";
            } else if (strcmp(e->state, "off") == 0) {
                txt = "OFF";
            } else {
                txt = e->state;
            }
            lv_label_set_text(w->value, txt);
            lv_obj_set_style_text_font(w->value, value_font_for(txt), 0);
            lv_obj_set_style_bg_color(w->card, e->on ? COLOR_CARD_ON : COLOR_CARD, 0);
            lv_obj_set_style_text_color(w->value, e->on ? COLOR_TEXT : COLOR_MUTED, 0);
            if (w->dot) {
                lv_obj_set_style_bg_color(w->dot, e->fresh ? COLOR_OK : COLOR_BAD, 0);
            }
        }
    }
}

/* 折线图结果：worker 任务只负责取数，画图统一回到 LVGL 线程。
   新工程 LV_USE_OS=NONE（没有 LVGL 互斥锁），所以 worker 只填结构体 + 置标志，
   由下面 500ms 的 refresh_timer_cb（跑在 LVGL 线程）取走并调用 LVGL。
   和 net.c 里 cfg/cmd 的"写缓冲 + 置 pending 标志"是同一套做法。 */
typedef struct {
    char entity[96];
    float vals[32];
    int n;
    float vmin, vmax, vavg;
    int gen;
} chart_res_t;

static chart_res_t s_chart_res;          /* worker 写 -> LVGL 线程读 */
static char s_chart_fail_entity[96];
static int s_chart_fail_gen;
static volatile int s_chart_result;      /* 0=没有, 1=成功数据, 2=取数失败 */

static void chart_show_result(void);
static void chart_fail_result(void);

static void refresh_timer_cb(lv_timer_t *t)
{
    if (s_chart_result == 1) {
        s_chart_result = 0;
        chart_show_result();
    } else if (s_chart_result == 2) {
        s_chart_result = 0;
        chart_fail_result();
    }

#if UI_ASSISTANT_PAGE
    refresh_selftest_label();      /* 小智页顶部「音频自检」按钮的文字/结果 */
#endif
    /* 旋钮最后值不再挂在这个 500ms 定时器上：由 s_knob_timer（50ms）负责补发 */
    if (s_detail_visible) {
        detail_apply_entity();     /* 子页数值跟着轮询刷新 */
    }
    (void)t;

    {
        if (s_wifi_icon != NULL) {
        const int rssi = net_wifi_rssi();
        if (!net_wifi_connected()) {
            s_wifi_color = COLOR_MUTED;                /* 未连接：灰（状态栏弱化） */
        } else if (rssi >= -60) {
            s_wifi_color = COLOR_OK;                  /* 强：绿 */
        } else if (rssi >= -78) {
            s_wifi_color = lv_color_hex(0xE5B95B);    /* 中：黄 */
        } else {
            s_wifi_color = lv_color_hex(0xFF9F0A);     /* 弱：橙 */
        }
        /* 信号格数：-55 以上 4 格，往下每档减一格 */
        if (!net_wifi_connected()) {
            s_wifi_level = 0;            /* 断开：全暗 */
        } else if (rssi >= -60) {
            s_wifi_level = 3;            /* 三道光弧全亮 */
        } else if (rssi >= -73) {
            s_wifi_level = 2;
        } else {
            s_wifi_level = 1;
        }
        lv_obj_invalidate(s_wifi_icon);
    }
    }

    /* Home Assistant 图标：在线 = HA 蓝，离线 = 灰（状态栏统一用图标，不用文字） */
    if (s_ha_icon != NULL) {
        lv_obj_set_style_text_color(s_ha_icon,
                                    ha_client_online() ? lv_color_hex(0x03A9F4) : COLOR_MUTED, 0);
    }

    refresh_values();
    refresh_status_page();
    refresh_bt_ui();               /* 蓝牙图标颜色 + 设置页状态（本版恒「未启用」） */

    if (s_hint) {
        if (s_layout_valid) {
            lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_label_set_text(s_hint, net_wifi_connected() ? "loading layout..." : "connecting Wi-Fi...");
            lv_obj_remove_flag(s_hint, LV_OBJ_FLAG_HIDDEN);
        }
    }

    /* 小智助手界面：开着才刷（表情动画在 assistant_ui 自己的 50ms 定时器里）*/
    assistant_ui_tick();
}

/* ------------------------------------------------------------------------- */
/* 构建                                                                       */
/* ------------------------------------------------------------------------- */

/* 折线图结果的两个消费者：只在 LVGL 线程（refresh_timer_cb）里被调用。 */
static void chart_show_result(void)
{
    if (s_chart_res.gen != s_chart_gen || !s_chart_wanted) {
        ESP_LOGI(TAG, "折线图结果已过期，丢弃 (%d != %d)", s_chart_res.gen, s_chart_gen);
        return;
    }
    ui_home_show_chart(s_chart_res.entity, s_chart_res.vals, s_chart_res.n,
                       s_chart_res.vmin, s_chart_res.vmax, s_chart_res.vavg);
}

static void chart_fail_result(void)
{
    if (s_chart_fail_gen != s_chart_gen) {
        return;
    }
    const ha_entity_t *ent = ha_client_find(s_chart_fail_entity);
    lv_label_set_text(s_chart_title, ent ? ent->name : s_chart_fail_entity);
    lv_label_set_text(s_chart_span, "取不到历史数据，稍后再试");
    lv_label_set_text(s_chart_min, "");
    lv_label_set_text(s_chart_avg, "");
    lv_label_set_text(s_chart_max, "");
    s_chart_pts_n = 0;
    lv_obj_invalidate(s_chart);
    /* 页面保持打开（返回手势照常），但允许下次重新请求 */
    s_chart_wanted = false;
    s_chart_visible = true;
    lv_obj_set_pos(s_chart_page, 0, 0);
    lv_obj_set_style_translate_x(s_chart_page, 0, 0);
    lv_obj_remove_flag(s_chart_page, LV_OBJ_FLAG_HIDDEN);
}

static void worker_task(void *arg)
{
    (void)arg;
    while (true) {
        cmd_t cmd;
        if (xQueueReceive(s_cmd_q, &cmd, portMAX_DELAY) == pdTRUE) {
            if (strcmp(cmd.service, "chart") == 0) {
                static float vals[32];
                int n = 0;
                float vmin = 0, vmax = 0, vavg = 0;
                esp_err_t he = ha_client_history(cmd.entity, 6, vals, 32, &n, &vmin, &vmax, &vavg);
                bg_image_set_paused(false);
                if (he == ESP_OK && n > 0) {
                    const int keep = (n > 32) ? 32 : n;
                    memset(&s_chart_res, 0, sizeof(s_chart_res));
                    snprintf(s_chart_res.entity, sizeof(s_chart_res.entity), "%s", cmd.entity);
                    memcpy(s_chart_res.vals, vals, sizeof(float) * (size_t)keep);
                    s_chart_res.n = keep;
                    s_chart_res.vmin = vmin;
                    s_chart_res.vmax = vmax;
                    s_chart_res.vavg = vavg;
                    s_chart_res.gen = cmd.gen;
                    s_chart_result = 1;      /* 交给 refresh_timer_cb 画 */
                } else {
                    ESP_LOGW(TAG, "折线图无数据: %s (%s)", cmd.entity, esp_err_to_name(he));
                    snprintf(s_chart_fail_entity, sizeof(s_chart_fail_entity), "%s", cmd.entity);
                    s_chart_fail_gen = cmd.gen;
                    s_chart_result = 2;
                }
                continue;
            }

            ESP_LOGI(TAG, "%s %s %s", cmd.service, cmd.entity, cmd.attrs);
            if (cmd.knob) {
                /* 旋钮命令：POST 一回来就放开 in-flight，让 UI 能马上补最新值。
                   连续转动的中间值不再各自等 300ms + 刷新实体，只有最后一条
                   （队列空了）才等一下让 HA 落定，再刷新状态。 */
                if (cmd.attrs[0] != '\0') {
                    ha_client_action_attrs(cmd.entity, cmd.service, cmd.attrs);
                } else {
                    ha_client_action(cmd.entity, cmd.service);
                }
                s_knob_inflight = false;
                for (int i = 0; i < 30; i++) {           /* 原 300ms，可被新命令打断 */
                    if (uxQueueMessagesWaiting(s_cmd_q) > 0) {
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                if (uxQueueMessagesWaiting(s_cmd_q) == 0) {
                    const ha_entity_t *e = ha_client_find(cmd.entity);
                    if (e) {
                        ha_client_refresh_at((int)(e - ha_client_at(0)));
                    }
                }
                continue;
            }
            if (cmd.attrs[0] != '\0') {
                ha_client_action_attrs(cmd.entity, cmd.service, cmd.attrs);
            } else {
                ha_client_action(cmd.entity, cmd.service);
            }
            vTaskDelay(pdMS_TO_TICKS(300));
            const ha_entity_t *e = ha_client_find(cmd.entity);
            if (e) {
                ha_client_refresh_at((int)(e - ha_client_at(0)));
            }
        }
    }
}

/* ---------------- 子页面（灯的详情） ---------------- */

static lv_obj_t *s_power_ring;
static lv_obj_t *s_power_bar;

/** 用两个基本图形画电源符号：缺口圆环 + 顶部竖线。颜色随开关状态。
 *  详情页现在用 lv_switch，这两个函数暂时没被调用：保留实现备用，
 *  加 unused 属性免得在 -Werror 配置下报警告。 */
__attribute__((unused))
static void power_icon_draw(lv_obj_t *parent)
{
    s_power_ring = lv_obj_create(parent);
    lv_obj_remove_style_all(s_power_ring);
    lv_obj_set_size(s_power_ring, 22, 22);
    lv_obj_set_style_radius(s_power_ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_power_ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_power_ring, 3, 0);
    lv_obj_set_style_border_color(s_power_ring, lv_color_white(), 0);
    lv_obj_center(s_power_ring);
    lv_obj_set_y(s_power_ring, 4);
    lv_obj_remove_flag(s_power_ring, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    s_power_bar = lv_obj_create(parent);
    lv_obj_remove_style_all(s_power_bar);
    lv_obj_set_size(s_power_bar, 3, 10);
    lv_obj_set_style_bg_opa(s_power_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_power_bar, lv_color_white(), 0);
    lv_obj_align(s_power_bar, LV_ALIGN_CENTER, 0, -7);
    lv_obj_remove_flag(s_power_bar, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
}

/** 开关状态 -> 图标颜色（开：白底上黑图标；关：黑底上白图标）。 */
__attribute__((unused))
static void power_icon_color(bool on)
{
    const lv_color_t c = on ? lv_color_black() : lv_color_white();
    if (s_power_ring) {
        lv_obj_set_style_border_color(s_power_ring, c, 0);
    }
    if (s_power_bar) {
        lv_obj_set_style_bg_color(s_power_bar, c, 0);
    }
}

/** 开尔文转近似色（用于页面背景直观体现色温）。 */
static lv_color_t kelvin_to_color(int kelvin)
{
    /* 2700K 偏橙、4000K 暖白、6500K 偏冷白，简单线性插值 */
    if (kelvin < 3000) {
        return lv_color_make(255, 170, 90);
    } else if (kelvin < 4500) {
        const int t = (kelvin - 3000) * 100 / 1500;    /* 0..100 */
        return lv_color_make(255, 170 + (245 - 170) * t / 100, 90 + (215 - 90) * t / 100);
    } else {
        const int t = (kelvin - 4500) * 100 / 2000;    /* 0..100 */
        return lv_color_make(255 - (255 - 210) * t / 100, 245, 215 + (255 - 215) * t / 100);
    }
}

/** 根据背景亮度切换文字颜色，保证可读。 */
static void detail_apply_foreground(bool dark_text)
{
    const lv_color_t fg = dark_text ? lv_color_black() : lv_color_white();
    const lv_color_t muted = dark_text ? lv_color_hex(0x3A3A3C) : COLOR_MUTED;
    if (s_detail_title)    lv_obj_set_style_text_color(s_detail_title, fg, 0);
    if (s_back_label)      lv_obj_set_style_text_color(s_back_label, fg, 0);
    if (s_br_lab)          lv_obj_set_style_text_color(s_br_lab, muted, 0);
    if (s_ct_lab)          lv_obj_set_style_text_color(s_ct_lab, muted, 0);
    if (s_detail_br_value) lv_obj_set_style_text_color(s_detail_br_value, fg, 0);
    if (s_detail_ct_value) lv_obj_set_style_text_color(s_detail_ct_value, fg, 0);
    /* 返回键底色跟着反色，避免在纯白背景上看不见 */
    if (s_back_label && lv_obj_get_parent(s_back_label)) {
        lv_obj_set_style_bg_color(lv_obj_get_parent(s_back_label),
                                  dark_text ? lv_color_hex(0xE5E5EA) : lv_color_hex(0x2C2C2E), 0);
        lv_obj_set_style_bg_opa(lv_obj_get_parent(s_back_label), LV_OPA_70, 0);
    }
}

/** 背景氛围：按"关=黑、开=灯色 × 亮度"更新子页底色。 */
static void detail_update_ambience(void)
{
    if (s_detail == NULL) {
        return;
    }
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (e == NULL || !e->on) {
        lv_obj_set_style_bg_color(s_detail, COLOR_BG, 0);      /* 关灯：纯黑 */
        lv_obj_set_style_bg_opa(s_detail, LV_OPA_COVER, 0);
        detail_apply_foreground(false);
        return;
    }
    int k = (s_ct_override_until > lv_tick_get()) ? s_ct_override
                                                  : (e->color_temp_k ? e->color_temp_k : 4000);
    int pct = (s_br_override_until > lv_tick_get()) ? s_br_override
                                                    : (e->brightness * 100 / 255);
    if (pct <= 0) {
        pct = 100;
    }
    const lv_color_t c = kelvin_to_color(k);

    /* 整个页面的底色就是这只灯：色温决定颜色，亮度决定明暗。
       视觉下限抬到 50%：滑条最小时页面大致相当于现在 50% 的亮度，最高才是纯白。 */
    int pct_eff = 50 + (pct / 2);
    if (pct_eff > 100) {
        pct_eff = 100;
    }
    const lv_color_t bg = lv_color_make(
        (uint8_t)(c.red * pct_eff / 100),
        (uint8_t)(c.green * pct_eff / 100),
        (uint8_t)(c.blue * pct_eff / 100));
    lv_obj_set_style_bg_color(s_detail, bg, 0);
    lv_obj_set_style_bg_opa(s_detail, LV_OPA_COVER, 0);

    /* 背景亮就用黑字，背景暗就用白字 */
    const int lum = (299 * bg.red + 587 * bg.green + 114 * bg.blue) / 1000;
    detail_apply_foreground(lum < 150);      /* true = 用深色文字 */
}

static void detail_anim_done_cb(lv_anim_t *a);
static void detail_close_done_cb(lv_anim_t *a);

static void detail_apply_entity(void)
{
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (e == NULL) {
        return;
    }
    ESP_LOGD(TAG, "light attrs: br=%d tint=%d k=%d range=%d-%d state=%s",
             e->has_brightness, e->has_tint, e->color_temp_k, e->min_k, e->max_k, e->state);
    lv_label_set_text(s_detail_title, e->name);

    /* 不再显示"已开/已关"文字：开=白色底、关=黑色底，一眼可辨 */
    bool on = e->on;
    if (s_sw_override_until > lv_tick_get()) {
        on = s_sw_override_on;
        if (e->on == on) {
            s_sw_override_until = 0;          /* HA 已跟上，交还控制权 */
        }
    } else {
        s_sw_override_until = 0;
    }
    if (s_detail_power_sw != NULL) {
        if (on) {
            lv_obj_add_state(s_detail_power_sw, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(s_detail_power_sw, LV_STATE_CHECKED);
        }
        lv_obj_set_style_bg_color(s_detail_power_sw,
                                  on ? lv_color_white() : lv_color_hex(0x151517), LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_detail_power_sw,
                                  on ? lv_color_black() : lv_color_white(), LV_PART_KNOB);
    }
    detail_update_ambience();

    if (s_detail_br_row) {
        if (e->has_brightness) {
            lv_obj_remove_flag(s_detail_br_row, LV_OBJ_FLAG_HIDDEN);
            int pct = e->brightness * 100 / 255;
            if (s_br_override_until > lv_tick_get()) {
                pct = s_br_override;          /* 用户刚调过：先按本地值显示 */
            } else {
                s_br_override_until = 0;
            }
            lv_slider_set_value(s_detail_br_slider, pct, LV_ANIM_OFF);
            char buf[16];
            snprintf(buf, sizeof(buf), "%d%%", pct);
            lv_label_set_text(s_detail_br_value, buf);
        } else {
            lv_obj_add_flag(s_detail_br_row, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_detail_ct_row) {
        if (e->has_tint) {
            lv_obj_remove_flag(s_detail_ct_row, LV_OBJ_FLAG_HIDDEN);
            lv_slider_set_range(s_detail_ct_slider, e->min_k, e->max_k);
            int k = e->color_temp_k ? e->color_temp_k : (e->min_k + e->max_k) / 2;
            if (s_ct_override_until > lv_tick_get()) {
                k = s_ct_override;
            } else {
                s_ct_override_until = 0;
            }
            lv_slider_set_value(s_detail_ct_slider, k, LV_ANIM_OFF);
            char buf[24];
            snprintf(buf, sizeof(buf), "%dK", k);
            lv_label_set_text(s_detail_ct_value, buf);
        } else {
            lv_obj_add_flag(s_detail_ct_row, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* 灯详情页：旋钮调亮度 / 色温                                                */
/* ------------------------------------------------------------------------- */

/** 这只灯现在算不算"开着"（和 detail_apply_entity 的判断一致，含本地开关覆盖）*/
static bool detail_light_on(const ha_entity_t *e)
{
    if (e == NULL) {
        return false;
    }
    if (s_sw_override_until > lv_tick_get()) {
        return s_sw_override_on;
    }
    return e->on;
}

/** 旋钮这次该调哪个：优先用用户点过的那个控件，不支持就退回另一个。 */
static int detail_knob_target_for(const ha_entity_t *e)
{
    if (e == NULL) {
        return -1;
    }
    if (s_detail_knob_target == DETAIL_KNOB_CT && e->has_tint) {
        return DETAIL_KNOB_CT;
    }
    if (e->has_brightness) {
        return DETAIL_KNOB_BR;
    }
    if (e->has_tint) {
        return DETAIL_KNOB_CT;
    }
    return -1;
}

/** 把"当前旋钮值"排进命令队列（不阻塞：真正的 HA 调用在 worker 任务里）。 */
static bool detail_knob_send(int target, int value)
{
    if (s_cmd_q == NULL || s_detail_entity[0] == '\0') {
        return false;
    }
    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.knob = true;                 /* worker 据此走快路径：不等 300ms 刷新 */
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_detail_entity);
    snprintf(cmd.service, sizeof(cmd.service), "turn_on");
    if (target == DETAIL_KNOB_CT) {
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"color_temp_kelvin\":%d}", value);
    } else {
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"brightness_pct\":%d}", value);
    }
    return xQueueSend(s_cmd_q, &cmd, 0) == pdTRUE;
}

/** 有脏值时唤醒 50ms 补发定时器（已经在跑就什么也不做）。*/
static void detail_knob_timer_wake(void)
{
    if (s_knob_timer == NULL || s_knob_timer_active) {
        return;
    }
    lv_timer_resume(s_knob_timer);
    lv_timer_ready(s_knob_timer);    /* 先查一次，别白等一个周期 */
    s_knob_timer_active = true;
}

/** 没有待发值了就停掉定时器：不跟着 500ms 刷新定时器空转。*/
static void detail_knob_timer_park(void)
{
    if (s_knob_timer == NULL || !s_knob_timer_active) {
        return;
    }
    lv_timer_pause(s_knob_timer);
    s_knob_timer_active = false;
}

/**
 * 把最新待发值排进命令队列。
 * in-flight 时不排：队列里最多只有一条旋钮命令，这期间再转多少格都只保留
 * s_knob_pending_* 里的最新值，中间值直接合并掉（不会积压一队过期亮度）。
 */
static bool detail_knob_queue_pending(void)
{
    if (!s_knob_dirty || !s_detail_visible || s_knob_inflight) {
        return false;
    }
    /* 先置 in-flight 再入队：worker 可能在 xQueueSend 里就抢占并跑完，
       若等发送返回后才置位，会把已经清掉的标志又置成 true，卡住后续补发。 */
    s_knob_inflight = true;
    if (!detail_knob_send(s_knob_pending_target, s_knob_pending_value)) {
        s_knob_inflight = false;
        return false;
    }
    s_knob_dirty = false;
    s_knob_last_send = lv_tick_get();
    return true;
}

/**
 * 旋钮在灯详情页里调值。
 * 返回 true 表示"这次旋转被详情页吃掉了，并且真的调了值"（调用方响一下触觉）；
 * 返回 false 表示没消费（详情页没开 / 灯是关的 / 这只灯不支持 / 手指正按着滑条），
 * 调用方据此决定要不要翻页 + 震动。
 */
bool ui_home_detail_knob(int delta)
{
    if (delta == 0 || !s_detail_visible || s_detail == NULL) {
        return false;
    }
    if (s_press_on_slider) {
        return false;                  /* 手指正在拖滑条：这一格让给手指 */
    }
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (e == NULL || !detail_light_on(e)) {
        return false;                  /* 灯关着：旋钮什么也不做（也不翻页）*/
    }
    const int target = detail_knob_target_for(e);
    if (target < 0) {
        return false;
    }

    lv_obj_t *slider = (target == DETAIL_KNOB_CT) ? s_detail_ct_slider : s_detail_br_slider;
    lv_obj_t *label = (target == DETAIL_KNOB_CT) ? s_detail_ct_value : s_detail_br_value;
    if (slider == NULL || label == NULL) {
        return false;
    }

    /* 以滑条当前值为基准（它已经反映了实体值或本地刚调过的值）*/
    int v = (int)lv_slider_get_value(slider);
    v += (target == DETAIL_KNOB_CT) ? delta * DETAIL_CT_STEP_K : delta;

    const int vmin = (int)lv_slider_get_min_value(slider);
    const int vmax = (int)lv_slider_get_max_value(slider);
    if (v < vmin) {
        v = vmin;
    }
    if (v > vmax) {
        v = vmax;
    }
    if (v == (int)lv_slider_get_value(slider)) {
        return true;                   /* 已经到头：吃掉这一格，但不用发命令 */
    }

    /* 1) 立刻更新界面（滑条 + 数字 + 背景色），不等网络 */
    lv_slider_set_value(slider, v, LV_ANIM_OFF);
    char buf[24];
    if (target == DETAIL_KNOB_CT) {
        snprintf(buf, sizeof(buf), "%dK", v);
        s_ct_override = v;
        s_ct_override_until = lv_tick_get() + OVERRIDE_MS;
    } else {
        snprintf(buf, sizeof(buf), "%d%%", v);
        s_br_override = v;
        s_br_override_until = lv_tick_get() + OVERRIDE_MS;
    }
    lv_label_set_text(label, buf);
    detail_update_ambience();

    /* 2) 网络命令：转动中最快 100ms 一条，多格只保留最新值（pending）。
       停手后由 50ms 专用定时器补发最后一条，不再等 500ms 刷新定时器。 */
    const uint32_t now = lv_tick_get();
    s_knob_dirty = true;
    s_knob_pending_target = target;
    s_knob_pending_value = v;
    s_knob_last_change = now;
    if ((uint32_t)(now - s_knob_last_send) >= DETAIL_KNOB_SEND_MS) {
        detail_knob_queue_pending();   /* in-flight / 队列满：留着下次补 */
    }
    if (s_knob_dirty) {
        detail_knob_timer_wake();
    }
    return true;
}

/**
 * 补发最后一条亮度/色温命令。只在 LVGL 线程调用，真正的网络请求仍在 worker 里。
 * 只在"停转 >= 80ms"后发（转动中的节奏由旋钮事件自己按 100ms 限流），
 * 这样每条最后值只会打一行日志，串口上能直接量延迟。
 */
static void detail_knob_flush(void)
{
    if (!s_knob_dirty || !s_detail_visible || s_knob_inflight) {
        return;
    }
    const uint32_t now = lv_tick_get();
    const uint32_t idle_ms = (uint32_t)(now - s_knob_last_change);
    if (idle_ms < DETAIL_KNOB_IDLE_MS) {
        return;
    }
    if ((uint32_t)(now - s_knob_last_send) < DETAIL_KNOB_SEND_MS) {
        return;                        /* 还差一点到 100ms 限流窗口：下个 tick 再发 */
    }

    /* 灯已经关了（或实体没了）：丢掉待发值，绝不能把它重新点亮 */
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (e == NULL || !detail_light_on(e)) {
        s_knob_dirty = false;
        return;
    }
    char name[48];
    snprintf(name, sizeof(name), "%.47s", e->name[0] ? e->name : s_detail_entity);   /* 显式精度，避免 -Wformat-truncation */

    const int target = s_knob_pending_target;
    const int value = s_knob_pending_value;
    if (!detail_knob_queue_pending()) {
        return;                        /* 队列满：留着下一次 tick 再试 */
    }
    char vbuf[16];
    if (target == DETAIL_KNOB_CT) {
        snprintf(vbuf, sizeof(vbuf), "%dK", value);
    } else {
        snprintf(vbuf, sizeof(vbuf), "%d%%", value);
    }
    /* 一行一条，延迟 = 距离最后一次转动的毫秒数，便于串口上量 */
    ESP_LOGI(TAG, "旋钮补发最后值 %s -> %s，延迟 %u ms", vbuf, name, (unsigned)idle_ms);
}

/** 50ms 专用定时器：有脏值才跑，补发完就自己停掉。*/
static void detail_knob_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (!s_knob_dirty || !s_detail_visible) {
        detail_knob_timer_park();
        return;
    }
    if (s_knob_inflight) {
        return;                        /* worker 正在发上一条；清标志后本定时器再补 */
    }
    detail_knob_flush();
    if (!s_knob_dirty) {
        detail_knob_timer_park();
    }
}

/** 点一下滑条就把旋钮切到那个控件上（拖动也会先经过 PRESSED）。 */
static void detail_slider_pressed_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    /* 手指按到滑条上：手指优先。丢掉还没补发的旋钮值，别让它在松手后
       又补一条旋钮命令，盖掉手指刚选的值。 */
    s_knob_dirty = false;
    detail_knob_timer_park();
    if (slider == s_detail_ct_slider) {
        s_detail_knob_target = DETAIL_KNOB_CT;
    } else if (slider == s_detail_br_slider) {
        s_detail_knob_target = DETAIL_KNOB_BR;
    }
}

static void detail_slider_changed_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    const int v = (int)lv_slider_get_value(slider);
    char buf[24];
    if (slider == s_detail_br_slider) {
        snprintf(buf, sizeof(buf), "%d%%", v);
        lv_label_set_text(s_detail_br_value, buf);
    } else {
        snprintf(buf, sizeof(buf), "%dK", v);
        lv_label_set_text(s_detail_ct_value, buf);
    }

    if (slider == s_detail_br_slider) {
        s_br_override = v;
        s_br_override_until = lv_tick_get() + OVERRIDE_MS;
    } else {
        s_ct_override = v;
        s_ct_override_until = lv_tick_get() + OVERRIDE_MS;
    }
    detail_update_ambience();   /* 拖动时底色实时跟着变 */

    if (lv_event_get_code(e) != LV_EVENT_RELEASED && lv_event_get_code(e) != LV_EVENT_PRESS_LOST) {
        return;   /* 拖动过程中只更新显示，松手才发命令 */
    }
    ESP_LOGI(TAG, "滑条松手: %s value=%d", (slider == s_detail_br_slider) ? "亮度" : "色温", v);
    if (s_detail_entity[0] == '\0' || s_cmd_q == NULL) {
        return;
    }
    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_detail_entity);
    snprintf(cmd.service, sizeof(cmd.service), "turn_on");
    if (slider == s_detail_br_slider) {
        s_br_override = v;
        s_br_override_until = lv_tick_get() + OVERRIDE_MS;
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"brightness_pct\":%d}", v);
        /* 调亮度会把灯点亮，但按钮交给轮询去更新，避免拖动时乱跳 */
    } else {
        s_ct_override = v;
        s_ct_override_until = lv_tick_get() + OVERRIDE_MS;
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"color_temp_kelvin\":%d}", v);
    }
    xQueueSend(s_cmd_q, &cmd, 0);
}

static void detail_power_changed_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    const bool want_on = lv_obj_has_state(sw, LV_STATE_CHECKED);

    /* 开关自身的视觉已经切换：记住它，别让轮询立刻拉回去 */
    s_sw_override_on = want_on;
    s_sw_override_until = lv_tick_get() + OVERRIDE_MS;

    if (!want_on) {
        /* 关灯：丢掉还没补发的旋钮值，别让它在关灯后又发一条 turn_on */
        s_knob_dirty = false;
        detail_knob_timer_park();
    }

    if (s_detail_entity[0] == '\0' || s_cmd_q == NULL) {
        return;
    }
    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_detail_entity);
    snprintf(cmd.service, sizeof(cmd.service), "%s", want_on ? "turn_on" : "turn_off");
    xQueueSend(s_cmd_q, &cmd, 0);
}

static void detail_back_cb(lv_event_t *e)
{
    (void)e;
    ui_home_close_detail();
}

static void build_detail_view(lv_obj_t *scr)
{
    s_detail = lv_obj_create(scr);
    lv_obj_remove_style_all(s_detail);
    lv_obj_set_size(s_detail, 360, 360);
    lv_obj_set_pos(s_detail, 0, 0);
    lv_obj_set_style_bg_color(s_detail, COLOR_BG, 0);
    lv_obj_set_style_bg_opa(s_detail, LV_OPA_COVER, 0);
    lv_obj_remove_flag(s_detail, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_detail, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *back = lv_button_create(s_detail);
    lv_obj_set_size(back, 36, 36);
    lv_obj_align(back, LV_ALIGN_TOP_MID, 0, 10);   /* 顶部中间：圆屏这里才看得见 */
    lv_obj_set_style_bg_opa(back, LV_OPA_20, 0);
    lv_obj_set_style_radius(back, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_event_cb(back, detail_back_cb, LV_EVENT_CLICKED, NULL);
    s_back_label = lv_label_create(back);
    lv_label_set_text(s_back_label, LV_SYMBOL_LEFT);
    lv_obj_center(s_back_label);

    s_detail_title = lv_label_create(s_detail);
    lv_label_set_text(s_detail_title, "--");
    lv_obj_set_style_text_font(s_detail_title, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_detail_title, COLOR_TEXT, 0);
    lv_obj_align(s_detail_title, LV_ALIGN_TOP_MID, 0, 58);

    /* 开关：lv_switch（开=白色轨道+黑滑块，关=深色轨道+白滑块） */
    s_detail_power_sw = lv_switch_create(s_detail);
    lv_obj_set_size(s_detail_power_sw, 96, 46);
    lv_obj_align(s_detail_power_sw, LV_ALIGN_TOP_MID, 0, 92);
    lv_obj_set_style_radius(s_detail_power_sw, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_detail_power_sw, lv_color_hex(0x151517), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_detail_power_sw, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_detail_power_sw, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_detail_power_sw, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_detail_power_sw, lv_color_white(), LV_PART_KNOB);
    lv_obj_add_event_cb(s_detail_power_sw, detail_power_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* 亮度 */
    s_detail_br_row = lv_obj_create(s_detail);
    lv_obj_remove_style_all(s_detail_br_row);
    lv_obj_set_size(s_detail_br_row, 300, 70);
    lv_obj_align(s_detail_br_row, LV_ALIGN_TOP_MID, 0, 152);
    s_br_lab = lv_label_create(s_detail_br_row);
    lv_label_set_text(s_br_lab, "亮度");
    lv_obj_set_style_text_font(s_br_lab, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_br_lab, COLOR_MUTED, 0);

    s_detail_br_value = lv_label_create(s_detail_br_row);
    lv_label_set_text(s_detail_br_value, "--");
    lv_obj_set_style_text_font(s_detail_br_value, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_detail_br_value, COLOR_TEXT, 0);
    lv_obj_align(s_detail_br_value, LV_ALIGN_TOP_RIGHT, 0, 0);

    s_detail_br_slider = lv_slider_create(s_detail_br_row);
    lv_obj_set_width(s_detail_br_slider, 300);
    lv_obj_set_height(s_detail_br_slider, 12);
    lv_slider_set_range(s_detail_br_slider, 1, 100);
    lv_obj_align(s_detail_br_slider, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_add_event_cb(s_detail_br_slider, detail_slider_pressed_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_detail_br_slider, detail_slider_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_detail_br_slider, detail_slider_changed_cb, LV_EVENT_RELEASED, NULL);

    /* 色温 */
    s_detail_ct_row = lv_obj_create(s_detail);
    lv_obj_remove_style_all(s_detail_ct_row);
    lv_obj_set_size(s_detail_ct_row, 208, 62);
    lv_obj_align(s_detail_ct_row, LV_ALIGN_TOP_MID, 0, 226);
    s_ct_lab = lv_label_create(s_detail_ct_row);
    lv_label_set_text(s_ct_lab, "色温");
    lv_obj_set_style_text_font(s_ct_lab, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_ct_lab, COLOR_MUTED, 0);

    s_detail_ct_value = lv_label_create(s_detail_ct_row);
    lv_label_set_text(s_detail_ct_value, "--");
    lv_obj_set_style_text_font(s_detail_ct_value, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_detail_ct_value, COLOR_TEXT, 0);
    lv_obj_align(s_detail_ct_value, LV_ALIGN_TOP_RIGHT, 0, 0);

    s_detail_ct_slider = lv_slider_create(s_detail_ct_row);
    lv_obj_set_width(s_detail_ct_slider, 200);
    lv_obj_set_height(s_detail_ct_slider, 12);
    lv_slider_set_range(s_detail_ct_slider, 2700, 6500);
    lv_obj_align(s_detail_ct_slider, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_add_event_cb(s_detail_ct_slider, detail_slider_pressed_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_detail_ct_slider, detail_slider_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_detail_ct_slider, detail_slider_changed_cb, LV_EVENT_RELEASED, NULL);
}

void ui_home_open_detail(const char *entity_id)
{
    if (s_detail == NULL || entity_id == NULL) {
        ESP_LOGW(TAG, "open detail 失败: detail=%p entity=%s", (void *)s_detail, entity_id ? entity_id : "(null)");
        return;
    }
    ESP_LOGI(TAG, "open detail: %s", entity_id);
    if (strcmp(entity_id, s_detail_entity) != 0) {
        /* 换了一只灯：旋钮回到"调亮度"（这是用户最常用的）*/
        s_detail_knob_target = DETAIL_KNOB_BR;
    }
    /* 新页面从干净状态开始：上一页的待发值（关页时已补发过）不再带进来 */
    s_knob_dirty = false;
    s_knob_pending_target = DETAIL_KNOB_BR;
    detail_knob_timer_park();
    snprintf(s_detail_entity, sizeof(s_detail_entity), "%s", entity_id);
    detail_apply_entity();
    s_detail_visible = true;

    /* 藏掉底下的页面内容，只留背景图，避免卡片透过半透明子页露出来 */
    if (s_page >= 0 && s_page < s_page_count && s_pages[s_page]) {
        lv_obj_add_flag(s_pages[s_page], LV_OBJ_FLAG_HIDDEN);
    }
    if (s_title) {
        lv_obj_add_flag(s_title, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < s_page_count; i++) {
        if (s_dots[i]) {
            lv_obj_add_flag(s_dots[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    lv_obj_remove_flag(s_detail, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_detail, 0, 0);          /* 位置固定在 0 */
    lv_obj_set_style_translate_x(s_detail, 360, 0);   /* 先移到屏幕右侧外 */

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_detail);
    lv_anim_set_values(&a, 360, 0);
    lv_anim_set_time(&a, 220);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, anim_translate_cb);       /* ← 之前这行丢了 */
    lv_anim_set_completed_cb(&a, detail_anim_done_cb);
    lv_anim_start(&a);
    s_animating = true;
}

static void detail_anim_done_cb(lv_anim_t *a)
{
    (void)a;
    s_animating = false;
    if (s_detail != NULL) {
        lv_area_t area;
        lv_obj_get_coords(s_detail, &area);
        ESP_LOGI(TAG, "detail_geom x=%d tx=%d hidden=%d idx=%d visible=%d",
                 (int)lv_obj_get_x(s_detail),
                 (int)lv_obj_get_style_translate_x(s_detail, 0),
                 (int)lv_obj_is_hidden(s_detail),
                 (int)lv_obj_get_index(s_detail),
                 (int)lv_obj_is_visible(s_detail));
        (void)area;
    }
}

static void detail_close_done_cb(lv_anim_t *a)
{
    (void)a;
    if (s_detail != NULL) {
        lv_obj_add_flag(s_detail, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_x(s_detail, 0, 0);
        lv_obj_set_pos(s_detail, 0, 0);
    }
    /* 恢复底下的页面 */
    if (s_page >= 0 && s_page < s_page_count && s_pages[s_page]) {
        lv_obj_remove_flag(s_pages[s_page], LV_OBJ_FLAG_HIDDEN);
    }
    if (s_title) {
        lv_obj_remove_flag(s_title, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < s_page_count; i++) {
        if (s_dots[i]) {
            lv_obj_remove_flag(s_dots[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(s_dots[i], dot_color(s_page, i), 0);
        }
    }
    s_animating = false;
}

void ui_home_close_detail(void)
{
    if (s_detail == NULL || !s_detail_visible) {
        return;
    }
    ESP_LOGI(TAG, "close detail");
    /* 页面要关了：把还没补发的最后值发掉（灯已经关了就不发，不能把它重新点亮）*/
    if (s_knob_dirty) {
        const ha_entity_t *e = ha_client_find(s_detail_entity);
        if (e != NULL && detail_light_on(e) && s_cmd_q != NULL) {
            const uint32_t delay_ms = (uint32_t)(lv_tick_get() - s_knob_last_change);
            s_knob_inflight = true;      /* 先置位再入队，理由同 queue_pending */
            if (!detail_knob_send(s_knob_pending_target, s_knob_pending_value)) {
                s_knob_inflight = false;
            } else {
                s_knob_last_send = lv_tick_get();
                char vbuf[16];
                if (s_knob_pending_target == DETAIL_KNOB_CT) {
                    snprintf(vbuf, sizeof(vbuf), "%dK", s_knob_pending_value);
                } else {
                    snprintf(vbuf, sizeof(vbuf), "%d%%", s_knob_pending_value);
                }
                ESP_LOGI(TAG, "旋钮补发最后值 %s -> %s，关页前发出，延迟 %u ms", vbuf,
                         e->name[0] ? e->name : s_detail_entity, (unsigned)delay_ms);
            }
        }
        s_knob_dirty = false;
    }
    detail_knob_timer_park();
    s_detail_visible = false;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_detail);
    lv_anim_set_values(&a, lv_obj_get_style_translate_x(s_detail, 0), 360);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&a, anim_translate_cb);
    lv_anim_set_completed_cb(&a, detail_close_done_cb);
    lv_anim_start(&a);
}

bool ui_home_detail_open(void)
{
    return s_detail_visible;
}

/* ---------------- 传感器折线图 ---------------- */

static void chart_back_cb(lv_event_t *e)
{
    (void)e;
    ui_home_close_chart();
}

/** 自己画折线：没有内部状态、不分配内存，内存紧张时也稳。 */
static void chart_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    if (s_chart_pts_n < 2 || layer == NULL) {
        return;
    }
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    const int32_t w = lv_area_get_width(&a);
    const int32_t h = lv_area_get_height(&a);
    if (w < 2 || h < 2) {
        return;
    }

    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.color = lv_color_hex(0x5AC8FA);
    d.width = 3;
    d.round_start = true;
    d.round_end = true;

    const float span = (s_chart_hi - s_chart_lo) > 0.0001f ? (s_chart_hi - s_chart_lo) : 1.0f;
    int32_t prev_x = 0, prev_y = 0;
    for (int i = 0; i < s_chart_pts_n; i++) {
        float t = (s_chart_pts[i] - s_chart_lo) / span;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        const int32_t x = a.x1 + (int32_t)((int64_t)(w - 1) * i / (s_chart_pts_n - 1));
        const int32_t y = a.y2 - (int32_t)(t * (h - 1));
        if (i > 0) {
            d.p1.x = prev_x;
            d.p1.y = prev_y;
            d.p2.x = x;
            d.p2.y = y;
            lv_draw_line(layer, &d);
        }
        prev_x = x;
        prev_y = y;
    }
}

static void build_chart_view(lv_obj_t *scr)
{
    s_chart_page = lv_obj_create(scr);
    lv_obj_remove_style_all(s_chart_page);
    lv_obj_set_size(s_chart_page, 360, 360);
    lv_obj_set_pos(s_chart_page, 0, 0);
    lv_obj_set_style_bg_color(s_chart_page, lv_color_hex(0x0B0B0D), 0);
    lv_obj_set_style_bg_opa(s_chart_page, LV_OPA_COVER, 0);
    lv_obj_remove_flag(s_chart_page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_chart_page, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *back = lv_button_create(s_chart_page);
    lv_obj_set_size(back, 36, 36);
    lv_obj_align(back, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_opa(back, LV_OPA_20, 0);
    lv_obj_set_style_radius(back, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_event_cb(back, chart_back_cb, LV_EVENT_CLICKED, NULL);
    s_chart_back_label = lv_label_create(back);
    lv_label_set_text(s_chart_back_label, LV_SYMBOL_LEFT);
    lv_obj_center(s_chart_back_label);

    s_chart_title = lv_label_create(s_chart_page);
    lv_label_set_text(s_chart_title, "--");
    lv_obj_set_style_text_font(s_chart_title, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_chart_title, COLOR_TEXT, 0);
    lv_obj_align(s_chart_title, LV_ALIGN_TOP_MID, 0, 58);

    s_chart_span = lv_label_create(s_chart_page);
    lv_label_set_text(s_chart_span, "近 3 小时");
    lv_obj_set_style_text_font(s_chart_span, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_chart_span, COLOR_MUTED, 0);
    lv_obj_align(s_chart_span, LV_ALIGN_TOP_MID, 0, 76);

    s_chart = lv_obj_create(s_chart_page);
    lv_obj_remove_style_all(s_chart);
    lv_obj_set_size(s_chart, 288, 150);
    lv_obj_align(s_chart, LV_ALIGN_TOP_MID, 0, 100);
    lv_obj_remove_flag(s_chart, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_chart, chart_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_chart_min = lv_label_create(s_chart_page);
    lv_label_set_text(s_chart_min, "");
    lv_obj_set_style_text_font(s_chart_min, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_chart_min, COLOR_MUTED, 0);
    lv_obj_align(s_chart_min, LV_ALIGN_TOP_MID, -100, 268);

    s_chart_avg = lv_label_create(s_chart_page);
    lv_label_set_text(s_chart_avg, "");
    lv_obj_set_style_text_font(s_chart_avg, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_chart_avg, COLOR_TEXT, 0);
    lv_obj_align(s_chart_avg, LV_ALIGN_TOP_MID, 0, 268);

    s_chart_max = lv_label_create(s_chart_page);
    lv_label_set_text(s_chart_max, "");
    lv_obj_set_style_text_font(s_chart_max, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_chart_max, COLOR_MUTED, 0);
    lv_obj_align(s_chart_max, LV_ALIGN_TOP_MID, 100, 268);
}

void ui_home_show_chart(const char *entity_id, const float *vals, int n,
                        float vmin, float vmax, float vavg)
{
    if (s_chart_page == NULL || vals == NULL || n <= 0) {
        return;
    }
    if (!isfinite(vmin) || !isfinite(vmax) || !isfinite(vavg) ||
        fabsf(vmin) > 1.0e5f || fabsf(vmax) > 1.0e5f) {
        ESP_LOGW(TAG, "图表数值异常，忽略 (%f %f %f)", vmin, vmax, vavg);
        return;
    }
    snprintf(s_chart_entity, sizeof(s_chart_entity), "%s", entity_id ? entity_id : "");
    const ha_entity_t *e = ha_client_find(s_chart_entity);
    lv_label_set_text(s_chart_title, e ? e->name : s_chart_entity);

    /* 数值按 10 倍整数化（lv_chart 只收整数），纵向留 10% 余量避免贴边 */
    float lo = vmin, hi = vmax;
    if (hi - lo < 0.01f) {
        lo -= 1.0f;
        hi += 1.0f;
    }
    const float pad = (hi - lo) * 0.1f;
    s_chart_lo = lo - pad;
    s_chart_hi = hi + pad;

    const int keep = (n > 32) ? 32 : n;
    memcpy(s_chart_pts, vals, sizeof(float) * (size_t)keep);
    s_chart_pts_n = keep;
    lv_obj_invalidate(s_chart);

    /* 记下这次结果，60 秒内复用 */
    snprintf(s_chart_cache_entity, sizeof(s_chart_cache_entity), "%s", s_chart_entity);
    memcpy(s_chart_cache_vals, vals, sizeof(float) * (size_t)keep);
    s_chart_cache_n = keep;
    s_chart_cache_min = vmin;
    s_chart_cache_max = vmax;
    s_chart_cache_avg = vavg;
    s_chart_cache_at = lv_tick_get();

    lv_label_set_text(s_chart_span, "近 3 小时");
    const char *unit = (e != NULL) ? e->unit : "";
    char buf[40];
    snprintf(buf, sizeof(buf), "最低 %.1f%s", vmin, unit);
    lv_label_set_text(s_chart_min, buf);
    snprintf(buf, sizeof(buf), "平均 %.1f%s", vavg, unit);
    lv_label_set_text(s_chart_avg, buf);
    snprintf(buf, sizeof(buf), "最高 %.1f%s", vmax, unit);
    lv_label_set_text(s_chart_max, buf);

    const bool already_open = s_chart_visible;
    s_chart_visible = true;
    lv_obj_remove_flag(s_chart_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_chart_page, 0, 0);
    if (!already_open) {
        /* 只有第一次出现才滑入；数据晚到时不要再演一遍动画 */
        lv_obj_set_style_translate_x(s_chart_page, 360, 0);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, s_chart_page);
        lv_anim_set_values(&a, 360, 0);
        lv_anim_set_time(&a, 220);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_set_exec_cb(&a, anim_translate_cb);
        lv_anim_start(&a);
    } else {
        lv_obj_set_style_translate_x(s_chart_page, 0, 0);
    }
    ESP_LOGI(TAG, "显示折线图: %s (%d 点)%s", s_chart_entity, n, already_open ? " (更新)" : "");
}

void ui_home_close_chart(void)
{
    if (s_chart_page == NULL || !s_chart_visible) {
        return;
    }
    s_chart_visible = false;
    s_chart_wanted = false;
    s_chart_gen++;          /* 在途的请求全部作废，返回后不会再自己弹出来 */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_chart_page);
    lv_anim_set_values(&a, 0, 360);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&a, anim_translate_cb);
    lv_anim_start(&a);
    lv_obj_add_flag(s_chart_page, LV_OBJ_FLAG_HIDDEN);
}

bool ui_home_chart_open(void)
{
    return s_chart_visible;
}

/* ---------------- 空调页 ---------------- */

static const char *hvac_label(const char *mode)
{
    if (strcmp(mode, "off") == 0) return "关";
    if (strcmp(mode, "auto") == 0) return "自动";
    if (strcmp(mode, "cool") == 0) return "制冷";
    if (strcmp(mode, "heat") == 0) return "制热";
    if (strcmp(mode, "dry") == 0) return "除湿";
    if (strcmp(mode, "fan_only") == 0) return "送风";
    return mode;
}

static const char *fan_label(const char *f)
{
    if (strcmp(f, "auto") == 0) return "自动";
    if (strcmp(f, "low") == 0) return "低";
    if (strcmp(f, "medium low") == 0) return "中低";
    if (strcmp(f, "medium") == 0) return "中";
    if (strcmp(f, "medium high") == 0) return "中高";
    if (strcmp(f, "high") == 0) return "高";
    if (strcmp(f, "silent") == 0) return "静音";
    if (strcmp(f, "turbo") == 0) return "强力";
    return f;
}

/* 一次点击的上下文：哪个实体、哪个服务、参数是什么 */
typedef struct {
    char entity[96];
    char service[24];
    char attrs[48];
} clim_cmd_t;

static void clim_chip_cb(lv_event_t *e)
{
    clim_cmd_t *c = (clim_cmd_t *)lv_event_get_user_data(e);
    if (c == NULL || s_cmd_q == NULL) {
        return;
    }
    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", c->entity);
    snprintf(cmd.service, sizeof(cmd.service), "%s", c->service);
    snprintf(cmd.attrs, sizeof(cmd.attrs), "%s", c->attrs);
    xQueueSend(s_cmd_q, &cmd, 0);
}

static void clim_add_chip(lv_obj_t *row, const char *entity, const char *service,
                          const char *attrs, const char *label, bool active)
{
    lv_obj_t *chip = lv_button_create(row);
    lv_obj_set_size(chip, 52, 28);
    lv_obj_set_style_radius(chip, 14, 0);
    lv_obj_set_style_bg_color(chip, active ? lv_color_white() : lv_color_hex(0x24262B), 0);
    lv_obj_set_style_bg_opa(chip, active ? LV_OPA_COVER : LV_OPA_80, 0);
    lv_obj_set_style_pad_all(chip, 0, 0);

    lv_obj_t *lbl = lv_label_create(chip);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_font(lbl, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(lbl, active ? lv_color_black() : COLOR_TEXT, 0);
    lv_obj_center(lbl);

    clim_cmd_t *ctx = calloc(1, sizeof(clim_cmd_t));
    if (ctx != NULL) {
        snprintf(ctx->entity, sizeof(ctx->entity), "%s", entity);
        snprintf(ctx->service, sizeof(ctx->service), "%s", service);
        snprintf(ctx->attrs, sizeof(ctx->attrs), "%s", attrs);
        lv_obj_add_event_cb(chip, clim_chip_cb, LV_EVENT_CLICKED, ctx);
    }
}

static void clim_slider_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    const int v = (int)lv_slider_get_value(slider);
    if (lv_event_get_code(e) != LV_EVENT_RELEASED || s_cmd_q == NULL) {
        return;
    }
    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_clim_entity);
    snprintf(cmd.service, sizeof(cmd.service), "set_temperature");
    snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"temperature\":%d}", v);
    xQueueSend(s_cmd_q, &cmd, 0);
    char buf[24];
    snprintf(buf, sizeof(buf), "目标 %d°", v);
    lv_obj_set_style_text_font(s_clim_temp, &lv_font_cjk16, 0);
    lv_label_set_text(s_clim_temp, buf);
}

static void clim_back_cb(lv_event_t *e)
{
    (void)e;
    ui_home_close_climate();
}

static void build_climate_view(lv_obj_t *scr)
{
    s_clim_page = lv_obj_create(scr);
    lv_obj_remove_style_all(s_clim_page);
    lv_obj_set_size(s_clim_page, 360, 360);
    lv_obj_set_pos(s_clim_page, 0, 0);
    lv_obj_set_style_bg_color(s_clim_page, lv_color_hex(0x0B0B0D), 0);
    lv_obj_set_style_bg_opa(s_clim_page, LV_OPA_COVER, 0);
    lv_obj_remove_flag(s_clim_page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_clim_page, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *back = lv_button_create(s_clim_page);
    lv_obj_set_size(back, 36, 36);
    lv_obj_align(back, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_opa(back, LV_OPA_20, 0);
    lv_obj_set_style_radius(back, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_event_cb(back, clim_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT);
    lv_obj_center(bl);

    s_clim_title = lv_label_create(s_clim_page);
    lv_label_set_text(s_clim_title, "--");
    lv_obj_set_style_text_font(s_clim_title, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_clim_title, COLOR_TEXT, 0);
    lv_obj_align(s_clim_title, LV_ALIGN_TOP_MID, 0, 54);

    s_clim_temp = lv_label_create(s_clim_page);
    lv_label_set_text(s_clim_temp, "--");
    lv_obj_set_style_text_font(s_clim_temp, &lv_font_cjk16, 0);   /* 含中文，不能用西文字体 */
    lv_obj_set_style_text_color(s_clim_temp, COLOR_TEXT, 0);
    lv_obj_align(s_clim_temp, LV_ALIGN_TOP_MID, 0, 84);

    s_clim_span = lv_label_create(s_clim_page);
    lv_label_set_text(s_clim_span, "");
    lv_obj_set_style_text_font(s_clim_span, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_clim_span, COLOR_MUTED, 0);
    lv_obj_align(s_clim_span, LV_ALIGN_TOP_MID, 0, 116);

    s_clim_slider = lv_slider_create(s_clim_page);
    lv_obj_set_width(s_clim_slider, 280);
    lv_obj_set_height(s_clim_slider, 12);
    lv_obj_align(s_clim_slider, LV_ALIGN_TOP_MID, 0, 152);
    lv_slider_set_range(s_clim_slider, 16, 30);
    lv_obj_add_event_cb(s_clim_slider, clim_slider_cb, LV_EVENT_RELEASED, NULL);

    s_clim_modes_row = lv_obj_create(s_clim_page);
    lv_obj_remove_style_all(s_clim_modes_row);
    lv_obj_set_size(s_clim_modes_row, 330, 64);
    lv_obj_align(s_clim_modes_row, LV_ALIGN_TOP_MID, 0, 190);
    lv_obj_set_flex_flow(s_clim_modes_row, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_gap(s_clim_modes_row, 6, 0);
    lv_obj_set_flex_align(s_clim_modes_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_clim_modes_row, 6, 0);

    s_clim_fans_row = lv_obj_create(s_clim_page);
    lv_obj_remove_style_all(s_clim_fans_row);
    lv_obj_set_size(s_clim_fans_row, 330, 64);
    lv_obj_align(s_clim_fans_row, LV_ALIGN_TOP_MID, 0, 262);
    lv_obj_set_flex_flow(s_clim_fans_row, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_gap(s_clim_fans_row, 6, 0);
    lv_obj_set_flex_align(s_clim_fans_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_clim_fans_row, 6, 0);
}

void ui_home_open_climate(const char *entity_id)
{
    if (s_clim_page == NULL || entity_id == NULL) {
        return;
    }
    snprintf(s_clim_entity, sizeof(s_clim_entity), "%s", entity_id);
    const ha_entity_t *e = ha_client_find(s_clim_entity);
    if (e == NULL) {
        return;
    }
    lv_label_set_text(s_clim_title, e->name);

    char buf[48];
    if (e->cur_temp > 0) {
        snprintf(buf, sizeof(buf), "室内 %.1f°  目标 %.0f°", e->cur_temp, e->target_temp);
    } else {
        snprintf(buf, sizeof(buf), "目标 %.0f°", e->target_temp);
    }
    lv_label_set_text(s_clim_temp, buf);
    lv_label_set_text(s_clim_span, e->state);

    lv_slider_set_range(s_clim_slider, (int32_t)e->min_temp, (int32_t)e->max_temp);
    if (e->target_temp > 0) {
        lv_slider_set_value(s_clim_slider, (int32_t)e->target_temp, LV_ANIM_OFF);
    }

    /* 重建模式/风速按钮 */
    lv_obj_clean(s_clim_modes_row);
    if (e->hvac_modes[0]) {
        char tmp[80];
        snprintf(tmp, sizeof(tmp), "%s", e->hvac_modes);
        char *save = NULL;
        for (char *tok = strtok_r(tmp, ",", &save); tok != NULL; tok = strtok_r(NULL, ",", &save)) {
            char attrs[48];
            snprintf(attrs, sizeof(attrs), "{\"hvac_mode\":\"%s\"}", tok);
            clim_add_chip(s_clim_modes_row, s_clim_entity, "set_hvac_mode", attrs,
                          hvac_label(tok), strcmp(tok, e->state) == 0);
        }
    }

    lv_obj_clean(s_clim_fans_row);
    if (e->fan_modes[0]) {
        char tmp[80];
        snprintf(tmp, sizeof(tmp), "%s", e->fan_modes);
        char *save = NULL;
        int n = 0;
        for (char *tok = strtok_r(tmp, ",", &save); tok != NULL && n < 6; tok = strtok_r(NULL, ",", &save), n++) {
            char attrs[48];
            snprintf(attrs, sizeof(attrs), "{\"fan_mode\":\"%s\"}", tok);
            clim_add_chip(s_clim_fans_row, s_clim_entity, "set_fan_mode", attrs,
                          fan_label(tok), false);
        }
    }

    s_clim_visible = true;
    lv_obj_remove_flag(s_clim_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_clim_page, 0, 0);
    lv_obj_set_style_translate_x(s_clim_page, 360, 0);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_clim_page);
    lv_anim_set_values(&a, 360, 0);
    lv_anim_set_time(&a, 220);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, anim_translate_cb);
    lv_anim_start(&a);
    ESP_LOGI(TAG, "打开空调页: %s (modes=%s)", s_clim_entity, e->hvac_modes);
}

void ui_home_close_climate(void)
{
    if (s_clim_page == NULL || !s_clim_visible) {
        return;
    }
    s_clim_visible = false;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_clim_page);
    lv_anim_set_values(&a, 0, 360);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&a, anim_translate_cb);
    lv_anim_start(&a);
    lv_obj_add_flag(s_clim_page, LV_OBJ_FLAG_HIDDEN);
}

bool ui_home_climate_open(void)
{
    return s_clim_visible;
}

static void build_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL) {
        /* 没有默认显示：lv_screen_active() 是 NULL，下面每次 lv_obj_create() 都会
           静默失败（LV_USE_ASSERT_* 全关，LVGL 不挡 NULL），界面会整片空白。 */
        ESP_LOGE(TAG, "构建界面失败：没有活动屏幕（lv_screen_active() 为 NULL），"
                      "检查 LVGL display 是否创建成功");
        return;
    }
    lv_obj_clean(scr);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    bg_image_bind(scr);       /* 背景图铺在最底层 */
    lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    memset(s_pages, 0, sizeof(s_pages));
    memset(s_dots, 0, sizeof(s_dots));
    memset(s_tiles, 0, sizeof(s_tiles));
    memset(s_page_tiles, 0, sizeof(s_page_tiles));
    memset(s_sys_lines, 0, sizeof(s_sys_lines));
    s_selftest_label = NULL;
    s_selftest_shown[0] = '\0';

    /* 设置页 / 蓝牙控件：重建后旧指针全部作废，先清空（refresh 里都看 NULL） */
    s_bt_icon = NULL;
    s_set_page = NULL;
    s_set_scroll = NULL;
    s_mem_bar = NULL;
    s_ota_btn_label = NULL;
    s_set_bt_state = NULL;
    s_set_scan_btn = NULL;
    s_set_scan_label = NULL;
    s_set_conn_btn = NULL;
    s_set_conn_label = NULL;
    s_set_empty = NULL;
    s_set_phone_btn = NULL;
    s_set_phone_label = NULL;
    s_set_bt_btn = NULL;
    s_set_bt_label = NULL;
    s_set_audio_hint = NULL;
    s_set_state_shown[0] = '\0';
    s_set_hint_until = 0;
    s_set_sel_addr[0] = '\0';

    s_json_pages = s_layout.page_count;
    s_page_count = s_json_pages + 1; /* 加合并后的「设置」页（第 N 页） */
#if UI_ASSISTANT_PAGE
    if (s_layout_valid && s_layout.voice_enabled) {
        s_page_count++;              /* 再加小智助手页（第 N+1 页，最后一页） */
    }
#endif
    s_title = NULL;
    s_wifi_icon = NULL;
    s_ha_icon = NULL;
    s_ota_btn = NULL;
    s_ota_bar = NULL;
    s_ota_status = NULL;

    /* 标题：状态栏下面一行，y=30（状态栏图标在 y=8..28） */
    s_title = lv_label_create(scr);
    const char *first_title = page_title(0);
    lv_label_set_text(s_title, first_title);
    lv_obj_set_style_text_color(s_title, COLOR_TEXT, 0);
    lv_obj_set_style_text_font(s_title, title_font_for(first_title), 0);
    lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 30);

    /* ---- 手机式状态栏：蓝牙 / WiFi / HA 三个图标一行，整组居中（y=8..28） ---- */
    s_wifi_color = COLOR_MUTED;
    s_wifi_icon = lv_obj_create(scr);
    if (s_wifi_icon != NULL) {
        lv_obj_remove_style_all(s_wifi_icon);
        lv_obj_set_size(s_wifi_icon, 22, 18);
        /* 居中的那个：x = 180 - 11；两侧图标用 TOP_MID 偏移夹住它 */
        lv_obj_set_pos(s_wifi_icon, 169, 8);
        lv_obj_remove_flag(s_wifi_icon, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(s_wifi_icon, wifi_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    }

    /* 蓝牙：灰 = 未启用/关闭，蓝 = 扫描中/已连接（颜色由 refresh_bt_icon 刷） */
    s_bt_icon = lv_label_create(scr);
    if (s_bt_icon != NULL) {
        lv_label_set_text(s_bt_icon, LV_SYMBOL_BLUETOOTH);
        lv_obj_set_style_text_font(s_bt_icon, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(s_bt_icon, COLOR_MUTED, 0);
        lv_obj_align(s_bt_icon, LV_ALIGN_TOP_MID, -34, 8);
    }

    /* Home Assistant：房子图标（LV_SYMBOL_HOME），在线 HA 蓝、离线灰，不再用「HA」文字 */
    s_ha_icon = lv_label_create(scr);
    if (s_ha_icon != NULL) {
        lv_label_set_text(s_ha_icon, LV_SYMBOL_HOME);
        lv_obj_set_style_text_font(s_ha_icon, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(s_ha_icon, COLOR_MUTED, 0);
        lv_obj_align(s_ha_icon, LV_ALIGN_TOP_MID, 34, 8);
    }

    /* JSON 页 */
    for (int p = 0; p < s_json_pages; p++) {
        lv_obj_t *page = lv_obj_create(scr);
        lv_obj_set_size(page, 360, 360);
        lv_obj_set_pos(page, 0, 0);
        lv_obj_set_style_bg_opa(page, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(page, 0, 0);
        lv_obj_set_style_pad_all(page, 0, 0);
        lv_obj_remove_flag(page, LV_OBJ_FLAG_SCROLLABLE);
            s_pages[p] = page;

        const int n = s_layout.pages[p].tile_count;
        s_page_tiles[p] = n;

        /* 卡片放进一个可纵向滚动的容器：超过 4 个时上下滑 */
        lv_obj_t *grid = lv_obj_create(page);
        lv_obj_remove_style_all(grid);
        lv_obj_set_size(grid, 336, 244);            /* 84 .. 328 */
        lv_obj_set_pos(grid, 12, GRID_TOP);
        lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
        lv_obj_add_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(grid, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(grid, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_style_pad_all(grid, 0, 0);

        for (int i = 0; i < n; i++) {
            /* 两列，行距 CARD_H + GAP；坐标相对滚动容器 */
            make_tile(grid, p, i, i % 2, i / 2);
        }
    }

    /* 设置页（第 N 页）：系统状态 + 蓝牙 + 音频输出 */
    build_settings_page(scr);
#if UI_ASSISTANT_PAGE
    if (s_layout_valid && s_layout.voice_enabled) {
        build_assistant_page(scr);          /* 第 N+1 页：小智页（白底） */
    }
#endif

    /* 翻页圆点 */
    if (s_page_count > 1) {
        for (int i = 0; i < s_page_count; i++) {
            int cx = 180 + (int)((i * 2 - (s_page_count - 1)) * 11); /* 间距 22 */
            lv_obj_t *hit = lv_obj_create(scr);
            lv_obj_set_size(hit, 24, 24);
            lv_obj_set_style_bg_opa(hit, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(hit, 0, 0);
            lv_obj_set_pos(hit, cx - 12, 318);
            lv_obj_add_flag(hit, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(hit, dot_clicked_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

            lv_obj_t *dot = lv_obj_create(hit);
            lv_obj_set_size(dot, 8, 8);
            lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(dot, lv_color_hex(0x3A3A3C), 0);
            lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(dot, 0, 0);
            lv_obj_center(dot);
            s_dots[i] = dot;
        }
    }

    s_hint = lv_label_create(scr);
    lv_label_set_text(s_hint, "");
    lv_obj_set_style_text_color(s_hint, COLOR_MUTED, 0);
    lv_obj_set_style_text_font(s_hint, &lv_font_montserrat_14, 0);
    lv_obj_align(s_hint, LV_ALIGN_CENTER, 0, 30);

    show_page(0);
    refresh_values();
    refresh_status_page();
    refresh_bt_ui();                 /* 蓝牙图标 + 设置页首帧（本版恒「未启用」） */

    if (s_refresh_timer) {
        lv_timer_delete(s_refresh_timer);
    }
    s_refresh_timer = lv_timer_create(refresh_timer_cb, 500, NULL);

    /* 旋钮补发专用定时器：创建后先暂停，只有 s_knob_dirty 时才唤醒 */
    if (s_knob_timer) {
        lv_timer_delete(s_knob_timer);
        s_knob_timer = NULL;
    }
    s_knob_timer = lv_timer_create(detail_knob_timer_cb, DETAIL_KNOB_TICK_MS, NULL);
    s_knob_timer_active = false;
    if (s_knob_timer) {
        lv_timer_pause(s_knob_timer);
    }
    s_knob_dirty = false;
    s_knob_pending_target = DETAIL_KNOB_BR;

    lv_obj_add_event_cb(scr, gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(scr, swipe_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(scr, swipe_cb, LV_EVENT_RELEASED, NULL);

    /* 首帧：页码夹紧后只显示当前页（否则所有页叠在一起） */
    if (s_page >= s_page_count) {
        s_page = 0;
    }
    if (s_page < 0) {
        s_page = 0;
    }
    pages_apply_visibility();
    assistant_ui_tick();             /* 小智页如果是当前页，立刻恢复表情动画 */
    s_animating = false;

    /* 页数对不对，一看日志就知道：s_pages[] 里空着的就是没建出来的页 */
    int built = 0;
    for (int i = 0; i < s_page_count; i++) {
        if (s_pages[i] != NULL) {
            built++;
        }
    }
    ESP_LOGI(TAG, "UI built: %d 个布局页 + 设置页（第 %d 页）+ 小智页（%s），共 %d 页",
             s_json_pages, s_json_pages,
             is_assistant_page(s_json_pages + 1) ? "有" : "无", s_page_count);
    if (built != s_page_count) {
        ESP_LOGE(TAG, "页面没建全：只有 %d/%d 页有效，缺失的页在翻页时会跳过（原因见上面的报错）",
                 built, s_page_count);
    }

    build_detail_view(scr);
    s_detail_visible = false;
    build_chart_view(scr);
    s_chart_visible = false;
    build_climate_view(scr);
    s_clim_visible = false;

    ui_enable_gesture_bubble(scr);   /* 左右滑动翻页 */
}

static void apply_async_cb(void *arg)
{
    (void)arg;
    build_ui();
}

void ui_home_boot(void)
{
    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL) {
        ESP_LOGE(TAG, "启动页无法显示：没有活动屏幕（lv_screen_active() 为 NULL）");
        return;
    }
    lv_obj_clean(scr);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    bg_image_bind(scr);       /* 背景图铺在最底层 */
    lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *t = lv_label_create(scr);
    lv_label_set_text(t, "Orbis");
    lv_obj_set_style_text_color(t, COLOR_TEXT, 0);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_20, 0);
    lv_obj_align(t, LV_ALIGN_CENTER, 0, -20);

    s_hint = lv_label_create(scr);
    lv_label_set_text(s_hint, "connecting...");
    lv_obj_set_style_text_color(s_hint, COLOR_MUTED, 0);
    lv_obj_set_style_text_font(s_hint, &lv_font_montserrat_14, 0);
    lv_obj_align(s_hint, LV_ALIGN_CENTER, 0, 20);

    if (s_cmd_q == NULL) {
        s_cmd_q = xQueueCreate(16, sizeof(cmd_t));
        if (s_cmd_q) {
            xTaskCreate(worker_task, "ui_cmd", 6144, NULL, 4, NULL);
        }
    }
}

void ui_home_apply(const layout_t *layout)
{
    if (layout != NULL && layout->page_count > 0) {
        s_layout = *layout;
        s_layout_valid = true;
    } else {
        /* 还没有收到 cfg：只保留合并后的「设置」页 */
        memset(&s_layout, 0, sizeof(s_layout));
        s_layout_valid = false;
    }
    /* 新工程 LV_USE_OS=NONE：LVGL 9.6 的 lv_lock()/lv_unlock() 本身就是空实现
       （见 components/lvgl/src/osal/lv_os.c），这里保留旧工程的调用形式，等价于 no-op。
       重建界面用 lv_async_call 推迟到下一次 lv_timer_handler（本函数现在由主循环调用）。 */
    lv_lock();
    lv_async_call(apply_async_cb, NULL);
    lv_unlock();
}
