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
    if (f & LV_OBJ_FLAG_ADV_HITTEST)    lv_obj_set_adv_hittest(obj, en);
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
static lv_obj_t *s_detail_power_sw;
/* 圆形拨盘：外环=亮度，内环=色温。每个环下面压一条更宽的半透明弧当"光晕"，
   子对象按创建顺序绘制，所以光晕必须先建、环后建。 */
static lv_obj_t *s_detail_br_arc, *s_detail_br_glow;
static lv_obj_t *s_detail_ct_arc, *s_detail_ct_glow;
static lv_obj_t *s_detail_icon;                  /* 中间：自绘的灯泡 */
static lv_obj_t *s_detail_br_value;              /* 中间：大号百分比 */
static lv_obj_t *s_detail_ct_value;              /* 中间：「3500K 中性」 */
static lv_obj_t *s_detail_off_hint;              /* 关灯时露出的「已关闭」 */
static bool s_detail_visible;

/* 360x360 圆屏上的圆盘几何（圆心 / 半径 / 线宽 / 扫角）。
   标题在 y=58..79、圆点行在 y=318，所以圆心定在 (180,196)：
   LVGL 的 arc 半径是"外径"，所以外环（外径 114 / 宽 20）占 94..114，
   内环（外径 84 / 宽 12）占 72..84，两环之间留 10px；
   竖直方向占 82..310，中间堆栈（灯泡 150..172 / 百分比 180..210 / 色温 212..233）
   全部落在内环以内（内环空出半径 72 的圆）。 */
#define DETAIL_CX            180
#define DETAIL_CY            196
#define DETAIL_ANG_START     135     /* 0°=右、90°=下：135° 是左下，顺时针扫 270° */
#define DETAIL_ANG_END        45     /* = 135+270，跨 0°，缺口正好朝下 */
#define DETAIL_ANG_SWEEP     270
#define DETAIL_BR_R          114     /* 外环（亮度）外径 */
#define DETAIL_BR_W           20     /* 外环线宽 18..22 */
#define DETAIL_CT_R           84     /* 内环（色温）外径 */
#define DETAIL_CT_W           12     /* 内环线宽 10..14 */
#define DETAIL_BR_GLOW_R     128     /* 亮度光晕：102..128，露在环外 14px */
#define DETAIL_BR_GLOW_W      26
#define DETAIL_CT_GLOW_R      90     /* 色温光晕：68..90，环两侧各露 4 / 6px */
#define DETAIL_CT_GLOW_W      22
#define DETAIL_GRAD_STEP       4     /* 色温渐变每 4° 一段（270° 最多 68 段）*/
#define DETAIL_GRAD_WARM  lv_color_hex(0xFFB46B)
#define DETAIL_GRAD_COOL  lv_color_hex(0xBFD8FF)
#define DETAIL_DIM_OPA   LV_OPA_60   /* 没聚焦的环 / 文字压到 ~60% */
#define DETAIL_GLOW_OPA  LV_OPA_20   /* 聚焦时的光晕 */
/* 中间百分比用哪个字号：montserrat_28 是工程里已经编进来、但之前没人引用（被
   --gc-sections 丢掉）的那套字；引用它会让 .rodata 多 ~37.5KB flash。
   想省 flash 就换成 &lv_font_montserrat_20（标题/卡片已经在用，0 成本）。 */
#define DETAIL_PCT_FONT  (&lv_font_montserrat_28)

/* ---- 空调详情页（和灯共用 s_detail，同一套圆盘语言）----
 *
 * 几何：圆心 / 扫角和灯页面完全一样（DETAIL_CX/CY、135°→45°），只是只有一条环：
 *   外环 = 目标温度（外径 114 / 线宽 22，光晕 102..128），范围 = 这台机器的
 *   min_temp..max_temp，每格 = 它的 target_temp_step（0.5 的机器按 0.5 走）。
 *   环值单位用 0.1°C 的整数（AC_UNIT），旋钮路径上不出现浮点，吸附没有误差。
 *
 * 360x360 圆屏、270° 环、缺口朝下 90°。缺口是一个"上窄下宽的楔形"：
 * 半径 > 92 且角度在 45°..135° 之间才没有环，所以越往下越宽。
 * 底部那几行就是按这个楔形排的（每行都验算过不压环）：
 *   风速按钮 92x26  y=234..260   （圆心下方 38..64，全在半径 92 以内，最宽处 124）
 *   5 个模式按钮 36x24 y=284..308（顶边 dy=88，环带外沿 sqrt(128²-88²)=93 > 88 才不压环）
 *   电源开关 72x32  y=314..346   （dy>=118 > 环外径 114，整行在环外）
 */
#define AC_TEMP_R          114     /* 目标温度环外径（= 灯的外环）*/
#define AC_TEMP_W           22
#define AC_TEMP_GLOW_R     128
#define AC_TEMP_GLOW_W      26
#define AC_UNIT             10     /* 弧值单位：0.1°C */
#define AC_ICON_Y          140     /* 模式图标 28x28 */
#define AC_ICON_SIZE        28
#define AC_TEMP_Y          170     /* 中间大号目标温度（montserrat_28）*/
#define AC_ROOM_Y          206     /* 「室内 17.0°C」 */
#define AC_FAN_Y           234     /* 风速按钮 */
#define AC_FAN_W            92
#define AC_FAN_H            26
#define AC_MODE_Y          284     /* 5 个模式按钮：36 宽 + 4 间距 = 196，x 82..278 */
#define AC_MODE_W           36
#define AC_MODE_H           24
#define AC_MODE_GAP          4
#define AC_SW_Y            314
#define AC_SW_W             72
#define AC_SW_H             32
#define AC_MODE_COUNT        5     /* 自动 / 制冷 / 制热 / 除湿 / 送风 */
#define AC_STATE_LEN        12

/* 模式配色（任务指定：制冷蓝 / 制热橙 / 除湿青 / 送风绿 / 自动灰白 / 关=暗）*/
#define AC_COOL_HEX  0x3B82F6
#define AC_HEAT_HEX  0xFF9F0A
#define AC_DRY_HEX   0x2DD4BF
#define AC_FAN_HEX   0x30D158
#define AC_AUTO_HEX  0xE5E5EA
#define AC_OFF_HEX   0x3A3A3C

static lv_obj_t *s_ac_arc;                  /* 目标温度环（可点可拖）*/
static lv_obj_t *s_ac_glow;                 /* 同一条环的光晕 */
static lv_obj_t *s_ac_icon;                 /* 中间：模式图标（自绘）*/
static lv_obj_t *s_ac_temp_value;           /* 中间：大号目标温度 */
static lv_obj_t *s_ac_room;                 /* 「室内 17.0°C」 */
static lv_obj_t *s_ac_fan_btn;               /* 风速按钮（点一下切下一档）*/
static lv_obj_t *s_ac_fan_label;
static lv_obj_t *s_ac_mode_btn[AC_MODE_COUNT];
static lv_obj_t *s_ac_mode_lbl[AC_MODE_COUNT];
static bool s_detail_ac;                    /* 当前详情页开的是 climate.* */
static char s_ac_mode_shown[AC_STATE_LEN];  /* 已经上过色的模式（避免 500ms 重复重绘）*/
static lv_color_t s_ac_accent;              /* 自绘图标读它（已过对比度处理）*/
static int s_sw_geom = -1;                  /* 开关当前的尺寸形态：-1=未知 0=灯 1=空调 */
static int8_t s_sw_on_shown = -1;           /* 开关上次画的开关状态（-1=未知）*/
/* 标签文字缓存：lv_label_set_text 不做内容比较（每次都重新 malloc + 重绘），
   500ms 轮询下必须自己比一下，否则三个标签每秒各重排两次。 */
static char s_ac_temp_txt[16];
static char s_ac_room_txt[28];
static char s_ac_fan_txt[32];
/* 本地覆盖：点了模式 / 风速之后先按本地值显示，HA 落定后交还（和灯的 6s 覆盖同一套）*/
static char s_ac_mode_local[AC_STATE_LEN];
static uint32_t s_ac_mode_local_until;
static char s_ac_fan_local[HA_FAN_LEN];
static uint32_t s_ac_fan_local_until;
static int s_ac_override_t10;               /* 目标温度覆盖（0.1°C）*/
static uint32_t s_ac_override_until;

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

/* ---- 传感器折线图页用的全局状态 ---- */
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
/* 旋钮在灯详情页里调什么：默认亮度，点过色温环就切色温（换灯 / 重新进页时复位）。
   这个变量同时就是"哪个环有焦点"——焦点不需要第二个变量，它只是它的另一种画法。 */
#define DETAIL_KNOB_BR        0
#define DETAIL_KNOB_CT        1
#define DETAIL_KNOB_TEMP      2        /* 空调页：目标温度（圆盘只有一个环，永远是焦点）*/
#define DETAIL_BR_STEP_PCT    5        /* 亮度每格 5% */
#define DETAIL_CT_STEP_K    250        /* 色温每格 250K */
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
static void detail_knob_timer_wake(void);
static bool detail_knob_queue_pending(void);
/* 用户刚拖过滑条时，短时间内以本地值为准（HA 的轮询会慢半拍，灯关着时甚至没有 brightness） */
static bool s_sw_override_on;
static uint32_t s_sw_override_until;
static int s_br_override;
static uint32_t s_br_override_until;
static int s_ct_override;
static uint32_t s_ct_override_until;
#define OVERRIDE_MS 6000

/* 圆盘配色：页面底色（氛围色）+ 亮度环 / 灯泡的最终颜色。
   s_detail_accent 已经按底色做过对比度处理，自绘回调直接读它。 */
static lv_color_t s_detail_bg_col;
static lv_color_t s_detail_accent;
static bool s_detail_fg_dirty;      /* 重建 / 换实体后强制刷一次底色与文字色 */

/* 子页函数的前置声明（refresh_timer_cb 里会用到） */
static void detail_apply_entity(void);
static void detail_apply_foreground(bool dark_text);
static bool detail_light_on(const ha_entity_t *e);
static void detail_ct_range(const ha_entity_t *e, int *lo, int *hi);
static void detail_update_ambience(void);
static void detail_apply_dial_colors(void);
static void detail_apply_focus(void);
/* 空调那一路（定义在后面，前面算底色 / 配色时要先能看到） */
static bool detail_is_climate(const ha_entity_t *e);
static lv_color_t ac_mode_bg(const char *state);
static lv_color_t ac_mode_color(const char *state);
static const char *ac_state_now(const ha_entity_t *e);
static bool ac_state_is_off(const char *st);
static void ac_apply_entity(const ha_entity_t *e);
static void ac_hide_all(bool hide);
static void ac_refresh_chips(const ha_entity_t *e, const char *state);
static void detail_hide_light(bool hide);
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
        /* 记下起手点是否落在"可拖的值控件"上（滑条 / 详情页的圆弧）：
           拖值不能被当成滑动翻页，旋钮也要让位给手指。 */
        lv_obj_t *target = lv_event_get_target(e);
        s_press_on_slider = (target != NULL &&
                             (lv_obj_check_type(target, &lv_slider_class) ||
                              lv_obj_check_type(target, &lv_arc_class)));
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
    if (ui_home_chart_open()) {
        if (dx > 40 && start.x < 70) {
            ui_home_close_chart();
        }
        return;
    }
    if (ui_home_detail_open()) {
        /* 灯和空调是同一个详情页：只有从屏幕左边缘起手的右滑才当"返回" */
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

    /* 空调：和灯走同一个详情页（打开时按实体 id 决定显示哪一组控件）
       旧实现是单独一页 s_clim_page，旋钮在它上面还会翻页，现在统一了 */
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

/* ------------------------------------------------------------------------- */
/* 灯详情页的圆盘：配色 / 焦点 / 自绘                                          */
/* （控制逻辑——乐观更新、100ms 限流、50ms 补发、6s 本地覆盖、手指优先——都在下面 */
/*   那一节，这里只管"怎么画"。）                                              */
/* ------------------------------------------------------------------------- */

/** 感知亮度（0..255）：判断"这个颜色放在这个底色上还看不看得见"。*/
static int detail_luma(lv_color_t c)
{
    return (299 * c.red + 587 * c.green + 114 * c.blue) / 1000;
}

/** 实体给的范围不可信（0 / 反了 / 离谱）时退回 2000..6500。*/
static void detail_ct_range(const ha_entity_t *e, int *lo, int *hi)
{
    int a = (e != NULL) ? e->min_k : 0;
    int b = (e != NULL) ? e->max_k : 0;
    if (a < 1000 || b > 12000 || b - a < 100) {
        a = 2000;
        b = 6500;
    }
    *lo = a;
    *hi = b;
}

/** 当前该用哪个开尔文值：本地覆盖优先，其次实体值，最后 4000K。*/
static int detail_kelvin_now(const ha_entity_t *e)
{
    if (s_ct_override_until > lv_tick_get()) {
        return s_ct_override;
    }
    if (e != NULL && e->color_temp_k > 0) {
        return e->color_temp_k;
    }
    return 4000;
}

/** 当前该用哪个亮度百分比：本地覆盖优先；灯关着时 HA 不返回亮度，别把页面弄成全黑。*/
static int detail_pct_now(const ha_entity_t *e)
{
    int pct = (s_br_override_until > lv_tick_get()) ? s_br_override
                                                   : (e != NULL ? e->brightness * 100 / 255 : 100);
    if (pct <= 0) {
        pct = 100;
    }
    return pct;
}

/** 页面底色：关=纯黑，开=灯色 × 亮度（和原来的氛围色算法完全一致，只是抽出来复用）。*/
static lv_color_t detail_ambient_bg(void)
{
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (s_detail_ac) {
        /* 空调页控件多，不能像灯那样把整页刷成灯色（亮底看不清小字）：
           只给一层很深的模式色，关掉就是纯黑。 */
        return ac_mode_bg(ac_state_now(e));
    }
    if (e == NULL || !detail_light_on(e)) {
        return COLOR_BG;
    }
    const lv_color_t c = kelvin_to_color(detail_kelvin_now(e));

    /* 整个页面的底色就是这只灯：色温决定颜色，亮度决定明暗。
       视觉下限抬到 50%：最暗时页面也有半亮，最高才是纯白。 */
    int pct_eff = 50 + (detail_pct_now(e) / 2);
    if (pct_eff > 100) {
        pct_eff = 100;
    }
    return lv_color_make((uint8_t)(c.red * pct_eff / 100),
                         (uint8_t)(c.green * pct_eff / 100),
                         (uint8_t)(c.blue * pct_eff / 100));
}

/** 灯色：支持色温就按色温算，只有亮度的灯用强调蓝。*/
static lv_color_t detail_light_base(void)
{
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (e != NULL && e->has_tint) {
        return kelvin_to_color(detail_kelvin_now(e));
    }
    return COLOR_ACCENT;
}

/**
 * 把"想要的灯色"变成"在当前底色上看得见的颜色"。
 * 页面底色就是这只灯的颜色（满亮度时正好等于灯色），指示灯再用同一个颜色就会直接
 * 融进背景——圆屏上最容易踩的坑。亮底时压暗一档，暗底时保持原色。
 */
static lv_color_t detail_on_bg(lv_color_t want)
{
    if (detail_luma(s_detail_bg_col) >= 140) {
        return lv_color_darken(want, 110);       /* 亮底：只留 ~43% 亮度 */
    }
    return want;
}

/** 色温渐变第 t% 的颜色（0 = 暖 0xFFB46B，100 = 冷 0xBFD8FF），同样过对比度。*/
static lv_color_t detail_grad_color(int t)
{
    const lv_color_t warm = DETAIL_GRAD_WARM;
    const lv_color_t cool = DETAIL_GRAD_COOL;
    if (t < 0) {
        t = 0;
    }
    if (t > 100) {
        t = 100;
    }
    const int r = warm.red + (cool.red - warm.red) * t / 100;
    const int g = warm.green + (cool.green - warm.green) * t / 100;
    const int b = warm.blue + (cool.blue - warm.blue) * t / 100;
    return detail_on_bg(lv_color_make((uint8_t)r, (uint8_t)g, (uint8_t)b));
}

/** 色温文字的档位：<3000 暖光 / <4500 中性 / <5500 自然 / 其它 冷光。*/
static const char *detail_ct_word(int k)
{
    if (k < 3000) return "暖光";
    if (k < 4500) return "中性";
    if (k < 5500) return "自然";
    return "冷光";
}

/** 中间的百分比文字（旋钮 / 拖动 / 轮询三条路径都走它）。*/
static void detail_set_br_text(int pct)
{
    if (s_detail_br_value == NULL) {
        return;
    }
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", pct);
    lv_label_set_text(s_detail_br_value, buf);
}

/** 中间的「3500K 中性」文字。 */
static void detail_set_ct_text(int k)
{
    if (s_detail_ct_value == NULL) {
        return;
    }
    char buf[28];
    snprintf(buf, sizeof(buf), "%dK %s", k, detail_ct_word(k));
    lv_label_set_text(s_detail_ct_value, buf);
}

static void detail_set_hidden(lv_obj_t *obj, bool hidden)
{
    if (obj == NULL) {
        return;
    }
    if (hidden) {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }
}

/** 环 / 光晕 / 灯泡的颜色：先按底色做对比度处理，再刷到样式上。*/
static void detail_apply_dial_colors(void)
{
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (s_detail_ac) {
        /* ---- 空调：只有一条环，颜色 = 当前 hvac 模式 ----
           模式没变就不碰样式（500ms 轮询不该每帧重绘），只有换模式 / 换实体才刷。 */
        if (e == NULL) {
            return;
        }
        const char *st = ac_state_now(e);
        s_ac_accent = detail_on_bg(ac_mode_color(st));
        if (strcmp(st, s_ac_mode_shown) == 0) {
            return;
        }
        snprintf(s_ac_mode_shown, sizeof(s_ac_mode_shown), "%s", st);
        if (s_ac_arc) {
            lv_obj_set_style_arc_color(s_ac_arc, s_ac_accent, LV_PART_INDICATOR);
        }
        if (s_ac_glow) {
            lv_obj_set_style_arc_color(s_ac_glow, s_ac_accent, LV_PART_INDICATOR);
        }
        if (s_ac_icon) {
            lv_obj_invalidate(s_ac_icon);        /* 自绘图标：样式变了得手动失效 */
        }
        /* 模式按钮的高亮 / 显示哪些键也跟着模式走；能力（hvac_modes）每台不同，
           所以放在这里和"模式真的变了"一起刷，500ms 轮询不会再把 5 个键重绘一遍。 */
        ac_refresh_chips(e, st);
        return;
    }

    if (s_detail_br_arc == NULL) {
        return;
    }
    const bool on = detail_light_on(e);

    /* 外环 = 亮度：灯的当前颜色；不知道色温（只有亮度）的灯用强调蓝 */
    s_detail_accent = detail_on_bg(on ? detail_light_base() : COLOR_ACCENT);
    lv_obj_set_style_arc_color(s_detail_br_arc, s_detail_accent, LV_PART_INDICATOR);
    if (s_detail_br_glow) {
        lv_obj_set_style_arc_color(s_detail_br_glow, s_detail_accent, LV_PART_INDICATOR);
    }
    if (s_detail_ct_glow) {
        /* 色温环的光晕就用当前灯色，看着像"这一圈在发光" */
        lv_obj_set_style_arc_color(s_detail_ct_glow,
                                   detail_on_bg(kelvin_to_color(detail_kelvin_now(e))),
                                   LV_PART_INDICATOR);
    }
    /* 这两块是自绘的：样式变了不会自动重画，得手动失效 */
    if (s_detail_ct_arc) lv_obj_invalidate(s_detail_ct_arc);
    if (s_detail_icon)   lv_obj_invalidate(s_detail_icon);
}

/**
 * 焦点只影响"看起来哪个环是活的"：旋钮调的那个环 指示灯全亮 + 一道光晕 +
 * 中间对应的数字全亮；另一个环压到 60%、光晕关掉。
 * 焦点没有独立变量——s_detail_knob_target 既是旋钮目标，也是焦点。
 */
static void detail_apply_focus(void)
{
    if (s_detail_ac) {
        /* 空调只有一个环：它永远是焦点。开着才有光晕，关掉连光晕都收掉。 */
        const ha_entity_t *e = ha_client_find(s_detail_entity);
        const bool on = (e != NULL) && !ac_state_is_off(ac_state_now(e));
        if (s_ac_arc) {
            lv_obj_set_style_arc_opa(s_ac_arc,
                                     on ? LV_OPA_COVER : DETAIL_DIM_OPA, LV_PART_INDICATOR);
        }
        if (s_ac_glow) {
            lv_obj_set_style_arc_opa(s_ac_glow,
                                     on ? DETAIL_GLOW_OPA : LV_OPA_TRANSP, LV_PART_INDICATOR);
        }
        if (s_ac_temp_value) {
            lv_obj_set_style_text_opa(s_ac_temp_value,
                                      on ? LV_OPA_COVER : DETAIL_DIM_OPA, 0);
        }
        return;
    }

    const bool br = (s_detail_knob_target == DETAIL_KNOB_BR);
    if (s_detail_br_arc) {
        lv_obj_set_style_arc_opa(s_detail_br_arc, br ? LV_OPA_COVER : DETAIL_DIM_OPA,
                                 LV_PART_INDICATOR);
    }
    if (s_detail_br_glow) {
        lv_obj_set_style_arc_opa(s_detail_br_glow, br ? DETAIL_GLOW_OPA : LV_OPA_TRANSP,
                                 LV_PART_INDICATOR);
    }
    if (s_detail_ct_glow) {
        lv_obj_set_style_arc_opa(s_detail_ct_glow, br ? LV_OPA_TRANSP : DETAIL_GLOW_OPA,
                                 LV_PART_INDICATOR);
    }
    if (s_detail_ct_arc) {
        /* 色温环的渐变是自绘的，透明度在自绘回调里按焦点算 → 只要重画 */
        lv_obj_invalidate(s_detail_ct_arc);
    }
    if (s_detail_br_value) {
        lv_obj_set_style_text_opa(s_detail_br_value, br ? LV_OPA_COVER : DETAIL_DIM_OPA, 0);
    }
    if (s_detail_ct_value) {
        lv_obj_set_style_text_opa(s_detail_ct_value, br ? DETAIL_DIM_OPA : LV_OPA_COVER, 0);
    }
}

/**
 * 色温环的自绘渐变：从起点画到当前值，沿角度由暖到冷逐段画。
 * LVGL 9.6 的 lv_arc 只支持纯色指示灯，所以渐变只能自己按角度分段画；
 * 角步长 4°，270° 最多 68 段，红色那段圆弧本来就只有十几个像素，足够细。
 */
static void detail_ct_grad_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    if (obj == NULL || layer == NULL) {
        return;
    }

    const lv_value_precise_t a0 = lv_arc_get_angle_start(obj);
    const lv_value_precise_t a1 = lv_arc_get_angle_end(obj);
    if (a1 <= a0) {
        return;                       /* 值为 0：只有底环 */
    }

    lv_area_t area;
    lv_obj_get_coords(obj, &area);
    const int w = area.x2 - area.x1 + 1;
    const int h = area.y2 - area.y1 + 1;
    const int r = (w < h ? w : h) / 2;

    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.width = DETAIL_CT_W;
    d.radius = (uint16_t)r;
    d.center.x = area.x1 + r;
    d.center.y = area.y1 + r;
    d.opa = (s_detail_knob_target == DETAIL_KNOB_CT) ? LV_OPA_COVER : DETAIL_DIM_OPA;

    for (int seg = (int)a0; seg < (int)a1; seg += DETAIL_GRAD_STEP) {
        int seg_end = seg + DETAIL_GRAD_STEP;
        if (seg_end > (int)a1) {
            seg_end = (int)a1;
        }
        d.color = detail_grad_color((seg - DETAIL_ANG_START) * 100 / DETAIL_ANG_SWEEP);
        d.start_angle = seg;
        d.end_angle = seg_end;
        /* 只有两端做圆头：每段都圆头会画成一串珠子 */
        d.rounded = (seg == (int)a0) || (seg_end == (int)a1);
        lv_draw_arc(layer, &d);
    }
}

/** 圆盘中间那颗灯泡：两笔就够（玻璃泡 = 圆，灯颈/灯座 = 两个小圆角矩形）。*/
static void detail_bulb_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    if (obj == NULL || layer == NULL) {
        return;
    }
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    const int cx = (a.x1 + a.x2) / 2;

    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = s_detail_accent;
    d.bg_opa = LV_OPA_COVER;

    lv_area_t glass = {(lv_coord_t)(cx - 9), a.y1, (lv_coord_t)(cx + 8), (lv_coord_t)(a.y1 + 17)};
    d.radius = 9;
    lv_draw_rect(layer, &d, &glass);

    lv_area_t neck = {(lv_coord_t)(cx - 3), (lv_coord_t)(a.y1 + 16),
                      (lv_coord_t)(cx + 2), (lv_coord_t)(a.y1 + 19)};
    d.radius = 1;
    lv_draw_rect(layer, &d, &neck);

    lv_area_t base = {(lv_coord_t)(cx - 5), (lv_coord_t)(a.y1 + 20),
                      (lv_coord_t)(cx + 4), (lv_coord_t)(a.y1 + 22)};
    lv_draw_rect(layer, &d, &base);
}

/** 根据背景亮度切换文字颜色，保证可读。 */
static void detail_apply_foreground(bool dark_text)
{
    const lv_color_t fg = dark_text ? lv_color_black() : lv_color_white();
    const lv_color_t muted = dark_text ? lv_color_hex(0x3A3A3C) : COLOR_MUTED;
    if (s_detail_title)    lv_obj_set_style_text_color(s_detail_title, fg, 0);
    if (s_back_label)      lv_obj_set_style_text_color(s_back_label, fg, 0);
    if (s_detail_br_value) lv_obj_set_style_text_color(s_detail_br_value, fg, 0);
    if (s_detail_ct_value) lv_obj_set_style_text_color(s_detail_ct_value, fg, 0);
    if (s_detail_off_hint) lv_obj_set_style_text_color(s_detail_off_hint, muted, 0);
    /* 空调页那组（和灯同一容器，只有一组可见）*/
    if (s_ac_temp_value)   lv_obj_set_style_text_color(s_ac_temp_value, fg, 0);
    if (s_ac_room)         lv_obj_set_style_text_color(s_ac_room, muted, 0);
    if (s_ac_fan_label)    lv_obj_set_style_text_color(s_ac_fan_label, fg, 0);
    /* 返回键底色跟着反色，避免在纯白背景上看不见 */
    if (s_back_label && lv_obj_get_parent(s_back_label)) {
        lv_obj_set_style_bg_color(lv_obj_get_parent(s_back_label),
                                  dark_text ? lv_color_hex(0xE5E5EA) : lv_color_hex(0x2C2C2E), 0);
        lv_obj_set_style_bg_opa(lv_obj_get_parent(s_back_label), LV_OPA_70, 0);
    }
}

/** 背景氛围：按"关=黑、开=灯色 × 亮度"更新子页底色，顺便把圆盘配色跟着刷新。 */
static void detail_update_ambience(void)
{
    if (s_detail == NULL) {
        return;
    }
    const lv_color_t bg = detail_ambient_bg();
    /* 只有底色真的变了才动样式：灯页的底色跟着亮度/色温走，
       空调页只在换模式时变；500ms 轮询不该每帧把整页重绘一遍。 */
    if (s_detail_fg_dirty || !lv_color_eq(bg, s_detail_bg_col)) {
        s_detail_fg_dirty = false;
        s_detail_bg_col = bg;
        lv_obj_set_style_bg_color(s_detail, s_detail_bg_col, 0);
        lv_obj_set_style_bg_opa(s_detail, LV_OPA_COVER, 0);

        /* 背景亮就用黑字，背景暗就用白字 */
        detail_apply_foreground(detail_luma(s_detail_bg_col) < 150);   /* true = 深色文字 */
    }
    detail_apply_dial_colors();                                    /* 环 / 光晕 / 灯泡跟着底色 */
}

static void detail_anim_done_cb(lv_anim_t *a);
static void detail_close_done_cb(lv_anim_t *a);

static void detail_apply_entity(void)
{
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (e == NULL) {
        return;
    }
    /* 同一个 s_detail 容器，两种页面：灯（亮度/色温两条环）和空调（温度一条环）。
       s_detail_ac 是"现在开的是哪一种"的唯一真相，旋钮/焦点/补发/配色全看它。 */
    s_detail_ac = detail_is_climate(e);
    ESP_LOGD(TAG, "detail attrs: %s br=%d tint=%d k=%d range=%d-%d state=%s",
             s_detail_ac ? "climate" : "light",
             e->has_brightness, e->has_tint, e->color_temp_k, e->min_k, e->max_k, e->state);
    if (s_detail_title != NULL) {
        lv_label_set_text(s_detail_title, e->name);
    }

    /* 电源开关（两类共用）：HA 原始状态 + 6s 本地覆盖。
       climate.* 的 state 是 auto/cool/heat/.../off，没有 "on"，所以单独判断。 */
    const bool ha_on = s_detail_ac ? !ac_state_is_off(e->state) : e->on;
    bool on = ha_on;
    if (s_sw_override_until > lv_tick_get()) {
        on = s_sw_override_on;
        if (ha_on == on) {
            s_sw_override_until = 0;          /* HA 已跟上，交还控制权 */
        }
    } else {
        s_sw_override_until = 0;
    }
    if (s_detail_power_sw != NULL) {
        /* 空调页底部那排更挤，开关换成窄一点的（灯的 96x46 放不下 5 个模式键）*/
        const int geom = s_detail_ac ? 1 : 0;
        if (s_sw_geom != geom) {
            s_sw_geom = geom;
            lv_obj_set_size(s_detail_power_sw,
                            s_detail_ac ? AC_SW_W : 96, s_detail_ac ? AC_SW_H : 46);
            lv_obj_align(s_detail_power_sw, LV_ALIGN_TOP_MID, 0, s_detail_ac ? AC_SW_Y : 273);
        }
        /* 开关的视觉只在状态真的变了时刷（500ms 轮询不该每帧重绘）*/
        const int8_t shown = on ? 1 : 0;
        if (s_sw_on_shown != shown) {
            s_sw_on_shown = shown;
            if (on) {
                lv_obj_add_state(s_detail_power_sw, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(s_detail_power_sw, LV_STATE_CHECKED);
            }
            lv_obj_set_style_bg_color(s_detail_power_sw,
                                      on ? lv_color_white() : lv_color_hex(0x151517),
                                      LV_PART_MAIN);
            lv_obj_set_style_bg_color(s_detail_power_sw,
                                      on ? lv_color_black() : lv_color_white(), LV_PART_KNOB);
        }
    }
    if (s_detail_ac) {
        /* 灯那组整体藏起来（空调页不显示亮度/色温），空调那组显示出来。
           顺序很重要：先显示整组，再算底色/配色（里面会按能力把不支持的模式键
           重新藏掉），否则 ac_hide_all(false) 会把刚藏掉的键又放出来。 */
        detail_hide_light(true);
        ac_hide_all(false);
        detail_update_ambience();
        ac_apply_entity(e);
        detail_apply_focus();                 /* 开着才有光晕 */
        return;
    }

    ac_hide_all(true);                        /* 空调那组藏起来 */
    detail_update_ambience();
    /* 圆盘：灯关着就整组藏起来、只留「已关闭」；不支持的能力也藏。
       值仍然照常写进去（哪怕藏着），开灯后第一帧就是对的。 */
    if (s_detail_br_arc != NULL) {
        int pct = e->brightness * 100 / 255;
        if (s_br_override_until > lv_tick_get()) {
            pct = s_br_override;              /* 用户刚调过：先按本地值显示 */
        } else {
            s_br_override_until = 0;
        }
        if (pct < 0) {
            pct = 0;
        }
        if (pct > 100) {
            pct = 100;
        }
        lv_arc_set_value(s_detail_br_arc, pct);
        if (s_detail_br_glow) {
            lv_arc_set_value(s_detail_br_glow, pct);
        }
        detail_set_br_text(pct);
        const bool show = on && e->has_brightness;
        detail_set_hidden(s_detail_br_arc, !show);
        detail_set_hidden(s_detail_br_glow, !show);
        detail_set_hidden(s_detail_br_value, !show);
    }
    if (s_detail_ct_arc != NULL) {
        int lo = 2000, hi = 6500;
        detail_ct_range(e, &lo, &hi);
        lv_arc_set_range(s_detail_ct_arc, lo, hi);
        if (s_detail_ct_glow) {
            lv_arc_set_range(s_detail_ct_glow, lo, hi);   /* 光晕的扫角要跟环一致 */
        }
        int k = e->color_temp_k ? e->color_temp_k : (lo + hi) / 2;
        if (s_ct_override_until > lv_tick_get()) {
            k = s_ct_override;
        } else {
            s_ct_override_until = 0;
        }
        lv_arc_set_value(s_detail_ct_arc, k);
        if (s_detail_ct_glow) {
            lv_arc_set_value(s_detail_ct_glow, k);
        }
        detail_set_ct_text(k);
        const bool show = on && e->has_tint;
        detail_set_hidden(s_detail_ct_arc, !show);
        detail_set_hidden(s_detail_ct_glow, !show);
        detail_set_hidden(s_detail_ct_value, !show);
    }
    detail_set_hidden(s_detail_icon, !on);
    detail_set_hidden(s_detail_off_hint, on);
    detail_apply_focus();                     /* 焦点可能被能力判断顶掉过 */
}

/* ------------------------------------------------------------------------- */
/* 空调详情页：和灯共用 s_detail 与同一套圆盘（外环 = 目标温度）              */
/* ------------------------------------------------------------------------- */

/* 模式按钮的固定顺序（auto, cool, heat, dry, fan_only）。
   mode = 发给 HA 的原样字符串，label = 中文标签；都是静态常量，
   所以按钮建一次就够，不占实体内存，也不用每次开页重建（旧实现每次 calloc）。*/
typedef struct {
    const char *mode;
    const char *label;
    uint32_t rgb;
} ac_mode_def_t;

static const ac_mode_def_t k_ac_modes[AC_MODE_COUNT] = {
    {"auto",     "自动", AC_AUTO_HEX},
    {"cool",     "制冷", AC_COOL_HEX},
    {"heat",     "制热", AC_HEAT_HEX},
    {"dry",      "除湿", AC_DRY_HEX},
    {"fan_only", "送风", AC_FAN_HEX},
};

/* 12 个方向的单位向量 ×1024（0°=右、顺时针，和 LVGL 角度一致）：
   自绘图标用查表代替 sinf/cosf，绘制回调里不出现浮点。 */
static const int16_t k_dir_x[12] = {1024, 886, 512, 0, -512, -886, -1024, -886, -512, 0, 512, 886};
static const int16_t k_dir_y[12] = {0, 512, 886, 1024, 886, 512, 0, -512, -886, -1024, -886, -512};

static bool detail_is_climate(const ha_entity_t *e)
{
    return e != NULL && strncmp(e->entity_id, "climate.", 8) == 0;
}

/** 关 / 不可用 / 还没拉到数据，都算"没开"。 */
static bool ac_state_is_off(const char *st)
{
    return st == NULL || st[0] == '\0' || strcmp(st, "off") == 0 ||
           strcmp(st, "unavailable") == 0 || strcmp(st, "unknown") == 0 ||
           strcmp(st, "?") == 0;
}

/** 当前该按哪个模式显示：本地覆盖优先（点了按钮先变色），HA 跟上就交还。 */
static const char *ac_state_now(const ha_entity_t *e)
{
    if (e == NULL) {
        return "off";
    }
    if (s_ac_mode_local_until > lv_tick_get()) {
        if (strcmp(s_ac_mode_local, e->state) == 0) {
            s_ac_mode_local_until = 0;        /* HA 已跟上 */
        } else {
            return s_ac_mode_local;
        }
    }
    return e->state;
}

/** 模式 → 指示灯颜色（制冷蓝 / 制热橙 / 除湿青 / 送风绿 / 自动灰白 / 关=暗）。*/
static lv_color_t ac_mode_color(const char *state)
{
    if (state != NULL) {
        if (strcmp(state, "cool") == 0)     return lv_color_hex(AC_COOL_HEX);
        if (strcmp(state, "heat") == 0)     return lv_color_hex(AC_HEAT_HEX);
        if (strcmp(state, "dry") == 0)      return lv_color_hex(AC_DRY_HEX);
        if (strcmp(state, "fan_only") == 0) return lv_color_hex(AC_FAN_HEX);
        if (strcmp(state, "auto") == 0)     return lv_color_hex(AC_AUTO_HEX);
    }
    return lv_color_hex(AC_OFF_HEX);
}

/** 模式 → 页面底色（很深的同色系；关掉纯黑）。*/
static lv_color_t ac_mode_bg(const char *state)
{
    if (state != NULL) {
        if (strcmp(state, "cool") == 0)     return lv_color_make(0x08, 0x12, 0x24);
        if (strcmp(state, "heat") == 0)     return lv_color_make(0x22, 0x12, 0x06);
        if (strcmp(state, "dry") == 0)      return lv_color_make(0x06, 0x1A, 0x1A);
        if (strcmp(state, "fan_only") == 0) return lv_color_make(0x08, 0x1A, 0x0C);
        if (strcmp(state, "auto") == 0)     return lv_color_make(0x14, 0x14, 0x18);
    }
    return COLOR_BG;
}

/** 风速的中文标签（HA 的 fan_mode 原样值 → 屏幕上的两个汉字）。*/
static const char *fan_label(const char *f)
{
    if (f == NULL) return "--";
    if (strcmp(f, "auto") == 0)        return "自动";
    if (strcmp(f, "low") == 0)         return "低速";
    if (strcmp(f, "medium low") == 0)  return "中低";
    if (strcmp(f, "medium") == 0)      return "中速";
    if (strcmp(f, "medium high") == 0) return "中高";
    if (strcmp(f, "high") == 0)        return "高速";
    if (strcmp(f, "silent") == 0)      return "静音";
    if (strcmp(f, "turbo") == 0)       return "强力";
    if (strcmp(f, "off") == 0)         return "关";
    return f;
}

/* ---- 温度的整数换算（全部以 0.1°C 为单位，旋钮路径上没有浮点）---- */

/** 这台机器每格多少（0.1°C）：0.5°C → 5，1°C → 10；没有/非法按 1°C。 */
static int ac_step_t10(const ha_entity_t *e)
{
    float s = (e != NULL) ? e->temp_step : 1.0f;
    if (!(s > 0.04f) || s > 5.0f) {
        s = 1.0f;
    }
    int t = (int)(s * (float)AC_UNIT + 0.5f);
    if (t < 1) t = 1;
    if (t > 50) t = 50;
    return t;
}

static int ac_min_t10(const ha_entity_t *e)
{
    float v = (e != NULL && e->min_temp >= 5.0f) ? e->min_temp : 16.0f;
    return (int)(v * (float)AC_UNIT + 0.5f);
}

static int ac_max_t10(const ha_entity_t *e)
{
    float v = (e != NULL && e->max_temp > 10.0f) ? e->max_temp : 30.0f;
    const int lo = ac_min_t10(e);
    int hi = (int)(v * (float)AC_UNIT + 0.5f);
    if (hi <= lo) {
        hi = lo + AC_UNIT;                    /* 脏数据兜底：至少留一格 */
    }
    return hi;
}

/**
 * 吸附：min + round((v-min)/step)*step（任务要求的公式，整数实现，无累积误差）。
 * 范围之外的先夹到 min/max；max 不一定落在格点上，所以最后再夹一次。
 */
static int ac_snap(const ha_entity_t *e, int v10)
{
    const int lo = ac_min_t10(e);
    const int hi = ac_max_t10(e);
    const int st = ac_step_t10(e);
    if (v10 < lo) v10 = lo;
    if (v10 > hi) v10 = hi;
    const int n = (v10 - lo + st / 2) / st;   /* round（非负，直接加半）*/
    int out = lo + n * st;
    if (out > hi) out = hi;
    return out;
}

/** 当前该显示的目标温度（0.1°C）：本地覆盖优先，其次实体值，最后取范围中点。 */
static int ac_temp_now(const ha_entity_t *e)
{
    if (s_ac_override_until > lv_tick_get()) {
        if (e != NULL && e->target_temp > 0.0f) {
            const int ha = (int)(e->target_temp * (float)AC_UNIT + 0.5f);
            if (ha == s_ac_override_t10) {
                s_ac_override_until = 0;      /* HA 已跟上，交还控制权 */
            } else {
                return s_ac_override_t10;     /* 我们刚发的值还没落定 */
            }
        } else {
            return s_ac_override_t10;
        }
    }
    if (e != NULL && e->target_temp > 0.0f) {
        return (int)(e->target_temp * (float)AC_UNIT + 0.5f);
    }
    return (ac_min_t10(e) + ac_max_t10(e)) / 2;
}

/** 0.1°C 整数 → "24" / "24.5"（step < 1 才带一位小数），负数也安全。 */
static void ac_fmt_temp(char *buf, size_t n, int t10, int step_t10)
{
    const int neg = (t10 < 0);
    const int a = neg ? -t10 : t10;
    if (step_t10 < AC_UNIT) {
        snprintf(buf, n, "%s%d.%d", neg ? "-" : "", a / AC_UNIT, a % AC_UNIT);
    } else {
        snprintf(buf, n, "%s%d", neg ? "-" : "", (a + AC_UNIT / 2) / AC_UNIT);
    }
}

/** 中间的大号目标温度（旋钮 / 拖动 / 轮询三条路径都走它）。*/
static void ac_set_temp_text(int t10, int step_t10)
{
    if (s_ac_temp_value == NULL) {
        return;
    }
    char num[12];
    char buf[16];
    ac_fmt_temp(num, sizeof(num), t10, step_t10);
    snprintf(buf, sizeof(buf), "%s°", num);
    if (strcmp(buf, s_ac_temp_txt) == 0) {
        return;
    }
    snprintf(s_ac_temp_txt, sizeof(s_ac_temp_txt), "%s", buf);
    lv_label_set_text(s_ac_temp_value, buf);
}

/** 「室内 17.0°C」；没有 current_temperature 就显示 --。 */
static void ac_set_room_text(float cur)
{
    if (s_ac_room == NULL) {
        return;
    }
    char buf[28];
    if (cur > 0.0f) {
        const int t10 = (int)(cur * (float)AC_UNIT + 0.5f);
        snprintf(buf, sizeof(buf), "室内 %d.%d°C", t10 / AC_UNIT, t10 % AC_UNIT);
    } else {
        snprintf(buf, sizeof(buf), "室内 --");
    }
    if (strcmp(buf, s_ac_room_txt) == 0) {
        return;
    }
    snprintf(s_ac_room_txt, sizeof(s_ac_room_txt), "%s", buf);
    lv_label_set_text(s_ac_room, buf);
}

/** 风速按钮上的文字：「风速 中低」；本地刚点过就先按本地值显示。 */
static void ac_set_fan_text(const ha_entity_t *e)
{
    if (s_ac_fan_label == NULL) {
        return;
    }
    const char *f = (e != NULL) ? e->fan_mode : "";
    if (s_ac_fan_local_until > lv_tick_get()) {
        if (e != NULL && strcmp(s_ac_fan_local, e->fan_mode) == 0) {
            s_ac_fan_local_until = 0;         /* HA 已跟上 */
        } else {
            f = s_ac_fan_local;
        }
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "风速 %s", fan_label(f));
    if (strcmp(buf, s_ac_fan_txt) == 0) {
        return;
    }
    snprintf(s_ac_fan_txt, sizeof(s_ac_fan_txt), "%s", buf);
    lv_label_set_text(s_ac_fan_label, buf);
}

/* ---- 自绘模式图标（28x28，比引一堆字体符号省事，也不会缺字形）---- */

/** 在 (x,y) 画一个半径 r 的实心圆。*/
static void ac_dr_dot(lv_layer_t *layer, int x, int y, int r, lv_color_t c)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = c;
    d.bg_opa = LV_OPA_COVER;
    d.radius = (lv_coord_t)r;
    lv_area_t a = {(lv_coord_t)(x - r), (lv_coord_t)(y - r),
                   (lv_coord_t)(x + r), (lv_coord_t)(y + r)};
    lv_draw_rect(layer, &d, &a);
}

/** 画一条圆头线。*/
static void ac_dr_line(lv_layer_t *layer, int x1, int y1, int x2, int y2, int w, lv_color_t c)
{
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.color = c;
    d.width = w;
    d.round_start = true;
    d.round_end = true;
    d.p1.x = x1;
    d.p1.y = y1;
    d.p2.x = x2;
    d.p2.y = y2;
    lv_draw_line(layer, &d);
}

/** 画一段圆弧（角度和 LVGL 一致：0°=右、顺时针）*/
static void ac_dr_arc(lv_layer_t *layer, int cx, int cy, int r, int w,
                      int a0, int a1, lv_color_t c)
{
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.color = c;
    d.opa = LV_OPA_COVER;
    d.width = w;
    d.radius = (uint16_t)r;
    d.center.x = cx;
    d.center.y = cy;
    d.start_angle = a0;
    d.end_angle = a1;
    d.rounded = true;
    lv_draw_arc(layer, &d);
}

/** 第 idx 个方向（0..11，每 30°）上半径 r 的坐标点。*/
static void ac_dir_point(int cx, int cy, int idx, int r, int *x, int *y)
{
    const int i = ((idx % 12) + 12) % 12;
    *x = cx + (int)((k_dir_x[i] * r) / 1024);
    *y = cy + (int)((k_dir_y[i] * r) / 1024);
}

/** 按当前模式画中间那颗图标：雪花 / 太阳 / 水滴 / 风扇 / A / 电源。*/
static void ac_icon_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    if (obj == NULL || layer == NULL) {
        return;
    }
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    const int cx = (a.x1 + a.x2) / 2;
    const int cy = (a.y1 + a.y2) / 2;
    const lv_color_t c = s_ac_accent;
    const char *m = s_ac_mode_shown;

    if (strcmp(m, "cool") == 0) {
        /* 雪花：3 条穿过圆心的线 = 6 根叉，端头各挂两小段斜枝 */
        for (int i = 0; i < 6; i += 2) {
            int x1, y1, x2, y2;
            ac_dir_point(cx, cy, i, 11, &x1, &y1);
            ac_dir_point(cx, cy, i + 6, 11, &x2, &y2);
            ac_dr_line(layer, x1, y1, x2, y2, 2, c);
            for (int k = -1; k <= 1; k += 2) {
                int bx, by, tx, ty;
                ac_dir_point(cx, cy, i, 6, &bx, &by);
                ac_dir_point(cx, cy, i + k, 9, &tx, &ty);
                ac_dr_line(layer, bx, by, tx, ty, 2, c);
                ac_dir_point(cx, cy, i + 6, 6, &bx, &by);
                ac_dir_point(cx, cy, i + 6 - k, 9, &tx, &ty);
                ac_dr_line(layer, bx, by, tx, ty, 2, c);
            }
        }
    } else if (strcmp(m, "heat") == 0) {
        /* 太阳：实心圆 + 6 根短光线 */
        ac_dr_dot(layer, cx, cy, 5, c);
        for (int i = 0; i < 12; i += 2) {
            int x1, y1, x2, y2;
            ac_dir_point(cx, cy, i, 8, &x1, &y1);
            ac_dir_point(cx, cy, i, 12, &x2, &y2);
            ac_dr_line(layer, x1, y1, x2, y2, 2, c);
        }
    } else if (strcmp(m, "dry") == 0) {
        /* 水滴：下面一个实心圆 + 上面两条斜边 */
        ac_dr_dot(layer, cx, cy + 3, 5, c);
        ac_dr_line(layer, cx, cy - 11, cx - 5, cy + 2, 2, c);
        ac_dr_line(layer, cx, cy - 11, cx + 5, cy + 2, 2, c);
        ac_dr_line(layer, cx - 3, cy + 3, cx + 3, cy + 3, 2, c);
    } else if (strcmp(m, "fan_only") == 0) {
        /* 风扇：一圈细环 + 三片扇叶（0/120/240 三个方向）*/
        ac_dr_arc(layer, cx, cy, 10, 2, 0, 360, c);
        for (int i = 0; i < 12; i += 4) {
            int x1, y1, x2, y2;
            ac_dir_point(cx, cy, i, 3, &x1, &y1);
            ac_dir_point(cx, cy, i, 8, &x2, &y2);
            ac_dr_line(layer, x1, y1, x2, y2, 3, c);
        }
        ac_dr_dot(layer, cx, cy, 2, c);
    } else if (strcmp(m, "auto") == 0) {
        /* 自动：一个 A（两条腿 + 一横）*/
        ac_dr_line(layer, cx - 7, cy + 9, cx, cy - 9, 2, c);
        ac_dr_line(layer, cx, cy - 9, cx + 7, cy + 9, 2, c);
        ac_dr_line(layer, cx - 4, cy + 2, cx + 4, cy + 2, 2, c);
    } else {
        /* 关 / 未知：电源符号（缺口圆环 + 顶部竖线）*/
        ac_dr_arc(layer, cx, cy, 9, 2, 120, 60, c);
        ac_dr_line(layer, cx, cy - 12, cx, cy - 3, 3, c);
    }
}

/** 模式按钮高亮：当前模式用该模式的颜色填充，其余保持深灰。 */
static void ac_refresh_chips(const ha_entity_t *e, const char *state)
{
    const bool on = !ac_state_is_off(state);
    for (int i = 0; i < AC_MODE_COUNT; i++) {
        lv_obj_t *btn = s_ac_mode_btn[i];
        lv_obj_t *lbl = s_ac_mode_lbl[i];
        if (btn == NULL) {
            continue;
        }
        const bool active = on && (strcmp(state, k_ac_modes[i].mode) == 0);
        const lv_color_t fill = active ? detail_on_bg(lv_color_hex(k_ac_modes[i].rgb))
                                       : lv_color_hex(0x24262B);
        lv_obj_set_style_bg_color(btn, fill, 0);
        lv_obj_set_style_bg_opa(btn, active ? LV_OPA_COVER : LV_OPA_80, 0);
        if (lbl != NULL) {
            const lv_color_t tc = active ? (detail_luma(fill) >= 140 ? lv_color_black()
                                                                     : lv_color_white())
                                         : COLOR_TEXT;
            lv_obj_set_style_text_color(lbl, tc, 0);
        }
        /* 只留这台机器 hvac_modes 里有的（顺序永远按 k_ac_modes）；
           还没拉到 hvac_modes（开机头几秒）就先全显示，别让整行消失。 */
        const bool have_list = (e != NULL && e->hvac_mode_count > 0);
        detail_set_hidden(btn, have_list && !ha_entity_has_hvac_mode(e, k_ac_modes[i].mode));
    }
}

/** 空调那组控件全部显示/隐藏（切到灯页时整组藏起来）。 */
static void ac_hide_all(bool hide)
{
    detail_set_hidden(s_ac_arc, hide);
    detail_set_hidden(s_ac_glow, hide);
    detail_set_hidden(s_ac_icon, hide);
    detail_set_hidden(s_ac_temp_value, hide);
    detail_set_hidden(s_ac_room, hide);
    detail_set_hidden(s_ac_fan_btn, hide);
    for (int i = 0; i < AC_MODE_COUNT; i++) {
        detail_set_hidden(s_ac_mode_btn[i], hide);
    }
}

/** 灯那组控件全部隐藏/显示（切到空调页时整组藏起来）。 */
static void detail_hide_light(bool hide)
{
    detail_set_hidden(s_detail_br_arc, hide);
    detail_set_hidden(s_detail_br_glow, hide);
    detail_set_hidden(s_detail_ct_arc, hide);
    detail_set_hidden(s_detail_ct_glow, hide);
    detail_set_hidden(s_detail_br_value, hide);
    detail_set_hidden(s_detail_ct_value, hide);
    detail_set_hidden(s_detail_icon, hide);
    detail_set_hidden(s_detail_off_hint, hide);
}

/** 把实体状态刷到空调页的所有控件上（轮询 500ms 也走这里）。 */
static void ac_apply_entity(const ha_entity_t *e)
{
    const int st = ac_step_t10(e);
    const int lo = ac_min_t10(e);
    const int hi = ac_max_t10(e);
    const int t10 = ac_snap(e, ac_temp_now(e));

    /* 外环：范围 = 这台机器的 min..max；值 = 目标温度（0.1°C）*/
    if (s_ac_arc != NULL) {
        lv_arc_set_range(s_ac_arc, lo, hi);
        lv_arc_set_value(s_ac_arc, t10);
    }
    if (s_ac_glow != NULL) {
        lv_arc_set_range(s_ac_glow, lo, hi);
        lv_arc_set_value(s_ac_glow, t10);
    }
    ac_set_temp_text(t10, st);
    ac_set_room_text(e->cur_temp);
    ac_set_fan_text(e);
    /* 模式按钮的高亮 / 可见性在 detail_apply_dial_colors 里（模式变了才刷）*/
    detail_set_hidden(s_ac_fan_btn, e->fan_mode_count == 0);
}

/** 旋钮在空调页上调目标温度：整格 = 这台机器的 target_temp_step。 */
static bool ac_knob_step(const ha_entity_t *e, int delta)
{
    if (s_ac_arc == NULL || e == NULL) {
        return false;
    }
    const int st = ac_step_t10(e);
    const int cur = ac_snap(e, (int)lv_arc_get_value(s_ac_arc));
    const int v = ac_snap(e, cur + delta * st);
    if (v == cur) {
        return true;                          /* 已经到头：吃掉这一格，不发命令 */
    }
    /* 1) 立刻更新界面（环 + 光晕 + 大号温度），不等网络 */
    lv_arc_set_value(s_ac_arc, v);
    if (s_ac_glow) {
        lv_arc_set_value(s_ac_glow, v);
    }
    ac_set_temp_text(v, st);
    s_ac_override_t10 = v;
    s_ac_override_until = lv_tick_get() + OVERRIDE_MS;

    /* 2) 网络命令：复用灯那套 100ms 限流 + 50ms 补发 + 合并 */
    const uint32_t now = lv_tick_get();
    s_knob_dirty = true;
    s_knob_pending_target = DETAIL_KNOB_TEMP;
    s_knob_pending_value = v;
    s_knob_last_change = now;
    if ((uint32_t)(now - s_knob_last_send) >= DETAIL_KNOB_SEND_MS) {
        detail_knob_queue_pending();
    }
    if (s_knob_dirty) {
        detail_knob_timer_wake();
    }
    return true;
}

/** 点模式按钮：立刻本地高亮，命令走队列。 */
static void ac_mode_cb(lv_event_t *e)
{
    const ac_mode_def_t *def = (const ac_mode_def_t *)lv_event_get_user_data(e);
    if (def == NULL || s_cmd_q == NULL || s_detail_entity[0] == '\0') {
        return;
    }
    snprintf(s_ac_mode_local, sizeof(s_ac_mode_local), "%s", def->mode);
    s_ac_mode_local_until = lv_tick_get() + OVERRIDE_MS;
    /* 选模式 = 开机：开关也立刻跟着亮（HA 跟上后自动交还）*/
    s_sw_override_on = true;
    s_sw_override_until = lv_tick_get() + OVERRIDE_MS;
    s_sw_on_shown = -1;
    const ha_entity_t *ent = ha_client_find(s_detail_entity);
    if (ent != NULL) {
        ac_refresh_chips(ent, def->mode);
    }
    /* 配色 / 图标也跟着新模式走 */
    if (strcmp(def->mode, s_ac_mode_shown) != 0) {
        snprintf(s_ac_mode_shown, sizeof(s_ac_mode_shown), "%s", def->mode);
        s_ac_accent = detail_on_bg(lv_color_hex(def->rgb));
        if (s_ac_arc) lv_obj_set_style_arc_color(s_ac_arc, s_ac_accent, LV_PART_INDICATOR);
        if (s_ac_glow) lv_obj_set_style_arc_color(s_ac_glow, s_ac_accent, LV_PART_INDICATOR);
        if (s_ac_icon) lv_obj_invalidate(s_ac_icon);
    }
    s_detail_fg_dirty = true;                 /* 底色要跟着模式换 */
    detail_update_ambience();
    detail_apply_focus();

    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_detail_entity);
    snprintf(cmd.service, sizeof(cmd.service), "set_hvac_mode");
    snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"hvac_mode\":\"%s\"}", def->mode);
    xQueueSend(s_cmd_q, &cmd, 0);
}

/** 点风速按钮：在实体报上来的 fan_modes 里循环到下一档。 */
static void ac_fan_cb(lv_event_t *e)
{
    (void)e;
    const ha_entity_t *ent = ha_client_find(s_detail_entity);
    if (ent == NULL || s_cmd_q == NULL || ent->fan_mode_count == 0) {
        return;
    }
    int n = ent->fan_mode_count;
    if (n > HA_MODE_MAX) {
        n = HA_MODE_MAX;
    }
    int idx = -1;
    for (int i = 0; i < n; i++) {
        if (strcmp(ent->fan_modes[i], ent->fan_mode) == 0) {
            idx = i;
            break;
        }
    }
    idx = (idx + 1) % n;
    const char *next = ent->fan_modes[idx];

    /* 乐观更新：本地先显示，HA 落定后由轮询 / WS 覆盖（走同一个 setter，缓存才对得上）*/
    snprintf(s_ac_fan_local, sizeof(s_ac_fan_local), "%s", next);
    s_ac_fan_local_until = lv_tick_get() + OVERRIDE_MS;
    ac_set_fan_text(ent);

    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_detail_entity);
    snprintf(cmd.service, sizeof(cmd.service), "set_fan_mode");
    snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"fan_mode\":\"%.11s\"}", next);
    xQueueSend(s_cmd_q, &cmd, 0);
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
    if (target == DETAIL_KNOB_TEMP) {
        /* 空调：value 是 0.1°C 的整数，整度就发整数（HA 看到 24 比 24.0 舒服），
           半度（0.5 的机器）才带一位小数。 */
        const int neg = (value < 0);
        const int a = neg ? -value : value;
        snprintf(cmd.service, sizeof(cmd.service), "set_temperature");
        if (a % AC_UNIT == 0) {
            snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"temperature\":%s%d}",
                     neg ? "-" : "", a / AC_UNIT);
        } else {
            snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"temperature\":%s%d.%d}",
                     neg ? "-" : "", a / AC_UNIT, a % AC_UNIT);
        }
        return xQueueSend(s_cmd_q, &cmd, 0) == pdTRUE;
    }
    snprintf(cmd.service, sizeof(cmd.service), "turn_on");
    if (target == DETAIL_KNOB_CT) {
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"color_temp_kelvin\":%d}", value);
    } else {
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"brightness_pct\":%d}", value);
    }
    return xQueueSend(s_cmd_q, &cmd, 0) == pdTRUE;
}

/** 旋钮值还要不要发出去：灯关着就不发（不能把它重新点亮）；空调关着也允许
 *  预先设目标温度（HA 的 set_temperature 不会开机）。*/
static bool detail_knob_can_send(const ha_entity_t *e)
{
    if (e == NULL) {
        return false;
    }
    if (s_knob_pending_target == DETAIL_KNOB_TEMP || detail_is_climate(e)) {
        return true;
    }
    return detail_light_on(e);
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
 * 旋钮在详情页里调值（灯：亮度/色温；空调：目标温度）。
 * 返回 true 表示"这次旋转被详情页吃掉了，并且真的调了值"（调用方响一下触觉）；
 * 返回 false 表示没消费（详情页没开 / 灯是关的 / 这只灯不支持 / 手指正按着圆盘），
 * 调用方据此决定要不要翻页 + 震动。
 *
 * 路由：s_detail 是灯和空调共用的一个容器，s_detail_ac 决定这次落到哪一组控件上。
 */
bool ui_home_detail_knob(int delta)
{
    if (delta == 0 || !s_detail_visible || s_detail == NULL) {
        return false;
    }
    if (s_press_on_slider) {
        return false;                  /* 手指正在拖圆盘：这一格让给手指 */
    }
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (e == NULL) {
        return false;
    }
    if (s_detail_ac) {
        return ac_knob_step(e, delta); /* 空调：整格 = 这台机器的 temp_step */
    }
    if (!detail_light_on(e)) {
        return false;                  /* 灯关着：旋钮什么也不做（也不翻页）*/
    }
    const int target = detail_knob_target_for(e);
    if (target < 0) {
        return false;
    }

    lv_obj_t *dial = (target == DETAIL_KNOB_CT) ? s_detail_ct_arc : s_detail_br_arc;
    lv_obj_t *glow = (target == DETAIL_KNOB_CT) ? s_detail_ct_glow : s_detail_br_glow;
    if (dial == NULL) {
        return false;
    }

    /* 以圆盘当前值为基准（它已经反映了实体值或本地刚调过的值）*/
    int v = (int)lv_arc_get_value(dial);
    v += (target == DETAIL_KNOB_CT) ? delta * DETAIL_CT_STEP_K : delta * DETAIL_BR_STEP_PCT;

    const int vmin = (int)lv_arc_get_min_value(dial);
    const int vmax = (int)lv_arc_get_max_value(dial);
    if (v < vmin) {
        v = vmin;
    }
    if (v > vmax) {
        v = vmax;
    }
    if (v == (int)lv_arc_get_value(dial)) {
        return true;                   /* 已经到头：吃掉这一格，但不用发命令 */
    }

    /* 1) 立刻更新界面（圆盘 + 光晕 + 中间数字 + 底色），不等网络 */
    lv_arc_set_value(dial, v);
    if (glow) {
        lv_arc_set_value(glow, v);
    }
    if (target == DETAIL_KNOB_CT) {
        detail_set_ct_text(v);
        s_ct_override = v;
        s_ct_override_until = lv_tick_get() + OVERRIDE_MS;
    } else {
        detail_set_br_text(v);
        s_br_override = v;
        s_br_override_until = lv_tick_get() + OVERRIDE_MS;
    }
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
 * 补发最后一条命令（亮度 / 色温 / 空调目标温度）。只在 LVGL 线程调用，
 * 真正的网络请求仍在 worker 里。只在"停转 >= 80ms"后发（转动中的节奏由旋钮事件
 * 自己按 100ms 限流），这样每条最后值只会打一行日志，串口上能直接量延迟。
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

    /* 灯已经关了（或实体没了）：丢掉待发值，绝不能把它重新点亮。
       空调允许在关机状态下预置目标温度，所以走 detail_knob_can_send 判断。 */
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (!detail_knob_can_send(e)) {
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
    if (target == DETAIL_KNOB_TEMP) {
        ac_fmt_temp(vbuf, sizeof(vbuf), value, AC_UNIT);   /* 日志里统一带一位小数 */
        const size_t n = strlen(vbuf);
        if (n + 2 < sizeof(vbuf)) {
            vbuf[n] = 'C';
            vbuf[n + 1] = '\0';
        }
    } else if (target == DETAIL_KNOB_CT) {
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

/** 点一下圆盘（外环或内环）就把焦点/旋钮切到那个环上（拖动也会先经过 PRESSED）。 */
static void detail_dial_pressed_cb(lv_event_t *e)
{
    lv_obj_t *dial = lv_event_get_target(e);
    /* 手指按到圆盘上：手指优先。丢掉还没补发的旋钮值，别让它在松手后
       又补一条旋钮命令，盖掉手指刚选的值。 */
    s_knob_dirty = false;
    detail_knob_timer_park();
    if (dial == s_detail_ct_arc) {
        s_detail_knob_target = DETAIL_KNOB_CT;
    } else if (dial == s_detail_br_arc) {
        s_detail_knob_target = DETAIL_KNOB_BR;
    } else if (dial == s_ac_arc) {
        s_detail_knob_target = DETAIL_KNOB_TEMP;   /* 空调只有一个环 */
    }
    detail_apply_focus();
}

/** 点中间的大号百分比 = 聚焦亮度环。 */
static void detail_focus_br_cb(lv_event_t *e)
{
    (void)e;
    s_detail_knob_target = DETAIL_KNOB_BR;
    detail_apply_focus();
}

/** 点中间的色温文字 = 聚焦色温环。 */
static void detail_focus_ct_cb(lv_event_t *e)
{
    (void)e;
    s_detail_knob_target = DETAIL_KNOB_CT;
    detail_apply_focus();
}

static void detail_dial_changed_cb(lv_event_t *e)
{
    lv_obj_t *dial = lv_event_get_target(e);
    const lv_event_code_t code = lv_event_get_code(e);
    const ha_entity_t *ent = ha_client_find(s_detail_entity);

    /* ---- 空调：目标温度环。拖动时按这台机器的 temp_step 吸附 ---- */
    if (dial == s_ac_arc) {
        const int step = ac_step_t10(ent);
        int v = (int)lv_arc_get_value(dial);
        const int snapped = ac_snap(ent, v);
        if (snapped != v) {
            /* 吸附会再触发一次 VALUE_CHANGED；第二次进来 snapped == v 就停了 */
            lv_arc_set_value(dial, snapped);
            v = snapped;
        }
        if (s_ac_glow) {
            lv_arc_set_value(s_ac_glow, v);
        }
        ac_set_temp_text(v, step);
        s_ac_override_t10 = v;
        s_ac_override_until = lv_tick_get() + OVERRIDE_MS;
        if (code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) {
            return;                        /* 拖动中只更新显示，松手才发命令 */
        }
        char vbuf[16];
        ac_fmt_temp(vbuf, sizeof(vbuf), v, AC_UNIT);
        ESP_LOGI(TAG, "温度圆盘松手: %s°C（每格 %d.%d°C）", vbuf, step / AC_UNIT, step % AC_UNIT);
        if (s_detail_entity[0] == '\0' || s_cmd_q == NULL) {
            return;
        }
        cmd_t c;
        memset(&c, 0, sizeof(c));
        snprintf(c.entity, sizeof(c.entity), "%s", s_detail_entity);
        snprintf(c.service, sizeof(c.service), "set_temperature");
        if (v % AC_UNIT == 0) {
            snprintf(c.attrs, sizeof(c.attrs), "{\"temperature\":%d}", v / AC_UNIT);
        } else {
            snprintf(c.attrs, sizeof(c.attrs), "{\"temperature\":%d.%d}",
                     v / AC_UNIT, v % AC_UNIT);
        }
        xQueueSend(s_cmd_q, &c, 0);
        return;
    }

    const bool is_ct = (dial == s_detail_ct_arc);
    const int v = (int)lv_arc_get_value(dial);

    /* 光晕是独立对象：值要手动跟一遍，扫角才对得上 */
    lv_obj_t *glow = is_ct ? s_detail_ct_glow : s_detail_br_glow;
    if (glow) {
        lv_arc_set_value(glow, v);
    }

    if (is_ct) {
        detail_set_ct_text(v);
        s_ct_override = v;
        s_ct_override_until = lv_tick_get() + OVERRIDE_MS;
    } else {
        detail_set_br_text(v);
        s_br_override = v;
        s_br_override_until = lv_tick_get() + OVERRIDE_MS;
    }
    detail_update_ambience();   /* 拖动时底色实时跟着变 */

    if (code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) {
        return;   /* 拖动过程中只更新显示，松手才发命令 */
    }
    ESP_LOGI(TAG, "圆盘松手: %s value=%d", is_ct ? "色温" : "亮度", v);
    if (s_detail_entity[0] == '\0' || s_cmd_q == NULL) {
        return;
    }
    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_detail_entity);
    snprintf(cmd.service, sizeof(cmd.service), "turn_on");
    if (is_ct) {
        s_ct_override = v;
        s_ct_override_until = lv_tick_get() + OVERRIDE_MS;
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"color_temp_kelvin\":%d}", v);
    } else {
        s_br_override = v;
        s_br_override_until = lv_tick_get() + OVERRIDE_MS;
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"brightness_pct\":%d}", v);
        /* 调亮度会把灯点亮，但按钮交给轮询去更新，避免拖动时乱跳 */
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
    s_sw_on_shown = -1;               /* 底色/滑块颜色立刻按新状态重刷 */

    if (!want_on) {
        /* 关掉：丢掉还没补发的旋钮值，别让它在关机后又发一条 turn_on */
        s_knob_dirty = false;
        detail_knob_timer_park();
        if (s_detail_ac) {
            /* 空调关机 = hvac 模式变成 off：本地先跟着变（环变暗、模式键全灭），
               别让模式灯还亮着等 HA 那 5 秒轮询 */
            snprintf(s_ac_mode_local, sizeof(s_ac_mode_local), "off");
            s_ac_mode_local_until = lv_tick_get() + OVERRIDE_MS;
            s_ac_mode_shown[0] = '\0';
            s_detail_fg_dirty = true;
            detail_update_ambience();
            detail_apply_focus();
        }
    } else if (s_detail_ac) {
        /* 开机：恢复到 HA 记着的那个模式（等它下一次 state_changed / 轮询）*/
        s_ac_mode_local_until = 0;
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

/**
 * 建一条 270° 圆弧（135° 起、缺口朝下）：
 *   halo=true  光晕：不要底环、不接收触摸，只当聚焦时的一层柔光；
 *   halo=false 圆环：深灰底环 + 可点可拖的指示灯。
 * 半径/线宽/圆心全部由几何宏决定，不依赖主题样式。
 */
static lv_obj_t *detail_make_arc(lv_obj_t *parent, int radius, int width, bool halo)
{
    lv_obj_t *arc = lv_arc_create(parent);
    if (arc == NULL) {
        return NULL;
    }
    lv_obj_set_size(arc, radius * 2, radius * 2);
    lv_obj_set_pos(arc, DETAIL_CX - radius, DETAIL_CY - radius);

    /* 所有 padding 归零：环心半径 = 对象尺寸 / 2，和几何宏一一对应 */
    lv_obj_set_style_pad_all(arc, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(arc, 0, LV_PART_INDICATOR);
    lv_obj_set_style_pad_all(arc, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(arc, 0, LV_PART_MAIN);

    /* 轨道：深灰 0x2C2C2E、圆头、270° */
    lv_obj_set_style_arc_width(arc, width, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, COLOR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_MAIN);

    /* 指示灯：颜色由 detail_apply_dial_colors / 自绘回调决定 */
    lv_obj_set_style_arc_width(arc, width, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_INDICATOR);

    /* 不要 LVGL 默认那个圆头滑块：视觉上只要一圈 */
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_border_width(arc, 0, LV_PART_KNOB);
    lv_obj_set_style_shadow_width(arc, 0, LV_PART_KNOB);

    lv_arc_set_bg_angles(arc, DETAIL_ANG_START, DETAIL_ANG_END);
    /* 点/拖一次到位：默认 720°/s 会让手指追上半天，点一下也跳不过去 */
    lv_arc_set_change_rate(arc, 3600);
    /* 命中区域只留这一圈（默认外扩 13px 会伸进隔壁环里抢手指）*/
    lv_obj_set_ext_click_area(arc, 4);
    /* 精确命中：圆角/内圈空白都按真实形状算 */
    lv_obj_add_flag(arc, LV_OBJ_FLAG_ADV_HITTEST);

    if (halo) {
        lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);      /* 不要底环 */
        lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_INDICATOR); /* 由焦点点亮 */
        lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    }
    return arc;
}

/** 递归数一下详情页的控件数：串口上一眼能看出对象有没有暴涨。 */
static int detail_count_objs(lv_obj_t *obj)
{
    if (obj == NULL) {
        return 0;
    }
    int n = 1;
    const uint32_t cnt = lv_obj_get_child_count(obj);
    for (uint32_t i = 0; i < cnt; i++) {
        n += detail_count_objs(lv_obj_get_child(obj, i));
    }
    return n;
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

    /* ---- 圆盘：先光晕（在下面），再圆环（在上面）---- */
    s_detail_br_glow = detail_make_arc(s_detail, DETAIL_BR_GLOW_R, DETAIL_BR_GLOW_W, true);
    lv_arc_set_range(s_detail_br_glow, 0, 100);
    lv_arc_set_value(s_detail_br_glow, 0);

    s_detail_ct_glow = detail_make_arc(s_detail, DETAIL_CT_GLOW_R, DETAIL_CT_GLOW_W, true);
    lv_arc_set_range(s_detail_ct_glow, 2000, 6500);
    lv_arc_set_value(s_detail_ct_glow, 4000);

    /* 外环 = 亮度：0..100%（和原来滑条同一套百分比）*/
    s_detail_br_arc = detail_make_arc(s_detail, DETAIL_BR_R, DETAIL_BR_W, false);
    lv_arc_set_range(s_detail_br_arc, 0, 100);
    lv_arc_set_value(s_detail_br_arc, 0);
    lv_obj_add_event_cb(s_detail_br_arc, detail_dial_pressed_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_detail_br_arc, detail_dial_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_detail_br_arc, detail_dial_changed_cb, LV_EVENT_RELEASED, NULL);

    /* 内环 = 色温：范围在 detail_apply_entity 里按实体刷新 */
    s_detail_ct_arc = detail_make_arc(s_detail, DETAIL_CT_R, DETAIL_CT_W, false);
    lv_arc_set_range(s_detail_ct_arc, 2000, 6500);
    lv_arc_set_value(s_detail_ct_arc, 4000);
    /* 原生指示灯永不绘制：这一段是自绘的暖→冷渐变 */
    lv_obj_set_style_arc_opa(s_detail_ct_arc, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_add_event_cb(s_detail_ct_arc, detail_ct_grad_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_add_event_cb(s_detail_ct_arc, detail_dial_pressed_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_detail_ct_arc, detail_dial_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_detail_ct_arc, detail_dial_changed_cb, LV_EVENT_RELEASED, NULL);

    /* ---- 返回键 + 标题（建在圆环之后：聚焦时的光晕不会盖住标题）---- */
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

    /* ---- 中间堆栈：灯泡 / 大号百分比 / 色温文字 ---- */
    s_detail_icon = lv_obj_create(s_detail);
    lv_obj_remove_style_all(s_detail_icon);
    lv_obj_set_size(s_detail_icon, 24, 24);
    lv_obj_set_pos(s_detail_icon, DETAIL_CX - 12, 150);
    lv_obj_remove_flag(s_detail_icon, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_detail_icon, detail_bulb_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_detail_br_value = lv_label_create(s_detail);
    lv_label_set_text(s_detail_br_value, "--");
    /* 项目里已经编进来的最大号字（montserrat_28），不新增字体 */
    lv_obj_set_style_text_font(s_detail_br_value, DETAIL_PCT_FONT, 0);
    lv_obj_set_style_text_color(s_detail_br_value, COLOR_TEXT, 0);
    lv_obj_align(s_detail_br_value, LV_ALIGN_TOP_MID, 0, 180);
    lv_obj_add_flag(s_detail_br_value, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_detail_br_value, detail_focus_br_cb, LV_EVENT_CLICKED, NULL);

    s_detail_ct_value = lv_label_create(s_detail);
    lv_label_set_text(s_detail_ct_value, "--");
    lv_obj_set_style_text_font(s_detail_ct_value, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_detail_ct_value, COLOR_TEXT, 0);
    lv_obj_align(s_detail_ct_value, LV_ALIGN_TOP_MID, 0, 212);
    lv_obj_add_flag(s_detail_ct_value, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_detail_ct_value, detail_focus_ct_cb, LV_EVENT_CLICKED, NULL);

    /* 关灯：整组圆盘藏起来，只留这一行提示 */
    s_detail_off_hint = lv_label_create(s_detail);
    lv_label_set_text(s_detail_off_hint, "已关闭");
    lv_obj_set_style_text_font(s_detail_off_hint, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_detail_off_hint, COLOR_MUTED, 0);
    lv_obj_align(s_detail_off_hint, LV_ALIGN_TOP_MID, 0, 190);

    /* ---------------- 空调那一组（同一个容器，默认全藏） ----------------
       几何见文件头的 AC_* 宏：环 114/22 + 光晕；中间堆栈（图标 / 大号温度 /
       室内温度）都在半径 92 以内；底部三行按"越往下越宽的缺口楔形"排。 */
    s_ac_glow = detail_make_arc(s_detail, AC_TEMP_GLOW_R, AC_TEMP_GLOW_W, true);
    lv_arc_set_range(s_ac_glow, AC_UNIT * 16, AC_UNIT * 30);
    lv_arc_set_value(s_ac_glow, AC_UNIT * 25);

    s_ac_arc = detail_make_arc(s_detail, AC_TEMP_R, AC_TEMP_W, false);
    lv_arc_set_range(s_ac_arc, AC_UNIT * 16, AC_UNIT * 30);
    lv_arc_set_value(s_ac_arc, AC_UNIT * 25);
    lv_obj_add_event_cb(s_ac_arc, detail_dial_pressed_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_ac_arc, detail_dial_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_ac_arc, detail_dial_changed_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(s_ac_arc, detail_dial_changed_cb, LV_EVENT_PRESS_LOST, NULL);

    /* 中间：模式图标（自绘，颜色/形状由 s_ac_accent + s_ac_mode_shown 决定）*/
    s_ac_icon = lv_obj_create(s_detail);
    lv_obj_remove_style_all(s_ac_icon);
    lv_obj_set_size(s_ac_icon, AC_ICON_SIZE, AC_ICON_SIZE);
    lv_obj_set_pos(s_ac_icon, DETAIL_CX - AC_ICON_SIZE / 2, AC_ICON_Y);
    lv_obj_remove_flag(s_ac_icon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_ac_icon, ac_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    /* 中间：大号目标温度（step<1 时带一位小数）*/
    s_ac_temp_value = lv_label_create(s_detail);
    lv_label_set_text(s_ac_temp_value, "--");
    lv_obj_set_style_text_font(s_ac_temp_value, DETAIL_PCT_FONT, 0);
    lv_obj_set_style_text_color(s_ac_temp_value, COLOR_TEXT, 0);
    lv_obj_align(s_ac_temp_value, LV_ALIGN_TOP_MID, 0, AC_TEMP_Y);

    /* 中间：室内温度 */
    s_ac_room = lv_label_create(s_detail);
    lv_label_set_text(s_ac_room, "室内 --");
    lv_obj_set_style_text_font(s_ac_room, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_ac_room, COLOR_MUTED, 0);
    lv_obj_align(s_ac_room, LV_ALIGN_TOP_MID, 0, AC_ROOM_Y);

    /* 风速：点一下切到下一档（中文标签，值来自实体的 fan_modes）*/
    s_ac_fan_btn = lv_button_create(s_detail);
    lv_obj_set_size(s_ac_fan_btn, AC_FAN_W, AC_FAN_H);
    lv_obj_align(s_ac_fan_btn, LV_ALIGN_TOP_MID, 0, AC_FAN_Y);
    lv_obj_set_style_radius(s_ac_fan_btn, AC_FAN_H / 2, 0);
    lv_obj_set_style_bg_color(s_ac_fan_btn, lv_color_hex(0x24262B), 0);
    lv_obj_set_style_bg_opa(s_ac_fan_btn, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_ac_fan_btn, 1, 0);
    lv_obj_set_style_border_color(s_ac_fan_btn, lv_color_hex(0x3A3A3C), 0);
    lv_obj_set_style_pad_all(s_ac_fan_btn, 0, 0);
    lv_obj_add_event_cb(s_ac_fan_btn, ac_fan_cb, LV_EVENT_CLICKED, NULL);
    s_ac_fan_label = lv_label_create(s_ac_fan_btn);
    lv_label_set_text(s_ac_fan_label, "风速 --");
    lv_obj_set_style_text_font(s_ac_fan_label, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_ac_fan_label, COLOR_TEXT, 0);
    lv_obj_center(s_ac_fan_label);

    /* 5 个模式按钮：固定顺序 auto/cool/heat/dry/fan_only，只显示实体支持的 */
    for (int i = 0; i < AC_MODE_COUNT; i++) {
        const int x = DETAIL_CX - (AC_MODE_COUNT * AC_MODE_W + (AC_MODE_COUNT - 1) * AC_MODE_GAP) / 2
                      + i * (AC_MODE_W + AC_MODE_GAP);
        lv_obj_t *btn = lv_button_create(s_detail);
        s_ac_mode_btn[i] = btn;
        lv_obj_set_size(btn, AC_MODE_W, AC_MODE_H);
        lv_obj_set_pos(btn, x, AC_MODE_Y);
        lv_obj_set_style_radius(btn, AC_MODE_H / 2, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x24262B), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_80, 0);
        lv_obj_set_style_pad_all(btn, 0, 0);
        lv_obj_add_event_cb(btn, ac_mode_cb, LV_EVENT_CLICKED, (void *)&k_ac_modes[i]);

        lv_obj_t *lbl = lv_label_create(btn);
        s_ac_mode_lbl[i] = lbl;
        lv_label_set_text(lbl, k_ac_modes[i].label);
        lv_obj_set_style_text_font(lbl, &lv_font_cjk16, 0);
        lv_obj_set_style_text_color(lbl, COLOR_TEXT, 0);
        lv_obj_center(lbl);
    }

    /* 开关：仍在（开关逻辑一个字没改），挪到圆环下方 45°..135° 的空缺口里。
       96x46 放在 (132..228, 273..319)，两个圆环的圆头都在它两侧，互不遮挡；
       空调页那一行更挤，detail_apply_entity 会把它改成 72x32 放到 y=314。
       它建在最后 = 在最上层，角落不会被子对象的命中区抢走。 */
    s_detail_power_sw = lv_switch_create(s_detail);
    lv_obj_set_size(s_detail_power_sw, 96, 46);
    lv_obj_align(s_detail_power_sw, LV_ALIGN_TOP_MID, 0, 273);
    lv_obj_set_style_radius(s_detail_power_sw, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_detail_power_sw, lv_color_hex(0x151517), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_detail_power_sw, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_detail_power_sw, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_detail_power_sw, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_detail_power_sw, lv_color_white(), LV_PART_KNOB);
    lv_obj_add_event_cb(s_detail_power_sw, detail_power_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* 新容器：默认按"灯页"显示，空调那组先整组藏起来（要先置脏，
       否则 detail_update_ambience 会以为底色没变，新容器上还是空样式）*/
    s_detail_ac = false;
    s_ac_mode_shown[0] = '\0';
    s_detail_fg_dirty = true;
    s_sw_geom = -1;
    s_sw_on_shown = -1;
    s_ac_temp_txt[0] = '\0';
    s_ac_room_txt[0] = '\0';
    s_ac_fan_txt[0] = '\0';
    ac_hide_all(true);
    detail_apply_focus();
    detail_update_ambience();       /* 底色 / 环色 / 自绘图标都先落到一个确定状态 */

    ESP_LOGI(TAG, "详情页建好：%d 个对象（灯：2 环 + 2 光晕 + 灯泡 + 百分比 + 色温 + 已关闭；"
                  "空调：温度环 + 光晕 + 模式图标 + 大号温度 + 室内 + 风速 + 5 模式键；"
                  "共用：开关 + 标题 + 返回键）",
             detail_count_objs(s_detail));
}

void ui_home_open_detail(const char *entity_id)
{
    if (s_detail == NULL || entity_id == NULL) {
        ESP_LOGW(TAG, "open detail 失败: detail=%p entity=%s", (void *)s_detail, entity_id ? entity_id : "(null)");
        return;
    }
    ESP_LOGI(TAG, "open detail: %s", entity_id);
    snprintf(s_detail_entity, sizeof(s_detail_entity), "%s", entity_id);
    /* 先按 entity_id 前缀定页面类型：实体还没拉到（ha_client_find 返回 NULL）时，
       至少显示对的那一组控件，而不是上一页的残留。 */
    s_detail_ac = (strncmp(entity_id, "climate.", 8) == 0);
    /* 每次进页面都以默认焦点开始：灯 = 亮度环，空调 = 温度环
       （灯/空调看实体 id，不看上一次开的是哪一页）*/
    const ha_entity_t *ent = ha_client_find(s_detail_entity);
    s_detail_knob_target = detail_is_climate(ent) ? DETAIL_KNOB_TEMP : DETAIL_KNOB_BR;
    /* 新页面从干净状态开始：上一页的待发值 / 覆盖值（关页时已补发过）不再带进来 */
    s_knob_dirty = false;
    s_knob_pending_target = DETAIL_KNOB_BR;
    detail_knob_timer_park();
    s_ac_mode_local_until = 0;
    s_ac_fan_local_until = 0;
    s_ac_override_until = 0;
    s_ac_mode_shown[0] = '\0';          /* 换实体：颜色/图标必须重刷一遍 */
    s_ac_temp_txt[0] = '\0';
    s_ac_room_txt[0] = '\0';
    s_ac_fan_txt[0] = '\0';
    s_detail_fg_dirty = true;
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
        if (detail_knob_can_send(e) && s_cmd_q != NULL) {
            const uint32_t delay_ms = (uint32_t)(lv_tick_get() - s_knob_last_change);
            s_knob_inflight = true;      /* 先置位再入队，理由同 queue_pending */
            if (!detail_knob_send(s_knob_pending_target, s_knob_pending_value)) {
                s_knob_inflight = false;
            } else {
                s_knob_last_send = lv_tick_get();
                char vbuf[16];
                if (s_knob_pending_target == DETAIL_KNOB_TEMP) {
                    ac_fmt_temp(vbuf, sizeof(vbuf), s_knob_pending_value, AC_UNIT);
                } else if (s_knob_pending_target == DETAIL_KNOB_CT) {
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

/* ---------------- 空调：走和灯同一个详情页 ---------------- */

/* 空调不再单独一页：它就是 s_detail 的另一种形态（几何见文件头的 AC_* 宏）。
   这三个函数保留成别名，main.c / assistant_ui.c 的调用点不用动。 */

void ui_home_open_climate(const char *entity_id)
{
    ui_home_open_detail(entity_id);
}

void ui_home_close_climate(void)
{
    ui_home_close_detail();
}

bool ui_home_climate_open(void)
{
    /* assistant_ui.c 用它判断"界面有没有被二级页盖住"：空调页现在也是 s_detail，
       所以返回"详情页开着、并且开的是 climate.*"。 */
    return s_detail_visible && s_detail_ac;
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

    build_detail_view(scr);      /* 灯 + 空调共用的详情页 */
    s_detail_visible = false;
    s_detail_ac = false;
    s_detail_entity[0] = '\0';   /* 重建后旧实体作废：不要把上一轮的旋钮目标带进来 */
    build_chart_view(scr);
    s_chart_visible = false;

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
