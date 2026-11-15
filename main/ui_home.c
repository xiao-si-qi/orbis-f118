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
static lv_obj_t *s_detail_name;                  /* 环心最上面一行：实体名（小号灰字）*/
static lv_obj_t *s_detail_power_sw;
/* 圆形拨盘：外环=亮度，内环=色温。每个环下面压一条更宽的半透明弧当"光晕"，
   子对象按创建顺序绘制，所以光晕必须先建、环后建。 */
static lv_obj_t *s_detail_br_arc, *s_detail_br_glow;
static lv_obj_t *s_detail_ct_arc, *s_detail_ct_glow;
static lv_obj_t *s_detail_icon;                  /* 中间：自绘的灯泡 */
static lv_obj_t *s_detail_br_value;              /* 中间：大号百分比 */
static lv_obj_t *s_detail_ct_value;              /* 中间：「3500K 中性」 */
static lv_obj_t *s_detail_off_hint;              /* 关灯时露出的「已关闭」 */
/* 焦点提示（见 detail_apply_focus）：环上"当前值"小圆点 + 环心下方那行小字 */
static lv_obj_t *s_detail_marker;
static lv_obj_t *s_detail_hint;
static char s_detail_hint_shown[16];
static bool s_detail_visible;

#define DETAIL_MARKER_SIZE    10     /* 焦点环值点直径 */
#define DETAIL_HINT_Y        246     /* 「旋钮：亮度 / 色温」那一行（字墨留在内孔里）*/

/* 360x360 圆屏上的圆盘几何（圆心 / 半径 / 线宽 / 扫角）。
   返回键和顶部标题都删掉之后，上沿空了出来，外环从 114 长到 128（+14）：
   抬头离状态栏（y=8..28）还有 26px，低头圆点行在 y=318（详情页里本来也藏着），
   光晕最外 142 → 圆环整体占 y=54..338，离屏幕圆形可视区（圆心 180,180 / 半径 180）
   最近还有 37px（左右两侧），不会切边。
   LVGL 的 arc 半径是"外径"、线宽向内画，所以：
     外环（亮度）外径 128 / 宽 22 → 占 106..128，光晕 116..142（露在环外 14px）；
     内环（色温）外径 96 / 宽 14 → 占 82..96，光晕 78..102；
     两环之间留 10px（96..106）。
   中间堆栈（实体名 148..169 / 灯泡 170..194 / 百分比 196..226 / 色温 226..247）
   和焦点提示行（246..267，只有 5 个汉字宽、字墨 r<=80）都在内环以内
   （内孔半径 82 的圆）：名字那一行最靠上，y=148 处内孔宽 ~133px，
   宽度仍卡 100px + LV_LABEL_LONG_DOT——名字再长也只会"…"掉，压不到环上
   （14 个汉字的完整一行要 224px，比整个内孔的直径 164px 还宽，物理上放不下）。 */
#define DETAIL_CX            180
#define DETAIL_CY            196
#define DETAIL_NAME_Y        148     /* 实体名（cjk16）：环心最上面一行 */
#define DETAIL_NAME_W        100     /* ~6 个汉字，超出用 … ；见上面的说明 */
/* 区域焦点：以 (DETAIL_CX, DETAIL_CY) 为圆心，r <= 82px 算内孔（色温区），
   82 < r 算环带/环外（亮度区）。82 = 色温环的内沿（外径 96 - 线宽 14），
   正好就是"环心那颗孔"的半径；按在环带上时轮的还是环自己的 PRESSED 回调。 */
#define DETAIL_REGION_R       82
#define DETAIL_ANG_START     135     /* 0°=右、90°=下：135° 是左下，顺时针扫 270° */
#define DETAIL_ANG_END        45     /* = 135+270，跨 0°，缺口正好朝下 */
#define DETAIL_ANG_SWEEP     270
#define DETAIL_BR_R          128     /* 外环（亮度）外径（114→128，吃掉删掉的返回键/标题）*/
#define DETAIL_BR_W           22     /* 外环线宽（20→22）*/
#define DETAIL_CT_R           96     /* 内环（色温）外径（84→96）*/
#define DETAIL_CT_W           14     /* 内环线宽（12→14）*/
#define DETAIL_BR_GLOW_R     142     /* 亮度光晕：116..142，露在环外 14px（跟着外环一起长）*/
#define DETAIL_BR_GLOW_W      26
#define DETAIL_CT_GLOW_R     102     /* 色温光晕：78..102，环两侧各露 4 / 6px */
#define DETAIL_CT_GLOW_W      24
#define DETAIL_GRAD_STEP       4     /* 色温渐变每 4° 一段（270° 最多 68 段）*/
#define DETAIL_DIM_OPA       90      /* 没焦点的环 / 数字压到 ~35%（要一眼看出差别）*/
#define DETAIL_GLOW_OPA  LV_OPA_20   /* 聚焦时的光晕 */
/* 没填满的那圈轨道：深而不显眼。焦点环 70%，非焦点环 45%（见 detail_apply_focus）。*/
#define DETAIL_TRACK_COLOR  lv_color_hex(0x1A1A1C)
#define DETAIL_TRACK_OPA     179     /* 70% */
#define DETAIL_TRACK_DIM_OPA 115     /* 45% */
/* 中间百分比用哪个字号：montserrat_28 是工程里已经编进来、但之前没人引用（被
   --gc-sections 丢掉）的那套字；引用它会让 .rodata 多 ~37.5KB flash。
   想省 flash 就换成 &lv_font_montserrat_20（标题/卡片已经在用，0 成本）。 */
#define DETAIL_PCT_FONT  (&lv_font_montserrat_28)

/* ---- 空调详情页（和灯共用 s_detail，同一套圆盘语言）----
 *
 * 几何：圆心 / 扫角和灯页面完全一样（DETAIL_CX/CY、135°→45°），只是只有一条环：
 *   外环 = 目标温度（外径 118 / 线宽 22，光晕 106..132），范围 = 这台机器的
 *   min_temp..max_temp，每格 = 它的 target_temp_step（0.5 的机器按 0.5 走）。
 *   环值单位用 0.1°C 的整数（AC_UNIT），旋钮路径上不出现浮点，吸附没有误差。
 *
 *   为什么不是和灯页面一样的 128：下面 y=284 那排模式键顶住了环的 135°/45° 圆头。
 *   圆头 = 半径 R-11 处一个半径 11 的圆；R=128 时圆头最低点 y=290，会压进模式键
 *   （顶边 284）约 6px；R=118 时最低点 y=282.7，留 1.3px。所以空调环只长到 118
 *   （+4），灯页面长到 128（+14）——宁可两页刻度差一点，也不能让环被按键切掉。
 *
 * 360x360 圆屏、270° 环、缺口朝下 90°。缺口是一个"上窄下宽的楔形"：
 * 半径 > 96 且角度在 45°..135° 之间才没有环，所以越往下越宽。
 * 环心那一列（名字 / 图标 / 大号温度 / 室内温度 / 风速）都验算过在内孔里：
 *   实体名  y=134..155（顶边 dy=62，内孔可用 half=sqrt(96²-62²)=73 > 60）
 *   模式图标 28x28   y=156..184
 *   目标温度（28 号字）y=186..215
 *   「室内 17.0°C」    y=218..239
 *   风速按钮 92x26    y=242..268（四角距圆心 sqrt(46²+72²)=85 < 96，落在下方缺口里）
 * 底部那几行都在下方缺口（45°..135°）里，验算的是"离环圆头还有多远"：
 *   5 个模式按钮 36x24 y=284..308（环圆头最低点 y=282.7，留 1.3px）
 *   电源开关 72x32  y=314..346   （离环圆头 ~47px）
 */
#define AC_TEMP_R          118     /* 目标温度环外径（灯是 128；这里受模式键限制）*/
#define AC_TEMP_W           22
#define AC_TEMP_GLOW_R     132     /* 光晕 106..132，同样露在环外 14px */
#define AC_TEMP_GLOW_W      26
#define AC_UNIT             10     /* 弧值单位：0.1°C */
#define AC_NAME_Y          134     /* 实体名（cjk16）：环心最上面一行（比灯高 14px）*/
#define AC_NAME_W          120     /* ~7 个汉字；空调只有一条环，内孔比灯大，能多放一个字 */
#define AC_ICON_Y          156     /* 模式图标 28x28（给上面的实体名让出 16px）*/
#define AC_ICON_SIZE        28
#define AC_TEMP_Y          186     /* 中间大号目标温度（montserrat_28）*/
#define AC_ROOM_Y          218     /* 「室内 17.0°C」 */
#define AC_FAN_Y           242     /* 风速按钮 */
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
static bool s_sw_on_now;                    /* 开关此刻该显示"开"（含 6s 本地覆盖）*/
static lv_color_t s_sw_track_shown;         /* 开关轨道上次刷的颜色（避免 500ms 重刷）*/
static int8_t s_sw_style_shown = -1;        /* 开关上次刷的配色：-1=未知 0=关 1=开 */
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

/* ---- 风扇详情页（同一个 s_detail，第三种形态，和灯 / 空调共用圆盘语言）----
 *
 * 几何和空调页完全一致（圆心 (180,196)、135°→45°、单环 + 光晕），只有颜色和
 * 底部控件不同：外环 = 风速 1..100%（外径 118 / 线宽 22，光晕 106..132）。
 *
 * 为什么不是灯的 128：下面 y=284 那排 4 个预设键顶住了 R=128 的环圆头。
 * 圆头 = 半径 R-11 处一个半径 11 的圆，R=128 时最低点 y=289.7，压进预设键
 * （顶边 284）约 6px；R=118 时最低点 y=282.7，留 1.3px —— 和空调页同一个理由、
 * 同一组数字。预设键 40x22 放在 x=91..269 / y=284..306（离环 1.3px、离屏幕圆边
 * 25.8px）；摇头键 52x24 + 电源开关 62x28 并排放在 y=314..342 的缺口里，
 * 这两个键最近的角离屏幕圆边还有 6.1px（验算见 FAN_* 宏和 build 日志）。
 *
 * 环色 = 当前预设的颜色（直吹青 / 自然绿 / 循环蓝 / 睡眠灰），没有预设就青；
 * 页面底色 = 这个颜色压到很暗的一层（关掉就是纯黑），和空调页同一套做法。 */
#define FAN_R              118     /* 风速环外径（灯 128 / 空调 118，和空调一致）*/
#define FAN_W               22
#define FAN_GLOW_R         132     /* 光晕 106..132，露在环外 14px */
#define FAN_GLOW_W          26
#define FAN_ICON_SIZE       24     /* 环心：自绘风扇（轮毂 + 3 片扇叶）*/
#define FAN_ICON_Y         170
#define FAN_PCT_Y          196     /* 大号百分比（montserrat_28），和灯同一行 */
#define FAN_PRESET_Y       226     /* 当前预设（cjk16，次要文字）*/
#define FAN_MODE_Y         284     /* 4 个预设键：40 宽 + 6 间距 = 178，x 91..269 */
#define FAN_MODE_W          40
#define FAN_MODE_H          22
#define FAN_MODE_GAP         6
#define FAN_MODE_COUNT       4     /* 直吹 / 自然 / 循环 / 睡眠 */
#define FAN_OSC_W           52     /* 摇头键（在下方缺口里靠左）*/
#define FAN_OSC_H           24
#define FAN_OSC_Y          316
#define FAN_OSC_DX         (-40)   /* 相对 TOP_MID 的横向偏移：和右边开关凑成一对 */
#define FAN_SW_W            62     /* 电源开关（比空调的 72 窄，给摇头让位）*/
#define FAN_SW_H            28
#define FAN_SW_Y           314
#define FAN_SW_DX           29     /* 开关中心 x = 209，摇头中心 x = 140 */
#define FAN_PCT_STEP         5     /* 旋钮每格 5%（任务指定）*/

static lv_obj_t *s_fan_arc;                  /* 风速环（可点可拖）*/
static lv_obj_t *s_fan_glow;                 /* 同一条环的光晕 */
static lv_obj_t *s_fan_icon;                 /* 中间：自绘风扇 */
static lv_obj_t *s_fan_pct;                  /* 中间：大号百分比 */
static lv_obj_t *s_fan_preset;               /* 中间：当前预设（次要文字）*/
static lv_obj_t *s_fan_mode_btn[FAN_MODE_COUNT];   /* 4 个预设键 */
static lv_obj_t *s_fan_mode_lbl[FAN_MODE_COUNT];
static lv_obj_t *s_fan_osc_btn;              /* 摇头键 */
static lv_obj_t *s_fan_osc_lbl;
static bool s_detail_fan;                    /* 当前详情页开的是 fan.* */
static lv_color_t s_fan_accent;              /* 自绘图标读它（已过对比度处理）*/
/* 文字 / 配色缓存：500ms 轮询下没变就别再 set_text / 重绘。
   s_fan_preset_shown 初值是 "\x01"（不可能是任何预设），保证进页面后第一次
   一定会刷一遍颜色和预设键。 */
static char s_fan_preset_shown[16];
static char s_fan_preset_txt[16];
static char s_fan_pct_txt[8];
/* 本地覆盖：点了预设 / 摇头 / 转过旋钮之后先按本地值显示，HA 落定后交还（6s）*/
static char s_fan_preset_local[16];
static uint32_t s_fan_preset_local_until;
static bool s_fan_osc_local;
static uint32_t s_fan_osc_local_until;
static int s_fan_pct_override;
static uint32_t s_fan_pct_override_until;

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
static char s_detail_name_shown[48];        /* 名字行当前文字（避免 500ms 轮询重复重排）*/
/* 旋钮在灯详情页里调什么：默认亮度，点过色温环就切色温（换灯 / 重新进页时复位）。
   这个变量同时就是"哪个环有焦点"——焦点不需要第二个变量，它只是它的另一种画法。 */
#define DETAIL_KNOB_BR        0
#define DETAIL_KNOB_CT        1
#define DETAIL_KNOB_TEMP      2        /* 空调页：目标温度（圆盘只有一个环，永远是焦点）*/
#define DETAIL_KNOB_FAN       3        /* 风扇页：风速（同样只有一个环，永远是焦点）*/
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
static bool detail_device_on(const ha_entity_t *e);
static void detail_ct_range(const ha_entity_t *e, int *lo, int *hi);
static void detail_update_ambience(void);
static void detail_apply_dial_colors(void);
static void detail_apply_focus(void);
static void detail_marker_update(void);   /* 焦点环"当前值"小圆点（值一变就跟着挪）*/
static lv_color_t detail_text_fg(void);   /* 主文字颜色：亮底黑 / 暗底白 */
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
/* 风扇那一路（定义在后面，前面算底色 / 配色 / 焦点时要先能看到） */
static bool detail_is_fan(const ha_entity_t *e);
static const char *fan_preset_now(const ha_entity_t *e);
static lv_color_t fan_preset_color(const char *preset);
static lv_color_t fan_ambient_bg(const char *preset, bool on);
static void fan_apply_entity(const ha_entity_t *e);
static void fan_hide_all(bool hide);
static void fan_refresh_chips(const ha_entity_t *e, const char *preset);
static void fan_refresh_osc(const ha_entity_t *e);
static bool fan_knob_step(const ha_entity_t *e, int delta);
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

    /* 风扇：和灯 / 空调走同一个详情页（风速弧 + 预设键 + 摇头 + 电源）*/
    if (strncmp(w->entity, "fan.", 4) == 0) {
        ui_home_open_detail(w->entity);
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

/** 黑体色（整数分段线性）：2700K 偏橙、4500K 暖白、6500K 冷白。
 *  这是"理论色"，实际 LED 在墙上看到的样子要柔和得多，见下面的 kelvin_to_color。 */
static lv_color_t kelvin_blackbody(int kelvin)
{
    if (kelvin < 3000) {
        return lv_color_make(255, 170, 90);
    }
    if (kelvin < 4500) {
        int t = (kelvin - 3000) * 100 / 1500;              /* 0..100 */
        if (t > 100) {
            t = 100;
        }
        return lv_color_make(255, 170 + (245 - 170) * t / 100, 90 + (215 - 90) * t / 100);
    }
    int t = (kelvin - 4500) * 100 / 2000;                  /* 0..100（>6500 也夹住）*/
    if (t > 100) {
        t = 100;
    }
    return lv_color_make(255 - (255 - 210) * t / 100, 245, 215 + (255 - 215) * t / 100);
}

/** 往白里混的比例（0..255）：暖端混得多，冷端混得少。
 *  2700..3000K ≈ 0.42、4000K ≈ 0.20、6500K ≈ 0.15——2700K 在真实灯下是奶白偏暖，
 *  不是屏幕原来那种深橙；宁可往"不够黄"那边偏一点。
 *  3000K 以下保持同一个系数：黑体基色本身在 3000K 以下是常数，这样 G/B 随色温单调。 */
static int kelvin_desat(int kelvin)
{
    if (kelvin <= 3000) {
        return 107;                                        /* 0.42 */
    }
    if (kelvin <= 4000) {
        return 107 - (107 - 51) * (kelvin - 3000) / 1000;  /* 0.42 → 0.20 */
    }
    if (kelvin >= 6500) {
        return 38;                                         /* 0.15 */
    }
    return 51 - (51 - 38) * (kelvin - 4000) / 2500;        /* 0.20 → 0.15 */
}

/** 开尔文转"屏幕上该显示的颜色"：黑体色再按上面比例往白里混。
 *  页面底色、灯环、开关的开色、色温渐变全部走这一个函数，保证同一色温同一颜色。 */
static lv_color_t kelvin_to_color(int kelvin)
{
    if (kelvin < 1000) {
        kelvin = 1000;
    }
    if (kelvin > 12000) {
        kelvin = 12000;
    }
    const lv_color_t c = kelvin_blackbody(kelvin);
    const int f = kelvin_desat(kelvin);                    /* 0..255 */
    return lv_color_make((uint8_t)(c.red + (255 - c.red) * f / 255),
                         (uint8_t)(c.green + (255 - c.green) * f / 255),
                         (uint8_t)(c.blue + (255 - c.blue) * f / 255));
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

/* 指示色和页面底色之间至少要拉开的亮度差（0..255）：90/255 ≈ 35%，
   约等于 WCAG 3:1（底色 0.23 / 指示色 0.85 时是 3.1:1）。
   小号灰字（实体名）用低一档的 55：保持"次要文字"的观感，但也不会糊在底色里。 */
#define DETAIL_MIN_LUMA_GAP   90
#define DETAIL_MUTED_LUMA_GAP 55

/** 把颜色整体压/提到指定的感知亮度（保留色相）。*/
static lv_color_t detail_color_at_luma(lv_color_t c, int target)
{
    const int l = detail_luma(c);
    if (target <= 0) {
        return lv_color_black();
    }
    if (target >= 255) {
        return lv_color_white();
    }
    if (l <= 0) {                    /* 纯黑没有色相可留：给等亮度灰 */
        return lv_color_make((uint8_t)target, (uint8_t)target, (uint8_t)target);
    }
    if (target == l) {
        return c;
    }
    if (target < l) {                /* 往黑压：等比缩，色相不变 */
        return lv_color_make((uint8_t)(c.red * target / l),
                             (uint8_t)(c.green * target / l),
                             (uint8_t)(c.blue * target / l));
    }
    const int head = 255 - l;        /* 往白提：按比例插值到白 */
    const int t = ((target - l) * 255) / head;
    return lv_color_make((uint8_t)(c.red + (255 - c.red) * t / 255),
                         (uint8_t)(c.green + (255 - c.green) * t / 255),
                         (uint8_t)(c.blue + (255 - c.blue) * t / 255));
}

/**
 * 把"想要的灯色"变成"在当前页底色上看得到"的颜色。
 * 以前是 luma>=140 的硬阈值一刀切（亮底就 lv_color_darken(want, 110) 一步压到 43%），
 * 亮度一过阈值整圈"啪"地变暗。现在改成按底色亮度连续处理：
 *   1) 看 want 本来在底色的哪一侧（页面底色是灯色 × <=55%，所以基本是"更亮"那侧）；
 *   2) 亮度差已经 >= DETAIL_MIN_LUMA_GAP：原样用（绝大多数情况颜色完全不变）；
 *   3) 不够就把 want 朝远离底色的方向补到刚好差这个 gap，补的量随底色连续变化。
 * 同一套规则被灯环 / 灯泡 / 色温渐变 / 开关的"开"色共用，所以不会在不同控件上
 * 出现不同阈值的跳变。
 */
static lv_color_t detail_on_bg(lv_color_t want)
{
    const int lb = detail_luma(s_detail_bg_col);
    const int lw = detail_luma(want);
    const int dir = (lw > lb) ? 1 : (lw < lb) ? -1 : (lb < 128 ? 1 : -1);
    if ((lw - lb) * dir >= DETAIL_MIN_LUMA_GAP) {
        return want;
    }
    int target = lb + dir * DETAIL_MIN_LUMA_GAP;
    if (target < 0) {
        target = 0;
    }
    if (target > 255) {
        target = 255;
    }
    return detail_color_at_luma(want, target);
}

/** 小号灰字（实体名）的对比度：只要 55 的亮度差，保持"弱化"但不至于糊掉。*/
static lv_color_t detail_muted_on_bg(void)
{
    const lv_color_t c = COLOR_MUTED;            /* 0x8E8E93 */
    const int lb = detail_luma(s_detail_bg_col);
    const int lw = detail_luma(c);
    const int dir = (lw >= lb) ? 1 : -1;
    if ((lw - lb) * dir >= DETAIL_MUTED_LUMA_GAP) {
        return c;
    }
    int target = lb + dir * DETAIL_MUTED_LUMA_GAP;
    if (target < 0) {
        target = 0;
    }
    if (target > 255) {
        target = 255;
    }
    return detail_color_at_luma(c, target);
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

/**
 * 页面底色：关=纯黑；开=中性深灰 + 一点点灯色（"氛围"而不是"整页染成灯色"）。
 *
 * 以前是灯色 × 亮度系数：2700K 低亮度时 (255,205,159)×27% = (68,55,42)，
 * 在 sRGB 里就是一块棕色。现在拆成两层：
 *   1) 底色基座 = 0x0A0A0C → 0x121214（随亮度从 10 长到 18，始终中性、不偏色）；
 *   2) 灯色先去饱和 70%（往自己的亮度灰靠），再按"染色量"混进基座：
 *      染色量在 40% 亮度以下为 0，40% → 100% 线性从 0 长到 18%。
 * 所以 2700K@1% ≈ 近黑中性色，2700K@100% 只是"偏暖的深色页面"，永远不会变棕。
 */
#define DETAIL_BG_BASE_LO     10     /* 0x0A0A0C：最暗的中性底（通道差 2）*/
#define DETAIL_BG_BASE_HI     18     /* 0x121214：最亮的中性底 */
#define DETAIL_BG_TINT_FROM   40     /* 这个亮度以下完全不染色 */
#define DETAIL_BG_TINT_MAX    18     /* 满亮度最多混 18% 灯色 */
#define DETAIL_BG_TINT_KEEP   30     /* 灯色只保留 30% 彩度（去饱和 70%）*/

static lv_color_t detail_ambient_bg(void)
{
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (s_detail_fan) {
        /* 风扇页：预设色的极暗版，关掉纯黑（和空调页同一套，见 fan_ambient_bg）*/
        return fan_ambient_bg(fan_preset_now(e), detail_device_on(e));
    }
    if (s_detail_ac) {
        /* 空调页控件多，不能像灯那样把整页刷成灯色（亮底看不清小字）：
           只给一层很深的模式色，关掉就是纯黑。 */
        return ac_mode_bg(ac_state_now(e));
    }
    if (e == NULL || !detail_device_on(e)) {
        return COLOR_BG;                 /* 关灯：纯黑，不带一点底色 */
    }
    const lv_color_t c = kelvin_to_color(detail_kelvin_now(e));
    const int pct = detail_pct_now(e);

    /* 中性基座：0x0A0A0C → 0x121214（B 比 R/G 大 2，冷一点的深灰）*/
    const int base = DETAIL_BG_BASE_LO + (DETAIL_BG_BASE_HI - DETAIL_BG_BASE_LO) * pct / 100;
    int r = base;
    int g = base;
    int b = base + 2;

    int tint = 0;
    if (pct > DETAIL_BG_TINT_FROM) {
        tint = DETAIL_BG_TINT_MAX * (pct - DETAIL_BG_TINT_FROM)
               / (100 - DETAIL_BG_TINT_FROM);
    }
    if (tint > 0) {
        const int l = detail_luma(c);
        const int tr = l + (c.red - l) * DETAIL_BG_TINT_KEEP / 100;
        const int tg = l + (c.green - l) * DETAIL_BG_TINT_KEEP / 100;
        const int tb = l + (c.blue - l) * DETAIL_BG_TINT_KEEP / 100;
        r += (tr - r) * tint / 100;
        g += (tg - g) * tint / 100;
        b += (tb - b) * tint / 100;
    }
    return lv_color_make((uint8_t)r, (uint8_t)g, (uint8_t)b);
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

/** 色温渐变第 t% 的颜色：t 沿圆环从暖端(0)到冷端(100)，
 *  直接映射到实体自己的 min..max 色温，再走 kelvin_to_color + 对比度保护，
 *  这样渐变里的每个点都和"这个位置对应的色温"完全一致（同一个 helper）。*/
static lv_color_t detail_grad_color(int t, int lo, int hi)
{
    if (t < 0) {
        t = 0;
    }
    if (t > 100) {
        t = 100;
    }
    if (hi <= lo) {
        return detail_on_bg(kelvin_to_color(lo));
    }
    const int k = lo + (hi - lo) * t / 100;
    return detail_on_bg(kelvin_to_color(k));
}

/** 色温文字的档位：<3000 暖光 / <4500 中性 / <5500 自然 / 其它 冷光。*/
static const char *detail_ct_word(int k)
{
    if (k < 3000) return "暖光";
    if (k < 4500) return "中性";
    if (k < 5500) return "自然";
    return "冷光";
}

/** 中间的百分比文字（旋钮 / 拖动 / 轮询三条路径都走它）。
 *  亮度下限是 1%：0% 会被 Home Assistant 当成"关灯"，开关才是唯一的电源控制。*/
static void detail_set_br_text(int pct)
{
    if (s_detail_br_value == NULL) {
        return;
    }
    if (pct < 1) {
        pct = 1;
    }
    if (pct > 100) {
        pct = 100;
    }
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", pct);
    lv_label_set_text(s_detail_br_value, buf);
    detail_marker_update();          /* 焦点值点跟着亮度环当前值走 */
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
    detail_marker_update();          /* 焦点值点跟着色温环当前值走 */
}

/**
 * 环心最上面那一行：实体名。最多 14 个字（ha_entity_t.name 是 48 字节，
 * 最多 15 个汉字 + 结束符），超了补一个"…"；宽度再由 LV_LABEL_LONG_DOT 按内孔
 * 卡死。两道都留着：内容 cap 是"名字本来就该短"，宽度 cap 是"再长也不能压到环上"。
 */
#define DETAIL_NAME_CHARS 14

static void detail_name_line(const char *name, char *out, size_t cap)
{
    size_t n = 0;
    int chars = 0;
    while (name[n] != '\0' && chars < DETAIL_NAME_CHARS) {
        const unsigned char c = (unsigned char)name[n];
        const size_t len = (c >= 0xF0) ? 4 : (c >= 0xE0) ? 3 : (c >= 0xC0) ? 2 : 1;
        if (n + len + 4 > cap) {          /* 末尾还要放得下 "…" + 结束符 */
            break;
        }
        memcpy(out + n, name + n, len);
        n += len;
        chars++;
    }
    if (name[n] != '\0') {
        memcpy(out + n, "…", 4);           /* 3 字节 + 结束符 */
        n += 3;
    }
    out[n] = '\0';
}

/** 写/藏那一行名字：名字是空的就整行藏掉，屏幕顶部只留状态栏。*/
static void detail_set_name(const char *name)
{
    if (s_detail_name == NULL) {
        return;
    }
    if (name == NULL) {
        name = "";
    }
    /* 500ms 轮询：文字没变就别再 set_text（每次都会重新 malloc + 重排）*/
    if (strcmp(name, s_detail_name_shown) == 0) {
        return;
    }
    snprintf(s_detail_name_shown, sizeof(s_detail_name_shown), "%.47s", name);   /* 显式精度，避免 -Wformat-truncation */
    if (name[0] == '\0') {
        lv_label_set_text(s_detail_name, "");
        lv_obj_add_flag(s_detail_name, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    char line[48];
    detail_name_line(name, line, sizeof(line));
    lv_label_set_text(s_detail_name, line);
    lv_obj_remove_flag(s_detail_name, LV_OBJ_FLAG_HIDDEN);
}

/** 名字行的位置 / 宽度跟着页面走：灯 y=148 宽 100，空调 y=134 宽 120。*/
static void detail_place_name(bool ac)
{
    if (s_detail_name == NULL) {
        return;
    }
    lv_obj_set_width(s_detail_name, ac ? AC_NAME_W : DETAIL_NAME_W);
    lv_obj_align(s_detail_name, LV_ALIGN_TOP_MID, 0, ac ? AC_NAME_Y : DETAIL_NAME_Y);
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

/**
 * 详情页电源开关（灯 / 空调共用一个）的配色：
 *   关：轨道 0x3A3A3C、滑块 0x8E8E93（和圆环底色同一套灰，全页没有一处蓝）
 *   开：轨道 = 当前页强调色（灯 = 灯自己的颜色；空调 = 模式色；两者都过了
 *       detail_on_bg 对比度处理，纯白灯也不会刷成白底白开关），滑块 0xF2F2F7
 * 主题默认那圈蓝来自 LV_PART_INDICATOR + LV_STATE_CHECKED，所以两个 part 都刷；
 * 调用点跟着灯色 / 模式色一起走（detail_apply_dial_colors），不是只在点开关时刷。
 * 颜色没变就直接返回：500ms 轮询不该每帧重绘一次开关。
 */
static void detail_apply_switch_style(bool on)
{
    if (s_detail_power_sw == NULL) {
        return;
    }
    const lv_color_t track = on ? (s_detail_fan ? s_fan_accent
                                                : s_detail_ac ? s_ac_accent : s_detail_accent)
                                : lv_color_hex(0x3A3A3C);
    const int8_t shown = on ? 1 : 0;
    if (s_sw_style_shown == shown && lv_color_eq(track, s_sw_track_shown)) {
        return;
    }
    s_sw_style_shown = shown;
    s_sw_track_shown = track;

    /* 轨道：LVGL 9 的开关把"开"的底色画在 INDICATOR 上，MAIN 也一起刷，
       这样过渡动画的两端都是同一套颜色（不会闪一下主题蓝）。 */
    lv_obj_set_style_bg_color(s_detail_power_sw, track, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_detail_power_sw, track, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_detail_power_sw, track,
                              (lv_style_selector_t)(LV_PART_INDICATOR | LV_STATE_CHECKED));
    /* 滑块：关 = 中灰、开 = 近白 */
    lv_obj_set_style_bg_color(s_detail_power_sw,
                              on ? lv_color_hex(0xF2F2F7) : lv_color_hex(0x8E8E93),
                              LV_PART_KNOB);
}

/** 环 / 光晕 / 灯泡的颜色：先按底色做对比度处理，再刷到样式上。*/
static void detail_apply_dial_colors(void)
{
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    if (s_detail_fan) {
        /* ---- 风扇：只有一条环，颜色 = 当前预设 ----
           预设没变就不碰样式（500ms 轮询不该每帧重绘）；换预设 / 换实体才刷。 */
        if (e == NULL) {
            return;
        }
        const char *preset = fan_preset_now(e);
        s_fan_accent = detail_on_bg(fan_preset_color(preset));
        /* 开关的"开"色 = 预设色：放在判重之前，颜色变了能跟着刷 */
        detail_apply_switch_style(s_sw_on_now);
        if (strcmp(preset, s_fan_preset_shown) == 0) {
            return;
        }
        snprintf(s_fan_preset_shown, sizeof(s_fan_preset_shown), "%.15s", preset);
        if (s_fan_arc) {
            lv_obj_set_style_arc_color(s_fan_arc, s_fan_accent, LV_PART_INDICATOR);
        }
        if (s_fan_glow) {
            lv_obj_set_style_arc_color(s_fan_glow, s_fan_accent, LV_PART_INDICATOR);
        }
        if (s_fan_icon) {
            lv_obj_invalidate(s_fan_icon);       /* 自绘图标：颜色变了得手动失效 */
        }
        /* 预设键的高亮 / 显示哪些键跟着预设走；能力（preset_modes）每台不同，
           所以和"预设真的变了"一起刷，500ms 轮询不会再把 4 个键重绘一遍。 */
        fan_refresh_chips(e, preset);
        fan_refresh_osc(e);                      /* 摇头的开色跟着预设色走 */
        return;
    }
    if (s_detail_ac) {
        /* ---- 空调：只有一条环，颜色 = 当前 hvac 模式 ----
           模式没变就不碰样式（500ms 轮询不该每帧重绘），只有换模式 / 换实体才刷。 */
        if (e == NULL) {
            return;
        }
        const char *st = ac_state_now(e);
        s_ac_accent = detail_on_bg(ac_mode_color(st));
        /* 开关的"开"色 = 模式色：放在模式判重之前，颜色变了能跟着刷 */
        detail_apply_switch_style(s_sw_on_now);
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
    const bool on = detail_device_on(e);

    /* 外环 = 亮度：灯的当前颜色；不知道色温（只有亮度）的灯用强调蓝 */
    s_detail_accent = detail_on_bg(on ? detail_light_base() : COLOR_ACCENT);
    lv_obj_set_style_arc_color(s_detail_br_arc, s_detail_accent, LV_PART_INDICATOR);
    if (s_detail_br_glow) {
        lv_obj_set_style_arc_color(s_detail_br_glow, s_detail_accent, LV_PART_INDICATOR);
    }
    /* 开关的"开"色 = 灯色（和亮度环同色）：色温/亮度变了跟着刷 */
    detail_apply_switch_style(s_sw_on_now);
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

/** 环是不是正显示着（对象在、没被藏）。*/
static bool detail_ring_visible(lv_obj_t *ring)
{
    return ring != NULL && !lv_obj_is_hidden(ring);
}

/** 环心下面那行小字：「旋钮：亮度 / 旋钮：色温」。text = NULL 就藏掉。*/
static void detail_set_hint(const char *text)
{
    if (s_detail_hint == NULL) {
        return;
    }
    if (text == NULL || text[0] == '\0') {
        lv_obj_add_flag(s_detail_hint, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (strcmp(text, s_detail_hint_shown) != 0) {    /* 文字没变就别重排 */
        snprintf(s_detail_hint_shown, sizeof(s_detail_hint_shown), "%.15s", text);
        lv_label_set_text(s_detail_hint, text);
    }
    lv_obj_set_style_text_color(s_detail_hint, detail_muted_on_bg(), 0);
    lv_obj_remove_flag(s_detail_hint, LV_OBJ_FLAG_HIDDEN);
}

/**
 * 焦点环"当前值"位置的小圆点：角度 = 值在 [min,max] 里的比例 × 270°（135° 起），
 * 半径取该环描边的中线。纯整数三角（lv_trigo_*），不建定时器、不做动画，
 * 只在值 / 焦点变化时挪一下；环被藏起来（关灯 / 切页 / 空调关机）就一起藏掉。
 * 只碰静态对象指针（每次 build 重新赋值），没有跨页残留的引用。
 */
static void detail_marker_update(void)
{
    if (s_detail_marker == NULL) {
        return;
    }

    lv_obj_t *ring = NULL;
    int radius = 0;
    lv_color_t col = COLOR_TEXT;

    if (s_detail_fan) {
        const ha_entity_t *e = ha_client_find(s_detail_entity);
        ring = detail_device_on(e) ? s_fan_arc : NULL;   /* 关着：环是暗的，不露值点 */
        radius = FAN_R - FAN_W / 2;
        col = s_fan_accent;
    } else if (s_detail_ac) {
        const ha_entity_t *e = ha_client_find(s_detail_entity);
        const bool on = (e != NULL) && !ac_state_is_off(ac_state_now(e));
        ring = on ? s_ac_arc : NULL;          /* 关机：环是暗的，不露值点 */
        radius = AC_TEMP_R - AC_TEMP_W / 2;
        col = s_ac_accent;
    } else {
        const ha_entity_t *e = ha_client_find(s_detail_entity);
        const bool br = (s_detail_knob_target != DETAIL_KNOB_CT);
        ring = br ? s_detail_br_arc : s_detail_ct_arc;
        radius = br ? (DETAIL_BR_R - DETAIL_BR_W / 2) : (DETAIL_CT_R - DETAIL_CT_W / 2);
        col = br ? s_detail_accent : detail_on_bg(kelvin_to_color(detail_kelvin_now(e)));
        if (!detail_device_on(e)) {
            ring = NULL;                      /* 关灯：整组圆盘是藏着的 */
        }
    }

    if (!detail_ring_visible(ring)) {
        lv_obj_add_flag(s_detail_marker, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    const int32_t vmin = lv_arc_get_min_value(ring);
    const int32_t vmax = lv_arc_get_max_value(ring);
    const int32_t v = lv_arc_get_value(ring);
    int t = 0;                                /* 0..1000 */
    if (vmax > vmin) {
        t = (int)(((int64_t)(v - vmin) * 1000) / (vmax - vmin));
        if (t < 0) {
            t = 0;
        }
        if (t > 1000) {
            t = 1000;
        }
    }
    const int deg = (DETAIL_ANG_START + DETAIL_ANG_SWEEP * t / 1000) % 360;
    const int mx = DETAIL_CX + (int)(((int32_t)radius * lv_trigo_cos((int16_t)deg)) >> LV_TRIGO_SHIFT);
    const int my = DETAIL_CY + (int)(((int32_t)radius * lv_trigo_sin((int16_t)deg)) >> LV_TRIGO_SHIFT);
    lv_obj_set_pos(s_detail_marker, mx - DETAIL_MARKER_SIZE / 2, my - DETAIL_MARKER_SIZE / 2);
    lv_obj_set_style_bg_color(s_detail_marker, col, 0);
    lv_obj_remove_flag(s_detail_marker, LV_OBJ_FLAG_HIDDEN);
}

/**
 * 焦点：让"旋钮现在调的是哪个环"一眼就能看出来。四路提示全在这一个函数里刷，
 * 所以永远不会互相矛盾：
 *   1) 焦点环指示灯 100% + 描边加粗 2px + 光晕；另一个环 ~35%、细描边、光晕关；
 *   2) 焦点环"当前值"位置一颗 10px 小圆点（跟值走，见 detail_marker_update）；
 *   3) 环心下面一行小字：旋钮：亮度 / 旋钮：色温（空调页单环，不出）；
 *   4) 焦点那个数字全亮、另一个压暗并转成灰色。
 * 焦点没有独立变量——s_detail_knob_target 既是旋钮目标，也是焦点。
 */
static void detail_apply_focus(void)
{
    const lv_color_t fg = detail_text_fg();
    const lv_color_t muted = detail_muted_on_bg();

    if (s_detail_fan) {
        /* 风扇也只有一个环：它永远是焦点。开着才有光晕 / 值点 / 提示；
           关掉时环压暗、并且**不可点**——fan.set_percentage 会顺带开机，
           而电源只归开关管（视线里那圈刻度还在，只是拨不动）。 */
        const ha_entity_t *e = ha_client_find(s_detail_entity);
        const bool on = detail_device_on(e);
        if (s_fan_arc) {
            lv_obj_set_style_arc_width(s_fan_arc, FAN_W + 2, LV_PART_INDICATOR);
            lv_obj_set_style_arc_opa(s_fan_arc, on ? LV_OPA_COVER : DETAIL_DIM_OPA,
                                     LV_PART_INDICATOR);
            /* 轨道（没填满那圈）：焦点环亮一档，关机时再暗一档 */
            lv_obj_set_style_arc_opa(s_fan_arc, on ? DETAIL_TRACK_OPA : DETAIL_TRACK_DIM_OPA,
                                     LV_PART_MAIN);
            lv_obj_set_clickable(s_fan_arc, on);
        }
        if (s_fan_glow) {
            lv_obj_set_style_arc_opa(s_fan_glow, on ? DETAIL_GLOW_OPA : LV_OPA_TRANSP,
                                     LV_PART_INDICATOR);
        }
        if (s_fan_pct) {
            lv_obj_set_style_text_opa(s_fan_pct, on ? LV_OPA_COVER : DETAIL_DIM_OPA, 0);
            lv_obj_set_style_text_color(s_fan_pct, on ? fg : muted, 0);
        }
        if (s_fan_preset) {
            lv_obj_set_style_text_color(s_fan_preset, detail_muted_on_bg(), 0);
        }
        detail_set_hint(on ? "旋钮：风速" : NULL);
        detail_marker_update();
        return;
    }

    if (s_detail_ac) {
        /* 空调只有一个环：它永远是焦点。开着才有光晕，关掉连光晕都收掉。 */
        const ha_entity_t *e = ha_client_find(s_detail_entity);
        const bool on = (e != NULL) && !ac_state_is_off(ac_state_now(e));
        if (s_ac_arc) {
            lv_obj_set_style_arc_width(s_ac_arc, AC_TEMP_W + 2, LV_PART_INDICATOR);
            lv_obj_set_style_arc_opa(s_ac_arc, on ? LV_OPA_COVER : DETAIL_DIM_OPA,
                                     LV_PART_INDICATOR);
            /* 轨道（没填满那圈）：焦点环亮一档，关机时再暗一档 */
            lv_obj_set_style_arc_opa(s_ac_arc, on ? DETAIL_TRACK_OPA : DETAIL_TRACK_DIM_OPA,
                                     LV_PART_MAIN);
        }
        if (s_ac_glow) {
            lv_obj_set_style_arc_opa(s_ac_glow, on ? DETAIL_GLOW_OPA : LV_OPA_TRANSP,
                                     LV_PART_INDICATOR);
        }
        if (s_ac_temp_value) {
            lv_obj_set_style_text_opa(s_ac_temp_value, on ? LV_OPA_COVER : DETAIL_DIM_OPA, 0);
            lv_obj_set_style_text_color(s_ac_temp_value, on ? fg : muted, 0);
        }
        detail_set_hint(NULL);            /* 单环：没有"选哪个"的问题 */
        detail_marker_update();
        return;
    }

    const bool br = (s_detail_knob_target == DETAIL_KNOB_BR);
    if (s_detail_br_arc) {
        lv_obj_set_style_arc_width(s_detail_br_arc, br ? DETAIL_BR_W + 2 : DETAIL_BR_W,
                                   LV_PART_INDICATOR);
        lv_obj_set_style_arc_opa(s_detail_br_arc, br ? LV_OPA_COVER : DETAIL_DIM_OPA,
                                 LV_PART_INDICATOR);
        /* 轨道：只有焦点环能看到一圈"刻度"（70%），另一个压到 45% */
        lv_obj_set_style_arc_opa(s_detail_br_arc, br ? DETAIL_TRACK_OPA : DETAIL_TRACK_DIM_OPA,
                                 LV_PART_MAIN);
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
        /* 轨道同理，只是焦点方向相反 */
        lv_obj_set_style_arc_opa(s_detail_ct_arc, br ? DETAIL_TRACK_DIM_OPA : DETAIL_TRACK_OPA,
                                 LV_PART_MAIN);
        /* 色温环的渐变是自绘的：宽度/透明度都在回调里按焦点算 → 重画即可 */
        lv_obj_invalidate(s_detail_ct_arc);
    }
    if (s_detail_br_value) {
        lv_obj_set_style_text_opa(s_detail_br_value, br ? LV_OPA_COVER : DETAIL_DIM_OPA, 0);
        lv_obj_set_style_text_color(s_detail_br_value, br ? fg : muted, 0);
    }
    if (s_detail_ct_value) {
        lv_obj_set_style_text_opa(s_detail_ct_value, br ? DETAIL_DIM_OPA : LV_OPA_COVER, 0);
        lv_obj_set_style_text_color(s_detail_ct_value, br ? muted : fg, 0);
    }

    /* 小字提示 + 值点：只有灯开着、焦点环真的在屏幕上才露出来 */
    const bool on = detail_device_on(ha_client_find(s_detail_entity));
    const bool ring_on_screen = detail_ring_visible(br ? s_detail_br_arc : s_detail_ct_arc);
    if (on && ring_on_screen) {
        detail_set_hint(br ? "旋钮：亮度" : "旋钮：色温");
    } else {
        detail_set_hint(NULL);
    }
    detail_marker_update();
}

/**
 * 色温环的自绘渐变：从起点画到当前值，沿角度由暖到冷逐段画。
 * LVGL 9.6 的 lv_arc 只支持纯色指示灯，所以渐变只能自己按角度分段画；
 * 角步长 4°，270° 最多 68 段，红色那段圆弧本来就只有十几个像素，足够细。
 *
 * 注意：扫角必须自己按"值 / 范围"算，不能读 lv_arc_get_angle_end()。
 * LVGL 的 arc 会把角度归一化到 0..360（lv_arc_set_angles 里 while(end > 360) end -= 360），
 * 所以值一旦越过 83.3% 的位置（2000..6500 的机器约 5750K，某些 2xxx..5xxx 的机器
 * 正好在 5200K 附近）终点角就回绕成 45°，和起点 135° 一比被当成"空环"直接 return，
 * 整段渐变色消失——这就是"色温大于 ~5200 圈上颜色就没了"的原因。
 * 这里用 135° + 270° × (v-min)/(max-min) 得到 135..405 的"未回绕"角度，
 * 画的时候照常交给 lv_draw_arc（它内部会做 >=360 的归一化）。
 */
static void detail_ct_grad_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    if (obj == NULL || layer == NULL) {
        return;
    }

    const int32_t v = (int32_t)lv_arc_get_value(obj);
    const int32_t vmin = (int32_t)lv_arc_get_min_value(obj);
    const int32_t vmax = (int32_t)lv_arc_get_max_value(obj);
    if (vmax <= vmin || v <= vmin) {
        return;                       /* 值为最小：只剩底环 */
    }
    const int a0 = DETAIL_ANG_START;
    int a1 = a0 + (int)(((int64_t)(v - vmin) * DETAIL_ANG_SWEEP) / (vmax - vmin));   /* 135..405 */

    /* 渐变的暖/冷端 = 这台机器自己的色温范围（只查一次，别在 68 段里反复查实体）*/
    int klo = 2000, khi = 6500;
    detail_ct_range(ha_client_find(s_detail_entity), &klo, &khi);

    lv_area_t area;
    lv_obj_get_coords(obj, &area);
    const int w = area.x2 - area.x1 + 1;
    const int h = area.y2 - area.y1 + 1;
    const int r = (w < h ? w : h) / 2;

    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.radius = (uint16_t)r;
    d.center.x = area.x1 + r;
    d.center.y = area.y1 + r;
    d.opa = (s_detail_knob_target == DETAIL_KNOB_CT) ? LV_OPA_COVER : DETAIL_DIM_OPA;
    /* 焦点环的指示灯加粗 2px（和亮度环的规则一致），没焦点时保持细描边 */
    d.width = DETAIL_CT_W + ((s_detail_knob_target == DETAIL_KNOB_CT) ? 2 : 0);

    for (int seg = a0; seg < a1; seg += DETAIL_GRAD_STEP) {
        int seg_end = seg + DETAIL_GRAD_STEP;
        if (seg_end > a1) {
            seg_end = a1;
        }
        d.color = detail_grad_color((seg - a0) * 100 / DETAIL_ANG_SWEEP, klo, khi);
        d.start_angle = seg;
        d.end_angle = seg_end;
        /* 只有两端做圆头：每段都圆头会画成一串珠子 */
        d.rounded = (seg == a0) || (seg_end == a1);
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

/** 主文字颜色：亮底黑字、暗底白字（和 detail_on_bg 用同一套亮度判据）。*/
static lv_color_t detail_text_fg(void)
{
    return (detail_luma(s_detail_bg_col) >= 150) ? lv_color_black() : lv_color_white();
}

/** 根据背景亮度切换文字颜色，保证可读。 */
static void detail_apply_foreground(bool dark_text)
{
    const lv_color_t fg = dark_text ? lv_color_black() : lv_color_white();
    const lv_color_t muted = dark_text ? lv_color_hex(0x3A3A3C) : COLOR_MUTED;
    /* 实体名是小灰字：过一遍"弱化色"的对比度保护（见 detail_muted_on_bg），
       不然底色一亮它就糊进背景里。 */
    if (s_detail_name) lv_obj_set_style_text_color(s_detail_name, detail_muted_on_bg(), 0);
    if (s_detail_off_hint) lv_obj_set_style_text_color(s_detail_off_hint, muted, 0);
    /* 两个数值文字的颜色由 detail_apply_focus 按焦点刷（焦点那个亮、另一个灰），
       这里不抢，否则每次底色刷新都会把焦点提示盖掉。 */
    /* 空调页那组（和灯同一容器，只有一组可见）*/
    if (s_ac_temp_value)   lv_obj_set_style_text_color(s_ac_temp_value, fg, 0);
    if (s_ac_room)         lv_obj_set_style_text_color(s_ac_room, muted, 0);
    if (s_ac_fan_label)    lv_obj_set_style_text_color(s_ac_fan_label, fg, 0);
    if (s_fan_preset)      lv_obj_set_style_text_color(s_fan_preset, detail_muted_on_bg(), 0);
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
        detail_apply_foreground(detail_luma(s_detail_bg_col) >= 150);  /* true = 深色文字 */
    }
    detail_apply_dial_colors();                                    /* 环 / 光晕 / 灯泡跟着底色 */
}

static void detail_anim_done_cb(lv_anim_t *a);
static void detail_close_done_cb(lv_anim_t *a);

static void detail_apply_entity(void)
{
    const ha_entity_t *e = ha_client_find(s_detail_entity);
    /* 名字行先摆好：实体还没拉到时 s_detail_ac 已经由 ui_home_open_detail 按
       实体 id 前缀定过，位置照样是对的；名字是空的就整行藏掉，不留上一页的残字。 */
    if (e != NULL) {
        s_detail_ac = detail_is_climate(e);
        s_detail_fan = detail_is_fan(e);
    }
    detail_place_name(s_detail_ac);
    detail_set_name(e != NULL ? e->name : NULL);
    if (e == NULL) {
        return;
    }
    /* 同一个 s_detail 容器，两种页面：灯（亮度/色温两条环）和空调（温度一条环）。
       s_detail_ac 是"现在开的是哪一种"的唯一真相，旋钮/焦点/补发/配色全看它。 */
    ESP_LOGD(TAG, "detail attrs: %s br=%d tint=%d k=%d range=%d-%d pct=%d preset=%s osc=%d state=%s",
             s_detail_ac ? "climate" : (s_detail_fan ? "fan" : "light"),
             e->has_brightness, e->has_tint, e->color_temp_k, e->min_k, e->max_k,
             e->percentage, e->preset_mode, (int)e->oscillating, e->state);

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
    /* 记住开关此刻该显示的状态：开关配色（detail_apply_switch_style）在
       detail_apply_dial_colors 里跟着灯色/模式色一起刷，要读它。 */
    s_sw_on_now = on;
    if (s_detail_power_sw != NULL) {
        /* 底部那一行一页比一页挤：灯 96x46 / 空调 72x32 / 风扇 62x28（还要给摇头让位）*/
        const int geom = s_detail_ac ? 1 : (s_detail_fan ? 2 : 0);
        if (s_sw_geom != geom) {
            s_sw_geom = geom;
            if (geom == 1) {
                lv_obj_set_size(s_detail_power_sw, AC_SW_W, AC_SW_H);
                lv_obj_align(s_detail_power_sw, LV_ALIGN_TOP_MID, 0, AC_SW_Y);
            } else if (geom == 2) {
                lv_obj_set_size(s_detail_power_sw, FAN_SW_W, FAN_SW_H);
                lv_obj_align(s_detail_power_sw, LV_ALIGN_TOP_MID, FAN_SW_DX, FAN_SW_Y);
            } else {
                lv_obj_set_size(s_detail_power_sw, 96, 46);
                lv_obj_align(s_detail_power_sw, LV_ALIGN_TOP_MID, 0, 273);
            }
        }
        /* CHECKED 状态只在真的变了时切；配色走 detail_apply_switch_style
           （那里自己判重，所以颜色变化不用等下一次开关动作）。 */
        const int8_t shown = on ? 1 : 0;
        if (s_sw_on_shown != shown) {
            s_sw_on_shown = shown;
            if (on) {
                lv_obj_add_state(s_detail_power_sw, LV_STATE_CHECKED);
            } else {
                lv_obj_remove_state(s_detail_power_sw, LV_STATE_CHECKED);
            }
            detail_apply_switch_style(on);
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

    if (s_detail_fan) {
        /* 风扇：灯 / 空调那两组都藏起来，这一组显示出来。顺序同空调：
           先显示整组，再算底色 / 配色（里面会按 preset_modes 藏掉不支持的预设键） */
        detail_hide_light(true);
        ac_hide_all(true);
        fan_hide_all(false);
        detail_update_ambience();
        fan_apply_entity(e);
        detail_apply_focus();                 /* 开着才有光晕 / 值点 / 提示 */
        return;
    }

    ac_hide_all(true);                        /* 空调那组藏起来 */
    fan_hide_all(true);                       /* 风扇那组藏起来 */
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
        if (pct < 1) {
            pct = 1;                          /* 亮度环的下限是 1%：0% 会被 HA 当成关灯 */
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
        /* 换灯 / 脏覆盖值：色温必须夹进这台机器自己的范围，绝不把越界值塞进 arc */
        if (k < lo) {
            k = lo;
        }
        if (k > hi) {
            k = hi;
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
    if (strcmp(buf, s_ac_temp_txt) != 0) {
        snprintf(s_ac_temp_txt, sizeof(s_ac_temp_txt), "%s", buf);
        lv_label_set_text(s_ac_temp_value, buf);
    }
    detail_marker_update();   /* 文字没变（四舍五入到同一步）时值点也要跟上 */
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
/* 风扇详情页：和灯 / 空调共用 s_detail 与同一套圆盘（外环 = 风速）             */
/* ------------------------------------------------------------------------- */

/* 4 个预设的固定顺序：raw = 发给 HA 的原样字符串（和这台风扇 preset_modes 里的
   写法一致），label = 屏幕上两个字的简称，rgb = 这个预设的环色。
   都是静态常量，按钮建一次就够，不占实体内存，也不用每次开页重建。 */
typedef struct {
    const char *raw;
    const char *label;
    uint32_t rgb;
} fan_preset_def_t;

static const fan_preset_def_t k_fan_presets[FAN_MODE_COUNT] = {
    {"直吹风", "直吹", 0x2DD4BF},   /* 青：直吹 */
    {"自然风", "自然", 0x30D158},   /* 绿：自然 */
    {"循环风", "循环", 0x3B82F6},   /* 蓝：循环 */
    {"睡眠",   "睡眠", 0x8E8E93},   /* 灰：睡眠 */
};

static bool detail_is_fan(const ha_entity_t *e)
{
    return e != NULL && strncmp(e->entity_id, "fan.", 4) == 0;
}

/** 当前该按哪个预设显示：本地覆盖优先（点了预设先变色），HA 跟上就交还。 */
static const char *fan_preset_now(const ha_entity_t *e)
{
    if (e == NULL) {
        return "";
    }
    if (s_fan_preset_local_until > lv_tick_get()) {
        if (strcmp(s_fan_preset_local, e->preset_mode) == 0) {
            s_fan_preset_local_until = 0;     /* HA 已跟上 */
        } else {
            return s_fan_preset_local;
        }
    }
    return e->preset_mode;
}

/** 预设 -> 环色（不认识的预设按直吹青处理）。 */
static lv_color_t fan_preset_color(const char *preset)
{
    if (preset != NULL) {
        for (int i = 0; i < FAN_MODE_COUNT; i++) {
            if (strcmp(preset, k_fan_presets[i].raw) == 0) {
                return lv_color_hex(k_fan_presets[i].rgb);
            }
        }
    }
    return lv_color_hex(0x2DD4BF);
}

/**
 * 风扇页底色：预设色压到很暗的一层，关掉纯黑（和空调页 ac_mode_bg 同一套做法）。
 * 不把整页染成预设色：底色一亮，中间的小字和预设键的对比度就保不住了。
 * 青 -> (0x07,0x21,0x1E)、绿 -> (0x07,0x20,0x0D)、蓝 -> (0x09,0x14,0x26)、
 * 灰 -> (0x16,0x16,0x17)，感知亮度都在 30 以下，白字照样拉得开。
 */
static lv_color_t fan_ambient_bg(const char *preset, bool on)
{
    if (!on) {
        return COLOR_BG;
    }
    const lv_color_t c = fan_preset_color(preset);
    return lv_color_make((uint8_t)(c.red * 40 / 255),
                         (uint8_t)(c.green * 40 / 255),
                         (uint8_t)(c.blue * 40 / 255));
}

/** 当前该显示 / 该用的风速百分比：本地覆盖优先，其次实体值，最后按 100%。
 *  下限 1%：0% 会被 Home Assistant 当成"关"，电源只归开关管（同灯的亮度）。 */
static int fan_pct_now(const ha_entity_t *e)
{
    int pct = 100;
    if (s_fan_pct_override_until > lv_tick_get()) {
        pct = s_fan_pct_override;
    } else if (e != NULL && e->has_percentage) {
        pct = e->percentage;
    }
    if (pct < 1) {
        pct = 1;
    }
    if (pct > 100) {
        pct = 100;
    }
    return pct;
}

/** 中间的大号百分比（旋钮 / 拖动 / 轮询三条路径都走它）。 */
static void fan_set_pct_text(int pct)
{
    if (s_fan_pct == NULL) {
        return;
    }
    if (pct < 1) {
        pct = 1;
    }
    if (pct > 100) {
        pct = 100;
    }
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", pct);
    if (strcmp(buf, s_fan_pct_txt) != 0) {
        snprintf(s_fan_pct_txt, sizeof(s_fan_pct_txt), "%s", buf);
        lv_label_set_text(s_fan_pct, buf);
    }
    detail_marker_update();          /* 焦点值点跟着风速环当前值走 */
}

/** 中间第二行：当前预设（原样中文，如「自然风」）；没有就 --。 */
static void fan_set_preset_text(const char *preset)
{
    if (s_fan_preset == NULL) {
        return;
    }
    const char *txt = (preset != NULL && preset[0] != '\0') ? preset : "--";
    if (strcmp(txt, s_fan_preset_txt) == 0) {
        return;
    }
    snprintf(s_fan_preset_txt, sizeof(s_fan_preset_txt), "%.15s", txt);
    lv_label_set_text(s_fan_preset, txt);
}

/** 预设键高亮：当前预设用该预设的颜色填充，其余保持深灰。
 *  实体 preset_modes 里没有的键先藏掉（还没拉到列表时先全显示，别让整行消失）。 */
static void fan_refresh_chips(const ha_entity_t *e, const char *preset)
{
    for (int i = 0; i < FAN_MODE_COUNT; i++) {
        lv_obj_t *btn = s_fan_mode_btn[i];
        lv_obj_t *lbl = s_fan_mode_lbl[i];
        if (btn == NULL) {
            continue;
        }
        const bool active = (preset != NULL) && (preset[0] != '\0') &&
                            (strcmp(preset, k_fan_presets[i].raw) == 0);
        const lv_color_t fill = active ? detail_on_bg(lv_color_hex(k_fan_presets[i].rgb))
                                       : lv_color_hex(0x24262B);
        lv_obj_set_style_bg_color(btn, fill, 0);
        lv_obj_set_style_bg_opa(btn, active ? LV_OPA_COVER : LV_OPA_80, 0);
        if (lbl != NULL) {
            lv_obj_set_style_text_color(lbl, active ? (detail_luma(fill) >= 140 ? lv_color_black()
                                                                                : lv_color_white())
                                                    : COLOR_TEXT, 0);
        }
        bool supported = (e == NULL) || (e->preset_mode_count == 0);
        if (e != NULL) {
            for (int k = 0; k < e->preset_mode_count && k < HA_FAN_PRESET_MAX; k++) {
                if (strcmp(e->preset_modes[k], k_fan_presets[i].raw) == 0) {
                    supported = true;
                }
            }
        }
        detail_set_hidden(btn, !supported);
    }
}

/** 摇头键：反映实体 oscillating（含 6s 本地覆盖）。开 = 预设色填充，关 = 深灰。 */
static void fan_refresh_osc(const ha_entity_t *e)
{
    if (s_fan_osc_btn == NULL) {
        return;
    }
    bool osc = (e != NULL) && e->oscillating;
    if (s_fan_osc_local_until > lv_tick_get()) {
        if (e != NULL && e->oscillating == s_fan_osc_local) {
            s_fan_osc_local_until = 0;            /* HA 已跟上 */
        } else {
            osc = s_fan_osc_local;
        }
    }
    const lv_color_t fill = osc ? s_fan_accent : lv_color_hex(0x24262B);
    lv_obj_set_style_bg_color(s_fan_osc_btn, fill, 0);
    lv_obj_set_style_bg_opa(s_fan_osc_btn, osc ? LV_OPA_COVER : LV_OPA_80, 0);
    lv_obj_set_style_border_color(s_fan_osc_btn,
                                  osc ? fill : lv_color_hex(0x3A3A3C), 0);
    if (s_fan_osc_lbl != NULL) {
        lv_obj_set_style_text_color(s_fan_osc_lbl,
                                    osc ? (detail_luma(fill) >= 140 ? lv_color_black()
                                                                    : lv_color_white())
                                        : COLOR_TEXT, 0);
    }
}

/** 把实体状态刷到风扇页的所有控件上（500ms 轮询也走这里）。 */
static void fan_apply_entity(const ha_entity_t *e)
{
    const int pct = fan_pct_now(e);
    if (s_fan_arc != NULL) {
        lv_arc_set_value(s_fan_arc, pct);
    }
    if (s_fan_glow != NULL) {
        lv_arc_set_value(s_fan_glow, pct);
    }
    fan_set_pct_text(pct);
    fan_set_preset_text(fan_preset_now(e));
    fan_refresh_osc(e);
    /* 环一直显示（关着只是压暗、不可点，刻度还在），和空调页一致。 */
}

/** 风扇那组控件全部显示 / 隐藏（切到灯 / 空调页时整组藏起来）。
 *  显示时不在这里放预设键：哪些键该显示由 fan_refresh_chips 按 preset_modes
 *  决定，否则每 500ms 会把不支持的键又放出来。 */
static void fan_hide_all(bool hide)
{
    detail_set_hidden(s_fan_arc, hide);
    detail_set_hidden(s_fan_glow, hide);
    detail_set_hidden(s_fan_icon, hide);
    detail_set_hidden(s_fan_pct, hide);
    detail_set_hidden(s_fan_preset, hide);
    detail_set_hidden(s_fan_osc_btn, hide);
    if (hide) {
        for (int i = 0; i < FAN_MODE_COUNT; i++) {
            detail_set_hidden(s_fan_mode_btn[i], true);
        }
    }
}

/** 圆盘中间那台风扇：轮毂 + 3 片扇叶（0° / 120° / 240°）+ 一道护网细弧。 */
static void fan_icon_draw_cb(lv_event_t *e)
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
    const lv_color_t c = s_fan_accent;

    ac_dr_arc(layer, cx, cy, 10, 2, 0, 360, c);      /* 护网 */
    for (int i = 0; i < 12; i += 4) {                /* 0° / 120° / 240°（每 30° 一格）*/
        int x1, y1, x2, y2;
        ac_dir_point(cx, cy, i, 1, &x1, &y1);
        ac_dir_point(cx, cy, i, 7, &x2, &y2);
        ac_dr_line(layer, x1, y1, x2, y2, 4, c);
        /* 每片叶尖一小段弧，看起来是"弯"的，不像三根直棍 */
        ac_dr_arc(layer, cx, cy, 7, 3, i * 30 - 25, i * 30 + 5, c);
    }
    ac_dr_dot(layer, cx, cy, 3, c);                  /* 轮毂 */
}

/** 旋钮在风扇页上调风速：每格 5%（任务指定），值域 1..100。 */
static bool fan_knob_step(const ha_entity_t *e, int delta)
{
    if (s_fan_arc == NULL || e == NULL) {
        return false;
    }
    if (!detail_device_on(e)) {
        return false;                     /* 关着：旋钮什么也不做（电源只归开关） */
    }
    const int cur = (int)lv_arc_get_value(s_fan_arc);
    int v = cur + delta * FAN_PCT_STEP;
    if (v < 1) {
        v = 1;
    }
    if (v > 100) {
        v = 100;
    }
    if (v == cur) {
        return true;                      /* 已经到头：吃掉这一格，不发命令 */
    }
    /* 1) 立刻更新界面（环 + 光晕 + 大号百分比），不等网络 */
    lv_arc_set_value(s_fan_arc, v);
    if (s_fan_glow) {
        lv_arc_set_value(s_fan_glow, v);
    }
    fan_set_pct_text(v);
    s_fan_pct_override = v;
    s_fan_pct_override_until = lv_tick_get() + OVERRIDE_MS;

    /* 2) 网络命令：复用灯那套 100ms 限流 + 50ms 补发 + 合并 */
    const uint32_t now = lv_tick_get();
    s_knob_dirty = true;
    s_knob_pending_target = DETAIL_KNOB_FAN;
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

/** 命令里必须是实体报上来的原样字符串（不是屏幕上的简称）：
 *  先按名字在实体的 preset_modes 里找同一个，找不到才退回静态表的写法。 */
static const char *fan_preset_raw(const ha_entity_t *e, const fan_preset_def_t *def)
{
    if (e != NULL) {
        for (int i = 0; i < e->preset_mode_count && i < HA_FAN_PRESET_MAX; i++) {
            if (strcmp(e->preset_modes[i], def->raw) == 0) {
                return e->preset_modes[i];
            }
        }
    }
    return def->raw;
}

/** 点预设键：立刻本地高亮（环色 / 底色 / 图标跟着换），命令走队列。 */
static void fan_preset_cb(lv_event_t *ev)
{
    const fan_preset_def_t *def = (const fan_preset_def_t *)lv_event_get_user_data(ev);
    if (def == NULL || s_cmd_q == NULL || s_detail_entity[0] == '\0') {
        return;
    }
    const ha_entity_t *ent = ha_client_find(s_detail_entity);
    snprintf(s_fan_preset_local, sizeof(s_fan_preset_local), "%s", def->raw);
    s_fan_preset_local_until = lv_tick_get() + OVERRIDE_MS;
    /* 选预设 = 想要这种风：开关也立刻跟着亮（HA 跟上后自动交还） */
    s_sw_override_on = true;
    s_sw_override_until = lv_tick_get() + OVERRIDE_MS;
    s_sw_on_shown = -1;
    fan_refresh_chips(ent, def->raw);
    if (strcmp(def->raw, s_fan_preset_shown) != 0) {
        snprintf(s_fan_preset_shown, sizeof(s_fan_preset_shown), "%s", def->raw);
        s_fan_accent = detail_on_bg(lv_color_hex(def->rgb));
        if (s_fan_arc) {
            lv_obj_set_style_arc_color(s_fan_arc, s_fan_accent, LV_PART_INDICATOR);
        }
        if (s_fan_glow) {
            lv_obj_set_style_arc_color(s_fan_glow, s_fan_accent, LV_PART_INDICATOR);
        }
        if (s_fan_icon) {
            lv_obj_invalidate(s_fan_icon);
        }
        fan_refresh_osc(ent);                /* 摇头的开色跟着预设色走 */
    }
    fan_set_preset_text(def->raw);
    s_detail_fg_dirty = true;                /* 底色要跟着预设换 */
    detail_update_ambience();
    detail_apply_focus();

    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_detail_entity);
    snprintf(cmd.service, sizeof(cmd.service), "set_preset_mode");
    snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"preset_mode\":\"%s\"}", fan_preset_raw(ent, def));
    xQueueSend(s_cmd_q, &cmd, 0);
}

/** 摇头键：乐观翻转，命令走队列；HA 落定后由轮询 / WS 交还。 */
static void fan_osc_cb(lv_event_t *ev)
{
    (void)ev;
    if (s_cmd_q == NULL || s_detail_entity[0] == '\0') {
        return;
    }
    const ha_entity_t *ent = ha_client_find(s_detail_entity);
    bool cur = (ent != NULL) && ent->oscillating;
    if (s_fan_osc_local_until > lv_tick_get()) {
        cur = s_fan_osc_local;
    }
    const bool next = !cur;
    s_fan_osc_local = next;
    s_fan_osc_local_until = lv_tick_get() + OVERRIDE_MS;
    fan_refresh_osc(ent);

    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_detail_entity);
    snprintf(cmd.service, sizeof(cmd.service), "set_oscillating");
    snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"oscillating\":%s}", next ? "true" : "false");
    xQueueSend(s_cmd_q, &cmd, 0);
    ESP_LOGI(TAG, "摇头 -> %s", next ? "开" : "关");
}

/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* 灯详情页：旋钮调亮度 / 色温                                                */
/* ------------------------------------------------------------------------- */

/** 这只灯 / 这台风扇现在算不算"开着"（HA 状态 + 6s 本地开关覆盖）。
 *  灯的"关着 ⇒ 旋钮不动"规则现在对风扇同样成立：关着就返回 false。*/
static bool detail_device_on(const ha_entity_t *e)
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

/**
 * 队列满时的丢弃日志（1s 最多一条）。
 * 连续转动时如果 HA 慢了，队列会满；这里只丢这一次的值（UI 已经显示最新值，
 * 50ms 定时器之后还会补最后一条），绝不能阻塞 LVGL 任务去等空位。
 */
static void detail_knob_log_drop(int target, int value)
{
    static uint32_t last_ms;
    const uint32_t now = lv_tick_get();
    if (last_ms != 0 && (uint32_t)(now - last_ms) < 1000) {
        return;
    }
    last_ms = now;
    const char *what = (target == DETAIL_KNOB_TEMP) ? "温度"
                       : (target == DETAIL_KNOB_CT) ? "色温"
                       : (target == DETAIL_KNOB_FAN) ? "风速" : "亮度";
    ESP_LOGW(TAG, "旋钮命令队列满：丢掉这次 %s=%d（UI 已显示，HA 会慢一拍）", what, value);
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
        if (xQueueSend(s_cmd_q, &cmd, 0) != pdTRUE) {
            detail_knob_log_drop(target, value);
            return false;
        }
        return true;
    }
    if (target == DETAIL_KNOB_FAN) {
        /* 风扇风速：fan.set_percentage。和灯一样下限 1%：
           percentage=0 会被 Home Assistant 当成"关"，而电源只归开关管。
           这里是风速唯一的发送口（旋钮 / 补发 / 关页补发都从这出去）。 */
        int pct = value;
        if (pct < 1) {
            pct = 1;
        } else if (pct > 100) {
            pct = 100;
        }
        snprintf(cmd.service, sizeof(cmd.service), "set_percentage");
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"percentage\":%d}", pct);
        if (xQueueSend(s_cmd_q, &cmd, 0) != pdTRUE) {
            detail_knob_log_drop(target, value);
            return false;
        }
        return true;
    }
    snprintf(cmd.service, sizeof(cmd.service), "turn_on");
    if (target == DETAIL_KNOB_CT) {
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"color_temp_kelvin\":%d}", value);
    } else {
        /* 亮度必须 >= 1%：Home Assistant 把 brightness_pct=0 当成"关灯"，
           而电源只归开关管。这里是亮度唯一的发送口，再夹一道当兜底
           （旋钮 / 补发 / 关页补发 都从这里出去）。 */
        int pct = value;
        if (pct < 1) {
            pct = 1;
        } else if (pct > 100) {
            pct = 100;
        }
        snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"brightness_pct\":%d}", pct);
    }
    if (xQueueSend(s_cmd_q, &cmd, 0) != pdTRUE) {
        detail_knob_log_drop(target, value);
        return false;
    }
    return true;
}

/** 旋钮值还要不要发出去：灯 / 风扇关着就不发（不能把它重新点亮 / 开机）；
 *  空调关着也允许预先设目标温度（HA 的 set_temperature 不会开机）。*/
static bool detail_knob_can_send(const ha_entity_t *e)
{
    if (e == NULL) {
        return false;
    }
    if (s_detail_fan) {
        return detail_device_on(e);       /* 风扇关着：电源只归开关，旋钮不发 */
    }
    if (s_knob_pending_target == DETAIL_KNOB_TEMP || detail_is_climate(e)) {
        return true;
    }
    return detail_device_on(e);
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
 * 旋钮在详情页里调值（灯：亮度/色温；空调：目标温度；风扇：风速，每格 5%）。
 * 返回 true 表示"这次旋转被详情页吃掉了，并且真的调了值"（调用方响一下触觉）；
 * 返回 false 表示没消费（详情页没开 / 设备是关的 / 这个实体不支持 / 手指正按着圆盘），
 * 调用方据此决定要不要翻页 + 震动。
 *
 * 路由：s_detail 是灯 / 空调 / 风扇共用的一个容器，s_detail_ac / s_detail_fan
 * 决定这次落到哪一组控件上。风扇和灯同一条规矩：关着 ⇒ 旋钮什么也不做。
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
    if (s_detail_fan) {
        return fan_knob_step(e, delta); /* 风扇：每格 5%（关着返回 false）*/
    }
    if (!detail_device_on(e)) {
        return false;                  /* 灯 / 风扇关着：旋钮什么也不做（也不翻页）*/
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

    /* 诊断：连续转 100 格打一行状态。真机上再"转着转着死机"时，
       这行能直接看出是队列堆住了还是 LVGL/系统堆在漏。 */
    static uint32_t s_detent_count;
    if (++s_detent_count >= 100) {
        s_detent_count = 0;
        lv_mem_monitor_t mon;
        lv_mem_monitor(&mon);
        ESP_LOGI(TAG, "旋钮 100 格：target=%d dirty=%d inflight=%d 队列空位=%u，"
                      "LVGL 空闲=%u 峰值=%u，系统最小堆=%u",
                 (int)s_knob_pending_target, (int)s_knob_dirty, (int)s_knob_inflight,
                 (unsigned)(s_cmd_q ? uxQueueSpacesAvailable(s_cmd_q) : 0u),
                 (unsigned)mon.free_size, (unsigned)mon.max_used,
                 (unsigned)esp_get_minimum_free_heap_size());
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
    } else if (dial == s_fan_arc) {
        s_detail_knob_target = DETAIL_KNOB_FAN;    /* 风扇只有一个环 */
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

/**
 * 圆盘按"区域"切焦点，不用精确戳到那圈细线上：
 *   r <= DETAIL_REGION_R(82px)  环心那颗孔（图标/数字所在）→ 色温
 *   r >  DETAIL_REGION_R        环带 / 环外                      → 亮度
 * 只处理"什么都没打中"的按压：判据是事件 target 必须就是 s_detail 本身。
 * 开关 / 两个数字标签 / 环 / 空调的模式·风速键都有自己的回调或命中区，
 * 它们的事件 bubbling 上来时 target 是子对象 → 这里直接跳过，决不会抢走它们。
 * 空调只有一个环（永远是焦点）：中间随便点不改焦点、也不发任何值。
 */
static void detail_region_pressed_cb(lv_event_t *e)
{
    if (s_detail == NULL || lv_event_get_target(e) != s_detail) {
        return;                          /* 打中了子控件：让它自己处理（冒泡上来的事件）*/
    }
    if (s_detail_ac || s_detail_fan) {
        return;                          /* 空调 / 风扇：单环，中间区域不做任何事 */
    }
    lv_indev_t *indev = lv_indev_active();
    if (indev == NULL) {
        return;
    }
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    const int dx = (int)p.x - DETAIL_CX;
    const int dy = (int)p.y - DETAIL_CY;
    const int r2 = dx * dx + dy * dy;    /* 比平方，省一个开方 */
    const int target = (r2 <= DETAIL_REGION_R * DETAIL_REGION_R)
                       ? DETAIL_KNOB_CT : DETAIL_KNOB_BR;
    if (target == s_detail_knob_target) {
        return;
    }
    s_detail_knob_target = target;
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

    /* ---- 风扇：风速环。拖动只更新显示，松手才发 set_percentage ---- */
    if (dial == s_fan_arc) {
        int fv = (int)lv_arc_get_value(dial);
        if (fv < 1) {
            fv = 1;                        /* 0% = 关，电源只归开关 */
        }
        if (fv > 100) {
            fv = 100;
        }
        if (s_fan_glow) {
            lv_arc_set_value(s_fan_glow, fv);
        }
        fan_set_pct_text(fv);
        s_fan_pct_override = fv;
        s_fan_pct_override_until = lv_tick_get() + OVERRIDE_MS;
        if (code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) {
            return;                        /* 拖动中只更新显示，松手才发命令 */
        }
        ESP_LOGI(TAG, "风速圆盘松手: %d%%", fv);
        if (s_detail_entity[0] == '\0' || s_cmd_q == NULL) {
            return;
        }
        if (!detail_device_on(ha_client_find(s_detail_entity))) {
            return;                        /* 关着：环本来就不该能拖（兜底不发）*/
        }
        cmd_t fc;
        memset(&fc, 0, sizeof(fc));
        snprintf(fc.entity, sizeof(fc.entity), "%s", s_detail_entity);
        snprintf(fc.service, sizeof(fc.service), "set_percentage");
        snprintf(fc.attrs, sizeof(fc.attrs), "{\"percentage\":%d}", fv);
        xQueueSend(s_cmd_q, &fc, 0);
        return;
    }

    const bool is_ct = (dial == s_detail_ct_arc);
    int v = (int)lv_arc_get_value(dial);
    if (!is_ct && v < 1) {
        v = 1;          /* 亮度下限 1%：环的范围已经是 1..100，这里再兜一道 */
    }

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
    s_sw_on_shown = -1;               /* CHECKED 状态交给下一次 detail_apply_entity 对齐 */
    s_sw_on_now = want_on;
    /* 轨道/滑块颜色立刻按新状态刷，不等 500ms 轮询（配色规则见 detail_apply_switch_style）*/
    detail_apply_switch_style(want_on);

    if (!want_on) {
        /* 关掉：丢掉还没补发的旋钮值（灯 / 风扇都一样：别让关机后补发的
           brightness_pct / percentage 又把它打开）*/
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

    /* 轨道（没填满的那段）：0x1A1A1C @70%，比原来的 0x2C2C2E 深一档也淡一档，
       不再像一条明显的灰带；焦点环 70% / 非焦点环 45% 由 detail_apply_focus 刷。 */
    lv_obj_set_style_arc_width(arc, width, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, DETAIL_TRACK_COLOR, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(arc, DETAIL_TRACK_OPA, LV_PART_MAIN);
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

    /* 区域点击：环心（r<=72）= 色温、环带/环外 = 亮度；
       子控件自己的事件冒泡上来时 target 不是容器，会被这个回调忽略。 */
    lv_obj_add_event_cb(s_detail, detail_region_pressed_cb, LV_EVENT_PRESSED, NULL);

    /* ---- 圆盘：先光晕（在下面），再圆环（在上面）---- */
    s_detail_br_glow = detail_make_arc(s_detail, DETAIL_BR_GLOW_R, DETAIL_BR_GLOW_W, true);
    lv_arc_set_range(s_detail_br_glow, 1, 100);
    lv_arc_set_value(s_detail_br_glow, 1);

    s_detail_ct_glow = detail_make_arc(s_detail, DETAIL_CT_GLOW_R, DETAIL_CT_GLOW_W, true);
    lv_arc_set_range(s_detail_ct_glow, 2000, 6500);
    lv_arc_set_value(s_detail_ct_glow, 4000);

    /* 外环 = 亮度：1..100%（0% 会被 HA 当成关灯，电源只归开关管）*/
    s_detail_br_arc = detail_make_arc(s_detail, DETAIL_BR_R, DETAIL_BR_W, false);
    lv_arc_set_range(s_detail_br_arc, 1, 100);
    lv_arc_set_value(s_detail_br_arc, 1);
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

    /* 顶上那个 36x36 返回键已经删掉：退出详情页走"从左边缘右滑"（swipe_cb 的
       边缘返回手势），所以顶部带里只剩状态栏。 */

    /* ---- 中间堆栈：实体名 / 灯泡 / 大号百分比 / 色温文字 ----
       名字建在圆环之后：聚焦时的光晕不会盖住它；它同时也是原来 y=58 那个标题，
       现在整行搬进环心最上面（顶部带不再画任何东西）。 */
    s_detail_name = lv_label_create(s_detail);
    lv_label_set_text(s_detail_name, "");
    lv_obj_set_style_text_font(s_detail_name, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_detail_name, COLOR_MUTED, 0);   /* 0x8E8E93 */
    lv_label_set_long_mode(s_detail_name, LV_LABEL_LONG_DOT);     /* 再长也"…"，压不到环上 */
    lv_obj_set_style_text_align(s_detail_name, LV_TEXT_ALIGN_CENTER, 0);  /* 定宽 + 居中 */
    lv_obj_set_width(s_detail_name, DETAIL_NAME_W);
    lv_obj_align(s_detail_name, LV_ALIGN_TOP_MID, 0, DETAIL_NAME_Y);
    lv_obj_add_flag(s_detail_name, LV_OBJ_FLAG_HIDDEN);           /* 还没拿到名字：整行不画 */

    s_detail_icon = lv_obj_create(s_detail);
    lv_obj_remove_style_all(s_detail_icon);
    lv_obj_set_size(s_detail_icon, 24, 24);
    lv_obj_set_pos(s_detail_icon, DETAIL_CX - 12, 170);
    lv_obj_remove_flag(s_detail_icon, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_detail_icon, detail_bulb_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_detail_br_value = lv_label_create(s_detail);
    lv_label_set_text(s_detail_br_value, "--");
    /* 项目里已经编进来的最大号字（montserrat_28），不新增字体 */
    lv_obj_set_style_text_font(s_detail_br_value, DETAIL_PCT_FONT, 0);
    lv_obj_set_style_text_color(s_detail_br_value, COLOR_TEXT, 0);
    lv_obj_align(s_detail_br_value, LV_ALIGN_TOP_MID, 0, 196);
    lv_obj_add_flag(s_detail_br_value, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_detail_br_value, detail_focus_br_cb, LV_EVENT_CLICKED, NULL);

    s_detail_ct_value = lv_label_create(s_detail);
    lv_label_set_text(s_detail_ct_value, "--");
    lv_obj_set_style_text_font(s_detail_ct_value, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_detail_ct_value, COLOR_TEXT, 0);
    lv_obj_align(s_detail_ct_value, LV_ALIGN_TOP_MID, 0, 226);   /* 给下面的焦点提示让 2px */
    lv_obj_add_flag(s_detail_ct_value, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_detail_ct_value, detail_focus_ct_cb, LV_EVENT_CLICKED, NULL);

    /* 关灯：整组圆盘藏起来，只留这一行提示 */
    s_detail_off_hint = lv_label_create(s_detail);
    lv_label_set_text(s_detail_off_hint, "已关闭");
    lv_obj_set_style_text_font(s_detail_off_hint, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_detail_off_hint, COLOR_MUTED, 0);
    lv_obj_align(s_detail_off_hint, LV_ALIGN_TOP_MID, 0, 190);

    /* ---------------- 空调那一组（同一个容器，默认全藏） ----------------
       几何见文件头的 AC_* 宏：环 114/22 + 光晕；中间堆栈（实体名 / 图标 /
       大号温度 / 室内温度 / 风速钮）都在内孔里；底部三行按"越往下越宽的缺口楔形"排。 */
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

    /* ---------------- 风扇那一组（同一个容器，默认全藏） ----------------
       几何见文件头的 FAN_* 宏：环 118/22 + 光晕；中间（实体名 / 风扇图标 /
       大号百分比 / 当前预设）都在内孔里；底部是 4 个预设键 + 摇头 + 电源开关。 */
    s_fan_glow = detail_make_arc(s_detail, FAN_GLOW_R, FAN_GLOW_W, true);
    lv_arc_set_range(s_fan_glow, 1, 100);
    lv_arc_set_value(s_fan_glow, 60);

    s_fan_arc = detail_make_arc(s_detail, FAN_R, FAN_W, false);
    lv_arc_set_range(s_fan_arc, 1, 100);
    lv_arc_set_value(s_fan_arc, 60);
    lv_obj_add_event_cb(s_fan_arc, detail_dial_pressed_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_fan_arc, detail_dial_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_fan_arc, detail_dial_changed_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(s_fan_arc, detail_dial_changed_cb, LV_EVENT_PRESS_LOST, NULL);

    /* 中间：风扇图标（自绘，颜色跟预设走；形状固定，不跑定时器）*/
    s_fan_icon = lv_obj_create(s_detail);
    lv_obj_remove_style_all(s_fan_icon);
    lv_obj_set_size(s_fan_icon, FAN_ICON_SIZE, FAN_ICON_SIZE);
    lv_obj_set_pos(s_fan_icon, DETAIL_CX - FAN_ICON_SIZE / 2, FAN_ICON_Y);
    lv_obj_remove_flag(s_fan_icon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_fan_icon, fan_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    /* 中间：大号百分比 */
    s_fan_pct = lv_label_create(s_detail);
    lv_label_set_text(s_fan_pct, "--");
    lv_obj_set_style_text_font(s_fan_pct, DETAIL_PCT_FONT, 0);
    lv_obj_set_style_text_color(s_fan_pct, COLOR_TEXT, 0);
    lv_obj_align(s_fan_pct, LV_ALIGN_TOP_MID, 0, FAN_PCT_Y);

    /* 中间：当前预设（次要文字，弱化色）*/
    s_fan_preset = lv_label_create(s_detail);
    lv_label_set_text(s_fan_preset, "--");
    lv_obj_set_style_text_font(s_fan_preset, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_fan_preset, COLOR_MUTED, 0);
    lv_obj_align(s_fan_preset, LV_ALIGN_TOP_MID, 0, FAN_PRESET_Y);

    /* 4 个预设键：固定顺序 直吹 / 自然 / 循环 / 睡眠，命令发原样中文 */
    for (int i = 0; i < FAN_MODE_COUNT; i++) {
        const int fx = DETAIL_CX - (FAN_MODE_COUNT * FAN_MODE_W + (FAN_MODE_COUNT - 1) * FAN_MODE_GAP) / 2
                       + i * (FAN_MODE_W + FAN_MODE_GAP);
        lv_obj_t *btn = lv_button_create(s_detail);
        s_fan_mode_btn[i] = btn;
        lv_obj_set_size(btn, FAN_MODE_W, FAN_MODE_H);
        lv_obj_set_pos(btn, fx, FAN_MODE_Y);
        lv_obj_set_style_radius(btn, FAN_MODE_H / 2, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x24262B), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_80, 0);
        lv_obj_set_style_pad_all(btn, 0, 0);
        lv_obj_add_event_cb(btn, fan_preset_cb, LV_EVENT_CLICKED, (void *)&k_fan_presets[i]);

        lv_obj_t *lbl = lv_label_create(btn);
        s_fan_mode_lbl[i] = lbl;
        lv_label_set_text(lbl, k_fan_presets[i].label);
        lv_obj_set_style_text_font(lbl, &lv_font_cjk16, 0);
        lv_obj_set_style_text_color(lbl, COLOR_TEXT, 0);
        lv_obj_center(lbl);
    }

    /* 摇头：小圆角键，开 = 预设色填充（和预设键同一套"亮起来"的语言）*/
    s_fan_osc_btn = lv_button_create(s_detail);
    lv_obj_set_size(s_fan_osc_btn, FAN_OSC_W, FAN_OSC_H);
    lv_obj_align(s_fan_osc_btn, LV_ALIGN_TOP_MID, FAN_OSC_DX, FAN_OSC_Y);
    lv_obj_set_style_radius(s_fan_osc_btn, FAN_OSC_H / 2, 0);
    lv_obj_set_style_bg_color(s_fan_osc_btn, lv_color_hex(0x24262B), 0);
    lv_obj_set_style_bg_opa(s_fan_osc_btn, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_fan_osc_btn, 1, 0);
    lv_obj_set_style_border_color(s_fan_osc_btn, lv_color_hex(0x3A3A3C), 0);
    lv_obj_set_style_pad_all(s_fan_osc_btn, 0, 0);
    lv_obj_add_event_cb(s_fan_osc_btn, fan_osc_cb, LV_EVENT_CLICKED, NULL);
    s_fan_osc_lbl = lv_label_create(s_fan_osc_btn);
    lv_label_set_text(s_fan_osc_lbl, "摇头");
    lv_obj_set_style_text_font(s_fan_osc_lbl, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_fan_osc_lbl, COLOR_TEXT, 0);
    lv_obj_center(s_fan_osc_lbl);

    /* 焦点提示两件套：环上"当前值"小圆点 + 环心下面一行小字。
       建在所有环之后 → 小圆点压在环的描边上（不然会被环自己盖住）。
       两者都不是可点对象，不参与命中；没有定时器 / 动画引用它们，
       每次 build 重新创建、指到新对象上，不会留悬空指针。 */
    s_detail_marker = lv_obj_create(s_detail);
    lv_obj_remove_style_all(s_detail_marker);
    lv_obj_set_size(s_detail_marker, DETAIL_MARKER_SIZE, DETAIL_MARKER_SIZE);
    lv_obj_set_style_radius(s_detail_marker, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_detail_marker, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_detail_marker, COLOR_TEXT, 0);
    lv_obj_remove_flag(s_detail_marker, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_detail_marker, LV_OBJ_FLAG_HIDDEN);

    s_detail_hint = lv_label_create(s_detail);
    lv_label_set_text(s_detail_hint, "");
    lv_obj_set_style_text_font(s_detail_hint, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_detail_hint, COLOR_MUTED, 0);
    lv_obj_set_style_text_align(s_detail_hint, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(s_detail_hint, 100);
    lv_obj_align(s_detail_hint, LV_ALIGN_TOP_MID, 0, DETAIL_HINT_Y);
    lv_obj_add_flag(s_detail_hint, LV_OBJ_FLAG_HIDDEN);

    /* 开关：仍在（开关逻辑一个字没改），挪到圆环下方 45°..135° 的空缺口里。
       96x46 放在 (132..228, 273..319)，两个圆环的圆头都在它两侧，互不遮挡；
       空调页那一行更挤，detail_apply_entity 会把它改成 72x32 放到 y=314；
       风扇页再窄一档（62x28 放到 x=178..240 / y=314），左边让给摇头键。
       它建在最后 = 在最上层，角落不会被子对象的命中区抢走。
       配色不用主题默认（那圈蓝在 LV_PART_INDICATOR|CHECKED 上）：轨道/指示器/
       滑块全部本地接管，关=灰、开=当前页强调色，见 detail_apply_switch_style。 */
    s_detail_power_sw = lv_switch_create(s_detail);
    lv_obj_set_size(s_detail_power_sw, 96, 46);
    lv_obj_align(s_detail_power_sw, LV_ALIGN_TOP_MID, 0, 273);
    lv_obj_set_style_radius(s_detail_power_sw, 16, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_detail_power_sw, lv_color_hex(0x3A3A3C), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_detail_power_sw, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_detail_power_sw, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(s_detail_power_sw, 0, LV_PART_MAIN);
    lv_obj_set_style_outline_width(s_detail_power_sw, 0, LV_PART_MAIN);
    /* 蓝色来自这里：默认主题给 INDICATOR + CHECKED 上了主色，必须一起接管 */
    lv_obj_set_style_radius(s_detail_power_sw, 16, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_detail_power_sw, lv_color_hex(0x3A3A3C), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_detail_power_sw, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_border_width(s_detail_power_sw, 0, LV_PART_INDICATOR);
    lv_obj_set_style_shadow_width(s_detail_power_sw, 0, LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(s_detail_power_sw, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_detail_power_sw, lv_color_hex(0x8E8E93), LV_PART_KNOB);
    lv_obj_set_style_bg_opa(s_detail_power_sw, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_width(s_detail_power_sw, 0, LV_PART_KNOB);
    lv_obj_set_style_shadow_width(s_detail_power_sw, 0, LV_PART_KNOB);
    lv_obj_add_event_cb(s_detail_power_sw, detail_power_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* 新容器：默认按"灯页"显示，空调那组先整组藏起来（要先置脏，
       否则 detail_update_ambience 会以为底色没变，新容器上还是空样式）*/
    s_detail_ac = false;
    s_detail_fan = false;
    s_ac_mode_shown[0] = '\0';
    s_detail_fg_dirty = true;
    s_sw_geom = -1;
    s_sw_on_shown = -1;
    s_sw_style_shown = -1;           /* 新开关：配色缓存作废，下一帧按当前状态重刷 */
    s_sw_on_now = false;
    s_ac_temp_txt[0] = '\0';
    s_ac_room_txt[0] = '\0';
    s_ac_fan_txt[0] = '\0';
    s_detail_name_shown[0] = '\0';   /* 新容器上的名字行还是空的：清掉缓存才会真的写进去 */
    s_detail_hint_shown[0] = '\0';   /* 提示行的文字缓存同理 */
    s_fan_preset_shown[0] = '\x01';  /* 不可能是任何预设：进页面后强制刷一遍颜色 */
    s_fan_preset_txt[0] = '\0';
    s_fan_pct_txt[0] = '\0';
    s_fan_accent = lv_color_hex(0x2DD4BF);
    ac_hide_all(true);
    fan_hide_all(true);
    detail_apply_focus();
    detail_update_ambience();       /* 底色 / 环色 / 自绘图标都先落到一个确定状态 */

    ESP_LOGI(TAG, "详情页建好：%d 个对象（灯：2 环 + 2 光晕 + 灯泡 + 百分比 + 色温 + 已关闭；"
                  "空调：温度环 + 光晕 + 模式图标 + 大号温度 + 室内 + 风速 + 5 模式键；"
                  "风扇：风速环 + 光晕 + 风扇图标 + 大号百分比 + 预设 + 4 预设键 + 摇头；"
                  "共用：开关 + 环心实体名 + 焦点值点/提示；无返回键，退出靠左边缘右滑）",
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
    s_detail_fan = (strncmp(entity_id, "fan.", 4) == 0);
    /* 每次进页面都以默认焦点开始：灯 = 亮度环，空调 = 温度环，风扇 = 风速环
       （看实体 id 前缀，不看上一次开的是哪一页；实体还没拉到时也照样对）*/
    s_detail_knob_target = s_detail_ac ? DETAIL_KNOB_TEMP
                         : s_detail_fan ? DETAIL_KNOB_FAN
                                        : DETAIL_KNOB_BR;
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
    s_fan_preset_local_until = 0;
    s_fan_osc_local_until = 0;
    s_fan_pct_override_until = 0;
    s_fan_preset_shown[0] = '\x01';     /* 换实体：颜色/预设键必须重刷一遍 */
    s_fan_preset_txt[0] = '\0';
    s_fan_pct_txt[0] = '\0';
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

    build_detail_view(scr);      /* 灯 + 空调 + 风扇共用的详情页 */
    s_detail_visible = false;
    s_detail_ac = false;
    s_detail_fan = false;
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
