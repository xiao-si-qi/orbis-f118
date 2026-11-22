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
/* 大号数字字体：64 号 / 4bpp / 只含 0-9 . % ° - : 和空格（lv_font_conv 生成，
   Bahnschrift；数字高 46px、行高 47px）。灯光/风扇/天气三个页面的大号数值用它，
   设计稿里那是 92px（≈66px 字高）——只差字号，字形和字重都对齐了。 */
LV_FONT_DECLARE(lv_font_num64);

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

/* 卡片几何：逐条对齐设计稿 .grid2 / .card
   （网格左上 (34,88)、2 列 140px、间距 12、卡片 140x90 圆角 18）。 */
#define CARD_W       140
#define CARD_H        90
#define CARD_GAP      12
#define CARD_RADIUS   18
#define CARD_ICON     19    /* .card .ic：19x19，左上角 (12,10) */
#define CARD_DOT       5    /* .card .dot：5x5，右上角 (12,14) */
#define CARD_PAD_X    12    /* .card padding: 10px 12px */
#define GRID_TOP      88    /* .grid2 top:88 */
#define GRID_LEFT     34    /* .grid2 left:34 */

#define COLOR_BG        lv_color_hex(0x000000)
#define COLOR_CARD      lv_color_hex(0x1C1C1E)
#define COLOR_CARD_ON   lv_color_hex(0x1D4ED8)
#define COLOR_BORDER    lv_color_hex(0x2C2C2E)
#define COLOR_TEXT      lv_color_hex(0xF2F2F7)
#define COLOR_MUTED     lv_color_hex(0x8E8E93)
#define COLOR_OK        lv_color_hex(0x30D158)
#define COLOR_BAD       lv_color_hex(0xFF453A)
#define COLOR_ACCENT    lv_color_hex(0x2563EB)

/* 卡片的"域"：决定左上角图标、语义色、有没有状态点（设计稿 CARD LIBRARY 的 12 类）。
   只用来决定**怎么画**，不决定点一下干什么——点击仍然走 tile_clicked_cb 的
   "light/climate/fan 开详情页、weather 开天气页、其它 toggle/sensor" 那套。 */
typedef enum {
    TILE_DOM_LIGHT = 0,
    TILE_DOM_CLIMATE,
    TILE_DOM_FAN,
    TILE_DOM_COVER,
    TILE_DOM_SWITCH,      /* switch. / input_boolean. / lock. / 未知可开关实体 */
    TILE_DOM_MEDIA,
    TILE_DOM_SENSOR,
    TILE_DOM_BINARY,
    TILE_DOM_WEATHER,
    TILE_DOM_SCENE,       /* scene. / script. */
    TILE_DOM_AUTO,        /* automation. */
} tile_dom_t;

typedef struct {
    lv_obj_t *card;
    lv_obj_t *label;      /* 标题（左下，cjk16 #8B95A8）*/
    lv_obj_t *value;      /* 数值 / 状态（左侧，montserrat_28 / cjk16）*/
    lv_obj_t *unit;       /* 单位（数值右边，montserrat_20 #97A1B3）*/
    lv_obj_t *icon;       /* 左上角 19x19 图标 */
    lv_obj_t *dot;        /* 右上角 5x5 状态点（sensor/scene/weather 没有）*/
    lv_color_t icon_col;  /* 图标当前颜色（自绘回调读它）*/
    uint8_t icon_alt;     /* 图标变体：传感器里 % 的（湿度）画水滴，其它画温度计 */
    char entity[96];
    tile_type_t type;
    tile_dom_t dom;
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
static lv_obj_t *s_batt_icon;      /* 状态栏：电量图标（本板无校准 → 未知态）*/
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
static lv_obj_t *s_detail_name_row;   /* 名字行：图标 + 名字的 flex 容器 */
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
/* 大号数字：64 号数字专用字体（数字 46px 高、行高 47px），灯光 / 风扇 / 天气用它；
   空调页那一列（名字/图标/温度/室内/风速）纵向太挤，继续用 montserrat_28。 */
#define BIG_NUM_FONT     (&lv_font_num64)

/* ---- 灯光页（s_detail 第一种形态）：几何 / 配色逐条对齐设计稿 index.html ----
 * 设计稿 .scr[data-scr="light"]：一条环 + 上中下三段式，没有内环、没有开关：
 *   名字行（灯泡 16px + 6px 间距 + 名字）          top 74
 *   大号数值（92px）                               top 112..191
 *   % / K 单位（28px，opacity .72）                和数值同一行
 *   档位文字「亮度 / 色温」（16px，字距 .24em）      top 202
 *   控件行（分段控件 + 40x40 电源键）               top 230
 *   提示行（16px，字距 .12em）                      top 288
 * 环：设计稿 r=156 / stroke 10；LVGL 的 arc 半径是"外径"、线宽向内画，
 * 所以取 161 = 156 + 10/2。环心 = 屏幕中心 (180,180)（设计稿 cx=cy=180），
 * 270° 弧从 135° 起、缺口朝下（DETAIL_ANG_START/END 已经是这组数）。
 *
 * 字号：设计稿的 92px 数字在固件里没有对应字（工程只编了 montserrat 14/20/28，
 * 且不允许新增字体），大号数值改用 montserrat_28，并把它摆到设计稿那段
 * 92px 字块的中心（112..191 → 中心 151）——数字变小，但重心和留白一致。
 * 字母间距（letter-spacing）用 LVGL 的 text_letter_space 还原：
 *   档位文字 .24em ≈ 4px，提示行 .12em ≈ 2px，名字行 .06em ≈ 1px。
 */
#define LIGHT_R             161    /* 环外径 = 156 + 10/2 */
#define LIGHT_W              10    /* 描边宽度（设计稿 stroke-width:10）*/
#define LIGHT_CY            180    /* 环心 y = 屏幕中心 */
#define LIGHT_GLOW_R        168    /* 弧外柔光（设计稿的 drop-shadow 近似）*/
#define LIGHT_GLOW_W         22
#define LIGHT_NAME_Y         66    /* 名字行（图标 + 文字）顶边 */
#define LIGHT_ICON           16    /* 名字行左侧灯泡图标（设计稿 .l-name .ic）*/
#define LIGHT_VALUE_Y       128    /* 大号数值顶边（64 号字体：46px 字高 / 47px 行高
                                      → 墨迹 129..174，中心 151，和设计稿 92px 字块的
                                      中心（112..191 → 151）对齐）*/
#define LIGHT_UNIT_DY        14    /* 单位相对数值下移：让 % 的字墨中心 ≈ 数值字墨中心（151）*/
#define LIGHT_LABEL_Y       196    /* 档位文字顶边 */
#define LIGHT_SEG_Y         226    /* 控件行顶边 */
#define LIGHT_SEG_H          36    /* 分段控件外高：3 padding + 30 按钮 + 3 */
#define LIGHT_SEG_PAD         3
#define LIGHT_BTN_W          62    /* 单个分段按钮：2 个汉字 32 + 左右各 15 */
#define LIGHT_BTN_H          30
#define LIGHT_SEG_GAP         4
#define LIGHT_SEG_W         (LIGHT_SEG_PAD * 2 + LIGHT_BTN_W * 2 + LIGHT_SEG_GAP)
#define LIGHT_CTL_GAP        10    /* 分段控件与电源键之间（设计稿 gap:10）*/
#define LIGHT_POWER          40    /* 电源键直径（设计稿 .p-btn 40x40）*/
#define LIGHT_HINT_Y        282    /* 提示行顶边（设计稿 288，cjk16 高 20 → 282）*/
#define LIGHT_MARKER_SIZE    14    /* 环上值点（设计稿 r=7.5）*/
/* 设计稿配色：rgba(255,255,255,a) 叠在近黑底上 → 这里用"白色 + opa"保留半透明 */
#define LIGHT_INK           lv_color_hex(0xD7DEE9)   /* .l-name             */
#define LIGHT_INK_MUTED     lv_color_hex(0x657086)   /* .l-label            */
#define LIGHT_HINT_INK      lv_color_hex(0x5D677A)   /* .l-hint             */
#define LIGHT_SEG_TXT       lv_color_hex(0x78829A)   /* .l-seg 未选中       */
#define LIGHT_SEG_TXT_ON    lv_color_hex(0xF3F6FB)   /* .l-seg 选中         */
#define LIGHT_OFF_NUM       lv_color_hex(0x5F6879)   /* .is-off .l-temp     */
#define LIGHT_OFF_LABEL     lv_color_hex(0x4B5464)   /* .is-off .l-label    */
#define LIGHT_ICON_INK      lv_color_hex(0x8D97A8)   /* .l-name .ic         */
#define LIGHT_ICON_OFF      lv_color_hex(0x5A6373)   /* .is-off .l-name .ic */
#define LIGHT_PWR_INK       lv_color_hex(0xC9D3E2)   /* .p-btn 图标         */
#define LIGHT_PWR_INK_ON    lv_color_hex(0x17110A)   /* .p-btn.on 图标      */

/* 氛围光（设计稿 .light-bg 的径向渐变近似）：三层同心半透明圆叠出"中间亮、边缘化开"。
   固件不支持径向渐变，只能用纯色圆叠加；透明度按 RGB565 面板的观感调过：
   22% / 11% / 4.7%（旧值 10% / 4% 在黑底上几乎看不见，这就是"光晕不显示"的原因）。
   颜色用**灯色/预设色的原始色相**（不是对比度处理后的 accent，那个会被推成灰白）。 */
#define GLOW_DISC_N 3
static const int k_glow_r[GLOW_DISC_N]   = {112, 148, 176};
static const lv_opa_t k_glow_opa[GLOW_DISC_N] = {66, 33, 15};
static lv_obj_t *s_glow_disc[GLOW_DISC_N];
/* 控件行：亮度 / 色温 分段控件 + 40x40 圆形电源键（设计稿 .l-seg / .p-btn）。
   电源键由灯光页和风扇页共用（两页不会同时出现），x 位置按页摆。 */
static lv_obj_t *s_light_seg;
static lv_obj_t *s_light_seg_btn[2];
static lv_obj_t *s_light_seg_lbl[2];
static lv_obj_t *s_dial_power;
/* 文字/外观缓存：500ms 轮询下没变就别再 set_text / 重摆 */
static int s_light_num_x = -1;
static int s_light_unit_x = -1;
static char s_light_num_shown[8];
static char s_light_unit_shown[8];
static char s_light_label_shown[16];
static int8_t s_light_chip_shown = -1;   /* 分段控件/电源键上次的视觉状态 */
static bool s_dial_pwr_on;               /* 电源键此刻该画"开"（自绘图标读它）*/
static int8_t s_hint_y_shown = -1;       /* 提示行当前位置（灯/风扇 282，空调 246）*/
static int s_pwr_x_shown = -1;           /* 共用电源键当前 x */
static int s_pwr_y_shown = -1;           /* 共用电源键当前 y */
static int s_pwr_size_shown = -1;        /* 共用电源键当前边长（40 灯/风扇、44 空调）*/

/** 大号数值用哪套字体：数字/小数点走 64 号数字字体（只含 0-9 . % ° - : 空格），
 *  "OFF" 这种字母退回 montserrat_28，中文（理论上不会出现）退回 cjk16。 */
static const lv_font_t *big_num_font_for(const char *s)
{
    if (s == NULL || s[0] == '\0') {
        return BIG_NUM_FONT;
    }
    for (const unsigned char *p = (const unsigned char *)s; *p != '\0'; p++) {
        if (*p >= 0x80) {
            return &lv_font_cjk16;
        }
        if (!((*p >= '0' && *p <= '9') || *p == '.')) {
            return DETAIL_PCT_FONT;      /* OFF / -- 里的字母 */
        }
    }
    return BIG_NUM_FONT;
}

static void light_place_value(void);
static void light_apply_values(const ha_entity_t *e);
static void light_apply_chips(bool on, bool has_tint, bool ct);
static void detail_power_set(bool want_on);
static void detail_set_hint_forced(const char *text);
static void detail_place_hint(int y);
static void detail_place_power(int size, int x, int y);
static void detail_apply_glow(lv_color_t base, bool on);
static void detail_set_hidden(lv_obj_t *obj, bool hidden);
static lv_obj_t *detail_make_glow_disc(lv_obj_t *parent, int r, lv_opa_t opa, int cx, int cy);
static void light_recenter(lv_obj_t *o, int r);
static void light_power_draw_cb(lv_event_t *e);
static void light_power_cb(lv_event_t *e);
static void light_tab_cb(lv_event_t *e);
/* 天气页（定义在 build_detail_view 前面，这里先声明给翻页 / 手势 / 旋钮用） */
static void wx_open(const char *entity_id);
static void wx_close(void);
static void wx_refresh_ui(void);
static void wx_scroll_step(int delta);
static void build_weather_page(lv_obj_t *scr);
/* 传感器详情页（同样定义在 build_detail_view 前面，这里先声明给点击 / 手势 / 刷新用） */
static bool s_sens_visible;
static void sensor_open(const char *entity_id);
static void sensor_close(void);
static void sensor_refresh(void);
static void build_sensor_page(lv_obj_t *scr);

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
/* 空调详情页（设计稿"屏幕 1 · 温控主界面" dial 帧），和灯/风扇同一套骨架：
 *   环 r=156 / stroke 10 / 270°（135°→45°，缺口朝下）
 *   88px 温度（°.deg 30px 强调色）/ 副标题「目标温度 · 室内 26.4°C」
 *   5 个 40x40 模式键（radius 13，选中 = 模式色 15% 底 + 40% 边 + 模式色图标）
 *   44x44 圆形电源键 / 顶部两枚胶囊（设计稿是"模式 + 风速"，这里合成"风速 + 摆风"）
 * 本页的纵向排布（全在 r=150 文字圆内）：
 *   名字行 60..80 / 数值 100..147（num64 字墨 101..146）/ 副标题 160..180
 *   胶囊行 186..216（电源 179..223）/ 5 个模式键 226..266 / 提示 282..302 */
#define AC_TEMP_R          161     /* 环外径 = 156 + 10/2（和灯 / 风扇完全一致）*/
#define AC_TEMP_W           10
#define AC_TEMP_GLOW_R     168     /* 弧外柔光，和灯 / 风扇同一组数 */
#define AC_TEMP_GLOW_W      22
#define AC_UNIT             10     /* 弧值单位：0.1°C */
#define AC_NAME_ROW_Y       60     /* 名字行（和灯 / 风扇同一骨架，整行上移）*/
#define AC_VALUE_Y         100     /* 大号目标温度（num64）*/
#define AC_UNIT_DY          14     /* ° 的字墨中心和大数字对齐 */
#define AC_ROOM_Y          160     /* 「制冷 · 室内 26.4°C」（设计稿 .d-unit）*/
#define AC_CHIP_Y          186     /* 胶囊行（设计稿 .d-chips）*/
#define AC_CHIP_H           30
#define AC_FAN_W           100     /* 风速胶囊（设计稿 min-width 92，中文要多一点）*/
#define AC_SWING_W          88     /* 摆风胶囊 */
#define AC_CHIP_GAP          8
#define AC_POWER            44     /* 设计稿 .d-power 44x44 */
#define AC_POWER_GAP        10
#define AC_MODE_Y          226     /* 5 个模式键：40x40 + gap 6 = 224，x 68..292 */
#define AC_MODE_W           40
#define AC_MODE_H           40
#define AC_MODE_GAP          6
#define AC_MODE_COUNT        5     /* 自动 / 制冷 / 制热 / 除湿 / 送风 */
#define AC_STATE_LEN        12

/* 模式配色（任务指定：制冷蓝 / 制热橙 / 除湿青 / 送风绿 / 自动灰白 / 关=暗）*/
#define AC_COOL_HEX  0x4C8DFF   /* 设计稿 --cool */
#define AC_HEAT_HEX  0xFF8A3D   /* --heat */
#define AC_DRY_HEX   0x2FD6C3   /* --dry  */
#define AC_FAN_HEX   0x5BD97A   /* --fan  */
#define AC_AUTO_HEX  0xB9C2D0   /* --auto */
#define AC_OFF_HEX   0x3A3A3C

static lv_obj_t *s_ac_arc;                  /* 目标温度环（可点可拖）*/
static lv_obj_t *s_ac_glow;                 /* 同一条环的光晕 */
static lv_obj_t *s_ac_icon;                 /* 中间：模式图标（自绘）*/
static lv_obj_t *s_ac_temp_value;           /* 大号目标温度（num64）*/
static lv_obj_t *s_ac_unit;                  /* 目标温度后面的 °（模式色）*/
static lv_obj_t *s_ac_room;                  /* 「制冷 · 室内 26.4°C」*/
static lv_obj_t *s_ac_fan_btn;               /* 风速胶囊（点一下切下一档）*/
static lv_obj_t *s_ac_fan_label;
static lv_obj_t *s_ac_swing_btn;             /* 摆风胶囊（default ↔ full_swing）*/
static lv_obj_t *s_ac_swing_label;
static lv_obj_t *s_ac_name_icon;             /* 名字行左侧：当前模式图标 */
static lv_obj_t *s_ac_mode_btn[AC_MODE_COUNT];
static lv_obj_t *s_ac_mode_icon[AC_MODE_COUNT];
static lv_color_t s_ac_key_col[AC_MODE_COUNT];   /* 每个模式键图标当前颜色 */
static char s_ac_swing_local[16];            /* 摆风：本地覆盖（HA 跟上就交还）*/
static uint32_t s_ac_swing_local_until;
static int s_ac_num_x = -1;                  /* 数值 / 单位当前 x（没变就不重摆）*/
static int s_ac_unit_x = -1;
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
static char s_ac_room_txt[48];
static char s_ac_fan_txt[32];
/* 本地覆盖：点了模式 / 风速之后先按本地值显示，HA 落定后交还（和灯的 6s 覆盖同一套）*/
static char s_ac_mode_local[AC_STATE_LEN];
static uint32_t s_ac_mode_local_until;
static char s_ac_fan_local[HA_FAN_LEN];
static uint32_t s_ac_fan_local_until;
static int s_ac_override_t10;               /* 目标温度覆盖（0.1°C）*/
static uint32_t s_ac_override_until;

/* ---- 风扇详情页（同一个 s_detail，第三种形态，和灯共用同一套骨架）----
 *
 * 布局照设计稿 index.html 的 .scr[data-scr="fan"]（和灯光页同一个"上中下三段"）：
 *   名字行（风图标 + 实体名）        y=66
 *   大号百分比 + "%"（单位）          y=136
 *   「风速」（设计稿 .l-label）        y=196
 *   胶囊行：预设 + 摇头 + 40x40 电源键  y=226
 *   提示行「旋转调风速 · 左滑返回」     y=282
 * 环：设计稿 r=156 / stroke 10 → LVGL 外径 161、线宽 10，环心 (180,180)；
 * 颜色 = 当前预设的颜色（直吹青 / 自然绿 / 循环蓝 / 睡眠灰），没有预设就青。
 *
 * 摇头：这台风扇 fan.xiaomi_cn_753076593_cfan_s_4_fan 的 supported_features=57
 * （TURN_ON|TURN_OFF|PRESET_MODE|SET_SPEED），**没有 OSCILLATE**，所以
 * 「摇头」胶囊按设计稿保留在原来的位置，但整颗压到 40%、不可点，代码里也没有
 * 任何 set_oscillating 的路径（选"可见地禁用"而不是删掉，三颗控件的排布保持
 * 和设计稿一致）。 */
#define FAN_R              161     /* 风速环外径 = 156 + 10/2（和灯同一个环）*/
#define FAN_W               10
#define FAN_GLOW_R         168     /* 弧外柔光（设计稿的 drop-shadow 近似）*/
#define FAN_GLOW_W          22
#define FAN_MODE_COUNT       4     /* 直吹 / 自然 / 循环 / 睡眠 */
#define FAN_PCT_STEP         5     /* 旋钮每格 5%（任务指定）*/
/* 胶囊（设计稿 .f-chip）：高 30、圆角 15、白 4.5% 底 + 白 7% 边；预设 94 宽、
   摇头 78 宽（设计稿 #fOsc min-width:78），间距 6，行顶 226。 */
#define FAN_CHIP_H          30
#define FAN_CHIP_PRESET_W   94
#define FAN_CHIP_OSC_W      78
#define FAN_CHIP_GAP         6
#define FAN_CHIP_ROW_Y     226
#define FAN_CHIP_DISABLED_OPA 102  /* 40%：不支持的胶囊压到这个亮度 */

static lv_obj_t *s_fan_arc;                  /* 风速环（可点可拖）*/
static lv_obj_t *s_fan_glow;                 /* 同一条环的光晕 */
static lv_obj_t *s_fan_name_icon;            /* 名字行左侧的风图标（设计稿 #i-wind）*/
static lv_obj_t *s_fan_pct;                  /* 大号百分比 */
static lv_obj_t *s_fan_unit;                 /* 百分比后面的 "%" */
static lv_obj_t *s_fan_label;                /* 「风速」（设计稿 .l-label）*/
static lv_obj_t *s_fan_chip_preset;          /* 预设胶囊（点一下换下一个）*/
static lv_obj_t *s_fan_chip_preset_lbl;
static lv_obj_t *s_fan_chip_osc;             /* 摇头胶囊（置灰：这台机器不支持）*/
static lv_obj_t *s_fan_chip_osc_lbl;
static bool s_detail_fan;                    /* 当前详情页开的是 fan.* */
static lv_color_t s_fan_accent;              /* 自绘图标读它（已过对比度处理）*/
/* 文字 / 配色缓存：500ms 轮询下没变就别再 set_text / 重排 */
static char s_fan_preset_shown[16];
static char s_fan_pct_txt[8];
static char s_fan_chip_txt[16];
static char s_fan_unit_shown[8];
static int s_fan_num_x = -1;
static int s_fan_unit_x = -1;
static int8_t s_fan_chip_shown = -1;
/* 本地覆盖：点了预设 / 转过旋钮之后先按本地值显示，HA 落定后交还（6s）*/
static char s_fan_preset_local[16];
static uint32_t s_fan_preset_local_until;
static int s_fan_pct_override;
static uint32_t s_fan_pct_override_until;

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
static void fan_apply_chips(const ha_entity_t *e, const char *preset, bool on);
static bool fan_knob_step(const ha_entity_t *e, int delta);
static uint32_t s_last_gesture_ms;   /* 最近一次手势时间，用来吃掉误触 */
static QueueHandle_t s_cmd_q;

typedef struct {
    char entity[96];
    char service[24];
    char attrs[64];      /* 非空则带参数调用（亮度/色温/空调） */
    int gen;             /* 保留字段（原折线图代际；历史曲线已取消，目前不用） */
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

/* 状态栏图标贴在 r=152 的圆上（设计稿 .h-status：四个图标按极坐标摆，不是弧形色带）*/
#define STATUS_ARC_R 152

/** 把对象按自身宽高居中到 (180,180)、半径 r 的圆上的 deg 度位置
 *  （LVGL 角度：0°=右、90°=下；-120° 等价于 240°）。 */
static void place_on_arc(lv_obj_t *obj, int deg)
{
    if (obj == NULL) {
        return;
    }
    lv_obj_update_layout(obj);        /* 先让标签算出宽高，才能真居中 */
    const int32_t x = (int32_t)STATUS_ARC_R * lv_trigo_cos((int16_t)deg);
    const int32_t y = (int32_t)STATUS_ARC_R * lv_trigo_sin((int16_t)deg);
    lv_obj_set_pos(obj, 180 + (int)(x >> LV_TRIGO_SHIFT) - lv_obj_get_width(obj) / 2,
                   180 + (int)(y >> LV_TRIGO_SHIFT) - lv_obj_get_height(obj) / 2);
}

/** 页面圆点（设计稿 .h-dots：top 294、间距 7；当前页是 14x5 的胶囊、其它是 5x5 点）。*/
static void dot_style(int cur_page, int i)
{
    lv_obj_t *d = s_dots[i];
    if (d == NULL) {
        return;
    }
    const bool act = (i == cur_page);
    const bool light_bg = is_assistant_page(cur_page);   /* 小智页纯白底：反色 */
    lv_obj_set_size(d, act ? 14 : 5, 5);
    lv_obj_set_style_radius(d, act ? 3 : LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(d, light_bg ? lv_color_hex(0x1C1C1E) : lv_color_white(), 0);
    lv_obj_set_style_bg_opa(d, act ? LV_OPA_COVER : (light_bg ? 90 : 46), 0);
    lv_obj_center(d);                 /* 尺寸变了仍然固定在 20x20 命中框正中 */
}

/** 标题位置：卡片页按设计稿 .h-title（top 56 → cjk16 盒子 y=48），
 *  设置页自己从 y=64 排，标题仍放 y=30 免得压住第一行。 */
static void title_place(int page)
{
    if (s_title == NULL) {
        return;
    }
    lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, is_settings_page(page) ? 64 : 48);
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
            dot_style(s_page, i);
        }
    }
    if (s_title) {
        const char *title = page_title(s_page);
        lv_label_set_text(s_title, title);
        lv_obj_set_style_text_font(s_title, title_font_for(title), 0);
        title_place(s_page);
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
            dot_style(page, i);
        }
    }
    if (s_title) {
        const char *title = page_title(page);
        lv_label_set_text(s_title, title);
        lv_obj_set_style_text_font(s_title, title_font_for(title), 0);
        title_place(page);
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
   子页面（详情 / 空调 / 天气）开着时不翻页，避免误触。 */
void ui_home_page_step(int delta)
{
    if (delta == 0 || s_page_count <= 0) return;
    /* 天气页把旋钮"借"去滚列表了：这里绝不能再翻页（main.c 已经先问过
       ui_home_detail_knob，双保险）*/
    if (ui_home_weather_open()) return;
    if (ui_home_detail_open() || ui_home_climate_open()) return;
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

    if (ui_home_weather_open()) {
        /* 天气页是整页滚动的覆盖层：只有左边缘右滑 = 返回 */
        if (dx > 40 && start.x < 70) {
            wx_close();
        }
        return;
    }
    if (s_sens_visible) {
        /* 传感器页：左边缘右滑关闭（没有拨盘，旋钮照常翻页） */
        if (dx > 40 && start.x < 70) {
            sensor_close();
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

    /* 天气：点开整页天气（实况 + 逐小时 + 逐日），它自己接管旋钮做页内滚动。
       注意要放在 SENSOR 分支之前：layout_type_of() 把 weather.* 归成 TOGGLE，
       直接 toggle 会发到 HA 报错。 */
    if (strncmp(w->entity, "weather.", 8) == 0) {
        wx_open(w->entity);
        return;
    }

    /* 传感器：点开"传感器详情页"（设计稿 屏幕 8）。设计稿已取消历史曲线，
       所以这里只开页、不取任何历史数据。 */
    if (w->type == TILE_SENSOR) {
        sensor_open(w->entity);
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

/* ---- 卡片：设计稿 .card（140x90 / 圆角 18 / 黑底半透明 / 图标左上 / 点右上 /
        26px 数值 + 15px 单位 / 16px 标题）------------------------------------ */

/** entity_id → 卡片域（只影响画法，不影响点击行为）。 */
static tile_dom_t tile_dom_of(const char *id)
{
    if (id == NULL) {
        return TILE_DOM_SWITCH;
    }
    if (strncmp(id, "light.", 6) == 0)        return TILE_DOM_LIGHT;
    if (strncmp(id, "climate.", 8) == 0)      return TILE_DOM_CLIMATE;
    if (strncmp(id, "fan.", 4) == 0)          return TILE_DOM_FAN;
    if (strncmp(id, "cover.", 6) == 0)        return TILE_DOM_COVER;
    if (strncmp(id, "media_player.", 13) == 0) return TILE_DOM_MEDIA;
    if (strncmp(id, "sensor.", 7) == 0)       return TILE_DOM_SENSOR;
    if (strncmp(id, "binary_sensor.", 14) == 0) return TILE_DOM_BINARY;
    if (strncmp(id, "weather.", 8) == 0)      return TILE_DOM_WEATHER;
    if (strncmp(id, "scene.", 6) == 0 || strncmp(id, "script.", 7) == 0) return TILE_DOM_SCENE;
    if (strncmp(id, "automation.", 11) == 0)  return TILE_DOM_AUTO;
    return TILE_DOM_SWITCH;
}

/** 语义色（设计稿 .card--* 的 --c）：图标 / 状态点 / 选中描边都用它。 */
static lv_color_t tile_dom_color(tile_dom_t dom)
{
    switch (dom) {
    case TILE_DOM_LIGHT:
    case TILE_DOM_BINARY:  return lv_color_hex(0xFFB020);   /* .card--light / binary */
    case TILE_DOM_CLIMATE:
    case TILE_DOM_SWITCH:  return lv_color_hex(0x4C8DFF);   /* .card--climate / switch */
    case TILE_DOM_COVER:   return lv_color_hex(0x2FD6C3);   /* .card--cover */
    case TILE_DOM_FAN:     return lv_color_hex(0x5BD97A);   /* .card--fan */
    case TILE_DOM_MEDIA:
    case TILE_DOM_SCENE:   return lv_color_hex(0xA78BFA);   /* .card--media / scene */
    case TILE_DOM_WEATHER: return lv_color_hex(0xFFC24B);   /* .card--weather */
    case TILE_DOM_AUTO:    return lv_color_hex(0x5BD97A);   /* .card--auto */
    case TILE_DOM_SENSOR:
    default:               return lv_color_hex(0xB9C2D0);   /* .card--sensor */
    }
}

/** 有没有右上角状态点：只读的 sensor / scene / weather 没有（设计稿一致）。 */
static bool tile_has_dot(tile_dom_t dom)
{
    return dom != TILE_DOM_SENSOR && dom != TILE_DOM_SCENE && dom != TILE_DOM_WEATHER;
}

/** 数值文字用哪套字体：纯 ASCII（数字/ON/OFF）用 montserrat_28，
 *  含中文（播放中 / 晴 16° / 已启用）退回 cjk16——大字体里没有汉字。 */
static const lv_font_t *value_font_for(const char *text)
{
    if (text != NULL) {
        for (const unsigned char *p = (const unsigned char *)text; *p; p++) {
            if (*p >= 0x80) {
                return &lv_font_cjk16;
            }
        }
    }
    return &lv_font_montserrat_28;
}

/** 数值 + 单位两个标签排成一行：单位紧跟在数值右边、低 10px（设计稿 .v small）。
 *  宽度用 lv_text_get_size 量，数值位数变了（9 → 100）单位跟着挪。 */
static void tile_place_value(tile_widget_t *w)
{
    if (w == NULL || w->value == NULL || w->unit == NULL) {
        return;
    }
    const char *vt = lv_label_get_text(w->value);
    const char *ut = lv_label_get_text(w->unit);
    if (vt == NULL || vt[0] == '\0' || ut == NULL || ut[0] == '\0') {
        return;
    }
    lv_point_t vsz = {0, 0};
    lv_text_get_size(&vsz, vt, lv_obj_get_style_text_font(w->value, 0), 0, 0,
                     LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    lv_obj_set_pos(w->unit, CARD_PAD_X + vsz.x + 2, 42);
}

static void tile_icon_draw_cb(lv_event_t *e);   /* 19x19 卡片图标（定义在空调自绘工具函数之后）*/
static void wx_bar(lv_layer_t *l, int x1, int y1, int x2, int y2, int r, lv_color_t c);

static void make_tile(lv_obj_t *parent, int page, int slot, int col, int row)
{
    const layout_tile_t *lt = &s_layout.pages[page].tiles[slot];
    tile_widget_t *w = &s_tiles[page][slot];

    memset(w, 0, sizeof(*w));
    snprintf(w->entity, sizeof(w->entity), "%s", lt->entity);
    w->type = lt->type;
    w->dom = tile_dom_of(lt->entity);
    w->icon_col = COLOR_MUTED;

    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, CARD_W, CARD_H);
    lv_obj_set_pos(card, col * (CARD_W + CARD_GAP), row * (CARD_H + CARD_GAP));
    /* 卡片是黑色半透明：叠在"背景图 + 暗化层"之上形成玻璃质感（设计稿 60~72%）*/
    lv_obj_set_style_bg_color(card, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(card, (lv_opa_t)(bg_image_card_opa_pct() * 255 / 100), 0);
    lv_obj_set_style_radius(card, CARD_RADIUS, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, lv_color_white(), 0);
    lv_obj_set_style_border_opa(card, 26, 0);              /* rgba(255,255,255,.10) */
    lv_obj_set_style_pad_all(card, 0, 0);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(card, tile_clicked_cb, LV_EVENT_CLICKED, w);

    /* 左上角图标 19x19（自绘，颜色随"开/关"变：关 #9AA4B5 / 开 语义色）*/
    lv_obj_t *icon = lv_obj_create(card);
    lv_obj_remove_style_all(icon);
    lv_obj_set_size(icon, CARD_ICON, CARD_ICON);
    lv_obj_set_pos(icon, CARD_PAD_X, 10);
    lv_obj_remove_flag(icon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(icon, tile_icon_draw_cb, LV_EVENT_DRAW_MAIN, w);

    /* 右上角状态点 5x5（设计稿 .dot margin-top:4 → y=14）*/
    if (tile_has_dot(w->dom)) {
        lv_obj_t *dot = lv_obj_create(card);
        lv_obj_remove_style_all(dot);
        lv_obj_set_size(dot, CARD_DOT, CARD_DOT);
        lv_obj_set_pos(dot, CARD_W - CARD_PAD_X - CARD_DOT, 14);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, lv_color_hex(0x626B7C), 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        w->dot = dot;
    }

    /* 数值（左 12 / 顶 32 / 宽 116）——26px 的近似：montserrat_28 */
    lv_obj_t *value = lv_label_create(card);
    lv_label_set_text(value, "--");
    lv_obj_set_style_text_font(value, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(value, lv_color_hex(0xF3F5F9), 0);
    lv_label_set_long_mode(value, LV_LABEL_LONG_DOT);
    lv_obj_set_width(value, CARD_W - CARD_PAD_X * 2);
    lv_obj_set_pos(value, CARD_PAD_X, 32);

    /* 单位（15px 的近似：montserrat_20，颜色 #97A1B3）*/
    lv_obj_t *unit = lv_label_create(card);
    lv_label_set_text(unit, "");
    lv_obj_set_style_text_font(unit, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(unit, lv_color_hex(0x97A1B3), 0);
    lv_obj_set_pos(unit, CARD_PAD_X, 42);
    lv_obj_add_flag(unit, LV_OBJ_FLAG_HIDDEN);

    /* 标题（左 12 / 底 8 → 顶 62，16px #8B95A8）*/
    lv_obj_t *label = lv_label_create(card);
    lv_label_set_text(label, lt->label[0] ? lt->label : lt->entity);
    lv_obj_set_style_text_font(label, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0x8B95A8), 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(label, CARD_W - CARD_PAD_X * 2);
    lv_obj_set_pos(label, CARD_PAD_X, 62);

    w->card = card;
    w->label = label;
    w->value = value;
    w->unit = unit;
    w->icon = icon;
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
/* 电量：只有"未知"态（这块板没有校准）                                        */
/* ------------------------------------------------------------------------- */
/* ESP32-S31 只有单档 ADC 衰减、没有校准方案，读到的只是原始值（约 4100-4160），
 * 不能换算成百分比、也不能换算成电压。所以界面统一显示"未知"态：
 * 状态栏电池图标 = 设计稿 #sbBatt.unknown 的样子（空壳 + 一道斜杠、颜色 #8B95A8），
 * 设置页那一行写"未知 raw N"（原始值留着，方便一眼看电池健康）。
 * 以后拿到校准参数：把 BATTERY_CALIBRATED 改成 1，在 battery_display_state() 里
 * 用原始值算出百分比，界面（图标填充 / 低电配色 / 百分比文字）不用改。 */
#define BATTERY_CALIBRATED 0

typedef enum {
    BATTERY_ST_UNKNOWN = 0,     /* 没有校准：只报"未知" */
    BATTERY_ST_PCT,             /* 有校准：pct 有效 */
} battery_state_t;

typedef struct {
    battery_state_t st;
    int pct;                    /* 0..100，仅 BATTERY_ST_PCT 有效 */
    int raw;                    /* ADC 原始值，无效为 -1 */
} battery_display_t;

static battery_display_t battery_display_state(void)
{
    battery_display_t b;
    b.st = BATTERY_ST_UNKNOWN;
    b.pct = -1;
    b.raw = battery_raw_value();
#if BATTERY_CALIBRATED
    /* 校准后的唯一改动点：raw -> pct，然后 b.st = BATTERY_ST_PCT; */
#endif
    return b;
}

static void battery_icon_draw_cb(lv_event_t *e);
static void ac_dr_line(lv_layer_t *layer, int x1, int y1, int x2, int y2, int w, lv_color_t c);

/** 状态栏电池图标（设计稿 #sbBatt）：空壳 + 正极帽；未知态再画一道斜杠。
 *  颜色：#8B95A8（unknown）/ #E4E9F2（有百分比时）。 */
static void battery_icon_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    if (obj == NULL || layer == NULL) {
        return;
    }
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    const battery_display_t b = battery_display_state();
    const lv_color_t c = (b.st == BATTERY_ST_PCT) ? lv_color_hex(0xE4E9F2) : lv_color_hex(0x8B95A8);

    /* 壳体：外框 16x10（22x22 画布里的 2..18 / 7..17），圆角 2.6 → 1.5px 描边 */
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_opa = LV_OPA_TRANSP;
    d.border_color = c;
    d.border_opa = LV_OPA_COVER;
    d.border_width = 2;
    d.radius = 3;
    lv_area_t shell = {(lv_coord_t)(a.x1 + 2), (lv_coord_t)(a.y1 + 7),
                       (lv_coord_t)(a.x1 + 18), (lv_coord_t)(a.y1 + 17)};
    lv_draw_rect(layer, &d, &shell);

    /* 正极帽 */
    wx_bar(layer, a.x1 + 19, a.y1 + 10, a.x1 + 21, a.y1 + 13, 1, c);

    if (b.st == BATTERY_ST_PCT) {
        /* 有百分比：按电量填格（最低留 2px，别画成空壳）*/
        int w = (14 * (b.pct < 0 ? 0 : (b.pct > 100 ? 100 : b.pct))) / 100;
        if (w < 2) {
            w = 2;
        }
        wx_bar(layer, a.x1 + 4, a.y1 + 9, a.x1 + 4 + w, a.y1 + 15, 1, c);
    } else {
        /* 未知：一道斜杠（设计稿 .slash: M4.4 17.4 L19.6 6.6）*/
        ac_dr_line(layer, a.x1 + 4, a.y1 + 16, a.x1 + 19, a.y1 + 7, 2, c);
    }
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
#define SET_ROW_W      252
#define SET_ROW_H       42    /* 设计稿 .s-row：42px，行间 1px 分隔线 */
#define SET_ROW_GAP      8
#define SET_ROW_PAD_X    0
#define SET_HEAD_H      28    /* 分组标题占位（设计稿没有分组，沿用它的字距/字色）*/
#define SET_SEC_GAP     14    /* 分组之间额外留白 */
#define SET_END_PAD     24    /* 滚动内容末尾留白，最后一行不贴圆边 */

/**
 * 设置页的一条设置行：设计稿 .s-row —— 透明底 + 高 42 + 底部 1px 分隔线
 * （rgba(255,255,255,.05) → 白色 opa 13），不再是一条一条的小卡片。失败返回 NULL。
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
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(row, 0, 0);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_color_white(), 0);
    lv_obj_set_style_border_opa(row, 13, 0);             /* rgba(255,255,255,.05) */
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    return row;
}

/** 行左侧标题（设计稿 .s-row 的 span：16px #B6C0D0，垂直居中）。 */
static lv_obj_t *settings_row_title(lv_obj_t *row, const char *text)
{
    lv_obj_t *lab = settings_label(row, text, 0, -1, 0, lv_color_hex(0xB6C0D0));
    if (lab != NULL) {
        lv_obj_align(lab, LV_ALIGN_LEFT_MID, SET_ROW_PAD_X, 0);
    }
    return lab;
}

/** 行右侧的值文字（设计稿 .s-row b：400、#F3F5F9、右对齐、垂直居中）；w 是最大宽度。 */
static lv_obj_t *settings_row_value(lv_obj_t *row, int w)
{
    lv_obj_t *lab = settings_label(row, "--", 0, -1, w, lv_color_hex(0xF3F5F9));
    if (lab != NULL) {
        lv_obj_set_style_text_align(lab, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(lab, LV_ALIGN_RIGHT_MID, -SET_ROW_PAD_X, 0);
    }
    return lab;
}

/** 分组标题：设计稿没有分组，沿用它的"小标题"语言（灰字 + 字距）。返回下一行的 y。 */
static int settings_heading(lv_obj_t *parent, int y, const char *text)
{
    if (parent != NULL) {
        lv_obj_t *lab = settings_label(parent, text, SET_ROW_PAD_X, y + 4, 0,
                                       lv_color_hex(0x7B8597));
        if (lab != NULL) {
            lv_obj_set_style_text_letter_space(lab, 2, 0);
        }
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
    lv_obj_set_style_bg_color(btn, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(btn, en ? 33 : 12, 0);          /* 可用 = 白 13%，禁用 = 白 4.5% */
    if (label != NULL) {
        lv_obj_set_style_text_color(label, en ? COLOR_TEXT : COLOR_MUTED, 0);
    }
}

/** 音频输出两个选项的高亮 + 「仍走耳机孔」提示的显隐。 */
static void settings_apply_audio(void)
{
    const bool bt = bt_link_audio_to_bt();
    /* 设计稿 .l-seg：选中 = 白 13% 底 + 近白字，未选 = 白 4.5% 底 + 灰字 */
    const lv_color_t on_bg = lv_color_white();
    const lv_color_t off_bg = lv_color_white();

    if (s_set_phone_btn != NULL) {
        lv_obj_set_style_bg_color(s_set_phone_btn, bt ? off_bg : on_bg, 0);
        lv_obj_set_style_bg_opa(s_set_phone_btn, bt ? 12 : 33, 0);
    }
    if (s_set_phone_label != NULL) {
        lv_obj_set_style_text_color(s_set_phone_label, bt ? COLOR_MUTED : COLOR_TEXT, 0);
    }
    if (s_set_bt_btn != NULL) {
        lv_obj_set_style_bg_color(s_set_bt_btn, bt ? on_bg : off_bg, 0);
        lv_obj_set_style_bg_opa(s_set_bt_btn, bt ? 33 : 12, 0);
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

    /* ---- 面板：设计稿 .s-panel（左/右 40、top 94、圆角 18、rgba(8,10,15,.55)、
       1px rgba(255,255,255,.055) 边、内边距 2/14 + 3px 自绘滚动条）。
       高度取 196（94..290）：比设计稿的 188 多 8px，给 20 多行设置多一点可视区。 */
    lv_obj_t *cont = lv_obj_create(page);
    if (cont == NULL) {
        report_page_create_failed("设置页滚动区", page);
        return;
    }
    lv_obj_remove_style_all(cont);
    lv_obj_set_pos(cont, 40, 94);
    lv_obj_set_size(cont, 280, 196);
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x080A0F), 0);
    lv_obj_set_style_bg_opa(cont, 189, 0);               /* .74：这是设计稿 glass-off 档，
                                                            亮壁纸上才读得清文字 */
    lv_obj_set_style_radius(cont, 18, 0);
    lv_obj_set_style_border_width(cont, 1, 0);
    lv_obj_set_style_border_color(cont, lv_color_white(), 0);
    lv_obj_set_style_border_opa(cont, 14, 0);            /* .055 */
    lv_obj_set_style_pad_hor(cont, 14, 0);
    lv_obj_set_style_pad_ver(cont, 2, 0);
    lv_obj_add_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);      /* 只上下滚，左右留给翻页 */
    lv_obj_set_scroll_chain(cont, false);         /* 滚到头不再把滚动传给父对象 */
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_AUTO);
    /* 设计稿 .s-bar：3px 宽、圆角 2、滑块 white 30% */
    lv_obj_set_style_width(cont, 3, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_color(cont, lv_color_white(), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(cont, 77, LV_PART_SCROLLBAR);        /* .30 */
    lv_obj_set_style_radius(cont, 2, LV_PART_SCROLLBAR);
    lv_obj_set_style_border_width(cont, 0, LV_PART_SCROLLBAR);
    lv_obj_set_style_pad_right(cont, 5, LV_PART_SCROLLBAR);
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
                    lv_obj_set_size(s_mem_bar, SET_ROW_W - SET_ROW_PAD_X * 2, 4);
                    lv_bar_set_range(s_mem_bar, 0, 100);
                    lv_obj_set_style_bg_color(s_mem_bar, lv_color_white(), LV_PART_MAIN);
                    lv_obj_set_style_bg_opa(s_mem_bar, 20, LV_PART_MAIN);     /* 轨道 white 8% */
                    lv_obj_set_style_radius(s_mem_bar, 2, LV_PART_MAIN);
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
        lv_obj_set_size(s_ota_bar, SET_ROW_W, 4);
        lv_bar_set_range(s_ota_bar, 0, 100);
        lv_obj_set_style_bg_color(s_ota_bar, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(s_ota_bar, 20, LV_PART_MAIN);      /* 轨道 white 8% */
        lv_obj_set_style_radius(s_ota_bar, 2, LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_ota_bar, COLOR_ACCENT, LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(s_ota_bar, LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_radius(s_ota_bar, 2, LV_PART_INDICATOR);
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
            lv_obj_set_style_radius(s_set_scan_btn, 15, 0);
            lv_obj_set_style_bg_color(s_set_scan_btn, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_set_scan_btn, 12, 0);   /* .045 */
            lv_obj_set_style_border_width(s_set_scan_btn, 1, 0);
            lv_obj_set_style_border_color(s_set_scan_btn, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_set_scan_btn, 18, 0); /* .07 */
            lv_obj_set_style_shadow_width(s_set_scan_btn, 0, 0);
            lv_obj_add_event_cb(s_set_scan_btn, settings_scan_cb, LV_EVENT_CLICKED, NULL);
            s_set_scan_label = settings_label(s_set_scan_btn, "扫描", -1, -1, 0, COLOR_TEXT);
        }

        s_set_conn_btn = lv_button_create(bt_btns);
        if (s_set_conn_btn != NULL) {
            lv_obj_set_size(s_set_conn_btn, 102, 30);
            lv_obj_set_pos(s_set_conn_btn, 132, 7);
            lv_obj_set_style_radius(s_set_conn_btn, 15, 0);
            lv_obj_set_style_bg_color(s_set_conn_btn, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_set_conn_btn, 12, 0);
            lv_obj_set_style_border_width(s_set_conn_btn, 1, 0);
            lv_obj_set_style_border_color(s_set_conn_btn, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_set_conn_btn, 18, 0);
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
            lv_obj_set_style_bg_color(s_set_phone_btn, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_set_phone_btn, 12, 0);
            lv_obj_set_style_border_width(s_set_phone_btn, 1, 0);
            lv_obj_set_style_border_color(s_set_phone_btn, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_set_phone_btn, 18, 0);
            lv_obj_set_style_shadow_width(s_set_phone_btn, 0, 0);
            lv_obj_add_event_cb(s_set_phone_btn, settings_audio_phone_cb, LV_EVENT_CLICKED, NULL);
            s_set_phone_label = settings_label(s_set_phone_btn, "耳机孔", -1, -1, 0, COLOR_TEXT);
        }

        s_set_bt_btn = lv_button_create(audio_row);
        if (s_set_bt_btn != NULL) {
            lv_obj_set_size(s_set_bt_btn, 108, 30);
            lv_obj_set_pos(s_set_bt_btn, 124, 7);
            lv_obj_set_style_radius(s_set_bt_btn, 15, 0);
            lv_obj_set_style_bg_color(s_set_bt_btn, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_set_bt_btn, 12, 0);
            lv_obj_set_style_border_width(s_set_bt_btn, 1, 0);
            lv_obj_set_style_border_color(s_set_bt_btn, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_set_bt_btn, 18, 0);
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
        const battery_display_t b = battery_display_state();
        if (b.st == BATTERY_ST_PCT) {
            snprintf(buf, sizeof(buf), "%d%%", b.pct);
        } else if (b.raw >= 0) {
            /* 没有校准 → 只报"未知"，原始值仍然留在这一行（看电池健康用）*/
            snprintf(buf, sizeof(buf), "未知 raw %d", b.raw);
        } else {
            snprintf(buf, sizeof(buf), "未知");
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

/** 把一张卡刷成当前实体状态（设计稿 .card：数值 + 单位 / 标题 / 图标 / 状态点）。
 *  行为不变：只读实体表，不点也不发命令。 */
static void tile_apply(tile_widget_t *w)
{
    if (w == NULL || w->value == NULL) {
        return;
    }
    const ha_entity_t *e = ha_client_find(w->entity);

    char val[48] = "--";
    char unit[12] = "";
    bool on = false;
    lv_color_t ink = lv_color_hex(0xF3F5F9);
    lv_color_t dom = tile_dom_color(w->dom);

    if (e != NULL) {
        const bool fresh = e->fresh && strcmp(e->state, "unavailable") != 0 &&
                           strcmp(e->state, "unknown") != 0;
        switch (w->dom) {
        case TILE_DOM_LIGHT:
            on = e->on;
            if (!fresh) {
                snprintf(val, sizeof(val), "--");
            } else if (!on) {
                snprintf(val, sizeof(val), "OFF");
                ink = COLOR_MUTED;
            } else {
                snprintf(val, sizeof(val), "%d", e->brightness * 100 / 255);
                snprintf(unit, sizeof(unit), "%%");
            }
            break;
        case TILE_DOM_FAN:
            on = e->on;
            if (!fresh) {
                snprintf(val, sizeof(val), "--");
            } else if (!on) {
                snprintf(val, sizeof(val), "OFF");
                ink = COLOR_MUTED;
            } else if (e->has_percentage) {
                snprintf(val, sizeof(val), "%d", e->percentage);
                snprintf(unit, sizeof(unit), "%%");
            } else {
                snprintf(val, sizeof(val), "ON");
            }
            break;
        case TILE_DOM_CLIMATE: {
            /* climate.* 的 state 是 cool/heat/off…，没有 "on" */
            on = fresh && strcmp(e->state, "off") != 0;
            if (!fresh) {
                snprintf(val, sizeof(val), "--");
            } else if (!on) {
                snprintf(val, sizeof(val), "OFF");
                ink = COLOR_MUTED;
            } else if (e->target_temp > 0.0f) {
                snprintf(val, sizeof(val), "%d", (int)(e->target_temp + 0.5f));
                snprintf(unit, sizeof(unit), "°");
            } else {
                snprintf(val, sizeof(val), "ON");
            }
            break;
        }
        case TILE_DOM_COVER:
            on = fresh && strcmp(e->state, "open") == 0;
            snprintf(val, sizeof(val), "%s", on ? "ON" : "OFF");
            if (!on) {
                ink = COLOR_MUTED;
            }
            break;
        case TILE_DOM_MEDIA: {
            const char *zh = "--";
            if (fresh) {
                if (strcmp(e->state, "playing") == 0)      zh = "播放中";
                else if (strcmp(e->state, "paused") == 0)  zh = "已暂停";
                else if (strcmp(e->state, "idle") == 0)    zh = "空闲";
                else if (strcmp(e->state, "off") == 0)     zh = "关机";
                else                                       zh = "开启";
            }
            snprintf(val, sizeof(val), "%s", zh);
            on = fresh && strcmp(e->state, "idle") != 0 && strcmp(e->state, "off") != 0;
            break;
        }
        case TILE_DOM_SENSOR:
            /* 只读：数值 + 单位（天气实体走下面那一档，显示"晴 16°"）*/
            if (!fresh) {
                snprintf(val, sizeof(val), "--");
            } else {
                snprintf(val, sizeof(val), "%s", e->state);
                snprintf(unit, sizeof(unit), "%s", e->unit);
            }
            break;
        case TILE_DOM_BINARY:
            on = fresh && e->on;
            snprintf(val, sizeof(val), "%s", on ? "ON" : "OFF");
            if (!on) {
                ink = COLOR_MUTED;
            }
            break;
        case TILE_DOM_WEATHER:
            if (!fresh || !e->wx_valid) {
                snprintf(val, sizeof(val), "--");
            } else {
                snprintf(val, sizeof(val), "%s %d°",
                         weather_state_zh(e->entity_id, e->state),
                         (e->wx_temp10 >= 0) ? ((e->wx_temp10 + 5) / 10)
                                             : ((e->wx_temp10 - 5) / 10));
            }
            break;
        case TILE_DOM_SCENE:
            snprintf(val, sizeof(val), "%s", fresh ? "执行" : "--");
            break;
        case TILE_DOM_AUTO:
            on = fresh && e->on;
            snprintf(val, sizeof(val), "%s", !fresh ? "--" : (on ? "已启用" : "已停用"));
            if (!on) {
                ink = COLOR_MUTED;
            }
            break;
        case TILE_DOM_SWITCH:
        default:
            on = fresh && e->on;
            snprintf(val, sizeof(val), "%s", fresh ? (on ? "ON" : "OFF") : "--");
            if (!on) {
                ink = COLOR_MUTED;
            }
            break;
        }
    }

    /* 数值 / 字体 / 单位（宽度变了才重排）*/
    if (strcmp(lv_label_get_text(w->value), val) != 0) {
        lv_label_set_text(w->value, val);
        lv_obj_set_style_text_font(w->value, value_font_for(val), 0);
        lv_obj_set_style_text_color(w->value, ink, 0);
    }
    lv_obj_set_style_text_color(w->value, ink, 0);
    if (w->unit != NULL) {
        if (strcmp(lv_label_get_text(w->unit), unit) != 0) {
            lv_label_set_text(w->unit, unit);
            detail_set_hidden(w->unit, unit[0] == '\0');
        }
        tile_place_value(w);
    }

    /* 图标：关 #9AA4B5 / 开 语义色（和设计稿 .card .ic / .card.on .ic 一致）*/
    if (w->icon != NULL) {
        if (w->dom == TILE_DOM_SENSOR) {
            w->icon_alt = (unit[0] == '%') ? 1 : 0;      /* 湿度画水滴，其它画温度计 */
        }
        w->icon_col = on ? dom : lv_color_hex(0x9AA4B5);
        lv_obj_invalidate(w->icon);
    }
    /* 状态点：关 #626B7C / 开 语义色；实体不新鲜时红点（沿用原来的"掉线"提示）*/
    if (w->dot != NULL) {
        lv_color_t dc = on ? dom : lv_color_hex(0x626B7C);
        if (e == NULL || !e->fresh) {
            dc = COLOR_BAD;
        }
        lv_obj_set_style_bg_color(w->dot, dc, 0);
    }
    /* 卡片描边：开 = 语义色 45%（设计稿 .card.on border-color: <c> 45%），关 = 白 10% */
    lv_obj_set_style_border_color(w->card, on ? dom : lv_color_white(), 0);
    lv_obj_set_style_border_opa(w->card, on ? 115 : 26, 0);
}

static void refresh_values(void)
{
    for (int p = 0; p < s_json_pages; p++) {
        for (int i = 0; i < s_page_tiles[p]; i++) {
            tile_apply(&s_tiles[p][i]);
        }
    }
}

/* 500ms 刷新定时器：状态栏颜色 / 卡片数值 / 传感器页 / 设置页（都在 LVGL 线程）。 */
static void refresh_timer_cb(lv_timer_t *t)
{
#if UI_ASSISTANT_PAGE
    refresh_selftest_label();      /* 小智页顶部「音频自检」按钮的文字/结果 */
#endif
    /* 旋钮最后值不再挂在这个 500ms 定时器上：由 s_knob_timer（50ms）负责补发 */
    if (s_detail_visible) {
        detail_apply_entity();     /* 子页数值跟着轮询刷新 */
    }
    sensor_refresh();              /* 传感器页的大号数值（不可见时直接返回）*/
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
    wx_refresh_ui();               /* 天气页开着时才真的刷（内部有可见性判断 + 判重）*/

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

static void worker_task(void *arg)
{
    (void)arg;
    while (true) {
        cmd_t cmd;
        if (xQueueReceive(s_cmd_q, &cmd, portMAX_DELAY) == pdTRUE) {
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

/** 中间的百分比文字（旋钮 / 拖动 / 轮询三条路径都走它）。
 *  亮度下限是 1%：0% 会被 Home Assistant 当成"关灯"，电源只归电源键。
 *  数字 / 单位 / 档位文字的排版与配色统一在 light_apply_values 里：
 *  它直接读圆环的当前值，所以这里只需要把环已经改好的值刷到屏幕上。 */
static void detail_set_br_text(int pct)
{
    if (s_detail_br_value == NULL) {
        return;
    }
    (void)pct;                        /* 值已经写进圆环，读环比传参更准 */
    light_apply_values(ha_client_find(s_detail_entity));
    detail_marker_update();          /* 焦点值点跟着亮度环当前值走 */
}

/** 中间的色温数字（同样交给 light_apply_values：单位是 K、档位文字是「色温」）。*/
static void detail_set_ct_text(int k)
{
    if (s_detail_ct_value == NULL) {
        return;
    }
    (void)k;
    light_apply_values(ha_client_find(s_detail_entity));
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
        if (s_detail_name_row != NULL) {
            lv_obj_add_flag(s_detail_name_row, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }
    char line[48];
    detail_name_line(name, line, sizeof(line));
    lv_label_set_text(s_detail_name, line);
    lv_obj_remove_flag(s_detail_name, LV_OBJ_FLAG_HIDDEN);
    if (s_detail_name_row != NULL) {
        lv_obj_remove_flag(s_detail_name_row, LV_OBJ_FLAG_HIDDEN);
    }
}

/* ------------------------------------------------------------------------- */
/* 灯光页：设计稿的"上中下三段式"（几何见文件头 LIGHT_* 宏）                    */
/*   y=66  名字行（灯泡图标 16 + 间距 6 + 实体名）                              */
/*   y=136 大号数值（montserrat_28）+ 单位（montserrat_20，低 8px）             */
/*   y=196 档位文字「亮度 / 色温」（cjk16，字距 4px）                           */
/*   y=226 控件行：分段控件（亮度/色温）+ 40x40 圆形电源键                      */
/*   y=282 提示行「旋转调节 · 左滑返回」                                        */
/* ------------------------------------------------------------------------- */

/** 氛围光圆盘：一个纯色圆（半径 r、透明度 opa），圆心 (cx,cy)。
 *  注意 lv_obj_remove_style_all() 会把 radius / bg / border 全部清掉，
 *  所以这里必须把半径、底色、透明度、边框宽度重新写一遍（这是"圆盘不显示"的第二个坑）。 */
static lv_obj_t *detail_make_glow_disc(lv_obj_t *parent, int r, lv_opa_t opa, int cx, int cy)
{
    lv_obj_t *o = lv_obj_create(parent);
    if (o == NULL) {
        return NULL;
    }
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, r * 2, r * 2);
    lv_obj_set_pos(o, cx - r, cy - r);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(o, lv_color_hex(0x4C8DFF), 0);
    lv_obj_set_style_bg_opa(o, opa, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_set_style_shadow_width(o, 0, 0);
    lv_obj_set_style_outline_width(o, 0, 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

/** 三层氛围光统一刷：显隐 + 颜色 + 透明度。
 *  base 用原始色相（灯色 / 预设色），关掉时压到 1/4，仍留一点"灯罩在发光"的感觉
 *  （设计稿 .is-off .light-bg{opacity:.12}）。 */
static void detail_apply_glow(lv_color_t base, bool on)
{
    for (int i = 0; i < GLOW_DISC_N; i++) {
        lv_obj_t *o = s_glow_disc[i];
        if (o == NULL) {
            continue;
        }
        detail_set_hidden(o, false);
        lv_obj_set_style_bg_color(o, base, 0);
        lv_obj_set_style_bg_opa(o, on ? k_glow_opa[i] : (lv_opa_t)(k_glow_opa[i] / 4), 0);
        lv_obj_invalidate(o);
    }
}

/** 灯的环 / 光晕以屏幕中心为心（detail_make_arc 是按 (DETAIL_CX,DETAIL_CY) 摆的）。*/
static void light_recenter(lv_obj_t *o, int r)
{
    if (o != NULL) {
        lv_obj_set_pos(o, DETAIL_CX - r, LIGHT_CY - r);
    }
}

/** 大号数值 + 单位：数字 montserrat_28、单位 montserrat_20（低 8px），
 *  两个标签一起水平居中（设计稿 .l-temp 的 flex 居中）。
 *  位置没变就不调 set_pos：否则 500ms 轮询会把两个标签每帧标脏重绘。 */
static void light_place_value(void)
{
    if (s_detail_br_value == NULL || s_detail_ct_value == NULL) {
        return;
    }
    lv_point_t nsz = {0, 0};
    lv_point_t usz = {0, 0};
    const char *ntxt = lv_label_get_text(s_detail_br_value);
    const char *utxt = lv_label_get_text(s_detail_ct_value);
    if (ntxt != NULL && ntxt[0] != '\0') {
        lv_text_get_size(&nsz, ntxt, lv_obj_get_style_text_font(s_detail_br_value, 0), 0, 0,
                     LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    }
    if (utxt != NULL && utxt[0] != '\0') {
        lv_text_get_size(&usz, utxt, &lv_font_montserrat_20, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    }
    const int gap = (usz.x > 0) ? 3 : 0;
    const int x0 = DETAIL_CX - (nsz.x + gap + usz.x) / 2;
    if (s_light_num_x != x0) {
        s_light_num_x = x0;
        lv_obj_set_pos(s_detail_br_value, x0, LIGHT_VALUE_Y);
    }
    const int ux = x0 + nsz.x + gap;
    if (s_light_unit_x != ux) {
        s_light_unit_x = ux;
        lv_obj_set_pos(s_detail_ct_value, ux, LIGHT_VALUE_Y + LIGHT_UNIT_DY);
    }
}

/** 灯光页的文案与配色：大号数值 / 单位 / 档位文字 / 提示行。
 *  数值直接读圆环（环已经由 apply_entity / 旋钮 / 拖动刷过），所以这里不碰实体值：
 *  "转动时先动数字、HA 落定后再对齐"的老规矩完全不变。
 *  关机态按设计稿 .scr.is-off：大号数字 "OFF"、档位文字 "已关闭" 并压暗。 */
static void light_apply_values(const ha_entity_t *e)
{
    if (s_detail_br_value == NULL || s_detail_ct_value == NULL) {
        return;
    }
    const bool on = detail_device_on(e);
    const bool has_tint = (e != NULL) && e->has_tint;
    const bool has_br = (e != NULL) && e->has_brightness;
    const bool ct = (s_detail_knob_target == DETAIL_KNOB_CT) && has_tint;
    const int pct = (s_detail_br_arc != NULL) ? (int)lv_arc_get_value(s_detail_br_arc) : 100;
    const int k = (s_detail_ct_arc != NULL) ? (int)lv_arc_get_value(s_detail_ct_arc) : 4000;

    char num[8];
    const char *unit = "";
    const char *label = "";
    lv_color_t num_col = lv_color_white();
    lv_color_t label_col = LIGHT_INK_MUTED;
    if (!on) {
        snprintf(num, sizeof(num), "OFF");
        label = "已关闭";
        num_col = LIGHT_OFF_NUM;
        label_col = LIGHT_OFF_LABEL;
    } else if (ct) {
        snprintf(num, sizeof(num), "%d", k);
        unit = "K";
        label = "色温";
        num_col = s_detail_accent;          /* 设计稿：色温档的数字用灯色 */
    } else {
        snprintf(num, sizeof(num), "%d", pct);
        unit = has_br ? "%" : "";
        label = has_br ? "亮度" : "开";
    }

    if (strcmp(num, s_light_num_shown) != 0) {
        snprintf(s_light_num_shown, sizeof(s_light_num_shown), "%s", num);
        lv_label_set_text(s_detail_br_value, num);
        /* 字体按内容选：数字走 64 号数字字体，OFF 走 montserrat_28（数字字体没有字母）*/
        lv_obj_set_style_text_font(s_detail_br_value, big_num_font_for(num), 0);
        light_place_value();                 /* 位数变了（9 → 100）要重新居中 */
    }
    if (strcmp(unit, s_light_unit_shown) != 0) {
        snprintf(s_light_unit_shown, sizeof(s_light_unit_shown), "%s", unit);
        lv_label_set_text(s_detail_ct_value, unit);
        light_place_value();
    }
    if (s_detail_off_hint != NULL && strcmp(label, s_light_label_shown) != 0) {
        snprintf(s_light_label_shown, sizeof(s_light_label_shown), "%s", label);
        lv_label_set_text(s_detail_off_hint, label);
    }
    lv_obj_set_style_text_color(s_detail_br_value, num_col, 0);
    lv_obj_set_style_text_color(s_detail_ct_value, lv_color_white(), 0);
    lv_obj_set_style_text_opa(s_detail_ct_value, on ? 184 : 120, 0);   /* .72 / 压暗 */
    if (s_detail_off_hint != NULL) {
        lv_obj_set_style_text_color(s_detail_off_hint, label_col, 0);
    }
    /* 名字行（设计稿 .l-name：#D7DEE9；关灯时压到 #5A6373）*/
    if (s_detail_name != NULL) {
        lv_obj_set_style_text_color(s_detail_name, on ? LIGHT_INK : LIGHT_ICON_OFF, 0);
    }
    detail_place_hint(LIGHT_HINT_Y);
    detail_set_hint_forced("旋转调节 · 左滑返回");
    if (s_detail_hint != NULL) {
        lv_obj_set_style_text_color(s_detail_hint, LIGHT_HINT_INK, 0);
    }
    detail_marker_update();                  /* 值点跟着环的当前值走 */
}

/** 分段控件 / 电源键的外观：
 *   选中项 = 白 13% 底 + 近白字（设计稿 .l-seg button.on）；
 *   未选   = 透明底 + #78829A 字；
 *   整组在关机时压到 42%（.scr.is-off .l-seg）；色温键在灯不支持色温时置灰；
 *   电源键：开 = 灯色实心 + 深色图标，关 = 白 6% 底 + 浅灰图标（.p-btn）。 */
static void light_apply_chips(bool on, bool has_tint, bool ct)
{
    const int8_t st = (int8_t)((on ? 1 : 0) | (has_tint ? 2 : 0) | (ct ? 4 : 0));
    if (st != s_light_chip_shown) {
        s_light_chip_shown = st;
        for (int i = 0; i < 2; i++) {
            lv_obj_t *btn = s_light_seg_btn[i];
            lv_obj_t *lbl = s_light_seg_lbl[i];
            if (btn == NULL) {
                continue;
            }
            const bool sel = (i == 1) ? ct : !ct;
            const bool usable = (i == 0) || has_tint;
            lv_obj_set_style_bg_color(btn, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(btn, sel ? 33 : LV_OPA_TRANSP, 0);   /* 13% */
            if (lbl != NULL) {
                lv_obj_set_style_text_color(lbl, sel ? LIGHT_SEG_TXT_ON : LIGHT_SEG_TXT, 0);
                lv_obj_set_style_text_opa(lbl, usable ? LV_OPA_COVER : LV_OPA_40, 0);
            }
        }
        if (s_light_seg != NULL) {
            lv_obj_set_style_opa(s_light_seg, on ? LV_OPA_COVER : 107, 0);   /* .42 */
        }
    }
    s_dial_pwr_on = on;
    if (s_dial_power != NULL) {
        lv_obj_set_style_bg_color(s_dial_power, on ? s_detail_accent : lv_color_white(), 0);
        lv_obj_set_style_bg_opa(s_dial_power, on ? LV_OPA_COVER : 15, 0);        /* 6% */
        lv_obj_set_style_border_opa(s_dial_power, on ? LV_OPA_TRANSP : 23, 0);   /* 9% */
        lv_obj_invalidate(s_dial_power);        /* 图标颜色跟着开关状态变 */
    }
}

/** 提示行文字：绕开 detail_set_hint 的"文字缓存"（空调/风扇也用同一行，
 *  如果只改 label 不改缓存，切页后那边会以为文字没变而不刷新）。 */
static void detail_set_hint_forced(const char *text)
{
    if (s_detail_hint == NULL || text == NULL) {
        return;
    }
    if (strcmp(text, s_detail_hint_shown) == 0) {
        lv_obj_remove_flag(s_detail_hint, LV_OBJ_FLAG_HIDDEN);
        return;                          /* 文字没变：500ms 轮询不重排这一行 */
    }
    snprintf(s_detail_hint_shown, sizeof(s_detail_hint_shown), "%.15s", text);
    lv_label_set_text(s_detail_hint, text);
    lv_obj_remove_flag(s_detail_hint, LV_OBJ_FLAG_HIDDEN);
}

/** 提示行的纵向位置：灯 / 风扇 282，空调 246（没变就不重摆，省一次重绘）。*/
static void detail_place_hint(int y)
{
    if (s_detail_hint == NULL || s_hint_y_shown == y) {
        return;
    }
    s_hint_y_shown = (int8_t)y;
    lv_obj_align(s_detail_hint, LV_ALIGN_TOP_MID, 0, y);
}

/** 共用电源键的位置 / 尺寸：灯 40x40 @232、风扇 40x40 @252、空调 44x44 @(261,179)。
 *  size 传 0 表示沿用上一次的尺寸（灯 / 风扇都是 40）。 */
static void detail_place_power(int size, int x, int y)
{
    if (s_dial_power == NULL) {
        return;
    }
    if (s_pwr_size_shown != size) {
        s_pwr_size_shown = size;
        lv_obj_set_size(s_dial_power, size, size);
        s_pwr_x_shown = -1;                /* 尺寸变了，位置必须重设 */
        s_pwr_y_shown = -1;
    }
    if (s_pwr_x_shown != x || s_pwr_y_shown != y) {
        s_pwr_x_shown = x;
        s_pwr_y_shown = y;
        lv_obj_set_pos(s_dial_power, x, y);
    }
}

/** 名字行的位置 / 宽度跟着页面走：灯 = 图标 + 名字一行居中，空调 y=134 宽 120。*/
static void detail_place_name(bool ac)
{
    if (s_detail_name_row == NULL) {
        return;
    }
    /* 整行固定 x=60..300，纵向按页面放：灯 / 风扇 y=66，空调 y=134。
       图标显隐：灯 = 灯泡、风扇 = 风图标、空调两个都不要（只留名字）。 */
    lv_obj_set_pos(s_detail_name_row, 60, ac ? AC_NAME_ROW_Y : LIGHT_NAME_Y);
    const bool show_bulb = !ac && !s_detail_fan;
    const bool show_wind = !ac && s_detail_fan;
    if (s_detail_icon != NULL) {
        if (show_bulb) {
            lv_obj_remove_flag(s_detail_icon, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_detail_icon, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_fan_name_icon != NULL) {
        if (show_wind) {
            lv_obj_remove_flag(s_fan_name_icon, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_fan_name_icon, LV_OBJ_FLAG_HIDDEN);
        }
    }
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

/** 统一的电源动作：开关（空调 / 风扇）和灯光页的圆形电源键都走这里。
 *  和原来 detail_power_changed_cb 的逻辑一模一样，只是入口多了一个。 */
static void detail_power_set(bool want_on)
{
    s_sw_override_on = want_on;
    s_sw_override_until = lv_tick_get() + OVERRIDE_MS;
    s_sw_on_shown = -1;               /* CHECKED 状态交给下一次 detail_apply_entity 对齐 */
    s_sw_on_now = want_on;

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
        }
    } else if (s_detail_ac) {
        /* 开机：恢复到 HA 记着的那个模式（等它下一次 state_changed / 轮询）*/
        s_ac_mode_local_until = 0;
    }

    if (s_detail_ac) {
        detail_apply_switch_style(want_on);
    }
    {
        /* 灯 / 风扇 / 空调都是圆形电源键：底色 / 环隐现 / 电源键配色，
           全在这一帧刷完（空调还要刷副标题和模式键）*/
        s_detail_fg_dirty = true;
        s_light_chip_shown = -1;
        s_fan_chip_shown = -1;
        detail_update_ambience();
        detail_apply_focus();
        if (s_detail_ac) {
            const ha_entity_t *ent = ha_client_find(s_detail_entity);
            if (ent != NULL) {
                ac_apply_entity(ent);
            }
        }
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

/** 灯光页电源键：点一下切换开 / 关（设计稿 .p-btn）。 */
static void light_power_cb(lv_event_t *e)
{
    (void)e;
    const ha_entity_t *ent = ha_client_find(s_detail_entity);
    bool on;
    if (s_detail_ac) {
        /* 空调的 state 是 cool/heat/…/off，没有 "on"：按"模式 != off"判 */
        on = (ent != NULL) && !ac_state_is_off(ac_state_now(ent));
    } else {
        on = detail_device_on(ent);
    }
    detail_power_set(!on);
}

/** 分段控件：亮度 / 色温（设计稿 .l-seg button）。user_data = 0 亮度 / 1 色温。 */
static void light_tab_cb(lv_event_t *e)
{
    const int idx = (int)(intptr_t)lv_event_get_user_data(e);
    const ha_entity_t *ent = ha_client_find(s_detail_entity);
    if (idx == 1 && (ent == NULL || !ent->has_tint)) {
        return;                        /* 这只灯不支持色温：按键是灰的，点了也不动 */
    }
    s_detail_knob_target = (idx == 1) ? DETAIL_KNOB_CT : DETAIL_KNOB_BR;
    ESP_LOGI(TAG, "灯光页切换档位: %s", idx == 1 ? "色温" : "亮度");
    s_light_chip_shown = -1;           /* 强制刷一次高亮 */
    detail_apply_focus();
}

/** 电源图标：缺口圆环 + 顶部竖线（设计稿 #i-power，19px / stroke 1.9）。*/
static void light_power_draw_cb(lv_event_t *e)
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
    const lv_color_t col = s_dial_pwr_on ? LIGHT_PWR_INK_ON : LIGHT_PWR_INK;

    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.color = col;
    d.opa = LV_OPA_COVER;
    d.width = 2;
    d.rounded = 1;
    d.radius = 7;
    d.center.x = cx;
    d.center.y = cy + 1;
    d.start_angle = 305;               /* 缺口朝上：305° → 235° */
    d.end_angle = 235;
    lv_draw_arc(layer, &d);

    lv_draw_rect_dsc_t r;
    lv_draw_rect_dsc_init(&r);
    r.bg_color = col;
    r.bg_opa = LV_OPA_COVER;
    r.radius = 1;
    lv_area_t bar = {(lv_coord_t)(cx - 1), (lv_coord_t)(cy - 7),
                     (lv_coord_t)(cx + 1), (lv_coord_t)(cy + 1)};
    lv_draw_rect(layer, &r, &bar);
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
        /* 氛围光必须放在"预设没变就 return"之前：否则第二次进风扇页时
           这段直接跳过，三个圆盘永远停在 detail_hide_light(true) 藏起来的状态。 */
        detail_apply_glow(fan_preset_color(preset), detail_device_on(e));
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
        if (s_fan_name_icon) {
            lv_obj_invalidate(s_fan_name_icon);      /* 自绘图标：颜色变了得手动失效 */
        }
        /* 胶囊（预设名 / 电源键开色）跟着预设走 */
        s_fan_chip_shown = -1;
        fan_apply_chips(e, preset, detail_device_on(e));
        /* 氛围光 = 预设色的原始色相（设计稿 .light-bg 在风扇页同样存在）*/
        detail_apply_glow(fan_preset_color(preset), detail_device_on(e));
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

    /* 氛围光（设计稿 .light-bg）：内圈灯色 22% + 中圈 11% + 外圈 4.7%，关灯压到 1/4。
       颜色用**原始灯色**（detail_light_base），不用对比度处理后的 accent ——
       accent 会在亮底上被推成灰白，看起来就像"没有光晕"。 */
    detail_apply_glow(detail_light_base(), on);
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
        const bool ct = (s_detail_knob_target == DETAIL_KNOB_CT) && (e != NULL) && e->has_tint;
        ring = ct ? s_detail_ct_arc : s_detail_br_arc;
        radius = LIGHT_R - LIGHT_W / 2;      /* 灯页单环：r=156 的中线 */
        col = ct ? detail_on_bg(kelvin_to_color(detail_kelvin_now(e))) : s_detail_accent;
        if (!detail_device_on(e)) {
            ring = NULL;                      /* 关灯：整组圆盘是藏着的 */
        }
    }

    if (!detail_ring_visible(ring)) {
        lv_obj_add_flag(s_detail_marker, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    /* 灯光页的值点按设计稿放大到 14px（r=7.5），空调 / 风扇保持 10px */
    const int msz = (!s_detail_ac && !s_detail_fan) ? LIGHT_MARKER_SIZE : DETAIL_MARKER_SIZE;
    static int msz_shown;
    if (msz_shown != msz) {
        msz_shown = msz;
        lv_obj_set_size(s_detail_marker, msz, msz);
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
    /* 灯 / 风扇的环心是屏幕中心 (180,180)，空调是 (180,196) */
    const int cy = LIGHT_CY;      /* 三个页面现在都以屏幕中心 (180,180) 为环心 */
    const int mx = DETAIL_CX + (int)(((int32_t)radius * lv_trigo_cos((int16_t)deg)) >> LV_TRIGO_SHIFT);
    const int my = cy + (int)(((int32_t)radius * lv_trigo_sin((int16_t)deg)) >> LV_TRIGO_SHIFT);
    lv_obj_set_pos(s_detail_marker, mx - msz / 2, my - msz / 2);
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
        /* 风扇页（设计稿 .scr[data-scr="fan"]）：单环 + 预设胶囊 + 置灰的摇头胶囊
           + 圆形电源键。开着才有光晕 / 值点；关掉时环压暗、不可点——
           fan.set_percentage 会顺带开机，电源只归电源键（刻度还在，只是拨不动）。 */
        const ha_entity_t *e = ha_client_find(s_detail_entity);
        const bool on = detail_device_on(e);
        if (s_fan_arc) {
            lv_obj_set_style_arc_width(s_fan_arc, FAN_W, LV_PART_INDICATOR);
            lv_obj_set_style_arc_opa(s_fan_arc, on ? LV_OPA_COVER : DETAIL_DIM_OPA,
                                     LV_PART_INDICATOR);
            lv_obj_set_style_arc_opa(s_fan_arc, on ? DETAIL_TRACK_OPA : DETAIL_TRACK_DIM_OPA,
                                     LV_PART_MAIN);
            lv_obj_set_clickable(s_fan_arc, on);
        }
        if (s_fan_glow) {
            lv_obj_set_style_arc_opa(s_fan_glow, on ? 26 : LV_OPA_TRANSP, LV_PART_INDICATOR);
        }
        if (s_fan_pct) {
            lv_obj_set_style_text_opa(s_fan_pct, on ? LV_OPA_COVER : DETAIL_DIM_OPA, 0);
            lv_obj_set_style_text_color(s_fan_pct, on ? fg : muted, 0);
        }
        if (s_fan_unit) {
            lv_obj_set_style_text_opa(s_fan_unit, on ? 184 : 120, 0);
        }
        if (s_fan_label) {
            lv_obj_set_style_text_color(s_fan_label, on ? LIGHT_INK_MUTED : LIGHT_OFF_LABEL, 0);
        }
        if (s_detail_name != NULL) {
            /* 名字行（设计稿 .l-name）：开 = #D7DEE9，关 = 压暗 */
            lv_obj_set_style_text_color(s_detail_name, on ? LIGHT_INK : LIGHT_ICON_OFF, 0);
        }
        /* 控件行：预设 / 摇头 胶囊 + 共用电源键（x=252，见 FAN_CHIP_*）*/
        detail_set_hidden(s_light_seg, true);
        detail_set_hidden(s_dial_power, false);
        detail_place_power(LIGHT_POWER, 252, LIGHT_SEG_Y + (LIGHT_SEG_H - LIGHT_POWER) / 2);
        fan_apply_chips(e, fan_preset_now(e), on);
        /* 氛围光（.light-bg）：预设色原始色相，三层同心圆 */
        detail_apply_glow(fan_preset_color(fan_preset_now(e)), on);
        detail_place_hint(LIGHT_HINT_Y);
        detail_set_hint_forced("旋转调风速 · 左滑返回");
        if (s_detail_hint != NULL) {
            lv_obj_set_style_text_color(s_detail_hint, LIGHT_HINT_INK, 0);
        }
        detail_marker_update();
        return;
    }

    if (s_detail_ac) {
        /* 空调页（设计稿 dial 帧）：单环 10px + 光晕，和灯 / 风扇同一套观感。
           开着才有光晕 / 值点；关掉环压暗、不可点（旋钮不发温度）。 */
        const ha_entity_t *e = ha_client_find(s_detail_entity);
        const bool on = (e != NULL) && !ac_state_is_off(ac_state_now(e));
        if (s_ac_arc) {
            lv_obj_set_style_arc_width(s_ac_arc, AC_TEMP_W, LV_PART_INDICATOR);
            lv_obj_set_style_arc_opa(s_ac_arc, on ? LV_OPA_COVER : DETAIL_DIM_OPA,
                                     LV_PART_INDICATOR);
            lv_obj_set_style_arc_opa(s_ac_arc, on ? DETAIL_TRACK_OPA : DETAIL_TRACK_DIM_OPA,
                                     LV_PART_MAIN);
            lv_obj_set_clickable(s_ac_arc, on);
        }
        if (s_ac_glow) {
            lv_obj_set_style_arc_opa(s_ac_glow, on ? 26 : LV_OPA_TRANSP, LV_PART_INDICATOR);
        }
        if (s_ac_temp_value) {
            lv_obj_set_style_text_opa(s_ac_temp_value, on ? LV_OPA_COVER : DETAIL_DIM_OPA, 0);
            lv_obj_set_style_text_color(s_ac_temp_value, on ? fg : muted, 0);
        }
        if (s_ac_unit) {
            lv_obj_set_style_text_opa(s_ac_unit, on ? LV_OPA_COVER : DETAIL_DIM_OPA, 0);
        }
        /* 模式键 / 摆风胶囊 / 副标题：跟着模式色和开关状态刷 */
        if (e != NULL) {
            ac_refresh_chips(e, ac_state_now(e));
        }
        detail_place_hint(LIGHT_HINT_Y);
        detail_set_hint_forced("旋转调温 · 左滑返回");
        if (s_detail_hint != NULL) {
            lv_obj_set_style_text_color(s_detail_hint, LIGHT_HINT_INK, 0);
        }
        detail_marker_update();
        return;
    }

    /* ---- 灯光页：单环（设计稿只有一条 r=156 的环）+ 分段控件 ----
       焦点 = 分段控件选中的那一项（亮度 / 色温）：两个值共用同一条环，
       只显示当前那一环（另一环连对象一起藏起来，也就不会被手指点到）。 */
    const ha_entity_t *le = ha_client_find(s_detail_entity);
    const bool on = detail_device_on(le);
    const bool has_tint = (le != NULL) && le->has_tint;
    const bool has_br = (le != NULL) && le->has_brightness;
    if (!has_tint) {
        s_detail_knob_target = DETAIL_KNOB_BR;   /* 没有色温能力：只能调亮度 */
    }
    const bool ct = (s_detail_knob_target == DETAIL_KNOB_CT) && has_tint;
    const bool br_show = on && has_br && !ct;
    const bool ct_show = on && has_tint && ct;

    detail_set_hidden(s_detail_br_arc, !br_show);
    detail_set_hidden(s_detail_br_glow, !br_show);
    detail_set_hidden(s_detail_ct_arc, !ct_show);
    detail_set_hidden(s_detail_ct_glow, !ct_show);
    if (s_detail_br_arc) {
        lv_obj_set_clickable(s_detail_br_arc, br_show);
    }
    if (s_detail_ct_arc) {
        lv_obj_set_clickable(s_detail_ct_arc, ct_show);
    }
    if (s_detail_br_arc) {
        lv_obj_set_style_arc_width(s_detail_br_arc, LIGHT_W, LV_PART_INDICATOR);
        lv_obj_set_style_arc_opa(s_detail_br_arc, LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_arc_opa(s_detail_br_arc, DETAIL_TRACK_OPA, LV_PART_MAIN);
    }
    if (s_detail_br_glow) {
        lv_obj_set_style_arc_opa(s_detail_br_glow, br_show ? 26 : LV_OPA_TRANSP,
                                 LV_PART_INDICATOR);
    }
    if (s_detail_ct_glow) {
        lv_obj_set_style_arc_opa(s_detail_ct_glow, ct_show ? 26 : LV_OPA_TRANSP,
                                 LV_PART_INDICATOR);
    }
    if (s_detail_ct_arc) {
        lv_obj_set_style_arc_opa(s_detail_ct_arc, DETAIL_TRACK_OPA, LV_PART_MAIN);
        /* 色温环的渐变是自绘的：切档 / 换灯都要重画 */
        lv_obj_invalidate(s_detail_ct_arc);
    }

    light_apply_chips(on, has_tint, ct);
    light_apply_values(le);
    detail_place_power(LIGHT_POWER, 232, LIGHT_SEG_Y + (LIGHT_SEG_H - LIGHT_POWER) / 2);
    detail_set_hidden(s_dial_power, false);
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
    d.opa = LV_OPA_COVER;
    /* 设计稿里亮度 / 色温共用同一条 10px 的环（不再分粗细）*/
    d.width = LIGHT_W;

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

/** 名字行左侧那颗灯泡（设计稿 .l-name .ic，16x16、颜色 #8D97A8 / 关灯 #5A6373）：
 *  玻璃泡 = 圆，灯颈 / 灯座 = 两个小圆角矩形，三笔就够。 */
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
    const bool on = detail_device_on(ha_client_find(s_detail_entity));

    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = on ? LIGHT_ICON_INK : LIGHT_ICON_OFF;
    d.bg_opa = LV_OPA_COVER;

    /* 16x16 画布：玻璃泡 3..12 / y 0..9，灯颈 y 9..11，灯座 y 11..14 */
    lv_area_t glass = {(lv_coord_t)(cx - 5), a.y1, (lv_coord_t)(cx + 5), (lv_coord_t)(a.y1 + 9)};
    d.radius = 5;
    lv_draw_rect(layer, &d, &glass);

    lv_area_t neck = {(lv_coord_t)(cx - 2), (lv_coord_t)(a.y1 + 9),
                      (lv_coord_t)(cx + 2), (lv_coord_t)(a.y1 + 11)};
    d.radius = 1;
    lv_draw_rect(layer, &d, &neck);

    lv_area_t base = {(lv_coord_t)(cx - 3), (lv_coord_t)(a.y1 + 11),
                      (lv_coord_t)(cx + 3), (lv_coord_t)(a.y1 + 14)};
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
    /* 名字行的位置依赖"文字实际宽度"：必须**先写文字再摆位置**（旧顺序是先摆后写，
       量到的是上一次的文字宽度，标签只有几像素宽 → 名字被挤到中线右边）。 */
    detail_place_name(s_detail_ac);
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
    /* 灯 / 风扇按设计稿都是 40x40 圆形电源键，开关只剩空调页在用 */
    if (s_detail_power_sw != NULL && !s_detail_ac) {
        detail_set_hidden(s_detail_power_sw, true);
    }
    if (s_detail_power_sw != NULL && s_detail_ac) {
        /* 空调页现在也用 44x44 圆形电源键（设计稿 .d-power），开关彻底不用了 */
        detail_set_hidden(s_detail_power_sw, true);
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
        fan_hide_all(true);                      /* ← 之前漏了：风扇组会叠在空调页上 */
        detail_set_hidden(s_dial_power, true);   /* 空调页只有开关，没有圆形电源键 */
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

    /* ---------------- 灯光页（设计稿 .scr[data-scr="light"]）----------------
       单环 + 分段控件：亮度和色温共用同一条 r=156 的环，只显示当前那一档；
       大号数字 / 单位 / 档位文字 / 提示行由 light_apply_values 统一刷。
       灯关着时数字还是显示的（设计稿写 "OFF"），所以这里不再整组藏起来。 */
    ac_hide_all(true);                        /* 空调那组藏起来 */
    fan_hide_all(true);                       /* 风扇那组藏起来 */
    detail_set_hidden(s_light_seg, false);
    detail_update_ambience();

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
    }
    /* 名字行左侧的灯泡：设计稿里关灯也留着（只是压暗），所以不跟着藏 */
    detail_set_hidden(s_detail_icon, false);
    detail_set_hidden(s_detail_br_value, false);
    detail_set_hidden(s_detail_ct_value, false);
    detail_set_hidden(s_detail_off_hint, false);
    detail_apply_focus();                     /* 环的隐现 / 分段控件 / 电源键 / 数字 */
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
/* 自绘工具函数在文件后面定义，这里先声明（模式图标要先用到）*/
static void ac_dr_dot(lv_layer_t *layer, int x, int y, int r, lv_color_t c);
static void ac_dr_line(lv_layer_t *layer, int x1, int y1, int x2, int y2, int w, lv_color_t c);
static void ac_dr_arc(lv_layer_t *layer, int cx, int cy, int r, int w, int a1, int a2, lv_color_t c);
static void ac_dir_point(int cx, int cy, int dir12, int dist, int *x, int *y);
static bool ac_swing_now(const ha_entity_t *e);

/* ---- 模式图标（自绘）：28px 的设计按 u 缩放，名字行(16) 和模式键(20) 共用 ---- */
static void ac_glyph_draw(lv_layer_t *l, int cx, int cy, int u, int mode, lv_color_t c)
{
    const int R = (u * 11) / 28;            /* 设计稿 28px 画布里最大半径 11 */
    const int lw = (u >= 24) ? 3 : 2;
    const int h = (R * 3) / 5;              /* 半臂长 */
    switch (mode) {
    case 0:                                   /* 自动：A 字（两撇 + 横杠）*/
        ac_dr_line(l, cx - h, cy + R, cx, cy - R, lw, c);
        ac_dr_line(l, cx, cy - R, cx + h, cy + R, lw, c);
        ac_dr_line(l, cx - (h * 2) / 3, cy + (R * 2) / 5, cx + (h * 2) / 3, cy + (R * 2) / 5, lw, c);
        break;
    case 1:                                   /* 制冷：雪花（三条交叉线 + 短枝）*/
        for (int i = 0; i < 12; i += 4) {
            int x1, y1, x2, y2;
            ac_dir_point(cx, cy, i, R, &x1, &y1);
            ac_dir_point(cx, cy, i + 6, R, &x2, &y2);
            ac_dr_line(l, x1, y1, x2, y2, lw, c);
        }
        break;
    case 2:                                   /* 制热：太阳（圆 + 六道光）*/
        ac_dr_dot(l, cx, cy, (R * 2) / 5, c);
        for (int i = 0; i < 12; i += 2) {
            int x1, y1, x2, y2;
            ac_dir_point(cx, cy, i, (R * 3) / 5, &x1, &y1);
            ac_dir_point(cx, cy, i, R, &x2, &y2);
            ac_dr_line(l, x1, y1, x2, y2, lw, c);
        }
        break;
    case 3:                                   /* 除湿：水滴 */
        ac_dr_dot(l, cx, cy + (R * 2) / 5, (R * 2) / 3, c);
        ac_dr_line(l, cx, cy - R, cx - (R * 2) / 3, cy + R / 4, lw, c);
        ac_dr_line(l, cx, cy - R, cx + (R * 2) / 3, cy + R / 4, lw, c);
        break;
    default:                                  /* 送风：护环 + 三片叶 + 轮毂 */
        ac_dr_arc(l, cx, cy, R, lw, 0, 360, c);
        for (int i = 0; i < 12; i += 4) {
            int x1, y1, x2, y2;
            ac_dir_point(cx, cy, i, (R * 2) / 5, &x1, &y1);
            ac_dir_point(cx, cy, i, (R * 4) / 5, &x2, &y2);
            ac_dr_line(l, x1, y1, x2, y2, lw + 1, c);
        }
        ac_dr_dot(l, cx, cy, (R * 2) / 5, c);
        break;
    }
}

/** 模式键里的图标（user_data = k_ac_modes 下标，颜色由 ac_refresh_chips 刷进 s_ac_key_col）。 */
static void ac_key_icon_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    const int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (obj == NULL || layer == NULL || idx < 0 || idx >= AC_MODE_COUNT) {
        return;
    }
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    const int u = a.x2 - a.x1 + 1;
    ac_glyph_draw(layer, (a.x1 + a.x2) / 2, (a.y1 + a.y2) / 2, u, idx, s_ac_key_col[idx]);
}

/** 名字行左侧的模式图标（当前模式，16px，颜色 = 模式色）。 */
static void ac_name_icon_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    if (obj == NULL || layer == NULL) {
        return;
    }
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    int mode = 4;
    for (int i = 0; i < AC_MODE_COUNT; i++) {
        if (strcmp(ac_state_now(ha_client_find(s_detail_entity)), k_ac_modes[i].mode) == 0) {
            mode = i;
            break;
        }
    }
    ac_glyph_draw(layer, (a.x1 + a.x2) / 2, (a.y1 + a.y2) / 2, a.x2 - a.x1 + 1, mode,
                  s_ac_accent);
}

/** 大号目标温度 + ° 一起水平居中（数字 num64、° 用 montserrat_20，字墨中心对齐）。 */
static void ac_place_value(void)
{
    if (s_ac_temp_value == NULL || s_ac_unit == NULL) {
        return;
    }
    lv_point_t nsz = {0, 0};
    lv_point_t usz = {0, 0};
    const char *ntxt = lv_label_get_text(s_ac_temp_value);
    const char *utxt = lv_label_get_text(s_ac_unit);
    if (ntxt != NULL && ntxt[0] != '\0') {
        lv_text_get_size(&nsz, ntxt, lv_obj_get_style_text_font(s_ac_temp_value, 0), 0, 0,
                         LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    }
    if (utxt != NULL && utxt[0] != '\0') {
        lv_text_get_size(&usz, utxt, &lv_font_montserrat_20, 0, 0, LV_COORD_MAX,
                         LV_TEXT_FLAG_NONE);
    }
    const int gap = (usz.x > 0) ? 3 : 0;
    const int x0 = DETAIL_CX - (nsz.x + gap + usz.x) / 2;
    if (s_ac_num_x != x0) {
        s_ac_num_x = x0;
        lv_obj_set_pos(s_ac_temp_value, x0, AC_VALUE_Y);
    }
    const int ux = x0 + nsz.x + gap;
    if (s_ac_unit_x != ux) {
        s_ac_unit_x = ux;
        lv_obj_set_pos(s_ac_unit, ux, AC_VALUE_Y + AC_UNIT_DY);
    }
}

static void ac_set_temp_text(int t10, int step_t10)
{
    if (s_ac_temp_value == NULL) {
        return;
    }
    char num[12];
    ac_fmt_temp(num, sizeof(num), t10, step_t10);
    if (strcmp(num, s_ac_temp_txt) != 0) {
        snprintf(s_ac_temp_txt, sizeof(s_ac_temp_txt), "%s", num);
        lv_label_set_text(s_ac_temp_value, num);
    }
    ac_place_value();
    detail_marker_update();   /* 文字没变（四舍五入到同一步）时值点也要跟上 */
}

/** 「制冷 · 室内 26.4°C」（设计稿 .d-unit：目标温度 · 室内 xx°C）；
 *  关机时前半截写「已关闭」；没有 current_temperature 就显示 --。 */
static void ac_set_room_text(const char *mode_zh, float cur)
{
    if (s_ac_room == NULL) {
        return;
    }
    char buf[40];
    const char *head = (mode_zh != NULL && mode_zh[0] != '\0') ? mode_zh : "空调";
    if (cur > 0.0f) {
        const int t10 = (int)(cur * (float)AC_UNIT + 0.5f);
        snprintf(buf, sizeof(buf), "%.12s · 室内 %d.%d°C", head, t10 / AC_UNIT, t10 % AC_UNIT);
    } else {
        snprintf(buf, sizeof(buf), "%.12s · 室内 --", head);
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

/** 卡片左上角图标（19x19，自绘；颜色由 tile_apply 写进 w->icon_col）。
 *  只用圆 / 圆角条 / 线 / 弧四种图元，不引字体符号也不加图片。
 *  w->dom 决定画什么，w->icon_alt 只用来区分"湿度传感器"（水滴）和别的传感器（温度计）。 */
static void tile_icon_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    tile_widget_t *w = (tile_widget_t *)lv_event_get_user_data(e);
    if (obj == NULL || layer == NULL || w == NULL) {
        return;
    }
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    const int cx = (a.x1 + a.x2) / 2;
    const int cy = (a.y1 + a.y2) / 2;
    const lv_color_t c = w->icon_col;
    const int lw = 2;

    switch (w->dom) {
    case TILE_DOM_LIGHT:                      /* 灯泡：玻璃泡 + 灯颈 + 灯座 */
        ac_dr_dot(layer, cx, cy - 2, 5, c);
        wx_bar(layer, cx - 2, cy + 3, cx + 2, cy + 5, 1, c);
        wx_bar(layer, cx - 4, cy + 5, cx + 4, cy + 6, 1, c);
        break;
    case TILE_DOM_CLIMATE:                    /* 雪花：三条交叉线 + 短枝 */
        for (int i = 0; i < 12; i += 4) {
            int x1, y1, x2, y2;
            ac_dir_point(cx, cy, i, 7, &x1, &y1);
            ac_dir_point(cx, cy, i + 6, 7, &x2, &y2);
            ac_dr_line(layer, x1, y1, x2, y2, lw, c);
        }
        break;
    case TILE_DOM_FAN:                        /* 风扇：护环 + 三片叶 + 轮毂 */
        ac_dr_arc(layer, cx, cy, 7, lw, 0, 360, c);
        for (int i = 0; i < 12; i += 4) {
            int x1, y1, x2, y2;
            ac_dir_point(cx, cy, i, 2, &x1, &y1);
            ac_dir_point(cx, cy, i, 6, &x2, &y2);
            ac_dr_line(layer, x1, y1, x2, y2, 3, c);
        }
        ac_dr_dot(layer, cx, cy, 2, c);
        break;
    case TILE_DOM_COVER:                      /* 窗帘：顶轨 + 两片布 + 中间的缝 */
        ac_dr_line(layer, cx - 8, cy - 7, cx + 8, cy - 7, lw, c);
        ac_dr_line(layer, cx - 6, cy - 6, cx - 3, cy + 7, lw, c);
        ac_dr_line(layer, cx + 6, cy - 6, cx + 3, cy + 7, lw, c);
        ac_dr_line(layer, cx - 1, cy - 6, cx - 1, cy + 7, 1, c);
        break;
    case TILE_DOM_SWITCH:                     /* 插头：两只脚 + 身体 + 线 */
        ac_dr_line(layer, cx - 3, cy - 8, cx - 3, cy - 4, lw, c);
        ac_dr_line(layer, cx + 3, cy - 8, cx + 3, cy - 4, lw, c);
        wx_bar(layer, cx - 6, cy - 4, cx + 6, cy + 3, 3, c);
        ac_dr_line(layer, cx, cy + 3, cx, cy + 8, lw, c);
        break;
    case TILE_DOM_MEDIA:                      /* 音箱：箱体 + 喇叭 + 高音孔 */
        wx_bar(layer, cx - 5, cy - 8, cx + 5, cy + 8, 3, c);
        ac_dr_arc(layer, cx, cy + 3, 3, lw, 0, 360, c);
        ac_dr_dot(layer, cx, cy - 4, 1, c);
        break;
    case TILE_DOM_SENSOR:
        if (w->icon_alt) {                    /* 湿度：水滴 */
            ac_dr_dot(layer, cx, cy + 3, 4, c);
            ac_dr_line(layer, cx, cy - 8, cx - 4, cy + 2, lw, c);
            ac_dr_line(layer, cx, cy - 8, cx + 4, cy + 2, lw, c);
        } else {                              /* 温度：温度计 */
            ac_dr_line(layer, cx, cy - 8, cx, cy + 1, lw, c);
            ac_dr_dot(layer, cx, cy + 4, 4, c);
        }
        break;
    case TILE_DOM_BINARY:                     /* 门：门板 + 门把手 */
        wx_bar(layer, cx - 5, cy - 8, cx + 5, cy + 8, 2, c);
        ac_dr_dot(layer, cx + 2, cy, 1, c);
        break;
    case TILE_DOM_WEATHER:                    /* 太阳：圆 + 六根光线 */
        ac_dr_dot(layer, cx, cy, 4, c);
        for (int i = 0; i < 12; i += 2) {
            int x1, y1, x2, y2;
            ac_dir_point(cx, cy, i, 6, &x1, &y1);
            ac_dir_point(cx, cy, i, 9, &x2, &y2);
            ac_dr_line(layer, x1, y1, x2, y2, lw, c);
        }
        break;
    case TILE_DOM_SCENE:                      /* 场景：播放三角 */
        ac_dr_line(layer, cx - 4, cy - 7, cx + 6, cy, lw, c);
        ac_dr_line(layer, cx + 6, cy, cx - 4, cy + 7, lw, c);
        ac_dr_line(layer, cx - 4, cy + 7, cx - 4, cy - 7, lw, c);
        break;
    case TILE_DOM_AUTO:                       /* 自动化：带箭头的圆弧 */
        ac_dr_arc(layer, cx, cy, 7, lw, 210, 120, c);
        ac_dr_line(layer, cx + 2, cy - 9, cx + 6, cy - 5, lw, c);
        ac_dr_line(layer, cx + 6, cy - 5, cx + 6, cy - 1, lw, c);
        break;
    default:
        ac_dr_dot(layer, cx, cy, 4, c);
        break;
    }
}

static void ac_refresh_chips(const ha_entity_t *e, const char *state)
{
    const bool on = !ac_state_is_off(state);
    for (int i = 0; i < AC_MODE_COUNT; i++) {
        lv_obj_t *btn = s_ac_mode_btn[i];
        if (btn == NULL) {
            continue;
        }
        /* 设计稿 .m-row button：未选 = 白 4% 底 + 白 5.5% 边 + 灰图标；
           选中 = 模式色 15% 底 + 模式色 40% 边 + 模式色图标 */
        const bool active = on && (strcmp(state, k_ac_modes[i].mode) == 0);
        const lv_color_t c = detail_on_bg(lv_color_hex(k_ac_modes[i].rgb));
        lv_obj_set_style_bg_color(btn, active ? c : lv_color_white(), 0);
        lv_obj_set_style_bg_opa(btn, active ? 38 : 10, 0);
        lv_obj_set_style_border_color(btn, active ? c : lv_color_white(), 0);
        lv_obj_set_style_border_opa(btn, active ? 102 : 14, 0);
        s_ac_key_col[i] = active ? c : lv_color_hex(0x6F798C);
        if (s_ac_mode_icon[i] != NULL) {
            lv_obj_invalidate(s_ac_mode_icon[i]);
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
    detail_set_hidden(s_ac_unit, hide);
    detail_set_hidden(s_ac_room, hide);
    detail_set_hidden(s_ac_fan_btn, hide);
    detail_set_hidden(s_ac_swing_btn, hide);
    detail_set_hidden(s_ac_name_icon, hide);
    for (int i = 0; i < AC_MODE_COUNT; i++) {
        detail_set_hidden(s_ac_mode_btn[i], hide);
    }
}

/** 灯那组控件全部隐藏/显示（切到空调 / 风扇页时整组藏起来）。 */
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
    /* 灯光页独有的控件：氛围光 / 分段控件（电源键两页共用，由各页自己摆） */
    for (int i = 0; i < GLOW_DISC_N; i++) {
        detail_set_hidden(s_glow_disc[i], hide);
    }
    detail_set_hidden(s_light_seg, hide);
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
    /* 副标题：模式名 · 室内温度（关机写"已关闭"）*/
    {
        const char *st_now = ac_state_now(e);
        const char *zh = "已关闭";
        if (!ac_state_is_off(st_now)) {
            for (int i = 0; i < AC_MODE_COUNT; i++) {
                if (strcmp(st_now, k_ac_modes[i].mode) == 0) {
                    zh = k_ac_modes[i].label;
                    break;
                }
            }
        }
        ac_set_room_text(zh, e->cur_temp);
    }
    ac_set_fan_text(e);
    /* 单位 ° / 名字行图标 / 摆风胶囊：颜色跟着模式色 */
    if (s_ac_unit != NULL) {
        lv_obj_set_style_text_color(s_ac_unit, s_ac_accent, 0);
    }
    if (s_ac_name_icon != NULL) {
        lv_obj_invalidate(s_ac_name_icon);
    }
    {
        const bool sw_on = ac_swing_now(e);
        const bool sw_ok = (e != NULL) && e->has_full_swing;
        if (s_ac_swing_btn != NULL) {
            detail_set_hidden(s_ac_swing_btn, !sw_ok);      /* 不支持摆风就不显示 */
            lv_obj_set_style_bg_color(s_ac_swing_btn,
                                      (sw_on && sw_ok) ? s_ac_accent : lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_ac_swing_btn, (sw_on && sw_ok) ? 38 : 12, 0);
            lv_obj_set_style_border_color(s_ac_swing_btn,
                                          (sw_on && sw_ok) ? s_ac_accent : lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_ac_swing_btn, (sw_on && sw_ok) ? 102 : 18, 0);
        }
        if (s_ac_swing_label != NULL) {
            lv_label_set_text(s_ac_swing_label, sw_on ? "摆风" : "固定");
            lv_obj_set_style_text_color(s_ac_swing_label,
                                        (sw_on && sw_ok) ? s_ac_accent : lv_color_hex(0xC2CBDB), 0);
        }
    }
    /* 电源键：设计稿 44x44，放在胶囊行最右 */
    detail_place_power(AC_POWER, DETAIL_CX + 250 / 2 - AC_POWER, AC_CHIP_Y - 7);
    detail_set_hidden(s_dial_power, false);
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
/* 风扇详情页：和灯共用 s_detail 与同一套圆盘（环 = 风速）                     */
/* （布局和颜色见文件头 FAN_* 宏；这里只放"怎么摆、怎么写、怎么发命令"）        */
/* ------------------------------------------------------------------------- */

/* 4 个预设的固定顺序：raw = 发给 HA 的原样中文（和这台风扇 preset_modes 一致），
   rgb = 这个预设的环色。胶囊上直接显示实体报上来的原样字符串。 */
typedef struct {
    const char *raw;
    uint32_t rgb;
} fan_preset_def_t;

static const fan_preset_def_t k_fan_presets[FAN_MODE_COUNT] = {
    {"直吹风", 0x2DD4BF},   /* 青 */
    {"自然风", 0x30D158},   /* 绿 */
    {"循环风", 0x3B82F6},   /* 蓝 */
    {"睡眠",   0x8E8E93},   /* 灰 */
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

/** 风扇页底色：预设色压到很暗的一层，关掉纯黑（和空调页 ac_mode_bg 同一套做法）。
 *  不把整页染成预设色：底色一亮，中间的小字和胶囊的对比度就保不住了。 */
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
 *  下限 1%：0% 会被 Home Assistant 当成"关"，电源只归电源键（同灯的亮度）。 */
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

/** 名字行左侧那个风图标：三条带圆头的横线（设计稿 #i-wind 的直线化近似）。 */
static void fan_wind_draw_cb(lv_event_t *e)
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
    const bool on = detail_device_on(ha_client_find(s_detail_entity));
    const lv_color_t c = on ? LIGHT_ICON_INK : LIGHT_ICON_OFF;
    ac_dr_line(layer, cx - 8, cy - 4, cx + 5, cy - 4, 2, c);
    ac_dr_line(layer, cx - 8, cy,     cx + 8, cy,     2, c);
    ac_dr_line(layer, cx - 8, cy + 4, cx + 3, cy + 4, 2, c);
}

/** 大号百分比 + "%" 一起水平居中（和灯光页同一个做法）。 */
static void fan_place_value(void)
{
    if (s_fan_pct == NULL || s_fan_unit == NULL) {
        return;
    }
    lv_point_t nsz = {0, 0};
    lv_point_t usz = {0, 0};
    const char *ntxt = lv_label_get_text(s_fan_pct);
    const char *utxt = lv_label_get_text(s_fan_unit);
    if (ntxt != NULL && ntxt[0] != '\0') {
        lv_text_get_size(&nsz, ntxt, lv_obj_get_style_text_font(s_fan_pct, 0), 0, 0,
                     LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    }
    if (utxt != NULL && utxt[0] != '\0') {
        lv_text_get_size(&usz, utxt, &lv_font_montserrat_20, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    }
    const int gap = (usz.x > 0) ? 3 : 0;
    const int x0 = DETAIL_CX - (nsz.x + gap + usz.x) / 2;
    if (s_fan_num_x != x0) {
        s_fan_num_x = x0;
        lv_obj_set_pos(s_fan_pct, x0, LIGHT_VALUE_Y);
    }
    const int ux = x0 + nsz.x + gap;
    if (s_fan_unit_x != ux) {
        s_fan_unit_x = ux;
        lv_obj_set_pos(s_fan_unit, ux, LIGHT_VALUE_Y + LIGHT_UNIT_DY);
    }
}

/* ---- 摆风（climate.swing_mode）：default ↔ full_swing ---- */
static bool ac_swing_now(const ha_entity_t *e)
{
    const char *m = (e != NULL) ? e->swing_mode : "";
    if (s_ac_swing_local_until > lv_tick_get()) {
        if (e != NULL && strcmp(e->swing_mode, s_ac_swing_local) == 0) {
            s_ac_swing_local_until = 0;        /* HA 已跟上 */
        } else {
            m = s_ac_swing_local;
        }
    }
    return (m != NULL) && (strcmp(m, "full_swing") == 0);
}

/** 摆风胶囊：点一下在 default / full_swing 之间切（实体只支持默认时置灰不可点）。*/
static void ac_swing_cb(lv_event_t *e)
{
    (void)e;
    const ha_entity_t *ent = ha_client_find(s_detail_entity);
    if (ent == NULL || s_cmd_q == NULL || s_detail_entity[0] == '\0') {
        return;
    }
    const bool next = !ac_swing_now(ent);
    snprintf(s_ac_swing_local, sizeof(s_ac_swing_local), "%s", next ? "full_swing" : "default");
    s_ac_swing_local_until = lv_tick_get() + OVERRIDE_MS;
    if (s_ac_swing_label != NULL) {
        lv_label_set_text(s_ac_swing_label, next ? "摆风" : "固定");
    }
    if (s_ac_swing_btn != NULL) {
        lv_obj_set_style_bg_color(s_ac_swing_btn, next ? s_ac_accent : lv_color_white(), 0);
        lv_obj_set_style_bg_opa(s_ac_swing_btn, next ? 38 : 12, 0);
        lv_obj_set_style_border_color(s_ac_swing_btn, next ? s_ac_accent : lv_color_white(), 0);
        lv_obj_set_style_border_opa(s_ac_swing_btn, next ? 102 : 18, 0);
        if (s_ac_swing_label != NULL) {
            lv_obj_set_style_text_color(s_ac_swing_label, next ? s_ac_accent : lv_color_hex(0xC2CBDB), 0);
        }
    }
    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_detail_entity);
    snprintf(cmd.service, sizeof(cmd.service), "set_swing_mode");
    snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"swing_mode\":\"%s\"}", s_ac_swing_local);
    xQueueSend(s_cmd_q, &cmd, 0);
    ESP_LOGI(TAG, "摆风 -> %s", s_ac_swing_local);
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
    snprintf(buf, sizeof(buf), "%d", pct);
    if (strcmp(buf, s_fan_pct_txt) != 0) {
        snprintf(s_fan_pct_txt, sizeof(s_fan_pct_txt), "%s", buf);
        lv_label_set_text(s_fan_pct, buf);
    }
    if (s_fan_unit != NULL && strcmp("%", s_fan_unit_shown) != 0) {
        snprintf(s_fan_unit_shown, sizeof(s_fan_unit_shown), "%s", "%");
        lv_label_set_text(s_fan_unit, "%");
    }
    fan_place_value();
    detail_marker_update();          /* 值点跟着风速环当前值走 */
}

/** 胶囊行 / 电源键的外观：
 *   预设胶囊 = 当前预设名（设计稿 #fPreset），关机时压暗；
 *   摇头胶囊 = 永远按"不支持"置灰（没有 OSCILLATE，也没有 click 回调）；
 *   电源键：开 = 预设色实心 + 深色图标，关 = 白 6% 底 + 浅灰图标（.p-btn）。 */
static void fan_apply_chips(const ha_entity_t *e, const char *preset, bool on)
{
    const int8_t st = (int8_t)((on ? 1 : 0) | ((preset != NULL && preset[0]) ? 2 : 0));
    if (st != s_fan_chip_shown) {
        s_fan_chip_shown = st;
        if (s_fan_chip_preset_lbl != NULL) {
            const char *txt = (preset != NULL && preset[0] != '\0') ? preset : "--";
            if (strcmp(txt, s_fan_chip_txt) != 0) {
                snprintf(s_fan_chip_txt, sizeof(s_fan_chip_txt), "%.15s", txt);
                lv_label_set_text(s_fan_chip_preset_lbl, txt);
            }
        }
        if (s_fan_chip_preset != NULL) {
            lv_obj_set_style_opa(s_fan_chip_preset, on ? LV_OPA_COVER : 115, 0);
        }
        if (s_fan_chip_osc != NULL) {
            /* 不支持摇头：整颗压到 40%，边框回到"中性"，永远不亮起来 */
            lv_obj_set_style_opa(s_fan_chip_osc, FAN_CHIP_DISABLED_OPA, 0);
            lv_obj_set_style_border_color(s_fan_chip_osc, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_fan_chip_osc, 18, 0);
        }
    }
    s_dial_pwr_on = on;
    if (s_dial_power != NULL) {
        lv_obj_set_style_bg_color(s_dial_power, on ? s_fan_accent : lv_color_white(), 0);
        lv_obj_set_style_bg_opa(s_dial_power, on ? LV_OPA_COVER : 15, 0);
        lv_obj_set_style_border_opa(s_dial_power, on ? LV_OPA_TRANSP : 23, 0);
        lv_obj_invalidate(s_dial_power);
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
    fan_apply_chips(e, fan_preset_now(e), detail_device_on(e));
    if (s_fan_name_icon != NULL) {
        lv_obj_invalidate(s_fan_name_icon);      /* 开 / 关时图标颜色会变 */
    }
    /* 环一直显示（关着只是压暗、不可点，刻度还在），和空调页一致。 */
}

/** 风扇那组控件全部显示 / 隐藏（切到灯 / 空调页时整组藏起来）。*/
static void fan_hide_all(bool hide)
{
    detail_set_hidden(s_fan_arc, hide);
    detail_set_hidden(s_fan_glow, hide);
    detail_set_hidden(s_fan_name_icon, hide);
    detail_set_hidden(s_fan_pct, hide);
    detail_set_hidden(s_fan_unit, hide);
    detail_set_hidden(s_fan_label, hide);
    detail_set_hidden(s_fan_chip_preset, hide);
    detail_set_hidden(s_fan_chip_osc, hide);
}

/** 旋钮在风扇页上调风速：每格 5%（任务指定），值域 1..100。 */
static bool fan_knob_step(const ha_entity_t *e, int delta)
{
    if (s_fan_arc == NULL || e == NULL) {
        return false;
    }
    if (!detail_device_on(e)) {
        return false;                     /* 关着：旋钮什么也不做（电源只归电源键） */
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

/** 预设胶囊：点一下换到实体 preset_modes 里的下一个（命令发原样中文）。
 *  设计稿 #fPreset 的交互就是"点一下换预设"，不再排 4 个键。 */
static void fan_preset_cycle_cb(lv_event_t *ev)
{
    (void)ev;
    const ha_entity_t *ent = ha_client_find(s_detail_entity);
    if (ent == NULL || s_cmd_q == NULL || s_detail_entity[0] == '\0') {
        return;
    }
    char cur[16];
    snprintf(cur, sizeof(cur), "%.15s", fan_preset_now(ent));
    char next[16] = {0};
    if (ent->preset_mode_count > 0) {
        int idx = -1;
        for (int i = 0; i < ent->preset_mode_count && i < HA_FAN_PRESET_MAX; i++) {
            if (strcmp(ent->preset_modes[i], cur) == 0) {
                idx = i;
                break;
            }
        }
        idx = (idx + 1) % (int)ent->preset_mode_count;
        snprintf(next, sizeof(next), "%.15s", ent->preset_modes[idx]);
    } else {
        int idx = -1;
        for (int i = 0; i < FAN_MODE_COUNT; i++) {
            if (strcmp(k_fan_presets[i].raw, cur) == 0) {
                idx = i;
            }
        }
        idx = (idx + 1) % FAN_MODE_COUNT;
        snprintf(next, sizeof(next), "%.15s", k_fan_presets[idx].raw);
    }
    snprintf(s_fan_preset_local, sizeof(s_fan_preset_local), "%s", next);
    s_fan_preset_local_until = lv_tick_get() + OVERRIDE_MS;
    /* 选预设 = 想要这种风：开关也立刻跟着亮（HA 跟上后自动交还） */
    s_sw_override_on = true;
    s_sw_override_until = lv_tick_get() + OVERRIDE_MS;
    s_sw_on_shown = -1;
    /* 环色 / 底色 / 图标立刻跟着新预设走 */
    if (strcmp(next, s_fan_preset_shown) != 0) {
        snprintf(s_fan_preset_shown, sizeof(s_fan_preset_shown), "%s", next);
        s_fan_accent = detail_on_bg(fan_preset_color(next));
        if (s_fan_arc) {
            lv_obj_set_style_arc_color(s_fan_arc, s_fan_accent, LV_PART_INDICATOR);
        }
        if (s_fan_glow) {
            lv_obj_set_style_arc_color(s_fan_glow, s_fan_accent, LV_PART_INDICATOR);
        }
        if (s_fan_name_icon) {
            lv_obj_invalidate(s_fan_name_icon);
        }
    }
    s_fan_chip_shown = -1;
    fan_apply_chips(ent, next, true);
    s_detail_fg_dirty = true;                /* 底色要跟着预设换 */
    detail_update_ambience();
    detail_apply_focus();

    cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    snprintf(cmd.entity, sizeof(cmd.entity), "%s", s_detail_entity);
    snprintf(cmd.service, sizeof(cmd.service), "set_preset_mode");
    snprintf(cmd.attrs, sizeof(cmd.attrs), "{\"preset_mode\":\"%s\"}", next);
    xQueueSend(s_cmd_q, &cmd, 0);
    ESP_LOGI(TAG, "预设 -> %s（这台风扇不支持摇头：supported_features=57 无 OSCILLATE）", next);
}

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
    if (delta == 0) {
        return false;
    }
    /* 天气页开着：旋钮只滚天气页内部（返回 true 让 main.c 不翻页、响一下触觉）。
       这是"页面接管旋钮"的唯一一处：翻页被 ui_home_page_step 和这里同时挡住。 */
    if (ui_home_weather_open()) {
        wx_scroll_step(delta);
        return true;
    }
    if (!s_detail_visible || s_detail == NULL) {
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
 * 圆盘按"区域"切焦点（旧版：环心 = 色温、环带 = 亮度）。
 *
 * 灯光页现在照设计稿用分段控件选亮度 / 色温，所以中间区域不再切焦点：
 * 点环心什么也不做，点环带 / 拖环仍然是"调当前那一档"（环自己的 PRESSED 回调）。
 * 空调 / 风扇本来就只有一条环，中间区域同样不做任何事。
 * 这个回调保留成"占位 + 注释"，因为 s_detail 的 PRESSED 事件还挂着它。
 */
static void detail_region_pressed_cb(lv_event_t *e)
{
    if (s_detail == NULL || lv_event_get_target(e) != s_detail) {
        return;                          /* 打中了子控件：让它自己处理（冒泡上来的事件）*/
    }
    /* 灯 / 空调 / 风扇：档位由分段控件或本来就只有一档，中间区域不再切焦点 */
    (void)e;
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
    /* 指示灯的角度也显式设成 270°（135°→45°，缺口朝下）。只设 bg_angles 时，
       指示灯的默认角度是 0..360——一旦某条路径只改样式不调 lv_arc_set_value，
       画出来就是一整圈；显式设一次把这条路堵死。 */
    lv_arc_set_angles(arc, DETAIL_ANG_START, DETAIL_ANG_END);
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

/* ================= WEATHER PAGE (ui_home.c) ================= */

/* ------------------------------------------------------------------------- */
/* 天气页：实况 + 逐小时（6 条）+ 逐日（3 条），整页纵向滚动、旋钮接管          */
/*                                                                           */
/* 设计稿 .scr[data-scr="weather"] 是"现在 / 逐小时 / 逐日"三页、用旋钮翻页；   */
/* 这里按任务要求合成一页长内容（旋钮在页内滚动），三段的排版逐条照设计稿：      */
/*   实况   40x40 图标 y=40 / 天况 y=86 / 大号温度 y=136 / 体感 y=172 /         */
/*          六格 y=204（2 列 × 3 行，见下面的说明）                            */
/*   逐小时 标题 y=300，6 行 × 40（时间 52 / 图标 20 / 降水 46 / 温度右对齐）   */
/*   逐日   标题 y=582，3 行 × 56（今天 / 明天 / 周X / 图标 / 天况 / 高低温）    */
/* 可视区 = 屏幕 0..300：300..360 留给圆屏收边，滚到底时最后一行正好露出来。    */
/* 设计稿这三页只有标题和分隔线、没有状态栏；本页是覆盖层，和灯 / 空调详情页    */
/* 用同一套滑入滑出动画（s_detail 那套），不画状态栏和圆点。                    */
/* ------------------------------------------------------------------------- */
#define WX_VIEW_H          300
#define WX_ROW_X            56
#define WX_ROW_W           248
#define WX_NOW_ICON_Y       40
#define WX_NOW_COND_Y       80
#define WX_NOW_BIG_Y       124     /* 64 号数字：墨迹 129..174，中心 151 */
#define WX_NOW_FEEL_Y      178
#define WX_GRID_Y          204
#define WX_GRID_ROW_H       26
#define WX_HOUR_HDR_Y      292
#define WX_HOUR_Y          322
#define WX_HOUR_H           40
#define WX_DAY_HDR_Y       (WX_HOUR_Y + HA_WX_HOURLY * WX_HOUR_H + 20)   /* 582 */
#define WX_DAY_Y           (WX_DAY_HDR_Y + 26)                           /* 608 */
#define WX_DAY_H            56
#define WX_CONTENT_H       (WX_DAY_Y + HA_WX_DAILY * WX_DAY_H + 60)      /* 836 */
#define WX_SCROLL_STEP      40
#define WX_CELL_MAX          6

static lv_obj_t *s_wx_page;
static lv_obj_t *s_wx_scroll;
static lv_obj_t *s_wx_cond;
static lv_obj_t *s_wx_big;
static lv_obj_t *s_wx_unit;
static lv_obj_t *s_wx_icon_big;
static lv_obj_t *s_wx_feel;
static lv_obj_t *s_wx_cell_lbl[WX_CELL_MAX];
static lv_obj_t *s_wx_cell_val[WX_CELL_MAX];
static lv_obj_t *s_wx_hour_row[HA_WX_HOURLY];
static lv_obj_t *s_wx_hour_t[HA_WX_HOURLY];
static lv_obj_t *s_wx_hour_ic[HA_WX_HOURLY];
static lv_obj_t *s_wx_hour_p[HA_WX_HOURLY];
static lv_obj_t *s_wx_hour_v[HA_WX_HOURLY];
static lv_obj_t *s_wx_day_row[HA_WX_DAILY];
static lv_obj_t *s_wx_day_d[HA_WX_DAILY];
static lv_obj_t *s_wx_day_ic[HA_WX_DAILY];
static lv_obj_t *s_wx_day_c[HA_WX_DAILY];
static lv_obj_t *s_wx_day_r[HA_WX_DAILY];
static int s_wx_big_x = -1;
static int s_wx_unit_x = -1;
static uint32_t s_wx_sum_shown;
static bool s_wx_visible;
static char s_wx_entity[96];

static void wx_refresh_ui(void);
static void wx_close(void);

/** 天气图标配色（设计稿：太阳 #FFC24B / 云 #9FB0C9 / 雨 #7FB2FF / 雪 #BFD9FF / 其它灰）。*/
static lv_color_t wx_kind_color(int kind)
{
    switch (kind) {
    case HA_WX_SUNNY:
    case HA_WX_CLEAR_NIGHT:
        return lv_color_hex(0xFFC24B);
    case HA_WX_PARTLY:
    case HA_WX_CLOUDY:
        return lv_color_hex(0x9FB0C9);
    case HA_WX_RAIN:
    case HA_WX_POURING:
    case HA_WX_LIGHTNING:
        return lv_color_hex(0x7FB2FF);
    case HA_WX_SNOW:
    case HA_WX_SNOWY_RAIN:
        return lv_color_hex(0xBFD9FF);
    default:
        return lv_color_hex(0x8D97A8);
    }
}

/** 逐日那行的中文（设计稿 .w-day .c 是"晴 / 小雨 / 多云"）。*/
static const char *wx_kind_zh(int kind)
{
    switch (kind) {
    case HA_WX_SUNNY:
    case HA_WX_CLEAR_NIGHT:
        return "晴";
    case HA_WX_PARTLY:
        return "多云";
    case HA_WX_CLOUDY:
        return "阴";
    case HA_WX_RAIN:
        return "小雨";
    case HA_WX_POURING:
        return "大雨";
    case HA_WX_LIGHTNING:
        return "雷阵雨";
    case HA_WX_SNOW:
        return "雪";
    case HA_WX_SNOWY_RAIN:
        return "雨夹雪";
    case HA_WX_FOG:
        return "雾";
    case HA_WX_WINDY:
        return "大风";
    case HA_WX_HAIL:
        return "冰雹";
    default:
        return "--";
    }
}

/** 圆角实心横条（画云、雾、雨丝用）。 */
static void wx_bar(lv_layer_t *l, int x1, int y1, int x2, int y2, int r, lv_color_t c)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = c;
    d.bg_opa = LV_OPA_COVER;
    d.radius = (lv_coord_t)r;
    lv_area_t a = {(lv_coord_t)x1, (lv_coord_t)y1, (lv_coord_t)x2, (lv_coord_t)y2};
    lv_draw_rect(l, &d, &a);
}

/** 云：三个圆 + 两条横条拼成（u = 画布边长，20px 行图标和 40px 大图标共用）。*/
static void wx_draw_cloud(lv_layer_t *l, int cx, int cy, int u, lv_color_t c)
{
    ac_dr_dot(l, cx - u * 8 / 40, cy + u * 2 / 40, u * 7 / 40, c);
    ac_dr_dot(l, cx,             cy - u * 1 / 40, u * 9 / 40, c);
    ac_dr_dot(l, cx + u * 8 / 40, cy + u * 3 / 40, u * 7 / 40, c);
    wx_bar(l, cx - u * 14 / 40, cy + u * 2 / 40, cx + u * 14 / 40, cy + u * 9 / 40, u * 3 / 40, c);
}

/** 天气图标（自绘，不新增字体/图片）：kind 存在对象的 user_data 里。 */
static void wx_icon_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    if (obj == NULL || layer == NULL) {
        return;
    }
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    const int w = a.x2 - a.x1 + 1;
    const int h = a.y2 - a.y1 + 1;
    const int u = (w < h ? w : h);
    const int cx = (a.x1 + a.x2) / 2;
    const int cy = (a.y1 + a.y2) / 2;
    const int kind = (int)(intptr_t)lv_obj_get_user_data(obj);
    const lv_color_t c = wx_kind_color(kind);
    const int lw = (u >= 36) ? 3 : 2;
    static const int16_t dx8[8] = {1024, 724, 0, -724, -1024, -724, 0, 724};
    static const int16_t dy8[8] = {0, 724, 1024, 724, 0, -724, -1024, -724};

    switch (kind) {
    case HA_WX_SUNNY:
        ac_dr_dot(layer, cx, cy, u * 9 / 40, c);
        for (int i = 0; i < 8; i++) {
            const int x1 = cx + (int)((int32_t)dx8[i] * (u * 12 / 40) / 1024);
            const int y1 = cy + (int)((int32_t)dy8[i] * (u * 12 / 40) / 1024);
            const int x2 = cx + (int)((int32_t)dx8[i] * (u * 18 / 40) / 1024);
            const int y2 = cy + (int)((int32_t)dy8[i] * (u * 18 / 40) / 1024);
            ac_dr_line(layer, x1, y1, x2, y2, lw, c);
        }
        break;
    case HA_WX_CLEAR_NIGHT:
        /* 弯月：整圆 + 一个页面底色的圆挖掉右上角 */
        ac_dr_dot(layer, cx, cy, u * 11 / 40, c);
        ac_dr_dot(layer, cx + u * 6 / 40, cy - u * 4 / 40, u * 10 / 40, lv_color_hex(0x05070B));
        break;
    case HA_WX_PARTLY:
        ac_dr_dot(layer, cx + u * 8 / 40, cy - u * 8 / 40, u * 6 / 40, wx_kind_color(HA_WX_SUNNY));
        wx_draw_cloud(layer, cx - u * 2 / 40, cy + u * 3 / 40, u, c);
        break;
    case HA_WX_RAIN:
    case HA_WX_POURING: {
        wx_draw_cloud(layer, cx, cy - u * 4 / 40, u, c);
        const int n = (kind == HA_WX_POURING) ? 3 : 2;
        for (int i = 0; i < n; i++) {
            const int x = cx + (i - (n - 1) / 2) * (u * 8 / 40);
            ac_dr_line(layer, x, cy + u * 9 / 40, x, cy + u * 16 / 40, lw, c);
        }
        break;
    }
    case HA_WX_LIGHTNING:
        wx_draw_cloud(layer, cx, cy - u * 5 / 40, u, c);
        ac_dr_line(layer, cx + u * 3 / 40, cy + u * 5 / 40, cx - u * 3 / 40, cy + u * 11 / 40, lw, c);
        ac_dr_line(layer, cx - u * 3 / 40, cy + u * 11 / 40, cx + u * 2 / 40, cy + u * 11 / 40, lw, c);
        ac_dr_line(layer, cx + u * 2 / 40, cy + u * 11 / 40, cx - u * 2 / 40, cy + u * 17 / 40, lw, c);
        break;
    case HA_WX_SNOW:
    case HA_WX_SNOWY_RAIN:
        wx_draw_cloud(layer, cx, cy - u * 4 / 40, u, c);
        for (int i = 0; i < 3; i++) {
            ac_dr_dot(layer, cx + (i - 1) * (u * 8 / 40), cy + u * 14 / 40, u * 2 / 40 + 1, c);
        }
        break;
    case HA_WX_FOG:
        wx_bar(layer, cx - u * 14 / 40, cy - u * 6 / 40, cx + u * 14 / 40, cy - u * 6 / 40 + lw, lw / 2, c);
        wx_bar(layer, cx - u * 10 / 40, cy,            cx + u * 10 / 40, cy + lw,            lw / 2, c);
        wx_bar(layer, cx - u * 6 / 40,  cy + u * 6 / 40, cx + u * 6 / 40, cy + u * 6 / 40 + lw, lw / 2, c);
        break;
    case HA_WX_WINDY:
        ac_dr_line(layer, cx - u * 10 / 40, cy - u * 5 / 40, cx + u * 10 / 40, cy - u * 5 / 40, lw, c);
        ac_dr_line(layer, cx - u * 6 / 40,  cy,            cx + u * 6 / 40,  cy,            lw, c);
        ac_dr_line(layer, cx - u * 10 / 40, cy + u * 5 / 40, cx + u * 4 / 40, cy + u * 5 / 40, lw, c);
        break;
    case HA_WX_HAIL:
        wx_draw_cloud(layer, cx, cy - u * 4 / 40, u, c);
        ac_dr_dot(layer, cx - u * 5 / 40, cy + u * 13 / 40, u * 3 / 40 + 1, c);
        ac_dr_dot(layer, cx + u * 5 / 40, cy + u * 15 / 40, u * 3 / 40 + 1, c);
        break;
    default:
        wx_draw_cloud(layer, cx, cy, u, c);
        break;
    }
}

/** 给天气图标换种类：kind 存 user_data，然后重画。 */
static void wx_set_icon(lv_obj_t *obj, int kind)
{
    if (obj == NULL) {
        return;
    }
    if ((int)(intptr_t)lv_obj_get_user_data(obj) == kind) {
        return;
    }
    lv_obj_set_user_data(obj, (void *)(intptr_t)kind);
    lv_obj_invalidate(obj);
}

/** 大号温度 + "°" 一起水平居中（和灯光页同一个做法）。 */
static void wx_place_big(void)
{
    if (s_wx_big == NULL || s_wx_unit == NULL) {
        return;
    }
    lv_point_t nsz = {0, 0};
    lv_point_t usz = {0, 0};
    const char *ntxt = lv_label_get_text(s_wx_big);
    const char *utxt = lv_label_get_text(s_wx_unit);
    if (ntxt != NULL && ntxt[0] != '\0') {
        lv_text_get_size(&nsz, ntxt, lv_obj_get_style_text_font(s_wx_big, 0), 0, 0,
                     LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    }
    if (utxt != NULL && utxt[0] != '\0') {
        lv_text_get_size(&usz, utxt, &lv_font_montserrat_20, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    }
    const int x0 = DETAIL_CX - (nsz.x + 3 + usz.x) / 2;
    if (s_wx_big_x != x0) {
        s_wx_big_x = x0;
        lv_obj_set_pos(s_wx_big, x0, WX_NOW_BIG_Y);
    }
    const int ux = x0 + nsz.x + 3;
    if (s_wx_unit_x != ux) {
        s_wx_unit_x = ux;
        lv_obj_set_pos(s_wx_unit, ux, WX_NOW_BIG_Y + 14);
    }
}

/** 建一条"时间 / 图标 / 降水 / 温度"的逐小时行。 */
static void wx_make_hour_row(lv_obj_t *parent, int i)
{
    const int y = WX_HOUR_Y + i * WX_HOUR_H;
    lv_obj_t *row = lv_obj_create(parent);
    s_wx_hour_row[i] = row;
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, WX_ROW_W, WX_HOUR_H);
    lv_obj_set_pos(row, WX_ROW_X, y);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_color_white(), 0);
    lv_obj_set_style_border_opa(row, 13, 0);          /* rgba(255,255,255,.05) */
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    s_wx_hour_t[i] = lv_label_create(row);
    lv_label_set_text(s_wx_hour_t[i], "--时");
    lv_obj_set_style_text_font(s_wx_hour_t[i], &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_wx_hour_t[i], lv_color_hex(0x8D97A8), 0);
    lv_obj_set_pos(s_wx_hour_t[i], 0, 10);

    s_wx_hour_ic[i] = lv_obj_create(row);
    lv_obj_remove_style_all(s_wx_hour_ic[i]);
    lv_obj_set_size(s_wx_hour_ic[i], 20, 20);
    lv_obj_set_pos(s_wx_hour_ic[i], 56, 10);
    lv_obj_set_user_data(s_wx_hour_ic[i], (void *)(intptr_t)HA_WX_CLOUDY);
    lv_obj_remove_flag(s_wx_hour_ic[i], LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_wx_hour_ic[i], wx_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_wx_hour_p[i] = lv_label_create(row);
    lv_label_set_text(s_wx_hour_p[i], "--");
    lv_obj_set_style_text_font(s_wx_hour_p[i], &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_wx_hour_p[i], lv_color_hex(0x6F8FB8), 0);
    lv_obj_set_style_text_align(s_wx_hour_p[i], LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_width(s_wx_hour_p[i], 46);
    lv_obj_set_pos(s_wx_hour_p[i], 100, 11);

    s_wx_hour_v[i] = lv_label_create(row);
    lv_label_set_text(s_wx_hour_v[i], "--°");
    lv_obj_set_style_text_font(s_wx_hour_v[i], &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_wx_hour_v[i], lv_color_hex(0xE8EEF8), 0);
    lv_obj_set_style_text_align(s_wx_hour_v[i], LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_width(s_wx_hour_v[i], 88);
    lv_obj_set_pos(s_wx_hour_v[i], 160, 8);
}

/** 建一条"今天 / 图标 / 天况 / 高低温"的逐日行。 */
static void wx_make_day_row(lv_obj_t *parent, int i)
{
    const int y = WX_DAY_Y + i * WX_DAY_H;
    lv_obj_t *row = lv_obj_create(parent);
    s_wx_day_row[i] = row;
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, WX_ROW_W, WX_DAY_H);
    lv_obj_set_pos(row, WX_ROW_X, y);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_color_white(), 0);
    lv_obj_set_style_border_opa(row, 13, 0);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    s_wx_day_d[i] = lv_label_create(row);
    lv_label_set_text(s_wx_day_d[i], "--");
    lv_obj_set_style_text_font(s_wx_day_d[i], &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_wx_day_d[i], lv_color_hex(0x9AA4B5), 0);
    lv_obj_set_pos(s_wx_day_d[i], 0, 18);

    s_wx_day_ic[i] = lv_obj_create(row);
    lv_obj_remove_style_all(s_wx_day_ic[i]);
    lv_obj_set_size(s_wx_day_ic[i], 20, 20);
    lv_obj_set_pos(s_wx_day_ic[i], 56, 18);
    lv_obj_set_user_data(s_wx_day_ic[i], (void *)(intptr_t)HA_WX_CLOUDY);
    lv_obj_remove_flag(s_wx_day_ic[i], LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_wx_day_ic[i], wx_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_wx_day_c[i] = lv_label_create(row);
    lv_label_set_text(s_wx_day_c[i], "--");
    lv_obj_set_style_text_font(s_wx_day_c[i], &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_wx_day_c[i], lv_color_hex(0x8D97A8), 0);
    lv_obj_set_pos(s_wx_day_c[i], 82, 18);

    s_wx_day_r[i] = lv_label_create(row);
    lv_label_set_text(s_wx_day_r[i], "--° / --°");
    lv_obj_set_style_text_font(s_wx_day_r[i], &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_wx_day_r[i], lv_color_hex(0xE8EEF8), 0);
    lv_obj_set_style_text_align(s_wx_day_r[i], LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_width(s_wx_day_r[i], 96);
    lv_obj_set_pos(s_wx_day_r[i], 152, 16);
}

/** 六个实况格子的标题（顺序照设计稿 .w-grid）。 */
static const char *const k_wx_cells[WX_CELL_MAX] = {
    "湿度", "云量", "风", "气压", "能见度", "露点",
};

static void build_weather_page(lv_obj_t *scr)
{
    s_wx_page = lv_obj_create(scr);
    if (s_wx_page == NULL) {
        return;
    }
    lv_obj_remove_style_all(s_wx_page);
    lv_obj_set_size(s_wx_page, 360, 360);
    lv_obj_set_pos(s_wx_page, 0, 0);
    lv_obj_set_style_bg_color(s_wx_page, lv_color_hex(0x05070B), 0);
    lv_obj_set_style_bg_opa(s_wx_page, LV_OPA_COVER, 0);
    lv_obj_remove_flag(s_wx_page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_wx_page, LV_OBJ_FLAG_HIDDEN);

    /* .w-sky：顶部一层蓝色氛围光（设计稿的径向渐变近似，同灯页做法）*/
    lv_obj_t *sky = detail_make_glow_disc(s_wx_page, 150, 12, DETAIL_CX, 62);
    if (sky != NULL) {
        lv_obj_set_style_bg_color(sky, lv_color_hex(0x4C8DFF), 0);
    }

    s_wx_scroll = lv_obj_create(s_wx_page);
    lv_obj_remove_style_all(s_wx_scroll);
    lv_obj_set_size(s_wx_scroll, 360, WX_VIEW_H);
    lv_obj_set_pos(s_wx_scroll, 0, 0);
    lv_obj_set_style_bg_opa(s_wx_scroll, LV_OPA_TRANSP, 0);
    lv_obj_add_flag(s_wx_scroll, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(s_wx_scroll, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_wx_scroll, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_set_style_width(s_wx_scroll, 3, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_color(s_wx_scroll, lv_color_white(), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(s_wx_scroll, 46, LV_PART_SCROLLBAR);
    lv_obj_set_style_radius(s_wx_scroll, 2, LV_PART_SCROLLBAR);
    lv_obj_set_style_pad_all(s_wx_scroll, 0, 0);

    lv_obj_t *c = s_wx_scroll;

    /* ---- 现在 ---- */
    s_wx_icon_big = lv_obj_create(c);
    lv_obj_remove_style_all(s_wx_icon_big);
    lv_obj_set_size(s_wx_icon_big, 40, 40);
    lv_obj_set_pos(s_wx_icon_big, DETAIL_CX - 20, WX_NOW_ICON_Y);
    lv_obj_set_user_data(s_wx_icon_big, (void *)(intptr_t)HA_WX_SUNNY);
    lv_obj_remove_flag(s_wx_icon_big, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_wx_icon_big, wx_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_wx_cond = lv_label_create(c);
    lv_label_set_text(s_wx_cond, "--");
    lv_obj_set_style_text_font(s_wx_cond, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_wx_cond, lv_color_hex(0xB9C4D6), 0);
    lv_obj_set_style_text_letter_space(s_wx_cond, 2, 0);
    lv_obj_set_style_text_align(s_wx_cond, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(s_wx_cond, 240);
    lv_obj_set_pos(s_wx_cond, DETAIL_CX - 120, WX_NOW_COND_Y);

    s_wx_big = lv_label_create(c);
    lv_label_set_text(s_wx_big, "--");
    lv_obj_set_style_text_font(s_wx_big, BIG_NUM_FONT, 0);
    lv_obj_set_style_text_color(s_wx_big, lv_color_white(), 0);
    lv_obj_set_pos(s_wx_big, 0, WX_NOW_BIG_Y);

    s_wx_unit = lv_label_create(c);
    lv_label_set_text(s_wx_unit, "°");
    lv_obj_set_style_text_font(s_wx_unit, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_wx_unit, lv_color_white(), 0);
    lv_obj_set_style_text_opa(s_wx_unit, 140, 0);      /* 设计稿 opacity .55 */
    lv_obj_set_pos(s_wx_unit, 0, WX_NOW_BIG_Y + 6);

    s_wx_feel = lv_label_create(c);
    lv_label_set_text(s_wx_feel, "体感 --");
    lv_obj_set_style_text_font(s_wx_feel, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_wx_feel, lv_color_hex(0x7E899D), 0);
    lv_obj_set_style_text_align(s_wx_feel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(s_wx_feel, 240);
    lv_obj_set_pos(s_wx_feel, DETAIL_CX - 120, WX_NOW_FEEL_Y);

    /* 六格：设计稿 3 列 × 2 行、12px 标签 + 16px 数值；固件只有 cjk16 一种中文，
       3 列放不下"能见度 11km"，改成 2 列 × 3 行（字号、配色不变）。*/
    for (int i = 0; i < WX_CELL_MAX; i++) {
        const int col = i % 2;
        const int row = i / 2;
        const int x = DETAIL_CX - 120 + col * 122;
        const int y = WX_GRID_Y + row * WX_GRID_ROW_H;
        s_wx_cell_lbl[i] = lv_label_create(c);
        lv_label_set_text(s_wx_cell_lbl[i], k_wx_cells[i]);
        lv_obj_set_style_text_font(s_wx_cell_lbl[i], &lv_font_cjk16, 0);
        lv_obj_set_style_text_color(s_wx_cell_lbl[i], lv_color_hex(0x66718A), 0);
        lv_obj_set_width(s_wx_cell_lbl[i], 54);          /* 标签列固定 54，"能见度"正好放下 */
        lv_obj_set_pos(s_wx_cell_lbl[i], x, y);

        s_wx_cell_val[i] = lv_label_create(c);
        lv_label_set_text(s_wx_cell_val[i], "--");
        lv_obj_set_style_text_font(s_wx_cell_val[i],
                                   (i == 2) ? &lv_font_cjk16 : &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(s_wx_cell_val[i], lv_color_hex(0xCDD6E4), 0);
        lv_obj_set_width(s_wx_cell_val[i], 64);          /* 54 + 8 间距 + 64 = 126 ≈ 单元格宽 */
        lv_obj_set_pos(s_wx_cell_val[i], x + 56, y);
    }

    /* ---- 逐小时 ---- */
    lv_obj_t *hh = lv_label_create(c);
    lv_label_set_text(hh, "逐小时");
    lv_obj_set_style_text_font(hh, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(hh, lv_color_hex(0xDBE3EF), 0);
    lv_obj_set_style_text_letter_space(hh, 2, 0);
    lv_obj_set_style_text_align(hh, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(hh, 240);
    lv_obj_set_pos(hh, DETAIL_CX - 120, WX_HOUR_HDR_Y);

    for (int i = 0; i < HA_WX_HOURLY; i++) {
        wx_make_hour_row(c, i);
    }

    /* ---- 逐日 ---- */
    lv_obj_t *dh = lv_label_create(c);
    lv_label_set_text(dh, "逐日");
    lv_obj_set_style_text_font(dh, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(dh, lv_color_hex(0xDBE3EF), 0);
    lv_obj_set_style_text_letter_space(dh, 2, 0);
    lv_obj_set_style_text_align(dh, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(dh, 240);
    lv_obj_set_pos(dh, DETAIL_CX - 120, WX_DAY_HDR_Y);

    for (int i = 0; i < HA_WX_DAILY; i++) {
        wx_make_day_row(c, i);
    }

    /* 内容高度：滚动到底时最后一行能滚进可视区（多给 60px 余量）*/
    lv_obj_t *pad = lv_obj_create(c);
    lv_obj_remove_style_all(pad);
    lv_obj_set_size(pad, 1, 1);
    lv_obj_set_pos(pad, 0, WX_CONTENT_H);
    lv_obj_remove_flag(pad, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    ESP_LOGI(TAG, "天气页建好：3 段 / 可视 %d / 内容 %d（旋钮滚动，每格 %d px）",
             WX_VIEW_H, WX_CONTENT_H, WX_SCROLL_STEP);
}

/** 十进制的 0.1 单位四舍五入成整数（-1.5 → -2）。*/
static int wx_r10(int v10)
{
    return (v10 >= 0) ? ((v10 + 5) / 10) : ((v10 - 5) / 10);
}

/** 把实况 + 预报刷到天气页上（轮询 500ms 调一次；内容没变就直接返回）。*/
static void wx_refresh_ui(void)
{
    if (!s_wx_visible || s_wx_page == NULL) {
        return;
    }
    const ha_entity_t *e = ha_client_find(s_wx_entity);
    ha_wx_forecast_t fc;
    const bool have_fc = ha_client_weather_forecast_copy(&fc);

    /* 便宜的变化检测：实况的数字 + 预报时间戳，一样就不重排任何标签 */
    uint32_t sum = 0;
    if (e != NULL) {
        sum = (uint32_t)(e->wx_temp10 + 100) * 31u + (uint32_t)(e->wx_feel10 + 100) * 17u
            + (uint32_t)e->wx_humidity * 7u + (uint32_t)e->wx_cloud * 5u
            + (uint32_t)(e->wx_press + 2000) * 3u + (uint32_t)(e->wx_dew10 + 100)
            + (uint32_t)e->wx_vis_km + (uint32_t)e->wx_wind_kmh * 11u
            + (uint32_t)e->wx_kind * 13u + (uint32_t)e->wx_valid * 999u;
        for (const char *p = e->wx_wind_bearing; *p != '\0'; p++) {
            sum = sum * 33u + (uint8_t)*p;
        }
        for (const char *p = e->state; *p != '\0'; p++) {
            sum = sum * 33u + (uint8_t)*p;
        }
    }
    sum = sum * 131u + fc.stamp_s + (uint32_t)fc.hour_n * 977u + (uint32_t)fc.day_n * 389u
        + (uint32_t)have_fc * 7919u;
    if (sum == s_wx_sum_shown) {
        return;
    }
    s_wx_sum_shown = sum;

    char buf[48];
    if (e == NULL || !e->wx_valid) {
        lv_label_set_text(s_wx_cond, "--");
        lv_label_set_text(s_wx_big, "--");
        lv_label_set_text(s_wx_unit, "");
        lv_label_set_text(s_wx_feel, "体感 --");
        for (int i = 0; i < WX_CELL_MAX; i++) {
            lv_label_set_text(s_wx_cell_val[i], "--");
        }
        wx_set_icon(s_wx_icon_big, HA_WX_CLOUDY);
    } else {
        lv_label_set_text(s_wx_cond, weather_state_zh(e->entity_id, e->state));
        wx_set_icon(s_wx_icon_big, (int)ha_wx_kind_of(e->state));
        snprintf(buf, sizeof(buf), "%d", wx_r10(e->wx_temp10));
        lv_label_set_text(s_wx_big, buf);
        lv_label_set_text(s_wx_unit, "°");
        snprintf(buf, sizeof(buf), "体感 %d°", wx_r10(e->wx_feel10));
        lv_label_set_text(s_wx_feel, buf);
        /* 六格：湿度 / 云量 / 风 / 气压 / 能见度 / 露点 */
        snprintf(buf, sizeof(buf), "%d%%", e->wx_humidity);
        lv_label_set_text(s_wx_cell_val[0], buf);
        snprintf(buf, sizeof(buf), "%d%%", e->wx_cloud);
        lv_label_set_text(s_wx_cell_val[1], buf);
        /* 风向：HA 给的是"东北风"，格子里去掉尾字"风"（设计稿写"东北 14"）*/
        char bearing[16];
        snprintf(bearing, sizeof(bearing), "%.15s", e->wx_wind_bearing);
        size_t bl = strlen(bearing);
        if (bl >= 3 && strcmp(bearing + bl - 3, "风") == 0) {
            bearing[bl - 3] = '\0';
        }
        if (bearing[0] != '\0') {
            snprintf(buf, sizeof(buf), "%s %d", bearing, (int)e->wx_wind_kmh);
        } else {
            snprintf(buf, sizeof(buf), "%d", (int)e->wx_wind_kmh);
        }
        lv_label_set_text(s_wx_cell_val[2], buf);
        snprintf(buf, sizeof(buf), "%d", (int)e->wx_press);
        lv_label_set_text(s_wx_cell_val[3], buf);
        snprintf(buf, sizeof(buf), "%dkm", (int)e->wx_vis_km);
        lv_label_set_text(s_wx_cell_val[4], buf);
        snprintf(buf, sizeof(buf), "%d°", wx_r10(e->wx_dew10));
        lv_label_set_text(s_wx_cell_val[5], buf);
    }
    wx_place_big();

    /* 逐小时：不够 6 条就把多出来的行整行藏掉 */
    for (int i = 0; i < HA_WX_HOURLY; i++) {
        const bool show = have_fc && i < fc.hour_n;
        detail_set_hidden(s_wx_hour_row[i], !show);
        if (!show) {
            continue;
        }
        snprintf(buf, sizeof(buf), "%d时", (int)fc.hour[i].hour);
        lv_label_set_text(s_wx_hour_t[i], buf);
        wx_set_icon(s_wx_hour_ic[i], (int)fc.hour[i].kind);
        snprintf(buf, sizeof(buf), "%d%%", (int)fc.hour[i].pop);
        lv_label_set_text(s_wx_hour_p[i], buf);
        snprintf(buf, sizeof(buf), "%d°", (int)fc.hour[i].temp);
        lv_label_set_text(s_wx_hour_v[i], buf);
    }

    /* 逐日：今天 / 明天 / 周X（day_off >= 2 时按星期几写）*/
    static const char *const k_week[7] = {"周日", "周一", "周二", "周三", "周四", "周五", "周六"};
    for (int i = 0; i < HA_WX_DAILY; i++) {
        const bool show = have_fc && i < fc.day_n;
        detail_set_hidden(s_wx_day_row[i], !show);
        if (!show) {
            continue;
        }
        const char *day = (fc.day[i].day_off == 0) ? "今天"
                          : (fc.day[i].day_off == 1) ? "明天"
                                                     : k_week[fc.day[i].weekday % 7];
        lv_label_set_text(s_wx_day_d[i], day);
        wx_set_icon(s_wx_day_ic[i], (int)fc.day[i].kind);
        lv_label_set_text(s_wx_day_c[i], wx_kind_zh(fc.day[i].kind));
        snprintf(buf, sizeof(buf), "%d° / %d°", (int)fc.day[i].tmax, (int)fc.day[i].tmin);
        lv_label_set_text(s_wx_day_r[i], buf);
    }
}

/** 旋钮在天气页里滚动（正转 = 往下看），每格 40px（正好一行逐小时）。 */
static void wx_scroll_step(int delta)
{
    if (s_wx_scroll == NULL) {
        return;
    }
    lv_obj_scroll_by_bounded(s_wx_scroll, 0, -delta * WX_SCROLL_STEP, LV_ANIM_ON);
}

/** 打开天气页：滑入 + 藏掉下面的页面 / 标题 / 圆点（和详情页同一套）。*/
static void wx_open(const char *entity_id)
{
    if (s_wx_page == NULL || entity_id == NULL) {
        return;
    }
    snprintf(s_wx_entity, sizeof(s_wx_entity), "%s", entity_id);
    ESP_LOGI(TAG, "打开天气页: %s", entity_id);
    s_wx_visible = true;
    s_wx_sum_shown = 0;
    s_wx_big_x = -1;
    s_wx_unit_x = -1;
    if (s_wx_scroll != NULL) {
        lv_obj_scroll_to_y(s_wx_scroll, 0, LV_ANIM_OFF);
    }
    wx_refresh_ui();

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

    lv_obj_remove_flag(s_wx_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_wx_page, 0, 0);
    lv_obj_set_style_translate_x(s_wx_page, 360, 0);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_wx_page);
    lv_anim_set_values(&a, 360, 0);
    lv_anim_set_time(&a, 220);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, anim_translate_cb);
    lv_anim_start(&a);
    s_animating = true;
}

static void wx_close_done_cb(lv_anim_t *a)
{
    (void)a;
    if (s_wx_page != NULL) {
        lv_obj_add_flag(s_wx_page, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_x(s_wx_page, 0, 0);
        lv_obj_set_pos(s_wx_page, 0, 0);
    }
    if (s_page >= 0 && s_page < s_page_count && s_pages[s_page]) {
        lv_obj_remove_flag(s_pages[s_page], LV_OBJ_FLAG_HIDDEN);
    }
    if (s_title) {
        lv_obj_remove_flag(s_title, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < s_page_count; i++) {
        if (s_dots[i]) {
            lv_obj_remove_flag(s_dots[i], LV_OBJ_FLAG_HIDDEN);
            dot_style(s_page, i);
        }
    }
    s_animating = false;
}

/** 关闭天气页（左边缘右滑 / MQTT 都可以调）。*/
static void wx_close(void)
{
    if (s_wx_page == NULL || !s_wx_visible) {
        return;
    }
    s_wx_visible = false;
    s_animating = true;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_wx_page);
    lv_anim_set_values(&a, 0, 360);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&a, anim_translate_cb);
    lv_anim_set_completed_cb(&a, wx_close_done_cb);
    lv_anim_start(&a);
}

bool ui_home_weather_open(void)
{
    return s_wx_visible;
}

void ui_home_weather_show(const char *entity_id)
{
    wx_open(entity_id);
}

void ui_home_weather_hide(void)
{
    wx_close();
}

/* ------------------------------------------------------------------------- */
/* 传感器详情页（设计稿"屏幕 8 · 传感器详情 / 数值型"，已按设计变更去掉历史曲线）*/
/*                                                                           */
/* 设计稿里这一页原本是三段：.s-head2（名字）、.s-value（大号数值 + 单位）、     */
/* .s-range / .s-stats（"近 3 小时 · N 次采样" + 最小/平均/最大）。             */
/* 设计把"历史曲线"整套取消后，后两段的数据源（历史采样）已经不存在，所以本页    */
/* 只保留**非历史**的部分：                                                    */
/*   .s-head2   top 74，居中（图标 16 + gap 6 + 名字），16px #D7DEE9、字距 .04em */
/*   .s-value   大号数值 + 单位（单位 montserrat_20、opacity .6）               */
/* 数值块放在 y=128（字墨中心 151 = 圆屏光学中心）：原设计下半屏是统计块，       */
/* 去掉统计后若仍从 106 起，整块会明显偏上。文字型传感器（state 不是数字）用     */
/* 同一套位置显示状态文字（cjk16）。                                          */
/* 本页是覆盖层（和灯 / 空调 / 天气同一套滑入滑出），没有拨盘、不接管旋钮：       */
/* 旋钮仍然翻页（main.c 的 ui_home_page_step 照常走），左缘右滑关闭。            */
/* 本页不取任何历史数据（不调用 ha_client_history）。                           */
/* ------------------------------------------------------------------------- */
#define SENS_NAME_Y       74
#define SENS_VALUE_Y     128     /* 64 号数字：字墨 129..174，中心 151（圆屏光学中心）*/
#define SENS_UNIT_DY      14
#define SENS_ICON         16

static lv_obj_t *s_sens_page;
static lv_obj_t *s_sens_icon;
static lv_obj_t *s_sens_name_row;      /* 图标 + 名字的 flex 容器 */
static lv_obj_t *s_sens_name;
static lv_obj_t *s_sens_value;
static lv_obj_t *s_sens_unit;
static bool s_sens_visible;
static char s_sens_entity[96];
static uint8_t s_sens_icon_kind;         /* 0=温度计 1=水滴 2=功率 3=通用 */
static char s_sens_num_shown[24];
static char s_sens_unit_shown[12];
static int s_sens_num_x = -1;
static int s_sens_unit_x = -1;

static void sensor_open(const char *entity_id);
static void sensor_close(void);
static void sensor_refresh(void);

/** 传感器图标（16x16，自绘）：按单位选温度计 / 水滴 / 功率 / 通用。 */
static void sensor_icon_draw_cb(lv_event_t *e)
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
    const lv_color_t c = lv_color_hex(0x8D97A8);      /* 设计稿 .s-head2 .ic */
    switch (s_sens_icon_kind) {
    case 1:                                           /* 湿度：水滴 */
        ac_dr_dot(layer, cx, cy + 2, 3, c);
        ac_dr_line(layer, cx, cy - 7, cx - 3, cy + 1, 2, c);
        ac_dr_line(layer, cx, cy - 7, cx + 3, cy + 1, 2, c);
        break;
    case 2:                                           /* 功率：闪电 */
        ac_dr_line(layer, cx + 1, cy - 7, cx - 3, cy, 2, c);
        ac_dr_line(layer, cx - 3, cy, cx + 2, cy, 2, c);
        ac_dr_line(layer, cx + 2, cy, cx - 2, cy + 7, 2, c);
        break;
    case 0:                                           /* 温度：温度计 */
        ac_dr_line(layer, cx, cy - 7, cx, cy + 1, 2, c);
        ac_dr_dot(layer, cx, cy + 3, 3, c);
        break;
    default:                                          /* 通用：小圆 + 十字 */
        ac_dr_arc(layer, cx, cy, 5, 2, 0, 360, c);
        ac_dr_dot(layer, cx, cy, 1, c);
        break;
    }
}

/** 大号数值 + 单位一起居中（数字用 num64，文字型退回 cjk16）。 */
static void sensor_place_value(void)
{
    if (s_sens_value == NULL || s_sens_unit == NULL) {
        return;
    }
    lv_point_t nsz = {0, 0};
    lv_point_t usz = {0, 0};
    const char *ntxt = lv_label_get_text(s_sens_value);
    const char *utxt = lv_label_get_text(s_sens_unit);
    if (ntxt != NULL && ntxt[0] != '\0') {
        lv_text_get_size(&nsz, ntxt, lv_obj_get_style_text_font(s_sens_value, 0), 0, 0,
                         LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    }
    if (utxt != NULL && utxt[0] != '\0') {
        lv_text_get_size(&usz, utxt, &lv_font_montserrat_20, 0, 0, LV_COORD_MAX,
                         LV_TEXT_FLAG_NONE);
    }
    const int gap = (usz.x > 0) ? 3 : 0;
    const int x0 = 180 - (nsz.x + gap + usz.x) / 2;
    if (s_sens_num_x != x0) {
        s_sens_num_x = x0;
        lv_obj_set_pos(s_sens_value, x0, SENS_VALUE_Y);
    }
    const int ux = x0 + nsz.x + gap;
    if (s_sens_unit_x != ux) {
        s_sens_unit_x = ux;
        lv_obj_set_pos(s_sens_unit, ux, SENS_VALUE_Y + SENS_UNIT_DY);
    }
}

/** 数字格式化：整数且 |v| >= 100 不带小数，否则一位小数。 */
static void sensor_fmt(char *buf, size_t n, float v)
{
    const float av = (v < 0) ? -v : v;
    if (av >= 100.0f || (float)(int)v == v) {
        snprintf(buf, n, "%d", (int)(v + ((v >= 0) ? 0.5f : -0.5f)));
    } else {
        snprintf(buf, n, "%.1f", (double)v);
    }
}

/** 按单位挑图标类型。 */
static void sensor_pick_icon(const char *unit)
{
    s_sens_icon_kind = 3;
    if (unit == NULL) {
        return;
    }
    if (strcmp(unit, "%") == 0) {
        s_sens_icon_kind = 1;
    } else if (strstr(unit, "°") != NULL || strcasecmp(unit, "C") == 0) {
        s_sens_icon_kind = 0;
    } else if (strcasecmp(unit, "W") == 0 || strcasecmp(unit, "kW") == 0 ||
               strcasecmp(unit, "kWh") == 0) {
        s_sens_icon_kind = 2;
    }
}

/** 把当前值刷到页面上（500ms 轮询 + 打开时各一次）。 */
static void sensor_refresh(void)
{
    if (!s_sens_visible || s_sens_page == NULL || s_sens_value == NULL) {
        return;
    }
    const ha_entity_t *e = ha_client_find(s_sens_entity);
    char num[24] = "--";
    char unit[12] = "";
    const lv_font_t *font = BIG_NUM_FONT;

    if (e != NULL && e->fresh && strcmp(e->state, "unavailable") != 0 &&
        strcmp(e->state, "unknown") != 0) {
        char *end = NULL;
        const double d = strtod(e->state, &end);
        const bool numeric = (end != NULL && end != e->state && *end == '\0');
        if (numeric) {
            sensor_fmt(num, sizeof(num), (float)d);
            snprintf(unit, sizeof(unit), "%s", e->unit);
            font = BIG_NUM_FONT;
        } else {
            /* 文字型（开 / 关 / 制水中…）：同一套布局，数值换成状态文字 */
            snprintf(num, sizeof(num), "%s", e->state);
            font = &lv_font_cjk16;
        }
    }
    if (strcmp(num, s_sens_num_shown) != 0) {
        snprintf(s_sens_num_shown, sizeof(s_sens_num_shown), "%s", num);
        lv_label_set_text(s_sens_value, num);
        lv_obj_set_style_text_font(s_sens_value, font, 0);
    }
    if (strcmp(unit, s_sens_unit_shown) != 0) {
        snprintf(s_sens_unit_shown, sizeof(s_sens_unit_shown), "%s", unit);
        lv_label_set_text(s_sens_unit, unit);
    }
    sensor_place_value();
}

static void sensor_open(const char *entity_id)
{
    if (s_sens_page == NULL || entity_id == NULL) {
        return;
    }
    snprintf(s_sens_entity, sizeof(s_sens_entity), "%s", entity_id);
    const ha_entity_t *e = ha_client_find(entity_id);
    ESP_LOGI(TAG, "打开传感器页: %s", entity_id);
    s_sens_visible = true;
    s_sens_num_shown[0] = '\x01';
    s_sens_unit_shown[0] = '\x01';
    s_sens_num_x = -1;
    s_sens_unit_x = -1;
    s_sens_icon_kind = 3;

    lv_label_set_text(s_sens_name, (e != NULL && e->name[0]) ? e->name : entity_id);
    sensor_pick_icon((e != NULL) ? e->unit : NULL);
    if (s_sens_icon != NULL) {
        lv_obj_invalidate(s_sens_icon);
    }
    sensor_refresh();

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

    lv_obj_remove_flag(s_sens_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_sens_page, 0, 0);
    lv_obj_set_style_translate_x(s_sens_page, 360, 0);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_sens_page);
    lv_anim_set_values(&a, 360, 0);
    lv_anim_set_time(&a, 220);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, anim_translate_cb);
    lv_anim_start(&a);
    s_animating = true;
}

static void sensor_close_done_cb(lv_anim_t *a)
{
    (void)a;
    if (s_sens_page != NULL) {
        lv_obj_add_flag(s_sens_page, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_translate_x(s_sens_page, 0, 0);
        lv_obj_set_pos(s_sens_page, 0, 0);
    }
    if (s_page >= 0 && s_page < s_page_count && s_pages[s_page]) {
        lv_obj_remove_flag(s_pages[s_page], LV_OBJ_FLAG_HIDDEN);
    }
    if (s_title) {
        lv_obj_remove_flag(s_title, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < s_page_count; i++) {
        if (s_dots[i]) {
            lv_obj_remove_flag(s_dots[i], LV_OBJ_FLAG_HIDDEN);
            dot_style(s_page, i);
        }
    }
    s_animating = false;
}

static void sensor_close(void)
{
    if (s_sens_page == NULL || !s_sens_visible) {
        return;
    }
    s_sens_visible = false;
    s_animating = true;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_sens_page);
    lv_anim_set_values(&a, 0, 360);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&a, anim_translate_cb);
    lv_anim_set_completed_cb(&a, sensor_close_done_cb);
    lv_anim_start(&a);
}

static void build_sensor_page(lv_obj_t *scr)
{
    s_sens_page = lv_obj_create(scr);
    if (s_sens_page == NULL) {
        return;
    }
    lv_obj_remove_style_all(s_sens_page);
    lv_obj_set_size(s_sens_page, 360, 360);
    lv_obj_set_pos(s_sens_page, 0, 0);
    lv_obj_set_style_bg_color(s_sens_page, lv_color_hex(0x05070B), 0);
    lv_obj_set_style_bg_opa(s_sens_page, LV_OPA_COVER, 0);
    lv_obj_remove_flag(s_sens_page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_sens_page, LV_OBJ_FLAG_HIDDEN);

    /* 名字行：和详情页同样的 flex 居中（x=60..300），图标 + 名字自动居中 */
    s_sens_name_row = lv_obj_create(s_sens_page);
    lv_obj_remove_style_all(s_sens_name_row);
    lv_obj_set_size(s_sens_name_row, 240, 20);
    lv_obj_set_pos(s_sens_name_row, 60, SENS_NAME_Y);
    lv_obj_set_flex_flow(s_sens_name_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_sens_name_row, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(s_sens_name_row, 6, 0);
    lv_obj_set_style_pad_all(s_sens_name_row, 0, 0);
    lv_obj_remove_flag(s_sens_name_row, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    s_sens_icon = lv_obj_create(s_sens_name_row);
    lv_obj_remove_style_all(s_sens_icon);
    lv_obj_set_size(s_sens_icon, SENS_ICON, SENS_ICON);
    lv_obj_remove_flag(s_sens_icon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_sens_icon, sensor_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_sens_name = lv_label_create(s_sens_name_row);
    lv_label_set_text(s_sens_name, "");
    lv_obj_set_style_text_font(s_sens_name, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_sens_name, lv_color_hex(0xD7DEE9), 0);
    lv_obj_set_style_text_letter_space(s_sens_name, 1, 0);
    lv_label_set_long_mode(s_sens_name, LV_LABEL_LONG_DOT);
    lv_obj_set_width(s_sens_name, LV_SIZE_CONTENT);
    lv_obj_set_style_max_width(s_sens_name, 200, 0);

    s_sens_value = lv_label_create(s_sens_page);
    lv_label_set_text(s_sens_value, "--");
    lv_obj_set_style_text_font(s_sens_value, BIG_NUM_FONT, 0);
    lv_obj_set_style_text_color(s_sens_value, lv_color_white(), 0);
    lv_obj_set_pos(s_sens_value, 120, SENS_VALUE_Y);

    s_sens_unit = lv_label_create(s_sens_page);
    lv_label_set_text(s_sens_unit, "");
    lv_obj_set_style_text_font(s_sens_unit, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_sens_unit, lv_color_hex(0xE8EEF8), 0);
    lv_obj_set_style_text_opa(s_sens_unit, 153, 0);             /* 设计稿 opacity .6 */
    lv_obj_set_pos(s_sens_unit, 200, SENS_VALUE_Y + SENS_UNIT_DY);

    ESP_LOGI(TAG, "传感器页建好：名字 y=%d / 数值 y=%d（无历史曲线）", SENS_NAME_Y, SENS_VALUE_Y);
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

    /* 区域点击：旧版是"环心 = 色温 / 环带 = 亮度"，灯光页现在用分段控件，
       这个回调只剩占位（见函数注释）。 */
    lv_obj_add_event_cb(s_detail, detail_region_pressed_cb, LV_EVENT_PRESSED, NULL);

    /* ---- 灯光页的氛围光三层圆盘（设计稿 .light-bg 的径向渐变近似）----
       必须在环之前建：它们在底层，环压在上面；透明度/颜色由 detail_apply_glow 刷。 */
    for (int i = 0; i < GLOW_DISC_N; i++) {
        s_glow_disc[i] = detail_make_glow_disc(s_detail, k_glow_r[i], 0, DETAIL_CX, 151);
    }

    /* ---- 圆盘：先光晕（在下面），再圆环（在上面）----
       灯光页按设计稿是"单环 r=156 / 10px"：亮度和色温共用同一个半径，
       哪个档位生效就显示哪一条（detail_apply_focus 负责藏另一条）。 */
    s_detail_br_glow = detail_make_arc(s_detail, LIGHT_GLOW_R, LIGHT_GLOW_W, true);
    lv_arc_set_range(s_detail_br_glow, 1, 100);
    lv_arc_set_value(s_detail_br_glow, 1);

    s_detail_ct_glow = detail_make_arc(s_detail, LIGHT_GLOW_R, LIGHT_GLOW_W, true);
    lv_arc_set_range(s_detail_ct_glow, 2000, 6500);
    lv_arc_set_value(s_detail_ct_glow, 4000);

    /* 亮度环 = 1..100%（0% 会被 HA 当成关灯，电源只归电源键）*/
    s_detail_br_arc = detail_make_arc(s_detail, LIGHT_R, LIGHT_W, false);
    lv_arc_set_range(s_detail_br_arc, 1, 100);
    lv_arc_set_value(s_detail_br_arc, 1);
    lv_obj_add_event_cb(s_detail_br_arc, detail_dial_pressed_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_detail_br_arc, detail_dial_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_detail_br_arc, detail_dial_changed_cb, LV_EVENT_RELEASED, NULL);

    /* 色温环：同一个半径，范围在 detail_apply_entity 里按实体刷新 */
    s_detail_ct_arc = detail_make_arc(s_detail, LIGHT_R, LIGHT_W, false);
    lv_arc_set_range(s_detail_ct_arc, 2000, 6500);
    lv_arc_set_value(s_detail_ct_arc, 4000);
    /* 原生指示灯永不绘制：这一段是自绘的暖→冷渐变 */
    lv_obj_set_style_arc_opa(s_detail_ct_arc, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_add_event_cb(s_detail_ct_arc, detail_ct_grad_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_add_event_cb(s_detail_ct_arc, detail_dial_pressed_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_detail_ct_arc, detail_dial_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_detail_ct_arc, detail_dial_changed_cb, LV_EVENT_RELEASED, NULL);

    /* 灯的环 / 光晕以屏幕中心 (180,180) 为心（detail_make_arc 默认按 180,196）*/
    light_recenter(s_detail_br_glow, LIGHT_GLOW_R);
    light_recenter(s_detail_ct_glow, LIGHT_GLOW_R);
    light_recenter(s_detail_br_arc, LIGHT_R);
    light_recenter(s_detail_ct_arc, LIGHT_R);

    /* 顶上那个 36x36 返回键已经删掉：退出详情页走"从左边缘右滑"（swipe_cb 的
       边缘返回手势），所以顶部带里只剩状态栏。 */

    /* ---- 中间堆栈：实体名 / 灯泡 / 大号数值 + 单位 / 档位文字 ----
       名字建在圆环之后：光晕不会盖住它。整行是一个 240x20 的 flex 容器（x=60..300），
       图标 + 文字由 flex 自动居中，不再手动量宽度（那套会被遗留的 align 样式覆盖）。 */
    /* 名字行：一个 240x20 的 flex 行（x=60..300，正好是文字安全圆的宽度），
       图标和名字都是它的子对象，由 flex **自动居中**——不再"量文字宽度再摆位置"。
       旧实现用手动测量 + lv_obj_set_pos，但标签身上还留着 build 时的
       LV_STYLE_ALIGN(TOP_MID)，下一次 layout 又把它按对齐样式摆回去，
       于是名字跑到屏幕右上角还被圆屏切掉。flex 布局不会出现这个问题。 */
    s_detail_name_row = lv_obj_create(s_detail);
    lv_obj_remove_style_all(s_detail_name_row);
    lv_obj_set_size(s_detail_name_row, 240, 20);
    lv_obj_set_pos(s_detail_name_row, 60, LIGHT_NAME_Y);
    lv_obj_set_flex_flow(s_detail_name_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_detail_name_row, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(s_detail_name_row, 6, 0);
    lv_obj_set_style_pad_all(s_detail_name_row, 0, 0);
    lv_obj_set_style_bg_opa(s_detail_name_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_detail_name_row, 0, 0);
    lv_obj_remove_flag(s_detail_name_row, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_detail_name_row, LV_OBJ_FLAG_HIDDEN);       /* 还没拿到名字：整行不画 */

    s_detail_icon = lv_obj_create(s_detail_name_row);
    lv_obj_remove_style_all(s_detail_icon);
    lv_obj_set_size(s_detail_icon, LIGHT_ICON, LIGHT_ICON);
    lv_obj_remove_flag(s_detail_icon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_detail_icon, detail_bulb_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_detail_name = lv_label_create(s_detail_name_row);
    lv_label_set_text(s_detail_name, "");
    lv_obj_set_style_text_font(s_detail_name, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_detail_name, LIGHT_INK, 0);
    lv_label_set_long_mode(s_detail_name, LV_LABEL_LONG_DOT);     /* 再长也"…" */
    lv_obj_set_style_text_align(s_detail_name, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(s_detail_name, LV_SIZE_CONTENT);
    lv_obj_set_style_max_width(s_detail_name, 200, 0);            /* 余下 16+6 给图标 */
    lv_obj_add_flag(s_detail_name, LV_OBJ_FLAG_HIDDEN);           /* 还没拿到名字：整行不画 */

    s_detail_br_value = lv_label_create(s_detail);
    lv_label_set_text(s_detail_br_value, "--");
    /* 项目里已经编进来的最大号字（montserrat_28），不新增字体 */
    lv_obj_set_style_text_font(s_detail_br_value, BIG_NUM_FONT, 0);
    lv_obj_set_style_text_color(s_detail_br_value, COLOR_TEXT, 0);
    lv_obj_set_pos(s_detail_br_value, 160, LIGHT_VALUE_Y);
    lv_obj_add_flag(s_detail_br_value, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_detail_br_value, detail_focus_br_cb, LV_EVENT_CLICKED, NULL);

    /* 单位（% / K）：设计稿里比数字小一号、低 8px；位置由 light_place_value 算 */
    s_detail_ct_value = lv_label_create(s_detail);
    lv_label_set_text(s_detail_ct_value, "%");
    lv_obj_set_style_text_font(s_detail_ct_value, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_detail_ct_value, lv_color_white(), 0);
    lv_obj_set_style_text_opa(s_detail_ct_value, 184, 0);
    lv_obj_set_pos(s_detail_ct_value, 200, LIGHT_VALUE_Y + LIGHT_UNIT_DY);
    lv_obj_add_flag(s_detail_ct_value, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_detail_ct_value, detail_focus_ct_cb, LV_EVENT_CLICKED, NULL);

    /* 档位文字「亮度 / 色温 / 已关闭」（设计稿 .l-label，16px + 字距 .24em）*/
    s_detail_off_hint = lv_label_create(s_detail);
    lv_label_set_text(s_detail_off_hint, "亮度");
    lv_obj_set_style_text_font(s_detail_off_hint, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_detail_off_hint, LIGHT_INK_MUTED, 0);
    lv_obj_set_style_text_letter_space(s_detail_off_hint, 4, 0);
    lv_obj_align(s_detail_off_hint, LV_ALIGN_TOP_MID, 0, LIGHT_LABEL_Y);

    /* ---------------- 空调那一组（同一个容器，默认全藏） ----------------
       设计稿 dial 帧，和灯 / 风扇同一骨架：环 r=156/10（外径 161，环心 180,180）+ 光晕；
       名字行（名字行走共用 flex 行，图标是当前模式的小图标）；数值 y=100；
       副标题 y=160；胶囊行 y=186（风速 + 摆风 + 44x44 电源键）；5 个 40x40 模式键 y=226。 */
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
    light_recenter(s_ac_glow, AC_TEMP_GLOW_R);
    light_recenter(s_ac_arc, AC_TEMP_R);

    /* 名字行左侧：当前模式的 16px 图标（灯是灯泡 / 风扇是风图标，空调是模式图标）*/
    s_ac_name_icon = lv_obj_create(s_detail_name_row);
    lv_obj_remove_style_all(s_ac_name_icon);
    lv_obj_set_size(s_ac_name_icon, LIGHT_ICON, LIGHT_ICON);
    lv_obj_remove_flag(s_ac_name_icon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_ac_name_icon, ac_name_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_add_flag(s_ac_name_icon, LV_OBJ_FLAG_HIDDEN);      /* 灯 / 风扇页不显示 */

    /* 大号目标温度（num64）+ °（montserrat_20、模式色、字墨中心对齐）*/
    s_ac_temp_value = lv_label_create(s_detail);
    lv_label_set_text(s_ac_temp_value, "--");
    lv_obj_set_style_text_font(s_ac_temp_value, BIG_NUM_FONT, 0);
    lv_obj_set_style_text_color(s_ac_temp_value, COLOR_TEXT, 0);
    lv_obj_set_pos(s_ac_temp_value, 120, AC_VALUE_Y);

    s_ac_unit = lv_label_create(s_detail);
    lv_label_set_text(s_ac_unit, "°");
    lv_obj_set_style_text_font(s_ac_unit, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_ac_unit, lv_color_white(), 0);
    lv_obj_set_pos(s_ac_unit, 200, AC_VALUE_Y + AC_UNIT_DY);

    /* 副标题「制冷 · 室内 26.4°C」（设计稿 .d-unit）*/
    s_ac_room = lv_label_create(s_detail);
    lv_label_set_text(s_ac_room, "室内 --");
    lv_obj_set_style_text_font(s_ac_room, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_ac_room, lv_color_hex(0x9AA4B5), 0);
    lv_obj_set_style_text_align(s_ac_room, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(s_ac_room, 260);
    lv_obj_set_pos(s_ac_room, 50, AC_ROOM_Y);

    /* 胶囊行：风速 100 + 8 + 摆风 88 + 10 + 电源 44 = 250 → x 55..305 */
    {
        const int row_w = AC_FAN_W + AC_CHIP_GAP + AC_SWING_W + AC_POWER_GAP + AC_POWER;
        const int x0 = DETAIL_CX - row_w / 2;                 /* 55 */

        s_ac_fan_btn = lv_obj_create(s_detail);
        if (s_ac_fan_btn != NULL) {
            lv_obj_remove_style_all(s_ac_fan_btn);
            lv_obj_set_size(s_ac_fan_btn, AC_FAN_W, AC_CHIP_H);
            lv_obj_set_pos(s_ac_fan_btn, x0, AC_CHIP_Y);
            lv_obj_set_style_radius(s_ac_fan_btn, AC_CHIP_H / 2, 0);
            lv_obj_set_style_bg_color(s_ac_fan_btn, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_ac_fan_btn, 12, 0);              /* 4.5% */
            lv_obj_set_style_border_width(s_ac_fan_btn, 1, 0);
            lv_obj_set_style_border_color(s_ac_fan_btn, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_ac_fan_btn, 18, 0);          /* 7% */
            lv_obj_remove_flag(s_ac_fan_btn, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(s_ac_fan_btn, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(s_ac_fan_btn, ac_fan_cb, LV_EVENT_CLICKED, NULL);
            s_ac_fan_label = lv_label_create(s_ac_fan_btn);
            lv_label_set_text(s_ac_fan_label, "风速 --");
            lv_obj_set_style_text_font(s_ac_fan_label, &lv_font_cjk16, 0);
            lv_obj_set_style_text_color(s_ac_fan_label, lv_color_hex(0xC2CBDB), 0);
            lv_obj_center(s_ac_fan_label);
        }

        s_ac_swing_btn = lv_obj_create(s_detail);
        if (s_ac_swing_btn != NULL) {
            lv_obj_remove_style_all(s_ac_swing_btn);
            lv_obj_set_size(s_ac_swing_btn, AC_SWING_W, AC_CHIP_H);
            lv_obj_set_pos(s_ac_swing_btn, x0 + AC_FAN_W + AC_CHIP_GAP, AC_CHIP_Y);
            lv_obj_set_style_radius(s_ac_swing_btn, AC_CHIP_H / 2, 0);
            lv_obj_set_style_bg_color(s_ac_swing_btn, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_ac_swing_btn, 12, 0);
            lv_obj_set_style_border_width(s_ac_swing_btn, 1, 0);
            lv_obj_set_style_border_color(s_ac_swing_btn, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_ac_swing_btn, 18, 0);
            lv_obj_remove_flag(s_ac_swing_btn, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(s_ac_swing_btn, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(s_ac_swing_btn, ac_swing_cb, LV_EVENT_CLICKED, NULL);
            s_ac_swing_label = lv_label_create(s_ac_swing_btn);
            lv_label_set_text(s_ac_swing_label, "摆风");
            lv_obj_set_style_text_font(s_ac_swing_label, &lv_font_cjk16, 0);
            lv_obj_set_style_text_color(s_ac_swing_label, lv_color_hex(0xC2CBDB), 0);
            lv_obj_center(s_ac_swing_label);
        }
    }

    /* 5 个模式键：固定顺序 auto/cool/heat/dry/fan_only，只显示实体支持的；
       键里只有图标（设计稿 .m-row 就是图标键），当前模式名在副标题里 */
    for (int i = 0; i < AC_MODE_COUNT; i++) {
        const int x = DETAIL_CX - (AC_MODE_COUNT * AC_MODE_W + (AC_MODE_COUNT - 1) * AC_MODE_GAP) / 2
                      + i * (AC_MODE_W + AC_MODE_GAP);
        lv_obj_t *btn = lv_obj_create(s_detail);
        s_ac_mode_btn[i] = btn;
        lv_obj_remove_style_all(btn);
        lv_obj_set_size(btn, AC_MODE_W, AC_MODE_H);
        lv_obj_set_pos(btn, x, AC_MODE_Y);
        lv_obj_set_style_radius(btn, 13, 0);
        lv_obj_set_style_bg_color(btn, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_white(), 0);
        lv_obj_set_style_border_opa(btn, 14, 0);
        lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(btn, ac_mode_cb, LV_EVENT_CLICKED, (void *)&k_ac_modes[i]);

        s_ac_key_col[i] = lv_color_hex(0x6F798C);
        lv_obj_t *ic = lv_obj_create(btn);
        s_ac_mode_icon[i] = ic;
        lv_obj_remove_style_all(ic);
        lv_obj_set_size(ic, 20, 20);
        lv_obj_center(ic);
        lv_obj_remove_flag(ic, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(ic, ac_key_icon_draw_cb, LV_EVENT_DRAW_MAIN, (void *)(intptr_t)i);
    }

    /* ---------------- 风扇那一组（同一个容器，默认全藏） ----------------
       布局照设计稿 .scr[data-scr="fan"]，和灯光页同一个骨架：
       环 r=156/10（外径 161，环心 180,180）+ 光晕；名字行（风图标 + 实体名）
       y=66；大号百分比 + "%" y=136；「风速」y=196；
       胶囊行 y=226：预设（点一下换下一个）+ 摇头（置灰：不支持）+ 40x40 电源键。
       电源键是灯光页建的那颗（两页不会同时出现），x 位置由 detail_place_power 摆。 */
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
    light_recenter(s_fan_glow, FAN_GLOW_R);
    light_recenter(s_fan_arc, FAN_R);

    /* 名字行左侧的风图标（设计稿 #i-wind）：位置由名字行整体居中算出来，
       这里先给个占位，进页面后 fan 那一路会把它和名字一起摆好。 */
    s_fan_name_icon = lv_obj_create(s_detail_name_row);
    lv_obj_remove_style_all(s_fan_name_icon);
    lv_obj_set_size(s_fan_name_icon, LIGHT_ICON, LIGHT_ICON);
    lv_obj_remove_flag(s_fan_name_icon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_fan_name_icon, fan_wind_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_add_flag(s_fan_name_icon, LV_OBJ_FLAG_HIDDEN);          /* 灯 / 空调页不显示风图标 */

    /* 大号百分比 + "%"（单位比数字小一号、低 8px，位置由 fan_place_value 算）*/
    s_fan_pct = lv_label_create(s_detail);
    lv_label_set_text(s_fan_pct, "--");
    lv_obj_set_style_text_font(s_fan_pct, BIG_NUM_FONT, 0);
    lv_obj_set_style_text_color(s_fan_pct, COLOR_TEXT, 0);
    lv_obj_set_pos(s_fan_pct, 150, LIGHT_VALUE_Y);

    s_fan_unit = lv_label_create(s_detail);
    lv_label_set_text(s_fan_unit, "%");
    lv_obj_set_style_text_font(s_fan_unit, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_fan_unit, lv_color_white(), 0);
    lv_obj_set_style_text_opa(s_fan_unit, 184, 0);
    lv_obj_set_pos(s_fan_unit, 200, LIGHT_VALUE_Y + LIGHT_UNIT_DY);

    /* 「风速」（设计稿 .l-label：16px + 字距 .24em）*/
    s_fan_label = lv_label_create(s_detail);
    lv_label_set_text(s_fan_label, "风速");
    lv_obj_set_style_text_font(s_fan_label, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(s_fan_label, LIGHT_INK_MUTED, 0);
    lv_obj_set_style_text_letter_space(s_fan_label, 4, 0);
    lv_obj_align(s_fan_label, LV_ALIGN_TOP_MID, 0, LIGHT_LABEL_Y);

    /* 胶囊行：预设 94 + 间距 6 + 摇头 78 + 间距 6 + 电源键 40 = 224，x 68..292 */
    {
        const int row_w = FAN_CHIP_PRESET_W + FAN_CHIP_GAP + FAN_CHIP_OSC_W
                          + FAN_CHIP_GAP + LIGHT_POWER;
        const int x0 = DETAIL_CX - row_w / 2;              /* 68 */

        s_fan_chip_preset = lv_obj_create(s_detail);
        if (s_fan_chip_preset != NULL) {
            lv_obj_remove_style_all(s_fan_chip_preset);
            lv_obj_set_size(s_fan_chip_preset, FAN_CHIP_PRESET_W, FAN_CHIP_H);
            lv_obj_set_pos(s_fan_chip_preset, x0, FAN_CHIP_ROW_Y);
            lv_obj_set_style_radius(s_fan_chip_preset, FAN_CHIP_H / 2, 0);
            lv_obj_set_style_bg_color(s_fan_chip_preset, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_fan_chip_preset, 12, 0);          /* 4.5% */
            lv_obj_set_style_border_width(s_fan_chip_preset, 1, 0);
            lv_obj_set_style_border_color(s_fan_chip_preset, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_fan_chip_preset, 18, 0);      /* 7% */
            lv_obj_remove_flag(s_fan_chip_preset, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(s_fan_chip_preset, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(s_fan_chip_preset, fan_preset_cycle_cb, LV_EVENT_CLICKED, NULL);
            s_fan_chip_preset_lbl = lv_label_create(s_fan_chip_preset);
            lv_label_set_text(s_fan_chip_preset_lbl, "自然风");
            lv_obj_set_style_text_font(s_fan_chip_preset_lbl, &lv_font_cjk16, 0);
            lv_obj_set_style_text_color(s_fan_chip_preset_lbl, lv_color_hex(0xC2CBDB), 0);
            lv_obj_center(s_fan_chip_preset_lbl);
        }

        /* 摇头：这台风扇没有 OSCILLATE 能力 → 保留位置、整颗置灰、不可点 */
        s_fan_chip_osc = lv_obj_create(s_detail);
        if (s_fan_chip_osc != NULL) {
            lv_obj_remove_style_all(s_fan_chip_osc);
            lv_obj_set_size(s_fan_chip_osc, FAN_CHIP_OSC_W, FAN_CHIP_H);
            lv_obj_set_pos(s_fan_chip_osc, x0 + FAN_CHIP_PRESET_W + FAN_CHIP_GAP, FAN_CHIP_ROW_Y);
            lv_obj_set_style_radius(s_fan_chip_osc, FAN_CHIP_H / 2, 0);
            lv_obj_set_style_bg_color(s_fan_chip_osc, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_fan_chip_osc, 12, 0);
            lv_obj_set_style_border_width(s_fan_chip_osc, 1, 0);
            lv_obj_set_style_border_color(s_fan_chip_osc, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_fan_chip_osc, 18, 0);
            lv_obj_set_style_opa(s_fan_chip_osc, FAN_CHIP_DISABLED_OPA, 0);   /* 40% */
            lv_obj_remove_flag(s_fan_chip_osc, LV_OBJ_FLAG_SCROLLABLE);       /* 不可点 */
            s_fan_chip_osc_lbl = lv_label_create(s_fan_chip_osc);
            lv_label_set_text(s_fan_chip_osc_lbl, "摇头");
            lv_obj_set_style_text_font(s_fan_chip_osc_lbl, &lv_font_cjk16, 0);
            lv_obj_set_style_text_color(s_fan_chip_osc_lbl, lv_color_hex(0xC2CBDB), 0);
            lv_obj_center(s_fan_chip_osc_lbl);
        }
        ESP_LOGI(TAG, "风扇页：摇头控件置灰（supported_features=57 无 OSCILLATE）");
    }

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
    lv_obj_set_style_text_letter_space(s_detail_hint, 2, 0);
    lv_obj_set_width(s_detail_hint, 240);
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

    /* ---- 灯光页控件行（设计稿 .l-ctl，top 230、居中、gap 10）----
       分段控件「亮度 | 色温」134x36（白 4.5% 底 + 白 7% 边、圆角 18、
       选中项白 13% 底 + 近白字）+ 40x40 圆形电源键。
       电源键和风扇页共用（两页不会同时出现），横向位置由 detail_place_power 摆。 */
    {
        const int row_w = LIGHT_SEG_W + LIGHT_CTL_GAP + LIGHT_POWER;
        const int x0 = DETAIL_CX - row_w / 2;              /* 88 */
        s_light_seg = lv_obj_create(s_detail);
        if (s_light_seg != NULL) {
            lv_obj_remove_style_all(s_light_seg);
            lv_obj_set_size(s_light_seg, LIGHT_SEG_W, LIGHT_SEG_H);
            lv_obj_set_pos(s_light_seg, x0, LIGHT_SEG_Y);
            lv_obj_set_style_radius(s_light_seg, LIGHT_SEG_H / 2, 0);
            lv_obj_set_style_bg_color(s_light_seg, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_light_seg, 12, 0);              /* 4.5% */
            lv_obj_set_style_border_width(s_light_seg, 1, 0);
            lv_obj_set_style_border_color(s_light_seg, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_light_seg, 18, 0);           /* 7% */
            lv_obj_remove_flag(s_light_seg, LV_OBJ_FLAG_SCROLLABLE);
            for (int i = 0; i < 2; i++) {
                lv_obj_t *btn = lv_obj_create(s_light_seg);
                s_light_seg_btn[i] = btn;
                lv_obj_remove_style_all(btn);
                lv_obj_set_size(btn, LIGHT_BTN_W, LIGHT_BTN_H);
                lv_obj_set_pos(btn, LIGHT_SEG_PAD + i * (LIGHT_BTN_W + LIGHT_SEG_GAP),
                               LIGHT_SEG_PAD);
                lv_obj_set_style_radius(btn, LIGHT_BTN_H / 2, 0);
                lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
                lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
                lv_obj_add_event_cb(btn, light_tab_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
                if (btn == NULL) {
                    continue;
                }
                lv_obj_t *lbl = lv_label_create(btn);
                s_light_seg_lbl[i] = lbl;
                lv_label_set_text(lbl, i == 0 ? "亮度" : "色温");
                lv_obj_set_style_text_font(lbl, &lv_font_cjk16, 0);
                lv_obj_set_style_text_color(lbl, LIGHT_SEG_TXT, 0);
                lv_obj_center(lbl);
            }
        }
        s_dial_power = lv_obj_create(s_detail);
        if (s_dial_power != NULL) {
            lv_obj_remove_style_all(s_dial_power);
            lv_obj_set_size(s_dial_power, LIGHT_POWER, LIGHT_POWER);
            lv_obj_set_pos(s_dial_power, x0 + LIGHT_SEG_W + LIGHT_CTL_GAP,
                           LIGHT_SEG_Y + (LIGHT_SEG_H - LIGHT_POWER) / 2);
            lv_obj_set_style_radius(s_dial_power, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(s_dial_power, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(s_dial_power, 15, 0);              /* 6% */
            lv_obj_set_style_border_width(s_dial_power, 1, 0);
            lv_obj_set_style_border_color(s_dial_power, lv_color_white(), 0);
            lv_obj_set_style_border_opa(s_dial_power, 23, 0);          /* 9% */
            lv_obj_remove_flag(s_dial_power, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(s_dial_power, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(s_dial_power, light_power_cb, LV_EVENT_CLICKED, NULL);
            lv_obj_add_event_cb(s_dial_power, light_power_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
        }
    }

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
    s_fan_chip_txt[0] = '\0';
    s_fan_chip_shown = -1;
    s_fan_num_x = -1;
    s_fan_unit_x = -1;
    s_fan_pct_txt[0] = '\0';
    s_fan_accent = lv_color_hex(0x2DD4BF);
    /* 灯光页的缓存 / 电源键位置：新容器上全部作废，第一帧才会真的写进去 */
    s_light_num_x = -1;
    s_light_unit_x = -1;
    s_light_chip_shown = -1;
    s_pwr_x_shown = -1;
    s_hint_y_shown = -1;
    s_light_num_shown[0] = '\x01';
    s_light_unit_shown[0] = '\x01';
    s_light_label_shown[0] = '\x01';
    s_ac_num_x = -1;
    s_ac_unit_x = -1;
    s_pwr_size_shown = -1;
    s_pwr_y_shown = -1;
    detail_place_power(LIGHT_POWER, 232, LIGHT_SEG_Y + (LIGHT_SEG_H - LIGHT_POWER) / 2);
    ac_hide_all(true);
    fan_hide_all(true);
    detail_apply_focus();
    detail_update_ambience();       /* 底色 / 环色 / 自绘图标都先落到一个确定状态 */

    ESP_LOGI(TAG, "详情页建好：%d 个对象（灯：单环 + 光晕 + 灯泡 + 大号数值/单位 + 档位文字 + "
                  "分段控件 + 圆形电源键；空调：温度环 + 光晕 + 模式图标 + 大号温度 + 室内 + 风速 + "
                  "5 模式键；风扇：风速环 + 光晕 + 大号百分比 + 预设胶囊 + 摇头胶囊（置灰）；"
                  "共用：环心实体名 + 值点/提示；无返回键，退出靠左边缘右滑）",
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
    s_fan_chip_shown = -1;
    s_fan_pct_override_until = 0;
    s_fan_preset_shown[0] = '\x01';     /* 换实体：颜色/预设键必须重刷一遍 */
    s_fan_chip_txt[0] = '\0';
    s_fan_chip_shown = -1;
    s_fan_num_x = -1;
    s_fan_unit_x = -1;
    s_fan_pct_txt[0] = '\0';
    s_detail_fg_dirty = true;
    /* 灯光页的文字 / 外观缓存：换实体后必须重写一遍标签 */
    s_light_num_x = -1;
    s_light_unit_x = -1;
    s_light_chip_shown = -1;
    s_pwr_x_shown = -1;
    s_hint_y_shown = -1;
    s_light_num_shown[0] = '\x01';
    s_light_unit_shown[0] = '\x01';
    s_light_label_shown[0] = '\x01';
    s_ac_num_x = -1;
    s_ac_unit_x = -1;
    s_ac_room_txt[0] = '\0';
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
            dot_style(s_page, i);
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

/* ---------------- 历史折线图：设计稿已移除，只留 ABI 桩 ---------------- */
/* 设计稿把"历史曲线"整套取消了：这里不再建页、不再取历史。
   main.c 的 MQTT 调试命令 ui_home_show_chart(...) 和 screenshot.c 还引用这三个符号，
   所以保留同名空实现（否则 main.c 链接不过，而 main.c 不归本任务改）。
   现在没有任何页面、卡片或手势能打开历史视图，也没有任何代码路径会拉历史数据。 */
void ui_home_show_chart(const char *entity_id, const float *vals, int n,
                        float vmin, float vmax, float vavg)
{
    (void)vals;
    (void)n;
    (void)vmin;
    (void)vmax;
    (void)vavg;
    ESP_LOGW(TAG, "历史曲线已取消（设计稿移除），忽略 ui_home_show_chart(%s)",
             entity_id ? entity_id : "(null)");
}

void ui_home_close_chart(void)
{
    /* 没有历史页可关：保留空实现给 screenshot.c / assistant_ui.c 调用 */
}

bool ui_home_chart_open(void)
{
    return false;      /* 永远没有历史页 */
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

    /* 标题：设计稿 .h-title —— top 56、16px/600、字距 .1em、#F4F7FC。
       cjk16 行高 20，所以盒子放 y=48（字墨约 50..70）。设置页自己的内容从 y=64 起，
       那边仍用 y=30（title_place() 按页选）。 */
    s_title = lv_label_create(scr);
    const char *first_title = page_title(0);
    lv_label_set_text(s_title, first_title);
    lv_obj_set_style_text_color(s_title, lv_color_hex(0xF4F7FC), 0);
    lv_obj_set_style_text_font(s_title, title_font_for(first_title), 0);
    lv_obj_set_style_text_letter_space(s_title, 2, 0);
    lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 48);

    /* ---- 状态栏：图标贴在 r=152 的圆上（设计稿 .h-status，角度 -120/-90/-60）----
       设计稿是"四个独立图标按极坐标摆"，不是一条弧形色带，所以 LVGL 里不需要裁剪 /
       遮罩：算出圆心坐标再按图标自身尺寸居中即可。三个图标（蓝牙 / HA / WiFi）
       在弧上等距；设计稿第 4 个是电量，固件没有标定过的电量百分比（只有 ADC 原值），
       所以这里不放电量图标，避免画一个永远"未知"的电量。 */

    s_wifi_color = COLOR_MUTED;
    s_wifi_icon = lv_obj_create(scr);
    if (s_wifi_icon != NULL) {
        lv_obj_remove_style_all(s_wifi_icon);
        lv_obj_set_size(s_wifi_icon, 22, 18);
        lv_obj_remove_flag(s_wifi_icon, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(s_wifi_icon, wifi_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
        place_on_arc(s_wifi_icon, 300);          /* 右上（-60°）*/
    }

    /* 蓝牙：灰 = 未启用/关闭，蓝 = 扫描中/已连接（颜色由 refresh_bt_icon 刷） */
    s_bt_icon = lv_label_create(scr);
    if (s_bt_icon != NULL) {
        lv_label_set_text(s_bt_icon, LV_SYMBOL_BLUETOOTH);
        lv_obj_set_style_text_font(s_bt_icon, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(s_bt_icon, COLOR_MUTED, 0);
        place_on_arc(s_bt_icon, 240);            /* 左上（-120°）*/
    }

    /* 电量：自绘图标（设计稿 #sbBatt），这块板没有校准 → 永远画"未知"态（空壳 + 斜杠）*/
    s_batt_icon = lv_obj_create(scr);
    if (s_batt_icon != NULL) {
        lv_obj_remove_style_all(s_batt_icon);
        lv_obj_set_size(s_batt_icon, 22, 22);
        lv_obj_remove_flag(s_batt_icon, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(s_batt_icon, battery_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
        place_on_arc(s_batt_icon, 260);          /* 左上偏中（-100°）*/
    }

    /* Home Assistant：房子图标（LV_SYMBOL_HOME），在线 HA 蓝、离线灰 */
    s_ha_icon = lv_label_create(scr);
    if (s_ha_icon != NULL) {
        lv_label_set_text(s_ha_icon, LV_SYMBOL_HOME);
        lv_obj_set_style_text_font(s_ha_icon, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(s_ha_icon, COLOR_MUTED, 0);
        place_on_arc(s_ha_icon, 280);            /* 右上偏中（-80°）*/
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

        /* 卡片放进一个可纵向滚动的容器（设计稿 .grid2：左上 (34,88)、2 列 140、间距 12）。
           4 张卡刚好两行；布局给 5 张以上时下面几行靠上下滑动看（滚动条隐藏，
           和设计稿一样不露滚动条）。 */
        lv_obj_t *grid = lv_obj_create(page);
        lv_obj_remove_style_all(grid);
        lv_obj_set_size(grid, CARD_W * 2 + CARD_GAP, 202);   /* 34..326 / 88..290 */
        lv_obj_set_pos(grid, GRID_LEFT, GRID_TOP);
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

    /* 翻页圆点（设计稿 .h-dots：top 294、间距 7；当前页 14x5 胶囊）*/
    if (s_page_count > 1) {
        for (int i = 0; i < s_page_count; i++) {
            int cx = 180 + (int)((i * 2 - (s_page_count - 1)) * 8); /* 间距 16 */
            lv_obj_t *hit = lv_obj_create(scr);
            lv_obj_set_size(hit, 20, 20);
            lv_obj_set_style_bg_opa(hit, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(hit, 0, 0);
            lv_obj_set_pos(hit, cx - 10, 284);          /* 命中框 284..304，点居中在 294 */
            lv_obj_add_flag(hit, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(hit, dot_clicked_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

            lv_obj_t *dot = lv_obj_create(hit);
            lv_obj_set_size(dot, 5, 5);
            lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(dot, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(dot, 46, 0);
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
    build_weather_page(scr);     /* 天气页（覆盖层，旋钮用它自己滚列表）*/
    build_sensor_page(scr);      /* 传感器详情页（覆盖层，旋钮不接管）*/

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
