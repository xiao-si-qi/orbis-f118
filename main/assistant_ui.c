/**
 * @file assistant_ui.c
 * @brief 小智助手页：纯白底 + 会动的脸（两只眼睛 + 一张嘴）+ 中文状态文字。
 *
 * 它是主界面（ui_home）的最后一页：ui_home 建好页面对象后调用
 * assistant_ui_build(page)，本模块把内容画进去，翻页/圆点/手势全按普通页走，
 * 不再有"二级悬浮页"。
 *
 * 为什么这么画（这块板子内存很紧，内部 RAM 只剩几十 KB）：
 *   - 不用任何图片资源：眼睛是带圆角的 lv_obj，嘴是 lv_arc / 小圆 / 药丸块，
 *     思考时是一圈转动的 lv_arc，全部靠"改尺寸、改角度"动起来；
 *   - 一共十几个对象、没有额外缓冲，动画在一个 50ms 的 LVGL 定时器里算，
 *     页面不可见时把定时器 pause 掉，不空转；
 *   - ui_home 重建界面（收到新 cfg）会把整棵控件树 lv_obj_clean 掉，这里用
 *     LV_EVENT_DELETE 把指针清空，下次 build 再建。
 *
 * 表情（对应 xiaozhi_state()）：
 *   IDLE       圆眼睛 + 微笑弧，每 3~6 秒眨一次眼
 *   LISTENING  眼睛略放大、轻轻上下浮动，嘴变成小 "o"，头顶一个呼吸的录音点
 *   THINKING   眼睛压成两道短线，嘴的位置是一圈转动的弧
 *   SPEAKING   嘴（药丸块）按两个正弦叠加张合，眼睛跟着微微眯，副标题显示 TTS 字幕
 *   ERROR      眼睛压平、颜色转灰，副标题显示服务端给的原因
 *   CONNECTING 正常眼睛 + 一条短横线的嘴，等握手
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   /* esp_rom_printf：裸 UART0 标记，先于任何日志/格式化解码 */
#include "lvgl.h"

#include "assistant_ui.h"
#include "ui_home.h"        /* 翻页：ui_home_goto_assistant() / ui_home_goto_first() */
#include "xiaozhi_client.h"

LV_FONT_DECLARE(lv_font_cjk16);

static const char *TAG = "assistant";

/* ---- 配色：白底黑脸（小智原版风格）---- */
#define C_BG      lv_color_hex(0xFFFFFF)
#define C_INK     lv_color_hex(0x1C1C1E)
#define C_MUTED   lv_color_hex(0x8E8E93)
#define C_ACCENT  lv_color_hex(0x2563EB)
#define C_TRACK   lv_color_hex(0xD9D9DE)
#define C_CHIP    lv_color_hex(0xF2F2F7)

/* ---- 几何：360x360 圆屏，内容压在 y=50..320 的安全带里 ---- */
#define EYE_CY    150
#define EYE_CX_L  146
#define EYE_CX_R  214
#define MOUTH_CX  180
#define MOUTH_CY  210

#define ANIM_MS   50              /* 表情动画节奏：20 帧/秒 */

/* ---- 副标题（最近一句识别结果 / 小智说的话）：最多两行，超长打点 ---- */
#define SUB_W     276             /* 圆屏 y≈295 处可见宽度约 276px，别越界 */
#define SUB_LINES 2
#define SUB_MAX   136

static lv_obj_t *s_page;          /* 调用方（ui_home）给的父对象，不是我们建的 */
static lv_obj_t *s_dot;           /* 聆听时的录音指示点 */
static lv_obj_t *s_eye[2];
static lv_obj_t *s_mouth_smile;   /* 微笑弧（空闲）*/
static lv_obj_t *s_mouth_o;       /* 小 "o"（聆听）*/
static lv_obj_t *s_mouth_line;    /* 药丸块（说话时张合 / 出错时一条线）*/
static lv_obj_t *s_think_arc;     /* 思考时转圈的弧 */
static lv_obj_t *s_status;        /* 待机 / 正在聆听… */
static lv_obj_t *s_sub;           /* 小智说的话 / 最近一句识别结果 / 出错原因 */
static lv_obj_t *s_hint;          /* 激活码 / 操作提示 */

static lv_timer_t *s_anim_timer;
static bool s_visible;            /* 页面当前是否可见（由 ui_home 翻页控制）*/
static bool s_need_sync;          /* 刚变成可见：下一次定时器先整帧刷新一次 */
static xz_state_t s_face = (xz_state_t)-1;

/** 脸是不是都建好了。LV_USE_ASSERT_* 全关时，LVGL 不会替我们挡空指针，
    所以每次动这些对象前都要先过一遍。 */
static bool face_ready(void)
{
    return s_page != NULL && s_eye[0] != NULL && s_eye[1] != NULL &&
           s_mouth_smile != NULL && s_mouth_o != NULL && s_mouth_line != NULL &&
           s_think_arc != NULL && s_dot != NULL;
}

static uint32_t s_tick;
static uint32_t s_seed = 0x2545F491u;
static int s_blink_wait;
static int s_blink_phase;
static bool s_blink_active;

/* 文字缓存：内容没变就不动 LVGL 标签（500ms 一次也别白重排）*/
static char s_txt_status[24];
static char s_txt_sub[SUB_MAX];
static char s_txt_hint[32];
static char s_sub_src[SUB_MAX];   /* 副标题原文：没变就不用重新排版 */

static lv_point_t s_press;
static bool s_swiped;

/* 防御用的"只报一次"标志：50ms/500ms 定时器里出错不能每帧刷一条日志。 */
static bool s_log_stale_page;         /* 页面指针失效（不在活动屏幕下）*/
static bool s_log_no_parent;          /* 建控件时 parent 为 NULL */
static bool s_log_no_label;           /* 文字控件是 NULL */
static bool s_log_no_sub;             /* 副标题控件是 NULL */
static bool s_log_null_activation;    /* xiaozhi_activation_code() 返回 NULL */
static bool s_log_null_last;          /* xiaozhi_last_text() 返回 NULL */
static bool s_log_null_tts;           /* xiaozhi_last_tts_text() 返回 NULL */
static bool s_log_null_status;        /* xiaozhi_client_status_text() 返回 NULL */

/* ------------------------------------------------------------------------- */
/* 小工具                                                                     */
/* ------------------------------------------------------------------------- */

/** 同一种失败只记一条日志：日志链本身有开销，定时器里更不能刷屏。 */
static void log_once(bool *flag, const char *what)
{
    if (flag == NULL || *flag) {
        return;
    }
    *flag = true;
    ESP_LOGW(TAG, "%s", what);           /* what 永远是字符串常量 */
}

/**
 * 上游（xiaozhi_client / LVGL）给的字符串一律过这一道：NULL 当空串用。
 * 绝不能把 NULL 交给 snprintf("%s") / lv_label_set_text()：LV_USE_ASSERT_* 全关时，
 * 那就是 ROM 里 strlen/memcpy/vsnprintf 的一次 Load access fault
 * （现场 MEPC 落在 ROM、A2/A3/A4 全 0 就是这个形状）。
 */
static const char *str_or_empty(const char *s, bool *warned, const char *who)
{
    if (s != NULL) {
        return s;
    }
    log_once(warned, who);
    return "";
}

/**
 * 把整张脸的状态清干净（指针全部置 NULL + 停掉动画定时器）。
 * page_delete_cb / build 失败 / build 开始时都走这里，保证不会有"半旧半新"的指针。
 * why 非 NULL 时顺带记一条日志（只在指针还都有效时调用它）。
 */
static void reset_face(const char *why)
{
    s_page = NULL;
    s_dot = NULL;
    s_eye[0] = NULL;
    s_eye[1] = NULL;
    s_mouth_smile = NULL;
    s_mouth_o = NULL;
    s_mouth_line = NULL;
    s_think_arc = NULL;
    s_status = NULL;
    s_sub = NULL;
    s_hint = NULL;
    s_visible = false;
    s_need_sync = false;
    s_face = (xz_state_t)-1;
    s_txt_status[0] = '\0';
    s_txt_sub[0] = '\0';
    s_txt_hint[0] = '\0';
    s_sub_src[0] = '\0';
    if (s_anim_timer != NULL) {
        lv_timer_pause(s_anim_timer);    /* 定时器不在对象树里，留着下次复用 */
    }
    if (why != NULL) {
        ESP_LOGW(TAG, "%s", why);
    }
}

/**
 * 在控件树里找 target（只比较指针，**不解引用 target**）。深度有限，代价可控。
 * 这样就算 future 的 ui_home 把页面多包一层容器，也不会误判成"页面已销毁"。
 */
static bool obj_in_tree(lv_obj_t *root, const lv_obj_t *target, int depth)
{
    if (root == NULL || depth < 0) {
        return false;
    }
    if (root == target) {
        return true;
    }
    const uint32_t n = lv_obj_get_child_count(root);
    for (uint32_t i = 0; i < n; i++) {
        if (obj_in_tree(lv_obj_get_child(root, i), target, depth - 1)) {
            return true;
        }
    }
    return false;
}

/**
 * 当前页面是否还挂在活动屏幕上。ui_home 的页都是 lv_screen_active() 下面的对象，
 * 这里只遍历控件树里的指针、不解引用 s_page，所以就算 s_page 已经是指向
 * 已释放内存的野指针也是安全的（这正是 500ms 定时器最怕的情况）。
 * 发现页面已经没了就顺手清状态：后面所有 lv_* 调用都会被 NULL 挡掉。
 */
static bool page_alive(void)
{
    if (s_page == NULL) {
        return false;
    }
    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL) {
        return false;
    }
    if (obj_in_tree(scr, s_page, 3)) {
        return true;
    }
    log_once(&s_log_stale_page,
             "小智页指针已失效（不在活动屏幕下），清空表情/文字状态，等下次 build 重建");
    reset_face(NULL);
    return false;
}

static const char *face_name(xz_state_t st)
{
    switch (st) {
    case XZ_CONNECTING: return "连接中";
    case XZ_LISTENING:  return "聆听";
    case XZ_THINKING:   return "思考";
    case XZ_SPEAKING:   return "说话";
    case XZ_ERROR:      return "出错";
    default:            return "待机";
    }
}

static const char *face_text(xz_state_t st, bool ready)
{
    switch (st) {
    case XZ_LISTENING:  return "正在聆听…";
    case XZ_THINKING:   return "思考中…";
    case XZ_SPEAKING:   return "正在说话";
    case XZ_CONNECTING: return "连接中…";
    case XZ_ERROR:      return "连接出错";
    default:            return ready ? "待机" : "未启用";
    }
}

/** 一块纯色圆角矩形（眼睛、嘴、指示点都用它，改尺寸就能动）。 */
static lv_obj_t *make_block(lv_obj_t *parent, lv_coord_t w, lv_coord_t h,
                            lv_color_t color, lv_coord_t radius)
{
    if (parent == NULL) {              /* 页面已经销毁：别再往下建 */
        log_once(&s_log_no_parent, "小智脸：parent 为 NULL，跳过控件创建");
        return NULL;
    }
    lv_obj_t *o = lv_obj_create(parent);
    if (o == NULL) {
        ESP_LOGE(TAG, "小智脸：控件创建失败（LVGL 内存不足）");
        return NULL;
    }
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_bg_color(o, color, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(o, radius, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_scrollable(o, false);
    lv_obj_set_clickable(o, false);      /* 点脸要落到页面本身（页面才处理轻触）*/
    return o;
}

/** 中文标签：居中；height<=0 表示按内容自适应，>0 表示固定高度、放不下打点。 */
static lv_obj_t *make_label(lv_obj_t *parent, lv_color_t color, lv_coord_t width,
                            lv_coord_t height)
{
    if (parent == NULL) {
        log_once(&s_log_no_parent, "小智脸：parent 为 NULL，跳过文字控件创建");
        return NULL;
    }
    lv_obj_t *l = lv_label_create(parent);
    if (l == NULL) {
        ESP_LOGE(TAG, "小智脸：文字控件创建失败（LVGL 内存不足）");
        return NULL;
    }
    lv_obj_set_style_text_font(l, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(l, color, 0);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(l, width);
    if (height > 0) {
        lv_obj_set_height(l, height);
    }
    lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
    lv_label_set_text(l, "");
    lv_obj_set_clickable(l, false);
    return l;
}

/** 眼睛：以 (EYE_CX_*, EYE_CY) 为中心改尺寸，眨眼的闭合看起来是"往下压"。 */
static void set_eyes(lv_coord_t w, lv_coord_t h)
{
    if (h < 6) {
        h = 6;
    }
    if (w <= 0) {
        w = 36;
    }
    for (int i = 0; i < 2; i++) {
        if (s_eye[i] == NULL) {      /* 半张脸（创建失败/页面已销毁）：跳过，绝不把 NULL 交给 LVGL */
            continue;
        }
        const lv_coord_t cx = (i == 0) ? EYE_CX_L : EYE_CX_R;
        lv_obj_set_size(s_eye[i], w, h);
        lv_obj_set_pos(s_eye[i], cx - w / 2, EYE_CY - h / 2);
    }
}

static void set_text_if(lv_obj_t *label, char *cache, size_t cap, const char *text)
{
    if (label == NULL) {
        log_once(&s_log_no_label, "文字控件是 NULL（没建出来或页面已销毁），跳过这次文字刷新");
        return;
    }
    if (cache == NULL || cap == 0) {
        return;
    }
    if (text == NULL) {              /* 兜底：任何空指针都不许进 snprintf("%s") */
        text = "";
    }
    if (strcmp(cache, text) == 0) {
        return;
    }
    snprintf(cache, cap, "%s", text);
    lv_label_set_text(label, cache);
}

/**
 * 把一句话裁成"最多 lines 行"：先按显示宽度量，超了就从尾部按 UTF-8 字符回退，
 * 最后补一个 "…"。这样长句只会被省略，不会把圆屏撑破。
 * 只在文字变化时调用（结果有缓存），所以这里的逐字测量不会拖慢 500ms 刷新。
 */
static void fit_lines(char *dst, size_t cap, const char *src, lv_coord_t width, int lines)
{
    if (dst == NULL || cap == 0) {
        return;
    }
    dst[0] = '\0';
    if (src == NULL || src[0] == '\0') {
        return;
    }
    if (width <= 0 || lines <= 0) {   /* 参数不合法：宁可空着，也不要拿负宽度去排版 */
        return;
    }

    const size_t ell = sizeof("…") - 1;                 /* "…" = 3 字节 */
    const size_t keep = (cap > ell + 1) ? cap - ell - 1 : cap;   /* 给 "…" 留位置 */
    size_t n = strlen(src);
    if (n > keep) {
        n = keep;
        while (n > 0 && ((unsigned char)src[n] & 0xC0) == 0x80) {
            n--;
        }
    }
    memcpy(dst, src, n);
    dst[n] = '\0';

    const int32_t max_h = lv_font_get_line_height(&lv_font_cjk16) * lines;
    lv_point_t sz;
    lv_text_get_size(&sz, dst, &lv_font_cjk16, 0, 0, width, LV_TEXT_FLAG_NONE);
    if (sz.y <= max_h) {
        return;                                          /* 放得下：原样显示（含换行）*/
    }

    while (n > 0) {
        n--;
        while (n > 0 && ((unsigned char)dst[n] & 0xC0) == 0x80) {
            n--;                                         /* 退到字符首字节 */
        }
        dst[n] = '\0';
        lv_text_get_size(&sz, dst, &lv_font_cjk16, 0, 0, width - 16, LV_TEXT_FLAG_NONE);
        if (sz.y <= max_h) {
            break;
        }
    }
    snprintf(dst + n, cap - n, "…");
}

/* ------------------------------------------------------------------------- */
/* 状态 → 表情                                                                */
/* ------------------------------------------------------------------------- */

static void face_apply(xz_state_t st)
{
    if (!face_ready()) {
        return;
    }
    s_face = st;
    s_tick = 0;
    s_blink_wait = 0;
    s_blink_phase = 0;
    s_blink_active = false;

    const bool err = (st == XZ_ERROR);
    const lv_color_t ink = err ? C_MUTED : C_INK;

    lv_obj_set_style_bg_color(s_eye[0], ink, 0);
    lv_obj_set_style_bg_color(s_eye[1], ink, 0);
    lv_obj_set_style_arc_color(s_mouth_smile, ink, LV_PART_INDICATOR);
    lv_obj_set_style_border_color(s_mouth_o, ink, 0);
    lv_obj_set_style_bg_color(s_mouth_line, ink, 0);

    /* 每种状态只露出该露的部件 */
    lv_obj_set_hidden(s_mouth_smile, st != XZ_IDLE);
    lv_obj_set_hidden(s_mouth_o, st != XZ_LISTENING);
    lv_obj_set_hidden(s_think_arc, st != XZ_THINKING);
    lv_obj_set_hidden(s_dot, st != XZ_LISTENING);

    const bool line_mouth = (st == XZ_SPEAKING || st == XZ_ERROR || st == XZ_CONNECTING);
    lv_obj_set_hidden(s_mouth_line, !line_mouth);
    if (st == XZ_CONNECTING) {
        lv_obj_set_size(s_mouth_line, 40, 6);
        lv_obj_set_pos(s_mouth_line, MOUTH_CX - 20, MOUTH_CY - 3);
    } else if (st == XZ_ERROR) {
        lv_obj_set_size(s_mouth_line, 30, 6);
        lv_obj_set_pos(s_mouth_line, MOUTH_CX - 15, MOUTH_CY - 3);
    }

    switch (st) {
    case XZ_THINKING:  set_eyes(34, 8);  break;   /* 两道短线 */
    case XZ_ERROR:     set_eyes(34, 6);  break;   /* 压平 */
    case XZ_LISTENING: set_eyes(40, 50); break;   /* 睁大 */
    case XZ_SPEAKING:  set_eyes(36, 42); break;
    default:           set_eyes(36, 46); break;
    }

    if (st == XZ_LISTENING) {
        lv_obj_set_style_bg_opa(s_dot, LV_OPA_COVER, 0);
    }
    if (st == XZ_THINKING) {
        lv_arc_set_rotation(s_think_arc, 0);
        lv_arc_set_angles(s_think_arc, 0, 90);
    }
    ESP_LOGI(TAG, "表情：%s", face_name(st));
}

/* 空闲：慢眨 + 微笑 */
static void anim_idle(void)
{
    if (!s_blink_active) {
        set_eyes(36, 46);
        if (++s_blink_wait >= 60 + (int)(s_seed & 0x3F)) {   /* 3.0 ~ 6.15 秒 */
            s_blink_active = true;
            s_blink_phase = 0;
            s_blink_wait = 0;
            s_seed = s_seed * 1103515245u + 12345u;          /* 省一个 rand() */
        }
        return;
    }
    s_blink_phase++;
    if (s_blink_phase <= 3) {
        set_eyes(36, 46 - s_blink_phase * 13);               /* 46 → 7 */
    } else if (s_blink_phase <= 6) {
        set_eyes(36, 7 + (s_blink_phase - 3) * 13);          /* 7 → 46 */
    } else {
        s_blink_active = false;
        set_eyes(36, 46);
    }
}

/* 聆听：眼睛轻轻浮动 + 录音点呼吸 */
static void anim_listen(void)
{
    if (!face_ready()) {
        return;
    }
    const int32_t s = lv_trigo_sin((int16_t)((s_tick * 7) % 360));
    const lv_opa_t opa = (lv_opa_t)(90 + (s + 32768) * 140 / 65536);   /* 90..230 */
    lv_obj_set_style_bg_opa(s_dot, opa, 0);

    const int dy = (int)((lv_trigo_sin((int16_t)((s_tick * 3) % 360)) * 2) / 32768);
    lv_obj_set_pos(s_eye[0], EYE_CX_L - 20, EYE_CY - 25 + dy);
    lv_obj_set_pos(s_eye[1], EYE_CX_R - 20, EYE_CY - 25 + dy);
}

/* 思考：嘴的位置转圈 */
static void anim_think(void)
{
    if (!face_ready()) {
        return;
    }
    lv_arc_set_rotation(s_think_arc, (int32_t)((s_tick * 10) % 360));  /* 1.8 秒一圈 */
}

/* 说话：嘴按两个频率张合 */
static void anim_speak(void)
{
    if (!face_ready()) {
        return;
    }
    const int32_t a = lv_trigo_sin((int16_t)((s_tick * 13) % 360));
    const int32_t b = lv_trigo_sin((int16_t)((s_tick * 7) % 360));
    int h = 12 + (int)((a + 32768) * 26 / 65536) + (int)((b + 32768) * 6 / 65536);
    if (h < 8) {
        h = 8;
    }
    if (h > 44) {
        h = 44;
    }
    lv_obj_set_size(s_mouth_line, 44, h);
    lv_obj_set_pos(s_mouth_line, MOUTH_CX - 22, MOUTH_CY - h / 2);
    set_eyes(36, 42 - h / 8);                     /* 嘴张得越大，眼睛越眯一点 */
}

/* ------------------------------------------------------------------------- */
/* 状态 / 文字刷新                                                            */
/* ------------------------------------------------------------------------- */

static void refresh_text(void)
{
    if (s_page == NULL) {
        return;
    }
    const xz_state_t st = s_face;
    const bool ready = xiaozhi_client_ready();
    /* 这四个 API 按约定"永远非 NULL"，但它们返回的是语音客户端内部的快照指针，
       而且客户端正在被改：这里一律当可能为 NULL 处理，绝不把 NULL 送进
       snprintf("%s") / lv_label_set_text()（那是 ROM 里 strlen/vsnprintf 的
       Load access fault，现场 MEPC 落在 ROM、A2/A3/A4 全 0 就是这个形状）。 */
    const char *code = str_or_empty(xiaozhi_activation_code(), &s_log_null_activation,
                                    "xiaozhi_activation_code() 返回 NULL，按空串处理");
    const char *last = str_or_empty(xiaozhi_last_text(), &s_log_null_last,
                                    "xiaozhi_last_text() 返回 NULL，按空串处理");
    const char *tts = str_or_empty(xiaozhi_last_tts_text(), &s_log_null_tts,
                                   "xiaozhi_last_tts_text() 返回 NULL，按空串处理");
    const char *stext = str_or_empty(xiaozhi_client_status_text(), &s_log_null_status,
                                     "xiaozhi_client_status_text() 返回 NULL，按空串处理");
    const char *sub_src = "";
    char hint[32] = "";

    if (st == XZ_ERROR) {
        sub_src = stext;                  /* 出错：显示服务端给的原因 */
    } else if (st == XZ_SPEAKING && tts[0] != '\0') {
        sub_src = tts;                    /* 正在说话：显示小智这句 TTS 字幕 */
    } else if (last[0] != '\0') {
        sub_src = last;                   /* 其它状态：还是最近一句识别结果 */
    }

    if (code[0] != '\0') {
        /* 激活码比什么都重要：单独一行，副标题让位 */
        snprintf(hint, sizeof(hint), "待激活 %s", code);
        sub_src = "";
    } else if (st == XZ_LISTENING) {
        snprintf(hint, sizeof(hint), "再次轻触结束");
    } else if (st == XZ_IDLE) {
        snprintf(hint, sizeof(hint), "%s", ready ? "轻触说话 · 滑动翻页" : "在 switchyard 打开语音");
    } else {
        snprintf(hint, sizeof(hint), "滑动翻页");
    }

    set_text_if(s_status, s_txt_status, sizeof(s_txt_status), face_text(st, ready));

    /* 原文没变就不重新排版（fit_lines 要逐字测量）。
       注意直接写进 s_txt_sub，不再在栈上开一个 SUB_MAX 的临时缓冲：
       500ms 定时器本来就跑在 LVGL 的定时器回调里，能省一点栈就省一点。 */
    if (strcmp(s_sub_src, sub_src) != 0) {
        snprintf(s_sub_src, sizeof(s_sub_src), "%s", sub_src);
        fit_lines(s_txt_sub, sizeof(s_txt_sub), sub_src, SUB_W, SUB_LINES);
        if (s_sub != NULL) {
            lv_label_set_text(s_sub, s_txt_sub);
        } else {
            log_once(&s_log_no_sub, "副标题控件是 NULL，跳过 TTS 字幕刷新");
        }
    }

    set_text_if(s_hint, s_txt_hint, sizeof(s_txt_hint), hint);
}

static void refresh(void)
{
    if (!s_visible || !face_ready()) {   /* 页面已销毁 / 脸没建全：一个 LVGL 对象都不碰 */
        return;
    }
    const xz_state_t st = xiaozhi_state();
    if (st != s_face) {
        face_apply(st);
    }
    refresh_text();
}

/* ------------------------------------------------------------------------- */
/* 交互                                                                       */
/* ------------------------------------------------------------------------- */

static void handle_tap(void)
{
    esp_rom_printf("[as] raw: tap\n");

    /* ↑ 裸串口标记：必须是第一行，而且只有 "\n"——先于任何 ESP_LOG / 格式化输出。
       现场判读：看到 [as] raw: tap 却看不到 [xz] raw: listen_start，说明卡在下面
       的状态判断/日志链；连这条都没有，说明 LVGL 的 CLICKED 都没派到本模块
       （问题在 ui_home 的控件树/事件绑定，不在这里）。 */
    if (!page_alive()) {
        return;                     /* 页面已经不在控件树里：一个对象都不碰 */
    }

    const xz_state_t st = xiaozhi_state();
    if (st == XZ_LISTENING) {
        xiaozhi_listen_stop();
        ESP_LOGI(TAG, "轻触：结束聆听");
        return;
    }
    if (st != XZ_IDLE) {
        ESP_LOGD(TAG, "轻触：状态 %s，忽略（思考/说话/连接中不打断）", face_name(st));
        return;                     /* 思考/说话/连接中：轻触不打断 */
    }
    if (!xiaozhi_client_ready()) {
        /* 状态文字同样可能没准备好：NULL 当空串，绝不交给 %s */
        const char *why = str_or_empty(xiaozhi_client_status_text(), &s_log_null_status,
                                       "xiaozhi_client_status_text() 返回 NULL，按空串处理");
        ESP_LOGW(TAG, "轻触：未连接，忽略（%s）", why);
        return;
    }
    if (xiaozhi_listen_start()) {
        ESP_LOGI(TAG, "轻触：开始聆听");
    } else {
        ESP_LOGW(TAG, "轻触：开麦请求被拒绝（详情见 xiaozhi 日志）");
    }
}

static void page_event_cb(lv_event_t *e)
{
    esp_rom_printf("[as] raw: page\n");

    /* ↑ 裸串口标记：区分"LVGL 回调进来了但 target 不是这一页"和"根本没进来"。
       每下轻触会打 1~3 条（按下/抬起/点击），只写 UART0，不走日志链。 */
    /* ui_home 给整棵控件树开了事件冒泡：只认落在页面/脸本身的触摸，
       忽略"音频自检"按钮等子对象冒上来的事件（否则点按钮会顺手开麦）。
       s_page 为 NULL（页面已销毁）时这里必然成立，直接返回。 */
    if (s_page == NULL || lv_event_get_target(e) != s_page) {
        return;
    }
    const lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_active();
    if (indev == NULL) {
        return;
    }
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (code == LV_EVENT_PRESSED) {
        s_press = p;
        s_swiped = false;
        return;
    }
    if (code == LV_EVENT_RELEASED) {
        const int dx = p.x - s_press.x;
        const int dy = p.y - s_press.y;
        const int ady = (dy < 0) ? -dy : dy;
        /* 横向拖得够远（左右都算）：这是翻页手势，交给 ui_home 处理，
           别再当成"轻触说话"（LVGL 在同对象上按下/抬起仍可能补一个 CLICKED）。 */
        if (abs(dx) > 40 && abs(dx) > ady) {
            s_swiped = true;
        }
        return;
    }
    if (code == LV_EVENT_CLICKED) {
        if (s_swiped) {
            s_swiped = false;       /* 刚才那下是滑动，不再当成轻触 */
            return;
        }
        handle_tap();
    }
}

static void page_delete_cb(lv_event_t *e)
{
    /* ui_home 重建界面时整棵控件树被删掉：指针必须清空，否则下次会踩野指针。
       先认一下这是不是"当前这一页"：万一某个旧页面的 DELETE 晚到了（重建时没解绑
       成功），绝不能让它把新页面的指针/可见性一起清掉。 */
    lv_obj_t *dead = lv_event_get_target(e);
    if (dead != NULL && s_page != NULL && dead != s_page) {
        ESP_LOGW(TAG, "小智页：收到旧页面的删除事件（%p != 当前 %p），忽略",
                 (void *)dead, (void *)s_page);
        return;
    }
    reset_face("小智页被界面重建移除（下次 build 会重新创建）");
}

static void anim_timer_cb(lv_timer_t *t)
{
    (void)t;
    /* 页面不可见 / 脸没建全 / 页面已被销毁：直接返回，绝不碰任何控件指针。
       （page_alive() 只遍历控件树指针、不解引用 s_page，发现页面没了会顺手清状态。） */
    if (!s_visible || !face_ready() || !page_alive()) {
        return;
    }

    if (s_need_sync) {                 /* 刚翻到这一页：先整帧对齐一次 */
        s_need_sync = false;
        face_apply(xiaozhi_state());
        refresh_text();
    }

    const xz_state_t st = xiaozhi_state();
    if (st != s_face) {
        face_apply(st);
        refresh_text();
    } else if ((s_tick % 10) == 0) {
        refresh_text();                /* 识别结果/TTS 字幕/激活码可能变了，500ms 看一次 */
    }

    s_tick++;
    switch (s_face) {
    case XZ_IDLE:      anim_idle();   break;
    case XZ_LISTENING: anim_listen(); break;
    case XZ_THINKING:  anim_think();  break;
    case XZ_SPEAKING:  anim_speak();  break;
    default:                          break;   /* 出错/连接中：静态脸 */
    }
}

/* ------------------------------------------------------------------------- */
/* 构建：把内容画进调用方给的页面（ui_home 的最后一页）                       */
/* ------------------------------------------------------------------------- */

void assistant_ui_build(lv_obj_t *parent)
{
    if (parent == NULL) {
        ESP_LOGE(TAG, "小智页创建失败：parent 为空");
        return;
    }
    if (s_page == parent && face_ready()) {
        return;                     /* 同一页重复构建、脸也齐了：什么都不用做 */
    }
    if (s_page != NULL) {
        /* 旧页面居然还活着：先把我们的删除回调从旧页摘掉再换指针。
           否则旧页以后被删时 page_delete_cb 还会跑，把**新页**的指针和可见性
           一并清空（症状：翻到小智页却是空白/表情不动）。注释里说的"先解绑"
           以前只是写着，代码里并没有真的解绑。 */
        lv_obj_remove_event_cb(s_page, page_delete_cb);
        ESP_LOGW(TAG, "小智页：旧页面还没销毁就重建，已从旧页解绑（指针以新页为准）");
    }
    /* 关键：先把所有指针清空，再从零建。任何一个控件创建失败都不会留下
       上一次 build 的野指针（ui_home 每次收到 cfg 都会重建整棵控件树）。 */
    reset_face(NULL);
    s_page = parent;

    /* 父对象归我们布置：纯白底、正好 360x360、铺满、整页接触摸 */
    lv_obj_remove_style_all(parent);
    lv_obj_set_size(parent, 360, 360);
    lv_obj_set_pos(parent, 0, 0);
    lv_obj_set_style_bg_color(parent, C_BG, 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(parent, 0, 0);
    lv_obj_set_style_radius(parent, 0, 0);
    lv_obj_set_style_pad_all(parent, 0, 0);
    lv_obj_set_scrollable(parent, false);
    lv_obj_set_clickable(parent, true);          /* 整页接轻触/滑动 */
    /* 幂等：同一页重复 build（首建失败后再建、或以后 ui_home 原地重建）时，
       先把本模块自己的回调摘掉，避免一次轻触触发两遍 handle_tap()。 */
    lv_obj_remove_event_cb(parent, page_event_cb);
    lv_obj_remove_event_cb(parent, page_delete_cb);
    lv_obj_add_event_cb(parent, page_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(parent, page_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(parent, page_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(parent, page_delete_cb, LV_EVENT_DELETE, NULL);

    /* 录音指示点（聆听时呼吸）*/
    s_dot = make_block(parent, 12, 12, C_ACCENT, LV_RADIUS_CIRCLE);
    /* 两只眼睛 */
    s_eye[0] = make_block(parent, 36, 46, C_INK, 16);
    s_eye[1] = make_block(parent, 36, 46, C_INK, 16);
    if (s_dot == NULL || s_eye[0] == NULL || s_eye[1] == NULL) {
        goto fail;                  /* LVGL 池不够：别再往下建，免得留下半张脸 */
    }
    lv_obj_set_pos(s_dot, 174, 88);
    set_eyes(36, 46);

    /* 微笑：只画下半圈的一段弧 */
    s_mouth_smile = lv_arc_create(parent);
    if (s_mouth_smile == NULL) {
        goto fail;
    }
    lv_obj_remove_style_all(s_mouth_smile);
    lv_obj_set_size(s_mouth_smile, 120, 120);
    lv_obj_set_pos(s_mouth_smile, MOUTH_CX - 60, 115);      /* 圆心约 (180,175) */
    lv_obj_set_style_pad_all(s_mouth_smile, 0, 0);
    lv_obj_set_style_arc_width(s_mouth_smile, 7, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(s_mouth_smile, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_mouth_smile, 7, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_mouth_smile, C_INK, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(s_mouth_smile, true, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_mouth_smile, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_border_width(s_mouth_smile, 0, LV_PART_KNOB);
    lv_obj_set_clickable(s_mouth_smile, false);
    lv_arc_set_bg_angles(s_mouth_smile, 0, 360);
    lv_arc_set_angles(s_mouth_smile, 25, 155);              /* 下半圈 = 微笑 */

    /* 聆听时的小 "o" */
    s_mouth_o = lv_obj_create(parent);
    if (s_mouth_o == NULL) {
        goto fail;
    }
    lv_obj_remove_style_all(s_mouth_o);
    lv_obj_set_size(s_mouth_o, 26, 26);
    lv_obj_set_pos(s_mouth_o, MOUTH_CX - 13, MOUTH_CY - 13);
    lv_obj_set_style_radius(s_mouth_o, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_mouth_o, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_mouth_o, 6, 0);
    lv_obj_set_style_border_color(s_mouth_o, C_INK, 0);
    lv_obj_set_scrollable(s_mouth_o, false);
    lv_obj_set_clickable(s_mouth_o, false);

    /* 说话的嘴（药丸块，改高度就是张嘴/闭嘴）*/
    s_mouth_line = make_block(parent, 44, 12, C_INK, LV_RADIUS_CIRCLE);
    if (s_mouth_line == NULL) {
        goto fail;
    }
    lv_obj_set_pos(s_mouth_line, MOUTH_CX - 22, MOUTH_CY - 6);

    /* 思考：一圈转动的弧 */
    s_think_arc = lv_arc_create(parent);
    if (s_think_arc == NULL) {
        goto fail;
    }
    lv_obj_remove_style_all(s_think_arc);
    lv_obj_set_size(s_think_arc, 60, 60);
    lv_obj_set_pos(s_think_arc, MOUTH_CX - 30, MOUTH_CY - 30);
    lv_obj_set_style_pad_all(s_think_arc, 0, 0);
    lv_obj_set_style_arc_width(s_think_arc, 5, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_think_arc, C_TRACK, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(s_think_arc, true, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_think_arc, 5, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_think_arc, C_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(s_think_arc, true, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_think_arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_border_width(s_think_arc, 0, LV_PART_KNOB);
    lv_obj_set_clickable(s_think_arc, false);
    lv_arc_set_bg_angles(s_think_arc, 0, 360);
    lv_arc_set_angles(s_think_arc, 0, 90);

    /* 文字：状态 / 小智说的话（或最近一句识别结果）/ 激活码或操作提示 */
    s_status = make_label(parent, C_INK, 300, 0);
    s_sub = make_label(parent, C_MUTED, SUB_W, lv_font_get_line_height(&lv_font_cjk16) * SUB_LINES);
    s_hint = make_label(parent, C_MUTED, 240, 0);
    if (s_status == NULL || s_sub == NULL || s_hint == NULL) {
        goto fail;
    }
    lv_label_set_text(s_status, "待机");
    lv_obj_align(s_status, LV_ALIGN_TOP_MID, 0, 236);
    lv_obj_align(s_sub, LV_ALIGN_TOP_MID, 0, 256);
    lv_obj_align(s_hint, LV_ALIGN_TOP_MID, 0, 300);

    s_txt_status[0] = '\0';
    s_txt_sub[0] = '\0';
    s_txt_hint[0] = '\0';
    s_sub_src[0] = '\0';

    s_visible = false;              /* ui_home 建完会自动隐藏非当前页 */
    face_apply(XZ_IDLE);            /* 先把各部件的显隐摆好 */
    s_face = XZ_IDLE;

    if (s_anim_timer == NULL) {
        s_anim_timer = lv_timer_create(anim_timer_cb, ANIM_MS, NULL);
    }
    if (s_anim_timer != NULL) {
        lv_timer_pause(s_anim_timer);   /* 还没翻到这一页：先不跑动画 */
    }

    lv_mem_monitor_t mon;
    lv_mem_monitor(&mon);
    ESP_LOGI(TAG, "小智页已建好（最后一页）：LVGL 池空闲 %u/%u 字节（已用 %u%%），内部 RAM %u KB",
             (unsigned)mon.free_size, (unsigned)mon.total_size, (unsigned)mon.used_pct,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    return;

fail:
    /* 建到一半失败（LVGL 内存不足）：把指针全部清掉，本模块就当没启用。
       这一页还剩一块白底，不影响翻页和其它页面。 */
    ESP_LOGE(TAG, "小智页创建失败（LVGL 内存不足），表情/文字停用（其它页面不受影响）");
    reset_face(NULL);
}

/* ------------------------------------------------------------------------- */
/* 对外接口                                                                   */
/* ------------------------------------------------------------------------- */

void assistant_ui_open(void)
{
    if (!page_alive()) {
        ESP_LOGW(TAG, "小智页还没建好（布局未就绪或 voice.enabled=false），忽略打开命令");
        return;
    }
    ui_home_goto_assistant();       /* 翻页；翻到后 ui_home 会回调 tick 刷新表情/文字 */
    ESP_LOGI(TAG, "打开小智页（翻到最后一页，状态：%s）", face_name(xiaozhi_state()));
}

void assistant_ui_close(void)
{
    if (!page_alive()) {
        ESP_LOGW(TAG, "小智页还没建好，忽略关闭命令");
        return;
    }
    /* 还在聆听：顺手关麦，别把麦克风留在后台 */
    if (xiaozhi_state() == XZ_LISTENING) {
        xiaozhi_listen_stop();
        ESP_LOGI(TAG, "关闭小智页时还在聆听，已请求关麦");
    }
    ui_home_goto_first();
    ESP_LOGI(TAG, "关闭小智页（回到第一页）");
}

bool assistant_ui_is_open(void)
{
    return s_visible && page_alive();
}

void assistant_ui_set_visible(bool visible)
{
    if (!page_alive()) {
        /* 页面已被销毁（或指针已失效）：无条件当不可见，一个 LVGL 对象都不碰 */
        s_visible = false;
        s_need_sync = false;
        return;
    }
    if (visible == s_visible) {
        return;
    }
    s_visible = visible;

    if (visible) {
        /* 只起定时器 + 打个标记：真正的表情/文字刷新放到 50ms 定时器里做，
           这样本函数可以安全地在翻页事件回调里调用（不碰 LVGL 对象）。 */
        s_need_sync = true;
        if (s_anim_timer != NULL) {
            lv_timer_resume(s_anim_timer);
        }
        ESP_LOGI(TAG, "小智页可见：表情动画恢复");
    } else {
        s_need_sync = false;
        if (s_anim_timer != NULL) {
            lv_timer_pause(s_anim_timer);
        }
        /* 离开这一页时如果还在聆听，顺手关麦 */
        if (xiaozhi_state() == XZ_LISTENING) {
            xiaozhi_listen_stop();
            ESP_LOGI(TAG, "离开小智页时还在聆听，已请求关麦");
        }
    }
}

void assistant_ui_tick(void)
{
    esp_rom_printf("[as] raw: tick\n");

    /* ↑ 裸串口标记：必须是第一行——这条来自 ui_home 的 500ms 刷新定时器，
       和 handle_tap() 里的 [as] raw: tap 配对，一眼就能看出崩在哪条路径上。 */
    /* 页面被 ui_home 重建/销毁后照样会被调用：这里先确认页面还在活动屏幕上，
       不在就只清状态（page_alive 内部做），绝不 lv_obj_is_hidden() 野指针。 */
    if (!page_alive()) {
        return;
    }

    /* 兜底：除了翻页，详情/折线/空调这些子页面会盖住当前页。
       注意只认"当前页就是小智页"（翻页动画期间 s_page 已经是目标页），
       否则会在滑出的动画里又把它点开。 */
    const bool covered = ui_home_detail_open() || ui_home_chart_open() || ui_home_climate_open();
    assistant_ui_set_visible(ui_home_assistant_current() && !covered && !lv_obj_is_hidden(s_page));

    if (s_visible) {
        refresh();
    }
}
