/**
 * @file assistant_ui.c
 * @brief 小智助手界面：纯白底 + 会动的脸（两只眼睛 + 一张嘴）+ 中文状态文字。
 *
 * 为什么这么画（这块板子内存很紧，内部 RAM 只剩几十 KB）：
 *   - 不用任何图片资源：眼睛是带圆角的 lv_obj，嘴是 lv_arc / 小圆 / 药丸块，
 *     思考时是一圈转动的 lv_arc，全部靠"改尺寸、改角度"动起来；
 *   - 一共十几个对象、没有额外缓冲，动画在一个 50ms 的 LVGL 定时器里算，
 *     关页面时把定时器 pause 掉，不空转；
 *   - 页面第一次打开时才创建，之后只隐藏；ui_home 重建界面（收到新 cfg）会把
 *     整棵控件树 lv_obj_clean 掉，这里用 LV_EVENT_DELETE 把指针清空，下次再建。
 *
 * 表情（对应 xiaozhi_state()）：
 *   IDLE       圆眼睛 + 微笑弧，每 3~6 秒眨一次眼
 *   LISTENING  眼睛略放大、轻轻上下浮动，嘴变成小 "o"，头顶一个呼吸的录音点
 *   THINKING   眼睛压成两道短线，嘴的位置是一圈转动的弧
 *   SPEAKING   嘴（药丸块）按两个正弦叠加张合，眼睛跟着微微眯
 *   ERROR      眼睛压平、颜色转灰，副标题显示服务端给的原因
 *   CONNECTING 正常眼睛 + 一条短横线的嘴，等握手
 */
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lvgl.h"

#include "assistant_ui.h"
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

/* ---- 几何：360x360 圆屏，内容压在 y=50..300 的安全带里 ---- */
#define EYE_CY    150
#define EYE_CX_L  146
#define EYE_CX_R  214
#define MOUTH_CX  180
#define MOUTH_CY  210

#define ANIM_MS   50              /* 表情动画节奏：20 帧/秒 */

static lv_obj_t *s_page;
static lv_obj_t *s_close;
static lv_obj_t *s_dot;           /* 聆听时的录音指示点 */
static lv_obj_t *s_eye[2];
static lv_obj_t *s_mouth_smile;   /* 微笑弧（空闲）*/
static lv_obj_t *s_mouth_o;       /* 小 "o"（聆听）*/
static lv_obj_t *s_mouth_line;    /* 药丸块（说话时张合 / 出错时一条线）*/
static lv_obj_t *s_think_arc;     /* 思考时转圈的弧 */
static lv_obj_t *s_status;        /* 待机 / 正在聆听… */
static lv_obj_t *s_sub;           /* 最近一句识别结果 / 出错原因 */
static lv_obj_t *s_hint;          /* 激活码 / 操作提示 */

static lv_timer_t *s_anim_timer;
static bool s_open;
static xz_state_t s_face = (xz_state_t)-1;

static uint32_t s_tick;
static uint32_t s_seed = 0x2545F491u;
static int s_blink_wait;
static int s_blink_phase;
static bool s_blink_active;

/* 文字缓存：内容没变就不动 LVGL 标签（500ms 一次也别白重排）*/
static char s_txt_status[24];
static char s_txt_sub[128];
static char s_txt_hint[32];

static lv_point_t s_press;
static bool s_swiped;

/* ------------------------------------------------------------------------- */
/* 小工具                                                                     */
/* ------------------------------------------------------------------------- */

static void anim_translate_cb(void *var, int32_t v)
{
    lv_obj_set_style_translate_x((lv_obj_t *)var, (lv_coord_t)v, 0);
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
    lv_obj_t *o = lv_obj_create(parent);
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

/** 一行中文标签（居中、超长打点）。 */
static lv_obj_t *make_label(lv_obj_t *parent, lv_color_t color, lv_coord_t width)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, &lv_font_cjk16, 0);
    lv_obj_set_style_text_color(l, color, 0);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(l, width);
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
    if (s_eye[0] == NULL) {
        return;
    }
    for (int i = 0; i < 2; i++) {
        const lv_coord_t cx = (i == 0) ? EYE_CX_L : EYE_CX_R;
        lv_obj_set_size(s_eye[i], w, h);
        lv_obj_set_pos(s_eye[i], cx - w / 2, EYE_CY - h / 2);
    }
}

static void set_text_if(lv_obj_t *label, char *cache, size_t cap, const char *text)
{
    if (label == NULL) {
        return;
    }
    if (strcmp(cache, text) == 0) {
        return;
    }
    snprintf(cache, cap, "%s", text);
    lv_label_set_text(label, cache);
}

/* ------------------------------------------------------------------------- */
/* 状态 → 表情                                                                */
/* ------------------------------------------------------------------------- */

static void face_apply(xz_state_t st)
{
    if (s_page == NULL) {
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
    lv_arc_set_rotation(s_think_arc, (int32_t)((s_tick * 10) % 360));  /* 1.8 秒一圈 */
}

/* 说话：嘴按两个频率张合 */
static void anim_speak(void)
{
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
    const char *code = xiaozhi_activation_code();
    const char *last = xiaozhi_last_text();
    char sub[128] = "";
    char hint[32] = "";

    if (st == XZ_ERROR) {
        snprintf(sub, sizeof(sub), "%s", xiaozhi_client_status_text());
    } else if (last[0] != '\0') {
        snprintf(sub, sizeof(sub), "%s", last);     /* 最近一句识别结果 */
    }

    if (code[0] != '\0') {
        /* 激活码比什么都重要：单独一行，副标题让位 */
        snprintf(hint, sizeof(hint), "待激活 %s", code);
        sub[0] = '\0';
    } else if (st == XZ_LISTENING) {
        snprintf(hint, sizeof(hint), "再次轻触结束");
    } else if (st == XZ_IDLE) {
        snprintf(hint, sizeof(hint), "%s", ready ? "轻触说话 · 右滑返回" : "在 switchyard 打开语音");
    } else {
        snprintf(hint, sizeof(hint), "右滑返回");
    }

    set_text_if(s_status, s_txt_status, sizeof(s_txt_status), face_text(st, ready));
    set_text_if(s_sub, s_txt_sub, sizeof(s_txt_sub), sub);
    set_text_if(s_hint, s_txt_hint, sizeof(s_txt_hint), hint);
}

static void refresh(void)
{
    if (!s_open || s_page == NULL) {
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
    const xz_state_t st = xiaozhi_state();
    if (st == XZ_LISTENING) {
        xiaozhi_listen_stop();
        ESP_LOGI(TAG, "轻触：结束聆听");
        return;
    }
    if (st != XZ_IDLE) {
        return;                     /* 思考/说话/连接中：轻触不打断 */
    }
    if (!xiaozhi_client_ready()) {
        ESP_LOGW(TAG, "轻触：小智还没连上服务端（%s）", xiaozhi_client_status_text());
        return;
    }
    if (xiaozhi_listen_start()) {
        ESP_LOGI(TAG, "轻触：开始聆听");
    }
}

static void page_event_cb(lv_event_t *e)
{
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
        /* 右滑关闭（和主界面的滑动判定同一套思路：横向位移够大且比纵向大）*/
        if (dx > 60 && dx > ady) {
            s_swiped = true;
            ESP_LOGI(TAG, "右滑：关闭小智界面");
            assistant_ui_close();
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

static void close_clicked_cb(lv_event_t *e)
{
    (void)e;
    ESP_LOGI(TAG, "点关闭按钮");
    assistant_ui_close();
}

static void page_delete_cb(lv_event_t *e)
{
    (void)e;
    /* ui_home 重建界面时整棵控件树被删掉：指针必须清空，否则下次会踩野指针 */
    s_page = NULL;
    s_close = NULL;
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
    s_open = false;
    s_face = (xz_state_t)-1;
    if (s_anim_timer != NULL) {
        lv_timer_pause(s_anim_timer);   /* 定时器不在对象树里，留着下次复用 */
    }
    ESP_LOGW(TAG, "小智界面被界面重建移除（下次打开会重新创建）");
}

static void anim_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (!s_open || s_page == NULL) {
        return;
    }

    const xz_state_t st = xiaozhi_state();
    if (st != s_face) {
        face_apply(st);
        refresh_text();
    } else if ((s_tick % 10) == 0) {
        refresh_text();                /* 识别结果/激活码可能变了，500ms 看一次 */
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
/* 创建                                                                       */
/* ------------------------------------------------------------------------- */

static void create_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL) {
        return;
    }

    lv_obj_t *page = lv_obj_create(scr);
    lv_obj_remove_style_all(page);
    lv_obj_set_size(page, 360, 360);
    lv_obj_set_pos(page, 0, 0);
    lv_obj_set_style_bg_color(page, C_BG, 0);
    lv_obj_set_style_bg_opa(page, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(page, 0, 0);
    lv_obj_set_style_radius(page, 0, 0);
    lv_obj_set_style_pad_all(page, 0, 0);
    lv_obj_set_scrollable(page, false);
    lv_obj_set_clickable(page, true);          /* 整页接轻触/滑动 */
    lv_obj_add_event_cb(page, page_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(page, page_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(page, page_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(page, page_delete_cb, LV_EVENT_DELETE, NULL);
    lv_obj_set_hidden(page, true);
    s_page = page;

    /* 顶部关闭按钮：圆屏顶上 y<30 只有中间看得见，所以放顶部正中 */
    s_close = lv_button_create(page);
    lv_obj_remove_style_all(s_close);
    lv_obj_set_size(s_close, 34, 34);
    lv_obj_align(s_close, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_radius(s_close, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_close, C_CHIP, 0);
    lv_obj_set_style_bg_opa(s_close, LV_OPA_COVER, 0);
    lv_obj_add_event_cb(s_close, close_clicked_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *x = lv_label_create(s_close);
    lv_label_set_text(x, LV_SYMBOL_CLOSE);
    lv_obj_set_style_text_color(x, C_MUTED, 0);
    lv_obj_center(x);

    /* 录音指示点（聆听时呼吸）*/
    s_dot = make_block(page, 12, 12, C_ACCENT, LV_RADIUS_CIRCLE);
    lv_obj_set_pos(s_dot, 174, 88);

    /* 两只眼睛 */
    s_eye[0] = make_block(page, 36, 46, C_INK, 16);
    s_eye[1] = make_block(page, 36, 46, C_INK, 16);
    set_eyes(36, 46);

    /* 微笑：只画下半圈的一段弧 */
    s_mouth_smile = lv_arc_create(page);
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
    s_mouth_o = lv_obj_create(page);
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
    s_mouth_line = make_block(page, 44, 12, C_INK, LV_RADIUS_CIRCLE);
    lv_obj_set_pos(s_mouth_line, MOUTH_CX - 22, MOUTH_CY - 6);

    /* 思考：一圈转动的弧 */
    s_think_arc = lv_arc_create(page);
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

    /* 文字：状态 / 最近一句（或出错原因）/ 激活码或操作提示 */
    s_status = make_label(page, C_INK, 300);
    lv_label_set_text(s_status, "待机");
    lv_obj_align(s_status, LV_ALIGN_TOP_MID, 0, 236);

    s_sub = make_label(page, C_MUTED, 280);
    lv_obj_align(s_sub, LV_ALIGN_TOP_MID, 0, 260);

    s_hint = make_label(page, C_MUTED, 240);
    lv_obj_align(s_hint, LV_ALIGN_TOP_MID, 0, 284);

    s_txt_status[0] = '\0';
    s_txt_sub[0] = '\0';
    s_txt_hint[0] = '\0';

    face_apply(XZ_IDLE);            /* 先把各部件的显隐摆好 */
    s_face = XZ_IDLE;
}

/* ------------------------------------------------------------------------- */
/* 对外接口                                                                   */
/* ------------------------------------------------------------------------- */

void assistant_ui_open(void)
{
    if (s_page == NULL) {
        create_ui();
        if (s_page == NULL) {
            ESP_LOGE(TAG, "小智界面创建失败（内存不足）");
            return;
        }
        lv_mem_monitor_t mon;
        lv_mem_monitor(&mon);
        ESP_LOGI(TAG, "小智界面已创建：LVGL 池空闲 %u/%u 字节（已用 %u%%），内部 RAM %u KB",
                 (unsigned)mon.free_size, (unsigned)mon.total_size, (unsigned)mon.used_pct,
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    }

    if (s_open) {
        lv_obj_move_foreground(s_page);     /* 已经在：只置顶 */
        return;
    }

    s_open = true;
    lv_obj_set_hidden(s_page, false);
    lv_obj_move_foreground(s_page);         /* 压在主界面所有页/子页之上 */

    const xz_state_t st = xiaozhi_state();
    face_apply(st);
    refresh_text();

    lv_obj_set_style_translate_x(s_page, 360, 0);   /* 从右滑入，和主界面翻页一致 */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_page);
    lv_anim_set_values(&a, 360, 0);
    lv_anim_set_time(&a, 220);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, anim_translate_cb);
    lv_anim_start(&a);

    if (s_anim_timer == NULL) {
        s_anim_timer = lv_timer_create(anim_timer_cb, ANIM_MS, NULL);
    }
    if (s_anim_timer != NULL) {
        lv_timer_resume(s_anim_timer);
    }

    ESP_LOGI(TAG, "小智界面打开（状态：%s）", face_name(st));
}

static void close_done_cb(lv_anim_t *a)
{
    (void)a;
    if (s_page == NULL || s_open) {
        return;                 /* 动画期间又被打开了：别把新界面藏掉 */
    }
    lv_obj_set_hidden(s_page, true);
    lv_obj_set_style_translate_x(s_page, 0, 0);
}

void assistant_ui_close(void)
{
    if (!s_open || s_page == NULL) {
        return;
    }
    s_open = false;
    if (s_anim_timer != NULL) {
        lv_timer_pause(s_anim_timer);       /* 关着就不跑动画，省 CPU */
    }

    /* 还在聆听：顺手关麦，别把麦克风留在后台 */
    if (xiaozhi_state() == XZ_LISTENING) {
        xiaozhi_listen_stop();
        ESP_LOGI(TAG, "关闭界面时还在聆听，已请求关麦");
    }

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_page);
    lv_anim_set_values(&a, lv_obj_get_style_translate_x(s_page, 0), 360);
    lv_anim_set_time(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&a, anim_translate_cb);
    lv_anim_set_completed_cb(&a, close_done_cb);
    lv_anim_start(&a);

    ESP_LOGI(TAG, "小智界面关闭");
}

bool assistant_ui_is_open(void)
{
    return s_open;
}

void assistant_ui_tick(void)
{
    refresh();
}
