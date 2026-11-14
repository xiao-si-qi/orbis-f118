/**
 * 蓝牙链路抽象层 —— 经典蓝牙（BR/EDR）+ Bluedroid + A2DP source，真实实现。
 *
 * 设计约定：
 *   - 界面（ui_home.c）只通过 bt_link.h 的函数拿状态、下命令，不直接调用蓝牙 API；
 *   - 状态 s_state 是单字 volatile：蓝牙任务写、LVGL 线程读，不需要锁；
 *   - 音箱名 / 扫描结果 / 音频输出偏好由 s_lock 保护（写 NVS 时也不放锁外）；
 *   - 生命周期是「按需」的：
 *       bt_link_init()     只读 NVS + 建互斥量 + 起一个空闲的蓝牙任务（栈 4 KB），
 *                          不初始化控制器、不初始化 Bluedroid；
 *       第一次 scan/connect/enable  →  bt_stack_start()：
 *                          esp_bt_controller_init/enable(BR_EDR) → esp_bluedroid_init/enable
 *                          → GAP 回调 / 设备名 / 配对参数 → A2DP source init；
 *       空闲 60 秒（扫描结束没连上则 30 秒）或 bt_link_disable()
 *                          →  bt_stack_stop()：断连 → A2DP deinit → Bluedroid disable/deinit
 *                          → 控制器 disable/deinit，状态回 BT_OFF。
 *
 * 音频：本版只提供钩子（bt_link_audio_is_ready / bt_link_audio_write），还没有接进
 *       voice_audio.c 的播放路径；数据走「单声道 → 44.1 kHz 立体声」重采样 + 8 KB 环形缓冲，
 *       A2DP source 的取数回调（esp_a2d_source_register_data_callback，IDF 6.1 的 legacy
 *       API，内部用 SBC 编码）从环形缓冲里读，欠载补静音。
 *
 * 线程模型：
 *   - bt_worker 任务：唯一会调用协议栈「启停 / 扫描 / 连接 / 播放」API 的地方，
 *     界面只是往队列里投命令，绝不阻塞 LVGL；
 *   - GAP / A2DP 回调在 BTU/BTC 任务里跑，只写 volatile 标志、给信号量、打日志；
 *   - 音频数据回调在 A2DP source 任务里跑，只读环形缓冲，不加锁。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "bt_link.h"

/* 协议栈是否编译进来：只跟 sdkconfig 有关（BT_BLUEDROID + CLASSIC + A2DP）。
   关掉蓝牙配置时本文件仍然能编译（退化成「未启用（蓝牙栈未编译）」的空实现），
   方便临时砍掉蓝牙做对比。 */
#if defined(CONFIG_BT_A2DP_ENABLE) && CONFIG_BT_A2DP_ENABLE && \
    defined(CONFIG_BT_CLASSIC_ENABLED) && CONFIG_BT_CLASSIC_ENABLED && \
    defined(CONFIG_BT_BLUEDROID_ENABLED) && CONFIG_BT_BLUEDROID_ENABLED
#define BT_LINK_ENABLED 1
#else
#define BT_LINK_ENABLED 0
#endif

#if BT_LINK_ENABLED
#include "esp_a2dp_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#endif

static const char *TAG = "bt";

/* 本机蓝牙名：手机/音箱列表里看到的就是它 */
#define BT_LOCAL_NAME        "Orbis-F118"

/* NVS：音频输出偏好和「上次连接的音箱」放独立命名空间，不碰 app_settings 的 "app"。
   NVS 的 namespace / key 都限制 15 字符。 */
#define NVS_NAMESPACE        "orbis_bt"
#define KEY_AUDIO_OUT        "audio_out"     /* u8：0=耳机孔（默认） 1=蓝牙音箱 */
#define KEY_LAST_BDA         "last_bda"      /* blob 6 字节：上次连上的音箱 MAC */
#define KEY_LAST_NAME        "last_name"     /* str：上次连上的音箱名 */

#define BT_ADDR_LEN          6
#define BT_MAX_SCAN          8               /* 扫描结果最多存 8 条 */
#define BT_INQ_LEN           12              /* 询问长度，单位 1.28s → 约 15 秒 */
#define BT_CONNECT_TIMEOUT_MS 15000          /* 连接最多等 15 秒 */
#define BT_IDLE_RELEASE_MS   60000           /* 空闲 60 秒释放协议栈 */
#define BT_SCAN_RELEASE_MS   30000           /* 扫描结束没连上：30 秒后释放 */
#define BT_CMD_QUEUE_LEN     8

/* 音频数据通路：A2DP source 内部 SBC 编码器固定按 44.1 kHz 立体声取 PCM */
#define BT_PCM_RATE          44100
#define BT_PCM_FIFO_BYTES    8192            /* 必须是 2 的幂（下标用掩码） */
#define BT_PCM_STAGE_FRAMES  256             /* 重采样输出的小暂存块（帧） */

/* 蓝牙任务：只在有请求/事件时干活，1 秒醒一次检查「空闲释放」 */
#define BT_TASK_STACK        4096
#define BT_TASK_PRIO         4

typedef enum {
    BT_CMD_ENABLE = 0,      /* 只启动协议栈 */
    BT_CMD_SCAN,            /* 启动协议栈（如需要）+ 扫描 */
    BT_CMD_CONNECT,         /* 启动协议栈（如需要）+ 连接指定地址 */
    BT_CMD_RECONNECT_SAVED, /* 启动协议栈（如需要）+ 连接上次连过的音箱 */
    BT_CMD_DISCONNECT,      /* 断开当前连接 */
    BT_CMD_DISABLE,         /* 立即关停协议栈、释放内存 */
} bt_cmd_op_t;

typedef struct {
    uint8_t op;
    uint8_t bda[BT_ADDR_LEN];
} bt_cmd_t;

typedef struct {
    char name[32];
    char addr[18];          /* "aa:bb:cc:dd:ee:ff" */
    uint8_t bda[BT_ADDR_LEN];
    bool audio;             /* COD 里带 Rendering 位 ⇒ 大概率是音箱/耳机 */
} bt_scan_dev_t;

/* ------------------------------------------------------------------------- */
/* 模块状态                                                                  */
/* ------------------------------------------------------------------------- */

/* 当前链路状态：单字 volatile，蓝牙任务/回调写、LVGL 线程读 */
static volatile bt_state_t s_state = BT_OFF;

/* 这次开机有没有启动过协议栈：用来区分「未启用（待启用）」和「蓝牙已释放（省内存）」 */
static volatile bool s_ever_started;

/* 音频输出偏好：true=蓝牙音箱，false=耳机孔（默认）。写的时候持 s_lock（要写 NVS） */
static volatile bool s_audio_bt;

/* 已连接音箱名（只有 s_state == BT_CONNECTED 时才有意义），s_lock 保护写入 */
static char s_dev_name[32];

/* 扫描结果表 + 条数，s_lock 保护 */
static bt_scan_dev_t s_scan[BT_MAX_SCAN];
static int s_scan_count;

/* 上次连过的音箱（NVS 里恢复），s_lock 保护 */
static uint8_t s_saved_bda[BT_ADDR_LEN];
static char s_saved_name[32];
static bool s_have_saved;

/* 各种同步对象 */
static SemaphoreHandle_t s_lock;      /* 保护名字/扫描表/NVS 写入 */
static SemaphoreHandle_t s_ack_sem;   /* A2DP media ctrl 的 ACK */
static SemaphoreHandle_t s_conn_sem;  /* 连接状态变化 */
static SemaphoreHandle_t s_prof_sem;  /* A2DP init/deinit 完成 */
static QueueHandle_t s_queue;
static TaskHandle_t s_task;

static bool s_inited;

/* bt_link_state_text() 拼「已连接 xxx」用；只在 LVGL 线程调用，不跨任务共享 */
static char s_text_buf[64];

/* 协议栈各层「当前是否已打开」，只有 bt_worker 任务读写 */
static bool s_ctrl_inited;
static bool s_ctrl_enabled;
static bool s_host_inited;
static bool s_host_enabled;
static bool s_a2dp_inited;
static bool s_stack_up;

/* 空闲释放时间点（esp_timer 微秒，0 = 不释放）；只有 bt_worker 读写 */
static int64_t s_idle_deadline_us;

/* 回调写 / 任务读的 volatile 标志 */
static volatile bool s_scanning;       /* 询问进行中 */
static volatile bool s_scan_done;      /* 询问结束（等任务收尾） */
static volatile bool s_conn_ok;        /* A2DP 已连接 */
static volatile bool s_media_started;  /* A2DP 音频流已启动 */
static volatile bool s_skip_auto_connect; /* 用户主动断开后，不要自动重连 */

/* A2DP media ctrl 的最近一次 ACK（回调写、任务读） */
static volatile int s_ack_cmd;
static volatile int s_ack_status;

/* 连接句柄 / 对端地址（回调写） */
static volatile uint16_t s_conn_hdl;
static uint8_t s_peer_bda[BT_ADDR_LEN];

/* 「音频通路就绪」只打一次日志 */
static bool s_ready_logged;

/* 音频流启动失败的重试次数（每次连上新连接清零） */
static int s_stream_retries;

/* 音频环形缓冲：单生产者（音频线程）/ 单消费者（A2DP 任务），无锁。
   缓冲本身不放 .bss：协议栈启动时从 PSRAM 里要 9 KB，关栈时还回去，
   这样「蓝牙关着」时连这 9 KB 都不占（内部 RAM 更是一点不占）。 */
static uint8_t *s_fifo;                /* BT_PCM_FIFO_BYTES 字节，2 的幂 */
static volatile uint32_t s_fifo_wr;
static volatile uint32_t s_fifo_rd;

/* 重采样状态 + 输出暂存块（同样在 PSRAM） */
static int16_t *s_stage;
static int s_stage_n;
static uint32_t s_rs_step;     /* 16.16：每输出一帧前进多少输入帧 */
static uint32_t s_rs_phase;    /* 16.16：s_rs_last 与当前输入样本之间的位置 */
static int32_t s_rs_last;
static int s_rs_rate;
static bool s_rs_started;
static uint32_t s_drop_bytes;
static bool s_drop_logged;
static uint32_t s_underrun_cnt;
static bool s_underrun_logged;

/* ------------------------------------------------------------------------- */
/* 小工具                                                                    */
/* ------------------------------------------------------------------------- */

static void bt_lock(void)
{
    if (s_lock != NULL) {
        (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

static void bt_unlock(void)
{
    if (s_lock != NULL) {
        (void)xSemaphoreGive(s_lock);
    }
}

/** 打一条「蓝牙内存」日志：内部 RAM / PSRAM 剩余 + 总堆剩余（KB），用来验收按需启停的效果。 */
static void bt_log_heap(const char *when)
{
    ESP_LOGI(TAG, "蓝牙内存[%s]：内部 RAM 剩余 %u KB，PSRAM 剩余 %u KB，总堆剩余 %u KB",
             when,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(esp_get_free_heap_size() / 1024));
}

#if BT_LINK_ENABLED
/** MAC → "aa:bb:cc:dd:ee:ff"（小写）。 */
static void bt_bda2str(const uint8_t *bda, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return;
    }
    snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

/** "aa:bb:cc:dd:ee:ff" → 6 字节；格式不对返回 false。 */
static bool bt_str2bda(const char *str, uint8_t *out)
{
    unsigned int v[BT_ADDR_LEN];
    if (str == NULL || out == NULL) {
        return false;
    }
    if (sscanf(str, "%2x:%2x:%2x:%2x:%2x:%2x",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != BT_ADDR_LEN) {
        return false;
    }
    for (int i = 0; i < BT_ADDR_LEN; i++) {
        if (v[i] > 0xffu) {
            return false;
        }
        out[i] = (uint8_t)v[i];
    }
    return true;
}
#endif /* BT_LINK_ENABLED */

/** 拷贝已连接音箱名（没连接时是空串）。 */
static void bt_copy_dev_name(char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return;
    }
    bt_lock();
    snprintf(out, out_len, "%s", s_dev_name);
    bt_unlock();
}

/** 按 MAC 从扫描表里找名字，找到就更新 s_dev_name。 */
static void bt_set_dev_name_from_scan(const uint8_t *bda, const char *fallback)
{
    char name[32] = "";
    bt_lock();
    for (int i = 0; i < s_scan_count; i++) {
        if (memcmp(s_scan[i].bda, bda, BT_ADDR_LEN) == 0 && s_scan[i].name[0] != '\0') {
            snprintf(name, sizeof(name), "%s", s_scan[i].name);
            break;
        }
    }
    if (name[0] == '\0' && fallback != NULL) {
        snprintf(name, sizeof(name), "%s", fallback);
    }
    snprintf(s_dev_name, sizeof(s_dev_name), "%s", name);
    bt_unlock();
}

/** 如果 bda 就是 NVS 里记着的那台音箱，返回它的名字（否则空串）。 */
static void bt_saved_name_for(const uint8_t *bda, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return;
    }
    out[0] = '\0';
    bt_lock();
    if (s_have_saved && memcmp(s_saved_bda, bda, BT_ADDR_LEN) == 0) {
        snprintf(out, out_len, "%s", s_saved_name);
    }
    bt_unlock();
}

/** 清空已连接音箱名（断开时用）。 */
static void bt_clear_dev_name(void)
{
    bt_lock();
    s_dev_name[0] = '\0';
    bt_unlock();
}

/* ------------------------------------------------------------------------- */
/* 音频环形缓冲 + 重采样（音频线程写、A2DP 任务读）                          */
/* ------------------------------------------------------------------------- */

/**
 * 申请音频缓冲（PSRAM，9 KB）：协议栈启动时调用。
 * 申请不到就打警告、按「只能发静音」继续跑 —— 不能因为 PSRAM 紧张把蓝牙搞挂。
 */
static void bt_pcm_buffers_alloc(void)
{
    if (s_fifo != NULL) {
        return;
    }
    s_fifo = (uint8_t *)heap_caps_malloc(BT_PCM_FIFO_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_stage = (int16_t *)heap_caps_malloc(BT_PCM_STAGE_FRAMES * 2 * sizeof(int16_t),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_fifo == NULL || s_stage == NULL) {
        ESP_LOGW(TAG, "蓝牙：音频缓冲（PSRAM %u 字节）分配失败，A2DP 只会发静音",
                 (unsigned)(BT_PCM_FIFO_BYTES + BT_PCM_STAGE_FRAMES * 2 * sizeof(int16_t)));
        heap_caps_free(s_fifo);
        heap_caps_free(s_stage);
        s_fifo = NULL;
        s_stage = NULL;
        return;
    }
    s_fifo_wr = 0;
    s_fifo_rd = 0;
    s_stage_n = 0;
}

/** 归还音频缓冲（关栈时调用）。 */
static void bt_pcm_buffers_free(void)
{
    heap_caps_free(s_fifo);
    heap_caps_free(s_stage);
    s_fifo = NULL;
    s_stage = NULL;
    s_fifo_wr = 0;
    s_fifo_rd = 0;
    s_stage_n = 0;
}

static size_t bt_fifo_write(const uint8_t *src, size_t n)
{
    if (s_fifo == NULL) {
        return 0;
    }
    const uint32_t wr = s_fifo_wr;
    const uint32_t rd = s_fifo_rd;
    const size_t used = (size_t)(wr - rd);
    size_t space = (used >= BT_PCM_FIFO_BYTES) ? 0 : (BT_PCM_FIFO_BYTES - used);

    if (n > space) {
        n = space;
    }
    for (size_t i = 0; i < n; i++) {
        s_fifo[(wr + i) & (BT_PCM_FIFO_BYTES - 1)] = src[i];
    }
    s_fifo_wr = wr + (uint32_t)n;
    return n;
}

static size_t bt_fifo_read(uint8_t *dst, size_t n)
{
    if (s_fifo == NULL) {
        return 0;
    }
    const uint32_t rd = s_fifo_rd;
    const uint32_t wr = s_fifo_wr;
    size_t avail = (size_t)(wr - rd);

    if (n > avail) {
        n = avail;
    }
    for (size_t i = 0; i < n; i++) {
        dst[i] = s_fifo[(rd + i) & (BT_PCM_FIFO_BYTES - 1)];
    }
    s_fifo_rd = rd + (uint32_t)n;
    return n;
}

static void bt_stage_flush(void)
{
    if (s_stage_n == 0 || s_stage == NULL) {
        s_stage_n = 0;
        return;
    }
    const size_t bytes = (size_t)s_stage_n * 2 * sizeof(int16_t);
    const size_t wr = bt_fifo_write((const uint8_t *)s_stage, bytes);
    if (wr < bytes) {
        s_drop_bytes += (uint32_t)(bytes - wr);
        if (!s_drop_logged) {
            s_drop_logged = true;
            ESP_LOGW(TAG, "蓝牙音频缓冲满，丢掉部分 PCM（累计 %u 字节）", (unsigned)s_drop_bytes);
        }
    }
    s_stage_n = 0;
}

static void bt_stage_push(int32_t v)
{
    if (s_stage == NULL) {
        return;
    }
    if (v > 32767) {
        v = 32767;
    } else if (v < -32768) {
        v = -32768;
    }
    s_stage[s_stage_n * 2] = (int16_t)v;      /* 左 */
    s_stage[s_stage_n * 2 + 1] = (int16_t)v;  /* 右（单声道复制成双声道） */
    if (++s_stage_n >= BT_PCM_STAGE_FRAMES) {
        bt_stage_flush();
    }
}

/* ------------------------------------------------------------------------- */
/* 协议栈回调（BTU/BTC 任务上下文：只写标志、给信号量、打日志）              */
/* ------------------------------------------------------------------------- */

#if BT_LINK_ENABLED
/** 扫描结果 → 存进 s_scan[]（名字 + MAC + 是否音频设备）。 */
static void bt_gap_store_disc_result(const esp_bt_gap_cb_param_t *param)
{
    char addr[18] = "";
    char name[32] = "";
    uint32_t cod = 0;
    bool has_cod = false;
    const uint8_t *bda = param->disc_res.bda;

    for (int i = 0; i < param->disc_res.num_prop; i++) {
        const esp_bt_gap_dev_prop_t *p = &param->disc_res.prop[i];
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(const uint32_t *)p->val;
            has_cod = true;
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME: {
            size_t n = (p->len > 0) ? (size_t)p->len : 0;
            if (n > sizeof(name) - 1) {
                n = sizeof(name) - 1;
            }
            if (n > 0 && p->val != NULL) {
                memcpy(name, p->val, n);
                name[n] = '\0';
            }
            break;
        }
        case ESP_BT_GAP_DEV_PROP_EIR: {
            uint8_t len = 0;
            const uint8_t *nm = esp_bt_gap_resolve_eir_data((uint8_t *)p->val,
                                                           ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
            if (nm == NULL) {
                nm = esp_bt_gap_resolve_eir_data((uint8_t *)p->val,
                                                 ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
            }
            if (nm != NULL) {
                size_t n = (len > sizeof(name) - 1) ? (sizeof(name) - 1) : len;
                memcpy(name, nm, n);
                name[n] = '\0';
            }
            break;
        }
        default:
            break;
        }
    }

    /* COD 的 Rendering 位：音箱 / 耳机 / 车机这类会出声的设备 */
    const bool audio = has_cod && esp_bt_gap_is_valid_cod(cod) &&
                       ((esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_RENDERING) != 0);

    bt_bda2str(bda, addr, sizeof(addr));

    bt_lock();
    int slot = -1;
    for (int i = 0; i < s_scan_count; i++) {
        if (memcmp(s_scan[i].bda, bda, BT_ADDR_LEN) == 0) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        if (s_scan_count < BT_MAX_SCAN) {
            slot = s_scan_count++;
        }
    }
    if (slot >= 0) {
        if (name[0] != '\0') {
            snprintf(s_scan[slot].name, sizeof(s_scan[slot].name), "%s", name);
        }
        snprintf(s_scan[slot].addr, sizeof(s_scan[slot].addr), "%s", addr);
        memcpy(s_scan[slot].bda, bda, BT_ADDR_LEN);
        s_scan[slot].audio = audio;
    }
    bt_unlock();

    ESP_LOGI(TAG, "扫描到：%s  %s%s", name[0] != '\0' ? name : "未知设备", addr,
             audio ? "（音频设备）" : "");
}

/** GAP 回调：扫描结果 / 配对请求。 */
static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
        bt_gap_store_disc_result(param);
        break;

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            s_scanning = true;
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            if (s_scanning) {
                s_scanning = false;
                s_scan_done = true;     /* 收尾交给 bt_worker */
            }
        }
        break;

    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        char addr[18] = "";
        bt_bda2str(param->auth_cmpl.bda, addr, sizeof(addr));
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "蓝牙：配对成功 %s（设备名 %s）", addr,
                     (const char *)param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "蓝牙：配对失败 %s（状态 %d）", addr, (int)param->auth_cmpl.stat);
        }
        break;
    }

    case ESP_BT_GAP_PIN_REQ_EVT:
        if (param->pin_req.min_16_digit) {
            esp_bt_pin_code_t pin = {0};
            ESP_LOGI(TAG, "蓝牙：对方要求 16 位 PIN，回 0000000000000000");
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin);
        } else {
            esp_bt_pin_code_t pin = {'1', '2', '3', '4'};
            ESP_LOGI(TAG, "蓝牙：对方要求 PIN，回 1234");
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        }
        break;

    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "蓝牙：简化配对确认码 %06u，自动确认", (unsigned)param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "蓝牙：配对密码 %06u", (unsigned)param->key_notif.passkey);
        break;

    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGW(TAG, "蓝牙：对方要求输入配对密码（音箱一般不会走到这里）");
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGD(TAG, "蓝牙：链路模式变化 %d", (int)param->mode_chg.mode);
        break;

    default:
        ESP_LOGD(TAG, "蓝牙：GAP 事件 %d", (int)event);
        break;
    }
}

/** 启动 A2DP 音频流：CHECK_SRC_RDY → START（照 IDF a2dp_source 例子的顺序）。 */
static bool bt_a2dp_stream_start(void)
{
    if (!s_a2dp_inited || !s_conn_ok) {
        return false;
    }

    (void)xSemaphoreTake(s_ack_sem, 0);
    s_ack_cmd = -1;
    s_ack_status = -1;

    if (esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙：A2DP 通道检查命令发送失败");
        return false;
    }
    if (xSemaphoreTake(s_ack_sem, pdMS_TO_TICKS(1500)) != pdTRUE) {
        ESP_LOGE(TAG, "蓝牙：A2DP 通道检查超时");
        return false;
    }
    if (s_ack_cmd != (int)ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY ||
            s_ack_status != (int)ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
        ESP_LOGE(TAG, "蓝牙：A2DP 通道没准备好（acks %d/%d）", s_ack_cmd, s_ack_status);
        return false;
    }

    (void)xSemaphoreTake(s_ack_sem, 0);
    if (esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START) != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙：A2DP 启动音频流命令发送失败");
        return false;
    }
    if (xSemaphoreTake(s_ack_sem, pdMS_TO_TICKS(1500)) != pdTRUE) {
        ESP_LOGE(TAG, "蓝牙：A2DP 启动音频流超时");
        return false;
    }
    if (s_ack_cmd != (int)ESP_A2D_MEDIA_CTRL_START ||
            s_ack_status != (int)ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
        ESP_LOGE(TAG, "蓝牙：A2DP 音频流启动失败（acks %d/%d）", s_ack_cmd, s_ack_status);
        return false;
    }

    s_media_started = true;
    ESP_LOGI(TAG, "蓝牙：A2DP 音频通道已打开（SBC，%d Hz 立体声）", BT_PCM_RATE);
    return true;
}

/** A2DP 回调：连接状态 / 音频流状态 / media ctrl ACK / 编解码能力。 */
static void bt_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_PROF_STATE_EVT:
        if (param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS) {
            s_a2dp_inited = true;
            ESP_LOGI(TAG, "蓝牙：A2DP source 初始化完成");
        } else {
            s_a2dp_inited = false;
            ESP_LOGI(TAG, "蓝牙：A2DP source 已反初始化");
        }
        if (s_prof_sem != NULL) {
            (void)xSemaphoreGive(s_prof_sem);
        }
        break;

    case ESP_A2D_CONNECTION_STATE_EVT: {
        const esp_a2d_connection_state_t st = param->conn_stat.state;
        if (st == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            char addr[18] = "";
            char saved[32] = "";
            bt_bda2str(param->conn_stat.remote_bda, addr, sizeof(addr));
            memcpy(s_peer_bda, param->conn_stat.remote_bda, BT_ADDR_LEN);
            s_conn_hdl = param->conn_stat.conn_hdl;
            s_conn_ok = true;
            s_media_started = false;
            s_ready_logged = false;
            s_stream_retries = 0;
            bt_saved_name_for(param->conn_stat.remote_bda, saved, sizeof(saved));
            bt_set_dev_name_from_scan(param->conn_stat.remote_bda, saved[0] != '\0' ? saved : NULL);
            s_state = BT_CONNECTED;
            ESP_LOGI(TAG, "蓝牙：已连接 %s（%s，句柄 %u，音频 MTU %u）",
                     bt_link_device_name(), addr, (unsigned)s_conn_hdl,
                     (unsigned)param->conn_stat.audio_mtu);
            if (s_conn_sem != NULL) {
                (void)xSemaphoreGive(s_conn_sem);
            }
        } else if (st == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            s_conn_ok = false;
            s_media_started = false;
            s_ready_logged = false;
            bt_clear_dev_name();
            if (s_state != BT_OFF && s_state != BT_INIT) {
                s_state = BT_IDLE;
            }
            ESP_LOGW(TAG, "蓝牙：连接已断开（原因 0x%02x）", (unsigned)param->conn_stat.disc_rsn);
            if (s_conn_sem != NULL) {
                (void)xSemaphoreGive(s_conn_sem);
            }
        } else if (st == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {
            ESP_LOGI(TAG, "蓝牙：正在断开…");
        } else {
            ESP_LOGI(TAG, "蓝牙：正在连接…");
        }
        break;
    }

    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "蓝牙：音频流状态 %d", (int)param->audio_stat.state);
        break;

    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        s_ack_cmd = (int)param->media_ctrl_stat.cmd;
        s_ack_status = (int)param->media_ctrl_stat.status;
        if (s_ack_sem != NULL) {
            (void)xSemaphoreGive(s_ack_sem);
        }
        break;

    case ESP_A2D_AUDIO_CFG_EVT:
        if (param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            ESP_LOGI(TAG, "蓝牙：编解码配置 SBC（采样率掩码 0x%x，声道掩码 0x%x）",
                     (unsigned)param->audio_cfg.mcc.cie.sbc_info.samp_freq,
                     (unsigned)param->audio_cfg.mcc.cie.sbc_info.ch_mode);
        }
        break;

    case ESP_A2D_REPORT_SNK_CODEC_CAPS_EVT:
        if (param->a2d_report_snk_codec_caps_stat.mcc.type == ESP_A2D_MCT_SBC) {
            ESP_LOGI(TAG, "蓝牙：音箱支持 SBC（采样率掩码 0x%x，声道掩码 0x%x）",
                     (unsigned)param->a2d_report_snk_codec_caps_stat.mcc.cie.sbc_info.samp_freq,
                     (unsigned)param->a2d_report_snk_codec_caps_stat.mcc.cie.sbc_info.ch_mode);
        }
        break;

    case ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT:
        ESP_LOGI(TAG, "蓝牙：音箱上报延时 %u（0.1 ms）",
                 (unsigned)param->a2d_report_delay_value_stat.delay_value);
        break;

    default:
        ESP_LOGD(TAG, "蓝牙：A2DP 事件 %d", (int)event);
        break;
    }
}

/** A2DP source 取数回调：A2DP 任务每次要 len 字节 PCM（44.1 kHz 立体声 16 bit）。 */
static int32_t bt_a2d_data_cb(uint8_t *buf, int32_t len)
{
    if (buf == NULL) {
        return 0;
    }
    if (len <= 0) {
        /* -1：协议栈要求清空缓存（例如刚启动音频流） */
        s_fifo_rd = s_fifo_wr;
        return 0;
    }

    const size_t got = bt_fifo_read(buf, (size_t)len);
    if (got < (size_t)len) {
        /* 欠载：补静音，别让音箱断流（下一轮把 voice_audio 接上后基本不会欠载） */
        memset(buf + got, 0, (size_t)len - got);
        s_underrun_cnt++;
        if (!s_underrun_logged) {
            s_underrun_logged = true;
            ESP_LOGW(TAG, "蓝牙：A2DP 要数据时缓冲空，先补静音（没有 TTS 时正常）");
        }
    }
    return len;
}
#endif /* BT_LINK_ENABLED */

/* ------------------------------------------------------------------------- */
/* 扫描结果（给界面用）                                                      */
/* ------------------------------------------------------------------------- */

static int bt_scan_audio_count(void)
{
    int n = 0;
    bt_lock();
    for (int i = 0; i < s_scan_count; i++) {
        if (s_scan[i].audio) {
            n++;
        }
    }
    bt_unlock();
    return n;
}

/** 把扫描表里第 index 条复制出来（名字 / MAC）；返回是否成功。 */
static bool bt_scan_copy(int index, char *name, size_t name_len, char *addr, size_t addr_len,
                         uint8_t *bda_out, bool *audio_out)
{
    bool ok = false;
    bt_lock();
    if (index >= 0 && index < s_scan_count) {
        if (name != NULL && name_len > 0) {
            snprintf(name, name_len, "%s", s_scan[index].name);
        }
        if (addr != NULL && addr_len > 0) {
            snprintf(addr, addr_len, "%s", s_scan[index].addr);
        }
        if (bda_out != NULL) {
            memcpy(bda_out, s_scan[index].bda, BT_ADDR_LEN);
        }
        if (audio_out != NULL) {
            *audio_out = s_scan[index].audio;
        }
        ok = true;
    }
    bt_unlock();
    return ok;
}

/* ------------------------------------------------------------------------- */
/* 协议栈启停（只在 bt_worker 任务里调用）                                    */
/* ------------------------------------------------------------------------- */

#if BT_LINK_ENABLED
/* 前向声明：启动失败时用 bt_stack_stop() 回滚已经打开的那几层 */
static void bt_stack_stop(const char *reason);

/** 把「本次运行」的连接/播放状态清干净（不动协议栈本身）。 */
static void bt_clear_session_state(void)
{
    s_conn_ok = false;
    s_media_started = false;
    s_ready_logged = false;
    s_scanning = false;
    s_scan_done = false;
    s_a2dp_inited = false;
    s_fifo_rd = s_fifo_wr;
    s_stage_n = 0;
    s_rs_started = false;
    s_rs_rate = 0;
}

/**
 * 启动协议栈（幂等）。顺序照 IDF examples/bluetooth/bluedroid/classic_bt/a2dp_source：
 *   控制器 init/enable(BR_EDR) → Bluedroid init/enable → GAP 设置 → A2DP source init。
 * 内存说明：esp32s31 的 esp_bt_controller_mem_release() 是空实现（IDF 源码里直接写
 * "not implement yet!"），所以这里不调用它。链接期就占掉的 IRAM 代码 + Bluedroid 的
 * 静态 .bss（约 52 KB）无论如何都还不回来；能还回来的是 init 时从堆里申请的
 * 控制器内存池 / 控制器与 host 任务栈 / Bluedroid 的 osi 缓冲（见 bt_stack_stop 日志）。
 */
static esp_err_t bt_stack_start(void)
{
    if (s_stack_up) {
        return ESP_OK;
    }

    s_state = BT_INIT;
    s_ever_started = true;
    s_skip_auto_connect = false;
    bt_clear_session_state();

    bt_log_heap("启动前");
    ESP_LOGI(TAG, "蓝牙：正在启动协议栈（经典蓝牙 BR/EDR + A2DP source）…");

    /* A2DP 的 PCM 缓冲（9 KB）也从 PSRAM 借，关栈时还 */
    bt_pcm_buffers_alloc();

    esp_err_t err;

    if (!s_ctrl_inited) {
        esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        err = esp_bt_controller_init(&cfg);
        if (err == ESP_ERR_INVALID_STATE) {
            /* 上一次 deinit 没完全落地：控制器还活着，直接复用 */
            ESP_LOGW(TAG, "蓝牙：控制器已是初始化状态，复用（%s）", esp_err_to_name(err));
            err = ESP_OK;
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "蓝牙：控制器初始化失败（%s）", esp_err_to_name(err));
            bt_stack_stop("启动失败");
            s_state = BT_ERROR;
            return err;
        }
        s_ctrl_inited = true;
    }

    if (!s_ctrl_enabled) {
        err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "蓝牙：控制器使能失败（%s）", esp_err_to_name(err));
            bt_stack_stop("启动失败");
            s_state = BT_ERROR;
            return err;
        }
        s_ctrl_enabled = true;
    }

    if (!s_host_inited) {
        esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
        err = esp_bluedroid_init_with_cfg(&bd_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "蓝牙：Bluedroid 初始化失败（%s）", esp_err_to_name(err));
            bt_stack_stop("启动失败");
            s_state = BT_ERROR;
            return err;
        }
        s_host_inited = true;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙：Bluedroid 使能失败（%s）", esp_err_to_name(err));
        bt_stack_stop("启动失败");
        s_state = BT_ERROR;
        return err;
    }
    s_host_enabled = true;

    /* ---- GAP：设备名 / 可发现可连接 / 配对参数 ----
       可发现 + 可连接：手机能搜到它，音箱也能主动回连；扫描时还会打开询问。 */
    esp_bt_gap_register_callback(bt_gap_cb);
    err = esp_bt_gap_set_device_name(BT_LOCAL_NAME);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "蓝牙：设置设备名失败（%s）", esp_err_to_name(err));
    }
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code = {0};
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    /* 音箱基本都是「Just Works」，无输入无输出最省事 */
    esp_bt_sp_param_t sp_param = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(sp_param, &iocap, sizeof(iocap));

    /* ---- A2DP source ----
       先注册回调，再 init；init 完成会回 ESP_A2D_PROF_STATE_EVT（ESP_A2D_INIT_SUCCESS）。
       legacy 的 data callback 是 IDF 6.1 里 source 推 PCM 的官方路径（内部 SBC 编码）。 */
    (void)xSemaphoreTake(s_prof_sem, 0);
    esp_a2d_register_callback(bt_a2d_cb);
    esp_a2d_source_register_data_callback(bt_a2d_data_cb);
    err = esp_a2d_source_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙：A2DP source 初始化请求失败（%s）", esp_err_to_name(err));
        bt_stack_stop("启动失败");
        s_state = BT_ERROR;
        return err;
    }
    if (xSemaphoreTake(s_prof_sem, pdMS_TO_TICKS(3000)) != pdTRUE) {
        ESP_LOGW(TAG, "蓝牙：等 A2DP source 初始化确认超时，先继续（连接时再确认）");
    }

    s_stack_up = true;
    s_state = BT_IDLE;

    {
        char own[18] = "";
        const uint8_t *own_bda = esp_bt_dev_get_address();
        if (own_bda != NULL) {
            bt_bda2str(own_bda, own, sizeof(own));
        }
        ESP_LOGI(TAG, "蓝牙已启动：本机名 %s，地址 %s（空闲 %u 秒后自动释放）",
                 BT_LOCAL_NAME, own[0] != '\0' ? own : "?",
                 (unsigned)(BT_IDLE_RELEASE_MS / 1000));
    }
    bt_log_heap("启动后");

    s_idle_deadline_us = esp_timer_get_time() + (int64_t)BT_IDLE_RELEASE_MS * 1000;
    return ESP_OK;
}

/**
 * 关停协议栈（幂等，可以从任意状态调用）：
 *   断开 → A2DP source deinit → Bluedroid disable/deinit → 控制器 disable/deinit。
 * 结束后状态回 BT_OFF（界面显示「蓝牙已释放（省内存）」）。
 */
static void bt_stack_stop(const char *reason)
{
    if (!s_stack_up && !s_ctrl_inited && !s_host_inited) {
        bt_clear_session_state();
        s_state = BT_OFF;
        return;
    }

    ESP_LOGI(TAG, "蓝牙：正在释放协议栈（%s）…", reason);
    s_idle_deadline_us = 0;

    if (s_scanning) {
        esp_bt_gap_cancel_discovery();
        s_scanning = false;
        s_scan_done = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (s_conn_ok) {
        (void)xSemaphoreTake(s_conn_sem, 0);
        esp_a2d_source_disconnect(s_peer_bda);
        (void)xSemaphoreTake(s_conn_sem, pdMS_TO_TICKS(2000));
        s_conn_ok = false;
    }

    if (s_a2dp_inited && s_host_enabled) {
        (void)xSemaphoreTake(s_prof_sem, 0);
        esp_a2d_source_deinit();
        if (xSemaphoreTake(s_prof_sem, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "蓝牙：A2DP 反初始化没回事件，继续关");
        }
        s_a2dp_inited = false;
    }

    if (s_host_enabled) {
        esp_bluedroid_disable();
        s_host_enabled = false;
    }
    if (s_host_inited) {
        esp_bluedroid_deinit();
        s_host_inited = false;
    }

    if (s_ctrl_enabled) {
        esp_bt_controller_disable();
        s_ctrl_enabled = false;
    }
    if (s_ctrl_inited) {
        esp_err_t err = esp_bt_controller_deinit();
        if (err == ESP_OK) {
            s_ctrl_inited = false;
        } else {
            ESP_LOGW(TAG, "蓝牙：控制器 deinit 返回 %s（下次启动会复用）", esp_err_to_name(err));
        }
    }

    bt_clear_session_state();
    bt_pcm_buffers_free();
    s_stack_up = false;
    s_state = BT_OFF;

    bt_log_heap("释放后");
    ESP_LOGI(TAG, "蓝牙已关闭（%s）：控制器/host 的堆内存已还给系统，"
                  "链接期固定的 IRAM + 静态 .bss（约 52 KB）还不了，这部分是开蓝牙的固定代价",
             reason);
}
#endif /* BT_LINK_ENABLED */

/* ------------------------------------------------------------------------- */
/* 蓝牙任务：扫描 / 连接 / 断开 / 启停 / 空闲释放                            */
/* ------------------------------------------------------------------------- */

#if BT_LINK_ENABLED
/* 前向声明：下面几个流程互相调用（重连 → 连接 → 启动音频流） */
static void bt_do_connect(const uint8_t *bda);

/** 连接上次成功连过的音箱（NVS 里有地址才做）。 */
static void bt_do_reconnect_saved(void)
{
    if (!s_have_saved) {
        ESP_LOGI(TAG, "蓝牙：还没有连过的音箱记录，先扫描");
        return;
    }
    ESP_LOGI(TAG, "蓝牙：按上次记录重连 %s（%02x:%02x:%02x:%02x:%02x:%02x）",
             s_saved_name[0] != '\0' ? s_saved_name : "音箱",
             s_saved_bda[0], s_saved_bda[1], s_saved_bda[2],
             s_saved_bda[3], s_saved_bda[4], s_saved_bda[5]);
    bt_do_connect(s_saved_bda);
}

/** 扫描结束收尾：打日志、列结果、必要时自动连接、安排空闲释放。 */
static void bt_scan_finished(void)
{
    int total = 0;
    int audio = 0;
    bt_lock();
    total = s_scan_count;
    bt_unlock();
    audio = bt_scan_audio_count();

    ESP_LOGI(TAG, "蓝牙：扫描结束，共 %d 个设备，其中音箱/耳机 %d 个", total, audio);
    for (int i = 0; i < total; i++) {
        char name[32] = "";
        char addr[18] = "";
        bool is_audio = false;
        if (bt_scan_copy(i, name, sizeof(name), addr, sizeof(addr), NULL, &is_audio)) {
            ESP_LOGI(TAG, "  [%d] %s  %s%s", i, name[0] != '\0' ? name : "未知设备", addr,
                     is_audio ? "（音频设备）" : "");
        }
    }

    if (total == 0) {
        ESP_LOGW(TAG, "蓝牙：一个设备都没扫到（音箱没开 / 不在旁边 / 已被手机连着？）");
    }

    s_idle_deadline_us = esp_timer_get_time() + (int64_t)BT_SCAN_RELEASE_MS * 1000;

    /* 界面（ui_home.c）现在还没有扫描结果列表，所以给一条「临时便捷路径」：
       只要用户把音频输出设为「蓝牙音箱」，且这次只扫到一台音箱，就直接连它。
       等界面加上列表、能选 MAC 之后，这段可以删掉（见本文件末尾的 TODO(bt-ui)）。 */
    if (s_stack_up && s_state == BT_IDLE && !s_skip_auto_connect &&
            bt_link_audio_to_bt() && audio == 1) {
        uint8_t bda[BT_ADDR_LEN];
        char name[32] = "";
        bool is_audio = false;
        for (int i = 0; i < total; i++) {
            if (bt_scan_copy(i, name, sizeof(name), NULL, 0, bda, &is_audio) && is_audio) {
                ESP_LOGI(TAG, "蓝牙：音频输出是蓝牙音箱，自动连接唯一找到的音箱 %s",
                         name[0] != '\0' ? name : "(未知)");
                bt_do_connect(bda);
                break;
            }
        }
    } else if (total > 0 && audio > 1) {
        ESP_LOGI(TAG, "蓝牙：扫到多台音箱，界面还没有列表，先从上面日志里挑 MAC（下一轮界面会支持点选）");
    }
}

/** 开始扫描（协议栈已就绪时）。 */
static void bt_do_scan(void)
{
    if (s_state == BT_CONNECTED) {
        ESP_LOGW(TAG, "蓝牙：已经连着音箱，先断开再扫描");
        return;
    }
    if (s_scanning) {
        ESP_LOGI(TAG, "蓝牙：已经在扫描了");
        return;
    }

    bt_lock();
    s_scan_count = 0;
    memset(s_scan, 0, sizeof(s_scan));
    bt_unlock();

    s_state = BT_SCANNING;
    s_idle_deadline_us = 0;

    const esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, BT_INQ_LEN, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙：启动扫描失败（%s）", esp_err_to_name(err));
        s_state = BT_IDLE;
        s_idle_deadline_us = esp_timer_get_time() + (int64_t)BT_IDLE_RELEASE_MS * 1000;
        return;
    }
    ESP_LOGI(TAG, "蓝牙：开始扫描（约 %u 秒）…", (unsigned)(BT_INQ_LEN * 128 / 100));
}

/** 启动音频流，并在「就绪」时打一次日志。 */
static void bt_start_stream_and_log(void)
{
    s_idle_deadline_us = 0;
    if (!bt_a2dp_stream_start()) {
        return;
    }
    if (bt_link_audio_to_bt()) {
        if (!s_ready_logged) {
            s_ready_logged = true;
            ESP_LOGI(TAG, "蓝牙音频通路已就绪：TTS 可以送 A2DP（下一轮接 voice_audio 的播放路由）");
        }
    } else {
        ESP_LOGI(TAG, "蓝牙音频通路已打开；当前音频输出还是耳机孔，切到「蓝牙音箱」就出声");
    }
}

/** 连接指定地址（协议栈已就绪时）。 */
static void bt_do_connect(const uint8_t *bda)
{
    if (bda == NULL) {
        return;
    }
    if (s_state == BT_CONNECTED) {
        ESP_LOGI(TAG, "蓝牙：已经连着了（%s），先断开", bt_link_device_name());
        return;
    }
    if (!s_a2dp_inited) {
        ESP_LOGE(TAG, "蓝牙：A2DP source 还没就绪，无法连接");
        return;
    }

    if (s_scanning) {
        esp_bt_gap_cancel_discovery();
        s_scanning = false;
        s_scan_done = false;
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    char addr[18] = "";
    char saved[32] = "";
    bt_bda2str(bda, addr, sizeof(addr));
    bt_saved_name_for(bda, saved, sizeof(saved));
    bt_set_dev_name_from_scan(bda, saved[0] != '\0' ? saved : NULL);
    memcpy(s_peer_bda, bda, BT_ADDR_LEN);
    s_conn_ok = false;
    s_media_started = false;
    s_state = BT_CONNECTING;
    s_idle_deadline_us = 0;
    (void)xSemaphoreTake(s_conn_sem, 0);

    const esp_err_t err = esp_a2d_source_connect(s_peer_bda);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙：连接请求发送失败（%s）", esp_err_to_name(err));
        s_state = BT_IDLE;
        s_idle_deadline_us = esp_timer_get_time() + (int64_t)BT_IDLE_RELEASE_MS * 1000;
        return;
    }
    ESP_LOGI(TAG, "蓝牙：正在连接 %s（%s），最多等 %u 秒…",
             bt_link_device_name(), addr, (unsigned)(BT_CONNECT_TIMEOUT_MS / 1000));

    if (xSemaphoreTake(s_conn_sem, pdMS_TO_TICKS(BT_CONNECT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "蓝牙：连接超时（音箱关机 / 不在范围内 / 已被别的设备占着），放弃 %s", addr);
        esp_a2d_source_disconnect(s_peer_bda);
        s_state = BT_IDLE;
        s_idle_deadline_us = esp_timer_get_time() + (int64_t)BT_IDLE_RELEASE_MS * 1000;
        return;
    }

    if (!s_conn_ok) {
        ESP_LOGE(TAG, "蓝牙：连接失败 %s（对方拒绝或链路断开）", addr);
        if (s_state != BT_OFF && s_state != BT_INIT) {
            s_state = BT_IDLE;
        }
        s_idle_deadline_us = esp_timer_get_time() + (int64_t)BT_IDLE_RELEASE_MS * 1000;
        return;
    }

    /* 连接成功：记住它（下次开机 / 切到蓝牙音箱时自动重连） */
    {
        char name[32] = "";
        bt_copy_dev_name(name, sizeof(name));
        bt_lock();
        memcpy(s_saved_bda, s_peer_bda, BT_ADDR_LEN);
        snprintf(s_saved_name, sizeof(s_saved_name), "%s", name);
        s_have_saved = true;
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
            esp_err_t e1 = nvs_set_blob(h, KEY_LAST_BDA, s_peer_bda, BT_ADDR_LEN);
            esp_err_t e2 = nvs_set_str(h, KEY_LAST_NAME, name);
            if (e1 == ESP_OK && e2 == ESP_OK) {
                nvs_commit(h);
            } else {
                ESP_LOGW(TAG, "蓝牙：保存音箱地址失败（%s/%s）",
                         esp_err_to_name(e1), esp_err_to_name(e2));
            }
            nvs_close(h);
        }
        bt_unlock();
    }

    bt_start_stream_and_log();
}

/** 断开当前连接（协议栈留着，空闲后自动释放）。 */
static void bt_do_disconnect(void)
{
    if (!s_conn_ok) {
        ESP_LOGI(TAG, "蓝牙：当前没有连接");
        if (s_state != BT_OFF && s_state != BT_INIT) {
            s_state = BT_IDLE;
        }
        s_skip_auto_connect = true;
        if (s_stack_up) {
            s_idle_deadline_us = esp_timer_get_time() + (int64_t)BT_IDLE_RELEASE_MS * 1000;
        }
        return;
    }

    ESP_LOGI(TAG, "蓝牙：正在断开 %s…", bt_link_device_name());
    (void)xSemaphoreTake(s_conn_sem, 0);
    esp_a2d_source_disconnect(s_peer_bda);
    if (xSemaphoreTake(s_conn_sem, pdMS_TO_TICKS(3000)) != pdTRUE) {
        ESP_LOGW(TAG, "蓝牙：断开没等到事件，强制当已断开");
    }
    s_conn_ok = false;
    s_media_started = false;
    s_ready_logged = false;
    s_fifo_rd = s_fifo_wr;
    s_skip_auto_connect = true;
    if (s_state != BT_OFF && s_state != BT_INIT) {
        s_state = BT_IDLE;
    }
    s_idle_deadline_us = esp_timer_get_time() + (int64_t)BT_IDLE_RELEASE_MS * 1000;
    ESP_LOGI(TAG, "蓝牙：已断开，%u 秒没动静就释放协议栈",
             (unsigned)(BT_IDLE_RELEASE_MS / 1000));
}

/** 投一条命令（界面线程用，不阻塞）。 */
static bool bt_post_cmd(bt_cmd_op_t op, const uint8_t *bda)
{
    if (s_queue == NULL) {
        return false;
    }
    bt_cmd_t cmd = {0};
    cmd.op = (uint8_t)op;
    if (bda != NULL) {
        memcpy(cmd.bda, bda, BT_ADDR_LEN);
    }
    if (xQueueSend(s_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "蓝牙：命令队列满（op=%d），请求被丢弃", (int)op);
        return false;
    }
    return true;
}

static void bt_worker_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "蓝牙任务已就绪（协议栈未启动、未占内存；扫描/连接时再按需启动）");

    for (;;) {
        /* 扫描收尾（GAP 回调只置标志，收尾在这里做，避免在 BTU 任务里调 API） */
        if (s_scan_done) {
            s_scan_done = false;
            bt_scan_finished();
        }

        /* 对端主动连上来（音箱自己回连）时也要把音频流开起来；
           失败最多重试 3 次，免得每 500ms 刷一条错误日志 */
        if (s_state == BT_CONNECTED && s_conn_ok && !s_media_started && s_a2dp_inited &&
                s_stream_retries < 3) {
            s_stream_retries++;
            bt_start_stream_and_log();
        }

        /* 空闲释放：到点就关协议栈 */
        int64_t deadline = s_idle_deadline_us;
        TickType_t wait = pdMS_TO_TICKS(500);
        if (deadline > 0) {
            const int64_t left = deadline - esp_timer_get_time();
            if (left <= 0) {
                bt_stack_stop("空闲超时");
                continue;
            }
            const TickType_t w = pdMS_TO_TICKS((left + 999) / 1000);
            if (w < wait) {
                wait = w;
            }
            if (wait == 0) {
                wait = 1;
            }
        }

        bt_cmd_t cmd;
        if (xQueueReceive(s_queue, &cmd, wait) != pdTRUE) {
            continue;
        }

        switch ((bt_cmd_op_t)cmd.op) {
        case BT_CMD_ENABLE:
            (void)bt_stack_start();
            break;

        case BT_CMD_SCAN:
            if (bt_stack_start() == ESP_OK) {
                bt_do_scan();
            }
            break;

        case BT_CMD_CONNECT:
            if (bt_stack_start() == ESP_OK) {
                bt_do_connect(cmd.bda);
            }
            break;

        case BT_CMD_RECONNECT_SAVED:
            if (bt_stack_start() == ESP_OK) {
                bt_do_reconnect_saved();
            }
            break;

        case BT_CMD_DISCONNECT:
            bt_do_disconnect();
            break;

        case BT_CMD_DISABLE:
            bt_stack_stop("界面关闭");
            break;

        default:
            ESP_LOGW(TAG, "蓝牙：未知命令 %d", (int)cmd.op);
            break;
        }
    }
}
#endif /* BT_LINK_ENABLED */

/* ------------------------------------------------------------------------- */
/* 对外 API                                                                  */
/* ------------------------------------------------------------------------- */

void bt_link_init(void)
{
    if (s_inited) {
        return;                 /* 幂等：界面重建 / 重复调用都安全 */
    }
    s_inited = true;
    s_state = BT_OFF;
    s_ever_started = false;

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            ESP_LOGE(TAG, "蓝牙：互斥量创建失败，模块不可用");
        }
    }

    /* 读 NVS：音频输出偏好 + 上次连过的音箱（都不启动协议栈） */
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        uint8_t v = 0;
        err = nvs_get_u8(h, KEY_AUDIO_OUT, &v);
        if (err == ESP_OK) {
            s_audio_bt = (v != 0);
        } else if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "读取音频输出偏好失败（%s），用默认耳机孔", esp_err_to_name(err));
        }

        size_t bda_len = sizeof(s_saved_bda);
        if (nvs_get_blob(h, KEY_LAST_BDA, s_saved_bda, &bda_len) == ESP_OK &&
                bda_len == BT_ADDR_LEN) {
            size_t name_len = sizeof(s_saved_name);
            if (nvs_get_str(h, KEY_LAST_NAME, s_saved_name, &name_len) != ESP_OK) {
                s_saved_name[0] = '\0';
            }
            s_have_saved = true;
        }
        nvs_close(h);
    } else {
        ESP_LOGD(TAG, "NVS 命名空间 %s 不可读（%s），音频输出用默认耳机孔",
                 NVS_NAMESPACE, esp_err_to_name(err));
    }

#if BT_LINK_ENABLED
    s_ack_sem = xSemaphoreCreateBinary();
    s_conn_sem = xSemaphoreCreateBinary();
    s_prof_sem = xSemaphoreCreateBinary();
    if (s_ack_sem == NULL || s_conn_sem == NULL || s_prof_sem == NULL) {
        ESP_LOGE(TAG, "蓝牙：信号量创建失败，模块不可用");
        s_state = BT_ERROR;
        return;
    }

    s_queue = xQueueCreate(BT_CMD_QUEUE_LEN, sizeof(bt_cmd_t));
    if (s_queue == NULL) {
        ESP_LOGE(TAG, "蓝牙：命令队列创建失败，模块不可用");
        s_state = BT_ERROR;
        return;
    }

    if (xTaskCreate(bt_worker_task, "bt_link", BT_TASK_STACK, NULL, BT_TASK_PRIO, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "蓝牙：任务创建失败，模块不可用");
        s_state = BT_ERROR;
        return;
    }

    bt_log_heap("模块初始化（协议栈未启动）");
    ESP_LOGI(TAG, "蓝牙：音频输出=%s，上次音箱=%s；扫描/连接时按需启动协议栈，空闲 %u 秒自动释放",
             s_audio_bt ? "蓝牙音箱" : "耳机孔",
             s_have_saved ? (s_saved_name[0] != '\0' ? s_saved_name : "(有记录)") : "无",
             (unsigned)(BT_IDLE_RELEASE_MS / 1000));
#else
    ESP_LOGI(TAG, "蓝牙：未启用（蓝牙栈未编译），音频输出=%s", s_audio_bt ? "蓝牙音箱" : "耳机孔");
#endif
}

bool bt_link_is_enabled(void)
{
    /* 「编译进来了」而不是「正在运行」：界面据此决定扫描/连接按钮是否可点，
       第一次点下去会按需启动协议栈。 */
    return BT_LINK_ENABLED != 0;
}

bool bt_link_enable(void)
{
#if !BT_LINK_ENABLED
    ESP_LOGW(TAG, "蓝牙未启用（蓝牙栈未编译），无法启动");
    return false;
#else
    if (!s_inited) {
        bt_link_init();
    }
    if (s_state == BT_ERROR) {
        ESP_LOGW(TAG, "蓝牙：上次出错，重试启动协议栈");
    }
    return bt_post_cmd(BT_CMD_ENABLE, NULL);
#endif
}

void bt_link_disable(void)
{
#if !BT_LINK_ENABLED
    ESP_LOGD(TAG, "蓝牙未启用（蓝牙栈未编译），关闭是空操作");
#else
    if (s_state == BT_OFF && !s_ever_started) {
        ESP_LOGD(TAG, "蓝牙：本来就没启动，关闭是空操作");
        return;
    }
    if (!bt_post_cmd(BT_CMD_DISABLE, NULL)) {
        ESP_LOGW(TAG, "蓝牙：关闭请求排队失败");
    }
#endif
}

bt_state_t bt_link_state(void)
{
    return BT_LINK_ENABLED ? s_state : BT_OFF;
}

const char *bt_link_state_text(void)
{
#if !BT_LINK_ENABLED
    return "未启用（蓝牙栈未编译）";
#else
    switch (s_state) {
    case BT_OFF:
        /* 启动过又释放 vs 从来没启动过：界面文案不一样 */
        return s_ever_started ? "蓝牙已释放（省内存）" : "未启用（待启用）";
    case BT_INIT:
        return "正在启动蓝牙…";
    case BT_IDLE:
        return "已就绪";
    case BT_SCANNING:
        return "扫描中…";
    case BT_CONNECTING:
        return "连接中…";
    case BT_CONNECTED: {
        char name[32] = "";
        bt_copy_dev_name(name, sizeof(name));
        if (name[0] != '\0') {
            snprintf(s_text_buf, sizeof(s_text_buf), "已连接 %s", name);
            return s_text_buf;
        }
        return "已连接";
    }
    case BT_ERROR:
        return "蓝牙出错";
    default:
        return "未知状态";
    }
#endif
}

const char *bt_link_device_name(void)
{
    if (!BT_LINK_ENABLED || s_state != BT_CONNECTED) {
        return "";
    }
    return s_dev_name;
}

bool bt_link_scan_start(void)
{
#if !BT_LINK_ENABLED
    ESP_LOGW(TAG, "蓝牙未启用（蓝牙栈未编译），无法扫描");
    return false;
#else
    if (!s_inited) {
        bt_link_init();
    }
    if (s_state == BT_CONNECTED) {
        ESP_LOGW(TAG, "蓝牙：已经连着音箱，先断开再扫描");
        return false;
    }
    return bt_post_cmd(BT_CMD_SCAN, NULL);
#endif
}

void bt_link_scan_stop(void)
{
#if !BT_LINK_ENABLED
    ESP_LOGD(TAG, "蓝牙未启用（蓝牙栈未编译），停止扫描是空操作");
#else
    if (!s_scanning) {
        return;
    }
    ESP_LOGI(TAG, "蓝牙：停止扫描");
    esp_bt_gap_cancel_discovery();
#endif
}

bool bt_link_get_scan_result(int index, char *name, size_t name_len,
                             char *addr, size_t addr_len)
{
    /* 防御：调用方可能拿 NULL 缓冲来问「有没有结果」 */
    if (name != NULL && name_len > 0) {
        name[0] = '\0';
    }
    if (addr != NULL && addr_len > 0) {
        addr[0] = '\0';
    }
#if !BT_LINK_ENABLED
    (void)index;
    return false;
#else
    return bt_scan_copy(index, name, name_len, addr, addr_len, NULL, NULL);
#endif
}

bool bt_link_connect(const char *addr)
{
#if !BT_LINK_ENABLED
    ESP_LOGW(TAG, "蓝牙未启用（蓝牙栈未编译），无法连接 %s",
             (addr != NULL && addr[0] != '\0') ? addr : "(空地址)");
    return false;
#else
    if (!s_inited) {
        bt_link_init();
    }
    if (s_state == BT_CONNECTED) {
        ESP_LOGI(TAG, "蓝牙：已经连着 %s，先断开", bt_link_device_name());
        return false;
    }

    if (addr == NULL || addr[0] == '\0') {
        /* 空地址：连上次连过的音箱（界面还没有列表时的兜底） */
        return bt_post_cmd(BT_CMD_RECONNECT_SAVED, NULL);
    }

    uint8_t bda[BT_ADDR_LEN];
    if (!bt_str2bda(addr, bda)) {
        ESP_LOGE(TAG, "蓝牙：地址格式不对（%s），要 aa:bb:cc:dd:ee:ff", addr);
        return false;
    }
    return bt_post_cmd(BT_CMD_CONNECT, bda);
#endif
}

void bt_link_disconnect(void)
{
#if !BT_LINK_ENABLED
    ESP_LOGD(TAG, "蓝牙未启用（蓝牙栈未编译），断开是空操作");
#else
    if (!s_conn_ok) {
        return;
    }
    (void)bt_post_cmd(BT_CMD_DISCONNECT, NULL);
#endif
}

bool bt_link_audio_to_bt(void)
{
    /* 协议栈没启用时有效输出只能是耳机孔（NVS 里存过 true 也一样） */
    return BT_LINK_ENABLED && s_audio_bt;
}

bool bt_link_audio_is_ready(void)
{
    return BT_LINK_ENABLED && s_conn_ok && s_media_started && s_audio_bt;
}

void bt_link_set_audio_to_bt(bool to_bt)
{
    if (to_bt && !BT_LINK_ENABLED) {
        /* 界面点了「蓝牙音箱」但协议栈没编译：选择照存，但这次运行的有效输出仍是耳机孔 */
        ESP_LOGW(TAG, "蓝牙未启用（蓝牙栈未编译），音频输出仍走耳机孔（选择已记录）");
    }

    bt_lock();
    if (s_audio_bt == to_bt) {
        /* 幂等：值没变就不写 NVS（界面重建/重复点击都不会碰 flash） */
        bt_unlock();
        return;
    }
    s_audio_bt = to_bt;
    s_skip_auto_connect = false;    /* 用户重新选了蓝牙输出：允许自动重连上次的音箱 */

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, KEY_AUDIO_OUT, to_bt ? 1 : 0);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    bt_unlock();

    if (err != ESP_OK) {
        /* 和 app_settings 一样：存不下也先让内存里的值生效，只报警 */
        ESP_LOGW(TAG, "音频输出偏好保存失败（%s），只在本次运行内生效", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "音频输出：%s", to_bt ? "蓝牙音箱" : "耳机孔");
    }

#if BT_LINK_ENABLED
    if (to_bt) {
        /* 用户明确要用蓝牙出声：按需启动协议栈（不占开机内存）；
           有上次连过的音箱就直接重连，没有就等扫描。 */
        ESP_LOGI(TAG, "蓝牙：切到蓝牙音箱，按需启动协议栈");
        if (s_have_saved) {
            (void)bt_post_cmd(BT_CMD_RECONNECT_SAVED, NULL);
        } else {
            (void)bt_post_cmd(BT_CMD_ENABLE, NULL);
        }
    }
#endif
}

void bt_link_audio_write(const int16_t *pcm, size_t samples, int sample_rate)
{
#if !BT_LINK_ENABLED
    (void)pcm;
    (void)samples;
    (void)sample_rate;
#else
    if (pcm == NULL || samples == 0) {
        return;
    }
    if (s_fifo == NULL || s_stage == NULL) {
        return;         /* 缓冲没申请到 / 协议栈没起来：直接丢 */
    }
    if (!bt_link_audio_is_ready()) {
        return;         /* 没就绪就丢，不阻塞调用方（下一轮接路由时再加统计） */
    }
    if (sample_rate <= 0) {
        sample_rate = 24000;
    }

    if (sample_rate == BT_PCM_RATE) {
        for (size_t i = 0; i < samples; i++) {
            bt_stage_push(pcm[i]);
        }
    } else {
        if (!s_rs_started || s_rs_rate != sample_rate) {
            /* 换采样率：重置相位（16.16 定点），从第一个样本开始 */
            s_rs_rate = sample_rate;
            s_rs_step = (uint32_t)(((uint64_t)65536u * (uint32_t)sample_rate) / BT_PCM_RATE);
            if (s_rs_step == 0) {
                s_rs_step = 1;
            }
            s_rs_phase = 0;
            s_rs_last = pcm[0];
            s_rs_started = true;
        }
        for (size_t i = 0; i < samples; i++) {
            const int32_t cur = pcm[i];
            while (s_rs_phase < 65536u) {
                /* 用 64 位算插值：差值最大 65535 × 相位 65535，int32 会溢出 */
                const int64_t d = (int64_t)(cur - s_rs_last) * (int64_t)s_rs_phase;
                bt_stage_push((int32_t)(s_rs_last + (int32_t)(d >> 16)));
                s_rs_phase += s_rs_step;
            }
            s_rs_phase -= 65536u;
            s_rs_last = cur;
        }
    }
    bt_stage_flush();
#endif
}
