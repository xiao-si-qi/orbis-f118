/*
 * LVGL 9.6 自定义分配器：把 lv_malloc/lv_realloc/lv_free 直接落到 PSRAM。
 *
 * 为什么这么做：
 *   LVGL 的内置分配器（LV_USE_STDLIB_MALLOC == LV_STDLIB_BUILTIN）会在
 *   .bss 里放一个 LV_MEM_SIZE（本项目 64 KB）的静态数组 work_mem_int[]，
 *   吃的是这块板子上最紧张的内部 RAM。LVGL 9.6 的 Kconfig 提供了一个
 *   "Custom (implemented externally)" 选项：选中后 liblvgl 不再编译内置
 *   的 mem core，而是由宿主工程实现下面这几个 lv_*_core()。
 *   这里用 ESP-IDF 的 heap_caps_* 显式从 PSRAM 取内存，64 KB 静态池随之
 *   消失，省下来的内部 RAM 留给以后要加的蓝牙协议栈。
 *
 * 注意：
 *   - PSRAM 不能给 SPI DMA 直接读，所以 main.c 里的两片绘图缓冲（本来就
 *     在 PSRAM）和 flush 用的内部 RAM 中转缓冲保持原样，本文件只管 LVGL
 *     的对象/样式/文本等堆内存。
 *   - 本项目 LV_USE_OS == LV_OS_NONE，所有 lv_* 调用都来自 LVGL 主循环
 *     这一个任务，因此这里不需要额外加锁（heap_caps_* 本身是线程安全的）。
 *   - 失败时返回 NULL，语义与 TLSF 内置池一致；lv_mem.c 会打印错误并给出
 *     内存统计，界面创建失败时由 ui_home.c 现有的防御逻辑兜底。
 */

/*********************
 *      INCLUDES
 *********************/
#include <stddef.h>
#include <stdint.h>

#include "esp_heap_caps.h"
#include "lvgl.h"

/*********************
 *      DEFINES
 *********************/
/* 只接受 PSRAM：显式带 MALLOC_CAP_SPIRAM，不让分配器回落到内部 RAM。 */
#define LV_PSRAM_CAPS   (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

/* 每个块前面留 8 字节记录请求大小：
 *   1) lv_mem_monitor_core() 要算出 LVGL 自己占了多少（heap_caps 只给整堆统计）；
 *   2) 8 字节偏移在 32 位目标上仍能让返回给 LVGL 的指针保持 4/8 字节对齐，
 *      与内置 TLSF 的 ALIGN_SIZE 一致。 */
#define LV_PSRAM_HDR    8u

/**********************
 *      TYPEDEFS
 **********************/
typedef struct {
    size_t size;        /* 负载字节数（不含本头部） */
} lv_psram_hdr_t;

/**********************
 *  STATIC VARIABLES
 **********************/
static size_t s_used;   /* 当前 LVGL 占用（负载字节） */
static size_t s_peak;   /* 峰值占用 */
static size_t s_live;   /* 当前存活的块数 */

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_mem_init(void)
{
    s_used = 0;
    s_peak = 0;
    s_live = 0;
}

void lv_mem_deinit(void)
{
    /* 没有独立池要销毁：块都在 PSRAM 堆里，正常流程下 lv_deinit() 之前
       对象已经逐个 lv_free() 归还。这里只把统计清零。 */
    s_used = 0;
    s_peak = 0;
    s_live = 0;
}

lv_mem_pool_t lv_mem_add_pool(void *mem, size_t bytes)
{
    /* 直接使用 PSRAM 堆，没有固定池可加（与 LVGL 的 CLIB 后端行为一致）。 */
    LV_UNUSED(mem);
    LV_UNUSED(bytes);
    return NULL;
}

void lv_mem_remove_pool(lv_mem_pool_t pool)
{
    LV_UNUSED(pool);
}

void *lv_malloc_core(size_t size)
{
    if (size == 0 || size > SIZE_MAX - LV_PSRAM_HDR) {
        return NULL;    /* lv_mem.c 对 0 字节有专门的 zero_mem，走不到这里 */
    }

    lv_psram_hdr_t *hdr = heap_caps_malloc(LV_PSRAM_HDR + size, LV_PSRAM_CAPS);
    if (hdr == NULL) {
        return NULL;
    }

    hdr->size = size;
    s_used += size;
    s_live++;
    if (s_used > s_peak) {
        s_peak = s_used;
    }
    return (uint8_t *)hdr + LV_PSRAM_HDR;
}

void *lv_realloc_core(void *p, size_t new_size)
{
    if (p == NULL) {
        return lv_malloc_core(new_size);
    }
    if (new_size == 0) {
        lv_free_core(p);
        return NULL;
    }
    if (new_size > SIZE_MAX - LV_PSRAM_HDR) {
        return NULL;
    }

    lv_psram_hdr_t *hdr = (lv_psram_hdr_t *)((uint8_t *)p - LV_PSRAM_HDR);
    const size_t old_size = hdr->size;

    /* heap_caps_realloc() 会保留头部内容（必要时自行搬移并拷贝）；失败时旧块不动。 */
    lv_psram_hdr_t *new_hdr = heap_caps_realloc(hdr, LV_PSRAM_HDR + new_size, LV_PSRAM_CAPS);
    if (new_hdr == NULL) {
        return NULL;
    }

    new_hdr->size = new_size;
    s_used = s_used - old_size + new_size;
    if (s_used > s_peak) {
        s_peak = s_used;
    }
    return (uint8_t *)new_hdr + LV_PSRAM_HDR;
}

void lv_free_core(void *p)
{
    if (p == NULL) {
        return;
    }

    lv_psram_hdr_t *hdr = (lv_psram_hdr_t *)((uint8_t *)p - LV_PSRAM_HDR);
    const size_t size = hdr->size;

    s_used = (s_used >= size) ? (s_used - size) : 0;
    if (s_live > 0) {
        s_live--;
    }
    heap_caps_free(hdr);
}

void lv_mem_monitor_core(lv_mem_monitor_t *mon_p)
{
    /* 没有固定池：把「LVGL 已用 + PSRAM 剩余」当作 LVGL 可支配的总量。
       total_size/free_size 反映的是 PSRAM 堆，max_used/used_cnt 才是 LVGL 自己的。 */
    const size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const size_t big_psram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    const size_t total = s_used + free_psram;

    mon_p->total_size = total;
    mon_p->free_size = free_psram;
    mon_p->free_biggest_size = big_psram;
    mon_p->used_cnt = s_live;
    mon_p->free_cnt = 0;
    mon_p->max_used = s_peak;
    mon_p->used_pct = (uint8_t)(total ? ((uint64_t)s_used * 100u / total) : 0);
    mon_p->frag_pct = free_psram
                      ? (uint8_t)(100u - ((uint64_t)big_psram * 100u / free_psram))
                      : 0;
}

lv_result_t lv_mem_test_core(void)
{
    /* 只做整块 PSRAM 堆的一致性检查（lv_mem_test() 目前没有调用点）。 */
    return heap_caps_check_integrity(MALLOC_CAP_SPIRAM, true)
           ? LV_RESULT_OK : LV_RESULT_INVALID;
}
