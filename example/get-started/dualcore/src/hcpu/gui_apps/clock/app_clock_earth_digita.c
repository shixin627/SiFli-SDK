#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "app_mainmenu.h"
#include "app_clock_main.h"
#include "app_mem.h"
#include "common_widget.h"
#include "watch_system_interact.h"
#include "ui_img_helper.h"
#include "ui_helper.h"
#include "lv_ext_resource_manager.h"

#define DBG_LVL DBG_LOG
#include <rtdbg.h>

LV_IMG_DECLARE(img_earth_digital_bg);
LV_IMG_DECLARE(media_gaus_bg);
LV_IMG_DECLARE(dig_1_00);
LV_IMG_DECLARE(dig_1_01);
LV_IMG_DECLARE(dig_1_02);
LV_IMG_DECLARE(dig_1_03);
LV_IMG_DECLARE(dig_1_04);
LV_IMG_DECLARE(dig_1_05);
LV_IMG_DECLARE(dig_1_06);
LV_IMG_DECLARE(dig_1_07);
LV_IMG_DECLARE(dig_1_08);
LV_IMG_DECLARE(dig_1_09);

typedef struct
{
    lv_obj_t *bg;
    lv_obj_t *hour_0_img;
    lv_obj_t *hour_1_img;
    lv_obj_t *minute_0_img;
    lv_obj_t *minute_1_img;
    lv_timer_t *redraw_task;
    app_clock_time_t last_redraw_time;
    lv_obj_t *earth_img;
    lv_obj_t *ampm_label; /* AM/PM tag, filled in 12h mode, empty in 24h */
} app_clock_earth_digital_t;

static app_clock_earth_digital_t *p_clk_earth_digital = NULL;

/* Digit image table, shared by the redraw task and the diagnostics below. */
static const lv_img_dsc_t *const s_dig[10] = {
    &dig_1_00, &dig_1_01, &dig_1_02, &dig_1_03, &dig_1_04,
    &dig_1_05, &dig_1_06, &dig_1_07, &dig_1_08, &dig_1_09};

/* ---- Digit diagnostics (2026-10-25) ---------------------------------------
   Field report: the first hour digit sometimes vanishes and comes back, with
   no change in the displayed minute. The digits are plain flash-resident EZIP
   images, so the question is whether LVGL stopped drawing them (object state,
   skipped draw) or drew them and the pixels never made it to the panel.

   This keeps a small RAM-only history (no flash writes) and writes it into the
   black-box ring + pushes it to the phone ONLY on request:
     - Settings > Development > "Dump Digit Diag" (do it right after you see the
       digit missing), or
     - automatically (once per 30 min) when an audit finds a digit hidden or
       holding the wrong image.
   Records are kept on the GUI thread only, so there is no locking. */
#ifndef WIN32
extern void log_file_diag_printf(const char *fmt, ...);
extern void log_file_diag_push(void);
extern volatile uint32_t lv_gpu_img_draw_err_cnt;
extern const void *volatile lv_gpu_img_draw_err_src;
#define DG_PRINTF log_file_diag_printf
#define DG_PUSH() log_file_diag_push()
#else
static volatile uint32_t lv_gpu_img_draw_err_cnt;
static const void *volatile lv_gpu_img_draw_err_src;
#define DG_PRINTF rt_kprintf
#define DG_PUSH() ((void)0)
#endif
#ifdef BSP_USING_EPIC
extern bool drv_epic_is_busy(void);
#endif

#define DG_AUDITS 24          /* 24 x 15 s = the last 6 minutes of the face */
#define DG_AUDIT_MS 15000
#define DG_AUTO_GAP_MS (30u * 60u * 1000u)

typedef struct
{
    uint32_t ms;              /* lv_tick_get() when taken */
    char tag;                 /* R resume, M minute change, T timer, D dump */
    uint8_t hh, mm;           /* what the face was told to show */
    uint8_t src[4];           /* which digit image each object holds; 0xFF = not a digit image */
    uint8_t hid;              /* bit i: digit i, or an ancestor of it, is HIDDEN */
    uint8_t opa[4];           /* effective opacity (object x ancestors) */
    uint8_t epic_busy;
    uint16_t draws[4];        /* LV_EVENT_DRAW_MAIN_END count per digit, cumulative */
    uint32_t heap_used, heap_max;
    uint32_t err_cnt;         /* images the EPIC path skipped (decode/open failed) */
} dg_audit_t;

static dg_audit_t s_dg_ring[DG_AUDITS];
static uint32_t s_dg_head;     /* total audits taken */
static uint32_t s_dg_last_ms;  /* last audit */
static uint32_t s_dg_auto_ms;  /* last automatic dump */
static bool s_dg_auto_used;
static uint8_t s_dg_expect[4] = {0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t s_dg_hh, s_dg_mm;
static struct
{
    uint32_t ms;
    uint16_t n;
    lv_area_t clip;
} s_dg_draw[4];

static void dg_draw_cb(lv_event_t *e)
{
    int i = (int)(intptr_t)lv_event_get_user_data(e);
    lv_draw_ctx_t *dc = lv_event_get_draw_ctx(e);

    s_dg_draw[i].ms = lv_tick_get();
    s_dg_draw[i].n++;
    if (dc != NULL && dc->clip_area != NULL)
    {
        s_dg_draw[i].clip = *dc->clip_area;
    }
}

static void app_clock_digit_diag_dump_now(const char *why);

static void dg_audit(char tag)
{
    app_clock_earth_digital_t *p = p_clk_earth_digital;
    if (p == NULL)
    {
        return;
    }
    lv_obj_t *img[4] = {p->hour_0_img, p->hour_1_img, p->minute_0_img,
                        p->minute_1_img};
    dg_audit_t *a = &s_dg_ring[s_dg_head++ % DG_AUDITS];
    rt_uint32_t total = 0;
    bool bad = false;

    memset(a, 0, sizeof(*a));
    a->ms = lv_tick_get();
    a->tag = tag;
    a->hh = s_dg_hh;
    a->mm = s_dg_mm;
    for (int i = 0; i < 4; i++)
    {
        uint32_t opa;
        a->src[i] = 0xFF;
        if (lv_obj_is_valid(img[i]))
        {
            const void *s = lv_img_get_src(img[i]);
            opa = lv_obj_get_style_img_opa(img[i], 0);
            for (int k = 0; k < 10; k++)
            {
                if (s == (const void *)s_dig[k])
                {
                    a->src[i] = (uint8_t)k;
                }
            }
            for (lv_obj_t *o = img[i]; o != NULL; o = lv_obj_get_parent(o))
            {
                if (lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN))
                {
                    a->hid |= (uint8_t)(1u << i);
                }
                opa = opa * lv_obj_get_style_opa(o, 0) / 255;
            }
            a->opa[i] = (uint8_t)opa;
        }
        a->draws[i] = s_dg_draw[i].n;
        if (a->src[i] != s_dg_expect[i] && s_dg_expect[i] != 0xFF)
        {
            bad = true;
        }
    }
    a->hid &= 0x0F;
    if (a->hid)
    {
        bad = true;
    }
#ifdef BSP_USING_EPIC
    a->epic_busy = drv_epic_is_busy() ? 1 : 0;
#endif
    rt_memory_info(&total, (rt_uint32_t *)&a->heap_used,
                   (rt_uint32_t *)&a->heap_max);
    a->err_cnt = lv_gpu_img_draw_err_cnt;

    if (bad && (!s_dg_auto_used || lv_tick_elaps(s_dg_auto_ms) >= DG_AUTO_GAP_MS))
    {
        s_dg_auto_used = true;
        s_dg_auto_ms = lv_tick_get();
        app_clock_digit_diag_dump_now("auto: digit hidden or wrong image");
    }
}

static void app_clock_digit_diag_dump_now(const char *why)
{
    rt_uint32_t total = 0, used = 0, max_used = 0;
    uint32_t n = s_dg_head < DG_AUDITS ? s_dg_head : DG_AUDITS;

    DG_PRINTF("\n=== DIGIT-DIAG: %s ===\n", why);
    DG_PRINTF("now=%u audits=%u (tag ms hh:mm src hidden opa draws epic heap_used/max err)\n",
              (unsigned)lv_tick_get(), (unsigned)s_dg_head);
    for (uint32_t k = 0; k < n; k++)
    {
        const dg_audit_t *a = &s_dg_ring[(s_dg_head - n + k) % DG_AUDITS];
        DG_PRINTF("%c %u %02d:%02d s=%x%x%x%x h=%x o=%x,%x,%x,%x d=%u,%u,%u,%u e%d %u/%u i%u\n",
                  a->tag, (unsigned)a->ms, a->hh, a->mm, a->src[0] & 0xF,
                  a->src[1] & 0xF, a->src[2] & 0xF, a->src[3] & 0xF, a->hid,
                  a->opa[0], a->opa[1], a->opa[2], a->opa[3], a->draws[0],
                  a->draws[1], a->draws[2], a->draws[3], a->epic_busy,
                  (unsigned)a->heap_used, (unsigned)a->heap_max,
                  (unsigned)a->err_cnt);
    }
    for (int i = 0; i < 4; i++)
    {
        DG_PRINTF("L%d age=%ums n=%u clip=(%d,%d)-(%d,%d)\n", i,
                  (unsigned)lv_tick_elaps(s_dg_draw[i].ms),
                  (unsigned)s_dg_draw[i].n, (int)s_dg_draw[i].clip.x1,
                  (int)s_dg_draw[i].clip.y1, (int)s_dg_draw[i].clip.x2,
                  (int)s_dg_draw[i].clip.y2);
    }
    DG_PRINTF("E img_draw_err=%u last_src=%x\n", (unsigned)lv_gpu_img_draw_err_cnt,
              (unsigned)(uintptr_t)lv_gpu_img_draw_err_src);
    rt_memory_info(&total, &used, &max_used);
    DG_PRINTF("H sys total=%u used=%u max=%u\n", (unsigned)total,
              (unsigned)used, (unsigned)max_used);
#ifdef RT_USING_MEMHEAP
    {
        /* app_mem's pools (image/FreeType/message caches). RT_NAME_MAX is 8, so
           names are cut; the pool size tells them apart. */
        struct
        {
            char name[RT_NAME_MAX + 1];
            rt_uint32_t pool, avail, max;
        } hp[12];
        int cnt = 0;
        struct rt_object_information *info =
            rt_object_get_information(RT_Object_Class_MemHeap);
        struct rt_list_node *node;

        rt_enter_critical();
        for (node = info->object_list.next;
             node != &info->object_list && cnt < 12; node = node->next)
        {
            struct rt_memheap *mh =
                (struct rt_memheap *)rt_list_entry(node, struct rt_object, list);
            rt_strncpy(hp[cnt].name, mh->parent.name, RT_NAME_MAX);
            hp[cnt].name[RT_NAME_MAX] = '\0';
            hp[cnt].pool = mh->pool_size;
            hp[cnt].avail = mh->available_size;
            hp[cnt].max = mh->max_used_size;
            cnt++;
        }
        rt_exit_critical();
        for (int i = 0; i < cnt; i++)
        {
            DG_PRINTF("H %s pool=%u avail=%u max=%u\n", hp[i].name,
                      (unsigned)hp[i].pool, (unsigned)hp[i].avail,
                      (unsigned)hp[i].max);
        }
    }
#endif
    DG_PRINTF("=== DIGIT-DIAG END ===\n");
    DG_PUSH();
}

/* Entry for Settings > Development > "Dump Digit Diag" (GUI thread). */
void app_clock_digit_diag_dump(const char *why)
{
    dg_audit('D');
    app_clock_digit_diag_dump_now(why);
}

#ifdef WIN32
/* PC simulator: `digit_diag` prints the same dump to the console. */
static int digit_diag_msh(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    app_clock_digit_diag_dump("msh");
    return 0;
}
MSH_CMD_EXPORT_ALIAS(digit_diag_msh, digit_diag, print the digit diagnostics);
#endif

static void app_clock_earth_digital_redraw(lv_timer_t *task)
{
    app_clock_time_t current_time;
    rt_uint8_t hours, minutes;
    static char time_buf[8];

    app_clock_main_get_current_time(&current_time);
    hours = current_time.h;
    minutes = current_time.m;

    if (task != NULL && lv_tick_elaps(s_dg_last_ms) >= DG_AUDIT_MS)
    {
        s_dg_last_ms = lv_tick_get();
        dg_audit('T');
    }

    /* Skip the per-tick image churn when the displayed minute hasn't changed.
       The 30 ms redraw task otherwise re-issues 4x lv_img_set_src (+ AM/PM
       label work) ~33 times/second on the always-visible watchface even
       though the digits only change once a minute. task == NULL is the forced
       first paint from resume_callback: always draw then. */
    if (task != NULL &&
        hours == p_clk_earth_digital->last_redraw_time.h &&
        minutes == p_clk_earth_digital->last_redraw_time.m)
    {
        return;
    }

    // Respect the 12/24-hour preference before splitting into digit images.
    rt_uint8_t disp_hours = ui_time_display_hour(hours);

    // Update hour display with images
    int hour_0 = disp_hours / 10;
    int hour_1 = disp_hours % 10;
    int min_0 = minutes / 10;
    int min_1 = minutes % 10;

    lv_img_set_src(p_clk_earth_digital->hour_0_img, s_dig[hour_0]);
    lv_img_set_src(p_clk_earth_digital->hour_1_img, s_dig[hour_1]);
    lv_img_set_src(p_clk_earth_digital->minute_0_img, s_dig[min_0]);
    lv_img_set_src(p_clk_earth_digital->minute_1_img, s_dig[min_1]);
    s_dg_expect[0] = (uint8_t)hour_0;
    s_dg_expect[1] = (uint8_t)hour_1;
    s_dg_expect[2] = (uint8_t)min_0;
    s_dg_expect[3] = (uint8_t)min_1;
    s_dg_hh = disp_hours;
    s_dg_mm = minutes;
    s_dg_last_ms = lv_tick_get();
    dg_audit(task != NULL ? 'M' : 'R');

    if (lv_obj_is_valid(p_clk_earth_digital->ampm_label))
    {
        const char *ampm = ui_time_ampm(hours);
        lv_label_set_text(p_clk_earth_digital->ampm_label, ampm ? ampm : "");
        /* Re-align here (not just at create): by now the minute image has its
           real size, so OUT_BOTTOM_RIGHT actually lands below the digit. The
           create-time align ran before the image size settled. */
        lv_obj_align_to(p_clk_earth_digital->ampm_label,
                        p_clk_earth_digital->minute_1_img,
                        LV_ALIGN_OUT_BOTTOM_RIGHT, -10, 0);
    }

    memcpy(&p_clk_earth_digital->last_redraw_time, &current_time,
           sizeof(app_clock_time_t));
}

static rt_int32_t resume_callback(void)
{
    if (NULL == p_clk_earth_digital->redraw_task)
    {
        p_clk_earth_digital->redraw_task =
            lv_timer_create(app_clock_earth_digital_redraw, 30, (void *)0);
        app_clock_earth_digital_redraw(NULL);
    }

    return RT_EOK;
}

static rt_int32_t pause_callback(void)
{
    if (p_clk_earth_digital->redraw_task)
    {
        lv_timer_del(p_clk_earth_digital->redraw_task);
        p_clk_earth_digital->redraw_task = NULL;
    }

    return RT_EOK;
}

lv_obj_t *lv_earth_digital_layout_create(lv_obj_t *parent)
{
    p_clk_earth_digital = (app_clock_earth_digital_t *)rt_malloc(
        sizeof(app_clock_earth_digital_t));
    memset(p_clk_earth_digital, 0, sizeof(app_clock_earth_digital_t));

    // Create background with subtle gradient
    p_clk_earth_digital->bg = lv_obj_create(parent);
    lv_obj_set_size(p_clk_earth_digital->bg, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_align(p_clk_earth_digital->bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(p_clk_earth_digital->bg, lv_color_hex(0x000000),
                              0);
    lv_obj_set_style_radius(p_clk_earth_digital->bg, 233, 0);
    lv_obj_clear_flag(p_clk_earth_digital->bg, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *h_0 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(h_0, 100, 136);
    lv_obj_align(h_0, LV_ALIGN_CENTER, -153, -10);
    lv_obj_set_style_bg_color(h_0, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(h_0, 0, 0);

    // Create hour label
    p_clk_earth_digital->hour_0_img = lv_img_create(h_0);
    lv_img_set_src(p_clk_earth_digital->hour_0_img, &dig_1_00);
    lv_obj_center(p_clk_earth_digital->hour_0_img);

    lv_obj_t *h_1 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(h_1, 100, 136);
    lv_obj_align(h_1, LV_ALIGN_CENTER, -52, -10);
    lv_obj_set_style_bg_color(h_1, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(h_1, 0, 0);

    p_clk_earth_digital->hour_1_img = lv_img_create(h_1);
    lv_img_set_src(p_clk_earth_digital->hour_1_img, &dig_1_00);
    lv_obj_center(p_clk_earth_digital->hour_1_img);

    lv_obj_t *m_0 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(m_0, 100, 136);
    lv_obj_align(m_0, LV_ALIGN_CENTER, 52, -10);
    lv_obj_set_style_bg_color(m_0, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(m_0, 0, 0);

    p_clk_earth_digital->minute_0_img = lv_img_create(m_0);
    lv_img_set_src(p_clk_earth_digital->minute_0_img, &dig_1_00);
    lv_obj_set_style_img_opa(p_clk_earth_digital->minute_0_img, LV_OPA_70, 0);
    lv_obj_center(p_clk_earth_digital->minute_0_img);

    lv_obj_t *m_1 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(m_1, 100, 136);
    lv_obj_align(m_1, LV_ALIGN_CENTER, 153, -10);
    lv_obj_set_style_bg_color(m_1, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(m_1, 0, 0);

    p_clk_earth_digital->minute_1_img = lv_img_create(m_1);
    lv_img_set_src(p_clk_earth_digital->minute_1_img, &dig_1_00);
    lv_obj_set_style_img_opa(p_clk_earth_digital->minute_1_img, LV_OPA_70, 0);
    lv_obj_center(p_clk_earth_digital->minute_1_img);

    /* Diagnostics: note every time LVGL actually draws each digit (and the clip
       it drew into). See the digit-diagnostics block above. */
    {
        lv_obj_t *dg_img[4] = {p_clk_earth_digital->hour_0_img,
                               p_clk_earth_digital->hour_1_img,
                               p_clk_earth_digital->minute_0_img,
                               p_clk_earth_digital->minute_1_img};
        for (int i = 0; i < 4; i++)
        {
            lv_obj_add_event_cb(dg_img[i], dg_draw_cb, LV_EVENT_DRAW_MAIN_END,
                                (void *)(intptr_t)i);
        }
    }

    lv_obj_set_style_bg_opa(h_0, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(h_1, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(m_0, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(m_1, LV_OPA_TRANSP, 0);

    p_clk_earth_digital->earth_img = lv_img_create(p_clk_earth_digital->bg);
    /* The earth picture is a file now (ui_img_helper.h, IMG_EARTH_DIGITAL_BG): a
       watch that has not received it yet shows the digits on plain black. */
    {
        const void *earth = ui_fs_img_or(IMG_EARTH_DIGITAL_BG, NULL);
        if (earth != NULL)
            lv_img_set_src(p_clk_earth_digital->earth_img, earth);
    }
    lv_obj_align(p_clk_earth_digital->earth_img, LV_ALIGN_BOTTOM_MID, 0, -30);

    /* AM/PM tag at the bottom-right corner of the minutes — small and
       half-transparent so it reads as a tag, not a digit. Shown only in
       12-hour mode; the redraw task fills it in / clears it based on
       hour_format. */
    p_clk_earth_digital->ampm_label = lv_label_create(p_clk_earth_digital->bg);
    lv_obj_set_style_text_font(p_clk_earth_digital->ampm_label,
                               LV_EXT_FONT_GET(get_system_font_size(-2)), 0);
    lv_obj_set_style_text_color(p_clk_earth_digital->ampm_label,
                                lv_color_white(), 0);
    lv_obj_set_style_text_opa(p_clk_earth_digital->ampm_label, LV_OPA_50, 0);
    lv_label_set_text(p_clk_earth_digital->ampm_label, "");
    /* Below the minute DIGIT, right edge aligned to the minute's right edge
       (-10 trims the digit image's transparent right margin). */
    lv_obj_align_to(p_clk_earth_digital->ampm_label,
                    p_clk_earth_digital->minute_1_img,
                    LV_ALIGN_OUT_BOTTOM_RIGHT, -10, 0);

    p_clk_earth_digital->redraw_task = NULL;

    return p_clk_earth_digital->bg;
}

static rt_int32_t init(lv_obj_t *parent)
{
    lv_earth_digital_layout_create(parent);
    return RT_EOK;
}

static rt_int32_t deinit(void)
{
    if (p_clk_earth_digital)
    {
        if (p_clk_earth_digital->redraw_task)
        {
            lv_timer_del(p_clk_earth_digital->redraw_task);
        }
        rt_free(p_clk_earth_digital);
        p_clk_earth_digital = NULL;
    }

    return RT_EOK;
}

static const app_clock_ops_t ops = {
    .init = init,
    .pause = pause_callback,
    .resume = resume_callback,
    .deinit = deinit,
};

void app_clock_earth_digital_register(void)
{
    app_clock_register("JW_wf1", &ops);
}
