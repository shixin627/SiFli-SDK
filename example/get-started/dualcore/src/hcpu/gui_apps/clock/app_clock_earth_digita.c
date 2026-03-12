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
#include "ui_helper.h"

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
} app_clock_earth_digital_t;

static app_clock_earth_digital_t *p_clk_earth_digital = NULL;

static void app_clock_earth_digital_redraw(lv_timer_t *task)
{
    app_clock_time_t current_time;
    rt_uint8_t hours, minutes, seconds;
    static char time_buf[8];
    static char date_buf[32];

    app_clock_main_get_current_time(&current_time);
    hours = current_time.h;
    minutes = current_time.m;
    seconds = current_time.s;

    // Update hour display with images
    int hour_0 = hours / 10;
    int hour_1 = hours % 10;
    int min_0 = minutes / 10;
    int min_1 = minutes % 10;

    // 指標陣列對應數字圖片資源
    const lv_img_dsc_t *dig_img[10] = {
        &dig_1_00, &dig_1_01, &dig_1_02, &dig_1_03, &dig_1_04,
        &dig_1_05, &dig_1_06, &dig_1_07, &dig_1_08, &dig_1_09};

    lv_img_set_src(p_clk_earth_digital->hour_0_img, dig_img[hour_0]);
    lv_img_set_src(p_clk_earth_digital->hour_1_img, dig_img[hour_1]);
    lv_img_set_src(p_clk_earth_digital->minute_0_img, dig_img[min_0]);
    lv_img_set_src(p_clk_earth_digital->minute_1_img, dig_img[min_1]);

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

// #define CHECK_CLOCK
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
    lv_obj_align(h_0, LV_ALIGN_CENTER, -153, -65);
    lv_obj_set_style_bg_color(h_0, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(h_0, 0, 0);

    // Create hour label
    p_clk_earth_digital->hour_0_img = lv_img_create(h_0);
    lv_img_set_src(p_clk_earth_digital->hour_0_img, &dig_1_00);
    lv_img_set_zoom(p_clk_earth_digital->hour_0_img, 256 * 0.9); // 100%
    lv_obj_center(p_clk_earth_digital->hour_0_img);

    lv_obj_t *h_1 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(h_1, 100, 136);
    lv_obj_align(h_1, LV_ALIGN_CENTER, -52, -65);
    lv_obj_set_style_bg_color(h_1, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(h_1, 0, 0);

    p_clk_earth_digital->hour_1_img = lv_img_create(h_1);
    lv_img_set_src(p_clk_earth_digital->hour_1_img, &dig_1_00);
    lv_img_set_zoom(p_clk_earth_digital->hour_1_img, 256 * 0.9); // 100%
    lv_obj_center(p_clk_earth_digital->hour_1_img);

    lv_obj_t *m_0 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(m_0, 100, 136);
    lv_obj_align(m_0, LV_ALIGN_CENTER, 52, -65);
    lv_obj_set_style_bg_color(m_0, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(m_0, 0, 0);

    p_clk_earth_digital->minute_0_img = lv_img_create(m_0);
    lv_img_set_src(p_clk_earth_digital->minute_0_img, &dig_1_00);
    lv_obj_set_style_img_opa(p_clk_earth_digital->minute_0_img, LV_OPA_70, 0);
    lv_img_set_zoom(p_clk_earth_digital->minute_0_img, 256 * 0.9); // 100%
    lv_obj_center(p_clk_earth_digital->minute_0_img);

    lv_obj_t *m_1 = lv_obj_create(p_clk_earth_digital->bg);
    lv_obj_set_size(m_1, 100, 136);
    lv_obj_align(m_1, LV_ALIGN_CENTER, 153, -65);
    lv_obj_set_style_bg_color(m_1, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(m_1, 0, 0);

    p_clk_earth_digital->minute_1_img = lv_img_create(m_1);
    lv_img_set_src(p_clk_earth_digital->minute_1_img, &dig_1_00);
    lv_obj_set_style_img_opa(p_clk_earth_digital->minute_1_img, LV_OPA_70, 0);
    lv_img_set_zoom(p_clk_earth_digital->minute_1_img, 256 * 0.9); // 100%
    lv_obj_center(p_clk_earth_digital->minute_1_img);

#ifdef CHECK_CLOCK
    lv_obj_set_style_border_width(h_0, 1, 0);
    lv_obj_set_style_border_color(h_0, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(h_1, 1, 0);
    lv_obj_set_style_border_color(h_1, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(m_0, 1, 0);
    lv_obj_set_style_border_color(m_0, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(m_1, 1, 0);
    lv_obj_set_style_border_color(m_1, lv_color_hex(0xFFFFFF), 0);
#else
    lv_obj_set_style_bg_opa(h_0, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(h_1, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(m_0, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(m_1, LV_OPA_TRANSP, 0);
#endif

    p_clk_earth_digital->earth_img = lv_img_create(p_clk_earth_digital->bg);
    lv_img_set_src(p_clk_earth_digital->earth_img, &img_earth_digital_bg);
    lv_obj_align(p_clk_earth_digital->earth_img, LV_ALIGN_BOTTOM_MID, 0, -5);

    p_clk_earth_digital->redraw_task = NULL;

    return p_clk_earth_digital->bg;
}

#define EXPAND_ANIM_DURATION 300
#define EXPAND_CONT_Y_OFFSET 35   // 容器下移量 (-65 -> -40)
#define EXPAND_EARTH_Y_OFFSET 90  // earth 下移量 (-5 -> 40)

static void _anim_zoom_cb(void *var, int32_t v)
{
    lv_img_set_zoom((lv_obj_t *)var, (uint16_t)v);
}

static void _anim_translate_y_cb(void *var, int32_t v)
{
    lv_obj_set_style_translate_y((lv_obj_t *)var, v, 0);
}

extern void set_dial_widget_opa(uint8_t opa);
static void _anim_dial_widget_cb(void *var, int32_t v)
{
    // 同時處理 translate_y 和透明度
    lv_obj_set_style_translate_y((lv_obj_t *)var, v, 0);
    // v 從 0 到 EXPAND_EARTH_Y_OFFSET，映射 opa 從 255 到 0
    uint8_t opa = (uint8_t)(255 - (255 * v / EXPAND_EARTH_Y_OFFSET));
    set_dial_widget_opa(opa);
}

void app_clock_earth_digital_set_expanded(bool expanded)
{
    if (!p_clk_earth_digital) return;

    lv_obj_t *digits[4] = {
        p_clk_earth_digital->hour_0_img,
        p_clk_earth_digital->hour_1_img,
        p_clk_earth_digital->minute_0_img,
        p_clk_earth_digital->minute_1_img,
    };

    // 數字圖片縮放動畫
    for (int i = 0; i < 4; i++)
    {
        int32_t cur_zoom = lv_img_get_zoom(digits[i]);
        int32_t tgt_zoom = expanded ? 256 : (int32_t)(256 * 0.9);

        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, digits[i]);
        lv_anim_set_values(&a, cur_zoom, tgt_zoom);
        lv_anim_set_time(&a, EXPAND_ANIM_DURATION);
        lv_anim_set_exec_cb(&a, _anim_zoom_cb);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
        lv_anim_start(&a);
    }

    // 數字容器 translate_y 動畫 (不影響原始 alignment)
    for (int i = 0; i < 4; i++)
    {
        lv_obj_t *cont = lv_obj_get_parent(digits[i]);
        int32_t cur_ty = lv_obj_get_style_translate_y(cont, 0);
        int32_t tgt_ty = expanded ? EXPAND_CONT_Y_OFFSET : 0;

        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, cont);
        lv_anim_set_values(&a, cur_ty, tgt_ty);
        lv_anim_set_time(&a, EXPAND_ANIM_DURATION);
        lv_anim_set_exec_cb(&a, _anim_translate_y_cb);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
        lv_anim_start(&a);
    }

    // earth_img translate_y 動畫
    {
        int32_t cur_ty = lv_obj_get_style_translate_y(p_clk_earth_digital->earth_img, 0);
        int32_t tgt_ty = expanded ? EXPAND_EARTH_Y_OFFSET : 0;

        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, p_clk_earth_digital->earth_img);
        lv_anim_set_values(&a, cur_ty, tgt_ty);
        lv_anim_set_time(&a, EXPAND_ANIM_DURATION);
        lv_anim_set_exec_cb(&a, _anim_translate_y_cb);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
        lv_anim_start(&a);
    }

    // dial_widget 跟著 earth_img 同步移動 + 透明度變化
    {
        extern lv_obj_t *app_clock_main_get_dial_widget(void);
        lv_obj_t *dw = app_clock_main_get_dial_widget();
        if (dw && lv_obj_is_valid(dw))
        {
            int32_t cur_ty = lv_obj_get_style_translate_y(dw, 0);
            int32_t tgt_ty = expanded ? EXPAND_EARTH_Y_OFFSET : 0;

            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, dw);
            lv_anim_set_values(&a, cur_ty, tgt_ty);
            lv_anim_set_time(&a, EXPAND_ANIM_DURATION);
            lv_anim_set_exec_cb(&a, _anim_dial_widget_cb);
            lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
            lv_anim_start(&a);
        }
    }
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
