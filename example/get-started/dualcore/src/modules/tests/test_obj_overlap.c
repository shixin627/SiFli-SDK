/*
 * SPDX-FileCopyrightText: 2026 SiFli / project contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * MSH command `ui_overlap`: list every LVGL object whose box intersects a
 * screen rectangle, in paint order (screen, then lv_layer_top, then
 * lv_layer_sys; parents before children, earlier siblings first). Whatever is
 * listed AFTER an object paints on top of it. Hidden objects are listed too
 * (marked H): they are the ones that can cover the rectangle the moment
 * something clears their HIDDEN flag.
 *
 * Default rectangle = the watch face's first hour digit container
 * (app_clock_earth_digita.c: 100x136 at CENTER -153,-10 on a 466 panel).
 *
 * Read-only walk from the FinSH thread; PC simulator only.
 */
#include <rtthread.h>
#include <stdlib.h>
#include "lvgl.h"

static int s_seq;

static void walk(lv_obj_t *o, int depth, bool hidden, const lv_area_t *rect,
                 const lv_area_t *want)
{
    lv_area_t a, hit;
    lv_obj_get_coords(o, &a);
    bool h = hidden || lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN);

    s_seq++;
    if (_lv_area_intersect(&hit, &a, rect))
    {
        const char *cls = lv_obj_check_type(o, &lv_img_class)     ? "img"
                          : lv_obj_check_type(o, &lv_label_class) ? "label"
                                                                  : "obj";
        bool exact = a.x1 == want->x1 && a.y1 == want->y1 && a.x2 == want->x2 &&
                     a.y2 == want->y2;
        rt_kprintf("%3d %*s%-5s %p (%d,%d)-(%d,%d) bg_opa=%d opa=%d%s%s%s%s\n",
                   s_seq, depth * 2, "", cls, o, a.x1, a.y1, a.x2, a.y2,
                   lv_obj_get_style_bg_opa(o, LV_PART_MAIN),
                   lv_obj_get_style_opa(o, LV_PART_MAIN),
                   h ? " H" : "",
                   lv_obj_has_flag(o, LV_OBJ_FLAG_CLICKABLE) ? " click" : "",
                   lv_obj_has_flag(o, LV_OBJ_FLAG_OVERFLOW_VISIBLE) ? " ovf" : "",
                   exact ? "  <== digit box" : "");
    }

    /* Children are clipped to the parent unless it lets them overflow. */
    lv_area_t child_rect = *rect;
    if (!lv_obj_has_flag(o, LV_OBJ_FLAG_OVERFLOW_VISIBLE) &&
        !_lv_area_intersect(&child_rect, rect, &a))
    {
        /* Nothing of this subtree can reach the rectangle; still count the
           sequence numbers so the printed order stays comparable. */
        child_rect = *rect;
        child_rect.x2 = child_rect.x1 - 1;
    }
    for (uint32_t i = 0; i < lv_obj_get_child_cnt(o); i++)
    {
        walk(lv_obj_get_child(o, i), depth + 1, h, &child_rect, want);
    }
}

static int ui_overlap(int argc, char *argv[])
{
    lv_area_t r = {30, 155, 129, 290};
    if (argc >= 5)
    {
        r.x1 = atoi(argv[1]);
        r.y1 = atoi(argv[2]);
        r.x2 = atoi(argv[3]);
        r.y2 = atoi(argv[4]);
    }
    rt_kprintf("ui_overlap rect (%d,%d)-(%d,%d)  usage: ui_overlap [x1 y1 x2 y2]\n",
               r.x1, r.y1, r.x2, r.y2);

    lv_obj_t *layers[3] = {lv_scr_act(), lv_layer_top(), lv_layer_sys()};
    const char *names[3] = {"screen", "layer_top", "layer_sys"};
    s_seq = 0;
    for (int i = 0; i < 3; i++)
    {
        rt_kprintf("--- %s ---\n", names[i]);
        walk(layers[i], 0, false, &r, &r);
    }
    rt_kprintf("--- end ---\n");
    return 0;
}
MSH_CMD_EXPORT(ui_overlap, ui_overlap [x1 y1 x2 y2] - list LVGL objects over a screen rect);
