/*
 * SPDX-FileCopyrightText: 2026 SiFli / project contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * MSH command shim for driving the PC simulator's synthetic touch indev.
 *
 * The actual indev (lv_touch_sim.c, under middleware/lvgl/lv_win32) is
 * registered alongside the real Win32 mouse pointer in lv_win32_init().
 * These commands push state into it from the FinSH thread; LVGL picks
 * the state up on its next read cycle.
 *
 * Build gate: PC simulator only (file is added to SConscript under
 * BSP_USING_PC_SIMULATOR).
 */
#include <rtthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "lv_touch_sim.h"

#define DBG_TAG "tsim.msh"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Minimum hold time for a tap and the inter-step delay for swipes.
 * LVGL polls indevs at ~30 Hz, so anything below ~35 ms risks the press
 * and release collapsing into a single read cycle. 60 ms gives margin. */
#define TSIM_TAP_HOLD_MS   60
#define TSIM_STEP_MIN_MS   20

static int touch_tap(int argc, char *argv[])
{
    if (argc < 3)
    {
        rt_kprintf("usage: touch_tap <x> <y>\n");
        return -1;
    }
    int x = atoi(argv[1]);
    int y = atoi(argv[2]);
    lv_touch_sim_set_state(x, y, true);
    rt_thread_mdelay(TSIM_TAP_HOLD_MS);
    lv_touch_sim_set_state(x, y, false);
    rt_kprintf("touch_tap (%d,%d)\n", x, y);
    return 0;
}
MSH_CMD_EXPORT(touch_tap, touch_tap x y - simulate tap at LVGL px);

static int touch_press(int argc, char *argv[])
{
    if (argc < 3)
    {
        rt_kprintf("usage: touch_press <x> <y>\n");
        return -1;
    }
    int x = atoi(argv[1]);
    int y = atoi(argv[2]);
    lv_touch_sim_set_state(x, y, true);
    rt_kprintf("touch_press (%d,%d) - sticky until touch_release\n", x, y);
    return 0;
}
MSH_CMD_EXPORT(touch_press, touch_press x y - hold a finger down at LVGL px);

static int touch_release(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    int x = 0, y = 0;
    bool pressed = false;
    if (!lv_touch_sim_get_state(&x, &y, &pressed))
    {
        rt_kprintf("touch_release: sim indev not initialized yet\n");
        return -1;
    }
    lv_touch_sim_set_state(x, y, false);
    rt_kprintf("touch_release at (%d,%d)\n", x, y);
    return 0;
}
MSH_CMD_EXPORT(touch_release, touch_release - lift the simulated finger);

static int touch_move(int argc, char *argv[])
{
    if (argc < 3)
    {
        rt_kprintf("usage: touch_move <x> <y>\n");
        return -1;
    }
    int x = atoi(argv[1]);
    int y = atoi(argv[2]);
    bool pressed = false;
    if (!lv_touch_sim_get_state(NULL, NULL, &pressed))
    {
        rt_kprintf("touch_move: sim indev not initialized yet\n");
        return -1;
    }
    /* Preserve the current pressed/released state — typical use is a
     * drag in progress (pressed=true) but moving a released cursor is
     * also valid (e.g. hover state, though LVGL ignores that). */
    lv_touch_sim_set_state(x, y, pressed);
    rt_kprintf("touch_move (%d,%d) pressed=%d\n", x, y, (int)pressed);
    return 0;
}
MSH_CMD_EXPORT(touch_move, touch_move x y - update touch point);

static int touch_swipe(int argc, char *argv[])
{
    if (argc < 5)
    {
        rt_kprintf("usage: touch_swipe <x1> <y1> <x2> <y2> [duration_ms]\n");
        return -1;
    }
    int x1 = atoi(argv[1]);
    int y1 = atoi(argv[2]);
    int x2 = atoi(argv[3]);
    int y2 = atoi(argv[4]);
    int dur = (argc >= 6) ? atoi(argv[5]) : 300;
    if (dur < 60) dur = 60;

    int steps = dur / TSIM_STEP_MIN_MS;
    if (steps < 3) steps = 3;
    int step_ms = dur / steps;

    /* Start: press at the origin and hold one tick so LVGL latches the
     * press event before any motion happens — without this, fast swipes
     * sometimes register as a release-only event with no anchor point. */
    lv_touch_sim_set_state(x1, y1, true);
    rt_thread_mdelay(step_ms);
    for (int i = 1; i <= steps; i++)
    {
        int cx = x1 + (x2 - x1) * i / steps;
        int cy = y1 + (y2 - y1) * i / steps;
        lv_touch_sim_set_state(cx, cy, true);
        rt_thread_mdelay(step_ms);
    }
    lv_touch_sim_set_state(x2, y2, false);
    rt_kprintf("touch_swipe (%d,%d)->(%d,%d) %dms %dsteps\n",
               x1, y1, x2, y2, dur, steps);
    return 0;
}
MSH_CMD_EXPORT(touch_swipe, touch_swipe x1 y1 x2 y2 [ms] - drag with hold);

static int touch_status(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    int x = 0, y = 0;
    bool pressed = false;
    if (!lv_touch_sim_get_state(&x, &y, &pressed))
    {
        rt_kprintf("touch_sim: not initialized\n");
        return -1;
    }
    rt_kprintf("touch_sim: x=%d y=%d pressed=%d\n", x, y, (int)pressed);
    return 0;
}
MSH_CMD_EXPORT(touch_status, touch_status - dump current sim touch state);
