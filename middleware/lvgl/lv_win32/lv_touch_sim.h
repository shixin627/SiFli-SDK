/*
 * SPDX-FileCopyrightText: 2026 SiFli / project contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Scripted touch input device for the PC simulator.
 *
 * Registers a second LVGL pointer indev that runs alongside the real
 * Win32 mouse indev. Synthetic events pushed via lv_touch_sim_set_state()
 * are visible to LVGL on the next read cycle (~33 ms), so the real mouse
 * keeps working concurrently — last-touched indev wins by LVGL convention.
 *
 * Intended driver: shell commands in dualcore tests (touch_tap, touch_press,
 * touch_release, touch_move, touch_swipe). State is mutex-protected because
 * write comes from the FinSH thread and read from the LVGL timer thread.
 */
#ifndef LV_TOUCH_SIM_H
#define LV_TOUCH_SIM_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Called from lv_win32_init() after the real indevs are registered. Idempotent. */
void lv_touch_sim_init(void);

/* Set the synthetic pointer state. (x,y) are LVGL pixel coords. */
void lv_touch_sim_set_state(int x, int y, bool pressed);

/* Read current state. Returns false if init hasn't run yet. */
bool lv_touch_sim_get_state(int *x, int *y, bool *pressed);

#ifdef __cplusplus
}
#endif

#endif /* LV_TOUCH_SIM_H */
