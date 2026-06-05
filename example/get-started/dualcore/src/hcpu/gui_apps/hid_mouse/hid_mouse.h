/**
 ******************************************************************************
 * @file   hid_mouse.h
 * @brief  Mouse (trackpad / keyboard / media) component API.
 *
 *  T1 part 1 (host decouple): expose the mouse UI as a hostable component so
 *  device_pager can mount it as the per-device control layer WITHOUT going
 *  through gui_app_run(). The existing APP_ID_MOUSE gui_app path is unchanged
 *  — its msg_handler still calls hid_mouse_create(lv_scr_act()) on ONSTART and
 *  hid_mouse_destroy() on ONSTOP, so behavior is byte-identical for the
 *  standalone mouse app.
 ******************************************************************************
 */
#ifndef HID_MOUSE_H
#define HID_MOUSE_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Build the mouse UI (trackpad/keyboard/media + bottom bar + arc
 *        scroll) under @p host and activate it as the control surface.
 * @param host Parent LVGL object to build under (e.g. lv_scr_act() for the
 *             standalone app, or a device_pager tile for the hosted case).
 */
void hid_mouse_create(lv_obj_t *host);

/**
 * @brief Tear down the mouse UI and deactivate the control surface
 *        (mirrors the gui_app ONSTOP cleanup).
 */
void hid_mouse_destroy(void);

/**
 * @brief Split of hid_mouse_create for hosts that want the UI built ONCE and
 *        persisted (device_pager: trackpad visible mid-swipe) but the global
 *        "mouse mode" side effects toggled per page entry/leave:
 *          hid_mouse_build_ui  — pure UI (no global state); call once per host.
 *          hid_mouse_enter_mode — mouse-mode flag + status-bar gesture zones +
 *                                 device-change-bar hit-test + gesture detect.
 *          hid_mouse_exit_mode  — reverse of enter_mode, WITHOUT tearing the UI
 *                                 down (the persisted UI survives for next entry).
 *        hid_mouse_create == build_ui + enter_mode and hid_mouse_destroy still does
 *        exit_mode + full UI teardown, so the standalone APP_ID_MOUSE path is
 *        unchanged.
 */
void hid_mouse_build_ui(lv_obj_t *host);
void hid_mouse_enter_mode(void);
void hid_mouse_exit_mode(void);

/**
 * @brief The host object the (singleton) mouse UI is currently built on, or NULL
 *        if none. device_pager persists its hosted UI and checks this to detect the
 *        standalone APP_ID_MOUSE app having rebuilt the shared globals elsewhere
 *        (host != its tile) — in which case it cleans its tile and rebuilds.
 */
lv_obj_t *hid_mouse_ui_host(void);

/**
 * @brief Fade the trackpad scroll wheel (the gray tick nodes) in from the black
 *        backdrop, by ramping their COLOR (not opacity). For hosts (device_pager)
 *        that build the mouse at settle and want to soften the wheel's
 *        appearance. Call right after hid_mouse_create(). No-op safe if the
 *        wheel isn't built. Hardware-safe (no full-object layer opacity).
 */
void hid_mouse_fade_in_scroll_wheel(void);

/**
 * @brief Register a "return to host" callback (T4 (b)). When set, the mouse is
 *        in hosted mode: the bottom-bar up gesture invokes @p cb (device_pager
 *        slides the instruction layer back) instead of firing a multitask
 *        command. Pass NULL to clear (back to standalone behavior).
 *        The callback is invoked on the LVGL thread (input release handler).
 */
void hid_mouse_set_host_back_cb(void (*cb)(void));

/**
 * @brief Register a "host pull" callback for the hosted (device_pager) case.
 *        When set, dragging UP on the bottom bar is delegated to the host so it
 *        can finger-follow pull its panel back into view, instead of showing the
 *        mouse's own multitask hint. Called on the LVGL thread:
 *          phase 0 = dragging   (up_px = pixels dragged up so far)
 *          phase 1 = released   (host decides commit vs cancel from up_px)
 *        Takes precedence over the back cb. Pass NULL to clear.
 */
void hid_mouse_set_host_pull_cb(void (*cb)(int up_px, int released));

/**
 * @brief Mark the trackpad as hosted by device_pager (true) or standalone
 *        (false). While hosted, the pager owns the top + bottom screen edges
 *        (its device-name strip + bottom input bar), so the mouse page's own
 *        media-center pull-down is suppressed. Set from device_pager_set_active.
 */
void hid_mouse_set_hosted(bool hosted);

#ifdef __cplusplus
}
#endif

#endif /* HID_MOUSE_H */
