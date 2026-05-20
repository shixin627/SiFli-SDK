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

#ifdef __cplusplus
}
#endif

#endif /* HID_MOUSE_H */
