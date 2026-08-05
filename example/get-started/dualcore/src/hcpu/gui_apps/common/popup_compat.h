/*
 * The lvsfpopup / msgbox-active-button shapes, rebuilt on LVGL v9's msgbox.
 *
 * v8 got lv_lvsfpopup_* from the closed gui_widgets library, which has no v9
 * build, and lv_msgbox_get_active_btn*() was dropped when v9 turned msgbox
 * footer buttons into ordinary objects you attach handlers to.
 *
 * Rather than restructure five call sites around the new object model, this
 * keeps their shape: the popup still raises LV_EVENT_READY for confirm and
 * LV_EVENT_CANCEL for dismiss on the popup object itself, and the active-button
 * queries still answer from inside a click handler.
 *
 * ponytail: a shim, not a widget. It owns no state and draws nothing -- it
 * wires v9 buttons to the events the existing handlers already understand.
 */
#ifndef POPUP_COMPAT_H
#define POPUP_COMPAT_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Confirm/cancel popup. `icon`, `confirm_txt` and `cancel_txt` may be NULL to
   omit that part. The returned object raises LV_EVENT_READY / LV_EVENT_CANCEL,
   so an existing lvsfpopup handler attaches to it unchanged. */
lv_obj_t *popup_confirm_create(lv_obj_t *parent,
                               const void *icon,
                               const char *title,
                               const char *content,
                               const char *confirm_txt,
                               const char *cancel_txt);

/* Label text of the footer button that raised `e`, or NULL if `e` did not come
   from one. Replaces lv_msgbox_get_active_btn_text(). */
const char *popup_clicked_btn_text(lv_event_t *e);

/* Index of the footer button that raised `e` (0-based, left to right), or
   0xFFFF if `e` did not come from one. Replaces lv_msgbox_get_active_btn(). */
uint16_t popup_clicked_btn_index(lv_event_t *e);

#ifdef __cplusplus
}
#endif

#endif /* POPUP_COMPAT_H */
