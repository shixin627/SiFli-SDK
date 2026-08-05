/* See popup_compat.h for why this exists. */
#include "popup_compat.h"

#include <string.h>

/* Marks a footer button so the queries below can tell one from any other
   object that might raise an event on the same popup. Value is the index. */
#define BTN_TAG_BASE 0x9000u

static void footer_btn_clicked(lv_event_t *e)
{
    lv_obj_t *btn  = lv_event_get_target(e);
    lv_obj_t *mbox = (lv_obj_t *)lv_event_get_user_data(e);
    uintptr_t tag  = (uintptr_t)lv_obj_get_user_data(btn);

    if (!mbox)
    {
        return;
    }

    /* First footer button is the confirm; anything after it dismisses. That is
       the order popup_confirm_create() adds them in, and it matches what
       lvsfpopup did with its confirm/cancel pair. */
    lv_obj_send_event(mbox,
                      (tag == BTN_TAG_BASE) ? LV_EVENT_READY : LV_EVENT_CANCEL,
                      NULL);
}

static lv_obj_t *add_footer_btn(lv_obj_t *mbox, const char *txt, uint16_t index)
{
    lv_obj_t *btn = lv_msgbox_add_footer_button(mbox, txt);

    if (btn)
    {
        lv_obj_set_user_data(btn, (void *)(uintptr_t)(BTN_TAG_BASE + index));
        lv_obj_add_event_cb(btn, footer_btn_clicked, LV_EVENT_CLICKED, mbox);
    }
    return btn;
}

lv_obj_t *popup_confirm_create(lv_obj_t *parent,
                               const void *icon,
                               const char *title,
                               const char *content,
                               const char *confirm_txt,
                               const char *cancel_txt)
{
    lv_obj_t *mbox = lv_msgbox_create(parent);
    uint16_t  n = 0;

    if (!mbox)
    {
        return NULL;
    }

    if (icon)    lv_msgbox_add_header_button(mbox, icon);
    if (title)   lv_msgbox_add_title(mbox, title);
    if (content) lv_msgbox_add_text(mbox, content);

    if (confirm_txt) add_footer_btn(mbox, confirm_txt, n++);
    if (cancel_txt)  add_footer_btn(mbox, cancel_txt, n++);

    return mbox;
}

static lv_obj_t *tagged_btn_of(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uintptr_t tag;

    if (!obj)
    {
        return NULL;
    }
    tag = (uintptr_t)lv_obj_get_user_data(obj);
    return (tag >= BTN_TAG_BASE) ? obj : NULL;
}

const char *popup_clicked_btn_text(lv_event_t *e)
{
    lv_obj_t *btn = tagged_btn_of(e);
    lv_obj_t *label;

    if (!btn || lv_obj_get_child_count(btn) == 0)
    {
        return NULL;
    }
    /* lv_msgbox_add_footer_button() puts a single label inside the button. */
    label = lv_obj_get_child(btn, 0);
    return label ? lv_label_get_text(label) : NULL;
}

uint16_t popup_clicked_btn_index(lv_event_t *e)
{
    lv_obj_t *btn = tagged_btn_of(e);

    if (!btn)
    {
        return 0xFFFFu;
    }
    return (uint16_t)((uintptr_t)lv_obj_get_user_data(btn) - BTN_TAG_BASE);
}
