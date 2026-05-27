/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "bloc_control.h"
#include "gui_app_fwk.h"
#include "ui_handler.h"
#include "bloc_notification.h"
#include "lv_ext_resource_manager.h"
#include "common_widget.h"
#include "ui_helper.h"
#include "ui_img_helper.h"
#include "lv_qrcode.h"
#include "app_mainmenu.h"
#include <stdio.h>
#include <string.h>

// 取得 BLE public address
typedef struct {
    uint8_t addr[6];
} bd_addr_t;
extern uint8_t ble_get_public_address(bd_addr_t * addr);

#define QR_URL_PREFIX "https://skaiwalk.com/download/id="

static void qrcode_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_CLICKED) {
        // 關閉 QRCode 畫面
        gui_app_exit(APP_ID_QRCODE);
    }
}

static void on_start(lv_obj_t *scr)
{
    // Step 1: 取得 BLE public address
    bd_addr_t addr;
    char code[20] = {0};
    if (ble_get_public_address(&addr) == 0) {
        // 轉成字串 (lowercase hex, 不補零)
        snprintf(code, sizeof(code), "%x-%x-%x-%x-%x-%x",
            addr.addr[0], addr.addr[1], addr.addr[2], addr.addr[3], addr.addr[4], addr.addr[5]);
    } else {
        strcpy(code, "0-0-0-0-0-0");
    }

    // Step 2: 組合 URL
    char url[128];
    snprintf(url, sizeof(url), "%s%s", QR_URL_PREFIX, code);

    // Step 3: 建立 QRCode widget
    lv_obj_t *bg = lv_obj_create(scr);
    lv_obj_set_size(bg, 466, 466);
    lv_obj_set_style_bg_color(bg, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);
    lv_obj_center(bg);
    lv_obj_add_event_cb(bg, qrcode_event_cb, LV_EVENT_CLICKED, NULL);

    /* White rounded card behind the QR — matches the device-page empty-state look
       (radius 16 / pad 12) and gives the QR a clean rounded quiet-zone. */
    lv_obj_t *card = lv_obj_create(bg);
    lv_obj_set_size(card, 224, 224);            /* 200 QR + 12 pad each side */
    lv_obj_set_style_bg_color(card, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 12, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_center(card);
    lv_obj_add_event_cb(card, qrcode_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *qrcode = lv_qrcode_create(card);
    lv_qrcode_setparam(qrcode, 200, lv_color_black(), lv_color_white());
    lv_qrcode_update(qrcode, url, strlen(url));
    lv_obj_center(qrcode);
    lv_obj_add_event_cb(qrcode, qrcode_event_cb, LV_EVENT_CLICKED, NULL);

    // Step 4: 顯示 BLE code 文字
    lv_obj_t *label = lv_label_create(bg);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -70);
    lv_obj_set_style_text_font(label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_label_set_text(label, code);
}

static void on_stop(void)
{
    // 清理資源 (如果有需要)
    
}

/**
 * @brief Message handler for app lifecycle events
 */
static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        /* app_run 直接開啟不經 Main 狀態機，左緣右滑返回 bar 仍隱藏，這裡補開 */
        extern void display_gesture_detect_objs(uint32_t idx, bool display);
        display_gesture_detect_objs(0, true);
        on_start(lv_scr_act());
        break;
    }

    case GUI_APP_MSG_ONRESUME:
        break;

    case GUI_APP_MSG_ONPAUSE:
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;

    default:
        break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_QRCODE, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(qr_code), ICON_QRCODE, APP_ID_QRCODE, app_main);