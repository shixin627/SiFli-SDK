/**
 * @file app_setting_device_list.c
 * @brief Bonded BLE devices list UI
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "custom_trans_anim.h"
#include "ui_handler.h"
#include "ble_device_manager.h"

#define DBG_TAG "app.setting.devlist"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_SETTING

// Forward declarations
extern void ble_app_clear_bonded_device(void);
extern void ble_app_advertising_start(bool mouse_mode, bool pairing_mode);
extern void ble_app_start_targeted_advertising(uint8_t device_idx);

// UI state
typedef struct
{
    lv_obj_t *page; // Main page container
    lv_obj_t *list;
    lv_obj_t *empty_label;
    lv_obj_t *add_btn;
    lv_obj_t *add_btn_label; // Label for add button
    bool is_pairing_mode;    // Track pairing mode state
} device_list_ui_t;

static device_list_ui_t g_dev_list_ui = {0};
static void device_item_click_callback(lv_event_t *e);

/**
 * @brief Update add button appearance based on pairing mode state
 */
static void update_add_button_appearance(void)
{
    if (!g_dev_list_ui.add_btn || !g_dev_list_ui.add_btn_label)
    {
        return;
    }

    if (g_dev_list_ui.is_pairing_mode)
    {
        // Pairing mode active - show as green "Stop Pairing" button
        lv_obj_set_style_bg_color(g_dev_list_ui.add_btn, lv_color_hex(0x00AA00), 0);
        lv_obj_set_style_bg_color(g_dev_list_ui.add_btn, lv_color_hex(0x008800), LV_STATE_PRESSED);
        lv_label_set_text(g_dev_list_ui.add_btn_label, LV_SYMBOL_CLOSE " Stop Pairing");
    }
    else
    {
        // Normal mode - show as blue "Add New Device" button
        lv_obj_set_style_bg_color(g_dev_list_ui.add_btn, lv_color_hex(0x0066CC), 0);
        lv_obj_set_style_bg_color(g_dev_list_ui.add_btn, lv_color_hex(0x0055AA), LV_STATE_PRESSED);
        lv_label_set_text(g_dev_list_ui.add_btn_label, LV_SYMBOL_PLUS " Add New Device");
    }
}

/**
 * @brief Get device type icon name
 */
static const char *get_device_type_icon(uint8_t device_type)
{
    switch (device_type)
    {
    case DEVICE_TYPE_PHONE:
        return "phone"; // TODO: Use actual icon resource
    case DEVICE_TYPE_COMPUTER:
        return "computer";
    case DEVICE_TYPE_TABLET:
        return "tablet";
    default:
        return "bluetooth";
    }
}

/**
 * @brief Get device type name
 */
static const char *get_device_type_name(uint8_t device_type)
{
    switch (device_type)
    {
    case DEVICE_TYPE_PHONE:
        return "Phone";
    case DEVICE_TYPE_COMPUTER:
        return "Computer";
    case DEVICE_TYPE_TABLET:
        return "Tablet";
    default:
        return "Device";
    }
}

/**
 * @brief Create a device list item
 */
static lv_obj_t *create_device_list_item(lv_obj_t *parent, const bonded_device_t *device,
                                         uint8_t device_idx, bool show_arrow)
{
    // Get active device index to check if this is the active device
    int active_idx = ble_dev_mgr_get_active_device();
    bool is_active = (active_idx == device_idx);

    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, LV_PCT(100), 120); // Increased from 85 to 105 for two labels
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x3A3A3A), LV_STATE_PRESSED);
    lv_obj_set_style_pad_all(btn, 8, 0);

    // Add blue border for active device
    if (is_active)
    {
        lv_obj_set_style_border_width(btn, 3, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x00AAFF), 0); // Bright blue
        lv_obj_set_style_border_opa(btn, LV_OPA_COVER, 0);
    }
    else
    {
        lv_obj_set_style_border_width(btn, 0, 0);
    }

    // Store device index as user data
    lv_obj_set_user_data(btn, (void *)(uintptr_t)device_idx);

    // Main container
    lv_obj_t *cont = lv_obj_create(btn);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(cont, LV_OPA_0, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 0, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_CLICKABLE); // Allow clicks to pass through

    // Icon (left)
    lv_obj_t *icon = lv_img_create(cont);
    // TODO: Use actual icons based on device type
    // lv_img_set_src(icon, LV_EXT_IMG_GET(get_device_type_icon(device->device_type)));
    lv_obj_set_size(icon, 40, 40);
    lv_obj_set_style_img_recolor(icon, lv_color_hex(0x00AAFF), 0);
    lv_obj_set_style_img_recolor_opa(icon, LV_OPA_70, 0);
    lv_obj_clear_flag(icon, LV_OBJ_FLAG_CLICKABLE); // Allow clicks to pass through

    // Text container (middle)
    lv_obj_t *text_cont = lv_obj_create(cont);
    lv_obj_set_size(text_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(text_cont, LV_OPA_0, 0);
    lv_obj_set_style_border_width(text_cont, 0, 0);
    lv_obj_set_style_pad_left(text_cont, 12, 0); // Increased from 10
    lv_obj_set_style_pad_right(text_cont, 0, 0);
    lv_obj_set_style_pad_top(text_cont, 0, 0);
    lv_obj_set_style_pad_bottom(text_cont, 0, 0);
    lv_obj_set_flex_flow(text_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(text_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(text_cont, 5, 0); // Add spacing between labels
    lv_obj_set_flex_grow(text_cont, 1);
    lv_obj_clear_flag(text_cont, LV_OBJ_FLAG_CLICKABLE); // Allow clicks to pass through

    // Device name
    lv_obj_t *name_label = lv_label_create(text_cont);
    lv_label_set_text(name_label, device->device_name);
    lv_obj_set_style_text_color(name_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(name_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_clear_flag(name_label, LV_OBJ_FLAG_CLICKABLE); // Allow clicks to pass through

    // Status text (Connected/Disconnected)
    lv_obj_t *status_label = lv_label_create(text_cont);

    if (device->conn_idx != 0xFF)
    {
        lv_label_set_text(status_label, "Connected");
        lv_obj_set_style_text_color(status_label, lv_color_hex(0x00FF00), 0); // Green for connected
    }
    else
    {
        lv_label_set_text(status_label, "Not Connected");
        lv_obj_set_style_text_color(status_label, lv_color_hex(0x888888), 0); // Gray for disconnected
    }
    lv_obj_set_style_text_font(status_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_clear_flag(status_label, LV_OBJ_FLAG_CLICKABLE); // Allow clicks to pass through

    // Arrow icon (right) - show for all devices
    if (show_arrow)
    {
        lv_obj_t *arrow = lv_label_create(cont);
        lv_label_set_text(arrow, LV_SYMBOL_RIGHT);
        lv_obj_set_style_text_color(arrow, lv_color_hex(0x888888), 0);
        lv_obj_clear_flag(arrow, LV_OBJ_FLAG_CLICKABLE); // Allow clicks to pass through
    }

    return btn;
}

/**
 * @brief Refresh device list UI
 */
static void refresh_device_list(void)
{
    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    if (!db)
    {
        return;
    }

    // Clear existing list
    if (g_dev_list_ui.list)
    {
        lv_obj_clean(g_dev_list_ui.list);
    }

    // Show/hide empty label
    if (db->count == 0)
    {
        if (g_dev_list_ui.empty_label)
        {
            lv_obj_clear_flag(g_dev_list_ui.empty_label, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }
    else
    {
        if (g_dev_list_ui.empty_label)
        {
            lv_obj_add_flag(g_dev_list_ui.empty_label, LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Create list items for each bonded device
    for (int i = 0; i < MAX_BONDED_DEVICES; i++)
    {
        if (db->devices[i].is_valid)
        {
            lv_obj_t *item = create_device_list_item(g_dev_list_ui.list,
                                                     &db->devices[i], i, true);
            // Add click event handlers
            lv_obj_add_event_cb(item, device_item_click_callback, LV_EVENT_SHORT_CLICKED, NULL);
            lv_obj_add_event_cb(item, device_item_click_callback, LV_EVENT_LONG_PRESSED, NULL);
        }
    }
}

/**
 * @brief Device item click callback - Toggle connection
 */
static void device_item_click_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);
    uint8_t device_idx = (uint8_t)(uintptr_t)lv_obj_get_user_data(btn);

    const bonded_devices_db_t *db = ble_dev_mgr_get_database();
    if (!db || device_idx >= MAX_BONDED_DEVICES)
    {
        return;
    }

    const bonded_device_t *dev = &db->devices[device_idx];
    if (!dev->is_valid)
    {
        return;
    }

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        LOG_D("Device item clicked: idx=%d, name=%s", device_idx, dev->device_name);
        // Short click: Switch to target device and start advertising
        // if (dev->conn_idx != 0xFF)
        // {
        //     // Device is already connected, set it as active
        //     LOG_I("Device [%d] already connected, setting as active: %s", device_idx, dev->device_name);
        //     ble_dev_mgr_set_active_device(device_idx);
        // }
        // else
        // {
        //     // Device is not connected, start targeted advertising
        //     LOG_I("Starting targeted advertising for device [%d]: %s", device_idx, dev->device_name);
        //     ble_app_start_targeted_advertising(device_idx);
        // }
        ble_dev_mgr_connect_device(device_idx);
    }
    else if (LV_EVENT_LONG_PRESSED == event)
    {
        LOG_D("Device item long pressed: idx=%d, name=%s", device_idx, dev->device_name);
        // Long press: Set as active device (for audio output)
        if (dev->conn_idx != 0xFF)
        {
            // Only allow setting active device if it's connected
            LOG_I("User requested to set device [%d] as active: %s", device_idx, dev->device_name);

            int ret = ble_dev_mgr_set_active_device(device_idx);
            if (ret == 0)
            {
                LOG_I("Successfully set device [%d] as active", device_idx);
                // UI will refresh automatically via event callback
            }
            else
            {
                LOG_E("Failed to set active device, error: %d", ret);
            }
        }
        else
        {
            LOG_W("Cannot set disconnected device as active");
        }
    }
}

/**
 * @brief Device manager event callback
 */
static void dev_mgr_event_cb(dev_mgr_event_t event, uint8_t device_idx, void *user_data)
{
    LOG_I("Device manager event: %d, device_idx: %d", event, device_idx);

    // Refresh UI on any database change
    refresh_device_list();
}

/**
 * @brief Quick switch active device button callback
 */
static void quick_switch_callback(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_SHORT_CLICKED)
    {
        int new_active = ble_dev_mgr_switch_to_next_device();
        if (new_active >= 0)
        {
            LOG_I("Quick switched to device [%d]", new_active);
        }
    }
}

/**
 * @brief Pairing mode switch callback
 */
static void pairing_switch_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_VALUE_CHANGED == event)
    {
        lv_obj_t *sw = lv_event_get_target(e);
        bool is_on = lv_obj_has_state(sw, LV_STATE_CHECKED);

        g_dev_list_ui.is_pairing_mode = is_on;

        // if (is_on)
        // {
        //     LOG_I("Pairing mode enabled - open advertising");
        //     // Restart advertising in pairing mode to allow all devices to discover the watch
        //     ble_app_advertising_start(SkaiWatchSys.gap_conn_state == GAP_CONN_STATE_DISCONNECTED, true, false);
        // }
        // else
        // {
        //     LOG_I("Pairing mode disabled - whitelist advertising");
        //     // Restart advertising in whitelist mode (only paired devices can connect)
        //     ble_app_advertising_start(SkaiWatchSys.gap_conn_state == GAP_CONN_STATE_DISCONNECTED, false, false);
        // }
    }
}

extern void generate_random_public_address(uint8_t device_id);
static void pairing_btn_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        if (g_dev_list_ui.is_pairing_mode)
        {
            generate_random_public_address(0);
        }
        ble_app_advertising_start(g_dev_list_ui.is_pairing_mode, false);
    }
}

/**
 * @brief Clear all devices button callback
 */
static void btn_clear_all_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_SHORT_CLICKED == event)
    {
        LOG_I("Clearing all bonded devices...");
        ble_dev_mgr_clear_all();
        refresh_device_list();
    }
}

/**
 * @brief Back button callback
 */
static void back_btn_event_callback(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);
    if (LV_EVENT_SHORT_CLICKED == event)
    {
        gui_app_goback();
    }
}

static void on_start(void)
{
    // Header
    lv_obj_t *title = lv_lvsfheader_create(lv_scr_act());
    lv_lvsfheader_set_title(title, "Bluetooth Devices");
    lv_lvsfheader_set_visible_item(title, LVSF_HEADER_BRANCH);
    lv_lvsfheader_back_event_cb(title, back_btn_event_callback);

    // Main container
    lv_obj_t *page = lv_obj_create(lv_scr_act());
    g_dev_list_ui.page = page; // Save page reference for pairing overlay
    lv_obj_set_size(page, LV_HOR_RES, LV_VER_RES - lv_obj_get_height(title) - 10);
    lv_obj_align_to(page, title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    lv_obj_set_style_bg_color(page, lv_color_hex(0x121212), 0);
    lv_obj_set_style_pad_all(page, 15, 0);
    lv_obj_set_flex_flow(page, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(page, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Instruction label
    lv_obj_t *hint_label = lv_label_create(page);
    lv_label_set_text(hint_label, "Tap to connect/disconnect\nLong press to set as active");
    lv_obj_set_style_text_color(hint_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_align(hint_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(hint_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    // Quick switch button (only visible when multiple devices connected)
    lv_obj_t *switch_btn = lv_btn_create(page);
    lv_obj_set_size(switch_btn, LV_PCT(90), 45);
    lv_obj_set_style_radius(switch_btn, 10, 0);
    lv_obj_set_style_bg_color(switch_btn, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_bg_color(switch_btn, lv_color_hex(0x3A3A3A), LV_STATE_PRESSED);

    lv_obj_t *switch_label = lv_label_create(switch_btn);
    lv_label_set_text(switch_label, LV_SYMBOL_SHUFFLE " Switch Active Device");
    lv_obj_set_style_text_color(switch_label, lv_color_hex(0x00AAFF), 0);
    lv_obj_center(switch_label);
    lv_obj_add_event_cb(switch_btn, quick_switch_callback, LV_EVENT_SHORT_CLICKED, NULL);

    // Hide switch button if less than 2 devices connected
    int connected_count = ble_dev_mgr_get_connected_count();
    if (connected_count < 2)
    {
        lv_obj_add_flag(switch_btn, LV_OBJ_FLAG_HIDDEN);
    }

    // Device list container
    lv_obj_t *list = lv_obj_create(page);
    lv_obj_set_size(list, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(list, LV_OPA_0, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_all(list, 0, 0);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list, 10, 0);
    g_dev_list_ui.list = list;

    // Empty state label
    lv_obj_t *empty_label = lv_label_create(page);
    lv_label_set_text(empty_label, "No paired devices\n\nTap \"Add Device\" to pair");
    lv_obj_set_style_text_color(empty_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_align(empty_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_flag(empty_label, LV_OBJ_FLAG_HIDDEN); // Initially hidden
    g_dev_list_ui.empty_label = empty_label;

    // Pairing mode switch container
    lv_obj_t *pairing_container = lv_obj_create(page);
    lv_obj_set_size(pairing_container, LV_PCT(90), 70);
    lv_obj_set_style_radius(pairing_container, 10, 0);
    lv_obj_set_style_bg_color(pairing_container, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_border_width(pairing_container, 0, 0);
    lv_obj_set_style_pad_all(pairing_container, 15, 0);
    lv_obj_set_flex_flow(pairing_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(pairing_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Left side - label and description
    lv_obj_t *label_container = lv_obj_create(pairing_container);
    lv_obj_set_size(label_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(label_container, LV_OPA_0, 0);
    lv_obj_set_style_border_width(label_container, 0, 0);
    lv_obj_set_style_pad_all(label_container, 0, 0);
    lv_obj_set_flex_flow(label_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(label_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t *pairing_label = lv_label_create(label_container);
    lv_label_set_text(pairing_label, "Pairing Mode");
    lv_obj_set_style_text_color(pairing_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(pairing_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    // Right side - switch control
    lv_obj_t *pairing_switch = lv_switch_create(pairing_container);
    lv_obj_set_size(pairing_switch, 100, 50);
    lv_obj_add_event_cb(pairing_switch, pairing_switch_event_callback, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *pairing_btn = lv_btn_create(page);
    lv_obj_set_size(pairing_btn, LV_PCT(90), 50);
    lv_obj_set_style_radius(pairing_btn, 10, 0);
    lv_obj_set_style_bg_color(pairing_btn, lv_color_hex(0x0066CC), 0);
    lv_obj_set_style_bg_color(pairing_btn, lv_color_hex(0x0055AA), LV_STATE_PRESSED);
    lv_obj_add_event_cb(pairing_btn, pairing_btn_event_callback, LV_EVENT_SHORT_CLICKED, NULL);

    lv_obj_t *pairing_label_btn = lv_label_create(pairing_btn);
    lv_label_set_text(pairing_label_btn, LV_SYMBOL_PLUS "reset BLE");
    lv_obj_set_style_text_color(pairing_label_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(pairing_label_btn);

    // Store switch reference for global control
    g_dev_list_ui.add_btn = pairing_switch;
    g_dev_list_ui.add_btn_label = pairing_label;

    // Clear all button
    lv_obj_t *clear_btn = lv_btn_create(page);
    lv_obj_set_size(clear_btn, LV_PCT(90), 50);
    lv_obj_set_style_radius(clear_btn, 10, 0);
    lv_obj_set_style_bg_color(clear_btn, lv_color_hex(0xCC0000), 0);
    lv_obj_set_style_bg_color(clear_btn, lv_color_hex(0xAA0000), LV_STATE_PRESSED);
    lv_obj_add_event_cb(clear_btn, btn_clear_all_event_callback, LV_EVENT_SHORT_CLICKED, NULL);

    lv_obj_t *clear_label = lv_label_create(clear_btn);
    lv_label_set_text(clear_label, LV_SYMBOL_TRASH " Clear All Devices");
    lv_obj_set_style_text_color(clear_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(clear_label);

    // Register device manager event callback
    ble_dev_mgr_register_callback(dev_mgr_event_cb, NULL);

    // Initial list population
    refresh_device_list();

    cust_trans_anim_config(CUST_ANIM_TYPE_0, NULL);
}

static void on_resume(void)
{
    // Refresh list when returning to this screen
    refresh_device_list();
}

static void on_pause(void)
{
}

static void on_stop(void)
{
    // Clean up UI state
    memset(&g_dev_list_ui, 0, sizeof(device_list_ui_t));
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
        break;

    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

void app_setting_device_list_main(void)
{
    gui_app_create_page("ble_devices", msg_handler);
}

#endif // APP_ID_SETTING
