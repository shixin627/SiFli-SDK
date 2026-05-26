#include "ble_hid.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "bf0_hal_hlp.h"
#include "bf0_sibles.h"
#include "bf0_ble_gap.h"
#include "bloc_control.h"
#include "watch_system_interact.h"
#include "communicate_task.h"   /* commu_send_mouse_* — device-page trackpad relay */

#define DBG_TAG "ble.hid"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/**********************Start of HID Constants and Structures
 * ****************************************************/

#define BASE_USB_HID_SPEC_VERSION 0x0101

enum
{
    HIDS_REMOTE_WAKE = 1,
    HIDS_NORMALLY_CONNECTABLE = 2,
};

enum
{
    HIDS_INPUT = 0x01,
    HIDS_OUTPUT = 0x02,
    HIDS_FEATURE = 0x03,
};

/**********************Static Variables
 * ****************************************************/

static struct hids_info info = {
    .version = BASE_USB_HID_SPEC_VERSION,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE,
};

#ifdef HID_MOUSE
static struct hids_report input_mouse = {
    .id = REPORT_ID_MOUSE,
    .type = HIDS_INPUT,
};
static struct mouse_state hid_mouse_state;
#endif

#ifdef HID_KEYBOARD
static struct hids_report input_keyborad = {
    .id = REPORT_ID_KEYBOARD,
    .type = HIDS_INPUT,
};
static struct keyboard_state hid_keyboard_state;
#endif

#ifdef HID_CONSUMER
static struct hids_report input_consumer = {
    .id = REPORT_ID_CONSUMER,
    .type = HIDS_INPUT,
};
static struct consume_key_state hid_consume_state;
#endif

#ifdef HID_TELEPHONY
static struct hids_report input_telephony = {
    .id = REPORT_ID_TELEPHONY,
    .type = HIDS_INPUT,
};
static struct telephony_key_state hid_telephony_state;
#endif

#ifdef HID_TOUCHSCREEN
static struct hids_report input_touchscreen = {
    .id = REPORT_ID_TOUCH,
    .type = HIDS_INPUT,
};
static struct touch_state hid_touch_state;
#endif

static uint8_t ctrl_point;
static uint8_t g_conn_idx = 0;
static ble_hid_data_t *g_hid_data = NULL;

/* Device-page trackpad relay switch. When set, the BLE_HID_Mouse_* primitives
   relay to the phone over SKAI_LINK (commu_send_mouse_*) instead of emitting a
   BLE HID report — the phone then actuates the event on the active target
   device. Toggled by device_pager for the device-page-hosted mouse ONLY; the
   standalone APP_ID_MOUSE app leaves it false and keeps direct BLE HID.
   Lives outside #ifdef HID_MOUSE so the setter is always linkable (device_pager
   is built in the PC sim too, where the mouse primitives aren't registered). */
static bool s_mouse_app_route = false;

/**********************HID Report Map
 * ****************************************************/

static const uint8_t report_map[] = {
// https://learn.microsoft.com/zh-tw/windows-hardware/design/component-guidelines/mouse-collection-report-descriptor
#ifdef HID_MOUSE
    0x05,
    0x01, // Usage Page (Generic Desktop)
    0x09,
    0x02, // Usage (Mouse)
    0xA1,
    0x01, // Collection (Application)
    0x85,
    REPORT_ID_MOUSE, /* Report ID */
    0x09,
    0x01, // Usage (Pointer)
    0xA1,
    0x00, // Collection (Physical)
    0x05,
    0x09, // Usage Page (Button)
    0x19,
    0x01, // Usage Minimum (1)
    0x29,
    0x05, // Usage Maximum (5)
    0x15,
    0x00, // Logical Minimum (0)
    0x25,
    0x01, // Logical Maximum (1)
    0x95,
    0x05, // Report Count (5)
    0x75,
    0x01, // Report Size (1)
    0x81,
    0x02, // Input (Data, Variable, Absolute)
    0x95,
    0x01, // Report Count (1)
    0x75,
    0x03, // Report Size (3)
    0x81,
    0x01, // Input (Constant)

    // === First group: Standard mouse axes (X, Y, Wheel) ===
    0x05,
    0x01, // Usage Page (Generic Desktop)
    0x09,
    0x30, // Usage (X)
    0x09,
    0x31, // Usage (Y)
    0x09,
    0x38, // Usage (Wheel) // Vertical scroll
    0x15,
    0x81, // Logical Minimum (-127)
    0x25,
    0x7F, // Logical Maximum (127)
    0x75,
    0x08, // Report Size (8)
    0x95,
    0x03, // Report Count (3) // Only X, Y, Wheel
    0x81,
    0x06, // Input (Data, Variable, Relative)

    // === Second group: Horizontal scroll (from Consumer Page) ===
    0x05,
    0x0C, // Usage Page (Consumer)
    0x0A,
    0x38,
    0x02, // Usage (AC Pan)
    0x95,
    0x01, // Report Count (1) // Only AC Pan
    0x81,
    0x06, // Input (Data, Variable, Relative)

    0xC0, // End Collection
    0xC0, // End Collection
#endif

#ifdef HID_KEYBOARD
    0x05,
    0x01, /* Usage Page (Generic Desktop) */
    0x09,
    0x06, /* Usage (Keyboard) */
    0xA1,
    0x01, /* Collection (Application) */
    0x85,
    REPORT_ID_KEYBOARD, /* Report ID */
    /* Keys */
    0x05,
    0x07, /* Usage Page (Key Codes) */
    0x19,
    0xe0, /* Usage Minimum (224) */
    0x29,
    0xe7, /* Usage Maximum (231) */
    0x15,
    0x00, /* Logical Minimum (0) */
    0x25,
    0x01, /* Logical Maximum (1) */
    0x75,
    0x01, /* Report Size (1) */
    0x95,
    0x08, /* Report Count (8) */
    0x81,
    0x02, /* Input (Data, Variable, Absolute) */

    0x95,
    0x01, /* Report Count (1) */
    0x75,
    0x08, /* Report Size (8) */
    0x81,
    0x01, /* Input (Constant) reserved byte(1) */

    0x95,
    0x06, /* Report Count (6) */
    0x75,
    0x08, /* Report Size (8) */
    0x15,
    0x00, /* Logical Minimum (0) */
    0x25,
    0x65, /* Logical Maximum (101) */
    0x05,
    0x07, /* Usage Page (Key codes) */
    0x19,
    0x00, /* Usage Minimum (0) */
    0x29,
    0x65, /* Usage Maximum (101) */
    0x81,
    0x00, /* Input (Data, Array) Key array(6 bytes) */

    /* LED */
    0x95,
    0x05, /* Report Count (5) */
    0x75,
    0x01, /* Report Size (1) */
    0x05,
    0x08, /* Usage Page (Page# for LEDs) */
    0x19,
    0x01, /* Usage Minimum (1) */
    0x29,
    0x05, /* Usage Maximum (5) */
    0x91,
    0x02, /* Output (Data, Variable, Absolute), */
    /* Led report */
    0x95,
    0x01, /* Report Count (1) */
    0x75,
    0x03, /* Report Size (3) */
    0x91,
    0x01, /* Output (Data, Variable, Absolute), */
    /* Led report padding */
    0xC0, /* End Collection (Application) */
#endif

#ifdef HID_CONSUMER
    0x05,
    0x0C, // Usage Page (Consumer)
    0x09,
    0x01, // Usage (Consumer Control)
    0xA1,
    0x01, // Collection (Application)
    0x85,
    REPORT_ID_CONSUMER, //   Report ID (3)
    0x15,
    0x00, // Logical minimum (0)
    0x25,
    0x01, // Logical maximum (1)
    0x75,
    0x01, // Report Size (1)
    0x95,
    0x01, // Report Count (1)

    0x09,
    0xCD, // Usage (Play/Pause)
    0x81,
    0x06, // Input (Data,Value,Relative,Bit Field)
    0x0A,
    0x83,
    0x01, // Usage (AL Consumer Control Configuration)
    0x81,
    0x06, // Input (Data,Value,Relative,Bit Field)
    0x09,
    0xB5, // Usage (Scan Next Track)
    0x81,
    0x06, // Input (Data,Value,Relative,Bit Field)
    0x09,
    0xB6, // Usage (Scan Previous Track)
    0x81,
    0x06, // Input (Data,Value,Relative,Bit Field)

    0x09,
    0xEA, // Usage (Volume Down)
    0x81,
    0x06, // Input (Data,Value,Relative,Bit Field)
    0x09,
    0xE9, // Usage (Volume Up)
    0x81,
    0x06, // Input (Data,Value,Relative,Bit Field)
    0x0A,
    0x25,
    0x02, // Usage (AC Forward)
    0x81,
    0x06, // Input (Data,Value,Relative,Bit Field)
    0x0A,
    0x24,
    0x02, // Usage (AC Back)
    0x81,
    0x06, // Input (Data,Value,Relative,Bit Field)
    0xC0, // End Collection
#endif

#ifdef HID_TELEPHONY
    0x05,
    0x0B, // Usage Page (Telephony)
    0x09,
    0x05, // Usage (Headset)
    0xA1,
    0x01, // Collection (Application)
    0x85,
    REPORT_ID_TELEPHONY, // Report ID
    0x15,
    0x00, // Logical Minimum (0)
    0x25,
    0x01, // Logical Maximum (1)
    0x75,
    0x01, // Report Size (1)
    0x95,
    0x01, // Report Count (1)
    0x09,
    0x20, // Usage (Hook Switch)
    0x81,
    0x06, // Input (Data,Var,Rel)
    0x09,
    0x26, // Usage (Drop)
    0x81,
    0x06, // Input (Data,Var,Rel)
    0x75,
    0x06, // Report Size (6)
    0x95,
    0x01, // Report Count (1)
    0x81,
    0x01, // Input (Const) - padding to byte
    0xC0, // End Collection
#endif

#ifdef HID_TOUCHSCREEN
    0x05,
    0x0D, // Usage Page (Digitizers)
    0x09,
    0x04, // Usage (Touch Screen)
    0xA1,
    0x01, // Collection (Application)
    0x85,
    REPORT_ID_TOUCH, // Report ID (Touch)
    0x09,
    0x22, // Usage (Finger)
    0xA1,
    0x02, // Collection (Logical)
    0x09,
    0x42, // Usage (Tip Switch)
    0x15,
    0x00, // Logical Minimum (0)
    0x25,
    0x01, // Logical Maximum (1)
    0x75,
    0x01, // Report Size (1)
    0x95,
    0x01, // Report Count (1)
    0x81,
    0x02, // Input (Data, Variable, Absolute)
    0x09,
    0x51, // Usage (Contact Identifier)
    0x25,
    0x7F, // Logical Maximum (127)
    0x75,
    0x07, // Report Size (7)
    0x95,
    0x01, // Report Count (1)
    0x81,
    0x02, // Input (Data, Variable, Absolute)
    0x05,
    0x01, // Usage Page (Generic Desktop)
    0x09,
    0x30, // Usage (X)
    0x09,
    0x31, // Usage (Y)
    0x16,
    0x00,
    0x00, // Logical Minimum (0)
    0x26,
    0xFF,
    0x7F, // Logical Maximum (32767)
    0x75,
    0x10, // Report Size (16)
    0x95,
    0x02, // Report Count (2)
    0x81,
    0x02, // Input (Data, Variable, Absolute)
    0xC0, // End Collection
    0x05,
    0x0d, //   USAGE_PAGE (Digitizers)
    0x55,
    0x0C, //   UNIT_EXPONENT (-4)
    0x66,
    0x01,
    0x10, //   UNIT (Seconds)
    0x47,
    0xff,
    0xff,
    0x00,
    0x00, //   PHYSICAL_MAXIMUM (65535)
    0x27,
    0xff,
    0xff,
    0x00,
    0x00, //   LOGICAL_MAXIMUM (65535)
    0x75,
    0x10, //   REPORT_SIZE (16)
    0x95,
    0x01, //   REPORT_COUNT (1)
    0x09,
    0x56, //   USAGE (Scan Time)
    0x81,
    0x02, //   INPUT (Data,Var,Abs)
    0x09,
    0x54, //   USAGE (Contact count)
    0x25,
    0x7f, //   LOGICAL_MAXIMUM (127)
    0x95,
    0x01, //   REPORT_COUNT (1)
    0x75,
    0x08, //   REPORT_SIZE (8)
    0x81,
    0x02, //   INPUT (Data,Var,Abs)
    0x85,
    0x04, //   REPORT_ID (Feature)
    0x09,
    0x55, //   USAGE(Contact Count Maximum)
    0x95,
    0x01, //   REPORT_COUNT (1)
    0x25,
    0x02, //   LOGICAL_MAXIMUM (2)
    0xb1,
    0x02, //   FEATURE (Data,Var,Abs)
    0x85,
    0x44, //   REPORT_ID (Feature)
    0x06,
    0x00,
    0xff, //   USAGE_PAGE (Vendor Defined)
    0x09,
    0xC5, //   USAGE (Vendor Usage 0xC5)
    0x15,
    0x00, //   LOGICAL_MINIMUM (0)
    0x26,
    0xff,
    0x00, //   LOGICAL_MAXIMUM (0xff)
    0x75,
    0x08, //   REPORT_SIZE (8)
    0x96,
    0x00,
    0x01, //   REPORT_COUNT (0x100 (256))
    0xb1,
    0x02, //   FEATURE (Data,Var,Abs)
    0xC0, // End Collection
#endif

};

/**********************HID Attribute Database
 * ****************************************************/

struct attm_desc hids_att_db[] = {
    // HID service
    [HIDS_IDX_SVC] = {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},

    // HID Info
    [HIDS_IDX_INFO_CHAR] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    [HIDS_IDX_INFO_VAL] = {ATT_CHAR_HID_INFO, PERM(RD, ENABLE),
                           PERM(RI, ENABLE), sizeof(info)},

    // HID Report map
    [HIDS_IDX_REPORT_MAP] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    [HIDS_IDX_REPORT_MAP_VAL] = {ATT_CHAR_REPORT_MAP, PERM(RD, ENABLE),
                                 PERM(RI, ENABLE), sizeof(report_map)},

#ifdef HID_MOUSE
    // HID Report (Mouse)
    [HIDS_MOUSE_IDX_REPORT] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    [HIDS_MOUSE_IDX_REPORT_VAL] = {ATT_CHAR_REPORT,
                                   PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) |
                                       PERM(WRITE_COMMAND, ENABLE) |
                                       PERM(NTF, ENABLE) | PERM(WP, UNAUTH),
                                   PERM(UUID_LEN, UUID_16) | PERM(RI, ENABLE),
                                   8},
    [HIDS_MOUSE_IDX_REPORT_NTF_CFG] = {ATT_DESC_CLIENT_CHAR_CFG,
                                       PERM(RD, ENABLE) |
                                           PERM(WRITE_REQ, ENABLE) |
                                           PERM(WP, UNAUTH),
                                       PERM(RI, ENABLE), 2},
    [HIDS_MOUSE_IDX_REPORT_REF] = {ATT_DESC_REPORT_REF, PERM(RD, ENABLE),
                                   PERM(RI, ENABLE), 2},
#endif

#ifdef HID_KEYBOARD
    // HID Report (Keyboard)
    [HIDS_KEYBOARD_IDX_REPORT] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0,
                                  0},
    [HIDS_KEYBOARD_IDX_REPORT_VAL] =
        {ATT_CHAR_REPORT,
         PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) |
             PERM(WRITE_COMMAND, ENABLE) | PERM(NTF, ENABLE) | PERM(WP, UNAUTH),
         PERM(UUID_LEN, UUID_16) | PERM(RI, ENABLE), 8},
    [HIDS_KEYBOARD_IDX_REPORT_NTF_CFG] = {ATT_DESC_CLIENT_CHAR_CFG,
                                          PERM(RD, ENABLE) |
                                              PERM(WRITE_REQ, ENABLE) |
                                              PERM(WP, UNAUTH),
                                          PERM(RI, ENABLE), 2},
    [HIDS_KEYBOARD_IDX_REPORT_REF] = {ATT_DESC_REPORT_REF, PERM(RD, ENABLE),
                                      PERM(RI, ENABLE), 2},
#endif

#ifdef HID_CONSUMER
    // HID Report (Consumer)
    [HIDS_CONSUMER_IDX_REPORT] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0,
                                  0},
    [HIDS_CONSUMER_IDX_REPORT_VAL] =
        {ATT_CHAR_REPORT,
         PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) |
             PERM(WRITE_COMMAND, ENABLE) | PERM(NTF, ENABLE) | PERM(WP, UNAUTH),
         PERM(UUID_LEN, UUID_16) | PERM(RI, ENABLE), 8},
    [HIDS_CONSUMER_IDX_REPORT_NTF_CFG] = {ATT_DESC_CLIENT_CHAR_CFG,
                                          PERM(RD, ENABLE) |
                                              PERM(WRITE_REQ, ENABLE) |
                                              PERM(WP, UNAUTH),
                                          PERM(RI, ENABLE), 2},
    [HIDS_CONSUMER_IDX_REPORT_REF] = {ATT_DESC_REPORT_REF, PERM(RD, ENABLE),
                                      PERM(RI, ENABLE), 2},
#endif

#ifdef HID_TELEPHONY
    // HID Report (Telephony)
    [HIDS_TELEPHONY_IDX_REPORT] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0,
                                   0},
    [HIDS_TELEPHONY_IDX_REPORT_VAL] =
        {ATT_CHAR_REPORT,
         PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) |
             PERM(WRITE_COMMAND, ENABLE) | PERM(NTF, ENABLE) | PERM(WP, UNAUTH),
         PERM(UUID_LEN, UUID_16) | PERM(RI, ENABLE), 8},
    [HIDS_TELEPHONY_IDX_REPORT_NTF_CFG] = {ATT_DESC_CLIENT_CHAR_CFG,
                                           PERM(RD, ENABLE) |
                                               PERM(WRITE_REQ, ENABLE) |
                                               PERM(WP, UNAUTH),
                                           PERM(RI, ENABLE), 2},
    [HIDS_TELEPHONY_IDX_REPORT_REF] = {ATT_DESC_REPORT_REF, PERM(RD, ENABLE),
                                       PERM(RI, ENABLE), 2},
#endif

#ifdef HID_TOUCHSCREEN
    // HID Report (Touchscreen)
    [HIDS_TOUCHSCREEN_IDX_REPORT] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE),
                                     0, 0},
    [HIDS_TOUCHSCREEN_IDX_REPORT_VAL] =
        {ATT_CHAR_REPORT,
         PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) |
             PERM(WRITE_COMMAND, ENABLE) | PERM(NTF, ENABLE) | PERM(WP, UNAUTH),
         PERM(UUID_LEN, UUID_16) | PERM(RI, ENABLE), 8},
    [HIDS_TOUCHSCREEN_IDX_REPORT_NTF_CFG] = {ATT_DESC_CLIENT_CHAR_CFG,
                                             PERM(RD, ENABLE) |
                                                 PERM(WRITE_REQ, ENABLE) |
                                                 PERM(WP, UNAUTH),
                                             PERM(RI, ENABLE), 2},
    [HIDS_TOUCHSCREEN_IDX_REPORT_REF] = {ATT_DESC_REPORT_REF, PERM(RD, ENABLE),
                                         PERM(RI, ENABLE), 2},
#endif

    // HID Control
    [HIDS_IDX_CTRL] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    [HIDS_IDX_CTRL_VAL] = {ATT_CHAR_HID_CTNL_PT,
                           PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 1},
};

/**********************HID GATT Callbacks
 * ****************************************************/

uint8_t *ble_hids_gatts_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    uint8_t *ret_val = NULL;
    *len = 0;

    LOG_I("HIDS get: idx=%d", idx);
    switch (idx)
    {
    case HIDS_IDX_INFO_VAL:
        ret_val = (uint8_t *)&info;
        *len = sizeof(info);
        break;
    case HIDS_IDX_REPORT_MAP_VAL:
        ret_val = (uint8_t *)report_map;
        *len = sizeof(report_map);
        break;
    case HIDS_IDX_CTRL_VAL:
    {
        ret_val = (uint8_t *)&ctrl_point;
        *len = sizeof(ctrl_point);
        LOG_I("HIDS %s", ctrl_point ? "Exit Suspend" : "Suspend");
    }
    break;
#ifdef HID_CONSUMER
    case HIDS_CONSUMER_IDX_REPORT_VAL:
        break;
    case HIDS_CONSUMER_IDX_REPORT_REF:
    {
        ret_val = (uint8_t *)&input_consumer;
        *len = sizeof(input_consumer);
    }
    break;
#endif
#ifdef HID_TELEPHONY
    case HIDS_TELEPHONY_IDX_REPORT_VAL:
        break;
    case HIDS_TELEPHONY_IDX_REPORT_REF:
    {
        ret_val = (uint8_t *)&input_telephony;
        *len = sizeof(input_telephony);
    }
    break;
#endif
#ifdef HID_KEYBOARD
    case HIDS_KEYBOARD_IDX_REPORT_VAL:
        break;
    case HIDS_KEYBOARD_IDX_REPORT_REF:
    {
        ret_val = (uint8_t *)&input_keyborad;
        *len = sizeof(input_keyborad);
    }
    break;
#endif
#ifdef HID_MOUSE
    case HIDS_MOUSE_IDX_REPORT_VAL:
        break;
    case HIDS_MOUSE_IDX_REPORT_REF:
    {
        ret_val = (uint8_t *)&input_mouse;
        *len = sizeof(input_mouse);
    }
    break;
#endif
#ifdef HID_TOUCHSCREEN
    case HIDS_TOUCHSCREEN_IDX_REPORT_VAL:
        break;
    case HIDS_TOUCHSCREEN_IDX_REPORT_REF:
    {
        ret_val = (uint8_t *)&input_touchscreen;
        *len = sizeof(input_touchscreen);
    }
    break;
#endif
    default:
        break;
    }
    return ret_val;
}

uint8_t ble_hids_gatts_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    if (!g_hid_data)
        return 0;

    LOG_I("HIDS set: idx=%d", para->idx);
    switch (para->idx)
    {
#ifdef HID_CONSUMER
    case HIDS_CONSUMER_IDX_REPORT_NTF_CFG:
        g_hid_data->is_consumer_config_on = *(para->value);
        LOG_I("CONSUMER CCCD %d", g_hid_data->is_consumer_config_on);
        break;
#endif
#ifdef HID_TELEPHONY
    case HIDS_TELEPHONY_IDX_REPORT_NTF_CFG:
        g_hid_data->is_telephony_config_on = *(para->value);
        LOG_I("TELEPHONY CCCD %d", g_hid_data->is_telephony_config_on);
        break;
#endif
#ifdef HID_KEYBOARD
    case HIDS_KEYBOARD_IDX_REPORT_NTF_CFG:
        g_hid_data->is_keyboard_config_on = *(para->value);
        LOG_I("KEYBOARD CCCD %d", g_hid_data->is_keyboard_config_on);
        break;
#endif
#ifdef HID_MOUSE
    case HIDS_MOUSE_IDX_REPORT_NTF_CFG:
        g_hid_data->is_mouse_config_on = *(para->value);
        LOG_I("MOUSE CCCD %d", g_hid_data->is_mouse_config_on);
        break;
#endif
#ifdef HID_TOUCHSCREEN
    case HIDS_TOUCHSCREEN_IDX_REPORT_NTF_CFG:
        g_hid_data->is_touchscreen_config_on = *(para->value);
        LOG_I("TOUCHSCREEN CCCD %d", g_hid_data->is_touchscreen_config_on);
        break;
#endif
    case HIDS_IDX_CTRL_VAL:
        RT_ASSERT(para->len <= sizeof(ctrl_point));
        memcpy(&ctrl_point, para->value, para->len);
        LOG_I("Updated app value to:%x", ctrl_point);
        break;
    default:
        break;
    }
    return 0;
}

/**********************HID State Management Functions
 * ****************************************************/

#ifdef HID_KEYBOARD
static uint8_t button_ctrl_code(uint8_t key)
{
    if (KEY_CTRL_CODE_MIN <= key && key <= KEY_CTRL_CODE_MAX)
    {
        return (uint8_t)(1U << (key - KEY_CTRL_CODE_MIN));
    }
    return 0;
}

static int hid_kbd_state_key_set(uint8_t key)
{
    uint8_t ctrl_mask = button_ctrl_code(key);

    if (ctrl_mask)
    {
        hid_keyboard_state.ctrl_keys_state |= ctrl_mask;
        return 0;
    }
    for (size_t i = 0; i < KEY_PRESS_MAX; ++i)
    {
        if (hid_keyboard_state.keys_state[i] == 0)
        {
            hid_keyboard_state.keys_state[i] = key;
            return 0;
        }
    }
    return -EBUSY;
}

static int hid_kbd_state_key_clear(uint8_t key)
{
    uint8_t ctrl_mask = button_ctrl_code(key);

    if (ctrl_mask)
    {
        hid_keyboard_state.ctrl_keys_state &= ~ctrl_mask;
        return 0;
    }
    for (size_t i = 0; i < KEY_PRESS_MAX; ++i)
    {
        if (hid_keyboard_state.keys_state[i] == key)
        {
            hid_keyboard_state.keys_state[i] = 0;
            return 0;
        }
    }
    return -EINVAL;
}
#endif

#ifdef HID_CONSUMER
static int hid_consume_state_key_set_bit(uint8_t key)
{
    if ((hid_consume_state.key_state & (1 << key)) == 0)
    {
        hid_consume_state.key_state |= 1 << key;
        return 0;
    }
    return -EBUSY;
}

static int hid_consume_state_key_clear_bit(uint8_t key)
{
    if ((hid_consume_state.key_state & (1 << key)) != 0)
    {
        hid_consume_state.key_state &= ~(1 << key);
        return 0;
    }
    return -EBUSY;
}
#endif

#ifdef HID_TELEPHONY
static int hid_telephony_state_key_set_bit(uint8_t key)
{
    if ((hid_telephony_state.key_state & (1 << key)) == 0)
    {
        hid_telephony_state.key_state |= 1 << key;
        return 0;
    }
    return -EBUSY;
}

static int hid_telephony_state_key_clear_bit(uint8_t key)
{
    if ((hid_telephony_state.key_state & (1 << key)) != 0)
    {
        hid_telephony_state.key_state &= ~(1 << key);
        return 0;
    }
    return -EBUSY;
}
#endif

#ifdef HID_MOUSE
static int hid_mouse_state_set(uint8_t buttons, int8_t x, int8_t y,
                               int8_t wheel, int8_t pan)
{
    hid_mouse_state.buttons = buttons;
    hid_mouse_state.x = x;
    hid_mouse_state.y = y;
    hid_mouse_state.wheel = wheel;
    hid_mouse_state.pan = pan;
    return 0;
}

static int hid_mouse_state_clear(void)
{
    hid_mouse_state.buttons = 0;
    hid_mouse_state.x = 0;
    hid_mouse_state.y = 0;
    hid_mouse_state.wheel = 0;
    hid_mouse_state.pan = 0;
    return 0;
}
#endif

#ifdef HID_TOUCHSCREEN
static int hid_touch_state_set(uint8_t touch, uint16_t x, uint16_t y)
{
    hid_touch_state.touch = touch;
    hid_touch_state.x = x;
    hid_touch_state.y = y;
    return 0;
}

static int hid_touch_state_clear(void)
{
    hid_touch_state.touch = 0;
    hid_touch_state.x = 0;
    hid_touch_state.y = 0;
    return 0;
}
#endif

/**********************HID Report Send Functions
 * ****************************************************/

#ifdef HID_TELEPHONY
static void telephony_report_send(uint8_t *key_val, uint16_t key_val_len)
{
    if (!g_hid_data || !g_hid_data->is_telephony_config_on)
        return;

    sibles_value_t value;
    value.hdl = g_hid_data->srv_handle;
    value.idx = HIDS_TELEPHONY_IDX_REPORT_VAL;
    value.len = key_val_len;
    value.value = key_val;
    int ret = sibles_write_value(g_conn_idx, &value);
    LOG_D("telephony report send retry:%d", g_conn_idx);
    if (ret == 0)
    {
        int retry = 20;
        while (retry > 0)
        {
            retry--;
            rt_thread_mdelay(50);
            ret = sibles_write_value(g_conn_idx, &value);
            if (ret == key_val_len)
            {
                LOG_I("telephony send retry success : %d", key_val[0]);
                return;
            }
        }
    }
    else
    {
        LOG_I("telephony send success : %d", key_val[0]);
    }
}
#endif

#ifdef HID_CONSUMER
static void consumer_report_send(uint8_t *key_val, uint16_t key_val_len)
{
    if (!g_hid_data || !g_hid_data->is_consumer_config_on)
        return;

    sibles_value_t value;
    value.hdl = g_hid_data->srv_handle;
    value.idx = HIDS_CONSUMER_IDX_REPORT_VAL;
    value.len = key_val_len;
    value.value = key_val;
    int ret = sibles_write_value(g_conn_idx, &value);
    LOG_D("consumer report send retry:%d", g_conn_idx);
    if (ret == 0)
    {
        int retry = 20;
        while (retry > 0)
        {
            retry--;
            rt_thread_mdelay(50);
            ret = sibles_write_value(g_conn_idx, &value);
            if (ret == key_val_len)
            {
                LOG_I("send retry success : %d", key_val[0]);
                return;
            }
        }
    }
    else
    {
        LOG_I("send success : %d", key_val[0]);
    }
}
#endif

#ifdef HID_KEYBOARD
static void key_report_send(uint8_t *key_val, uint16_t key_val_len)
{
    if (!g_hid_data || !g_hid_data->is_keyboard_config_on)
        return;

    sibles_value_t value;
    value.hdl = g_hid_data->srv_handle;
    value.idx = HIDS_KEYBOARD_IDX_REPORT_VAL;
    value.len = key_val_len;
    value.value = key_val;
    int ret = sibles_write_value(g_conn_idx, &value);
    LOG_D("key report send retry:%d", g_conn_idx);
    if (ret == 0)
    {
        int retry = 20;
        while (retry > 0)
        {
            retry--;
            rt_thread_mdelay(50);
            ret = sibles_write_value(g_conn_idx, &value);
            if (ret == key_val_len)
            {
                LOG_I("send retry success");
                return;
            }
        }
    }
    else
    {
        LOG_I("send success");
    }
}
#endif

#ifdef HID_MOUSE
static void mouse_report_send(uint8_t *key_val, uint16_t key_val_len)
{
    if (!g_hid_data || !g_hid_data->is_mouse_config_on)
    {
        return;
    }

    sibles_value_t value;
    value.hdl = g_hid_data->srv_handle;
    value.idx = HIDS_MOUSE_IDX_REPORT_VAL;
    value.len = key_val_len;
    value.value = key_val;
    int ret = sibles_write_value(g_conn_idx, &value);
    LOG_D("mouse report send retry:%d", g_conn_idx);

    if (ret == 0)
    {
        int retry = 20;
        while (retry > 0)
        {
            retry--;
            rt_thread_mdelay(50);
            ret = sibles_write_value(g_conn_idx, &value);
            if (ret == key_val_len)
            {
                LOG_I("send retry success");
                return;
            }
        }
    }
}
#endif

#ifdef HID_TOUCHSCREEN
static void touchscreen_report_send(uint8_t *key_val, uint16_t key_val_len)
{
    if (!g_hid_data || !g_hid_data->is_touchscreen_config_on)
        return;

    sibles_value_t value;
    value.hdl = g_hid_data->srv_handle;
    value.idx = HIDS_TOUCHSCREEN_IDX_REPORT_VAL;
    value.len = key_val_len;
    value.value = key_val;
    sibles_write_value(g_conn_idx, &value);
}
#endif

/**********************Public API Functions
 * ****************************************************/

void ble_hid_service_init(ble_hid_data_t *hid_data)
{
    g_hid_data = hid_data;

    sibles_register_svc_t svc;
    svc.att_db = (struct attm_desc *)&hids_att_db;
    svc.num_entry = HDIS_ATT_NB;
    svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH) | PERM(SVC_UUID_LEN, UUID_16);
    svc.uuid = ATT_SVC_HID;

    g_hid_data->srv_handle = sibles_register_svc(&svc);
    if (g_hid_data->srv_handle)
    {
        LOG_I("Register HID service handle");
        sibles_register_cbk(g_hid_data->srv_handle, ble_hids_gatts_get_cbk,
                            ble_hids_gatts_set_cbk);
    }
    else
    {
        LOG_E("HID service registration failed");
    }
}

ble_hid_data_t *ble_hid_get_data(void)
{
    return g_hid_data;
}

void ble_hid_set_conn_idx(uint8_t conn_idx)
{
    g_conn_idx = conn_idx;
}

uint8_t ble_hid_get_conn_idx(void)
{
    return g_conn_idx;
}

void ble_hid_mouse_set_app_route(bool on)
{
    s_mouse_app_route = on;
}

bool ble_hid_mouse_app_route(void)
{
    return s_mouse_app_route;
}

void ble_hid_reset_on_disconnect(void)
{
    if (!g_hid_data)
        return;

#ifdef HID_CONSUMER
    g_hid_data->is_consumer_config_on = 0;
#endif
#ifdef HID_TELEPHONY
    g_hid_data->is_telephony_config_on = 0;
#endif
#ifdef HID_MOUSE
    g_hid_data->is_mouse_config_on = 0;
#endif
#ifdef HID_KEYBOARD
    g_hid_data->is_keyboard_config_on = 0;
#endif
#ifdef HID_TOUCHSCREEN
    g_hid_data->is_touchscreen_config_on = 0;
#endif
}

/**********************HID Input Functions - Mouse
 * ****************************************************/

#ifdef HID_MOUSE
void BLE_HID_Mouse_Move(int8_t dx, int8_t dy)
{
    if (s_mouse_app_route) { commu_send_mouse_move(dx, dy); return; }
    hid_mouse_state_set(hid_mouse_state.buttons, dx, dy, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
}

void BLE_HID_Mouse_Wheel_Scroll(int8_t delta)
{
    if (s_mouse_app_route) { commu_send_mouse_scroll(0, delta); return; }
    hid_mouse_state_set(hid_mouse_state.buttons, 0, 0, delta, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
}

void BLE_HID_Mouse_Pan_Scroll(int8_t delta)
{
    if (s_mouse_app_route) { commu_send_mouse_scroll(delta, 0); return; }
    hid_mouse_state_set(hid_mouse_state.buttons, 0, 0, 0, delta);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
}

static void BLE_HID_Mouse_LeftPress(void)
{
    if (s_mouse_app_route) { commu_send_mouse_button(0, 1); return; }
    hid_mouse_state_set(1, 0, 0, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
}

static void BLE_HID_Mouse_LeftRelease(void)
{
    if (s_mouse_app_route) { commu_send_mouse_button(0, 0); return; }
    hid_mouse_state_set(0, 0, 0, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
    hid_mouse_state_clear();
}

void BLE_HID_Mouse_LeftClick(void)
{
    /* Relay a single click event (no 200ms BLE hold) when routing to the app. */
    if (s_mouse_app_route) { commu_send_mouse_button(0, 2); return; }
    BLE_HID_Mouse_LeftPress();
    rt_thread_mdelay(200);
    BLE_HID_Mouse_LeftRelease();
}

static void BLE_HID_Mouse_RightPress(void)
{
    if (s_mouse_app_route) { commu_send_mouse_button(1, 1); return; }
    hid_mouse_state_set(2, 0, 0, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
    hid_mouse_state_clear();
}

static void BLE_HID_Mouse_RightRelease(void)
{
    if (s_mouse_app_route) { commu_send_mouse_button(1, 0); return; }
    hid_mouse_state_set(0, 0, 0, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
    hid_mouse_state_clear();
}

void BLE_HID_Mouse_RightClick(void)
{
    if (s_mouse_app_route) { commu_send_mouse_button(1, 2); return; }
    BLE_HID_Mouse_RightPress();
    rt_thread_mdelay(200);
    BLE_HID_Mouse_RightRelease();
}

static void BLE_HID_Mouse_BackPress(void)
{
    hid_mouse_state_set(8, 0, 0, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
}

static void BLE_HID_Mouse_BackRelease(void)
{
    hid_mouse_state_set(0, 0, 0, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
    hid_mouse_state_clear();
}

/**********************HID Touch-to-Mouse: long-press + edge-pan
 * ****************************************************/

#define MOUSE_LONG_PRESS_MS           500    /* hold threshold */
#define MOUSE_DOUBLECLICK_WINDOW_MS   300    /* after 1st release: wait this long for a 2nd press */
#define MOUSE_DOUBLECLICK_RADIUS_PX   40     /* same-place tolerance for 2nd press */
#define MOUSE_LONGPRESS_MOVE_CANCEL   5      /* finger must stay inside this radius (px) to arm long-press — matches SCROLLING_THRESHOLD */

#define WATCH_SCREEN_SIZE             466
#define WATCH_CENTER_PX               (WATCH_SCREEN_SIZE / 2)   /* 233 */
#define WATCH_RADIUS_PX               (WATCH_SCREEN_SIZE / 2)   /* 233 */
#define EDGE_PAN_MARGIN_PX            65                         /* edge-band width — pan starts this far inward from the circular edge */
#define EDGE_PAN_PERIOD_MS            30                         /* pan tick */
#define EDGE_PAN_MAX_SPEED            18                         /* px per tick at the very edge */

typedef enum
{
    MOUSE_TOUCH_IDLE = 0,
    MOUSE_TOUCH_ARMED,          /* finger down, long-press timer running */
    MOUSE_TOUCH_LONG_PRESSING,  /* long-press active, left button held */
    MOUSE_TOUCH_PENDING_CLICK,  /* finger up after short tap, waiting to see if a 2nd press arrives */
} mouse_touch_state_t;

static mouse_touch_state_t s_touch_state = MOUSE_TOUCH_IDLE;
static rt_timer_t s_long_press_timer    = NULL;
static rt_timer_t s_edge_pan_timer      = NULL;
static rt_timer_t s_pending_click_timer = NULL;
static bool       s_edge_pan_running    = false;
static int8_t     s_edge_pan_dx         = 0;
static int8_t     s_edge_pan_dy         = 0;

static uint16_t   s_press_start_x = 0;   /* position of the most recent press (used for double-click match) */
static uint16_t   s_press_start_y = 0;

static int mouse_sq_dist(int dx, int dy) { return dx * dx + dy * dy; }

static void mouse_enter_long_press(void)
{
    if (s_touch_state == MOUSE_TOUCH_LONG_PRESSING)
        return;
    s_touch_state = MOUSE_TOUCH_LONG_PRESSING;
    BLE_HID_Mouse_LeftPress();
    motor_pattern_scrolling_app();
    LOG_I("HID mouse long-press start");
}

static void mouse_exit_long_press(void)
{
    if (s_touch_state != MOUSE_TOUCH_LONG_PRESSING)
        return;
    BLE_HID_Mouse_LeftRelease();
    s_touch_state = MOUSE_TOUCH_IDLE;
    LOG_I("HID mouse long-press end");
}

static void long_press_timer_cb(void *parameter)
{
    (void)parameter;
    if (s_touch_state == MOUSE_TOUCH_ARMED)
        mouse_enter_long_press();
}

static void long_press_timer_arm(void)
{
    if (s_long_press_timer == NULL)
    {
        s_long_press_timer = rt_timer_create(
            "hid_lp", long_press_timer_cb, RT_NULL,
            rt_tick_from_millisecond(MOUSE_LONG_PRESS_MS),
            RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (s_long_press_timer)
    {
        rt_timer_stop(s_long_press_timer);
        rt_timer_start(s_long_press_timer);
    }
}

static void long_press_timer_cancel(void)
{
    if (s_long_press_timer)
        rt_timer_stop(s_long_press_timer);
}

static void pending_click_timer_cb(void *parameter)
{
    (void)parameter;
    if (s_touch_state != MOUSE_TOUCH_PENDING_CLICK)
        return;
    s_touch_state = MOUSE_TOUCH_IDLE;
    LOG_I("HID mouse deferred click fires");
    BLE_HID_Mouse_LeftClick();
}

static void pending_click_timer_arm(void)
{
    if (s_pending_click_timer == NULL)
    {
        s_pending_click_timer = rt_timer_create(
            "hid_pc", pending_click_timer_cb, RT_NULL,
            rt_tick_from_millisecond(MOUSE_DOUBLECLICK_WINDOW_MS),
            RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (s_pending_click_timer)
    {
        rt_timer_stop(s_pending_click_timer);
        rt_timer_start(s_pending_click_timer);
    }
}

static void pending_click_timer_cancel(void)
{
    if (s_pending_click_timer)
        rt_timer_stop(s_pending_click_timer);
}

static void edge_pan_timer_cb(void *parameter)
{
    (void)parameter;
    if (s_touch_state != MOUSE_TOUCH_LONG_PRESSING)
        return;
    if (s_edge_pan_dx == 0 && s_edge_pan_dy == 0)
        return;
    BLE_HID_Mouse_Move(s_edge_pan_dx, s_edge_pan_dy);
}

static void edge_pan_stop(void)
{
    if (s_edge_pan_running && s_edge_pan_timer)
        rt_timer_stop(s_edge_pan_timer);
    s_edge_pan_running = false;
    s_edge_pan_dx = 0;
    s_edge_pan_dy = 0;
}

static void edge_pan_start(void)
{
    if (s_edge_pan_timer == NULL)
    {
        s_edge_pan_timer = rt_timer_create(
            "hid_pan", edge_pan_timer_cb, RT_NULL,
            rt_tick_from_millisecond(EDGE_PAN_PERIOD_MS),
            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (s_edge_pan_timer && !s_edge_pan_running)
    {
        rt_timer_start(s_edge_pan_timer);
        s_edge_pan_running = true;
    }
}

/* Approximate vector length, avoids sqrt. Max error ~3% vs true magnitude. */
static int approx_len(int dx, int dy)
{
    int a = dx < 0 ? -dx : dx;
    int b = dy < 0 ? -dy : dy;
    return (a > b) ? (a + (b >> 1)) : (b + (a >> 1));
}

static void edge_pan_update(uint16_t x, uint16_t y)
{
    int dx = (int)x - WATCH_CENTER_PX;
    int dy = (int)y - WATCH_CENTER_PX;
    int inner = WATCH_RADIUS_PX - EDGE_PAN_MARGIN_PX;

    if (mouse_sq_dist(dx, dy) <= inner * inner)
    {
        edge_pan_stop();
        return;
    }

    int len = approx_len(dx, dy);
    if (len == 0)
    {
        edge_pan_stop();
        return;
    }

    int depth = len - inner;                      /* 0..margin (or beyond) */
    if (depth > EDGE_PAN_MARGIN_PX) depth = EDGE_PAN_MARGIN_PX;
    int intensity = (depth * EDGE_PAN_MAX_SPEED) / EDGE_PAN_MARGIN_PX;
    if (intensity < 1) intensity = 1;

    s_edge_pan_dx = (int8_t)((dx * intensity) / len);
    s_edge_pan_dy = (int8_t)((dy * intensity) / len);
    edge_pan_start();
}

void BLE_HID_Mouse_Touch_Press(uint16_t x, uint16_t y)
{
    /* Case A: 2nd press arrived within the double-click window → long-press. */
    if (s_touch_state == MOUSE_TOUCH_PENDING_CLICK)
    {
        int ddx = (int)x - (int)s_press_start_x;
        int ddy = (int)y - (int)s_press_start_y;
        bool same_place =
            mouse_sq_dist(ddx, ddy) <=
            MOUSE_DOUBLECLICK_RADIUS_PX * MOUSE_DOUBLECLICK_RADIUS_PX;

        pending_click_timer_cancel();

        if (same_place)
        {
            s_press_start_x = x;
            s_press_start_y = y;
            s_touch_state = MOUSE_TOUCH_ARMED;    /* treat as pressed-in */
            mouse_enter_long_press();
            return;
        }
        /* Different place → honour the pending single click, then restart. */
        BLE_HID_Mouse_LeftClick();
        s_touch_state = MOUSE_TOUCH_IDLE;
    }

    /* Clear any residuals from an abnormal prior interaction. */
    long_press_timer_cancel();
    edge_pan_stop();
    if (s_touch_state == MOUSE_TOUCH_LONG_PRESSING)
        mouse_exit_long_press();

    s_press_start_x = x;
    s_press_start_y = y;
    s_touch_state = MOUSE_TOUCH_ARMED;
    long_press_timer_arm();
}

void BLE_HID_Mouse_Touch_Move(uint16_t x, uint16_t y)
{
    if (s_touch_state == MOUSE_TOUCH_ARMED)
    {
        int dx = (int)x - (int)s_press_start_x;
        int dy = (int)y - (int)s_press_start_y;
        if (mouse_sq_dist(dx, dy) >
            MOUSE_LONGPRESS_MOVE_CANCEL * MOUSE_LONGPRESS_MOVE_CANCEL)
        {
            long_press_timer_cancel();
            s_touch_state = MOUSE_TOUCH_IDLE;
        }
    }
    else if (s_touch_state == MOUSE_TOUCH_LONG_PRESSING)
    {
        edge_pan_update(x, y);
    }
}

bool BLE_HID_Mouse_Touch_Release(uint16_t x, uint16_t y)
{
    (void)x;
    (void)y;

    long_press_timer_cancel();
    edge_pan_stop();

    if (s_touch_state == MOUSE_TOUCH_LONG_PRESSING)
    {
        mouse_exit_long_press();
        return true;    /* caller must skip its own click */
    }

    if (s_touch_state == MOUSE_TOUCH_ARMED)
    {
        /* Short tap — defer the click; a 2nd press inside the window upgrades to long-press. */
        s_touch_state = MOUSE_TOUCH_PENDING_CLICK;
        pending_click_timer_arm();
        return true;    /* caller must NOT fire its own click — we own it now */
    }

    /* Moved-too-far cancel, or nothing in progress. */
    s_touch_state = MOUSE_TOUCH_IDLE;
    return false;
}

static int init_ble_mouse_func(void)
{
    control_provider.ble_hid_mouse_move = BLE_HID_Mouse_Move;
    control_provider.ble_hid_mouse_wheel_scroll = BLE_HID_Mouse_Wheel_Scroll;
    control_provider.ble_hid_mouse_pan_scroll = BLE_HID_Mouse_Pan_Scroll;
    control_provider.ble_hid_mouse_left_press = BLE_HID_Mouse_LeftPress;
    control_provider.ble_hid_mouse_left_release = BLE_HID_Mouse_LeftRelease;
    control_provider.ble_hid_mouse_left_click = BLE_HID_Mouse_LeftClick;
    control_provider.ble_hid_mouse_right_press = BLE_HID_Mouse_RightPress;
    control_provider.ble_hid_mouse_right_release = BLE_HID_Mouse_RightRelease;
    control_provider.ble_hid_mouse_right_click = BLE_HID_Mouse_RightClick;
    return 0;
}
    #ifndef BSP_USING_PC_SIMULATOR
INIT_APP_EXPORT(init_ble_mouse_func);
    #endif
#endif // HID_MOUSE

/**********************HID Input Functions - Consumer
 * ****************************************************/

#ifdef HID_CONSUMER
/* Press → wait → release helper. Each consumer button is "set bit, send,
   hold for delay_ms, clear bit, send" — same structure for play/next/prev
   (200ms hold) and volume up/down (50ms — shorter so repeated taps register
   discretely on the host OS volume mixer). */
static void hid_consumer_press_release(uint16_t key, uint32_t delay_ms)
{
    hid_consume_state_key_set_bit(key);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
    rt_thread_mdelay(delay_ms);
    hid_consume_state_key_clear_bit(key);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
}

void play_pause_through_hid(void)  { hid_consumer_press_release(HIDS_CTRL_PLAY,      200); }
void play_next_through_hid(void)   { hid_consumer_press_release(HIDS_CTRL_SCAN_NEX,  200); }
void play_prev_through_hid(void)   { hid_consumer_press_release(HIDS_CTRL_SCAN_PREV, 200); }
void volume_up_through_hid(void)   { hid_consumer_press_release(HIDS_CTRL_VOL_UP,     50); }
void volume_down_through_hid(void) { hid_consumer_press_release(HIDS_CTRL_VOL_DOWN,   50); }

void HID_CONSUMER_GoBack(void)
{
    LOG_D("Set key back");
    BLE_HID_Mouse_BackPress();
    rt_thread_mdelay(200);
    BLE_HID_Mouse_BackRelease();
}

#ifdef HID_TELEPHONY
static void hid_telephony_press_release(uint16_t key, uint32_t delay_ms)
{
    hid_telephony_state_key_set_bit(key);
    telephony_report_send((uint8_t *)&hid_telephony_state,
                          sizeof(hid_telephony_state));
    rt_thread_mdelay(delay_ms);
    hid_telephony_state_key_clear_bit(key);
    telephony_report_send((uint8_t *)&hid_telephony_state,
                          sizeof(hid_telephony_state));
}

void hang_up_through_hid(void)
{
    LOG_I("HID telephony: drop");
    hid_telephony_press_release(HIDS_TEL_DROP, 200);
}

void hook_switch_through_hid(void)
{
    LOG_I("HID telephony: hook switch");
    hid_telephony_press_release(HIDS_TEL_HOOK_SWITCH, 200);
}
#endif

void HID_CONSUMER_GoHome(void)
{
    // Implementation for home button if needed
}
#endif // HID_CONSUMER

/**********************HID Input Functions - Keyboard
 * ****************************************************/

#ifdef HID_KEYBOARD
    // Key code definitions
    #define KEY_1 0x1e
    #define KEY_2 0x1f
    #define KEY_3 0x20
    #define KEY_4 0x21
    #define KEY_5 0x22
    #define KEY_6 0x23
    #define KEY_7 0x24
    #define KEY_8 0x25
    #define KEY_9 0x26
    #define KEY_0 0x27
    #define KEY_A 0x04
    #define KEY_B 0x05
    #define KEY_C 0x06
    #define KEY_D 0x07
    #define KEY_E 0x08
    #define KEY_F 0x09
    #define KEY_G 0x0A
    #define KEY_H 0x0B
    #define KEY_I 0x0C
    #define KEY_J 0x0D
    #define KEY_K 0x0E
    #define KEY_L 0x0F
    #define KEY_M 0x10
    #define KEY_N 0x11
    #define KEY_O 0x12
    #define KEY_P 0x13
    #define KEY_Q 0x14
    #define KEY_R 0x15
    #define KEY_S 0x16
    #define KEY_T 0x17
    #define KEY_U 0x18
    #define KEY_V 0x19
    #define KEY_W 0x1A
    #define KEY_X 0x1B
    #define KEY_Y 0x1C
    #define KEY_Z 0x1D
    #define KEY_SPACE 0x2C
    #define KEY_ENTER 0x28
    #define KEY_BACKSPACE 0x2A
    #define KEY_MINUS 0x2D
    #define KEY_EQUAL 0x2E
    #define KEY_LEFT_BRACKET 0x2F
    #define KEY_RIGHT_BRACKET 0x30
    #define KEY_BACKSLASH 0x31
    #define KEY_LEFTGUI 0xE3
    #define KEY_PLUS 0x57
    #define KEY_ALT 0xE2
    #define KEY_ESC 0x29
    #define KEY_CTRL 0x01
    #define KEY_SHIFT 0xe1
    #define KEY_F3 0x3C
    #define KEY_TAB 0x2B
    #define KEY_MOD_LMETA 0xE3
    #define KEY_PAGE_UP 0x4B
    #define KEY_PAGE_DOWN 0x4E

    #define HID_KEY_SET(key) hid_kbd_state_key_set(key)
    #define HID_KEY_CLEAR(key) hid_kbd_state_key_clear(key)
    #define HID_KEY_SEND()                                                     \
        key_report_send((uint8_t *)&hid_keyboard_state,                        \
                        sizeof(hid_keyboard_state))

void BLE_HID_Keyboard_Multitask(bool state)
{
    if (state)
    {
        HID_KEY_SET(KEY_MOD_LMETA);
        HID_KEY_SEND();
        rt_thread_mdelay(50);
        HID_KEY_SET(KEY_TAB);
        HID_KEY_SEND();
        rt_thread_mdelay(50);
        HID_KEY_CLEAR(KEY_TAB);
        HID_KEY_SEND();
        rt_thread_mdelay(50);
        HID_KEY_CLEAR(KEY_MOD_LMETA);
        HID_KEY_SEND();
    }
    else
    {
        HID_KEY_SET(KEY_ESC);
        HID_KEY_SEND();
        rt_thread_mdelay(50);
        HID_KEY_CLEAR(KEY_ESC);
        HID_KEY_SEND();
    }
}

static void BLE_HID_Keyboard_Scrollpage(bool up)
{
    if (up)
    {
        HID_KEY_SET(KEY_PAGE_UP);
        HID_KEY_SEND();
        rt_thread_mdelay(50);
        HID_KEY_CLEAR(KEY_PAGE_UP);
        HID_KEY_SEND();
    }
    else
    {
        HID_KEY_SET(KEY_PAGE_DOWN);
        HID_KEY_SEND();
        rt_thread_mdelay(50);
        HID_KEY_CLEAR(KEY_PAGE_DOWN);
        HID_KEY_SEND();
    }
}
void BLE_HID_keyboard_go_home(void)
{
    HID_KEY_SET(KEY_MOD_LMETA);
    HID_KEY_SEND();
    rt_thread_mdelay(200);
    HID_KEY_SET(KEY_D);
    HID_KEY_SEND();
    rt_thread_mdelay(200);
    HID_KEY_CLEAR(KEY_D);
    HID_KEY_SEND();
    rt_thread_mdelay(200);
    HID_KEY_CLEAR(KEY_MOD_LMETA);
    HID_KEY_SEND();
}

void BLE_HID_Keyboard_Paste(void)
{
    HID_KEY_SET(KEY_CTRL);
    HID_KEY_SEND();
    rt_thread_mdelay(50);
    HID_KEY_SET(KEY_V);
    HID_KEY_SEND();
    rt_thread_mdelay(50);
    HID_KEY_CLEAR(KEY_V);
    HID_KEY_SEND();
    rt_thread_mdelay(50);
    HID_KEY_CLEAR(KEY_CTRL);
    HID_KEY_SEND();
}

void BLE_HID_keyboard_Shift(bool state)
{
    if (state)
    {
        HID_KEY_SET(KEY_SHIFT);
        HID_KEY_SEND();
    }
    else
    {
        HID_KEY_CLEAR(KEY_SHIFT);
        HID_KEY_SEND();
    }
}

static void BLE_HID_WINDOWS_ZOOM(bool state)
{
    HID_KEY_SET(KEY_LEFTGUI);
    HID_KEY_SEND();
    rt_thread_mdelay(100);
    HID_KEY_SET(state ? KEY_PLUS : KEY_MINUS);
    HID_KEY_SEND();
    rt_thread_mdelay(100);
    HID_KEY_CLEAR(state ? KEY_PLUS : KEY_MINUS);
    HID_KEY_SEND();
    rt_thread_mdelay(100);
    HID_KEY_CLEAR(KEY_LEFTGUI);
    HID_KEY_SEND();
}

static void BLE_HID_MAC_ZOOM(bool state)
{
    HID_KEY_SET(KEY_LEFTGUI);
    HID_KEY_SEND();
    rt_thread_mdelay(50);
    HID_KEY_SET(KEY_ALT);
    HID_KEY_SEND();
    rt_thread_mdelay(50);
    HID_KEY_SET(state ? KEY_PLUS : KEY_MINUS);
    HID_KEY_SEND();
    rt_thread_mdelay(50);
    HID_KEY_CLEAR(state ? KEY_PLUS : KEY_MINUS);
    HID_KEY_SEND();
    rt_thread_mdelay(50);
    HID_KEY_CLEAR(KEY_ALT);
    HID_KEY_SEND();
    rt_thread_mdelay(50);
    HID_KEY_CLEAR(KEY_LEFTGUI);
    HID_KEY_SEND();
}



void BLE_HID_computer_zoom(uint8_t device, bool state)
{
    if (device == 0)
    {
        BLE_HID_WINDOWS_ZOOM(state);
    }
    else
    {
        BLE_HID_MAC_ZOOM(state);
    }
}

void BLE_HID_ZOOM_ESC(void)
{
    HID_KEY_SET(KEY_LEFTGUI);
    HID_KEY_SEND();
    rt_thread_mdelay(70);
    HID_KEY_SET(KEY_ESC);
    HID_KEY_SEND();
    rt_thread_mdelay(100);
    HID_KEY_CLEAR(KEY_ESC);
    HID_KEY_SEND();
    rt_thread_mdelay(70);
    HID_KEY_CLEAR(KEY_LEFTGUI);
    HID_KEY_SEND();
}

static void BLE_HID_Send_Character(char character)
{
    uint8_t key_code = 0;
    bool need_shift = false;

    if (character >= 'a' && character <= 'z')
    {
        key_code = KEY_A + (character - 'a');
    }
    else if (character >= 'A' && character <= 'Z')
    {
        key_code = KEY_A + (character - 'A');
        need_shift = true;
    }
    else if (character >= '1' && character <= '9')
    {
        key_code = KEY_1 + (character - '1');
    }
    else if (character == '0')
    {
        key_code = KEY_0;
    }
    else
    {
        switch (character)
        {
        case ' ':
            key_code = KEY_SPACE;
            break;
        case '\n':
            key_code = KEY_ENTER;
            break;
        case '\b':
            key_code = KEY_BACKSPACE;
            break;
        case '!':
            key_code = KEY_1;
            need_shift = true;
            break;
        case '@':
            key_code = KEY_2;
            need_shift = true;
            break;
        case '#':
            key_code = KEY_3;
            need_shift = true;
            break;
        case '$':
            key_code = KEY_4;
            need_shift = true;
            break;
        case '%':
            key_code = KEY_5;
            need_shift = true;
            break;
        case '^':
            key_code = KEY_6;
            need_shift = true;
            break;
        case '&':
            key_code = KEY_7;
            need_shift = true;
            break;
        case '*':
            key_code = KEY_8;
            need_shift = true;
            break;
        case '(':
            key_code = KEY_9;
            need_shift = true;
            break;
        case ')':
            key_code = KEY_0;
            need_shift = true;
            break;
        case '-':
            key_code = KEY_MINUS;
            break;
        case '_':
            key_code = KEY_MINUS;
            need_shift = true;
            break;
        case '=':
            key_code = KEY_EQUAL;
            break;
        case '+':
            key_code = KEY_EQUAL;
            need_shift = true;
            break;
        case '[':
            key_code = KEY_LEFT_BRACKET;
            break;
        case '{':
            key_code = KEY_LEFT_BRACKET;
            need_shift = true;
            break;
        case ']':
            key_code = KEY_RIGHT_BRACKET;
            break;
        case '}':
            key_code = KEY_RIGHT_BRACKET;
            need_shift = true;
            break;
        case '\\':
            key_code = KEY_BACKSLASH;
            break;
        case '|':
            key_code = KEY_BACKSLASH;
            need_shift = true;
            break;
        default:
            return;
        }
    }

    if (need_shift)
    {
        HID_KEY_SET(KEY_SHIFT);
        HID_KEY_SEND();
        rt_thread_mdelay(20);
    }
    else
    {
        rt_thread_mdelay(20);
    }

    HID_KEY_SET(key_code);
    HID_KEY_SEND();
    rt_thread_mdelay(20);
    HID_KEY_CLEAR(key_code);
    HID_KEY_SEND();

    if (need_shift)
    {
        rt_thread_mdelay(20);
        HID_KEY_CLEAR(KEY_SHIFT);
        HID_KEY_SEND();
    }
    else
    {
        rt_thread_mdelay(20);
    }
}

void BLE_HID_Send_String(const char *str)
{
    if (str == NULL)
        return;

    while (*str)
    {
        BLE_HID_Send_Character(*str);
        rt_thread_mdelay(50);
        str++;
    }
}

static int init_ble_keyboard_func(void)
{
    control_provider.ble_hid_mouse_back = HID_CONSUMER_GoBack;
    control_provider.ble_hid_keyboard_scroll_page = BLE_HID_Keyboard_Scrollpage;
    control_provider.ble_hid_keyboard_multitask = BLE_HID_Keyboard_Multitask;
    control_provider.ble_hid_keyboard_shift = BLE_HID_keyboard_Shift;
    control_provider.ble_hid_keyboard_go_home = BLE_HID_keyboard_go_home;
    control_provider.ble_hid_keyboard_paste = BLE_HID_Keyboard_Paste;
    control_provider.ble_hid_keyboard_input = BLE_HID_Send_String;
    control_provider.ble_hid_zoom = BLE_HID_computer_zoom;
    control_provider.ble_hid_zoom_esc = BLE_HID_ZOOM_ESC;
    return 0;
}
    #ifndef BSP_USING_PC_SIMULATOR
INIT_APP_EXPORT(init_ble_keyboard_func);
    #endif
#endif // HID_KEYBOARD

/**********************HID Input Functions - Touchscreen
 * ****************************************************/

#ifdef HID_TOUCHSCREEN
void BLE_HID_Touchscreen_Press(uint16_t x, uint16_t y)
{
    hid_touch_state_set(1, x, y);
    touchscreen_report_send((uint8_t *)&hid_touch_state,
                            sizeof(hid_touch_state));
}

void BLE_HID_Touchscreen_Release(uint16_t x, uint16_t y)
{
    hid_touch_state_set(0, x, y);
    touchscreen_report_send((uint8_t *)&hid_touch_state,
                            sizeof(hid_touch_state));
}

static int init_ble_touch_screen_func(void)
{
    control_provider.ble_hid_touch_screen_press = BLE_HID_Touchscreen_Press;
    control_provider.ble_hid_touch_screen_release = BLE_HID_Touchscreen_Release;
    return 0;
}
    #ifndef BSP_USING_PC_SIMULATOR
INIT_APP_EXPORT(init_ble_touch_screen_func);
    #endif
#endif // HID_TOUCHSCREEN

/**********************HID Test Commands (MSH)
 * ****************************************************/

#ifdef HIDS_TEST

    #ifdef HID_KEYBOARD
static rt_err_t test_hids_key(int argc, char **argv)
{
    if (argc < 3)
    {
        LOG_D("usage: test_hids key [p|r] ");
    }
    else
    {
        uint8_t key = (uint8_t)atoi(&argv[1][0]);
        if (argv[2][0] == 'p')
        {
            LOG_D("Set key %d", key);
            HID_KEY_SET(key);
        }
        else
            HID_KEY_CLEAR(key);
        HID_KEY_SEND();
    }
    return 0;
}
FINSH_FUNCTION_EXPORT(test_hids_key, Test HIDS Key);
MSH_CMD_EXPORT(test_hids_key, Test HIDS Key);

static rt_err_t test_hids_key_ctrl(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_D("usage: test_hids_ctrl type [code]");
        return 0;
    }

    if (strcmp(argv[1], "copy") == 0)
    {
        LOG_D("Set key Control + C");
        HID_KEY_SET(KEY_CTRL);
        HID_KEY_SEND();
        rt_thread_mdelay(200);
        HID_KEY_SET(KEY_C);
        HID_KEY_SEND();
        rt_thread_mdelay(200);
        HID_KEY_CLEAR(KEY_C);
        HID_KEY_SEND();
        rt_thread_mdelay(200);
        HID_KEY_CLEAR(KEY_CTRL);
        HID_KEY_SEND();
    }
    else if (strcmp(argv[1], "paste") == 0)
    {
        LOG_D("Set key Control + V");
        HID_KEY_SET(KEY_CTRL);
        HID_KEY_SEND();
        rt_thread_mdelay(200);
        HID_KEY_SET(KEY_V);
        HID_KEY_SEND();
        rt_thread_mdelay(200);
        HID_KEY_CLEAR(KEY_V);
        HID_KEY_SEND();
        rt_thread_mdelay(200);
        HID_KEY_CLEAR(KEY_CTRL);
        HID_KEY_SEND();
    }
    else if (argc >= 3)
    {
        if (strcmp(argv[1], "num") == 0)
        {
            uint8_t num = (uint8_t)atoi(&argv[2][0]);
            if (num < 1 || num > 9)
            {
                LOG_W("Invalid num %d", num);
                return 0;
            }
            LOG_D("Set key %d", num);
            uint8_t code = KEY_1 + num - 1;
            HID_KEY_SET(code);
            HID_KEY_SEND();
            rt_thread_mdelay(200);
            HID_KEY_CLEAR(code);
            HID_KEY_SEND();
        }
        else if (strcmp(argv[1], "letter") == 0)
        {
            uint8_t letter = (uint8_t)argv[2][0];
            if (letter < 'a' || letter > 'z')
            {
                LOG_W("Invalid letter %c", letter);
                return 0;
            }
            LOG_D("Set key %c", letter);
            uint8_t code = letter - 'a' + 0x04;
            HID_KEY_SET(code);
            HID_KEY_SEND();
            rt_thread_mdelay(200);
            HID_KEY_CLEAR(code);
            HID_KEY_SEND();
        }
    }
    return 0;
}
FINSH_FUNCTION_EXPORT(test_hids_key_ctrl, Test HIDS Key Control);
MSH_CMD_EXPORT(test_hids_key_ctrl, Test HIDS Key Control);
    #endif // HID_KEYBOARD

    #ifdef HID_CONSUMER
static rt_err_t test_hids_consumer(int argc, char **argv)
{
    if (argc < 3)
    {
        LOG_D("usage: test_hids_consumer key [p|r] ");
    }
    else
    {
        uint8_t key = (uint8_t)atoi(&argv[1][0]);
        if (argv[2][0] == 'p')
        {
            LOG_D("Set key %d", key);
            hid_consume_state_key_set_bit(key);
        }
        else
            hid_consume_state_key_clear_bit(key);
        consumer_report_send((uint8_t *)&hid_consume_state,
                             sizeof(hid_consume_state));
    }
    return 0;
}
FINSH_FUNCTION_EXPORT(test_hids_consumer, Test HIDS Consumer);
MSH_CMD_EXPORT(test_hids_consumer, Test HIDS Consumer);

static rt_err_t test_hids_consumer_ctrl(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_D("usage: test_hids_ctrl key ");
    }
    else if (strcmp(argv[1], "play") == 0)
    {
        LOG_D("Set key play");
        play_pause_through_hid();
    }
    else if (strcmp(argv[1], "play_prev") == 0)
    {
        LOG_D("Set key play_prev");
        play_prev_through_hid();
    }
    else if (strcmp(argv[1], "play_next") == 0)
    {
        LOG_D("Set key play_next");
        play_next_through_hid();
    }
    else if (strcmp(argv[1], "vol_up") == 0)
    {
        LOG_D("Set key vol_up");
        volume_up_through_hid();
    }
    else if (strcmp(argv[1], "vol_down") == 0)
    {
        LOG_D("Set key vol_down");
        volume_down_through_hid();
    }
    else if (strcmp(argv[1], "forward") == 0)
    {
        LOG_D("Set key forward");
        hid_consume_state_key_set_bit(HIDS_CTRL_FWD);
        consumer_report_send((uint8_t *)&hid_consume_state,
                             sizeof(hid_consume_state));
        rt_thread_mdelay(200);
        hid_consume_state_key_clear_bit(HIDS_CTRL_FWD);
        consumer_report_send((uint8_t *)&hid_consume_state,
                             sizeof(hid_consume_state));
    }
    else if (strcmp(argv[1], "back") == 0)
    {
        LOG_D("Set key back");
        HID_CONSUMER_GoBack();
    }
    else if (strcmp(argv[1], "home") == 0)
    {
        LOG_D("Set key home");
        HID_CONSUMER_GoHome();
    }
    else
    {
        LOG_D("usage: test_hids_ctrl "
              "play|play_prev|play_next|vol_up|vol_down|forward|back|home");
    }
    return 0;
}
FINSH_FUNCTION_EXPORT(test_hids_consumer_ctrl, Test HIDS Consumer Control);
MSH_CMD_EXPORT(test_hids_consumer_ctrl, Test HIDS Consumer Control);
    #endif // HID_CONSUMER

    #ifdef HID_MOUSE
static rt_err_t test_hids_mouse_ctrl(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_D("usage: test_hids_mouse_ctrl move|wheel|lc|rc");
        return 0;
    }

    if (strcmp(argv[1], "move") == 0)
    {
        if (argc < 4)
        {
            LOG_D("usage: test_hids_mouse_ctrl move x y");
            return 0;
        }
        int8_t x = atoi(argv[2]);
        int8_t y = atoi(argv[3]);
        BLE_HID_Mouse_Move(x, y);
    }
    else if (strcmp(argv[1], "wheel") == 0)
    {
        if (argc < 3)
        {
            LOG_D("usage: test_hids_mouse_ctrl wheel delta");
            return 0;
        }
        int8_t wheel = atoi(argv[2]);
        BLE_HID_Mouse_Wheel_Scroll(wheel);
    }
    else if (strcmp(argv[1], "lc") == 0)
    {
        BLE_HID_Mouse_LeftClick();
    }
    else if (strcmp(argv[1], "rc") == 0)
    {
        BLE_HID_Mouse_RightClick();
    }
    return 0;
}
FINSH_FUNCTION_EXPORT(test_hids_mouse_ctrl, Test HIDS Mouse Control);
MSH_CMD_EXPORT(test_hids_mouse_ctrl, Test HIDS Mouse Control);
    #endif // HID_MOUSE

    #ifdef HID_TOUCHSCREEN
static rt_err_t test_hids_touchscreen_ctrl(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_D("usage: test_hids_touchscreen_ctrl "
              "press|release|slide_ver|slide_hor");
        return 0;
    }

    if (strcmp(argv[1], "press") == 0)
    {
        if (argc < 4)
        {
            LOG_D("usage: test_hids_touchscreen_ctrl press x y");
            return 0;
        }
        uint16_t x = atoi(argv[2]);
        uint16_t y = atoi(argv[3]);
        LOG_D("Set touch press %d %d", x, y);
        hid_touch_state_set(1, x, y);
        touchscreen_report_send((uint8_t *)&hid_touch_state,
                                sizeof(hid_touch_state));
    }
    else if (strcmp(argv[1], "release") == 0)
    {
        LOG_D("Set touch release");
        hid_touch_state_clear();
        touchscreen_report_send((uint8_t *)&hid_touch_state,
                                sizeof(hid_touch_state));
    }
    else if (strcmp(argv[1], "slide_ver") == 0)
    {
        if (argc < 4)
        {
            LOG_D("usage: test_hids_touchscreen_ctrl slide_ver y_start y_end");
            return 0;
        }
        uint16_t y_start = atoi(argv[2]);
        uint16_t y_end = atoi(argv[3]);
        LOG_D("Set touch slide vertical");
        hid_touch_state_set(1, 64, y_start);
        touchscreen_report_send((uint8_t *)&hid_touch_state,
                                sizeof(hid_touch_state));
        rt_thread_mdelay(100);
        hid_touch_state_set(1, 64, y_end);
        touchscreen_report_send((uint8_t *)&hid_touch_state,
                                sizeof(hid_touch_state));
        rt_thread_mdelay(100);
        hid_touch_state_set(0, 64, y_end);
        touchscreen_report_send((uint8_t *)&hid_touch_state,
                                sizeof(hid_touch_state));
        hid_touch_state_clear();
    }
    else if (strcmp(argv[1], "slide_hor") == 0)
    {
        if (argc < 4)
        {
            LOG_D("usage: test_hids_touchscreen_ctrl slide_hor x_start x_end");
            return 0;
        }
        uint16_t x_start = atoi(argv[2]);
        uint16_t x_end = atoi(argv[3]);
        LOG_D("Set touch slide horizontal");
        hid_touch_state_set(1, x_start, 64);
        touchscreen_report_send((uint8_t *)&hid_touch_state,
                                sizeof(hid_touch_state));
        rt_thread_mdelay(100);
        hid_touch_state_set(1, x_end, 64);
        touchscreen_report_send((uint8_t *)&hid_touch_state,
                                sizeof(hid_touch_state));
        rt_thread_mdelay(100);
        hid_touch_state_set(0, x_end, 64);
        touchscreen_report_send((uint8_t *)&hid_touch_state,
                                sizeof(hid_touch_state));
        hid_touch_state_clear();
    }
    return 0;
}
FINSH_FUNCTION_EXPORT(test_hids_touchscreen_ctrl,
                      Test HIDS Touchscreen Control);
MSH_CMD_EXPORT(test_hids_touchscreen_ctrl, Test HIDS Touchscreen Control);
    #endif // HID_TOUCHSCREEN

#endif // HIDS_TEST
