#include "ble_hid.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "bf0_hal_hlp.h"
#include "bf0_sibles.h"
#include "bf0_ble_gap.h"
#include "bloc_control.h"

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

#ifdef HID_TOUCHSCREEN
static struct hids_report input_touchscreen = {
    .id = REPORT_ID_TOUCH,
    .type = HIDS_INPUT,
};
static struct touch_state hid_touch_state;
#endif

#ifdef HID_TOUCHPAD
static struct hids_report input_touchpad = {
    .id = REPORT_ID_TOUCHPAD,
    .type = HIDS_INPUT,
};
static struct touchpad_state hid_touchpad_state;
#endif

static uint8_t ctrl_point;
static uint8_t g_conn_idx = 0;
static ble_hid_data_t *g_hid_data = NULL;

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

#ifdef HID_TOUCHPAD
    0x05,
    0x0D, // Usage Page (Digitizers)
    0x09,
    0x05, // Usage (Touch Pad)
    0xA1,
    0x01, // Collection (Application)
    0x85,
    REPORT_ID_TOUCHPAD, // Report ID (Touchpad)
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

#ifdef HID_TOUCHPAD
    // HID Report (Touchpad)
    [HIDS_TOUCHPAD_IDX_REPORT] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0,
                                  0},
    [HIDS_TOUCHPAD_IDX_REPORT_VAL] =
        {ATT_CHAR_REPORT,
         PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) |
             PERM(WRITE_COMMAND, ENABLE) | PERM(NTF, ENABLE) | PERM(WP, UNAUTH),
         PERM(UUID_LEN, UUID_16) | PERM(RI, ENABLE), 8},
    [HIDS_TOUCHPAD_IDX_REPORT_NTF_CFG] = {ATT_DESC_CLIENT_CHAR_CFG,
                                          PERM(RD, ENABLE) |
                                              PERM(WRITE_REQ, ENABLE) |
                                              PERM(WP, UNAUTH),
                                          PERM(RI, ENABLE), 2},
    [HIDS_TOUCHPAD_IDX_REPORT_REF] = {ATT_DESC_REPORT_REF, PERM(RD, ENABLE),
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
#ifdef HID_TOUCHPAD
    case HIDS_TOUCHPAD_IDX_REPORT_VAL:
        break;
    case HIDS_TOUCHPAD_IDX_REPORT_REF:
    {
        ret_val = (uint8_t *)&input_touchpad;
        *len = sizeof(input_touchpad);
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
#ifdef HID_TOUCHPAD
    case HIDS_TOUCHPAD_IDX_REPORT_NTF_CFG:
        g_hid_data->is_touchpad_config_on = *(para->value);
        LOG_I("TOUCHPAD CCCD %d", g_hid_data->is_touchpad_config_on);
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

#ifdef HID_TOUCHPAD
static int hid_touchpad_state_set(uint8_t touch, uint16_t x, uint16_t y)
{
    hid_touchpad_state.touch = touch;
    hid_touchpad_state.x = x;
    hid_touchpad_state.y = y;
    return 0;
}

static int hid_touchpad_state_clear(void)
{
    hid_touchpad_state.touch = 0;
    hid_touchpad_state.x = 0;
    hid_touchpad_state.y = 0;
    return 0;
}
#endif

/**********************HID Report Send Functions
 * ****************************************************/

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
        return;

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

#ifdef HID_TOUCHPAD
static void touchpad_report_send(uint8_t *key_val, uint16_t key_val_len)
{
    if (!g_hid_data || !g_hid_data->is_touchpad_config_on)
        return;

    sibles_value_t value;
    value.hdl = g_hid_data->srv_handle;
    value.idx = HIDS_TOUCHPAD_IDX_REPORT_VAL;
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

void ble_hid_reset_on_disconnect(void)
{
    if (!g_hid_data)
        return;

#ifdef HID_CONSUMER
    g_hid_data->is_consumer_config_on = 0;
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
#ifdef HID_TOUCHPAD
    g_hid_data->is_touchpad_config_on = 0;
#endif
}

/**********************HID Input Functions - Mouse
 * ****************************************************/

#ifdef HID_MOUSE
void BLE_HID_Mouse_Move(int8_t dx, int8_t dy)
{
    hid_mouse_state_set(hid_mouse_state.buttons, dx, dy, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
}

void BLE_HID_Mouse_Wheel_Scroll(int8_t delta)
{
    hid_mouse_state_set(hid_mouse_state.buttons, 0, 0, delta, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
}

void BLE_HID_Mouse_Pan_Scroll(int8_t delta)
{
    hid_mouse_state_set(hid_mouse_state.buttons, 0, 0, 0, delta);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
}

static void BLE_HID_Mouse_LeftPress(void)
{
    hid_mouse_state_set(1, 0, 0, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
}

static void BLE_HID_Mouse_LeftRelease(void)
{
    hid_mouse_state_set(0, 0, 0, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
    hid_mouse_state_clear();
}

void BLE_HID_Mouse_LeftClick(void)
{
    BLE_HID_Mouse_LeftPress();
    rt_thread_mdelay(200);
    BLE_HID_Mouse_LeftRelease();
}

static void BLE_HID_Mouse_RightPress(void)
{
    hid_mouse_state_set(2, 0, 0, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
    hid_mouse_state_clear();
}

static void BLE_HID_Mouse_RightRelease(void)
{
    hid_mouse_state_set(0, 0, 0, 0, 0);
    mouse_report_send((uint8_t *)&hid_mouse_state, sizeof(hid_mouse_state));
    hid_mouse_state_clear();
}

void BLE_HID_Mouse_RightClick(void)
{
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
void play_pause_through_hid(void)
{
    hid_consume_state_key_set_bit(HIDS_CTRL_PLAY);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
    rt_thread_mdelay(200);
    hid_consume_state_key_clear_bit(HIDS_CTRL_PLAY);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
}

void play_next_through_hid(void)
{
    hid_consume_state_key_set_bit(HIDS_CTRL_SCAN_NEX);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
    rt_thread_mdelay(200);
    hid_consume_state_key_clear_bit(HIDS_CTRL_SCAN_NEX);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
}

void play_prev_through_hid(void)
{
    hid_consume_state_key_set_bit(HIDS_CTRL_SCAN_PREV);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
    rt_thread_mdelay(200);
    hid_consume_state_key_clear_bit(HIDS_CTRL_SCAN_PREV);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
}

void volume_up_through_hid(void)
{
    hid_consume_state_key_set_bit(HIDS_CTRL_VOL_UP);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
    rt_thread_mdelay(50);
    hid_consume_state_key_clear_bit(HIDS_CTRL_VOL_UP);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
}

void volume_down_through_hid(void)
{
    hid_consume_state_key_set_bit(HIDS_CTRL_VOL_DOWN);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
    rt_thread_mdelay(50);
    hid_consume_state_key_clear_bit(HIDS_CTRL_VOL_DOWN);
    consumer_report_send((uint8_t *)&hid_consume_state,
                         sizeof(hid_consume_state));
}

void HID_CONSUMER_GoBack(void)
{
    LOG_D("Set key back");
    BLE_HID_Mouse_BackPress();
    rt_thread_mdelay(200);
    BLE_HID_Mouse_BackRelease();
}

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
        rt_thread_mdelay(200);
        HID_KEY_SET(KEY_TAB);
        HID_KEY_SEND();
        rt_thread_mdelay(200);
        HID_KEY_CLEAR(KEY_TAB);
        HID_KEY_SEND();
    }
    else
    {
        HID_KEY_CLEAR(KEY_MOD_LMETA);
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
    control_provider.ble_hid_keyboard_multitask = BLE_HID_Keyboard_Multitask;
    control_provider.ble_hid_keyboard_shift = BLE_HID_keyboard_Shift;
    control_provider.ble_hid_keyboard_go_home = BLE_HID_keyboard_go_home;
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

/**********************HID Input Functions - Touchpad
 * ****************************************************/

#ifdef HID_TOUCHPAD
void BLE_HID_Touchpad_Press(uint16_t x, uint16_t y)
{
    hid_touchpad_state_set(1, x, y);
    touchpad_report_send((uint8_t *)&hid_touchpad_state,
                         sizeof(hid_touchpad_state));
}

void BLE_HID_Touchpad_Release(uint16_t x, uint16_t y)
{
    hid_touchpad_state_set(0, x, y);
    touchpad_report_send((uint8_t *)&hid_touchpad_state,
                         sizeof(hid_touchpad_state));
}

static int init_ble_touchpad_func(void)
{
    control_provider.ble_hid_touchpad_press = BLE_HID_Touchpad_Press;
    control_provider.ble_hid_touchpad_release = BLE_HID_Touchpad_Release;
    return 0;
}
    #ifndef BSP_USING_PC_SIMULATOR
INIT_APP_EXPORT(init_ble_touchpad_func);
    #endif
#endif // HID_TOUCHPAD

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

    #ifdef HID_TOUCHPAD
static rt_err_t test_hids_touchpad_ctrl(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_D(
            "usage: test_hids_touchpad_ctrl press|release|slide_ver|slide_hor");
        return 0;
    }

    if (strcmp(argv[1], "press") == 0)
    {
        if (argc < 4)
        {
            LOG_D("usage: test_hids_touchpad_ctrl press x y");
            return 0;
        }
        uint16_t x = atoi(argv[2]);
        uint16_t y = atoi(argv[3]);
        LOG_D("Set touchpad press %d %d", x, y);
        hid_touchpad_state_set(1, x, y);
        touchpad_report_send((uint8_t *)&hid_touchpad_state,
                             sizeof(hid_touchpad_state));
    }
    else if (strcmp(argv[1], "release") == 0)
    {
        LOG_D("Set touchpad release");
        hid_touchpad_state_clear();
        touchpad_report_send((uint8_t *)&hid_touchpad_state,
                             sizeof(hid_touchpad_state));
    }
    // Add slide_ver and slide_hor similar to touchscreen
    return 0;
}
FINSH_FUNCTION_EXPORT(test_hids_touchpad_ctrl, Test HIDS Touchpad Control);
MSH_CMD_EXPORT(test_hids_touchpad_ctrl, Test HIDS Touchpad Control);
    #endif // HID_TOUCHPAD

#endif // HIDS_TEST
