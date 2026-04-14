#ifndef __BLE_HID_H__
#define __BLE_HID_H__

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "bf0_sibles.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HID Device Type Selection */
#define HID_MOUSE
#define HID_KEYBOARD
#define HID_CONSUMER
#define HID_TELEPHONY
// #define HID_TOUCHSCREEN
// #define HID_TOUCHPAD

/* HID Report IDs */
typedef enum
{
#ifdef HID_MOUSE
    REPORT_ID_MOUSE = 0x01,
#endif
#ifdef HID_KEYBOARD
    REPORT_ID_KEYBOARD = 0x02,
#endif
#ifdef HID_CONSUMER
    REPORT_ID_CONSUMER = 0x03,
#endif
#ifdef HID_TOUCHSCREEN
    REPORT_ID_TOUCH = 0x04,
#endif
#ifdef HID_TOUCHPAD
    REPORT_ID_TOUCHPAD = 0x05,
#endif
#ifdef HID_TELEPHONY
    REPORT_ID_TELEPHONY = 0x06,
#endif
} report_id_t;

/* HID Info Structure */
struct hids_info
{
    uint16_t version; /* version number of base USB HID Specification */
    uint8_t code;     /* country HID Device hardware is localized for. */
    uint8_t flags;
}
#ifndef BSP_USING_PC_SIMULATOR
__packed
#endif
;

/* HID Report Structure */
struct hids_report
{
    uint8_t id;   /* report id */
    uint8_t type; /* report type */
}
#ifndef BSP_USING_PC_SIMULATOR
__packed
#endif
;

#ifdef HID_KEYBOARD
/* Keyboard State Structure */
#define KEY_CTRL_CODE_MIN 224     /* Control key codes - required 8 of them */
#define KEY_CTRL_CODE_MAX 231     /* Control key codes - required 8 of them */
#define KEYBOARD_KEY_CODE_MIN 0   /* Normal key codes */
#define KEYBOARD_KEY_CODE_MAX 101 /* Normal key codes */
#define KEY_PRESS_MAX 6           /* Maximum number of non-control keys pressed simultaneously*/
#define INPUT_REPORT_KEYS_MAX_LEN (1 + 1 + KEY_PRESS_MAX)

struct keyboard_state
{
    uint8_t ctrl_keys_state; /* Current keys state */
    uint8_t reserved;
    uint8_t keys_state[KEY_PRESS_MAX];
};
#endif

#ifdef HID_CONSUMER
/* Consumer Control Key Codes */
enum
{
    HIDS_CTRL_PLAY,
    HIDS_CTRL_CONFG,
    HIDS_CTRL_SCAN_NEX,
    HIDS_CTRL_SCAN_PREV,
    HIDS_CTRL_VOL_DOWN,
    HIDS_CTRL_VOL_UP,
    HIDS_CTRL_FWD,
    HIDS_CTRL_BACK,
};

#define CONSUMER_KEY_CODE_MIN HIDS_CTRL_PLAY
#define CONSUMER_KEY_CODE_MAX HIDS_CTRL_BACK

struct consume_key_state
{
    uint8_t key_state;
};
#endif

#ifdef HID_TELEPHONY
/* Telephony Usage Page (0x0B) key codes (bit positions in report) */
enum
{
    HIDS_TEL_HOOK_SWITCH = 0, /* Usage 0x20 */
    HIDS_TEL_DROP = 1,        /* Usage 0x26 */
};

struct telephony_key_state
{
    uint8_t key_state;
};
#endif

#ifdef HID_MOUSE
/* Mouse State Structure */
struct mouse_state
{
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheel;
    int8_t pan;
};
#endif

#ifdef HID_TOUCHSCREEN
/* Touchscreen State Structure */
struct touch_state
{
    uint8_t touch;
    uint16_t x;
    uint16_t y;
};
#endif

#ifdef HID_TOUCHPAD
/* Touchpad State Structure */
struct touchpad_state
{
    uint8_t touch;
    uint16_t x;
    uint16_t y;
};
#endif

/* HID Service Data Structure */
typedef struct
{
    sibles_hdl srv_handle;
    uint8_t is_audio_subscribed;
#ifdef HID_MOUSE
    uint8_t is_mouse_config_on;
#endif
#ifdef HID_KEYBOARD
    uint8_t is_keyboard_config_on;
#endif
#ifdef HID_CONSUMER
    uint8_t is_consumer_config_on;
#endif
#ifdef HID_TELEPHONY
    uint8_t is_telephony_config_on;
#endif
#ifdef HID_TOUCHSCREEN
    uint8_t is_touchscreen_config_on;
#endif
#ifdef HID_TOUCHPAD
    uint8_t is_touchpad_config_on;
#endif
} ble_hid_data_t;

/* HID Service Attributes Indexes */
enum
{
    HIDS_IDX_SVC,
    HIDS_IDX_INFO_CHAR,
    HIDS_IDX_INFO_VAL,
    HIDS_IDX_REPORT_MAP,
    HIDS_IDX_REPORT_MAP_VAL,
#ifdef HID_MOUSE
    HIDS_MOUSE_IDX_REPORT,
    HIDS_MOUSE_IDX_REPORT_VAL,
    HIDS_MOUSE_IDX_REPORT_REF,
    HIDS_MOUSE_IDX_REPORT_NTF_CFG,
#endif
#ifdef HID_KEYBOARD
    HIDS_KEYBOARD_IDX_REPORT,
    HIDS_KEYBOARD_IDX_REPORT_VAL,
    HIDS_KEYBOARD_IDX_REPORT_REF,
    HIDS_KEYBOARD_IDX_REPORT_NTF_CFG,
#endif
#ifdef HID_CONSUMER
    HIDS_CONSUMER_IDX_REPORT,
    HIDS_CONSUMER_IDX_REPORT_VAL,
    HIDS_CONSUMER_IDX_REPORT_REF,
    HIDS_CONSUMER_IDX_REPORT_NTF_CFG,
#endif
#ifdef HID_TELEPHONY
    HIDS_TELEPHONY_IDX_REPORT,
    HIDS_TELEPHONY_IDX_REPORT_VAL,
    HIDS_TELEPHONY_IDX_REPORT_REF,
    HIDS_TELEPHONY_IDX_REPORT_NTF_CFG,
#endif
#ifdef HID_TOUCHSCREEN
    HIDS_TOUCHSCREEN_IDX_REPORT,
    HIDS_TOUCHSCREEN_IDX_REPORT_VAL,
    HIDS_TOUCHSCREEN_IDX_REPORT_REF,
    HIDS_TOUCHSCREEN_IDX_REPORT_NTF_CFG,
#endif
#ifdef HID_TOUCHPAD
    HIDS_TOUCHPAD_IDX_REPORT,
    HIDS_TOUCHPAD_IDX_REPORT_VAL,
    HIDS_TOUCHPAD_IDX_REPORT_REF,
    HIDS_TOUCHPAD_IDX_REPORT_NTF_CFG,
#endif
    HIDS_IDX_CTRL,
    HIDS_IDX_CTRL_VAL,
    HDIS_ATT_NB
};

/* Public Function Declarations */

/**
 * @brief Initialize HID service
 * @param hid_data Pointer to HID data structure
 */
void ble_hid_service_init(ble_hid_data_t *hid_data);

/**
 * @brief Get HID data structure pointer
 * @return Pointer to HID data structure
 */
ble_hid_data_t* ble_hid_get_data(void);

/**
 * @brief Set connection index for HID service
 * @param conn_idx Connection index
 */
void ble_hid_set_conn_idx(uint8_t conn_idx);

/**
 * @brief Get current connection index for HID service
 * @return Current connection index
 */
uint8_t ble_hid_get_conn_idx(void);

/**
 * @brief Reset HID configuration on disconnect
 */
void ble_hid_reset_on_disconnect(void);

#ifdef HID_MOUSE
/* Mouse Control Functions */
void BLE_HID_Mouse_Move(int8_t dx, int8_t dy);
void BLE_HID_Mouse_Wheel_Scroll(int8_t delta);
void BLE_HID_Mouse_Pan_Scroll(int8_t delta);
void BLE_HID_Mouse_LeftClick(void);
void BLE_HID_Mouse_RightClick(void);
#endif

#ifdef HID_KEYBOARD
/* Keyboard Control Functions */
void BLE_HID_Send_String(const char *str);
void BLE_HID_Keyboard_Multitask(bool state);
void BLE_HID_keyboard_Shift(bool state);
void BLE_HID_keyboard_go_home(void);
void BLE_HID_computer_zoom(uint8_t device, bool state);
void BLE_HID_ZOOM_ESC(void);
#endif

#ifdef HID_CONSUMER
/* Consumer Control Functions */
void play_pause_through_hid(void);
void play_next_through_hid(void);
void play_prev_through_hid(void);
void volume_up_through_hid(void);
void volume_down_through_hid(void);
void HID_CONSUMER_GoBack(void);
void HID_CONSUMER_GoHome(void);
#endif

#ifdef HID_TELEPHONY
/* Telephony Control Functions */
void hang_up_through_hid(void);
void hook_switch_through_hid(void);
#endif

#ifdef HID_TOUCHSCREEN
/* Touchscreen Control Functions */
void BLE_HID_Touchscreen_Press(uint16_t x, uint16_t y);
void BLE_HID_Touchscreen_Release(uint16_t x, uint16_t y);
#endif

#ifdef HID_TOUCHPAD
/* Touchpad Control Functions */
void BLE_HID_Touchpad_Press(uint16_t x, uint16_t y);
void BLE_HID_Touchpad_Release(uint16_t x, uint16_t y);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BLE_HID_H__ */
