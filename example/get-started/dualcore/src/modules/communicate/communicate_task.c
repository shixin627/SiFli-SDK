/**
 *****************************************************************************************
 *     Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 *****************************************************************************************
 * @file      communicate_task.c
 * @brief     Direct-send API: build L2 packets and write to BLE notify.
 *
 *            All commu_send_* functions share three primitives:
 *              - commu_send_status / _empty / _string / _blob
 *            which centralize connection guard, header build, and notify.
 *****************************************************************************************
 */

#include <rtthread.h>
#include <string.h>
#include "communicate_protocol.h"
#include "communicate_parse.h"
#include "communicate_parse_skailink.h"
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "gesture_model_loader.h"

#ifdef BSP_USING_BLOC
    #include "bloc_control.h"
    #include "bloc_v2t.h"
    #include "bloc_notification.h"
    #include "bloc_skaiwalk.h"
    #include "bloc_filesystem.h"
    #include "bloc_peripheral.h"
#endif

#ifndef BSP_USING_PC_SIMULATOR
    #include "audio_codec_i2s.h"
#endif

#define DBG_TAG "communicate.task"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

/*============================================================================*
 *                              Constants
 *============================================================================*/

/* Max payload bytes carried in one un-fragmented L2 frame (BLE_APP_CHAR_MAX_LEN
   minus the 5-byte L2 header). Larger payloads must use skaiwatch_ble_send_l2
   directly so the fragmenter handles splitting. */
#define MAX_PACKET_PAYLOAD_SIZE 507

#define BOND_SUCCESS  0x00
#define BOND_FAIL     0x01
#define LOGIN_SUCCESS 0x00
#define LOGIN_FAIL    0x01

/*============================================================================*
 *                              Internal helpers
 *============================================================================*/

/* Big-endian field writers. write_be16 lives in communicate_parse.h; the wider
   variants are local since no other file currently needs them. */
static inline void write_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}
static inline void write_be40(uint8_t *p, uint64_t v)
{
    p[0] = (uint8_t)(v >> 32);
    p[1] = (uint8_t)(v >> 24);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 8);
    p[4] = (uint8_t)v;
}

/* Write the 5-byte L2 header into buf. The phone side masks the length field
   with 0x1FF, so writing a full byte in buf[3] is harmless for length < 512. */
static inline void l2_write_header(uint8_t *buf, uint8_t cmd_id, uint8_t key,
                                    uint16_t length)
{
    buf[0] = cmd_id;
    buf[1] = L2_HEADER_VERSION;
    buf[2] = key;
    buf[3] = (uint8_t)((length >> 8) & 0xFF);
    buf[4] = (uint8_t)(length & 0xFF);
}

/* Connection guard shared by all direct-send APIs. */
static inline bool commu_can_send(void)
{
    return SkaiWatchSys.connected_to_phone;
}

/* OTA path uses the same connection guard. */
static inline bool commu_can_send_ota(void)
{
    return SkaiWatchSys.gap_conn_state == GAP_CONN_STATE_CONNECTED &&
           SkaiWatchSys.connected_to_phone;
}

/* Header-only frame (no payload). */
static bool commu_send_empty(uint8_t cmd_id, uint8_t key)
{
    if (!commu_can_send()) return false;
    uint8_t buf[L2_FIRST_VALUE_POS];
    l2_write_header(buf, cmd_id, key, 0);
    return skaiwatch_ble_notify(buf, sizeof(buf));
}

/* Single-byte status frame (header + 1 byte). Small fixed buffer so the
   common case avoids the MAX_PACKET_PAYLOAD_SIZE stack cost. */
static bool commu_send_status(uint8_t cmd_id, uint8_t key, uint8_t status)
{
    if (!commu_can_send()) return false;
    uint8_t buf[L2_FIRST_VALUE_POS + 1];
    l2_write_header(buf, cmd_id, key, 1);
    buf[L2_FIRST_VALUE_POS] = status;
    return skaiwatch_ble_notify(buf, sizeof(buf));
}

/* Variable-length payload up to MAX_PACKET_PAYLOAD_SIZE. */
static bool commu_send_blob(uint8_t cmd_id, uint8_t key,
                             const void *payload, uint16_t length)
{
    if (!commu_can_send()) return false;
    if (length > MAX_PACKET_PAYLOAD_SIZE) return false;

    uint8_t buf[L2_FIRST_VALUE_POS + MAX_PACKET_PAYLOAD_SIZE];
    l2_write_header(buf, cmd_id, key, length);
    if (length > 0 && payload != NULL)
    {
        memcpy(buf + L2_FIRST_VALUE_POS, payload, length);
    }
    return skaiwatch_ble_notify(buf, (uint16_t)(L2_FIRST_VALUE_POS + length));
}

/* String payload via strlen; rejects NULL. */
static bool commu_send_string(uint8_t cmd_id, uint8_t key, const char *s)
{
    if (s == NULL) return false;
    return commu_send_blob(cmd_id, key, s, (uint16_t)strlen(s));
}

/* ADR-0008 E7: watch→phone active-target selection (sent on tileview page
   change). UNVERIFIED — build-verify rt_snprintf is available (else use
   rt_sprintf with a bounds guard). */
bool commu_send_active_device(const char *device_id)
{
    if (device_id == NULL) device_id = "";
    /* Shared active-target de-dup. Both the LEFT page (instruction_list, target
       = the directly-connected primary) and the RIGHT page (device_pager, target
       = the selected non-primary device) assert the active target; sending only
       on an actual CHANGE keeps a single source of truth across both pages, so
       switching pages re-asserts cleanly without spamming identical frames. */
    static char s_last_active[SYNCED_DEVICE_ID_LEN];
    static bool s_last_active_valid = false;
    if (s_last_active_valid &&
        strncmp(s_last_active, device_id, sizeof(s_last_active)) == 0)
        return true; /* unchanged → no-op success */
    char json[16 + SYNCED_DEVICE_ID_LEN];
    int n = rt_snprintf(json, sizeof(json), "{\"device_id\":\"%s\"}", device_id);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_ACTIVE_SELECT, json);
    if (ok)
    {
        strncpy(s_last_active, device_id, sizeof(s_last_active) - 1);
        s_last_active[sizeof(s_last_active) - 1] = '\0';
        s_last_active_valid = true;
    }
    return ok;
}

/*============================================================================*
 *                              Bond / Login
 *============================================================================*/

bool commu_send_bond_success(void)  { return commu_send_status(BOND_COMMAND_ID, KEY_BOND_RESPOSE,   BOND_SUCCESS); }
bool commu_send_bond_fail(void)     { return commu_send_status(BOND_COMMAND_ID, KEY_BOND_RESPOSE,   BOND_FAIL); }
bool commu_send_login_success(void) { return commu_send_status(BOND_COMMAND_ID, KEY_LOGIN_RESPONSE, LOGIN_SUCCESS); }
bool commu_send_login_fail(void)    { return commu_send_status(BOND_COMMAND_ID, KEY_LOGIN_RESPONSE, LOGIN_FAIL); }

/*============================================================================*
 *                              Settings
 *============================================================================*/

/* Side effect (preserved from legacy): scanning the alarm list also prunes
   "fired once" entries (day_repeat_flag == 0 && reserved == 0) by decrementing
   SkaiWatchSys.alarm_num. The phone receives only the surviving items, and
   local state is left consistent with what was sent. Don't move the prune
   out without auditing callers — they rely on this combined behavior. */
bool commu_send_alarm_settings(void)
{
    if (!commu_can_send()) return false;

    uint8_t buf[L2_FIRST_VALUE_POS + 5 * 16];
    uint8_t alarm_item_count = 0;
    uint8_t alarm_num = SkaiWatchSys.alarm_num;

    for (uint8_t i = 0; i < alarm_num; i++)
    {
        const T_ALARM *a = &SkaiWatchSys.alarms[i];
        if (a->alarm.day_repeat_flag == 0 && a->alarm.reserved == 0)
        {
            SkaiWatchSys.alarm_num--;
            LOG_I("delete alarm once item...");
            continue;
        }
        write_be40(buf + L2_FIRST_VALUE_POS + alarm_item_count * 5, a->data);
        alarm_item_count++;
    }

    uint16_t payload_len = (uint16_t)(alarm_item_count * 5);
    l2_write_header(buf, SET_CONFIG_COMMAND_ID, KEY_RETURN_ALARM_SETTINGS, payload_len);
    return skaiwatch_ble_notify(buf, (uint16_t)(L2_FIRST_VALUE_POS + payload_len));
}

bool commu_send_hour_format(void)              { return commu_send_status(SET_CONFIG_COMMAND_ID, KEY_HOUR_FORMAT_RETURN,       SkaiWatchSys.flag_field.hour_format); }
bool commu_send_distance_unit(void)            { return commu_send_status(SET_CONFIG_COMMAND_ID, KEY_DISTANCE_UNIT_RETURN,     SkaiWatchSys.flag_field.distance_unit); }
bool commu_send_oled_display_time(uint8_t t)   { return commu_send_status(SET_CONFIG_COMMAND_ID, KEY_OLED_DISPLAY_TIME_RETURN, t); }
bool commu_send_language(void)                 { return commu_send_status(SET_CONFIG_COMMAND_ID, KEY_LANGUAGE_RETURN,          SkaiWatchSys.language); }
bool commu_send_dial_change(void)              { return commu_send_status(SET_CONFIG_COMMAND_ID, KEY_DIAL_RETURN,              SkaiWatchSys.clock_status); }

/*============================================================================*
 *                              Health
 *============================================================================*/

bool commu_send_sport_data(void)
{
    return commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_RETURN_SPORTS_DATA,
                            &SkaiWatchSys.health_info_today,
                            (uint16_t)sizeof(watch_sys_heath_info_t));
}

bool commu_send_heart_data(int hr)
{
    return commu_send_status(HEALTH_DATA_COMMAND_ID, KEY_HEART_DATA_RETURN, (uint8_t)hr);
}

bool commu_send_heart_curve_sample(uint32_t timestamp, uint8_t bpm)
{
    /* Packed 5-byte wire payload {timestamp:u32 LE, bpm:u8} — explicit packing
       so there is no struct padding on the wire (the dart parser reads 4+1). */
    struct __attribute__((packed))
    {
        uint32_t timestamp;
        uint8_t bpm;
    } sample = {.timestamp = timestamp, .bpm = bpm};
    return commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_HEART_CURVE_SAMPLE,
                           &sample, (uint16_t)sizeof(sample));
}

bool commu_send_heart_curve_skip(uint32_t timestamp, uint8_t reason)
{
    /* Packed 5-byte wire payload {timestamp:u32 LE, reason:u8} — byte-identical
       shape to commu_send_heart_curve_sample so the dart parser reads 4+1 the
       same way. Marks a 5-min bucket that produced no HR point, with why. */
    struct __attribute__((packed))
    {
        uint32_t timestamp;
        uint8_t reason;
    } skip = {.timestamp = timestamp, .reason = reason};
    return commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_HEART_CURVE_SKIP,
                           &skip, (uint16_t)sizeof(skip));
}

bool commu_send_wear_diag(uint32_t ts, uint8_t evt, uint8_t status,
                          uint16_t dc_q4, uint16_t pi_e6,
                          uint16_t pi_range_e6, uint16_t imu_var_e4)
{
    /* Packed 14-byte wire payload — explicit packing so there is no struct
       padding on the wire (the dart parser reads 4+1+1+2+2+2+2 LE). */
    struct __attribute__((packed))
    {
        uint32_t ts;
        uint8_t evt;
        uint8_t status;
        uint16_t dc_q4;
        uint16_t pi_e6;
        uint16_t pi_range_e6;
        uint16_t imu_var_e4;
    } rec = {.ts = ts, .evt = evt, .status = status, .dc_q4 = dc_q4,
             .pi_e6 = pi_e6, .pi_range_e6 = pi_range_e6,
             .imu_var_e4 = imu_var_e4};
    return commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_WEAR_DIAG,
                           &rec, (uint16_t)sizeof(rec));
}

bool commu_send_sleep_data(void)
{
    return commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_RETURN_SLEEP_DATA,
                           &SkaiWatchSys.sleep_state,
                           (uint16_t)sizeof(watch_sys_sleep_state_t));
}

/*============================================================================*
 *                              Control
 *============================================================================*/

bool commu_send_phone_control_cmd(void)        { return commu_send_empty(CONTROL_COMMAND_ID, KEY_TAKE_PHOTO); }
bool commu_send_find_mobile(void)              { return commu_send_empty(CONTROL_COMMAND_ID, KEY_FIND_PHONE); }
bool commu_send_media_control(void)            { return commu_send_status(CONTROL_COMMAND_ID, KEY_PHONE_MEDIA_CONTROL, app_audio_get_control_command()); }
bool commu_send_volume_percentage(uint8_t v)   { return commu_send_status(CONTROL_COMMAND_ID, KEY_RETURN_VOLUMN, v); }

/*============================================================================*
 *                              Notification
 *============================================================================*/

bool commu_send_charge_status(void)                  { return commu_send_status(NOTIFY_COMMAND_ID, KEY_BATTERY_CHARGE_STATUS, SkaiWatchSys.charger_status); }
bool commu_send_weather_request(void)                { return commu_send_empty (NOTIFY_COMMAND_ID, KEY_REQUEST_WEATHER); }
bool commu_send_remote_input(const char *json)       { return commu_send_string(NOTIFY_COMMAND_ID, KEY_REMOTE_INPUT, json); }
bool commu_send_dismiss_notification(const char *id) { return commu_send_string(NOTIFY_COMMAND_ID, KEY_DISMISS_NOTIFICATION, id); }
bool commu_send_user_speaking_state(uint8_t s)       { return commu_send_status(NOTIFY_COMMAND_ID, KEY_USER_SPEAKING_STATE, s); }
bool commu_send_chat_with_ai(const char *json)       { return commu_send_string(NOTIFY_COMMAND_ID, KEY_RETURN_CHAT_INTENT, json); }
bool commu_send_battery_level(uint8_t level)         { return commu_send_status(NOTIFY_COMMAND_ID, KEY_BATTERY_LEVEL, level); }

bool commu_send_battery_voltage(uint16_t millivolts)
{
    if (!commu_can_send()) return false;
    uint8_t buf[L2_FIRST_VALUE_POS + 2];
    l2_write_header(buf, NOTIFY_COMMAND_ID, KEY_BATTERY_VOLTAGE, 2);
    buf[L2_FIRST_VALUE_POS]     = (uint8_t)(millivolts >> 8);
    buf[L2_FIRST_VALUE_POS + 1] = (uint8_t)(millivolts & 0xFF);
    return skaiwatch_ble_notify(buf, sizeof(buf));
}
bool commu_send_update_instruction(const char *json) { return commu_send_string(NOTIFY_COMMAND_ID, KEY_SKAI_CREATION_INSTRUCTIONS, json); }
bool commu_send_get_instruction_img(const char *id)  { return commu_send_string(NOTIFY_COMMAND_ID, KEY_SKAI_INSTRUCTION_IMAGE, id); }
bool commu_send_skaibar_selected(uint8_t idx)        { return commu_send_status(NOTIFY_COMMAND_ID, KEY_SKAIBAR_SELECTED, idx); }
bool commu_send_skaibar_committed(uint8_t idx)       { return commu_send_status(NOTIFY_COMMAND_ID, KEY_SKAIBAR_COMMITTED, idx); }
/* watch→phone (SKAI_LINK): device-list option the user interacted with, BY INDEX
   (0-based, matching the items[] order the phone sent in KEY_DEVICE_ACTIONS_BATCH).
   commit = tapped/confirmed (KEY_ACTION_SELECT); focus = scrolled to centre
   (KEY_ACTION_FOCUS). JSON {"index":N} to match the SKAI_LINK group's convention. */
bool commu_send_option_commit(uint8_t idx)
{
    char json[20];
    int n = rt_snprintf(json, sizeof(json), "{\"index\":%u}", (unsigned)idx);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_ACTION_SELECT, json);
    LOG_I("send option commit idx=%u -> %s", (unsigned)idx, ok ? "ok" : "FAILED");
    return ok;
}
bool commu_send_option_focus(uint8_t idx)
{
    char json[20];
    int n = rt_snprintf(json, sizeof(json), "{\"index\":%u}", (unsigned)idx);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    return commu_send_string(SKAI_LINK_COMMAND_ID, KEY_ACTION_FOCUS, json);
}

/* watch→phone (SKAI_LINK): @-conversation chat room (P5 "run @ chat on the watch").
   open over-provides identity (index/title/id) so the phone resolves the conversation
   route however its aggregation prefers. NOTE: title/id are interpolated RAW into the
   JSON (no escape helper on-device) — a contact name with a literal '"' would break the
   phone-side parse and the open is then dropped fail-safe; contact/app titles virtually
   never contain quotes, matching the rest of this group's raw-%s convention. */
bool commu_send_conv_open(const char *title, const char *id, uint8_t index)
{
    char json[256];
    int n = rt_snprintf(json, sizeof(json),
                        "{\"index\":%u,\"title\":\"%s\",\"id\":\"%s\"}",
                        (unsigned)index, title ? title : "", id ? id : "");
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_CONV_OPEN, json);
    LOG_I("send conv open idx=%u title=%s -> %s", (unsigned)index,
          title ? title : "", ok ? "ok" : "FAILED");
    return ok;
}
bool commu_send_conv_send(const char *text)
{
    if (text == NULL || text[0] == '\0') return false;
    char json[300];
    int n = rt_snprintf(json, sizeof(json), "{\"text\":\"%s\"}", text);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_CONV_SEND, json);
    LOG_I("send conv send len=%u -> %s", (unsigned)strlen(text), ok ? "ok" : "FAILED");
    return ok;
}
bool commu_send_conv_close(void)
{
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_CONV_CLOSE, "{}");
    LOG_I("send conv close -> %s", ok ? "ok" : "FAILED");
    return ok;
}

/* watch→phone (SKAI_LINK): device-page trackpad relay. The right-side device
   page hosts the hid_mouse trackpad; rather than emitting BLE HID reports, its
   events stream here and the phone actuates them on the active target device.
   JSON shapes match communicate_parse_skailink.h / the dart SkaiLinkKey doc. */
bool commu_send_mouse_move(int dx, int dy)
{
    char json[28];
    int n = rt_snprintf(json, sizeof(json), "{\"dx\":%d,\"dy\":%d}", dx, dy);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_MOUSE_MOVE, json);
    /* per-move log 靜音:air-mouse 以 ~125Hz 取樣,每次移動印一行會洗版 log */
    // LOG_D("send mouse move dx=%d dy=%d -> %s", dx, dy, ok ? "ok" : "FAIL");
    return ok;
}
bool commu_send_mouse_button(uint8_t btn, uint8_t act)
{
    char json[28];
    int n = rt_snprintf(json, sizeof(json), "{\"btn\":%u,\"act\":%u}",
                        (unsigned)btn, (unsigned)act);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_MOUSE_BUTTON, json);
    LOG_I("send mouse button btn=%u act=%u -> %s", (unsigned)btn, (unsigned)act,
          ok ? "ok" : "FAIL");
    return ok;
}
bool commu_send_mouse_scroll(int dx, int dy)
{
    char json[28];
    int n = rt_snprintf(json, sizeof(json), "{\"dx\":%d,\"dy\":%d}", dx, dy);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_MOUSE_SCROLL, json);
    /* per-move log 靜音:同 mouse move,捲動連續觸發會洗版 */
    // LOG_D("send mouse scroll dx=%d dy=%d -> %s", dx, dy, ok ? "ok" : "FAIL");
    return ok;
}
bool commu_send_mouse_back(void)
{
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_MOUSE_BACK, "{}");
    LOG_I("send mouse back -> %s", ok ? "ok" : "FAIL");
    return ok;
}
bool commu_send_skaibar_dismiss(void)
{
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_SKAIBAR_DISMISS, "{}");
    LOG_I("send skaibar dismiss -> %s", ok ? "ok" : "FAIL");
    return ok;
}
bool commu_send_skaibar_view(char cat)
{
    /* ADR-0024 round-trip: tell the phone which skaibar view just opened so it fans the
       matching query to every device. '@' / '/' pass through; anything else (the middle
       bar / "all" view) sends an EMPTY cat. */
    char c[2];
    c[0] = (cat == '@' || cat == '/') ? cat : '\0';
    c[1] = '\0';
    char json[16];
    rt_snprintf(json, sizeof(json), "{\"cat\":\"%s\"}", c);
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_SKAIBAR_VIEW_CHANGE, json);
    LOG_I("send skaibar view %s -> %s", json, ok ? "ok" : "FAIL");
    return ok;
}
bool commu_send_skaibar_open_device(void)
{
    /* Standalone mouse app (APP_ID_MOUSE) bar tap1: tell the phone to open the SINGLE
       controlled device's skaibar (single-target summon, NOT the aggregated broadcast of
       commu_send_skaibar_view). The phone routes summonSkaibar to the active device + latches
       single-device mode so the following voice transcript fills that device's panel too. */
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_SKAIBAR_OPEN_DEVICE, "{}");
    LOG_I("send skaibar open-device -> %s", ok ? "ok" : "FAIL");
    return ok;
}

/*============================================================================*
 *                              Sensor
 *============================================================================*/

bool commu_send_linear_acce_buffer(const uint8_t *acce, uint16_t length)
{
    if (!commu_can_send() || acce == NULL) return false;
    LOG_D("commu_send_linear_acce_buffer length=%d", length);
    /* L2 fragmenter handles MTU-based splitting transparently. */
    return skaiwatch_ble_send_l2(NOTIFY_COMMAND_ID, KEY_GSENSOR_SAMPLE, acce, length);
}

/*============================================================================*
 *                              File sync
 *============================================================================*/

bool commu_send_start_sync_file(uint32_t total_size)
{
    if (!commu_can_send()) return false;
    const char *file_path = get_sync_in_file_path();
    uint8_t path_len = (uint8_t)strlen(file_path);

    /* Payload: [total_size:4 BE][file_path:path_len] */
    uint8_t buf[L2_FIRST_VALUE_POS + 4 + 256];
    if ((size_t)(4 + path_len) > sizeof(buf) - L2_FIRST_VALUE_POS) return false;

    l2_write_header(buf, NOTIFY_COMMAND_ID, KEY_START_SYNC_FILE,
                    (uint16_t)(4 + path_len));
    write_be32(buf + L2_FIRST_VALUE_POS, total_size);
    memcpy(buf + L2_FIRST_VALUE_POS + 4, file_path, path_len);
    return skaiwatch_ble_notify(buf, (uint16_t)(L2_FIRST_VALUE_POS + 4 + path_len));
}

bool commu_send_sync_file(const uint8_t *chunk, uint16_t length)
{
    if (chunk == NULL) return false;
    return commu_send_blob(NOTIFY_COMMAND_ID, KEY_SYNC_FILE, chunk, length);
}

bool commu_send_end_sync_file(void)             { return commu_send_empty (NOTIFY_COMMAND_ID, KEY_END_SYNC_FILE); }
bool commu_send_file_compare_result(uint8_t r)  { return commu_send_status(NOTIFY_COMMAND_ID, KEY_FILE_COMPARE_RESULT, r); }

/*============================================================================*
 *                              Other
 *============================================================================*/

bool commu_send_bluetooth_log(const char *log) { return commu_send_string(BLUETOOTH_LOG_COMMAND_ID, KEY_DEBUG, log); }

/* Watch system snapshot can be larger than one MTU — route through send_l2 to
   avoid putting sizeof(SkaiWatchSys) on the stack. Wire format is identical
   to the legacy notify path (single fragment when it fits). */
bool commu_send_watch_system_sync(void)
{
    if (!commu_can_send()) return false;
    LOG_D("commu_send_watch_system_sync total_size = %u", (unsigned)sizeof(SkaiWatchSys));
    return skaiwatch_ble_send_l2(NOTIFY_COMMAND_ID, KEY_WATCH_SYS_RETURN,
                                  (const uint8_t *)&SkaiWatchSys,
                                  (uint16_t)sizeof(SkaiWatchSys));
}

bool commu_send_ota_status(uint8_t status)
{
    if (!commu_can_send_ota()) return false;
    uint8_t buf[L2_FIRST_VALUE_POS + 1];
    l2_write_header(buf, NOTIFY_COMMAND_ID, KEY_OTA_STATUS, 1);
    buf[L2_FIRST_VALUE_POS] = status;
    return skaiwatch_ble_notify(buf, sizeof(buf));
}

bool commu_send_device_info(void)
{
    if (!commu_can_send()) return false;
    /* Payload: [board_id:2 BE][major:1][minor:1][rev:1] = 5 bytes */
    uint8_t buf[L2_FIRST_VALUE_POS + 5];
    l2_write_header(buf, SET_CONFIG_COMMAND_ID, KEY_DEVICEINFO_RETURN, 5);
    write_be16(buf + L2_FIRST_VALUE_POS, (uint16_t)CUSTOMER_BOARD_VER);
    buf[L2_FIRST_VALUE_POS + 2] = VERSION_MAJOR;
    buf[L2_FIRST_VALUE_POS + 3] = VERSION_MINOR;
    buf[L2_FIRST_VALUE_POS + 4] = VERSION_REVISION;
    return skaiwatch_ble_notify(buf, sizeof(buf));
}

/* Replies with the manifest JSON as raw bytes (no NUL). Phone parses the
   string into {name → {version, size}} and reconciles with its SP cache. */
bool commu_send_model_versions(void)
{
    if (!commu_can_send()) return false;

    char json[MODEL_MANIFEST_JSON_MAX_LEN];
    int len = model_manifest_to_json(json, sizeof(json));
    if (len < 0)
    {
        LOG_E("model_manifest_to_json failed: %d", len);
        return false;
    }
    return commu_send_blob(SET_CONFIG_COMMAND_ID, KEY_MODEL_VERSION_RETURN,
                           json, (uint16_t)len);
}

/*============================================================================*
 *                              Audio
 *============================================================================*/

extern int audio_profile_send_voice_data(uint8_t *voice_data, uint16_t voice_data_len);
bool skaiwatch_ble_audio_send(uint8_t *buf, uint16_t length)
{
    return audio_profile_send_voice_data(buf, length) > 0;
}
