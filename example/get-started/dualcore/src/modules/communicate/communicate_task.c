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
    /* PROBE 2026-08-25: 「滑鼠 app 送出後 active 被清掉，文字掉進備忘錄」——這是所有
       active-target uplink 的唯一咽喉點，先確認送出了什麼，caller 由下面各處的 tag 區分。 */
    LOG_W("[active] send id=%s", device_id);
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_ACTIVE_SELECT, json);
    if (ok)
    {
        strncpy(s_last_active, device_id, sizeof(s_last_active) - 1);
        s_last_active[sizeof(s_last_active) - 1] = '\0';
        s_last_active_valid = true;
    }
    return ok;
}

/* watch→phone: relay a media transport command to the active target device
   (KEY_ACTIVE_SELECT). cmd ∈ playPause|next|previous|volumeUp|volumeDown — the
   air-mouse mediaControl verbs, forwarded by the phone without a mapping. Used by
   the mouse app's media centre when a remote target is active (otherwise the
   buttons keep driving the phone's own media over BLE HID, unchanged). Distinct
   from the legacy commu_send_media_control() which drives the phone's own media. */
bool commu_send_media_relay(const char *cmd)
{
    if (cmd == NULL) return false;
    char json[24]; /* {"cmd":"volumeDown"} = 20 chars + NUL */
    int n = rt_snprintf(json, sizeof(json), "{\"cmd\":\"%s\"}", cmd);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    return commu_send_string(SKAI_LINK_COMMAND_ID, KEY_MEDIA_CONTROL, json);
}

/* 音量條(founder 2026-08-18「我拉多少就調多少」):送**絕對**音量,不是 volumeUp/Down。
   那兩個在接收端是按下系統的音量鍵 —— 相對、而且被 OS 的級距量化,永遠落不到「拉到 42」。
   接收端改走各自的音訊 API(Windows: Core Audio SetMasterVolumeLevelScalar)。
   夾在 0..100:手錶端已經夾過一次,這裡是最後一道 —— 越界值送出去會讓桌面直接靜音。 */
bool commu_send_media_volume(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    char json[48]; /* {"cmd":"setVolume","value":100} = 31 chars + NUL */
    int n = rt_snprintf(json, sizeof(json),
                        "{\"cmd\":\"setVolume\",\"value\":%d}", percent);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    return commu_send_string(SKAI_LINK_COMMAND_ID, KEY_MEDIA_CONTROL, json);
}

/* watch→phone (SKAI_LINK 0x22): one TV remote key. Verb strings are brand-neutral
   (see KEY_TV_CONTROL); the phone maps them onto the bound TV's driver. Logged at
   INFO because these are user-initiated, low-rate presses — unlike mouse move /
   dial frames there is no flooding risk, and "did the key leave the watch?" is the
   first question when the TV doesn't react. */
bool commu_send_tv_key(const char *verb)
{
    if (verb == NULL) return false;
    char json[40]; /* {"cmd":"volumeDown"} = 20 chars; longest verb is playPause */
    int n = rt_snprintf(json, sizeof(json), "{\"cmd\":\"%s\"}", verb);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_TV_CONTROL, json);
    LOG_I("tv key %s -> %s", verb, ok ? "sent" : "FAIL");
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

bool commu_send_heart_curve_sample(uint32_t timestamp, uint8_t bpm,
                                   uint8_t worn)
{
    /* Packed 6-byte wire payload {timestamp:u32 LE, bpm:u8, worn:u8}.
       Explicit packing so there is no struct padding on the wire.
       Was 5 bytes until 2026-08-31; `worn` is appended so an older phone
       stops after bpm and a newer phone reading older firmware sees a 5-byte
       frame, which it must treat as UNKNOWN — never as worn.
       WATCH_SYS_WORN_* : 0 = not worn, 1 = worn, 0xFF = unknown.
       Backfilled points are always UNKNOWN: /health/hr_*.json has no wear
       column, so a replay honestly cannot say. */
    struct __attribute__((packed))
    {
        uint32_t timestamp;
        uint8_t bpm;
        uint8_t worn;
    } sample = {.timestamp = timestamp, .bpm = bpm, .worn = worn};
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
                          uint16_t pi_range_e6, uint16_t imu_var_e4,
                          uint16_t base_q4)
{
    /* Packed 16-byte wire payload — explicit packing so there is no struct
       padding on the wire (4+1+1+2+2+2+2, then the base_q4 tail). */
    struct __attribute__((packed))
    {
        uint32_t ts;
        uint8_t evt;
        uint8_t status;
        uint16_t dc_q4;
        uint16_t pi_e6;
        uint16_t pi_range_e6;
        uint16_t imu_var_e4;
        /* Appended 2026-08-31 after the frozen 14; an older phone stops at
           imu_var and a newer one reading older firmware sees no tail.
           worn_dc_base is the per-session learned DC baseline that EVERY
           ON/OFF decision is measured against, and it was the one quantity
           the diagnostics never reported. Without it a whole day was spent
           unable to tell whether the baseline had been dragged onto a desk —
           twice a wrong conclusion was drawn from code alone because the data
           simply could not answer it. Same q4 scaling as dc_q4 so the two are
           directly comparable. */
        uint16_t base_q4;
    } rec = {.ts = ts, .evt = evt, .status = status, .dc_q4 = dc_q4,
             .pi_e6 = pi_e6, .pi_range_e6 = pi_range_e6,
             .imu_var_e4 = imu_var_e4, .base_q4 = base_q4};
    return commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_WEAR_DIAG,
                           &rec, (uint16_t)sizeof(rec));
}

#if SKAI_HEALTH_DIAG
bool commu_send_sleep_diag(uint32_t ts, uint16_t score, uint8_t hr,
                           uint8_t hr_std, uint8_t stage, uint8_t veto,
                           uint8_t rhr, uint8_t worn, uint8_t rest,
                           uint8_t fresh, uint16_t total, uint16_t deep,
                           uint16_t rem, uint16_t light, uint16_t pi_e3,
                           uint16_t frame_pct, uint16_t rate_info,
                           uint16_t own_info, uint8_t rep_pct,
                           uint16_t accel_act)
{
    /* Packed 28-byte wire payload (phone reads 4+2+1*8+2*7 LE). total/deep/rem/
       light are the firmware's daily accumulators; pi_e3 is the last burst's
       perfusion index ×1000 (PPG signal-quality candidate); frame_pct is that
       burst's delivered PPG frames as a % of 25 Hz * burst seconds — the test
       for whether the nightly 2x episodes are lost-frame timebase errors;
       rate_info = (hr divider << 8) | algo-frames-%, the DECIMATION-stage view
       that frame_pct is blind to (divider 2 at a 25 Hz chip = exactly double). */
    struct __attribute__((packed))
    {
        uint32_t ts;
        uint16_t score;
        uint8_t hr;
        uint8_t hr_std;
        uint8_t stage;
        uint8_t veto;
        uint8_t rhr;
        uint8_t worn;
        uint8_t rest;
        uint8_t fresh;
        uint16_t total;
        uint16_t deep;
        uint16_t rem;
        uint16_t light;
        uint16_t pi_e3;
        uint16_t frame_pct;
        uint16_t rate_info;
        /* own_info = (hr_autocorr confidence << 8) | longest identical raw-PPG
           run; rep_pct = share of samples equal to their predecessor. Appended
           last so older phone builds, which gate on 24/26/28 bytes, keep parsing
           every field they already know. */
        uint16_t own_info;
        uint8_t  rep_pct;
        /* accel_act = wrist activity of the window the estimate came from (mean
           |d(accel)|/sample summed over 3 axes). It gates the motion
           compensation, and the gate's threshold was chosen against SYNTHETIC
           accelerometer data — this column is how a real night moves it.

           It is also the only place the reference's liveness is observable off
           the watch: hr_service runs on the LCPU, whose console is uart4 and is
           not wired on the bench, so the log line next to this value cannot be
           read. A dead feed reads as a permanent zero here, which is exactly the
           failure that went unnoticed for a night. */
        uint16_t accel_act;
    } rec = {.ts = ts, .score = score, .hr = hr, .hr_std = hr_std,
             .stage = stage, .veto = veto, .rhr = rhr, .worn = worn,
             .rest = rest, .fresh = fresh, .total = total, .deep = deep,
             .rem = rem, .light = light, .pi_e3 = pi_e3,
             .frame_pct = frame_pct, .rate_info = rate_info,
             .own_info = own_info, .rep_pct = rep_pct,
             .accel_act = accel_act};
    return commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_SLEEP_DIAG,
                           &rec, (uint16_t)sizeof(rec));
}

bool commu_send_hr_cont(uint32_t base_ts, uint8_t interval_s, uint8_t count,
                        const uint8_t *bpm, const uint8_t *qscore,
                        const uint8_t *qlevel, const uint8_t *accst,
                        const uint8_t *accel, const uint16_t *pi_e3)
{
    /* Variable-length: 6-byte header + five count-long byte arrays + one u16 LE
       array. At the 30-sample cap that is 216 B, comfortably inside
       MAX_PACKET_PAYLOAD_SIZE (507). */
    if (count == 0 || bpm == NULL || qscore == NULL || qlevel == NULL ||
        accst == NULL || accel == NULL || pi_e3 == NULL) return false;
    uint16_t len = (uint16_t)(6 + 7 * count);
    if (len > MAX_PACKET_PAYLOAD_SIZE) return false;

    uint8_t buf[6 + 7 * 30];
    buf[0] = (uint8_t)(base_ts & 0xFF);
    buf[1] = (uint8_t)((base_ts >> 8) & 0xFF);
    buf[2] = (uint8_t)((base_ts >> 16) & 0xFF);
    buf[3] = (uint8_t)((base_ts >> 24) & 0xFF);
    buf[4] = interval_s;
    buf[5] = count;
    uint16_t o = 6;
    memcpy(buf + o, bpm, count);    o += count;
    memcpy(buf + o, qscore, count); o += count;
    memcpy(buf + o, qlevel, count); o += count;
    memcpy(buf + o, accst, count);  o += count;
    memcpy(buf + o, accel, count);  o += count;
    for (uint8_t i = 0; i < count; i++)   /* u16 LE, matching every other key here */
    {
        buf[o++] = (uint8_t)(pi_e3[i] & 0xFF);
        buf[o++] = (uint8_t)((pi_e3[i] >> 8) & 0xFF);
    }
    return commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_HR_CONT_DIAG, buf, len);
}

bool commu_send_hr_window(uint32_t ts, uint8_t bpm, uint8_t conf,
                          uint16_t count, const int8_t *win,
                          uint16_t acc_count, uint8_t acc_shift, const int8_t *acc)
{
    /* 8-byte header + count int8 samples, then an OPTIONAL accel block appended
       after them: {acc_count u8, acc_shift u8, acc int8[acc_count]}. 256+64
       lands at 330 B, inside MAX_PACKET_PAYLOAD_SIZE (507) — that single-frame
       fit is why the samples are int8 rather than the int16 the estimator works
       in, and why the accel is decimated 4:1.
       Appended rather than inserted so both directions of version skew are safe:
       an old phone stops after 8+count and ignores the tail; a new phone sees no
       tail from old firmware and reports acc_count 0. */
    if (win == NULL || count == 0) return false;
    if (count > 256) count = 256;
    if (acc == NULL) acc_count = 0;
    if (acc_count > 64) acc_count = 64;

    uint8_t buf[8 + 256 + 2 + 64];
    buf[0] = (uint8_t)(ts & 0xFF);
    buf[1] = (uint8_t)((ts >> 8) & 0xFF);
    buf[2] = (uint8_t)((ts >> 16) & 0xFF);
    buf[3] = (uint8_t)((ts >> 24) & 0xFF);
    buf[4] = bpm;
    buf[5] = conf;
    /* u16, not u8. The window is 256 samples and 256 & 0xFF is 0, so the byte
       version told the phone "zero samples" for every capture ever sent; the
       phone dropped all of them as malformed and two nights of waveforms were
       lost while both sides looked healthy. The one length this field has to
       carry is the one value it could not represent. */
    buf[6] = (uint8_t)(count & 0xFF);
    buf[7] = (uint8_t)((count >> 8) & 0xFF);
    memcpy(buf + 8, win, count);
    uint16_t len = (uint16_t)(8 + count);
    if (acc_count > 0)
    {
        buf[len++] = (uint8_t)acc_count;
        buf[len++] = acc_shift;
        memcpy(buf + len, acc, acc_count);
        len = (uint16_t)(len + acc_count);
    }
    bool ok = commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_HR_WINDOW_DUMP,
                              buf, len);
    /* The only point of the whole capture that is observable on the HCPU log:
       the decision and its LOG_I both live on the LCPU, whose console is uart4,
       not the COM12 firmware log. Without this a bench session cannot tell a
       burst that shipped a window from one that never captured. */
    LOG_I("send hr window bpm=%u conf=%u n=%u acc=%u<<%u -> %s", (unsigned)bpm,
          (unsigned)conf, (unsigned)count, (unsigned)acc_count,
          (unsigned)acc_shift, ok ? "ok" : "FAIL");
    return ok;
}

bool commu_send_hr_window_raw(uint32_t ts, int64_t fit_a_q16, int64_t fit_b_q16,
                              uint8_t shift, uint16_t first_index,
                              uint16_t count, const int16_t *win)
{
    /* @ref KEY_HR_WINDOW_RAW. 23-byte header + count int16, little-endian.
       128 samples = 279 B, well inside MAX_PACKET_PAYLOAD_SIZE (507); the
       chunking lives in the caller so this stays a pure serialiser.
       The fit is 64-bit: 24-bit raw counts shifted left 16 do not fit int32. */
    if (win == NULL || count == 0) return false;
    if (count > 128) count = 128;

    uint8_t buf[23 + 128 * 2];
    uint16_t n = 0;
    buf[n++] = (uint8_t)(ts & 0xFF);
    buf[n++] = (uint8_t)((ts >> 8) & 0xFF);
    buf[n++] = (uint8_t)((ts >> 16) & 0xFF);
    buf[n++] = (uint8_t)((ts >> 24) & 0xFF);
    uint64_t a = (uint64_t)fit_a_q16, b = (uint64_t)fit_b_q16;
    for (int k = 0; k < 8; k++) buf[n++] = (uint8_t)((a >> (8 * k)) & 0xFF);
    for (int k = 0; k < 8; k++) buf[n++] = (uint8_t)((b >> (8 * k)) & 0xFF);
    buf[n++] = (uint8_t)(first_index & 0xFF);
    buf[n++] = (uint8_t)((first_index >> 8) & 0xFF);
    buf[n++] = shift;
    for (uint16_t i = 0; i < count; i++)
    {
        uint16_t v = (uint16_t)win[i];
        buf[n++] = (uint8_t)(v & 0xFF);
        buf[n++] = (uint8_t)((v >> 8) & 0xFF);
    }
    return commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_HR_WINDOW_RAW, buf, n);
}

bool commu_send_hr_burst(uint32_t ts, uint32_t dur_ms, uint32_t samples,
                         uint16_t reads, uint16_t readfail, uint16_t frame_pct,
                         uint16_t rate_info, uint8_t extends, uint8_t best,
                         uint8_t reason, uint8_t power_veto)
{
    /* @ref KEY_HR_BURST_SUMMARY. 23 bytes, little-endian. */
    uint8_t buf[24];
    uint16_t n = 0;
    uint32_t w32[3] = { ts, dur_ms, samples };
    for (int j = 0; j < 3; j++)
        for (int k = 0; k < 4; k++) buf[n++] = (uint8_t)((w32[j] >> (8 * k)) & 0xFF);
    uint16_t w16[4] = { reads, readfail, frame_pct, rate_info };
    for (int j = 0; j < 4; j++)
        for (int k = 0; k < 2; k++) buf[n++] = (uint8_t)((w16[j] >> (8 * k)) & 0xFF);
    buf[n++] = extends;
    buf[n++] = best;
    buf[n++] = reason;
    /* Appended after the frozen 23 -- an older phone stops at reason and a newer
       one reading older firmware simply sees no tail. */
    buf[n++] = power_veto;
    return commu_send_blob(HEALTH_DATA_COMMAND_ID, KEY_HR_BURST_SUMMARY, buf, n);
}
#endif /* SKAI_HEALTH_DIAG */

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
bool commu_send_call_accept(const char *id)          { return commu_send_string(NOTIFY_COMMAND_ID, KEY_INCOMMING_CALL_ACCEPT, id ? id : ""); }
bool commu_send_call_refuse(const char *id)          { return commu_send_string(NOTIFY_COMMAND_ID, KEY_INCOMMING_CALL_REFUSE, id ? id : ""); }
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
/* Answer to KEY_REQUEST_BATTERY / the CCCD-subscribe intro: push what we hold
   right now, no change gate. Level 0 is skipped because battery_level_value is
   a live mirror of the LCPU report and reads 0 for the first seconds after boot
   (see watch_global_data.c) — sending it would paint a fresh "0%" on the phone.
   Three small notifies is the same burst shape as the ordinary poll tick. */
bool commu_send_battery_snapshot(void)
{
    if (!commu_can_send()) return false;
    uint8_t level = (uint8_t)SkaiWatchSys.battery_level_value;
    bool ok = true;
    if (level >= 1 && level <= 100) ok = commu_send_battery_level(level) && ok;
    ok = commu_send_charge_status() && ok;
    if (SkaiWatchSys.battery_vol_value != 0)
        ok = commu_send_battery_voltage(SkaiWatchSys.battery_vol_value) && ok;
    /* Two args max: this logger silently drops the whole call when the
       argument count grows (memory: watch LOG_W arg-count trap). */
    LOG_I("battery snapshot lvl=%u ok=%d", (unsigned)level, (int)ok);
    return ok;
}

bool commu_send_update_instruction(const char *json) { return commu_send_string(NOTIFY_COMMAND_ID, KEY_SKAI_CREATION_INSTRUCTIONS, json); }
bool commu_send_get_instruction_img(const char *id)  { return commu_send_string(NOTIFY_COMMAND_ID, KEY_SKAI_INSTRUCTION_IMAGE, id); }
/* 跟手機要一張 Bot 頭像(內容雜湊當鍵);圖走檔案通道回來,這裡不等回覆。 */
bool commu_send_conv_avatar_req(const char *av)
{
    char json[40];
    if (av == NULL || av[0] == '\0') return false;
    int n = rt_snprintf(json, sizeof(json), "{\"av\":\"%s\"}", av);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    return commu_send_string(SKAI_LINK_COMMAND_ID, KEY_CONV_AVATAR_REQ, json);
}
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
/* Ask the phone to (re)push the desktop session list. The Data path has no pull, so the
   session pager sends this on build + on reconnect to recover a list pushed while the
   watch was away. */
/* [device] NULL/"" = every desktop. */
bool commu_send_conv_list_req(const char *device)
{
    char json[128];
    if (device != NULL && device[0] != '\0')
    {
        int n = rt_snprintf(json, sizeof(json), "{\"device\":\"%s\"}", device);
        if (n <= 0 || n >= (int)sizeof(json)) return false;
    }
    else
    {
        rt_strncpy(json, "{}", sizeof(json));
    }
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_CONV_LIST_REQ, json);
    /* LOG_W: 這台 dev 錶只印 W/E,而「手錶到底有沒有問」是 per-device 斷點診斷的第一格。 */
    LOG_W("send conv list req dev=%s -> %s", (device && device[0]) ? device : "*",
          ok ? "ok" : "FAILED");
    return ok;
}
/* Ask [device] to create a NEW session. Expects no direct reply — the new row comes back
   in that desktop's ordinary session list (KEY_CONV_LIST). See KEY_CONV_NEW in
   communicate_parse_skailink.h for why the new id never travels on this key. */
bool commu_send_conv_new_ex(const char *device, const char *text); /* 定義在下方(本檔不 include 自己的標頭) */

bool commu_send_conv_new(const char *device)
{
    return commu_send_conv_new_ex(device, NULL);
}

/* [text] = 這個新對話的第一句(左頁語音搜尋沒中任何項目時,使用者講的那句話)。
   為什麼一定要帶:桌面的 Hermes **只在 session 有了第一則訊息之後才落地**(2026-08-13
   實測:建立回報成功、`select … from sessions` 卻查無此列)。空的建立請求等於什麼都沒
   發生 —— 清單永遠不會出現那一列,手錶也就等不到、走不進聊天室。帶著第一句去建,
   session 才存在得下來。 */
bool commu_send_conv_new_ex(const char *device, const char *text)
{
    if (device == NULL || device[0] == '\0')
    {
        /* Refuse rather than let the phone guess: with two desktops online an unaddressed
           create lands on whichever one it happened to list last. */
        LOG_W("send conv new: no device, refused");
        return false;
    }
    char json[320];
    int n;
    if (text != NULL && text[0] != '\0')
    {
        /* 轉錄可能含 " 或 \ —— 逐字元跳脫,別讓一句話破壞整個 JSON。 */
        char esc[224];
        size_t w = 0;
        for (const char *p = text; *p && w + 2 < sizeof(esc); p++)
        {
            if (*p == '"' || *p == '\\')
                esc[w++] = '\\';
            else if ((unsigned char)*p < 0x20)
                continue; /* 控制字元直接丟掉 */
            esc[w++] = *p;
        }
        esc[w] = '\0';
        n = rt_snprintf(json, sizeof(json), "{\"device\":\"%s\",\"text\":\"%s\"}", device, esc);
    }
    else
    {
        n = rt_snprintf(json, sizeof(json), "{\"device\":\"%s\"}", device);
    }
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_CONV_NEW, json);
    LOG_W("send conv new dev=%s textLen=%u -> %s", device,
          (unsigned)(text ? strlen(text) : 0), ok ? "ok" : "FAILED");
    return ok;
}

/* watch→phone (SKAI_LINK): SkaiApp install/remove result (ADR-0037).
   id is charset-whitelisted upstream ([a-z0-9-]) so raw %s is quote-safe. */
bool commu_send_skaiapp_ack(const char *id, int code)
{
    if (id == NULL || id[0] == '\0') return false;
    char json[64];
    int n = rt_snprintf(json, sizeof(json), "{\"id\":\"%s\",\"code\":%d}", id, code);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_SKAIAPP_ACK, json);
    LOG_I("send skaiapp ack id=%s code=%d -> %s", id, code, ok ? "ok" : "FAILED");
    return ok;
}

/* watch→phone (SKAI_LINK): the user tapped a memo's 🎤 to voice-fill it (ADR-0037).
   app_id/memo_id are charset-whitelisted ([a-z0-9-]) so raw %s is quote-safe. The
   phone remembers this target and routes the next STT transcript to setMemoText. */
bool commu_send_skaiapp_voice(const char *app_id, const char *memo_id)
{
    if (app_id == NULL || app_id[0] == '\0' || memo_id == NULL || memo_id[0] == '\0')
        return false;
    char json[80];
    int n = rt_snprintf(json, sizeof(json), "{\"id\":\"%s\",\"memo\":\"%s\"}",
                        app_id, memo_id);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_SKAIAPP_VOICE, json);
    LOG_I("send skaiapp voice app=%s memo=%s -> %s", app_id, memo_id, ok ? "ok" : "FAILED");
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
/* watch→phone (SKAI_LINK): trackpad-hold radial dial (see KEY_DIAL_DIR contract).
   phase is one of the three fixed literals start|update|end so raw %s is quote-safe;
   dir -1..7, mag 0..1000. Throttled by the caller (air_mouse_process) — no per-frame
   log so the ~update stream can't flood the console (same reason as mouse move). */
bool commu_send_dial_dir(const char *phase, int dir, int mag)
{
    if (phase == NULL) return false;
    char json[56]; /* {"phase":"update","dir":-1,"mag":1000} = ~40 chars + NUL */
    int n = rt_snprintf(json, sizeof(json),
                        "{\"phase\":\"%s\",\"dir\":%d,\"mag\":%d}", phase, dir, mag);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    return commu_send_string(SKAI_LINK_COMMAND_ID, KEY_DIAL_DIR, json);
}
/* watch→phone (SKAI_LINK): 側立手寫 ink 串流 (see KEY_HANDWRITE contract)。點批次是
   變長度陣列,固定欄位 builder 不合用 — caller (bloc_motion_tracking / hid_mouse) 自組
   JSON。上傳由 caller 節流(~25Hz);無 per-frame log,同 mouse move / dial 防洗版。 */
bool commu_send_handwrite(const char *json)
{
    if (json == NULL) return false;
    return commu_send_string(SKAI_LINK_COMMAND_ID, KEY_HANDWRITE, json);
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
/* 立起輸入面板(2026-07-31)的 0x0E:forceOpen + inputOnly 兩旗標版。inputOnly=true 讓電腦
   把 skaibar 叫出來但只留輸入框(不出選項),並「記住」召喚前聚焦的那個輸入框當之後 icon_send
   的目的地 —— 而不是像舊的 force_open=false 流程那樣邊講邊把逐字稿打進去。舊呼叫點沿用下面
   的 commu_send_skaibar_open_device(等同 input_only=false),wire 對舊手機不變。 */
bool commu_send_skaibar_open_device_ex(bool force_open, bool input_only)
{
    char json[48];
    rt_snprintf(json, sizeof(json), "{\"forceOpen\":%s,\"inputOnly\":%s}",
                force_open ? "true" : "false", input_only ? "true" : "false");
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_SKAIBAR_OPEN_DEVICE, json);
    LOG_I("send skaibar open-device forceOpen=%d inputOnly=%d -> %s",
          (int)force_open, (int)input_only, ok ? "ok" : "FAIL");
    return ok;
}

/* 立起輸入面板的送出(見 KEY_LIFT_INPUT_COMMIT 契約)。dest 只有 "field" / "skaibar" 兩個
   固定字面值,故 raw %s 是引號安全的;文字不隨行,由手機端用它手上的最終稿。 */
bool commu_send_lift_input_commit(const char *dest)
{
    if (dest == NULL) return false;
    char json[32]; /* {"dest":"skaibar"} = 20 chars + NUL */
    int n = rt_snprintf(json, sizeof(json), "{\"dest\":\"%s\"}", dest);
    if (n <= 0 || n >= (int)sizeof(json)) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_LIFT_INPUT_COMMIT, json);
    LOG_I("send lift-input commit dest=%s -> %s", dest, ok ? "ok" : "FAIL");
    return ok;
}

/* 立起輸入面板的插入點(見 KEY_LIFT_INPUT_CARET 契約)。pos 是字元索引,不是 byte。
   **text 一起送**:pos 是對「手錶畫面上這串文字」算出來的,手機那份副本只要有一次不同步
   (app 重啟/程序被回收),用它去切前後半就會默默切錯 —— 真機 2026-08-01 撞過:手機 app 被
   重裝後暫存是空的,切出來前後半都是空字串,結果整段只剩新講的那句。帶著文字走,手機就能
   用「使用者真正看到的那串」重新對齊,不必假設兩邊一致。 */
bool commu_send_lift_input_caret(int pos, const char *text)
{
    if (pos < 0) return false;
    cJSON *root = cJSON_CreateObject();
    if (!root) return false;
    cJSON_AddNumberToObject(root, "pos", pos);
    cJSON_AddStringToObject(root, "text", text ? text : "");
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_LIFT_INPUT_CARET, json);
    LOG_I("send lift-input caret pos=%d -> %s", pos, ok ? "ok" : "FAIL");
    cJSON_free(json);
    return ok;
}

/* 立起輸入面板的刪除鍵(見 KEY_LIFT_INPUT_DELETE 契約)。一次一個字;長按由手錶端 timer 重送。 */
bool commu_send_lift_input_delete(void)
{
    return commu_send_string(SKAI_LINK_COMMAND_ID, KEY_LIFT_INPUT_DELETE, "{}");
}

/* 滑鼠 app 語音站的送出(0x1d 加選填 text)。與立起面板那條的差別:語音站的文字真相在
   **手錶本地**(鍵盤模式的 input_buffer,才能跟注音/英文混著用),所以這裡必須把文字一起帶
   上去;手機收到帶 text 的就用它,不用自己那份暫存稿。AI 口語整理由手機端收到後再跑。 */
bool commu_send_voice_station_commit(const char *dest, const char *text)
{
    if (!dest || !text) return false;
    cJSON *root = cJSON_CreateObject();
    if (!root) return false;
    cJSON_AddStringToObject(root, "dest", dest);
    cJSON_AddStringToObject(root, "text", text);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_LIFT_INPUT_COMMIT, json);
    LOG_I("send voice-station commit dest=%s -> %s", dest, ok ? "ok" : "FAIL");
    cJSON_free(json);
    return ok;
}

/* 同一把鑰匙(0x1e),preview = 手錶目前顯示的文字。語音站的文字真相在手錶本地,電腦那條
   輸入框要跟著顯示就得靠這個持續推(立起面板不需要 —— 那時真相在手機)。呼叫端已做防抖。 */
bool commu_send_voice_station_preview(const char *text)
{
    if (!text) return false;
    cJSON *root = cJSON_CreateObject();
    if (!root) return false;
    cJSON_AddStringToObject(root, "preview", text);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return false;
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_LIFT_INPUT_CARET, json);
    cJSON_free(json);
    return ok;
}

/* 同一把鑰匙(0x1e),cancel = 把「這次按住錄到的那一段」整個丟掉,暫存文字退回按下之前。
   用在「長按本來在講話、手指一移動就變成框選」的轉場 —— 那一段從來不是使用者要的字。 */
bool commu_send_lift_input_cancel_segment(void)
{
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_LIFT_INPUT_CARET, "{\"cancel\":true}");
    LOG_I("send lift-input cancel segment -> %s", ok ? "ok" : "FAIL");
    return ok;
}

/* 同一把鑰匙,帶範圍 = 刪掉框選的那一段(字元索引,半開區間 [from,to))。 */
bool commu_send_lift_input_delete_range(int from, int to)
{
    char json[48];
    rt_snprintf(json, sizeof(json), "{\"from\":%d,\"to\":%d}", from, to);
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_LIFT_INPUT_DELETE, json);
    LOG_I("send lift-input delete range [%d,%d) -> %s", from, to, ok ? "ok" : "FAIL");
    return ok;
}

bool commu_send_skaibar_open_device(bool force_open)
{
    /* Standalone mouse app (APP_ID_MOUSE) bar tap1: tell the phone to open the SINGLE
       controlled device's skaibar (single-target summon, NOT the aggregated broadcast of
       commu_send_skaibar_view). The phone routes summonSkaibar to the active device + latches
       single-device mode so the following voice transcript fills that device's panel too.
       force_open=true (manual bar-tap) always makes the desktop show its panel; false (the
       lift-gesture direct voice-input flow) lets the desktop defer to an already-focused
       external text input instead of popping the panel — see
       instruction_list_open_lift_mic_view's only caller of force_open=false. */
    char json[24];
    rt_snprintf(json, sizeof(json), "{\"forceOpen\":%s}", force_open ? "true" : "false");
    bool ok = commu_send_string(SKAI_LINK_COMMAND_ID, KEY_SKAIBAR_OPEN_DEVICE, json);
    LOG_I("send skaibar open-device forceOpen=%d -> %s", (int)force_open, ok ? "ok" : "FAIL");
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
bool commu_send_file_sync_result(uint8_t r)     { return commu_send_status(NOTIFY_COMMAND_ID, KEY_FILE_SYNC_RESULT, r); }

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
