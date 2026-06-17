/**
 * @file   communicate_parse_skailink.h
 * @brief  ADR-0008 § E7: SKAI_LINK (0x20) device-sync command group.
 *
 * Phone (primary) → watch: the account device list + per-device status + each
 * device's actions, with incremental deltas. Watch → phone: the active-target
 * selection. Subkeys live in their own space under SKAI_LINK_COMMAND_ID (0x20),
 * which the firmware reserved but did not previously handle.
 *
 * Cross-device protocol — keep in lockstep with the dart side:
 *   SkaiLink/lib/shared/watch/communicate/communicate_protocol.dart
 *   (skaiLinkCommandId 0x20 + matching subkeys).
 */
#ifndef COMMUNICATE_PARSE_SKAILINK_H
#define COMMUNICATE_PARSE_SKAILINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        KEY_DEVICE_LIST_BATCH = 0x01,    /* phone→watch: full list [{id,name,platform,status}] */
        KEY_DEVICE_STATUS_DELTA = 0x02,  /* phone→watch: {id,status} one device's status */
        KEY_DEVICE_ACTIONS_BATCH = 0x03, /* phone→watch: {device_id,items:[...]} per-device actions */
        KEY_DEVICE_REMOVED = 0x04,       /* phone→watch: {id} device removed (logout) */
        KEY_ACTIVE_SELECT = 0x05,        /* watch→phone (UPLINK): {"device_id":"..."} active target ("" = none) */
        KEY_ACTION_SELECT = 0x06,        /* watch→phone (UPLINK): {"index":N} option the user TAPPED (0-based) */
        KEY_ACTION_FOCUS  = 0x07,        /* watch→phone (UPLINK): {"index":N} option SCROLLED to centre (0-based) */
        /* Device-page trackpad relay (UPLINK). The watch's right-side device page
           hosts the hid_mouse trackpad; instead of driving BLE HID directly, those
           events now stream to the phone, which actuates them on the active target
           device (KEY_ACTIVE_SELECT). Only the device-page mouse routes here — the
           standalone APP_ID_MOUSE app still talks BLE HID. */
        KEY_MOUSE_MOVE    = 0x08,        /* watch→phone (UPLINK): {"dx":N,"dy":N} relative pointer move */
        KEY_MOUSE_BUTTON  = 0x09,        /* watch→phone (UPLINK): {"btn":0|1,"act":0|1|2} btn 0=left 1=right; act 0=up 1=down 2=click */
        KEY_MOUSE_SCROLL  = 0x0A,        /* watch→phone (UPLINK): {"dx":N,"dy":N} wheel=dy, pan=dx */
        KEY_MOUSE_BACK    = 0x0B,        /* watch→phone (UPLINK): {} browser/navigation back */
        /* watch→phone (UPLINK): {} the user CANCEL-closed the floating skaibar
           (left-swipe / bar tap) WITHOUT committing an option. The phone treats a
           v2t stop as a pause (not a close), so without this it leaves the active
           skaibar open + its option list stale. On receipt the phone dismisses the
           active target's skaibar (device page → that device's skaibar; watch face
           → the phone launcher) and lets its list revert to the default. */
        KEY_SKAIBAR_DISMISS = 0x0C,
        /* watch→phone (UPLINK): {"cat":"@"|"/"|""} the skaibar VIEW the user just opened
           (ADR-0024 round-trip): left drawer = "@", right drawer = "/", middle bar = "".
           The phone fans the matching query to EVERY device so each view is its own
           per-device result list (@ destinations / recents / actions), not a filter of
           one shared 16-item list. Sent once when a view settles open. */
        KEY_SKAIBAR_VIEW_CHANGE = 0x0D,
        /* watch→phone (UPLINK): {} the STANDALONE mouse app (APP_ID_MOUSE) opened the
           SINGLE controlled device's skaibar (bar tap1). Single-target: the phone summons
           that one device's skaibar panel + (on the following voice) fills it — the
           pre-ADR-0024-P5 single-active-target behaviour, kept ONLY for the mouse app's
           single-device control context. Distinct from KEY_SKAIBAR_VIEW_CHANGE (0x0D),
           which is the watch-face bar's aggregated broadcast to EVERY device. */
        KEY_SKAIBAR_OPEN_DEVICE = 0x0E,
    } SKAI_LINK_KEY;

    /* Dispatched from communicate_parse.c for cmd_id == SKAI_LINK_COMMAND_ID.
       pValue/length are the reassembled L2 payload (UTF-8 JSON for 0x01-0x04). */
    void resolve_skailink_command(uint8_t key, uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATE_PARSE_SKAILINK_H */
