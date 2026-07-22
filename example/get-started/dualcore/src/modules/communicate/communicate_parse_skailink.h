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
        /* ── @-conversation chat room (P5 "run @ chat on the watch") ──
           The watch taps a left-@ contact row and opens an in-watch chat room that
           MIRRORS the desktop conversation: the phone-primary routes the open to the
           owning desktop (km-relay convOpen), the desktop runs it headless + streams
           turns back, and the phone pushes the folded chat state DOWN to the watch.
           Keep in lockstep with the phone WatchProtocol (android/ios) conv handling. */
        /* watch→phone (UPLINK): {"index":N,"title":"...","id":"..."} the left-@ row the
           user tapped. Over-provides identity (index/title/id) so the phone resolves the
           conversation route however its aggregation prefers (id if a conv:<svc>:<handle>,
           else title, else index). Opens the chat room. */
        KEY_CONV_OPEN = 0x0F,
        /* watch→phone (UPLINK): {"text":"..."} send one turn into the open conversation
           (the watch's mic→V2T transcript). */
        KEY_CONV_SEND = 0x10,
        /* watch→phone (UPLINK): {} the user left the chat room (back gesture). The phone
           stops observing + unbinds (km-relay convStop). */
        KEY_CONV_CLOSE = 0x11,
        /* phone→watch (DOWNLINK): {"title":"...","sending":bool,"messages":[{"role":"...",
           "text":"..."}]} the folded chat state the watch renders. role ∈ incoming/outgoing/
           user/assistant. Pushed on every convEvent the phone folds. */
        KEY_CONV_STATE = 0x12,
        /* ── SkaiApp: AI-generated declarative mini-apps (SkaiLink ADR-0037).
           Keep in lockstep with the phone WatchProtocol (android/ios). ── */
        /* phone→watch (DOWNLINK): {"id","seq","n","crc"(seq 0 only),"chunk"} —
           one package pushed as ≤3 KB-raw base64 chunks; CRC32 over the whole
           raw JSON; reassembled + installed by gui_apps/skaiapp/skaiapp_proto.c */
        KEY_SKAIAPP_PUSH = 0x13,
        /* phone→watch (DOWNLINK): {"id"} uninstall one mini-app */
        KEY_SKAIAPP_REMOVE = 0x14,
        /* watch→phone (UPLINK): {"id","code"} install/remove result —
           0 ok / 1 crc / 2 parse / 3 unsupported / 4 storage / 5 limit */
        KEY_SKAIAPP_ACK = 0x15,
        /* watch→phone (UPLINK): {"id":"<appId>","memo":"<memoId>"} — the user tapped
           the memo's 🎤 on the watch to voice-fill it (ADR-0037). Sent right before
           the watch starts streaming mic audio with V2T_INTENT_MEMO; the phone
           remembers this target, runs STT on the audio, and writes the transcript
           back with SkaiappController.setMemoText (which re-pushes the package). */
        KEY_SKAIAPP_VOICE = 0x16,
        /* phone→watch (DOWNLINK): {"focused":bool} — the box the watch's standalone mouse app
           (APP_ID_MOUSE) is currently controlling just gained/lost a genuinely focused native
           text input (Windows WindowsFocusedTextInput.HasActiveExternalTextInput / Mac
           RemoteFocusProbe.hasActiveExternalTextInput, re-checked after every relayed click and
           pushed down edge-triggered — see the air-mouse focusGained/focusLost group in
           bridge/schema/air-mouse-protocol.schema.json). Cached by
           instruction_list_set_remote_target_focus() in lv_instruction_list_layout.c and read by
           instruction_list_bar_tap_device(): true → the bar-tap skips straight to the text/voice
           input box; false (the default) → it shows the device's option list first, unchanged
           from the original behaviour. Reset to false on instruction_list_bar_device_dismiss()
           so a stale true never leaks into the next device/session. */
        KEY_REMOTE_TEXT_FOCUS = 0x17,
        /* watch→phone (UPLINK): {"cmd":"playPause|next|previous|volumeUp|volumeDown"} —
           the standalone mouse app's (APP_ID_MOUSE) media-centre transport buttons,
           relayed to the active target device (KEY_ACTIVE_SELECT) instead of the watch's
           own BLE HID consumer report. cmd strings match the air-mouse mediaControl verbs
           so the phone forwards them without a mapping. Only sent when an active remote
           target is selected (ble_hid_mouse_app_route()); otherwise the buttons keep
           driving the phone's own media over HID, unchanged. */
        KEY_MEDIA_CONTROL = 0x18,
        /* phone→watch (DOWNLINK): {"device_id":"...","title":"...","artist":"...","playing":bool}
           — the ACTIVE target device's now-playing, for the mouse app's media centre ONLY.
           Kept SEPARATE from the phone's own media title (NOTIFY_KEY_MEDIA_TITLE 0x46), which
           still feeds the watch-face media widget / dial header. The watch filters by device_id
           so a late frame for a just-deselected device can't overwrite the current UI. */
        KEY_MEDIA_STATE = 0x19,
        /* watch→phone (UPLINK): {"phase":"start|update|end","dir":-1..7,"mag":0..1000}
           — the standalone mouse app's (APP_ID_MOUSE) TRACKPAD-HOLD radial dial. Holding
           the main trackpad still (~250 ms, no drift) arms a wrist-tilt direction pad: the
           watch integrates the air-mouse delta into a vector and reports its 8-way sector +
           magnitude, throttled. dir 0=up then CLOCKWISE (1=up-right 2=right 3=down-right
           4=down 5=down-left 6=left 7=up-left); dir=-1 = inside the centre dead-zone (no
           selection). mag 0..1000 = distance/force from centre (0 at the dead-zone edge,
           1000 at full deflection). phase start = dial opened; update = live sector while
           held; end = finger released, dir carries the COMMITTED sector (-1 = released in
           the dead-zone, cancelled). The phone forwards each frame to the active target as
           the air-mouse `dialDirection` verb; the desktop draws a cursor-centred radial
           overlay and (this round) only logs the committed direction — no action bound yet. */
        KEY_DIAL_DIR = 0x1a,
        /* watch→phone (UPLINK): 側立手寫 ink 串流 — 滑鼠 app (APP_ID_MOUSE) 內把錶面
           轉向側邊(重力落在 ±X、穩定 ~300ms)進入手寫模式:按住錶面=下筆、手腕在空中寫
           (gyro 積分成虛擬畫布筆尖)、放開=提筆,轉回錶面朝上(gravity.z 高持續 ~400ms)
           結束。payload:
             {"ph":"start","w":W,"h":H}  進入(虛擬畫布尺寸=螢幕解析度)
             {"ph":"d","p":[[x,y]]}      下筆(第一點)
             {"ph":"m","p":[[x,y],...]}  筆點批次(~25Hz 節流,order=筆畫形狀)
             {"ph":"h","p":[[x,y]]}      提筆懸浮位置(低頻,桌面畫「筆尖將落在哪」)
             {"ph":"u"}                  提筆
             {"ph":"end"}                結束
           手機轉發整串給 active 桌面畫軌跡(air-mouse `handwriting` verb),同時跑 ML Kit
           digital-ink 辨識(zh-Hant):每筆 u 後即時 setSkaibarText、end 時
           runSkaibar{submit:true} 自動送出。Keep in lockstep with
           WatchProtocol.kt SKAILINK_KEY_HANDWRITE. */
        KEY_HANDWRITE = 0x1b,
        /* phone→watch (DOWNLINK): {"c":["候","選","字",...]} — 手寫當前字的辨識候選
           (ML Kit top-5,founder 2026-07-20 晚)。每次辨識更新即推;空陣列=清單清空
           (字已定稿/板已清)。手錶在手寫 view 退出鈕下方顯示候選列:按候選=送
           {"ph":"n","i":idx}(KEY_HANDWRITE)用該候選定稿+換下個字;候選非空時
           「輸入」鈕隱藏(先定稿才能送出)。Keep in lockstep with
           WatchProtocol.kt SKAILINK_KEY_HANDWRITE_CAND. */
        KEY_HANDWRITE_CAND = 0x1c,
    } SKAI_LINK_KEY;

    /* Dispatched from communicate_parse.c for cmd_id == SKAI_LINK_COMMAND_ID.
       pValue/length are the reassembled L2 payload (UTF-8 JSON for 0x01-0x04). */
    void resolve_skailink_command(uint8_t key, uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATE_PARSE_SKAILINK_H */
