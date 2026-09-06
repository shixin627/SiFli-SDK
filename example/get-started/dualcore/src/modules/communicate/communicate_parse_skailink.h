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
        /* phone→watch (DOWNLINK): {"sid":"...","title":"...","sending":bool,"messages":
           [{"role":"...","text":"..."}]} the folded chat state the watch renders. role ∈
           incoming/outgoing/user/assistant. Pushed on every convEvent the phone folds.
           "sid" (2026-08-05, ADDITIVE) names WHICH conversation this state belongs to —
           the session pager keeps one page per desktop session and only ever has one open
           at a time, so a state whose sid ≠ the settled page's is a late frame from the
           conversation just left and MUST be dropped. Absent (older phone) ⇒ the receiver
           treats it as belonging to whatever is currently open, the pre-pager behaviour. */
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
        /* watch→phone (UPLINK): {"dest":"field"|"skaibar"} — 立起輸入面板(2026-07-31)的
           送出。面板期間語音轉錄只「暫存」在手錶面板 + 電腦的純輸入框(0x0E 帶 inputOnly),
           由使用者在手錶上挑去處:
             "field"   = icon_send:打進召喚 skaibar 之前使用者點的那個輸入框(手機轉成
                         air-mouse `sendToFocusedInput`,電腦打完字順手收掉面板);手錶收掉
                         面板、離開此模式。
             "skaibar" = img_logo:當成該台的 skaibar 查詢送出(手機轉成 runSkaibar
                         submit=false 讓電腦展開選項 + skaibarQuery 讓它把選項 push 回來);
                         手錶叫出鏡像那份選項的清單。
           **刻意不帶文字**:最終稿在手機那邊(它剛跑完 AI 口語拆字還原並回推給兩邊顯示),
           手錶再送一份只會多出一個可能不一致的真相,長口述也有 L2 邊界截斷 UTF-8 的風險。
           Keep in lockstep with WatchProtocol.kt SKAILINK_KEY_LIFT_INPUT_COMMIT。 */
        KEY_LIFT_INPUT_COMMIT = 0x1d,
        /* watch→phone (UPLINK): {"pos":N} — 立起輸入面板的**插入點**(founder 2026-08-01:
           長按輸入框的某個位置,接下來講的話插在那裡,而不是取代整段)。N 是**字元索引**
           (code point,非 byte;由 LVGL 的 lv_label_get_letter_on 命中觸點得到),0 = 最前面。
           手機收到後記住,下一段轉錄 T 就組成 prefix + T + suffix 再推回兩邊顯示;AI 口語
           拆字還原只套用在 T(既有文字已經定稿過)。面板重開或文字被換掉即失效,要再長按一次。
           Keep in lockstep with WatchProtocol.kt SKAILINK_KEY_LIFT_INPUT_CARET。 */
        KEY_LIFT_INPUT_CARET = 0x1e,
        /* watch→phone (UPLINK): {} — 立起輸入面板的刪除鍵按了一下 = **刪掉一個字**(founder
           2026-08-01:「不是清空,要一個個刪除;按一次刪一個字,長按才開始連續刪除」)。長按時
           手錶用自己的 timer 以固定速率重送。刪除實作在手機端:暫存文字的單一真相在那裡,
           而且「一個字」要按 code point 算(中文一個字 3 bytes),手錶只發指令。刪的是插入點
           前面那個字(沒指定插入點時就是最後一個)。
           Keep in lockstep with WatchProtocol.kt SKAILINK_KEY_LIFT_INPUT_DELETE。 */
        KEY_LIFT_INPUT_DELETE = 0x1f,
        /* ── Desktop-session pager (2026-08-05). The watch face's RIGHT tile is one
           horizontally-snapping page per DESKTOP chat session (the Hermes conversations
           the phone already enumerates through AgentSessionBridge's convList). Opening a
           page reuses KEY_CONV_OPEN/SEND/CLOSE + KEY_CONV_STATE verbatim — these two keys
           only add the LIST that the pager pages over.
           Keep in lockstep with WatchProtocol.kt/.swift SKAILINK_KEY_CONV_LIST(_REQ). ── */
        /* phone→watch (DOWNLINK): {"device":{"id":"...","name":"..."},
             "sessions":[{"id":"conv:hermes:<id>","title":"...","preview":"..."}]}
           ONE PUSH PER DESKTOP — the watch upserts by device id and keeps a list per
           device, because the watch face's right side is one COLUMN per desktop (founder
           2026-08-10: 往左滑一次是第一台電腦的 sessions，再往左滑是第二台，每一欄從上面
           拉下來就是那一台的媒體頁). A single flat list could never say which desktop a
           row belongs to, so a second desktop would silently overwrite the first.
           Sessions are newest first, capped by the watch at SESSION_PAGER_MAX; devices at
           SESSION_DEVICE_MAX. "id" is the SAME identity KEY_CONV_OPEN takes. "preview" is
           the last turn's text, shown until that session is opened and real turns arrive
           on KEY_CONV_STATE.
           COMPATIBILITY: "device" absent = the single unnamed desktop (slot 0), which is
           exactly what the pre-2026-08-10 phone sends. */
        KEY_CONV_LIST = 0x20,
        /* watch→phone (UPLINK): {} for every desktop, or {"device":"<id>"} for one.
           Sent when the tile is built and whenever the watch reconnects; the Data path has
           no pull, so this is the watch's only way to recover a list it missed. */
        KEY_CONV_LIST_REQ = 0x21,
        /* ── TV remote (APP_ID_TV_REMOTE). The watch has no IP stack, so the TV app is a
           pure input surface: it emits BRAND-NEUTRAL verbs and the phone owns discovery,
           pairing and the per-vendor driver (Android TV Remote v2 / LG SSAP / Roku ECP /
           Samsung Tizen ws). Keeping the verbs neutral is what lets one watch UI drive
           every brand — never leak a vendor key name (KEY_VOLUP / Lit_) up here. ── */
        /* watch→phone (UPLINK): {"cmd":"<verb>"} — one remote key press. verb ∈
             power | up | down | left | right | ok | back | home
             volumeUp | volumeDown | mute | playPause
           plus two control verbs that carry no key:
             discover  = (re)scan the LAN and (re)bind a TV; answered by KEY_TV_STATE
             pair      = the user asked to (re)start pairing on the bound TV
           The phone maps each verb to the bound device's driver. Unmappable verbs on a
           given platform are dropped by the phone with a log — the watch does NOT hide
           buttons per platform, because the bound TV can change under it at any time. */
        KEY_TV_CONTROL = 0x22,
        /* phone→watch (DOWNLINK): {"name":"客廳電視","platform":"androidtv|webos|roku|
           samsung","state":"none|scanning|pairing|ready|error","detail":"..."} — which TV
           the phone is currently bound to, for the watch's status line. "state" drives the
           watch's dot colour AND its text (§1.4: colour is never the sole indicator):
             none     = nothing found on the LAN yet
             scanning = discovery in flight
             pairing  = needs the user to finish pairing ON THE PHONE (PIN entry / TV prompt)
             ready    = bound, keys will actuate
             error    = last command failed (detail carries a short reason)
           Pushed on every binding change and as the answer to cmd:"discover". */
        KEY_TV_STATE = 0x23,
        /* watch→phone (UPLINK): {"device":"<id>"} — create a NEW session ON THAT DESKTOP
           and open it.
           2026-08-10 (founder): the right tile now shows the session LIST first and the
           mic on that layer means "start a new conversation", so every other conv key —
           OPEN / SEND / CLOSE — is unusable here: all three require an id that does not
           exist yet.
           The device is NAMED rather than inferred: the phone used to resolve "which
           desktop" from whichever list it pushed last, which is unanswerable once two
           desktops are online (founder 2026-08-10 拍板帶 device id).
           The watch deliberately learns nothing back on this key. The phone asks that
           desktop to create the session, and the desktop's normal
           convListResult (→ KEY_CONV_LIST 0x20) carries the new row; the watch opens
           whatever arrived NEWEST-FIRST that it has not seen before. That keeps the new
           id off this wire entirely — no reply key, no correlation id, no watch-side
           pending state to leak if the desktop never answers.
           Keep in lockstep with WatchProtocol.kt/.swift SKAILINK_KEY_CONV_NEW. */
        KEY_CONV_NEW = 0x24,
        /* watch→phone (UPLINK): {"av":"<key>"} — 手錶清單上有一列的 Bot 頭像鍵是 <key>,
           但 /assets/images/bot/<key>.bin 不在手錶上,跟手機要那張圖。
           <key> 是手機自己算的**內容雜湊**(12 字 hex,隨頭像內容變),所以手機收到後
           不需要對帳「是哪個 Bot」—— 它照著同一支雜湊反查那張圖,畫成 100x100 圓形
           ezip RGB565,走既有的檔案通道(0x52/0x53/0x54)推到那個路徑。手錶端收檔後
           instruction_list_on_bot_avatar_file() 把列的右緣從設備名換成頭像。
           沒有回覆鍵:圖到了就是回覆,沒到就維持設備名(可見的降級)。
           Keep in lockstep with WatchProtocol.kt/.swift SKAILINK_KEY_CONV_AVATAR_REQ. */
        KEY_CONV_AVATAR_REQ = 0x25,
        /* 0x26 = deviceDrag(手機端 WatchProtocol.kt 已佔,韌體在 andrew_v29.5 分支)—— 這裡保留不用。 */
        /* gestureClick 0x27,雙向。watch→phone(UPLINK) {"on":1|0}:滑鼠 app 的「手勢點擊」
           模式切換(手錶錶面朝下 + 捏指 tap 翻轉;預設關)。開著時手錶照 RAW 收集那條
           路送 IMU+PPG(NOTIFY 0x50),按放判定由**手機**跑因果按壓模型(手錶塞不下 ~9MB),
           手機把每次判到的按下/放開當左鍵 down/up 送到 active 電腦(同 0x09 那條 relay)。
           phone→watch(DOWNLINK) {"on":1|0,"ok":1|0}:手機武裝/解除完成的回執;ok=0 =
           手機沒帶模型,手錶自己退出模式,免得畫著一隻手卻什麼都不會發生。
           Keep in lockstep with WatchProtocol.kt SKAILINK_KEY_GESTURE_CLICK. */
        KEY_GESTURE_CLICK = 0x27,
    } SKAI_LINK_KEY;

    /* Dispatched from communicate_parse.c for cmd_id == SKAI_LINK_COMMAND_ID.
       pValue/length are the reassembled L2 payload (UTF-8 JSON for 0x01-0x04). */
    void resolve_skailink_command(uint8_t key, uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATE_PARSE_SKAILINK_H */
