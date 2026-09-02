/**
 ******************************************************************************
 * @file   lv_chat_page.c
 * @brief  In-watch @-conversation chat room (P5). See lv_chat_page.h.
 ******************************************************************************
 */

#include "lvgl.h"
#include "lv_ext_resource_manager.h"
#include "app_mem.h" /* app_cache_alloc(IMAGE_CACHE_PSRAM) — per-session 轉錄快取 */
#include "lv_chat_page.h"
#include "communicate_task.h"
#include "ui_handler.h"
#include "ui_helper.h"
#include "bloc_v2t.h"
#include <cJSON.h>
#include <string.h>

#define DBG_TAG "lv_chat_page"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* Bottom voice control uses the shared mic glyph image (resource/images/.../micro_icon.png) — the
   same asset the recorder quick-setting + hid_mouse space-bar use. Icon-only, no button chrome. */
LV_IMG_DECLARE(micro_icon);
/* The send glyph shown (in place of the mic) while recording — tap it to stop + send the transcript. */
LV_IMG_DECLARE(icon_send);
/* The pill frame image behind the live transcript — the SAME asset the watch-face skaibar voice box
   uses (lv_instruction_list_layout.c), so the chat input box matches it pixel-for-pixel. */
LV_IMG_DECLARE(message_widget_bg);

/* The floating chat-room panel on lv_layer_top (above the @-list layer), or NULL when
   closed. A FRESH panel is built per open + torn down on close (no reuse). */
static lv_obj_t *s_chat_panel = NULL;
static lv_obj_t *s_title_label = NULL;

/* ── 螢幕診斷 HUD(2026-08-26)──
   這個組態拿不到手錶的即時 log:kReleaseMode=1、ULOG_OUTPUT_LVL_W 濾掉大半、
   RT_USING_FINSH 沒開所以 msh 不通,而黑盒子只在**當機重開**時才把 ring 推出來 ——
   一旦不當機,內部狀態就完全看不到。所以把關鍵計數直接畫在標題列上,由使用者唸回來。
   查完就把 CHAT_DEBUG_HUD 關掉。 */
#define CHAT_DEBUG_HUD 0
#if CHAT_DEBUG_HUD
/* 追加被擋下來的最後一個原因:
   1=沒訊息 2=則數變了 3=還沒對帳(live=0) 4=有 clarify
   5=最後一則是我講的 6=沒有變長 7=前綴雜湊對不上 8=環滿了 */
#define DBG_REJ(c) do { s_dbg_rej = (c); s_dbg_rejn++; } while (0)
static uint32_t s_dbg_rej;
static uint32_t s_dbg_rejn;
static uint32_t s_dbg_full;   /* 跑了幾次完整重建 */
static uint32_t s_dbg_append; /* 走了幾次追加快路徑 */
static uint32_t s_dbg_tick;   /* 揭露 timer 跑了幾拍 */
static uint32_t s_dbg_ins;    /* 實際呼叫 ins_text 幾次 */
static uint32_t s_dbg_setlen; /* 最後一次重建時,交給 label 的完整內容有多長 */
static int s_dbg_keep;       /* 最後一次重建時,決定保留幾 byte 已畫內容 */
#else
#define DBG_REJ(c) do { } while (0)
#endif
static lv_obj_t *s_msg_list = NULL;      /* scrollable flex column the messages render into */
static lv_obj_t *s_loading_label = NULL; /* "載入中…" centered on the panel, shown while empty */

/* Pending conv-state: parsed off the BLE thread (skai_chat_on_conv_state) into bounded STATIC
   buffers (no heap on the 4KB BLE stack), then rendered on the LVGL thread
   (chat_page_apply_pending_state) via LVGL_MSG_TYPE_REFRESH_CHAT. */
#define CHAT_MAX_MSGS 16

/* ■ 文字池(founder 2026-08-26)
   原本每則訊息各自一個固定陣列(16 × 256),兩個毛病同時存在:
     · 短訊息也持續佔掉整格 ——「嗨」跟 250 字一樣貴
     · 長訊息再怎麼調也卡死在那個格子大小,超過就默默被砍
   改成**所有訊息共用一塊固定大小的池**,每則只記 (offset, len):
     · 用多少算多少 —— 一串短訊息就擠得很緊
     · 單則可以拉到 CHAT_MSG_TEXT_MAX(遠比舊的 256/512 寬)
     · 總量硬上限 = 池子大小,全靜態、沒有 malloc —— 不會因為文字太多把記憶體吃光,
       也沒有動態配置的 use-after-free / 碎片化問題
   RAM:3072 + 16×16 ≈ 3.3KB(舊的 16×512 = 8.4KB),容量反而變大、佔用變小。 */
#define CHAT_TEXT_POOL 4096    /* 整間聊天室的文字總量硬上限 */
/* 單則上限。1024 是實測打臉出來的:founder 要「300 字小說」,Hermes 實際回了
   418 個字元(手機 log `textLen=418`)≈ 1250 bytes,尾巴被 1024 切掉。3072 ≈ 1000
   個中文字,單則可以吃掉大半個池子——長回覆本來就該是房裡的主角。 */
#define CHAT_MSG_TEXT_MAX 3072
typedef struct
{
    char role[12]; /* user / ai / assistant / incoming / outgoing */
    uint16_t off;  /* 文字在 s_text_pool 的起點 */
    uint16_t len;  /* 不含結尾 NUL */
} chat_msg_t;
/* +1 = 永久 NUL 哨兵,**永遠不寫入**。BLE 執行緒重填池子的同時 LVGL 可能正在畫上一版,
   讀到的 off 可能是舊的(指向池子中段而那裡已經沒有結尾 NUL)—— 有哨兵在,最壞情況也
   只是讀到池尾就停,不會一路 strlen 出界。無鎖交接下這是最後一道界線。 */
static char s_text_pool[CHAT_TEXT_POOL + 1];
static uint16_t s_pool_used;

static inline const char *chat_msg_text(const chat_msg_t *m)
{
    /* off 只可能落在 [0, CHAT_TEXT_POOL];越界就當空字串,絕不解出界指標。 */
    return (m->off <= CHAT_TEXT_POOL) ? (s_text_pool + m->off) : "";
}
/* ── 逐字揭露:走在文字池上的一個游標(2026-08-26 重寫)──
   原本設計成「只收手機送來的增量」,但實測打臉:整篇 1038 bytes 的回覆,手錶總共只收到
   10 包 conv_state,而且**最後一包就含著整篇**(HUD 的 R3.9 → R2.10)。手機那端一秒推
   幾百包沒錯,但 BLE 吞不下,絕大多數在手機的寫入佇列裡就被後來的蓋掉 —— 手錶這邊根本
   沒有「串流」,自然也沒有尾巴可以追加。所以揭露的來源改成**手錶自己已經拿到的完整
   文字**:文字池裡那一整段就在那裡,讓游標自己走過去。
   環形緩衝跟 BLE 端那條追加快路徑因此整個刪掉 —— 少 2KB RAM、少一整套跨執行緒交接。
   「來源回頭改了前面」還是守得住:重建時拿已畫長度的雜湊對一次,對不上就從頭畫。 */
static uint32_t chat_fnv1a(const char *p, size_t n)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++)
        h = (h ^ (uint8_t)p[i]) * 16777619u;
    return h;
}

/* 逐字是給「我在這裡等回覆」用的;翻舊訊息還要一個字一個字浮出來只是擋著人看
   (founder 2026-08-26)。
   第一版用「進房後的第一次重畫」當判準,錯了 —— 舊房的歷史是**一則一則補進來的**
   (手機推 msgs=0 → 1 → 2 → … 每則一包、間隔十幾毫秒),所以只有第一則吃到「直接
   出現」,後面每一則都被當成新內容而逐字,包括最後那則早就回完的 AI 回覆。
   該問的不是「是不是第一次重畫」,是「這一則是不是我在這裡等來的」:送出過(或手機
   說 sending)才翻真,進舊房讀歷史時是 false。 */
static bool s_live_turn;

static char s_pending_title[64];
static bool s_pending_sending;
static chat_msg_t s_pending_msgs[CHAT_MAX_MSGS];
static volatile int s_pending_msg_count; /* written LAST on BLE, read FIRST on LVGL */

/* ── per-session 轉錄快取(2026-08-30,PSRAM)──
   左右滑切房時,鄰居的內容要「滑進來就在」(founder:切換完才刷上去不行)。手機
   push 的每一份完整 conv_state 都帶 sid —— 順手在 BLE 端拓一份進快取;切房當下
   (chat_page_try_restore)命中就立刻整份畫出來,桌面繞一圈回來的 live 狀態稍後
   照常覆蓋。LRU 12 格 × ~4.4KB ≈ 53KB,放 PSRAM(同 session pager 倉庫的理由)。
   配置只在 LVGL 執行緒(chat_page_open);BLE 端指標 NULL 就跳過不拓。 */
#define CHAT_CACHE_ENTRIES 12
typedef struct
{
    char sid[64];        /* "conv:hermes:<id>";"" = 空格 */
    uint32_t used_tick;  /* LRU */
    uint16_t pool_used;
    int msg_count;
    chat_msg_t msgs[CHAT_MAX_MSGS];
    char pool[CHAT_TEXT_POOL + 1];
} chat_cache_entry_t;
static chat_cache_entry_t *s_chat_cache; /* [CHAT_CACHE_ENTRIES] */
/* 最近一包 conv_state 的 sid(BLE 執行緒寫;快取拓寫也在同一執行緒,不跨)。 */
static char s_state_sid[64];

static void chat_cache_ensure(void)
{
    if (s_chat_cache != NULL)
        return;
#ifdef BSP_USING_PC_SIMULATOR
    static chat_cache_entry_t s_chat_cache_store[CHAT_CACHE_ENTRIES];
    s_chat_cache = s_chat_cache_store;
#else
    void *blk = app_cache_alloc(sizeof(chat_cache_entry_t) * CHAT_CACHE_ENTRIES,
                                IMAGE_CACHE_PSRAM);
    if (blk == NULL)
    {
        LOG_E("chat cache PSRAM alloc failed");
        return;
    }
    memset(blk, 0, sizeof(chat_cache_entry_t) * CHAT_CACHE_ENTRIES);
    s_chat_cache = (chat_cache_entry_t *)blk;
#endif
}

static chat_cache_entry_t *chat_cache_find(const char *sid)
{
    if (s_chat_cache == NULL || sid == NULL || sid[0] == '\0')
        return NULL;
    for (int i = 0; i < CHAT_CACHE_ENTRIES; i++)
        if (strncmp(s_chat_cache[i].sid, sid, sizeof(s_chat_cache[i].sid)) == 0)
            return &s_chat_cache[i];
    return NULL;
}

/* BLE 執行緒:把剛解析好的 pending 狀態拓進 [sid] 的快取格(找同 sid,否則 LRU)。 */
static void chat_cache_store(const char *sid)
{
    if (s_chat_cache == NULL || sid == NULL || sid[0] == '\0' || s_pending_msg_count <= 0)
        return;
    chat_cache_entry_t *e = chat_cache_find(sid);
    if (e == NULL)
    {
        e = &s_chat_cache[0];
        for (int i = 0; i < CHAT_CACHE_ENTRIES; i++)
        {
            if (s_chat_cache[i].sid[0] == '\0')
            {
                e = &s_chat_cache[i];
                break;
            }
            if (s_chat_cache[i].used_tick < e->used_tick)
                e = &s_chat_cache[i];
        }
        strncpy(e->sid, sid, sizeof(e->sid) - 1);
        e->sid[sizeof(e->sid) - 1] = '\0';
    }
    e->used_tick = (uint32_t)rt_tick_get();
    e->pool_used = s_pool_used;
    e->msg_count = s_pending_msg_count;
    memcpy(e->msgs, s_pending_msgs, sizeof(e->msgs));
    memcpy(e->pool, s_text_pool, sizeof(e->pool));
}

/* 送出一句之後、AI 還沒開口的這段時間要有「在等」的表示(founder 2026-08-26)。
   本機旗標而不是只信手機的 `sending`:手機那邊 sending 何時翻真不在我們手上,
   而「我剛按了送出」是手錶自己最確定的事實。收到任何 AI 發言(或 clarify)就清掉。 */
static bool s_awaiting_reply = false;

/* 剛送出的那一句,在手機把它回音進 conv_state 之前先自己畫出來。
   ⚠ 這裡**不碰文字池** —— 池子由 BLE 執行緒獨佔填寫,LVGL 執行緒(送出是在這條上)
   若也去 memmove/改 offset,兩邊會撞在同一塊緩衝上。改成獨立的一格小緩衝、渲染時
   當成串尾多出來的一則 user 訊息畫上去;手機的 conv_state 一到就清掉(裡面已含這句)。 */
static char s_local_echo[192];

/* Pending clarify/approval (0x12 top-level `approval` {rid,q,opts:[{id,label}]}, 2026-08-13):
   the desktop agent is BLOCKED asking — render the question + one tappable chip per option.
   A chip tap answers on the 0x10 uplink as `\x01decision:<rid>\x1f<optionId>` (the phone routes
   it to convDecision; the `\x01newsession:` pattern). `id` is the ANSWER Hermes receives
   (clarify: the choice sentence itself), so it gets the bigger buffer and a UTF-8-safe cut. */
#define CHAT_APPR_OPTS 4
#define CHAT_APPR_TEXT 96
static char s_appr_rid[64];
static char s_appr_q[192];
static struct
{
    char id[CHAT_APPR_TEXT];
    char label[CHAT_APPR_TEXT];
} s_appr_opts[CHAT_APPR_OPTS];
static int s_appr_count;
static volatile bool s_appr_pending; /* written LAST on BLE, read FIRST on LVGL */

/* Bounded copy that never leaves a torn UTF-8 sequence at the end (a mid-sequence cut renders
   as tofu AND, for clarify, would send back a corrupted answer string). */
static void chat_copy_utf8(char *dst, size_t cap, const char *src)
{
    size_t n = strlen(src);
    if (n >= cap)
    {
        n = cap - 1;
        /* back up over any continuation bytes (10xxxxxx) so the cut lands on a boundary */
        while (n > 0 && ((unsigned char)src[n] & 0xC0) == 0x80)
            n--;
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
}

/* AI 回覆逐字浮現的狀態機定義在下方(Turn treatment 區)—— 關房時要清乾淨,所以先宣告。 */
static void chat_type_stop(void);
static void chat_type_timer_start(void);
static void chat_type_reset(void);
static void chat_wait_stop(void);

/* 把 text 放進文字池尾端,回寫 dst 的 (off,len)。UTF-8 邊界安全;放不下就切。 */
static void chat_pool_put(chat_msg_t *dst, const char *text)
{
    size_t avail = (size_t)(CHAT_TEXT_POOL - s_pool_used); /* 含結尾 NUL */
    size_t cap = (avail < (size_t)CHAT_MSG_TEXT_MAX + 1) ? avail : (size_t)CHAT_MSG_TEXT_MAX + 1;
    dst->off = s_pool_used;
    if (cap == 0)
    {
        dst->len = 0;
        return;
    }
    char *dstp = s_text_pool + s_pool_used;
    chat_copy_utf8(dstp, cap, text);
    dst->len = (uint16_t)strlen(dstp);
    s_pool_used = (uint16_t)(s_pool_used + dst->len + 1);
}

/* ── 重建合流(founder 2026-08-26「卡住不動,看門狗自動重啟」)──
   看門狗會咬,代表有一條執行緒一直在跑、把 idle 餓死 —— 不是「慢」,是「從不讓出 CPU」。
   兇手是 GUI:手機串流時一秒推幾百包 conv_state,只要有一包不符合追加條件就會觸發一次
   lv_obj_clean + 重建所有訊息泡泡(比 ins_text 更貴,長回覆要重排幾百個中文字形)。
   一秒幾十次這種重建,GUI 就再也沒空做別的。
   所以在這裡硬性設一個下限:250ms 內已經重建過就不重建,改掛一次性 timer 稍後補做
   (最後一份狀態一定會被畫出來,只是延後)。這道閘門跟動畫無關,是**不管上游怎麼洗頻
   都打不死 GUI** 的保險。 */
#define CHAT_REBUILD_MIN_MS 250

static uint32_t s_last_rebuild_tick;
static lv_timer_t *s_rebuild_defer;

static void chat_rebuild_defer_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    if (s_rebuild_defer != NULL)
    {
        lv_timer_del(s_rebuild_defer);
        s_rebuild_defer = NULL;
    }
    chat_page_apply_pending_state();
}

bool chat_page_is_open(void)
{
    return s_chat_panel != NULL && lv_obj_is_valid(s_chat_panel);
}

/* R69(founder:「可以先把我輸入的顯示出來」):開新對話時,session 要 ~4 秒才在桌面建好,
   房間在那之前是空的。把使用者剛講的那句話當成一則已送出的訊息**先畫上去** —— 它本來
   就是這個對話的第一句。等桌面的真實 conv_state 回來,apply_pending_state 會整份重畫
   (lv_obj_clean + 依 s_pending_msgs 重建),裡面已經含有這一句,所以不會變成兩則。 */
void chat_page_seed_local_message(const char *text)
{
    if (text == NULL || text[0] == '\0')
        return;
    s_pending_msg_count = 0; /* 這是新房間的第一句 */
    s_appr_pending = false;  /* 上個房間殘留的 clarify 不屬於新房間 */
    rt_strncpy(s_pending_msgs[0].role, "user", sizeof(s_pending_msgs[0].role) - 1);
    s_pending_msgs[0].role[sizeof(s_pending_msgs[0].role) - 1] = '\0';
    s_pool_used = 0; /* 新房間 = 空的池子 */
    s_local_echo[0] = 0; /* 新房間,上一房的本機回音不屬於這裡 */
    chat_pool_put(&s_pending_msgs[0], text);
    s_pending_msg_count = 1; /* count 最後寫(與 BLE 側同一個順序約定) */
    s_awaiting_reply = true; /* 這句剛送出去,接下來就是在等 AI */
    s_live_turn = true;      /* 這一輪的回覆要逐字 */
    chat_page_apply_pending_state();
}

/* SWIPE-BACK is handled by the watch's NATIVE left-edge back gesture (display_gesture_detect_objs
   → ESC → handle_back_event), which now checks chat_page_is_open() FIRST and closes this room —
   see watch_demo.c. This panel is left NON-clickable (below) so the edge swipe passes through to
   the native edge detect object instead of being captured here (LVGL LV_EVENT_GESTURE / a manual
   drag both failed: a clickable panel starves the native edge object; a non-clickable one gets no
   LVGL events — so the native path is the only one that works for a full-screen layer_top overlay). */

/* ── Voice input (mic → V2T → commu_send_conv_send) ──
   start_voice_recognition streams the mic to the phone, which transcribes and streams the result back
   as a VOICE_RECOGNITION_PAYLOAD → interact_voice_recognition (watch_system_interact.c). While this
   chat room is open, that dispatch accumulates the transcript via handle_v2t_result. The mic button
   TOGGLES: tap to START recording, tap again to STOP + send the accumulated transcript. */
/* start_voice_recognition / stop_voice_recognition / get_combined_voice2text / clearVoice2Text +
   the `voice_provider` (vad_init/vad_deinit) come from bloc_v2t.h — the SAME header app_speech uses.
   vad_init BEFORE start is REQUIRED (app_speech does it): without it the mic captures but the VAD/
   encoding pipeline isn't set up, so the phone gets no usable audio → an EMPTY transcript. */
#define CHAT_V2T_INTENT 2 /* V2T_INTENT_REMOTE_INPUT — "chat reply" (watch_system_interact.h) */

static lv_obj_t *s_mic_btn = NULL; /* the clickable mic glyph (== s_mic_img) */
static lv_obj_t *s_mic_img = NULL;
static lv_obj_t *s_mic_backplate = NULL; /* separates the floating glyph from transcript text */
static lv_obj_t *s_send_btn = NULL;         /* the send glyph shown in the mic's place while recording */
static lv_obj_t *s_transcript_box = NULL;   /* live V2T transcript input box, shown while recording */
static lv_obj_t *s_transcript_label = NULL;
static lv_obj_t *s_transcript_pill = NULL;  /* message_widget_bg frame image inside the box (faded in on open) */
static lv_obj_t *s_input_scrim = NULL;      /* full-screen tap-catcher (recording only): tap OUTSIDE the box → cancel */
static bool s_recording = false;

static void chat_set_mic_visual(bool recording)
{
    /* While recording the mic glyph is REPLACED by the input box + a send glyph (founder 2026-06-29):
       hide the mic, show the box (listening hint until words land) + the send icon; reverse when idle. */
    if (s_mic_btn != NULL && lv_obj_is_valid(s_mic_btn))
    {
        if (recording)
            lv_obj_add_flag(s_mic_btn, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(s_mic_btn, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_mic_backplate != NULL && lv_obj_is_valid(s_mic_backplate))
    {
        if (recording)
            lv_obj_add_flag(s_mic_backplate, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(s_mic_backplate, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_send_btn != NULL && lv_obj_is_valid(s_send_btn))
    {
        if (recording)
        {
            lv_obj_clear_flag(s_send_btn, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_img_opa(s_send_btn, LV_OPA_40, 0); /* dim until words arrive */
        }
        else
            lv_obj_add_flag(s_send_btn, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_transcript_box != NULL && lv_obj_is_valid(s_transcript_box))
    {
        if (recording)
        {
            lv_obj_clear_flag(s_transcript_box, LV_OBJ_FLAG_HIDDEN);
            if (s_transcript_label != NULL && lv_obj_is_valid(s_transcript_label))
                lv_label_set_text(s_transcript_label, "聆聽中…");
        }
        else
        {
            lv_obj_add_flag(s_transcript_box, LV_OBJ_FLAG_HIDDEN);
        }
    }
    /* The tap-outside-to-cancel catcher follows the box: live while recording, gone when idle. */
    if (s_input_scrim != NULL && lv_obj_is_valid(s_input_scrim))
    {
        if (recording)
            lv_obj_clear_flag(s_input_scrim, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(s_input_scrim, LV_OBJ_FLAG_HIDDEN);
    }
}

/* LVGL-thread update of the live mic transcript — called via the shared voice router
   (refresh_ai_chat_input_message → chat branch), which delivers the live PARTIAL text the watch-face
   skaibar voice box uses. [text] is the running transcript (empty → keep the listening hint). */
void chat_page_set_transcript(const char *text)
{
    if (!s_recording || s_transcript_label == NULL || !lv_obj_is_valid(s_transcript_label))
        return;
    bool has = (text != NULL && text[0] != '\0');
    lv_label_set_text(s_transcript_label, has ? text : "聆聽中…");
    /* Dim the send glyph until there are words to send (desktop CanSend parity). */
    if (s_send_btn != NULL && lv_obj_is_valid(s_send_btn))
        lv_obj_set_style_img_opa(s_send_btn, has ? LV_OPA_COVER : LV_OPA_40, 0);
}

/* ── mic → input-box MORPH (mirrors the @-list's lmic_grow_cb so the chat box opens with the SAME
   grow-and-slide-up transition, not a hard pop — founder 2026-06-29). Phase 1: the dark box grows from
   a slim pill at the mic up into the full 442×252 box while the mic glyph fades out. Phase 2: the pill
   frame + transcript + send fade in over the grown backdrop. Close reverses phase 1. Geometry/timing
   mirror the list: collapsed≈slim pill at the mic, open == 442×252 @ +75 (matches device_pager SKAIB). */
#define CBOX_W0 160 /* collapsed (at-mic) box */
#define CBOX_H0 44
#define CBOX_Y0 (-8)  /* sits where the mic glyph is (BOTTOM_MID -8) */
#define CBOX_R0 18
#define CBOX_W1 442 /* open box (== @-list LBOX) */
#define CBOX_H1 252
#define CBOX_Y1 75
#define CBOX_R1 80
#define CBOX_GROW_MS 220 /* == LMORPH_GROW_MS */
#define CBOX_FADE_MS 160 /* == LMORPH_FRAME_MS */

static void chat_pill_fade_cb(void *var, int32_t v)
{
    (void)var;
    if (s_transcript_pill != NULL && lv_obj_is_valid(s_transcript_pill))
        lv_obj_set_style_img_opa(s_transcript_pill, (lv_opa_t)v, 0);
}

/* Interpolate the box between the collapsed (f=0) and open (f=255) geometry; fade the mic glyph out as
   the box grows (mic img_opa = 255-f), exactly as lmic_grow_cb dissolves the @-list's bar glyph. */
static void chat_box_grow_cb(void *var, int32_t f)
{
    (void)var;
    if (s_transcript_box == NULL || !lv_obj_is_valid(s_transcript_box))
        return;
    lv_obj_set_size(s_transcript_box, CBOX_W0 + (CBOX_W1 - CBOX_W0) * f / 255,
                    CBOX_H0 + (CBOX_H1 - CBOX_H0) * f / 255);
    lv_obj_align(s_transcript_box, LV_ALIGN_BOTTOM_MID, 0, CBOX_Y0 + (CBOX_Y1 - CBOX_Y0) * f / 255);
    lv_obj_set_style_radius(s_transcript_box, CBOX_R0 + (CBOX_R1 - CBOX_R0) * f / 255, 0);
    if (s_mic_btn != NULL && lv_obj_is_valid(s_mic_btn))
        lv_obj_set_style_img_opa(s_mic_btn, (lv_opa_t)(255 - f), 0);
    if (s_mic_backplate != NULL && lv_obj_is_valid(s_mic_backplate))
        lv_obj_set_style_opa(s_mic_backplate, (lv_opa_t)(255 - f), 0);
}

/* Phase 2 (open): the box finished growing → hide the now-faded mic, then fade the pill frame +
   transcript + send in over the backdrop (mirrors lmic_open_reveal_cb). */
static void chat_open_reveal_cb(lv_anim_t *a)
{
    (void)a;
    if (!s_recording)
        return; /* cancelled mid-grow — leave the close path to clean up */
    chat_box_grow_cb(NULL, 255); /* pin exact open geometry */
    if (s_mic_btn != NULL && lv_obj_is_valid(s_mic_btn))
        lv_obj_add_flag(s_mic_btn, LV_OBJ_FLAG_HIDDEN);
    if (s_mic_backplate != NULL && lv_obj_is_valid(s_mic_backplate))
        lv_obj_add_flag(s_mic_backplate, LV_OBJ_FLAG_HIDDEN);
    if (s_transcript_label != NULL && lv_obj_is_valid(s_transcript_label))
        lv_obj_set_style_text_opa(s_transcript_label, LV_OPA_80, 0);
    if (s_send_btn != NULL && lv_obj_is_valid(s_send_btn))
    {
        lv_obj_clear_flag(s_send_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_img_opa(s_send_btn, LV_OPA_40, 0); /* dim until words arrive */
    }
    if (s_transcript_pill != NULL && lv_obj_is_valid(s_transcript_pill))
    {
        chat_pill_fade_cb(NULL, 0); /* start invisible */
        lv_anim_t fr;
        lv_anim_init(&fr);
        lv_anim_set_var(&fr, s_transcript_pill);
        lv_anim_set_values(&fr, 0, 255);
        lv_anim_set_time(&fr, CBOX_FADE_MS);
        lv_anim_set_path_cb(&fr, lv_anim_path_ease_out);
        lv_anim_set_exec_cb(&fr, chat_pill_fade_cb);
        lv_anim_start(&fr);
    }
}

static void chat_play_open_morph(void)
{
    if (s_transcript_box == NULL || !lv_obj_is_valid(s_transcript_box))
        return;
    /* Scrim live from the start so a tap-outside even during the morph cancels. */
    if (s_input_scrim != NULL && lv_obj_is_valid(s_input_scrim))
        lv_obj_clear_flag(s_input_scrim, LV_OBJ_FLAG_HIDDEN);
    /* Box visible but content invisible (the frame fades in at phase 2); mic full + visible (fades out
       via the grow cb); send hidden until phase 2. */
    lv_obj_clear_flag(s_transcript_box, LV_OBJ_FLAG_HIDDEN);
    if (s_transcript_pill != NULL && lv_obj_is_valid(s_transcript_pill))
        lv_obj_set_style_img_opa(s_transcript_pill, LV_OPA_TRANSP, 0);
    if (s_transcript_label != NULL && lv_obj_is_valid(s_transcript_label))
    {
        lv_label_set_text(s_transcript_label, "聆聽中…");
        lv_obj_set_style_text_opa(s_transcript_label, LV_OPA_TRANSP, 0);
    }
    if (s_mic_btn != NULL && lv_obj_is_valid(s_mic_btn))
    {
        lv_obj_clear_flag(s_mic_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_img_opa(s_mic_btn, LV_OPA_COVER, 0);
    }
    if (s_mic_backplate != NULL && lv_obj_is_valid(s_mic_backplate))
    {
        lv_obj_clear_flag(s_mic_backplate, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_opa(s_mic_backplate, LV_OPA_COVER, 0);
    }
    if (s_send_btn != NULL && lv_obj_is_valid(s_send_btn))
        lv_obj_add_flag(s_send_btn, LV_OBJ_FLAG_HIDDEN);
    lv_anim_del(s_transcript_box, chat_box_grow_cb);
    chat_box_grow_cb(NULL, 0); /* collapsed geometry, mic full */
    lv_anim_t g;
    lv_anim_init(&g);
    lv_anim_set_var(&g, s_transcript_box);
    lv_anim_set_values(&g, 0, 255);
    lv_anim_set_time(&g, CBOX_GROW_MS);
    lv_anim_set_path_cb(&g, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&g, chat_box_grow_cb);
    lv_anim_set_ready_cb(&g, chat_open_reveal_cb);
    lv_anim_start(&g);
}

static void chat_close_done_cb(lv_anim_t *a)
{
    (void)a;
    if (s_transcript_box != NULL && lv_obj_is_valid(s_transcript_box))
        lv_obj_add_flag(s_transcript_box, LV_OBJ_FLAG_HIDDEN);
    if (s_mic_btn != NULL && lv_obj_is_valid(s_mic_btn))
        lv_obj_set_style_img_opa(s_mic_btn, LV_OPA_COVER, 0); /* mic fully back */
    if (s_mic_backplate != NULL && lv_obj_is_valid(s_mic_backplate))
        lv_obj_set_style_opa(s_mic_backplate, LV_OPA_COVER, 0);
}

/* Reverse the open morph: drop the content, shrink the backdrop back to the mic while the mic fades in,
   then hide the box (mirrors the @-list's close). Falls back to an instant reset if nothing is showing. */
static void chat_play_close_morph(void)
{
    if (s_transcript_box == NULL || !lv_obj_is_valid(s_transcript_box) ||
        lv_obj_has_flag(s_transcript_box, LV_OBJ_FLAG_HIDDEN))
    {
        chat_set_mic_visual(false);
        return;
    }
    lv_anim_del(s_transcript_pill, chat_pill_fade_cb);
    if (s_transcript_pill != NULL && lv_obj_is_valid(s_transcript_pill))
        lv_obj_set_style_img_opa(s_transcript_pill, LV_OPA_TRANSP, 0);
    if (s_transcript_label != NULL && lv_obj_is_valid(s_transcript_label))
        lv_obj_set_style_text_opa(s_transcript_label, LV_OPA_TRANSP, 0);
    if (s_send_btn != NULL && lv_obj_is_valid(s_send_btn))
        lv_obj_add_flag(s_send_btn, LV_OBJ_FLAG_HIDDEN);
    if (s_input_scrim != NULL && lv_obj_is_valid(s_input_scrim))
        lv_obj_add_flag(s_input_scrim, LV_OBJ_FLAG_HIDDEN);
    if (s_mic_btn != NULL && lv_obj_is_valid(s_mic_btn))
        lv_obj_clear_flag(s_mic_btn, LV_OBJ_FLAG_HIDDEN); /* fades IN via the grow cb (f:255→0) */
    if (s_mic_backplate != NULL && lv_obj_is_valid(s_mic_backplate))
        lv_obj_clear_flag(s_mic_backplate, LV_OBJ_FLAG_HIDDEN);
    lv_anim_del(s_transcript_box, chat_box_grow_cb);
    lv_anim_t g;
    lv_anim_init(&g);
    lv_anim_set_var(&g, s_transcript_box);
    lv_anim_set_values(&g, 255, 0);
    lv_anim_set_time(&g, CBOX_GROW_MS);
    lv_anim_set_path_cb(&g, lv_anim_path_ease_in);
    lv_anim_set_exec_cb(&g, chat_box_grow_cb);
    lv_anim_set_ready_cb(&g, chat_close_done_cb);
    lv_anim_start(&g);
}

static void chat_stop_recording_and_send(void)
{
    if (!s_recording)
        return;
    s_recording = false;
    voice_provider.auto_stop_listening(); /* finalize (mirror app_skai's send_to_ai) */
    chat_play_close_morph();
    const char *text = get_combined_voice2text();
    /* 「文字出現一瞬間就消失、也沒送出去」(founder 2026-08-17):轉錄確實到了手錶
       (`[v2t] rx len=15` 等),chat 分支也認領了,但送出這一刻拿到空的。V2T 緩衝是**單一
       全域**、有六個 app 都會 clearVoice2Text(),沒有任何所有權概念 —— 所以要先知道
       這裡到底拿到什麼,才能判斷是被別人清掉還是根本沒累積。 */
    LOG_D("[chat] mic stop -> combined len=%d", text ? (int)strlen(text) : -1);
    if (text != NULL && text[0] != '\0')
    {
        commu_send_conv_send(text);
        clearVoice2Text();
        /* 不等手機回音:自己那句馬上上牆,並立刻進入「等待中」。 */
        chat_copy_utf8(s_local_echo, sizeof(s_local_echo), text);
        s_awaiting_reply = true;
        s_live_turn = true;
        chat_page_apply_pending_state();
    }
}

/* Tap OUTSIDE the input box (on the transparent catcher) → leave voice-input mode WITHOUT sending:
   stop the mic, drop the partial transcript, restore the idle mic glyph (founder 2026-06-29). */
static void chat_cancel_recording(void)
{
    if (!s_recording)
        return;
    s_recording = false;
    voice_provider.auto_stop_listening();
    clearVoice2Text();
    chat_play_close_morph();
    LOG_I("chat mic: cancel (tap outside)");
}

static void chat_scrim_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED)
        chat_cancel_recording();
}

static void chat_mic_toggle(void)
{
    if (!s_recording)
    {
        clearVoice2Text();
        /* Use the PROVEN voice path app_skai's AI widget uses (voice_provider.start_v2t): the bare
           start_voice_recognition didn't trigger the phone-side transcription (it returned
           KEY_VOICE_RECOGNITION_END:0 — no text). start_v2t does the full setup + tells the phone to
           listen. The result streams back via interact_voice_recognition → our chat branch (FIRST). */
        voice_provider.start_v2t();
        s_recording = true;
        chat_play_open_morph();
        LOG_I("chat mic: recording start");
    }
    else
    {
        LOG_I("chat mic: stop + send");
        chat_stop_recording_and_send();
    }
}

static void chat_mic_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    chat_mic_toggle();
}

/* Global RELEASE gesture hook (gesture_recognition_task.c): the @-list stays "open" underneath
   the chat panel for the whole session (back-gesture needs it — see chat_page_is_open() note
   above), so the release-gesture dispatcher's is_at_instruction_list() check can't tell chat
   apart from the list. Callers must check chat_page_is_open() FIRST and route here instead of
   animate_open_ai_widget(), or release silently opens the (hidden, covered) list AI widget. */
void chat_page_start_voice_input(void)
{
    if (!chat_page_is_open())
        return;
    chat_mic_toggle();
}

/* ── 聊天室左右滑切換 session(2026-08-30,滑鼠抽屜 bot 房)──────────────────
   tileview 手感:內容跟著手指平移,過門檻放開才換房(founder 2026-08-30:
   「我希望他用像tileview的方式切換」)。第一版走 LV_EVENT_GESTURE,真機沒反應
   —— 訊息列表是垂直捲動物件,手指只要先累積 ~10px 垂直位移 LVGL 就把 scroll_obj
   釘在列表上,之後整個手勢的 GESTURE 事件被捲動抑制,永遠輪不到我們(同
   reference_watch_gesture_scroll_deadend 的教訓)。正解 = repo 慣例的「catcher
   當捲動死路」:一片透明、**不可捲動**的 catcher 蓋在訊息區上自己收 press,
   軸向自己判 —— 垂直=手動捲列表(帶簡易慣性),水平=平移列表跟手;點一下=
   把 CLICKED 轉送給底下的 clarify chip。catcher 平時 HIDDEN,只有註冊了
   switcher 的房(滑鼠抽屜 bot 房)才現身 —— 錶盤 '@' 房保持原生捲動手感。
   左緣右滑仍是返回:native 邊緣偵測物件疊在 catcher 之上。 */
static lv_obj_t *s_swipe_catcher = NULL;
/* commit=false 只查詢:這個方向有沒有鄰居?有就把 sid/title 填進 out(可 NULL)。
   commit=true 才真的換(呼叫端更新自己的狀態 + 送 conv_open;視覺已由 chat page
   處理完)。dir=+1 滑左(更舊)、-1 滑右(更新)。 */
static chat_session_switch_cb_t s_session_switch_cb = NULL;

void chat_page_set_session_switcher(chat_session_switch_cb_t cb)
{
    s_session_switch_cb = cb;
    if (s_swipe_catcher != NULL && lv_obj_is_valid(s_swipe_catcher))
    {
        if (cb != NULL)
            lv_obj_clear_flag(s_swipe_catcher, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(s_swipe_catcher, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 換房(就地,不拆面板):標題換字、清空泡泡、回到「載入中」,揭露/等待/clarify
   全部歸零 —— 新房的 conv_state 一到就正常補畫。滑入滑出動畫由 catcher 驅動,
   這裡只換內容。 */
void chat_page_switch_session(const char *title)
{
    if (!chat_page_is_open())
        return;
    if (s_title_label != NULL && lv_obj_is_valid(s_title_label))
        lv_label_set_text(s_title_label, (title && title[0]) ? title : "聊天室");
    if (s_msg_list != NULL && lv_obj_is_valid(s_msg_list))
        lv_obj_clean(s_msg_list);
    if (s_loading_label != NULL && lv_obj_is_valid(s_loading_label))
        lv_obj_clear_flag(s_loading_label, LV_OBJ_FLAG_HIDDEN);
    s_pending_msg_count = 0;
    s_pool_used = 0;
    s_appr_pending = false;
    s_local_echo[0] = 0;
    s_awaiting_reply = false;
    s_live_turn = false;
    chat_wait_stop();
    chat_type_reset();
    /* reset 內含 chat_type_stop(刪 timer)—— 那是給**關房**用的。這裡房間還開著,
       timer 必須活著,否則之後每一則 live AI 回覆都停在「空 label 等打字」,要到
       下一次整份重畫才現形(founder 2026-08-30:「再次輸入東西後上個的AI回復就會
       出現」)。教訓同 2026-08-26:揭露 timer 開房建立、關房刪除、**中間永不停**。 */
    chat_type_timer_start();
    s_last_rebuild_tick = 0;
    if (s_rebuild_defer != NULL)
    {
        lv_timer_del(s_rebuild_defer);
        s_rebuild_defer = NULL;
    }
}

/* 切房/開房後試著用快取立即補畫(LVGL thread):命中就把那個 session 最後已知的
   整份轉錄搬回 pending+文字池並立刻渲染 —— 內容「滑進來就在」;桌面繞一圈回來的
   live conv_state 稍後照常覆蓋。回傳 false = 沒看過這個 session,維持「載入中」。 */
bool chat_page_try_restore(const char *sid)
{
    if (!chat_page_is_open())
        return false;
    chat_cache_entry_t *e = chat_cache_find(sid);
    if (e == NULL || e->msg_count <= 0)
        return false;
    memcpy(s_text_pool, e->pool, sizeof(s_text_pool));
    memcpy(s_pending_msgs, e->msgs, sizeof(s_pending_msgs));
    s_pool_used = e->pool_used;
    s_pending_sending = false;
    s_appr_pending = false;
    s_live_turn = false; /* 快取是歷史,不逐字 */
    e->used_tick = (uint32_t)rt_tick_get();
    s_pending_msg_count = e->msg_count;
    chat_page_apply_pending_state();
    LOG_W("[chat] cache hit sid=%.24s msgs=%d", sid, e->msg_count);
    return true;
}

/* ── catcher 手勢狀態機 ── */
#define CHAT_HS_MODE_IDLE 0
#define CHAT_HS_MODE_VSCROLL 1
#define CHAT_HS_MODE_HSWIPE 2
#define CHAT_HS_DECIDE_V 12  /* 垂直先過這麼多 = 捲動 */
#define CHAT_HS_DECIDE_H 28  /* 水平過這麼多且明顯佔優 = 換房拖曳 */
#define CHAT_HS_COMMIT_PX (LV_HOR_RES / 4)
static lv_point_t s_hs_last;
static int s_hs_dx, s_hs_dy;
static int s_hs_mode;
static int s_hs_last_vy; /* 慣性:最後一拍的垂直速度 */

/* ── 鄰居預建頁(2026-08-30 第二版,founder:「滑動過程中就要看到下一份的內容
   跟著進來」)──
   拖曳一判定成 HSWIPE 就把該方向的鄰居 session 用快取**先渲染成第二頁**(標題+
   轉錄容器,擺在 ±螢幕寬),之後兩頁一起跟手平移 —— 內容在滑動過程中就進來,
   放開只是把動畫走完、把第二頁升格成現任(舊頁刪除、conv_open 補發)。快取沒有
   的房間第二頁畫「載入中…」,live conv_state 落地後照常補畫。 */
static lv_obj_t *s_next_list = NULL;
static lv_obj_t *s_next_title = NULL;
static int s_next_dir = 0; /* s_next_* 是哪個方向的鄰居;0 = 沒建 */
static char s_next_sid[64];
static char s_next_title_txt[64];

static lv_obj_t *chat_add_hermes_turn(lv_obj_t *parent, const char *text, bool mine);

static void chat_next_page_destroy(void)
{
    if (s_next_list != NULL && lv_obj_is_valid(s_next_list))
        lv_obj_del(s_next_list);
    if (s_next_title != NULL && lv_obj_is_valid(s_next_title))
        lv_obj_del(s_next_title);
    s_next_list = NULL;
    s_next_title = NULL;
    s_next_dir = 0;
}

/* 跟 chat_page_open 的訊息容器同幾何的第二個轉錄容器。 */
static lv_obj_t *chat_transcript_container(void)
{
    lv_obj_t *list = lv_obj_create(s_chat_panel);
    lv_obj_set_size(list, LV_HOR_RES - 26, LV_VER_RES - 78);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 66);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list, 6, 0);
    lv_obj_set_style_pad_hor(list, 2, 0);
    lv_obj_set_style_pad_top(list, 180, 0);
    lv_obj_set_style_pad_bottom(list, 180, 0);
    lv_obj_clear_flag(list, LV_OBJ_FLAG_CLICKABLE); /* catcher 在上面收手勢 */
    return list;
}

/* 建 [dir] 方向的鄰居頁;沒有鄰居就不建(拖曳變 rubber-band,放開彈回)。 */
static void chat_next_page_build(int dir)
{
    if (s_next_dir == dir)
        return;
    chat_next_page_destroy();
    if (s_session_switch_cb == NULL)
        return;
    char sid[64];
    char title[64];
    sid[0] = '\0';
    title[0] = '\0';
    if (!s_session_switch_cb(dir, false, sid, sizeof(sid), title, sizeof(title)))
        return;
    lv_obj_t *list = chat_transcript_container();
    chat_cache_entry_t *e = chat_cache_find(sid);
    if (e != NULL && e->msg_count > 0)
    {
        for (int i = 0; i < e->msg_count && i < CHAT_MAX_MSGS; i++)
        {
            const chat_msg_t *m = &e->msgs[i];
            const char *text = (m->off <= CHAT_TEXT_POOL) ? (e->pool + m->off) : "";
            bool mine = (strcmp(m->role, "user") == 0 || strcmp(m->role, "outgoing") == 0);
            chat_add_hermes_turn(list, text, mine);
        }
        lv_obj_update_layout(list);
        lv_obj_scroll_to_y(list, LV_COORD_MAX, LV_ANIM_OFF); /* 停在最新一則 */
        e->used_tick = (uint32_t)rt_tick_get();
    }
    else
    {
        lv_obj_t *hint = lv_label_create(list);
        lv_label_set_text(hint, "載入中…");
        lv_obj_set_style_text_font(hint, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
        lv_obj_set_style_text_color(hint, lv_color_hex(0x888888), 0);
    }
    lv_obj_t *tl = lv_label_create(s_chat_panel);
    lv_label_set_long_mode(tl, LV_LABEL_LONG_DOT);
    lv_obj_set_width(tl, LV_HOR_RES - 120);
    lv_obj_set_style_text_align(tl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(tl, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(tl, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(tl, title[0] ? title : "聊天室");
    lv_obj_align(tl, LV_ALIGN_TOP_MID, 0, 18);
    /* z-order:壓回訊息層 —— 剛建的物件在最上面,會蓋住 catcher/麥克風/錄音 UI。 */
    if (s_msg_list != NULL && lv_obj_is_valid(s_msg_list))
    {
        lv_obj_move_to_index(list, (int32_t)lv_obj_get_index(s_msg_list) + 1);
        lv_obj_move_to_index(tl, (int32_t)lv_obj_get_index(list) + 1);
    }
    s_next_list = list;
    s_next_title = tl;
    s_next_dir = dir;
    strncpy(s_next_sid, sid, sizeof(s_next_sid) - 1);
    s_next_sid[sizeof(s_next_sid) - 1] = '\0';
    strncpy(s_next_title_txt, title, sizeof(s_next_title_txt) - 1);
    s_next_title_txt[sizeof(s_next_title_txt) - 1] = '\0';
    LOG_W("[chat] next page built dir=%d sid=%.24s cached=%d", dir, sid,
          (int)(e != NULL && e->msg_count > 0));
}

/* 兩頁一起平移:現任(內容+標題)在 x,鄰居在 x±螢幕寬。 */
static void chat_hslide_apply(lv_coord_t x)
{
    if (s_msg_list != NULL && lv_obj_is_valid(s_msg_list))
        lv_obj_set_style_translate_x(s_msg_list, x, 0);
    if (s_title_label != NULL && lv_obj_is_valid(s_title_label))
        lv_obj_set_style_translate_x(s_title_label, x, 0);
    if (s_next_dir != 0)
    {
        lv_coord_t off = (s_next_dir > 0) ? LV_HOR_RES : -LV_HOR_RES;
        if (s_next_list != NULL && lv_obj_is_valid(s_next_list))
            lv_obj_set_style_translate_x(s_next_list, x + off, 0);
        if (s_next_title != NULL && lv_obj_is_valid(s_next_title))
            lv_obj_set_style_translate_x(s_next_title, x + off, 0);
    }
}

static void chat_hslide_exec(void *var, int32_t v)
{
    (void)var;
    chat_hslide_apply((lv_coord_t)v);
}

/* 動畫走完(現任滑出、鄰居落到 0):鄰居升格成現任。舊頁刪除;pending 換成鄰居的
   快取內容(遲到的 REFRESH_CHAT 重畫的才是**這間房**的東西);房間狀態(揭露/等待/
   clarify/回音)全歸零;最後補發 conv_open(commit=true 交給 switcher 的資料端)。 */
static void chat_hslide_commit_done(lv_anim_t *a)
{
    (void)a;
    int dir = s_next_dir;
    if (dir == 0 || s_next_list == NULL)
        return;
    if (s_msg_list != NULL && lv_obj_is_valid(s_msg_list))
        lv_obj_del(s_msg_list);
    if (s_title_label != NULL && lv_obj_is_valid(s_title_label))
        lv_obj_del(s_title_label);
    s_msg_list = s_next_list;
    s_title_label = s_next_title;
    s_next_list = NULL;
    s_next_title = NULL;
    s_next_dir = 0;
    lv_obj_set_style_translate_x(s_msg_list, 0, 0);
    if (s_title_label != NULL)
        lv_obj_set_style_translate_x(s_title_label, 0, 0);
    /* pending/池 ← 鄰居的快取:內容已經畫在升格的容器裡,這一步只是讓之後任何
       整份重畫畫出同樣的東西,而不是上一間房的殘影。 */
    chat_cache_entry_t *e = chat_cache_find(s_next_sid);
    if (e != NULL && e->msg_count > 0)
    {
        memcpy(s_text_pool, e->pool, sizeof(s_text_pool));
        memcpy(s_pending_msgs, e->msgs, sizeof(s_pending_msgs));
        s_pool_used = e->pool_used;
        s_pending_msg_count = e->msg_count;
        if (s_loading_label != NULL && lv_obj_is_valid(s_loading_label))
            lv_obj_add_flag(s_loading_label, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        s_pending_msg_count = 0;
        s_pool_used = 0;
        /* 房裡只有「載入中…」提示,共用的置中 loading label 不再重複顯示。 */
    }
    s_pending_sending = false;
    s_appr_pending = false;
    s_local_echo[0] = 0;
    s_awaiting_reply = false;
    s_live_turn = false;
    chat_wait_stop();
    chat_type_reset();
    chat_type_timer_start(); /* 房間還開著 —— 揭露 timer 中間永不停(見 switch_session) */
    s_last_rebuild_tick = 0;
    if (s_rebuild_defer != NULL)
    {
        lv_timer_del(s_rebuild_defer);
        s_rebuild_defer = NULL;
    }
    if (s_session_switch_cb != NULL)
        s_session_switch_cb(dir, true, NULL, 0, NULL, 0);
}

static void chat_hslide_cancel_done(lv_anim_t *a)
{
    (void)a;
    chat_next_page_destroy();
    chat_hslide_apply(0);
}

static void chat_hslide_settle(int from, int to, uint16_t ms, lv_anim_ready_cb_t done)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_msg_list);
    lv_anim_set_values(&a, from, to);
    lv_anim_set_time(&a, ms);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_exec_cb(&a, chat_hslide_exec);
    if (done != NULL)
        lv_anim_set_ready_cb(&a, done);
    lv_anim_start(&a);
}

/* 點一下(沒拖成任何模式):把 CLICKED 轉送給 catcher 底下、訊息列表裡命中的
   clarify chip(唯一可點的東西)。catcher 蓋住了 hit-test,不轉送 chips 就死了。 */
static void chat_catcher_forward_click(lv_point_t *p)
{
    if (s_msg_list == NULL || !lv_obj_is_valid(s_msg_list))
        return;
    uint32_t n = lv_obj_get_child_cnt(s_msg_list);
    for (uint32_t i = 0; i < n; i++)
    {
        lv_obj_t *child = lv_obj_get_child(s_msg_list, i);
        if (child == NULL || !lv_obj_has_flag(child, LV_OBJ_FLAG_CLICKABLE))
            continue;
        if (lv_obj_has_flag(child, LV_OBJ_FLAG_HIDDEN))
            continue;
        lv_area_t coords;
        lv_obj_get_coords(child, &coords);
        if (p->x >= coords.x1 && p->x <= coords.x2 && p->y >= coords.y1 && p->y <= coords.y2)
        {
            lv_event_send(child, LV_EVENT_CLICKED, NULL);
            return;
        }
    }
}

static void chat_swipe_catcher_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL)
        return;
    if (code == LV_EVENT_PRESSED)
    {
        lv_indev_get_point(indev, &s_hs_last);
        s_hs_dx = 0;
        s_hs_dy = 0;
        s_hs_last_vy = 0;
        s_hs_mode = CHAT_HS_MODE_IDLE;
        /* 手指一按住就停掉尚未走完的平移動畫,別跟拖曳打架。動畫被中斷 = 升格/
           退場的 ready cb 不會跑,把鄰居頁收掉、回到乾淨的 0 位。 */
        if (s_msg_list != NULL)
            lv_anim_del(s_msg_list, chat_hslide_exec);
        chat_next_page_destroy();
        chat_hslide_apply(0);
        return;
    }
    if (code == LV_EVENT_PRESSING)
    {
        lv_point_t now;
        lv_indev_get_point(indev, &now);
        int vx = now.x - s_hs_last.x;
        int vy = now.y - s_hs_last.y;
        s_hs_last = now;
        s_hs_dx += vx;
        s_hs_dy += vy;
        if (s_hs_mode == CHAT_HS_MODE_IDLE)
        {
            if (LV_ABS(s_hs_dy) > CHAT_HS_DECIDE_V && LV_ABS(s_hs_dy) >= LV_ABS(s_hs_dx))
                s_hs_mode = CHAT_HS_MODE_VSCROLL;
            else if (LV_ABS(s_hs_dx) > CHAT_HS_DECIDE_H &&
                     LV_ABS(s_hs_dx) > LV_ABS(s_hs_dy) * 2)
                s_hs_mode = CHAT_HS_MODE_HSWIPE;
        }
        if (s_hs_mode == CHAT_HS_MODE_VSCROLL && s_msg_list != NULL &&
            lv_obj_is_valid(s_msg_list))
        {
            /* 內容跟著手指走;夾在可捲範圍內(scroll_by 本身不夾)。 */
            lv_coord_t st = lv_obj_get_scroll_top(s_msg_list);
            lv_coord_t sb = lv_obj_get_scroll_bottom(s_msg_list);
            int dy = vy;
            if (dy > st)
                dy = st;
            if (dy < -sb)
                dy = -sb;
            if (dy != 0)
                lv_obj_scroll_by(s_msg_list, 0, dy, LV_ANIM_OFF);
            s_hs_last_vy = vy;
        }
        else if (s_hs_mode == CHAT_HS_MODE_HSWIPE && s_msg_list != NULL &&
                 lv_obj_is_valid(s_msg_list))
        {
            /* 鄰居頁跟拖曳方向走:一動就先建好(內容在滑動過程中就看得到),
               中途反向就換建另一側的。 */
            if (s_hs_dx != 0)
                chat_next_page_build((s_hs_dx < 0) ? 1 : -1);
            chat_hslide_apply((lv_coord_t)s_hs_dx);
        }
        return;
    }
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
    {
        int mode = s_hs_mode;
        s_hs_mode = CHAT_HS_MODE_IDLE;
        if (mode == CHAT_HS_MODE_HSWIPE)
        {
            int dir = (s_hs_dx < 0) ? 1 : -1;
            bool commit = (LV_ABS(s_hs_dx) >= CHAT_HS_COMMIT_PX) &&
                          s_next_dir == dir && s_next_list != NULL;
            if (commit)
            {
                chat_hslide_settle(s_hs_dx, (dir > 0) ? -LV_HOR_RES : LV_HOR_RES, 160,
                                   chat_hslide_commit_done);
            }
            else
            {
                /* 沒過門檻 / 這個方向沒有房了:彈回原位,鄰居頁退場。 */
                chat_hslide_settle(s_hs_dx, 0, 200, chat_hslide_cancel_done);
            }
        }
        else if (mode == CHAT_HS_MODE_VSCROLL && s_msg_list != NULL &&
                 lv_obj_is_valid(s_msg_list) && code == LV_EVENT_RELEASED)
        {
            /* 簡易慣性:最後一拍速度 × 8,交給 LVGL 的捲動動畫收尾(scroll_by 帶
               ANIM_ON 自帶 ease)。範圍一樣先夾。 */
            int throw_dy = s_hs_last_vy * 8;
            lv_coord_t st = lv_obj_get_scroll_top(s_msg_list);
            lv_coord_t sb = lv_obj_get_scroll_bottom(s_msg_list);
            if (throw_dy > st)
                throw_dy = st;
            if (throw_dy < -sb)
                throw_dy = -sb;
            if (throw_dy != 0)
                lv_obj_scroll_by(s_msg_list, 0, throw_dy, LV_ANIM_ON);
        }
        else if (mode == CHAT_HS_MODE_IDLE && code == LV_EVENT_RELEASED)
        {
            lv_point_t p;
            lv_indev_get_point(indev, &p);
            chat_catcher_forward_click(&p);
        }
        return;
    }
}

void chat_page_open(const char *title, const char *icon_src)
{
    if (chat_page_is_open())
        chat_page_close();
    chat_cache_ensure(); /* per-session 轉錄快取(LVGL thread 配置;BLE 端 NULL 就跳過) */

    lv_obj_t *panel = lv_obj_create(lv_layer_top());
    lv_obj_set_size(panel, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(panel, 0, 0);
    lv_obj_set_style_radius(panel, 0, 0);
    lv_obj_set_style_border_width(panel, 0, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_set_style_bg_color(panel, lv_color_black(), 0); /* fallback if the gaussian image won't load */
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    /* Gaussian-blur backdrop (the watch's glassy dial blur), darkened a touch so the white title + grey
       bubbles keep contrast — matches the desktop ConversationWindow's frosted panel (founder 2026-06-29). */
    {
        extern char *GAUS_DEFAULT_PICTURE;
        if (GAUS_DEFAULT_PICTURE != NULL)
        {
            lv_obj_set_style_bg_img_src(panel, GAUS_DEFAULT_PICTURE, 0);
            lv_obj_set_style_bg_img_recolor(panel, lv_color_black(), 0);
            lv_obj_set_style_bg_img_recolor_opa(panel, LV_OPA_40, 0);
        }
    }
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    /* Deliberately leave the panel NON-clickable: a clickable full-screen panel would CAPTURE the
       left-edge swipe and starve the watch's native edge-back detect object. Non-clickable, the
       swipe passes through to the native back (→ ESC → handle_back_event closes this chat first). */

    /* Header: a small service logo (if the tapped row carried one) + the contact / AI name, as one
       centered group — mirrors the desktop ConversationWindow's avatar + name (founder 2026-06-29). No
       back button: the native left-edge swipe-back closes the room. */
    lv_obj_t *header = lv_obj_create(panel);
    lv_obj_remove_style_all(header);
    lv_obj_set_size(header, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 18);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(header, 6, 0);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

    if (icon_src != NULL)
    {
        /* Show the ~80-100px service logo at ~44px before the name. The @-list dots prove ezip images
           DO zoom — but setting a size ON the img fights the transform (the bitmap shows at natural size
           clipped to the box → only the top-left quarter). Mirror app_exercise: zoom with a CENTRE pivot
           + OVERFLOW_VISIBLE and NO size on the img; reserve the flex footprint with a 44px wrapper box
           the natural-size img is centred in (founder 2026-06-29). */
        const lv_img_dsc_t *dsc = (const lv_img_dsc_t *)icon_src;
        lv_obj_t *iconbox = lv_obj_create(header);
        lv_obj_remove_style_all(iconbox);
        lv_obj_set_size(iconbox, 44, 44);
        lv_obj_clear_flag(iconbox, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(iconbox, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
        lv_obj_t *hicon = lv_img_create(iconbox);
        lv_img_set_src(hicon, icon_src);
        lv_img_set_pivot(hicon, dsc->header.w / 2, dsc->header.h / 2);
        lv_obj_add_flag(hicon, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
        lv_img_set_zoom(hicon, dsc->header.w > 0 ? (uint16_t)((256u * 44u) / dsc->header.w) : 256);
        lv_obj_center(hicon);
    }

    lv_obj_t *title_lbl = lv_label_create(header);
    lv_label_set_long_mode(title_lbl, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_font(title_lbl, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_set_style_text_color(title_lbl, lv_color_hex(0xFFFFFF), 0);
    {
        /* Content-sized so the icon hugs the name, but capped so a long name truncates (…) instead of
           shoving the icon off-screen — measure the natural width and only pin a width when it overflows. */
        const char *t = (title && title[0]) ? title : "聊天室";
        const lv_font_t *fnt = lv_obj_get_style_text_font(title_lbl, LV_PART_MAIN);
        lv_coord_t lsp = lv_obj_get_style_text_letter_space(title_lbl, LV_PART_MAIN);
        lv_coord_t lnsp = lv_obj_get_style_text_line_space(title_lbl, LV_PART_MAIN);
        lv_point_t tsz;
        lv_txt_get_size(&tsz, t, fnt, lsp, lnsp, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
        lv_coord_t cap = (icon_src != NULL) ? (LV_HOR_RES - 96) : (LV_HOR_RES - 56);
        if (tsz.x > cap)
            lv_obj_set_width(title_lbl, cap);
        lv_label_set_text(title_lbl, t);
    }
    s_title_label = title_lbl;

    /* Faint hairline under the header, separating it from the transcript (desktop ConversationWindow
       parity). Kept narrow so its ends stay inside the round display near the top. */
    lv_obj_t *hairline = lv_obj_create(panel);
    lv_obj_remove_style_all(hairline);
    lv_obj_set_size(hairline, LV_HOR_RES - 170, 1);
    lv_obj_align(hairline, LV_ALIGN_TOP_MID, 0, 63);
    lv_obj_set_style_bg_color(hairline, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(hairline, LV_OPA_30, 0);
    lv_obj_clear_flag(hairline, LV_OBJ_FLAG_SCROLLABLE);

    /* Scrollable message column. Extends almost to the BOTTOM of the screen (the mic is now a small
       floating glyph, not a solid bar) so the conversation isn't cut short by a black band (founder
       2026-06-29). Wide (13px side margin) so bubbles reach close to the screen edge; pad_bottom
       reserves the bottom band for the floating mic so the newest message scrolls up clear of it. The
       round display's dead corners only bite the top/bottom-most rows, which scroll through the
       readable middle band. */
    lv_obj_t *list = lv_obj_create(panel);
    lv_obj_set_size(list, LV_HOR_RES - 26, LV_VER_RES - 78);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 66); /* starts BELOW the ~44px header so its icon isn't covered */
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF); /* founder 2026-06-29: hide the right scrollbar */
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list, 6, 0);
    lv_obj_set_style_pad_hor(list, 2, 0);
    /* ~half-viewport top+bottom padding so the FIRST message can scroll to the centre (pull to the top)
       and the LAST message can too (pull to the bottom) — centre-focus scrolling (founder 2026-06-29).
       The big bottom pad also keeps the newest message clear of the floating mic. */
    lv_obj_set_style_pad_top(list, 180, 0);
    lv_obj_set_style_pad_bottom(list, 180, 0);
    s_msg_list = list;

    /* 左右滑切 session 的 catcher(見上方狀態機說明):蓋住訊息區、避開底部麥克風
       那條帶。平時 HIDDEN;只有 chat_page_set_session_switcher 註冊了 cb 的房
       (滑鼠抽屜 bot 房)才現身,錶盤 '@' 房維持原生捲動。建立順序刻意在 list 之後、
       麥克風/錄音 scrim 之前 —— 那些浮在 catcher 上面照常可點。 */
    lv_obj_t *catcher = lv_obj_create(panel);
    lv_obj_remove_style_all(catcher);
    lv_obj_set_size(catcher, LV_HOR_RES, LV_VER_RES - 66 - 84);
    lv_obj_set_pos(catcher, 0, 66);
    lv_obj_set_style_bg_opa(catcher, LV_OPA_TRANSP, 0);
    lv_obj_add_flag(catcher, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(catcher, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_clear_flag(catcher, LV_OBJ_FLAG_SCROLLABLE); /* 捲動死路 — 事件全留在這 */
    lv_obj_add_flag(catcher, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(catcher, chat_swipe_catcher_cb, LV_EVENT_ALL, NULL);
    s_swipe_catcher = catcher;

    /* "Loading" placeholder, CENTERED on the screen (not in the list, which would push it to the top-
       left). Shown until the first message arrives — the Matrix backlog can take many seconds for a
       cold room (founder 2026-06-29). Toggled by chat_page_apply_pending_state. */
    lv_obj_t *loading = lv_label_create(panel);
    lv_label_set_text(loading, "載入中…");
    lv_obj_set_style_text_font(loading, LV_EXT_FONT_GET(get_system_font_size(-1)), 0);
    lv_obj_set_style_text_color(loading, lv_color_hex(0x888888), 0);
    lv_obj_align(loading, LV_ALIGN_CENTER, 0, 0);
    s_loading_label = loading;

    /* Mic / voice-input — a CLICKABLE GLYPH (micro_icon), bottom-center, FLOATING over the transcript
       with NO button background so it doesn't block the chat behind it (founder 2026-06-29). Tap to
       record, tap again to send (chat_mic_btn_cb); recording tints it red. ext_click_area widens the
       tap target around the glyph. The left-edge swipe-back is unaffected (it's at x≈0). */
    /* Dark-blue glass backplate keeps the mic legible when a long transcript scrolls behind it. */
    lv_obj_t *mic_plate = lv_obj_create(panel);
    lv_obj_remove_style_all(mic_plate);
    lv_obj_set_size(mic_plate, 68, 68);
    lv_obj_align(mic_plate, LV_ALIGN_BOTTOM_MID, 0, -1);
    lv_obj_set_style_radius(mic_plate, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(mic_plate, lv_color_hex(0x071923), 0);
    lv_obj_set_style_bg_grad_color(mic_plate, lv_color_hex(0x174A61), 0);
    lv_obj_set_style_bg_grad_dir(mic_plate, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(mic_plate, 220, 0);
    lv_obj_set_style_border_width(mic_plate, 1, 0);
    lv_obj_set_style_border_color(mic_plate, lv_color_hex(0x8DDCF5), 0);
    lv_obj_set_style_border_opa(mic_plate, 90, 0);
    lv_obj_clear_flag(mic_plate, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    s_mic_backplate = mic_plate;

    lv_obj_t *mic = lv_img_create(panel);
    lv_img_set_src(mic, &micro_icon);
    /* Half-size the mic glyph (founder 2026-06-29): zoom 128 (50%) with a CENTRE pivot +
       OVERFLOW_VISIBLE so the ezip bitmap scales cleanly (a set_size would clip it). The object keeps
       its natural size for the tap target. */
    lv_img_set_pivot(mic, micro_icon.header.w / 2, micro_icon.header.h / 2);
    lv_obj_add_flag(mic, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_img_set_zoom(mic, 128);
    lv_obj_align(mic, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_add_flag(mic, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(mic, 20);
    lv_obj_add_event_cb(mic, chat_mic_btn_cb, LV_EVENT_CLICKED, NULL);
    s_mic_btn = mic;
    s_mic_img = mic;
    s_recording = false;

    /* Tap-outside-to-cancel catcher: a transparent, clickable, full-screen layer created BETWEEN the
       messages and the input box/send glyph (which are created after it, so they sit above). Shown only
       while recording; a tap anywhere it covers (i.e. outside the box + send) cancels voice-input mode
       (founder 2026-06-29). Not scrollable, so a drag does nothing — only a discrete tap cancels. The
       native left-edge back detector is raised above this overlay, so swipe-back still closes the room. */
    lv_obj_t *scrim = lv_obj_create(panel);
    lv_obj_remove_style_all(scrim);
    lv_obj_set_size(scrim, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(scrim, 0, 0);
    lv_obj_set_style_bg_opa(scrim, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(scrim, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(scrim, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(scrim, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(scrim, chat_scrim_cb, LV_EVENT_CLICKED, NULL);
    s_input_scrim = scrim;

    /* Live V2T transcript INPUT BOX — the watch-face skaibar voice box, pixel-for-pixel: the
       message_widget_bg PILL FRAME image sized 442×252 and sitting 75px below the bottom (so its top
       portion shows as a pill rising from the edge — exactly the skaibar's LBOX_W/H/Y), with the
       transcript on its visible top. Hidden until the mic starts; replaces the mic while recording. */
    lv_obj_t *tbox = lv_obj_create(panel);
    lv_obj_remove_style_all(tbox);
    lv_obj_set_size(tbox, 442, 252);
    lv_obj_align(tbox, LV_ALIGN_BOTTOM_MID, 0, 75);
    /* Opaque fill behind the pill frame so scrolled-up chat bubbles do not show through it. */
    lv_obj_set_style_bg_color(tbox, lv_color_hex(0x1C1C1E), 0);
    lv_obj_set_style_bg_opa(tbox, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(tbox, 80, 0);
    lv_obj_clear_flag(tbox, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(tbox, LV_OBJ_FLAG_HIDDEN);
    /* Clickable (no handler) so a tap ON the box is ABSORBED here instead of falling through to the
       cancel-scrim below it — tapping inside the input box must NOT exit input mode. */
    lv_obj_add_flag(tbox, LV_OBJ_FLAG_CLICKABLE);
    s_transcript_box = tbox;
    lv_obj_t *pill = lv_img_create(tbox);
    lv_img_set_src(pill, &message_widget_bg);
    lv_obj_center(pill);
    lv_obj_clear_flag(pill, LV_OBJ_FLAG_CLICKABLE);
    s_transcript_pill = pill;
    lv_obj_t *tlbl = lv_label_create(tbox);
    lv_label_set_long_mode(tlbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(tlbl, 360);
    lv_obj_set_style_text_font(tlbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(tlbl, lv_color_white(), 0);
    lv_obj_set_style_text_opa(tlbl, LV_OPA_80, 0);
    lv_obj_set_style_text_align(tlbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(tlbl, "");
    lv_obj_align(tlbl, LV_ALIGN_TOP_MID, 0, 26);
    s_transcript_label = tlbl;

    /* Send glyph — shown in the mic's place (bottom-centre) while recording; tap to stop + send. */
    lv_obj_t *send = lv_img_create(panel);
    lv_img_set_src(send, &icon_send);
    lv_obj_align(send, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_add_flag(send, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(send, 20);
    lv_obj_add_event_cb(send, chat_mic_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(send, LV_OBJ_FLAG_HIDDEN);
    s_send_btn = send;

    s_chat_panel = panel;

    /* SWIPE-BACK: the watch's native left-edge back detector + its drag hint are created at boot as
       EARLY children of lv_layer_top (watch_demo.c lvsf_gesture_init(lv_layer_top())). This chat
       panel is a LATER lv_layer_top child, so it sits ABOVE them — the edge swipe never reached the
       detector (no ESC) and the hint (the arrow that follows the finger) was hidden behind the panel.
       Enable the detector + raise BOTH it and the hint above this overlay so the swipe is caught
       (→ ESC → handle_back_event closes this room) AND the user sees the hint track their finger. */
    {
        extern void display_gesture_detect_objs(uint32_t idx, bool display);
        extern void lvsf_gesture_bring_to_front(void);
        display_gesture_detect_objs(0, true);
        /* R76(founder:「聊天室底部的麥克風點了沒反應」):bring_to_front 抬的是**四條**
           透明邊緣 bar,idx3 是整寬的 BOTTOM bar —— 跟底部麥克風完全重疊。若上一個 app
           離開時手勢處於啟用狀態(gesture_enable_update(true) 四條全顯示),抬到面板上方
           的底部 bar 會把 mic 的 tap 整顆吃掉(透明的,看起來就是沒反應);時好時壞取決
           於進房前的 app 路徑。聊天室只需要左緣返回,其餘三條明確藏掉。 */
        display_gesture_detect_objs(1, false);
        display_gesture_detect_objs(2, false);
        display_gesture_detect_objs(3, false);
        lvsf_gesture_bring_to_front();
    }

    /* heap 輪:hosted 滑鼠模式下,聊天 overlay 不透明蓋全螢幕 —— 底下的滑鼠圖層
       藏掉,別讓看不見的整層白付 EPIC 合成(低 heap 時就是 render OOM 的差額)。
       chat_page_close 對稱還原;非滑鼠模式 no-op。 */
    {
        extern void lv_top_panel_mouse_layer_set_covered(bool covered);
        lv_top_panel_mouse_layer_set_covered(true);
    }

    s_live_turn = false;     /* 剛進房:接下來畫的是歷史,不逐字 */
    chat_type_timer_start(); /* 常駐揭露 timer:整個房間開著期間都在跑,不靠任何時序啟動 */
    LOG_I("chat page opened: %s", (title && title[0]) ? title : "(none)");
}

void chat_page_close(void)
{
    if (!chat_page_is_open())
        return;
    if (s_recording)
    {
        s_recording = false;
        voice_provider.auto_stop_listening();
        clearVoice2Text();
    }
    /* Kill any in-flight morph anims before the objects are freed (their ready cbs touch these). */
    lv_anim_del(s_transcript_box, chat_box_grow_cb);
    lv_anim_del(s_transcript_pill, chat_pill_fade_cb);
    lv_anim_del(s_msg_list, chat_hslide_exec); /* 換房平移動畫的 ready cb 會碰 list */
    lv_obj_del(s_chat_panel);
    s_chat_panel = NULL;
    s_title_label = NULL;
    s_msg_list = NULL;
    s_loading_label = NULL;
    s_mic_btn = NULL;
    s_mic_img = NULL;
    s_mic_backplate = NULL;
    s_send_btn = NULL;
    s_transcript_box = NULL;
    s_transcript_label = NULL;
    s_transcript_pill = NULL;
    s_input_scrim = NULL;
    s_swipe_catcher = NULL;      /* 隨面板一起被 lv_obj_del 帶走 */
    s_next_list = NULL;          /* 同上(panel 子物件) */
    s_next_title = NULL;
    s_next_dir = 0;
    s_session_switch_cb = NULL;  /* 重開時由開房的人重新註冊 */
    s_hs_mode = CHAT_HS_MODE_IDLE;
    chat_type_reset();
    if (s_rebuild_defer != NULL)
    {
        lv_timer_del(s_rebuild_defer);
        s_rebuild_defer = NULL;
    }
    s_last_rebuild_tick = 0;
    chat_wait_stop();
    s_local_echo[0] = 0;
    s_awaiting_reply = false;
    {
        extern void lv_top_panel_mouse_layer_set_covered(bool covered);
        lv_top_panel_mouse_layer_set_covered(false);
    }
    LOG_I("chat page closed");
}

/* DOWNLINK (KEY_CONV_STATE, BLE parse thread): parse {title, sending, messages:[{role,text}]} into
   the pending STATIC buffer (bounded copies — heap-only cJSON, no 4KB BLE-stack blowup), then DEFER
   the render to the LVGL thread (LVGL ops are not thread-safe off the GUI task). Defined here so
   resolve_skailink_command links against a STRONG symbol regardless. */
void skai_chat_on_conv_state(const uint8_t *json, uint16_t length)
{
    LOG_D("[chat] conv_state rx len=%u open=%d", (unsigned)length, (int)chat_page_is_open());
    if (json == NULL || length == 0)
        return;
    cJSON *root = cJSON_ParseWithLength((const char *)json, length);
    if (!cJSON_IsObject(root))
    {
        /* 解析失敗就整包靜默丟棄 —— 而丟棄等於「畫面停在上一版」,正是「我的輸入閃一下
           就消失」會有的表現。payload 隨對話成長(實測 93→388→680→970 bytes),被截斷的
           那一刻就會落到這裡,所以要印出長度與尾端幾個 byte 才分得出「截斷」與「內容
           真的壞了」(founder 2026-08-17)。 */
        LOG_W("[chat] conv_state PARSE FAILED len=%u tail=\"%.16s\" — whole update dropped",
              (unsigned)length,
              length >= 16 ? (const char *)(json + length - 16) : (const char *)json);
        cJSON_Delete(root);
        return;
    }

    cJSON *j_title = cJSON_GetObjectItem(root, "title");
    if (cJSON_IsString(j_title))
    {
        strncpy(s_pending_title, j_title->valuestring, sizeof(s_pending_title) - 1);
        s_pending_title[sizeof(s_pending_title) - 1] = '\0';
    }
    else
    {
        s_pending_title[0] = '\0';
    }

    /* sid = 這份狀態屬於哪個 session(手機 open 時綁的 conv:hermes:<id>)——
       快取拓寫的 key。沒帶(舊 APK)就不拓,行為不變。 */
    {
        cJSON *j_sid = cJSON_GetObjectItem(root, "sid");
        if (cJSON_IsString(j_sid))
        {
            strncpy(s_state_sid, j_sid->valuestring, sizeof(s_state_sid) - 1);
            s_state_sid[sizeof(s_state_sid) - 1] = '\0';
        }
        else
        {
            s_state_sid[0] = '\0';
        }
    }

    s_pending_sending = cJSON_IsTrue(cJSON_GetObjectItem(root, "sending"));

    int count = 0;
    cJSON *j_msgs = cJSON_GetObjectItem(root, "messages");
    if (cJSON_IsArray(j_msgs))
    {
        /* 兩趟。第一趟只收「有效訊息」的指標(最多 CHAT_MAX_MSGS 則,滿了就頂掉最舊的),
           第二趟從**最新往回**累加長度,決定文字池放得下哪一段 —— 這個順序保證剛送出/
           剛回覆的那則一定留得住,被擠掉的永遠是最舊的歷史(founder 2026-06-29 踩過反過來
           的版本:backlog 塞滿時新訊息反而不見)。 */
        cJSON *items[CHAT_MAX_MSGS];
        int n = 0;
        cJSON *m = NULL;
        cJSON_ArrayForEach(m, j_msgs)
        {
            cJSON *jt = cJSON_GetObjectItem(m, "text");
            if (!cJSON_IsString(jt) || jt->valuestring[0] == '\0')
                continue;
            if (n < CHAT_MAX_MSGS)
            {
                items[n++] = m;
            }
            else
            {
                for (int i = 1; i < CHAT_MAX_MSGS; i++)
                    items[i - 1] = items[i];
                items[CHAT_MAX_MSGS - 1] = m;
            }
        }

        /* ── 純追加的快路徑 ──
           條件全部成立才算:則數沒變、沒有待答 clarify、最後一則是 AI、而且它的前
           s_live_len 個 byte 跟我們已經收下的**完全一樣**(雜湊比對)。成立就只把多出來
           的尾巴推進環,文字池 / count / approval 一律不動,LVGL 那邊也不重建。
           串流時這條會吃掉幾乎所有更新 —— 每包只多 1~2 個字。 */
        int first = n;
        size_t need = 0;
        for (int i = n - 1; i >= 0; i--)
        {
            size_t l = strlen(cJSON_GetObjectItem(items[i], "text")->valuestring);
            if (l > (size_t)CHAT_MSG_TEXT_MAX)
                l = (size_t)CHAT_MSG_TEXT_MAX;
            if (need + l + 1 > (size_t)CHAT_TEXT_POOL)
                break;
            need += l + 1;
            first = i;
        }
        if (first >= n && n > 0)
            first = n - 1; /* 連最新一則都超過整個池 —— 還是收它,由 chat_pool_put 切 */

        s_pool_used = 0;
        for (int i = first; i < n; i++)
        {
            cJSON *j_text = cJSON_GetObjectItem(items[i], "text");
            cJSON *j_role = cJSON_GetObjectItem(items[i], "role");
            const char *role = cJSON_IsString(j_role) ? j_role->valuestring : "";
            strncpy(s_pending_msgs[count].role, role, sizeof(s_pending_msgs[count].role) - 1);
            s_pending_msgs[count].role[sizeof(s_pending_msgs[count].role) - 1] = '\0';
            /* 截斷要**出聲** —— 靜默截斷正是「電腦上完整、手錶只有前半」這種看起來像
               UI bug、其實是容量不夠的症狀(founder 2026-08-26)。 */
            int src_len = (int)strlen(j_text->valuestring);
            chat_pool_put(&s_pending_msgs[count], j_text->valuestring);
            if (s_pending_msgs[count].len < (uint16_t)src_len)
                LOG_W("[chat] msg TRUNCATED src=%d kept=%d",
                      src_len, (int)s_pending_msgs[count].len);
            count++;
        }
        if (first > 0)
            LOG_W("[chat] pool: dropped %d oldest, used=%d", first, (int)s_pool_used);

    }
    /* 解析出幾則、丟了幾則。手機端的 msgs 是單調成長的(實測 3→4→5→6,從不縮),所以只要
       這裡的 count 比它小,差額就是在這一層掉的 —— 分辨「手機沒送」與「手錶沒收下」。 */
    LOG_D("[chat] parsed=%d sending=%d", count, (int)s_pending_sending);

    /* Pending clarify/approval — parse BEFORE publishing msg_count so one REFRESH_CHAT renders
       both. Absent object ⇒ nothing pending (an answered/expired prompt clears this way too). */
    s_appr_pending = false;
    s_appr_count = 0;
    cJSON *j_appr = cJSON_GetObjectItem(root, "approval");
    if (cJSON_IsObject(j_appr))
    {
        cJSON *j_rid = cJSON_GetObjectItem(j_appr, "rid");
        cJSON *j_q = cJSON_GetObjectItem(j_appr, "q");
        if (cJSON_IsString(j_rid) && j_rid->valuestring[0] != '\0')
        {
            chat_copy_utf8(s_appr_rid, sizeof(s_appr_rid), j_rid->valuestring);
            chat_copy_utf8(s_appr_q, sizeof(s_appr_q),
                           cJSON_IsString(j_q) ? j_q->valuestring : "");
            cJSON *j_opts = cJSON_GetObjectItem(j_appr, "opts");
            if (cJSON_IsArray(j_opts))
            {
                cJSON *o = NULL;
                cJSON_ArrayForEach(o, j_opts)
                {
                    if (s_appr_count >= CHAT_APPR_OPTS)
                        break;
                    cJSON *j_id = cJSON_GetObjectItem(o, "id");
                    cJSON *j_label = cJSON_GetObjectItem(o, "label");
                    const char *id = cJSON_IsString(j_id) ? j_id->valuestring : "";
                    const char *label =
                        (cJSON_IsString(j_label) && j_label->valuestring[0] != '\0')
                            ? j_label->valuestring : id;
                    chat_copy_utf8(s_appr_opts[s_appr_count].id,
                                   sizeof(s_appr_opts[s_appr_count].id), id);
                    chat_copy_utf8(s_appr_opts[s_appr_count].label,
                                   sizeof(s_appr_opts[s_appr_count].label), label);
                    s_appr_count++;
                }
            }
            s_appr_pending = true; /* publish LAST (same ordering rule as msg_count) */
        }
    }
    cJSON_Delete(root);

    s_local_echo[0] = '\0'; /* 手機的版本到了,本機回音讓位 */
    s_pending_msg_count = count; /* publish LAST so an LVGL reader never sees a half-filled buffer */
    /* 拓一份進 per-session 快取(同一條 BLE 執行緒,pending 剛寫完就 memcpy,不跨執行緒)。 */
    chat_cache_store(s_state_sid);
    LOG_I("conv_state rx: title=%s msgs=%d sending=%d", s_pending_title, count, (int)s_pending_sending);
    /* 這條是 W 級:本機組態 ULOG_OUTPUT_LVL_W,D/I 在 COM12 上一個字都看不到,
       而「手錶收到多少 bytes / 解析出幾則」是所有聊天室問題的第一個分岔口。 */
    LOG_W("[chat] rx len=%u parsed=%d", (unsigned)length, count);

    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_CHAT};
    lvgl_send_msg(msg);
}

/* A clarify/approval chip was tapped: answer on the 0x10 uplink as
   `\x01decision:<rid>\x1f<optionId>` (BleWatchConnection routes it to convDecision), then drop the
   chips locally — the desktop's `decision` convEvent (or a re-ask) is the authoritative refresh. */
static void chat_appr_chip_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_appr_pending)
        return;
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= s_appr_count)
        return;
    /* "\x01" kept as its OWN literal: "\x01dec…" would let the hex escape swallow the d/e/c. */
    static char out[16 + sizeof(s_appr_rid) + sizeof(s_appr_opts[0].id)];
    rt_snprintf(out, sizeof(out), "\x01" "decision:%s\x1f%s", s_appr_rid, s_appr_opts[idx].id);
    commu_send_conv_send(out);
    LOG_I("chat clarify: answered opt=%d rid=%s", idx, s_appr_rid);
    s_appr_pending = false;
    chat_page_apply_pending_state(); /* re-render without the chips */
}

/* ── Turn treatment(desktop ConversationPane parity, founder 2026-08-15)──
   Hermes(AI session)房間照桌面的 Hermes desktop 畫法:**不是左右氣泡** ——
     · 使用者的訊息 = 全寬玻璃卡片(SkGlassBgSoft 白6% 填色 + 1px SkGlassEdge 白8%
       hairline、圓角 18、pad 12×8)
     · AI 回覆 = 無框無底的平鋪全寬文字(長文是閱讀,不上聊天裝)
   @聯絡人房間(WhatsApp/Messenger…)保留 iMessage 氣泡 —— 桌面的 `.messaging`
   同款分流。以開房那一刻的 id 判別(conv: 前綴 = Hermes session)。 */
static bool s_hermes_style = true;
void chat_page_set_style_hermes(bool hermes)
{
    s_hermes_style = hermes;
}

/* ── AI 回覆逐字浮現(founder 2026-08-26)──
   桌面的 AI 回覆是串流打字,手錶這邊 conv_state 是整包重畫,所以每次刷新整段字
   瞬間跳出。這裡在「最後一則 AI 訊息」上補一層逐字揭露:記住上次已顯示到哪(s_type_shown),
   只把新增的那一段一個字一個字補上 —— 內容成長時接著打,不會每收一包就從頭重來。
   只用在 Hermes(AI)房;@聯絡人房的來訊是真人講的話,不該假裝在打字。 */
/* ── 重繪成本才是真正的天花板(founder 2026-08-26「回覆完後手錶直接當機」)──
   每一次 lv_label_ins_text 都會讓**整個** label 重新斷行 + 重新取字形。回覆長到
   三四百個中文字時,單次重繪本身就很貴;一拍 40ms 追加一次 = 一秒 25 次全量重繪,
   GUI 執行緒被自己餵飽,畫面就此不動(BLE 還活著、也沒有重開 → 不是 crash,是 GUI
   餓死)。所以節流的對象不是「一次接幾個字」,是「一秒重繪幾次」:
     · 一拍拉長到 100ms → 最多一秒 10 次
     · 一拍把環**整個排空**成一次 ins_text,而不是分很多次
     · 捲動只在排空後做一次(它會逼一次 layout)
     · label 長到一定程度就完全停掉動畫,直接整段補上 —— 長文的閱讀價值遠高於打字感 */
#define CHAT_TYPE_PERIOD_MS 100
#define CHAT_TYPE_NOANIM_LEN 900 /* 超過這個長度就不再逐字,直接整段接上 */
#define CHAT_TYPE_PIECE_MAX 256 /* 一拍最多搬進來的 byte 數 */
#define CHAT_TYPE_STEP 2       /* 一拍接 2 個字:每拍都要重排版+FT 取字形,一次一個字
                                  在長回覆上是白白多花一倍的重繪 */

/* 直播那一則的 label。字就存在它裡面 —— 我們這邊**沒有**第二份全文緩衝。 */
/* 接字的工作緩衝。**静態不放 stack**:lvgl_task 的 stack 只有 14*256=3584 bytes,
   而 ins_text 還要往下跑 realloc + 重新斷行 + 畫圖。在這條路上放幾百 bytes 的
   局部陣列是在踩線(參考 tpread stack 溢位那次)。只有 LVGL 執行緒碰它。 */
static char s_type_piece[CHAT_TYPE_PIECE_MAX + 1];
static lv_obj_t *s_live_lbl = NULL;
/* 揭露游標。s_reveal_src 指進文字池 —— BLE 重寫池子時內容會變,但池尾有永久 NUL
   哨兵、off 也夾過,讀取永遠在界內;而任何重寫都會帶來一次重建,那時重新對帳。 */
static const char *s_reveal_src;
static int s_reveal_total;
static int s_reveal_pos;
static uint32_t s_paint_hash;
static lv_timer_t *s_type_timer = NULL;

static void chat_type_stop(void)
{
    if (s_type_timer != NULL)
    {
        lv_timer_del(s_type_timer);
        s_type_timer = NULL;
    }
}

/* 關房重置:下次開房是另一個對話,環裡的尾巴跟前綴雜湊都不屬於它。 */
static void chat_type_reset(void)
{
    chat_type_stop();
    s_live_lbl = NULL;
    s_reveal_src = NULL;
    s_reveal_total = 0;
    s_reveal_pos = 0;
    s_paint_hash = 0;
}

/* 把 t 的前 upto 個 byte 分段餵進 label(切點對齊 UTF-8 字元邊界)。 */
static void chat_reveal_paint_upto(lv_obj_t *lbl, const char *t, int upto)
{
    lv_label_set_text(lbl, "");
    int off = 0;
    while (off < upto)
    {
        int n = upto - off;
        if (n > CHAT_TYPE_PIECE_MAX)
        {
            n = CHAT_TYPE_PIECE_MAX;
            while (n > 0 && ((uint8_t)t[off + n] & 0xC0) == 0x80)
                n--; /* 別切在字元中間 */
        }
        if (n <= 0)
            break;
        memcpy(s_type_piece, t + off, (size_t)n);
        s_type_piece[n] = '\0';
        lv_label_ins_text(lbl, LV_LABEL_POS_LAST, s_type_piece);
        off += n;
    }
}

static void chat_type_tick(lv_timer_t *t)
{
    LV_UNUSED(t);
#if CHAT_DEBUG_HUD
    s_dbg_tick++;
#endif
    /* timer 是常駐的:沒 label、沒東西可揭露,這一拍就什麼都不做。 */
    if (s_live_lbl == NULL || !lv_obj_is_valid(s_live_lbl))
    {
        s_live_lbl = NULL;
        return;
    }
    if (s_reveal_src == NULL || s_reveal_pos >= s_reveal_total)
        return;

    /* 還短就慢慢來(打字感),長了就大口吃 —— 每拍都要整段重排版+取字形,長回覆上
       一次多接一點才不會把 GUI 餵飽(那是先前看門狗咬下去的原因)。 */
    int take = (s_reveal_pos < CHAT_TYPE_NOANIM_LEN) ? (CHAT_TYPE_STEP * 4)
                                                     : CHAT_TYPE_PIECE_MAX;
    int left = s_reveal_total - s_reveal_pos;
    if (take > left)
        take = left;
    if (take > CHAT_TYPE_PIECE_MAX)
        take = CHAT_TYPE_PIECE_MAX;
    /* 切在 UTF-8 字元邊界上,不然中文會閃出半個字的 tofu。 */
    while (take > 0 && ((uint8_t)s_reveal_src[s_reveal_pos + take] & 0xC0) == 0x80)
        take--;
    if (take <= 0)
        return;

    memcpy(s_type_piece, s_reveal_src + s_reveal_pos, (size_t)take);
    s_type_piece[take] = '\0';
    lv_label_ins_text(s_live_lbl, LV_LABEL_POS_LAST, s_type_piece);
#if CHAT_DEBUG_HUD
    s_dbg_ins++;
#endif
    s_reveal_pos += take;
    s_paint_hash = chat_fnv1a(s_reveal_src, (size_t)s_reveal_pos);
    /* 捲動會逼一次 layout —— 一拍只做一次。 */
    if (s_msg_list != NULL && lv_obj_is_valid(s_msg_list))
        lv_obj_scroll_to_y(s_msg_list, LV_COORD_MAX, LV_ANIM_OFF);
}

/* 開房時建立、關房時刪除,中間**永遠不停**。 */
static void chat_type_timer_start(void)
{
    if (s_type_timer == NULL)
        s_type_timer = lv_timer_create(chat_type_tick, CHAT_TYPE_PERIOD_MS, NULL);
}

/* Hermes 卡片/平鋪的共用塗裝。mine=true → 玻璃卡;false → 平鋪文字。 */
/* 回傳裡面的 label(逐字揭露要拿它) —— card 可從 lv_obj_get_parent() 取得。 */
/* The display ultimately quantizes the glass gradient to RGB565, so LVGL's one-pixel ordered
   dithering is nearly invisible on the panel.  Add a deliberately sparse, very-low-opacity
   dashed weave between the card background and its label.  The staggered rows break up broad
   horizontal colour bands without reading as stripes or reducing text contrast. */
static void chat_user_card_texture_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN_END)
        return;

    lv_obj_t *card = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);
    lv_area_t a;
    lv_obj_get_coords(card, &a);

    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_hex(0x8DDCF5);
    dsc.width = 1;
    dsc.opa = 28;
    dsc.dash_width = 2;
    dsc.dash_gap = 5;

    uint16_t row = 0;
    for (lv_coord_t y = a.y1 + 7; y <= a.y2 - 7; y += 4, ++row)
    {
        lv_point_t p1 = {a.x1 + 8 + (row & 1 ? 3 : 0), y};
        lv_point_t p2 = {a.x2 - 8, y};
        lv_draw_line(draw_ctx, &dsc, &p1, &p2);
    }
}

static lv_obj_t *chat_add_hermes_turn(lv_obj_t *parent, const char *text, bool mine)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_width(card, lv_pct(100));
    lv_obj_set_height(card, LV_SIZE_CONTENT);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    /* 卡片不攔按壓:act_obj 落到 s_msg_list,左右滑切 session 的手勢才收得到
       (捲動不受影響 —— list 仍是捲動的主人)。 */
    lv_obj_clear_flag(card, LV_OBJ_FLAG_CLICKABLE);
    if (mine)
    {
        /* Match the floating mic backplate exactly: deep-blue glass with a cool edge highlight. */
        lv_obj_set_style_bg_color(card, lv_color_hex(0x071923), 0);
        lv_obj_set_style_bg_grad_color(card, lv_color_hex(0x174A61), 0);
        lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_dither_mode(card, LV_DITHER_ORDERED, 0);
        lv_obj_set_style_bg_opa(card, 220, 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_border_color(card, lv_color_hex(0x8DDCF5), 0);
        lv_obj_set_style_border_opa(card, 90, 0);
        lv_obj_set_style_radius(card, 18, 0);
        lv_obj_set_style_pad_hor(card, 12, 0);
        lv_obj_set_style_pad_ver(card, 8, 0);
        lv_obj_add_event_cb(card, chat_user_card_texture_cb, LV_EVENT_DRAW_MAIN_END, NULL);
    }
    else
    {
        lv_obj_set_style_bg_opa(card, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(card, 0, 0);
        lv_obj_set_style_pad_hor(card, 2, 0);
        lv_obj_set_style_pad_ver(card, 2, 0);
    }
    lv_obj_t *lbl = lv_label_create(card);
    lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl, lv_pct(100));
    lv_obj_set_style_text_font(lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_align(lbl, mine ? LV_TEXT_ALIGN_RIGHT : LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
    return lbl;
}

/* ── 等待 AI 回覆的提示(founder 2026-08-26)──
   送出之後到第一個字浮出來之間,畫面上完全沒有東西在動,分不出「在想」還是「斷線了」。
   這裡在訊息串尾端掛一則 AI 側的平鋪文字,點點循環跑;第一個字一到就被正常重畫取代。
   觸發條件是「手機說 sending」**或**「本機剛送出還沒等到回覆」—— 後者是手錶自己最
   確定的事實,不必等手機的狀態翻真。 */
#define CHAT_WAIT_PERIOD_MS 400

static lv_obj_t *s_wait_lbl = NULL;
static lv_timer_t *s_wait_timer = NULL;
static int s_wait_phase;

static void chat_wait_stop(void)
{
    if (s_wait_timer != NULL)
    {
        lv_timer_del(s_wait_timer);
        s_wait_timer = NULL;
    }
    s_wait_lbl = NULL;
}

static void chat_wait_tick(lv_timer_t *t)
{
    LV_UNUSED(t);
    if (s_wait_lbl == NULL || !lv_obj_is_valid(s_wait_lbl))
    {
        chat_wait_stop();
        return;
    }
    static const char *const frames[4] =
    {
        "思考中",
        "思考中 ·",
        "思考中 · ·",
        "思考中 · · ·",
    };
    s_wait_phase = (s_wait_phase + 1) & 3;
    lv_label_set_text(s_wait_lbl, frames[s_wait_phase]);
}

static void chat_render_waiting(void)
{
    if (s_msg_list == NULL || !lv_obj_is_valid(s_msg_list))
        return;
    lv_obj_t *lbl = chat_add_hermes_turn(s_msg_list, "思考中", false);
    /* 比真正的回覆淡:它是狀態不是內容,不該跟 AI 說的話一樣重。 */
    lv_obj_set_style_text_opa(lbl, LV_OPA_60, 0);
    s_wait_phase = 0;
    s_wait_lbl = lbl;
    s_wait_timer = lv_timer_create(chat_wait_tick, CHAT_WAIT_PERIOD_MS, NULL);
}

/* Render the pending clarify/approval under the transcript: the question as a THEIRS bubble, then
   one full-width tappable chip per option (bordered pill, desktop approval-buttons parity). */
static void chat_render_pending_approval(void)
{
    if (!s_appr_pending || s_msg_list == NULL || !lv_obj_is_valid(s_msg_list))
        return;

    if (s_appr_q[0] != '\0')
    {
        /* Clarify 只發生在 Hermes 房:問題照桌面畫法 = 平鋪全寬文字(assistant turn),
           不再上灰色氣泡。 */
        /* 這題反問才是房裡最新的 AI 發言 —— 逐字揭露的對象是它(chips 照常直接出現)。 */
        chat_add_hermes_turn(s_msg_list, s_appr_q, false);
    }

    for (int i = 0; i < s_appr_count; i++)
    {
        lv_obj_t *chip = lv_obj_create(s_msg_list);
        lv_obj_set_width(chip, lv_pct(96));
        lv_obj_set_height(chip, LV_SIZE_CONTENT);
        lv_obj_set_style_pad_hor(chip, 14, 0);
        lv_obj_set_style_pad_ver(chip, 9, 0);
        lv_obj_set_style_radius(chip, 21, 0);
        /* Outlined, not filled: chips must read as ACTIONS, distinct from message bubbles —
           Skaiwalk sky accent border on a translucent fill. */
        lv_obj_set_style_border_width(chip, 1, 0);
        lv_obj_set_style_border_color(chip, lv_color_hex(0x5C9CB8), 0);
        lv_obj_set_style_bg_color(chip, lv_color_hex(0x5C9CB8), 0);
        lv_obj_set_style_bg_opa(chip, LV_OPA_20, 0);
        lv_obj_clear_flag(chip, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(chip, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(chip, chat_appr_chip_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        lv_obj_t *lbl = lv_label_create(chip);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(lbl, lv_pct(100));
        lv_obj_set_style_text_font(lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(lbl, s_appr_opts[i].label[0] != '\0'
                          ? s_appr_opts[i].label : "讓 Agent 自行決定");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
    }
}

/* LVGL thread: rebuild the transcript bubbles from the pending buffer (left=them grey / right=me
   blue). No-op if the chat closed between the BLE parse and this deferred render. */
void chat_page_apply_pending_state(void)
{
    if (!chat_page_is_open() || s_msg_list == NULL || !lv_obj_is_valid(s_msg_list))
    {
        /* 早退等於「解析好的內容從來沒被畫出來」,而畫面就停在上一版 —— 這是「我的輸入閃
           一下就消失」的候選之一,原本靜默。 */
        LOG_W("[chat] render SKIPPED open=%d list=%d",
              (int)chat_page_is_open(), (int)(s_msg_list != NULL && lv_obj_is_valid(s_msg_list)));
        return;
    }
    LOG_D("[chat] render count=%d appr=%d", (int)s_pending_msg_count, (int)s_appr_pending);

    if (s_title_label != NULL && lv_obj_is_valid(s_title_label) && s_pending_title[0] != '\0')
        lv_label_set_text(s_title_label, s_pending_title);
#if CHAT_DEBUG_HUD
    if (s_title_label != NULL && lv_obj_is_valid(s_title_label))
    {
        int clen = 0;
        if (s_live_lbl != NULL && lv_obj_is_valid(s_live_lbl))
        {
            const char *ct = lv_label_get_text(s_live_lbl);
            clen = (ct != NULL) ? (int)strlen(ct) : 0;
        }
        lv_label_set_text_fmt(s_title_label, "P%d/%d V%d K%d C%d N%d T%u",
                              s_reveal_pos, s_reveal_total,
                              (int)s_live_turn, (int)s_dbg_keep, clen,
                              (int)s_pending_msg_count, (unsigned)s_dbg_tick);
    }
#endif

    /* 重建太密就延後 —— 這是 GUI 不被上游洗頻打死的閘門(見上方說明)。 */
    if (s_last_rebuild_tick != 0 && lv_tick_elaps(s_last_rebuild_tick) < CHAT_REBUILD_MIN_MS)
    {
        if (s_rebuild_defer == NULL)
            s_rebuild_defer = lv_timer_create(chat_rebuild_defer_cb, CHAT_REBUILD_MIN_MS, NULL);
        return;
    }
#if CHAT_DEBUG_HUD
    s_dbg_full++;
#endif
    s_last_rebuild_tick = lv_tick_get();
    if (s_rebuild_defer != NULL)
    {
        lv_timer_del(s_rebuild_defer);
        s_rebuild_defer = NULL;
    }

    s_live_lbl = NULL;        /* 下面重建時會重新指到直播那一則 */
    chat_wait_stop();         /* 等待點點同理;要不要重新掛下面重算 */
    lv_obj_clean(s_msg_list); /* drop the old bubbles (and the M1 "連線中…" hint) */

    int count = s_pending_msg_count;
    if (count > CHAT_MAX_MSGS)
        count = CHAT_MAX_MSGS;

    /* Toggle the centered "載入中…" placeholder: shown while the transcript is still empty (the Matrix
       backlog can take many seconds for a cold room), hidden once any message has arrived — or once a
       clarify question is pending (the question IS the content then). */
    /* 還在等 AI 嗎?最後一則是我講的(或這房還空)才算在等 —— AI 一開口
       (或丟出 clarify)就不是了，點點要讓位給真正的內容。 */
    bool echo = (s_local_echo[0] != '\0');
    bool last_is_mine = echo; /* 本機回音一定是我講的 */
    if (!echo && count > 0)
    {
        const char *r = s_pending_msgs[count - 1].role;
        last_is_mine = (strcmp(r, "user") == 0 || strcmp(r, "outgoing") == 0);
    }
    if ((count > 0 && !last_is_mine) || s_appr_pending)
        s_awaiting_reply = false;
    if (s_pending_sending || s_awaiting_reply)
        s_live_turn = true; /* 有一輪正在進行 → 接下來長出來的字要逐字 */
    bool waiting = (s_pending_sending || s_awaiting_reply) && !s_appr_pending &&
                   (count == 0 || last_is_mine);

    bool empty = (count == 0 && !s_appr_pending && !waiting && !echo);
    if (s_loading_label != NULL && lv_obj_is_valid(s_loading_label))
    {
        if (empty)
            lv_obj_clear_flag(s_loading_label, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(s_loading_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (empty)
        return; /* nothing to render yet — the centered placeholder is up */

    for (int i = 0; i < count; i++)
    {
        chat_msg_t *cm = &s_pending_msgs[i];
        bool mine = (strcmp(cm->role, "user") == 0 || strcmp(cm->role, "outgoing") == 0);

        /* Hermes(AI)房:桌面 ConversationPane 同款 —— 使用者=全寬玻璃卡、AI=平鋪
           全寬文字(見 chat_add_hermes_turn 的說明)。@聯絡人房走下面的氣泡。 */
        if (s_hermes_style)
        {
            lv_obj_t *lbl = chat_add_hermes_turn(s_msg_list, chat_msg_text(cm), mine);
            if (i == count - 1 && !echo)
            {
                /* 直播那一則:掛上揭露游標。新文字若是已畫內容的**延長**(拿已畫長度
                   的雜湊對一次),就保留已畫的部分、只接後面 —— 不會閃回開頭;對不上
                   代表來源回頭改了前面,才從 0 重畫(founder 最早提的那個漏洞)。 */
                if (!mine && !s_appr_pending)
                {
                    const char *t = chat_msg_text(cm);
                    int tl = (int)strlen(t);
                    int keep = 0;
                    bool trim = true;
                    if (!s_live_turn)
                    {
                        /* 不是我等來的(讀歷史)→ 整段直接出現。
                           **不要清掉再貼回去** —— chat_add_hermes_turn 剛剛才把完整內容
                           放上去了,重貼一遭是多餘的;而且那是在 flex 版面正在建立
                           的當下連續做 ins_text,外層卡片的 LV_SIZE_CONTENT 高度會停在
                           「空字串」那個值,整則被裁成看不見(founder 2026-08-26)。 */
                        keep = tl;
                        trim = false;
                    }
                    else if (s_reveal_pos > 0 && s_reveal_pos <= tl &&
                             chat_fnv1a(t, (size_t)s_reveal_pos) == s_paint_hash)
                        keep = s_reveal_pos;
                    s_reveal_src = t;
                    s_reveal_total = tl;
                    s_reveal_pos = keep;
                    s_paint_hash = chat_fnv1a(t, (size_t)keep);
                    if (trim)
                        chat_reveal_paint_upto(lbl, t, keep);
#if CHAT_DEBUG_HUD
                    s_dbg_keep = trim ? keep : -1; /* -1 = 完全沒動 label */
#endif
                    s_live_lbl = lbl;
                }
                else if (mine)
                {
                    /* 換我講話 → 游標歸零,下一則 AI 回覆從頭打。 */
                    s_reveal_src = NULL;
                    s_reveal_total = 0;
                    s_reveal_pos = 0;
                    s_paint_hash = 0;
                }
            }
            continue;
        }

        /* Full-width transparent row → the bubble aligns left (them) / right (me) within it. */
        lv_obj_t *row = lv_obj_create(s_msg_list);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_CLICKABLE); /* 同 hermes 卡片:讓手勢落到 list */
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *bubble = lv_obj_create(row);
        lv_obj_set_width(bubble, LV_SIZE_CONTENT);
        lv_obj_set_height(bubble, LV_SIZE_CONTENT);
        /* Desktop ConversationWindow parity (founder 2026-06-29): bubble = pad 11×7; mine = Skaiwalk
           sky-accent-deep #5C9CB8 (a MUTED brand sky-blue, NOT iOS systemBlue — the desktop's
           SkSkyAccentDeep token), theirs = systemGray5 #2C2C2E.
           Radius 21 (not the desktop's 14): the watch body font is taller than the desktop's 14px, so
           the same 14px radius read as squarer on the taller bubble — bumped so the perceived
           roundness (radius÷height) matches the desktop; founder tuned 14→18→21 by eye (2026-06-29). */
        lv_obj_set_style_pad_hor(bubble, 11, 0);
        lv_obj_set_style_pad_ver(bubble, 7, 0);
        lv_obj_set_style_radius(bubble, 21, 0);
        lv_obj_set_style_border_width(bubble, 0, 0);
        lv_obj_set_style_bg_color(bubble, mine ? lv_color_hex(0x5C9CB8) : lv_color_hex(0x2C2C2E), 0);
        lv_obj_clear_flag(bubble, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(bubble, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_align(bubble, mine ? LV_ALIGN_TOP_RIGHT : LV_ALIGN_TOP_LEFT, 0, 0);

        lv_obj_t *lbl = lv_label_create(bubble);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(lbl, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        /* Content-sized bubble: hug short text, but WRAP (not clip) once it would exceed ~72% of the
           screen. LVGL's max_width on a SIZE_CONTENT label CLIPS instead of re-wrapping (founder
           2026-06-29: long messages were cut off) — so measure the text's natural width and pin the
           label to the cap ONLY when it overflows; otherwise let it shrink to content. */
        {
            const lv_font_t *fnt = lv_obj_get_style_text_font(lbl, LV_PART_MAIN);
            lv_coord_t lsp = lv_obj_get_style_text_letter_space(lbl, LV_PART_MAIN);
            lv_coord_t lnsp = lv_obj_get_style_text_line_space(lbl, LV_PART_MAIN);
            lv_point_t tsz;
            lv_txt_get_size(&tsz, chat_msg_text(cm), fnt, lsp, lnsp, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
            lv_coord_t cap = (LV_HOR_RES * 72) / 100;
            lv_obj_set_width(lbl, tsz.x > cap ? cap : LV_SIZE_CONTENT);
        }
        lv_label_set_text(lbl, chat_msg_text(cm));
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
    }

    if (echo)
        chat_add_hermes_turn(s_msg_list, s_local_echo, true);

    if (waiting)
        chat_render_waiting();

    chat_render_pending_approval();

    lv_obj_scroll_to_y(s_msg_list, LV_COORD_MAX, LV_ANIM_OFF); /* pin to the newest */
}
