/**
 ******************************************************************************
 * @file   lv_chat_page.c
 * @brief  In-watch @-conversation chat room (P5). See lv_chat_page.h.
 ******************************************************************************
 */

#include "lvgl.h"
#include "lv_chat_page.h"
#include "communicate_task.h"
#include "ui_handler.h"
#include "bloc_v2t.h"
#include <cJSON.h>
#include <string.h>

#define DBG_TAG "lv_chat_page"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* The floating chat-room panel on lv_layer_top (above the @-list layer), or NULL when
   closed. A FRESH panel is built per open + torn down on close (no reuse). */
static lv_obj_t *s_chat_panel = NULL;
static lv_obj_t *s_title_label = NULL;
static lv_obj_t *s_msg_list = NULL; /* scrollable flex column the messages render into */

/* Pending conv-state: parsed off the BLE thread (skai_chat_on_conv_state) into bounded STATIC
   buffers (no heap on the 4KB BLE stack), then rendered on the LVGL thread
   (chat_page_apply_pending_state) via LVGL_MSG_TYPE_REFRESH_CHAT. */
#define CHAT_MAX_MSGS 16
#define CHAT_MSG_TEXT_LEN 256
typedef struct
{
    char role[12]; /* user / ai / assistant / incoming / outgoing */
    char text[CHAT_MSG_TEXT_LEN];
} chat_msg_t;
static char s_pending_title[64];
static bool s_pending_sending;
static chat_msg_t s_pending_msgs[CHAT_MAX_MSGS];
static volatile int s_pending_msg_count; /* written LAST on BLE, read FIRST on LVGL */

bool chat_page_is_open(void)
{
    return s_chat_panel != NULL && lv_obj_is_valid(s_chat_panel);
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

static lv_obj_t *s_mic_btn = NULL;
static lv_obj_t *s_mic_label = NULL;
static bool s_recording = false;

static void chat_set_mic_visual(bool recording)
{
    if (s_mic_btn != NULL && lv_obj_is_valid(s_mic_btn))
        lv_obj_set_style_bg_color(s_mic_btn, recording ? lv_color_hex(0xE0245E) : lv_color_hex(0x0A84FF), 0);
    if (s_mic_label != NULL && lv_obj_is_valid(s_mic_label))
        lv_label_set_text(s_mic_label, recording ? "錄音中·點送出" : "說話");
}

static void chat_stop_recording_and_send(void)
{
    if (!s_recording)
        return;
    s_recording = false;
    voice_provider.auto_stop_listening(); /* finalize (mirror app_skai's send_to_ai) */
    chat_set_mic_visual(false);
    const char *text = get_combined_voice2text();
    if (text != NULL && text[0] != '\0')
    {
        commu_send_conv_send(text);
        clearVoice2Text();
    }
}

static void chat_mic_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
    if (!s_recording)
    {
        clearVoice2Text();
        /* Use the PROVEN voice path app_skai's AI widget uses (voice_provider.start_v2t): the bare
           start_voice_recognition didn't trigger the phone-side transcription (it returned
           KEY_VOICE_RECOGNITION_END:0 — no text). start_v2t does the full setup + tells the phone to
           listen. The result streams back via interact_voice_recognition → our chat branch (FIRST). */
        voice_provider.start_v2t();
        s_recording = true;
        chat_set_mic_visual(true);
        LOG_I("chat mic: recording start");
    }
    else
    {
        LOG_I("chat mic: stop + send");
        chat_stop_recording_and_send();
    }
}

void chat_page_open(const char *title)
{
    if (chat_page_is_open())
        chat_page_close();

    lv_obj_t *panel = lv_obj_create(lv_layer_top());
    lv_obj_set_size(panel, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(panel, 0, 0);
    lv_obj_set_style_radius(panel, 0, 0);
    lv_obj_set_style_border_width(panel, 0, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_set_style_bg_color(panel, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    /* Deliberately leave the panel NON-clickable: a clickable full-screen panel would CAPTURE the
       left-edge swipe and starve the watch's native edge-back detect object. Non-clickable, the
       swipe passes through to the native back (→ ESC → handle_back_event closes this chat first). */

    /* Title — the contact / AI name. No back button: the native left-edge swipe-back closes the room. */
    lv_obj_t *title_lbl = lv_label_create(panel);
    lv_label_set_long_mode(title_lbl, LV_LABEL_LONG_DOT);
    lv_obj_set_width(title_lbl, LV_HOR_RES - 40);
    lv_obj_set_style_text_align(title_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(title_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(title_lbl, (title && title[0]) ? title : "聊天室");
    lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 20);
    s_title_label = title_lbl;

    /* Scrollable message column. Sits between the title and the bottom mic button. */
    lv_obj_t *list = lv_obj_create(panel);
    lv_obj_set_size(list, LV_HOR_RES - 120, LV_VER_RES - 150);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 52);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list, 6, 0);
    lv_obj_set_style_pad_all(list, 4, 0);
    s_msg_list = list;

    /* Placeholder until the first conv-state lands. */
    lv_obj_t *hint = lv_label_create(list);
    lv_label_set_text(hint, "連線中…");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x888888), 0);

    /* Mic / voice-input button (bottom-center, inside the round display) — tap to record, tap again
       to send. Drives V2T (chat_mic_btn_cb). The left-edge swipe-back is unaffected (it's at x≈0). */
    lv_obj_t *mic = lv_btn_create(panel);
    lv_obj_set_size(mic, 160, 58);
    lv_obj_align(mic, LV_ALIGN_BOTTOM_MID, 0, -14);
    lv_obj_set_style_radius(mic, 29, 0);
    lv_obj_set_style_bg_color(mic, lv_color_hex(0x0A84FF), 0);
    lv_obj_add_event_cb(mic, chat_mic_btn_cb, LV_EVENT_CLICKED, NULL);
    s_mic_btn = mic;
    lv_obj_t *mic_lbl = lv_label_create(mic);
    lv_label_set_text(mic_lbl, "說話");
    lv_obj_set_style_text_color(mic_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(mic_lbl);
    s_mic_label = mic_lbl;
    s_recording = false;

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
        lvsf_gesture_bring_to_front();
    }

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
    lv_obj_del(s_chat_panel);
    s_chat_panel = NULL;
    s_title_label = NULL;
    s_msg_list = NULL;
    s_mic_btn = NULL;
    s_mic_label = NULL;
    LOG_I("chat page closed");
}

/* DOWNLINK (KEY_CONV_STATE, BLE parse thread): parse {title, sending, messages:[{role,text}]} into
   the pending STATIC buffer (bounded copies — heap-only cJSON, no 4KB BLE-stack blowup), then DEFER
   the render to the LVGL thread (LVGL ops are not thread-safe off the GUI task). Defined here so
   resolve_skailink_command links against a STRONG symbol regardless. */
void skai_chat_on_conv_state(const uint8_t *json, uint16_t length)
{
    if (json == NULL || length == 0)
        return;
    cJSON *root = cJSON_ParseWithLength((const char *)json, length);
    if (!cJSON_IsObject(root))
    {
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

    s_pending_sending = cJSON_IsTrue(cJSON_GetObjectItem(root, "sending"));

    int count = 0;
    cJSON *j_msgs = cJSON_GetObjectItem(root, "messages");
    if (cJSON_IsArray(j_msgs))
    {
        cJSON *m = NULL;
        cJSON_ArrayForEach(m, j_msgs)
        {
            if (count >= CHAT_MAX_MSGS)
                break;
            cJSON *j_text = cJSON_GetObjectItem(m, "text");
            if (!cJSON_IsString(j_text) || j_text->valuestring[0] == '\0')
                continue;
            cJSON *j_role = cJSON_GetObjectItem(m, "role");
            const char *role = cJSON_IsString(j_role) ? j_role->valuestring : "";
            strncpy(s_pending_msgs[count].role, role, sizeof(s_pending_msgs[count].role) - 1);
            s_pending_msgs[count].role[sizeof(s_pending_msgs[count].role) - 1] = '\0';
            strncpy(s_pending_msgs[count].text, j_text->valuestring, sizeof(s_pending_msgs[count].text) - 1);
            s_pending_msgs[count].text[sizeof(s_pending_msgs[count].text) - 1] = '\0';
            count++;
        }
    }
    cJSON_Delete(root);

    s_pending_msg_count = count; /* publish LAST so an LVGL reader never sees a half-filled buffer */
    LOG_I("conv_state rx: title=%s msgs=%d sending=%d", s_pending_title, count, (int)s_pending_sending);

    lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_CHAT};
    lvgl_send_msg(msg);
}

/* LVGL thread: rebuild the transcript bubbles from the pending buffer (left=them grey / right=me
   blue). No-op if the chat closed between the BLE parse and this deferred render. */
void chat_page_apply_pending_state(void)
{
    if (!chat_page_is_open() || s_msg_list == NULL || !lv_obj_is_valid(s_msg_list))
        return;

    if (s_title_label != NULL && lv_obj_is_valid(s_title_label) && s_pending_title[0] != '\0')
        lv_label_set_text(s_title_label, s_pending_title);

    lv_obj_clean(s_msg_list); /* drop the old bubbles (and the M1 "連線中…" hint) */

    int count = s_pending_msg_count;
    if (count > CHAT_MAX_MSGS)
        count = CHAT_MAX_MSGS;
    for (int i = 0; i < count; i++)
    {
        chat_msg_t *cm = &s_pending_msgs[i];
        bool mine = (strcmp(cm->role, "user") == 0 || strcmp(cm->role, "outgoing") == 0);

        /* Full-width transparent row → the bubble aligns left (them) / right (me) within it. */
        lv_obj_t *row = lv_obj_create(s_msg_list);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *bubble = lv_obj_create(row);
        lv_obj_set_width(bubble, LV_SIZE_CONTENT);
        lv_obj_set_height(bubble, LV_SIZE_CONTENT);
        lv_obj_set_style_pad_all(bubble, 8, 0);
        lv_obj_set_style_radius(bubble, 12, 0);
        lv_obj_set_style_border_width(bubble, 0, 0);
        lv_obj_set_style_bg_color(bubble, mine ? lv_color_hex(0x0A84FF) : lv_color_hex(0x333333), 0);
        lv_obj_clear_flag(bubble, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_align(bubble, mine ? LV_ALIGN_TOP_RIGHT : LV_ALIGN_TOP_LEFT, 0, 0);

        lv_obj_t *lbl = lv_label_create(bubble);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(lbl, LV_HOR_RES / 2); /* fixed wrap width inside the SIZE_CONTENT bubble */
        lv_label_set_text(lbl, cm->text);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
    }

    lv_obj_scroll_to_y(s_msg_list, LV_COORD_MAX, LV_ANIM_OFF); /* pin to the newest */
}
