/*
 * voice_ai_logo — 見 voice_ai_logo.h。
 *
 * 狀態機(全域唯一,手錶上同時只有一段語音):
 *   IDLE ──點一下──▶ SETTLING(畫面剛停錄,等最後幾個字)──▶ AWAIT_REPLY(0x2a polish)──▶ IDLE
 *   IDLE ──按住───▶ WAIT_MIC(語音管線冷卻)──▶ INSTRUCTING(錄修改指示)
 *                    ──放開──▶ AWAIT_REPLY(0x2a amend)──▶ IDLE
 *   WAIT_MIC 期間就放開 = 這一按不算數。
 */
#include "voice_ai_logo.h"
#include <rtthread.h>
#include <string.h>
#include "ui_handler.h"
#include "watch_system_interact.h"
#include "bloc_v2t.h"

#define DBG_TAG "voice_ai_logo"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

LV_IMG_DECLARE(img_logo);

extern bool commu_send_voice_ai(int id, const char *op, const char *text);

#define VAI_ACCENT 0x5DA8FF          /* 與底部 bar 麥克風的漣漪同色(LMIC_RIPPLE_COLOR) */
#define VAI_BG 0x1C1C1E
/* 停掉語音後到能再開麥克風的冷卻。真機 2026-07-31:STOP 後 0.37s 就 START → voice 執行緒
   hard fault(lv_instruction_list_layout.c 的 LIFT_VOICE_RESTART_COOLDOWN_MS 同一個坑)。 */
#define VAI_MIC_COOLDOWN_MS 800
/* 畫面剛停錄時,最後一段轉錄從手機回來要 0.5~2s;太早取文字會少掉最後幾個字,
   而那幾個字晚到又會把潤色後的結果蓋掉。 */
#define VAI_SETTLE_MS 1500
/* 按住短於這個 = 手指只是碰了一下,裡面不會有一句話(同立起面板 LIFT_VOICE_MIN_UTTERANCE_MS)。 */
#define VAI_MIN_INSTRUCTION_MS 700
/* 手機沒回(斷線/模型掛了)時,多久後放棄等待、讓 logo 回到可用。 */
#define VAI_REPLY_TIMEOUT_MS 20000

typedef enum
{
    VAI_IDLE = 0,
    VAI_SETTLING,
    VAI_WAIT_MIC,
    VAI_INSTRUCTING,
    VAI_AWAIT_REPLY,
} vai_phase_t;

static vai_phase_t s_phase = VAI_IDLE;
static const voice_ai_ops_t *s_ops = NULL; /* 這次動作是哪個畫面的 */
static lv_obj_t *s_btn = NULL;             /* 這次動作的那顆 logo */
static bool s_pressed = false;             /* 手指還按在 logo 上 */
static int s_req_id = 0;
static rt_tick_t s_dictation_stopped_at = 0; /* 最近一次停掉語音(畫面的或按住的)的時刻 */
static rt_tick_t s_instr_started_at = 0;

/* BLE 執行緒會讀:按住中或剛放開(遲到的指示逐字稿也要攔下,不能掉進畫面)。 */
static volatile bool s_capturing = false;

static lv_timer_t *s_settle_timer = NULL;
static lv_timer_t *s_mic_timer = NULL;
static lv_timer_t *s_timeout_timer = NULL;

static lv_obj_t *s_caption = NULL; /* logo 上方的小字泡:提示 / 指示逐字稿 / 整理中 */

/* 指示逐字稿:BLE → GUI 的單槽。PC 模擬器沒有 BLE 執行緒(也沒有 rt_hw_interrupt_*),不必鎖。 */
static char *s_instr_pending = NULL;
#ifndef BSP_USING_PC_SIMULATOR
#define VAI_LOCK() rt_base_t vai_level = rt_hw_interrupt_disable()
#define VAI_UNLOCK() rt_hw_interrupt_enable(vai_level)
#else
#define VAI_LOCK() do { } while (0)
#define VAI_UNLOCK() do { } while (0)
#endif

/* ── helpers ─────────────────────────────────────────────────────────────── */

static uint32_t vai_ms_since(rt_tick_t t)
{
    return (uint32_t)((rt_tick_get() - t) * 1000 / RT_TICK_PER_SECOND);
}

static void vai_timer_kill(lv_timer_t **t)
{
    if (*t != NULL)
    {
        lv_timer_del(*t);
        *t = NULL;
    }
}

static void vai_reject(void)
{
    extern void motor_pattern_unlocked(void);
    motor_pattern_unlocked();
}

static bool vai_btn_valid(void)
{
    return s_btn != NULL && lv_obj_is_valid(s_btn);
}

static void vai_caption_show(const char *text)
{
    if (!vai_btn_valid())
        return;
    if (s_caption == NULL || !lv_obj_is_valid(s_caption))
    {
        s_caption = lv_label_create(lv_layer_top());
        lv_obj_set_style_bg_color(s_caption, lv_color_hex(VAI_BG), 0);
        lv_obj_set_style_bg_opa(s_caption, LV_OPA_90, 0);
        lv_obj_set_style_radius(s_caption, 14, 0);
        lv_obj_set_style_pad_hor(s_caption, 12, 0);
        lv_obj_set_style_pad_ver(s_caption, 6, 0);
        lv_obj_set_style_text_color(s_caption, lv_color_hex(VAI_ACCENT), 0);
        lv_label_set_long_mode(s_caption, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_max_width(s_caption, 300, 0);
        lv_obj_set_width(s_caption, LV_SIZE_CONTENT);
        lv_obj_clear_flag(s_caption, LV_OBJ_FLAG_CLICKABLE);
    }
    lv_label_set_text(s_caption, text);
    lv_obj_clear_flag(s_caption, LV_OBJ_FLAG_HIDDEN);
    lv_obj_update_layout(s_caption);
    lv_obj_align_to(s_caption, s_btn, LV_ALIGN_OUT_TOP_MID, 0, -12);
    /* 貼著螢幕上緣時改放到 logo 下方,別被切掉。 */
    if (lv_obj_get_y(s_caption) < 8)
        lv_obj_align_to(s_caption, s_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 12);
}

static void vai_caption_hide(void)
{
    if (s_caption != NULL && lv_obj_is_valid(s_caption))
        lv_obj_add_flag(s_caption, LV_OBJ_FLAG_HIDDEN);
}

/* logo 本身的三種樣子:平常(淡框)/ 按住錄音中(亮框+漣漪)/ 等手機(轉圈)。
   漣漪與轉圈是 logo 的子物件,存在 user_data 裡找得到。 */
typedef struct
{
    lv_obj_t *ring;
    lv_obj_t *spinner;
    const voice_ai_ops_t *ops;
} vai_parts_t;

static void vai_ring_anim_cb(void *var, int32_t v)
{
    lv_obj_t *ring = (lv_obj_t *)var;
    lv_obj_t *btn = lv_obj_get_parent(ring);
    lv_coord_t base = lv_obj_get_width(btn);
    lv_coord_t d = base + (base * v) / 400; /* v 0..200 → 1.0x..1.5x */
    lv_obj_set_size(ring, d, d);
    lv_obj_center(ring);
    lv_obj_set_style_border_opa(ring, (lv_opa_t)(LV_OPA_COVER - (v * LV_OPA_COVER) / 200), 0);
}

static void vai_visual(lv_obj_t *btn, vai_phase_t phase)
{
    if (btn == NULL || !lv_obj_is_valid(btn))
        return;
    vai_parts_t *p = (vai_parts_t *)lv_obj_get_user_data(btn);
    bool instructing = phase == VAI_WAIT_MIC || phase == VAI_INSTRUCTING;
    bool working = phase == VAI_SETTLING || phase == VAI_AWAIT_REPLY;
    lv_obj_set_style_border_opa(btn, instructing ? LV_OPA_COVER : LV_OPA_50, 0);
    lv_obj_set_style_transform_zoom(btn, instructing ? 282 : 256, 0); /* 按住時放大一點 */
    if (p == NULL)
        return;
    if (p->ring != NULL && lv_obj_is_valid(p->ring))
    {
        lv_anim_del(p->ring, vai_ring_anim_cb);
        if (phase == VAI_INSTRUCTING)
        {
            lv_obj_clear_flag(p->ring, LV_OBJ_FLAG_HIDDEN);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, p->ring);
            lv_anim_set_values(&a, 0, 200);
            lv_anim_set_time(&a, 900);
            lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
            lv_anim_set_exec_cb(&a, vai_ring_anim_cb);
            lv_anim_start(&a);
        }
        else
        {
            lv_obj_add_flag(p->ring, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (p->spinner != NULL && lv_obj_is_valid(p->spinner))
    {
        if (working)
            lv_obj_clear_flag(p->spinner, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(p->spinner, LV_OBJ_FLAG_HIDDEN);
    }
}

static void vai_set_phase(vai_phase_t phase)
{
    s_phase = phase;
    vai_visual(s_btn, phase);
}

/* 回到 IDLE:計時器、字泡、樣子都收掉。不動畫面的文字。 */
static void vai_reset(void)
{
    vai_timer_kill(&s_settle_timer);
    vai_timer_kill(&s_mic_timer);
    vai_timer_kill(&s_timeout_timer);
    vai_caption_hide();
    s_capturing = false;
    vai_set_phase(VAI_IDLE);
    s_ops = NULL;
    s_btn = NULL;
}

/* ── 送請求 / 等回覆 ───────────────────────────────────────────────────── */

static void vai_timeout_cb(lv_timer_t *t)
{
    (void)t;
    s_timeout_timer = NULL;
    LOG_W("[voice_ai] no reply, giving up id=%d", s_req_id);
    s_req_id++; /* 之後才到的那筆作廢 */
    vai_reject();
    vai_reset();
}

static void vai_send(const char *op)
{
    const char *text = (s_ops && s_ops->get_text) ? s_ops->get_text() : NULL;
    if (text == NULL)
        text = "";
    bool polish = strcmp(op, "polish") == 0;
    if (polish && text[0] == '\0')
    {
        LOG_W("[voice_ai] %s: nothing to polish", s_ops ? s_ops->tag : "?");
        vai_reject();
        vai_reset();
        return;
    }
    s_req_id++;
#ifndef BSP_USING_PC_SIMULATOR
    bool sent = commu_send_voice_ai(s_req_id, op, text);
#else
    bool sent = false;
#endif
    if (!sent)
    {
        /* 斷線,或文字長到超過一個 L2 上行。 */
        LOG_W("[voice_ai] %s request not sent len=%d", op, (int)strlen(text));
        vai_reject();
        vai_reset();
        return;
    }
    LOG_W("[voice_ai] %s: %s sent", s_ops ? s_ops->tag : "?", op);
    vai_set_phase(VAI_AWAIT_REPLY);
    vai_caption_show("AI 整理中…");
    vai_timer_kill(&s_timeout_timer);
    s_timeout_timer = lv_timer_create(vai_timeout_cb, VAI_REPLY_TIMEOUT_MS, NULL);
    lv_timer_set_repeat_count(s_timeout_timer, 1);
}

void voice_ai_logo_on_result(int id, bool ok, const char *text)
{
    if (s_phase != VAI_AWAIT_REPLY || id != s_req_id)
    {
        LOG_W("[voice_ai] stale result id=%d want=%d", id, s_req_id);
        return;
    }
    LOG_W("[voice_ai] result id=%d ok=%d", id, (int)ok);
    const voice_ai_ops_t *ops = s_ops;
    bool owner_alive = vai_btn_valid();
    vai_reset();
    if (!ok || text == NULL || text[0] == '\0')
    {
        vai_reject(); /* 原文不動,震一下讓人知道沒改到 */
        return;
    }
    if (owner_alive && ops && ops->set_text)
        ops->set_text(text);
}

/* ── 點一下:潤色 ─────────────────────────────────────────────────────── */

static void vai_settle_cb(lv_timer_t *t)
{
    (void)t;
    s_settle_timer = NULL;
    if (s_phase != VAI_SETTLING)
        return;
    vai_send("polish");
}

static void vai_tap(lv_obj_t *btn, const voice_ai_ops_t *ops)
{
    if (s_phase != VAI_IDLE)
        return;
    if (ops->on_tap)
    {
        ops->on_tap();
        return;
    }
    s_ops = ops;
    s_btn = btn;
    bool was = ops->stop_dictation ? ops->stop_dictation() : false;
    if (was)
        s_dictation_stopped_at = rt_tick_get();
    vai_set_phase(VAI_SETTLING);
    vai_caption_show("AI 整理中…");
    vai_timer_kill(&s_settle_timer);
    s_settle_timer = lv_timer_create(vai_settle_cb, was ? VAI_SETTLE_MS : 1, NULL);
    lv_timer_set_repeat_count(s_settle_timer, 1);
}

/* ── 按住:修改指示 ───────────────────────────────────────────────────── */

static void vai_mic_start(void)
{
#ifndef BSP_USING_PC_SIMULATOR
    extern bool get_bluetooth_connection_status(void);
    if (!get_bluetooth_connection_status())
    {
        LOG_W("[voice_ai] hold: not connected");
        vai_reject();
        vai_reset();
        return;
    }
    interact_amend_v2t_input(true);
#endif
    s_capturing = true;
    s_instr_started_at = rt_tick_get();
    vai_set_phase(VAI_INSTRUCTING);
    vai_caption_show("說出要怎麼改…");
    LOG_W("[voice_ai] %s: instruction START", s_ops ? s_ops->tag : "?");
}

static void vai_mic_timer_cb(lv_timer_t *t)
{
    (void)t;
    s_mic_timer = NULL;
    if (s_phase != VAI_WAIT_MIC)
        return;
    if (!s_pressed)
    {
        vai_reset();
        return;
    }
    vai_mic_start();
}

static void vai_hold_start(lv_obj_t *btn, const voice_ai_ops_t *ops)
{
    if (s_phase != VAI_IDLE)
        return;
    s_ops = ops;
    s_btn = btn;
    if (ops->stop_dictation && ops->stop_dictation())
        s_dictation_stopped_at = rt_tick_get();
    vai_set_phase(VAI_WAIT_MIC);
    vai_caption_show("說出要怎麼改…");
    uint32_t since = s_dictation_stopped_at ? vai_ms_since(s_dictation_stopped_at) : VAI_MIC_COOLDOWN_MS;
    uint32_t wait = since >= VAI_MIC_COOLDOWN_MS ? 1 : VAI_MIC_COOLDOWN_MS - since + 20;
    vai_timer_kill(&s_mic_timer);
    s_mic_timer = lv_timer_create(vai_mic_timer_cb, wait, NULL);
    lv_timer_set_repeat_count(s_mic_timer, 1);
}

static void vai_hold_end(void)
{
    if (s_phase == VAI_WAIT_MIC)
    {
        /* 麥克風還沒開就放開了 —— 這一按不算數。 */
        LOG_W("[voice_ai] hold released before the mic opened");
        vai_reset();
        return;
    }
    if (s_phase != VAI_INSTRUCTING)
        return;
    uint32_t held_ms = vai_ms_since(s_instr_started_at);
#ifndef BSP_USING_PC_SIMULATOR
    interact_amend_v2t_input(false);
#endif
    s_dictation_stopped_at = rt_tick_get();
    LOG_W("[voice_ai] instruction STOP held=%ums", (unsigned)held_ms);
    if (held_ms < VAI_MIN_INSTRUCTION_MS)
    {
        vai_reject();
        vai_reset();
        return;
    }
    /* s_capturing 留著:指示的最後幾個字還在路上,到之前不能掉進畫面。收到結果才放。 */
    vai_send("amend");
    if (s_phase == VAI_AWAIT_REPLY)
        s_capturing = true;
}

/* ── 指示逐字稿(BLE → GUI)──────────────────────────────────────────── */

bool voice_ai_logo_capturing(void)
{
    return s_capturing;
}

void voice_ai_logo_post_instruction(const uint8_t *text, uint16_t len)
{
    char *buf = (char *)rt_malloc((rt_size_t)len + 1);
    if (buf == NULL)
        return;
    if (len > 0 && text != NULL)
        memcpy(buf, text, len);
    buf[len] = '\0';
    VAI_LOCK();
    char *old = s_instr_pending;
    s_instr_pending = buf;
    VAI_UNLOCK();
    if (old != NULL)
        rt_free(old);
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_VOICE_AI_INSTRUCTION;
    lvgl_send_msg(msg);
}

void voice_ai_logo_apply_instruction(void)
{
    VAI_LOCK();
    char *buf = s_instr_pending;
    s_instr_pending = NULL;
    VAI_UNLOCK();
    if (buf == NULL)
        return;
    if (s_phase == VAI_INSTRUCTING && buf[0] != '\0')
        vai_caption_show(buf);
    rt_free(buf);
}

/* ── 事件 ─────────────────────────────────────────────────────────────── */

static void vai_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_current_target(e);
    vai_parts_t *p = (vai_parts_t *)lv_obj_get_user_data(btn);
    const voice_ai_ops_t *ops = p ? p->ops : NULL;
    switch (code)
    {
    case LV_EVENT_PRESSED:
        s_pressed = true;
        break;
    case LV_EVENT_SHORT_CLICKED:
        if (ops)
            vai_tap(btn, ops);
        break;
    case LV_EVENT_LONG_PRESSED:
        if (ops)
            vai_hold_start(btn, ops);
        break;
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        s_pressed = false;
        if (s_btn == btn)
            vai_hold_end();
        break;
    case LV_EVENT_DELETE:
        if (s_btn == btn)
        {
            /* 畫面關了:按住中的錄音要停,等待中的回覆作廢(沒有地方可以寫了)。 */
            if (s_phase == VAI_INSTRUCTING)
            {
#ifndef BSP_USING_PC_SIMULATOR
                interact_amend_v2t_input(false);
#endif
                s_dictation_stopped_at = rt_tick_get();
            }
            s_req_id++;
            s_btn = NULL; /* 已經在刪,別再碰它的樣子 */
            vai_reset();
        }
        if (p)
        {
            lv_obj_set_user_data(btn, NULL);
            lv_mem_free(p);
        }
        break;
    default:
        break;
    }
}

static void vai_bind(lv_obj_t *obj, const voice_ai_ops_t *ops, lv_obj_t *ring, lv_obj_t *spinner)
{
    vai_parts_t *p = (vai_parts_t *)lv_mem_alloc(sizeof(vai_parts_t));
    if (p == NULL)
        return;
    p->ring = ring;
    p->spinner = spinner;
    p->ops = ops;
    lv_obj_set_user_data(obj, p);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(obj, vai_event_cb, LV_EVENT_ALL, NULL);
}

lv_obj_t *voice_ai_logo_create(lv_obj_t *parent, const voice_ai_ops_t *ops, lv_coord_t size)
{
    lv_obj_t *btn = lv_obj_create(parent);
    lv_obj_remove_style_all(btn);
    lv_obj_set_size(btn, size, size);
    lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(VAI_BG), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(VAI_ACCENT), 0);
    lv_obj_set_style_border_width(btn, 2, 0);
    lv_obj_set_style_border_opa(btn, LV_OPA_50, 0);
    lv_obj_set_style_transform_pivot_x(btn, size / 2, 0);
    lv_obj_set_style_transform_pivot_y(btn, size / 2, 0);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_ext_click_area(btn, 10);

    lv_obj_t *img = lv_img_create(btn);
    lv_img_set_src(img, &img_logo);
    /* img_logo 是 80px;縮到圓的 62%。 */
    lv_img_set_zoom(img, (uint16_t)((size * 62 / 100) * 256 / 80));
    lv_obj_add_flag(img, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_center(img);

    /* 按住錄音時往外擴散的漣漪。 */
    lv_obj_t *ring = lv_obj_create(btn);
    lv_obj_remove_style_all(ring);
    lv_obj_set_size(ring, size, size);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_color(ring, lv_color_hex(VAI_ACCENT), 0);
    lv_obj_set_style_border_width(ring, 3, 0);
    lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(ring, LV_OBJ_FLAG_HIDDEN);
    lv_obj_center(ring);

    /* 等手機時的轉圈。 */
    lv_obj_t *spinner = lv_spinner_create(btn, 1000, 90);
    lv_obj_set_size(spinner, size + 8, size + 8);
    lv_obj_center(spinner);
    lv_obj_set_style_arc_width(spinner, 3, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(spinner, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_arc_width(spinner, 3, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(spinner, lv_color_hex(VAI_ACCENT), LV_PART_INDICATOR);
    lv_obj_clear_flag(spinner, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(spinner, LV_OBJ_FLAG_HIDDEN);

    vai_bind(btn, ops, ring, spinner);
    return btn;
}

void voice_ai_logo_attach(lv_obj_t *obj, const voice_ai_ops_t *ops)
{
    if (obj == NULL)
        return;
    vai_bind(obj, ops, NULL, NULL);
}

bool voice_ai_logo_busy(void)
{
    return s_phase != VAI_IDLE;
}

void voice_ai_logo_replace_v2t(const char *text)
{
    clearVoice2Text();
    if (text == NULL || text[0] == '\0')
        return;
    VOICE_RECOGNITION_PAYLOAD p;
    p.header = get_speech_coding();
    p.p_msg_value = (uint8_t *)text;
    p.length = (uint16_t)strlen(text);
    handle_v2t_result(&p);
}
