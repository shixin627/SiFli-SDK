/*
 * voice_ai_logo — 手錶上每個語音輸入畫面共用的 Skai logo(founder 2026-09-23)。
 *
 * 跟手機輸入框上方那顆一樣:
 *   點一下       = 用 AI 語意理解並潤色畫面上的文字(去贅字、套用自我更正、還原口語拆字、補標點)。
 *   按住 + 說話  = 這段話是對畫面上文字的「修改指示」,放開後套用
 *                  (「林是新」+ 按住說「森林的林士兵的士心臟的心」→「林士心」)。
 *
 * 文字在手錶(每個畫面自己的),模型在手機:KEY_VOICE_AI 0x2a 上行請求、KEY_VOICE_AI_RESULT 0x2b
 * 回結果。按住那段錄音走 V2T_INTENT_AMEND,手機與 interact_voice_recognition 都不把它當成
 * 畫面的逐字稿。
 *
 * 畫面只要提供 voice_ai_ops_t,把 voice_ai_logo_create() 回傳的物件擺到想要的位置即可。
 * 同一時間只有一個 logo 在動作(手錶上同時只會有一段語音)。只能在 LVGL 執行緒呼叫。
 */
#ifndef VOICE_AI_LOGO_H
#define VOICE_AI_LOGO_H

#include "lvgl.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* log 用的畫面名。 */
    const char *tag;
    /* 畫面目前的文字(沒有 = "" 或 NULL)。 */
    const char *(*get_text)(void);
    /* 用 AI 的結果換掉畫面的文字。 */
    void (*set_text)(const char *text);
    /* logo 要動作了:停掉畫面自己的錄音但**不要送出**,文字留著。回傳剛才是否真的在錄 ——
       是的話 logo 會等最後幾個字從手機回來再取文字。NULL = 畫面沒有自己的錄音可停。 */
    bool (*stop_dictation)(void);
    /* 選填:點一下改做畫面自己的事,不潤色(立起面板的 logo 本來就是「送去 skaibar」,
       那裡的文字放開麥克風時手機已經自動整理過)。按住修改照常。NULL = 點一下潤色。 */
    void (*on_tap)(void);
} voice_ai_ops_t;

/* 建一顆 logo(size×size 的圓,置中 Skai logo)。ops 必須是 static 生命週期。 */
lv_obj_t *voice_ai_logo_create(lv_obj_t *parent, const voice_ai_ops_t *ops, lv_coord_t size);

/* 把「按住說話 = 修改」接到畫面上已經存在的 logo 物件(立起面板),點一下的行為交給
   ops->on_tap。 */
void voice_ai_logo_attach(lv_obj_t *obj, const voice_ai_ops_t *ops);

/* logo 正在等錄音/等手機:畫面這時不該自己開麥克風,也不該把文字送出去。 */
bool voice_ai_logo_busy(void);

/* 用 text 取代手錶的語音緩衝(get_combined_voice2text 讀到的那份)。畫面的文字真相就是這份
   緩衝時(skaibar 語音框、語音 app、訊息回覆),set_text 用它 —— 送出時才會送改過的字。 */
void voice_ai_logo_replace_v2t(const char *text);

/* ── 下面是給 BLE / ui_handler 的接口 ── */
/* BLE 執行緒:按住那段錄音進行中或剛放開(修改指示的逐字稿要攔下來,不進畫面)。 */
bool voice_ai_logo_capturing(void);
/* BLE 執行緒:修改指示的逐字稿(interact_voice_recognition 攔下來的)。 */
void voice_ai_logo_post_instruction(const uint8_t *text, uint16_t len);
/* GUI 執行緒(LVGL_MSG_TYPE_VOICE_AI_INSTRUCTION)。 */
void voice_ai_logo_apply_instruction(void);
/* GUI 執行緒:手機回來的結果(0x2b,由 voice_ai_result_apply_pending 解好)。 */
void voice_ai_logo_on_result(int id, bool ok, const char *text);

#ifdef __cplusplus
}
#endif

#endif /* VOICE_AI_LOGO_H */
