/**
 ******************************************************************************
 * @file   bloc_v2t.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "communicate_protocol.h"
#include "communicate_task.h"
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "app_mainmenu.h"
#include "app_speech.h"
#include "os_adaptor.h"
#include "dfs_file.h"
#include "dfs_posix.h"
#include "bloc_control.h"
#include "bloc_v2t.h"
#include "bloc_skaiwalk.h"
#include "bloc_motion_tracking.h"
#include "bloc_peripheral.h"
#include "ui_handler.h"
#include "ui_helper.h"
#include "app_mem.h"
#include "webrtc/common_audio/vad/include/webrtc_vad.h"

#ifdef ENABLE_OPUS_ENCODER
#include "opus.h"
#include "opus_types.h"
#endif

#define DBG_TAG "bloc.v2t"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

/* Constants */
#define VOICE_ANIMATION_INTERVAL 200
#define AUDIO_REC_TEMP_SIZE 512
#define THREAD_STACK_SIZE 1536
#define THREAD_PRIORITY 26
#define THREAD_TIMESLICE 20

/* Opus recording configuration */
#ifdef ENABLE_OPUS_ENCODER
#define OPUS_REC_SAMPLE_RATE    16000   // 16kHz
#define OPUS_REC_CHANNELS       1       // Mono
#define OPUS_REC_FRAME_SIZE     160     // 10ms at 16kHz (160 samples)
#define OPUS_REC_MAX_PACKET     320     // Max opus packet size
#define OPUS_REC_BITRATE        16000   // 16kbps
#endif

/* Voice recognition event flags */
#define VOICE_RECOGNITION_START (1 << 0)
#define VOICE_RECOGNITION_AUTO_STOP (1 << 1)
#define VOICE_RECOGNITION_STOP (1 << 2)
#define VOICE_RECOGNITION_ALL_EVENT                                            \
    (VOICE_RECOGNITION_START | VOICE_RECOGNITION_AUTO_STOP |                   \
     VOICE_RECOGNITION_STOP)

/* Global variables */
VoiceProvider voice_provider;

/* ── Voice-pipeline heap guard (see bloc_v2t.h doc) ───────────────────────
   Statically-initialised recursive mutex: usable from the first caller with
   no init-order dependency on the INIT_APP_EXPORT sequence. */
static struct rt_mutex s_voice_pl_mutex;
static bool s_voice_pl_mutex_inited = false;

void voice_pipeline_lock(void)
{
    if (!s_voice_pl_mutex_inited)
    {
        rt_base_t level = rt_hw_interrupt_disable();
        if (!s_voice_pl_mutex_inited)
        {
            rt_mutex_init(&s_voice_pl_mutex, "voice_pl", RT_IPC_FLAG_PRIO);
            s_voice_pl_mutex_inited = true;
        }
        rt_hw_interrupt_enable(level);
    }
    rt_mutex_take(&s_voice_pl_mutex, RT_WAITING_FOREVER);
}

void voice_pipeline_unlock(void)
{
    rt_mutex_release(&s_voice_pl_mutex);
}

/* Static variables */
static bool _vad_status = false;
static int rec_fd = -1;
static uint32_t record_bytes;

/* Set true by the audio thread (audio_record_*) when a write to the recording
   file fails — almost always a full disk. The audio thread only raises the
   flag and stops feeding; the UI thread (recorder's timer) owns the teardown
   so the file/encoder are released exactly once. Reset at each start. */
static volatile bool rec_disk_full = false;

/* Minimum free space required to start a recording. ~128 KB is roughly 64 s
   of 16 kbps opus, leaving headroom for logs and other writers so a fresh
   recording doesn't die after a couple of seconds. */
#define REC_MIN_FREE_BYTES (128 * 1024)

/* ── Shared Opus encoder (recording ⇄ V2T streaming) ──────────────────────
   Recording and V2T streaming never encode at the same time: the audio thread
   picks ONE path per frame (V2T-priority else-if; see audio_code_i2s.c), and
   each path refuses to (re)start while already active. So the two paths share
   a SINGLE encoder instance — reconfigured for whichever path owns it — instead
   of two simultaneous encoders. opus_encoder_create() sizes the allocation
   exactly; an owner guard keeps the two paths from clobbering each other, and
   release() destroys it (the old V2T encoder was created lazily and never
   destroyed — a permanent ~10-12 KB leak that this removes). */
#ifdef ENABLE_OPUS_ENCODER
typedef enum
{
    OPUS_OWNER_NONE = 0,
    OPUS_OWNER_REC,  // file recording  (10 ms frames, no FEC)
    OPUS_OWNER_V2T,  // BLE streaming   (20 ms frames, in-band FEC)
} opus_owner_t;

static OpusEncoder *shared_opus_encoder = NULL;  // non-NULL only while owned
static opus_owner_t opus_enc_owner = OPUS_OWNER_NONE;

/* ── Opus heap backend → PSRAM ─────────────────────────────────────────────
   opus.c defines opus_heap_malloc/opus_heap_free as RT_WEAK (rt_malloc, i.e. the
   SRAM system heap). This STRONG override wins at link time and routes opus's
   persistent allocations — chiefly the ~24 KB encoder state — to the PSRAM image
   pool instead. The SRAM heap runs ~99 % full during voice, so once the 1st
   session fragments it the 2nd session's 24 KB *contiguous* alloc failed
   ("opus_encoder_create failed err=-7") and that voice turn produced no audio; the
   PSRAM pool has >1.8 MB free. The encoder state is CPU-only (no DMA), so cached
   PSRAM is safe, and opus's per-frame scratch uses alloca (USE_ALLOCA), not this
   backend — the hot encode path stays on-stack.

   Alignment: app_cache_alloc() returns (heap base + 4) → only 4-byte aligned, but
   the OpusEncoder state needs 8-byte alignment (armclang emits LDRD/STRD against
   it; a 4-byte base HardFaults — that was the "voice keeps crashing/rebooting").
   So we over-allocate, align the result up to 8, and stash the real app_cache
   pointer in the word just below the aligned address for opus_heap_free(). */
#define OPUS_PSRAM_ALIGN 8u
void *opus_heap_malloc(uint32_t size)
{
    uint8_t *base = (uint8_t *)app_cache_alloc(size + OPUS_PSRAM_ALIGN + sizeof(void *),
                                               IMAGE_CACHE_PSRAM);
    if (!base)
    {
        return NULL;
    }
    /* leave room (>= sizeof(void*)) below the aligned address for the back-pointer */
    uintptr_t aligned = ((uintptr_t)base + sizeof(void *) + (OPUS_PSRAM_ALIGN - 1))
                        & ~(uintptr_t)(OPUS_PSRAM_ALIGN - 1);
    ((void **)aligned)[-1] = base;
    return (void *)aligned;
}

void opus_heap_free(void *p)
{
    if (p)
    {
        app_cache_free(((void **)p)[-1]);
    }
}

#endif /* ENABLE_OPUS_ENCODER (heap backend below is opus-independent) */

/* ── WebRTC (NS/AGC/VAD/AECM) heap backend → dedicated PSRAM memheap ──────
   webrtc_mem.h maps the whole WebRTC package's malloc/free onto these. The old
   mapping went to audio_mem_malloc = bare rt_malloc + RT_ASSERT on the SRAM
   heap, which is ~99 % full during voice — and the SiFli port's AUDIO_MEM_ALLOC
   mode makes NSX allocate ~a dozen scratch buffers PER 10 ms FRAME, so mic
   start/processing died with "audio_mem_malloc:163 (ptr)" whenever the heap
   couldn't serve one more block. Route it all to PSRAM instead (>1.8 MB free).

   app_cache_alloc/free CANNOT back per-frame churn directly: app_cache_free is
   deferred (64-slot queue drained only when the GPU goes idle) and overflows
   → leak. So we grab ONE long-lived 128 KB block from the PSRAM image pool at
   first use and run a private rt_memheap (thread-safe, frees immediately)
   inside it. Every block gets an 8-byte header below an 8-aligned user
   pointer: [-1]=raw base, [-2]=size<<1|origin (origin: 1=pool, 0=rt_malloc
   fallback when the pool is full — degrades to the old behaviour instead of
   failing). CPU-only DSP state, no DMA → cached PSRAM is safe (opus/framebuf
   precedent). */
#define WEBRTC_PSRAM_POOL_SIZE (128 * 1024)
#define WEBRTC_HEAP_ALIGN 8u
static struct rt_memheap s_webrtc_memheap;
static volatile bool s_webrtc_heap_ready = false;
static volatile bool s_webrtc_heap_initing = false;

static void webrtc_heap_ensure(void)
{
    if (s_webrtc_heap_ready)
    {
        return;
    }
    bool doit = false;
    rt_base_t lv = rt_hw_interrupt_disable();
    if (!s_webrtc_heap_ready && !s_webrtc_heap_initing)
    {
        s_webrtc_heap_initing = true;
        doit = true;
    }
    rt_hw_interrupt_enable(lv);
    if (doit)
    {
        void *mem = app_cache_alloc(WEBRTC_PSRAM_POOL_SIZE, IMAGE_CACHE_PSRAM);
        if (mem)
        {
            rt_memheap_init(&s_webrtc_memheap, "webrtc_ps", mem,
                            WEBRTC_PSRAM_POOL_SIZE);
            s_webrtc_heap_ready = true;
        }
        else
        {
            LOG_E("webrtc PSRAM pool alloc failed, falling back to SRAM");
            s_webrtc_heap_initing = false; /* retry on next call */
        }
    }
    else
    {
        /* Another thread is initialising — brief spin, then either use the
           pool or fall back to rt_malloc for this one allocation. */
        for (int i = 0; i < 10 && s_webrtc_heap_initing && !s_webrtc_heap_ready; i++)
        {
            rt_thread_mdelay(1);
        }
    }
}

void *webrtc_heap_malloc(uint32_t size)
{
    webrtc_heap_ensure();
    uint32_t origin = 1;
    uint8_t *raw = NULL;
    if (s_webrtc_heap_ready)
    {
        raw = (uint8_t *)rt_memheap_alloc(&s_webrtc_memheap,
                                          size + WEBRTC_HEAP_ALIGN + 8);
    }
    if (!raw)
    {
        /* Pool missing/full → SRAM system heap, i.e. the old behaviour. */
        origin = 0;
        raw = (uint8_t *)rt_malloc(size + WEBRTC_HEAP_ALIGN + 8);
    }
    if (!raw)
    {
        LOG_E("webrtc_heap_malloc %u failed", (unsigned)size);
        return NULL;
    }
    uintptr_t aligned = ((uintptr_t)raw + 8 + (WEBRTC_HEAP_ALIGN - 1))
                        & ~(uintptr_t)(WEBRTC_HEAP_ALIGN - 1);
    ((void **)aligned)[-1] = raw;
    ((uint32_t *)aligned)[-2] = (size << 1) | origin;
    return (void *)aligned;
}

void webrtc_heap_free(void *p)
{
    if (!p)
    {
        return;
    }
    void *raw = ((void **)p)[-1];
    if (((uint32_t *)p)[-2] & 1)
    {
        rt_memheap_free(raw);
    }
    else
    {
        rt_free(raw);
    }
}

void *webrtc_heap_calloc(uint32_t count, uint32_t size)
{
    uint32_t total = count * size;
    void *p = webrtc_heap_malloc(total);
    if (p)
    {
        memset(p, 0, total);
    }
    return p;
}

void *webrtc_heap_realloc(void *p, uint32_t newsize)
{
    if (!p)
    {
        return webrtc_heap_malloc(newsize);
    }
    if (newsize == 0)
    {
        webrtc_heap_free(p);
        return NULL;
    }
    uint32_t oldsize = ((uint32_t *)p)[-2] >> 1;
    void *np = webrtc_heap_malloc(newsize);
    if (np)
    {
        memcpy(np, p, oldsize < newsize ? oldsize : newsize);
        webrtc_heap_free(p);
    }
    return np;
}

#ifdef ENABLE_OPUS_ENCODER
static uint8_t rec_opus_output[OPUS_REC_MAX_PACKET] __attribute__((aligned(4)));
static int16_t rec_pcm_buffer[OPUS_REC_FRAME_SIZE];
static uint16_t rec_pcm_buffer_idx = 0;

/* Acquire the shared encoder for `owner`, configured for that path's profile.
   Returns the encoder, or NULL if the OTHER path already owns it (caller must
   then skip encoding) or creation failed. Re-acquiring as the current owner
   just returns the live encoder. */
static OpusEncoder *opus_enc_acquire(opus_owner_t owner)
{
    voice_pipeline_lock();
    if (opus_enc_owner != OPUS_OWNER_NONE && opus_enc_owner != owner)
    {
        LOG_W("opus encoder busy (owner=%d), request %d denied\n",
              opus_enc_owner, owner);
        voice_pipeline_unlock();
        return NULL;
    }
    if (shared_opus_encoder != NULL && opus_enc_owner == owner)
    {
        voice_pipeline_unlock();
        return shared_opus_encoder;
    }

    int err;
    OpusEncoder *enc = opus_encoder_create(OPUS_REC_SAMPLE_RATE, OPUS_REC_CHANNELS,
                                           OPUS_APPLICATION_VOIP, &err);
    if (err != OPUS_OK || enc == NULL)
    {
        LOG_E("opus_encoder_create failed err=%d\n", err);
        voice_pipeline_unlock();
        return NULL;
    }

    // Shared profile (identical in both original encoders).
    opus_encoder_ctl(enc, OPUS_SET_VBR(1));
    opus_encoder_ctl(enc, OPUS_SET_BITRATE(OPUS_REC_BITRATE));
    opus_encoder_ctl(enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(0));
    opus_encoder_ctl(enc, OPUS_SET_LSB_DEPTH(16));
    opus_encoder_ctl(enc, OPUS_SET_MAX_BANDWIDTH(OPUS_BANDWIDTH_WIDEBAND));

    if (owner == OPUS_OWNER_REC)
    {
        // File recording: 10 ms frames, constrained VBR, no FEC/DTX.
        opus_encoder_ctl(enc, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
        opus_encoder_ctl(enc, OPUS_SET_VBR_CONSTRAINT(1));
        opus_encoder_ctl(enc, OPUS_SET_DTX(0));
        opus_encoder_ctl(enc, OPUS_SET_INBAND_FEC(0));
        opus_encoder_ctl(enc, OPUS_SET_PACKET_LOSS_PERC(0));
        opus_encoder_ctl(enc, OPUS_SET_PREDICTION_DISABLED(0));
        opus_encoder_ctl(enc, OPUS_SET_BANDWIDTH(OPUS_AUTO));
    }
    else // OPUS_OWNER_V2T
    {
        // BLE streaming: 20 ms frames + in-band FEC so a silent drop is
        // recoverable phone-side (opus_stream_decoder.dart drives FEC/PLC).
        opus_encoder_ctl(enc, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_20_MS));
        opus_encoder_ctl(enc, OPUS_SET_INBAND_FEC(1));
        opus_encoder_ctl(enc, OPUS_SET_PACKET_LOSS_PERC(10));
    }

    shared_opus_encoder = enc;
    opus_enc_owner = owner;
    LOG_D("opus encoder acquired by owner=%d\n", owner);
    voice_pipeline_unlock();
    return enc;
}

/* Release + destroy the shared encoder if `owner` holds it. Idempotent AND
   thread-safe: GUI direct-stop and the voice_recognition AUTO_STOP event can
   race here — swap-then-free under the pipeline lock so the loser sees NULL
   instead of destroying the same encoder twice. */
static void opus_enc_release(opus_owner_t owner)
{
    voice_pipeline_lock();
    if (opus_enc_owner == owner && shared_opus_encoder != NULL)
    {
        OpusEncoder *enc = shared_opus_encoder;
        shared_opus_encoder = NULL;
        opus_enc_owner = OPUS_OWNER_NONE;
        opus_encoder_destroy(enc);
        LOG_D("opus encoder released by owner=%d\n", owner);
    }
    voice_pipeline_unlock();
}
#endif
static uint32_t _voice_recording_time = 0;
static char current_recording_file[MAX_RECORD_PATH_LEN] = {0};
static voice2text_t _v2t_result = {0};
static voice2text_t _v2t_result_temp = {0};
static char
    combined_text[sizeof(_v2t_result.text) + sizeof(_v2t_result_temp.text) + 1];
static struct rt_event voice_recog_event;
static bool voice2TextStatus = false;
static uint8_t voice2TextIntent = V2T_INTENT_NOTHING;

/* One-shot override for the next VOICE_RECOGNITION_START handler. See
   bloc_v2t.h doc. Default V2T_INTENT_CHAT matches the historical
   hardcoded value, so existing call sites that don't set this behave
   unchanged. */
static uint8_t pending_v2t_start_intent = V2T_INTENT_CHAT;

void voice_set_pending_v2t_intent(uint8_t intent)
{
    pending_v2t_start_intent = intent;
}

/* ========== 語音辨識自動停止邏輯區塊 ========== */
static uint32_t last_vad_trigger_tick = 0;
/* Debounce timer for checking if user is speaking */
static bool is_user_speaking = false;
static bool is_user_speaking_to_ai = false;
static rt_timer_t speaking_debounce_timer = NULL;
static bool is_voice_recognition_notified = false;
static bool is_voice_recognition_notified_from_mouse = false;

static bool voiceRecordIntent = false;
static bool isVoiceRecording = false;
void set_voice_recognition_notified_from_mouse(bool status)
{
    is_voice_recognition_notified_from_mouse = status;
}

/*************** Voice ***************/
bool app_voice_get_listening_status(void)
{
    return skaiwalk_provider.voice_activated;
}
void app_voice_set_listening_status(bool status)
{
    skaiwalk_provider.voice_activated = status;
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_MIC;
    msg.data.mic_state = status;
    lvgl_send_msg(msg);
}

/* ========== 語音辨識自動停止邏輯核心函數 ========== */
static void notify_user_speaking_intent(int status)
{
    commu_send_user_speaking_state((uint8_t)status);
}
/**
 * @brief 自動停止計時器回調函數
 *
 * 當計時器超時時觸發，執行自動停止語音辨識
 */
static void speaking_debounce_timer_callback(void *parameter)
{
    LOG_D("time out");
    is_user_speaking = false;
    // notify_user_speaking_intent(0);
    voice_provider.auto_stop_listening();
}

bool check_if_user_speaking_to_ai(void)
{
    return is_user_speaking_to_ai;
}

void reset_user_speaking_to_ai(void)
{
    is_user_speaking_to_ai = false;
}

/**
 * @brief 啟動自動停止計時器
 *
 * 創建並啟動計時器，預設5秒後自動停止語音辨識
 * 如果已有語音辨識結果，計時器會調整為1秒
 */
static void speaking_debounce_timer_start(uint16_t seconds)
{
    if (!speaking_debounce_timer)
    {
        speaking_debounce_timer = rt_timer_create(
            "speaking_debounce_timer", speaking_debounce_timer_callback, NULL,
            seconds, RT_TIMER_FLAG_ONE_SHOT);
        if (!speaking_debounce_timer)
        {
            LOG_E("debounce timer create failed (no mem)");
            return;
        }
    }
    else
    {
        /* Timer object now persists across sessions (see _stop) — re-apply
           the requested period, which creation used to provide. */
        rt_uint32_t t = seconds;
        rt_timer_stop(speaking_debounce_timer);
        rt_timer_control(speaking_debounce_timer, RT_TIMER_CTRL_SET_TIME, &t);
    }
    rt_timer_start(speaking_debounce_timer);
    is_user_speaking = true;
}

/**
 * @brief 停止自動停止計時器
 *
 * 停止計時器並重置運行狀態
 */
static void speaking_debounce_timer_stop(void)
{
    /* Keep the timer object alive across sessions: the old code NULLed the
       handle without rt_timer_delete (one leaked timer control block per
       session), and the audio thread's notify_vad_status could then hit
       rt_timer_stop(NULL) → RTOS assert. Stop only; create-once forever. */
    if (speaking_debounce_timer)
    {
        is_user_speaking = false;
        LOG_D("SPEAK DEGUG TEST3");
        rt_timer_stop(speaking_debounce_timer);
    }
}

/**
 * @brief VAD狀態通知處理
 *
 * 處理VAD檢測狀態變化，管理自動停止邏輯：
 * - 有聲音時：重新啟動計時器，更新VAD觸發時間
 * - 無聲音時：檢查自動停止條件，滿足則觸發自動停止
 *
 * 自動停止條件：
 * 1. 已收到語音辨識結果
 * 2. 距離上次輸入訊息更新超過5秒
 * 3. 距離上次VAD觸發超過0.5秒
 * 4. 不在AI處理中
 */
static void notify_vad_status(bool status)
{
    rt_tick_t current_tick = rt_tick_get();
    if (_vad_status != status)
    {
        _vad_status = status;
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_VAD_STATUS;
        msg.data.mic_state = status;
        lvgl_send_msg(msg);
        if (status)
        {
            /* The right device_pager skaibar is also a "speaking-to-AI" surface —
               without this, speaking there never sets is_user_speaking_to_ai, so
               interact_voice_recognition drops the transcript (none of its
               branches match the device page) and the text never reaches
               device_pager_skaibar_say. (Previously it only worked if the flag
               lingered from a prior left instruction_list session.) */
            extern bool device_pager_skaibar_is_open(void);
            /* R3: the shared floating instruction-list box is now the device-page
               voice surface too — opened IN PLACE over the mouse page, NOT on the
               LEFT instruction_list tile, so is_at_instruction_list() is false
               there. Recognise the box being open so the VAD gate still arms
               is_user_speaking_to_ai on the device/mouse page; otherwise the
               transcript is dropped here and only the phone-relayed copy reaches
               the Mac, never the watch's own floating box. */
            extern bool get_is_open_instruction_list_ai(void);
            if (get_gravity_position() == GRAVITY_POSITION_AI ||
                is_at_ai_interface() || is_at_instruction_list() ||
                is_at_speech_interface() || device_pager_skaibar_is_open() ||
                get_is_open_instruction_list_ai())
            {
                is_user_speaking_to_ai = true;
            }
        }
    }

    if (status)
    {
        if (is_at_mouse_mode())
        {
            if (is_voice_recognition_notified_from_mouse)
            {
                if (speaking_debounce_timer)
                {
                    rt_uint32_t time_left = 500;
                    LOG_D("SPEAK DEGUG TEST4");
                    rt_timer_stop(speaking_debounce_timer);
                    rt_timer_control(speaking_debounce_timer,
                                     RT_TIMER_CTRL_SET_TIME, &time_left);
                    rt_timer_start(speaking_debounce_timer);
                }
                is_voice_recognition_notified_from_mouse = false;
            }
            speaking_debounce_timer_start(500);
        }
        // LOG_D("status:%d,is_user_speaking:%d,check_if_ai_processing:%d",
        last_vad_trigger_tick = current_tick;
    }
}

/**
 * @brief 啟動語音辨識
 *
 * 啟動語音辨識流程並初始化相關狀態：
 * 1. 設置高性能模式
 * 2. 啟用音頻麥克風感測器
 * 3. 設置語音監聽和辨識狀態
 * 4. 重置自動停止相關標記
 *
 * @param intent 語音辨識意圖
 */

uint8_t speech_coding = 0;
void reset_speech_coding(void)
{
    speech_coding = 0;
}

uint8_t get_speech_coding(void)
{
    return speech_coding;
}

void count_speech_coding(void)
{
    speech_coding++;
}

void start_voice_recognition(uint8_t intent)
{
    if (voice2TextStatus)
    {
        LOG_W("Voice recognition is already active.");
        return;
    }
    /* 開始辨識的短震回饋(founder 2026-08-17)。**這裡**才是真正的共同咽喉點:啟動有兩條
       家族 —— voice_provider.start_v2t()(聊天室/session 列表/skai widget…)與直接呼叫
       start_voice_recognition()(滑鼠語音站 hid_mouse.c:1547、message、speech、錶盤長按),
       而前者最後也會走到這裡。先前放在 send_start_listen_event 只涵蓋前者,所以語音站的
       長按麥克風沒有震(founder:「長按麥克風或輸入框開啟語音辨識也沒有震動」)。
       放在 voice2TextStatus 的早退之後:重複啟動不會連震。 */
    extern void motor_pattern_unlocked(void);
    motor_pattern_unlocked();
    speech_coding = 0;
    notify_user_speaking_intent(intent);
    skaiwatch_ble_set_performance(BLE_PERF_FAST);
#ifdef BSP_USING_BLOC_PERIPHERAL
    peripheral_provider.subscribe_audio_mic_sensor(true);
#endif
    app_voice_set_listening_status(true);
    app_voice_set_voice2text_status(true);
    app_voice_set_voice2text_intent(intent);

    // 重置語音辨識狀態標記
    is_voice_recognition_notified = false;
    last_vad_trigger_tick = rt_tick_get();
}

/**
 * @brief 停止語音辨識
 *
 * 停止語音辨識流程並清理相關狀態：
 * 1. 停止自動停止計時器
 * 2. 關閉音頻麥克風感測器
 * 3. 重置語音監聽和辨識狀態
 * 4. 關閉高性能模式
 */
void stop_voice_recognition(uint8_t intent)
{
    uint8_t stop_intent = intent + 100;
    app_voice_set_listening_status(false);
#ifdef BSP_USING_BLOC_PERIPHERAL
    peripheral_provider.subscribe_audio_mic_sensor(false);
#endif
    speaking_debounce_timer_stop();
    notify_user_speaking_intent(stop_intent);
    app_voice_set_voice2text_status(false);
#ifdef ENABLE_OPUS_ENCODER
    // V2T streaming ended — release the shared encoder. This path used to
    // create it lazily and never destroy it; that ~10-12 KB leak is now gone.
    opus_enc_release(OPUS_OWNER_V2T);
#endif
    /* V2T ended — fall back from the 1 s UI-indicator cadence to the 10 s
       TPC cadence. Don't stop the checker entirely; it stays running for
       the whole connection lifetime and is only stopped on disconnect. */
    skaiwatch_ble_set_performance(BLE_PERF_SLOW);
    reset_ai_open_mic();
    // 重置語音辨識狀態標記
    is_voice_recognition_notified = false;
    _vad_status = false;
}

/* Free bytes available on the root filesystem (where /recorder lives). */
static uint64_t fs_free_bytes(void)
{
    struct statfs st;
    if (statfs("/", &st) != 0)
    {
        return 0;
    }
    return (uint64_t)st.f_bfree * st.f_bsize;
}

/* Delete the oldest recording in /recorder. Filenames are timestamped
   (record_YYYYMMDD_HHMMSS.ext) so the lexicographically smallest name is the
   oldest. Returns 0 if a file was deleted, -1 if none was found. */
static int delete_oldest_recording(void)
{
    DIR *dir = opendir("/recorder");
    if (dir == NULL)
    {
        return -1;
    }

    char oldest[MAX_RECORD_PATH_LEN] = {0};
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type != DT_REG)
            continue;
        if (strncmp(entry->d_name, "record", 6) != 0)
            continue;
        if (strstr(entry->d_name, ".opus") == NULL &&
            strstr(entry->d_name, ".pcm") == NULL)
            continue;
        if (oldest[0] == '\0' || strcmp(entry->d_name, oldest) < 0)
        {
            strncpy(oldest, entry->d_name, sizeof(oldest) - 1);
            oldest[sizeof(oldest) - 1] = '\0';
        }
    }
    closedir(dir);

    if (oldest[0] == '\0')
    {
        return -1;
    }

    char path[MAX_RECORD_PATH_LEN + 16];
    snprintf(path, sizeof(path), "/recorder/%s", oldest);
    if (unlink(path) != 0)
    {
        LOG_E("disk low: failed to delete %s", path);
        return -1;
    }
    LOG_W("disk low: rotated out oldest recording %s", path);
    return 0;
}

/* Ensure at least REC_MIN_FREE_BYTES is free before recording, rotating out
   the oldest recordings if necessary. Returns 0 if space is available, -1 if
   the disk is still full after deleting everything we can. */
static int ensure_recording_space(void)
{
    while (fs_free_bytes() < REC_MIN_FREE_BYTES)
    {
        if (delete_oldest_recording() != 0)
        {
            return -1; /* nothing left to free */
        }
    }
    return 0;
}

int start_voice_recording(void)
{
    if (app_voice_get_recording_status() == true)
    {
        return 0;
    }

    record_bytes = 0;
    rec_disk_full = false;

    if (access("/recorder", 0))
    {
        if (mkdir("/recorder", 0x777) != 0)
        {
            LOG_E("failed to create /recorder directory");
            return -1;
        }
        LOG_D("created /recorder directory");
    }

    /* Pre-flight: make room before we touch the mic / allocate the encoder. */
    if (ensure_recording_space() != 0)
    {
        LOG_E("storage full, cannot start recording");
        return -1;
    }

#ifdef BSP_USING_BLOC_PERIPHERAL
    peripheral_provider.subscribe_audio_mic_sensor(true);
#endif

    char file_path[MAX_RECORD_PATH_LEN];
    time_t now;
    struct tm *tm_info;

    time(&now);
    tm_info = localtime(&now);

#ifdef ENABLE_OPUS_ENCODER
    // Acquire the shared Opus encoder for recording (profile set inside
    // opus_enc_acquire). NULL ⇒ V2T owns it or create failed; recording then
    // falls back to raw PCM via audio_record_pcm().
    if (opus_enc_acquire(OPUS_OWNER_REC) == NULL)
    {
        LOG_E("Failed to acquire Opus encoder for recording\n");
    }
    else
    {
        rec_pcm_buffer_idx = 0;
        LOG_D("Opus encoder initialized for recording\n");
    }
    const char *rec_ext = ".opus";
#else
    const char *rec_ext = ".pcm";
#endif
    snprintf(file_path, sizeof(file_path),
             "/recorder/record_%04d%02d%02d_%02d%02d%02d%s",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec, rec_ext);

    LOG_D("[%s] %s\n", __FUNCTION__, file_path);
    rec_fd = open(file_path, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    if (rec_fd < 0)
    {
        LOG_E("failed to open recording file %s", file_path);
#ifdef ENABLE_OPUS_ENCODER
        opus_enc_release(OPUS_OWNER_REC);
#endif
#ifdef BSP_USING_BLOC_PERIPHERAL
        peripheral_provider.subscribe_audio_mic_sensor(false);
#endif
        return -1;
    }

    // Save current recording file path
    strncpy(current_recording_file, file_path, MAX_RECORD_PATH_LEN - 1);
    current_recording_file[MAX_RECORD_PATH_LEN - 1] = '\0';

    app_voice_set_listening_status(true);
    app_voice_set_recording_status(true);
    return 0;
}

static uint8_t audio_buffer[AUDIO_REC_TEMP_SIZE];

#ifdef ENABLE_OPUS_ENCODER
/**
 * @brief Encode PCM data with Opus and write to file
 *
 * This function accumulates PCM samples until we have enough for one Opus frame
 * (160 samples = 10ms at 16kHz), then encodes and writes to file.
 * File format: [2-byte length][opus packet data][2-byte length][opus packet data]...
 *
 * @param temp_buf Input PCM buffer (16-bit samples)
 * @param data_len Length of input data in bytes
 * @return 0 on success, -1 on failure
 */
static int audio_record_opus(const void *temp_buf, rt_uint32_t data_len)
{
    if (shared_opus_encoder == NULL)
    {
        LOG_E("Opus encoder not initialized\n");
        return -1;
    }

    const int16_t *pcm_input = (const int16_t *)temp_buf;
    uint32_t samples = data_len / 2; // Convert bytes to samples
    uint32_t samples_processed = 0;

    while (samples_processed < samples)
    {
        // Fill PCM buffer until we have OPUS_REC_FRAME_SIZE samples
        while (rec_pcm_buffer_idx < OPUS_REC_FRAME_SIZE &&
               samples_processed < samples)
        {
            rec_pcm_buffer[rec_pcm_buffer_idx++] = pcm_input[samples_processed++];
        }

        // When buffer is full, encode and write
        if (rec_pcm_buffer_idx >= OPUS_REC_FRAME_SIZE)
        {
            // Encode using opus_encode directly (same as opus example main.c)
            opus_int32 encoded_len = opus_encode(shared_opus_encoder,
                                                 rec_pcm_buffer,
                                                 OPUS_REC_FRAME_SIZE,
                                                 rec_opus_output,
                                                 OPUS_REC_MAX_PACKET);

            if (encoded_len > 0 && encoded_len <= OPUS_REC_MAX_PACKET)
            {
                // Write packet length (2 bytes, little-endian) followed by packet data
                uint16_t pkt_len = (uint16_t)encoded_len;
                if (write(rec_fd, &pkt_len, sizeof(pkt_len)) != sizeof(pkt_len))
                {
                    LOG_E("audio_record_opus write length err (disk full?)\n");
                    rec_disk_full = true;
                    return -1;
                }
                if (write(rec_fd, rec_opus_output, encoded_len) != encoded_len)
                {
                    LOG_E("audio_record_opus write data err (disk full?)\n");
                    rec_disk_full = true;
                    return -1;
                }
                record_bytes += sizeof(pkt_len) + encoded_len;
            }
            else
            {
                LOG_E("Opus encode failed, ret=%d\n", encoded_len);
            }

            // Reset buffer index for next frame
            rec_pcm_buffer_idx = 0;
        }
    }

    return 0;
}


// ── Real-time V2T Opus streaming (shares the single encoder; see top) ──
// Streaming config differs from recording: in-band FEC ON + packet-loss-perc,
// because the BLE audio characteristic is fire-and-forget (silent drops). 20ms
// frames halve the BLE notification rate vs the recording path's 10ms.
// NOTE: authored without an on-device build — needs Keil compile + review.
#define V2T_OPUS_FRAME_SIZE 320 // 20ms @ 16kHz
static int16_t v2t_pcm_buf[V2T_OPUS_FRAME_SIZE];
static uint16_t v2t_pcm_idx = 0;
static uint8_t v2t_opus_seq = 0;
// Wire frame: [0]=speaking index, [1]=8-bit seq (wraps), [2..]=opus packet.
static uint8_t v2t_send_buf[2 + OPUS_REC_MAX_PACKET] __attribute__((aligned(4)));

static int v2t_opus_lazy_init(void)
{
    // 20ms + FEC profile is applied inside opus_enc_acquire(); the encoder is
    // shared with recording and owner-guarded there. Reset the frame/seq
    // accumulators only on a fresh acquire (start of a V2T session), not on
    // every per-frame call, so frame assembly survives across calls.
    bool fresh = (opus_enc_owner != OPUS_OWNER_V2T);
    if (opus_enc_acquire(OPUS_OWNER_V2T) == NULL)
    {
        return -1;
    }
    if (fresh)
    {
        v2t_pcm_idx = 0;
        v2t_opus_seq = 0;
    }
    return 0;
}

// Encode PCM16 (mono 16kHz) into 20ms Opus frames and stream each as one BLE
// audio notification, framed [idx][seq][opus]. Mirrors audio_record_opus()'s
// accumulation but sends over BLE instead of writing to a file. Dropped packets
// are concealed phone-side via FEC/PLC keyed on the sequence number.
void v2t_opus_stream_send(const int16_t *pcm, uint32_t samples)
{
    if (v2t_opus_lazy_init() != 0)
    {
        return;
    }
    uint32_t i = 0;
    while (i < samples)
    {
        while (v2t_pcm_idx < V2T_OPUS_FRAME_SIZE && i < samples)
        {
            v2t_pcm_buf[v2t_pcm_idx++] = pcm[i++];
        }
        if (v2t_pcm_idx >= V2T_OPUS_FRAME_SIZE)
        {
            opus_int32 n = opus_encode(shared_opus_encoder, v2t_pcm_buf,
                                       V2T_OPUS_FRAME_SIZE,
                                       v2t_send_buf + 2, OPUS_REC_MAX_PACKET);
            v2t_pcm_idx = 0;
            if (n > 0 && n <= OPUS_REC_MAX_PACKET)
            {
                v2t_send_buf[0] = get_speech_coding(); // speaking-sentence index
                v2t_send_buf[1] = v2t_opus_seq++;      // 8-bit seq (wraps)
                /* 原本是 fire-and-forget(「phone conceals any drop via FEC/PLC」),
                   代價是 BLE 通知送不出去時**完全靜默** —— 錄音照跑、沒有任何錯誤、
                   手機收不到音訊所以永遠沒有文字回來(founder 2026-08-07 的症狀)。
                   同檔另一條 ADPCM 路徑早就有 drop 統計了,這裡補齊:每 50 幀印一次
                   送出/失敗數,不洗版但看得出來音訊到底有沒有出去。 */
                bool sent =
                    skaiwatch_ble_audio_send(v2t_send_buf, (uint16_t)(n + 2));
                if (!sent)
                {
                    /* 限流:BLE 壅塞時一次會連掉很多幀 */
                    static uint32_t s_v2t_tx_fail = 0;
                    s_v2t_tx_fail++;
                    if ((s_v2t_tx_fail & 0x1F) == 1)
                        LOG_W("v2t audio TX drop (count=%u)", s_v2t_tx_fail);
                }
            }
            else
            {
                LOG_E("v2t opus encode failed n=%d\n", n);
            }
        }
    }
}
#endif

int audio_record_pcm(const void *temp_buf, rt_uint32_t data_len)
{
    if (app_voice_get_recording_status() == false)
    {
        return -1;
    }
    if (rec_disk_full)
    {
        /* A prior write failed (disk full). Stop feeding; the recorder's UI
           timer will tear the session down and tell the user. */
        return -1;
    }

#ifdef ENABLE_OPUS_ENCODER
    // Use Opus encoding when recording owns the shared encoder.
    if (shared_opus_encoder != NULL && opus_enc_owner == OPUS_OWNER_REC)
    {
        return audio_record_opus(temp_buf, data_len);
    }
#endif

    // Fallback to raw PCM recording
    rt_uint32_t wrt_len = 0;
    memcpy(audio_buffer, temp_buf, data_len);
    uint8_t *buf_ptr = audio_buffer;

    /*write data into pcm file*/
    while (data_len)
    {
        wrt_len =
            data_len > AUDIO_REC_TEMP_SIZE ? AUDIO_REC_TEMP_SIZE : data_len;
        if (write(rec_fd, buf_ptr, wrt_len) != wrt_len)
        {
            LOG_E("audio_record_pcm write err (disk full?)\n");
            rec_disk_full = true;
            return -1;
        }
        data_len -= wrt_len;
        buf_ptr += wrt_len; // 增加指针以指向下一个要写入的数据块
        record_bytes += wrt_len;
    }

    return 0;
}

void stop_voice_recording(void)
{
    if (app_voice_get_recording_status() == false)
    {
        return;
    }
    app_voice_set_recording_status(false);
    app_voice_set_listening_status(false);
#ifdef BSP_USING_BLOC_PERIPHERAL
    peripheral_provider.subscribe_audio_mic_sensor(false);
#endif

    /* lvgl (prio 9) preempts audio_station (16): without the lock this flush
       could destroy the encoder while the audio thread is inside opus_encode
       on it, and close rec_fd under a concurrent write. */
    voice_pipeline_lock();
#ifdef ENABLE_OPUS_ENCODER
    // Flush remaining PCM samples if any (pad with zeros to complete last frame)
    if (shared_opus_encoder != NULL && rec_pcm_buffer_idx > 0)
    {
        // Pad remaining buffer with zeros
        while (rec_pcm_buffer_idx < OPUS_REC_FRAME_SIZE)
        {
            rec_pcm_buffer[rec_pcm_buffer_idx++] = 0;
        }

        // Encode and write final frame
        opus_int32 encoded_len = opus_encode(shared_opus_encoder,
                                             rec_pcm_buffer,
                                             OPUS_REC_FRAME_SIZE,
                                             rec_opus_output,
                                             OPUS_REC_MAX_PACKET);
        if (encoded_len > 0 && encoded_len <= OPUS_REC_MAX_PACKET)
        {
            uint16_t pkt_len = (uint16_t)encoded_len;
            write(rec_fd, &pkt_len, sizeof(pkt_len));
            write(rec_fd, rec_opus_output, encoded_len);
            record_bytes += sizeof(pkt_len) + encoded_len;
        }
    }

    // Release (destroy) the shared encoder.
    opus_enc_release(OPUS_OWNER_REC);
    rec_pcm_buffer_idx = 0;
#endif

    LOG_D("recorded %d bytes to file: %s", record_bytes, current_recording_file);
    close(rec_fd);
    rec_fd = -1;
    rec_disk_full = false;
    voice_pipeline_unlock();
}

const char* get_last_recording_file(void)
{
    return current_recording_file;
}

void start_sync_voice_recording(void)
{
    app_voice_set_recording_intent(true);
    _voice_recording_time = get_current_time();
    LOG_D("start_sync_voice_recording: %d", _voice_recording_time);
}

void stop_sync_voice_recording(void)
{
    app_voice_set_recording_intent(false);
    LOG_D("stop_sync_voice_recording: %d", _voice_recording_time);
    _voice_recording_time = 0;
}

char *get_combined_voice2text(void)
{
    snprintf(combined_text, sizeof(combined_text), "%s%s", _v2t_result.text,
             _v2t_result_temp.text);
    return combined_text;
}

static void notifyVoice2Text()
{
    _v2t_result.state = true;
    // app_speech_set_content((const uint8_t *)_v2t_result.text);
    if (isTextEmpty())
    {
        app_speech_set_content((const uint8_t *)"");
        return;
    }
    LOG_D("notifyVoice2Text: %s", _v2t_result.text);
    app_speech_set_content((const uint8_t *)get_combined_voice2text());
}

void setVoice2Text(char *text)
{
    if (text != NULL)
    {
        strncpy(_v2t_result.text, text, sizeof(_v2t_result.text) - 1);
        _v2t_result.text[sizeof(_v2t_result.text) - 1] = '\0';
        notifyVoice2Text();
    }
}

void clearVoice2Text(void)
{
    _v2t_result.text[0] = '\0';
    _v2t_result_temp.text[0] = '\0';
    notifyVoice2Text();
}

bool isTextEmpty(void)
{
    return _v2t_result.text[0] == '\0' && _v2t_result_temp.text[0] == '\0';
}

static void appendVoice2Text(char *buffer, uint16_t len)
{
    if (len == 0)
    {
        return;
    }
    if (isTextEmpty())
    {
        setVoice2Text(buffer);
        return;
    }
    if (strlen(_v2t_result.text) + len + 1 < sizeof(_v2t_result.text))
    {
        strcat(_v2t_result.text, " ");
        strcat(_v2t_result.text, buffer);
        notifyVoice2Text();
    }
    else
    {
        LOG_E("Voice2Text buffer overflow");
    }
}

static void insertVoiceText(char *buffer)
{
    if (buffer == NULL)
    {
        return;
    }
    strncpy(_v2t_result_temp.text, buffer, sizeof(_v2t_result_temp.text) - 1);
    _v2t_result_temp.text[sizeof(_v2t_result_temp.text) - 1] = '\0';
    notifyVoice2Text();
}

void handle_v2t_result(VOICE_RECOGNITION_PAYLOAD *msgData)
{
    // if (app_voice_get_voice2text_status() == false)
    // {
    // 	LOG_D("Voice to text is not enabled, ignoring result");
    // 	return;
    // }
    static uint8_t last_sentence_index = 0;
    uint8_t sentence_index = msgData->header;
    if (msgData->p_msg_value == NULL || msgData->length == 0)
    {
        return;
    }
    // copy buffer based on length
    char *buffer = (char *)malloc(msgData->length + 1);
    if (buffer == NULL)
    {
        LOG_E("Failed to allocate buffer");
        return;
    }
    memcpy(buffer, msgData->p_msg_value, msgData->length);
    buffer[msgData->length] = '\0';
    if (sentence_index != last_sentence_index)
    {
        // LOG_D("[handle_v2t_result]New sentence: %d, insert len:%d",
        // sentence_index, msgData->length);
        appendVoice2Text(_v2t_result_temp.text, strlen(_v2t_result_temp.text));
        insertVoiceText(buffer);
        last_sentence_index = sentence_index;
    }
    else
    {
        // LOG_D("[handle_v2t_result]Append text len(%d) to sentence(%d)",
        // msgData->length, sentence_index);
        insertVoiceText(buffer);
    }

    free(buffer);
}

static void send_voice_recognition_event(uint8_t event)
{
    if (VOICE_RECOGNITION_ALL_EVENT & event)
    {
        rt_event_send(&voice_recog_event, event);
    }
    else
    {
        LOG_E("Invalid voice recognition event: %d", event);
    }
    LOG_D("send_voice_recognition_event: %d", event);
}

static void send_start_listen_event(void)
{
    /* 震動不放這裡 —— 見 start_voice_recognition():那是兩條啟動家族的共同匯流點,放這裡
       只涵蓋 start_v2t 這一條。 */
    if (!voice_provider.audio_subscribed)
    {
        send_voice_recognition_event(VOICE_RECOGNITION_START);
    }
}

static void send_auto_stop_listening_event(void)
{
    LOG_D("send_auto_stop_listening_event");
    send_voice_recognition_event(VOICE_RECOGNITION_AUTO_STOP);
}

static void send_stop_listening_event(void)
{
    LOG_D("send_stop_listening_event");
    send_voice_recognition_event(VOICE_RECOGNITION_STOP);
}

/**
 * @brief Initializes the Voice Activity Detection (VAD) module
 *
 * Creates and initializes the VAD instance if it doesn't exist
 */
static void vad_init(void)
{
    voice_pipeline_lock();
    if (voice_provider.vad_inst == NULL)
    {
        void *inst = WebRtcVad_Create();
        if (inst != NULL)
        {
            WebRtcVad_Init(inst);
            voice_provider.vad_inst = inst;
        }
        else
        {
            /* Graceful refusal: the mic-send gate is `vad_inst && status`, so
               a failed create just means no audio streams — no assert. */
            LOG_E("vad create failed (no mem), voice input unavailable");
        }
    }
    voice_pipeline_unlock();
}

/**
 * @brief Deinitializes the Voice Activity Detection (VAD) module
 *
 * Frees resources allocated for VAD instance
 */
static void vad_deinit(void)
{
    /* Swap-then-free under the pipeline lock. Callers race from three
       threads (voice_recognition AUTO_STOP, lvgl direct stops, communicate
       task interact paths); the unlocked check-then-free here was a real
       double-free of vad_inst (rt_free MEM_USED assert, voice_re thread). */
    voice_pipeline_lock();
    void *inst = voice_provider.vad_inst;
    voice_provider.vad_inst = NULL;
    if (inst)
    {
        WebRtcVad_Free(inst);
    }
    voice_pipeline_unlock();
}

/* ========== 語音辨識狀態管理函數 ========== */

/* ========== voice/recording state accessors ========== */
bool app_voice_get_voice2text_status(void)      { return voice2TextStatus; }
void app_voice_set_voice2text_status(bool s)    { voice2TextStatus = s; }
void app_voice_set_voice2text_intent(uint8_t i) { voice2TextIntent = i; }
void app_voice_set_recording_intent(bool i)     { voiceRecordIntent = i; }
bool app_voice_get_recording_status(void)       { return isVoiceRecording; }
void app_voice_set_recording_status(bool s)     { isVoiceRecording = s; }
bool app_voice_recording_disk_full(void)        { return rec_disk_full; }

static bool voice_recognition_started = false;
void set_voice_recognition_started(bool started)
{
    voice_recognition_started = started;
    LOG_I("set_voice_recognition_started: %d", started);
}
bool get_voice_recognition_started(void)
{
    return voice_recognition_started;
}
/**
 * @brief Main voice recognition thread entry function
 *
 * Waits for events and processes voice recognition requests
 *
 * @param parameter Thread parameter (unused)
 */
extern void refresh_ai_chat_input_message(char *text);
extern bool get_selected_function(void);
void voice_recognition_entry(void *parameter)
{
    rt_uint32_t evt = 0;
    LOG_D("voice_recognition_entry initialized");

    rt_event_init(&voice_recog_event, "voice_recognition", RT_IPC_FLAG_FIFO);

    while (1)
    {
        // Wait for voice recognition events
        if (rt_event_recv(&voice_recog_event, VOICE_RECOGNITION_ALL_EVENT,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &evt) == RT_EOK)
        {
            if (evt & VOICE_RECOGNITION_START)
            {
                if (!get_bluetooth_connection_status())
                {
                    create_connection_tips();
                    LOG_D("Bluetooth is connected, ignoring voice recognition "
                          "event");
                    continue;
                }
                if (check_if_ai_processing() && !isTextEmpty())
                {
                    LOG_D("AI processing is active, ignoring start event:%d,%d",
                          check_if_ai_processing(), isTextEmpty());
                    continue;
                }

                if (voice_recognition_started)
                {
                    LOG_D("Voice recognition already started, ignoring start "
                          "event");
                    continue;
                }
                set_voice_recognition_started(true);

                LOG_I("Received start listening event, started: %d",
                      voice_recognition_started);
                clearVoice2Text();
                set_last_refresh_input_message_tick(rt_tick_get());
                if (is_at_ai_interface())
                {
                    show_speech_indicator(true);
                }
                // Register message handlers
                extern void refresh_ai_reply_message(char *new_text);
                extern void is_on_speech_input(bool is_speech);
                refresh_ai_chat_input_message("");
                lvgl_msg_handler.handle_input_message =
                    refresh_ai_chat_input_message;
                lvgl_msg_handler.refresh_message_stream =
                    refresh_ai_reply_message;
                lvgl_msg_handler.handle_vad_status = is_on_speech_input;

                vad_init();
                /* Use whatever pending_v2t_start_intent the caller set
                   before voice_provider.start_v2t() (default CHAT).
                   Reset to CHAT immediately so the next callsite that
                   didn't override gets the historical default. */
                uint8_t this_start_intent = pending_v2t_start_intent;
                pending_v2t_start_intent = V2T_INTENT_CHAT;
                start_voice_recognition(this_start_intent);

                is_voice_recognition_notified = false;
                last_vad_trigger_tick = rt_tick_get();

                set_ai_processing(false);
            }
            else if (evt & VOICE_RECOGNITION_AUTO_STOP)
            {
                if (is_at_ai_interface())
                {
                    if (isTextEmpty())
                    {
                        LOG_D("isTextEmpty, hide indicator1");
                        show_speech_indicator(false);
                    }
                    else
                    {
                        hidden_speech_indicator();
                        set_ai_processing(true);
                    }
                }
                else if (is_at_mouse_mode())
                {
                    interact_mic_listen(false);
                }
                else if (is_at_speech_interface() && get_selected_function() &&
                         !isTextEmpty())
                {
                    hidden_speech_indicator();
                    set_ai_processing(true);
                }
                // notify_user_speaking_intent(100);
                handle_user_speech_intent(V2T_INTENT_CHAT,
                                          get_combined_voice2text());
                stop_voice_recognition(V2T_INTENT_CHAT);
                vad_deinit();
                set_voice_recognition_started(false);
            }
            else if (evt & VOICE_RECOGNITION_STOP)
            {
                if (check_if_ai_processing())
                {
                    LOG_D("AI processing is active, ignoring stop event");
                    continue;
                }
                if (isTextEmpty())
                {
                    LOG_D("isTextEmpty, hide indicator2");
                    show_speech_indicator(false);
                }
                set_ai_processing(false);
                stop_voice_recognition(V2T_INTENT_NOTHING);
                vad_deinit();
                set_voice_recognition_started(false);
            }
            else
            {
                LOG_E("Unknown event: %d", evt);
            }
            LOG_D("Voice recognition event processed: %d", evt);
        }
    }
}

/**
 * @brief Initializes and registers the voice provider functions
 *
 * @return 0 on success
 */
static int bloc_voice_provider_register(void)
{
    voice_provider.vad_inst = NULL;
    voice_provider.vad_init = vad_init;
    voice_provider.vad_deinit = vad_deinit;
    voice_provider.notify_vad_status = notify_vad_status;
    voice_provider.start_v2t = send_start_listen_event;
    voice_provider.auto_stop_listening = send_auto_stop_listening_event;
    voice_provider.stop_v2t = send_stop_listening_event;

    return 0;
}

void ai_tap_cb(void)
{
    if (voice_recognition_started)
    {
        if (isTextEmpty())
        {
            return;
        }
        voice_provider.auto_stop_listening();
    }
    else
        voice_provider.start_v2t();
}

static uint8_t ai_coding = 0;
void reset_ai_coding(void)
{
    ai_coding = 0;
}
uint8_t get_ai_coding(void)
{
    return ai_coding;
}

void back_tap_cb(void)
{
    if (isTextEmpty())
    {
        voice_provider.stop_v2t();
        show_speech_indicator(false);
    }
    else
    {
        /* Text present -> BACK clears the input box. This is the 1st press of
           the two-step back (clear + keep the widget open; a later back with the
           box now empty closes it). The clear must NOT be gated on voice still
           running: once VAD auto-stops, the old code fell into the branch that
           only hid the speech indicator and left the transcript on screen, so
           the first back appeared to do nothing (the regressed "back clears the
           AI widget" behaviour). */
        count_speech_coding();
        clearVoice2Text();
        refresh_ai_chat_input_message("");
        if (!get_voice_recognition_started())
        {
            show_speech_indicator(false);
        }
    }
}

/**
 * @brief Initializes the voice recognition task
 *
 * Creates and starts the voice recognition thread
 *
 * @return 0 on success
 */
static int voice_recognition_task_init(void)
{
    rt_thread_t voice_recognition_thread =
        rt_thread_create("voice_recognition", voice_recognition_entry, RT_NULL,
                         THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);

    if (voice_recognition_thread == RT_NULL)
    {
        LOG_E("Failed to create voice recognition thread");
        return -1;
    }

    rt_thread_startup(voice_recognition_thread);
    return 0;
}

/* Register initialization functions with RT-Thread */
INIT_APP_EXPORT(bloc_voice_provider_register);
INIT_APP_EXPORT(voice_recognition_task_init);
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
