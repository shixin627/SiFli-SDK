/**
 ******************************************************************************
 * @file   audio_code_i2s.c
 * @author Sifli software development team
 ******************************************************************************
 */
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated
 * circuit in a product or a software update for such product, must reproduce
 * the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be
 * reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <rtdevice.h>
#include <drivers/audio.h>
#include "audio_codec_i2s.h"
#include "app_mainmenu.h"
#include "bf0_hal.h"
#include "bsp_board.h"
#ifdef PA_USING_AW8155
    #include "sifli_aw8155.h"
#endif
#include "communicate_protocol.h"
#include "bf0_ble_gap.h"
#ifdef BSP_USING_BLOC
    #include "bloc_peripheral.h"
    #include "bloc_v2t.h"
#endif

#include "audio_server.h"

#include "webrtc/common_audio/vad/include/webrtc_vad.h"

/* ===== Optional WebRTC NS + AGC on the mic capture path (ASR pre-clean) =====
   Inserts noise-suppression then auto-gain-control on each 10 ms mic frame,
   right where the blunt gain_pcm_sat() shift sits. Default OFF: the existing
   path (gain_pcm_sat shift 6) is unchanged unless explicitly enabled at runtime
   via `mic_nsagc on`, so a buggy NS/AGC stage can never silently regress audio.

   Build wiring: the WebRTC package (external/WebRTC_audio_processing) is already
   compiled+linked into this build (CONFIG_PKG_USING_WEBRTC=y). Its Kconfig
   defaults already select fixed-point NS (WEBRTC_ANS_FIX) and AGC
   (WEBRTC_AGC_FIX), and the SConscript already puts the NS/AGC include dirs on
   the global CPPPATH -- so we just call the APIs here, no SConscript / proj.conf
   / external/ change needed. We use the *fixed-point* NS (WebRtcNsx_*) because
   that is the variant the project already builds; switching to the float NS
   (WebRtcNs_*) would require flipping the Kconfig NS choice. Both expose the
   same 0/1/2 policy (Mild/Medium/Aggressive). */
#define BSP_USING_MIC_NS_AGC  /* compile the NS+AGC stage in (runtime-gated below) */

#ifdef BSP_USING_MIC_NS_AGC
    #if defined(WEBRTC_ANS_FIX)
        #include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"
    #elif defined(WEBRTC_ANS_FLOAT)
        #include "webrtc/modules/audio_processing/ns/include/noise_suppression.h"
    #endif
    #ifdef WEBRTC_AGC_FIX
        #include "webrtc/modules/audio_processing/agc/legacy/gain_control.h"
    #endif
#endif

#define AUDIO_TRS_I2S_RX_EVENT (1 << 0)
#define AUDIO_TRS_I2S_TX_EVENT (1 << 1)
#define AUDIO_TRS_AUDPRC_ADC_EVENT (1 << 2)
#define AUDIO_TRS_AUDPRC_DAC_EVENT (1 << 3)
#define AUDIO_TRS_ALL_EVENT                                                    \
    (AUDIO_TRS_I2S_RX_EVENT | AUDIO_TRS_I2S_TX_EVENT |                         \
     AUDIO_TRS_AUDPRC_ADC_EVENT | AUDIO_TRS_AUDPRC_DAC_EVENT)

#define AUDIO_STATION_SAMPLE_RATE_OPT AUDIO_SAMPLE_RATE_16K

#define RINGBUFFER_SIZE 320 * 6      // 320 * 6 = 1920
static uint8_t compressed_data[500]; //(RINGBUFFER_SIZE / 4) + 1
// TEMP DIAGNOSTIC (Phase 1 voice-accuracy): count dropped BLE audio sends.
static uint32_t audio_tx_drop_count = 0;
#define READ_AUDIO_BUF_SIZE 320

const rt_uint32_t audio_sample_rate[] = {8000, 16000};

static struct rt_event aud_event;
#ifdef BSP_USING_I2S
    #define I2S_DEVICE "i2s2"
static rt_device_t g_i2s2_dev = RT_NULL;
static rt_uint8_t i2s_rx_buf[320 * 3];
static rt_uint8_t i2s_rx_temp[320];
static rt_uint8_t i2s_tx_buf[3][320];
static struct rt_ringbuffer i2s_rx_ringbuffer;
#endif

#define AUD_CODEC_DEVICE "audcodec"
#define AUD_PRC_DEVICE "audprc"
static rt_device_t g_audprc_dev = RT_NULL;
static rt_device_t g_codec_dev = RT_NULL;

#if PERIPHERAL_AUD_SPEAKER
static rt_uint8_t audprc_buf[320 * 3];
static rt_uint8_t audprc_dac_buf[3][320];
#else
static rt_uint8_t audprc_buf[RINGBUFFER_SIZE];
#endif
static rt_uint8_t audprc_adc_temp[320];
static rt_uint8_t audio_input_buf[RINGBUFFER_SIZE];
static uint16_t buffer_index = 0;

static struct rt_ringbuffer audprc_ringbuffer;

static bool mic_prepared = false;
static uint8_t mic_prepared_count = 0;

#ifdef BSP_USING_MIC_NS_AGC
/* Runtime-tunable NS+AGC state. All knobs are live-adjustable via MSH so the
   user can sweep on real hardware without reflashing. Starting values are
   ASR-oriented (see commit message) and expected to be tuned on-device.

   mic_nsagc_enabled: master on/off. Default FALSE -> existing path runs.
   mic_nsagc_shift  : software pre-shift used INSTEAD of the default 6 when the
                      stage is on. Reduced to 2 so AGC does the digital leveling
                      rather than a blunt ×64 shift (set to 0 to let AGC do it
                      all). When the stage is OFF, the original shift 6 is used. */
static bool    mic_nsagc_enabled = false;
static uint8_t mic_nsagc_shift   = 2;       /* shift applied when stage ON */

    #if defined(WEBRTC_ANS_FIX)
static NsxHandle *mic_ns_inst = NULL;       /* fixed-point NS instance (heap) */
static int        mic_ns_mode = 1;          /* 0 Mild / 1 Medium / 2 Aggressive */
    #endif
    #ifdef WEBRTC_AGC_FIX
static void *mic_agc_inst        = NULL;     /* AGC instance (heap) */
static int   mic_agc_target_dbfs = 6;        /* targetLevelDbfs (lower = louder) */
static int   mic_agc_gain_db     = 20;       /* compressionGaindB */
static int   mic_agc_limiter     = 1;        /* limiterEnable */
static int32_t mic_agc_mic_level = 0;        /* adaptive-digital feedback level */
    #endif

/* Tear down NS+AGC instances (idempotent). Called on mic-path close. */
static void mic_nsagc_free(void)
{
    #if defined(WEBRTC_ANS_FIX)
    if (mic_ns_inst)
    {
        WebRtcNsx_Free(mic_ns_inst);
        mic_ns_inst = NULL;
    }
    #endif
    #ifdef WEBRTC_AGC_FIX
    if (mic_agc_inst)
    {
        WebRtcAgc_Free(mic_agc_inst);
        mic_agc_inst = NULL;
    }
    #endif
}

/* Create + init NS+AGC instances on heap (their Create() allocs the big state,
   keeping it off the 24 KB audio_station thread stack). Safe to call when the
   stage is disabled -- instances just sit idle until `mic_nsagc on`. Frees and
   recreates if already allocated so MSH retune-then-reinit is clean. */
static void mic_nsagc_init(void)
{
    uint32_t fs = audio_sample_rate[AUDIO_STATION_SAMPLE_RATE_OPT]; /* 16000 */
    mic_nsagc_free();
    #if defined(WEBRTC_ANS_FIX)
    mic_ns_inst = WebRtcNsx_Create();
    if (mic_ns_inst)
    {
        WebRtcNsx_Init(mic_ns_inst, fs);
        WebRtcNsx_set_policy(mic_ns_inst, mic_ns_mode);
    }
    else
    {
        rt_kprintf("[mic_nsagc] NS create failed (no mem)\n");
    }
    #endif
    #ifdef WEBRTC_AGC_FIX
    mic_agc_inst = WebRtcAgc_Create();
    if (mic_agc_inst)
    {
        /* AdaptiveDigital: no real analog mic gain to ride; AGC applies digital
           leveling. minLevel/maxLevel 0/255 per the vendor agc test. */
        WebRtcAgc_Init(mic_agc_inst, 0, 255, kAgcModeAdaptiveDigital, fs);
        WebRtcAgcConfig cfg;
        cfg.targetLevelDbfs   = (int16_t)mic_agc_target_dbfs;
        cfg.compressionGaindB = (int16_t)mic_agc_gain_db;
        cfg.limiterEnable     = (uint8_t)mic_agc_limiter;
        cfg.thrhold           = 0;
        WebRtcAgc_set_config(mic_agc_inst, cfg);
        mic_agc_mic_level = 0;
    }
    else
    {
        rt_kprintf("[mic_nsagc] AGC create failed (no mem)\n");
    }
    #endif
}

/* Run NS -> AGC in place on one 10 ms / 160-sample int16 frame. NS first
   (clean), AGC second (level). No-ops gracefully if an instance is missing.
   `samples` is 160 for 16 kHz / 10 ms (READ_AUDIO_BUF_SIZE/2). */
static void mic_nsagc_process(int16_t *frame, size_t samples)
{
    #if defined(WEBRTC_ANS_FIX)
    if (mic_ns_inst)
    {
        int16_t *in_p[1]  = { frame };
        int16_t *out_p[1] = { frame }; /* in-place OK for fixed-point NS */
        WebRtcNsx_Process(mic_ns_inst, (const int16_t *const *)in_p, 1, out_p);
    }
    #endif
    #ifdef WEBRTC_AGC_FIX
    if (mic_agc_inst)
    {
        int16_t *in_p[1]  = { frame };
        int16_t *out_p[1] = { frame }; /* "May be the same vector as the input" */
        int32_t level_out = 0;
        uint8_t sat = 0;
        if (WebRtcAgc_Process(mic_agc_inst, (const int16_t *const *)in_p, 1,
                              samples, (int16_t *const *)out_p,
                              mic_agc_mic_level, &level_out, 0, &sat) == 0)
        {
            mic_agc_mic_level = level_out;
        }
    }
    #endif
}
#endif /* BSP_USING_MIC_NS_AGC */

rt_uint8_t *get_audio_adc_buf(void)
{
    return audprc_adc_temp;
}

static rt_err_t audprc_adc_ind(rt_device_t dev, rt_size_t size)
{
    rt_event_send(&aud_event, AUDIO_TRS_AUDPRC_ADC_EVENT);
    return 0;
}

rt_err_t audprc_dac_done(rt_device_t dev, void *buffer)
{
    rt_event_send(&aud_event, AUDIO_TRS_AUDPRC_DAC_EVENT);

    return 0;
}

int audio_codec_open(void)
{
    uint32_t inf;
    int stream;

    g_codec_dev = rt_device_find(AUD_CODEC_DEVICE);
    if (!g_codec_dev)
    {
        // rt_kprintf("Fail to find device %s\n", AUD_CODEC_DEVICE);
        return -1;
    }
    if (RT_EOK != rt_device_open(g_codec_dev, RT_DEVICE_OFLAG_WRONLY))
    {
        // rt_kprintf("Fail to open device %s\n", AUD_CODEC_DEVICE);
        return -1;
    }

    // codec adc rx interface, data send to audprc
    inf = AUDPRC_RX_FROM_CODEC;
    rt_device_control(g_codec_dev, AUDIO_CTL_SETINPUT, (void *)inf);
    // codec adc rx audio arg cfg
    struct rt_audio_caps caps = {
        .main_type = AUDIO_TYPE_INPUT,
        .sub_type = 1 << HAL_AUDCODEC_ADC_CH0,
        .udata.config.channels = 1,
        .udata.config.samplefmt = 16,
        .udata.config.samplerate =
            audio_sample_rate[AUDIO_STATION_SAMPLE_RATE_OPT],
    };
    rt_device_control(g_codec_dev, AUDIO_CTL_CONFIGURE, &caps);

    caps.main_type = AUDIO_TYPE_MIXER; /* 音量管理类型 */
    caps.sub_type = AUDIO_MIXER_MIC;   /* 设置录音的主音量 */
    caps.udata.value = 100;            /* 范围 0 ~ 100 */
    rt_device_control(g_codec_dev, AUDIO_CTL_CONFIGURE, &caps);

    // codec adc rx start
    stream = AUDIO_STREAM_RECORD;
    stream |= ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
    rt_device_control(g_codec_dev, AUDIO_CTL_START, &stream);

#if PERIPHERAL_AUD_SPEAKER
    // codec dac tx interface, data recv from audprc
    inf = AUDPRC_TX_TO_CODEC;
    rt_device_control(g_codec_dev, AUDIO_CTL_SETOUTPUT, (void *)inf);
    // codec dac tx audio arg cfg
    for (int i = 1; i <= 2; i++)
    {
        caps.main_type = AUDIO_TYPE_OUTPUT;
        caps.sub_type = i; // 1 << HAL_AUDCODEC_DAC_CH0;
        caps.udata.config.channels = 1;
        caps.udata.config.samplefmt = 16;
        caps.udata.config.samplerate = 16000;
        rt_device_control(g_codec_dev, AUDIO_CTL_CONFIGURE, &caps);
    }

    // codec set vol (-36 to 0 dB range; see audio_codec_set_volume)
    int vol = -20;
    rt_device_control(g_codec_dev, AUDIO_CTL_SETVOLUME, (void *)vol);
    // codec dac tx start
    stream = AUDIO_STREAM_REPLAY;
    stream |= ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
    rt_device_control(g_codec_dev, AUDIO_CTL_START, &stream);
    int mute = 0;
    rt_device_control(g_codec_dev, AUDIO_CTL_MUTE, (void *)mute);
#endif

    return 0;
}

int audio_codec_close(void)
{
    int ret = RT_EOK;

    if (g_codec_dev)
    {
        ret = rt_device_close(g_codec_dev);
        RT_ASSERT(ret == RT_EOK);
        g_codec_dev = NULL;
    }
    else
    {
        return -1;
    }

    return ret;
}

int audio_codec_set_volume(int vol)
{
    if (vol > 0)
        vol = 0;
    if (vol < -36)
        vol = -36;

    if (g_codec_dev)
    {
        rt_device_control(g_codec_dev, AUDIO_CTL_SETVOLUME, (void *)vol);
    }
    else
    {
        return -1;
    }

    return 0;
}

int audio_prc_open(void)
{
    uint32_t inf;
    int stream;
    struct rt_audio_sr_convert cfg;
#if PERIPHERAL_AUD_SPEAKER
    rt_ringbuffer_init(&audprc_ringbuffer, audprc_buf, 320 * 3);
#else
    rt_ringbuffer_init(&audprc_ringbuffer, audprc_buf, RINGBUFFER_SIZE);
    rt_ringbuffer_reset(&audprc_ringbuffer);
#endif
    mic_prepared = false;
    mic_prepared_count = 0;
    g_audprc_dev = rt_device_find(AUD_PRC_DEVICE);
    if (!g_audprc_dev)
    {
        return -1;
    }
    if (RT_EOK != rt_device_open(g_audprc_dev, RT_DEVICE_OFLAG_RDWR))
    {
        return -1;
    }

    // audprc rx from codec
    inf = AUDPRC_RX_FROM_CODEC;
    rt_device_control(g_audprc_dev, AUDIO_CTL_SETINPUT, (void *)inf);
    // audprc rx audio arg cfg
    struct rt_audio_caps caps = {
        .main_type = AUDIO_TYPE_INPUT,
        .sub_type = 0,
        .udata.config.channels = 1,
        .udata.config.samplefmt = 16,
        .udata.config.samplerate =
            audio_sample_rate[AUDIO_STATION_SAMPLE_RATE_OPT],
    };
    rt_device_control(g_audprc_dev, AUDIO_CTL_CONFIGURE, &caps);
    caps.main_type = AUDIO_TYPE_MIXER; /* 音量管理类型 */
    caps.sub_type = AUDIO_MIXER_MIC;   /* 设置录音的主音量 */
    caps.udata.value = 100;            /* 范围 0 ~ 100 */
    rt_device_control(g_codec_dev, AUDIO_CTL_CONFIGURE, &caps);
    // register prc rx indicate callback
    rt_device_set_rx_indicate(g_audprc_dev, audprc_adc_ind);
    // audprc rx ch0 start
    stream = AUDIO_STREAM_RECORD;
    stream |= ((1 << HAL_AUDPRC_RX_CH0) << 8);
    rt_device_control(g_audprc_dev, AUDIO_CTL_START, &stream);

#if PERIPHERAL_AUD_SPEAKER
    // audprc tx to codec
    inf = AUDPRC_TX_TO_CODEC;
    rt_device_control(g_audprc_dev, AUDIO_CTL_SETOUTPUT, (void *)inf);

    cfg.channel = 2;
    cfg.source_sr = 16000;
    cfg.dest_sr = 16000;
    rt_device_control(g_audprc_dev, AUDIO_CTL_OUTPUTSRC, (void *)(&cfg));
    /*
    caps.main_type = AUDIO_TYPE_SELECTOR;
    caps.sub_type = 0xFF;
    caps.udata.value = 0x5050;
    rt_device_control(g_audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    caps.main_type = AUDIO_TYPE_MIXER;
    caps.sub_type = 0xFF;
    caps.udata.value = 0x5050; //0x5150
    rt_device_control(g_audprc_dev, AUDIO_CTL_CONFIGURE, &caps);
    */
    // audprc tx audio arg cfg
    caps.main_type = AUDIO_TYPE_OUTPUT;
    caps.sub_type = 0;
    caps.udata.config.channels = 2;
    caps.udata.config.samplefmt = 16;
    caps.udata.config.samplerate = 16000;
    rt_device_control(g_audprc_dev, AUDIO_CTL_CONFIGURE, &caps);
    // register tx complete callback
    rt_device_set_tx_complete(g_audprc_dev, audprc_dac_done);

    // audprc tx ch0 start
    stream = AUDIO_STREAM_REPLAY;
    stream |= ((1 << HAL_AUDPRC_TX_CH0) << 8);
    rt_device_control(g_audprc_dev, AUDIO_CTL_START, &stream);
#endif
#ifdef BSP_USING_MIC_NS_AGC
    /* Allocate NS+AGC when the mic path opens (frees on close). Instances sit
       idle unless `mic_nsagc on` has been issued -- creating them up front means
       toggling on at runtime needs no allocation in the audio thread. */
    mic_nsagc_init();
#endif
    return 0;
}

int audio_prc_close(void)
{
    int ret = RT_EOK;

#ifdef BSP_USING_MIC_NS_AGC
    mic_nsagc_free();
#endif
    if (g_audprc_dev)
    {
        ret = rt_device_close(g_audprc_dev);
        RT_ASSERT(ret == RT_EOK);
        g_audprc_dev = NULL;
    }
    else
    {
        return -1;
    }

    return ret;
}

#ifdef BSP_USING_I2S
static rt_err_t i2s_rx_ind(rt_device_t dev, rt_size_t size)
{
    // rt_kprintf("i2s_rx_ind size=%d\n", size);
    rt_event_send(&aud_event, AUDIO_TRS_I2S_RX_EVENT);

    return 0;
}

rt_err_t i2s_tx_done(rt_device_t dev, void *buffer)
{
    // rt_kprintf("i2s_tx_done\n");
    rt_event_send(&aud_event, AUDIO_TRS_I2S_TX_EVENT);

    return 0;
}

void i2s_pinmux_cfg(void)
{
    #ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA03, I2S1_SDO, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA04, I2S1_SDI, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA05, I2S1_BCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA06, I2S1_LRCK, PIN_NOPULL, 1);
    #elif defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA39, I2S1_SDO, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, I2S1_SDI, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA40, I2S1_BCK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA41, I2S1_LRCK, PIN_NOPULL, 1);
    #endif
}

int i2s_device_open(void)
{
    int stream = 0;
    rt_uint32_t inter;
    #if PERIPHERAL_AUD_SPEAKER
    rt_ringbuffer_init(&i2s_rx_ringbuffer, i2s_rx_buf, 320 * 3);
    #else
    rt_ringbuffer_init(&i2s_rx_ringbuffer, i2s_rx_buf, 320 * 1);
    #endif

    i2s_pinmux_cfg();

    g_i2s2_dev = rt_device_find(I2S_DEVICE);
    if (!g_i2s2_dev)
    {
        rt_kprintf("Fail to find device %s\n", I2S_DEVICE);
        return -1;
    }

    if (RT_EOK != rt_device_open(g_i2s2_dev, RT_DEVICE_OFLAG_RDWR))
    {
        rt_kprintf("Fail to open device %s\n", I2S_DEVICE);
        return -1;
    }

    // config audio arg
    struct rt_audio_caps caps = {
        .main_type = AUDIO_TYPE_INPUT,
        .sub_type = AUDIO_DSP_PARAM,
        .udata.config.channels = 1,
        .udata.config.samplefmt = 16,
        .udata.config.samplerate = 16000,
    };

    if (RT_EOK != rt_device_control(g_i2s2_dev, AUDIO_CTL_CONFIGURE, &caps))
    {
        rt_kprintf("Fail to control device %s\n", I2S_DEVICE);
        return -1;
    }

    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = AUDIO_DSP_MODE;
    caps.udata.value = I2S_WORK_MODE; // 1:slave mode, 0:master mode
    if (RT_EOK != rt_device_control(g_i2s2_dev, AUDIO_CTL_CONFIGURE, &caps))
    {
        rt_kprintf("Fail to control device %s\n", I2S_DEVICE);
        return -1;
    }

    // set i2s use dma interface. 0:dma 1:audprc
    inter = 0;
    if (RT_EOK !=
        rt_device_control(g_i2s2_dev, AUDIO_CTL_SETINPUT, (void *)inter))
    {
        rt_kprintf("Fail to control device %s\n", I2S_DEVICE);
        return -1;
    }

    // register rx indicate callback
    rt_device_set_rx_indicate(g_i2s2_dev, i2s_rx_ind);

    // start i2s rx
    stream = AUDIO_STREAM_RECORD;
    rt_device_control(g_i2s2_dev, AUDIO_CTL_START, &stream);
    #if PERIPHERAL_AUD_SPEAKER
    // set i2s use dma interface. 0:dma 1:audprc
    inter = 0;
    if (RT_EOK !=
        rt_device_control(g_i2s2_dev, AUDIO_CTL_SETOUTPUT, (void *)inter))
    {
        rt_kprintf("Fail to control device %s\n", I2S_DEVICE);
        return -1;
    }
    // register tx complete callback
    rt_device_set_tx_complete(g_i2s2_dev, i2s_tx_done);

    // start i2s tx
    stream = AUDIO_STREAM_REPLAY;
    rt_device_control(g_i2s2_dev, AUDIO_CTL_START, &stream);
    #endif

    return 0;
}

int i2s_device_close(void)
{
    int ret = RT_EOK;

    if (g_i2s2_dev)
    {
        ret = rt_device_close(g_i2s2_dev);
        RT_ASSERT(ret == RT_EOK);
        g_i2s2_dev = NULL;
    }
    else
    {
        rt_kprintf("i2s device close fail\n");
        return -1;
    }

    return ret;
}
#endif

#define TEMP_PCM_BUF_SIZE (RINGBUFFER_SIZE / 2)
static int16_t pcm_data[TEMP_PCM_BUF_SIZE];

static bool voice_activity_detect(uint8_t *buf, uint16_t len)
{
    bool vad_active = false;
    // rt_kprintf("[%d ms]handle_mic_data len=%d\n", rt_tick_get_millisecond(),
    // len);
    int16_t audio_fs = audio_sample_rate[AUDIO_STATION_SAMPLE_RATE_OPT];
    size_t frame_length = len / 2;
    /*計算幀長度
    10ms 幀長度：16000 Hz * 10ms / 1000 = 160
    20ms 幀長度：16000 Hz * 20ms / 1000 = 320
    30ms 幀長度：16000 Hz * 30ms / 1000 = 480
    */
    uint16_t valid_frame_len = audio_fs * 30 / 1000;
    uint16_t loop = frame_length / valid_frame_len;
    for (uint16_t i = 0; i < loop; i++)
    {
        for (size_t j = 0; j < valid_frame_len; j++)
        {
            pcm_data[j] =
                (int16_t)(buf[2 * (i * valid_frame_len + j)] |
                          (buf[2 * (i * valid_frame_len + j) + 1] << 8));
        }
        int active = WebRtcVad_Process(voice_provider.vad_inst, audio_fs,
                                       pcm_data, valid_frame_len);
        if (active)
        {
            vad_active = true;
            break;
        }
    }
    return vad_active;
}

// ADPCM 编码器状态
typedef struct
{
    int16_t prev_sample;
    int step_index;
} adpcm_state_t;

// ADPCM 步长表
static const int step_table[89] = {
    7,     8,     9,     10,    11,    12,    13,    14,    16,    17,
    19,    21,    23,    25,    28,    31,    34,    37,    41,    45,
    50,    55,    60,    66,    73,    80,    88,    97,    107,   118,
    130,   143,   157,   173,   190,   209,   230,   253,   279,   307,
    337,   371,   408,   449,   494,   544,   598,   658,   724,   796,
    876,   963,   1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
    2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,
    5894,  6484,  7132,  7845,  8630,  9493,  10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767};

// ADPCM 索引表
static const int index_table[16] = {-1, -1, -1, -1, 2, 4, 6, 8,
                                    -1, -1, -1, -1, 2, 4, 6, 8};

// 限制值函数
static inline int clamp(int val, int min, int max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

// Correct int16-SATURATING software gain for the mic capture path. The shared
// middleware auido_gain_pcm() clamps to ±65535 then assigns to int16_t
// (audio_server.c:3057-3065) → loud samples WRAP (sign-flip) instead of
// saturating. We use a correct local version so raising gain never injects wrap
// distortion. The real level lift is the analog PDM gain (g_pdm_volume); this
// software shift stays conservative.
static void gain_pcm_sat(int16_t *p, rt_size_t bytes, uint8_t shift)
{
    for (rt_size_t i = 0; i < bytes / 2; i++)
    {
        int v = ((int)p[i]) << shift;
        if (v > 32767)
            v = 32767;
        else if (v < -32768)
            v = -32768;
        p[i] = (int16_t)v;
    }
}

// ADPCM 编码函数
uint8_t adpcm_encode(adpcm_state_t *state, int16_t sample)
{
    int delta = sample - state->prev_sample;
    int step = step_table[state->step_index];
    int code = 0;
    int diffq = step >> 3;

    // 计算编码值
    if (delta < 0)
    {
        code = 8;
        delta = -delta;
    }
    if (delta >= step)
    {
        code |= 4;
        delta -= step;
        diffq += step;
    }
    step >>= 1;
    if (delta >= step)
    {
        code |= 2;
        delta -= step;
        diffq += step;
    }
    step >>= 1;
    if (delta >= step)
    {
        code |= 1;
        diffq += step;
    }

    // 更新状态
    if (code & 8)
    {
        state->prev_sample -= diffq;
    }
    else
    {
        state->prev_sample += diffq;
    }
    state->prev_sample = clamp(state->prev_sample, -32768, 32767);

    state->step_index += index_table[code];
    state->step_index = clamp(state->step_index, 0, 88);

    return code & 0x0F;
}

// 压缩音频数据
void compress_audio(int16_t *input, uint8_t *output, size_t len)
{
    adpcm_state_t state = {0, 0};
    for (size_t i = 0; i < len; i++)
    {
        uint8_t code = adpcm_encode(&state, input[i]);
        if (i % 2 == 0)
        {
            output[i / 2] = code;
        }
        else
        {
            output[i / 2] |= (code << 4);
        }
    }
}

void compress_audio_and_send_via_ble(int16_t *audio_data, size_t len)
{
    size_t compressed_len = (len + 1) / 2; // 每两个采样点压缩为一个字节
    compress_audio(audio_data, compressed_data + 1, len);
    // Send length = 1-byte speaking-index header + ADPCM payload. Was sending
    // only `compressed_len`, an off-by-one that dropped the final ADPCM byte
    // (2 samples) every block — fixed to `compressed_len + 1`.
    uint16_t send_len = (uint16_t)(compressed_len + 1);
    // Honor the send result: skaiwatch_ble_audio_send returns false when the BLE
    // notification is dropped (TX-queue saturation). This return was previously
    // ignored, so dropped audio blocks vanished SILENTLY. Surface it, rate-limited
    // so a congestion burst can't flood the log.
    bool sent = skaiwatch_ble_audio_send(compressed_data, send_len);
    if (!sent)
    {
        audio_tx_drop_count++;
        if ((audio_tx_drop_count & 0x1F) == 1)
        {
            rt_kprintf("AUDIO: BLE audio TX drop (count=%u)\n",
                       audio_tx_drop_count);
        }
    }
}

extern bool get_is_open_app_list_ai(void);
void audio_transfer_entry(void *parameter)
{
    rt_uint32_t evt = 0, rdlen = 0, putlen = 0, wrlen = 0, getlen = 0;
    rt_uint8_t index1 = 0, index2 = 0;
    rt_event_init(&aud_event, "audio_trans", RT_IPC_FLAG_FIFO);

#ifdef PA_USING_AW8155
    sifli_aw8155_start();
#endif

    while (1)
    {
        if (rt_event_recv(&aud_event, AUDIO_TRS_ALL_EVENT,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &evt) == RT_EOK)
        {
// Note: Need to enable I2S driver for BSP
#ifdef BSP_USING_I2S
            if (evt & AUDIO_TRS_I2S_RX_EVENT)
            {
                rdlen = rt_device_read(g_i2s2_dev, 0, i2s_rx_temp, 320);
                putlen =
                    rt_ringbuffer_put(&i2s_rx_ringbuffer, i2s_rx_temp, 320);
                rt_kprintf("I2S_RX_EVENT rdlen=%d, putlen=%d, data_len=%d\n",
                           rdlen, putlen,
                           rt_ringbuffer_data_len(&i2s_rx_ringbuffer));
            }
            if (evt & AUDIO_TRS_I2S_TX_EVENT)
            {
                if (rt_ringbuffer_data_len(&audprc_ringbuffer) >= 320)
                {
                    getlen = rt_ringbuffer_get(&audprc_ringbuffer,
                                               i2s_tx_buf[index1], 320);
                    wrlen =
                        rt_device_write(g_i2s2_dev, 0, i2s_tx_buf[index1], 320);
                    rt_kprintf("I2S_TX_EVENT getlen=%d, wrlen=%d, data_len=%d, "
                               "index1=%d\n",
                               getlen, wrlen,
                               rt_ringbuffer_data_len(&audprc_ringbuffer),
                               index1);
                    if (++index1 == 3)
                        index1 = 0;
                }
            }
#endif
            if (evt & AUDIO_TRS_AUDPRC_ADC_EVENT)
            {
                if (g_audprc_dev == RT_NULL)
                    continue;
                rdlen = rt_device_read(g_audprc_dev, 0, audprc_adc_temp,
                                       READ_AUDIO_BUF_SIZE);

                /*audio data alg process, gain dc ramp nr*/
                // Phase 2 fix: software gain back to ×8 (shift 3) — the level is
                // now raised at the ANALOG source via PDM gain (g_pdm_volume,
                // drv_audprc.c:34, default 56 = 28 dB, max 90 = 45 dB). Tune it
                // LIVE with the `pdm_gain <val>` MSH command (0.5 dB units), no
                // reflash. Analog gain pre-ADC = better SNR than software shift.
                // Saturating gain (not the wrap-prone middleware auido_gain_pcm).
#ifdef BSP_USING_MIC_NS_AGC
                if (mic_nsagc_enabled)
                {
                    /* NS -> AGC for ASR. Use a smaller software pre-shift
                       (mic_nsagc_shift, default 2) since AGC does the digital
                       leveling; then run NS (clean) -> AGC (level). 320 bytes =
                       160 int16 samples = one 10 ms / 16 kHz frame, the exact
                       NS/AGC frame size. */
                    gain_pcm_sat((int16_t *)audprc_adc_temp,
                                 READ_AUDIO_BUF_SIZE, mic_nsagc_shift);
                    mic_nsagc_process((int16_t *)audprc_adc_temp,
                                      READ_AUDIO_BUF_SIZE / 2);
                }
                else
#endif
                {
                    gain_pcm_sat((int16_t *)audprc_adc_temp,
                                 READ_AUDIO_BUF_SIZE, 6); // ×64; analog does the lifting
                }

                // rt_kprintf("audprc_adc_temp[0]=%d, audprc_adc_temp[1]=%d\n",
                // audprc_adc_temp[0], audprc_adc_temp[1]);
                putlen = rt_ringbuffer_put(&audprc_ringbuffer, audprc_adc_temp,
                                           READ_AUDIO_BUF_SIZE);
                // rt_kprintf("[%d ms]AUDPRC_ADC_EVENT rdlen=%d, putlen=%d,
                // data_len=%d\n",
                //            rt_tick_get_millisecond(), rdlen, putlen,
                //            rt_ringbuffer_data_len(&audprc_ringbuffer));

                // ***************** 將音頻暫存，滿960位元組再處理
                // ***************** //
                if (rt_ringbuffer_data_len(&audprc_ringbuffer) ==
                    RINGBUFFER_SIZE)
                {
                    if (!mic_prepared)
                    {
                        if (mic_prepared_count < 7)
                        {
                            mic_prepared_count++;
                            if (voice_provider.vad_inst &&
                                app_voice_get_voice2text_status())
                            {
                                voice_activity_detect(audio_input_buf,
                                                      RINGBUFFER_SIZE);
                            }
                            rt_ringbuffer_reset(&audprc_ringbuffer);
                            continue;
                        }
                        else
                        {
                            mic_prepared = true;
                        }
                    }

                    if (voice_provider.vad_inst &&
                        app_voice_get_voice2text_status())
                    {
                        getlen =
                            rt_ringbuffer_get(&audprc_ringbuffer,
                                              audio_input_buf, RINGBUFFER_SIZE);

                        bool active = voice_activity_detect(audio_input_buf,
                                                            RINGBUFFER_SIZE);
                        voice_provider.notify_vad_status(active);
                        // 有說話聲才傳送至手機辨識
                        if (active)
                        {
                            // Convert byte buffer to int16_t PCM data
                            for (uint16_t i = 0; i < RINGBUFFER_SIZE / 2; i++)
                            {
                                pcm_data[i] =
                                    (int16_t)(audio_input_buf[2 * i] |
                                              (audio_input_buf[2 * i + 1]
                                               << 8));
                            }
#if defined(V2T_USE_OPUS) && defined(ENABLE_OPUS_ENCODER)
                            // Opus streaming (VOIP + in-band FEC). Each 20ms
                            // frame is one BLE notification framed [idx][seq][opus];
                            // phone opus_stream_decoder.dart conceals drops via
                            // FEC/PLC. Wire format must match phone _useOpusAudio.
                            extern void v2t_opus_stream_send(const int16_t *pcm,
                                                             uint32_t samples);
                            v2t_opus_stream_send(pcm_data, TEMP_PCM_BUF_SIZE);
#else
                            // Add header: speaking sentence index
                            compressed_data[0] = get_speech_coding();
                            // ADPCM encoding (legacy real-time path)
                            compress_audio_and_send_via_ble(pcm_data,
                                                            TEMP_PCM_BUF_SIZE);
#endif
                        }
                    }
                    else if (app_voice_get_recording_status())
                    {
                        getlen =
                            rt_ringbuffer_get(&audprc_ringbuffer,
                                              audio_input_buf, RINGBUFFER_SIZE);

                        audio_record_pcm(audio_input_buf, RINGBUFFER_SIZE);
                    }
                    // clear ringbuffer
                    rt_ringbuffer_reset(&audprc_ringbuffer);
                }
            }
#if PERIPHERAL_AUD_SPEAKER
    #if 1 // i2s rx to audio prc tx
            if (evt & AUDIO_TRS_AUDPRC_DAC_EVENT)
            {
                if (rt_ringbuffer_data_len(&i2s_rx_ringbuffer) >= 320)
                {
                    getlen = rt_ringbuffer_get(&i2s_rx_ringbuffer,
                                               audprc_dac_buf[index2], 320);
                    wrlen = rt_device_write(g_audprc_dev, 0,
                                            audprc_dac_buf[index2], 320);
                    // rt_kprintf("AUDPRC_DAC_EVENT getlen=%d, wrlen=%d,
                    // data_len=%d, index2=%d\n",
                    //            getlen, wrlen,
                    //            rt_ringbuffer_data_len(&i2s_rx_ringbuffer),
                    //            index2);
                    if (++index2 == 3)
                        index2 = 0;
                }
            }
    #endif
#endif
        }
    }
}


void audio_subscribe(void)
{
    audio_codec_open();
    audio_prc_open();
    // i2s_device_open();
}

void audio_unsubscribe(void)
{
    // i2s_device_close();
    audio_codec_close();
    audio_prc_close();
}

#ifdef BSP_USING_MIC_NS_AGC
/* Live tuning of the optional NS+AGC mic stage, no reflash needed. Usage:
     mic_nsagc                 -- print current settings
     mic_nsagc on | off        -- enable / disable the whole NS+AGC stage
     mic_nsagc shift <0..8>    -- software pre-shift used while stage ON
     mic_nsagc ns <0|1|2>      -- NS policy: 0 Mild / 1 Medium / 2 Aggressive
     mic_nsagc target <dbfs>   -- AGC targetLevelDbfs (lower = louder)
     mic_nsagc gain <db>       -- AGC compressionGaindB
     mic_nsagc limiter <0|1>   -- AGC limiterEnable
   NS/AGC param changes apply immediately to the live instance (if the mic path
   is open); otherwise they take effect at the next mic open. */
static void mic_nsagc_print(void)
{
    rt_kprintf("[mic_nsagc] enabled=%d shift=%u", mic_nsagc_enabled,
               (unsigned)mic_nsagc_shift);
    #if defined(WEBRTC_ANS_FIX)
    rt_kprintf(" ns_mode=%d(inst=%s)", mic_ns_mode, mic_ns_inst ? "y" : "n");
    #endif
    #ifdef WEBRTC_AGC_FIX
    rt_kprintf(" agc[target=%d gain=%d limiter=%d](inst=%s)",
               mic_agc_target_dbfs, mic_agc_gain_db, mic_agc_limiter,
               mic_agc_inst ? "y" : "n");
    #endif
    rt_kprintf("\n");
}

    #ifdef WEBRTC_AGC_FIX
/* Re-push AGC config to a live instance after an MSH param change. */
static void mic_agc_apply_config(void)
{
    if (!mic_agc_inst)
    {
        return;
    }
    WebRtcAgcConfig cfg;
    cfg.targetLevelDbfs   = (int16_t)mic_agc_target_dbfs;
    cfg.compressionGaindB = (int16_t)mic_agc_gain_db;
    cfg.limiterEnable     = (uint8_t)mic_agc_limiter;
    cfg.thrhold           = 0;
    WebRtcAgc_set_config(mic_agc_inst, cfg);
}
    #endif

static void mic_nsagc(int argc, char **argv)
{
    if (argc < 2)
    {
        mic_nsagc_print();
        return;
    }
    if (rt_strcmp(argv[1], "on") == 0)
    {
        mic_nsagc_enabled = true;
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        mic_nsagc_enabled = false;
    }
    else if (rt_strcmp(argv[1], "shift") == 0 && argc >= 3)
    {
        int v = atoi(argv[2]);
        if (v < 0) v = 0;
        if (v > 8) v = 8;
        mic_nsagc_shift = (uint8_t)v;
    }
    #if defined(WEBRTC_ANS_FIX)
    else if (rt_strcmp(argv[1], "ns") == 0 && argc >= 3)
    {
        int v = atoi(argv[2]);
        if (v < 0) v = 0;
        if (v > 2) v = 2;
        mic_ns_mode = v;
        if (mic_ns_inst)
        {
            WebRtcNsx_set_policy(mic_ns_inst, mic_ns_mode);
        }
    }
    #endif
    #ifdef WEBRTC_AGC_FIX
    else if (rt_strcmp(argv[1], "target") == 0 && argc >= 3)
    {
        mic_agc_target_dbfs = atoi(argv[2]);
        mic_agc_apply_config();
    }
    else if (rt_strcmp(argv[1], "gain") == 0 && argc >= 3)
    {
        mic_agc_gain_db = atoi(argv[2]);
        mic_agc_apply_config();
    }
    else if (rt_strcmp(argv[1], "limiter") == 0 && argc >= 3)
    {
        mic_agc_limiter = atoi(argv[2]) ? 1 : 0;
        mic_agc_apply_config();
    }
    #endif
    else
    {
        rt_kprintf("usage: mic_nsagc [on|off|shift N|ns 0/1/2|target N|gain N|"
                   "limiter 0/1]\n");
        return;
    }
    mic_nsagc_print();
}
MSH_CMD_EXPORT(mic_nsagc, tune optional WebRTC NS+AGC mic stage);
#endif /* BSP_USING_MIC_NS_AGC */

#if !kReleaseMode
static void audio_open_demo(uint8_t argc, char **argv)
{
    audio_codec_open();
    audio_prc_open();
    // i2s_device_open();
}
MSH_CMD_EXPORT(audio_open_demo, audio_open_demo test);

static void audio_close_demo(uint8_t argc, char **argv)
{
    // i2s_device_close();
    audio_codec_close();
    audio_prc_close();
}
MSH_CMD_EXPORT(audio_close_demo, audio_close_demo test);
#endif

#ifndef BSP_USING_PC_SIMULATOR
// Increased stack size for Opus encoder
// Stack usage: opus_encode() ~6-8KB + local variables + RT-Thread
// overhead ~2KB Total: ~10-12KB required for safe operation
    #define AUDIO_STATION_STACK_SIZE (24 * 1024)
    #define AUDIO_STATION_PRIORITY 16
    #define AUDIO_STATION_TICK 10
static rt_thread_t tid_audio_station;

static int module_init(void)
{
    tid_audio_station = rt_thread_create(
        "audio_station", audio_transfer_entry, NULL, AUDIO_STATION_STACK_SIZE,
        AUDIO_STATION_PRIORITY, AUDIO_STATION_TICK);
    rt_thread_startup(tid_audio_station);
    return 0;
}
INIT_APP_EXPORT(module_init);
#endif
