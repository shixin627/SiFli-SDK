/**
 ******************************************************************************
 * @file   i2s_test.c
 * @author Sifli software development team
 ******************************************************************************
 */
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
 *is_test.c
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

bool mic_prepared = false;
static uint8_t mic_prepared_count = 0;

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

    // codec set vol
    int volumex2 = -20;
    rt_device_control(g_codec_dev, AUDIO_CTL_SETVOLUME, (void *)volumex2);
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
    return 0;
}

int audio_prc_close(void)
{
    int ret = RT_EOK;

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
    skaiwatch_ble_audio_send(compressed_data, compressed_len);
}

extern bool get_is_open_app_list_ai(void);
void audio_transfer_entry(void *parameter)
{
    rt_uint32_t evt = 0, rdlen = 0, putlen = 0, wrlen = 0, getlen = 0;
    rt_uint8_t index = 0, index1 = 0, index2 = 0;
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
                // auido_gain_pcm((int16_t *)audprc_adc_temp,
                // READ_AUDIO_BUF_SIZE, 5); // pcm data left shift 4 bits

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
                            // Add header: speaking sentence index
                            compressed_data[0] = get_speech_coding();
                            // ADPCM encoding
                            compress_audio_and_send_via_ble(pcm_data,
                                                            TEMP_PCM_BUF_SIZE);
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
    peripheral_provider.set_audio_code_status(true);
}

static void audio_open_demo(uint8_t argc, char **argv)
{
    audio_codec_open();
    audio_prc_open();
    // i2s_device_open();
    peripheral_provider.set_audio_code_status(true);
}

MSH_CMD_EXPORT(audio_open_demo, audio_open_demo test);

void audio_unsubscribe(void)
{
    // i2s_device_close();
    audio_codec_close();
    audio_prc_close();
    peripheral_provider.set_audio_code_status(false);
}

static void audio_close_demo(uint8_t argc, char **argv)
{
    // i2s_device_close();
    audio_codec_close();
    audio_prc_close();
    peripheral_provider.set_audio_code_status(false);
}

MSH_CMD_EXPORT(audio_close_demo, audio_close_demo test);


#ifndef BSP_USING_PC_SIMULATOR
// Audio Station
#ifdef ENABLE_OPUS_ENCODER
// Increased stack size for Opus encoder
// Stack usage: opus_encode() ~6-8KB + local variables + RT-Thread overhead ~2KB
// Total: ~10-12KB required for safe operation
#define AUDIO_STATION_STACK_SIZE (20 * 1024)
#else
#define AUDIO_STATION_STACK_SIZE (20 * 1024)
#endif

#define AUDIO_STATION_PRIORITY 16
#define AUDIO_STATION_TICK 10
static rt_thread_t tid_audio_station;

static int module_init(void)
{
    tid_audio_station = rt_thread_create("audio_station", audio_transfer_entry, NULL, AUDIO_STATION_STACK_SIZE,
                                         AUDIO_STATION_PRIORITY, AUDIO_STATION_TICK);
    rt_thread_startup(tid_audio_station);
    return 0;
}
INIT_APP_EXPORT(module_init);
#endif