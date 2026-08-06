/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co, Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "audio_server.h"
#include "rtthread.h"
#include "usb_audio.h"
#include "usbd_audio.h"
#include "usbd_core.h"

#define USBD_VID 0x38f4
#define USBD_PID 0xFFFF
#define USBD_MAX_POWER 100
#define USBD_LANGID_STRING 1033

#define EP_INTERVAL_FS 0x01

#define AUDIO_VERSION 0x0100

#define AUDIO_IN_EP 0x85  // Endpoint 5 IN for microphone
#define AUDIO_OUT_EP 0x02 // Endpoint 2 OUT for speaker

#define AUDIO_SPEAKER_INTF 1
#define AUDIO_MIC_INTF 2

#define AUDIO_IN_FU_ID 0x02
#define AUDIO_OUT_FU_ID 0x05

/* AUDIO Class Config */
#define AUDIO_MIC_FREQ 16000U
#define AUDIO_MIC_FRAME_SIZE_BYTE 2u
#define AUDIO_MIC_RESOLUTION_BIT 16u
#define AUDIO_SPEAKER_FREQ 16000U
#define AUDIO_SPEAKER_FRAME_SIZE_BYTE 2u
#define AUDIO_SPEAKER_RESOLUTION_BIT 16u

#define IN_CHANNEL_NUM 1
#define OUT_CHANNEL_NUM 1

#define AUDIO_SAMPLE_FREQ(frq)                                                 \
    (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define AUDIO_OUT_PACKET                                                       \
    ((uint32_t)((AUDIO_SPEAKER_FREQ * AUDIO_SPEAKER_FRAME_SIZE_BYTE * 1) /     \
                1000))
/* 16bit(2 Bytes) Two Channels(Mono:2) */
#define AUDIO_IN_PACKET                                                        \
    ((uint32_t)((AUDIO_MIC_FREQ * AUDIO_MIC_FRAME_SIZE_BYTE * 1) / 1000))

#define USB_AUDIO_CONFIG_DESC_SIZ                                              \
    (unsigned long)(9 + AUDIO_AC_DESCRIPTOR_LEN(2) +                           \
                    AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC +                      \
                    AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(1, 1) +                  \
                    AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC +                     \
                    AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC +                      \
                    AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(1, 1) +                  \
                    AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC +                     \
                    AUDIO_AS_DESCRIPTOR_LEN(1) +                               \
                    AUDIO_AS_DESCRIPTOR_LEN(1))

#define AUDIO_AC_SIZ                                                           \
    (AUDIO_SIZEOF_AC_HEADER_DESC(2) + AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC +    \
     AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(1, 1) +                                 \
     AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC +                                    \
     AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC +                                     \
     AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(1, 1) +                                 \
     AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC)

static const uint8_t device_descriptor[] =
{
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID,
                               0x0001, 0x01),
};

static const uint8_t config_descriptor_fs[] =
{
    USB_CONFIG_DESCRIPTOR_INIT(USB_AUDIO_CONFIG_DESC_SIZ, 0x03, 0x01,
                               USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    AUDIO_AC_DESCRIPTOR_INIT(0x00, 0x03, AUDIO_AC_SIZ, 0x00, 0x01, 0x02),
    AUDIO_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x01, AUDIO_INTERM_MIC,
                                            IN_CHANNEL_NUM, 0x0000),
    AUDIO_AC_FEATURE_UNIT_DESCRIPTOR_INIT(AUDIO_IN_FU_ID, 0x01, 0x01, 0x03,
                                          0x00),
    AUDIO_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x03, AUDIO_TERMINAL_STREAMING,
            AUDIO_IN_FU_ID),
    AUDIO_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x04, AUDIO_TERMINAL_STREAMING,
                                            OUT_CHANNEL_NUM, 0x0000),
    AUDIO_AC_FEATURE_UNIT_DESCRIPTOR_INIT(AUDIO_OUT_FU_ID, 0x04, 0x01, 0x03,
                                          0x00),
    AUDIO_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x06, AUDIO_OUTTERM_SPEAKER,
            AUDIO_OUT_FU_ID),
    AUDIO_AS_DESCRIPTOR_INIT(
        0x01, 0x04, OUT_CHANNEL_NUM, AUDIO_SPEAKER_FRAME_SIZE_BYTE,
        AUDIO_SPEAKER_RESOLUTION_BIT, AUDIO_OUT_EP, 0x09, AUDIO_OUT_PACKET,
        EP_INTERVAL_FS, AUDIO_SAMPLE_FREQ_3B(AUDIO_SPEAKER_FREQ)),
    AUDIO_AS_DESCRIPTOR_INIT(
        0x02, 0x03, IN_CHANNEL_NUM, AUDIO_MIC_FRAME_SIZE_BYTE,
        AUDIO_MIC_RESOLUTION_BIT, AUDIO_IN_EP, 0x05, AUDIO_IN_PACKET,
        EP_INTERVAL_FS, AUDIO_SAMPLE_FREQ_3B(AUDIO_MIC_FREQ)),
};

static const uint8_t device_quality_descriptor[] =
{
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, 0x01),
};

static const char *string_descriptors[] =
{
    (const char[]){0x09, 0x04}, /* Langid */
    "SiFli",                    /* Manufacturer */
    "SiFli UAC DEMO",           /* Product */
    "2025083018",               /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return config_descriptor_fs;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (sizeof(string_descriptors) / sizeof(char *)))
    {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor audio_v1_descriptor =
{
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

enum uac_msg_type
{
    UAC_MSG_MIC_OPEN,
    UAC_MSG_MIC_CLOSE,
    UAC_MSG_SPEAKER_OPEN,
    UAC_MSG_SPEAKER_CLOSE,
    UAC_MSG_CONTROL_SYNC,
    UAC_MSG_SPEAKER_DATA,
};

struct uac_msg
{
    enum uac_msg_type type;
    rt_uint32_t nbytes;
    rt_uint32_t stream_id;
    rt_uint8_t buf_idx;
};

#define UAC_MQ_DEPTH 16
#define UAC_SPEAKER_BUF_COUNT 4
#define UAC_SPEAKER_BUF_INVALID 0xffu
#define UAC_MQ_MSG_SIZE RT_ALIGN(sizeof(struct uac_msg), RT_ALIGN_SIZE)
#define UAC_MQ_POOL_SIZE                                                      \
    ((UAC_MQ_MSG_SIZE + sizeof(void *)) * UAC_MQ_DEPTH)
#define UAC_MQ_POOL_WORDS                                                     \
    ((UAC_MQ_POOL_SIZE + sizeof(rt_uint32_t) - 1) / sizeof(rt_uint32_t))

struct uac_context
{
    struct rt_messagequeue mq;
    rt_uint32_t mq_pool[UAC_MQ_POOL_WORDS];
    struct rt_semaphore mic_tx_sem;

    volatile bool mic_tx_active;
    volatile bool speaker_rx_active;
    volatile bool mic_target_open;
    volatile bool speaker_target_open;
    volatile bool mic_force_close_pending;
    volatile bool speaker_force_close_pending;
    volatile bool speaker_out_armed;
    volatile bool control_resync_pending;
    volatile rt_uint8_t speaker_armed_buf;
    volatile rt_uint8_t speaker_pending_mask;
    volatile rt_uint32_t speaker_stream_id;

    audio_client_t mic_client;
    audio_client_t speaker_client;

    uint32_t mic_sample_rate;
    uint32_t speaker_sample_rate;
};

struct uac_usb_buffers
{
    uint8_t speaker[UAC_SPEAKER_BUF_COUNT][AUDIO_OUT_PACKET];
};

static struct uac_context g_uac =
{
    .speaker_armed_buf = UAC_SPEAKER_BUF_INVALID,
    .mic_sample_rate = AUDIO_MIC_FREQ,
    .speaker_sample_rate = AUDIO_SPEAKER_FREQ,
};

/* USB DMA buffer must stay in non-cacheable memory. */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static struct uac_usb_buffers
g_uac_usb_buffers;

static struct usbd_interface intf0;
static struct usbd_interface intf1;
static struct usbd_interface intf2;

static void usbd_audio_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes);
static void usbd_audio_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes);
static struct usbd_endpoint audio_in_ep = {.ep_cb = usbd_audio_in_callback,
           .ep_addr = AUDIO_IN_EP
};
static struct usbd_endpoint audio_out_ep = {.ep_cb = usbd_audio_out_callback,
           .ep_addr = AUDIO_OUT_EP
};

static void uac_mark_control_resync(void)
{
    g_uac.control_resync_pending = true;
}

static rt_err_t uac_post_msg(enum uac_msg_type type, rt_uint32_t nbytes,
                             rt_uint32_t stream_id, rt_uint8_t buf_idx)
{
    struct uac_msg msg =
    {
        .type = type,
        .nbytes = nbytes,
        .stream_id = stream_id,
        .buf_idx = buf_idx,
    };
    rt_err_t ret = rt_mq_send(&g_uac.mq, &msg, sizeof(msg));

    if ((ret != RT_EOK) && (type != UAC_MSG_SPEAKER_DATA))
    {
        uac_mark_control_resync();
    }

    return ret;
}

static void uac_request_control_sync(void)
{
    uac_mark_control_resync();
    uac_post_msg(UAC_MSG_CONTROL_SYNC, 0, 0, UAC_SPEAKER_BUF_INVALID);
}

static rt_uint8_t speaker_reserve_read_buffer(void)
{
    rt_base_t level;
    rt_uint8_t buf_idx;

    level = rt_hw_interrupt_disable();
    if (!g_uac.speaker_rx_active || g_uac.speaker_out_armed)
    {
        rt_hw_interrupt_enable(level);
        return UAC_SPEAKER_BUF_INVALID;
    }

    for (buf_idx = 0; buf_idx < UAC_SPEAKER_BUF_COUNT; buf_idx++)
    {
        if ((g_uac.speaker_pending_mask & (1u << buf_idx)) == 0)
        {
            break;
        }
    }

    if (buf_idx >= UAC_SPEAKER_BUF_COUNT)
    {
        rt_hw_interrupt_enable(level);
        return UAC_SPEAKER_BUF_INVALID;
    }

    g_uac.speaker_out_armed = true;
    g_uac.speaker_armed_buf = buf_idx;
    rt_hw_interrupt_enable(level);

    return buf_idx;
}

static bool speaker_take_pending_buffer(rt_uint8_t buf_idx)
{
    rt_base_t level;
    bool pending;

    if (buf_idx >= UAC_SPEAKER_BUF_COUNT)
    {
        return false;
    }

    level = rt_hw_interrupt_disable();
    pending = (g_uac.speaker_pending_mask & (1u << buf_idx)) != 0;
    if (pending)
    {
        g_uac.speaker_pending_mask &= ~(1u << buf_idx);
    }
    rt_hw_interrupt_enable(level);

    return pending;
}

static void speaker_reset_usb_state(bool bump_stream)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    g_uac.speaker_rx_active = false;
    g_uac.speaker_out_armed = false;
    g_uac.speaker_armed_buf = UAC_SPEAKER_BUF_INVALID;
    g_uac.speaker_pending_mask = 0;
    if (bump_stream)
    {
        g_uac.speaker_stream_id++;
    }
    rt_hw_interrupt_enable(level);
}

static void speaker_arm_read_if_needed(uint8_t busid)
{
    int ret;
    rt_uint8_t buf_idx;

    buf_idx = speaker_reserve_read_buffer();
    if (buf_idx == UAC_SPEAKER_BUF_INVALID)
    {
        return;
    }

    ret = usbd_ep_start_read(busid, AUDIO_OUT_EP,
                             g_uac_usb_buffers.speaker[buf_idx],
                             AUDIO_OUT_PACKET);
    if (ret != 0)
    {
        rt_base_t level = rt_hw_interrupt_disable();
        g_uac.speaker_out_armed = false;
        g_uac.speaker_armed_buf = UAC_SPEAKER_BUF_INVALID;
        rt_hw_interrupt_enable(level);
    }
}

static struct audio_entity_info audio_entity_table[] =
{
    {
        .bEntityId = AUDIO_IN_FU_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
        .ep = AUDIO_IN_EP
    },
    {
        .bEntityId = AUDIO_OUT_FU_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
        .ep = AUDIO_OUT_EP
    },
};

/* extern function definition */
static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;

    switch (event)
    {
    case USBD_EVENT_RESET:
    case USBD_EVENT_DISCONNECTED:
        g_uac.mic_target_open = false;
        g_uac.speaker_target_open = false;
        g_uac.mic_force_close_pending = true;
        g_uac.speaker_force_close_pending = true;
        g_uac.mic_tx_active = false;
        speaker_reset_usb_state(true);
        uac_request_control_sync();
        break;
    default:
        break;
    }
}

void usbd_audio_set_volume(uint8_t busid, uint8_t ep, uint8_t ch, int volume)
{
    (void)busid;
    (void)ch;

    if (volume < -100)
        volume = -100;
    if (volume > 0)
        volume = 0;

    uint8_t server_volume = (uint8_t)((volume + 100) * AUDIO_MAX_VOLUME / 100);

    if (ep == AUDIO_IN_EP)
    {
        audio_server_set_public_volume(server_volume);
    }
    else if (ep == AUDIO_OUT_EP)
    {
        audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, server_volume);
    }
}

int usbd_audio_get_volume(uint8_t busid, uint8_t ep, uint8_t ch)
{
    (void)busid;
    (void)ch;

    uint8_t server_volume = 0;

    if (ep == AUDIO_IN_EP)
    {
        server_volume =
            audio_server_get_private_volume(AUDIO_TYPE_LOCAL_RECORD);
    }
    else if (ep == AUDIO_OUT_EP)
    {
        server_volume = audio_server_get_private_volume(AUDIO_TYPE_LOCAL_MUSIC);
    }

    int host_volume = (server_volume * 100) / AUDIO_MAX_VOLUME - 100;

    return host_volume;
}

void usbd_audio_set_mute(uint8_t busid, uint8_t ep, uint8_t ch, bool mute)
{
    (void)busid;
    (void)ch;
    if (ep == AUDIO_IN_EP)
    {
        audio_server_set_public_mic_mute(mute);
    }
    else if (ep == AUDIO_OUT_EP)
    {
        audio_server_set_public_speaker_mute(mute);
    }
}

bool usbd_audio_get_mute(uint8_t busid, uint8_t ep, uint8_t ch)
{
    (void)busid;
    (void)ch;
    if (ep == AUDIO_IN_EP)
    {
        return audio_server_get_public_mic_mute();
    }
    else if (ep == AUDIO_OUT_EP)
    {
        return audio_server_get_public_speaker_mute();
    }
    return false;
}

void usbd_audio_set_sampling_freq(uint8_t busid, uint8_t ep,
                                  uint32_t sampling_freq)
{
    (void)busid;
    if (ep == AUDIO_IN_EP)
    {
        g_uac.mic_sample_rate = sampling_freq;
    }
    else if (ep == AUDIO_OUT_EP)
    {
        g_uac.speaker_sample_rate = sampling_freq;
    }
}

uint32_t usbd_audio_get_sampling_freq(uint8_t busid, uint8_t ep)
{
    (void)busid;
    if (ep == AUDIO_IN_EP)
    {
        return g_uac.mic_sample_rate;
    }
    else if (ep == AUDIO_OUT_EP)
    {
        return g_uac.speaker_sample_rate;
    }
    return 0;
}

/**
 * @brief Record audio callback - sends audio data to USB in chunks
 *
 * @param cmd Callback command (as_callback_cmd_data_coming for new data)
 * @param callback_userdata User data pointer (unused)
 * @param reserved Pointer to audio_server_coming_data_t with audio data
 * @return 0 on success, -1 on timeout
 */
static int audio_callback_mic(audio_server_callback_cmt_t cmd,
                              void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;

        if (!g_uac.mic_tx_active)
        {
            return 0;
        }

        const uint32_t chunk_size = AUDIO_IN_PACKET;

        const uint32_t total_chunks = p->data_len / chunk_size;
        uint8_t *data_ptr = (uint8_t *)p->data;

        for (uint32_t i = 0; i < total_chunks; i++)
        {
            /* Send the current data chunk */
            if (usbd_ep_start_write(0, AUDIO_IN_EP,
                                    data_ptr + (i * chunk_size),
                                    chunk_size) != 0)
            {
                return -1;
            }

            rt_err_t result =
                rt_sem_take(&g_uac.mic_tx_sem, rt_tick_from_millisecond(10));
            if (result != RT_EOK)
            {
                return -1;
            }
        }
    }

    return 0;
}

static int audio_callback_speaker(audio_server_callback_cmt_t cmd,
                                  void *callback_userdata, uint32_t reserved)
{
    (void)cmd;
    (void)callback_userdata;
    (void)reserved;

    return 0;
}

void usbd_audio_open(uint8_t busid, uint8_t intf)
{
    (void)busid;

    if (intf == AUDIO_MIC_INTF)
    {
        g_uac.mic_target_open = true;
        uac_post_msg(UAC_MSG_MIC_OPEN, 0, 0, UAC_SPEAKER_BUF_INVALID);
    }
    else if (intf == AUDIO_SPEAKER_INTF)
    {
        g_uac.speaker_target_open = true;
        uac_post_msg(UAC_MSG_SPEAKER_OPEN, 0, 0, UAC_SPEAKER_BUF_INVALID);
    }
}

void usbd_audio_close(uint8_t busid, uint8_t intf)
{
    (void)busid;

    if (intf == AUDIO_MIC_INTF)
    {
        g_uac.mic_target_open = false;
        g_uac.mic_tx_active = false;
        uac_post_msg(UAC_MSG_MIC_CLOSE, 0, 0, UAC_SPEAKER_BUF_INVALID);
    }
    else if (intf == AUDIO_SPEAKER_INTF)
    {
        g_uac.speaker_target_open = false;
        speaker_reset_usb_state(true);
        uac_post_msg(UAC_MSG_SPEAKER_CLOSE, 0, 0, UAC_SPEAKER_BUF_INVALID);
    }
}

static void audio_device_init(audio_parameter_t *param, uint8_t bits_per_sample,
                              uint8_t channel_num, uint32_t samplerate,
                              uint32_t write_cache_size,
                              uint32_t read_cache_size)
{
    rt_memset(param, 0, sizeof(audio_parameter_t));

    param->write_bits_per_sample = bits_per_sample;
    param->write_channnel_num = channel_num;
    param->write_samplerate = samplerate;
    param->write_cache_size = write_cache_size;

    param->read_bits_per_sample = bits_per_sample;
    param->read_channnel_num = channel_num;
    param->read_samplerate = samplerate;
    param->read_cache_size = read_cache_size;
}

static void uac_open_mic(void)
{
    if (g_uac.mic_client || !g_uac.mic_target_open)
    {
        return;
    }

    rt_sem_control(&g_uac.mic_tx_sem, RT_IPC_CMD_RESET, RT_NULL);

    audio_parameter_t mic_param;
    audio_device_init(&mic_param, 16, 1, 16000, 32, 320);

    g_uac.mic_client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX,
                                  &mic_param, audio_callback_mic, NULL);
    g_uac.mic_tx_active = (g_uac.mic_client != NULL);
}

static void uac_close_mic(void)
{
    g_uac.mic_tx_active = false;
    if (g_uac.mic_client)
    {
        audio_close(g_uac.mic_client);
        g_uac.mic_client = 0;
    }
}

static void uac_open_speaker(uint8_t busid)
{
    if (!g_uac.speaker_target_open)
    {
        return;
    }

    if (g_uac.speaker_client)
    {
        g_uac.speaker_rx_active = true;
        speaker_arm_read_if_needed(busid);
        return;
    }

    audio_parameter_t speaker_param;
    audio_device_init(&speaker_param, 16, 1, 16000, AUDIO_OUT_PACKET, 320);

    g_uac.speaker_client =
        audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &speaker_param,
                   audio_callback_speaker, NULL);

    if (g_uac.speaker_client)
    {
        g_uac.speaker_rx_active = true;
        speaker_arm_read_if_needed(busid);
    }
    else
    {
        speaker_reset_usb_state(false);
    }
}

static void uac_close_speaker(void)
{
    speaker_reset_usb_state(false);
    if (g_uac.speaker_client)
    {
        audio_close(g_uac.speaker_client);
        g_uac.speaker_client = 0;
    }
}

static void uac_apply_targets(uint8_t busid)
{
    bool close_mic = g_uac.mic_force_close_pending || !g_uac.mic_target_open;
    bool close_speaker = g_uac.speaker_force_close_pending ||
                         !g_uac.speaker_target_open;

    g_uac.mic_force_close_pending = false;
    g_uac.speaker_force_close_pending = false;

    if (close_mic)
    {
        uac_close_mic();
    }

    if (close_speaker)
    {
        uac_close_speaker();
    }

    if (g_uac.mic_target_open)
    {
        uac_open_mic();
    }

    if (g_uac.speaker_target_open)
    {
        uac_open_speaker(busid);
    }
}

static void uac_apply_resync_if_needed(uint8_t busid)
{
    if (g_uac.control_resync_pending)
    {
        g_uac.control_resync_pending = false;
        uac_apply_targets(busid);
    }
}

void audio_v1_init(uint8_t busid, uint32_t reg_base)
{
    // Initialize the semaphore for TX synchronization
    // Initial value is 0, max value is 1 (binary semaphore)
    rt_sem_init(&g_uac.mic_tx_sem, "mic_tx_sem", 0, RT_IPC_FLAG_FIFO);

    rt_mq_init(&g_uac.mq, "uac_mq", &g_uac.mq_pool[0],
               sizeof(struct uac_msg), sizeof(g_uac.mq_pool),
               RT_IPC_FLAG_FIFO);

    usbd_desc_register(busid, &audio_v1_descriptor);
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf0, AUDIO_VERSION,
                       audio_entity_table, 2));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf1, AUDIO_VERSION,
                       audio_entity_table, 2));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf2, AUDIO_VERSION,
                       audio_entity_table, 2));
    usbd_add_endpoint(busid, &audio_in_ep);
    usbd_add_endpoint(busid, &audio_out_ep);

    usbd_initialize(busid, reg_base, usbd_event_handler);
}

void audio_v1_task(uint8_t busid)
{
    struct uac_msg msg;

    while (1)
    {
        if (rt_mq_recv(&g_uac.mq, &msg, sizeof(msg),
                       RT_WAITING_FOREVER) == RT_EOK)
        {
            switch (msg.type)
            {
            case UAC_MSG_MIC_OPEN:
                uac_open_mic();
                break;

            case UAC_MSG_MIC_CLOSE:
                uac_close_mic();
                break;

            case UAC_MSG_SPEAKER_OPEN:
                uac_open_speaker(busid);
                break;

            case UAC_MSG_SPEAKER_DATA:
                {
                    bool pending = speaker_take_pending_buffer(msg.buf_idx);

                    if (pending && g_uac.speaker_rx_active &&
                            g_uac.speaker_client &&
                            (msg.stream_id == g_uac.speaker_stream_id) &&
                            (msg.nbytes > 0))
                    {
                        audio_write(g_uac.speaker_client,
                                    g_uac_usb_buffers.speaker[msg.buf_idx],
                                    msg.nbytes);
                    }
                }
                speaker_arm_read_if_needed(busid);
                break;

            case UAC_MSG_SPEAKER_CLOSE:
                uac_close_speaker();
                break;

            case UAC_MSG_CONTROL_SYNC:
                g_uac.control_resync_pending = false;
                uac_apply_targets(busid);
                break;

            default:
                break;
            }

            uac_apply_resync_if_needed(busid);
        }
    }
}

/* static function definition */
static void usbd_audio_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;

    // This callback is in an interrupt context or a high-priority USB task.
    // Release the semaphore to signal that the transfer is complete.
    rt_sem_release(&g_uac.mic_tx_sem);
}

static void usbd_audio_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    rt_base_t level;
    rt_uint8_t buf_idx;
    rt_uint32_t stream_id;

    (void)ep;

    level = rt_hw_interrupt_disable();
    buf_idx = g_uac.speaker_armed_buf;
    g_uac.speaker_out_armed = false;
    g_uac.speaker_armed_buf = UAC_SPEAKER_BUF_INVALID;

    if (g_uac.speaker_rx_active && buf_idx < UAC_SPEAKER_BUF_COUNT)
    {
        g_uac.speaker_pending_mask |= (1u << buf_idx);
        stream_id = g_uac.speaker_stream_id;
        rt_hw_interrupt_enable(level);

        if (uac_post_msg(UAC_MSG_SPEAKER_DATA, nbytes, stream_id,
                         buf_idx) != RT_EOK)
        {
            speaker_take_pending_buffer(buf_idx);
        }
        speaker_arm_read_if_needed(busid);
    }
    else
    {
        rt_hw_interrupt_enable(level);
    }
}
