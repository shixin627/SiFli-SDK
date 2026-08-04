/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#include "time.h"
#include <rtdevice.h>
#include "audio_server.h"

/* User code start from here --------------------------------------------------------*/
#define RECORD_SECONDS         (5)
#define RECORD_SAMPLE_RATE     (16000)
#define RECORD_CHANNELS        (1)
#define RECORD_BITS_PER_SAMPLE (16)
#define RECORD_BUFFER_SIZE     (RECORD_SAMPLE_RATE * RECORD_CHANNELS * (RECORD_BITS_PER_SAMPLE / 8) * RECORD_SECONDS)
#define AUDIO_WRITE_CACHE_SIZE (4096)
#define AUDIO_READ_CACHE_SIZE  (2048)
static uint8_t record_mem[RECORD_BUFFER_SIZE];
static int g_record_mem_offset = 0;

/* Semaphore used to wait aes interrupt. */
static rt_sem_t g_audio_sem;
/* pcm buffer */
static uint16_t *s_pcm = NULL;
/* read offset */
static int g_pcm_mem_offset = 0;
/* audio client */
static audio_client_t g_client;

/**
 * @brief Common initialization.
 */
static rt_err_t comm_init(void)
{
    g_audio_sem = rt_sem_create("audio_sem", 0, RT_IPC_FLAG_FIFO);
    return RT_EOK;
}

/**
 * @brief Audio callback function for recording.
 *        Save data to memory buffer.
 */
static int audio_callback_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int wr_len = 0;

    (void)callback_userdata;

    rt_kprintf("[RECORD]%s cmd %d\n", __func__, cmd);
    if (cmd == as_callback_cmd_data_coming)
    {
        /* get recording pcm data. */
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        /* pcm data left shift 4 bits to increase volume. */
        //auido_gain_pcm((int16_t *)p->data, p->data_len, 4);
        /* save recording pcm data to memory buffer. */
        if (g_record_mem_offset + p->data_len <= sizeof(record_mem))
        {
            memcpy(&record_mem[g_record_mem_offset], p->data, p->data_len);
            wr_len = p->data_len;
            g_record_mem_offset += wr_len;
        }
        else
        {
            rt_kprintf("[RECORD]%s buffer full! offset:%d, data_len:%d\n", __func__, g_record_mem_offset, p->data_len);
            wr_len = 0;
        }
        // rt_kprintf("[RECORD]%s recording save %d bytes.\n", __func__, wr_len);
    }

    return 0;
}

/**
 * @brief Example for recording.
 *        Recording for 5 seconds.
 */
static void start_recording(void)
{
    rt_kprintf("[RECORD]%s\n", __func__);

    audio_parameter_t param = {0};

    /* Reset memory buffer offset. */
    g_record_mem_offset = 0;

    /* audio parameters. */
    param.write_bits_per_sample = RECORD_BITS_PER_SAMPLE;
    param.write_channnel_num = RECORD_CHANNELS;
    param.write_samplerate = RECORD_SAMPLE_RATE; /* 16k sampling rate. */
    param.write_cache_size = AUDIO_WRITE_CACHE_SIZE;  /* write cache size */

    param.read_bits_per_sample = RECORD_BITS_PER_SAMPLE;
    param.read_channnel_num = RECORD_CHANNELS;
    param.read_samplerate = RECORD_SAMPLE_RATE;  /* 16k sampling rate. */
    param.read_cache_size = AUDIO_READ_CACHE_SIZE;   /* read cache size */

    /* Start recording. */
    audio_client_t client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &param, audio_callback_record, NULL);
    RT_ASSERT(client);

    /* Recording for 5 seconds.  */
    rt_thread_mdelay(RECORD_SECONDS * 1000);

    rt_kprintf("[RECORD]close audio client.\n");
    /* Stop recording. */
    audio_close(client);
}

/**
 * @brief Audio callback function for recording play.
 *        Read from memory buffer.
 */
static int audio_callback_play(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int rd_len = 0;
    int wr_len = 0;
    static uint8_t read_finish = 0;
    int read_size = AUDIO_WRITE_CACHE_SIZE / 2;

    (void)callback_userdata;
    (void)reserved;

    rt_kprintf("[RECORD]%s cmd: %d\n", __func__, cmd);
    /* Feed audio data from the memory buffer. */
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        if (s_pcm && g_client)
        {
            /* Check if there's more data to read from memory buffer. */
            if (g_pcm_mem_offset >= g_record_mem_offset)
            {
                read_finish = 1;
                rt_kprintf("[RECORD]%s read finish. offset:%d, total:%d\n", __func__, g_pcm_mem_offset, g_record_mem_offset);
            }
            else
            {
                /* Calculate how much data we can read. */
                int remaining = g_record_mem_offset - g_pcm_mem_offset;
                rd_len = (remaining < read_size) ? remaining : read_size;

                /* Read data from memory buffer. */
                memcpy((void *)s_pcm, &record_mem[g_pcm_mem_offset], rd_len);

                wr_len = audio_write(g_client, (uint8_t *)s_pcm, rd_len);
                g_pcm_mem_offset += wr_len;
                // rt_kprintf("%s audio write %d bytes. \n", __func__, wr_len);
            }
        }
    }

    if (read_finish)
    {
        /* Notify to exit playing. */
        rt_kprintf("[RECORD]%s ready to exit play.\n", __func__);
        rt_sem_release(g_audio_sem);
    }

    return 0;
}

/**
 * @brief Example for recording play.
 *        Read from memory buffer and play.
 */
static void recording_play(void)
{
    rt_kprintf("[RECORD]%s\n", __func__);

    /* Reset read offset. */
    g_pcm_mem_offset = 0;
    rt_kprintf("[RECORD]memory buffer size %d bytes.\n", g_record_mem_offset);

    /* audio parameters. */
    audio_parameter_t param = {0};
    param.write_bits_per_sample = RECORD_BITS_PER_SAMPLE;
    param.write_channnel_num = RECORD_CHANNELS;
    param.write_samplerate = RECORD_SAMPLE_RATE; /* 16k sampling rate. */
    param.write_cache_size = AUDIO_WRITE_CACHE_SIZE;  /* write cache size */
    param.read_bits_per_sample = RECORD_BITS_PER_SAMPLE;
    param.read_channnel_num = RECORD_CHANNELS;
    param.read_samplerate = RECORD_SAMPLE_RATE;  /* 16k sampling rate. */
    param.read_cache_size = AUDIO_READ_CACHE_SIZE;   /* read cache size */
    /* Open audio client. */
    g_client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &param, audio_callback_play, NULL);
    RT_ASSERT(g_client >= 0);

    /* Read data from memory buffer.
     * Read AUDIO_WRITE_CACHE_SIZE for the first time, then read in half afterward.
     */
    rt_kprintf("[RECORD]audio write %d bytes. \n", AUDIO_WRITE_CACHE_SIZE);
    s_pcm = (uint16_t *)rt_malloc(AUDIO_WRITE_CACHE_SIZE);
    RT_ASSERT(s_pcm);

    /* Read first chunk from memory buffer. */
    int read_size = (g_record_mem_offset < AUDIO_WRITE_CACHE_SIZE) ? g_record_mem_offset : AUDIO_WRITE_CACHE_SIZE;
    if (read_size > 0)
    {
        memcpy((void *)s_pcm, record_mem, read_size);
        int wr_len = audio_write(g_client, (uint8_t *)s_pcm, read_size);
        g_pcm_mem_offset += wr_len;
    }

    /* Wait for playback to complete. */
    rt_sem_take(g_audio_sem, RT_WAITING_FOREVER);
    uint32_t cache_ms = 0;
    audio_ioctl(g_client, AUDIO_IOCTL_FLUSH_TIME_MS, &cache_ms);
    rt_thread_mdelay(cache_ms);

    /* Close audio client. */
    rt_kprintf("[RECORD]close audio client.\n");
    audio_close(g_client);
    free(s_pcm);
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("\n[RECORD]Record Example.\n");

    /* Semaphore initialization. */
    comm_init();

    /* Recording and save to memory buffer. */
    start_recording();
    /* Play recording from memory buffer. */
    recording_play();

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }

    return 0;
}
