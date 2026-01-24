#ifndef __AUD_COD_I2S_H__
#define __AUD_COD_I2S_H__

enum AUDIO_SAMPLE_RATE
{
    AUDIO_SAMPLE_RATE_8K,
    AUDIO_SAMPLE_RATE_16K,
};

int audio_codec_open(void);
int audio_codec_close(void);
int audio_codec_set_volume(int vol); // -36 ~ 0
rt_uint8_t *get_audio_adc_buf(void);
int audio_prc_open(void);
int audio_prc_close(void);

#define I2S_WORK_MODE (1) // 1:slave mode, 0:master mode
int i2s_device_open(void);
int i2s_device_close(void);

void audio_transfer_entry(void *parameter);
#endif