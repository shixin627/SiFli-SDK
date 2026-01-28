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
#include "webrtc/common_audio/vad/include/webrtc_vad.h"

#ifdef ENABLE_OPUS_ENCODER
#include "opus.h"
#include "opus_types.h"
#endif

#define DBG_TAG "bloc.v2t"
#define DBG_LVL DBG_LOG
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

static bool audio_subscribed = false;

/* Static variables */
static bool _vad_status = false;
static folder_t _recorder_folder = {0, NULL};
static int rec_fd = -1;
static uint32_t record_bytes;

/* Opus encoder for recording */
#ifdef ENABLE_OPUS_ENCODER
static OpusEncoder *rec_opus_encoder = NULL;
static uint8_t rec_opus_output[OPUS_REC_MAX_PACKET] __attribute__((aligned(4)));
static int16_t rec_pcm_buffer[OPUS_REC_FRAME_SIZE];
static uint16_t rec_pcm_buffer_idx = 0;
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

/* ========== 語音辨識自動停止邏輯區塊 ========== */
static uint32_t last_vad_trigger_tick = 0;
/* Debounce timer for checking if user is speaking */
static bool is_user_speaking = false;
static bool is_user_speaking_to_ai = false;
static rt_timer_t speaking_debounce_timer = NULL;
static bool is_voice_recognition_notified = false;
static bool is_voice_recognition_notified_from_mouse = false;
/* Debounce timer for V2T completion */
static bool is_v2t_completed = false;
static rt_timer_t v2t_complete_debounce_timer = NULL;

static bool voiceRecordIntent = false;
static bool isVoiceRecording = false;
static chat_message_t chat_message_list[10];
static uint8_t chat_message_count = 0;
void add_chat_message(bool is_me, const char *text)
{
    if (chat_message_count >= 10)
    {
        return;
    }
    chat_message_list[chat_message_count].from_me = is_me;
    strcpy(chat_message_list[chat_message_count].content, text);
    chat_message_count++;
}

void set_voice_recognition_notified_from_mouse(bool status)
{
    is_voice_recognition_notified_from_mouse = status;
}

void clear_chat_message(void)
{
    chat_message_count = 0;
}

chat_message_t *get_chat_message_list(void)
{
    return chat_message_list;
}

uint8_t get_chat_message_count(void)
{
    return chat_message_count;
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
    L1SendData data = {.event = L1SEND_RETURN_USER_SPEAKING_STATE,
                       .res.status = status};
    L1_send_event(data);
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
    }
    else
    {
        rt_timer_stop(speaking_debounce_timer);
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
    if (speaking_debounce_timer)
    {
        is_user_speaking = false;
        LOG_D("SPEAK DEGUG TEST3");
        rt_timer_stop(speaking_debounce_timer);
        speaking_debounce_timer = NULL;
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
extern bool get_is_open_app_list_ai(void);
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
            if (get_gravity_position() == GRAVITY_POSITION_AI ||
                is_at_ai_interface() || is_at_app_list() ||
                is_at_speech_interface())
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
                rt_uint32_t time_left = 500;
                LOG_D("SPEAK DEGUG TEST4");
                rt_timer_stop(speaking_debounce_timer);
                rt_timer_control(speaking_debounce_timer,
                                 RT_TIMER_CTRL_SET_TIME, &time_left);
                rt_timer_start(speaking_debounce_timer);
                is_voice_recognition_notified_from_mouse = false;
            }
            speaking_debounce_timer_start(500);
        }
        // LOG_D("status:%d,is_user_speaking:%d,check_if_ai_processing:%d",
        last_vad_trigger_tick = current_tick;
    }
}

/**
 * @brief 語音辨識結果通知處理
 *
 * 當收到語音辨識文字結果時：
 * 1. 設置已收到語音辨識結果標記
 * 2. 調整計時器為1.5秒（縮短等待時間）
 * 3. 重新啟動計時器
 *
 * @param text_len 辨識文字長度
 */
void notify_voice_recognition(uint16_t text_len)
{
    if (text_len == 0)
    {
        return;
    }

    if (is_user_speaking)
    {
        if (!is_voice_recognition_notified)
        {
            // 收到第一個語音辨識結果後，縮短計時器時間為0.5秒
            rt_uint32_t time_left = 500;
            rt_timer_stop(speaking_debounce_timer);
            rt_timer_control(speaking_debounce_timer, RT_TIMER_CTRL_SET_TIME,
                             &time_left);
            rt_timer_start(speaking_debounce_timer);
            is_voice_recognition_notified = true;
        }
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
    speech_coding = 0;
    notify_user_speaking_intent(intent);
    extern void start_ble_rssi_checker(void);
    start_ble_rssi_checker();
    skaiwatch_ble_set_performance(true);
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
    extern void stop_ble_rssi_checker(void);
    stop_ble_rssi_checker();
    skaiwatch_ble_set_performance(false);
    reset_ai_open_mic();
    // 重置語音辨識狀態標記
    is_voice_recognition_notified = false;
    _vad_status = false;
}

/* ========== 語音辨識自動停止邏輯流程總結 ========== */
/*
 * 整體邏輯流程：
 *
 * 1. 語音辨識啟動階段 (start_voice_recognition)：
 *    - 初始化狀態標記：is_voice_recognition_notified = false
 *    - 記錄當前時間：last_vad_trigger_tick = rt_tick_get()
 *    - 設置高性能模式和感測器
 *
 * 2. 自動停止計時器管理：
 *    - 預設計時器：3秒超時
 *    - 收到語音辨識結果後：調整為1.5秒超時
 *    - 計時器回調：觸發 voice_provider.auto_stop_listening()
 *
 * 3. VAD狀態處理 (notify_vad_status)：
 *    - 檢測到語音：重新啟動計時器，更新last_vad_trigger_tick
 *    - 無語音活動：檢查自動停止條件
 *
 * 4. 自動停止條件 (所有條件同時滿足)：
 *    - is_voice_recognition_notified == true (已收到語音辨識結果)
 *    - current_tick - get_last_refresh_input_message_tick() > 5000
 * (距上次輸入更新>5秒)
 *    - current_tick - last_vad_trigger_tick > 500 (距上次VAD觸發>0.5秒)
 *    - !check_if_ai_processing() (不在AI處理中)
 *
 * 5. 語音辨識結果處理 (notify_voice_recognition)：
 *    - 設置 is_voice_recognition_notified = true
 *    - 調整計時器為1.5秒
 *    - 重新啟動計時器
 *
 * 6. 語音辨識停止階段 (stop_voice_recognition)：
 *    - 停止自動停止計時器
 *    - 重置所有狀態標記
 *    - 關閉感測器和高性能模式
 */

int free_recorder_folder(folder_t *folder)
{
    if (folder->files != NULL)
    {
        // Free each file name buffer first
        for (uint8_t i = 0; i < folder->file_num; i++)
        {
            if (folder->files[i].name != NULL)
            {
                free(folder->files[i].name);
                folder->files[i].name = NULL;
            }
        }

        free(folder->files);
        folder->files = NULL;
        folder->file_num = 0;
        return 0;
    }
    return -1; // 没有内存需要释放
}

folder_t *list_files_in_recorder(void)
{
    DIR *dir;
    struct dirent *entry;
    const int filename_max_length = 64; // Increased from 20 to 64 bytes

    // Open the /recorder directory
    dir = opendir("/recorder");
    if (dir == NULL)
    {
        perror("opendir");
        return NULL;
    }

    free_recorder_folder(&_recorder_folder);

    // Read and print all files in the directory
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type == DT_REG) // Check if the entry is a regular file
        {
            // only show the pcm file with the name of "record"
            if (strncmp(entry->d_name, "record", 6) == 0 &&
                (strstr(entry->d_name, ".pcm") != NULL || 
                 strstr(entry->d_name, ".opus") != NULL))
            {
                LOG_D("File: %s", entry->d_name);
                _recorder_folder.file_num++;
                _recorder_folder.files = (file_t *)realloc(
                    _recorder_folder.files,
                    _recorder_folder.file_num * sizeof(file_t));

                uint16_t name_len = strlen(entry->d_name) + 1;
                _recorder_folder.files[_recorder_folder.file_num - 1].name =
                    (char *)malloc(name_len);
                strcpy(
                    _recorder_folder.files[_recorder_folder.file_num - 1].name,
                    entry->d_name);
                _recorder_folder.files[_recorder_folder.file_num - 1]
                    .name_length = name_len - 1;
            }
        }
    }

    // Close the directory
    closedir(dir);

    return &_recorder_folder;
}

void copy_temp_file_to_recorder(void)
{
    LOG_D("%s", __FUNCTION__);
    char *temp_file = "/recorder/temp.pcm";
    char recorder_file[MAX_RECORD_PATH_LEN];

    time_t now;
    struct tm *tm_info;

    // Get current time
    time(&now);
    tm_info = localtime(&now);

    // Create timestamp filename
    snprintf(recorder_file, sizeof(recorder_file),
             "/recorder/record_%04d%02d%02d_%02d%02d%02d.pcm",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);

    int fd_temp = open(temp_file, O_RDONLY | O_BINARY);
    RT_ASSERT(fd_temp >= 0);

    if (access(recorder_file, 0) == 0)
    {
        LOG_D("recorder_file already exists, delete it first.");
        unlink(recorder_file);
    }
    LOG_D("create the new recorder_file: %s", recorder_file);
    int fd_recorder =
        open(recorder_file, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    RT_ASSERT(fd_recorder >= 0);

    char buf[1024];
    int read_bytes;
    while ((read_bytes = read(fd_temp, buf, sizeof(buf))) > 0)
    {
        if (write(fd_recorder, buf, read_bytes) != read_bytes)
        {
            LOG_E("write file error");
            break;
        }
    }

    close(fd_temp);
    close(fd_recorder);
}

void start_voice_recording(void)
{
    if (app_voice_get_recording_status() == true)
    {
        return;
    }
#ifdef BSP_USING_BLOC_PERIPHERAL
    peripheral_provider.subscribe_audio_mic_sensor(true);
#endif
    record_bytes = 0;
    if (access("/recorder", 0))
    {
        RT_ASSERT(mkdir("/recorder", 0x777) == 0)
        LOG_D("canot find dir recorder, make it.\n");
    }

    char file_path[MAX_RECORD_PATH_LEN];
    time_t now;
    struct tm *tm_info;

    time(&now);
    tm_info = localtime(&now);

#ifdef ENABLE_OPUS_ENCODER
    // Initialize Opus encoder for recording (same as opus example main.c)
    int err;
    rec_opus_encoder = opus_encoder_create(OPUS_REC_SAMPLE_RATE, OPUS_REC_CHANNELS,
                                           OPUS_APPLICATION_VOIP, &err);
    if (err != OPUS_OK || rec_opus_encoder == NULL)
    {
        LOG_E("Failed to create Opus encoder, error=%d\n", err);
        rec_opus_encoder = NULL;
    }
    else
    {
        // Configure encoder (same settings as opus example)
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_VBR(1));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_VBR_CONSTRAINT(1));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_BITRATE(OPUS_REC_BITRATE));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_COMPLEXITY(0));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_LSB_DEPTH(16));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_DTX(0));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_INBAND_FEC(0));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_PACKET_LOSS_PERC(0));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_PREDICTION_DISABLED(0));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_MAX_BANDWIDTH(OPUS_BANDWIDTH_WIDEBAND));
        opus_encoder_ctl(rec_opus_encoder, OPUS_SET_BANDWIDTH(OPUS_AUTO));

        rec_pcm_buffer_idx = 0;
        LOG_D("Opus encoder initialized for recording\n");
    }

    // Use .opus extension when Opus encoding is enabled
    snprintf(file_path, sizeof(file_path),
             "/recorder/record_%04d%02d%02d_%02d%02d%02d.opus",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
#else
    snprintf(file_path, sizeof(file_path),
             "/recorder/record_%04d%02d%02d_%02d%02d%02d.pcm",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
#endif

    LOG_D("[%s] %s\n", __FUNCTION__, file_path);
    rec_fd = open(file_path, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    RT_ASSERT(rec_fd >= 0);

    // Save current recording file path
    strncpy(current_recording_file, file_path, MAX_RECORD_PATH_LEN - 1);
    current_recording_file[MAX_RECORD_PATH_LEN - 1] = '\0';

    app_voice_set_listening_status(true);
    app_voice_set_recording_status(true);
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
    if (rec_opus_encoder == NULL)
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
            opus_int32 encoded_len = opus_encode(rec_opus_encoder,
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
                    LOG_E("audio_record_opus write length err\n");
                    return -1;
                }
                if (write(rec_fd, rec_opus_output, encoded_len) != encoded_len)
                {
                    LOG_E("audio_record_opus write data err\n");
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
#endif

int audio_record_pcm(const void *temp_buf, rt_uint32_t data_len)
{
    if (app_voice_get_recording_status() == false)
    {
        return -1;
    }

#ifdef ENABLE_OPUS_ENCODER
    // Use Opus encoding when available
    if (rec_opus_encoder != NULL)
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
            LOG_E("audio_record_pcm write err\n");
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

#ifdef ENABLE_OPUS_ENCODER
    // Flush remaining PCM samples if any (pad with zeros to complete last frame)
    if (rec_opus_encoder != NULL && rec_pcm_buffer_idx > 0)
    {
        // Pad remaining buffer with zeros
        while (rec_pcm_buffer_idx < OPUS_REC_FRAME_SIZE)
        {
            rec_pcm_buffer[rec_pcm_buffer_idx++] = 0;
        }

        // Encode and write final frame
        opus_int32 encoded_len = opus_encode(rec_opus_encoder,
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

    // Destroy Opus encoder
    if (rec_opus_encoder != NULL)
    {
        opus_encoder_destroy(rec_opus_encoder);
        rec_opus_encoder = NULL;
        rec_pcm_buffer_idx = 0;
        LOG_D("Opus encoder destroyed\n");
    }
#endif

    LOG_D("recorded %d bytes to file: %s", record_bytes, current_recording_file);
    close(rec_fd);
    rec_fd = -1;
}

const char* get_last_recording_file(void)
{
    return current_recording_file;
}

#ifdef ENABLE_OPUS_ENCODER
/**
 * @brief Decode Opus file to PCM file
 *
 * This function reads an Opus file and decodes it to a PCM file.
 * The output PCM file will have the same name but with .pcm extension.
 *
 * @param opus_file_path Path to the input Opus file
 * @param pcm_file_path Output buffer for the decoded PCM file path (must be at least MAX_RECORD_PATH_LEN)
 * @return 0 on success, -1 on failure
 */
int opus_decode_to_pcm(const char *opus_file_path, char *pcm_file_path)
{
    if (opus_file_path == NULL || pcm_file_path == NULL)
    {
        LOG_E("Invalid parameters\n");
        return -1;
    }

    // Generate PCM output file path (replace .opus with .pcm)
    strncpy(pcm_file_path, opus_file_path, MAX_RECORD_PATH_LEN - 1);
    pcm_file_path[MAX_RECORD_PATH_LEN - 1] = '\0';

    char *ext = strstr(pcm_file_path, ".opus");
    if (ext != NULL)
    {
        strcpy(ext, ".pcm");
    }
    else
    {
        // If no .opus extension, just append .pcm
        strncat(pcm_file_path, ".pcm", MAX_RECORD_PATH_LEN - strlen(pcm_file_path) - 1);
    }

    // Open input Opus file
    int opus_fd = open(opus_file_path, O_RDONLY | O_BINARY);
    if (opus_fd < 0)
    {
        LOG_E("Failed to open Opus file: %s\n", opus_file_path);
        return -1;
    }

    // Open output PCM file
    int pcm_fd = open(pcm_file_path, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    if (pcm_fd < 0)
    {
        LOG_E("Failed to create PCM file: %s\n", pcm_file_path);
        close(opus_fd);
        return -1;
    }

    // Create Opus decoder
    int err;
    OpusDecoder *decoder = opus_decoder_create(OPUS_REC_SAMPLE_RATE, OPUS_REC_CHANNELS, &err);
    if (err != OPUS_OK || decoder == NULL)
    {
        LOG_E("Failed to create Opus decoder, error=%d\n", err);
        close(opus_fd);
        close(pcm_fd);
        return -1;
    }

    LOG_D("Decoding %s to %s\n", opus_file_path, pcm_file_path);

    // Buffers for decoding
    uint8_t opus_packet[OPUS_REC_MAX_PACKET];
    int16_t pcm_output[OPUS_REC_FRAME_SIZE];
    uint32_t total_frames = 0;
    uint32_t total_pcm_bytes = 0;

    // Read and decode each packet
    while (1)
    {
        // Read packet length (2 bytes)
        uint16_t pkt_len;
        if (read(opus_fd, &pkt_len, sizeof(pkt_len)) != sizeof(pkt_len))
        {
            // End of file
            break;
        }

        // Validate packet length
        if (pkt_len == 0 || pkt_len > OPUS_REC_MAX_PACKET)
        {
            LOG_E("Invalid packet length: %d\n", pkt_len);
            break;
        }

        // Read Opus packet data
        if (read(opus_fd, opus_packet, pkt_len) != pkt_len)
        {
            LOG_E("Failed to read Opus packet\n");
            break;
        }

        // Decode Opus packet to PCM
        opus_int32 decoded_samples = opus_decode(decoder, opus_packet, pkt_len,
                                                  pcm_output, OPUS_REC_FRAME_SIZE, 0);

        if (decoded_samples > 0)
        {
            // Write PCM data to file
            int pcm_bytes = decoded_samples * sizeof(int16_t);
            if (write(pcm_fd, pcm_output, pcm_bytes) != pcm_bytes)
            {
                LOG_E("Failed to write PCM data\n");
                break;
            }
            total_pcm_bytes += pcm_bytes;
            total_frames++;
        }
        else
        {
            LOG_E("Opus decode failed, ret=%d\n", decoded_samples);
        }
    }

    // Cleanup
    opus_decoder_destroy(decoder);
    close(opus_fd);
    close(pcm_fd);

    LOG_D("Decoded %d frames, total PCM bytes: %d\n", total_frames, total_pcm_bytes);

    return 0;
}
#endif

void start_sync_voice_recording(void)
{
    app_voice_set_recording_intent(true);
    _voice_recording_time = get_current_time();
    LOG_D("start_sync_voice_recording: %d", _voice_recording_time);
    L1SendData data = {.event = L1SEND_RETURN_VOICE_RECORD_INTENT,
                       .res.id = _voice_recording_time};
    L1_send_event(data);
}

void stop_sync_voice_recording(void)
{
    app_voice_set_recording_intent(false);
    LOG_D("stop_sync_voice_recording: %d", _voice_recording_time);
    L1SendData data = {.event = L1SEND_RETURN_VOICE_RECORD_INTENT,
                       .res.id = _voice_recording_time};
    L1_send_event(data);
    _voice_recording_time = 0;
}

char *get_combined_voice2text(void)
{
    snprintf(combined_text, sizeof(combined_text), "%s%s", _v2t_result.text,
             _v2t_result_temp.text);
    return combined_text;
}

void setVoice2TextState(bool state)
{
    _v2t_result.state = state;
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

void copy_string_to_voice2text(voice2text_t *v2t, const char *str)
{
    strcpy(v2t->text, str);
}

void setVoice2Text(char *text)
{
    if (text != NULL)
    {
        copy_string_to_voice2text(&_v2t_result, text);
        notifyVoice2Text();
    }
}

void clearVoice2Text(void)
{
    copy_string_to_voice2text(&_v2t_result, "");
    RT_ASSERT(strlen(_v2t_result.text) == 0);
    copy_string_to_voice2text(&_v2t_result_temp, "");
    RT_ASSERT(strlen(_v2t_result_temp.text) == 0);
    notifyVoice2Text();
}

bool isTextEmpty(void)
{
    return strlen(_v2t_result.text) == 0 && strlen(_v2t_result_temp.text) == 0;
}

static void addVoice2Text(char *text, uint16_t length)
{
    if (text != NULL)
    {
        if (strlen(_v2t_result.text) + length + 1 < sizeof(_v2t_result.text))
        {
            strcat(_v2t_result.text, " ");
            strcat(_v2t_result.text, text);
            notifyVoice2Text();
        }
        else
        {
            LOG_E("Voice2Text buffer overflow");
        }
    }
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
    }
    else
    {
        addVoice2Text(buffer, len);
    }
}

static void insertVoiceText(char *buffer, uint16_t len)
{
    copy_string_to_voice2text(&_v2t_result_temp, "");
    RT_ASSERT(strlen(_v2t_result_temp.text) == 0);
    strcat(_v2t_result_temp.text, buffer);
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
        insertVoiceText(buffer, msgData->length);
        last_sentence_index = sentence_index;
    }
    else
    {
        // LOG_D("[handle_v2t_result]Append text len(%d) to sentence(%d)",
        // msgData->length, sentence_index);
        insertVoiceText(buffer, msgData->length);
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
    if (voice_provider.vad_inst == NULL)
    {
        voice_provider.vad_inst = WebRtcVad_Create();
        if (voice_provider.vad_inst != NULL)
        {
            WebRtcVad_Init(voice_provider.vad_inst);
        }
    }
}

/**
 * @brief Deinitializes the Voice Activity Detection (VAD) module
 *
 * Frees resources allocated for VAD instance
 */
static void vad_deinit(void)
{
    if (voice_provider.vad_inst)
    {
        WebRtcVad_Free(voice_provider.vad_inst);
        voice_provider.vad_inst = NULL;
    }
}

/* ========== 語音辨識狀態管理函數 ========== */

/**
 * @brief Gets the combined voice-to-text result
 * @return Current voice-to-text result string
 */
bool app_voice_get_voice2text_status(void)
{
    return voice2TextStatus;
}

/**
 * @brief Sets the voice-to-text status
 * @param status true to enable voice-to-text, false to disable
 */
void app_voice_set_voice2text_status(bool status)
{
    voice2TextStatus = status;
}

uint8_t app_voice_get_voice2text_intent(void)
{
    return voice2TextIntent;
}
void app_voice_set_voice2text_intent(uint8_t intent)
{
    voice2TextIntent = intent;
}

/**
 * @brief Gets the recording intent status
 * @return true if recording is intended, false otherwise
 */
bool app_voice_get_recording_intent(void)
{
    return voiceRecordIntent;
}

/**
 * @brief Sets the recording intent status
 * @param intent true to set recording intent, false to clear
 */
void app_voice_set_recording_intent(bool intent)
{
    voiceRecordIntent = intent;
}

/**
 * @brief Gets the current recording status
 * @return true if currently recording, false otherwise
 */
bool app_voice_get_recording_status(void)
{
    return isVoiceRecording;
}

/**
 * @brief Sets the recording status
 * @param status true to indicate recording is active, false otherwise
 */
void app_voice_set_recording_status(bool status)
{
    isVoiceRecording = status;
}

/**
 * @brief Gets the current voice recording time
 * @return Pointer to the voice recording time value
 */
uint32_t *app_voice_get_record_time(void)
{
    return &_voice_recording_time;
}

/**
 * @brief Sets the voice recording time
 * @param time New recording time value
 */
void app_voice_set_record_time(uint32_t time)
{
    _voice_recording_time = time;
}

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
                start_voice_recognition(V2T_INTENT_CHAT);

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
                    bool status = false;
                    watch_system_interact(INTERACT_MIC_LISTEN, &status);
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

extern bool get_voice_recognition_started(void);
void back_tap_cb(void)
{
    if (isTextEmpty())
    {
        voice_provider.stop_v2t();
        show_speech_indicator(false);
    }
    else
    {
        if (!get_voice_recognition_started())
        {
            show_speech_indicator(false);
        }
        else
        {
            count_speech_coding();
            clearVoice2Text();
            refresh_ai_chat_input_message("");
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
