/*********************************************************************************************************
 *               Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     bloc_v2t.h
 * @brief    Business logic of voice to text page
 * @details
 * @author   jack
 * @date     2023-02-08
 * @version  v1.0
 *********************************************************************************************************
 */

#ifndef __BLOC_VOICE_TO_TEXT_H__
#define __BLOC_VOICE_TO_TEXT_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "watch_global_data.h"

#ifdef BSP_USING_WEBRTC_VAD
#include "include/webrtc_vad.h"
#endif

	typedef struct
	{
		bool state;
		char text[512];
	} voice2text_t;

	typedef struct
	{
		bool from_me;
		char content[128];
	} chat_message_t;

	typedef struct
	{
		char *name;
		uint16_t name_length;
	} file_t;

	typedef struct
	{
		uint16_t file_num;
		file_t *files;
	} folder_t;

#define MAX_RECORD_PATH_LEN 64

	extern void start_voice_recognition(uint8_t intent);
	extern void stop_voice_recognition(uint8_t intent);
	extern void reset_speech_coding(void);
	extern uint8_t get_speech_coding(void);
	extern void count_speech_coding(void);
	extern bool check_if_user_speaking_to_ai(void);
	extern void reset_user_speaking_to_ai(void);
	extern folder_t *list_files_in_recorder(void);
	extern int free_recorder_folder(folder_t *folder);
	extern void copy_temp_file_to_recorder(void);

	extern void start_voice_recording(void);
	extern int audio_record_pcm(const void *temp_buf, rt_uint32_t data_len);
	extern void stop_voice_recording(void);
	extern const char* get_last_recording_file(void);
	extern int opus_decode_to_pcm(const char *opus_file_path, char *pcm_file_path);

	extern void start_sync_voice_recording(void);
	extern void stop_sync_voice_recording(void);

	extern voice2text_t *getVoice2TextResult(void);
	extern char *get_combined_voice2text(void);
	extern void setVoice2TextState(bool state);
	extern void setVoice2Text(char *text);
	extern void clearVoice2Text(void);
	extern bool isTextEmpty(void);
	extern void handle_v2t_result(VOICE_RECOGNITION_PAYLOAD *msgData);
	/* voice */
	extern bool app_voice_get_listening_status(void);
	extern void app_voice_set_listening_status(bool status);
	/* voice text2voice */
	extern uint16_t record_time_string[8];
	extern void voice_record_timer_start(void);
	extern void voice_record_timer_stop(void);
	extern uint8_t app_voice_get_voice2text_intent(void);
	extern void app_voice_set_voice2text_intent(uint8_t intent);
	extern bool app_voice_get_voice2text_status(void);
	extern void app_voice_set_voice2text_status(bool status);
	/* voice record */
	extern bool app_voice_get_recording_intent(void);
	extern void app_voice_set_recording_intent(bool intent);
	extern bool app_voice_get_recording_status(void);
	extern void app_voice_set_recording_status(bool status);
	extern uint32_t *app_voice_get_record_time(void);
	extern void app_voice_set_record_time(uint32_t time);
	/*message*/
	extern void add_chat_message(bool is_me, const char *text);
	extern void clear_chat_message(void);
	extern chat_message_t *get_chat_message_list(void);
	extern uint8_t get_chat_message_count(void);

	typedef struct
	{
		void *vad_inst;
		void (*vad_init)(void);
		void (*vad_deinit)(void);
		void (*notify_vad_status)(bool active);
		void (*start_v2t)(void);
		void (*auto_stop_listening)(void);
		void (*stop_v2t)(void);

	} VoiceProvider;
	extern VoiceProvider voice_provider;
#ifdef __cplusplus
}
#endif

#endif //__BLOC_VOICE_TO_TEXT_H__
