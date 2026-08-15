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

	/* Voice-pipeline heap guard. The mic frame processor (audio_station, prio
	   16) writes into heap objects (VAD inst, NS/AGC insts, shared Opus
	   encoder) that HIGHER-priority threads (peripheral 8 / lvgl 9) tear down:
	   an unlucky preemption frees a state block mid-write → heap corruption →
	   the rt_free (MEM_USED) / audio_mem_malloc asserts. Every use AND every
	   free/create of those objects must hold this lock. Recursive (same thread
	   may re-take) with priority inheritance. */
	extern void voice_pipeline_lock(void);
	extern void voice_pipeline_unlock(void);

	/* One-shot override for the intent passed to start_voice_recognition()
	   inside voice_recognition_entry's VOICE_RECOGNITION_START handler.
	   The handler hardcodes V2T_INTENT_CHAT — callers that need a
	   different intent (e.g. instruction_list AI widget wanting
	   V2T_INTENT_SKAIBAR) set this before voice_provider.start_v2t().
	   Resets to V2T_INTENT_CHAT after each START handler runs, so a
	   subsequent caller that didn't override gets the default. */
	extern void voice_set_pending_v2t_intent(uint8_t intent);

	/* Re-entry gate for VOICE_RECOGNITION_START. Set true inside the
	   event task's START handler, cleared inside AUTO_STOP / STOP. The
	   STOP branch can be skipped if AI is still processing the prior
	   utterance, so callers that force-close mid-AI-processing (e.g.
	   close_ai_widget) must clear it directly to keep the next START
	   tap from being short-circuited. */
	extern void set_voice_recognition_started(bool started);
	extern void reset_speech_coding(void);
	extern uint8_t get_speech_coding(void);
	extern void count_speech_coding(void);
	extern bool check_if_user_speaking_to_ai(void);
	extern void reset_user_speaking_to_ai(void);

	extern int start_voice_recording(void);
	extern int audio_record_pcm(const void *temp_buf, rt_uint32_t data_len);
	extern void stop_voice_recording(void);
	extern const char* get_last_recording_file(void);

	extern void start_sync_voice_recording(void);
	extern void stop_sync_voice_recording(void);

	extern char *get_combined_voice2text(void);
	extern void setVoice2Text(char *text);
	extern void clearVoice2Text(void);
	extern bool isTextEmpty(void);
	extern void handle_v2t_result(VOICE_RECOGNITION_PAYLOAD *msgData);
	/* voice */
	extern bool app_voice_get_listening_status(void);
	extern void app_voice_set_listening_status(bool status);
	/* voice text2voice */
	extern void app_voice_set_voice2text_intent(uint8_t intent);
	extern bool app_voice_get_voice2text_status(void);
	extern void app_voice_set_voice2text_status(bool status);
	/* voice record */
	extern void app_voice_set_recording_intent(bool intent);
	extern bool app_voice_get_recording_status(void);
	extern void app_voice_set_recording_status(bool status);
	/* True once a write to the recording file failed (disk full). Set by the
	   audio thread; consumed by the recorder UI to stop and warn the user. */
	extern bool app_voice_recording_disk_full(void);

	typedef struct
	{
		void *vad_inst;
		void (*vad_init)(void);
		void (*vad_deinit)(void);
		void (*notify_vad_status)(bool active);
		void (*start_v2t)(void);
		void (*auto_stop_listening)(void);
		void (*stop_v2t)(void);
		bool audio_subscribed;
	} VoiceProvider;
	extern VoiceProvider voice_provider;
#ifdef __cplusplus
}
#endif

#endif //__BLOC_VOICE_TO_TEXT_H__
