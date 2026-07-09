/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_parse_notify.h
* @brief
* @details
* @author
* @date
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef __COMMUNICATE_PARSE_NOTIFY_H__
#define __COMMUNICATE_PARSE_NOTIFY_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"

	typedef enum
	{
		KEY_INCOMMING_CALL = 0x01,
		KEY_INCOMMING_CALL_ACCEPT = 0x02,
		KEY_INCOMMING_CALL_REFUSE = 0x03,
		KEY_INCOMMING_MESSAGE = 0x04,
		KEY_BATTERY_CHARGE_STATUS = 0X06,
		KEY_DISMISS_NOTIFICATION = 0x08,
		KEY_VOICE_RECOGNITION_RESULT = 0X14,
		KEY_VOICE_RECOGNITION_END = 0X15,
		KEY_GESTURE_DETECT = 0X17,
		KEY_REMOTE_INPUT = 0X20,

		KEY_CALENDAR_SYNC_START = 0X26,
		KEY_CALENDAR_SYNC_END = 0X27,
		KEY_UPDATE_CALENDAR = 0X28,

		KEY_WEATHER_SYNC_START = 0x29,
		KEY_UPDATE_WEATHER = 0x2a,
		KEY_REQUEST_WEATHER = 0x2b,
		KEY_REQUEST_CALENDAR = 0x2c,

		KEY_OTA_SYNC_STATUS = 0X32,
		KEY_WATCH_IMAGE_SYNC_START = 0X33,
		KEY_WATCH_IMAGE_SYNC_END = 0X34,
		KEY_UPDATE_WATCH_IMAGE = 0X35,
		KEY_WATCH_LCPU_SYNC_START = 0X36,
		KEY_WATCH_LCPU_SYNC_END = 0X37,
		KEY_UPDATE_WATCH_LCPU = 0X38,
		KEY_WATCH_HCPU_SYNC_START = 0X39,
		KEY_WATCH_HCPU_SYNC_END = 0X3a,
		KEY_UPDATE_WATCH_HCPU = 0X3b,
		KEY_WATCH_FTAB_SYNC_START = 0X3c,
		KEY_WATCH_FTAB_SYNC_END = 0X3d,
		KEY_UPDATE_WATCH_FTAB = 0X3e,
		KEY_OTA_STATUS = 0X3f,

		KEY_HEART_RATE_SENSOR_SAMPLE = 0X40,
		KEY_WATCH_SYS_REQUEST = 0X42,
		KEY_WATCH_SYS_RETURN = 0X43,
		KEY_RETURN_CHAT_INTENT = 0X44,
		KEY_CHAT_RESULT = 0X45,
		KEY_MEDIA_TITLE = 0X46,
		KEY_BATTERY_LEVEL = 0X4D,
		KEY_GSENSOR_SAMPLE = 0X50,
		KEY_START_SYNC_FILE = 0X52,
		KEY_SYNC_FILE = 0X53,
		KEY_END_SYNC_FILE = 0X54,
		KEY_USER_SPEAKING_STATE = 0X58,
		KEY_AI_PROCESS_TOOLKIT = 0X59,
		KEY_LOCATION_DATA = 0X5C,
		KEY_WATCH_BOOTLOADER_SYNC_START = 0X5D,
		KEY_WATCH_BOOTLOADER_SYNC_END = 0X5E,
		KEY_UPDATE_WATCH_BOOTLOADER = 0X5F,
		KEY_WATCH_LCPU_PATCH_SYNC_START = 0X60,
		KEY_WATCH_LCPU_PATCH_SYNC_END = 0X61,
		KEY_UPDATE_WATCH_LCPU_PATCH = 0X62,
		KEY_FILE_COMPARE_RESULT = 0X63,
		KEY_SKAI_CREATION_INSTRUCTIONS = 0X65,
		KEY_SKAI_INSTRUCTION_IMAGE = 0X66,
		KEY_DISMISS_SKAI_INSTRUCTION = 0X67,
		/* 0x68 / 0x69 reserved on phone side for Skaibar options/selected. */
		// skaibar (hid_mouse 長按 bar AI 模式) 選項列表：
		//   PC → watch  : KEY_SKAIBAR_OPTIONS (JSON {"options":["a","b",...]})
		//   watch → PC  : KEY_SKAIBAR_SELECTED (1 byte index, 0-based)
		KEY_SKAIBAR_OPTIONS = 0X68,
		KEY_SKAIBAR_SELECTED = 0X69,
		KEY_BATTERY_VOLTAGE = 0X6A,
		/* watch → PC: 1-byte 0-based index. Fires when the user TAPS an
		   option (commit / confirm), distinct from KEY_SKAIBAR_SELECTED
		   which only reports the currently-highlighted option as the
		   wheel / list scrolls. Phone uses the commit signal to invoke
		   the option's action; the prior SELECTED stream is just for
		   keeping the panel highlight in sync. Founder direction
		   2026-05-19. */
		KEY_SKAIBAR_COMMITTED = 0X6C,
		/* Phone → watch: full-list replace of custom instructions in one
		   BLE message. Payload is a UTF-8 JSON array of the same per-item
		   object 0x65 carries: [{id,title,trigger,enabled,version}, ...].
		   Watch clears existing custom instructions, then appends up to
		   MAX_LIST_ITEMS - app_base_count items (excess silently dropped),
		   and fires a single deferred UI rebuild. Saves the per-item
		   200 ms BLE pacing the old per-instruction loop required. */
		KEY_SKAI_CREATION_INSTRUCTIONS_BATCH = 0X6B,
		/* watch -> phone: 1-byte ack sent after KEY_END_SYNC_FILE finishes
		   writing the phone-pushed file (1 = written OK, 0 = size mismatch /
		   write error). The generic file-sync protocol (0x52/0x53/0x54) had
		   no completion signal at all -- unlike OTA's KEY_OTA_STATUS -- so
		   the phone had no way to know when the watch actually finished,
		   and declared "done" the instant it queued its last BLE write
		   locally. Mirrors send_file_compare_result's shape
		   (commu_send_status). */
		KEY_FILE_SYNC_RESULT = 0X6D,
	} NOTIFY_KEY;

	void resolve_Notify_command(uint8_t key, uint8_t *pValue, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_NOTIFY_H__
