/**
 ******************************************************************************
 * @file   bloc_skaiwalk.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered, decompiled, modified,
 *    or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <rtthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <cJSON.h>
#include "bloc_v2t.h"
#include "bloc_skaiwalk.h"
#include "bloc_weather.h"
#include "bloc_calendar.h"
#include "communicate_protocol.h"
#include "communicate_parse.h"
#include "watch_global_data.h"
#include "ui_handler.h"
#include "gui_app_fwk.h"
#include "app_mainmenu.h"
#include "watch_system_interact.h"

#define DBG_TAG "bloc.skaiwalk"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

#define ITEM_AMOUNT 10

//  ********  Global Variables ********  //
/** @brief Skaiwalk provider instance */
SkaiwalkProvider skaiwalk_provider;

/** @brief Quaternion buffer for IEEE754 format */
uint8_t quaternion_buffer[16] = {0};

/** @brief Temporary storage for finance data */
static finance_t temp_finance;

/** @brief Temporary storage for currency conversion data */
static currency_conversion_t temp_currency_conversion;

/** @brief Temporary storage for calendar event data */
static calendar_event_t temp_calendar_event;

//  ********  Helper Functions ********  //
/**
 * @brief Generate random string of specified length
 * @param dest Destination buffer
 * @param length Length of random string to generate
 * @note Based on: https://stackoverflow.com/questions/15767691/generate-random-string-28-in-c
 */
#define RANDOM_ID_LEN 28
static void rand_str(char *dest, size_t length)
{
	static const char charset[] = "0123456789"
								  "abcdefghijklmnopqrstuvwxyz"
								  "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

	if (dest == NULL || length == 0)
	{
		return;
	}

	while (length-- > 0)
	{
		size_t index = (double)rand() / RAND_MAX * (sizeof(charset) - 1);
		*dest++ = charset[index];
	}
	*dest = '\0';
}

/**
 * @brief Generate a random ID string
 * @return Allocated string containing random ID, caller must free
 * @retval NULL if memory allocation failed
 */
char *generate_random_id(void)
{
	char *id = rt_malloc(RANDOM_ID_LEN + 1);
	if (id == NULL)
	{
		LOG_E("Failed to allocate memory for random ID");
		return NULL;
	}

	rand_str(id, RANDOM_ID_LEN);
	id[RANDOM_ID_LEN] = '\0'; // Ensure null termination
	return id;
}

//  ********  Business Logic ********  //
/**
 * @brief Run an application by name
 * @param name Application name to run
 */
static void app_run(char *name)
{
	if (name != NULL)
	{
		gui_app_run(name);
	}
}

/**
 * @brief Exit an application by name
 * @param name Application name to exit
 */
static void app_exit(char *name)
{
	if (name != NULL)
	{
		gui_app_exit(name);
	}
}

/**
 * @brief Exit current application
 */
static void app_goback(void)
{
	gui_app_self_exit();
}

static int bloc_skaiwalk_provider_register(void)
{
	skaiwalk_provider.app_run = app_run;
	skaiwalk_provider.app_exit = app_exit;
	skaiwalk_provider.app_goback = app_goback;
	return 0;
}

INIT_APP_EXPORT(bloc_skaiwalk_provider_register);

void bloc_skaiwalk_prepare_quaternion_buffer(float *quat)
{
	if (quat == NULL)
	{
		return;
	}
	memcpy(quaternion_buffer, quat, sizeof(float) * 4);
}

/**
 * @brief Send message notification to LVGL
 */
static void bloc_notify_skai_message(void)
{
	lvgl_msg_t msg;
	msg.type = LVGL_MSG_TYPE_MESSAGE;
	lvgl_send_msg(msg);
}

/**
 * @brief Send message stream notification to LVGL
 * @param text Text content to send
 */
static void bloc_notify_skai_message_stream(char *text)
{
	if (text == NULL)
	{
		LOG_E("bloc_notify_skai_message_stream: text is NULL");
		return;
	}

	lvgl_msg_t msg;
	msg.type = LVGL_MSG_TYPE_MESSAGE_STREAM;
	msg.data.app_message = (char *)lv_mem_alloc(strlen(text) + 1);
	if (msg.data.app_message == NULL)
	{
		LOG_E("lvgl_msg_handler.refresh_message_stream msg.data.app_message is NULL");
		return;
	}
	strcpy(msg.data.app_message, text);
	lvgl_send_msg(msg);
	indicate_ai_processing();
}

/**
 * @brief Parse AI processing toolkit description string
 * @param description UTF-8 string describing the AI's current action
 *
 * The LVGL handler takes ownership of the allocated copy and frees it after
 * dispatch (same pattern as bloc_notify_skai_message_stream).
 */
void parse_ai_processing_toolkit(const char *description)
{
	if (description == NULL)
	{
		return;
	}
	LOG_I("parse_ai_processing_toolkit: %s", description);
	lvgl_msg_t msg;
	msg.type = LVGL_MSG_TYPE_UPDATE_PROCESS_TOOLKIT;
	size_t len = strlen(description);
	msg.data.app_message = (char *)lv_mem_alloc(len + 1);
	if (msg.data.app_message == NULL)
	{
		LOG_E("parse_ai_processing_toolkit: alloc failed");
		return;
	}
	memcpy(msg.data.app_message, description, len);
	msg.data.app_message[len] = '\0';
	lvgl_send_msg(msg);
}

/* Copy a cJSON string field into a fixed-size char buffer (NUL-terminated).
   `dst` must be a real array (sizeof works on it). */
#define COPY_JSON_STR(root, key, dst) do {                       \
	cJSON *_v = cJSON_GetObjectItem((root), (key));              \
	if (_v && cJSON_IsString(_v)) {                              \
		strncpy((dst), cJSON_GetStringValue(_v), sizeof(dst) - 1); \
		(dst)[sizeof(dst) - 1] = '\0';                           \
	}                                                            \
} while (0)

#define COPY_JSON_NUM(root, key, dst) do {                       \
	cJSON *_v = cJSON_GetObjectItem((root), (key));              \
	if (_v && cJSON_IsNumber(_v)) (dst) = cJSON_GetNumberValue(_v); \
} while (0)

static void parse_finance(const char *json_str, finance_t *finance)
{
	if (json_str == NULL || finance == NULL) return;
	cJSON *root = cJSON_Parse(json_str);
	if (!root) return;

	COPY_JSON_STR(root, "id",        finance->id);
	COPY_JSON_STR(root, "code",      finance->code);
	COPY_JSON_STR(root, "name",      finance->name);
	COPY_JSON_NUM(root, "price",     finance->price);
	COPY_JSON_NUM(root, "change",    finance->change);
	COPY_JSON_NUM(root, "volume",    finance->volume);
	COPY_JSON_NUM(root, "high",      finance->high);
	COPY_JSON_NUM(root, "low",       finance->low);
	COPY_JSON_NUM(root, "open",      finance->open);
	COPY_JSON_NUM(root, "close",     finance->close);
	COPY_JSON_NUM(root, "updatedAt", finance->updatedAt);

	cJSON_Delete(root);
}

static void parse_currency_conversion(const char *json_str, currency_conversion_t *cc)
{
	if (json_str == NULL || cc == NULL) return;
	cJSON *root = cJSON_Parse(json_str);
	if (!root) return;

	COPY_JSON_STR(root, "from_currency",    cc->from_currency);
	COPY_JSON_STR(root, "to_currency",      cc->to_currency);
	COPY_JSON_NUM(root, "amount",           cc->amount);
	COPY_JSON_NUM(root, "converted_amount", cc->converted_amount);
	COPY_JSON_NUM(root, "exchange_rate",    cc->exchange_rate);
	COPY_JSON_STR(root, "result",           cc->result);
	COPY_JSON_STR(root, "timestamp",        cc->timestamp);

	cJSON_Delete(root);
}

//  ********  Temporary Data Access Functions ********  //
/**
 * @brief Get pointer to temporary finance data
 * @return Pointer to temporary finance structure
 */
void *get_temp_finance(void)
{
	return (void *)&temp_finance;
}

/**
 * @brief Get pointer to temporary currency conversion data
 * @return Pointer to temporary currency conversion structure
 */
void *get_temp_currency_conversion(void)
{
	return (void *)&temp_currency_conversion;
}

/**
 * @brief Get pointer to temporary calendar event data
 * @return Pointer to temporary calendar event structure
 */
void *get_temp_calendar(void)
{
	return (void *)&temp_calendar_event;
}

void handle_ai_reply_calendar(char *json, uint8_t action, lv_obj_t *parent)
{
	if (!json)
	{
		LOG_E("Null JSON string in handle_ai_reply_calendar");
		return;
	}
	memset(&temp_calendar_event, 0, sizeof(temp_calendar_event));
	parse_calendar(json, &temp_calendar_event);
}

void handle_ai_reply_finance(char *json, lv_obj_t *parent)
{
	if (!json)
	{
		LOG_E("Null JSON string in handle_ai_reply_finance");
		return;
	}
	memset(&temp_finance, 0, sizeof(temp_finance));
	parse_finance(json, &temp_finance);

	lv_finance_object_builder(parent, (void *)&temp_finance);
}

void handle_ai_reply_currency_conversion(char *json, lv_obj_t *parent)
{
	if (!json)
	{
		LOG_E("Null JSON string in handle_ai_reply_currency_conversion");
		return;
	}
	memset(&temp_currency_conversion, 0, sizeof(temp_currency_conversion));
	parse_currency_conversion(json, &temp_currency_conversion);

	lv_currency_conversion_object_builder(parent, (void *)&temp_currency_conversion);
}

void handle_ai_reply_hint(char *json)
{
	if (!json)
	{
		LOG_E("Null JSON string in handle_ai_reply_hint");
		return;
	}

	cJSON *root = cJSON_Parse(json);
	if (!root)
	{
		LOG_E("Failed to parse hint JSON");
		return;
	}

	cJSON *type = cJSON_GetObjectItem(root, "type");
	cJSON *message = cJSON_GetObjectItem(root, "message");

	if (type && cJSON_IsString(type) && message && cJSON_IsString(message))
	{
		const char *type_str = cJSON_GetStringValue(type);
		const char *message_str = cJSON_GetStringValue(message);

		if (strcmp(type_str, "permission_request") == 0)
		{
			ui_show_hint_toast("%s", message_str);
		}
	}

	cJSON_Delete(root);
}

void parse_ai_reply_data(uint8_t *data, uint16_t len, lv_obj_t *parent)
{
	if (len < 1)
	{
		LOG_E("ai reply command length is less than 1");
		return;
	}
	switch (data[0])
	{
	case WeatherTool:
	{
		if (len < 2)
		{
			LOG_E("ai reply command length is less than 2");
			return;
		}

		handle_ai_reply_weather((char *)&data[1], parent);
		break;
	}

	case CalendarQueryTool:
	{
		handle_ai_reply_calendar((char *)&data[1], CalendarQueryTool, parent);
		break;
	}

	case CalendarCreateTool:
	{
		handle_ai_reply_calendar((char *)&data[1], CalendarCreateTool, parent);
		break;
	}

		// case CreateNoteTool:
		// {
		// 	handle_ai_reply_create_note((char *)&data[1]);
		// 	break;
		// }

		// case WebPageTool:
		// {
		// 	handle_ai_reply_web_page((char *)&data[1]);
		// 	break;
		// }

		// case WebSearchTool:
		// {
		// 	handle_ai_reply_web_search((char *)&data[1]);
		// 	break;
		// }

		// case RagTool:
		// {
		// 	handle_ai_reply_rag((char *)&data[1]);
		// 	break;
		// }

	case FinanceTool:
	{
		handle_ai_reply_finance((char *)&data[1], parent);
		break;
	}

	case CurrencyConversionTool:
	{
		handle_ai_reply_currency_conversion((char *)&data[1], parent);
		break;
	}

		// case ImageAnalyzeTool:
		// {
		// 	handle_ai_reply_image_analyze((char *)&data[1]);
		// 	break;
		// }

	case HintTool:
	{
		handle_ai_reply_hint((char *)&data[1]);
		break;
	}

	default:
		break;
	}
}

static void bloc_notify_skai_input_message()
{
	char *text = get_combined_voice2text();
	LOG_D("bloc_notify_skai_input_message: %s", text);
	
	lvgl_msg_t msg;
	msg.type = LVGL_MSG_TYPE_INPUT_MESSAGE;
	msg.data.message = text;
	lvgl_send_msg(msg);
}

// Skai AI
static chat_t _message_list[ITEM_AMOUNT];
chat_t *get_message_list()
{
	return _message_list;
}

static uint16_t message_items_amount = 0;
uint16_t *skai_message_count_ptr(void)
{
	return &message_items_amount;
}

chat_t *get_skai_message(chat_t *chat_list, uint16_t items_amount, int index, bool is_reverse)
{
	if (is_reverse)
	{
		index = items_amount - index - 1;
	}
	return &chat_list[index];
}

static void set_skai_message(chat_t *chat_list, chat_t message, int index)
{
	chat_list[index] = message;
}

static void append_message_content(chat_t *chat_list, int index, char *message)
{
	// Preserve the original timestamp
	time_t original_timestamp = chat_list[index].timestamp;
	strcat(chat_list[index].message, message);
	chat_list[index].timestamp = original_timestamp;
}

void clear_skai_message_list(chat_t *list, uint16_t *items_amount_ptr)
{
	// clear message list
	memset(list, 0, sizeof(chat_t) * ITEM_AMOUNT);
	*items_amount_ptr = 0;
}

void update_skai_message(chat_t *chat_list, uint16_t *items_amount_ptr, chat_t new_message)
{
	if (*items_amount_ptr > 0)
	{
		uint8_t i = 0;
		if (*items_amount_ptr == ITEM_AMOUNT)
		{
			i = 1;
		}
		for (; i <= *items_amount_ptr - 1; i++)
		{
			chat_t prev_message = *get_skai_message(chat_list, *items_amount_ptr, i, true);
			set_skai_message(chat_list, prev_message, *items_amount_ptr - i);
		}
	}
	set_skai_message(chat_list, new_message, 0);
	if (*items_amount_ptr < ITEM_AMOUNT)
	{
		(*items_amount_ptr)++;
	}
}

void delete_skai_message(chat_t *chat_list, uint16_t *items_amount_ptr, uint8_t index)
{
	if (index < *items_amount_ptr)
	{
		for (uint8_t i = index; i < *items_amount_ptr - 1; i++)
		{
			chat_t prev_message = *get_skai_message(chat_list, *items_amount_ptr, i + 1, false);
			set_skai_message(chat_list, prev_message, i);
		}
		(*items_amount_ptr)--;
	}
	bloc_notify_skai_message();
}

void append_text_to_latest_message(chat_t *chat_list, uint16_t *items_amount_ptr, char *text)
{
	// chat_t *latest_message = get_skai_message(chat_list, *items_amount_ptr, 0, false);
	// LOG_D("append_text_to_latest_message:is_self=%d, %s", latest_message->is_self, text);
	// if (latest_message->is_self)
	// {
	// 	strcat(latest_message->message, text);
	// }
	// else
	// {
	chat_t new_message = {0};

	// Generate random ID for the new message
	char *random_id = generate_random_id();
	if (random_id)
	{
		strncpy(new_message.id, random_id, sizeof(new_message.id) - 1);
		rt_free(random_id);
	}
	else
	{
		new_message.id[0] = '\0';
	}

	strncpy(new_message.message, text, sizeof(new_message.message) - 1);
	new_message.attachment[0] = '\0';
	new_message.state = true;
	new_message.is_self = true;
	new_message.timestamp = time(NULL); // Add current timestamp
	new_message.widget_data = NULL;
	update_skai_message(chat_list, items_amount_ptr, new_message);
	// }
	// bloc_notify_skai_message();
}

void append_text_to_input_message()
{
	bloc_notify_skai_input_message();
}

void add_self_message(chat_t *chat_list, uint16_t *items_amount_ptr, const char *message)
{
	append_text_to_latest_message(chat_list, items_amount_ptr, (char *)message);
}

void add_skai_message(chat_t *chat_list, uint16_t *items_amount_ptr, const char *message)
{
	chat_t new_message = {0};

	// Generate random ID for the new message
	char *random_id = generate_random_id();
	if (random_id)
	{
		strncpy(new_message.id, random_id, sizeof(new_message.id) - 1);
		rt_free(random_id);
	}
	else
	{
		new_message.id[0] = '\0';
	}

	strncpy(new_message.message, message, sizeof(new_message.message) - 1);
	new_message.attachment[0] = '\0';
	new_message.state = true;
	new_message.is_self = false;
	new_message.timestamp = time(NULL); // Add current timestamp
	new_message.widget_data = NULL;
	update_skai_message(chat_list, items_amount_ptr, new_message);
	bloc_notify_skai_message();
}

static uint32_t last_updated_ts;
static chat_t temp_message = {
	.id = "",
	.state = false,
	.message = "",
	.attachment = "",
	.widget_data = NULL,
	.is_self = false,
	.timestamp = 0};
/* Append `src_len` bytes from `src` onto *buf (allocating or reallocating as
   needed). Maintains NUL-termination at *buf[*len]. Returns 0 on success, -1
   on alloc failure (in which case *buf is freed and zeroed). */
static int sk_buf_append(char **buf, int *len, const uint8_t *src, uint16_t src_len)
{
	if (*buf == NULL)
	{
		*buf = rt_malloc(src_len + 1);
		if (*buf == NULL) { *len = 0; return -1; }
		memcpy(*buf, src, src_len);
		*len = src_len;
	}
	else
	{
		char *nb = rt_realloc(*buf, *len + src_len + 1);
		if (nb == NULL) { rt_free(*buf); *buf = NULL; *len = 0; return -1; }
		*buf = nb;
		memcpy(*buf + *len, src, src_len);
		*len += src_len;
	}
	(*buf)[*len] = '\0';
	return 0;
}

void handle_skai_message(char *app_id, MSG_DATA_PAYLOAD *msgData)
{
	static char *accumulated_text = NULL; // Buffer to accumulate message parts
	static int accumulated_length = 0;	  // Length of the accumulated message

	if (msgData == NULL || msgData->p_msg_value == NULL)
	{
		LOG_E("Invalid message data received");
		return;
	}

	LOG_D("[%s]msg Header:%d, Data len=%d", __func__, msgData->header, msgData->length);
	bool prepared = false;

	switch (msgData->header)
	{
	case MESSAGE_PART_START:
		/* L2 fragmentation guarantees this packet carries the full message
		   regardless of size, so header=0 always means "complete one-shot
		   message" — drop any pending multi-part state and render. The old
		   `length < 235` heuristic that distinguished single from multi was
		   tied to the phone-side 200-byte chunking and is no longer needed. */
		if (accumulated_text != NULL)
		{
			rt_free(accumulated_text);
			accumulated_text = NULL;
			accumulated_length = 0;
		}
		if (sk_buf_append(&accumulated_text, &accumulated_length,
						  msgData->p_msg_value, msgData->length) != 0)
		{
			LOG_E("Failed to allocate memory for message");
			return;
		}
		prepared = true;
		break;

	case MESSAGE_PART_MIDDLE:
		/* Streaming start/middle: append to the running buffer, no render yet.
		   Tolerates a missing prior START — we just init the buffer. */
		if (sk_buf_append(&accumulated_text, &accumulated_length,
						  msgData->p_msg_value, msgData->length) != 0)
		{
			LOG_E("Failed to (re)allocate memory for message");
			return;
		}
		break;

	case MESSAGE_PART_END:
		/* Streaming end: append, then render. Tolerates a missing START so
		   a lone END behaves like a single-shot message. */
		if (sk_buf_append(&accumulated_text, &accumulated_length,
						  msgData->p_msg_value, msgData->length) != 0)
		{
			LOG_E("Failed to (re)allocate memory for final message part");
			return;
		}
		prepared = true;
		break;

	default:
		LOG_E("Unknown message header type: %d", msgData->header);
		return;
	}

	if (prepared && accumulated_text != NULL)
	{
		temp_message.is_self = false;
		temp_message.state = true;
		strncpy(temp_message.message, accumulated_text, sizeof(temp_message.message));
		temp_message.timestamp = time(NULL);
		bloc_notify_skai_message_stream(temp_message.message);

		// Free the accumulated buffer
		rt_free(accumulated_text);
		accumulated_text = NULL;
		accumulated_length = 0;
	}
}

void parse_open_app_command(const uint8_t *pValue, uint16_t length)
{
	if (pValue == NULL || length == 0)
	{
		LOG_E("Invalid input parameters");
		return;
	}

	AppIntent intent;
	memset(&intent, 0, sizeof(AppIntent));

	// Ensure the JSON string is null-terminated
	char *json_str = rt_malloc(length + 1);
	if (json_str == NULL)
	{
		LOG_E("Failed to allocate memory for JSON string");
		return;
	}

	memcpy(json_str, pValue, length);
	json_str[length] = '\0';

	// Parse JSON
	cJSON *root = cJSON_Parse(json_str);
	if (root == NULL)
	{
		LOG_E("JSON Parse Error: %s", cJSON_GetErrorPtr());
		rt_free(json_str);
		return;
	}

	// Extract "appId" field
	cJSON *id_item = cJSON_GetObjectItem(root, "appId");
	if (id_item != NULL && cJSON_IsString(id_item) && id_item->valuestring != NULL)
	{
		rt_strncpy(intent.app_id, id_item->valuestring, sizeof(intent.app_id) - 1);
		intent.app_id[sizeof(intent.app_id) - 1] = '\0';
	}
	else
	{
		LOG_W("App ID not found in JSON or invalid");
	}

	// Extract "param" field (if exists)
	cJSON *param_item = cJSON_GetObjectItem(root, "param");
	if (param_item != NULL)
	{
		if (cJSON_IsString(param_item) && param_item->valuestring != NULL)
		{
			// If parameter is a string
			rt_strncpy(intent.intent, param_item->valuestring, sizeof(intent.intent) - 1);
		}
		else
		{
			// If parameter is another JSON format, convert to string
			char *param_str = cJSON_PrintUnformatted(param_item);
			if (param_str != NULL)
			{
				rt_strncpy(intent.intent, param_str, sizeof(intent.intent) - 1);
				cJSON_free(param_str);
			}
		}
		intent.intent[sizeof(intent.intent) - 1] = '\0';
	}

	// Clean up resources
	cJSON_Delete(root);
	rt_free(json_str);

	// Call system interaction function
	animate_to_home_from_ai_page();
	watch_run_app_by_intent(&intent);
}

void save_user_and_ai_chat(char *user_text, char *ai_text)
{
	if (user_text == NULL || ai_text == NULL)
	{
		LOG_E("Cannot save chat: text is NULL");
		return;
	}

	// Create chat history directory if it doesn't exist
	if (access("/chat_history", 0) != 0)
	{
		if (mkdir("/chat_history", 0x777) != 0)
		{
			LOG_E("Failed to create chat history directory");
			return;
		}
		LOG_D("Created chat history directory");
	}

	// Generate a timestamp-based filename
	time_t now;
	struct tm *time_info;
	char filename[64];
	time(&now);
	time_info = localtime(&now);
	snprintf(filename, sizeof(filename), "/chat_history/chat_%04d%02d%02d_%02d%02d%02d.json",
			 time_info->tm_year + 1900, time_info->tm_mon + 1, time_info->tm_mday,
			 time_info->tm_hour, time_info->tm_min, time_info->tm_sec);

	// Create JSON object
	cJSON *root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "timestamp", (long)now);
	cJSON_AddStringToObject(root, "user", user_text);
	cJSON_AddStringToObject(root, "ai", ai_text);

	// Convert to string
	char *json_str = cJSON_Print(root);

	// Open file for writing
	FILE *fp = fopen(filename, "w");
	if (fp == NULL)
	{
		LOG_E("Failed to open chat file for writing: %s", filename);
		cJSON_free(json_str);
		cJSON_Delete(root);
		return;
	}

	// Write to file
	if (fwrite(json_str, strlen(json_str), 1, fp) != 1)
	{
		LOG_E("Failed to write chat data to file");
		fclose(fp);
		cJSON_free(json_str);
		cJSON_Delete(root);
		return;
	}

	// Close the file and clean up
	fclose(fp);
	cJSON_free(json_str);
	cJSON_Delete(root);

	LOG_D("Chat saved to %s", filename);
}

void clear_chat_history(void)
{
	// Remove all files in the chat history directory
	DIR *dir = opendir("/chat_history");
	if (dir == NULL)
	{
		LOG_E("Failed to open chat history directory");
		return;
	}

	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL)
	{
		if (entry->d_type == DT_REG) // Regular file
		{
			char filepath[64];
			snprintf(filepath, sizeof(filepath), "/chat_history/%s", entry->d_name);
			unlink(filepath); // Delete the file
		}
	}

	closedir(dir);
	LOG_D("Chat history cleared");
}

int read_user_and_ai_chat(const char *filename, chat_history_entry_t *entry)
{
	if (filename == NULL || entry == NULL)
	{
		LOG_E("Invalid parameters");
		return -RT_ERROR;
	}

	// Open file for reading
	int fd = open(filename, O_RDONLY);
	if (fd < 0)
	{
		LOG_E("Failed to open chat file for reading: %s", filename);
		return -RT_ERROR;
	}

	LOG_D("Reading chat file: %s", filename);

	// Read file content
	char buffer[1024] = {0};
	int read_size = read(fd, buffer, sizeof(buffer) - 1);
	close(fd);

	if (read_size <= 0)
	{
		LOG_E("Failed to read chat file or file is empty");
		return -RT_ERROR;
	}

	buffer[read_size] = '\0'; // Ensure null termination

	// Parse JSON content
	cJSON *root = cJSON_Parse(buffer);
	if (root == NULL)
	{
		LOG_E("Failed to parse JSON: %s", cJSON_GetErrorPtr());
		return -RT_ERROR;
	}

	// Extract timestamp
	cJSON *timestamp_json = cJSON_GetObjectItem(root, "timestamp");
	if (cJSON_IsNumber(timestamp_json))
	{
		entry->timestamp = (time_t)timestamp_json->valuedouble;
	}
	else
	{
		entry->timestamp = 0;
		LOG_W("Timestamp not found or invalid in chat file");
	}

	// Extract user text
	cJSON *user_json = cJSON_GetObjectItem(root, "user");
	if (cJSON_IsString(user_json) && user_json->valuestring != NULL)
	{
		strncpy(entry->user_text, user_json->valuestring, sizeof(entry->user_text) - 1);
		entry->user_text[sizeof(entry->user_text) - 1] = '\0';
	}
	else
	{
		entry->user_text[0] = '\0';
		LOG_W("User text not found or invalid in chat file");
	}

	// Extract AI text
	cJSON *ai_json = cJSON_GetObjectItem(root, "ai");
	if (cJSON_IsString(ai_json) && ai_json->valuestring != NULL)
	{
		strncpy(entry->ai_text, ai_json->valuestring, sizeof(entry->ai_text) - 1);
		entry->ai_text[sizeof(entry->ai_text) - 1] = '\0';
	}
	else
	{
		entry->ai_text[0] = '\0';
		LOG_W("AI text not found or invalid in chat file");
	}

	// Clean up
	cJSON_Delete(root);

	return RT_EOK;
}

int get_recent_chat_history(chat_history_entry_t *entries, int max_entries)
{
	if (entries == NULL || max_entries <= 0)
	{
		LOG_E("Invalid parameters");
		return 0;
	}

	// Check if chat history directory exists
	if (access("/chat_history", 0) != 0)
	{
		LOG_W("Chat history directory does not exist");
		return 0;
	}

	// Open the chat history directory
	DIR *dir = opendir("/chat_history");
	if (dir == NULL)
	{
		LOG_E("Failed to open chat history directory");
		return 0;
	}

	// Array to store filenames and their timestamps
	typedef struct
	{
		char filename[64];
		time_t timestamp;
	} file_info_t;

	file_info_t *files = rt_malloc(sizeof(file_info_t) * max_entries);
	if (files == NULL)
	{
		LOG_E("Failed to allocate memory for file list");
		closedir(dir);
		return 0;
	}

	int file_count = 0;
	struct dirent *entry;

	// Read all chat files
	while ((entry = readdir(dir)) != NULL && file_count < max_entries)
	{
		// Skip if not a chat file
		if (strncmp(entry->d_name, "chat_", 5) != 0 ||
			strstr(entry->d_name, ".json") == NULL)
		{
			continue;
		}

		// Construct full path
		char fullpath[256];
		snprintf(fullpath, sizeof(fullpath), "/chat_history/%s", entry->d_name);

		// Read the file to get timestamp
		chat_history_entry_t temp_entry;
		if (read_user_and_ai_chat(fullpath, &temp_entry) == RT_EOK)
		{
			strcpy(files[file_count].filename, fullpath);
			files[file_count].timestamp = temp_entry.timestamp;
			file_count++;
		}
	}

	closedir(dir);

	// Sort files by timestamp (newest first)
	for (int i = 0; i < file_count - 1; i++)
	{
		for (int j = i + 1; j < file_count; j++)
		{
			if (files[j].timestamp > files[i].timestamp)
			{
				file_info_t temp = files[i];
				files[i] = files[j];
				files[j] = temp;
			}
		}
	}

	// Read the most recent files
	int entry_count = 0;
	for (int i = 0; i < file_count && i < max_entries; i++)
	{
		if (read_user_and_ai_chat(files[i].filename, &entries[i]) == RT_EOK)
		{
			entry_count++;
		}
	}

	rt_free(files);
	return entry_count;
}


/**
 * @brief Parse a single chat_t item from cJSON object
 * @param json_item cJSON object containing chat item data
 * @param chat_item Pointer to chat_t structure to populate
 */
void parse_chat_item(cJSON *json_item, chat_t *chat_item)
{
	if (json_item == NULL || chat_item == NULL)
	{
		return;
	}

	cJSON *id = cJSON_GetObjectItem(json_item, "id");
	cJSON *msg = cJSON_GetObjectItem(json_item, "message");
	cJSON *attachment = cJSON_GetObjectItem(json_item, "attachment");
	cJSON *is_self = cJSON_GetObjectItem(json_item, "is_self");
	cJSON *timestamp = cJSON_GetObjectItem(json_item, "timestamp");

	// Handle id
	if (id && cJSON_IsString(id))
		strncpy(chat_item->id, id->valuestring, sizeof(chat_item->id) - 1);
	else
		chat_item->id[0] = '\0';

	// Handle message
	if (msg && cJSON_IsString(msg))
		strncpy(chat_item->message, msg->valuestring, sizeof(chat_item->message) - 1);
	else
		chat_item->message[0] = '\0';

	// Handle attachment
	if (attachment && cJSON_IsString(attachment))
		strncpy(chat_item->attachment, attachment->valuestring, sizeof(chat_item->attachment) - 1);
	else
		chat_item->attachment[0] = '\0';

	// Default state to true
	chat_item->state = true;

	// Handle is_self
	chat_item->is_self = (is_self && cJSON_IsBool(is_self)) ? cJSON_IsTrue(is_self) : false;

	// Handle timestamp - use current time if not available
	if (timestamp && cJSON_IsNumber(timestamp))
		chat_item->timestamp = (time_t)timestamp->valuedouble;
	else
		chat_item->timestamp = time(NULL);

	// Initialize widget_data pointer to NULL
	chat_item->widget_data = NULL;
}


/// AI ///

static rt_timer_t ai_processing_timer = RT_NULL;
static bool is_ai_processing = false;
static bool received_ai_response = false;

/**
 * @brief Timeout handler for AI processing timer
 * @param parameter Timer parameter (unused)
 */
static void ai_processing_timeout(void *parameter)
{
	LOG_D("AI processing timeout reached");
	// extern void open_ai_tap_hint_bg(bool open);
	// open_ai_tap_hint_bg(true);
	lvgl_msg_t msg;
	msg.type = LVGL_MSG_TYPE_AI_TAP_HINT;
	lvgl_send_msg(msg);
	set_ai_processing(false);
}

/**
 * @brief Create and start AI processing timer
 */
static void create_ai_processing_timer(void)
{
	if (!ai_processing_timer)
	{
		ai_processing_timer = rt_timer_create("ai_processing", ai_processing_timeout,
											  NULL, 20000, RT_TIMER_FLAG_ONE_SHOT);
	}
	else
	{
		rt_timer_stop(ai_processing_timer);
		rt_timer_control(ai_processing_timer, RT_TIMER_CTRL_SET_TIME, (void *)&(rt_uint32_t){20000});
	}

	rt_timer_start(ai_processing_timer);
}

/**
 * @brief Stop AI processing timer
 */
static void stop_ai_processing_timer(void)
{
	if (ai_processing_timer)
	{
		rt_timer_stop(ai_processing_timer);
	}
}

/**
 * @brief Indicate AI processing response received
 * Resets and restarts timer with shorter timeout when first response arrives
 */
void indicate_ai_processing(void)
{
	if (!is_ai_processing)
	{
		return;
	}

	if (!received_ai_response)
	{
		rt_timer_stop(ai_processing_timer);
		rt_uint32_t time_left = 1000;
		rt_timer_control(ai_processing_timer, RT_TIMER_CTRL_SET_TIME, &time_left);
		received_ai_response = true;
	}
	rt_timer_start(ai_processing_timer);
}

/**
 * @brief Check if AI is currently processing
 * @return Processing status
 */
bool check_if_ai_processing(void)
{
	return is_ai_processing;
}

/**
 * @brief Set AI processing state
 * @param state New processing state
 */
void set_ai_processing(bool state)
{
	is_ai_processing = state;
	if (is_ai_processing)
	{
		received_ai_response = false;
		create_ai_processing_timer();
	}
	else
	{
		stop_ai_processing_timer();
	}
}

//  ********  test ********  //
#if !kReleaseMode
static int utest_skaiwalk(int argc, char *argv[])
{
	if (argc >= 2)
	{
		if (strcmp(argv[1], "-self_msg") == 0)
		{
			if (argc == 3)
			{
				add_self_message(get_message_list(), &message_items_amount, argv[2]);
			}
			else
			{
				add_self_message(get_message_list(), &message_items_amount, "Hello, Skaiwalk!");
			}
		}
		else if (strcmp(argv[1], "-skai_msg") == 0)
		{
			if (argc == 3)
			{
				add_skai_message(get_message_list(), &message_items_amount, argv[2]);
			}
			else
			{
				add_skai_message(get_message_list(), &message_items_amount, "How can I help you?");
			}
		}
		else if (strcmp(argv[1], "-clear") == 0)
		{
			clear_chat_history();
		}
		else if (strcmp(argv[1], "-read") == 0)
		{
			if (argc == 3)
			{
				chat_history_entry_t entry;
				if (read_user_and_ai_chat(argv[2], &entry) == RT_EOK)
				{
					LOG_D("Timestamp: %lu, User: %s, AI: %s", (unsigned long)entry.timestamp, entry.user_text, entry.ai_text);
				}
			}
		}
		else if (strcmp(argv[1], "-get_recent") == 0)
		{
			int max_entries = ITEM_AMOUNT;
			if (argc == 3)
			{
				max_entries = atoi(argv[2]);
			}
			/* MSVC: no VLA in C — heap-allocate the buffer instead. */
			chat_history_entry_t *entries = rt_malloc(sizeof(chat_history_entry_t) * max_entries);
			if (entries)
			{
				int count = get_recent_chat_history(entries, max_entries);
				for (int i = 0; i < count; i++)
				{
					LOG_D("Entry %d: Timestamp: %lu, User: %s, AI: %s", i + 1, (unsigned long)entries[i].timestamp, entries[i].user_text, entries[i].ai_text);
				}
				rt_free(entries);
			}
		}
		else if (strcmp(argv[1], "-generate_random_id") == 0)
		{
			char *random_id = generate_random_id();
			if (random_id != NULL)
			{
				LOG_D("Generated random ID: %s", random_id);
				rt_free(random_id); // Free the allocated memory
			}
			else
			{
				LOG_E("Failed to generate random ID");
			}
		}
	}
	return 0;
}
MSH_CMD_EXPORT(utest_skaiwalk, "utest_skaiwalk [OPTION] ...");
#endif

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
