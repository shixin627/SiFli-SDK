/**
*****************************************************************************************
*     Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
*****************************************************************************************
* @file      bloc_skaiwalk.h
* @brief     business logic of skaiwalk page
* @author    Jack
* @date      2023-09-02
* @version   v1.0
**************************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2023 Skaiwalk Corporation</center></h2>
**************************************************************************************
*/

#ifndef __BLOC_SKAIWALK_H__
#define __BLOC_SKAIWALK_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "lv_obj.h"
#include "watch_global_data.h"
#include <cJSON.h>

	extern char *generate_random_id(void);
	typedef struct
	{
		char id[32]; // Note ID
		bool state;
		char message[128];	  // Note message content
		char attachment[128]; // Attachment content (.doc file)
		char *widget_data;	  // Widget data (dynamically allocated)
		bool is_self;
		time_t timestamp;
	} chat_t;

	typedef enum
	{
		CHAT_STATUS_NONE = 0,
		CHAT_STATUS_SENDING,
		CHAT_STATUS_SENT,
		CHAT_STATUS_RECEIVED,
		CHAT_STATUS_READ,
		CHAT_STATUS_ERROR,
	} chat_status_t;

	// finance_t structure
	typedef struct
	{
		char id[32];
		char code[16];
		char name[64];
		double price;
		double change;
		double volume;
		double high;
		double low;
		double open;
		double close;
		uint32_t updatedAt;
	} finance_t;

	// currency_conversion_t structure
	typedef struct
	{
		char from_currency[8];
		char to_currency[8];
		double amount;
		double converted_amount;
		double exchange_rate;
		char result[64];
		char timestamp[32];
	} currency_conversion_t;

	typedef struct
	{
		void (*app_run)(char *name);
		void (*app_exit)(char *name);
		void (*app_goback)(void);
		chat_status_t chat_status;
		bool voice_activated;
	} SkaiwalkProvider;

	extern uint8_t quaternion_buffer[16];
	extern void bloc_skaiwalk_prepare_quaternion_buffer(float *quat);

	extern chat_t *get_message_list(void);
	extern chat_t *get_note_list(void);
	extern chat_t *temp_chat(void);
	extern SkaiwalkProvider skaiwalk_provider;
	extern uint16_t *skai_message_count_ptr(void);
	extern uint16_t *skai_note_count_ptr(void);
	extern chat_t *get_skai_message(chat_t *chat_list, uint16_t items_amount, int index, bool is_reverse);
	extern void add_self_message(chat_t *chat_list, uint16_t *items_amount_ptr, const char *message);
	extern void add_skai_message(chat_t *chat_list, uint16_t *items_amount_ptr, const char *message);
	extern void handle_skai_message(char *app_id, MSG_DATA_PAYLOAD *msgData);
	extern void append_text_to_latest_message(chat_t *chat_list, uint16_t *items_amount_ptr, char *text);
	extern void delete_skai_message(chat_t *chat_list, uint16_t *items_amount_ptr, uint8_t index);
	extern void clear_skai_message_list(chat_t *list, uint16_t *items_amount_ptr);
	extern void update_skai_message(chat_t *chat_list, uint16_t *items_amount_ptr, chat_t new_message);

	/**
	 * @brief Structure to hold a chat history entry
	 */
	typedef struct
	{
		time_t timestamp;
		char user_text[64];
		char ai_text[512];
	} chat_history_entry_t;
	extern void save_user_and_ai_chat(char *user_text, char *ai_text);
	/**
	 * @brief Read a user and AI chat from a file
	 *
	 * @param filename The name of the file to read
	 * @param entry Pointer to a chat_history_entry_t structure to store the chat
	 * @return int RT_EOK on success, negative value on error
	 */
	int read_user_and_ai_chat(const char *filename, chat_history_entry_t *entry);
	/**
	 * @brief Get the most recent chat history entries
	 *
	 * @param entries Array of chat_history_entry_t to store the entries
	 * @param max_entries Maximum number of entries to retrieve
	 * @return int Number of entries successfully read
	 */
	int get_recent_chat_history(chat_history_entry_t *entries, int max_entries);
	extern void parse_ai_processing_toolkit(const char *description);
	extern void parse_ai_reply_data(uint8_t *data, uint16_t len, lv_obj_t *parent);

	// Note list save/load/clear
	void save_note_list_to_file(void);
	void load_note_list_from_file(void);
	void parse_chat_item(cJSON *item, chat_t *note);
	void clear_note_list_file(void);

	extern void *get_temp_calendar(void);
	extern void *get_temp_finance(void);
	extern void *get_temp_currency_conversion(void);
	extern bool check_if_ai_processing(void);
	extern void set_ai_processing(bool state);
	extern void indicate_ai_processing(void);

#ifdef __cplusplus
}
#endif

#endif
