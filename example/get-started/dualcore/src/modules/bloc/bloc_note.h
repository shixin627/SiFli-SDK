/**
*****************************************************************************************
*     Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
*****************************************************************************************
* @file      bloc_note.h
* @brief     business logic of note page
* @author    jack
* @date      2023-01-28
* @version   v1.0
**************************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2023 Skaiwalk Corporation</center></h2>
**************************************************************************************
*/

#ifndef __BLOC_NOTE_H__
#define __BLOC_NOTE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"

#define MAX_NOTE_TITLE_LENGTH 64
#define MAX_NOTE_CONTENT_LENGTH 256

	typedef struct
	{
		bool state;
		char id[32];
		char title[MAX_NOTE_TITLE_LENGTH];
		char content[MAX_NOTE_CONTENT_LENGTH];
	} note_t;

	typedef struct
	{
		uint8_t str_title_len;
		uint16_t str_title_buffer[16];
		uint8_t str_content_len;
		uint16_t str_content_buffer[64];
	} note_ui_t;

	extern note_t *current_note_list(void);
	extern note_t *get_note(int index);
	extern note_t *get_cur_note(void);
	extern uint8_t note_items_amount;
	extern int selected_note_index;
	// extern void initialize_note_state(void);
	extern void init_note_list(void);
	extern void set_note(uint8_t index, char *title, char *content);
	extern void handle_note(char *json);
	extern void handle_note_sync(bool beginOrEnd, char *id);
	extern note_t empty_note(void);
	extern void update_note_content(note_t *note, char *content);
	extern void store_notes_before_sw_shutdown(void);
	extern void get_notes_after_sw_reboot(void);
	extern void create_note(note_t *note);

#ifdef __cplusplus
}
#endif

#endif //__BLOC_CONTROL_H__
