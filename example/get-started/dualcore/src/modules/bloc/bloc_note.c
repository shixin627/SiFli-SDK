/**
 ******************************************************************************
 * @file   bloc_note.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bloc_note.h"
#include "bloc_skaiwalk.h"
#include "watch_global_data.h"
#include "wristband_ble_log.h"
#include "communicate_protocol.h"
#include "watch_global_data.h"
#include "cJSON.h"
#include "ui_handler.h"

#ifdef APP_ID_NOTE
#define DBG_TAG "bloc_note"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define ITEM_AMOUNT 5

static note_t default_note_list[] =
	{
		[0] = {
			.title = "標題1",
			.content = "內容1",
			.state = true,
		},
		[1] = {
			.title = "標題2",
			.content = "內容2",
			.state = true,
		},
		[2] = {
			.title = "標題3",
			.content = "內容3",
			.state = true,
		},
		[3] = {
			.title = "標題4",
			.content = "內容4",
			.state = true,
		},
		[4] = {
			.title = "標題5",
			.content = "內容5",
			.state = true,
		},
};

static note_t note_list[ITEM_AMOUNT];
note_t *current_note_list(void)
{
	return note_list;
}
uint8_t note_items_amount = 0;
int selected_note_index = 0;
static char current_sync_id[44];

static void notifyNote(uint8_t index)
{
	note_list[index].state = true;
}

note_t *get_note(int index)
{
	return &note_list[index];
}

note_t *get_cur_note(void)
{
	return &note_list[selected_note_index];
}

void set_cur_note(note_t note, int index)
{
	note_list[index] = note;
	notifyNote(index);
}

void init_note_list(void)
{
	for (uint8_t i = 0; i < SkaiWatchSys.notelist_number; i++)
	{
		if (get_note(i)->id[0] != '\0')
		{
			note_items_amount++;
			// set_cur_note(default_note_list[i], i);
			// notifyNote(i);
		}
	}
}

void set_note(uint8_t index, char *title, char *content)
{
	if (title != NULL && strcmp(title, note_list[index].title) != 0)
	{
		strcpy(note_list[index].title, title);
	}
	if (content != NULL && strcmp(content, note_list[index].content) != 0)
	{
		strcpy(note_list[index].content, content);
	}
	notifyNote(index);
}

note_t empty_note(void)
{
	return (note_t){.state = true, .title = "", .content = ""};
}

static void parse_note(const char *json_str, note_t *note)
{
	LOG_D("parse_note json_str: %s", json_str);
	// TODO: parse note from json string
	// cJSON *root = cJSON_Parse(json_str);
	// cJSON *id_json = cJSON_GetObjectItem(root, "id");
	// strncpy(note->id, cJSON_GetStringValue(id_json), sizeof(note->id));
	// cJSON *title_json = cJSON_GetObjectItem(root, "ts");
	// strncpy(note->title, cJSON_GetStringValue(title_json), sizeof(note->title));
	// cJSON *content_json = cJSON_GetObjectItem(root, "message");
	// strncpy(note->content, cJSON_GetStringValue(content_json), sizeof(note->content));
	// cJSON_Delete(root);
}

void add_note(note_t *notes, uint8_t *size, note_t new_note)
{
	if (*size == ITEM_AMOUNT)
	{
		return;
	}
	notes[(*size)++] = new_note;
	selected_note_index = (*size) - 1;
	// TODO: enable note widget
	// extern void ENABLE_NOTE_WIDGET(uint8_t index);
	// ENABLE_NOTE_WIDGET((*size) - 1);
}

void handle_note(char *json)
{
	if (note_items_amount == ITEM_AMOUNT)
		return;
	note_t note;
	parse_note(json, &note);
	note.state = true;
	add_note(note_list, &note_items_amount, note);
	// TODO: Note gui update
}

static void remove_note(note_t *notes, uint8_t *size, char *id)
{
	int output_index = 0;
	int count = 0;
	for (int i = 0; i < *size; i++)
	{
		if ((strcmp(notes[i].id, "") != 0) && (strcmp(notes[i].id, id) != 0))
		{
			notes[i].state = true;
			notes[output_index++] = notes[i];
		}
		else
		{
			count++;
			// TODO: disable note widget
			// extern void DISABLE_NOTE_WIDGET(uint8_t index);
			// DISABLE_NOTE_WIDGET(*size - count);
		}
	}
	*size = output_index;
}

void handle_note_sync(bool beginOrEnd, char *id)
{
	if (beginOrEnd)
	{
		strcpy(current_sync_id, id);
		remove_note(note_list, &note_items_amount, current_sync_id);
	}
	else
	{
		memset(current_sync_id, 0, 44);
	}
	SkaiWatchSys.notelist_number = note_items_amount;
	// TODO: gui_update_Note();
}

static void note_to_json_string(note_t *note)
{
	strcpy(temp_send_json_string, "");
	cJSON *obj = cJSON_CreateObject();
	cJSON_AddStringToObject(obj, "content", note->content);
	strcpy(temp_send_json_string, cJSON_PrintUnformatted(obj));
	cJSON_Delete(obj);
}

void update_note_content(note_t *note, char *content)
{
	if (content != NULL && strcmp(content, note->content) != 0)
	{
		strcpy(note->content, content);
		note->state = true;
	}
}

void create_note(note_t *note)
{
	if (strlen(note->content) == 0)
	{
		return;
	}
	note_to_json_string(note);
	L1SendData data;
	data.event = L1SEND_CREATE_NOTE;
	data.res.json_string_ptr = temp_send_json_string;
	L1_send_event(data);
}

void update_note(note_t *note)
{
	if (strcmp(note->id, "") == 0)
	{
		return;
	}
	note_to_json_string(note);
	L1SendData data;
	data.event = L1SEND_UPDATE_NOTE;
	data.res.json_string_ptr = temp_send_json_string;
	L1_send_event(data);
}

void delete_note(note_t *note)
{
	if (note_items_amount == 0)
	{
		return;
	}
	if (strcmp(note->id, "") == 0)
	{
		return;
	}
	note_to_json_string(note);
	L1SendData data;
	data.event = L1SEND_DELETE_NOTE;
	data.res.json_string_ptr = temp_send_json_string;
	L1_send_event(data);
}

static int utest_note(int argc, char *argv[])
{
	if (argc >= 2)
	{
		if (strcmp(argv[1], "-create") == 0)
		{
			if (argc == 3)
			{
				note_t note = empty_note();
				char *random_id = generate_random_id();
				strcpy(note.id, random_id);
				free(random_id);
				strcpy(note.content, argv[2]);
				create_note(&note);
			}
		}
		else if (strcmp(argv[1], "-update") == 0)
		{
			if (argc == 4)
			{
				note_t note = empty_note();
				strcpy(note.id, argv[2]);
				strcpy(note.content, argv[3]);
				update_note(&note);
			}
		}
		else if (strcmp(argv[1], "-delete") == 0)
		{
			if (argc == 3)
			{
				note_t note = empty_note();
				strcpy(note.id, argv[2]);
				delete_note(&note);
			}
		}
	}
	return 0;
}
MSH_CMD_EXPORT(utest_note, "utest_note [OPTION] ...");

#define STORE_BUFFER_SIZE 4096
void store_notes_before_sw_shutdown(void)
{
	SkaiWatchSys.notelist_number = note_items_amount;
	uint8_t prev_bp_lv = 0;
	// uint16_t len = SkaiWatchSys.notelist_number * sizeof(note_t);
	uint8_t buffer[STORE_BUFFER_SIZE];
	for (int i = 0; i < SkaiWatchSys.notelist_number; i++)
	{
		uint8_t *ptr = buffer + i * sizeof(note_t);
		memcpy(ptr, get_note(i), sizeof(note_t));
	}

	// TODO: write note data to flash
}

void get_notes_after_sw_reboot(void)
{
	// TODO: fix reboot reason
	// if (reboot_reason != 0xF0 && reboot_reason != 0xAB)
	// {
	// 	return;
	// }
	uint16_t len = SkaiWatchSys.notelist_number * sizeof(note_t);
	uint8_t buffer[STORE_BUFFER_SIZE];
	uint32_t data = 0;
	// TODO: read note data from flash
	for (int i = 0; i < SkaiWatchSys.notelist_number; i++)
	{
		uint8_t *ptr = buffer + i * sizeof(note_t);
		memcpy(get_note(i), ptr, sizeof(note_t));
	}
}
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
