/**
 ******************************************************************************
 * @file   bloc_task.c
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
#include "bloc_task.h"
// #include "cJSON.h"
#include "communicate_protocol.h"
#include "watch_global_data.h"
#include "wristband_ble_log.h"
#include "ui_handler.h"

#ifdef APP_ID_TODOLIST
#define DBG_TAG "bloc.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define STORE_BUFFER_SIZE 4096

static task_t task_list[TASK_ITEM_AMOUNT];
task_t *current_task_list(void)
{
	return task_list;
}
uint8_t task_items_amount = 0;
int selected_task_index = 0;

static char current_sync_id[44];
char task_json_string[256];

const task_t todo_list[] =
	{
		[0] = {
			.state = true, .completed = false, .content = "緊急！這個超緊急"},
		[1] = {.state = true, .completed = true, .content = "下一個待辦在這..."},
		[2] = {.state = true, .completed = false, .content = "手錶反向工程"},
		[3] = {.state = true, .completed = true, .content = "手錶GUI"},
		[4] = {.state = true, .completed = false, .content = "手機藍芽OTA"},
		[5] = {.state = true, .completed = true, .content = "手機藍芽通信"},
		[6] = {.state = true, .completed = false, .content = "完成報告"},
		//[7] = {.state = true, .completed = true, .content = "閱讀新書"},
		//[8] = {.state = true, .completed = false, .content = "參加會議"},
		//[9] = {.state = true, .completed = true, .content = "回覆郵件"},
		//[10] = {.state = true, .completed = false, .content = "更新專案進度"},
		//[11] = {.state = true, .completed = true, .content = "準備簡報"},
		//[12] = {.state = true, .completed = false, .content = "安排下週行程"},
};

static void notifyTask(uint8_t index)
{
	get_task(index)->state = true;
}

task_t *get_cur_task(void)
{
	return &task_list[selected_task_index];
}

task_t *get_task(int index)
{
	return &task_list[index];
}

void set_cur_task(task_t task, int index)
{
	task_list[index] = task;
	notifyTask(index);
}

static void task_to_json_string(task_t *task)
{
	// Print the JSON object to the string
	int length = snprintf(task_json_string, 256,
						  "{\"id\":\"%s\",\"completed\":%s,\"content\":\"%s\"}",
						  task->id,
						  task->completed ? "true" : "false",
						  task->content);
}

task_t empty_task(void)
{
	return (task_t){.state = true, .completed = false, .content = "", .id = ""};
}

void init_task_list(void)
{
	for (uint8_t i = 0; i < 0; i++)
	// for (uint8_t i = 0; i < SkaiWatchSys.todolist_number; i++)
	{
		if (get_task(i)->id[0] != '\0')
		{
			task_items_amount++;
			notifyTask(i);
		}
	}
}

void get_task_list_from_template(void)
{
	for (uint8_t i = 0; i < 7; i++)
	{
		task_list[i] = todo_list[i];
		LOG_D("get task %d", task_items_amount);
		task_items_amount++;
	}
}

void toggle_task(uint8_t index)
{
	get_task(index)->completed = !get_task(index)->completed;
	notifyTask(index);
	// notifyMenuTodoList(true);
}

void sync_task_by_way(task_t *task, TASK_SYNC_WAY way)
{
	if (strcmp(task->content, "") == 0)
	{
		return;
	}

	task_to_json_string(task);
	LOG_D("[sync_task] task_json_string:%s", task_json_string);
	if (way == TASK_SYNC_CREATE)
	{
		L1SendData data;
		data.event = L1SEND_CREATE_TASK;
		L1_send_event(data);
	}
	else if (way == TASK_SYNC_TOGGLE)
	{
		L1SendData data;
		data.event = L1SEND_TOGGLE_TASK;
		L1_send_event(data);
	}
	else if (way == TASK_SYNC_UPDATE)
	{
		L1SendData data;
		data.event = L1SEND_UPDATE_TASK;
		L1_send_event(data);
	}
}

void update_task_content(task_t *task, char *content)
{
	if (content != NULL && strcmp(content, task->content) != 0)
	{
		strcpy(task->content, content);
		task->state = true;
	}
}

static uint8_t check_task_existed(task_t task)
{
	for (uint8_t i = 0; i < task_items_amount; i++)
	{
		if (strcmp(get_task(i)->id, task.id) == 0)
		{
			return i;
		}
	}
	return 0;
}

static void parse_task(const char *json_str, task_t *task)
{
	LOG_D("[parse_task] json_str:%s", json_str);
	// TODO: fix parsing function
	// cJSON *root = cJSON_Parse(json_str);
	// cJSON *id_json = cJSON_GetObjectItem(root, "id");
	// strncpy(task->id, cJSON_GetStringValue(id_json), sizeof(task->id));
	// cJSON *completed_json = cJSON_GetObjectItemCaseSensitive(root, "completed");
	// if (cJSON_IsTrue(completed_json))
	// {
	// 	task->completed = true;
	// }
	// else
	// {
	// 	task->completed = false;
	// }
	// LOG_D("get value:%f", value);
	// cJSON *content_json = cJSON_GetObjectItem(root, "content");
	// strncpy(task->content, cJSON_GetStringValue(content_json), sizeof(task->content));
	// LOG_D("get task->content:%d", task->content);
	// cJSON_Delete(root);
}

static void remove_task(task_t *tasks, uint8_t *size, char *id)
{
	int output_index = 0;
	int count = 0;
	LOG_D("[remove_task] original size = %d", *size);
	for (int i = 0; i < *size; i++)
	{
		if ((strcmp(tasks[i].id, "") != 0) && (strcmp(tasks[i].id, id) != 0) && (tasks[i].completed == false))
		{
			tasks[i].state = true;
			tasks[output_index++] = tasks[i];
		}
		else
		{
			count++;
			// TODO: disable task widget
			// extern void DISABLE_TASK_WIDGET(uint8_t index);
			// DISABLE_TASK_WIDGET(*size - count);
		}
	}
	*size = output_index;
}

void handle_task_sync(bool beginOrEnd, char *id)
{
	if (beginOrEnd)
	{
		strcpy(current_sync_id, id);
		remove_task(task_list, &task_items_amount, current_sync_id);
	}
	else
	{
		memset(current_sync_id, 0, 44);
	}
	// SkaiWatchSys.todolist_number = task_items_amount;
	// TODO: gui_update_TodoList();
}

void add_task(task_t *tasks, uint8_t *size, task_t new_task)
{
	if (*size == TASK_ITEM_AMOUNT)
	{
		return;
	}
	tasks[(*size)++] = new_task;
	// selected_task_index = (*size) - 1;
	//  TODO: enable task widget
	//  extern void ENABLE_TASK_WIDGET(uint8_t index);
	//  ENABLE_TASK_WIDGET((*size) - 1);
}

void handle_task(char *json)
{
	if (task_items_amount == TASK_ITEM_AMOUNT)
		return;
	task_t task;
	parse_task(json, &task);
	task.state = true;
	add_task(task_list, &task_items_amount, task);
}

void store_tasks_before_sw_shutdown(void)
{
	// SkaiWatchSys.todolist_number = task_items_amount;
	uint8_t prev_bp_lv = 0;
	// uint32_t len = SkaiWatchSys.todolist_number * sizeof(task_t);
	uint32_t len = 0;
	uint8_t buffer[STORE_BUFFER_SIZE];
	// for (int i = 0; i < SkaiWatchSys.todolist_number; i++)
	for (int i = 0; i < 0; i++)
	{
		uint8_t *ptr = buffer + i * sizeof(task_t);
		memcpy(ptr, &task_list[i], sizeof(task_t));
	}
	// TODO: fix writing function
}

void get_tasks_after_sw_reboot(void)
{
	if (reboot_reason != 0xF0 && reboot_reason != 0xAB)
	{
		return;
	}
	// uint16_t len = SkaiWatchSys.todolist_number * sizeof(task_t);
	uint16_t len = 0;
	uint8_t buffer[STORE_BUFFER_SIZE];
	uint32_t data = 0;

	// for (int i = 0; i < SkaiWatchSys.todolist_number; i++)
	for (int i = 0; i < 0; i++)
	{
		uint8_t *ptr = buffer + i * sizeof(task_t);
		memcpy(&task_list[i], ptr, sizeof(task_t));
	}
	// Print the contents of the task array
}
#endif

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
