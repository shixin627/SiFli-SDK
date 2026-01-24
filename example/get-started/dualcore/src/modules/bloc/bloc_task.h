/**
*****************************************************************************************
*     Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
*****************************************************************************************
* @file      bloc_task.h
* @brief     business logic of task page
* @author    jack
* @date      2023-01-28
* @version   v1.0
**************************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2023 Skaiwalk Corporation</center></h2>
**************************************************************************************
*/

#ifndef __BLOC_TASK_H__
#define __BLOC_TASK_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "watch_global_data.h"

#define TASK_ITEM_AMOUNT 15
#define TASK_SECTOR_SIZE 0x1000
#define TASK_BUF_SIZE 128
#define MAX_CONTENT_LENGTH 128

	typedef struct
	{
		bool state;
		char id[44];
		bool completed;
		char content[MAX_CONTENT_LENGTH];
		//T_UTC_TIME todo_time;
	} task_t;

	typedef struct
	{
		uint8_t str_content_len;
		uint16_t str_content_buffer[32];
	} task_ui_t;

	typedef enum
	{
		TASK_SYNC_CREATE = 0x00,
		TASK_SYNC_TOGGLE = 0x01,
		TASK_SYNC_UPDATE = 0x02,
	} TASK_SYNC_WAY;

	// extern bool check_menu_todo_list_notified(void);
	// extern void notifyMenuTodoList(bool notify);
	extern char task_json_string[256];
	extern task_t *current_task_list(void);
	extern uint8_t task_items_amount;
	extern int selected_task_index;

	extern void init_task_list(void);

	extern task_t empty_task(void);
	extern task_t *get_cur_task(void);
	extern task_t *get_task(int index);
	extern void add_task(task_t *tasks, uint8_t *size, task_t new_task);
	extern void toggle_task(uint8_t index);
	extern void update_task_content(task_t *task, char *content);
	extern void sync_task_by_way(task_t *task, TASK_SYNC_WAY way);

	extern void handle_task_sync(bool beginOrEnd, char *id);
	extern void handle_task(char *json);
	extern void store_tasks_before_sw_shutdown(void);
	extern void get_tasks_after_sw_reboot(void);
#ifdef __cplusplus
}
#endif

#endif //__BLOC_TASK_H__
