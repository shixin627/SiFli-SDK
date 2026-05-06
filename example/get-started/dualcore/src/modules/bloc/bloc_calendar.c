/**
 ******************************************************************************
 * @file   bloc_calendar.c
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
#include <stdint.h>
#include "cJSON.h"
#include "time.h"
#include "bloc_calendar.h"
#include "bloc_skaiwalk.h"
#include "watch_global_data.h"
#include "gui_app_fwk.h"
#include "mem_section.h"
#include "communicate_protocol.h"
#include "communicate_task.h"
#include "ui_handler.h"
#include "rtthread.h"

#define DBG_TAG "bloc.calendar"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

// Calendar constants
#define DAYS_PER_WEEK 7
#define DEFAULT_CALENDAR_DAY 0 // Day 0 for default calendar entries

uint8_t calendar_items_amount = 0;
int selected_calendar_index = 0;

// 動態日曆存儲 - 每天一個日曆對象
ITCM_NON_RET_DATA_SECT(calendar)
calendar_t *calendar_days[DAYS_PER_WEEK] = {NULL};

/**
 * 分配並複製字符串
 */
static char *allocate_string(const char *src, uint16_t *length)
{
	if (src == NULL) {
		*length = 0;
		return NULL;
	}
	
	*length = strlen(src);
	char *dest = (char *)rt_malloc(*length + 1);
	if (dest == NULL) {
		*length = 0;
		return NULL;
	}
	
	strcpy(dest, src);
	return dest;
}

/**
 * 創建一個新的日曆日
 */
calendar_t *create_calendar_day(uint16_t event_count)
{
	calendar_t *calendar_day = (calendar_t *)rt_malloc(sizeof(calendar_t));
	if (calendar_day == NULL) {
		return NULL;
	}
	
	calendar_day->event_count = event_count;
	
	if (event_count > 0) {
		calendar_day->events = (calendar_event_t *)rt_malloc(sizeof(calendar_event_t) * event_count);
		if (calendar_day->events == NULL) {
			rt_free(calendar_day);
			return NULL;
		}
		// 初始化所有事件
		memset(calendar_day->events, 0, sizeof(calendar_event_t) * event_count);
	} else {
		calendar_day->events = NULL;
	}
	
	return calendar_day;
}

/**
 * 釋放日曆日的記憶體
 */
void free_calendar_day(calendar_t *calendar_day)
{
	if (calendar_day == NULL) {
		return;
	}
	
	// 釋放每個事件的動態字符串
	for (uint16_t i = 0; i < calendar_day->event_count; i++) {
		calendar_event_t *event = &calendar_day->events[i];
		if (event->id) rt_free(event->id);
		if (event->summary) rt_free(event->summary);
		if (event->description) rt_free(event->description);
		if (event->location) rt_free(event->location);
	}
	
	// 釋放事件陣列
	if (calendar_day->events) {
		rt_free(calendar_day->events);
	}
	
	// 釋放日曆日結構體
	rt_free(calendar_day);
}

/**
 * 添加新的日曆事件到日曆日
 */
int add_calendar_event(calendar_t *calendar_day, const char *id, const char *summary, 
					  const char *description, const char *location, 
					  T_UTC_TIME start_time, T_UTC_TIME end_time)
{
	if (calendar_day == NULL) {
		return -1;
	}
	
	// 重新分配事件陣列記憶體
	uint16_t new_count = calendar_day->event_count + 1;
	calendar_event_t *new_events = (calendar_event_t *)rt_realloc(
		calendar_day->events, sizeof(calendar_event_t) * new_count);
	
	if (new_events == NULL) {
		return -1;
	}
	
	calendar_day->events = new_events;
	calendar_event_t *new_event = &calendar_day->events[calendar_day->event_count];
	
	// 初始化新事件
	memset(new_event, 0, sizeof(calendar_event_t));
	
	// 分配並複製字符串
	new_event->id = allocate_string(id, &new_event->id_length);
	new_event->summary = allocate_string(summary, &new_event->summary_length);
	new_event->description = allocate_string(description, &new_event->description_length);
	new_event->location = allocate_string(location, &new_event->location_length);
	
	// 設置時間
	new_event->startTime = start_time;
	new_event->endTime = end_time;
	new_event->notified = false;
	
	// 更新事件數量
	calendar_day->event_count = new_count;
	
	return calendar_day->event_count - 1; // 返回新事件的索引
}

calendar_event_t empty_calendar_event(void)
{
	calendar_event_t temp = {0};
	return temp;
}

calendar_event_t *get_calendar_event(int day, int index)
{
	if (day < 0 || day >= DAYS_PER_WEEK || calendar_days[day] == NULL) {
		return NULL;
	}
	
	if (index < 0 || index >= calendar_days[day]->event_count) {
		return NULL;
	}
	
	return &calendar_days[day]->events[index];
}

calendar_t *get_calendar_day(int day)
{
	if (day < 0 || day >= DAYS_PER_WEEK) {
		return NULL;
	}
	
	return calendar_days[day];
}

void parse_calendar(const char *json_str, calendar_event_t *calendar_event)
{
	if (!json_str || !calendar_event)
	{
		LOG_E("Invalid parameters in parse_calendar");
		return;
	}

	LOG_D("json_str.length: %d", strlen(json_str));
	cJSON *root = cJSON_Parse(json_str);
	if (!root)
	{
		LOG_E("Failed to parse JSON data");
		return;
	}

	// Parse ID field
	cJSON *id_json = cJSON_GetObjectItem(root, "id");
	if (id_json && cJSON_IsString(id_json))
	{
		calendar_event->id = allocate_string(cJSON_GetStringValue(id_json), &calendar_event->id_length);
	}

	// Parse summary field
	cJSON *summary_json = cJSON_GetObjectItem(root, "summary");
	if (summary_json && cJSON_IsString(summary_json))
	{
		calendar_event->summary = allocate_string(cJSON_GetStringValue(summary_json), &calendar_event->summary_length);
	}

	// Parse description field
	cJSON *description_json = cJSON_GetObjectItem(root, "description");
	if (description_json && cJSON_IsString(description_json))
	{
		calendar_event->description = allocate_string(cJSON_GetStringValue(description_json), &calendar_event->description_length);
	}

	// Parse location field
	cJSON *location_json = cJSON_GetObjectItem(root, "location");
	if (location_json && cJSON_IsString(location_json))
	{
		calendar_event->location = allocate_string(cJSON_GetStringValue(location_json), &calendar_event->location_length);
	}

	// Parse start time
	cJSON *startTime_json = cJSON_GetObjectItem(root, "startTime");
	if (startTime_json && cJSON_IsNumber(startTime_json))
	{
		time_t secondsSinceEpoch = cJSON_GetNumberValue(startTime_json);
		struct tm *timeinfo = localtime(&secondsSinceEpoch);
		if (timeinfo)
		{
			calendar_event->startTime = (T_UTC_TIME){
				.year = timeinfo->tm_year + 1900,
				.month = timeinfo->tm_mon + 1,
				.day = timeinfo->tm_mday,
				.hour = timeinfo->tm_hour,
				.minutes = timeinfo->tm_min,
				.seconds = timeinfo->tm_sec,
			};
			LOG_D("calendar_event->startTime: %d-%d-%d %d:%d:%d",
				  calendar_event->startTime.year, calendar_event->startTime.month, calendar_event->startTime.day,
				  calendar_event->startTime.hour, calendar_event->startTime.minutes, calendar_event->startTime.seconds);
		}
	}

	// Parse end time
	cJSON *endTime_json = cJSON_GetObjectItem(root, "endTime");
	if (endTime_json && cJSON_IsNumber(endTime_json))
	{
		time_t secondsSinceEpoch = cJSON_GetNumberValue(endTime_json);
		struct tm *timeinfo = localtime(&secondsSinceEpoch);
		if (timeinfo)
		{
			calendar_event->endTime = (T_UTC_TIME){
				.year = timeinfo->tm_year + 1900,
				.month = timeinfo->tm_mon + 1,
				.day = timeinfo->tm_mday,
				.hour = timeinfo->tm_hour,
				.minutes = timeinfo->tm_min,
				.seconds = timeinfo->tm_sec,
			};
		}
	}

	cJSON_Delete(root);
}

void notify_calendar(void)
{
	lvgl_msg_t msg;

	// Notify calendar widget
	msg.type = LVGL_MSG_TYPE_REFRESH_CALENDAR_WIDGET;
	lvgl_send_msg(msg);

	// Notify calendar app if active
	if (gui_app_is_actived("calendar"))
	{
		msg.type = LVGL_MSG_TYPE_CALNEDAR;
		lvgl_send_msg(msg);
	}

	SkaiWatchSys.calendar_sync_time = (uint32_t)time(NULL);
}

static uint8_t calendar_data_updata_count = 0;

static void update_calendar(calendar_event_t newEvent, int day)
{
	if (day >= DAYS_PER_WEEK)
	{
		LOG_E("Invalid day index: %d", day);
		return;
	}

	// 如果該天還沒有日曆，創建一個
	if (calendar_days[day] == NULL) {
		calendar_days[day] = create_calendar_day(0);
		if (calendar_days[day] == NULL) {
			LOG_E("Failed to create calendar for day %d", day);
			return;
		}
	}

	// 檢查是否已經存在相同的事件（通過 ID 比較）
	if (newEvent.id != NULL) {
		calendar_t *calendar_day = calendar_days[day];
		for (uint16_t i = 0; i < calendar_day->event_count; i++) {
			calendar_event_t *existing_event = &calendar_day->events[i];
			if (existing_event->id != NULL && 
				strcmp(existing_event->id, newEvent.id) == 0) {
				LOG_D("Event with ID '%s' already exists on day %d, skipping duplicate", 
					  newEvent.id, day);
				return;
			}
		}
	}

	// 添加新事件到該天的日曆
	int result = add_calendar_event(calendar_days[day], 
		newEvent.id, newEvent.summary, newEvent.description, newEvent.location,
		newEvent.startTime, newEvent.endTime);
		
	if (result >= 0) {
		LOG_D("Added event to day %d, event index: %d", day, result);
	} else {
		LOG_E("Failed to add event to day %d", day);	}
}

void handle_calendar(char *json, uint8_t day)
{
	if (!json)
	{
		LOG_E("NULL JSON data in handle_calendar");
		return;
	}

	if (day >= DAYS_PER_WEEK)
	{
		LOG_E("Invalid day index: %d", day);
		return;
	}

	// Parse the calendar data
	calendar_event_t calendar_event = empty_calendar_event();
	parse_calendar(json, &calendar_event);
	LOG_D("day: %d, summary: %s", day, calendar_event.summary ? calendar_event.summary : "NULL");

	// Update calendar storage
	update_calendar(calendar_event, day);

	// Special handling for day 6
	if (day == 6)
	{		// Check if we have received all expected calendar items
		calendar_t *calendar_day = get_calendar_day(day);
		uint8_t expected_count = calendar_day ? calendar_day->event_count : 0;
		
		LOG_D("calendar_data_updata_count: %d, expected_count: %d",
			  calendar_data_updata_count, expected_count);

		if (calendar_data_updata_count >= expected_count)
		{
			// Notify UI and reset counter when all calendar items received
			notify_calendar();
			calendar_data_updata_count = 0;
		}
		else
		{
			calendar_data_updata_count++;
		}
	}
}

uint8_t get_calendar_day_sync_amout(uint8_t day)
{
	if (day >= DAYS_PER_WEEK)
	{
		LOG_E("Invalid day index: %d", day);
		return 0;
	}

	calendar_t *calendar_day = get_calendar_day(day);
	if (calendar_day == NULL)
	{
		return 0;
	}

	return calendar_day->event_count;
}

/**
 * @brief Request calendar with rate limiting, allowing one request every 30 seconds.
 * * @param active_call Indicates if the request is active or not.
 */

#define CALENDAR_REQUEST_COOLDOWN_SEC 30

static uint32_t last_calendar_request_time = 0;

void request_calendar_on_mobile(bool active_call)
{
	if (SkaiWatchSys.connected_to_phone == false)
	{
		LOG_D("Request calendar skipped - not connected to phone");
		return;
	}

	uint32_t current_time = (uint32_t)time(NULL);
	bool has_request = (current_time - last_calendar_request_time < 5); // 5 seconds
	if (has_request)
	{
		LOG_D("Calendar request cooldown active, skipping request");
		return;
	}
	bool has_synchronized_recently = (current_time - SkaiWatchSys.calendar_sync_time < 30); // 30 seconds
	if (active_call || !has_synchronized_recently)
	{
		LOG_D("Requesting calendar");
		commu_send_calendar_request();

		// Update last request timestamp
		last_calendar_request_time = current_time;
	}
}

/**
 * 清理所有日曆記憶體
 */
void cleanup_all_calendars(void)
{
	for (int i = 0; i < DAYS_PER_WEEK; i++) {
		if (calendar_days[i] != NULL) {
			free_calendar_day(calendar_days[i]);
			calendar_days[i] = NULL;
		}
	}
	LOG_D("All calendar memory cleaned up");
}
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
