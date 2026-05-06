/**
 ******************************************************************************
 * @file   bloc_calendar.h
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
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

#ifndef __BLOC_CALENDAR_H__
#define __BLOC_CALENDAR_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "lv_obj.h"
#include "watch_global_data.h"

#define MAX_CALENDAR_TITLE_LENGTH 64
#define MAX_CALENDAR_CONTENT_LENGTH 128
	typedef struct calendar_event
	{
		char *id;
		uint16_t id_length;
		char *summary;
		uint16_t summary_length;
		char *description;
		uint16_t description_length;
		char *location;
		uint16_t location_length;
		T_UTC_TIME startTime;
		T_UTC_TIME endTime;
		bool notified;
	} calendar_event_t;

	typedef struct calendar
	{
		uint16_t event_count;
		calendar_event_t *events;
	} calendar_t;

	typedef struct calendar_today
	{
		lv_obj_t *day;
		lv_obj_t *time;
		lv_obj_t *summary;
		lv_obj_t *event_category;
	} calendar_model_t;

	extern calendar_event_t *get_calendar_event(int day, int index);
	extern calendar_t *get_calendar_day(int day);
	extern void calendar_today_refresh(void);
	extern uint8_t calendar_items_amount;
	extern int selected_calendar_index;
	extern void notify_calendar(void);
	extern uint8_t get_calendar_day_sync_amout(uint8_t day);
	extern void parse_calendar(const char *json_str, calendar_event_t *calendar_event);
	extern void handle_calendar(char *json, uint8_t day);
	extern calendar_event_t empty_calendar_event(void);
	extern calendar_t *create_calendar_day(uint16_t event_count);
	extern void free_calendar_day(calendar_t *calendar_day);
	extern int add_calendar_event(calendar_t *calendar_day, const char *id, const char *summary, const char *description, const char *location, T_UTC_TIME start_time, T_UTC_TIME end_time);
	extern void request_calendar_on_mobile(bool active_call);
	extern void cleanup_all_calendars(void);

#ifdef __cplusplus
}
#endif

#endif // __BLOC_CALENDAR_H__

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
