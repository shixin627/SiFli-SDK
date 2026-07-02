/**
 ******************************************************************************
 * @file   bloc_weather.h
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

#ifndef __BLOC_WEATHER_H__
#define __BLOC_WEATHER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "watch_global_data.h"
#include "lvgl.h"

#define MAX_CALENDAR_TITLE_LENGTH 64
#define MAX_CALENDAR_CONTENT_LENGTH 128

#define WEATHER_TODAT_ITEM_AMOUNT 4
#define WEATHER_DAILY_ITEM_AMOUNT 5
	typedef struct weather
	{
		T_UTC_TIME time;
		float rainLastHour;
		float snowLastHour;
		float temperature;
		float max_temperature;
		float min_temperature;
		int precipitationProbability;
		char description[16];
		char location[48];
		bool isDailySummary; // Indicates if this weather data is a daily summary
		bool notified;
	} weather_t;

	typedef struct weather_datac
	{
		lv_obj_t *time;
		lv_obj_t *img;
		lv_obj_t *temperature;
		lv_obj_t *degree;
		lv_obj_t *chance_of_rain;
	} weather_data_t;

	extern weather_t *get_weather(int index);
	extern void weather_layout_update(void);
	extern void handle_weather(char *json);
	extern weather_t empty_weather(void);
	extern void handle_ai_reply_weather(char *json, lv_obj_t *parent);
	extern void request_weather_within_six_hours(bool active_call);
	extern weather_t *current_weather_week_list(void);
	extern void handle_location_data(char *json);
	extern const char *get_current_location(void);

#ifdef __cplusplus
}
#endif

#endif // __BLOC_CALENDAR_H__

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
