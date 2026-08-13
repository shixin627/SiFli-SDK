/**
 ******************************************************************************
 * @file   bloc_weather.c
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
#include <time.h>
#include "cJSON.h"
#include "bloc_weather.h"
#include "bloc_skaiwalk.h"
#include "bloc_peripheral.h"
#include "watch_global_data.h"
#include "communicate_protocol.h"
#include "communicate_task.h"
#include "ui_handler.h"

#define DBG_TAG "bloc.weather"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>


/* Module state variables */
static uint8_t weather_items_amount = 0;
static uint8_t weather_items_amount_week = 0;
// hourly weather every 3 hours
static weather_t today_weather_list[WEATHER_TODAT_ITEM_AMOUNT];
// summary daily weather
static weather_t week_weather_list[WEATHER_DAILY_ITEM_AMOUNT];
static char current_location[48] = {0};

weather_t *current_weather_week_list(void)
{
	return week_weather_list;
}

/**
 * @brief Create an empty weather structure
 * @return Empty weather structure
 */
weather_t empty_weather(void)
{
	weather_t temp = {
		.time = 0,
		.rainLastHour = 0,
		.snowLastHour = 0,
		.temperature = 0,
		.description = "",
		.notified = false,
	};

	return temp;
}

/**
 * @brief Get weather item at specified index
 * @param index Index of weather item to retrieve
 * @return Pointer to weather item
 */
weather_t *get_weather(int index)
{
	return &today_weather_list[index];
}

/**
 * @brief Parse weather data from JSON string
 * @param json_str JSON string containing weather data
 * @param weather Pointer to weather structure to populate
 */
static void parse_weather(const char *json_str, weather_t *weather)
{
	if (!json_str || !weather)
	{
		LOG_E("Invalid parameters in parse_weather");
		return;
	}

	LOG_D("parse_weather json_str: %s", json_str);
	cJSON *root = cJSON_Parse(json_str);
	if (!root)
	{
		LOG_E("Failed to parse JSON in parse_weather");
		return;
	}

	cJSON *description_json = cJSON_GetObjectItem(root, "description");
	if (description_json)
	{
		strncpy(weather->description, cJSON_GetStringValue(description_json), sizeof(weather->description) - 1);
		weather->description[sizeof(weather->description) - 1] = '\0'; // Ensure null termination
	}

	cJSON *precipitationProbability_json = cJSON_GetObjectItem(root, "precipitationProbability");
	if (precipitationProbability_json)
	{
		weather->precipitationProbability = cJSON_GetNumberValue(precipitationProbability_json);
	}
	else
	{
		weather->precipitationProbability = 0; // Default value if not present
	}

	cJSON *temperature_json = cJSON_GetObjectItem(root, "temperature");
	weather->temperature = temperature_json ? (float)cJSON_GetNumberValue(temperature_json) : 0;

	cJSON *isDailySummary_json = cJSON_GetObjectItem(root, "isDailySummary");
	weather->isDailySummary = isDailySummary_json ? cJSON_IsTrue(isDailySummary_json) : false;
	if (weather->isDailySummary)
	{
		cJSON *maxTemperature_json = cJSON_GetObjectItem(root, "maxTemperature");
		weather->max_temperature = maxTemperature_json ? (float)cJSON_GetNumberValue(maxTemperature_json) : 0;

		cJSON *minTemperature_json = cJSON_GetObjectItem(root, "minTemperature");
		weather->min_temperature = minTemperature_json ? (float)cJSON_GetNumberValue(minTemperature_json) : 0;
	}
	else
	{
		cJSON *rainLastHour_json = cJSON_GetObjectItem(root, "rainLastHour");
		weather->rainLastHour = rainLastHour_json ? (float)cJSON_GetNumberValue(rainLastHour_json) : 0;

		cJSON *snowLastHour_json = cJSON_GetObjectItem(root, "snowLastHour");
		weather->snowLastHour = snowLastHour_json ? (float)cJSON_GetNumberValue(snowLastHour_json) : 0;
	}

	cJSON *location_json = cJSON_GetObjectItem(root, "location");
	if (cJSON_IsString(location_json))
	{
		const char *loc = cJSON_GetStringValue(location_json);
		if (loc && loc[0] != '\0')
		{
			strncpy(weather->location, loc, sizeof(weather->location) - 1);
			weather->location[sizeof(weather->location) - 1] = '\0';
			/* Phone embeds location in the weather JSON instead of sending a
			   separate KEY_LOCATION_DATA packet. Mirror it into the global so
			   get_current_location() (used by the weather UI) returns it. */
			strncpy(current_location, loc, sizeof(current_location) - 1);
			current_location[sizeof(current_location) - 1] = '\0';
		}
	}

	cJSON *time_json = cJSON_GetObjectItem(root, "date");
#ifndef BSP_USING_PC_SIMULATOR
	if (time_json)
	{
		time_t secondsSinceEpoch = cJSON_GetNumberValue(time_json);
		struct tm *timeinfo = localtime(&secondsSinceEpoch);
		if (timeinfo)
		{
			weather->time = (T_UTC_TIME){
				.year = timeinfo->tm_year + 1900,
				.month = timeinfo->tm_mon + 1,
				.day = timeinfo->tm_mday,
				.hour = timeinfo->tm_hour,
				.minutes = timeinfo->tm_min,
				.seconds = timeinfo->tm_sec,
			};
		}
	}
#endif

	cJSON_Delete(root);
}

/* Push `new_weather` to the head of `list[max]` (newest first), shifting older
   entries by one. `*count` is incremented up to `max`; once full, the oldest
   item is dropped. Shared body of update_weather / update_weather_week. */
static void weather_push_front(weather_t *list, uint8_t max, uint8_t *count,
                               weather_t new_weather)
{
	if (*count > 0)
	{
		uint8_t i = (*count == max) ? 1 : 0;
		for (; i <= *count - 1; i++)
		{
			list[*count - i] = list[*count - i - 1];
		}
	}
	list[0] = new_weather;
	if (*count < max) (*count)++;
}

static void update_weather(weather_t new_weather)
{
	weather_push_front(today_weather_list, WEATHER_TODAT_ITEM_AMOUNT,
	                   &weather_items_amount, new_weather);
}

static void update_weather_week(weather_t new_weather)
{
	weather_push_front(week_weather_list, WEATHER_DAILY_ITEM_AMOUNT,
	                   &weather_items_amount_week, new_weather);
}

/**
 * @brief Notify UI of weather data updates
 */
static void notify_weather(void)
{
	lvgl_msg_t msg;
	msg.type = LVGL_MSG_TYPE_REFRESH_WEATHER_WIDGET;
	lvgl_send_msg(msg);
	extern void notify_weather_screen_update(void);
	notify_weather_screen_update();
	SkaiWatchSys.weather_sync_time = (uint32_t)time(NULL);
}

/**
 * @brief Handle incoming weather JSON data
 * @param json JSON string with weather data
 */
void handle_weather(char *json)
{
	if (!json)
	{
		LOG_E("Null JSON string in handle_weather");
		return;
	}

	weather_t weather = empty_weather();
	parse_weather(json, &weather);
	if (weather.isDailySummary)
	{
		LOG_D("Received daily summary weather data");
		update_weather_week(weather);
	}
	else
	{
		LOG_D("Received hourly weather data");
		update_weather(weather);
	}
	notify_weather();
}

/**
 * @brief Request weather update if not requested in the last 30 minutes or if the hour has changed
 * @param active_call Flag indicating if the request is active
 */
void request_weather_within_six_hours(bool active_call)
{
	LOG_D("Requesting weather update");
	if (SkaiWatchSys.connected_to_phone == false)
	{
		return;
	}

	static time_t last_request_time = 0;
	time_t current_time = get_current_time();

	bool has_request = (current_time - last_request_time < 10); // 10 seconds
	if (has_request)
	{
		return;
	}

	bool has_synchronized_recently = (current_time - SkaiWatchSys.weather_sync_time < 1800); // 30 minutes 
	if (active_call || !has_synchronized_recently)
	{
		LOG_D("Requesting weather update within six hours");
		commu_send_weather_request();

		/* Update the last request time */
		last_request_time = current_time;
	}
}

/**
 * @brief Get current location string
 * @return Pointer to current location string
 */
const char *get_current_location(void)
{
	return current_location;
}

/**
 * @brief Parse location data from JSON string and update global location
 * @param json_str JSON string containing location data
 */
static void parse_location_data(const char *json_str)
{
	if (!json_str)
	{
		LOG_E("Invalid parameters in parse_location_data");
		return;
	}

	LOG_D("parse_location_data json_str: %s", json_str);
	cJSON *root = cJSON_Parse(json_str);
	if (!root)
	{
		LOG_E("Failed to parse JSON in parse_location_data");
		return;
	}

	// Extract location name if available
	cJSON *location_name_json = cJSON_GetObjectItem(root, "locationName");
	if (location_name_json && cJSON_IsString(location_name_json))
	{
		const char *location_name = cJSON_GetStringValue(location_name_json);
		if (location_name && strlen(location_name) > 0)
		{
			strncpy(current_location, location_name, sizeof(current_location) - 1);
			current_location[sizeof(current_location) - 1] = '\0';
			LOG_I("Location updated to: %s", current_location);
		}
	}

	// You can also extract and store other location data if needed
	cJSON *latitude_json = cJSON_GetObjectItem(root, "latitude");
	cJSON *longitude_json = cJSON_GetObjectItem(root, "longitude");

	if (latitude_json && longitude_json)
	{
		double latitude = cJSON_GetNumberValue(latitude_json);
		double longitude = cJSON_GetNumberValue(longitude_json);
		LOG_D("Location coordinates: lat=%.6f, lon=%.6f", latitude, longitude);
	}

	cJSON_Delete(root);
}

/**
 * @brief Handle incoming location JSON data
 * @param json JSON string with location data
 */
void handle_location_data(char *json)
{
	if (!json)
	{
		LOG_E("Null JSON string in handle_location_data");
		return;
	}

	parse_location_data(json);
	// TODO: Notify UI of location update if necessary
}
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
