/**
*****************************************************************************************
*     Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
*****************************************************************************************
* @file      bloc_notification.h
* @brief     business logic of notification page
* @author    jack
* @date      2023-01-28
* @version   v1.0
**************************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2023 Skaiwalk Corporation</center></h2>
**************************************************************************************
*/

#ifndef __BLOC_NOTIFICATION_H__
#define __BLOC_NOTIFICATION_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "communicate_protocol.h"

#define ITEM_AMOUNT_NOTIFICATION 10
#define NOTIFICATION_ID_LEN 48
#define NOTIFICATION_TITLE_LEN 64
#define NOTIFICATION_MESSAGE_LEN 128
	typedef struct _st_notification_item
	{
		uint8_t index;
		bool state;
		char id[NOTIFICATION_ID_LEN];
		char title[NOTIFICATION_TITLE_LEN];
		char message[NOTIFICATION_MESSAGE_LEN];
		uint32_t sec_time;
		uint16_t type; // app type
		bool can_reply;
		bool calling;
	} notification_t;

	typedef struct
	{
		uint8_t index;
		uint32_t icon_addr;
		uint8_t str_title_len;
		uint16_t str_title_buffer[16];
		uint8_t str_subtitle_len;
		uint16_t str_subtitle_buffer[16];
		uint8_t str_content_len;
		uint16_t str_content_buffer[32];
	} notification_ui_t;

	extern char temp_send_json_string[512];
	extern char replying_notification_id[NOTIFICATION_ID_LEN];
	extern void init_notification_items(void);
	extern uint32_t notification_center_get_info_count(void);
	extern notification_t *notification_center_get_header(void);
	extern uint8_t notification_items_amount;
	extern uint8_t selected_notification_index;
	extern notification_t *get_notification(int index);
	extern notification_t *get_notification_in_reversed_ui(int index);
	extern notification_t *get_cur_notification(void);
	extern void set_notification(notification_t notification, int index);
	extern void handle_notification(uint8_t notify_id, char *json_string);
	extern void navigate_notification_info(notification_t *notification);
	extern void receive_notificaiton_process(bool replyAvailable);
	extern uint8_t get_notification_type_from_ios_ancs_name(const char *name);
	extern void interact_with_notification(notification_t *notification);
	extern void remove_notification_by_id(const char *id);
	extern void dismiss_notification_from_phone(const char *id);
	/* flash storage */
	extern void store_notifications_before_sw_shutdown(void);
	extern void get_notifications_after_sw_reboot(void);
	extern void handle_user_speech_intent(uint8_t intent, char *message);

	/* NotifyProvider: unify notification related callbacks (similar to SettingProvider) */
	typedef struct
	{
		void (*accelerometer)(float x, float y, float z);                /* 通知加速度資料 */
		void (*gyroscope)(float x, float y, float z);                    /* 通知陀螺儀資料 */
		void (*attitude)(float roll, float pitch, float yaw);            /* 通知姿態資料 */
		void (*hr)(int hr);                                              /* 通知心率 */
		void (*battery_voltage)(uint16_t voltage);                       /* 通知電池電壓 */
		void (*battery_level)(uint8_t level);                            /* 通知電池電量百分比 */
		void (*charge_status)(uint8_t status);                              /* 通知充電狀態 */
		void (*notification_refresh)(void);                              /* 刷新通知頁面 */
		void (*navigate_to_reply)(notification_t *notification);         /* 導航到回覆頁面 */
		void (*time_refresh)(void *parameter);                           /* 刷新時間顯示 */
		void (*bluetooth_connection)(void);                              /* 通知藍牙連線狀態 */
		void (*holding_displacement)(uint8_t event,int x, int y);                  /* 通知持續位移 */
	} NotifyProvider;

#ifdef BSP_USING_UI_HANDLER
	extern NotifyProvider notify_provider;
#endif
#ifdef __cplusplus
}
#endif

#endif //__BLOC_NOTIFICATION_H__
