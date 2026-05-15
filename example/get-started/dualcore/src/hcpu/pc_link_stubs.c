/* Auto-generated stubs for PC simulator (do not edit by hand).
 * Covers 124 symbols normally provided by ARM-only modules
 * (BLE stack, voice/skai/gesture apps, IPC peripherals, etc.).
 * Each function logs once and returns a sensible default. */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdbool.h>
#include <stdint.h>

#include "lvgl/lvgl.h"
#include "lv_ex_data.h"
#include "ble_device_manager.h"
#include "gui_app_pm.h"
#include "bloc_v2t.h"
#include "ble_hid.h"
#include "communicate_protocol.h"

/* bd_addr_t comes from BLE stack header that's not on PC; provide a stub. */
typedef struct { uint8_t addr[6]; } bd_addr_t;

/* ble_hid.h */ void HID_CONSUMER_GoBack(void) {  }
/* watch_demo.c */ void ai_tap_cb(void) {  }
/* app_mainmenu.c */ void ai_widget_start(void) {  }
/* app_exercise.c */ void app_exercise_background_hr_cb(int hr) {  }
/* app_speech.h */ void app_speech_data_init(void) {  }
/* app_message.c */ lv_ex_data_t * app_speech_get_content_data(void) { return 0; }
/* bloc_v2t.h */ bool app_voice_get_recording_status(void) { return false; }
/* app_system_interface.c */ void append_skai_widget_ai_reply(const char *text) {  }
/* hid_mouse.c */ void append_text_to_mouse_input(void) {  }
/* bloc_peripheral.h */ void audio_subscribe(void) {  }
/* bloc_peripheral.h */ void audio_unsubscribe(void) {  }
/* watch_demo.c */ void back_on_skai_widget(void) {  }
/* watch_demo.c */ void back_tap_cb(void) {  }
/* watch_system_interact.h */ void ble_app_advertising_start(bool mouse_mode, bool pairing_mode) {  }
/* ble_device_manager.h */ int ble_dev_mgr_clear_all(void) { return 0; }
/* ble_device_manager.h */ int ble_dev_mgr_connect_device(uint8_t device_idx) { return 0; }
/* ble_device_manager.h */ int ble_dev_mgr_disconnect_device(uint8_t device_idx) { return 0; }
/* ble_device_manager.h */ int ble_dev_mgr_get_active_device(void) { return 0; }
/* ble_device_manager.h */ int ble_dev_mgr_get_connected_count(void) { return 0; }
/* ble_device_manager.h */ const bonded_devices_db_t * ble_dev_mgr_get_database(void) { return 0; }
/* ble_device_manager.h */ int ble_dev_mgr_register_callback(dev_mgr_event_cb_t cb, void *user_data) { return 0; }
/* ble_device_manager.h */ int ble_dev_mgr_remove_device(uint8_t device_idx) { return 0; }
/* ble_device_manager.h */ int ble_dev_mgr_set_active_device(uint8_t device_idx) { return 0; }
/* main.c */ void ble_dev_mgr_start_main_phone_check_timer(uint32_t interval_ms) {  }
/* ble_device_manager.h */ int ble_dev_mgr_switch_to_next_device(void) { return 0; }
/* app_qrcode.c */ uint8_t ble_get_public_address(bd_addr_t * addr) { return 0; }
/* ble_hid.h */ void ble_hid_set_conn_idx(uint8_t conn_idx) {  }
/* bloc_v2t.h */ bool check_if_user_speaking_to_ai(void) { return false; }
/* bloc_v2t.h */ void clearVoice2Text(void) {  }
/* lv_message_list_layout.c */ void clear_media_widget(void) {  }
/* lv_instruction_list_layout.c */ void clear_skai_widget_ai_reply(void) {  }
/* communicate_task.h */ bool commu_send_battery_level(uint8_t level) { return false; }
/* communicate_task.h */ bool commu_send_calendar_request(void) { return false; }
/* communicate_task.h */ bool commu_send_charge_status(void) { return false; }
/* communicate_task.h */ bool commu_send_sleep_data(void) { return false; }
/* communicate_task.h */ bool commu_send_chat_with_ai(const char *json) { return false; }
/* communicate_task.h */ bool commu_send_dial_change(void) { return false; }
/* communicate_task.h */ bool commu_send_dismiss_notification(const char *id) { return false; }
/* communicate_task.h */ bool commu_send_end_sync_file(void) { return false; }
/* communicate_task.h */ bool commu_send_file_compare_result(uint8_t result) { return false; }
/* communicate_task.h */ bool commu_send_find_mobile(void) { return false; }
/* communicate_task.h */ bool commu_send_get_instruction_img(const char *id) { return false; }
/* communicate_task.h */ bool commu_send_heart_data(int hr) { return false; }
/* communicate_task.h */ bool commu_send_hour_format(void) { return false; }
/* communicate_task.h */ bool commu_send_language(void) { return false; }
/* communicate_task.h */ bool commu_send_linear_acce_buffer(const uint8_t *acce, uint16_t length) { return false; }
/* communicate_task.h */ bool commu_send_media_control(void) { return false; }
/* communicate_task.h */ bool commu_send_oled_display_time(uint8_t time) { return false; }
/* communicate_task.h */ bool commu_send_phone_control_cmd(void) { return false; }
/* communicate_task.h */ bool commu_send_remote_input(const char *json) { return false; }
/* communicate_task.h */ bool commu_send_sport_data(void) { return false; }
/* communicate_task.h */ bool commu_send_start_sync_file(uint32_t total_size) { return false; }
/* communicate_task.h */ bool commu_send_sync_file(const uint8_t *chunk, uint16_t length) { return false; }
/* communicate_task.h */ bool commu_send_update_instruction(const char *json) { return false; }
/* communicate_task.h */ bool commu_send_volume_percentage(uint8_t volume) { return false; }
/* communicate_task.h */ bool commu_send_weather_request(void) { return false; }
/* main.c */ void generate_random_public_address(uint8_t device_id) {  }
/* bloc_v2t.c */ uint8_t get_ai_coding(void) { return 0; }
/* main.c */ bool get_bluetooth_broadcasting_status(void) { return false; }
/* bloc_v2t.h */ char * get_combined_voice2text(void) { return 0; }
/* gesture_recognition_task.h */ int get_gesture_recognition_threshold(void) { return 0; }
/* hid_mouse.c */ bool get_hid_mouse_handfree_mode(void) { return false; }
/* app_gesture.c */ bool get_imu_data_collection_status(void) { return false; }
/* main.c */ uint8_t get_main_phonepeer_conn_idx(void) { return 0; }
/* lv_instruction_list_layout.c */ bool get_skai_input_text_is_null(void) { return false; }
/* bloc_v2t.h */ uint8_t get_speech_coding(void) { return 0; }
/* lv_instruction_list_layout.c */ bool get_voice_recognition_started(void) { return false; }
/* gui_app_pm.h */ bool gui_is_active(void) { return false; }
/* gui_app_pm.h */ void gui_pm_fsm(gui_pm_action_t action) {  }
/* app_media.c */ void handle_media_play_state(bool media_state) {  }
/* app_media.c */ void handle_media_title(char *media_title_text) {  }
/* app_media.c */ void handle_media_widget_play_state(bool media_state) {  }
/* app_media.c */ void handle_media_widget_title(char *media_title_text) {  }
/* bloc_v2t.h */ void handle_v2t_result(VOICE_RECOGNITION_PAYLOAD *msgData) {  }
/* app_gesture.c */ void imu_lock_sw_event_callback(lv_event_t *e) {  }
/* app_gesture.c */ void imu_raw_data_collection_sw_event_callback(lv_event_t *e) {  }
/* app_incoming_call.h */ void incoming_call_close_if_active(const char *id) {  }
/* app_incoming_call.h */ void incoming_call_set_caller(const char *title, const char *id, uint8_t type) {  }
/* main_functions.h */ void init_gesture_recognition_release_model(void) {  }
/* bloc_v2t.h */ bool isTextEmpty(void) { return false; }
/* communicate_update_image.h */ bool is_ble_dfu_thread_running(void) { return false; }
/* log_file_backend.c */ int log_file_backend_is_enabled(void) { return 0; }
/* log_file_backend.c */ int log_file_backend_set_enabled(int enable) { return 0; }
/* PC sim: return a minimal lv_obj on `parent` so callers that immediately
 * lv_obj_align/lv_obj_set_size on the result don't NULL-deref. */
/* lv_message_list_layout.c */ lv_obj_t * lv_media_widget_builder(lv_obj_t * parent) { return parent ? lv_obj_create(parent) : NULL; }
/* lv_instruction_list_layout.c */ lv_obj_t * lv_skai_widget_builder(lv_obj_t * parent) { return parent ? lv_obj_create(parent) : NULL; }
/* ui_helper.h */ void lvgl_set_global_keypad_enter_cmd(void) {  }
/* ui_helper.h */ void lvgl_set_global_keypad_esc_cmd(void) {  }
/* lv_message_list_layout.c */ void media_widget_start(void) {  }
/* lv_instruction_list_layout.c */ void media_widget_stop(void) {  }
/* lv_instruction_list_layout.c */ void media_widget_tap_event_cb(void) {  }
/* lv_instruction_list_layout.c */ void media_widget_trigger_drag_by_py(int p_y) {  }
/* hid_mouse.c */ void mouse_apply_v2t_input(const char *text) {  }
/* hid_mouse.c */ void mouse_mode_handle_media_title(const char *title) { (void)title; }
/* hid_mouse.c */ void mouse_mode_handle_media_play_state(bool playing) { (void)playing; }
/* lv_instruction_list_layout.c */ void open_skai_widget_ai(bool open) {  }
/* hid_mouse.c */ void open_v2t_mic(void) {  }
/* app_gesture.c */ bool pause_sleep_cause_of_imu_reson(void) { return false; }
/* ble_hid.h */ void play_next_through_hid(void) {  }
/* ble_hid.h */ void play_pause_through_hid(void) {  }
/* ble_hid.h */ void play_prev_through_hid(void) {  }
/* app_clock_status_bar.c */ void refresh_connected_device_label(void) {  }
/* app_mainmenu.c */ void reset_ai_coding(void) {  }
/* app_system_interface.c */ void reset_skai_widget_input_text(void) {  }
/* bloc_v2t.h */ void reset_speech_coding(void) {  }
/* bloc_v2t.h */ void reset_user_speaking_to_ai(void) {  }
/* lv_instruction_list_layout.c */ void send_to_ai(void) {  }
/* app_media.c */ void set_app_vol_bar_value(uint8_t volume) {  }
/* app_clock_main.c */ void set_dial_media_widget_opa(uint8_t opa) {  }
/* gesture_recognition_task.h */ void set_gesture_recognition_threshold(int threshold) {  }
/* lv_instruction_list_layout.c */ void set_skai_widget_awaiting_ai(void) {  }
/* lv_instruction_list_layout.c */ void set_skai_widget_input_text(const char *text) {  }
/* lv_instruction_list_layout.c */ void set_skai_widget_opa(uint8_t opa) {  }
/* app_skai.c */ void set_skai_widget_processing_text(const char *text) {  }
/* app_media.c */ void set_widget_vol_bar_value(uint8_t volume) {  }
/* lv_instruction_list_layout.c */ bool skai_widget_has_ai_reply(void) { return false; }
/* communicate_protocol.h */ void skaiwatch_ble_set_performance(ble_perf_level_t level) {  }
/* app_mainmenu.c */ rt_int32_t speech_on_pause(void) { return 0; }
/* app_mainmenu.c */ rt_int32_t speech_on_resume(void) { return 0; }
/* bloc_v2t.h */ void start_voice_recognition(uint8_t intent) {  }
/* bloc_v2t.h */ void stop_voice_recognition(uint8_t intent) {  }
/* gui_app_pm.h */ void sys_poweron_fsm(sys_pwron_evt_t evt) {  }
/* hid_mouse.c */ void toggle_keyboard_visibility(void) {  }
/* gesture_model_loader.h */ int unload_release_model(void) { return 0; }
/* ble_hid.h */ void volume_down_through_hid(void) {  }
/* ble_hid.h */ void volume_up_through_hid(void) {  }
/* hid_mouse.c */ void watch_system_mouse_pause(void) {  }
/* hid_mouse.c */ void watch_system_mouse_resume(void) {  }
/* hid_mouse.c */ void mouse_skaibar_set_options_json(const char *json) { (void)json; }

/* Manually-stubbed symbols not found by genstub: */
#include "lvgl/lvgl.h"
#include "bloc_v2t.h"

/* gesture indicator on layer-top: real impl now lives in lvsf_gesture.c
 * (re-enabled by switching APP_TRANS_ANIMATION_NONE → OVERWRITE).
 * Don't stub here. */

/* IMU collection state globals — gesture/exercise app provides them on ARM */
bool imu_data_collection = false;
bool imu_raw_data_collection = false;

/* LVGL image referenced by interact app (mobile icon). Empty descriptor. */
const lv_img_dsc_t mobile = {0};

/* Watchdog driver init — no WDT on PC */
int rt_wdt_init(void) { return 0; }

/* Voice provider struct — bloc_v2t module excluded; expose empty instance */
VoiceProvider voice_provider = {0};

/* bloc_v2t.h — referenced by watch_system_interact MSH_CMD handlers (now
 * pulled in by FINSH on PC sim), but bloc_v2t.c is excluded above. */
void setVoice2Text(char *text) { (void)text; }

/* BLE RSSI checker — real impl in main.c (HCPU firmware) which is excluded on
 * PC sim; watch_system_interact.c calls these in wakeup/sleep paths so we need
 * empty stubs to satisfy the linker. */
#include <stdint.h>
void start_ble_rssi_checker(uint32_t period_ms) { (void)period_ms; }
void stop_ble_rssi_checker(void) { }
void app_voice_set_voice2text_intent(uint8_t intent) { (void)intent; }
