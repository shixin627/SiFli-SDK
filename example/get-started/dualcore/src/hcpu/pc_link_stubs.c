/* Auto-generated stubs for PC simulator (do not edit by hand, mostly).
 * Covers symbols normally provided by ARM-only modules
 * (BLE stack, voice/skai/gesture apps, IPC peripherals, etc.).
 * Each function logs once and returns a sensible default.
 *
 * Manual hand-edit zone: ble_dev_mgr_* (8 entries) are removed below — real
 * fake implementations live in modules/tests/fake_ui_data.c so the device-
 * list UI shows something on PC. Symbols also removed from _syms_clean.txt
 * so the next _genstub.py run does not re-add them.
 *
 * T1 part 3 (ARM seam): hid_mouse.c is no longer in gui_apps pc_skip, so its
 * 10 own functions (append_text_to_mouse_input, open_v2t_mic, watch_system_
 * mouse_pause/resume, ...) are now provided by the real TU and were removed
 * from here (were LNK2005 duplicates). The 5 ARM-only symbols hid_mouse
 * *calls* (BLE_HID_Mouse_Touch_*, air_mouse_movement_lock_reset,
 * set_voice_recognition_notified_from_mouse) are stubbed in the manual zone
 * below. TODO: add these to _syms_clean.txt handling so _genstub.py neither
 * re-adds the 10 nor dueling-defines the 5. */

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
/* bloc_peripheral.h */ void audio_subscribe(void) {  }
/* bloc_peripheral.h */ void audio_unsubscribe(void) {  }
/* watch_demo.c */ void back_on_skai_widget(void) {  }
/* watch_demo.c */ void back_tap_cb(void) {  }
/* watch_system_interact.h */ void ble_app_advertising_start(bool mouse_mode, bool pairing_mode) {  }
/* ble_device_manager.h: 8 ble_dev_mgr_* functions moved to
 * modules/tests/fake_ui_data.c to back the UI device list. Only the timer
 * helper (declared in main.c, not a database accessor) stays here. */
/* main.c */ void ble_dev_mgr_start_main_phone_check_timer(uint32_t interval_ms) {  }
/* app_qrcode.c */ uint8_t ble_get_public_address(bd_addr_t * addr) { return 0; }
/* ble_hid.h */ void ble_hid_set_conn_idx(uint8_t conn_idx) {  }
/* bloc_v2t.h */ bool check_if_user_speaking_to_ai(void) { return false; }
/* bloc_v2t.h */ void clearVoice2Text(void) {  }
/* lv_message_list_layout.c */ void clear_media_widget(void) {  }
/* lv_instruction_list_layout.c */ void clear_skai_widget_ai_reply(void) {  }
/* communicate_task.h */ bool commu_send_active_device(const char *device_id) { (void)device_id; return false; }
/* communicate_task.h */ bool commu_send_battery_level(uint8_t level) { return false; }
/* communicate_task.h */ bool commu_send_battery_voltage(uint16_t millivolts) { (void)millivolts; return false; }
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
/* communicate_task.h */ bool commu_send_heart_curve_sample(uint32_t timestamp, uint8_t bpm) { return false; }
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
/* lv_instruction_list_layout.c */ void open_skai_widget_ai(bool open) {  }
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
/* bloc_v2t.h */ void voice_set_pending_v2t_intent(uint8_t intent) {  }
/* bloc_v2t.h */ void set_voice_recognition_started(bool started) {  }
/* communicate_task.h */ bool commu_send_skaibar_selected(uint8_t idx) { return false; }
/* communicate_task.h */ bool commu_send_skaibar_committed(uint8_t idx) { return false; }
/* gui_app_pm.h */ void sys_poweron_fsm(sys_pwron_evt_t evt) {  }
/* gesture_model_loader.h */ int unload_release_model(void) { return 0; }
/* ble_hid.h */ void volume_down_through_hid(void) {  }
/* ble_hid.h */ void volume_up_through_hid(void) {  }

/* Manually-stubbed symbols not found by genstub: */
#include "lvgl/lvgl.h"
#include "bloc_v2t.h"

/* disp_refr_governor.c (layer-2 indev throttle) queries the hardware touch
   indev accessor from lv_touch.c. The PC sim uses lv_touch_sim instead, so
   that symbol isn't linked here — return NULL (the governor null-checks it,
   so layer-2 just no-ops in the sim). */
lv_indev_t *touch_get_indev_handler(void) { return NULL; }

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

/* T1 part 3 (ARM seam): the 5 ARM-only symbols hid_mouse.c CALLS — BLE HID
 * touch reports (ble_hid.c), air-mouse movement lock (bloc_motion_tracking.c),
 * and the v2t-notified flag (bloc_v2t.c) — all excluded on PC sim. Empty
 * stubs so the now-PC-compiled hid_mouse UI links. */
void BLE_HID_Mouse_Touch_Press(uint16_t x, uint16_t y) { (void)x; (void)y; }
void BLE_HID_Mouse_Touch_Move(uint16_t x, uint16_t y) { (void)x; (void)y; }
bool BLE_HID_Mouse_Touch_Release(uint16_t x, uint16_t y) { (void)x; (void)y; return false; }
void air_mouse_movement_lock_reset(void) { }
void set_voice_recognition_notified_from_mouse(bool status) { (void)status; }
