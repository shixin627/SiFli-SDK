/**
 ******************************************************************************
 * @file   gesture_recognition_task.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk
 * integrated circuit in a product or a software update for such product, must
 * reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered,
 * decompiled, modified, or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rtthread.h>
#include <board.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Project headers */
#include "gesture_recognition_task.h"
#include "ui_helper.h"
#include "ui_handler.h"
#include "drv_touch.h"
#include "gui_app_fwk.h"
#include "app_mainmenu.h"
#include "common_widget.h"
#include "bf0_hal_fft.h"
#include "real_fft.h"

/* Conditional includes */
#include "watch_sys_service.h"
#include "communicate_protocol.h"
#include "communicate_task.h"
#include "communicate_update_image.h"
#include "main_functions.h"
#include "constants.h"
#include "gesture_handler.h"
#include "watch_system_interact.h"
#include "bloc_peripheral.h"
#include "bloc_setting.h"
#include "bloc_notification.h"
#include "bloc_control.h"
#include "bloc_v2t.h"
#include "bloc_motion_tracking.h"
#include "watch_global_data.h"
#include "gui_app_pm.h"  /* always — gui_is_active() gates the inference loop */

#define DBG_TAG "APP.GESTURE"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

/* Defined next to the inference loop; declared here so the recognition path
   above it can report the model's verdict on the same channel. */
static void gesture_stage2_report(const char *what);

/* External function from app_gesture.c for data collection */
extern void app_gesture_receive_imu_data(int16_t (*dataset)[6], int sample_num);

static rt_thread_t gesture_recognition_thread = RT_NULL;
static rt_thread_t send_gesture_data_thread = RT_NULL;

/* Buffers for sensor data processing */
static float identifyWindow[TAP_TARGET_SAMPLE_NUM][4];
static float release_identifyWindow[RELEASE_TARGET_SAMPLE_NUM][4];
static float ppgidentifyWindow[16][kChannelReleaseNumber];

/**
 * @brief Handle detected gesture by updating gesture index
 *
 * @param label The gesture label detected by the recognition algorithm
 */
static void handle_gesture(int label)
{
    if (label >= 0 && label < kGestureCount)
    {
        setGestureIndex(label);
    }
}

static void get_gesture_data(gesture_data_t *gesture, int sample_num,
                             uint8_t divide_rate)
{
    if (sample_num > TAP_TARGET_SAMPLE_NUM)
    {
        for (int i = 0; i < sample_num; i++)
        {
            int index = i * divide_rate;
            float acceleration_x =
                gesture->dataset[index].x / INT16_to_G * GRAVITY;
            float acceleration_y =
                gesture->dataset[index].y / INT16_to_G * GRAVITY;
            float acceleration_z =
                gesture->dataset[index].z / INT16_to_G * GRAVITY;
            release_identifyWindow[i][0] = acceleration_x;
            release_identifyWindow[i][1] = acceleration_y;
            release_identifyWindow[i][2] = acceleration_z;
            release_identifyWindow[i][3] = gesture->dataset[index].ppg_data;
        }
    }
    else
    {
        for (int i = 0; i < sample_num; i++)
        {
            int index = i * divide_rate;
            float acceleration_x =
                gesture->dataset[index].x / INT16_to_G * GRAVITY;
            float acceleration_y =
                gesture->dataset[index].y / INT16_to_G * GRAVITY;
            float acceleration_z =
                gesture->dataset[index].z / INT16_to_G * GRAVITY;
            identifyWindow[i][0] = acceleration_x;
            identifyWindow[i][1] = acceleration_y;
            identifyWindow[i][2] = acceleration_z;
            identifyWindow[i][3] = gesture->dataset[index].ppg_data;
        }
        // PPG min-max normalization to [-1, 1]
        float ppg_min = identifyWindow[0][3];
        float ppg_max = identifyWindow[0][3];
        for (int i = 1; i < sample_num; i++)
        {
            if (identifyWindow[i][3] < ppg_min)
                ppg_min = identifyWindow[i][3];
            if (identifyWindow[i][3] > ppg_max)
                ppg_max = identifyWindow[i][3];
        }
        float ppg_range = ppg_max - ppg_min;
        if (ppg_range > 0.0f)
        {
            for (int i = 0; i < sample_num; i++)
            {
                identifyWindow[i][3] =
                    2.0f * (identifyWindow[i][3] - ppg_min) / ppg_range - 1.0f;
            }
        }
        else
        {
            for (int i = 0; i < sample_num; i++)
            {
                identifyWindow[i][3] = 0.0f;
            }
        }
    }
}

static void get_gesture_data_with_ppg(gesture_data_t *dataset, int sample_num,
                                      uint8_t divide_rate)
{
    for (int i = 0; i < sample_num; i++)
    {
#if kChannelReleaseNumber == 4
        if (i == 0)
        {
            continue;
        }
        ppgidentifyWindow[i - 1][0] =
            dataset->dataset_ppg[i].acce.x / INT16_to_G * GRAVITY;
        ppgidentifyWindow[i - 1][1] =
            dataset->dataset_ppg[i].acce.y / INT16_to_G * GRAVITY;
        ppgidentifyWindow[i - 1][2] =
            dataset->dataset_ppg[i].acce.z / INT16_to_G * GRAVITY;
        float diff_ppg =
            dataset->dataset_ppg[i].ppg - dataset->dataset_ppg[i - 1].ppg;
        ppgidentifyWindow[i - 1][3] = diff_ppg;
#else
        int index = i * divide_rate;
        ppgidentifyWindow[i][0] =
            dataset->dataset_ppg[index].acce.x / INT16_to_G * GRAVITY;
        ppgidentifyWindow[i][1] =
            dataset->dataset_ppg[index].acce.y / INT16_to_G * GRAVITY;
        ppgidentifyWindow[i][2] =
            dataset->dataset_ppg[index].acce.z / INT16_to_G * GRAVITY;
#endif
    }
}

static bool gesture_recognition_lock = false;
static uint8_t unknown_gesture_count = 0;
static rt_tick_t last_gesture_recognition_time = 0;
static rt_timer_t gesture_lock_timer = RT_NULL;
static void gesture_lock_timer_callback(void *parameter)
{
    gesture_recognition_lock = false;
    unknown_gesture_count = 0;
    if (gesture_lock_timer != RT_NULL)
    {
        rt_timer_delete(gesture_lock_timer);
        gesture_lock_timer = RT_NULL;
    }
}

static void trigger_gesture_unlock_timer(void)
{
    if (gesture_lock_timer == RT_NULL)
    {
        gesture_lock_timer =
            rt_timer_create("gesture_lock_timer", gesture_lock_timer_callback,
                            RT_NULL, 1000, RT_TIMER_FLAG_ONE_SHOT);
    }

    if (gesture_lock_timer != RT_NULL)
    {
        rt_timer_start(gesture_lock_timer);
    }
}

extern bool imu_data_collection;
extern bool imu_data_collection_error;

int tap_recognition_score;
int release_recognition_score;
int gesture_recognition_threshold = 17;
void set_gesture_recognition_threshold(int threshold)
{
    // if (threshold >= 50 && threshold <= 100)
    {
        gesture_recognition_threshold = threshold;
    }
}
int get_gesture_recognition_threshold(void)
{
    return gesture_recognition_threshold;
}

extern bool get_switch_freehand_mode(void);
extern bool get_switch_mouse_scroll_mode(void);
extern bool get_scroll_up_mode(void);
extern bool get_hid_mouse_handfree_mode(void);
extern void set_hid_mouse_handfree_mode(void);
extern bool level_bar_is_flat(void);
static void gesture_recognition_algorithm(gesture_data_t *gesture)
{
    LOG_D("gesture_recognition_algorithm sample_num:%d", gesture->sample_num);
    uint8_t sample_num = gesture->sample_num;
#if 1 // !kReleaseMode
    if (imu_data_collection)
    {
        if (imu_data_collection_error)
        {
            // incorrect tap gesture
            if (sample_num == TAP_TARGET_SAMPLE_NUM &&
                (!SkaiWatchSys.motion_control_lock &&
                 gui_app_is_actived(APP_ID_GESTURE)))
            {
                get_gesture_data(gesture, TAP_TARGET_SAMPLE_NUM, 1);
                rt_tick_t tick_time_start = rt_tick_get();
                tap_recognition_score =
                    recognize_gesture_release(identifyWindow);
                rt_tick_t tick_time_end = rt_tick_get();
                rt_tick_t cost_tick = tick_time_end - tick_time_start;
                LOG_D("recognize tap gesture cost_tick:%d, score:%d", cost_tick,
                      tap_recognition_score);
                if (tap_recognition_score == 1)
                {
                    packMatrixToBuffer(gsensorSamplesBuffer, gesture->dataset,
                                       NULL, TAP_TARGET_SAMPLE_NUM);
                    commu_send_linear_acce_buffer(gsensorSamplesBuffer,
                                                  TAP_TARGET_SAMPLE_NUM *
                                                      BYTES_PER_SAMPLE);
                }
            }
            // incorrect release gesture
            else if (sample_num == RELEASE_TARGET_SAMPLE_NUM &&
                     (SkaiWatchSys.motion_control_lock &&
                      gui_app_is_actived(APP_ID_GESTURE)))
            {
                get_gesture_data(gesture, sample_num, 1);
                rt_tick_t tick_time_start = rt_tick_get();
                release_recognition_score =
                    recognize_gesture_release(identifyWindow);
                rt_tick_t tick_time_end = rt_tick_get();
                rt_tick_t cost_tick = tick_time_end - tick_time_start;
                LOG_D("recognize release gesture cost_tick:%d, score:%d",
                      cost_tick, release_recognition_score);
                if (release_recognition_score == 0)
                {
                    packMatrixToBuffer(gsensorSamplesBuffer, gesture->dataset,
                                       NULL, sample_num);
                    commu_send_linear_acce_buffer(
                        gsensorSamplesBuffer, sample_num * BYTES_PER_SAMPLE);
                }
            }
        }
        else
        {
            packMatrixToBuffer(gsensorSamplesBuffer, gesture->dataset, NULL,
                               sample_num);
            commu_send_linear_acce_buffer(gsensorSamplesBuffer,
                                          sample_num * BYTES_PER_SAMPLE);
            static int16_t converted_data[RELEASE_TARGET_SAMPLE_NUM][6];
            for (int i = 0; i < sample_num && i < RELEASE_TARGET_SAMPLE_NUM;
                 i++)
            {
                converted_data[i][0] = gesture->dataset[i].x;
                converted_data[i][1] = gesture->dataset[i].y;
                converted_data[i][2] = gesture->dataset[i].z;
                converted_data[i][3] = gesture->dataset[i].gravity_x;
                converted_data[i][4] = gesture->dataset[i].gravity_y;
                converted_data[i][5] = gesture->dataset[i].gravity_z;
            }
            app_gesture_receive_imu_data(converted_data, sample_num);
        }
    }
    else
#endif // !kReleaseMode
        if (gesture_recognition_lock)
        {
            LOG_D("gesture_recognition_algorithm is locked, ignore gesture");
            return;
        }
        else if (sample_num == TAP_TARGET_SAMPLE_NUM)
        {
            get_gesture_data(gesture, sample_num, 1);

            int label = kNoGesture;
            rt_tick_t tick_time_start = rt_tick_get();
            tap_recognition_score = recognize_gesture_release(identifyWindow);
            last_gesture_recognition_time = rt_tick_get();
            LOG_I("recognize tap gesture cost_tick:%d, score:%d",
                  last_gesture_recognition_time - tick_time_start,
                  tap_recognition_score);
            {
                /* The model's verdict on the same channel as the CUT lines, so
                   the phone shows the whole chain: window cut → gate passed →
                   score. score==1 is the one that fires a gesture. */
                char verdict[24];
                rt_snprintf(verdict, sizeof(verdict), "score=%d",
                            tap_recognition_score);
                gesture_stage2_report(verdict);
            }
            if (tap_recognition_score == 1)
            {
                if (app_control_get_mouse_mode())
                {
                    motor_pattern_scrolling_app();
                    if (get_switch_freehand_mode())
                    {
                        /* 2026-07-17 founder 要求關閉「tap 手勢 toggle 飛鼠」:正常戴姿(freehand
                           姿態區)tap 誤辨識會偷偷開飛鼠,又被 air_mouse 的姿態 gate 擋住不顯形,
                           直到顛倒手錶(姿態離開 gate 區間)才突然開始飛。滑鼠 app 的飛鼠只留
                           「頂部按住」一條明確路徑。恢復=取消註解下行。
                        set_hid_mouse_handfree_mode(); */
                    }
                    else if (get_switch_mouse_scroll_mode())
                    {
                        control_provider.ble_hid_keyboard_scroll_page(
                            get_scroll_up_mode());
                    }
                    else if (get_hid_mouse_handfree_mode())
                    {
                        control_provider.ble_hid_mouse_left_click();
                    }
                }
                else
                {
                    label = kTapGesture;
                    send_virtual_gesture_event(GESTURE_EVENT_PRESS);
                    extern bool get_enable_tap_and_hold(void);
                    if (get_enable_tap_and_hold())
                    {
                        // if (watch_sys_sync.is_ppg_enabled())
                        watch_sys_sync.notify_tap_detected();
                    }
                    else
                    {
                        rt_thread_mdelay(100);
                        send_virtual_gesture_event(GESTURE_EVENT_TAP);
                    }
                }
            }
            else if (tap_recognition_score == 0)
            {
                if (app_control_get_mouse_mode())
                {
                    motor_pattern_scrolling_app();
                    control_provider.ble_hid_keyboard_multitask(true);
                }
                else
                {
                    extern bool chat_page_is_open(void);
                    if (chat_page_is_open())
                    {
                        /* Chat panel is on top of the @-list (kept open underneath for the
                           back-gesture), so is_at_instruction_list() below would also match —
                           check chat FIRST or release silently opens the hidden list AI widget
                           instead of the chat mic. */
                        LOG_D("Unlock gesture recognized at chat page, opening chat mic");
                        extern bool get_bluetooth_connection_status(void);
                        if (get_bluetooth_connection_status())
                            motor_pattern_unlocked();
                        extern void chat_page_start_voice_input(void);
                        chat_page_start_voice_input();
                    }
                    else if (is_at_instruction_list())
                    {
                        LOG_D("Unlock gesture recognized at instruction list, "
                              "unlocking watch and open AI widget");
                        /* Haptic = "successfully triggered". When disconnected the
                           box won't open (animate_open_ai_widget just shows the
                           not-connected tip) — so skip the buzz. */
                        extern bool get_bluetooth_connection_status(void);
                        if (get_bluetooth_connection_status())
                            motor_pattern_unlocked();
                        extern void animate_open_ai_widget(void);
                        animate_open_ai_widget();
                    }
                    else if (level_bar_is_flat())
                    {
                        LOG_D("Unlock gesture recognized, unlocking watch");
                        handle_gesture_unlock();
                    }
                }
            }
            else
            {
                control_provider.trigger_unknown_event();
                unknown_gesture_count++;
                if (unknown_gesture_count >= 3 &&
                    last_gesture_recognition_time - tick_time_start > 1000)
                {
                    gesture_recognition_lock = true;
                    trigger_gesture_unlock_timer();
                }
            }
            handle_gesture(label);
        }
        else if (sample_num == RELEASE_TARGET_SAMPLE_NUM)
        {
            get_gesture_data(gesture, sample_num, 1);

            rt_tick_t tick_time_start = rt_tick_get();
            release_recognition_score =
                recognize_gesture_release(release_identifyWindow);
            rt_tick_t tick_time_end = rt_tick_get();
            rt_tick_t cost_tick = tick_time_end - tick_time_start;
            if (release_recognition_score > gesture_recognition_threshold)
            {
                handle_gesture_unlock();
                LOG_D("recognize release gesture cost_tick:%d, score:%d",
                      cost_tick, release_recognition_score);
            }
            else
            {
                LOG_D("recognize unknown gesture cost_tick:%d, score:%d",
                      cost_tick, release_recognition_score);
            }
        }
}

#define IMU_THREAD_STACK_SIZE 4 * 1024
#define IMU_THREAD_PRIORITY RT_THREAD_PRIORITY_MIDDLE - 1
#define IMU_THREAD_TIMESLICE 10
extern bool get_motor_status(void);

/**
 * @brief Report what stage 2 did with a window that stage 1 handed over.
 *
 * Stage 1 (bloc_motion_tracking) logs every window it cuts as a `CUT ...`
 * line. Without this, a window that clears stage 1 and then hits one of the
 * six gates below disappears with no trace — which on the log reads exactly
 * like stage 1 having missed the gesture. Measured: 20 windows emitted, only
 * 2-3 ever reached the model, and nothing said why.
 *
 * Same dual transport as the CUT lines: UART for bench work, BLE because a
 * walking test can't drag a cable along. Dev builds only.
 */
static void gesture_stage2_report(const char *what)
{
#if !kReleaseMode
    LOG_I("GST %s", what);
    #ifdef BSP_USING_COMMUNICATE
    char line[48];
    rt_snprintf(line, sizeof(line), "GST %s", what);
    commu_send_bluetooth_log(line);
    #endif
#else
    (void)what;
#endif
}

static void gesture_recognition_thread_entry(void *parameter)
{
    init_gesture_recognition_release_model();

    watch_sensor.gesture_sem =
        rt_sem_create("gesture_sem", 0, RT_IPC_FLAG_FIFO);

    static Vector3 accData, gyroData, magData;
    while (1)
    {
        rt_sem_take(watch_sensor.gesture_sem, RT_WAITING_FOREVER);

        /* GUI suspended (screen off): skip TFLite inference. LCPU still posts
           IMU samples but HCPU returns to sleep without running the model. */
        if (!gui_is_active())
        {
            gesture_stage2_report("drop=gui-off");
            continue;
        }

        // if (SkaiWatchSys.charger_status != NoCharge)
        // {
        //     continue;
        // }
        if (!SkaiWatchSys.flag_field.is_wearing)
        {
            gesture_stage2_report("drop=not-worn");
            continue;
        }

        if (is_user_touching_screen())
        {
            gesture_stage2_report("drop=touching");
            continue;
        }

        if (!is_at_instruction_list())
        {
            if (!open_gesture_model())
            {
                gesture_stage2_report("drop=model-off");
                continue;
            }

            // 飛鼠手持模式下
            if (app_control_get_mouse_mode() &&
                !get_hid_mouse_handfree_mode() && !get_switch_freehand_mode())
            {
                // LOG_D("DEBUG 1:%d,%d", app_control_get_mouse_mode(),
                //       get_hid_mouse_handfree_mode());
                gesture_stage2_report("drop=mouse-mode");
                continue;
            }

            if (has_user_started_controlling_with_arm())
            {
                gesture_stage2_report("drop=arm-control");
                continue;
            }
        }

        if (!is_user_touching_screen() && !get_motor_status())
        {
            gesture_stage2_report("run");
            gesture_recognition_algorithm(&watch_sensor.gesture_data);
        }
        else
        {
            /* The tail condition had no else — a window blocked by the motor
               (haptic feedback still running) vanished silently. */
            gesture_stage2_report("drop=motor");
        }
    }
}

/**
 * @brief Thread initialization function for gesture recognition
 *
 * @return RT_EOK on success, -RT_ERROR on failure
 */
static int gesture_recognition_thread_init(void)
{
    gesture_recognition_thread = rt_thread_create(
        "gesture_recognition", gesture_recognition_thread_entry, RT_NULL,
        IMU_THREAD_STACK_SIZE, IMU_THREAD_PRIORITY, IMU_THREAD_TIMESLICE);
    if (gesture_recognition_thread != RT_NULL)
    {
        rt_thread_startup(gesture_recognition_thread);
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}
#ifndef BSP_USING_PC_SIMULATOR
INIT_APP_EXPORT(gesture_recognition_thread_init);
#endif

#if defined(BSP_USING_PM) && !kReleaseMode
extern void set_gravity_position(int position);
static int utest_gesture(int argc, char *argv[])
{
    if (argc >= 2)
    {
        if (strcmp(argv[1], "release") == 0)
        {
            handle_gesture_unlock();
        }
        else if (strcmp(argv[1], "tap") == 0)
        {
            send_virtual_gesture_event(GESTURE_EVENT_PRESS);
            extern bool get_enable_tap_and_hold(void);
            if (get_enable_tap_and_hold())
            {
                // if (watch_sys_sync.is_ppg_enabled())
                watch_sys_sync.notify_tap_detected();
            }
            else
            {
                rt_thread_mdelay(100);
                send_virtual_gesture_event(GESTURE_EVENT_TAP);
            }
        }
        else if (strcmp(argv[1], "movement") == 0)
        {
            // gesture_recognition_lock = true;
            // trigger_gesture_unlock_timer();
        }
        else if (strcmp(argv[1], "wrist_pronation") == 0)
        {
            send_virtual_gesture_event(GESTURE_EVENT_WRIST_PRONATION);
        }
        else if (strcmp(argv[1], "back") == 0)
        {
            send_virtual_gesture_event(GESTURE_EVENT_BACK);
        }
        else if (strcmp(argv[1], "raise_hand") == 0)
        {
            set_gravity_position(GRAVITY_POSITION_AI);
        }
        else if (strcmp(argv[1], "test_cb") == 0)
        {
        }
        else if (strcmp(argv[1], "instruction") == 0)
        {
            const char *test_json =
                "{\"id\":\"dc396755-43ce-4fdb-ade4-d1ad683140c3\","
                "\"title\":\"每分鐘提醒走動\",\"version\":1,"
                "\"trigger\":{\"type\":\"interval\",\"intervalSeconds\":60}}";
            add_or_update_custom_instruction(
                "dc396755-43ce-4fdb-ade4-d1ad683140c3", "每分鐘提醒走動",
                "interval", 60, true, 1, NULL);
            /* MSH commands run on tshell thread; defer the rebuild to the
               LVGL thread (same reason as the BLE notify path). */
            lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_INSTRUCTION_LIST};
            lvgl_send_msg(msg);
            rt_kprintf("Test instruction added\n");
        }
    }
    return 0;
}
MSH_CMD_EXPORT(utest_gesture, "utest_gesture [OPTION] ...");
#endif /* BSP_USING_PM */

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/