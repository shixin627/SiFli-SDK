/**
 ******************************************************************************
 * @file   app_test.c
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
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "watch_system_core_task.h"
#include "gesture_handler.h"
#include "watch_global_data.h"
#ifdef BSP_USING_BLOC
#include "bloc_v2t.h"
#include "bloc_setting.h"
#include "bloc_peripheral.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif
#ifdef BSP_USING_COMMUNICATE
#include "communicate_protocol.h"
#include "communicate_task.h"
#endif
#define DBG_TAG "app.test"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#if (kReleaseMode == 0)

#define APP_ID "test"


typedef enum
{
    TEST_MODE_SELECT = 0,
    TEST_MODE_NORMAL,
    TEST_MODE_STRESS,
} test_mode_t;

typedef enum
{
    TEST_STATE_INIT = 0,
    TEST_STATE_AMOLED,
    TEST_STATE_TOUCH,
    TEST_STATE_SENSOR_IMU,
    TEST_STATE_SENSOR_PPG,
    TEST_STATE_SOUND_RECORD,
    TEST_STATE_SOUND_PLAY,
    TEST_STATE_MOTOR,
    TEST_STATE_OK,
    TEST_STATE_EXIT,
} test_state_t;

static test_mode_t test_mode = TEST_MODE_SELECT;
static test_state_t test_state = TEST_STATE_INIT;
static volatile bool stress_test_running = false;

static bool check_amoled();
static bool wait_for_touch();
static bool check_imu_sensor();
static bool check_ppg_sensor();
static bool record_with_mic(int seconds);
static bool play_with_speaker(int seconds);
static bool run_motor(int seconds);
static void send_ok_to_phone_and_exit();
static void exit_app();

static rt_thread_t test_thread = RT_NULL;
static rt_thread_t test_operational_thread = RT_NULL;
static rt_thread_t cpu_stress_thread = RT_NULL;
static rt_thread_t amoled_thread = RT_NULL;
static rt_thread_t imu_thread = RT_NULL;
static rt_thread_t ppg_thread = RT_NULL;
static rt_thread_t motor_thread = RT_NULL;

static lv_obj_t *test_label;
static lv_obj_t *status_label;
static lv_obj_t *mode_select_container;

// CPU intensive stress test functions
static void cpu_stress_thread_entry(void *parameter)
{
    LOG_D("CPU stress test thread started");
    uint32_t counter = 0;
    double result = 0.0;

    while (stress_test_running)
    {
        // Mix of integer and floating point operations for maximum CPU usage

        // 1. Prime number calculation (CPU intensive)
        uint32_t n = 1000 + (counter % 10000);
        for (uint32_t i = 2; i <= n; i++)
        {
            bool is_prime = true;
            for (uint32_t j = 2; j * j <= i; j++)
            {
                if (i % j == 0)
                {
                    is_prime = false;
                    break;
                }
            }
            if (is_prime)
                result += i;
        }

        // 2. Floating point intensive operations
        for (int i = 0; i < 100; i++)
        {
            result += sin(counter * 0.1) * cos(counter * 0.2);
            result += sqrt(fabs(result)) * tan(counter * 0.05);
            result += pow(2.5, (counter % 10)) / (result + 1.0);
        }

        // 3. Matrix multiplication simulation
        float matrix_a[16], matrix_b[16], matrix_c[16];
        for (int i = 0; i < 16; i++)
        {
            matrix_a[i] = sin(counter + i);
            matrix_b[i] = cos(counter - i);
        }
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                matrix_c[i * 4 + j] = 0;
                for (int k = 0; k < 4; k++)
                {
                    matrix_c[i * 4 + j] += matrix_a[i * 4 + k] * matrix_b[k * 4 + j];
                }
            }
        }

        // 4. Hash-like computation (bit operations)
        uint32_t hash = counter;
        for (int i = 0; i < 100; i++)
        {
            hash = ((hash << 5) + hash) ^ (hash >> 27);
            hash *= 0x5bd1e995;
            hash ^= hash >> 15;
        }

        counter++;

        // Minimal delay to keep CPU at maximum load
        rt_thread_mdelay(1);
    }

    LOG_D("CPU stress test thread stopped, iterations: %lu", counter);
}

static void amoled_stress_thread_entry(void *parameter)
{
    LOG_D("AMOLED stress thread started");
    lv_color_t colors[] = {
        LV_COLOR_WHITE,
        LV_COLOR_RED,
        LV_COLOR_ORANGE,
        LV_COLOR_YELLOW,
        LV_COLOR_GREEN,
        LV_COLOR_BLUE,
        LV_COLOR_NAVY,
        LV_COLOR_PURPLE,
    };
    int num_colors = sizeof(colors) / sizeof(colors[0]);
    int idx = 0;

    while (stress_test_running)
    {
        lv_obj_set_style_bg_color(lv_scr_act(), colors[idx % num_colors], LV_PART_MAIN);
        idx++;
        rt_thread_mdelay(500);
    }
    LOG_D("AMOLED stress thread stopped");
}

static void imu_stress_thread_entry(void *parameter)
{
    LOG_D("IMU stress thread started");

    while (stress_test_running)
    {
        // Continuously read IMU data
        if (fabs(watch_sensor.imu_data.acce.x) > 0 ||
            fabs(watch_sensor.imu_data.acce.y) > 0 ||
            fabs(watch_sensor.imu_data.acce.z) > 0)
        {
            // Update status label with IMU data
            char buf[128];
            rt_sprintf(buf, "IMU: A(%ld,%ld,%ld) G(%ld,%ld,%ld)",
                       (long)watch_sensor.imu_data.acce.x, (long)watch_sensor.imu_data.acce.y, (long)watch_sensor.imu_data.acce.z,
                       (long)watch_sensor.imu_data.gyro.x, (long)watch_sensor.imu_data.gyro.y, (long)watch_sensor.imu_data.gyro.z);
            if (status_label != NULL)
            {
                lv_label_set_text(status_label, buf);
            }
        }
        rt_thread_mdelay(100);
    }
    LOG_D("IMU stress thread stopped");
}

static void ppg_stress_thread_entry(void *parameter)
{
    LOG_D("PPG stress thread started");
    sensor_subscription_t sensor_subscription;
    sensor_subscription.type = SENSOR_TYPE_PPG;
    sensor_subscription.thread_safe = true;
    sensor_subscription.status = true;
    watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);

    while (stress_test_running)
    {
        rt_thread_mdelay(100);
    }

    sensor_subscription.status = false;
    watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
    LOG_D("PPG stress thread stopped");
}

static void motor_stress_thread_entry(void *parameter)
{
    LOG_D("Motor stress thread started");

    while (stress_test_running)
    {
        motor_params_t param = {
            .duty_cycle = 30,  // 30%
            .period = 500000,  // 0.5s
            .repeat_times = 1, // 1 time
        };
        peripheral_provider.control_motor(true, &param);
        rt_thread_mdelay(5000); // Vibrate every 5 seconds
    }
    LOG_D("Motor stress thread stopped");
}

static void test_thread_entry(void *parameter)
{
    lv_label_set_text(test_label, "[AMOLED] Changing color...");
    if (!check_amoled())
    {
        goto EXIT;
    }

    lv_label_set_text(test_label, "[Touch] Waiting for click...(3 times!!!)");
    if (!wait_for_touch())
    {
        goto EXIT;
    }

    // lv_label_set_text(test_label, "[Motor] Running for 5 seconds...");
    // if (!run_motor(5))
    // {
    //     goto EXIT;
    // }

    lv_label_set_text(test_label, "[IMU] Checking sensor raw data...");
    if (!check_imu_sensor())
    {
        goto EXIT;
    }

    lv_label_set_text(test_label, "[PPG] Checking sensor raw data...");
    if (!check_ppg_sensor())
    {
        goto EXIT;
    }

    lv_label_set_text(test_label, "[Audio] record sound for 5 seconds...");
    if (!record_with_mic(5))
    {
        goto EXIT;
    }

    // lv_label_set_text(test_label, "[Audio] playing sound for 5 seconds...");
    // if (!play_with_speaker(5))
    // {
    //     goto EXIT;
    // }

    send_ok_to_phone_and_exit();

EXIT:
    exit_app();
}

static void create_test_thread(void)
{
    test_thread = rt_thread_create("utest", test_thread_entry, RT_NULL, 2048, 10, 10);
    if (test_thread != RT_NULL)
    {
        rt_thread_startup(test_thread);
    }
}

static bool open_operational_thread = false;
void close_operational_test_thread(void)
{
    open_operational_thread = false;
    if (test_operational_thread != RT_NULL)
    {
        rt_thread_delete(test_operational_thread);
        test_operational_thread = RT_NULL;
    }
}
extern uint8_t return_app_count(void);
static void operational_thread_entry(void *parameter)
{
    uint8_t page = 0;
    open_operational_thread = true;
    while (open_operational_thread)
    {
        watch_system_interact(WATCH_SLEEP, NULL);
        rt_thread_mdelay(1000);
        watch_system_interact(HCPU_WAKEUP, NULL);
        rt_thread_mdelay(1000);
        // send_virtual_gesture_event(GESTURE_EVENT_HAND_RELEASE);
        // animate_to_app_list();
        watch_system_interact(WATCH_GESTURE_UNLOCK, NULL);
        rt_thread_mdelay(1000);
        // extern void app_list_scroll_to_app(int8_t action);
        // int app_index = atoi(argv[2]);
        // app_list_scroll_to_app(app_index);
        // rt_thread_mdelay(500);
        lvgl_msg_t msg;
        msg.type = LVGL_MSG_TYPE_NAV_BAR_CONTROL;
        msg.data.action = page;
        lvgl_send_msg(msg);
        // LOG_D("Operational test: switching to app page %d", page);
        if (page >= return_app_count() - 1)
            page = 0;
        else
            page++;
        rt_thread_mdelay(1000);
        send_virtual_gesture_event(GESTURE_EVENT_PRESS);
        rt_thread_mdelay(1000);
    }
}

static void create_operational_test_thread(void)
{
    test_operational_thread = rt_thread_create("operationaltest", operational_thread_entry, RT_NULL, 2048, 10, 10);
    if (test_operational_thread != RT_NULL)
    {
        rt_thread_startup(test_operational_thread);
    }
}

static void start_stress_test(void)
{
    LOG_I("Starting stress test mode");
    stress_test_running = true;

    // Start CPU stress thread (high priority for maximum load)
    cpu_stress_thread = rt_thread_create("cpu_stress", cpu_stress_thread_entry, RT_NULL, 4096, 5, 10);
    if (cpu_stress_thread != RT_NULL)
    {
        rt_thread_startup(cpu_stress_thread);
    }

    // Start AMOLED stress thread
    amoled_thread = rt_thread_create("amoled_stress", amoled_stress_thread_entry, RT_NULL, 2048, 15, 10);
    if (amoled_thread != RT_NULL)
    {
        rt_thread_startup(amoled_thread);
    }

    // Start IMU monitoring thread
    imu_thread = rt_thread_create("imu_stress", imu_stress_thread_entry, RT_NULL, 2048, 12, 10);
    if (imu_thread != RT_NULL)
    {
        rt_thread_startup(imu_thread);
    }

    // Start PPG monitoring thread
    ppg_thread = rt_thread_create("ppg_stress", ppg_stress_thread_entry, RT_NULL, 2048, 12, 10);
    if (ppg_thread != RT_NULL)
    {
        rt_thread_startup(ppg_thread);
    }

    // Start motor stress thread
    motor_thread = rt_thread_create("motor_stress", motor_stress_thread_entry, RT_NULL, 1024, 20, 10);
    if (motor_thread != RT_NULL)
    {
        rt_thread_startup(motor_thread);
    }

    // Start audio recording
    voice_provider.vad_init();
    start_voice_recognition(V2T_INTENT_CHAT);

    lv_label_set_text(test_label, "STRESS TEST RUNNING\nSwipe right to exit");
}

static void stop_stress_test(void)
{
    LOG_I("Stopping stress test mode");
    stress_test_running = false;

    rt_thread_mdelay(100);

    if (cpu_stress_thread != RT_NULL)
    {
        rt_thread_delete(cpu_stress_thread);
        cpu_stress_thread = RT_NULL;
    }

    if (amoled_thread != RT_NULL)
    {
        rt_thread_delete(amoled_thread);
        amoled_thread = RT_NULL;
    }

    if (imu_thread != RT_NULL)
    {
        rt_thread_delete(imu_thread);
        imu_thread = RT_NULL;
    }

    if (ppg_thread != RT_NULL)
    {
        rt_thread_delete(ppg_thread);
        ppg_thread = RT_NULL;
    }

    if (motor_thread != RT_NULL)
    {
        rt_thread_delete(motor_thread);
        motor_thread = RT_NULL;
    }

    stop_voice_recognition(V2T_INTENT_NOTHING);
    voice_provider.vad_deinit();
}

#define REQUIRE_TOUCH_COUNT 3
static uint8_t click_count = 0;

static void mode_button_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        lv_obj_t *btn = lv_event_get_target(e);
        test_mode_t *mode = (test_mode_t *)lv_event_get_user_data(e);

        if (mode != NULL)
        {
            test_mode = *mode;
            LOG_I("Selected test mode: %d", test_mode);

            // Hide mode selection UI
            if (mode_select_container != NULL)
            {
                lv_obj_add_flag(mode_select_container, LV_OBJ_FLAG_HIDDEN);
            }

            // Start appropriate test mode
            if (test_mode == TEST_MODE_NORMAL)
            {
                lv_label_set_text(test_label, "Starting Normal Test...");
                create_test_thread();
            }
            else if (test_mode == TEST_MODE_STRESS)
            {
                start_stress_test();
            }
        }
    }
}

static void operational_button_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        create_operational_test_thread();
    }
}

static void screen_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_GESTURE)
    {
        lv_dir_t g = lv_indev_get_gesture_dir(lv_indev_get_act());
        if (g == LV_DIR_RIGHT)
        {
            /* Code to exit the screen */
            if (test_mode == TEST_MODE_STRESS)
            {
                stop_stress_test();
            }
            gui_app_self_exit();
        }
    }
    // detect click
    else if (code == LV_EVENT_CLICKED)
    {
        if (test_state == TEST_STATE_TOUCH)
        {
            click_count++;
            int remaining_clicks = REQUIRE_TOUCH_COUNT - click_count;
            char buf[32];
            rt_sprintf(buf, "Touch %d more times", remaining_clicks);
            lv_label_set_text(test_label, buf);
            motor_pattern_normal();
        }
    }
}
static void create_mode_selection_ui(lv_obj_t *parent)
{
    static test_mode_t normal_mode = TEST_MODE_NORMAL;
    static test_mode_t stress_mode = TEST_MODE_STRESS;

    mode_select_container = lv_obj_create(parent);
    lv_obj_set_size(mode_select_container, LV_PCT(100), LV_PCT(100));
    lv_obj_center(mode_select_container);
    lv_obj_set_style_bg_opa(mode_select_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mode_select_container, 0, 0);

    // Title label
    lv_obj_t *title = lv_label_create(mode_select_container);
    lv_label_set_text(title, "Select Test Mode");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(1)), 0);

    // Normal test button
    lv_obj_t *btn_normal = lv_btn_create(mode_select_container);
    lv_obj_set_size(btn_normal, 180, 60);
    lv_obj_align(btn_normal, LV_ALIGN_CENTER, 0, -40);
    lv_obj_add_event_cb(btn_normal, mode_button_event_handler, LV_EVENT_CLICKED, &normal_mode);

    lv_obj_t *label_normal = lv_label_create(btn_normal);
    lv_label_set_text(label_normal, "Normal Test");
    lv_obj_center(label_normal);

    // Stress test button
    lv_obj_t *btn_stress = lv_btn_create(mode_select_container);
    lv_obj_set_size(btn_stress, 180, 60);
    lv_obj_align(btn_stress, LV_ALIGN_CENTER, 0, 40);
    lv_obj_set_style_bg_color(btn_stress, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_event_cb(btn_stress, mode_button_event_handler, LV_EVENT_CLICKED, &stress_mode);

    lv_obj_t *label_stress = lv_label_create(btn_stress);
    lv_label_set_text(label_stress, "Stress Test");
    lv_obj_center(label_stress);

    // Stress test Operational
    lv_obj_t *btn_operational = lv_btn_create(mode_select_container);
    lv_obj_set_size(btn_operational, 180, 60);
    lv_obj_align(btn_operational, LV_ALIGN_CENTER, 0, 120);
    lv_obj_set_style_bg_color(btn_operational, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_add_event_cb(btn_operational, operational_button_event_handler, LV_EVENT_CLICKED, &stress_mode);

    lv_obj_t *label_operational = lv_label_create(btn_operational);
    lv_label_set_text(label_operational, "Operational Test");
    lv_obj_center(label_operational);

    // Description
    lv_obj_t *desc = lv_label_create(mode_select_container);
    lv_label_set_text(desc, "Stress: All sensors + CPU load");
    lv_obj_align(desc, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_text_font(desc, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
}

static lv_obj_t *on_start(lv_obj_t *parent)
{
    lv_obj_add_event_cb(parent, screen_event_handler, LV_EVENT_ALL, NULL);

    test_label = lv_label_create(parent);
    lv_obj_align(test_label, LV_ALIGN_CENTER, 0, -30);
    lv_label_set_text(test_label, "");

    status_label = lv_label_create(parent);
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_label_set_text(status_label, "");
    lv_obj_set_style_text_font(status_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    // Show mode selection UI
    create_mode_selection_ui(parent);

    return parent;
}

static void on_resume(void)
{
    setting_provider.set_power_save_mode(0);
}

static void on_pause(void)
{
    setting_provider.set_power_save_mode(1);
}

static void on_stop(void)
{
    if (test_state == TEST_STATE_EXIT)
    {
        return;
    }

    // Stop stress test if running
    if (test_mode == TEST_MODE_STRESS && stress_test_running)
    {
        stop_stress_test();
    }

    // Stop normal test thread if running
    if (test_thread != RT_NULL)
    {
        rt_thread_delete(test_thread);
        test_thread = RT_NULL;
    }
}
static void sync_back_to_phone(test_state_t state)
{
    LOG_D("Sync back to phone");
    commu_send_utest_state((uint8_t)state);
}

static bool check_amoled()
{
    LOG_D("Testing AMOLED");
    test_state = TEST_STATE_AMOLED;
    lv_color_t colors[] = {
        LV_COLOR_WHITE,
        LV_COLOR_RED,
        LV_COLOR_ORANGE,
        LV_COLOR_YELLOW,
        LV_COLOR_GREEN,
        LV_COLOR_BLUE,
        LV_COLOR_NAVY,
        LV_COLOR_PURPLE,
        LV_COLOR_BLACK,
    };
    int num_colors = sizeof(colors) / sizeof(colors[0]);

    for (int i = 0; i < num_colors; i++)
    {
        lv_obj_set_style_bg_color(lv_scr_act(), colors[i % num_colors], LV_PART_MAIN);
        rt_thread_mdelay(300);
    }
    sync_back_to_phone(test_state);
    return true;
}

static bool wait_for_touch()
{
    LOG_D("Waiting for touch");
    test_state = TEST_STATE_TOUCH;
    click_count = 0;
    while (click_count < REQUIRE_TOUCH_COUNT)
    {
        rt_thread_mdelay(100);
    }
    sync_back_to_phone(test_state);
    return true;
}

static bool check_imu_sensor()
{
    test_state = TEST_STATE_SENSOR_IMU;
    watch_sensor.imu_data.acce.x = 0;
    watch_sensor.imu_data.acce.y = 0;
    watch_sensor.imu_data.acce.z = 0;
    watch_sensor.imu_data.gyro.x = 0;
    watch_sensor.imu_data.gyro.y = 0;
    watch_sensor.imu_data.gyro.z = 0;
    for (int i = 0; i < 10; i++)
    {
        rt_thread_mdelay(100);
        if (fabs(watch_sensor.imu_data.acce.x) < 1 && fabs(watch_sensor.imu_data.acce.y) < 1 && fabs(watch_sensor.imu_data.acce.z) < 1)
        {
            LOG_E("IMU sensor acce is invalid");
            return false;
        }
        if (fabs(watch_sensor.imu_data.gyro.x) == 0 && fabs(watch_sensor.imu_data.gyro.y) == 0 && fabs(watch_sensor.imu_data.gyro.z) == 0)
        {
            LOG_E("IMU sensor gyro is invalid");
            return false;
        }
        // print out the sensor data
        lv_label_set_text_fmt(test_label, "IMU: acce(%ld, %ld, %ld), gyro(%ld, %ld, %ld)",
                              (long)watch_sensor.imu_data.acce.x, (long)watch_sensor.imu_data.acce.y, (long)watch_sensor.imu_data.acce.z,
                              (long)watch_sensor.imu_data.gyro.x, (long)watch_sensor.imu_data.gyro.y, (long)watch_sensor.imu_data.gyro.z);
    }
    sync_back_to_phone(test_state);

    return true;
}

static bool check_ppg_sensor()
{
    test_state = TEST_STATE_SENSOR_PPG;
    sensor_subscription_t sensor_subscription;
    sensor_subscription.type = SENSOR_TYPE_PPG;
    sensor_subscription.thread_safe = true;
    sensor_subscription.status = true;
    watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
    for (int i = 0; i < 10; i++)
    {
        rt_thread_mdelay(100);
        if (watch_sensor.ppg_data.raw_data[0] == 0 && watch_sensor.ppg_data.raw_data[1] == 0)
        {
            sensor_subscription.status = false;
            watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
            rt_thread_mdelay(100);
            LOG_E("Heart rate sensor is invalid");
            return false;
        }
        // print out the sensor data
        lv_label_set_text_fmt(test_label, "PPG: raw_data(%d, %d)",
                              watch_sensor.ppg_data.raw_data[0], watch_sensor.ppg_data.raw_data[1]);
    }
    rt_thread_mdelay(200);
    sensor_subscription.status = false;
    watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
    rt_thread_mdelay(200);
    sync_back_to_phone(test_state);
    return true;
}

static bool record_with_mic(int seconds)
{
    test_state = TEST_STATE_SOUND_RECORD;
    // start_voice_recording();
    voice_provider.vad_init();
    start_voice_recognition(V2T_INTENT_CHAT);
    rt_thread_mdelay(seconds * 1000);
    // stop_voice_recording();
    sync_back_to_phone(test_state);
    return true;
}

static bool play_with_speaker(int seconds)
{
    test_state = TEST_STATE_SOUND_PLAY;
    // peripheral_provider.audio_playback(true);
    // rt_thread_mdelay(seconds * 1000);
    // peripheral_provider.audio_playback(false);
    start_sync_voice_recording();
    rt_thread_mdelay((seconds + 1) * 1000);
    sync_back_to_phone(test_state);
    return true;
}

static bool run_motor(int seconds)
{
    test_state = TEST_STATE_MOTOR;
    motor_params_t param = {
        .duty_cycle = 50,        // 50%
        .period = 1000000,       // 1s
        .repeat_times = seconds, // 5s
    };
    peripheral_provider.control_motor(true, &param);
    rt_thread_mdelay(seconds * 1000);
    sync_back_to_phone(test_state);
    return true;
}

static void send_ok_to_phone_and_exit()
{
    LOG_D("Sending OK to phone");
    test_state = TEST_STATE_OK;
    sync_back_to_phone(test_state);
    gui_app_exit(APP_ID);
}

static void exit_app()
{
    LOG_D("Exiting app");
    stop_voice_recognition(V2T_INTENT_NOTHING);
    voice_provider.vad_deinit();
    test_state = TEST_STATE_EXIT;
    sync_back_to_phone(test_state);
    gui_app_exit(APP_ID);
}

/// @brief
/// @param msg
/// @param param
static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start(lv_scr_act());
        break;

    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}

static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(skaiwalk_demo), IMG_LOGO, APP_ID, app_main);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/