/**
 ******************************************************************************
 * @file   app_test.c
 * @author Skaiwalk software development team
 ******************************************************************************
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

// #if (kReleaseMode == 1)
#if 1

    #define APP_ID "test"

typedef enum
{
    TEST_MODE_SELECT = 0,
    TEST_MODE_STRESS,
} test_mode_t;

static test_mode_t test_mode = TEST_MODE_SELECT;
static volatile bool stress_test_running = false;

static rt_thread_t cpu_stress_thread = RT_NULL;
static rt_thread_t motor_thread = RT_NULL;

/* AMOLED colour-cycling + sensor readout used to run on their own worker
 * threads that called LVGL directly -> concurrent mutation of LVGL state
 * against the GUI thread corrupted lv_mem and faulted (UFSR=0x100 UNALIGNED)
 * the instant the test started. LVGL is single-threaded, so that work now runs
 * in this lv_timer, which lv_timer_handler drives inside the GUI thread. */
static lv_timer_t *stress_ui_timer = RT_NULL;

static lv_obj_t *test_label;
static lv_obj_t *sensor_label;
static lv_obj_t *mode_select_container;

/* Live microphone metrics, defined in audio_code_i2s.c. Declared inline here
   following this file's existing extern style (see generate_random_public_address). */
extern uint16_t mic_get_rms_level(void);
extern bool mic_get_vad_active(void);

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
                    matrix_c[i * 4 + j] +=
                        matrix_a[i * 4 + k] * matrix_b[k * 4 + j];
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

/* AMOLED colour cycling + live sensor/mic readout. Runs in the GUI thread
 * (driven by lv_timer_handler), so this is the ONLY place it is safe to touch
 * LVGL during the test. Called every 100 ms; the screen colour flips every
 * ~500 ms (every 5th tick) to keep the AMOLED stress of the old amoled thread,
 * and the label refreshes every tick like the old imu thread. The cpu/motor
 * worker threads deliberately never call LVGL. */
static void stress_ui_timer_cb(lv_timer_t *timer)
{
    /* Not static: LV_COLOR_* are not constant initialisers under MSVC (PC sim).
     * 8 entries re-built each tick is negligible. */
    lv_color_t colors[] = {
        LV_COLOR_WHITE, LV_COLOR_RED,  LV_COLOR_ORANGE, LV_COLOR_YELLOW,
        LV_COLOR_GREEN, LV_COLOR_BLUE, LV_COLOR_NAVY,   LV_COLOR_PURPLE,
    };
    static uint32_t tick = 0;

    if (tick % 5 == 0)
    {
        int num_colors = sizeof(colors) / sizeof(colors[0]);
        lv_obj_set_style_bg_color(lv_scr_act(),
                                  colors[(tick / 5) % num_colors], LV_PART_MAIN);
    }

    if (sensor_label != NULL)
    {
        char buf[256];
        rt_snprintf(buf, sizeof(buf),
                    "ACC %.1f %.1f %.1f\n"
                    "GYR %.1f %.1f %.1f\n"
                    "PPG %u %u\n"
                    "HR %d\n"
                    "MIC %u %s",
                    watch_sensor.imu_data.acce.x,
                    watch_sensor.imu_data.acce.y,
                    watch_sensor.imu_data.acce.z,
                    watch_sensor.imu_data.gyro.x,
                    watch_sensor.imu_data.gyro.y,
                    watch_sensor.imu_data.gyro.z,
                    watch_sensor.motion_data.ppg_raw_data.raw_data[0],
                    watch_sensor.motion_data.ppg_raw_data.raw_data[1],
                    watch_sensor.hr_data.hr,
                    mic_get_rms_level(),
                    mic_get_vad_active() ? "VAD" : "---");
        lv_label_set_text(sensor_label, buf);
    }

    tick++;
}

static void motor_stress_thread_entry(void *parameter)
{
    LOG_D("Motor stress thread started");

    /* Vibrate every 5 seconds, but poll the exit flag at 100ms granularity
     * so stop_stress_test() doesn't have to wait up to 5s for this thread
     * to wake up and notice stress_test_running == false. */
    while (stress_test_running)
    {
        motor_params_t param = {
            .duty_cycle = 30,  // 30%
            .period = 500000,  // 0.5s
            .repeat_times = 1, // 1 time
        };
        peripheral_provider.control_motor(true, &param);

        for (int i = 0; i < 50 && stress_test_running; i++)
        {
            rt_thread_mdelay(100);
        }
    }
    LOG_D("Motor stress thread stopped");
}

typedef struct
{
    rt_thread_t *handle;
    const char *name;
    void (*entry)(void *);
    rt_uint32_t stack_size;
    rt_uint8_t priority;
} stress_thread_def_t;

/* Priorities MUST stay below EVERY real system/GUI thread (i.e. a LARGER
 * RT-Thread priority number == lower priority; 30 sits just above the idle
 * thread at 31). Rationale: HCPU WDT1 is fed ONLY by the idle-thread hook
 * (rt_hw_watchdog_hook), so any CPU-bound thread that outranks idle and
 * busy-loops starves idle -> watchdog never pet -> WDT1 reboot. On 2026-05-27
 * cpu_stress at priority 5 monopolised the HCPU and tripped "HCPU WDT1 timeout"
 * ~10 s after start (the dump's free_min=7720 proved it was NOT a stack
 * overflow). Keeping all four threads below every real thread lets touch/GUI
 * stay responsive (swipe-right to exit keeps working) and lets idle run in the
 * gaps to pet the watchdog. DO NOT raise these numbers.
 *
 * Only cpu and motor run as worker threads; neither touches LVGL. All screen
 * work moved to stress_ui_timer_cb (GUI thread) to kill the cross-thread LVGL
 * race (see stress_ui_timer above).
 *
 * Stack sizes intentionally generous (separate concern from priority):
 * - cpu_stress: heavy double-precision math (pow/sin/cos/sqrt/tan) + matrix
 * - motor: control_motor goes through data_service IPC -> deeper call chain
 * Previously 1024/2048/4096 caused STKOF (UFSR=0x10) shortly after startup. */
static const stress_thread_def_t stress_thread_defs[] = {
    {&cpu_stress_thread, "cpu_stress", cpu_stress_thread_entry, 8192, 30},
    {&motor_thread, "motor_stress", motor_stress_thread_entry, 2048, 30},
};

    #define STRESS_THREAD_COUNT                                                \
        (sizeof(stress_thread_defs) / sizeof(stress_thread_defs[0]))

static void start_stress_test(void)
{
    LOG_I("Starting stress test mode");
    stress_test_running = true;

    for (size_t i = 0; i < STRESS_THREAD_COUNT; i++)
    {
        const stress_thread_def_t *def = &stress_thread_defs[i];
        *def->handle = rt_thread_create(def->name, def->entry, RT_NULL,
                                        def->stack_size, def->priority, 10);
        if (*def->handle != RT_NULL)
        {
            rt_thread_startup(*def->handle);
        }
    }

    voice_provider.vad_init();
    start_voice_recognition(V2T_INTENT_CHAT);

    /* subscribe_hr_sensor starts the HR service: it fills watch_sensor.hr_data.hr
     * (needs a few seconds of on-wrist PPG before bpm goes non-zero) and keeps
     * the PPG sensor sampling. Raw PPG is then read from the motion stream
     * (watch_sensor.motion_data.ppg_raw_data) in stress_ui_timer_cb — the same
     * channel gesture recognition uses (acce_service bundles ppg into every
     * motion sample) — NOT the separate "PPG" data service, which isn't
     * published in this build. */
    peripheral_provider.subscribe_hr_sensor(true);

    /* Drives all screen updates from the GUI thread (see stress_ui_timer_cb).
     * start_stress_test() runs in an LVGL event callback, so creating the timer
     * here is already on the GUI thread. */
    if (stress_ui_timer == RT_NULL)
    {
        stress_ui_timer = lv_timer_create(stress_ui_timer_cb, 100, RT_NULL);
    }

    lv_label_set_text(test_label, "STRESS TEST RUNNING\nSwipe right to exit");
}

static void stop_stress_test(void)
{
    LOG_I("Stopping stress test mode");
    stress_test_running = false;

    /* Stop the screen updater first. stop_stress_test() runs in the GUI thread
     * (gesture callback / ONSTOP), the same context that drives the timer, so
     * deleting it here is safe and guarantees no further LVGL access. */
    if (stress_ui_timer != RT_NULL)
    {
        lv_timer_del(stress_ui_timer);
        stress_ui_timer = RT_NULL;
    }

    /* Wait for the worker threads to exit naturally. Worst-case latency:
     *   - cpu_stress finishes its current iteration (prime calc + FP +
     *     matrix + hash) which can take ~200ms on M33
     *   - motor_stress wakes from its 100ms poll
     * 600ms covers everyone with slack.
     *
     * IMPORTANT: do NOT call rt_thread_delete() here. rt_thread_create()
     * makes dynamic threads that RT-Thread auto-cleans up after the entry
     * function returns. Calling rt_thread_delete on an already-exited
     * thread asserts at rtthread.c:421 (rt_object_get_type fails because
     * the thread object was already freed). */
    rt_thread_mdelay(600);

    for (size_t i = 0; i < STRESS_THREAD_COUNT; i++)
    {
        *stress_thread_defs[i].handle = RT_NULL;
    }

    stop_voice_recognition(V2T_INTENT_NOTHING);
    voice_provider.vad_deinit();

    peripheral_provider.subscribe_hr_sensor(false);
}

static void mode_button_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        test_mode_t *mode = (test_mode_t *)lv_event_get_user_data(e);

        if (mode != NULL && *mode == TEST_MODE_STRESS)
        {
            test_mode = *mode;
            LOG_I("Selected test mode: %d", test_mode);

            if (mode_select_container != NULL)
            {
                lv_obj_add_flag(mode_select_container, LV_OBJ_FLAG_HIDDEN);
            }

            start_stress_test();
        }
    }
}

static void random_address_button_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        extern void generate_random_public_address(uint8_t device_id);
        generate_random_public_address(0);
        rt_thread_mdelay(50);
        peripheral_provider.hcpu_reboot();
    }
}

static void screen_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_GESTURE)
    {
        lv_dir_t g = lv_indev_get_gesture_dir(lv_indev_get_act());
        if (g == LV_DIR_RIGHT)
        {
            if (test_mode == TEST_MODE_STRESS)
            {
                stop_stress_test();
            }
            gui_app_self_exit();
        }
    }
}
static lv_obj_t *create_mode_button(lv_obj_t *parent, const char *text,
                                    lv_coord_t y_offset,
                                    lv_palette_t bg_palette, lv_event_cb_t cb,
                                    void *user_data)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 180, 60);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, y_offset);
    if (bg_palette != LV_PALETTE_NONE)
    {
        lv_obj_set_style_bg_color(btn, lv_palette_main(bg_palette), 0);
    }
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    return btn;
}

static void create_mode_selection_ui(lv_obj_t *parent)
{
    static test_mode_t stress_mode = TEST_MODE_STRESS;

    mode_select_container = lv_obj_create(parent);
    lv_obj_set_size(mode_select_container, LV_PCT(100), LV_PCT(100));
    lv_obj_center(mode_select_container);
    lv_obj_set_style_bg_opa(mode_select_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mode_select_container, 0, 0);

    lv_obj_t *title = lv_label_create(mode_select_container);
    lv_label_set_text(title, "Select Test Mode");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_text_font(title, LV_EXT_FONT_GET(get_system_font_size(1)),
                               0);

    create_mode_button(mode_select_container, "Stress Test", 0, LV_PALETTE_RED,
                       mode_button_event_handler, &stress_mode);
    create_mode_button(mode_select_container, "Random Address", 80,
                       LV_PALETTE_BLUE, random_address_button_event_handler,
                       NULL);
}

static lv_obj_t *on_start(lv_obj_t *parent)
{
    lv_obj_add_event_cb(parent, screen_event_handler, LV_EVENT_ALL, NULL);

    test_label = lv_label_create(parent);
    lv_obj_align(test_label, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_align(test_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(test_label, "");

    /* All sensor values + mic, centered for easy reading during the test. */
    sensor_label = lv_label_create(parent);
    lv_obj_align(sensor_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_align(sensor_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(sensor_label, "");
    lv_obj_set_style_text_font(sensor_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(sensor_label, lv_color_white(), 0);
    lv_obj_set_style_bg_color(sensor_label, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(sensor_label, LV_OPA_50, 0);
    lv_obj_set_style_pad_all(sensor_label, 6, 0);
    lv_obj_set_style_radius(sensor_label, 0, 0);

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
    if (test_mode == TEST_MODE_STRESS && stress_test_running)
    {
        stop_stress_test();
    }
}
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
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/