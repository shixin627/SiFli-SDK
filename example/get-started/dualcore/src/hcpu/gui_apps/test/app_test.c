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
static rt_thread_t amoled_thread = RT_NULL;
static rt_thread_t imu_thread = RT_NULL;
static rt_thread_t ppg_thread = RT_NULL;
static rt_thread_t motor_thread = RT_NULL;

static lv_obj_t *test_label;
static lv_obj_t *status_label;
static lv_obj_t *ppg_status_label;
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

static void amoled_stress_thread_entry(void *parameter)
{
    LOG_D("AMOLED stress thread started");
    lv_color_t colors[] = {
        LV_COLOR_WHITE, LV_COLOR_RED,  LV_COLOR_ORANGE, LV_COLOR_YELLOW,
        LV_COLOR_GREEN, LV_COLOR_BLUE, LV_COLOR_NAVY,   LV_COLOR_PURPLE,
    };
    int num_colors = sizeof(colors) / sizeof(colors[0]);
    int idx = 0;

    while (stress_test_running)
    {
        lv_obj_set_style_bg_color(lv_scr_act(), colors[idx % num_colors],
                                  LV_PART_MAIN);
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
        char buf[96];
        rt_sprintf(buf, "LinAcc: %.2f, %.2f, %.2f",
                   watch_sensor.motion_data.linear_acce.x,
                   watch_sensor.motion_data.linear_acce.y,
                   watch_sensor.motion_data.linear_acce.z);
        if (status_label != NULL)
        {
            lv_label_set_text(status_label, buf);
        }
        rt_thread_mdelay(100);
    }
    LOG_D("IMU stress thread stopped");
}

static void ppg_stress_thread_entry(void *parameter)
{
    LOG_D("PPG stress thread started");

    while (stress_test_running)
    {
        char buf[64];
        rt_sprintf(buf, "PPG: %d, %d", watch_sensor.ppg_data.raw_data[0],
                   watch_sensor.ppg_data.raw_data[1]);
        if (ppg_status_label != NULL)
        {
            lv_label_set_text(ppg_status_label, buf);
        }
        rt_thread_mdelay(100);
    }
    LOG_D("PPG stress thread stopped");
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

/* Stack sizes intentionally generous:
 * - cpu_stress: heavy double-precision math (pow/sin/cos/sqrt/tan) + matrix
 * - amoled/imu/ppg: each iteration touches LVGL from non-LVGL thread (TODO:
 *   move to lvgl_send_msg per project convention) plus sprintf buffers
 * - motor: control_motor goes through data_service IPC -> deeper call chain
 * Previously 1024/2048/4096 caused STKOF (UFSR=0x10) shortly after startup. */
static const stress_thread_def_t stress_thread_defs[] = {
    {&cpu_stress_thread, "cpu_stress", cpu_stress_thread_entry, 8192, 5},
    {&amoled_thread, "amoled_stress", amoled_stress_thread_entry, 4096, 15},
    {&imu_thread, "imu_stress", imu_stress_thread_entry, 4096, 12},
    {&ppg_thread, "ppg_stress", ppg_stress_thread_entry, 4096, 12},
    {&motor_thread, "motor_stress", motor_stress_thread_entry, 2048, 20},
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

    lv_label_set_text(test_label, "STRESS TEST RUNNING\nSwipe right to exit");
}

static void stop_stress_test(void)
{
    LOG_I("Stopping stress test mode");
    stress_test_running = false;

    /* Wait for all stress threads to exit naturally. Worst-case latency:
     *   - cpu_stress finishes its current iteration (prime calc + FP +
     *     matrix + hash) which can take ~200ms on M33
     *   - motor_stress wakes from its 100ms poll
     *   - others wake from their 100/500ms mdelay
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
    lv_obj_align(test_label, LV_ALIGN_CENTER, 0, -30);
    lv_label_set_text(test_label, "");

    status_label = lv_label_create(parent);
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -40);
    lv_label_set_text(status_label, "");
    lv_obj_set_style_text_font(status_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    ppg_status_label = lv_label_create(parent);
    lv_obj_align(ppg_status_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_label_set_text(ppg_status_label, "");
    lv_obj_set_style_text_font(ppg_status_label,
                               LV_EXT_FONT_GET(get_system_font_size(0)), 0);

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