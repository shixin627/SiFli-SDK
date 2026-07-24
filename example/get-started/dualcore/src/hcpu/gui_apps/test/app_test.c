/**
 ******************************************************************************
 * @file   app_test.c
 * @brief  Interactive factory test and minimal factory utility.
 ******************************************************************************
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#include <string.h>
#include <dfs_posix.h>

#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "watch_global_data.h"
#include "ui_img_helper.h"
#include "app_test.h"

#ifdef BSP_USING_BLOC
    #include "bloc_setting.h"
    #include "bloc_peripheral.h"
#endif

#define DBG_TAG "app.test"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define APP_ID                   "test"
#define TOUCH_ALL                0x0F
#define TOUCH_TIMEOUT_MS         45000
#define CHARGE_TIMEOUT_MS        30000
#define SCREEN_FLASH_PATH        "/hwtest.tmp"
#define SCREEN_STRESS_MS         5000
#define SCREEN_SENSOR_TIMEOUT_MS 7000
#define SCREEN_RESULT_HOLD_MS    1500
#define SYSTEM_STRESS_TILE_COUNT 6
#define SYSTEM_STRESS_BLOCKS     8
#define SYSTEM_STRESS_MAX_ERRORS 10

typedef enum
{
    TEST_STAGE_IDLE = 0,
    TEST_STAGE_COLOR,
    TEST_STAGE_TOUCH,
    TEST_STAGE_DISPLAY_CONFIRM,
    TEST_STAGE_MOTOR_CONFIRM,
    TEST_STAGE_CHARGE,
    TEST_STAGE_DONE,
} interactive_stage_t;

static bool test_active;
static bool app_visible;
static bool cancel_pending;
static interactive_stage_t test_stage;
static lv_timer_t *test_timer;
static lv_obj_t *title_label;
static lv_obj_t *status_label;
static lv_obj_t *menu_container;
static lv_obj_t *control_container;
static lv_obj_t *confirm_container;
static lv_obj_t *touch_cells[4];
static uint8_t touch_mask;
static uint8_t color_index;
static int failures;
static rt_uint32_t stage_started;
static rt_uint32_t last_charge_request;
static char failed_items[64];

static app_test_screening_callbacks_t screen_callbacks;
static volatile bool screen_request_pending;
static volatile bool screening_active;
static volatile bool screen_cancel_requested;
static volatile bool screen_done;
static rt_thread_t screen_worker;
static lv_timer_t *screen_ui_timer;
static int screen_failures;
static int screen_progress_percent;
static uint32_t screen_ui_refresh_count;
static char screen_failed_items[64];
static char screen_progress_stage[24];
static char screen_progress_detail[96];
static char screen_result_text[128];
#define SCREEN_LOG_LINE_COUNT 8
#define SCREEN_LOG_LINE_SIZE  112
static char screen_log_lines[SCREEN_LOG_LINE_COUNT][SCREEN_LOG_LINE_SIZE];
static uint8_t screen_log_next;
static uint8_t screen_log_count;
static struct rt_mutex screen_log_lock;
static bool screen_log_lock_ready;

static volatile bool system_stress_active;
static volatile bool system_stress_stop_requested;
static volatile uint32_t system_stress_cycles;
static volatile uint32_t system_stress_errors;
static volatile uint32_t system_stress_alloc_failures;
static rt_uint32_t system_stress_started;
static rt_uint32_t system_stress_heap_start;
static uint32_t system_stress_ui_refresh_count;
static uint32_t system_stress_ui_actions;
static uint32_t system_stress_ui_events;
static uint32_t system_stress_random_state;
static rt_thread_t system_stress_worker;
static lv_timer_t *system_stress_ui_timer;
static lv_obj_t *system_stress_tiles[SYSTEM_STRESS_TILE_COUNT];
static lv_obj_t *system_stress_controls[SYSTEM_STRESS_TILE_COUNT];
static bool system_stress_finalized;

static void start_interactive_test(lv_obj_t *parent);
static void finish_interactive_test(bool cancelled);
static void start_system_stress_test(lv_obj_t *parent);

static lv_obj_t *create_button(lv_obj_t *parent, const char *text,
                               lv_coord_t width, lv_coord_t height,
                               lv_event_cb_t callback, void *user_data)
{
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_set_size(button, width, height);
    lv_obj_add_event_cb(
        button, callback, LV_EVENT_CLICKED, user_data);

    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);
    return button;
}

static void set_status(const char *text)
{
    if (status_label != RT_NULL && lv_obj_is_valid(status_label))
    {
        lv_label_set_text(status_label, text);
    }
}

static void remove_object(lv_obj_t **object)
{
    if (*object != RT_NULL && lv_obj_is_valid(*object))
    {
        lv_obj_del(*object);
    }
    *object = RT_NULL;
}

static void delete_touch_cells(void)
{
    uint32_t i;
    for (i = 0; i < 4; i++)
    {
        remove_object(&touch_cells[i]);
    }
}

static void report_result(const char *name, bool passed)
{
    if (passed)
    {
        return;
    }
    failures++;
    if (failed_items[0] != '\0')
    {
        strncat(failed_items, ",",
                sizeof(failed_items) - strlen(failed_items) - 1);
    }
    strncat(failed_items, name,
            sizeof(failed_items) - strlen(failed_items) - 1);
}

static void cancel_async(void *user_data)
{
    (void)user_data;
    cancel_pending = false;
    if (test_active)
    {
        finish_interactive_test(true);
    }
}

static void cancel_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED &&
        test_active && !cancel_pending)
    {
        cancel_pending = true;
        lv_async_call(cancel_async, RT_NULL);
    }
}

static void exit_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED)
    {
        gui_app_self_exit();
    }
}

static void screen_log_add(const char *line)
{
    if (!screen_log_lock_ready || line == RT_NULL)
    {
        return;
    }
    rt_mutex_take(&screen_log_lock, RT_WAITING_FOREVER);
    rt_strncpy(screen_log_lines[screen_log_next], line,
               SCREEN_LOG_LINE_SIZE);
    screen_log_lines[screen_log_next][SCREEN_LOG_LINE_SIZE - 1] = '\0';
    screen_log_next =
        (uint8_t)((screen_log_next + 1) % SCREEN_LOG_LINE_COUNT);
    if (screen_log_count < SCREEN_LOG_LINE_COUNT)
    {
        screen_log_count++;
    }
    rt_mutex_release(&screen_log_lock);
}

static void screen_progress(int percent, const char *stage,
                            const char *detail)
{
    char line[SCREEN_LOG_LINE_SIZE];
    const char *safe_stage = stage == RT_NULL ? "" : stage;
    const char *safe_detail = detail == RT_NULL ? "" : detail;

    if (percent < 0)
    {
        percent = 0;
    }
    else if (percent > 100)
    {
        percent = 100;
    }

    rt_mutex_take(&screen_log_lock, RT_WAITING_FOREVER);
    screen_progress_percent = percent;
    rt_strncpy(screen_progress_stage, safe_stage,
               sizeof(screen_progress_stage));
    screen_progress_stage[sizeof(screen_progress_stage) - 1] = '\0';
    rt_strncpy(screen_progress_detail, safe_detail,
               sizeof(screen_progress_detail));
    screen_progress_detail[sizeof(screen_progress_detail) - 1] = '\0';
    rt_mutex_release(&screen_log_lock);

    rt_snprintf(line, sizeof(line), "[%3d%%] %-10s %s",
                percent, safe_stage, safe_detail);
    screen_log_add(line);
    LOG_I("board screening: %d%% %s %s",
          percent, safe_stage, safe_detail);
    if (screen_callbacks.progress != RT_NULL)
    {
        screen_callbacks.progress(
            screen_callbacks.context, percent, safe_stage, safe_detail);
    }
}

static void screen_report(const char *name, bool passed, const char *detail)
{
    char line[SCREEN_LOG_LINE_SIZE];
    const char *safe_detail = detail == RT_NULL ? "" : detail;

    if (!passed)
    {
        screen_failures++;
        if (screen_failed_items[0] != '\0')
        {
            strncat(screen_failed_items, ",",
                    sizeof(screen_failed_items) -
                        strlen(screen_failed_items) - 1);
        }
        strncat(screen_failed_items, name,
                sizeof(screen_failed_items) -
                    strlen(screen_failed_items) - 1);
    }
    rt_snprintf(line, sizeof(line), "[%s] %-10s %s",
                passed ? "PASS" : "FAIL", name, safe_detail);
    screen_log_add(line);
    LOG_I("board screening: %s %s %s", name,
          passed ? "PASS" : "FAIL", safe_detail);
    if (screen_callbacks.item != RT_NULL)
    {
        screen_callbacks.item(
            screen_callbacks.context, name, passed, safe_detail);
    }
}

static bool screen_flash_rw(char *detail, size_t detail_size)
{
    static const uint8_t expected[] = {
        0x53, 0x4B, 0x41, 0x49, 0x5F, 0x48, 0x57, 0x54,
        0x45, 0x53, 0x54, 0x5F, 0xA5, 0x5A, 0xC3, 0x3C,
    };
    uint8_t actual[sizeof(expected)];
    FILE *file = RT_NULL;
    size_t written = 0;
    size_t read_size = 0;
    bool passed = false;

    watch_storage_api_lock();
    file = fopen(SCREEN_FLASH_PATH, "wb");
    if (file != RT_NULL)
    {
        written = fwrite(expected, 1, sizeof(expected), file);
        fclose(file);
        file = RT_NULL;
    }
    if (written == sizeof(expected))
    {
        file = fopen(SCREEN_FLASH_PATH, "rb");
        if (file != RT_NULL)
        {
            read_size = fread(actual, 1, sizeof(actual), file);
            fclose(file);
            file = RT_NULL;
        }
    }
    if (read_size == sizeof(expected) &&
        memcmp(expected, actual, sizeof(expected)) == 0)
    {
        passed = true;
    }
    remove(SCREEN_FLASH_PATH);
    watch_storage_api_unlock();

    rt_snprintf(detail, detail_size, "write=%u read=%u compare=%s",
                (unsigned int)written, (unsigned int)read_size,
                passed ? "ok" : "fail");
    return passed;
}

static void screen_cancel_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED &&
        screening_active)
    {
        screen_cancel_requested = true;
        set_status("Cancelling board screening...");
    }
}

static void screen_ui_timer_cb(lv_timer_t *timer)
{
    char display[1200];
    size_t used = 0;
    uint8_t count;
    uint8_t next;
    uint8_t i;
    int progress;
    char stage[sizeof(screen_progress_stage)];
    char detail[sizeof(screen_progress_detail)];
    bool finished;
    (void)timer;

    screen_ui_refresh_count++;
    rt_mutex_take(&screen_log_lock, RT_WAITING_FOREVER);
    count = screen_log_count;
    next = screen_log_next;
    progress = screen_progress_percent;
    rt_strncpy(stage, screen_progress_stage, sizeof(stage));
    stage[sizeof(stage) - 1] = '\0';
    rt_strncpy(detail, screen_progress_detail, sizeof(detail));
    detail[sizeof(detail) - 1] = '\0';
    finished = screen_done;
    used = (size_t)rt_snprintf(
        display, sizeof(display),
        "UART BOARD SCREENING\n"
        "--------------------\n");
    for (i = 0; i < count && used < sizeof(display) - 1; i++)
    {
        uint8_t index = (uint8_t)(
            (next + SCREEN_LOG_LINE_COUNT - count + i) %
            SCREEN_LOG_LINE_COUNT);
        int written = rt_snprintf(
            display + used, sizeof(display) - used,
            "%s\n", screen_log_lines[index]);
        if (written <= 0)
        {
            break;
        }
        used += (size_t)written;
        if (used >= sizeof(display))
        {
            used = sizeof(display) - 1;
        }
    }
    rt_mutex_release(&screen_log_lock);

    if (!finished && used < sizeof(display) - 1)
    {
        rt_snprintf(display + used, sizeof(display) - used,
                    "\n[%c] %d%% %s\n%s\nrefresh=%lu",
                    "|/-\\"[screen_ui_refresh_count % 4],
                    progress, stage, detail,
                    (unsigned long)screen_ui_refresh_count);
    }
    else if (finished && used < sizeof(display) - 1)
    {
        rt_snprintf(display + used, sizeof(display) - used,
                    "\n%s", screen_result_text);
        remove_object(&control_container);
    }
    set_status(display);

    if ((screen_ui_refresh_count & 1U) == 0U)
    {
        lv_obj_set_style_bg_color(
            lv_scr_act(), lv_color_hex(0x000000), 0);
        if (status_label != RT_NULL && lv_obj_is_valid(status_label))
        {
            lv_obj_set_style_text_color(
                status_label, lv_color_hex(0x7CFF7C), 0);
        }
    }
    else
    {
        lv_obj_set_style_bg_color(
            lv_scr_act(), lv_color_hex(0x001B2A), 0);
        if (status_label != RT_NULL && lv_obj_is_valid(status_label))
        {
            lv_obj_set_style_text_color(
                status_label, lv_color_white(), 0);
        }
    }

}

static void show_screen_control(void)
{
    lv_obj_t *button;
    remove_object(&control_container);
    control_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(control_container, 210, 75);
    lv_obj_align(control_container, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_opa(control_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(control_container, 0, 0);
    lv_obj_set_style_pad_all(control_container, 0, 0);
    lv_obj_clear_flag(control_container, LV_OBJ_FLAG_SCROLLABLE);
    button = create_button(
        control_container, "CANCEL", 190, 60,
        screen_cancel_event, RT_NULL);
    lv_obj_center(button);
}

static bool screen_wait_or_cancel(rt_uint32_t milliseconds)
{
    rt_uint32_t waited = 0;
    while (waited < milliseconds)
    {
        rt_uint32_t slice =
            (milliseconds - waited) > 50 ? 50 : milliseconds - waited;
        if (screen_cancel_requested)
        {
            return false;
        }
        rt_thread_mdelay(slice);
        waited += slice;
    }
    return !screen_cancel_requested;
}

static void board_screen_worker_entry(void *parameter)
{
    uint32_t memory[256];
    uint32_t seed = 0x13579BDF;
    uint32_t stress_iterations = 0;
    uint32_t stress_errors = 0;
    uint32_t i;
    rt_uint32_t stress_started;
    rt_uint32_t last_progress;
    uint32_t imu_before;
    uint32_t ppg_before;
    bool imu_reported = false;
    bool ppg_reported = false;
    bool imu_was_subscribed;
    bool hr_was_subscribed;
    bool cancelled = false;
    char detail[96];
    (void)parameter;

    screen_progress(0, "start", "headless worker started");
    screen_progress(3, "flash", "read/write/compare");
    screen_report(
        "flash", screen_flash_rw(detail, sizeof(detail)), detail);

    if (screen_cancel_requested)
    {
        cancelled = true;
        goto screening_complete;
    }

    screen_progress(10, "cpu_ram", "stress test");
    stress_started = rt_tick_get_millisecond();
    last_progress = stress_started;
    while ((rt_tick_get_millisecond() - stress_started) <
           SCREEN_STRESS_MS)
    {
        uint32_t verify_seed = seed;
        for (i = 0; i < (sizeof(memory) / sizeof(memory[0])); i++)
        {
            seed = seed * 1664525U + 1013904223U;
            memory[i] = seed;
        }
        for (i = 0; i < (sizeof(memory) / sizeof(memory[0])); i++)
        {
            verify_seed = verify_seed * 1664525U + 1013904223U;
            if (memory[i] != verify_seed)
            {
                stress_errors++;
            }
        }
        stress_iterations++;
        if (screen_cancel_requested)
        {
            cancelled = true;
            goto screening_complete;
        }
        if ((rt_tick_get_millisecond() - last_progress) >= 500)
        {
            rt_uint32_t elapsed =
                rt_tick_get_millisecond() - stress_started;
            rt_snprintf(detail, sizeof(detail),
                        "iterations=%lu errors=%lu elapsed=%lums",
                        (unsigned long)stress_iterations,
                        (unsigned long)stress_errors,
                        (unsigned long)elapsed);
            screen_progress(
                10 + (int)(elapsed * 45 / SCREEN_STRESS_MS),
                "cpu_ram", detail);
            last_progress = rt_tick_get_millisecond();
        }
        rt_thread_mdelay(1);
    }
    rt_snprintf(detail, sizeof(detail),
                "iterations=%lu errors=%lu",
                (unsigned long)stress_iterations,
                (unsigned long)stress_errors);
    screen_report("cpu_ram",
                  stress_iterations > 0 && stress_errors == 0, detail);

    screen_progress(58, "battery", "request voltage");
    if (watch_sys_sync.request_battery_voltage != RT_NULL)
    {
        watch_sys_sync.request_battery_voltage();
    }
    if (!screen_wait_or_cancel(800))
    {
        cancelled = true;
        goto screening_complete;
    }
    {
        bool passed =
            (SkaiWatchSys.battery_vol_value >= 2800 &&
             SkaiWatchSys.battery_vol_value <= 5000 &&
             SkaiWatchSys.battery_level_value <= 100);
        rt_snprintf(detail, sizeof(detail), "mv=%u level=%u",
                    SkaiWatchSys.battery_vol_value,
                    SkaiWatchSys.battery_level_value);
        screen_report("battery", passed, detail);
    }

    screen_progress(65, "sensors", "waiting for IMU and PPG");
    imu_was_subscribed = (watch_sensor.imu_client_num > 0);
    hr_was_subscribed = (watch_sensor.hr_client_num > 0);
    imu_before = (uint32_t)watch_sensor.imu_data.timestamp;
    ppg_before =
        (uint32_t)watch_sensor.motion_data.ppg_raw_data.timestamp;
    if (!imu_was_subscribed &&
        peripheral_provider.subscribe_accelerometer_sensor != RT_NULL)
    {
        peripheral_provider.subscribe_accelerometer_sensor(true);
    }
    if (!hr_was_subscribed &&
        peripheral_provider.subscribe_hr_sensor != RT_NULL)
    {
        peripheral_provider.subscribe_hr_sensor(true);
    }
    {
        rt_uint32_t sensor_started = rt_tick_get_millisecond();
        last_progress = sensor_started;
        while ((!imu_reported || !ppg_reported) &&
               (rt_tick_get_millisecond() - sensor_started) <
                   SCREEN_SENSOR_TIMEOUT_MS)
        {
            rt_uint32_t now = rt_tick_get_millisecond();
            if (screen_cancel_requested)
            {
                cancelled = true;
                break;
            }
            if (!imu_reported &&
                (uint32_t)watch_sensor.imu_data.timestamp != 0 &&
                (uint32_t)watch_sensor.imu_data.timestamp != imu_before)
            {
                bool passed = !watch_sensor.imu_abnormal;
                rt_snprintf(
                    detail, sizeof(detail),
                    "timestamp=%lu abnormal=%u",
                    (unsigned long)watch_sensor.imu_data.timestamp,
                    watch_sensor.imu_abnormal ? 1 : 0);
                screen_report("imu", passed, detail);
                imu_reported = true;
            }
            if (!ppg_reported &&
                (uint32_t)
                    watch_sensor.motion_data.ppg_raw_data.timestamp != 0 &&
                (uint32_t)
                    watch_sensor.motion_data.ppg_raw_data.timestamp !=
                        ppg_before)
            {
                rt_snprintf(
                    detail, sizeof(detail),
                    "timestamp=%lu samples=%u raw0=%lu",
                    (unsigned long)
                        watch_sensor.motion_data.ppg_raw_data.timestamp,
                    watch_sensor.motion_data.ppg_raw_data.sample_num,
                    (unsigned long)
                        watch_sensor.motion_data.ppg_raw_data.raw_data[0]);
                screen_report("ppg", true, detail);
                ppg_reported = true;
            }
            if ((now - last_progress) >= 500)
            {
                rt_uint32_t elapsed = now - sensor_started;
                rt_snprintf(detail, sizeof(detail),
                            "imu=%s ppg=%s elapsed=%lums",
                            imu_reported ? "ok" : "wait",
                            ppg_reported ? "ok" : "wait",
                            (unsigned long)elapsed);
                screen_progress(
                    65 + (int)(elapsed * 30 /
                               SCREEN_SENSOR_TIMEOUT_MS),
                    "sensors", detail);
                last_progress = now;
            }
            rt_thread_mdelay(20);
        }
    }
    if (!imu_was_subscribed &&
        peripheral_provider.subscribe_accelerometer_sensor != RT_NULL)
    {
        peripheral_provider.subscribe_accelerometer_sensor(false);
    }
    if (!hr_was_subscribed &&
        peripheral_provider.subscribe_hr_sensor != RT_NULL)
    {
        peripheral_provider.subscribe_hr_sensor(false);
    }
    if (cancelled)
    {
        goto screening_complete;
    }
    if (!imu_reported)
    {
        screen_report("imu", false, "no_new_sample");
    }
    if (!ppg_reported)
    {
        screen_report("ppg", false, "no_new_sample");
    }

screening_complete:
    if (cancelled)
    {
        screen_report("screening", false, "operator_cancelled");
        screen_progress(100, "cancelled", "operator cancelled");
    }
    else
    {
        rt_snprintf(detail, sizeof(detail), "failures=%d",
                    screen_failures);
        screen_progress(100, "complete", detail);
    }

    if (screen_failures == 0)
    {
        rt_snprintf(screen_result_text, sizeof(screen_result_text),
                    "BOARD SCREENING PASS\nUART response sent");
    }
    else
    {
        rt_snprintf(screen_result_text, sizeof(screen_result_text),
                    "BOARD SCREENING FAILED (%d)\n%s\nUART response sent",
                    screen_failures, screen_failed_items);
    }

    /* done() sends SKAI_HWTEST END synchronously.  Publish screen_done only
     * after that response, so the UI cannot close before UART has replied. */
    if (screen_callbacks.done != RT_NULL)
    {
        screen_callbacks.done(
            screen_callbacks.context, screen_failures);
    }
    screen_request_pending = false;
    screening_active = false;
    screen_done = true;
    memset(&screen_callbacks, 0, sizeof(screen_callbacks));

    /* gui_app_self_exit() depends on the caller's active-app context and can
     * be ignored from an LVGL timer.  Exit by explicit app id from this
     * background worker after the result has been visible briefly. */
    rt_thread_mdelay(SCREEN_RESULT_HOLD_MS);
    gui_app_exit(APP_ID);
    screen_worker = RT_NULL;
}

static void start_board_screening_ui(lv_obj_t *parent)
{
    screen_ui_refresh_count = 0;
    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
    title_label = RT_NULL;
    status_label = lv_label_create(parent);
    lv_obj_set_width(status_label, LV_PCT(94));
    lv_obj_set_style_text_align(
        status_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(status_label, LV_ALIGN_TOP_LEFT, 10, 10);
    show_screen_control();
    screen_ui_timer = lv_timer_create(
        screen_ui_timer_cb, 100, RT_NULL);
    if (screen_ui_timer != RT_NULL)
    {
        screen_ui_timer_cb(screen_ui_timer);
    }
}

bool app_test_board_screening_start(
    const app_test_screening_callbacks_t *new_callbacks)
{
    if (new_callbacks == RT_NULL ||
        new_callbacks->item == RT_NULL ||
        new_callbacks->done == RT_NULL ||
        screen_request_pending || screening_active ||
        screen_worker != RT_NULL || test_active ||
        system_stress_active || system_stress_worker != RT_NULL ||
        app_visible)
    {
        return false;
    }
    if (!screen_log_lock_ready)
    {
        if (rt_mutex_init(
                &screen_log_lock, "screenlog",
                RT_IPC_FLAG_PRIO) != RT_EOK)
        {
            return false;
        }
        screen_log_lock_ready = true;
    }

    screen_callbacks = *new_callbacks;
    screen_cancel_requested = false;
    screen_done = false;
    screen_failures = 0;
    screen_failed_items[0] = '\0';
    screen_result_text[0] = '\0';
    screen_progress_percent = 0;
    screen_progress_stage[0] = '\0';
    screen_progress_detail[0] = '\0';
    screen_log_next = 0;
    screen_log_count = 0;
    screening_active = true;
    screen_request_pending = true;
    screen_worker = rt_thread_create(
        "bd_screen", board_screen_worker_entry,
        RT_NULL, 5120, 28, 10);
    if (screen_worker == RT_NULL)
    {
        screening_active = false;
        screen_request_pending = false;
        memset(&screen_callbacks, 0, sizeof(screen_callbacks));
        return false;
    }
    if (rt_thread_startup(screen_worker) != RT_EOK)
    {
        rt_thread_delete(screen_worker);
        screen_worker = RT_NULL;
        screening_active = false;
        screen_request_pending = false;
        memset(&screen_callbacks, 0, sizeof(screen_callbacks));
        return false;
    }

    /* The screen is optional.  A missing display or GUI launch failure must
     * not prevent the headless worker from screening the board. */
    if (gui_app_run(APP_ID) != RT_EOK)
    {
        screen_request_pending = false;
    }
    return true;
}

bool app_test_board_screening_cancel(void)
{
    if (!screen_request_pending && !screening_active)
    {
        return false;
    }
    screen_cancel_requested = true;
    return true;
}

static void delete_system_stress_tiles(void)
{
    uint32_t i;
    for (i = 0; i < SYSTEM_STRESS_TILE_COUNT; i++)
    {
        remove_object(&system_stress_tiles[i]);
        system_stress_controls[i] = RT_NULL;
    }
}

static void system_stress_control_event(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);

    if (code == LV_EVENT_CLICKED ||
        code == LV_EVENT_VALUE_CHANGED ||
        code == LV_EVENT_SCROLL)
    {
        system_stress_ui_events++;
    }
}

static lv_obj_t *create_system_stress_tile(uint32_t index)
{
    lv_obj_t *tile;
    lv_obj_t *control = RT_NULL;
    lv_coord_t x;
    lv_coord_t y;

    if (index >= SYSTEM_STRESS_TILE_COUNT)
    {
        return RT_NULL;
    }
    x = (lv_coord_t)(((int)(index % 3) - 1) * 135);
    y = (lv_coord_t)(62 + (index / 3) * 65);
    tile = lv_obj_create(lv_scr_act());
    if (tile == RT_NULL)
    {
        system_stress_controls[index] = RT_NULL;
        system_stress_alloc_failures++;
        system_stress_errors++;
        return RT_NULL;
    }
    lv_obj_set_size(tile, 116, 56);
    lv_obj_align(tile, LV_ALIGN_TOP_MID, x, y);
    lv_obj_set_style_radius(tile, 8, 0);
    lv_obj_set_style_border_width(tile, 0, 0);
    lv_obj_set_style_pad_all(tile, 2, 0);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    system_stress_tiles[index] = tile;

    switch (index)
    {
    case 0:
    {
        lv_obj_t *label;
        control = lv_btn_create(tile);
        if (control != RT_NULL)
        {
            lv_obj_set_size(control, 100, 40);
            lv_obj_center(control);
            label = lv_label_create(control);
            if (label != RT_NULL)
            {
                lv_label_set_text(label, "CLICK");
                lv_obj_center(label);
            }
            lv_obj_add_event_cb(
                control, system_stress_control_event,
                LV_EVENT_CLICKED, RT_NULL);
        }
        break;
    }
    case 1:
        control = lv_switch_create(tile);
        if (control != RT_NULL)
        {
            lv_obj_center(control);
            lv_obj_add_event_cb(
                control, system_stress_control_event,
                LV_EVENT_VALUE_CHANGED, RT_NULL);
        }
        break;
    case 2:
        control = lv_slider_create(tile);
        if (control != RT_NULL)
        {
            lv_obj_set_size(control, 96, 18);
            lv_obj_center(control);
            lv_obj_add_event_cb(
                control, system_stress_control_event,
                LV_EVENT_VALUE_CHANGED, RT_NULL);
        }
        break;
    case 3:
        control = lv_checkbox_create(tile);
        if (control != RT_NULL)
        {
            lv_checkbox_set_text(control, "CHECK");
            lv_obj_center(control);
            lv_obj_add_event_cb(
                control, system_stress_control_event,
                LV_EVENT_VALUE_CHANGED, RT_NULL);
        }
        break;
    case 4:
        control = lv_dropdown_create(tile);
        if (control != RT_NULL)
        {
            lv_dropdown_set_options(control, "A\nB\nC\nD");
            lv_obj_set_size(control, 100, 40);
            lv_obj_center(control);
            lv_obj_add_event_cb(
                control, system_stress_control_event,
                LV_EVENT_VALUE_CHANGED, RT_NULL);
        }
        break;
    case 5:
    {
        uint32_t row;
        control = lv_obj_create(tile);
        if (control != RT_NULL)
        {
            lv_obj_set_size(control, 100, 42);
            lv_obj_center(control);
            lv_obj_set_scroll_dir(control, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(
                control, LV_SCROLLBAR_MODE_ACTIVE);
            lv_obj_set_style_pad_all(control, 2, 0);
            for (row = 0; row < 4; row++)
            {
                lv_obj_t *label = lv_label_create(control);
                if (label != RT_NULL)
                {
                    lv_label_set_text_fmt(
                        label, "ROW %lu", (unsigned long)(row + 1U));
                    lv_obj_align(
                        label, LV_ALIGN_TOP_LEFT, 2,
                        (lv_coord_t)(row * 22U));
                }
            }
            lv_obj_add_event_cb(
                control, system_stress_control_event,
                LV_EVENT_SCROLL, RT_NULL);
        }
        break;
    }
    default:
        break;
    }

    if (control == RT_NULL)
    {
        system_stress_alloc_failures++;
        system_stress_errors++;
    }
    system_stress_controls[index] = control;
    return tile;
}

static uint32_t system_stress_random_next(void)
{
    system_stress_random_state =
        system_stress_random_state * 1664525U + 1013904223U;
    return system_stress_random_state;
}

static void system_stress_run_random_ui_action(void)
{
    uint32_t random = system_stress_random_next();
    uint32_t index = random % SYSTEM_STRESS_TILE_COUNT;
    uint32_t events_before = system_stress_ui_events;
    lv_obj_t *control = system_stress_controls[index];

    if (control == RT_NULL || !lv_obj_is_valid(control))
    {
        remove_object(&system_stress_tiles[index]);
        system_stress_controls[index] = RT_NULL;
        create_system_stress_tile(index);
        control = system_stress_controls[index];
    }
    if (control == RT_NULL)
    {
        return;
    }

    switch (index)
    {
    case 0:
        lv_obj_add_state(control, LV_STATE_PRESSED);
        lv_event_send(control, LV_EVENT_PRESSED, RT_NULL);
        lv_obj_clear_state(control, LV_STATE_PRESSED);
        lv_event_send(control, LV_EVENT_RELEASED, RT_NULL);
        lv_event_send(control, LV_EVENT_CLICKED, RT_NULL);
        break;
    case 1:
    case 3:
        if (lv_obj_has_state(control, LV_STATE_CHECKED))
        {
            lv_obj_clear_state(control, LV_STATE_CHECKED);
        }
        else
        {
            lv_obj_add_state(control, LV_STATE_CHECKED);
        }
        lv_event_send(control, LV_EVENT_VALUE_CHANGED, RT_NULL);
        break;
    case 2:
        lv_slider_set_value(
            control, (int32_t)((random >> 8) % 101U), LV_ANIM_OFF);
        lv_event_send(control, LV_EVENT_VALUE_CHANGED, RT_NULL);
        break;
    case 4:
        lv_dropdown_set_selected(
            control, (uint16_t)((random >> 8) % 4U));
        lv_event_send(control, LV_EVENT_VALUE_CHANGED, RT_NULL);
        break;
    case 5:
        lv_obj_scroll_to_y(
            control, (lv_coord_t)((random >> 8) % 48U), LV_ANIM_OFF);
        lv_event_send(control, LV_EVENT_SCROLL, RT_NULL);
        break;
    default:
        return;
    }
    system_stress_ui_actions++;
    if (system_stress_ui_events == events_before)
    {
        system_stress_errors++;
    }
}

static void system_stress_worker_entry(void *parameter)
{
    uint32_t memory[512];
    void *blocks[SYSTEM_STRESS_BLOCKS];
    rt_size_t block_sizes[SYSTEM_STRESS_BLOCKS];
    uint32_t seed = 0xA55A1234U;
    uint32_t i;
    (void)parameter;

    memset(blocks, 0, sizeof(blocks));
    while (!system_stress_stop_requested &&
           system_stress_errors < SYSTEM_STRESS_MAX_ERRORS)
    {
        uint32_t verify_seed = seed;

        for (i = 0; i < (sizeof(memory) / sizeof(memory[0])); i++)
        {
            seed = seed * 1664525U + 1013904223U;
            memory[i] = seed;
        }
        for (i = 0; i < (sizeof(memory) / sizeof(memory[0])); i++)
        {
            verify_seed = verify_seed * 1664525U + 1013904223U;
            if (memory[i] != verify_seed)
            {
                system_stress_errors++;
                break;
            }
        }

        memset(blocks, 0, sizeof(blocks));
        for (i = 0; i < SYSTEM_STRESS_BLOCKS; i++)
        {
            uint8_t *data;
            uint8_t pattern =
                (uint8_t)(system_stress_cycles + i * 37U);
            rt_size_t j;

            block_sizes[i] =
                64U + ((system_stress_cycles * 31U + i * 137U) % 1024U);
            blocks[i] = rt_malloc(block_sizes[i]);
            if (blocks[i] == RT_NULL)
            {
                system_stress_alloc_failures++;
                system_stress_errors++;
                break;
            }
            data = (uint8_t *)blocks[i];
            memset(data, pattern, block_sizes[i]);
            for (j = 0; j < block_sizes[i]; j++)
            {
                if (data[j] != pattern)
                {
                    system_stress_errors++;
                    break;
                }
            }
            if (system_stress_errors >= SYSTEM_STRESS_MAX_ERRORS)
            {
                break;
            }
        }
        for (i = SYSTEM_STRESS_BLOCKS; i > 0; i--)
        {
            if (blocks[i - 1] != RT_NULL)
            {
                rt_free(blocks[i - 1]);
                blocks[i - 1] = RT_NULL;
            }
        }
        system_stress_cycles++;
        rt_thread_mdelay(1);
    }

    system_stress_worker = RT_NULL;
    system_stress_active = false;
}

static void system_stress_stop_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED &&
        system_stress_active)
    {
        system_stress_stop_requested = true;
        set_status("Stopping system stress test...");
    }
}

static void show_system_stress_stop_control(void)
{
    lv_obj_t *button;
    remove_object(&control_container);
    control_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(control_container, 210, 70);
    lv_obj_align(control_container, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_bg_opa(control_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(control_container, 0, 0);
    lv_obj_set_style_pad_all(control_container, 0, 0);
    lv_obj_clear_flag(control_container, LV_OBJ_FLAG_SCROLLABLE);

    button = create_button(
        control_container, "STOP", 190, 60,
        system_stress_stop_event, RT_NULL);
    lv_obj_set_style_bg_color(
        button, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_center(button);
}

static void system_stress_retry_async(void *user_data)
{
    (void)user_data;
    remove_object(&control_container);
    remove_object(&title_label);
    remove_object(&status_label);
    delete_system_stress_tiles();
    start_system_stress_test(lv_scr_act());
}

static void system_stress_retry_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED &&
        !system_stress_active && system_stress_worker == RT_NULL)
    {
        lv_async_call(system_stress_retry_async, RT_NULL);
    }
}

static void show_system_stress_final_controls(void)
{
    lv_obj_t *retry;
    lv_obj_t *exit;

    remove_object(&control_container);
    control_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(control_container, LV_PCT(100), 80);
    lv_obj_align(control_container, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_bg_opa(control_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(control_container, 0, 0);
    lv_obj_set_style_pad_all(control_container, 0, 0);
    lv_obj_clear_flag(control_container, LV_OBJ_FLAG_SCROLLABLE);

    retry = create_button(
        control_container, "RETRY", 135, 58,
        system_stress_retry_event, RT_NULL);
    lv_obj_align(retry, LV_ALIGN_CENTER, -75, 0);
    exit = create_button(
        control_container, "EXIT", 135, 58, exit_event, RT_NULL);
    lv_obj_align(exit, LV_ALIGN_CENTER, 75, 0);
}

static void finish_system_stress_test(void)
{
    char summary[256];
    rt_uint32_t total = 0;
    rt_uint32_t used = 0;
    rt_uint32_t max_used = 0;
    rt_uint32_t elapsed =
        rt_tick_get_millisecond() - system_stress_started;

    if (system_stress_finalized)
    {
        return;
    }
    system_stress_finalized = true;
    if (system_stress_ui_timer != RT_NULL)
    {
        lv_timer_del(system_stress_ui_timer);
        system_stress_ui_timer = RT_NULL;
    }
    delete_system_stress_tiles();
    rt_memory_info(&total, &used, &max_used);
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);

    rt_snprintf(
        summary, sizeof(summary),
        "%s\n"
        "elapsed=%lus  cycles=%lu\n"
        "heap=%lu/%lu KB  delta=%ld KB\n"
        "UI actions=%lu  events=%lu\n"
        "alloc_fail=%lu  errors=%lu",
        system_stress_errors == 0 ?
            "SYSTEM STRESS STOPPED - PASS" :
            "SYSTEM STRESS FAILED",
        (unsigned long)(elapsed / 1000U),
        (unsigned long)system_stress_cycles,
        (unsigned long)(used / 1024U),
        (unsigned long)(total / 1024U),
        (long)((int32_t)used - (int32_t)system_stress_heap_start) / 1024L,
        (unsigned long)system_stress_ui_actions,
        (unsigned long)system_stress_ui_events,
        (unsigned long)system_stress_alloc_failures,
        (unsigned long)system_stress_errors);
    set_status(summary);
    show_system_stress_final_controls();
}

static void system_stress_ui_timer_cb(lv_timer_t *timer)
{
    static const lv_palette_t palettes[] = {
        LV_PALETTE_RED, LV_PALETTE_ORANGE, LV_PALETTE_YELLOW,
        LV_PALETTE_GREEN, LV_PALETTE_BLUE, LV_PALETTE_PURPLE,
    };
    char text[256];
    rt_uint32_t total = 0;
    rt_uint32_t used = 0;
    rt_uint32_t max_used = 0;
    rt_uint32_t elapsed;
    uint32_t i;
    (void)timer;

    if (!system_stress_active)
    {
        finish_system_stress_test();
        return;
    }

    system_stress_ui_refresh_count++;
    elapsed = rt_tick_get_millisecond() - system_stress_started;
    rt_memory_info(&total, &used, &max_used);
    system_stress_run_random_ui_action();
    for (i = 0; i < SYSTEM_STRESS_TILE_COUNT; i++)
    {
        if (system_stress_tiles[i] == RT_NULL ||
            !lv_obj_is_valid(system_stress_tiles[i]))
        {
            system_stress_controls[i] = RT_NULL;
            create_system_stress_tile(i);
        }
        if (system_stress_tiles[i] == RT_NULL)
        {
            continue;
        }
        lv_obj_set_style_bg_color(
            system_stress_tiles[i],
            lv_palette_main(
                palettes[(i + system_stress_ui_refresh_count) %
                         (sizeof(palettes) / sizeof(palettes[0]))]),
            0);
    }

    /* Recreate one LVGL object periodically to exercise object allocation,
     * style initialization and deletion without blocking the UI thread. */
    if ((system_stress_ui_refresh_count % 5U) == 0U)
    {
        uint32_t recreate = (system_stress_ui_refresh_count / 5U) %
                            SYSTEM_STRESS_TILE_COUNT;
        remove_object(&system_stress_tiles[recreate]);
        system_stress_controls[recreate] = RT_NULL;
        create_system_stress_tile(recreate);
    }
    lv_obj_set_style_bg_color(
        lv_scr_act(),
        (system_stress_ui_refresh_count & 1U) ?
            lv_color_hex(0x07131F) : lv_color_hex(0x160A1C),
        0);

    rt_snprintf(
        text, sizeof(text),
        "RUNNING - touch STOP to finish\n"
        "elapsed=%lus  cycles=%lu\n"
        "heap=%lu/%lu KB  peak=%lu KB\n"
        "alloc_fail=%lu  errors=%lu\n"
        "UI actions=%lu  events=%lu  refresh=%lu",
        (unsigned long)(elapsed / 1000U),
        (unsigned long)system_stress_cycles,
        (unsigned long)(used / 1024U),
        (unsigned long)(total / 1024U),
        (unsigned long)(max_used / 1024U),
        (unsigned long)system_stress_alloc_failures,
        (unsigned long)system_stress_errors,
        (unsigned long)system_stress_ui_actions,
        (unsigned long)system_stress_ui_events,
        (unsigned long)system_stress_ui_refresh_count);
    set_status(text);
}

static void start_system_stress_test(lv_obj_t *parent)
{
    rt_uint32_t total = 0;
    rt_uint32_t max_used = 0;
    uint32_t i;

    if (system_stress_active || system_stress_worker != RT_NULL)
    {
        return;
    }
    if (menu_container != RT_NULL && lv_obj_is_valid(menu_container))
    {
        lv_obj_add_flag(menu_container, LV_OBJ_FLAG_HIDDEN);
    }

    system_stress_stop_requested = false;
    system_stress_active = true;
    system_stress_finalized = false;
    system_stress_cycles = 0;
    system_stress_errors = 0;
    system_stress_alloc_failures = 0;
    system_stress_ui_refresh_count = 0;
    system_stress_ui_actions = 0;
    system_stress_ui_events = 0;
    system_stress_started = rt_tick_get_millisecond();
    rt_memory_info(&total, &system_stress_heap_start, &max_used);
    system_stress_random_state =
        system_stress_started ^ system_stress_heap_start ^ 0xC001D00DU;

    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
    title_label = lv_label_create(parent);
    lv_label_set_text(title_label, "SYSTEM STRESS TEST");
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 18);
    status_label = lv_label_create(parent);
    lv_obj_set_width(status_label, LV_PCT(94));
    lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(status_label, LV_ALIGN_CENTER, 0, 55);
    memset(system_stress_tiles, 0, sizeof(system_stress_tiles));
    memset(system_stress_controls, 0, sizeof(system_stress_controls));
    for (i = 0; i < SYSTEM_STRESS_TILE_COUNT; i++)
    {
        create_system_stress_tile(i);
    }
    show_system_stress_stop_control();

    system_stress_ui_timer = lv_timer_create(
        system_stress_ui_timer_cb, 100, RT_NULL);
    if (system_stress_ui_timer == RT_NULL)
    {
        system_stress_errors++;
        system_stress_active = false;
        set_status("SYSTEM STRESS FAILED\nUI timer allocation failed");
        finish_system_stress_test();
        return;
    }
    system_stress_worker = rt_thread_create(
        "sys_stress", system_stress_worker_entry,
        RT_NULL, 5120, 29, 10);
    if (system_stress_worker == RT_NULL ||
        rt_thread_startup(system_stress_worker) != RT_EOK)
    {
        if (system_stress_worker != RT_NULL)
        {
            rt_thread_delete(system_stress_worker);
            system_stress_worker = RT_NULL;
        }
        system_stress_errors++;
        system_stress_active = false;
        finish_system_stress_test();
        return;
    }
}

static void system_stress_start_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED &&
        !system_stress_active && system_stress_worker == RT_NULL)
    {
        start_system_stress_test(lv_scr_act());
    }
}

static void retry_async(void *user_data)
{
    (void)user_data;
    remove_object(&control_container);
    remove_object(&confirm_container);
    delete_touch_cells();
    remove_object(&title_label);
    remove_object(&status_label);
    start_interactive_test(lv_scr_act());
}

static void retry_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED && !test_active)
    {
        lv_async_call(retry_async, RT_NULL);
    }
}

static void show_cancel_control(void)
{
    lv_obj_t *button;
    remove_object(&control_container);
    control_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(control_container, 140, 60);
    lv_obj_align(control_container, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_bg_opa(control_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(control_container, 0, 0);
    lv_obj_set_style_pad_all(control_container, 0, 0);
    lv_obj_clear_flag(control_container, LV_OBJ_FLAG_SCROLLABLE);

    button = create_button(
        control_container, "CANCEL", 130, 52, cancel_event, RT_NULL);
    lv_obj_center(button);
}

static void show_final_controls(void)
{
    lv_obj_t *retry;
    lv_obj_t *exit;
    remove_object(&control_container);
    control_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(control_container, LV_PCT(100), 80);
    lv_obj_align(control_container, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_bg_opa(control_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(control_container, 0, 0);
    lv_obj_set_style_pad_all(control_container, 0, 0);
    lv_obj_clear_flag(control_container, LV_OBJ_FLAG_SCROLLABLE);

    retry = create_button(
        control_container, "RETRY", 135, 58, retry_event, RT_NULL);
    lv_obj_align(retry, LV_ALIGN_CENTER, -75, 0);
    exit = create_button(
        control_container, "EXIT", 135, 58, exit_event, RT_NULL);
    lv_obj_align(exit, LV_ALIGN_CENTER, 75, 0);
}

static void finish_interactive_test(bool cancelled)
{
    if (!test_active)
    {
        return;
    }
    if (test_timer != RT_NULL)
    {
        lv_timer_del(test_timer);
        test_timer = RT_NULL;
    }
    remove_object(&confirm_container);
    remove_object(&control_container);
    delete_touch_cells();
    if (peripheral_provider.control_motor != RT_NULL)
    {
        peripheral_provider.control_motor(false, RT_NULL);
    }

    test_active = false;
    test_stage = TEST_STAGE_DONE;
    if (cancelled)
    {
        set_status("INTERACTIVE TEST CANCELLED");
    }
    else if (failures == 0)
    {
        set_status("ALL INTERACTIVE TESTS PASS");
    }
    else
    {
        char summary[128];
        rt_snprintf(summary, sizeof(summary),
                    "INTERACTIVE TEST FAILED (%d)\n%s",
                    failures, failed_items);
        set_status(summary);
    }
    show_final_controls();
}

static void touch_event(lv_event_t *event)
{
    uint32_t index = (uint32_t)(uintptr_t)lv_event_get_user_data(event);
    char text[48];
    unsigned int count = 0;
    uint8_t value;

    if (lv_event_get_code(event) != LV_EVENT_PRESSED || index >= 4)
    {
        return;
    }
    touch_mask |= (uint8_t)(1U << index);
    lv_obj_set_style_bg_color(
        touch_cells[index], lv_palette_main(LV_PALETTE_GREEN), 0);

    value = touch_mask & TOUCH_ALL;
    while (value != 0)
    {
        count += value & 1U;
        value >>= 1;
    }
    rt_snprintf(text, sizeof(text), "Touch all 4 zones: %u/4", count);
    set_status(text);
}

static void confirm_event(lv_event_t *event)
{
    bool passed;
    if (lv_event_get_code(event) != LV_EVENT_CLICKED)
    {
        return;
    }
    passed = (bool)(uintptr_t)lv_event_get_user_data(event);
    if (test_stage == TEST_STAGE_DISPLAY_CONFIRM)
    {
        report_result("display", passed);
        if (peripheral_provider.control_motor != RT_NULL)
        {
            motor_params_t motor = {
                .duty_cycle = 80,
                .period = 300000,
                .repeat_times = 2,
            };
            peripheral_provider.control_motor(true, &motor);
        }
        test_stage = TEST_STAGE_MOTOR_CONFIRM;
    }
    else if (test_stage == TEST_STAGE_MOTOR_CONFIRM)
    {
        report_result("motor", passed);
        test_stage = TEST_STAGE_CHARGE;
        stage_started = rt_tick_get_millisecond();
        last_charge_request = 0;
    }
    remove_object(&confirm_container);
}

static void show_confirmation(const char *question)
{
    lv_obj_t *label;
    lv_obj_t *pass;
    lv_obj_t *fail;
    lv_obj_t *cancel;

    remove_object(&confirm_container);
    confirm_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(confirm_container, LV_PCT(100), LV_PCT(100));
    lv_obj_center(confirm_container);
    lv_obj_set_style_bg_color(confirm_container, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(confirm_container, LV_OPA_COVER, 0);
    lv_obj_clear_flag(confirm_container, LV_OBJ_FLAG_SCROLLABLE);

    label = lv_label_create(confirm_container);
    lv_label_set_text(label, question);
    lv_obj_set_width(label, LV_PCT(90));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -75);

    pass = create_button(
        confirm_container, "PASS", 180, 70, confirm_event,
        (void *)(uintptr_t)true);
    lv_obj_set_style_bg_color(
        pass, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_align(pass, LV_ALIGN_CENTER, -100, 35);

    fail = create_button(
        confirm_container, "FAIL", 180, 70, confirm_event,
        (void *)(uintptr_t)false);
    lv_obj_set_style_bg_color(
        fail, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(fail, LV_ALIGN_CENTER, 100, 35);

    cancel = create_button(
        confirm_container, "CANCEL", 150, 55, cancel_event, RT_NULL);
    lv_obj_align(cancel, LV_ALIGN_BOTTOM_MID, 0, -18);
}

static void create_touch_cells(void)
{
    static const lv_align_t alignments[4] = {
        LV_ALIGN_TOP_LEFT, LV_ALIGN_TOP_RIGHT,
        LV_ALIGN_BOTTOM_LEFT, LV_ALIGN_BOTTOM_RIGHT,
    };
    uint32_t i;
    for (i = 0; i < 4; i++)
    {
        touch_cells[i] = lv_obj_create(lv_scr_act());
        lv_obj_set_size(touch_cells[i], LV_PCT(48), LV_PCT(38));
        lv_obj_align(touch_cells[i], alignments[i], 0, 0);
        lv_obj_set_style_bg_color(
            touch_cells[i], lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_obj_set_style_bg_opa(touch_cells[i], LV_OPA_50, 0);
        lv_obj_add_flag(touch_cells[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(touch_cells[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(
            touch_cells[i], touch_event, LV_EVENT_PRESSED,
            (void *)(uintptr_t)i);
    }
}

static void interactive_timer_cb(lv_timer_t *timer)
{
    rt_uint32_t now = rt_tick_get_millisecond();
    (void)timer;

    switch (test_stage)
    {
    case TEST_STAGE_COLOR:
        if ((now - stage_started) >= 600)
        {
            lv_color_t colors[] = {
                LV_COLOR_WHITE, LV_COLOR_RED, LV_COLOR_GREEN,
                LV_COLOR_BLUE, LV_COLOR_BLACK,
            };
            lv_obj_set_style_bg_color(
                lv_scr_act(), colors[color_index], 0);
            color_index++;
            stage_started = now;
            if (color_index >= (sizeof(colors) / sizeof(colors[0])))
            {
                lv_obj_clear_flag(title_label, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(status_label, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(control_container, LV_OBJ_FLAG_HIDDEN);
                touch_mask = 0;
                create_touch_cells();
                lv_obj_move_foreground(control_container);
                test_stage = TEST_STAGE_TOUCH;
                stage_started = now;
                set_status("Touch all 4 zones: 0/4");
            }
        }
        break;

    case TEST_STAGE_TOUCH:
        if (touch_mask == TOUCH_ALL)
        {
            report_result("touch", true);
            delete_touch_cells();
            test_stage = TEST_STAGE_DISPLAY_CONFIRM;
        }
        else if ((now - stage_started) >= TOUCH_TIMEOUT_MS)
        {
            report_result("touch", false);
            delete_touch_cells();
            test_stage = TEST_STAGE_DISPLAY_CONFIRM;
        }
        break;

    case TEST_STAGE_DISPLAY_CONFIRM:
        if (confirm_container == RT_NULL)
        {
            show_confirmation("Were all display colors uniform?");
        }
        break;

    case TEST_STAGE_MOTOR_CONFIRM:
        if (confirm_container == RT_NULL)
        {
            show_confirmation("Did you feel two vibrations?");
        }
        break;

    case TEST_STAGE_CHARGE:
        set_status("Place watch on charger...");
        if (SkaiWatchSys.charger_status == InCharging ||
            SkaiWatchSys.charger_status == ChargingComplete)
        {
            report_result("charge", true);
            finish_interactive_test(false);
            break;
        }
        if (last_charge_request == 0 ||
            (now - last_charge_request) >= 1000)
        {
            if (watch_sys_sync.request_charge_status != RT_NULL)
            {
                watch_sys_sync.request_charge_status();
            }
            last_charge_request = now;
        }
        if ((now - stage_started) >= CHARGE_TIMEOUT_MS)
        {
            report_result("charge", false);
            finish_interactive_test(false);
        }
        break;

    default:
        break;
    }
}

static void start_interactive_test(lv_obj_t *parent)
{
    test_active = true;
    cancel_pending = false;
    failures = 0;
    failed_items[0] = '\0';
    touch_mask = 0;
    color_index = 0;
    memset(touch_cells, 0, sizeof(touch_cells));

    if (menu_container != RT_NULL && lv_obj_is_valid(menu_container))
    {
        lv_obj_add_flag(menu_container, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
    title_label = lv_label_create(parent);
    lv_label_set_text(title_label, "INTERACTIVE TEST");
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 20);
    status_label = lv_label_create(parent);
    lv_obj_set_width(status_label, LV_PCT(90));
    lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(status_label, LV_ALIGN_CENTER, 0, 0);
    set_status("Display color test...");
    show_cancel_control();

    lv_obj_add_flag(title_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(status_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(control_container, LV_OBJ_FLAG_HIDDEN);
    test_stage = TEST_STAGE_COLOR;
    stage_started = rt_tick_get_millisecond();
    test_timer = lv_timer_create(interactive_timer_cb, 100, RT_NULL);
}

static void interactive_start_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED && !test_active)
    {
        start_interactive_test(lv_scr_act());
    }
}

static void random_address_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED)
    {
        extern void generate_random_public_address(uint8_t device_id);
        generate_random_public_address(0);
        rt_thread_mdelay(50);
        peripheral_provider.hcpu_reboot();
    }
}

static void screen_event_handler(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_GESTURE)
    {
        lv_dir_t direction =
            lv_indev_get_gesture_dir(lv_indev_get_act());
        if (direction == LV_DIR_RIGHT)
        {
            if (test_active)
            {
                finish_interactive_test(true);
                gui_app_self_exit();
            }
            else if (system_stress_active ||
                     system_stress_worker != RT_NULL)
            {
                system_stress_stop_requested = true;
                gui_app_self_exit();
            }
            else if (screening_active)
            {
                screen_cancel_requested = true;
                set_status("Cancelling board screening...");
            }
            else
            {
                gui_app_self_exit();
            }
        }
    }
}

static lv_obj_t *on_start(lv_obj_t *parent)
{
    bool remote_screening = screen_request_pending;
    lv_obj_t *title;
    lv_obj_t *interactive;
    lv_obj_t *stress;
    lv_obj_t *random;

    screen_request_pending = false;
    app_visible = true;
    lv_obj_add_event_cb(
        parent, screen_event_handler, LV_EVENT_ALL, RT_NULL);
    if (remote_screening)
    {
        start_board_screening_ui(parent);
        return parent;
    }

    menu_container = lv_obj_create(parent);
    lv_obj_set_size(menu_container, LV_PCT(100), LV_PCT(100));
    lv_obj_center(menu_container);
    lv_obj_set_style_bg_opa(menu_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(menu_container, 0, 0);
    lv_obj_clear_flag(menu_container, LV_OBJ_FLAG_SCROLLABLE);

    title = lv_label_create(menu_container);
    lv_label_set_text(title, "FACTORY TEST");
    lv_obj_set_style_text_font(
        title, LV_EXT_FONT_GET(get_system_font_size(1)), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 25);

    interactive = create_button(
        menu_container, "INTERACTIVE TEST", 300, 82,
        interactive_start_event, RT_NULL);
    lv_obj_set_style_bg_color(
        interactive, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_align(interactive, LV_ALIGN_CENTER, 0, -105);

    stress = create_button(
        menu_container, "SYSTEM STRESS TEST", 300, 82,
        system_stress_start_event, RT_NULL);
    lv_obj_set_style_bg_color(
        stress, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_align(stress, LV_ALIGN_CENTER, 0, 0);

    random = create_button(
        menu_container, "GENERATE RANDOM\nADDRESS", 300, 82,
        random_address_event, RT_NULL);
    lv_obj_set_style_bg_color(
        random, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_align(random, LV_ALIGN_CENTER, 0, 105);
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
    lv_async_call_cancel(cancel_async, RT_NULL);
    lv_async_call_cancel(retry_async, RT_NULL);
    lv_async_call_cancel(system_stress_retry_async, RT_NULL);
    cancel_pending = false;
    if (screen_ui_timer != RT_NULL)
    {
        lv_timer_del(screen_ui_timer);
        screen_ui_timer = RT_NULL;
    }
    if (test_active)
    {
        finish_interactive_test(true);
    }
    if (system_stress_ui_timer != RT_NULL)
    {
        lv_timer_del(system_stress_ui_timer);
        system_stress_ui_timer = RT_NULL;
    }
    if (system_stress_active || system_stress_worker != RT_NULL)
    {
        system_stress_stop_requested = true;
        system_stress_finalized = true;
    }
    delete_system_stress_tiles();
    if (screening_active)
    {
        screen_cancel_requested = true;
    }
    app_visible = false;
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    (void)param;
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

static int app_main(intent_t intent)
{
    (void)intent;
    gui_app_regist_msg_handler(APP_ID, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(
    LV_EXT_STR_ID(factory_test), IMG_LOGO, APP_ID, app_main);
