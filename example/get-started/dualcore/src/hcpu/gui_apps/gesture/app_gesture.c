/**
 ******************************************************************************
 * @file   app_gesture.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/*********************
 *      INCLUDES
 *********************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>
#include <dfs_posix.h>
#include <time.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gesture_recognition_task.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "watch_system_core_task.h"
#include "bloc_control.h"
#include "bloc_motor.h"
#include "bloc_peripheral.h"
#include "bloc_filesystem.h"
#ifdef BSP_USING_MODEL_WATCH_GLOBAL_DATA
#include "watch_global_data.h"
#endif
#ifdef BSP_USING_MODEL_WATCH_SYS_INTERACT
#include "watch_system_interact.h"
#endif
#ifdef BSP_USING_GESTURE_HANDLER
#include "gesture_handler.h"
#include "gesture_model_loader.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif

#define DBG_TAG "app.gesture"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_GESTURE


/**
 * @brief Structure to hold all UI components of the gesture app
 */
typedef struct
{
    lv_obj_t *gesture_circle; // Center circle for gesture display
    lv_obj_t *status_label;   // Label showing current gesture status
    lv_obj_t *count_label;    // Label showing tap count
    lv_obj_t *reset_button;   // Button to reset counter
    lv_obj_t *main_container; // Main container
    lv_obj_t *combo_label;    // Label showing current selection combo
    lv_obj_t *height_dd;      // Height dropdown
    lv_obj_t *weight_dd;      // Weight dropdown
    lv_obj_t *age_dd;         // Age dropdown
    lv_obj_t *gender_dd;      // Gender dropdown
    lv_obj_t *collect_button; // Button to start/stop gesture data collection
    lv_obj_t *gesture_count_label; // Label showing collected gesture count
    lv_obj_t *confirm_msgbox; // Message box for save confirmation
    lv_obj_t *sync_button; // Button to sync all gesture files to phone
    lv_obj_t *sync_status_label; // Label showing sync status
} gesture_ui_t;

typedef struct
{
    uint32_t tap_count; // Total tap count
    bool is_pressed;    // Current press state
    int height;         // Selected height (100-200)
    int weight;         // Selected weight (30-200)
    int age;            // Selected age (5-110)
    int gender;         // Selected gender (0=male, 1=female)
    char combo_str[64]; // Combination string: gender_age_weight_height
    bool is_recording;  // Is currently recording gesture data
    uint32_t gesture_record_count; // Number of gestures recorded
    int csv_fd;         // CSV file descriptor
    char csv_filename[128]; // CSV file path
    char gesture_type[32]; // Current gesture type being recorded
    bool is_tap_gesture; // true for TAP, false for RELEASE
    bool is_syncing; // Is currently syncing files
    int sync_file_count; // Number of files to sync
    int synced_count; // Number of files synced
} app_gesture_data_ctx_t;


/* IMU data buffer for CSV writing */
typedef struct {
    rt_tick_t timestamp;
    float ax, ay, az;  // Linear acceleration
    float gx, gy, gz;  // Gravity component
} imu_sample_t;

static gesture_ui_t ui; // All UI components
static app_gesture_data_ctx_t app_gesture_data_ctx = {0};
static void handle_gesture_data_collection(int16_t (*dataset)[6], int sample_num, rt_tick_t *timestamps);
/**
 * @brief Global function to receive gesture IMU data from gesture_recognition_task
 * This function should be called from gesture_recognition_task.c when gesture is detected
 */
void app_gesture_receive_imu_data(int16_t (*dataset)[6], int sample_num)
{
    if (!app_gesture_data_ctx.is_recording)
    {
        return;
    }

    // Stop accepting new data if already collected 20 gestures
    if (app_gesture_data_ctx.gesture_record_count >= 20)
    {
        return;
    }

    // Filter by gesture type
    if (app_gesture_data_ctx.is_tap_gesture && sample_num != TAP_TARGET_SAMPLE_NUM)
    {
        return; // Collecting TAP but received RELEASE
    }
    if (!app_gesture_data_ctx.is_tap_gesture && sample_num != RELEASE_TARGET_SAMPLE_NUM)
    {
        return; // Collecting RELEASE but received TAP
    }

    handle_gesture_data_collection(dataset, sample_num, NULL);
}

/* Forward declarations for functions */
static void update_gesture_display(void);
static void update_count_display(void);
static void reset_counter(void);
static void update_combo_string(void);
static void dropdown_event_cb(lv_event_t *e);
static lv_obj_t *create_gesture_screen(lv_obj_t *parent);
static int open_csv_file(const char *gesture_type);
static void write_gesture_to_csv(uint8_t gesture);
static void close_and_save_csv(void);
static void close_and_delete_csv(void);
static void start_gesture_collection(void);
static void stop_gesture_collection(bool save);
static void show_save_confirmation_dialog(void);
static void update_gesture_count_display(void);
static void sync_all_gesture_files(void);
static void update_sync_status_display(void);

/**
 * @brief Update the gesture circle display
 */
static void update_gesture_display(void)
{
    if (app_gesture_data_ctx.is_pressed)
    {
        // Change to light blue when pressed
        if (lv_obj_is_valid(ui.gesture_circle))
        {
            lv_obj_set_style_bg_color(ui.gesture_circle, lv_color_make(173, 216, 230), LV_PART_MAIN | LV_STATE_DEFAULT);
        }

        if (lv_obj_is_valid(ui.status_label))
        {
            lv_label_set_text(ui.status_label, "Pressed");
        }
    }
    else
    {
        // Change back to gray when released
        if (lv_obj_is_valid(ui.gesture_circle))
        {
            lv_obj_set_style_bg_color(ui.gesture_circle, lv_color_make(30, 30, 30), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        if (lv_obj_is_valid(ui.status_label))
        {
            lv_label_set_text(ui.status_label, "Released");
        }
    }
}

/**
 * @brief Update the count display
 */
static void update_count_display(void)
{
    if (!lv_obj_is_valid(ui.count_label))
    {
        return;
    }

    char count_str[32];
    snprintf(count_str, sizeof(count_str), "Taps: %d", app_gesture_data_ctx.tap_count);
    lv_label_set_text(ui.count_label, count_str);
}

/**
 * @brief Reset the tap counter
 */
static void reset_counter(void)
{
    app_gesture_data_ctx.tap_count = 0;
    update_count_display();
    LOG_D("Counter reset");
}

/**
 * @brief Update the combination string display
 */
static bool gesture_tap_hz = false;
static void update_combo_string(void)
{
    const char *gender_str = app_gesture_data_ctx.gender == 0 ? "male" : "female";
    const char *mode_suffix = gesture_tap_hz ? "tap" : "releas";
    
    snprintf(app_gesture_data_ctx.combo_str, sizeof(app_gesture_data_ctx.combo_str),
             "%s_%d_%d_%d_%s", 
             gender_str,
             app_gesture_data_ctx.age,
             app_gesture_data_ctx.weight,
             app_gesture_data_ctx.height,
             mode_suffix);
    
    if (lv_obj_is_valid(ui.combo_label))
    {
        lv_label_set_text(ui.combo_label, app_gesture_data_ctx.combo_str);
    }
    
    LOG_I("Combo: %s", app_gesture_data_ctx.combo_str);
}

/**
 * @brief Dropdown event callback
 */
static void dropdown_event_cb(lv_event_t *e)
{
    lv_obj_t *dropdown = lv_event_get_target(e);
    uint16_t selected = lv_dropdown_get_selected(dropdown);
    
    if (dropdown == ui.height_dd)
    {
        app_gesture_data_ctx.height = 100 + selected;
    }
    else if (dropdown == ui.weight_dd)
    {
        app_gesture_data_ctx.weight = 30 + selected;
    }
    else if (dropdown == ui.age_dd)
    {
        app_gesture_data_ctx.age = 5 + selected;
    }
    else if (dropdown == ui.gender_dd)
    {
        app_gesture_data_ctx.gender = selected;
    }
    
    update_combo_string();
}

/**
 * @brief Open CSV file for gesture data collection
 */
static int open_csv_file(const char *gesture_type)
{
    time_t now;
    time(&now);

    // Create filename: /gesture/gender_age_weight_height_gesture_timestamp.csv
    // Using RTC time in seconds since epoch
    snprintf(app_gesture_data_ctx.csv_filename, sizeof(app_gesture_data_ctx.csv_filename),
             "/gesture/%s_%ld.csv",
             app_gesture_data_ctx.combo_str,
             (long)now);

    // Create directory if not exists
    mkdir("/gesture", 0777);

    // Open file
    app_gesture_data_ctx.csv_fd = open(app_gesture_data_ctx.csv_filename, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (app_gesture_data_ctx.csv_fd < 0)
    {
        LOG_E("Failed to open CSV file: %s", app_gesture_data_ctx.csv_filename);
        return -1;
    }

    // Write CSV header
    const char *header = "timestamp,ax,ay,az,gx,gy,gz\n";
    write(app_gesture_data_ctx.csv_fd, header, strlen(header));

    LOG_I("CSV file opened: %s", app_gesture_data_ctx.csv_filename);
    return 0;
}

/**
 * @brief Write IMU samples to CSV file
 */
static void write_imu_samples_to_csv(imu_sample_t *samples, int num_samples)
{
    if (app_gesture_data_ctx.csv_fd < 0 || !app_gesture_data_ctx.is_recording)
    {
        return;
    }

    for (int i = 0; i < num_samples; i++)
    {
        char line[256];
        snprintf(line, sizeof(line), "%lu,%f,%f,%f,%f,%f,%f\n",
                 samples[i].timestamp,
                 samples[i].ax, samples[i].ay, samples[i].az,
                 samples[i].gx, samples[i].gy, samples[i].gz);
        write(app_gesture_data_ctx.csv_fd, line, strlen(line));
    }

    LOG_D("Written %d IMU samples to CSV", num_samples);
}

/**
 * @brief Close and save CSV file
 */
static void close_and_save_csv(void)
{
    if (app_gesture_data_ctx.csv_fd >= 0)
    {
        close(app_gesture_data_ctx.csv_fd);
        app_gesture_data_ctx.csv_fd = -1;
        LOG_I("CSV file saved: %s", app_gesture_data_ctx.csv_filename);
    }
}

/**
 * @brief Close and delete CSV file
 */
static void close_and_delete_csv(void)
{
    if (app_gesture_data_ctx.csv_fd >= 0)
    {
        close(app_gesture_data_ctx.csv_fd);
        app_gesture_data_ctx.csv_fd = -1;
    }

    if (strlen(app_gesture_data_ctx.csv_filename) > 0)
    {
        unlink(app_gesture_data_ctx.csv_filename);
        LOG_I("CSV file deleted: %s", app_gesture_data_ctx.csv_filename);
    }
}

/**
 * @brief Update gesture count display
 */
static void update_gesture_count_display(void)
{
    if (!lv_obj_is_valid(ui.gesture_count_label))
    {
        return;
    }

    char count_str[64];
    snprintf(count_str, sizeof(count_str), "手勢數: %d/20", app_gesture_data_ctx.gesture_record_count);
    lv_label_set_text(ui.gesture_count_label, count_str);
}

/**
 * @brief Handle gesture data for collection mode
 */
static void handle_gesture_data_collection(int16_t (*dataset)[6], int sample_num, rt_tick_t *timestamps)
{
    if (!app_gesture_data_ctx.is_recording)
    {
        return;
    }

    // Convert int16_t to float and write to CSV
    imu_sample_t samples[RELEASE_TARGET_SAMPLE_NUM];
    int num_samples = sample_num > RELEASE_TARGET_SAMPLE_NUM ? RELEASE_TARGET_SAMPLE_NUM : sample_num;

    for (int i = 0; i < num_samples; i++)
    {
        samples[i].timestamp = timestamps ? timestamps[i] : rt_tick_get();
        // Convert raw data to m/s^2 for acceleration and g for gravity
        samples[i].ax = dataset[i][0] / INT16_to_G * GRAVITY;
        samples[i].ay = dataset[i][1] / INT16_to_G * GRAVITY;
        samples[i].az = dataset[i][2] / INT16_to_G * GRAVITY;
        samples[i].gx = dataset[i][3] / INT16_to_G;
        samples[i].gy = dataset[i][4] / INT16_to_G;
        samples[i].gz = dataset[i][5] / INT16_to_G;
    }

    write_imu_samples_to_csv(samples, num_samples);

    app_gesture_data_ctx.gesture_record_count++;
    update_gesture_count_display();

    LOG_D("Gesture %d collected, count: %d", app_gesture_data_ctx.gesture_record_count, app_gesture_data_ctx.gesture_record_count);

    // Check if reached 20 gestures
    if (app_gesture_data_ctx.gesture_record_count >= 20)
    {
        show_save_confirmation_dialog();
    }
}

/**
 * @brief Handle tap indicator - called by UI handler
 */
static void handle_tap_indicator(uint8_t gesture)
{
    LOG_D("Tap indicator received: %d", gesture);
    if (gesture == 1) // Press
    {
        app_gesture_data_ctx.is_pressed = true;
        update_gesture_display();
        LOG_D("Gesture press detected");
    }
    else if (gesture == 0) // Release
    {
        app_gesture_data_ctx.is_pressed = false;
        app_gesture_data_ctx.tap_count++;
        update_gesture_display();
        update_count_display();
        LOG_D("Gesture release detected, tap count: %d", app_gesture_data_ctx.tap_count);
    }
}

/**
 * @brief Reset button event callback
 */
static void reset_button_event_cb(lv_event_t *e)
{
    reset_counter();
}

/**
 * @brief Start gesture data collection
 */
static void start_gesture_collection(void)
{
    const char *gesture_type = gesture_tap_hz ? "tap" : "release";

    if (open_csv_file(gesture_type) < 0)
    {
        LOG_E("Failed to start gesture collection");
        return;
    }

    app_gesture_data_ctx.is_recording = true;
    app_gesture_data_ctx.gesture_record_count = 0;
    app_gesture_data_ctx.is_tap_gesture = gesture_tap_hz;
    strncpy(app_gesture_data_ctx.gesture_type, gesture_type, sizeof(app_gesture_data_ctx.gesture_type) - 1);

    // Update button appearance
    if (lv_obj_is_valid(ui.collect_button))
    {
        lv_obj_set_style_bg_color(ui.collect_button, lv_color_make(200, 0, 0), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_t *label = lv_obj_get_child(ui.collect_button, 0);
        if (lv_obj_is_valid(label))
        {
            lv_label_set_text(label, "捨棄");
        }
    }

    update_gesture_count_display();
    LOG_I("Gesture collection started: %s", gesture_type);
}

/**
 * @brief Stop gesture data collection
 */
static void stop_gesture_collection(bool save)
{
    app_gesture_data_ctx.is_recording = false;

    if (save)
    {
        close_and_save_csv();
    }
    else
    {
        close_and_delete_csv();
    }

    // Reset button appearance
    if (lv_obj_is_valid(ui.collect_button))
    {
        lv_obj_set_style_bg_color(ui.collect_button, lv_color_make(0, 120, 215), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_t *label = lv_obj_get_child(ui.collect_button, 0);
        if (lv_obj_is_valid(label))
        {
            lv_label_set_text(label, "開始收集");
        }
    }

    app_gesture_data_ctx.gesture_record_count = 0;
    update_gesture_count_display();
    update_sync_status_display();

    LOG_I("Gesture collection stopped, saved: %d", save);
}

/**
 * @brief Confirmation dialog button callback
 */
static void msgbox_event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint16_t btn_id = lv_msgbox_get_active_btn(obj);

    if (btn_id == 0) // "是" button
    {
        stop_gesture_collection(true);
    }
    else // "否" button
    {
        stop_gesture_collection(false);
    }

    lv_msgbox_close(obj);
    lv_msgbox_close(ui.confirm_msgbox);
    ui.confirm_msgbox = NULL;
}

/**
 * @brief Show save confirmation dialog
 */
static void show_save_confirmation_dialog(void)
{
    if (lv_obj_is_valid(ui.confirm_msgbox))
    {
        return; // Dialog already shown
    }

    static const char *btns[] = {"是", "否", ""};
    ui.confirm_msgbox = lv_msgbox_create(lv_scr_act(), "確認", "已收集20筆手勢\n是否儲存?", btns, false);
    lv_obj_add_event_cb(ui.confirm_msgbox, msgbox_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_center(ui.confirm_msgbox);
    lv_obj_set_style_bg_color(ui.confirm_msgbox, lv_color_make(40, 40, 40), LV_PART_MAIN | LV_STATE_DEFAULT);

    LOG_I("Save confirmation dialog shown");
}

/**
 * @brief Collect button event callback
 */
static void collect_button_event_cb(lv_event_t *e)
{
    if (app_gesture_data_ctx.is_recording)
    {
        // Manually stop collection
        stop_gesture_collection(false);
    }
    else
    {
        // Start collection
        start_gesture_collection();
    }
}

/**
 * @brief Count CSV files in gesture directory
 */
static int count_gesture_files(void)
{
    DIR *dir;
    struct dirent *entry;
    int file_count = 0;

    dir = opendir("/gesture");
    if (dir == NULL)
    {
        return 0;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        if (strstr(entry->d_name, ".csv") != NULL)
        {
            file_count++;
        }
    }
    closedir(dir);

    return file_count;
}

/**
 * @brief Update sync status display
 */
static void update_sync_status_display(void)
{
    if (!lv_obj_is_valid(ui.sync_status_label))
    {
        return;
    }

    if (app_gesture_data_ctx.is_syncing)
    {
        char status_str[64];
        snprintf(status_str, sizeof(status_str), "同步中: %d/%d",
                 app_gesture_data_ctx.synced_count,
                 app_gesture_data_ctx.sync_file_count);
        lv_label_set_text(ui.sync_status_label, status_str);
        lv_obj_set_style_text_color(ui.sync_status_label, lv_color_make(100, 200, 255), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
        // Display file count when not syncing
        int file_count = count_gesture_files();
        char status_str[64];
        snprintf(status_str, sizeof(status_str), "檔案數: %d", file_count);
        lv_label_set_text(ui.sync_status_label, status_str);
        lv_obj_set_style_text_color(ui.sync_status_label, lv_color_make(200, 200, 200), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

/**
 * @brief Sync all gesture files to phone
 */
static void sync_all_gesture_files(void)
{
    DIR *dir;
    struct dirent *entry;
    int file_count = 0;

    // Check if phone is connected
    if (!SkaiWatchSys.connected_to_phone)
    {
        LOG_E("Phone not connected");
        if (lv_obj_is_valid(ui.sync_status_label))
        {
            lv_label_set_text(ui.sync_status_label, "手機未連接");
            lv_obj_set_style_text_color(ui.sync_status_label, lv_color_make(255, 0, 0), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        return;
    }

    // Open gesture directory
    dir = opendir("/gesture");
    if (dir == NULL)
    {
        LOG_E("Failed to open /gesture directory");
        if (lv_obj_is_valid(ui.sync_status_label))
        {
            lv_label_set_text(ui.sync_status_label, "目錄不存在");
        }
        return;
    }

    // Count .csv files
    while ((entry = readdir(dir)) != NULL)
    {
        if (strstr(entry->d_name, ".csv") != NULL)
        {
            file_count++;
        }
    }
    closedir(dir);

    if (file_count == 0)
    {
        LOG_I("No gesture files to sync");
        if (lv_obj_is_valid(ui.sync_status_label))
        {
            lv_label_set_text(ui.sync_status_label, "沒有檔案");
            lv_obj_set_style_text_color(ui.sync_status_label, lv_color_make(255, 165, 0), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        return;
    }

    // Start syncing
    app_gesture_data_ctx.is_syncing = true;
    app_gesture_data_ctx.sync_file_count = file_count;
    app_gesture_data_ctx.synced_count = 0;

    if (lv_obj_is_valid(ui.sync_status_label))
    {
        lv_obj_set_style_text_color(ui.sync_status_label, lv_color_make(100, 200, 255), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    update_sync_status_display();

    // Sync each file
    dir = opendir("/gesture");
    if (dir != NULL)
    {
        while ((entry = readdir(dir)) != NULL)
        {
            if (strstr(entry->d_name, ".csv") != NULL)
            {
                char file_path[256];
                snprintf(file_path, sizeof(file_path), "/gesture/%s", entry->d_name);

                LOG_I("Syncing file: %s", file_path);
                int ret = bloc_file_system.sync_file(file_path, true); // Delete after sync

                if (ret == 0)
                {
                    app_gesture_data_ctx.synced_count++;
                    update_sync_status_display();
                    LOG_I("File synced: %s (%d/%d)", file_path, app_gesture_data_ctx.synced_count, file_count);
                }
                else
                {
                    LOG_E("Failed to sync file: %s", file_path);
                }

                rt_thread_mdelay(500); // Delay between files
            }
        }
        closedir(dir);
    }

    // Sync complete
    app_gesture_data_ctx.is_syncing = false;

    if (lv_obj_is_valid(ui.sync_status_label))
    {
        char status_str[64];
        snprintf(status_str, sizeof(status_str), "完成: %d/%d",
                 app_gesture_data_ctx.synced_count, file_count);
        lv_label_set_text(ui.sync_status_label, status_str);
        lv_obj_set_style_text_color(ui.sync_status_label, lv_color_make(0, 255, 0), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    LOG_I("Sync complete: %d/%d files", app_gesture_data_ctx.synced_count, file_count);

    // After 2 seconds, restore to show file count
    rt_thread_mdelay(2000);
    update_sync_status_display();
}

/**
 * @brief Sync button event callback
 */
static void sync_button_event_cb(lv_event_t *e)
{
    if (app_gesture_data_ctx.is_syncing)
    {
        LOG_W("Already syncing");
        return;
    }

    sync_all_gesture_files();
}

static void bottom_clear_tap_model_cb(lv_event_t *e)
{
    clear_tap_model_file();
}

static void bottom_clear_release_model_cb(lv_event_t *e)
{
    clear_release_model_file();
}

bool imu_data_collection_error = false;
bool accel_steps_data_collection = false;
bool imu_data_collection = false;
bool imu_6Axis_data_collection = false;
bool imu_raw_data_collection = false;

bool pause_sleep_cause_of_imu_reson(void)
{
    return imu_data_collection || imu_raw_data_collection || accel_steps_data_collection || imu_6Axis_data_collection;
}
bool get_imu_data_collection_status(void)
{
    return imu_data_collection;
}

static void imu_data_collection_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    imu_data_collection = !imu_data_collection;
    lv_obj_set_style_bg_color(obj, imu_data_collection ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
    watch_sys_sync.notify_imu_data_collection(imu_data_collection);
}

static void imu_data_collection_error_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    imu_data_collection_error = !imu_data_collection_error;
    lv_obj_set_style_bg_color(obj, imu_data_collection_error ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void accel_steps_data_collection_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    accel_steps_data_collection = !accel_steps_data_collection;
    lv_obj_set_style_bg_color(obj, accel_steps_data_collection ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void imu_6Axis_data_collection_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    imu_6Axis_data_collection = !imu_6Axis_data_collection;
    lv_obj_set_style_bg_color(obj, imu_6Axis_data_collection ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
}

static bool have_change_tap_hz = false;
static void imu_raw_data_collection_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    imu_raw_data_collection = !imu_raw_data_collection;
    lv_obj_set_style_bg_color(obj, imu_raw_data_collection ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    if (imu_raw_data_collection)
    {
        if (gesture_tap_hz)
        {
            switch_watch_motion_control_mode(true, true);
            have_change_tap_hz = true;
        }
    }
    else
    {
        if (have_change_tap_hz)
        {
            switch_watch_motion_control_mode(true, false);
            have_change_tap_hz = false;
        }
    }
    watch_sys_sync.notify_imu_rawdata_collection(imu_raw_data_collection);
}

static void imu_lock_sw_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    gesture_tap_hz = !gesture_tap_hz;
    lv_obj_set_style_bg_color(obj, gesture_tap_hz ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    if (gesture_tap_hz)
    {
        switch_watch_motion_control_mode(true, false);
    }
    else
    {
        switch_watch_motion_control_mode(false, false);
    }
    
    // Update combo string when gesture_tap_hz changes
    update_combo_string();
}

/**
 * @brief Create the main gesture screen
 */
static lv_obj_t *create_gesture_screen(lv_obj_t *parent)
{
    lv_obj_t *bg_container = lv_obj_create(parent);
    lv_obj_set_size(bg_container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(bg_container, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(bg_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(bg_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Main container
    ui.main_container = lv_obj_create(bg_container);
    lv_obj_set_size(ui.main_container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(ui.main_container, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui.main_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(ui.main_container, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(ui.main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui.main_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(ui.main_container, 4, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Count label
    ui.count_label = lv_label_create(ui.main_container);
    lv_label_set_text(ui.count_label, "Taps --");
    lv_obj_set_style_text_color(ui.count_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui.count_label, LV_EXT_FONT_GET(get_system_font_size(1)), LV_PART_MAIN | LV_STATE_DEFAULT);

    // Gesture circle
    ui.gesture_circle = lv_obj_create(ui.main_container);
    lv_obj_set_size(ui.gesture_circle, 120, 120);
    lv_obj_center(ui.gesture_circle);
    lv_obj_set_style_radius(ui.gesture_circle, 60, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui.gesture_circle, lv_color_make(30, 30, 30), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui.gesture_circle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Reset button
    ui.reset_button = lv_btn_create(ui.main_container);
    lv_obj_set_size(ui.reset_button, 100, 40);
    lv_obj_set_style_bg_color(ui.reset_button, lv_color_make(70, 130, 180), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui.reset_button, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(ui.reset_button, reset_button_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *reset_label = lv_label_create(ui.reset_button);
    lv_label_set_text(reset_label, "Reset");
    lv_obj_set_style_text_color(reset_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(reset_label, LV_ALIGN_CENTER, 0, 0);

    // Combo string label
    ui.combo_label = lv_label_create(ui.main_container);
    lv_label_set_text(ui.combo_label, "male_30_70_170");
    lv_obj_set_style_text_color(ui.combo_label, lv_color_make(100, 200, 255), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui.combo_label, LV_EXT_FONT_GET(get_system_font_size(0)), LV_PART_MAIN | LV_STATE_DEFAULT);

    /* Row: 身高 + 體重 */
    lv_obj_t *info_row1 = lv_obj_create(ui.main_container);
    lv_obj_set_size(info_row1, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(info_row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(info_row1, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(info_row1, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(info_row1, lv_color_make(20, 20, 20), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(info_row1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(info_row1, 8, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* 身高 */
    lv_obj_t *height_container = lv_obj_create(info_row1);
    lv_obj_set_size(height_container, LV_PCT(48), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(height_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(height_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(height_container, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(height_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(height_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *height_label = lv_label_create(height_container);
    lv_label_set_text(height_label, "身高:");
    lv_obj_set_style_text_color(height_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(height_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    static char height_opts[1200];
    char *p = height_opts;
    for (int i = 100; i <= 200; i++) {
        p += snprintf(p, height_opts + sizeof(height_opts) - p, "%d%s", i, i < 200 ? "\n" : "");
    }
    ui.height_dd = lv_dropdown_create(height_container);
    lv_dropdown_set_options_static(ui.height_dd, height_opts);
    lv_dropdown_set_selected(ui.height_dd, 70); // 默認170
    lv_dropdown_set_symbol(ui.height_dd, NULL); // 移除向下箭頭
    lv_obj_set_width(ui.height_dd, 80);
    lv_obj_add_event_cb(ui.height_dd, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* 體重 */
    lv_obj_t *weight_container = lv_obj_create(info_row1);
    lv_obj_set_size(weight_container, LV_PCT(48), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(weight_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(weight_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(weight_container, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(weight_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(weight_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *weight_label = lv_label_create(weight_container);
    lv_label_set_text(weight_label, "體重:");
    lv_obj_set_style_text_color(weight_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(weight_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    static char weight_opts[2000];
    p = weight_opts;
    for (int i = 30; i <= 200; i++) {
        p += snprintf(p, weight_opts + sizeof(weight_opts) - p, "%d%s", i, i < 200 ? "\n" : "");
    }
    ui.weight_dd = lv_dropdown_create(weight_container);
    lv_dropdown_set_options_static(ui.weight_dd, weight_opts);
    lv_dropdown_set_selected(ui.weight_dd, 40); // 默認70
    lv_dropdown_set_symbol(ui.weight_dd, NULL); // 移除向下箭頭
    lv_obj_set_width(ui.weight_dd, 80);
    lv_obj_add_event_cb(ui.weight_dd, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Row: 年齡 + 性別 */
    lv_obj_t *info_row2 = lv_obj_create(ui.main_container);
    lv_obj_set_size(info_row2, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(info_row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(info_row2, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(info_row2, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(info_row2, lv_color_make(20, 20, 20), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(info_row2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(info_row2, 8, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* 年齡 */
    lv_obj_t *age_container = lv_obj_create(info_row2);
    lv_obj_set_size(age_container, LV_PCT(48), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(age_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(age_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(age_container, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(age_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(age_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *age_label = lv_label_create(age_container);
    lv_label_set_text(age_label, "年齡:");
    lv_obj_set_style_text_color(age_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(age_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    static char age_opts[1200];
    p = age_opts;
    for (int i = 5; i <= 110; i++) {
        p += snprintf(p, age_opts + sizeof(age_opts) - p, "%d%s", i, i < 110 ? "\n" : "");
    }
    ui.age_dd = lv_dropdown_create(age_container);
    lv_dropdown_set_options_static(ui.age_dd, age_opts);
    lv_dropdown_set_selected(ui.age_dd, 25); // 默認30
    lv_dropdown_set_symbol(ui.age_dd, NULL); // 移除向下箭頭
    lv_obj_set_width(ui.age_dd, 80);
    lv_obj_add_event_cb(ui.age_dd, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* 性別 */
    lv_obj_t *gender_container = lv_obj_create(info_row2);
    lv_obj_set_size(gender_container, LV_PCT(48), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(gender_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(gender_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(gender_container, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(gender_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(gender_container, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *gender_label = lv_label_create(gender_container);
    lv_label_set_text(gender_label, "性別:");
    lv_obj_set_style_text_color(gender_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(gender_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);

    static const char *gender_opts = "male\nfemale";
    ui.gender_dd = lv_dropdown_create(gender_container);
    lv_dropdown_set_options_static(ui.gender_dd, gender_opts);
    lv_dropdown_set_symbol(ui.gender_dd, NULL); // 移除向下箭頭
    lv_obj_set_width(ui.gender_dd, 80);
    lv_obj_add_event_cb(ui.gender_dd, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    

    /* Row 1: gesture + error data + Tap/Rle */
    lv_obj_t *imu_row1 = lv_obj_create(ui.main_container);
    lv_obj_set_size(imu_row1, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(imu_row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(imu_row1, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(imu_row1, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(imu_row1, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(imu_row1, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(imu_row1, lv_color_make(20, 20, 20), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(imu_row1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(imu_row1, 8, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* gesture */
    lv_obj_t *imu_data_collection_container = lv_btn_create(imu_row1);
    lv_obj_set_size(imu_data_collection_container, LV_PCT(30), 60);
    lv_obj_set_style_bg_color(imu_data_collection_container, imu_data_collection ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(imu_data_collection_container, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(imu_data_collection_container, imu_data_collection_sw_event_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t *imu_data_collection_label = lv_label_create(imu_data_collection_container);
    lv_label_set_text(imu_data_collection_label, "gesture");
    lv_obj_set_style_text_font(imu_data_collection_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(imu_data_collection_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(imu_data_collection_label);

    /* error data */
    lv_obj_t *imu_error_container = lv_btn_create(imu_row1);
    lv_obj_set_size(imu_error_container, LV_PCT(30), 60);
    lv_obj_set_style_bg_color(imu_error_container, imu_data_collection_error ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(imu_error_container, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(imu_error_container, imu_data_collection_error_sw_event_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t *imu_data_collection_error_label = lv_label_create(imu_error_container);
    lv_label_set_text(imu_data_collection_error_label, "error");
    lv_obj_set_style_text_font(imu_data_collection_error_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(imu_data_collection_error_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(imu_data_collection_error_label);

    /* Tap/Rle */
    lv_obj_t *lock_imu_container = lv_btn_create(imu_row1);
    lv_obj_set_size(lock_imu_container, LV_PCT(30), 60);
    lv_obj_set_style_bg_color(lock_imu_container, gesture_tap_hz ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(lock_imu_container, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(lock_imu_container, imu_lock_sw_event_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lock_imu_label = lv_label_create(lock_imu_container);
    lv_label_set_text(lock_imu_label, "Tap/Rle");
    lv_obj_set_style_text_font(lock_imu_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(lock_imu_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(lock_imu_label);

    /* Row 2: 6Axis data + RAW data + STEP data */
    lv_obj_t *imu_row2 = lv_obj_create(ui.main_container);
    lv_obj_set_size(imu_row2, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(imu_row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(imu_row2, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(imu_row2, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(imu_row2, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(imu_row2, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(imu_row2, lv_color_make(20, 20, 20), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(imu_row2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(imu_row2, 8, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* 6Axis data */
    lv_obj_t *imu_6axis_container = lv_btn_create(imu_row2);
    lv_obj_set_size(imu_6axis_container, LV_PCT(30), 60);
    lv_obj_set_style_bg_color(imu_6axis_container, imu_6Axis_data_collection ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(imu_6axis_container, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(imu_6axis_container, imu_6Axis_data_collection_sw_event_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t *imu_6Axis_data_collection_label = lv_label_create(imu_6axis_container);
    lv_label_set_text(imu_6Axis_data_collection_label, "6Axis");
    lv_obj_set_style_text_font(imu_6Axis_data_collection_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(imu_6Axis_data_collection_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(imu_6Axis_data_collection_label);

    /* RAW data */
    lv_obj_t *imu_raw_container = lv_btn_create(imu_row2);
    lv_obj_set_size(imu_raw_container, LV_PCT(30), 60);
    lv_obj_set_style_bg_color(imu_raw_container, imu_raw_data_collection ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(imu_raw_container, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(imu_raw_container, imu_raw_data_collection_sw_event_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t *imu_raw_data_collection_label = lv_label_create(imu_raw_container);
    lv_label_set_text(imu_raw_data_collection_label, "RAW");
    lv_obj_set_style_text_font(imu_raw_data_collection_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(imu_raw_data_collection_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(imu_raw_data_collection_label);

    /* STEP data */
    lv_obj_t *step_container = lv_btn_create(imu_row2);
    lv_obj_set_size(step_container, LV_PCT(30), 60);
    lv_obj_set_style_bg_color(step_container, accel_steps_data_collection ? lv_color_make(0, 150, 0) : lv_color_make(60, 60, 60), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(step_container, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(step_container, accel_steps_data_collection_sw_event_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t *accel_steps_data_collection_label = lv_label_create(step_container);
    lv_label_set_text(accel_steps_data_collection_label, "STEP");
    lv_obj_set_style_text_font(accel_steps_data_collection_label, LV_EXT_FONT_GET(get_system_font_size(0)), 0);
    lv_obj_set_style_text_color(accel_steps_data_collection_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(accel_steps_data_collection_label);

    /* Gesture collection row */
    lv_obj_t *collect_row = lv_obj_create(ui.main_container);
    lv_obj_set_size(collect_row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(collect_row, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(collect_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(collect_row, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(collect_row, lv_color_make(20, 20, 20), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(collect_row, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(collect_row, 8, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* Gesture count label */
    ui.gesture_count_label = lv_label_create(collect_row);
    lv_label_set_text(ui.gesture_count_label, "手勢數: 0/20");
    lv_obj_set_style_text_color(ui.gesture_count_label, lv_color_make(100, 200, 255), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui.gesture_count_label, LV_EXT_FONT_GET(get_system_font_size(0)), LV_PART_MAIN | LV_STATE_DEFAULT);

    /* Collect button */
    ui.collect_button = lv_btn_create(collect_row);
    lv_obj_set_size(ui.collect_button, LV_PCT(80), 50);
    lv_obj_set_style_bg_color(ui.collect_button, lv_color_make(0, 120, 215), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui.collect_button, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(ui.collect_button, collect_button_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *collect_label = lv_label_create(ui.collect_button);
    lv_label_set_text(collect_label, "開始收集");
    lv_obj_set_style_text_color(collect_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(collect_label, LV_EXT_FONT_GET(get_system_font_size(0)), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(collect_label);

    /* Sync row */
    lv_obj_t *sync_row = lv_obj_create(ui.main_container);
    lv_obj_set_size(sync_row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(sync_row, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(sync_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(sync_row, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(sync_row, lv_color_make(20, 20, 20), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(sync_row, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(sync_row, 8, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* Sync status label */
    ui.sync_status_label = lv_label_create(sync_row);
    lv_label_set_text(ui.sync_status_label, "");
    lv_obj_set_style_text_color(ui.sync_status_label, lv_color_make(100, 200, 255), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui.sync_status_label, LV_EXT_FONT_GET(get_system_font_size(0)), LV_PART_MAIN | LV_STATE_DEFAULT);

    /* Sync button */
    ui.sync_button = lv_btn_create(sync_row);
    lv_obj_set_size(ui.sync_button, LV_PCT(80), 50);
    lv_obj_set_style_bg_color(ui.sync_button, lv_color_make(0, 180, 120), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui.sync_button, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(ui.sync_button, sync_button_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *sync_label = lv_label_create(ui.sync_button);
    lv_label_set_text(sync_label, "同步到手機");
    lv_obj_set_style_text_color(sync_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(sync_label, LV_EXT_FONT_GET(get_system_font_size(0)), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(sync_label);

    lv_obj_t *top_margin = lv_obj_create(bg_container);
    lv_obj_set_size(top_margin, 100, 100);
    lv_obj_set_style_bg_opa(top_margin, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align_to(top_margin, ui.main_container, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_obj_t *bottom_clear_tap_model = lv_btn_create(ui.main_container);
    lv_obj_set_size(bottom_clear_tap_model, LV_PCT(80), 50);
    lv_obj_set_style_bg_color(bottom_clear_tap_model, lv_color_make(200, 50, 50), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(bottom_clear_tap_model, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(bottom_clear_tap_model, bottom_clear_tap_model_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *clear_tap_model_label = lv_label_create(bottom_clear_tap_model);
    lv_label_set_text(clear_tap_model_label, "清除Tap模型");
    lv_obj_set_style_text_color(clear_tap_model_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(clear_tap_model_label, LV_EXT_FONT_GET(get_system_font_size(0)), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(clear_tap_model_label);

    lv_obj_t *bottom_clear_release_model = lv_btn_create(ui.main_container);
    lv_obj_set_size(bottom_clear_release_model, LV_PCT(80), 50);
    lv_obj_set_style_bg_color(bottom_clear_release_model, lv_color_make(200, 50, 50), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(bottom_clear_release_model, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(bottom_clear_release_model, bottom_clear_release_model_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *clear_release_model_label = lv_label_create(bottom_clear_release_model);
    lv_label_set_text(clear_release_model_label, "清除Release模型");
    lv_obj_set_style_text_color(clear_release_model_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(clear_release_model_label, LV_EXT_FONT_GET(get_system_font_size(0)), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(clear_release_model_label);

    return bg_container;
}

static void on_start(void)
{
    // Initialize UI context
    memset(&ui, 0, sizeof(gesture_ui_t));
    create_gesture_screen(lv_scr_act());

    // Initialize data
    app_gesture_data_ctx.tap_count = 0;
    app_gesture_data_ctx.is_pressed = false;
    app_gesture_data_ctx.height = 170;  // Default height
    app_gesture_data_ctx.weight = 70;   // Default weight
    app_gesture_data_ctx.age = 30;      // Default age
    app_gesture_data_ctx.gender = 0;    // Default male
    app_gesture_data_ctx.is_recording = false;
    app_gesture_data_ctx.gesture_record_count = 0;
    app_gesture_data_ctx.csv_fd = -1;

    // Update initial display
    update_gesture_display();
    update_count_display();
    update_combo_string();
    update_sync_status_display();

    LOG_D("Gesture app started");
}

static void on_pause(void)
{
    // No timer to pause since we're using direct event handling
}

static void on_resume(void)
{
    // No timer to resume since we're using direct event handling

#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_tap_indicator = handle_tap_indicator;
#endif
}

static void on_stop(void)
{
    // Stop any ongoing collection
    if (app_gesture_data_ctx.is_recording)
    {
        stop_gesture_collection(false);
    }

    // Reset UI handler
#ifdef BSP_USING_UI_HANDLER
    lvgl_msg_handler.handle_tap_indicator = NULL;
#endif

    // Reset app context
    memset(&ui, 0, sizeof(gesture_ui_t));
    memset(&app_gesture_data_ctx, 0, sizeof(app_gesture_data_ctx_t));
    app_gesture_data_ctx.csv_fd = -1;
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
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
    gui_app_regist_msg_handler(APP_ID_GESTURE, msg_handler);

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(gesture), IMG_LOGO, APP_ID_GESTURE, app_main);

#endif // APP_ID_GESTURE