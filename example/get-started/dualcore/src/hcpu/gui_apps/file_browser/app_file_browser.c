/**
 ******************************************************************************
 * @file   app_file_browser.c
 * @author Skaiwalk software development team
 * @brief  File browser app - browse all files on the device filesystem
 ******************************************************************************
 */
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <dfs_posix.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "common_widget.h"
#include "app_mainmenu.h"
#include "gesture_handler.h"
#include "ui_helper.h"
#include "watch_global_data.h"
#ifdef BSP_USING_BLOC_FILESYSTEM
#include "bloc_filesystem.h"
#endif
#ifdef BSP_USING_UI_HANDLER
#include "ui_handler.h"
#include "ui_img_helper.h"
#endif

#define DBG_TAG "app.filebrowser"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_FILE_BROWSER

/*********************
 *      DEFINES
 *********************/
#define MAX_PATH_LEN        256
#define MAX_ENTRIES         128
#define FILE_LIST_ITEM_H     50

/* Circular display layout constants (466x466) */
#define CIRCULAR_PAD_TOP     55
#define CIRCULAR_PAD_BOTTOM  55
#define CIRCULAR_PAD_SIDE    30
#define LIST_AREA_W          (LV_HOR_RES - CIRCULAR_PAD_SIDE * 2)
#define LIST_AREA_H          (LV_VER_RES - CIRCULAR_PAD_TOP - CIRCULAR_PAD_BOTTOM - 75)
#define SEND_BTN_W           140
#define SEND_BTN_H           32

/*********************
 *   STATIC VARIABLES
 *********************/
static char current_path[MAX_PATH_LEN] = "/";
static char selected_file_path[MAX_PATH_LEN] = {0};
static lv_obj_t *path_label = NULL;
static lv_obj_t *file_list = NULL;
static lv_obj_t *info_label = NULL;
static lv_obj_t *main_container = NULL;
static lv_obj_t *send_btn = NULL;

/*********************
 *   STATIC PROTOTYPES
 *********************/
static void refresh_file_list(void);
static void file_item_event_cb(lv_event_t *e);
static void go_up_event_cb(lv_event_t *e);
static void send_btn_event_cb(lv_event_t *e);
static bool is_sendable_file(const char *name);
static void hide_send_btn(void);

/*********************
 *   HELPER FUNCTIONS
 *********************/

/**
 * @brief Check if path is a directory
 */
static bool is_directory(const char *path)
{
    struct stat st;
    if (stat(path, &st) == 0)
    {
        return S_ISDIR(st.st_mode);
    }
    return false;
}

/**
 * @brief Format file size to human readable string
 */
static void format_size(uint32_t size, char *buf, int buf_len)
{
    if (size < 1024)
    {
        rt_snprintf(buf, buf_len, "%lu B", (unsigned long)size);
    }
    else if (size < 1024 * 1024)
    {
        rt_snprintf(buf, buf_len, "%lu KB", (unsigned long)(size / 1024));
    }
    else
    {
        rt_snprintf(buf, buf_len, "%lu MB", (unsigned long)(size / (1024 * 1024)));
    }
}

/**
 * @brief Navigate into a subdirectory
 */
static void navigate_to(const char *dir_name)
{
    if (rt_strcmp(current_path, "/") == 0)
    {
        rt_snprintf(current_path, MAX_PATH_LEN, "/%s", dir_name);
    }
    else
    {
        int len = rt_strlen(current_path);
        rt_snprintf(current_path + len, MAX_PATH_LEN - len, "/%s", dir_name);
    }
    refresh_file_list();
}

/**
 * @brief Navigate to parent directory
 */
static void navigate_up(void)
{
    if (rt_strcmp(current_path, "/") == 0)
    {
        return; /* Already at root */
    }

    /* Find last '/' and truncate */
    char *last_slash = strrchr(current_path, '/');
    if (last_slash == current_path)
    {
        /* Parent is root */
        current_path[1] = '\0';
    }
    else if (last_slash != NULL)
    {
        *last_slash = '\0';
    }
    refresh_file_list();
}

/**
 * @brief Check whether a file name is sendable (log/txt/bin extension)
 */
static bool is_sendable_file(const char *name)
{
    const char *dot = strrchr(name, '.');
    if (dot == NULL)
    {
        return false;
    }
    return (strcasecmp(dot, ".log") == 0 ||
            strcasecmp(dot, ".txt") == 0 ||
            strcasecmp(dot, ".bin") == 0);
}

/**
 * @brief Hide the send button and clear selection
 */
static void hide_send_btn(void)
{
    if (send_btn)
    {
        lv_obj_add_flag(send_btn, LV_OBJ_FLAG_HIDDEN);
    }
    selected_file_path[0] = '\0';
}

/**
 * @brief Event callback for the "Send to Phone" button
 */
static void send_btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    {
        return;
    }

    if (selected_file_path[0] == '\0')
    {
        return;
    }

#ifdef BSP_USING_BLOC_FILESYSTEM
    const char *base = strrchr(selected_file_path, '/');
    base = base ? base + 1 : selected_file_path;

    if (!SkaiWatchSys.connected_to_phone)
    {
        ui_show_hint_toast("Phone not connected");
        return;
    }

    if (bloc_file_system.sync_file == NULL)
    {
        ui_show_hint_toast("Send not available");
        return;
    }

    int ret = bloc_file_system.sync_file(selected_file_path, false);
    if (ret == RT_EOK)
    {
        ui_show_hint_toast("Sending %s", base);
        LOG_I("Start sync file: %s", selected_file_path);
    }
    else
    {
        ui_show_hint_toast("Send failed");
        LOG_E("Failed to start sync: %s (ret=%d)", selected_file_path, ret);
    }
#else
    ui_show_hint_toast("Bloc filesystem disabled");
#endif
}

/**
 * @brief Event callback for file/folder items
 */
static void file_item_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED)
    {
        return;
    }

    lv_obj_t *btn = lv_event_get_target(e);
    const char *name = (const char *)lv_event_get_user_data(e);
    if (name == NULL)
    {
        return;
    }

    /* Build full path */
    char full_path[MAX_PATH_LEN];
    if (rt_strcmp(current_path, "/") == 0)
    {
        rt_snprintf(full_path, MAX_PATH_LEN, "/%s", name);
    }
    else
    {
        rt_snprintf(full_path, MAX_PATH_LEN, "%s/%s", current_path, name);
    }

    if (is_directory(full_path))
    {
        hide_send_btn();
        navigate_to(name);
    }
    else
    {
        /* Show file info */
        struct stat st;
        if (stat(full_path, &st) == 0)
        {
            char size_str[32];
            format_size(st.st_size, size_str, sizeof(size_str));
            lv_label_set_text_fmt(info_label, "%s\nSize: %s", name, size_str);
        }

        /* Save selection and toggle Send button for sendable extensions */
        rt_strncpy(selected_file_path, full_path, MAX_PATH_LEN - 1);
        selected_file_path[MAX_PATH_LEN - 1] = '\0';

        if (send_btn)
        {
            if (is_sendable_file(name))
            {
                lv_obj_clear_flag(send_btn, LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                lv_obj_add_flag(send_btn, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

/**
 * @brief Event callback for ".." go up item
 */
static void go_up_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED)
    {
        navigate_up();
    }
}

/**
 * @brief Screen gesture handler (swipe right to go back)
 */
static void screen_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_GESTURE)
    {
        lv_dir_t g = lv_indev_get_gesture_dir(lv_indev_get_act());
        if (g == LV_DIR_RIGHT)
        {
            if (rt_strcmp(current_path, "/") == 0)
            {
                gui_app_self_exit();
            }
            else
            {
                navigate_up();
            }
        }
    }
}

/* Static buffer for storing entry names (avoid dynamic allocation per item) */
static char entry_names[MAX_ENTRIES][64];
static int entry_count = 0;

/**
 * @brief Refresh the file list for current_path
 */
static void refresh_file_list(void)
{
    DIR *dir;
    struct dirent *entry;
    struct stat st;
    char full_path[MAX_PATH_LEN];

    /* Update path label */
    lv_label_set_text(path_label, current_path);

    /* Clear info label */
    lv_label_set_text(info_label, "");

    /* Clear existing list items */
    lv_obj_clean(file_list);
    entry_count = 0;

    /* Navigating to a new folder invalidates any previous selection */
    hide_send_btn();

    /* Add ".." go-up item when not at root */
    if (rt_strcmp(current_path, "/") != 0)
    {
        lv_obj_t *up_btn = lv_obj_create(file_list);
        lv_obj_set_size(up_btn, LV_PCT(100), FILE_LIST_ITEM_H);
        lv_obj_set_style_pad_all(up_btn, 5, 0);
        lv_obj_set_style_bg_color(up_btn, lv_color_hex(0x2A2A3E), 0);
        lv_obj_set_style_bg_opa(up_btn, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(up_btn, 0, 0);
        lv_obj_clear_flag(up_btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(up_btn, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t *up_icon = lv_label_create(up_btn);
        lv_obj_set_style_text_color(up_icon, lv_palette_main(LV_PALETTE_AMBER), 0);
        lv_label_set_text(up_icon, LV_SYMBOL_LEFT);
        lv_obj_align(up_icon, LV_ALIGN_LEFT_MID, 0, 0);

        lv_obj_t *up_label = lv_label_create(up_btn);
        lv_obj_set_style_text_color(up_label, lv_color_white(), 0);
        lv_label_set_text(up_label, ".. (Back)");
        lv_obj_align(up_label, LV_ALIGN_LEFT_MID, 25, 0);

        lv_obj_add_event_cb(up_btn, go_up_event_cb, LV_EVENT_CLICKED, NULL);
    }

    dir = opendir(current_path);
    if (dir == NULL)
    {
        LOG_E("Failed to open directory: %s", current_path);
        lv_label_set_text(info_label, "Cannot open directory");
        return;
    }

    int dir_count = 0;
    int file_count = 0;

    while ((entry = readdir(dir)) != NULL && entry_count < MAX_ENTRIES)
    {
        /* Skip . and .. */
        if (rt_strcmp(entry->d_name, ".") == 0 || rt_strcmp(entry->d_name, "..") == 0)
        {
            continue;
        }

        /* Store entry name */
        rt_strncpy(entry_names[entry_count], entry->d_name, 63);
        entry_names[entry_count][63] = '\0';

        /* Build full path for stat */
        if (rt_strcmp(current_path, "/") == 0)
        {
            rt_snprintf(full_path, MAX_PATH_LEN, "/%s", entry->d_name);
        }
        else
        {
            rt_snprintf(full_path, MAX_PATH_LEN, "%s/%s", current_path, entry->d_name);
        }

        bool is_dir = false;
        uint32_t file_size = 0;
        if (stat(full_path, &st) == 0)
        {
            is_dir = S_ISDIR(st.st_mode);
            file_size = st.st_size;
        }
        else
        {
            /* If stat fails, check d_type if available */
            is_dir = (entry->d_type == DT_DIR);
        }

        if (is_dir)
        {
            dir_count++;
        }
        else
        {
            file_count++;
        }

        /* Create list button */
        lv_obj_t *btn = lv_obj_create(file_list);
        lv_obj_set_size(btn, LV_PCT(100), FILE_LIST_ITEM_H);
        lv_obj_set_style_pad_all(btn, 5, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1A1A2E), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_border_width(btn, 1, LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, lv_palette_main(LV_PALETTE_BLUE), LV_STATE_PRESSED);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);

        /* Icon/prefix label */
        lv_obj_t *icon_label = lv_label_create(btn);
        lv_obj_set_style_text_color(icon_label, is_dir ? lv_palette_main(LV_PALETTE_AMBER) : lv_palette_main(LV_PALETTE_LIGHT_BLUE), 0);
        lv_label_set_text(icon_label, is_dir ? LV_SYMBOL_DIRECTORY : LV_SYMBOL_FILE);
        lv_obj_align(icon_label, LV_ALIGN_LEFT_MID, 0, 0);

        /* File name label */
        lv_obj_t *name_label = lv_label_create(btn);
        lv_obj_set_style_text_color(name_label, lv_color_white(), 0);
        lv_label_set_long_mode(name_label, LV_LABEL_LONG_DOT);
        lv_obj_set_width(name_label, LIST_AREA_W - 100);
        lv_label_set_text(name_label, entry->d_name);
        lv_obj_align(name_label, LV_ALIGN_LEFT_MID, 25, 0);

        /* Size label (for files only) */
        if (!is_dir)
        {
            char size_str[32];
            format_size(file_size, size_str, sizeof(size_str));
            lv_obj_t *size_label = lv_label_create(btn);
            lv_obj_set_style_text_color(size_label, lv_color_hex(0x888888), 0);
            lv_label_set_text(size_label, size_str);
            lv_obj_align(size_label, LV_ALIGN_RIGHT_MID, 0, 0);
        }

        lv_obj_add_event_cb(btn, file_item_event_cb, LV_EVENT_CLICKED,
                            (void *)entry_names[entry_count]);

        entry_count++;
    }

    closedir(dir);

    /* Update info with summary */
    lv_label_set_text_fmt(info_label, "%d dirs, %d files", dir_count, file_count);
    LOG_I("Path: %s - %d dirs, %d files", current_path, dir_count, file_count);
}

/**
 * @brief Create the file browser UI (designed for 466x466 circular display)
 *
 * Layout for circular screen:
 *   - Top area (~55px): path label, centered to avoid clipped corners
 *   - Middle area: file list with horizontal padding (30px each side)
 *   - Bottom area (~55px): info label, centered
 */
static lv_obj_t *on_start(lv_obj_t *parent)
{
    /* Screen gesture handler */
    lv_obj_add_event_cb(parent, screen_event_cb, LV_EVENT_GESTURE, NULL);

    /* Main container */
    main_container = lv_obj_create(parent);
    lv_obj_set_size(main_container, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(main_container, lv_color_hex(0x0F0F1A), 0);
    lv_obj_set_style_bg_opa(main_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_pad_all(main_container, 0, 0);
    lv_obj_clear_flag(main_container, LV_OBJ_FLAG_SCROLLABLE);

    /* Path label - centered near top, away from corners */
    path_label = lv_label_create(main_container);
    lv_obj_set_style_text_color(path_label, lv_color_white(), 0);
    lv_label_set_long_mode(path_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(path_label, LIST_AREA_W);
    lv_obj_set_style_text_align(path_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(path_label, "/");
    lv_obj_align(path_label, LV_ALIGN_TOP_MID, 0, CIRCULAR_PAD_TOP);

    /* File list (scrollable) - centered with side padding */
    file_list = lv_obj_create(main_container);
    lv_obj_set_size(file_list, LIST_AREA_W, LIST_AREA_H);
    lv_obj_align(file_list, LV_ALIGN_TOP_MID, 0, CIRCULAR_PAD_TOP + 35);
    lv_obj_set_style_bg_color(file_list, lv_color_hex(0x0F0F1A), 0);
    lv_obj_set_style_bg_opa(file_list, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(file_list, 0, 0);
    lv_obj_set_style_pad_all(file_list, 2, 0);
    lv_obj_set_flex_flow(file_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(file_list, 2, 0);
    lv_obj_add_event_cb(file_list, screen_event_cb, LV_EVENT_GESTURE, NULL);

    /* Bottom info label - centered, away from bottom edge */
    info_label = lv_label_create(main_container);
    lv_obj_set_style_text_color(info_label, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_align(info_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(info_label, LV_ALIGN_BOTTOM_MID, 0, -CIRCULAR_PAD_BOTTOM);
    lv_label_set_text(info_label, "");

    /* "Send to Phone" button - shown only when a .log/.txt/.bin file is selected */
    send_btn = lv_btn_create(main_container);
    lv_obj_set_size(send_btn, SEND_BTN_W, SEND_BTN_H);
    lv_obj_align(send_btn, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_obj_set_style_bg_color(send_btn, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_bg_color(send_btn, lv_palette_darken(LV_PALETTE_BLUE, 2),
                              LV_STATE_PRESSED);
    lv_obj_set_style_radius(send_btn, SEND_BTN_H / 2, 0);
    lv_obj_set_style_border_width(send_btn, 0, 0);
    lv_obj_t *send_label = lv_label_create(send_btn);
    lv_obj_set_style_text_color(send_label, lv_color_white(), 0);
    lv_label_set_text(send_label, "Send to Phone");
    lv_obj_center(send_label);
    lv_obj_add_event_cb(send_btn, send_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(send_btn, LV_OBJ_FLAG_HIDDEN);

    /* Start from root */
    rt_strncpy(current_path, "/", MAX_PATH_LEN);
    refresh_file_list();

    return main_container;
}

static void on_resume(void)
{
    /* Refresh file list on resume in case files changed */
    refresh_file_list();
}

static void on_pause(void)
{
    /* Nothing to do */
}

static void on_stop(void)
{
    if (main_container)
    {
        lv_obj_del(main_container);
        main_container = NULL;
    }
    path_label = NULL;
    file_list = NULL;
    info_label = NULL;
    send_btn = NULL;
    entry_count = 0;
    selected_file_path[0] = '\0';

    /* Reset to root for next launch */
    rt_strncpy(current_path, "/", MAX_PATH_LEN);
}

/**
 * @brief Message handler for app lifecycle events
 */
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

/**
 * @brief Main entry point for the file browser app
 */
static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_FILE_BROWSER, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(file_browser), IMG_LOGO, APP_ID_FILE_BROWSER, app_main);
#endif /* APP_ID_FILE_BROWSER */
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
