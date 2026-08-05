/**
 ******************************************************************************
 * @file   app_photo.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "dfs_posix.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "gesture_handler.h"
#include "custom_trans_anim.h"
#include "common_widget.h"
#include "bloc_filesystem.h"
#include "ui_img_helper.h"
#include "ui_handler.h"
#define DBG_TAG "app.photo"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef APP_ID_PHOTO
    /*********************
     *      DEFINES
     *********************/
    #define PHOTO_THUMB_SIZE 218
    #define PHOTO_LIST_PAD 10
    #define MAX_PHOTO_PATH_LEN 64
    #define PHOTO_ZOOM_SCALE 128 /* 256 = 100%, 128 = 50%, 64 = 25% */

typedef struct
{
    lv_obj_t *main_window;
    lv_obj_t *photo_list;
    bool multi_select_mode;
    lv_obj_t **selected_imgs;
    int selected_count;
    int selected_capacity;
} app_photo_t;

/* Global variables */
static app_photo_t *p_app_photo = NULL;

// 動態管理選取清單
static void add_selected_img(lv_obj_t *img_container)
{
    if (!p_app_photo)
        return;
    if (!p_app_photo->selected_imgs)
    {
        p_app_photo->selected_capacity = 8;
        p_app_photo->selected_imgs =
            lv_mem_alloc(sizeof(lv_obj_t *) * p_app_photo->selected_capacity);
        p_app_photo->selected_count = 0;
    }
    // 檢查是否已存在
    for (int i = 0; i < p_app_photo->selected_count; ++i)
    {
        if (p_app_photo->selected_imgs[i] == img_container)
            return;
    }
    if (p_app_photo->selected_count >= p_app_photo->selected_capacity)
    {
        p_app_photo->selected_capacity *= 2;
        p_app_photo->selected_imgs =
            lv_mem_realloc(p_app_photo->selected_imgs,
                           sizeof(lv_obj_t *) * p_app_photo->selected_capacity);
    }
    p_app_photo->selected_imgs[p_app_photo->selected_count++] = img_container;
}
static void remove_selected_img(lv_obj_t *img_container)
{
    if (!p_app_photo || !p_app_photo->selected_imgs)
        return;
    for (int i = 0; i < p_app_photo->selected_count; ++i)
    {
        if (p_app_photo->selected_imgs[i] == img_container)
        {
            for (int j = i; j < p_app_photo->selected_count - 1; ++j)
                p_app_photo->selected_imgs[j] =
                    p_app_photo->selected_imgs[j + 1];
            p_app_photo->selected_count--;
            break;
        }
    }
}
static void clear_selected_imgs()
{
    if (p_app_photo && p_app_photo->selected_imgs)
    {
        lv_mem_free(p_app_photo->selected_imgs);
        p_app_photo->selected_imgs = NULL;
        p_app_photo->selected_count = 0;
        p_app_photo->selected_capacity = 0;
    }
}

/**
 * @brief Create photo folder if it doesn't exist
 */
static void create_photo_folder(void)
{
    if (access("/photo", 0))
    {
        if (mkdir("/photo", 0x777) == 0)
        {
            LOG_I("Photo folder created successfully");
        }
        else
        {
            LOG_E("Failed to create photo folder");
        }
    }
    else
    {
        LOG_I("Photo folder already exists");
    }
}

/**
 * @brief Check if file is an image based on extension
 *
 * @param filename File name to check
 * @return true if file is an image, false otherwise
 */
static bool is_image_file(const char *filename)
{
    const char *ext = strrchr(filename, '.');
    if (ext == NULL)
    {
        return false;
    }
#ifdef _MSC_VER
#define strcasecmp _stricmp
#endif
    /* Support common image formats */
    if (strcasecmp(ext, ".png") == 0 || strcasecmp(ext, ".jpg") == 0 ||
        strcasecmp(ext, ".jpeg") == 0 || strcasecmp(ext, ".bin") == 0 ||
        strcasecmp(ext, ".sjpg") == 0)
    {
        return true;
    }
    return false;
}

static void delete_photo(const char *file_path)
{
    LOG_I("Deleting photo: %s", file_path);
    if (remove(file_path) == 0)
    {
        bloc_file_system.delete_file(file_path);
        LOG_I("Deleted photo: %s", file_path);
    }
    else
    {
        LOG_E("Failed to delete photo: %s", file_path);
    }
}

static void load_photo_list(lv_obj_t *parent);
static void btn_del_event_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *sheet = lv_obj_get_parent(btn);
    // lv_obj_t *mask = lv_obj_get_parent(sheet);
    // 多選模式下刪除所有選取
    if (p_app_photo && p_app_photo->multi_select_mode &&
        p_app_photo->selected_imgs)
    {
        for (int i = 0; i < p_app_photo->selected_count; ++i)
        {
            lv_obj_t *img_container = p_app_photo->selected_imgs[i];
            const char *file_path =
                (const char *)lv_obj_get_user_data(img_container);
            LOG_I("Photo deleteds: %s", file_path);
            if (file_path)
                delete_photo(file_path);
        }
        clear_selected_imgs();
        p_app_photo->multi_select_mode = false;
    }
    else
    {
        // 單選模式
        const char *file_path = (const char *)lv_event_get_user_data(e);
        LOG_I("Photo deleted: %s", file_path);
        if (file_path)
            delete_photo(file_path);
    }
    lv_obj_del(sheet); // 關閉彈窗
    // 刷新照片列表
    if (p_app_photo && p_app_photo->photo_list)
    {
        lv_obj_clean(p_app_photo->photo_list);
        load_photo_list(p_app_photo->photo_list);
    }
}

static void btn_cancel_event_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *sheet = lv_obj_get_parent(btn);
    // 離開多選模式
    if (p_app_photo && p_app_photo->multi_select_mode)
    {
        p_app_photo->multi_select_mode = false;
        clear_selected_imgs();
    }
    lv_obj_del(sheet); // 關閉彈窗
}

static void btn_close_photo_container_cb(lv_event_t *e)
{
    lv_obj_t *photo_container_btn = lv_event_get_target(e);
    lv_obj_t *photo_container = lv_obj_get_parent(photo_container_btn);
    lv_obj_del(photo_container); // 關閉大圖
}
/**
 * @brief Load and display all photos from /photo folder
 *
 * @param parent Parent object to add photos to
 */
// 彈出底部操作欄（刪除/取消）
static void show_photo_action_sheet(lv_obj_t *img_container,
                                    const char *file_path)
{
    // 進入多選模式
    if (p_app_photo)
    {
        p_app_photo->multi_select_mode = true;
        clear_selected_imgs();
        add_selected_img(img_container);
    }

    // 建立底部操作欄
    lv_obj_t *sheet = lv_obj_create(lv_scr_act());
    lv_obj_set_size(sheet, LV_HOR_RES, 120);
    lv_obj_align(sheet, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_radius(sheet, 0, 0);
    lv_obj_set_style_bg_color(sheet, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(sheet, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(sheet, 0, 0);
    lv_obj_clear_flag(sheet, LV_OBJ_FLAG_SCROLLABLE);

    // 刪除按鈕
    lv_obj_t *btn_del = lv_btn_create(sheet);
    lv_obj_set_size(btn_del, 100, 50);
    lv_obj_align(btn_del, LV_ALIGN_TOP_MID, 60, 0);
    lv_obj_t *label_del = lv_label_create(btn_del);
    lv_label_set_text(label_del, LV_EXT_STR_GET_BY_KEY(delete_selected, "Delete Selected"));
    lv_obj_center(label_del);
    lv_obj_set_style_bg_color(btn_del, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_text_color(label_del, lv_color_white(), 0);

    // 取消按鈕
    lv_obj_t *btn_cancel = lv_btn_create(sheet);
    lv_obj_set_size(btn_cancel, 100, 50);
    lv_obj_align(btn_cancel, LV_ALIGN_TOP_MID, -60, 0);
    lv_obj_t *label_cancel = lv_label_create(btn_cancel);
    lv_label_set_text(label_cancel, LV_EXT_STR_GET_BY_KEY(cancel, "Cancel"));
    lv_obj_center(label_cancel);

    // 刪除事件，無需傳 file_path
    lv_obj_add_event_cb(btn_del, btn_del_event_cb, LV_EVENT_CLICKED, NULL);
    // 取消事件
    lv_obj_add_event_cb(btn_cancel, btn_cancel_event_cb, LV_EVENT_CLICKED,
                        NULL);
}

// 點擊或長按事件：點擊顯示大圖，長按顯示操作欄
static bool event_cb_registered = false;

// 列表滾動時重置 event_cb_registered，防止誤觸發點擊/長按
static void photo_list_scroll_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_SCROLL)
    {
        event_cb_registered = false;
    }
}
    #define LONG_PRESS_TIME_MS 600
static void photo_container_event_cb(lv_event_t *e)
{
    static uint32_t press_time = 0;
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *container = lv_event_get_target(e);
    const char *file_path = (const char *)lv_event_get_user_data(e);
    if (code == LV_EVENT_PRESSED)
    {
        event_cb_registered = true;
        press_time = lv_tick_get();
    }
    else if (code == LV_EVENT_RELEASED)
    {
        uint32_t release_time = lv_tick_get();
        if (release_time - press_time < LONG_PRESS_TIME_MS &&
            event_cb_registered)
        {
            // 多選模式下切換選取狀態
            if (p_app_photo && p_app_photo->multi_select_mode)
            {
                // 反選
                bool already = false;
                if (p_app_photo->selected_imgs)
                {
                    for (int i = 0; i < p_app_photo->selected_count; ++i)
                    {
                        if (p_app_photo->selected_imgs[i] == container)
                            already = true;
                    }
                }
                if (already)
                {
                    remove_selected_img(container);
                    lv_obj_set_style_border_color(container, lv_color_black(),
                                                  0);
                }
                else
                {
                    add_selected_img(container);
                    lv_obj_set_style_border_color(
                        container, lv_palette_main(LV_PALETTE_BLUE), 0);
                }
            }
            else
            {
                // 點擊小於1.5秒顯示大圖
                event_cb_registered = false;
                lv_obj_t *photo_container = lv_obj_create(lv_scr_act());
                lv_obj_set_size(photo_container, LV_HOR_RES, LV_VER_RES);
                lv_obj_set_style_bg_opa(photo_container, LV_OPA_100, 0);
                lv_obj_set_style_bg_color(photo_container, lv_color_black(), 0);
                lv_obj_clear_flag(photo_container, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_t *img = lv_img_create(photo_container);
                lv_img_set_src(img, file_path);
                lv_obj_center(img);
                lv_obj_t *photo_container_btn = lv_obj_create(photo_container);
                lv_obj_set_size(photo_container_btn, LV_HOR_RES, LV_VER_RES);
                lv_obj_set_style_bg_opa(photo_container_btn, LV_OPA_0, 0);
                lv_obj_set_style_bg_color(photo_container_btn, lv_color_black(),
                                          0);
                lv_obj_clear_flag(photo_container_btn, LV_OBJ_FLAG_SCROLLABLE);
                // 點擊遮罩關閉大圖
                lv_obj_add_event_cb(photo_container_btn,
                                    btn_close_photo_container_cb,
                                    LV_EVENT_CLICKED, NULL);
            }
        }
    }
    else if (code == LV_EVENT_PRESSING && event_cb_registered)
    {
        uint32_t pressing_time = lv_tick_get();
        // 長按超過1.5秒進入多選模式
        if (pressing_time - press_time > LONG_PRESS_TIME_MS)
        {
            event_cb_registered = false;
            show_photo_action_sheet(container, file_path);
        }
    }
}

static void load_photo_list(lv_obj_t *parent)
{
    DIR *dir;
    struct dirent *entry;
    char file_path[MAX_PHOTO_PATH_LEN];

    /* Open the /photo directory */
    dir = opendir("/photo");
    if (dir == NULL)
    {
        LOG_W("Cannot open /photo directory");
        lv_obj_t *label = lv_label_create(parent);
        lv_obj_set_style_text_font(label,
                                   LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_label_set_text(label, "No photos found");
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_center(label);
        return;
    }

    uint8_t photo_count = 0;
    /* 綁定滾動事件，防止滑動時觸發點擊/長按 */
    lv_obj_add_event_cb(parent, photo_list_scroll_event_cb, LV_EVENT_SCROLL,
                        NULL);
    /* Add top spacer */
    uint16_t top_spacer_height = PHOTO_LIST_PAD * 3;
    lv_obj_t *top_spacer = lv_obj_create(parent);
    lv_obj_set_size(top_spacer, PHOTO_THUMB_SIZE * 2 + PHOTO_LIST_PAD * 3,
                    top_spacer_height);
    lv_obj_set_pos(top_spacer, 0, 0);
    lv_obj_set_style_bg_opa(top_spacer, LV_OPA_0, 0);
    lv_obj_set_style_border_width(top_spacer, 0, 0);
    lv_obj_set_style_pad_all(top_spacer, 0, 0);
    lv_obj_set_style_radius(top_spacer, 0, 0);
    lv_obj_clear_flag(top_spacer, LV_OBJ_FLAG_SCROLLABLE);

    /* Read all files in the directory */
    while ((entry = readdir(dir)) != NULL)
    {
        /* Check if the entry is a regular file and is an image */
        if (entry->d_type == DT_REG && is_image_file(entry->d_name))
        {
            LOG_D("Found photo: %s", entry->d_name);

            /* Build full file path */
            snprintf(file_path, sizeof(file_path), "/photo/%s", entry->d_name);

            /* Get image original size first */
            lv_img_header_t header;
            if (lv_img_decoder_get_info(file_path, &header) != LV_RES_OK)
            {
                LOG_W("Cannot get image info: %s", file_path);
                continue;
            }

            /* Calculate zoom: scale so that the shorter side becomes
             * PHOTO_THUMB_SIZE */
            uint16_t zoom = 256; /* Default: no zoom (256 = 100%) */
            if (header.w < header.h && header.w > PHOTO_THUMB_SIZE)
            {
                zoom = (256 * PHOTO_THUMB_SIZE) / header.w;
            }
            else if (header.h <= header.w && header.h > PHOTO_THUMB_SIZE)
            {
                zoom = (256 * PHOTO_THUMB_SIZE) / header.h;
            }

            /* Create container with fixed PHOTO_THUMB_SIZE, two per row */
            uint16_t x =
                PHOTO_LIST_PAD +
                (photo_count % 2) * (PHOTO_THUMB_SIZE + PHOTO_LIST_PAD);
            uint16_t y =
                top_spacer_height + PHOTO_LIST_PAD +
                (photo_count / 2) * (PHOTO_THUMB_SIZE + PHOTO_LIST_PAD);
            lv_obj_t *img_container = lv_obj_create(parent);
            lv_obj_set_size(img_container, PHOTO_THUMB_SIZE, PHOTO_THUMB_SIZE);
            lv_obj_set_pos(img_container, x, y);
            lv_obj_set_style_radius(img_container, 0, 0);
            lv_obj_set_style_bg_color(img_container, lv_color_black(), 0);
            lv_obj_set_style_border_width(img_container, 3, 0);
            lv_obj_set_style_border_color(img_container, lv_color_black(), 0);
            lv_obj_set_style_pad_all(img_container, 0, 0);

            lv_obj_clear_flag(img_container, LV_OBJ_FLAG_SCROLLABLE);
            // 綁定 file_path 到 user_data
            lv_obj_set_user_data(img_container, strdup(file_path));
            // 綁定點擊與長按事件
            lv_obj_add_event_cb(img_container, photo_container_event_cb,
                                LV_EVENT_ALL, strdup(file_path));

            /* Create image inside container */
            lv_obj_t *img = lv_img_create(img_container);
            lv_img_set_src(img, file_path);
            if (zoom != 256)
            {
                lv_img_set_zoom(img, zoom);
            }
            lv_obj_center(img);
            LOG_I("img loaded: %s,pos: %d,%d", file_path, PHOTO_LIST_PAD,
                  photo_count * (PHOTO_THUMB_SIZE + PHOTO_LIST_PAD) +
                      PHOTO_LIST_PAD);
            photo_count++;
        }
    }

    /* Add bottom spacer */
    lv_obj_t *bottom_spacer = lv_obj_create(parent);
    lv_obj_set_size(bottom_spacer, PHOTO_THUMB_SIZE * 2 + PHOTO_LIST_PAD * 3,
                    PHOTO_LIST_PAD * 3);
    lv_obj_set_pos(bottom_spacer, 0,
                   top_spacer_height + PHOTO_LIST_PAD +
                       ((photo_count + 1) / 2) *
                           (PHOTO_THUMB_SIZE + PHOTO_LIST_PAD));
    lv_obj_set_style_bg_opa(bottom_spacer, LV_OPA_0, 0);
    lv_obj_set_style_border_width(bottom_spacer, 0, 0);
    lv_obj_set_style_pad_all(bottom_spacer, 0, 0);
    lv_obj_set_style_radius(bottom_spacer, 0, 0);
    lv_obj_clear_flag(bottom_spacer, LV_OBJ_FLAG_SCROLLABLE);

    /* Close the directory */
    closedir(dir);

    if (photo_count == 0)
    {
        lv_obj_t *label = lv_label_create(parent);
        lv_label_set_text(label, "No photos found");
        lv_obj_set_style_text_font(label,
                                   LV_EXT_FONT_GET(get_system_font_size(0)), 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_center(label);
    }
    else
    {
        LOG_I("Loaded %d photos", photo_count);
    }
}

/**
 * @brief Rebuild the photo list from /photo. Must run on the LVGL thread
 * (creates/deletes lv_obj_t). No-op if the album isn't currently open.
 * Exposed so photo_app_refresh_list() can force a re-scan after a
 * phone-pushed photo finishes writing.
 */
void photo_app_do_refresh_list(void)
{
    if (p_app_photo && p_app_photo->photo_list)
    {
        lv_obj_clean(p_app_photo->photo_list);
        load_photo_list(p_app_photo->photo_list);
    }
}

/**
 * @brief Request a photo-list refresh from any thread. bloc_filesystem's
 * received_file_handler calls this after a phone-pushed photo finishes
 * writing; that runs on the BLE parse thread (KE_EVT2, 4KB stack), where
 * rebuilding the list is too stack-heavy and not thread-safe — defer to the
 * LVGL thread, same pattern as skai_device_ui_refresh().
 */
void photo_app_refresh_list(void)
{
    if (!is_on_lvgl_thread())
    {
        lvgl_msg_t msg = {.type = LVGL_MSG_TYPE_REFRESH_PHOTO_LIST};
        lvgl_send_msg(msg);
        return;
    }
    photo_app_do_refresh_list();
}

/**
 * @brief Creates the photo screen with scrollable photo list
 *
 * @param scr Parent screen object
 * @return lv_obj_t* Created screen object
 */
lv_obj_t *create_photo_screen(lv_obj_t *scr)
{
    lv_obj_t *bg = lv_obj_create(scr);
    lv_obj_set_size(bg, LV_HOR_RES, LV_VER_RES);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);

    /* Create photo folder if not exists */
    create_photo_folder();

    /* Create scrollable list container - vertical layout */
    lv_obj_t *photo_list = lv_obj_create(bg);
    lv_obj_set_size(photo_list, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(photo_list, lv_color_black(), 0);
    lv_obj_add_flag(photo_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(photo_list, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(photo_list, LV_DIR_VER);

    /* Store reference */
    if (p_app_photo)
    {
        p_app_photo->photo_list = photo_list;
    }

    /* Load photos from folder */
    load_photo_list(photo_list);

    return bg;
}

char *GAUS_DEFAULT_PICTURE = "/assets/gaus_images/gaus_default_picture.bin";

/**
 * @brief Initialize the app on start
 */
static lv_obj_t *on_start(lv_obj_t *scr)
{
    RT_ASSERT(NULL == p_app_photo);
    p_app_photo = (app_photo_t *)lv_mem_alloc(sizeof(app_photo_t));
    if (!p_app_photo)
    {
        LOG_E("Failed to allocate memory for photo app");
        return NULL;
    }

    memset(p_app_photo, 0, sizeof(app_photo_t));
    p_app_photo->main_window = create_photo_screen(scr);

    cust_trans_anim_config(CUST_ANIM_TYPE_1, NULL);
    return p_app_photo->main_window;
}

static void on_resume(void)
{
}

static void on_pause(void)
{
}

static void on_stop(void)
{
    if (p_app_photo)
    {
        if (p_app_photo->main_window)
        {
            lv_obj_del(p_app_photo->main_window);
            p_app_photo->main_window = NULL;
        }

        lv_mem_free(p_app_photo);
        p_app_photo = NULL;
    }

    LOG_I("Photo app stopped and resources cleaned up");
}

/**
 * @brief Message handler for app lifecycle events
 */
static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
    {
        /* app_run 直接開啟不經 Main 狀態機，左緣右滑返回 bar 仍隱藏，這裡補開 */
        extern void display_gesture_detect_objs(uint32_t idx, bool display);
        display_gesture_detect_objs(0, true);
        on_start(lv_scr_act());
        break;
    }

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
 * @brief Main entry point for the app
 */
static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID_PHOTO, msg_handler);
    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(photo), IMG_PHOTO, APP_ID_PHOTO, app_main, 1);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/