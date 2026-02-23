/**
 ******************************************************************************
 * @file   bloc_filesystem.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "communicate_protocol.h"
#include "communicate_parse.h"
#include "communicate_parse_notify.h"
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "app_mainmenu.h"
#include "app_speech.h"
#include "os_adaptor.h"
#include "dfs_file.h"
#include "dfs_posix.h"
#include <unistd.h>
#ifdef BSP_USING_BLOC
    #include "bloc_control.h"
    #include "bloc_v2t.h"
    #include "bloc_skaiwalk.h"
    #include "bloc_peripheral.h"
    #include "bloc_filesystem.h"
#endif
#ifdef BSP_USING_UI_HANDLER
    #include "ui_handler.h"
#endif
#include "ui_helper.h"
#include "ui_img_helper.h"

// 動態媒體圖片路徑定義
char MEDIA_IMG[40] = "/assets/images/media_img.bin";
char MEDIA_HEADER_IMG[40] = "/assets/images/media_header_img.bin";
#include "gesture_model_loader.h"

#define DBG_TAG "bloc.filesystem"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* Configuration constants */
#define FILE_SYNC_THREAD_STACK_SIZE 4096
#define FILE_SYNC_THREAD_PRIORITY 25
#define FILE_SYNC_THREAD_TIMESLICE 20
#define FILE_SYNC_MQ_SIZE 10
#define FILE_SYNC_MSG_SIZE sizeof(file_system_msg_data_t)
#define FILE_SYNC_BUFFER_SIZE 500
#define FILE_SYNC_DELAY_MS 6
#define FILE_RECEIVE_TIMEOUT_MS 5000 /* 5 seconds timeout for file receive */

/* Message types for the file system queue */
typedef enum
{
    SYNC_FILE_MSG = 0,
    DELETE_FILE_MSG,
    START_RECEIVE_FILE_MSG,
    RECEIVE_FILE_DATA_MSG,
    END_RECEIVE_FILE_MSG,
    CANCEL_RECEIVE_FILE_MSG
} file_system_msg_t;

/* File receive data for message passing */
typedef struct
{
    char file_path[FILE_PATH_MAX_LEN];
    uint32_t total_size;
    uint16_t data_length;
    uint8_t data[FILE_SYNC_BUFFER_SIZE];
} file_receive_msg_data_t;

/* Message data structure for file operations */
typedef struct file_system_msg_data
{
    file_system_msg_t msg_type;
    union
    {
        /* For SYNC_FILE_MSG and DELETE_FILE_MSG */
        struct
        {
            char *file_path;
            bool delete_after_sync;
        } sync;

        /* For file receive messages */
        file_receive_msg_data_t receive;
    } data;
} file_system_msg_data_t;

static file_receive_state_t g_file_receive = {0};

file_receive_state_t *get_file_receive_state(void)
{
    return &g_file_receive;
}

/* Global variables */
static sync_progress_t sync_progress = {0};
static rt_mq_t file_system_mq = RT_NULL;
BLocFileSystem bloc_file_system;

/* Function prototypes */
static int sync_file_to_remote_client(const char *file_path);
static void delete_file(char *file_path);
static void file_system_entry(void *parameter);
static int file_system_task_init(void);
static int bloc_file_system_register(void);
static void send_file_compare_result(uint8_t result);

/**
 * @brief Get the current sync progress
 * @return Pointer to the sync_progress structure
 */
sync_progress_t *get_sync_progress(void)
{
    return &sync_progress;
}

/**
 * @brief Get the current file path being synced
 * @return Pointer to the file path string
 */
char *get_sync_in_file_path(void)
{
    return sync_progress.sync_in_file_path;
}

/**
 * @brief Synchronize a file to the remote client
 * @param file_path Path to the file to be synchronized
 * @return 1 on success, 0 on failure
 */
static int sync_file_to_remote_client(const char *file_path)
{
    struct stat file_stat;
    L1SendData data;
    int fd, read_bytes;
    char buf[FILE_SYNC_BUFFER_SIZE];
    lvgl_msg_t ui_msg;

    // Get file size first
    if (stat(file_path, &file_stat) == 0)
    {
        sync_progress.total_bytes = file_stat.st_size;
    }
    else
    {
        LOG_E("Failed to get file stats for %s", file_path);
        sync_progress.total_bytes = 0;
        return 0;
    }

    // Open and read the file
    fd = open(file_path, O_RDONLY | O_BINARY);
    if (fd < 0)
    {
        LOG_E("Failed to open file %s", file_path);
        return 0;
    }

    // Send start sync event with file size
    data.event = L1SEND_START_SYNC_FILE;
    data.res.id = sync_progress.total_bytes;
    L1_send_event(data);
    rt_thread_mdelay(50);

    // Send file data in chunks
    while ((read_bytes = read(fd, buf, sizeof(buf))) > 0)
    {
        data.event = L1SEND_SYNC_FILE;
        data.res.file_buffer.length = read_bytes;
        data.res.file_buffer.data = (uint8_t *)buf;
        int res = L1_send_event(data);
        if (res <= 0)
        {
            LOG_E("Failed to send file data for %s", file_path);
            goto exit_sync;
        }

        // Update progress
        sync_progress.transferred_bytes += read_bytes;
        sync_progress.percent_complete =
            (sync_progress.total_bytes > 0) ? (sync_progress.transferred_bytes *
                                               100 / sync_progress.total_bytes)
                                            : 0;

        // Send progress update to UI
        ui_msg.type = LVGL_MSG_TYPE_SYNC_PROGRESS;
        ui_msg.data.action = sync_progress.percent_complete;
        lvgl_send_msg(ui_msg);

        rt_thread_mdelay(FILE_SYNC_DELAY_MS);
    }

exit_sync:
    close(fd);
    rt_thread_mdelay(50);
    data.event = L1SEND_END_SYNC_FILE;
    L1_send_event(data);

    return 1;
}

/**
 * @brief Delete a file from the file system
 * @param file_path Path to the file to be deleted
 */
static void delete_file(char *file_path)
{
    if (file_path == NULL)
    {
        LOG_E("Cannot delete file: path is NULL");
        return;
    }

    if (remove(file_path) == 0)
    {
        LOG_D("File %s deleted successfully", file_path);
    }
    else
    {
        LOG_E("Failed to delete file %s", file_path);
    }
}

/**
 * @brief Send file compare result to phone
 * @param result 0 = file exists and matches (skip update), 1 = file doesn't
 * exist or different (proceed with update)
 */
static void send_file_compare_result(uint8_t result)
{
    L1SendData data;
    data.event = L1SEND_FILE_COMPARE_RESULT;
    data.res.status = result;
    L1_send_event(data);
    LOG_I("Sent file compare result: %d", result);
}

extern char *get_media_title(void);
static void notify_media_img(char *path)
{
    lvgl_msg_t msg;
    msg.type = LVGL_MSG_TYPE_MEDIA_IMG;
    msg.data.media_data.img_path = path;
    msg.data.media_data.title = get_media_title();
    lvgl_send_msg(msg);
}

static char prev_media_img_path[40];
void received_file_handler(const char *path)
{
    rt_thread_mdelay(500); // Small delay to ensure file is closed
    /* Check if it's a gesture model file */
    if (strstr(path, "gesture_tap.tflite") != NULL)
    {
        unload_tap_model();
        /* Reinitialize interpreters */
        extern void init_gesture_recognition_model(void);
        if (strstr(path, "gesture_tap.tflite") != NULL)
        {
            init_gesture_recognition_model();
        }
    }
    else if (strstr(path, "gesture_release.tflite") != NULL)
    {
        unload_release_model();
        /* Reinitialize interpreters */
        extern void init_gesture_recognition_release_model(void);
        if (strstr(path, "gesture_release.tflite") != NULL)
        {
            init_gesture_recognition_release_model();
        }
    }
    // else if (strncmp(path, "/JW_wf", 6) == 0)
    // {
    // 	/* Handle watch face image file */
    // 	LOG_I("Received watch face image: %s", path);
    // 	/* TODO: Add watch face image processing logic here */
    // 	lv_img_cache_invalidate_src(path);
    // 	// ui_show_hint_toast("New watch face image received");
    // }
    else if (strstr(path, "media_img") != NULL)
    {
        // lv_img_cache_invalidate_src(path);
        strncpy(prev_media_img_path, MEDIA_IMG,
                sizeof(prev_media_img_path) - 1);
        prev_media_img_path[sizeof(prev_media_img_path) - 1] = '\0';
        strncpy(MEDIA_IMG, path, sizeof(MEDIA_IMG) - 1);
        MEDIA_IMG[sizeof(MEDIA_IMG) - 1] = '\0';
		LOG_I("Invalidate image cache for src: %s", MEDIA_IMG);
        lv_img_cache_invalidate_src(prev_media_img_path);
		lv_img_cache_invalidate_src(MEDIA_IMG);
		rt_thread_mdelay(200);
        notify_media_img(prev_media_img_path);
        LOG_I("Received media image file: %s,rm:%s", MEDIA_IMG,
              prev_media_img_path);
        // rt_thread_mdelay(1000);
		// remove(prev_media_img_path);
    }
    else if (strstr(path, "media_header_img") != NULL)
    {
        // lv_img_cache_invalidate_src(path);
        strncpy(prev_media_img_path, MEDIA_HEADER_IMG,
                sizeof(prev_media_img_path) - 1);
        prev_media_img_path[sizeof(prev_media_img_path) - 1] = '\0';
        strncpy(MEDIA_HEADER_IMG, path, sizeof(MEDIA_HEADER_IMG) - 1);
        MEDIA_HEADER_IMG[sizeof(MEDIA_HEADER_IMG) - 1] = '\0';
		LOG_I("Invalidate image cache for src: %s", MEDIA_HEADER_IMG);
        lv_img_cache_invalidate_src(prev_media_img_path);
		lv_img_cache_invalidate_src(MEDIA_HEADER_IMG);
		rt_thread_mdelay(200);
        notify_media_img(prev_media_img_path);
        LOG_I("Received media image file: %s,rm:%s", MEDIA_HEADER_IMG,
              prev_media_img_path);
        // rt_thread_mdelay(1000);
		// remove(prev_media_img_path);
    }
    else
    {
        LOG_W("Received file is not recognized: %s", path);
    }
}

/**
 * @brief Thread entry function for file system operations
 * @param parameter Thread parameters (unused)
 */
static void file_system_entry(void *parameter)
{
    file_system_msg_data_t msg_data;
    lvgl_msg_t ui_msg;
    rt_err_t result;

    while (1)
    {
        // Wait for messages in the queue with timeout to check for receive
        // timeout
        result = rt_mq_recv(file_system_mq, &msg_data, sizeof(msg_data),
                            RT_WAITING_FOREVER);

        if (result == RT_EOK)
        {
            switch (msg_data.msg_type)
            {
            case SYNC_FILE_MSG:
            {
                // Initialize sync progress
                sync_progress.sync_status = true;
                sync_progress.transferred_bytes = 0;
                sync_progress.percent_complete = 0;

                // Copy file path to sync progress structure
                if (msg_data.data.sync.file_path)
                {
                    strncpy(sync_progress.sync_in_file_path,
                            msg_data.data.sync.file_path,
                            FILE_PATH_MAX_LEN - 1);
                    sync_progress.sync_in_file_path[FILE_PATH_MAX_LEN - 1] =
                        '\0';
                }

                // Notify UI about sync start
                ui_msg.type = LVGL_MSG_TYPE_SYNC_STATUS;
                ui_msg.data.sync_state = sync_progress.sync_status;
                lvgl_send_msg(ui_msg);

                // Perform synchronization
                skaiwatch_ble_set_performance(
                    true); // Enable performance mode for BLE
                sync_file_to_remote_client(msg_data.data.sync.file_path);
                skaiwatch_ble_set_performance(
                    false); // Disable performance mode after sync
                // Delete file if requested
                if (msg_data.data.sync.delete_after_sync)
                {
                    delete_file(msg_data.data.sync.file_path);
                }

                // Update sync status and notify UI
                sync_progress.sync_status = false;
                ui_msg.type = LVGL_MSG_TYPE_SYNC_STATUS;
                ui_msg.data.sync_state = sync_progress.sync_status;
                lvgl_send_msg(ui_msg);
                break;
            }

            case DELETE_FILE_MSG:
            {
                delete_file(msg_data.data.sync.file_path);
                break;
            }

            case START_RECEIVE_FILE_MSG:
            {
                const char *file_path = msg_data.data.receive.file_path;
                uint32_t total_size = msg_data.data.receive.total_size;

                if (g_file_receive.is_active)
                {
                    LOG_E("File receive already in progress");
                    break;
                }

                /* Check if file already exists and compare size */
                struct stat file_stat;
                if (stat(file_path, &file_stat) == 0)
                {
                    /* File exists, compare size */
                    if ((uint32_t)file_stat.st_size == total_size)
                    {
                        /* File exists and size matches, skip update */
                        LOG_I("File already exists with matching size: %s (%d "
                              "bytes)",
                              file_path, total_size);
                        send_file_compare_result(0); /* 0 = skip update */
                        break;
                    }
                    else
                    {
                        /* File exists but size differs, proceed with update */
                        LOG_I("File exists but size differs: %s (existing: %d, "
                              "new: %d)",
                              file_path, file_stat.st_size, total_size);
                        send_file_compare_result(
                            1); /* 1 = proceed with update */
                        if (is_ble_dfu_thread_running())
                        {
                            handle_download_progress_update(0);
                        }
                    }
                }
                else
                {
                    /* File doesn't exist, proceed with update */
                    LOG_I("File doesn't exist, proceed with update: %s",
                          file_path);
                    send_file_compare_result(1); /* 1 = proceed with update */
                    if (is_ble_dfu_thread_running())
                    {
                        handle_download_progress_update(0);
                    }
                }

                /* Check if file path already has .temp extension */
                char temp_file_path[FILE_PATH_MAX_LEN];
                char *temp_ext = strstr(file_path, ".temp");
                if (temp_ext != NULL && *(temp_ext + 5) == '\0')
                {
                    /* Already has .temp extension, use as is */
                    rt_strncpy(temp_file_path, file_path,
                               FILE_PATH_MAX_LEN - 1);
                    temp_file_path[FILE_PATH_MAX_LEN - 1] = '\0';
                }
                else
                {
                    /* Add .temp extension */
                    rt_snprintf(temp_file_path, FILE_PATH_MAX_LEN, "%s.temp",
                                file_path);
                }

                /* Extract directory path and create all parent dirs if needed */
                char dir_path[FILE_PATH_MAX_LEN];
                rt_strncpy(dir_path, temp_file_path, FILE_PATH_MAX_LEN);
                char *last_slash = strrchr(dir_path, '/');
                if (last_slash)
                {
                    *last_slash = '\0';
                    /* Recursively create parent directories */
                    for (char *p = dir_path + 1; *p; p++)
                    {
                        if (*p == '/')
                        {
                            *p = '\0';
                            mkdir(dir_path, 0777);
                            *p = '/';
                        }
                    }
                    mkdir(dir_path, 0777);
                }

                /* Open file for writing */
                g_file_receive.fd =
                    open(temp_file_path,
                         O_WRONLY | O_CREAT | O_TRUNC | O_BINARY, 0666);
                if (g_file_receive.fd < 0)
                {
                    LOG_E("Failed to open file for writing: %s",
                          temp_file_path);
                    break;
                }

                /* Initialize receive state with .temp path */
                rt_strncpy(g_file_receive.file_path, temp_file_path,
                           FILE_PATH_MAX_LEN);
                g_file_receive.total_size = total_size;
                g_file_receive.received_size = 0;
                g_file_receive.is_active = true;
                g_file_receive.last_receive_tick = rt_tick_get();

                LOG_I("Started receiving file: %s (%d bytes)", temp_file_path,
                      total_size);
                skaiwatch_ble_set_performance(
                    true); // Enable performance mode for BLE
                break;
            }

            case RECEIVE_FILE_DATA_MSG:
            {
                if (!g_file_receive.is_active)
                {
                    LOG_E("No active file receive");
                    break;
                }

                uint16_t length = msg_data.data.receive.data_length;
                const uint8_t *data = msg_data.data.receive.data;

                /* Write data to file */
                size_t written = write(g_file_receive.fd, data, length);
                if (written != length)
                {
                    LOG_E("Failed to write data: %d/%d bytes", written, length);
                    close(g_file_receive.fd);
                    g_file_receive.is_active = false;
                    remove(g_file_receive.file_path);
                    break;
                }

                g_file_receive.received_size += written;
                g_file_receive.last_receive_tick = rt_tick_get();

                uint8_t progress = g_file_receive.total_size > 0
                                       ? (g_file_receive.received_size * 100 /
                                          g_file_receive.total_size)
                                       : 0;
                if (is_ble_dfu_thread_running())
                {
                    handle_download_progress_update(progress);
                }

                LOG_D("Received %d/%d bytes (%d%%)",
                      g_file_receive.received_size, g_file_receive.total_size,
                      progress);
                break;
            }

            case END_RECEIVE_FILE_MSG:
            {
                if (!g_file_receive.is_active)
                {
                    LOG_E("No active file receive");
                    break;
                }

                /* Close file */
                close(g_file_receive.fd);

                /* Check if all data received */
                if (g_file_receive.received_size != g_file_receive.total_size)
                {
                    LOG_E("Incomplete file: received %d/%d bytes",
                          g_file_receive.received_size,
                          g_file_receive.total_size);
                    remove(g_file_receive.file_path);
                    g_file_receive.is_active = false;
                    break;
                }

                LOG_I("File receive completed: %s (%d bytes)",
                      g_file_receive.file_path, g_file_receive.received_size);

                /* Check if file has .temp extension and rename to final name */
                char *temp_ext = strstr(g_file_receive.file_path, ".temp");
                if (temp_ext != NULL && *(temp_ext + 5) == '\0')
                {
                    /* This is a .temp file, rename to final name */
                    char final_path[FILE_PATH_MAX_LEN];
                    size_t base_len = temp_ext - g_file_receive.file_path;
                    rt_strncpy(final_path, g_file_receive.file_path, base_len);
                    final_path[base_len] = '\0';

                    /* Remove existing final file if it exists */
                    if (remove(final_path))
                    {
                        LOG_W("No existing file to remove: %s", final_path);
                    }
                    else
                    {
                        LOG_I("Removed existing file: %s", final_path);
                    }
                    // remove(final_path);

                    /* Rename temp file to final file */
                    if (rename(g_file_receive.file_path, final_path) == 0)
                    {
                        LOG_I("Renamed %s to %s", g_file_receive.file_path,
                              final_path);
                        /* Call file handler with final path */
                        received_file_handler(final_path);
                    }
                    else
                    {
                        LOG_E("Failed to rename %s to %s",
                              g_file_receive.file_path, final_path);
                        /* Still call handler with temp path as fallback */
                        received_file_handler(g_file_receive.file_path);
                    }
                }
                else
                {
                    /* Not a .temp file, call handler directly */
                    received_file_handler(g_file_receive.file_path);
                }

                /* Reset state */
                skaiwatch_ble_set_performance(
                    false); // Disable performance mode after BLE transfer
                g_file_receive.is_active = false;
                break;
            }

            case CANCEL_RECEIVE_FILE_MSG:
            {
                if (g_file_receive.is_active)
                {
                    close(g_file_receive.fd);
                    remove(g_file_receive.file_path);
                    g_file_receive.is_active = false;
                    LOG_I("Cancel file receive for: %s",
                          g_file_receive.file_path);
                }
                break;
            }

            default:
                LOG_E("Unknown message type: %d", msg_data.msg_type);
                break;
            }

            // Free allocated memory for file path (only for SYNC and DELETE
            // messages)
            if ((msg_data.msg_type == SYNC_FILE_MSG ||
                 msg_data.msg_type == DELETE_FILE_MSG) &&
                msg_data.data.sync.file_path)
            {
                rt_free(msg_data.data.sync.file_path);
                msg_data.data.sync.file_path = NULL;
            }
        }
    }
}

/**
 * @brief Initialize the file system task and message queue
 * @return RT_EOK on success, -RT_ERROR on failure
 */
static int file_system_task_init(void)
{
    rt_thread_t thread;

    // Create message queue
    file_system_mq = rt_mq_create("file_system_mq", FILE_SYNC_MSG_SIZE,
                                  FILE_SYNC_MQ_SIZE, RT_IPC_FLAG_FIFO);

    if (file_system_mq == RT_NULL)
    {
        LOG_E("Failed to create file system message queue");
        return -RT_ERROR;
    }

    // Create thread for file system operations
    thread = rt_thread_create(
        "file_system", file_system_entry, RT_NULL, FILE_SYNC_THREAD_STACK_SIZE,
        FILE_SYNC_THREAD_PRIORITY, FILE_SYNC_THREAD_TIMESLICE);

    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        LOG_E("Failed to create file system thread");
        rt_mq_delete(file_system_mq);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief Synchronize a file to the remote client
 * @param file_path Path to the file to be synchronized
 * @param delete_after_sync Whether to delete the file after synchronization
 * @return RT_EOK on success, -RT_ERROR on failure
 */
int bloc_system_sync(const char *file_path, bool delete_after_sync)
{
    if (file_path == NULL)
    {
        LOG_E("Invalid file path: NULL pointer");
        return -RT_ERROR;
    }

    file_system_msg_data_t msg_data;
    size_t path_len;

    // Prepare message data
    msg_data.msg_type = SYNC_FILE_MSG;
    msg_data.data.sync.delete_after_sync = delete_after_sync;

    // Allocate and copy file path
    path_len = strlen(file_path) + 1;
    msg_data.data.sync.file_path = (char *)rt_malloc(path_len);
    if (msg_data.data.sync.file_path == NULL)
    {
        LOG_E("Failed to allocate memory for file path");
        return -RT_ERROR;
    }

    rt_strncpy(msg_data.data.sync.file_path, file_path, path_len);

    // Send message to queue
    if (rt_mq_send(file_system_mq, (void *)&msg_data, sizeof(msg_data)) !=
        RT_EOK)
    {
        LOG_E("Failed to send file sync message to queue");
        rt_free(msg_data.data.sync.file_path);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief Delete a file asynchronously
 * @param file_path Path to the file to be deleted
 * @return RT_EOK on success, -RT_ERROR on failure
 */
int bloc_system_delete_file(const char *file_path)
{
    file_system_msg_data_t msg_data;
    size_t path_len;

    if (file_path == NULL)
    {
        LOG_E("Invalid file path: NULL pointer");
        return -RT_ERROR;
    }

    // Prepare message data
    msg_data.msg_type = DELETE_FILE_MSG;

    // Allocate and copy file path
    path_len = strlen(file_path) + 1;
    msg_data.data.sync.file_path = (char *)rt_malloc(path_len);
    if (msg_data.data.sync.file_path == NULL)
    {
        LOG_E("Failed to allocate memory for file path");
        return -RT_ERROR;
    }

    rt_strncpy(msg_data.data.sync.file_path, file_path, path_len);

    // Send message to queue
    if (rt_mq_send(file_system_mq, (void *)&msg_data, sizeof(msg_data)) !=
        RT_EOK)
    {
        LOG_E("Failed to send file delete message to queue");
        rt_free(msg_data.data.sync.file_path);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief Register file system functions
 * @return 0 on success
 */
static int bloc_file_system_register(void)
{
    bloc_file_system.sync_file = bloc_system_sync;
    bloc_file_system.delete_file = bloc_system_delete_file;
    return 0;
}

/* Initialize file system task and register functions */
INIT_APP_EXPORT(file_system_task_init);
INIT_APP_EXPORT(bloc_file_system_register);

/********************************************************************************
 * File Receive Functions
 ********************************************************************************/
/**
 * @brief Start receiving a file from remote (post message to queue)
 * @param file_path Destination file path
 * @param total_size Total file size in bytes
 * @return 0 on success, negative on error
 */
int bloc_start_receive_file(const char *file_path, uint32_t total_size)
{
    if (!file_path || total_size == 0)
    {
        LOG_E("Invalid parameters");
        return -RT_ERROR;
    }

    file_system_msg_data_t msg_data;
    msg_data.msg_type = START_RECEIVE_FILE_MSG;
    rt_strncpy(msg_data.data.receive.file_path, file_path,
               FILE_PATH_MAX_LEN - 1);
    msg_data.data.receive.file_path[FILE_PATH_MAX_LEN - 1] = '\0';
    msg_data.data.receive.total_size = total_size;

    if (rt_mq_send(file_system_mq, (void *)&msg_data, sizeof(msg_data)) !=
        RT_EOK)
    {
        LOG_E("Failed to send start receive file message to queue");
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief Receive file data chunk (post message to queue)
 * @param data Data buffer
 * @param length Data length
 * @return 0 on success, negative on error
 */
int bloc_receive_file_data(const uint8_t *data, uint16_t length)
{
    if (!data || length == 0)
    {
        LOG_E("Invalid data");
        return -RT_ERROR;
    }

    if (length > FILE_SYNC_BUFFER_SIZE)
    {
        LOG_E("Data too large: %d bytes (max: %d)", length,
              FILE_SYNC_BUFFER_SIZE);
        return -RT_ERROR;
    }

    file_system_msg_data_t msg_data;
    msg_data.msg_type = RECEIVE_FILE_DATA_MSG;
    msg_data.data.receive.data_length = length;
    rt_memcpy(msg_data.data.receive.data, data, length);

    if (rt_mq_send(file_system_mq, (void *)&msg_data, sizeof(msg_data)) !=
        RT_EOK)
    {
        LOG_E("Failed to send receive file data message to queue");
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief End file receive and finalize (post message to queue)
 * @return 0 on success, negative on error
 */
int bloc_end_receive_file(void)
{
    file_system_msg_data_t msg_data;
    msg_data.msg_type = END_RECEIVE_FILE_MSG;

    if (rt_mq_send(file_system_mq, (void *)&msg_data, sizeof(msg_data)) !=
        RT_EOK)
    {
        LOG_E("Failed to send end receive file message to queue");
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief Cancel file receive (post message to queue)
 * @return 0 on success
 */
int bloc_cancel_receive_file(void)
{
    file_system_msg_data_t msg_data;
    msg_data.msg_type = CANCEL_RECEIVE_FILE_MSG;

    if (rt_mq_send(file_system_mq, (void *)&msg_data, sizeof(msg_data)) !=
        RT_EOK)
    {
        LOG_E("Failed to send cancel receive file message to queue");
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief Get current file receive progress
 * @return Pointer to receive state (or NULL if not active)
 */
const file_receive_state_t *bloc_get_receive_progress(void)
{
    return g_file_receive.is_active ? &g_file_receive : NULL;
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
