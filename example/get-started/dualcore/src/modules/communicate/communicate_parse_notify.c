/*********************************************************************************************************
 *               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     communicate_parse_notify.c
 * @brief
 * @details
 * @author
 * @date
 * @version  v0.1
 *********************************************************************************************************
 */
#include <rtthread.h>
#include <cJSON.h>
#include "communicate_parse.h"
#include "communicate_protocol.h"
#include "communicate_task.h"
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "string.h"
#include "gui_app_fwk.h"
#include "ui_helper.h"
#include "ui_handler.h"
#include "bloc_v2t.h"
#include "bloc_control.h"
#include "app_mainmenu.h"
#include "bloc_calendar.h"
#include "bloc_weather.h"
#include "bloc_notification.h"
#include "bloc_skaiwalk.h"
#include "bloc_flash.h"
#include "bloc_filesystem.h"
#include "gesture_model_loader.h"
#include <sys/stat.h>
#include "dfs_posix.h"

#define DBG_TAG "commu.parse.notify"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include <rtdbg.h>

/**
 * @brief   resolve notify command received from remote APP
 * @param   key: L2 key
 * @param   pValue: received value pointer
 * @param   length: value length
 * @retval  error code
 */

extern void reset_skai_widget_input_text(void);
extern bool get_is_open_instruction_list_ai(void);
extern void request_instruction_image(const char *id);
extern void update_instruction_image(const char *id, const char *path);
extern void remove_custom_instruction(const char *id);
extern void refresh_custom_instructions(void);

extern void parse_chat_item(cJSON *item, chat_t *note);
static uint8_t weather_data_updata_count = 0;

/* Copy a BLE payload slice into a NUL-terminated stack buffer.
   Truncates if payload exceeds dst_size-1. Avoids the
   `pValue[length] = '\0'` pattern, which writes one byte past the
   L1 receive buffer when length == max payload size. */
static inline void ble_payload_to_cstr(char *dst, size_t dst_size,
                                       const uint8_t *src, uint16_t len)
{
    size_t n = (len < dst_size - 1) ? len : dst_size - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

#ifndef BSP_USING_PC_SIMULATOR
/* Common dispatch for all OTA *_SYNC_START handlers — the 6 image flavours
   only differ in img_id and download address. Payload layout:
   [0..3] size (BE32), [4..7] compressed_size (BE32, optional),
   [8] compression_type (optional). */
static void ota_sync_start(uint8_t img_id, uint32_t addr, const uint8_t *pValue)
{
    uint32_t size = read_be32(&pValue[0]);
    #ifdef PKG_USING_LZ4
    uint32_t compressed_size = read_be32(&pValue[4]);
    uint8_t compression_type = pValue[8];
    if (compression_type == 1)
    {
        init_ble_dfu_thread_compressed(img_id, addr, size, compressed_size);
        return;
    }
    #endif
    init_ble_dfu_thread(img_id, addr, size);
}
#endif

void resolve_Notify_command(uint8_t key, uint8_t *pValue, uint16_t length)
{
    static MSG_DATA_PAYLOAD payload;
    static VOICE_RECOGNITION_PAYLOAD voice_recog_payload;
    static uint8_t calendar_day = 0;

    switch (key)
    {
    case KEY_INCOMMING_CALL:
    {
        if (SkaiWatchSys.msg_switch.bit.switch_call_msg)
        {
            LOG_I("watch received imcoming call:%.*s", length, pValue);
        }
        break;
    }
    case KEY_INCOMMING_MESSAGE:
    {
        if (length >= 1)
        {
            uint8_t msg_notify_type = pValue[0];
            char msg_buf[240];
            ble_payload_to_cstr(msg_buf, sizeof(msg_buf),
                                pValue + 1, length - 1);
            handle_notification(msg_notify_type, msg_buf);
        }
        break;
    }
    case KEY_DISMISS_NOTIFICATION:
    {
        if (length > 0)
        {
            char id_buf[NOTIFICATION_ID_LEN];
            ble_payload_to_cstr(id_buf, sizeof(id_buf), pValue, length);
            LOG_I("Dismiss notification id: %s", id_buf);
            dismiss_notification_from_phone(id_buf);
        }
        break;
    }

    case KEY_VOICE_RECOGNITION_RESULT:
    {
        voice_recog_payload.header = pValue[0];
        voice_recog_payload.length = length - 1;
        voice_recog_payload.p_msg_value = pValue + 1;
        watch_system_interact(INTERACT_VOICE_RECOGNITION, &voice_recog_payload);
        LOG_D("KEY_VOICE_RECOGNITION_RESULT");
        break;
    }

    case KEY_VOICE_RECOGNITION_END:
    {
        // voice_provider.auto_stop_listening();
        LOG_D("KEY_VOICE_RECOGNITION_END: %d", pValue[0]);
        if (pValue[0])
        {
            if (is_at_ai_interface() || is_at_speech_interface() ||
                get_is_open_instruction_list_ai())
                show_ai_processing_indicator(true);
        }
        else
        {
            if (is_at_speech_interface())
            {
                lvgl_msg_t msg;
                msg.type = LVGL_MSG_TYPE_RESET_AI_WIDGET;
                lvgl_send_msg(msg);
            }
            else if (is_at_ai_interface())
            {
                animate_to_home_from_ai_page();
            }
        }
        break;
    }

    case KEY_CHAT_RESULT:
    {
        LOG_D("KEY_CHAT_RESULT");
        payload.header = pValue[0];
        payload.length = length - 1;
        payload.p_msg_value = pValue + 1;
        watch_system_interact(INTERACT_CHAT_RESULT, &payload);
        break;
    }
#if defined(APP_ID_CALENDAR)
    case KEY_CALENDAR_SYNC_START:
    {
        // LOG_D("KEY_CALENDAR_SYNC_START");
        if (pValue != NULL)
        {
            calendar_day = pValue[0];
            if (calendar_day == 0)
            {
                cleanup_all_calendars();
            }
            LOG_D("calendar_day: %d, amount: %d", calendar_day, pValue[1]);
            if (calendar_day == 6 && pValue[1] == 0)
            {
                LOG_D("No calendar data for today, skipping notification.");
                notify_calendar();
            }
        }
        break;
    }
    case KEY_UPDATE_CALENDAR:
    {
        if (pValue != NULL)
        {
            handle_calendar((char *)pValue, calendar_day);
        }
        break;
    }

    case KEY_CALENDAR_SYNC_END:
    {
        notify_calendar();
        break;
    }
#endif

    case KEY_WEATHER_SYNC_START:
    {
        weather_data_updata_count = 0;
        LOG_D("KEY_WEATHER_SYNC_START");
        break;
    }

    case KEY_UPDATE_WEATHER:
    {
        LOG_D("KEY_UPDATE_WEATHER: %d", weather_data_updata_count);
        handle_weather((char *)pValue);
        weather_data_updata_count++;
        // for (int i = 0; i < 4; i++)
        // {
        //     weather_t *weather = get_weather_week(i);
        //     // LOG_D("weather[%d]: %d-%d-%d %d:%d\n", i, weather->time.year,
        //     // weather->time.month, weather->time.day, weather->time.hour,
        //     // weather->time.minutes);
        // }
        SkaiWatchSys.weather_moment_count = weather_data_updata_count;
        break;
    }

    //// OTA (start)
    case KEY_OTA_SYNC_STATUS:
    {
        uint8_t status = pValue[0];
        if (status == 0x00)
        {
            LOG_W("[OTA]SYNC STATUS: FAIL");
            set_ble_dfu_thread_run(0);
        }
        else if (status == 0x01)
        {
            LOG_I("[OTA]SYNC STATUS: SUCCESS");
            verify_and_upgrade_dfu_image();
        }
        else if (status == 100)
        {
            mark_ota_started();
        }

        break;
    }
    case KEY_WATCH_IMAGE_SYNC_START:
    {
#ifndef BSP_USING_PC_SIMULATOR
        ota_sync_start(DFU_IMG_ID_NAND_IMAGE, FS_START_DOWNLOAD_ADDRESS, pValue);
#endif
    }
    break;
    case KEY_WATCH_IMAGE_SYNC_END:
    {
        stop_ble_dfu_thread();
    }
    break;
    case KEY_UPDATE_WATCH_IMAGE:
    {
        handle_ble_dfu_data(pValue, length);
    }
    break;
    case KEY_WATCH_LCPU_SYNC_START:
    {
#ifndef BSP_USING_PC_SIMULATOR
        ota_sync_start(DFU_IMG_ID_NAND_LCPU,
                       LCPU_CODE_DOWNLOAD_START_ADDRESS, pValue);
#endif
    }
    break;
    case KEY_WATCH_LCPU_SYNC_END:
    {
        stop_ble_dfu_thread();
    }
    break;
    case KEY_UPDATE_WATCH_LCPU:
    {
        handle_ble_dfu_data(pValue, length);
    }
    break;
    case KEY_WATCH_HCPU_SYNC_START:
    {
#ifndef BSP_USING_PC_SIMULATOR
        ota_sync_start(DFU_IMG_ID_NAND_HCPU,
                       HCPU_CODE_DOWNLOAD_START_ADDRESS, pValue);
#endif
    }
    break;
    case KEY_WATCH_HCPU_SYNC_END:
    {
        stop_ble_dfu_thread();
        break;
    }
    case KEY_UPDATE_WATCH_HCPU:
    {
        handle_ble_dfu_data(pValue, length);
        break;
    }
    case KEY_WATCH_FTAB_SYNC_START:
    {
#ifndef BSP_USING_PC_SIMULATOR
        ota_sync_start(DFU_IMG_ID_NAND_FTAB, FTAB_START_DOWNLOAD_ADDRESS,
                       pValue);
#endif
    }
    break;
    case KEY_WATCH_FTAB_SYNC_END:
    {
        stop_ble_dfu_thread();
    }
    break;
    case KEY_UPDATE_WATCH_FTAB:
    {
        handle_ble_dfu_data(pValue, length);
    }
    break;
    case KEY_WATCH_BOOTLOADER_SYNC_START:
    {
#ifndef BSP_USING_PC_SIMULATOR
        ota_sync_start(DFU_IMG_ID_NAND_BOOTLOADER,
                       BOOTLOADER_DOWNLOAD_START_ADDRESS, pValue);
#endif
    }
    break;
    case KEY_WATCH_BOOTLOADER_SYNC_END:
    {
        stop_ble_dfu_thread();
    }
    break;
    case KEY_UPDATE_WATCH_BOOTLOADER:
    {
        handle_ble_dfu_data(pValue, length);
    }
    break;
    case KEY_WATCH_LCPU_PATCH_SYNC_START:
    {
#ifndef BSP_USING_PC_SIMULATOR
        ota_sync_start(DFU_IMG_ID_NAND_LCPU_PATCH,
                       LCPU_PATCH_DOWNLOAD_START_ADDRESS, pValue);
#endif
    }
    break;
    case KEY_WATCH_LCPU_PATCH_SYNC_END:
    {
        stop_ble_dfu_thread();
    }
    break;
    case KEY_UPDATE_WATCH_LCPU_PATCH:
    {
        handle_ble_dfu_data(pValue, length);
    }
    break;
    //// OTA (end) ////

    //// Watch System Request (start) ////
    case KEY_WATCH_SYS_REQUEST:
    {
        if (length == 0x00)
        {
            commu_send_watch_system_sync();
        }
        break;
    }
    //// Watch System Request (end) ////
    case KEY_MEDIA_TITLE:
    {
        char title_buf[128];
        ble_payload_to_cstr(title_buf, sizeof(title_buf), pValue, length);
        LOG_D("media title: %s", title_buf);
        watch_system_interact(INTERACT_SHOW_MEDIA_TITLE, title_buf);
        break;
    }

    case KEY_AI_PROCESS_TOOLKIT:
    {
        /* Payload changed from a single-byte tool key to a UTF-8 string
           describing the AI's current action. Copy into a null-terminated
           buffer and forward the string to the UI layer. */
        if (length > 0 && pValue != NULL)
        {
            char *desc = (char *)rt_malloc(length + 1);
            if (desc != NULL)
            {
                memcpy(desc, pValue, length);
                desc[length] = '\0';
                parse_ai_processing_toolkit(desc);
                rt_free(desc);
            }
        }
        break;
    }

    case KEY_START_SYNC_FILE:
    {
        /* Protocol: [file_path_length(1)][file_path][total_size(4)] */
        if (length < 5)
        {
            LOG_E("Invalid KEY_START_SYNC_FILE packet");
            break;
        }

        uint8_t path_len = pValue[0];
        if (length < (1 + path_len + 4))
        {
            LOG_E("Invalid KEY_START_SYNC_FILE packet length");
            break;
        }

        char file_path[FILE_PATH_MAX_LEN];
        rt_memcpy(file_path, &pValue[1], path_len);
        file_path[path_len] = '\0';

        uint32_t total_size = read_be32(&pValue[1 + path_len]);

        LOG_I("Start receiving file: %s (%d bytes)", file_path, total_size);
        bloc_start_receive_file(file_path, total_size);
        break;
    }

    case KEY_SYNC_FILE:
    {
        /* Protocol: [data...] */
        if (length > 0)
        {
            bloc_receive_file_data(pValue, length);
        }
        break;
    }

    case KEY_END_SYNC_FILE:
    {
        /* End file receive */
        int ret = bloc_end_receive_file();
        if (ret == RT_EOK)
        {
            LOG_I("File receive completed successfully");

            /* Check if it's a model file and reload if needed */
            // const file_receive_state_t *state = bloc_get_receive_progress();
            // if (state)
            // {
            // 	send_sys_interact_event(SYS_EVENT_RELOAD_GESTURE_MODEL);
            // }
        }
        else
        {
            LOG_E("File receive failed");
        }
        break;
    }

    case KEY_LOCATION_DATA:
    {
        if (length > 0)
        {
            char json_buf[240];
            ble_payload_to_cstr(json_buf, sizeof(json_buf), pValue, length);
            LOG_I("watch received location data:%s", json_buf);
            handle_location_data(json_buf);
        }
        break;
    }

    case KEY_SKAI_CREATION_INSTRUCTIONS:
    {
        if (length > 0)
        {
            LOG_I("Received instruction: %s", pValue);
            cJSON *root = cJSON_Parse((const char *)pValue);
            if (root)
            {
                cJSON *j_id = cJSON_GetObjectItem(root, "id");
                cJSON *j_title = cJSON_GetObjectItem(root, "title");
                cJSON *j_trigger = cJSON_GetObjectItem(root, "trigger");

                const char *id = cJSON_IsString(j_id) ? j_id->valuestring : "";
                const char *title = cJSON_IsString(j_title) ? j_title->valuestring : "";
                const char *trigger_type = "";
                uint32_t interval_sec = 0;

                if (j_trigger)
                {
                    cJSON *j_type = cJSON_GetObjectItem(j_trigger, "type");
                    if (cJSON_IsString(j_type))
                    {
                        trigger_type = j_type->valuestring;
                        if (strcmp(trigger_type, "interval") == 0)
                        {
                            cJSON *j_sec = cJSON_GetObjectItem(j_trigger,
                                                               "intervalSeconds");
                            if (cJSON_IsNumber(j_sec))
                                interval_sec = (uint32_t)j_sec->valuedouble;
                        }
                    }
                }

                uint32_t version = 0;
                cJSON *j_version = cJSON_GetObjectItem(root, "version");
                if (cJSON_IsNumber(j_version))
                    version = (uint32_t)j_version->valuedouble;

                bool enabled = false;
                cJSON *j_enabled = cJSON_GetObjectItem(root, "enabled");
                if (cJSON_IsBool(j_enabled))
                    enabled = cJSON_IsTrue(j_enabled);

                add_or_update_custom_instruction(id, title, trigger_type,
                                                  interval_sec, enabled,
                                                  version);

                /* Check if instruction image exists, request from phone if not */
                if (id[0] != '\0')
                {
                    char id_prefix[64];
                    strncpy(id_prefix, id, sizeof(id_prefix) - 1);
                    id_prefix[sizeof(id_prefix) - 1] = '\0';
                    char *dash = strchr(id_prefix, '-');
                    if (dash)
                        *dash = '\0';

                    char img_path[128];
                    rt_snprintf(img_path, sizeof(img_path),
                                "/assets/images/instruction/%s.bin", id_prefix);

                    struct stat st;
                    if (stat(img_path, &st) == 0)
                    {
                        /* Image already exists, set it on the instruction */
                        update_instruction_image(id, img_path);
                    }
                    else
                    {
                        /* Image not found, request from phone */
                        request_instruction_image(id);
                    }
                }

                refresh_custom_instructions();
                cJSON_Delete(root);
            }
        }
        break;
    }

    case KEY_DISMISS_SKAI_INSTRUCTION:
    {
        if (length > 0)
        {
            LOG_I("Dismiss instruction: %s", pValue);
            remove_custom_instruction((const char *)pValue);
            refresh_custom_instructions();
        }
        break;
    }

    default:
        break;
    }
}
