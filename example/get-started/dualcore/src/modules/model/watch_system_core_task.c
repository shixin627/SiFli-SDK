// /**
//  ******************************************************************************
//  * @file   watch_system_core_task.c
//  * @author Skaiwalk software development team
//  ******************************************************************************
//  */
// /**
//  * Copyright (c) 2024 - 2025, Skaiwalk Technology
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright notice,
//  * this list of conditions and the following disclaimer.
//  *
//  * 2. Redistributions in binary form, except as embedded into a Skaiwalk
//  * integrated circuit in a product or a software update for such product, must
//  * reproduce the above copyright notice, this list of conditions and the
//  * following disclaimer in the documentation and/or other materials provided
//  * with the distribution.
//  *
//  * 3. The names of Skaiwalk or its contributors may not be used to endorse
//  *    or promote products derived from this software without specific prior
//  * written permission.
//  *
//  * 4. This software, with or without modification, must only be used with a
//  *    Skaiwalk integrated circuit.
//  *
//  * 5. Any binary form of this software must not be reverse engineered,
//  * decompiled, modified, or disassembled.
//  *
//  * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
//  * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
//  * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  */

// #include <rtthread.h>
// #include <string.h>
// #include <stdlib.h>
// #include <board.h>
// #include "watch_system_interact.h"
// #include "watch_system_core_task.h"
// #include "data_service_subscriber.h"
// #include "power_manager_service.h"
// #include "gesture_handler.h"
// #include "lv_ext_resource_manager.h"
// #include "bf0_ble_common.h"
// #include "app_mainmenu.h"
// #include "ui_handler.h"
// #include "ui_helper.h"
// #include "drv_touch.h"
// #include "bloc_control.h"
// #include "bloc_peripheral.h"
// #include "bloc_notification.h"
// #include "bloc_setting.h"
// #include "bloc_v2t.h"
// #include "bloc_weather.h"
// #include "bloc_skaiwalk.h"
// #include "bloc_motion_tracking.h"
// #include "bloc_filesystem.h"
// #include "watch_sys_service.h"
// #include "communicate_protocol.h"
// #ifdef BSP_USING_MAHONY_AHRS
//     #include "sensor_fusion.h"
// #endif
// #ifdef BSP_USING_PM
//     #include "bf0_pm.h"
//     #include "gui_app_pm.h"
// #endif
// #include "bf0_sibles_internal.h"
// #include "bf0_ble_gap.h"

// #define DBG_TAG "system.core"
// #define DBG_LVL DBG_LOG
// #include <rtdbg.h>

// #define SYS_THREAD_STACK_SIZE 1024 * 4
// #define SYS_THREAD_PRIORITY RT_THREAD_PRIORITY_LOW - 2
// #define SYS_THREAD_TIMESLICE 10

// extern lv_obj_t *loading_screen;
// extern void stop_ble_rssi_checker(void);

// void send_sys_interact_event(uint32_t event);

// uint8_t get_sys_power_status(void)
// {
//     return SkaiWatchSys.sys_power_status;
// }

// static void set_sys_power_status(uint8_t status)
// {
//     SkaiWatchSys.sys_power_status = status;
// }

// static struct rt_event sys_interact_event;

// static watch_prefs_key local_key;
// void store_watch_shared_prefs(watch_prefs_key key)
// {
//     local_key = key;
//     send_sys_interact_event(SYS_EVENT_SAVE_SHARE_PREFS);
// }

// static bool get_touch_state(void)
// {
//     return peripheral_provider.get_tap_status() || is_user_touching_screen();
// }

// static rt_timer_t watch_sleep_timer = RT_NULL;

// static void watch_sleep_timer_cb(void *parameter)
// {
//     send_sys_interact_event(SYS_EVENT_HCPU_SUSPEND);
// }

// void watch_hcpu_resume_with_reason(uint8_t reason)
// {
//     SkaiWatchSys.wakeup_reson = reason;
//     LOG_D("%s reason:%d", __func__, reason);
//     send_sys_interact_event(SYS_EVENT_HCPU_RESUME);
// }

// static void open_gui(void)
// {
//     // 恢復狀態
//     gui_set_brightness(SkaiWatchSys.brightness, false);
//     SkaiWatchSys.flag_field.is_wearing = true;
//     switch (SkaiWatchSys.wakeup_reson)
//     {
//     case WAKEUP_REASON_OTHER:
//     {
//         if (gui_app_is_all_closed())
//         {
//             gui_app_run("Main");
//         }
//         else
//         {
//             watch_sys_sync.sync_api_lock(SkaiWatchSys.motion_control_lock);
//         }
//         break;
//     }
//     case WAKEUP_REASON_ROTATE_INWARD:
//     {
//         if (!gui_app_is_actived(APP_ID_MAIN))
//         {
//             gui_app_run("Main");
//             // while (!gui_app_is_actived(APP_ID_MAIN))
//             // {
//             //   rt_thread_mdelay(100);
//             // }
//             // animate_to_app_list();
//         }
//         else
//         {

//             if (is_at_home())
//             {
//                 // motor_pattern_unlocked();
//                 LOG_D("Rotate inward, open app list");
//                 // animate_to_app_list();
//                 switch_watch_motion_control_mode(true, true);
//                 animate_to_message_list();
//             }
//         }
//         break;
//     }
//     case WAKEUP_REASON_CHARGER:
//     {
//         if (is_ble_dfu_thread_running())
//         {
//             LOG_W("BLE DFU thread is running, cannot launch app");
//             return;
//         }
// #if CUSTOMER_BOARD_VER != BOARD_VER_13
//         AppIntent appIntent;
//         strcpy(appIntent.app_id, APP_ID_BATTERY);
//         strcpy(appIntent.intent, "CHARGING");
//         watch_run_app_by_intent(&appIntent);
// #endif
//         break;
//     }
//     default:
//         break;
//     }
// }

// static void watchdata_update(void)
// {
//     request_weather_within_six_hours(false);
// }

// #ifdef SOC_BF0_HCPU
// static void watch_hcpu_wakeup(void)
// {
//     LOG_D("%s", __func__);
//     #ifdef BSP_USING_PM
//     if (!gui_is_active())
//     {
//         gui_pm_fsm(GUI_PM_ACTION_BUTTON_CLICKED);
//         SkaiWatchSys.pre_hcpu_wakeup_tick = rt_tick_get();
//         set_sys_power_status(SYS_POWER_STATUS_ON);
//         open_gui();
//         // set_idle_state(false);
//         rt_thread_mdelay(50);
//         LOG_D("[%s]subscribe imu sensor", __func__);
//         sensor_subscription_t sensor_subscription = (sensor_subscription_t){
//             .type = SENSOR_TYPE_ACCELEROMETER,
//             .status = true,
//             .thread_safe = true,
//         };
//         watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
//         rt_thread_mdelay(60);
//         LOG_D("[%s]notify lcpu resume, hcpu resume", __func__);
//         peripheral_provider.hcpu_resume();
//         #if (CUSTOMER_BOARD_VER > BOARD_VER_13)
//         if (SkaiWatchSys.hrs_start_up_mode == 0)
//         {
//             peripheral_provider.hr_set_power(1);
//         }
//         #endif
//         #if USE_FFT_FILTER
//         extern void init_fft(void);
//         init_fft();
//         #endif
//     }
//     else
//     #endif
//     {
//         // if (get_idle_state())
//         // {
//         //   lv_disp_trig_activity(NULL);
//         //   stop_watch_sleep_timer();
//         //   close_standby_page();
//         //   set_idle_state(false);
//         // }
//         watchdata_update();
//         open_gui();
//     }
// }

// static void watch_hcpu_sleep(void)
// {
//     #ifndef BSP_USING_PC_SIMULATOR
//         #ifdef BSP_USING_PM
//     if (gui_is_active())
//     {
//             #if USE_FFT_FILTER
//         extern void deinit_fft(void);
//         deinit_fft();
//             #endif
//         peripheral_provider.hcpu_suspend();
//             #if (CUSTOMER_BOARD_VER > BOARD_VER_13)
//         if (SkaiWatchSys.hrs_start_up_mode == 0)
//         {
//             peripheral_provider.hr_set_power(0);
//         }
//             #endif
//         rt_thread_mdelay(60);
//         sensor_subscription_t sensor_subscription = (sensor_subscription_t){
//             .type = SENSOR_TYPE_ACCELEROMETER,
//             .status = false,
//             .thread_safe = true,
//         };
//         watch_system_interact(WATCH_SENSOR_SUBSCRIBE, &sensor_subscription);
//         rt_thread_mdelay(50);
//         // set_idle_state(true);
//         set_sys_power_status(SYS_POWER_STATUS_SLEEP);
//         gui_pm_fsm(GUI_PM_ACTION_SLEEP);
//     }
//         #endif
//     #endif
// }
// #endif // SOC_BF0_HCPU

// // --------- watch wakeup from power off status ---------
// void handle_wakeup_event(void)
// {
//     BSP_Motor_PowerSwitch(true);
//     SkaiWatchSys.motion_control_lock = true;
//     setting_provider.set_power_save_mode(1);

//     lv_disp_trig_activity(NULL);

//     SubscribeDualCoreSyncService();
//     rt_thread_mdelay(3000);
//     peripheral_provider.hcpu_resume();
//     peripheral_provider.subscribe_accelerometer_sensor(true);

//     rt_thread_mdelay(100);

//     gui_app_exit("Loader");
//     // set_idle_state(false);
//     extern void ble_dev_mgr_start_main_phone_check_timer(uint32_t interval_ms);
//     ble_dev_mgr_start_main_phone_check_timer(5000);
//     // LOG_D("wakeup end");
// }

// // --------- watch software reboot ---------
// static void handle_reboot_event(void)
// {
//     watch_config_struct_flash_write();
//     rt_thread_mdelay(50);
// #ifndef BSP_USING_PC_SIMULATOR
//     extern void drv_reboot(void);
//     drv_reboot();
// #endif
// }

// static void handle_power_off_event(void)
// {
//     set_sys_power_status(SYS_POWER_STATUS_OFF);
//     watch_config_struct_flash_write();
//     rt_thread_mdelay(50);
// #ifdef BSP_USING_PM
//     pm_shutdown();
// #endif
// }

// extern int lcpu_bt_disable(void);
// static void handle_reboot_lcpu_event(void)
// {
//     // extern void halt_and_reset_lcpu_nor_flash(void);
//     // halt_and_reset_lcpu_nor_flash();
//     peripheral_provider.subscribe_accelerometer_sensor(false);
//     rt_thread_mdelay(100);
//     UnsubscribeDualCoreSyncService();
//     rt_thread_mdelay(100);
//     peripheral_provider.hcpu_suspend();
//     rt_thread_mdelay(100);
//     lcpu_bt_disable();
//     rt_thread_mdelay(100);
//     extern uint8_t lcpu_power_off(void);
//     lcpu_power_off();
//     rt_thread_mdelay(100);
// #ifndef BSP_USING_PC_SIMULATOR
//     extern uint8_t lcpu_power_on(void);
//     lcpu_power_on();
//     rt_thread_mdelay(3000);
//     extern void lcpu_bt_enable(void);
//     lcpu_bt_enable();
//     rt_thread_mdelay(100);
//     SubscribeDualCoreSyncService();
//     rt_thread_mdelay(100);
//     peripheral_provider.hcpu_resume();
//     rt_thread_mdelay(100);
//     peripheral_provider.subscribe_accelerometer_sensor(true);
// #endif
// }

// static void handle_save_share_prefs_event(void)
// {
// #ifndef BSP_USING_PC_SIMULATOR
//     store_watch_prefs(local_key);
// #endif
// }

// static void handle_hcpu_suspend_event(void)
// {
//     if (!is_sleep_mode())
//     {
//         // if (gui_app_is_actived("Main") || gui_app_is_actived(APP_ID_MESSAGE))
//         {
//             gui_app_cleanup();
//         }

//         watch_hcpu_sleep();
//     }
//     else
//     {
//         LOG_D("Already in sleep mode");
//     }
//     stop_ble_rssi_checker();
// }

// static void handle_hcpu_prepare_sleep_event(void)
// {
//     if (get_sys_power_status() == SYS_POWER_STATUS_ON)
//     {
//         if (!get_idle_state() && !get_touch_state())
//         {
//             set_idle_state(true);
//             lvgl_msg_t msg;
//             msg.type = LVGL_MSG_TYPE_CONTROL_QUICK_BTN;
//             msg.data.quick_btn_action.new_point = 0;
//             msg.data.quick_btn_action.swich_quick_btn = 0;
//             lvgl_send_msg(msg);
//             open_standby_page();
//             start_watch_sleep_timer();
//         }
//     }
// }

// #if ENABLE_VIRTUAL_TOUCH
// static void simulate_finger_press_screen(int x, int y)
// {
//     setCoordinateX(x);
//     setCoordinateY(y);
//     LOG_D("simulate_finger_press_screen x:%d, y:%d", x, y);
// }

// static void simulate_press_by_orientation(void)
// {
//     LOG_D("simulate_press_by_orientation");
//     if (lv_disp_get_rotation(NULL) != LV_DISP_ROT_90 &&
//         lv_disp_get_rotation(NULL) != LV_DISP_ROT_270)
//     {
//         simulate_finger_press_screen(LV_HOR_RES - 1, HALF_SCREEN_HEIGHT + 100);
//     }
//     else
//     {
//         simulate_finger_press_screen(40, HALF_SCREEN_HEIGHT);
//     }
// }

// void gesture_touch_event_handler(void)
// {
//     if (app_control_get_mouse_mode())
//     {
//         return;
//     }
//     if (is_at_instruction_list())
//     {

//         simulate_press_by_orientation();
//         extern void press_navigation_bar(void);
//         press_navigation_bar();
//     }
// }
// #endif

// static void handle_hcpu_resume_event(void)
// {
//     watch_hcpu_wakeup();
// }

// void sys_core_task_handler(void *parameter)
// {
//     rt_uint32_t recv_set = 0;
//     // handle_wakeup_event();
//     while (1)
//     {
//         if (rt_event_recv(&sys_interact_event, WATCH_SYS_ALL_EVENT,
//                           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
//                           RT_WAITING_FOREVER, &recv_set) == RT_EOK)
//         {
//             switch (recv_set)
//             {
//                 // case SYS_EVENT_POWER_ON:
//                 //   handle_wakeup_event();
//                 //   break;

//                 // case SYS_EVENT_REBOOT_SYS:
//                 //   handle_reboot_event();
//                 //   break;

//                 // case SYS_EVENT_POWER_OFF:
//                 //   handle_power_off_event();
//                 //   break;

//                 // case SYS_EVENT_REBOOT_LCPU:
//                 //   handle_reboot_lcpu_event();
//                 //   break;

//             case SYS_EVENT_SAVE_SHARE_PREFS:
//                 handle_save_share_prefs_event();
//                 break;

//             case SYS_EVENT_BATT_VOLTAGE:
//             {
// #ifndef BSP_USING_PC_SIMULATOR
//                 extern void notify_battery_voltage(void);
//                 notify_battery_voltage();
// #endif
//                 break;
//             }

//             case SYS_EVENT_BATT_CHARGE:
// #ifdef BSP_USING_BLOC_NOTIFY
//                 notify_provider.charge_status(SkaiWatchSys.charger_status);
// #endif
//                 break;

//             // case SYS_EVENT_PREPARE_SLEEP:
//             //   handle_hcpu_prepare_sleep_event();
//             //   break;

//             // case SYS_EVENT_HCPU_RESUME:
//             //   handle_hcpu_resume_event();
//             //   break;

//             // case SYS_EVENT_HCPU_SUSPEND:
//             //   handle_hcpu_suspend_event();
//             //   break;

//             // case SYS_EVENT_WATCH_LOCK:
//             //   watch_sys_sync.sync_api_lock(SkaiWatchSys.motion_control_lock);
//             //   break;
//             default:
//                 break;
//             }
//         }
//     }
// }

// int sys_core_task_init(void)
// {
//     // rt_err_t result;
//     rt_thread_t sys_core_task;

//     /* Create the event */
//     if (rt_event_init(&sys_interact_event, "sys_interact_evt",
//                       RT_IPC_FLAG_FIFO) != RT_EOK)
//     {
//         LOG_E("Failed to create event");
//         return -1;
//     }
//     sys_core_task = rt_thread_create("sys_core_task", sys_core_task_handler,
//                                      RT_NULL, SYS_THREAD_STACK_SIZE,
//                                      SYS_THREAD_PRIORITY, SYS_THREAD_TIMESLICE);

//     if (sys_core_task)
//     {
//         rt_thread_startup(sys_core_task);
//     }
//     else
//     {
//         LOG_E("Failed to create sys_core_task");
//         return -1;
//     }

//     return 0;
// }
// INIT_APP_EXPORT(sys_core_task_init);

// void send_sys_interact_event(uint32_t event)
// {
//     rt_event_send(&sys_interact_event, event);
// }

// /************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
//  * FILE****/