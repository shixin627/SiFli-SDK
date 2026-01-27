/**
 ******************************************************************************
 * @file   watch_system_core_task.h
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

#ifndef __WATCH_SYSTEM_CORE_TASK_H__
#define __WATCH_SYSTEM_CORE_TASK_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "watch_global_data.h"
#include "watch_system_core_task.h"

    // wake up reson: 0: lift wrist, 1: charger, 2: other
    enum wakeup_reason
    {
        WAKEUP_REASON_OTHER = 0,
        WAKEUP_REASON_ROTATE_INWARD = 1,
        WAKEUP_REASON_CHARGER = 2,
    };
    extern uint8_t get_sys_power_status(void);
    extern void send_sys_interact_event(uint32_t event);

    extern void set_watch_ready_to_open_display(bool state);
    extern void set_user_want_to_open_display_to_app_list(bool state);
    extern void store_watch_shared_prefs(watch_prefs_key key);
    extern void watch_hcpu_resume_with_reason(uint8_t reason);
    extern void switch_watch_motion_control_mode(bool enable, bool animation);
    extern bool is_user_want_to_open_display_to_app_list(void);
    extern void gesture_touch_event_handler(void);
    extern void ble_app_advertising_start(bool restart_adv, bool mouse_mode, bool pairing_mode);

#ifdef __cplusplus
}
#endif

#endif //__WATCH_SYSTEM_CORE_TASK_H__
       /************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/