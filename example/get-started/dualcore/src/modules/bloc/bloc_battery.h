/**
 ******************************************************************************
 * @file   bloc_battery.h
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
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

#ifndef __BLOC_BATTERY_H__
    #define __BLOC_BATTERY_H__

    #ifdef __cplusplus
extern "C"
{
    #endif

    #include "bsp_board.h"

    typedef struct
    {
        //! A percentage (0-100) of how full the battery is
        uint8_t charge_percent;
        //! True if the battery is currently being charged. False if not.
        bool is_charging;
        bool is_full; // Indicates if the battery is fully charged
        //! True if the charger cable is connected. False if not.
        bool is_plugged;
    } BatteryChargeState;

    extern BatteryChargeState* battery_get_charge_state(void);
    extern BatteryChargeState battery_charge_state;

    extern void bloc_battery_init(void);
    extern void bloc_battery_read_voltage(void);
    extern void bloc_battery_read_voltage_after_settle(void);
    extern void bloc_battery_read_charge_status(void);
    extern void bloc_battery_handle_charging_event(void);
    extern void bloc_battery_handle_voltage_event(void);
    extern void bloc_battery_handle_voltage_poll_event(void);
    /* Restore a SOC persisted by the HCPU across a reset. No-op unless it
       arrives before the gauge has anchored itself. */
    extern void bloc_battery_seed_soc(uint8_t percent);
    #ifdef __cplusplus
}
    #endif

#endif // __BLOC_BATTERY_H__

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/
