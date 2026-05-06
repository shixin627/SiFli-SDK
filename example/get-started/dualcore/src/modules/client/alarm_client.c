/**
 ******************************************************************************
 * @file   alarm_client.c
 * @author Skaiwalk software development team
 * @brief power client source.
 *
 ******************************************************************************
 */
/**
 * @attention
 * Copyright (c) 2018 - 2023,  Skaiwalk Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <rtthread.h>
#include <board.h>
#include <string.h>
#include "data_service_subscriber.h"
#include "ui_datasrv_subscriber.h"
#include "alarm_manager_service.h"
#include "alarm_client.h"
#include "littlevgl2rtt.h"
#include "math.h"
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "bloc_peripheral.h"   /* control_motor + motor_params_t (via bloc_motor.h) */
#include "gui_app_fwk.h"
#include "gui_app_pm.h"
#include "ui_handler.h"
#ifdef BSP_USING_BLOC_NOTIFY
#include "bloc_notification.h"
#endif

#define DBG_LVL DBG_LOG
#define DBG_TAG "CLIENT.ALARM"
#include "rtdbg.h"

/* Alarm */
static datac_handle_t alarm_client_handle;

static int alarm_service_callback(data_callback_arg_t *arg)
{
    if (ALARMMGR_MSG_GET_ALARM_REQ == arg->msg_id)
    {
        alarm_msg_t *rsp_msg;

        RT_ASSERT(arg->data_len == sizeof(alarm_msg_t));
        rsp_msg = (alarm_msg_t *)arg->data;
        LOG_D("Get alarm request:%d", rsp_msg->idx);
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        RT_ASSERT(alarm_client_handle == rsp->handle);
        LOG_D("Subscribe result:%d", rsp->result);
    }
    else if (MSG_SERVICE_UNSUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        RT_ASSERT(alarm_client_handle == rsp->handle);
        LOG_D("Unsubscribe result:%d", rsp->result);
    }
    return 0;
}

int subscribe_alarm_client(void)
{
    /* Idempotent: skip if already subscribed. */
    if (DATA_CLIENT_INVALID_HANDLE != alarm_client_handle)
    {
        return 0;
    }
    alarm_client_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != alarm_client_handle);
    datac_subscribe(alarm_client_handle, "alarmmgr", alarm_service_callback, 0);
    return 0;
}

void unsubscribe_alarm_client(void)
{
    datac_close(alarm_client_handle);
    alarm_client_handle = DATA_CLIENT_INVALID_HANDLE;
}

/* Pack an alarm_contxt_t into the 40-bit T_ALARM bit-field layout that the
   wire format and SkaiWatchSys.alarms[] both expect.

   IMPORTANT: build the value in a local T_ALARM (stack — guaranteed 8-byte
   aligned) and memcpy() it to @p out. Writing `out->data = 0` directly can
   emit a 64-bit STRD; if @p out happens to be on a 4-byte-but-not-8-byte
   boundary inside SkaiWatchSys, the CPU traps with SCB_CFSR.UNALIGNED. */
static void pack_ctx_to_t_alarm(const alarm_contxt_t *ctx, uint8_t idx,
                                T_ALARM *out)
{
    T_ALARM tmp;
    tmp.data = 0;
    tmp.alarm.day_repeat_flag = ctx->days & 0x7F;
    tmp.alarm.reserved = (ctx->state == ALARM_STATE_ENABLE) ? 0x1 : 0x0;
    tmp.alarm.id = idx & 0x07;
    tmp.alarm.minute = ctx->minute & 0x3F;
    tmp.alarm.hour = ctx->hour & 0x1F;
    /* day/month/year stay zero. */
    memcpy(out, &tmp, sizeof(T_ALARM));
}

void watch_alarm_local_sync(int op, int32_t idx, const alarm_contxt_t *ctx)
{
    switch (op)
    {
    case 0: /* ADD */
        if (SkaiWatchSys.alarm_num < MAX_ALARM_NUM && ctx)
        {
            pack_ctx_to_t_alarm(ctx, SkaiWatchSys.alarm_num,
                                &SkaiWatchSys.alarms[SkaiWatchSys.alarm_num]);
            SkaiWatchSys.alarm_num++;
        }
        break;
    case 1: /* EDIT in place */
        if (idx >= 0 && idx < SkaiWatchSys.alarm_num && ctx)
        {
            pack_ctx_to_t_alarm(ctx, (uint8_t)idx, &SkaiWatchSys.alarms[idx]);
        }
        break;
    case 2: /* DELETE — shift down */
        if (idx >= 0 && idx < SkaiWatchSys.alarm_num)
        {
            /* memmove instead of struct assignment: T_ALARM is 8 bytes and
               struct copy can compile to STRD/LDRD which faults on
               unaligned slots. */
            if (idx < SkaiWatchSys.alarm_num - 1)
            {
                memmove(&SkaiWatchSys.alarms[idx],
                        &SkaiWatchSys.alarms[idx + 1],
                        sizeof(T_ALARM)
                            * (SkaiWatchSys.alarm_num - 1 - idx));
            }
            SkaiWatchSys.alarm_num--;
            /* Zero the freed slot for cleanliness. */
            memset(&SkaiWatchSys.alarms[SkaiWatchSys.alarm_num], 0,
                   sizeof(T_ALARM));
        }
        break;
    default: return;
    }
    watch_prefs_save_alarms();
}

void apply_alarms_from_ble(const T_ALARM *alarms, uint8_t num)
{
    /* The phone sends the full alarm list each sync, so clear existing
       entries before adding. The manager has no "clear all" message — fire
       enough delete-at-idx-0 messages to drain its list (caps at 8 per
       BSP_ALARM_MAX in alarm_manager_service.c). Extras are no-ops. */
    data_msg_t msg;
    alarm_msg_t *p;

    subscribe_alarm_client();

    for (int i = 0; i < 8; i++)
    {
        p = (alarm_msg_t *)data_service_init_msg(
            &msg, ALARMMGR_MSG_DELETE_ALARM_REQ, sizeof(alarm_msg_t));
        p->idx = 0;
        datac_send_msg(alarm_client_handle, &msg);
    }

    for (uint8_t i = 0; i < num; i++)
    {
        p = (alarm_msg_t *)data_service_init_msg(
            &msg, ALARMMGR_MSG_ADD_ALARM_REQ, sizeof(alarm_msg_t));
        p->idx = i;
        /* Phone packs `enabled` flag into reserved[0] (bit 7 of the 40-bit
           T_ALARM value); honour it instead of always arming. */
        p->ctx.state  = (alarms[i].alarm.reserved & 0x1)
                            ? ALARM_STATE_ENABLE : ALARM_STATE_DISABLE;
        p->ctx.hour   = alarms[i].alarm.hour;
        p->ctx.minute = alarms[i].alarm.minute;
        p->ctx.snooze = ALARM_SNOOZE_DISABLE;
        /* day_repeat_flag bit layout: bit0=Sunday..bit6=Saturday — matches
           the watch's runtime use of tm_wday in setup_hw_alarm(). */
        p->ctx.days   = alarms[i].alarm.day_repeat_flag;
        datac_send_msg(alarm_client_handle, &msg);
    }
}

static void get_alarm(void)
{
    data_msg_t msg;
    (void)data_service_init_msg(&msg, ALARMMGR_MSG_GET_ALARM_REQ, 0);
    rt_err_t err = datac_send_msg(alarm_client_handle, &msg);
    RT_ASSERT(RT_EOK == err);
}

/* ── Currently-ringing alarm: shared between fire worker, alarm UI, and
   stop/snooze callbacks. Set by bloc_alarm_on_fire, cleared by
   bloc_alarm_stop_ringing. ── */
static volatile int32_t s_ringing_idx = -1;

/* Apple-Watch-style pulsed haptic. A periodic rt_timer fires every
   PULSE_INTERVAL_MS and submits a short triple-tap burst via control_motor.
   The motor finishes the burst on its own, and the timer fires the next
   burst after the silence — giving a distinct "tap-tap-tap … tap-tap-tap"
   rhythm instead of a constant buzz. */
#define ALARM_PULSE_INTERVAL_MS 2500   /* between bursts — total cadence */
#define ALARM_PULSE_PERIOD_US   200000 /* 200ms total per pulse (100 on/100 off) */
#define ALARM_PULSE_REPEATS     3      /* taps per burst */
static rt_timer_t s_pulse_timer = RT_NULL;

int32_t bloc_alarm_get_ringing_idx(void)
{
    return s_ringing_idx;
}

static void alarm_pulse_burst(void)
{
    if (s_ringing_idx < 0) return;          /* late tick after stop — ignore */
    if (!get_motor_switch_state()) return;  /* user globally muted vibration */

    motor_params_t params = {
        .duty_cycle  = 51,
        .period      = ALARM_PULSE_PERIOD_US,
        .repeat_times = ALARM_PULSE_REPEATS,
    };
    peripheral_provider.control_motor(true, &params);
}

static void alarm_pulse_timer_cb(void *param)
{
    (void)param;
    alarm_pulse_burst();
}

void bloc_alarm_stop_ringing(bool snooze_5min)
{
    if (s_ringing_idx < 0) return;
    s_ringing_idx = -1;

    /* Halt the periodic timer first so no new burst is queued, then cancel
       any in-flight burst. */
    if (s_pulse_timer) rt_timer_stop(s_pulse_timer);
    peripheral_provider.control_motor(false, NULL);

    if (snooze_5min)
    {
        /* Schedule a one-shot alarm 5 minutes from the current wall clock.
           Use SkaiWatchSys.Global_Time so we don't have to drag in a
           toolchain-conditional localtime_r. */
        int snooze_min = SkaiWatchSys.Global_Time.minutes + 5;
        int snooze_hr  = SkaiWatchSys.Global_Time.hour;
        if (snooze_min >= 60)
        {
            snooze_min -= 60;
            snooze_hr = (snooze_hr + 1) % 24;
        }

        subscribe_alarm_client();
        data_msg_t msg;
        alarm_msg_t *p = (alarm_msg_t *)data_service_init_msg(
            &msg, ALARMMGR_MSG_ADD_ALARM_REQ, sizeof(alarm_msg_t));
        p->idx = 0;
        p->ctx.state  = ALARM_STATE_ENABLE;
        p->ctx.hour   = snooze_hr;
        p->ctx.minute = snooze_min;
        p->ctx.snooze = ALARM_SNOOZE_DISABLE;
        p->ctx.days   = ALARM_REPEAT_ONE_SHOT; /* 0 — fires once */
        datac_send_msg(alarm_client_handle, &msg);
        LOG_I("Snoozed: new one-shot at %02d:%02d", snooze_hr, snooze_min);
    }
}

/* Override the weak hook in alarm_manager_service. Runs on the fire worker
   thread (NOT soft-RTC), so calling LVGL/motor/wake APIs is safe.

   When the watch is asleep, peripheral power rails (including the motor PWM
   controller) are down. Just calling start_motor() returns silently because
   the GPIO/PWM is unpowered. Mirror INTERACT_TIMER_REMINDER's wake pattern:
   trigger the PM FSM, resume HCPU peripherals, wait ~500ms for rails to
   stabilise, *then* vibrate and surface the alarm UI. */
void bloc_alarm_on_fire(int32_t alarm_idx)
{
    LOG_I("Alarm[%d] firing", alarm_idx);
    s_ringing_idx = alarm_idx;

    if (!gui_is_active())
    {
        gui_pm_fsm(GUI_PM_ACTION_BUTTON_CLICKED);
        peripheral_provider.hcpu_resume();
        rt_thread_mdelay(500);
    }

    /* Apple-Watch-style pulsed pattern: short triple-tap burst, ~2 seconds
       silence, repeat — all the way until bloc_alarm_stop_ringing() is
       called. control_motor() goes through the peripheral message thread
       (which itself checks sys_power_status), and motor_params_t.period is
       in MICROSECONDS (not ms). */
    alarm_pulse_burst();   /* fire one immediately so user feels it on wake */
    if (!s_pulse_timer)
    {
        s_pulse_timer = rt_timer_create(
            "alm_buzz", alarm_pulse_timer_cb, RT_NULL,
            rt_tick_from_millisecond(ALARM_PULSE_INTERVAL_MS),
            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (s_pulse_timer) rt_timer_start(s_pulse_timer);

    /* Pop the alarm app so the user has something to dismiss into. */
    gui_app_run(APP_ID_ALARM);
}

#if !kReleaseMode
static int alarm_request(int argc, char *argv[])
{
    if (argc == 2)
    {
        if (strcmp(argv[1], "subscribe") == 0)
        {
            watch_system_interact(WATCH_ALARM_INIT, NULL);
        }
        else if (strcmp(argv[1], "unsubscribe") == 0)
        {
            unsubscribe_alarm_client();
        }
        else if (strcmp(argv[1], "get") == 0)
        {
            get_alarm();
        }
    }
    return 0;
}
MSH_CMD_EXPORT(alarm_request, "alarm request");
#endif
