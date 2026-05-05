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
#include "bloc_peripheral.h"
#ifdef BSP_USING_BLOC_NOTIFY
#include "bloc_notification.h"
#endif
#ifdef BSP_USING_COMMUNICATE
#include "communicate_protocol.h"
#include "communicate_parse.h"
#endif

#define DBG_LVL DBG_LOG
#define DBG_TAG "CLIENT.ALARM"
#include "rtdbg.h"

/* Alarm */
static datac_handle_t alarm_client_handle;
static alarm_msg_t alarm_msg;

#ifdef BSP_USING_COMMUNICATE
/* Push-back state machine: when watch UI mutates an alarm, we iterate the
   alarm_manager_service list asynchronously, repack each entry into the
   T_ALARM 40-bit wire format, and notify the phone via KEY_RETURN_ALARM_SETTINGS.
   The same packet shape `commu_send_alarm_settings()` already uses, so phone-side
   `onAlarms` callback (watch_protocol_service.dart) handles it unchanged. */
static struct
{
    bool active;
    bool dirty;     /* re-fire after current pass if another change came in */
    uint8_t count;
    uint8_t buf[5 + 5 * 8]; /* L2 header + up to BSP_ALARM_MAX entries */
} alarm_push;

static void alarm_push_request_next(const alarm_msg_t *prev)
{
    data_msg_t msg;
    if (prev == NULL)
    {
        data_service_init_msg(&msg, ALARMMGR_MSG_GET_ALARM_LIST_NEXT_REQ, 0);
    }
    else
    {
        uint8_t *body = data_service_init_msg(
            &msg, ALARMMGR_MSG_GET_ALARM_LIST_NEXT_REQ, sizeof(alarm_msg_t));
        memcpy(body, prev, sizeof(alarm_msg_t));
    }
    datac_send_msg(alarm_client_handle, &msg);
}

static void alarm_push_start(void)
{
    if (alarm_push.active)
    {
        alarm_push.dirty = true; /* coalesce — fire one more pass after this */
        return;
    }
    alarm_push.active = true;
    alarm_push.dirty = false;
    alarm_push.count = 0;
    alarm_push.buf[0] = SET_CONFIG_COMMAND_ID;
    alarm_push.buf[1] = L2_HEADER_VERSION;
    alarm_push.buf[2] = KEY_RETURN_ALARM_SETTINGS;
    alarm_push.buf[3] = 0;
    alarm_push.buf[4] = 0;
    alarm_push_request_next(NULL);
}

static void alarm_push_append(const alarm_msg_t *a)
{
    if (alarm_push.count >= 8) return;
    /* Pack alarm_contxt_t back into the 40-bit T_ALARM layout that the phone
       expects (bits: 0-6 days, 7 enabled, 11-13 id, 14-19 minute, 20-24 hour). */
    uint64_t v = 0;
    v |= ((uint64_t)(a->ctx.days & 0x7F));
    v |= ((uint64_t)(a->ctx.state == ALARM_STATE_ENABLE ? 1 : 0)) << 7;
    v |= ((uint64_t)(a->idx & 0x07)) << 11;
    v |= ((uint64_t)(a->ctx.minute & 0x3F)) << 14;
    v |= ((uint64_t)(a->ctx.hour & 0x1F)) << 20;
    /* day/month/year stay zero — watch HW alarm doesn't use them. */

    size_t pos = 5 + alarm_push.count * 5;
    alarm_push.buf[pos + 0] = (uint8_t)(v >> 32);
    alarm_push.buf[pos + 1] = (uint8_t)(v >> 24);
    alarm_push.buf[pos + 2] = (uint8_t)(v >> 16);
    alarm_push.buf[pos + 3] = (uint8_t)(v >> 8);
    alarm_push.buf[pos + 4] = (uint8_t)(v);
    alarm_push.count++;
}

static void alarm_push_finish(void)
{
    uint16_t payload_len = alarm_push.count * 5;
    alarm_push.buf[3] = (uint8_t)((payload_len >> 8) & 0xFF);
    alarm_push.buf[4] = (uint8_t)(payload_len & 0xFF);
    skaiwatch_ble_notify(alarm_push.buf, 5 + payload_len);

    bool retry = alarm_push.dirty;
    alarm_push.active = false;
    alarm_push.dirty = false;
    if (retry) alarm_push_start();
}
#endif /* BSP_USING_COMMUNICATE */

static int alarm_service_callback(data_callback_arg_t *arg)
{
#ifdef BSP_USING_COMMUNICATE
    if (alarm_push.active &&
        ALARMMGR_MSG_GET_ALARM_LIST_NEXT_RSP == arg->msg_id)
    {
        const alarm_msg_t *a = (const alarm_msg_t *)arg->data;
        if (a)
        {
            alarm_push_append(a);
            alarm_push_request_next(a);
        }
        else
        {
            alarm_push_finish();
        }
        return 0;
    }
#endif
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
    uint8_t *body;
    rt_err_t err = RT_EOK;
    body = data_service_init_msg(&msg, ALARMMGR_MSG_GET_ALARM_REQ, 0);
    err = datac_send_msg(alarm_client_handle, &msg);
    RT_ASSERT(RT_EOK == err);
}

void bloc_alarm_push_to_phone(void)
{
#ifdef BSP_USING_COMMUNICATE
    /* Iterate the manager and ship the snapshot to the phone. Coalesces:
       calling rapidly during a burst of UI changes is safe — only the last
       snapshot wins. */
    subscribe_alarm_client();
    alarm_push_start();
#endif
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
