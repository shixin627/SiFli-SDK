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
#include "littlevgl2rtt.h"
#include "bloc_peripheral.h"
#ifdef BSP_USING_BLOC_NOTIFY
#include "bloc_notification.h"
#endif
#include "watch_global_data.h"
#include "watch_system_interact.h"
#include "math.h"

#define DBG_LVL DBG_LOG
#define DBG_TAG "CLIENT.POWER"
#include "rtdbg.h"

/* Alarm */
static datac_handle_t alarm_client_handle;
static alarm_msg_t alarm_msg;
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

static void get_alarm(void)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    body = data_service_init_msg(&msg, ALARMMGR_MSG_GET_ALARM_REQ, 0);
    err = datac_send_msg(alarm_client_handle, &msg);
    RT_ASSERT(RT_EOK == err);
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
