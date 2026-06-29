
#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <board.h>
#include "data_service_provider.h"
#include "sensor.h"
#ifdef BSP_USING_BLOC_SKAIWALK
#include "bloc_skaiwalk.h"
#endif
#ifdef BSP_USING_BLOC_PERIPHERAL
#include "bloc_peripheral.h"
#endif
#ifdef BSP_USING_BLOC_NOTIFY
#include "bloc_notification.h"
#endif
#ifdef BSP_USING_COMMUNICATE
#include "communicate_protocol.h"
#endif
#include "watch_system_interact.h"
#ifdef BSP_USING_WATCH_SYS_CLIENT
#include "watch_sys_service.h"
#endif

#define DBG_LVL DBG_LOG
#define DBG_TAG "CLIENT.HR"
#include "rtdbg.h"

static datac_handle_t hr_service_handle;
static datac_handle_t ppg_service_handle;
static hr_sensor_data_t client_hr_data;

static bool check_ppg_sensor(void)
{
#ifdef BSP_USING_WATCH_SYS_CLIENT
    if (!watch_sys_sync.is_ppg_enabled())
    {
        LOG_E("ppg sensor is not enabled\n");
        return false;
    }
#endif
    return true;
}

static void process_hr_sensor_data(struct rt_sensor_data *data)
{
    if (data->type == RT_SENSOR_CLASS_HR)
    {
        client_hr_data.timestamp = data->timestamp;
        client_hr_data.hr = data->data.hr;
        hr_data_handler(&client_hr_data);
        memset(&client_hr_data, 0, sizeof(client_hr_data));
    }
}

static int hr_callback(data_callback_arg_t *arg)
{
    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        uint16_t len;
        uint8_t *buffer;
        RT_ASSERT(arg->data);
        len = arg->data_len;
        buffer = arg->data;
        uint8_t buf_len = len / sizeof(struct rt_sensor_data);
        if (buf_len == 1)
        {
            struct rt_sensor_data object = *(struct rt_sensor_data *)buffer;
            if (len != sizeof(object))
            {
                return -RT_ERROR;
            }
            process_hr_sensor_data(&object);
        }
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        LOG_I("Subscribe HR id(%d), result(%d)\r\n", arg->msg_id, rsp->result);
        if (rsp->result == 0)
        {
            watch_sensor.hr_client_num++;
            SkaiWatchSys.hrs_start_up_mode = 1;
        }
        else
        {
            LOG_E("Subscribe HR failed\n");
        }
    }
    else if (MSG_SERVICE_UNSUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        LOG_I("Unsubscribed HR id(%d), result(%d)\r\n", arg->msg_id, rsp->result);
        if (rsp->result == 0)
        {
            watch_sensor.hr_client_num--;
            SkaiWatchSys.hrs_start_up_mode = 0;
        }
        else
        {
            LOG_E("Unsubscribe HR failed\n");
        }
    }
    return RT_EOK;
}

void heart_rate_subscribe(void)
{
    if (hr_service_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        hr_service_handle = datac_open();
    }
    if (hr_service_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        datac_subscribe(hr_service_handle, "HR", hr_callback, 0);
        LOG_I("Subscribe HR\n");
    }
}

void heart_rate_unsubscribe(void)
{
    if (hr_service_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("hr_service_handle is invalid\n");
        return;
    }
    if (datac_unsubscribe(hr_service_handle) == RT_EOK)
    {
        LOG_I("Unsubscribe HR\n");
    }
    else
    {
        LOG_E("hr service unsubscribe failed\n");
    }
}

#if !kReleaseMode
MSH_CMD_EXPORT(heart_rate_subscribe, Subscribe HR data service);
MSH_CMD_EXPORT(heart_rate_unsubscribe, Unsubscribe HR data service);
#endif

#if !kReleaseMode
#define PPG_FIFO_LENGTH 4
static int ppg_callback(data_callback_arg_t *arg)
{
    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        uint16_t len;
        uint8_t *buffer;
        RT_ASSERT(arg->data);
        len = arg->data_len;
        buffer = arg->data;
        if (len == PPG_FIFO_LENGTH * sizeof(uint32_t))
        {
            uint8_t sample_num = PPG_FIFO_LENGTH / 2; // 2 channels
            uint32_t ppg1[PPG_FIFO_LENGTH / 2];
            uint32_t ppg2[PPG_FIFO_LENGTH / 2];
            for (int i = 0; i < sample_num; i++)
            {
                ppg1[i] = *(uint32_t *)(buffer + 2 * i * sizeof(uint32_t));
                ppg2[i] = *(uint32_t *)(buffer + (2 * i + 1) * sizeof(uint32_t));
            }
            process_ppg_sensor_data(sample_num, ppg1, ppg2);
        }
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        LOG_I("Subscribe PPG id(%d), result(%d)\r\n", arg->msg_id, rsp->result);
        if (rsp->result == 0)
        {
            watch_sensor.ppg_client_num++;
            // LOG_D("Subscribe PPG client_num:%d\n", watch_sensor.ppg_client_num);
        }
        else
        {
            LOG_E("Subscribe PPG failed\n");
        }
    }
    else if (MSG_SERVICE_UNSUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        LOG_I("Unsubscribed PPG id(%d), result(%d)\r\n", arg->msg_id, rsp->result);
        if (rsp->result == 0)
        {
            watch_sensor.ppg_client_num--;
            // LOG_D("Unsubscribe PPG client_num:%d\n", watch_sensor.ppg_client_num);
            // datac_close(ppg_service_handle);
            // ppg_service_handle = DATA_CLIENT_INVALID_HANDLE;
        }
        else
        {
            LOG_E("Unsubscribe PPG failed\n");
        }
    }
    return RT_EOK;
}

bool ppg_service_subscribed(void)
{
    return (ppg_service_handle != DATA_CLIENT_INVALID_HANDLE) && (watch_sensor.ppg_client_num > 0);
}

void ppg_subscribe(void)
{
    if (ppg_service_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        ppg_service_handle = datac_open();
    }
    if (ppg_service_handle != DATA_CLIENT_INVALID_HANDLE)
    {
        datac_subscribe(ppg_service_handle, "PPG", ppg_callback, 0);
        LOG_I("Subscribe PPG\n");
    }
}

void ppg_unsubscribe(void)
{
    if (ppg_service_handle == DATA_CLIENT_INVALID_HANDLE)
    {
        LOG_E("ppg service is not subscribed\n");
        return;
    }
    if (datac_unsubscribe(ppg_service_handle) == RT_EOK)
    {
        LOG_I("Unsubscribe PPG\n");
    }
    else
    {
        LOG_E("ppg service unsubscribe failed\n");
    }
}

MSH_CMD_EXPORT(ppg_subscribe, Subscribe PPG data service);
MSH_CMD_EXPORT(ppg_unsubscribe, Unsubscribe PPG data service);
#endif
