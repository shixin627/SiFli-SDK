/**
 ******************************************************************************
 * @file   acce_service
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
/* Standard includes */
#include <string.h>

/* RT-Thread includes */
#include "acce_service.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <board.h>

/* Project specific includes */
#include "data_service_provider.h"
#include "sensor.h"
#include "bloc_peripheral.h"

/* Feature specific includes */
#ifdef BSP_USING_HAND_TRACKING
    #include "hand_tracking.h"
#endif
#ifdef BSP_USING_HEALTH_ALGO
    #include "health_algo.h"
#endif

/* Hardware specific includes */
#ifdef ACC_USING_BMI270
    #include "common/sensor_bmi270.h"
    #define ACCE_MODEL_NAME "bmi270"
#elif defined(ACC_USING_LSM6DSL)
    #include "sensor_st_lsm6dsl.h"
    #define ACCE_MODEL_NAME "lsm6dsl"
#endif

#define DBG_TAG "DS.ACC"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

#define ACCE_DEV_NAME "acce_" ACCE_MODEL_NAME
#define ACCE_TIMER_PERIOD_MS (10)

/**
 * @brief Accelerator service environment structure
 */
typedef struct
{
    int ref_count;             /* Reference counter for subscriptions */
    rt_device_t device;        /* Accelerometer device */
    datas_handle_t service;    /* Data service handle */
    motion_sensor_data_t data; /* Current sensor data */
    rt_timer_t timer;          /* Periodic timer for data sampling */
    bool is_ready;             /* Service initialization status */
} acce_service_env_t;

/* Global service environment */
static acce_service_env_t acce_service_env;

/**
 * @brief Handle service subscription request
 * @param service Service handle
 * @return Operation result (0 = success)
 */
static int32_t acce_subscribe(datas_handle_t service)
{
    if (acce_service_env.timer && acce_service_env.ref_count == 0)
    {
        rt_timer_start(acce_service_env.timer);
    }

    acce_service_env.ref_count++;
    LOG_I("Accelerometer subscribed, ref_count %d", acce_service_env.ref_count);
    return RT_EOK;
}

/**
 * @brief Handle service unsubscription request
 * @param service Service handle
 * @return Operation result (0 = success)
 */
static int32_t acce_unsubscribe(datas_handle_t service)
{
    acce_service_env.ref_count--;

    if (acce_service_env.timer && acce_service_env.ref_count == 0)
    {
        rt_timer_stop(acce_service_env.timer);
    }

    LOG_I("Accelerometer unsubscribed, ref_count %d",
          acce_service_env.ref_count);
    return RT_EOK;
}

/**
 * @brief Configure accelerometer service
 * @param service Service handle
 * @param config Configuration data
 * @return Operation result
 */
static rt_err_t acce_service_config(datas_handle_t service, uint8_t *config)
{
    if (!config)
    {
        return -RT_ERROR;
    }

    // Update service configuration. Save config for future filtering.
    return RT_EOK;
}

/**
 * @brief Ping accelerometer device for self-test
 * @param service Service handle
 * @param data Response data buffer
 * @return Operation result
 */
static rt_err_t acce_service_ping(datas_handle_t service, uint8_t *data)
{
    acce_service_env_t *env = &acce_service_env;

    if (!env->device)
    {
        LOG_E("Accelerometer device not available");
        return -RT_ERROR;
    }

    return rt_device_control(env->device, RT_SENSOR_CTRL_SELF_TEST, data);
}

/**
 * @brief Fetch IMU data from hardware sensors
 * @param data Pointer to data structure to be filled
 */
static void imu_data_fetch(motion_sensor_data_t *data)
{
    if (!data)
    {
        LOG_E("Invalid data pointer in imu_data_fetch");
        return;
    }

    data->timestamp = rt_sensor_get_ts();

    // IMU data
#ifdef ACC_USING_BMI270
    struct bmi2_sens_axes_data *imu = bmi270_get_accel();
#endif
    data->imu.timestamp = watch_sensor.imu_data.timestamp;
    data->imu.sample_rate = watch_sensor.imu_data.sample_rate;
    data->imu.acce = watch_sensor.imu_data.acce;
    data->imu.gyro = watch_sensor.imu_data.gyro;

    // Motion data
    data->motion.timestamp = watch_sensor.motion_data.timestamp;
    data->motion.linear_acce = watch_sensor.motion_data.linear_acce;
    data->motion.gravity = watch_sensor.motion_data.gravity;
    data->motion.global_q = watch_sensor.motion_data.global_q;
    data->motion.sensor_q = watch_sensor.motion_data.sensor_q;
}

/**
 * @brief Fetch accelerometer data for service clients
 * @param service Service handle
 * @param data_size Expected data size
 * @param data Pointer to data buffer pointer
 * @return Size of fetched data or negative error code
 */
static int32_t acce_service_data_fetch(datas_handle_t service,
                                       uint32_t data_size, uint8_t **data)
{
    acce_service_env_t *env = &acce_service_env;

    if (!data)
    {
        LOG_E("Invalid data pointer");
        return -RT_ERROR;
    }

    *data = NULL;

    if (!env->device)
    {
        LOG_E("Accelerometer device not available");
        return -RT_ERROR;
    }

    *data = (uint8_t *)&env->data;
    imu_data_fetch(&env->data);

    return sizeof(env->data);
}

/**
 * @brief Filter function for checking compatibility of data requests
 * @param config Data request configuration
 * @param msg_id Message ID
 * @param len Data length
 * @param data Data pointer
 * @return Whether the request passes the filter (true = pass)
 */
bool acce_service_filter(data_req_t *config, uint16_t msg_id, uint32_t len,
                         uint8_t *data)
{
    // Check if config is compatible with current config.
    return true;
}

/**
 * @brief Set power mode for accelerometer
 * @param power_mode Power mode to set
 */
void acce_set_power(uint32_t power_mode)
{
    acce_service_env_t *env = &acce_service_env;
    rt_err_t ret;

    if (!env->device)
    {
        LOG_E("Cannot set power - device not initialized");
        return;
    }

#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif /* RT_USING_PM */

    LOG_I("Setting accelerometer power mode to %d", power_mode);
    ret = rt_device_control(env->device, RT_SENSOR_CTRL_SET_POWER,
                            (void *)power_mode);

    if (ret != RT_EOK)
    {
        LOG_E("Failed to set power mode %d, error: %d", power_mode, ret);
    }

#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif /* RT_USING_PM */
}

/**
 * @brief Message handler for accelerometer service
 * @param service Service handle
 * @param msg Message pointer
 * @return Operation result
 */
static int32_t acce_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    if (!msg)
    {
        LOG_E("Null message received in accelerometer service");
        return -RT_ERROR;
    }

    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
        return acce_subscribe(service);

    case MSG_SERVICE_UNSUBSCRIBE_REQ:
        return acce_unsubscribe(service);

    case MSG_SERVICE_CONFIG_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        if (!req)
        {
            LOG_E("Invalid config request");
            datas_send_response(service, msg, -RT_ERROR);
            return -RT_ERROR;
        }
        rt_err_t result = acce_service_config(service, &req->data[0]);
        datas_send_response(service, msg, result);
        return result;
    }

    case MSG_SERVICE_PING_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        if (!req)
        {
            LOG_E("Invalid ping request");
            datas_send_response(service, msg, -RT_ERROR);
            return -RT_ERROR;
        }
        rt_err_t result = acce_service_ping(service, &req->data[0]);
        datas_send_response(service, msg, result);
        return result;
    }

    case MSG_SERVICE_TX_REQ:
    case MSG_SERVICE_RX_REQ:
        // Not implemented
        return RT_EOK;

    case MSG_SERVICE_DATA_RDY_IND:
    {
        data_rdy_ind_t *data_ind =
            (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        uint8_t *data = NULL;

        if (!data_ind)
        {
            LOG_E("Invalid data ready indication");
            return -RT_ERROR;
        }

        int32_t size = acce_service_data_fetch(service, data_ind->len, &data);
        if (size > 0 && data != NULL)
        {
            datas_push_data_to_client(service, data_ind->len, data);
        }
        return RT_EOK;
    }

    default:
        LOG_E("Unknown message ID %d in accelerometer service", msg->msg_id);
        return -RT_ERROR;
    }
}

/**
 * @brief Data service callback configuration
 */
static data_service_config_t acce_service_cb = {
    .max_client_num = 1,
    .queue = RT_NULL,
    .data_filter = acce_service_filter,
    .msg_handler = acce_service_msg_handler,
};

/**
 * @brief Notify clients about new accelerometer data
 */
void notify_acce_data(void)
{
    if (acce_service_env.service && acce_service_env.ref_count > 0)
    {
        datas_ind_size(acce_service_env.service, sizeof(acce_service_env.data));
    }
}

/**
 * @brief Timer callback for periodic data sampling
 * @param param User parameter
 */
static void timeout_ind(void *param)
{
    if (is_sleep_mode())
    {
        return;
    }
    notify_acce_data();
}

/**
 * @brief Check if accelerometer service is ready
 * @return Initialization status
 */
bool is_acce_service_ready(void)
{
    return acce_service_env.is_ready;
}

/**
 * @brief Initialize and register accelerometer service
 * @return Operation result
 */
int acce_service_register(void)
{
    struct rt_sensor_config cfg;
    acce_service_env_t *env = &acce_service_env;
    rt_err_t err;
    int res = RT_EOK;

    // Initialize service environment
    memset(env, 0, sizeof(acce_service_env_t));

    LOG_I("Initializing accelerometer service (%s)", ACCE_MODEL_NAME);

    // Initialize hardware
#ifdef ACC_USING_BMI270
    res = rt_hw_bmi270_init();
#elif defined(ACC_USING_LSM6DSL)
    res = rt_hw_lsm6dsl_init();
#endif
    if (res != RT_EOK)
    {
        LOG_E("Hardware accelerometer init (%s) failed: %d", ACCE_MODEL_NAME,
              res);
        return -RT_ERROR;
    }

    // Register sensor driver
#ifdef ACC_USING_BMI270
    cfg.intf.dev_name = BMI270_BUS_NAME;
#ifdef BMI270_INT_GPIO_BIT
    cfg.irq_pin.pin = BMI270_INT_GPIO_BIT;
#elif defined(IMU_INT_PIN)
    cfg.irq_pin.pin = IMU_INT_PIN;
#endif
    res = rt_hw_bmi270_register(ACCE_MODEL_NAME, &cfg);
#elif defined(ACC_USING_LSM6DSL)
    cfg.intf.dev_name = LSM6DSL_BUS_NAME;
    res = rt_hw_lsm6dsl_register(ACCE_MODEL_NAME, &cfg);
#endif
    if (res != RT_EOK)
    {
        LOG_E("Failed to register %s driver: %d", ACCE_MODEL_NAME, res);
        return -RT_ERROR;
    }

    // Open device
    env->device = rt_device_find(ACCE_DEV_NAME);
    if (env->device == NULL)
    {
        LOG_E("Failed to find device %s", ACCE_DEV_NAME);
        return -RT_ERROR;
    }

    err = rt_device_open(env->device, RT_DEVICE_OFLAG_RDONLY);
    if (err != RT_EOK)
    {
        LOG_E("Failed to open device %s: %d", ACCE_DEV_NAME, err);
        return -RT_ERROR;
    }

    // Register data service
    env->service = datas_register("ACCE", &acce_service_cb);
    if (env->service == NULL)
    {
        LOG_E("Failed to register accelerometer data service");
        rt_device_close(env->device);
        return -RT_ERROR;
    }

    // Create periodic timer
    env->timer = rt_timer_create("ACCE", timeout_ind, 0,
                                 rt_tick_from_millisecond(ACCE_TIMER_PERIOD_MS),
                                 RT_TIMER_FLAG_PERIODIC);
    if (env->timer == NULL)
    {
        LOG_E("Failed to create accelerometer timer");
        rt_device_close(env->device);
        // Should free data service here
        return -RT_ERROR;
    }

    // Register with peripheral provider
    // peripheral_provider.imu_set_power = acce_set_power;

#ifdef BSP_USING_HEALTH_ALGO
    health_algo_init();
#endif

    env->is_ready = true;
    LOG_I("Accelerometer service initialized successfully");

    return RT_EOK;
}

int close_acce_service(void)
{
    acce_service_env_t *env = &acce_service_env;

    if (env->timer)
    {
        rt_timer_delete(env->timer);
        env->timer = NULL;
    }

    if (env->service)
    {
        // Should unregister data service here
        env->service = NULL;
    }

    if (env->device)
    {
        rt_device_close(env->device);
        env->device = NULL;
    }

    env->is_ready = false;
    LOG_I("Accelerometer service closed");

    return RT_EOK;
}

INIT_COMPONENT_EXPORT(acce_service_register);
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF
 * FILE****/