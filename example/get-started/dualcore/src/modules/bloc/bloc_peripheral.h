/**
*****************************************************************************************
*     Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
*****************************************************************************************
* @file      bloc_peripheral.h
* @brief     business logic of peripheral
* @author    Jack
* @date      2024-02-21
* @version   v1.0
**************************************************************************************
*/

#ifndef __BLOC_PERIPHERAL_H__
#define __BLOC_PERIPHERAL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include <rtthread.h>
#include <sensor.h>
#include "bloc_motor.h"
#include "bsp_board.h"
#include "watch_global_data.h"

    typedef struct
    {
        rt_uint32_t timestamp;
        Vector3 acce;
        Vector3 gyro;
        Vector3 mag;
        float sample_rate;
    } imu_sensor_data_t;

    typedef struct
    {
        uint32_t timestamp;
        uint32_t raw_data[2];
        uint8_t sample_num;
    } ppg_sensor_data_t;
    
    typedef struct
    {
        rt_uint32_t timestamp;
        Vector3 linear_acce;
        Vector3 gravity;
        Quaternion global_q;
        Quaternion sensor_q;
        ppg_sensor_data_t ppg_raw_data;
    } motion_data_t;

    typedef struct hr_sensor_data
    {
        rt_uint32_t timestamp;
        int hr;
    } hr_sensor_data_t;

    typedef struct
    {
        uint32_t timestamp;
        uint8_t sample_num;
        watch_sys_linear_acce_t dataset[MAX_RAWDATA_TIME_STEP];
        watch_sys_linear_acce_ppg_t dataset_ppg[16];
    } gesture_data_t;

#if defined(SOC_BF0_HCPU)
    #define ENABLE_IMU_SEM_FIFO 1
#else
    #define ENABLE_IMU_SEM_FIFO 0
#endif

    typedef struct
    {
        bool is_imu_enabled;
        int imu_client_num;
        bool imu_abnormal;
        imu_sensor_data_t imu_data;
        gesture_data_t gesture_data;
        motion_data_t motion_data;
        bool is_ppg_enabled;
        int ppg_client_num;
        ppg_sensor_data_t ppg_data;
        ppg_sensor_data_t ppg_data2;
        int hr_client_num;
        hr_sensor_data_t hr_data;

#if ENABLE_IMU_SEM_FIFO
        rt_sem_t imu_sem;
#endif

        rt_sem_t gesture_sem;
    } watch_sensor_t;

    typedef struct
    {
        rt_uint32_t timestamp;
        imu_sensor_data_t imu;
        motion_data_t motion;
        // ppg_sensor_data_t ppg;
    } motion_sensor_data_t;

    // sensor type
    typedef enum
    {
        SENSOR_IMU = 0,
        SENSOR_PPG,
        SENSOR_AUDIO_MIC,
        SENSOR_BATTERY,
        SENSOR_MAGNETOMETER,
    } sensor_type_t;

    extern watch_sensor_t watch_sensor;
    extern void process_motion_sensor_data(motion_sensor_data_t *data);
    extern void motion_data_fetch(motion_data_t *data);
    extern void process_ppg_sensor_data(uint8_t sample_num, uint32_t *data,
                                        uint32_t *data2);

    typedef struct
    {
        bool (*get_tap_status)(void);
        void (*set_tap_status)(bool status);
#ifndef SOC_BF0_LCPU
        void (*hcpu_reboot)(void);
        void (*hcpu_resume)(void);
        void (*hcpu_suspend)(void);
        void (*subscribe_system_service)(bool status);
        void (*subscribe_accelerometer_sensor)(bool status);
        void (*subscribe_hr_sensor)(bool status);
        void (*subscribe_ppg_signal)(bool status);
        void (*control_motor)(bool enable, motor_params_t *params);
        // audio
        void (*subscribe_audio_mic_sensor)(bool status);
        // save shared prefs
        void (*save_watch_shared_prefs)(watch_prefs_key key);
        void (*notify_battery_voltage)(uint16_t voltage_mv);
        void (*charge_status_callback)(uint8_t status);
#else

    void (*lift_status_callback)(uint8_t status);
    void (*soft_adt_callback)(bool status);
    void (*gesture_callback)(uint32_t event);
    void (*sensor_power_manage)(uint8_t type, uint32_t status);

    void (*imu_set_power)(rt_uint32_t power_mode);
#endif

        void (*hr_set_power)(uint8_t status);
    } PeripheralProvider;
    extern PeripheralProvider peripheral_provider;

    /// @brief peripheral event
    typedef enum
    {
        SUBSCRIBE_ACCELEROMETER = 0,
        SUBSCRIBE_HR,
        SUBSCRIBE_PPG,
        SUBSCRIBE_AUDIO_MIC,
        HCPU_REBOOT,
        HCPU_RESUME,
        HCPU_SUSPEND,
        POWER_MANAGE_IMU,
        POWER_MANAGE_HR,
        CONTROL_MOTOR,
        SAVE_SHARE_PREFS,
        NOTIFY_BATTERY_VOLTAGE,
        CHARGE_STATUS_CALLBACK,
    } PeripheralEvent;

    /// @brief peripheral message arg
    typedef union
    {
        bool subscribe_status;
        bool audio_record_toggle;
        bool audio_play_toggle;
        uint16_t value;
        uint32_t data;
        struct
        {
            bool enable;
            motor_params_t params;
        } motor_control;
    } PeripheralArg;
    /// @brief peripheral message data
    typedef struct
    {
        PeripheralEvent event;
        PeripheralArg arg;
    } PeripheralMessageData;

    struct wheel_driver
    {
        rt_bool_t (*probe)(void);
        rt_sem_t isr_sem;
        void *user_data;
    };

    typedef struct wheel_driver *wheel_drv_t;
    bool is_sleep_mode(void);
    bool is_hcpu_suspend(void);
    void set_sleep_mode(bool mode);
    /* Weak hook the LCPU app can override to react to screen-state changes
       (e.g. switch raise-wrist source). See bloc_peripheral.c. */
    void on_lcpu_sleep_mode_changed(bool sleep);

    extern bool get_enable_tap_and_hold(void);
    extern void set_enable_tap_and_hold(bool enable);

#ifdef SOC_BF0_LCPU

    void set_imu_data_collection(bool enable);
    void set_imu_rawdata_collection(bool enable);

    #define GSENSOR_FIFO_BUFFER_SIZE 25
    extern int16_t gsensor_fifo_buffer[GSENSOR_FIFO_BUFFER_SIZE][3];
    extern uint16_t gsensor_fifo_buffer_index;
#else

extern void accelerometer_subscribe(void);
extern void accelerometer_unsubscribe(void);
extern void heart_rate_subscribe(void);
extern void heart_rate_unsubscribe(void);
extern void hr_data_handler(hr_sensor_data_t *data);
extern void ppg_subscribe(void);
extern void ppg_unsubscribe(void);
extern void audio_subscribe(void);
extern void audio_unsubscribe(void);
extern bool get_motor_status(void);
extern time_t get_current_time(void);
extern void drv_reboot(void);

#ifdef USING_FSR_ADC_SAMPLER
    /* FSR-402 ADC HW driver (HCPU only). Implementation lives in
     * bloc_peripheral.c; sampling thread that consumes these lives in
     * bloc_control.c. */
    void fsr_adc_init(void);
    rt_uint32_t fsr_adc_read_value(void);
#endif
#endif
#ifdef __cplusplus
}
#endif

#endif