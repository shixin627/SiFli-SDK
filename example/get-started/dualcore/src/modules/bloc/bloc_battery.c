/**
 ******************************************************************************
 * @file   bloc_battery.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "watch_sys_service.h"
#include "bloc_battery.h"
#include "bloc_peripheral.h"
#include "battery_calculator.h"
#include "charge.h"

#define DBG_TAG "bloc.battery"
#include "bsp_board.h"
#define DBG_LVL BSP_DBG_LVL
#include "rtdbg.h"

#ifdef BSP_USING_ADC
    #define ADC_DEV_NAME "bat1"
    #define ADC_DEV_CHANNEL 5

    #ifdef USING_BATTERY_ADC_LOW_ACCURACY
        #define BATTERY_VOLTAGE_RATIO_NUM 100
        #define BATTERY_VOLTAGE_RATIO_DEN 100
    #endif

    #ifdef USING_BATTERY_ADC_HIGH_ACCURACY
        // R1=470K, R2=1M, ratio = (R1+R2)/R2 = 1.47
        #define BATTERY_VOLTAGE_RATIO_NUM 147
        #define BATTERY_VOLTAGE_RATIO_DEN 100
    #endif

static battery_calculator_t s_calculator;
static bool s_calculator_initialized = false;

static uint32_t read_battery_voltage(void)
{
    rt_device_t adc_dev = rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        LOG_E("Can't find adc device %s", ADC_DEV_NAME);
        return 0;
    }

    rt_err_t r = rt_adc_enable((rt_adc_device_t)adc_dev, ADC_DEV_CHANNEL);
    if (r != RT_EOK)
    {
        LOG_E("Enable adc channel %d fail", ADC_DEV_CHANNEL);
        return 0;
    }

    uint32_t value = rt_adc_read((rt_adc_device_t)adc_dev, ADC_DEV_CHANNEL);
    rt_adc_disable((rt_adc_device_t)adc_dev, ADC_DEV_CHANNEL);

    if (value == 0)
    {
        LOG_E("ADC read returned 0");
        return 0;
    }

    // value is in 0.1mV, apply voltage divider ratio using integer math
    uint32_t battery_voltage = (value * BATTERY_VOLTAGE_RATIO_NUM) / BATTERY_VOLTAGE_RATIO_DEN;

    LOG_D("ADC raw: %d (0.1mV), battery: %d (0.1mV)", value, battery_voltage);
    return battery_voltage;
}

static battery_calculator_config_t s_calculator_config;

void bloc_battery_init(void)
{
    extern const battery_lookup_point_t discharge_curve_table[];
    extern const battery_lookup_point_t charging_curve_table[];
    extern const uint32_t discharge_curve_table_size;
    extern const uint32_t charging_curve_table_size;

    s_calculator_config.charging_table = charging_curve_table;
    s_calculator_config.charging_table_size = charging_curve_table_size;
    s_calculator_config.discharging_table = discharge_curve_table;
    s_calculator_config.discharging_table_size = discharge_curve_table_size;
    s_calculator_config.charge_filter_threshold = 200;      // 20mV in 0.1mV units
    s_calculator_config.discharge_filter_threshold = 200;   // 20mV in 0.1mV units
    s_calculator_config.filter_count = 3;
    s_calculator_config.secondary_filter_enabled = true;
    s_calculator_config.secondary_filter_weight_pre = 80;
    s_calculator_config.secondary_filter_weight_cur = 20;

    int ret = battery_calculator_init(&s_calculator, &s_calculator_config);
    if (ret != BATTERY_CALC_SUCCESS)
    {
        LOG_E("Battery calculator init failed: %d", ret);
        return;
    }
    s_calculator_initialized = true;
    LOG_I("Battery calculator initialized");
}

#endif // BSP_USING_ADC

/// ************** Charging Detect ************** ///
extern void main_send_read_charge_status_event(void);
extern void main_send_read_voltage_event(void);

#define CHARGING_VOLTAGE_READ_INTERVAL_MS 1000

static rt_timer_t s_charging_voltage_timer = RT_NULL;

static void charging_voltage_timer_callback(void *parameter)
{
    main_send_read_voltage_event();
}

static void charging_voltage_timer_start(void)
{
    if (s_charging_voltage_timer == RT_NULL)
    {
        s_charging_voltage_timer = rt_timer_create(
            "chg_vol",
            charging_voltage_timer_callback,
            RT_NULL,
            rt_tick_from_millisecond(CHARGING_VOLTAGE_READ_INTERVAL_MS),
            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    }
    if (s_charging_voltage_timer)
    {
        rt_timer_start(s_charging_voltage_timer);
        LOG_D("Charging voltage timer started (interval: %dms)", CHARGING_VOLTAGE_READ_INTERVAL_MS);
    }
}

static void charging_voltage_timer_stop(void)
{
    if (s_charging_voltage_timer)
    {
        rt_timer_stop(s_charging_voltage_timer);
        LOG_D("Charging voltage timer stopped");
    }
}

void bloc_battery_read_charge_status(void)
{
    main_send_read_charge_status_event();
}

void bloc_battery_read_voltage(void)
{
    main_send_read_voltage_event();
}

BatteryChargeState battery_charge_state = {0};

BatteryChargeState *battery_get_charge_state(void)
{
    return &battery_charge_state;
}

static void read_charge_status(void)
{
    battery_charger_status_t status = battery_get_charging_status();
    bool charging = (status == BATTERY_CHARGER_STATUS_CHARGING);

    LOG_D("[%s] Is charging? %d", __func__, charging);

    if (battery_charge_state.is_charging != charging)
    {
        if (watch_sys_sync.charge_status_callback)
            watch_sys_sync.charge_status_callback(
                charging ? (battery_charge_state.charge_percent == 100 ? 2 : 1)
                         : 0);
    }
    battery_charge_state.is_charging = charging;
    battery_charge_state.is_plugged = charging;

    if (charging)
    {
        // read voltage immediately and start periodic 1s timer
        main_send_read_voltage_event();
        charging_voltage_timer_start();
    }
    else
    {
        charging_voltage_timer_stop();
    }
}

static void check_battery_voltage(void)
{
#ifdef BSP_USING_ADC
    if (!s_calculator_initialized)
    {
        LOG_E("Battery calculator not initialized");
        return;
    }

    uint32_t battery_voltage = read_battery_voltage();
    if (battery_voltage == 0)
        return;

    uint8_t percentage = battery_calculator_get_percent(&s_calculator, battery_voltage);

    // Convert 0.1mV to mV for reporting
    uint32_t battery_mv = battery_voltage / 10;

    LOG_D("[%s] %d mV, display=%d%%, charging=%d",
          __func__, battery_mv, percentage,
          battery_charge_state.is_charging);

    uint32_t data = (battery_mv << 16) | (percentage & 0xFFFF);
    battery_charge_state.charge_percent = percentage;
    if (watch_sys_sync.notify_battery_voltage)
        watch_sys_sync.notify_battery_voltage(data);
#endif
}

void bloc_battery_handle_charging_event(void)
{
    read_charge_status();
}

void bloc_battery_handle_voltage_event(void)
{
    check_battery_voltage();
}
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF *
 * FILE****/
