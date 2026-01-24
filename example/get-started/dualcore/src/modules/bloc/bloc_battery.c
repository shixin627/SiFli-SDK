/**
 ******************************************************************************
 * @file   bloc_battery.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "math.h"
#include "watch_sys_service.h"
#include "bloc_battery.h"
#include "bloc_peripheral.h"

#define DBG_TAG "bloc.battery"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

#ifdef BSP_USING_ADC
    #define ADC_DEV_NAME "bat1" /* ADC device name */
    #define ADC_DEV_CHANNEL 5   /* ADC channel */

static rt_device_t s_adc_dev; /* 定义一个rt_device设备 */
static rt_adc_cmd_read_arg_t read_arg;

static uint32_t read_battery_voltage(void)
{
    rt_err_t r;
    s_adc_dev = rt_device_find(ADC_DEV_NAME);
    if (s_adc_dev == RT_NULL)
    {
        LOG_E("Can't find adc device %s\n", ADC_DEV_NAME);
        return -RT_ERROR;
    }

    read_arg.channel = ADC_DEV_CHANNEL;

    r = rt_adc_enable((rt_adc_device_t)s_adc_dev, read_arg.channel);
    if (r != RT_EOK)
    {
        LOG_E("Enable adc channel %d fail\n", read_arg.channel);
        return r;
    }

    r = rt_device_control(s_adc_dev, RT_ADC_CMD_READ, &read_arg.channel);
    if (r != RT_EOK)
    {
        LOG_E("ADC read control fail\n");
        rt_adc_disable((rt_adc_device_t)s_adc_dev, read_arg.channel);
        return r;
    }
    LOG_I("adc channel:%d,value:%d", read_arg.channel,
          read_arg.value); /* (0.1mV), 20846 is 2084.6mV or 2.0846V */

    rt_uint32_t value =
        rt_adc_read((rt_adc_device_t)s_adc_dev, ADC_DEV_CHANNEL);
    LOG_I("rt_adc_read:%d,value:%d", read_arg.channel,
          value); /* (0.1mV), 20700 is 2070mV or 2.070V */

    rt_adc_disable((rt_adc_device_t)s_adc_dev, read_arg.channel);

    RT_ASSERT(value > 0);
    return value;
}

    #ifdef USING_BATTERY_ADC_LOW_ACCURACY
        #define BATTERY_VOLTAGE_RATIO 1.0
    #endif

    #ifdef USING_BATTERY_ADC_HIGH_ACCURACY
        // R1=470K, R2=1M
        // 分壓R1+R2/R2=1.47
        #define BATTERY_VOLTAGE_RATIO 1.47
    #endif

static uint32_t convert_adc_to_battery_mv(uint32_t adc_value)
{
    double gpio_mv = adc_value * 0.1;
    double battery_mv = gpio_mv * BATTERY_VOLTAGE_RATIO;
    return (uint32_t)(battery_mv);
}

    // 電池電壓範圍常量
    #define MIN_VOLTAGE_DISCHARGE 3462
    #define MAX_VOLTAGE_DISCHARGE 4004
    #define MIN_VOLTAGE_CHARGING 3750
    #define MAX_VOLTAGE_CHARGING 4120

    // 充電時判定充飽的電壓閾值
    #define FULL_CHARGE_VOLTAGE_THRESHOLD 4120

    // 顯示平滑參數
    #define MAX_PERCENTAGE_CHANGE_PER_MINUTE 2 // 每分鐘最多變化2%

// 电池放电曲线 (OCV-SOC曲线，101个点，从100%到0%)
// 这是基于您提供的实测数据：静置状态下的开路电压
static const int battery_discharge_curve[101] = {
    4004, 3985, 3973, 3960, 3948, 3936, 3927, 3915, 3906, 3898, // 100-91%
    3891, 3885, 3879, 3871, 3864, 3857, 3850, 3844, 3837, 3832, // 90-81%
    3824, 3818, 3812, 3807, 3800, 3794, 3790, 3784, 3779, 3774, // 80-71%
    3769, 3764, 3760, 3755, 3751, 3747, 3743, 3739, 3735, 3731, // 70-61%
    3728, 3726, 3722, 3718, 3716, 3712, 3709, 3706, 3703, 3700, // 60-51%
    3698, 3695, 3693, 3690, 3688, 3686, 3684, 3683, 3681, 3679, // 50-41%
    3678, 3677, 3674, 3673, 3671, 3670, 3668, 3667, 3664, 3661, // 40-31%
    3659, 3657, 3654, 3651, 3648, 3644, 3640, 3636, 3631, 3628, // 30-21%
    3622, 3618, 3612, 3606, 3601, 3594, 3587, 3580, 3573, 3566, // 20-11%
    3557, 3550, 3541, 3533, 3523, 3514, 3504, 3495, 3484, 3473, // 10-1%
    3462                                                        // 0%
};

// 电池充电曲线 (充电中的电压-SOC曲线，101个点，从0%到100%)
// 用于初始化时如果正在充电，使用此曲线获得更准确的初始电量
static const int battery_charge_curve[101] = {
    3750, 3791, 3820, 3845, 3871, 3889, 3905, 3919, 3931, 3941, // 0-9%
    3951, 3961, 3968, 3973, 3978, 3982, 3987, 3990, 3994, 3998, // 10-19%
    4002, 4004, 4007, 4010, 4012, 4015, 4017, 4019, 4021, 4023, // 20-29%
    4025, 4027, 4030, 4032, 4033, 4036, 4037, 4040, 4042, 4044, // 30-39%
    4047, 4049, 4052, 4054, 4056, 4058, 4060, 4063, 4064, 4066, // 40-49%
    4069, 4070, 4073, 4075, 4078, 4080, 4082, 4084, 4086, 4089, // 50-59%
    4091, 4094, 4095, 4099, 4100, 4102, 4104, 4106, 4107, 4108, // 60-69%
    4109, 4109, 4109, 4109, 4110, 4110, 4110, 4111, 4111, 4112, // 70-79%
    4112, 4112, 4112, 4113, 4113, 4113, 4114, 4114, 4115, 4115, // 80-89%
    4115, 4116, 4116, 4116, 4117, 4117, 4117, 4118, 4118, 4119, // 90-99%
    4120                                                        // 100%
};

typedef struct
{
    int displayed_percentage;  // 當前顯示的電量百分比
    int real_percentage;       // 實際計算的電量百分比
    uint32_t last_update_tick; // 上次更新的時間戳 (tick)
    bool initialized;          // 是否已初始化
    bool use_charge_curve;     // 是否使用充電曲線
} BatteryDisplayState;

static BatteryDisplayState display_state = {0};

/**
 * @brief 使用二分查找找到最接近的电压值索引
 * @param voltage 目标电压值
 * @param curve 电压曲线数组
 * @param is_ascending 曲线是否遞增（true=遞增，false=遞減）
 * @return 最接近的索引 (0-100)
 */
static int binary_search_closest_index(int voltage, const int *curve,
                                       bool is_ascending)
{
    int left = 0;
    int right = 100;
    int closest_index = 0;
    int min_diff = 0x7FFFFFFF; // 最大整数

    // 边界检查
    if (is_ascending)
    {
        // 充電曲線：遞增
        if (voltage <= curve[0])
            return 0; // 0%
        if (voltage >= curve[100])
            return 100; // 100%
    }
    else
    {
        // 放電曲線：遞減
        if (voltage >= curve[0])
            return 0; // 100%
        if (voltage <= curve[100])
            return 100; // 0%
    }

    // 二分查找
    while (left <= right)
    {
        int mid = left + (right - left) / 2;
        int current_voltage = curve[mid];

        // 计算与当前电压的差值
        int diff = (voltage > current_voltage) ? (voltage - current_voltage)
                                               : (current_voltage - voltage);

        // 更新最接近的索引
        if (diff < min_diff)
        {
            min_diff = diff;
            closest_index = mid;
        }

        // 如果找到完全匹配，直接返回
        if (diff == 0)
        {
            return mid;
        }

        // 根据电压大小和曲线方向调整搜索范围
        if (is_ascending)
        {
            // 充電曲線：遞增
            if (voltage < current_voltage)
            {
                right = mid - 1;
            }
            else
            {
                left = mid + 1;
            }
        }
        else
        {
            // 放電曲線：遞減
            if (voltage > current_voltage)
            {
                right = mid - 1;
            }
            else
            {
                left = mid + 1;
            }
        }
    }

    return closest_index;
}

/**
 * @brief 根据电压计算实际电量百分比（不含平滑处理）
 * @param voltage 测量到的电压 (mV)
 * @param is_charging 是否正在充電
 * @return 实际电量百分比 (0-100)
 */
static int calculate_real_percentage(int voltage, bool is_charging)
{
    int percentage = 0;

    // 特殊处理：充电中且电压超过充飽閾值，直接返回100%
    if (is_charging && voltage >= FULL_CHARGE_VOLTAGE_THRESHOLD)
    {
        LOG_D("Charging and voltage >= %d mV, battery full (100%%)",
              FULL_CHARGE_VOLTAGE_THRESHOLD);
        return 100;
    }

    // 根據狀態選擇曲線
    if (is_charging && !display_state.initialized)
    {
        // 第一次測量且正在充電，使用充電曲線
        const int *curve = battery_charge_curve;

        // 边界检查
        if (voltage >= MAX_VOLTAGE_CHARGING)
            return 100;
        if (voltage <= MIN_VOLTAGE_CHARGING)
            return 0;

        // 使用二分查找（充電曲線是遞增的）
        int closest_index = binary_search_closest_index(voltage, curve, true);
        percentage = closest_index; // 充電曲線：索引0對應0%，索引100對應100%

        LOG_D("Using charge curve for initialization: %d mV -> %d%%", voltage,
              percentage);
    }
    else
    {
        // 其他情況使用放電曲線（包括：已初始化、不在充電）
        const int *curve = battery_discharge_curve;

        // 边界检查
        if (voltage >= MAX_VOLTAGE_DISCHARGE)
            return 100;
        if (voltage <= MIN_VOLTAGE_DISCHARGE)
            return 0;

        // 使用二分查找（放電曲線是遞減的）
        int closest_index = binary_search_closest_index(voltage, curve, false);
        percentage =
            100 - closest_index; // 放電曲線：索引0對應100%，索引100對應0%

        if (!is_charging)
        {
            LOG_D("Using discharge curve: %d mV -> %d%%", voltage, percentage);
        }
        else
        {
            LOG_D("Using discharge curve (charging but initialized): %d mV -> "
                  "%d%%",
                  voltage, percentage);
        }
    }

    // 确保结果在有效范围内
    if (percentage < 0)
        percentage = 0;
    else if (percentage > 100)
        percentage = 100;

    return percentage;
}

/**
 * @brief 平滑處理電量顯示，限制變化速度
 * @param real_percentage 實際計算的電量百分比
 * @return 經過平滑處理後的顯示電量百分比
 */
static int smooth_battery_percentage(int real_percentage)
{
    uint32_t current_tick = rt_tick_get();

    // 首次初始化
    if (!display_state.initialized)
    {
        display_state.displayed_percentage = real_percentage;
        display_state.real_percentage = real_percentage;
        display_state.last_update_tick = current_tick;
        display_state.initialized = true;
        LOG_D("Battery display initialized at %d%%", real_percentage);
        return real_percentage;
    }

    // 更新實際電量
    display_state.real_percentage = real_percentage;

    // 計算時間差（毫秒）
    uint32_t time_diff_ms = (current_tick - display_state.last_update_tick) *
                            1000 / RT_TICK_PER_SECOND;

    // 計算允許的最大變化量
    // 每分鐘最多變化 MAX_PERCENTAGE_CHANGE_PER_MINUTE %
    // 允許變化 = (時間差ms / 60000ms) * MAX_PERCENTAGE_CHANGE_PER_MINUTE
    int max_change = (time_diff_ms * MAX_PERCENTAGE_CHANGE_PER_MINUTE) / 60000;

    // 至少允許變化1%（避免卡住不動）
    if (max_change < 1 && time_diff_ms > 30000) // 超過30秒至少允許變化1%
    {
        max_change = 1;
    }

    // 計算實際需要變化的量
    int diff = real_percentage - display_state.displayed_percentage;

    // 限制變化速度
    if (diff > 0 && battery_charge_state.is_charging)
    {
        // 充電/電量上升
        if (diff > max_change)
        {
            display_state.displayed_percentage += max_change;
            LOG_D("Battery rising: real=%d%%, display=%d%%, limited by +%d%%",
                  real_percentage, display_state.displayed_percentage,
                  max_change);
        }
        else
        {
            display_state.displayed_percentage = real_percentage;
            LOG_D("Battery rising: real=%d%%, display=%d%% (direct)",
                  real_percentage, display_state.displayed_percentage);
        }
    }
    else if (diff < 0 && !battery_charge_state.is_charging)
    {
        // 放電/電量下降
        if (-diff > max_change)
        {
            display_state.displayed_percentage -= max_change;
            LOG_D("Battery falling: real=%d%%, display=%d%%, limited by -%d%%",
                  real_percentage, display_state.displayed_percentage,
                  max_change);
        }
        else
        {
            display_state.displayed_percentage = real_percentage;
            LOG_D("Battery falling: real=%d%%, display=%d%% (direct)",
                  real_percentage, display_state.displayed_percentage);
        }
    }
    else
    {
        // 沒有變化
        LOG_D("Battery stable: %d%%", display_state.displayed_percentage);
    }

    // 更新時間戳
    display_state.last_update_tick = current_tick;

    // 邊界保護
    if (display_state.displayed_percentage < 0)
        display_state.displayed_percentage = 0;
    if (display_state.displayed_percentage > 100)
        display_state.displayed_percentage = 100;

    return display_state.displayed_percentage;
}

/**
 * @brief 獲取電池電量百分比（對外接口，含平滑處理）
 * @param voltage 測量到的電壓 (mV)
 * @return 經過平滑處理的電量百分比 (0-100)
 */
int get_battery_percentage(int voltage)
{
    // 1. 計算實際電量（根據充電狀態選擇曲線）
    int real_percentage =
        calculate_real_percentage(voltage, battery_charge_state.is_charging);

    // 2. 平滑處理
    int display_percentage = smooth_battery_percentage(real_percentage);

    return display_percentage;
}

#endif // BSP_USING_ADC

/// ************** Charging Detect ************** ///
extern void main_send_read_charge_status_event(void);
extern void main_send_read_voltage_event(void);

void bloc_battery_read_charge_status(void)
{
    main_send_read_charge_status_event();
}

void bloc_battery_read_voltage(void)
{
    main_send_read_voltage_event();
}

BatteryChargeState battery_charge_state = {0};

BatteryChargeState* battery_get_charge_state(void)
{
    return &battery_charge_state;
}

#if defined(CHARGE_DETECT_PIN)
    // 充電中斷防抖動配置
    #define CHARGE_INT_DEBOUNCE_MS 500 // 防抖動時間: 500ms
static rt_timer_t charging_debounce_timer = RT_NULL;
static volatile rt_uint32_t last_charge_int_tick = 0;

static void read_charge_status(void)
{
    int pin_status = rt_pin_read(CHARGE_DETECT_PIN);
    bool charging = !pin_status;
    LOG_D("[%s] Is charging? %d", __func__, charging);
    // Notify the system about the charging status change
    // 0 = not charging, 1 = charging, 2 = full
    if (battery_charge_state.is_charging != charging)
    {
        if (watch_sys_sync.charge_status_callback)
            watch_sys_sync.charge_status_callback(
                charging ? (battery_charge_state.charge_percent == 100 ? 2 : 1)
                         : 0);
    }
    battery_charge_state.is_charging = charging;
    battery_charge_state.is_plugged = charging;
}

// 防抖動定時器回調
static void charging_debounce_timeout(void *parameter)
{
    main_send_read_charge_status_event();
}

static void charging_int_handle(void *args)
{
    rt_uint32_t current_tick;
    rt_uint32_t time_diff_ms;

    current_tick = rt_tick_get();
    time_diff_ms =
        (current_tick - last_charge_int_tick) * 1000 / RT_TICK_PER_SECOND;
    last_charge_int_tick = current_tick;

    // 防抖動: 檢查距離上次中斷的時間
    if (time_diff_ms < CHARGE_INT_DEBOUNCE_MS)
    {
        // 在防抖動時間窗口內，重啟定時器
        if (charging_debounce_timer)
        {
            rt_timer_stop(charging_debounce_timer);
            rt_timer_start(charging_debounce_timer);
        }
        return;
    }

    // 立即發送一次事件
    main_send_read_charge_status_event();

    // 啟動防抖動定時器，如果後續還有中斷會重置定時器
    if (charging_debounce_timer)
    {
        rt_timer_stop(charging_debounce_timer);
        rt_timer_start(charging_debounce_timer);
    }
}

void hal_charging_int_init(void)
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    // 創建防抖動定時器
    if (charging_debounce_timer == RT_NULL)
    {
        charging_debounce_timer =
            rt_timer_create("chg_deb", charging_debounce_timeout, RT_NULL,
                            rt_tick_from_millisecond(CHARGE_INT_DEBOUNCE_MS),
                            RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        if (charging_debounce_timer == RT_NULL)
        {
            LOG_E("Failed to create charging debounce timer\n");
        }
    }

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at Charging LP469\n");
        return;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);

    // int pin cfg
    m.pin = CHARGE_DETECT_PIN;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable charging int
    rt_pin_mode(CHARGE_DETECT_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING_FALLING, charging_int_handle,
                      (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);

    rt_device_close(device);
}
#endif

static void check_battery_voltage(void)
{
    uint32_t adc_value = read_battery_voltage();
    uint32_t battery_mv = convert_adc_to_battery_mv(adc_value);
    int percentage = get_battery_percentage(battery_mv);

    LOG_D("[%s] %d mV, real=%d%%, display=%d%%, charging=%d, initialized=%d",
          __func__, battery_mv, display_state.real_percentage, percentage,
          battery_charge_state.is_charging, display_state.initialized);

    uint32_t data = (battery_mv << 16) | (percentage & 0xFFFF);
    battery_charge_state.charge_percent = percentage;
    if (watch_sys_sync.notify_battery_voltage)
        watch_sys_sync.notify_battery_voltage(data);
}

#if defined(CHARGE_DETECT_PIN)
void bloc_battery_handle_charging_event(void)
{
    read_charge_status();
}
#endif

void bloc_battery_handle_voltage_event(void)
{
    check_battery_voltage();
}

#if defined(CHARGE_DETECT_PIN)
static int charing_detect_register(void)
{
    hal_charging_int_init();
    return 0;
}
INIT_COMPONENT_EXPORT(charing_detect_register);
#endif
/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF *
 * FILE****/