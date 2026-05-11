/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "board.h"
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
    #include "drv_gpio.h"
#endif /* BSP_USING_PM */
#include "button.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "drivers/rt_drv_pwm.h"
#include "bloc_battery.h"
#include "charge.h"
#ifdef ACC_USING_BMI270
    #include "bmi270_driver.h"
#endif
#ifdef BSP_USING_HAND_TRACKING
    #include "hand_tracking.h"
#endif
#ifdef BSP_USING_BLOC_PERIPHERAL
    #include "bloc_peripheral.h"  /* on_lcpu_sleep_mode_changed weak hook */
#endif
#ifdef BSP_KEY1_ACTIVE_HIGH
    #define BUTTON_ACTIVE_POL BUTTON_ACTIVE_HIGH
#else
    #define BUTTON_ACTIVE_POL BUTTON_ACTIVE_LOW
#endif

static rt_timer_t rc10k_time_handle;
static int32_t key1_button_handle;
static rt_event_t main_event;
static rt_timer_t wrist_wake_init_timer;

/* ===== Raise-wrist source selection =====
   0 = software (existing hand_tracking.c state machine, gyro_x threshold).
       Lower latency, supports back-gesture / wrist-rotation; consumes IMU
       samples on every interrupt.
   1 = hardware (BMI270 BMI2_WRIST_WEAR_WAKE_UP). The chip detects raise-
       wrist internally and fires INT1 only on the gesture; SW state machine
       is killed. No back-gesture / wrist-rotation in this mode.
   Toggle this and rebuild LCPU. The two modes are mutually exclusive; the
   selected mode is applied a few seconds after boot once the BMI270 is
   guaranteed to be open (it's opened lazily on first subscriber). */
#ifndef USE_BMI270_HW_WRIST_WAKE
    #define USE_BMI270_HW_WRIST_WAKE 1
#endif

#define MAIN_EVENT_BATTERY_CHARGING (1 << 0)
#define MAIN_EVENT_BATTERY_VOLTAGE (1 << 1)
#define MAIN_EVENT_HAND_LIFT (1 << 2)
#define MAIN_EVENT_ALL                                                         \
    (MAIN_EVENT_BATTERY_CHARGING | MAIN_EVENT_BATTERY_VOLTAGE |                \
     MAIN_EVENT_HAND_LIFT)

void main_send_read_charge_status_event(void)
{
    if (main_event)
    {
        rt_event_send(main_event, MAIN_EVENT_BATTERY_CHARGING);
    }
}

void main_send_read_voltage_event(void)
{
    if (main_event)
    {
        rt_event_send(main_event, MAIN_EVENT_BATTERY_VOLTAGE);
    }
}


void main_send_hand_lift_event(void)
{
    if (main_event)
    {
        rt_event_send(main_event, MAIN_EVENT_HAND_LIFT);
    }
}

#ifdef ACC_USING_BMI270
/* Override the weak default in bmi270_driver.c. The BMI270 fires this when
   its internal wrist-wake detector triggers; route it through the same
   path the software algorithm uses. */
extern void hand_tracking_lift_callback(uint8_t lift);
void bmi270_on_wrist_wake_detected(void)
{
    rt_kprintf("[wrist-wake] LCPU override fired tick=%u -> hand_lift event\n",
               (unsigned)rt_tick_get());
    // main_send_hand_lift_event();
    hand_tracking_lift_callback(2);
}

/* === LCPU PM policy override =============================================
   The middleware default in bf0_pm.c (RT_WEAK pm_policy[]) picks STANDBY
   sleep for LCPU when PM_STANDBY_ENABLE is set in rtconfig — STANDBY/DEEP
   power down enough of the GPIO controller that PB26 (BMI270 INT1) cannot
   wake the SoC on this board (LPAON wakeup map only covers PB32~36 / PA50
   ~54 / PBR0~3, and BMI270 INT is wired to PB26 = GPIO 122).

   Override here with a LIGHT-only policy: idle threshold 15 ms, max sleep
   = LIGHT. LIGHT keeps GPIO controller alive so PB26 IRQ can wake LCPU.
   Power penalty vs STANDBY is small in practice because LCPU has nothing
   else periodic to run during screen-off (no audio / no LVGL).

   Override whole table — strong defn beats RT_WEAK in the middleware. */
#ifdef BSP_USING_PM
const pm_policy_t pm_policy[] =
{
    { 15, PM_SLEEP_MODE_LIGHT },
};

/* Print the active PM policy at boot so we can confirm the strong override
   above actually beat the RT_WEAK in bf0_pm.c. If we still see STANDBY
   (=4) instead of LIGHT (=2), the linker didn't pick up our override. */
static int log_active_pm_policy(void)
{
    rt_kprintf("[pm] policy override active: thresh=%u mode=%u (LIGHT=2 DEEP=3 STANDBY=4)\n",
               (unsigned)pm_policy[0].thresh, (unsigned)pm_policy[0].mode);
    return 0;
}
INIT_APP_EXPORT(log_active_pm_policy);
#endif

    #if USE_BMI270_HW_WRIST_WAKE
/* HW-mode auto-switch driven by HCPU's screen state — two-mode design.

   ACTIVE (screen on):
     - accel/gyro both 100 Hz, DRDY IRQ on (already configured by
       acce_set_power(HIGH) that fires right before this hook).
     - SW hand_tracking ON for post-wake gestures (put-down / back /
       pronation / lift2).
     - HW wrist-wake feature OFF (we'd just fire redundant events).

   DARK (screen off):
     - accel 25 Hz low-power (already configured by acce_set_power(LOW)
       that fires before this hook); gyro suspended on top of that.
     - DRDY interrupt un-routed from INT1 at the chip — the GPIO IRQ stays
       armed so wrist-wake can still wake the host, but per-sample DRDY no
       longer fires every 40 ms. Step counter feature continues running
       internally on the chip's own clock.
     - SW hand_tracking OFF (no DRDY → no samples flowing anyway).
     - HW wrist-wake feature ON. */
void on_lcpu_sleep_mode_changed(bool sleep)
{
    if (sleep)
    {
        bmi270_hw_wrist_wake_enable(1);
        bmi270_set_gyro_suspend(1);
        bmi270_set_drdy_int_routing(0);
        #ifdef BSP_USING_HAND_TRACKING
        hand_tracking_set_enabled(false);
        #endif
    }
    else
    {
        #ifdef BSP_USING_HAND_TRACKING
        hand_tracking_set_enabled(true);
        #endif
        bmi270_set_gyro_suspend(0);
        bmi270_set_drdy_int_routing(1);
        bmi270_hw_wrist_wake_enable(0);
    }
}
    #endif /* USE_BMI270_HW_WRIST_WAKE */

/* Open the BMI270 if the timer fires before any subscriber has done it,
   then sync to whatever screen state HCPU is currently in. In SW mode this
   is a one-shot log; in HW mode it primes the auto-switch. */
static void wrist_wake_init_apply(void *param)
{
    (void)param;
    #if USE_BMI270_HW_WRIST_WAKE
    if (bmi270_open() != 0)
    {
        rt_kprintf("[wrist-wake] bmi270_open failed; HW path disabled\n");
        return;
    }
    /* Enable BMI270 internal step counter once at boot. It stays on across
       screen-on/off transitions — chip runs it on its own (very low power)
       and just exposes a register the host can poll periodically. Replaces
       the SW Kraepelin step accumulation in the dark-screen path. */
    if (bmi270_hw_step_counter_enable(1) != 0)
    {
        rt_kprintf("[wrist-wake] HW step counter enable failed\n");
    }
    /* Apply current state — HCPU may have already moved us to sleep before
       the 3 s timer fired, in which case we need to arm HW now. Default
       _sleep_mode at boot is false (awake), so this typically just leaves
       SW enabled. */
    on_lcpu_sleep_mode_changed(is_sleep_mode());
    rt_kprintf("[wrist-wake] HW auto-switch armed: HW on screen-off, SW on "
               "screen-on (current: %s)\n",
               is_sleep_mode() ? "off" : "on");
    #else
    rt_kprintf("[wrist-wake] SW mode (hand_tracking.c state machine)\n");
    #endif
}
#endif /* ACC_USING_BMI270 */

static rt_err_t charge_event_rx_ind(rt_device_t dev, rt_size_t size)
{
    // SDK charge driver triggers this on charging state change
    main_send_read_charge_status_event();
    return RT_EOK;
}

void button_event_handler(int32_t pin, button_action_t button_action)
{
    rt_kprintf("pin: %d, action: %d\n", pin, button_action);
}

static void init_pin(void)
{
#if (BSP_KEY1_PIN >= GPIO1_PIN_NUM)
    button_cfg_t cfg;
    #if defined(BSP_USING_PM)
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;
    #endif /* BSP_USING_PM */

    cfg.pin = BSP_KEY1_PIN;
    cfg.active_state = BUTTON_ACTIVE_POL;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));
    key1_button_handle = id;

    #if defined(BSP_USING_PM)
    gpio = GET_GPIO_INSTANCE(BSP_KEY1_PIN);
    gpio_pin = GET_GPIOx_PIN(BSP_KEY1_PIN);

    wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
    RT_ASSERT(wakeup_pin >= 0);

    pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_DOUBLE_EDGE);
    #endif /* BSP_USING_PM */

#endif /* BSP_KEY1_PIN < GPIO1_PIN_NUM */
}

void rc10k_timeout_handler(void *parameter)
{
    if (HAL_LXT_DISABLED())
    {
        HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
    }
    else
    {
        rt_timer_stop(rc10k_time_handle);
    }
}

#ifdef RT_USING_WDT
/**
 * @brief This function is invoked in WDT_IRQHandler.
 *        It can be overidden to do some work when WDT1 timeout occured.
 *        Ex. to store exception context and reboot immediately.
 */
void wdt_store_exception_information(void)
{
    rt_kprintf("LCPH WDT1 timeout occurs.\n");
    extern void drv_reboot(void);
    drv_reboot();
    return;
}

/**
 * @brief WDT ON/OFF
 * @param en 0: OFF 1: ON
 */
static void watchdog_set_status(uint8_t en)
{
    #ifdef RT_USING_WDT
    /* Set wdt status 0. */
    rt_hw_watchdog_set_status(en);
    /* Avoid repeat set hook. */
    rt_hw_watchdog_hook(0);
    if (!en)
    {
        /* Stop wdt. */
        rt_hw_watchdog_deinit();
    }
    else
    {
        /* Set hook for watchdog petting. */
        rt_hw_watchdog_hook(1);
    }
    #endif
}
#endif

#if defined(BSP_USING_PM) && defined(BMI270_INT_GPIO_BIT)
/* Register the BMI270 INT1 line as a wakeup source so the chip can pull
   LCPU out of deep sleep when its internal wrist-wake (or any other
   feature mapped to INT1) fires. Falling edge: BMI270 default int polarity
   on this board is active-low, push-pull. */
static void bmi270_init_wakeup_pin(void)
{
    GPIO_TypeDef *gpio = GET_GPIO_INSTANCE(BMI270_INT_GPIO_BIT);
    uint16_t gpio_pin = GET_GPIOx_PIN(BMI270_INT_GPIO_BIT);
    int8_t wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
    if (wakeup_pin >= 0)
    {
        pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_NEG_EDGE);
        rt_kprintf("[pm] BMI270 INT1 registered as wakeup pin %d\n",
                   wakeup_pin);
    }
    else
    {
        rt_kprintf("[pm] BMI270 INT pin %d cannot be a wakeup source\n",
                   BMI270_INT_GPIO_BIT);
    }
}
#endif

int main(void)
{
    // init_pin();   /* Register the button as a wakeup source when PM is on.
    //                  No-op'd internally when BSP_KEY1_PIN < GPIO1_PIN_NUM. */
#if defined(BSP_USING_PM) && defined(BMI270_INT_GPIO_BIT)
    bmi270_init_wakeup_pin();
#endif
    if (HAL_LXT_DISABLED())
    {
        rc10k_time_handle = rt_timer_create(
            "rc10", rc10k_timeout_handler, NULL,
            rt_tick_from_millisecond(15 * 1000),
            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER); // 15s
        RT_ASSERT(rc10k_time_handle);
        rt_timer_start(rc10k_time_handle);
    }

#ifdef RT_USING_WDT
    /* Diable WDT. */
    watchdog_set_status(0);
    rt_kprintf("LCPH WDT off.\n");
    /* Enable WDT. */
    watchdog_set_status(1);
    rt_kprintf("LCPH WDT on.(timeout: %d seconds)\n", WDT_TIMEOUT);
#endif /* RT_USING_WDT */

    // 創建事件對象
    main_event = rt_event_create("main_evt", RT_IPC_FLAG_FIFO);
    RT_ASSERT(main_event != RT_NULL);

    // 初始化電池管理系統 (SDK battery_calculator)
    bloc_battery_init();

    // 註冊 SDK 充電事件回調，當充電狀態改變時通知主循環
    rt_charge_set_rx_ind(charge_event_rx_ind);

#ifdef ACC_USING_BMI270
    /* One-shot 3-second delay before applying wrist-wake mode. The BMI270 is
       opened lazily on first subscriber (acce service / hand_tracking init);
       waiting a few seconds lets that happen first. */
    wrist_wake_init_timer = rt_timer_create(
        "ww_init", wrist_wake_init_apply, NULL,
        rt_tick_from_millisecond(3000),
        RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    if (wrist_wake_init_timer)
    {
        rt_timer_start(wrist_wake_init_timer);
    }
#endif

    rt_uint32_t recv_set = 0;
    while (1)
    {
        rt_err_t result = rt_event_recv(main_event, MAIN_EVENT_ALL,
                                        RT_EVENT_FLAG_OR |
                                        RT_EVENT_FLAG_CLEAR,
                                        RT_WAITING_FOREVER, &recv_set);

        if (result == RT_EOK)
        {
            if (recv_set & MAIN_EVENT_BATTERY_CHARGING)
            {
                bloc_battery_handle_charging_event();
            }

            if (recv_set & MAIN_EVENT_BATTERY_VOLTAGE)
            {
                bloc_battery_handle_voltage_event();
            }

            if (recv_set & MAIN_EVENT_HAND_LIFT)
            {
                extern void hand_tracking_lift_callback(uint8_t lift);
                hand_tracking_lift_callback(0);
            }
        }
    }
    return RT_EOK;
}
