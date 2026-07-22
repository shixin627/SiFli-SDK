/**
 ******************************************************************************
 * @file   hardware_selftest.c
 * @brief  Release-firmware hardware smoke test over the download UART.
 ******************************************************************************
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include <string.h>

#include "watch_global_data.h"
#include "bloc_peripheral.h"

#define HWTEST_UART_NAME       "uart1"
#define HWTEST_COMMAND_PREFIX  "SKAI_HWTEST RUN "
#define HWTEST_NONCE_LEN       8
#define HWTEST_LINE_SIZE       96
#define HWTEST_REPLY_SIZE      192
#define HWTEST_THREAD_STACK    3072
#define HWTEST_THREAD_PRIORITY 27
#define HWTEST_LISTEN_WINDOW_MS 120000

static rt_device_t hwtest_uart;
static struct rt_semaphore hwtest_rx_sem;
static volatile rt_bool_t hwtest_completed;

static void hwtest_write(const char *text)
{
    if (hwtest_uart != RT_NULL && text != RT_NULL)
    {
        rt_device_write(hwtest_uart, 0, text, strlen(text));
    }
}

static void hwtest_item(const char *nonce, const char *item,
                        const char *status, const char *detail)
{
    char reply[HWTEST_REPLY_SIZE];
    rt_snprintf(reply, sizeof(reply), "SKAI_HWTEST ITEM %s %s %s %s\r\n",
                nonce, item, status, detail);
    hwtest_write(reply);
}

static rt_bool_t valid_nonce(const char *nonce)
{
    int i;
    if (nonce == RT_NULL || strlen(nonce) != HWTEST_NONCE_LEN)
    {
        return RT_FALSE;
    }
    for (i = 0; i < HWTEST_NONCE_LEN; i++)
    {
        char c = nonce[i];
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
              (c >= 'a' && c <= 'f')))
        {
            return RT_FALSE;
        }
    }
    return RT_TRUE;
}

static rt_bool_t wait_for_imu_change(uint32_t before, rt_uint32_t timeout_ms)
{
    rt_uint32_t started = rt_tick_get_millisecond();
    while ((rt_tick_get_millisecond() - started) < timeout_ms)
    {
        uint32_t current = (uint32_t)watch_sensor.imu_data.timestamp;
        if (current != 0 && current != before)
        {
            return RT_TRUE;
        }
        rt_thread_mdelay(100);
    }
    return RT_FALSE;
}

static rt_bool_t wait_for_ppg_change(uint32_t before, rt_uint32_t timeout_ms)
{
    rt_uint32_t started = rt_tick_get_millisecond();
    while ((rt_tick_get_millisecond() - started) < timeout_ms)
    {
        uint32_t current =
            (uint32_t)watch_sensor.motion_data.ppg_raw_data.timestamp;
        if (current != 0 && current != before)
        {
            return RT_TRUE;
        }
        rt_thread_mdelay(100);
    }
    return RT_FALSE;
}

static void run_hardware_selftest(const char *nonce)
{
    char reply[HWTEST_REPLY_SIZE];
    int failures = 0;
    rt_bool_t passed;

    rt_snprintf(reply, sizeof(reply),
                "SKAI_HWTEST BEGIN %s version=%d.%d.%d release=%d uptime_ms=%lu\r\n",
                nonce, VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION,
                kReleaseMode, (unsigned long)rt_tick_get_millisecond());
    hwtest_write(reply);

    passed = (kReleaseMode == 1);
    hwtest_item(nonce, "release", passed ? "PASS" : "FAIL",
                passed ? "kReleaseMode=1" : "kReleaseMode=0");
    if (!passed) failures++;

    hwtest_item(nonce, "uart", "PASS", "uart1_rx_tx_ok");

    if (watch_sys_sync.request_battery_voltage != RT_NULL)
    {
        watch_sys_sync.request_battery_voltage();
        rt_thread_mdelay(800);
    }
    passed = (SkaiWatchSys.battery_vol_value >= 2800 &&
              SkaiWatchSys.battery_vol_value <= 5000 &&
              SkaiWatchSys.battery_level_value <= 100);
    rt_snprintf(reply, sizeof(reply), "mv=%u level=%u",
                SkaiWatchSys.battery_vol_value,
                SkaiWatchSys.battery_level_value);
    hwtest_item(nonce, "battery", passed ? "PASS" : "FAIL", reply);
    if (!passed) failures++;

    {
        rt_bool_t imu_was_subscribed = (watch_sensor.imu_client_num > 0);
        rt_bool_t hr_was_subscribed = (watch_sensor.hr_client_num > 0);
        uint32_t before = (uint32_t)watch_sensor.imu_data.timestamp;

        if (!imu_was_subscribed &&
            peripheral_provider.subscribe_accelerometer_sensor != RT_NULL)
        {
            peripheral_provider.subscribe_accelerometer_sensor(true);
        }
        passed = wait_for_imu_change(before, 3000);
        rt_snprintf(reply, sizeof(reply), "timestamp=%lu abnormal=%u",
                    (unsigned long)watch_sensor.imu_data.timestamp,
                    watch_sensor.imu_abnormal ? 1 : 0);
        if (watch_sensor.imu_abnormal) passed = RT_FALSE;
        hwtest_item(nonce, "imu", passed ? "PASS" : "FAIL", reply);
        if (!passed) failures++;

        /* Raw PPG is bundled in the ACCE motion stream in release builds.
         * Keep IMU subscribed while HR enables the optical sensor. */
        before = (uint32_t)watch_sensor.motion_data.ppg_raw_data.timestamp;
        if (!hr_was_subscribed &&
            peripheral_provider.subscribe_hr_sensor != RT_NULL)
        {
            peripheral_provider.subscribe_hr_sensor(true);
        }
        passed = wait_for_ppg_change(before, 6000);
        rt_snprintf(reply, sizeof(reply), "timestamp=%lu samples=%u raw0=%lu",
                    (unsigned long)watch_sensor.motion_data.ppg_raw_data.timestamp,
                    watch_sensor.motion_data.ppg_raw_data.sample_num,
                    (unsigned long)watch_sensor.motion_data.ppg_raw_data.raw_data[0]);
        hwtest_item(nonce, "ppg", passed ? "PASS" : "FAIL", reply);
        if (!passed) failures++;

        if (!hr_was_subscribed &&
            peripheral_provider.subscribe_hr_sensor != RT_NULL)
        {
            peripheral_provider.subscribe_hr_sensor(false);
        }
        if (!imu_was_subscribed &&
            peripheral_provider.subscribe_accelerometer_sensor != RT_NULL)
        {
            peripheral_provider.subscribe_accelerometer_sensor(false);
        }
    }

    rt_snprintf(reply, sizeof(reply), "SKAI_HWTEST END %s %s failures=%d\r\n",
                nonce, failures == 0 ? "PASS" : "FAIL", failures);
    hwtest_write(reply);
    hwtest_completed = RT_TRUE;
}

static rt_err_t hwtest_rx_indicate(rt_device_t dev, rt_size_t size)
{
    (void)dev;
    (void)size;
    return rt_sem_release(&hwtest_rx_sem);
}

static void hardware_selftest_thread(void *parameter)
{
    char line[HWTEST_LINE_SIZE];
    size_t used = 0;
    char ch;
    rt_uint32_t listen_started;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    (void)parameter;

    hwtest_uart = rt_device_find(HWTEST_UART_NAME);
    if (hwtest_uart == RT_NULL)
    {
        return;
    }

    config.baud_rate = BAUD_RATE_1000000;
    rt_device_control(hwtest_uart, RT_DEVICE_CTRL_CONFIG, &config);
    if (rt_device_open(hwtest_uart,
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        hwtest_uart = RT_NULL;
        return;
    }
    rt_device_set_rx_indicate(hwtest_uart, hwtest_rx_indicate);
    listen_started = rt_tick_get_millisecond();

    while (!hwtest_completed &&
           (rt_tick_get_millisecond() - listen_started) < HWTEST_LISTEN_WINDOW_MS)
    {
        while (rt_device_read(hwtest_uart, 0, &ch, 1) == 1)
        {
            if (ch == '\r') continue;
            if (ch == '\n')
            {
                line[used] = '\0';
                if (strncmp(line, HWTEST_COMMAND_PREFIX,
                            strlen(HWTEST_COMMAND_PREFIX)) == 0)
                {
                    const char *nonce = line + strlen(HWTEST_COMMAND_PREFIX);
                    if (valid_nonce(nonce))
                    {
                        run_hardware_selftest(nonce);
                    }
                }
                used = 0;
            }
            else if (used < sizeof(line) - 1)
            {
                line[used++] = ch;
            }
            else
            {
                used = 0;
            }
        }
        rt_sem_take(&hwtest_rx_sem, rt_tick_from_millisecond(1000));
    }

    rt_device_set_rx_indicate(hwtest_uart, RT_NULL);
    rt_device_close(hwtest_uart);
    hwtest_uart = RT_NULL;
}

static int hardware_selftest_init(void)
{
    rt_thread_t thread;
    rt_sem_init(&hwtest_rx_sem, "hwtest_rx", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("hw_selftest", hardware_selftest_thread, RT_NULL,
                              HWTEST_THREAD_STACK, HWTEST_THREAD_PRIORITY, 10);
    if (thread == RT_NULL)
    {
        return -RT_ENOMEM;
    }
    return rt_thread_startup(thread);
}
INIT_APP_EXPORT(hardware_selftest_init);
