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
#include "app_test.h"

#define HWTEST_UART_NAME       "uart1"
#define HWTEST_COMMAND_PREFIX  "SKAI_HWTEST RUN "
#define HWTEST_SCREEN_PREFIX   "SKAI_HWTEST SCREEN "
#define HWTEST_CANCEL_PREFIX   "SKAI_HWTEST CANCEL "
#define HWTEST_NONCE_LEN       8
#define HWTEST_LINE_SIZE       96
#define HWTEST_REPLY_SIZE      192
#define HWTEST_THREAD_STACK    3072
#define HWTEST_THREAD_PRIORITY 27
#define HWTEST_LISTEN_WINDOW_MS 300000

static rt_device_t hwtest_uart;
static struct rt_semaphore hwtest_rx_sem;
static volatile rt_bool_t hwtest_completed;
static volatile rt_bool_t hwtest_running;
static char hwtest_nonce[HWTEST_NONCE_LEN + 1];
static char hwtest_end_reply[HWTEST_REPLY_SIZE];
static int hwtest_preflight_failures;

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

static void hwtest_finish(int app_failures)
{
    int failures = hwtest_preflight_failures + app_failures;

    rt_snprintf(hwtest_end_reply, sizeof(hwtest_end_reply),
                "SKAI_HWTEST END %s %s failures=%d\r\n",
                hwtest_nonce, failures == 0 ? "PASS" : "FAIL", failures);
    hwtest_write(hwtest_end_reply);
    hwtest_running = RT_FALSE;
    hwtest_completed = RT_TRUE;
}

static void hwtest_app_item(void *context, const char *name, bool passed,
                            const char *detail)
{
    (void)context;
    hwtest_item(hwtest_nonce, name, passed ? "PASS" : "FAIL",
                detail == RT_NULL ? "" : detail);
}

static void hwtest_app_progress(void *context, int percent,
                                const char *stage, const char *detail)
{
    char reply[HWTEST_REPLY_SIZE];
    (void)context;
    rt_snprintf(reply, sizeof(reply),
                "SKAI_HWTEST PROGRESS %s %d %s %s\r\n",
                hwtest_nonce, percent,
                stage == RT_NULL ? "" : stage,
                detail == RT_NULL ? "" : detail);
    hwtest_write(reply);
}

static void hwtest_app_done(void *context, int failures)
{
    (void)context;
    hwtest_finish(failures);
}

static void run_hardware_selftest(const char *nonce)
{
    app_test_screening_callbacks_t callbacks;
    char reply[HWTEST_REPLY_SIZE];
    rt_bool_t passed;

    if (hwtest_running)
    {
        return;
    }
    if (hwtest_completed && strcmp(nonce, hwtest_nonce) == 0)
    {
        /* The PC retries until it receives a response.  Re-send the cached
         * END for the same request instead of launching the app twice. */
        hwtest_write(hwtest_end_reply);
        return;
    }

    rt_strncpy(hwtest_nonce, nonce, sizeof(hwtest_nonce));
    hwtest_nonce[HWTEST_NONCE_LEN] = '\0';
    hwtest_preflight_failures = 0;
    hwtest_completed = RT_FALSE;
    hwtest_running = RT_TRUE;

    rt_snprintf(reply, sizeof(reply),
                "SKAI_HWTEST BEGIN %s version=%d.%d.%d release=%d uptime_ms=%lu\r\n",
                hwtest_nonce, VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION,
                kReleaseMode, (unsigned long)rt_tick_get_millisecond());
    hwtest_write(reply);

    /* UART and release-mode validation belong to the PC release preflight.
     * Automated screening is a hidden UART mode of the existing test app. */
    hwtest_item(hwtest_nonce, "uart", "PASS", "uart1_rx_tx_ok");
    passed = (kReleaseMode == 1);
    hwtest_item(hwtest_nonce, "release", passed ? "PASS" : "FAIL",
                passed ? "kReleaseMode=1" : "kReleaseMode=0");
    if (!passed)
    {
        hwtest_preflight_failures++;
    }

    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.progress = hwtest_app_progress;
    callbacks.item = hwtest_app_item;
    callbacks.done = hwtest_app_done;
    callbacks.context = RT_NULL;
    if (!app_test_board_screening_start(&callbacks))
    {
        hwtest_item(hwtest_nonce, "test_app", "FAIL",
                    "app_launch_failed");
        hwtest_finish(1);
    }
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

    while (hwtest_running ||
           (rt_tick_get_millisecond() - listen_started) <
               HWTEST_LISTEN_WINDOW_MS)
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
                else if (strncmp(line, HWTEST_SCREEN_PREFIX,
                                 strlen(HWTEST_SCREEN_PREFIX)) == 0)
                {
                    const char *nonce =
                        line + strlen(HWTEST_SCREEN_PREFIX);
                    if (valid_nonce(nonce))
                    {
                        run_hardware_selftest(nonce);
                    }
                }
                else if (strncmp(line, HWTEST_CANCEL_PREFIX,
                                 strlen(HWTEST_CANCEL_PREFIX)) == 0)
                {
                    const char *nonce =
                        line + strlen(HWTEST_CANCEL_PREFIX);
                    if (valid_nonce(nonce) && hwtest_running &&
                        strcmp(nonce, hwtest_nonce) == 0)
                    {
                        app_test_board_screening_cancel();
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
