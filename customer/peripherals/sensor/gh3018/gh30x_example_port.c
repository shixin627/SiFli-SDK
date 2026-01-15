/**
 * @copyright (c) 2003 - 2020, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3018_example_port.c
 *
 * @brief   example code for gh3018 (condensed  hbd_ctrl lib)
 *
 */

#include "gh30x_example_common.h"
#include "gh30x_example.h"
#include <rtthread.h>
#include <math.h>
#include "stdlib.h"
#include "board.h"
#include "bf0_hal.h"
#include "gh3018.h"
#include "bloc_peripheral.h"

#define DRV_DEBUG
#define LOG_TAG "gh30x.port"
#include <drv_log.h>

#define GH3018_USE_INT
static struct rt_i2c_bus_device *gh3018_i2cbus = NULL;
#ifdef GH3018_USE_INT
static struct rt_semaphore gh3018_int_sem;
#endif

#define THREAD_STACK_SIZE 6144
#define THREAD_PRIORITY 7
#define THREAD_TIMESLICE RT_THREAD_TICK_DEFAULT

rt_thread_t gh3018_thread = NULL;

static uint8_t loc_hb_value;
static uint32_t loc_ppg_buf[4] = {0};
static uint32_t loc_ppg_buf2[4] = {0};
static uint16_t loc_ppg_rawdata_len;

#ifdef GOODIX_DEMO_PLANFORM
extern uint8_t gh30x_run_mode;
extern bool goodix_app_start_app_mode;
extern uint8_t gh30x_mcu_start_mode_get(void);
#endif

int gh3018_power_onoff(uint8_t on)
{
    rt_err_t ret = RT_EOK;
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_D("GPIO pin device not found at motor ctrl");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
        return ret;
#if GH3018_POW_PIN
    m.pin = GH3018_POW_PIN;
#else
    m.pin = PPG_POWER_EN_PIN;
#endif
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

#if GH3018_POW_PIN
    st.pin = GH3018_POW_PIN;
#else
    st.pin = PPG_POWER_EN_PIN;
#endif
    st.status = on;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);
    LOG_D("GH3018 power %s", on ? "on" : "off");
    return ret;
}

/// reset pin init for gh30x
void hal_gh30x_pin_set(uint8_t en)
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at GH3018\n");
        return;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);

#ifdef GH3018_RST_PIN
    m.pin = GH3018_RST_PIN;
#else
    m.pin = PPG_RST_PIN;
#endif
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = m.pin;
    st.status = 0;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    // move to power up
    if (en)
    {
        rt_thread_delay(2);
#ifdef GH3018_RST_PIN
        st.pin = GH3018_RST_PIN;
#else
        st.pin = PPG_RST_PIN;
#endif
        st.status = 1;
        rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    }
    rt_device_close(device);
}

/* gh30x i2c interface */

/// i2c for gh30x init
int hal_gh30x_i2c_init(void)
{
    gh3018_power_onoff(1);

#if 1
    hal_gh30x_pin_set(1);
#endif
    rt_err_t ret = RT_EOK;
    /* get i2c bus device */
    gh3018_i2cbus = (struct rt_i2c_bus_device *)rt_device_find(GH3018_I2C_BUS);
    if (RT_Device_Class_I2CBUS != gh3018_i2cbus->parent.type)
    {
        gh3018_i2cbus = NULL;
    }
    if (gh3018_i2cbus == NULL)
    {
        LOG_E("Can not found i2c bus %s, init fail\n", BMI270_BUS_NAME);
        return -1;
    }
    LOG_D("Find i2c bus device %s\n", GH3018_I2C_BUS);
    /* open i2c bus device */
    // ret = rt_device_open((rt_device_t)gh3018_i2cbus, RT_DEVICE_FLAG_RDWR |
    // RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX); if (ret != RT_EOK)
    // {
    //     LOG_E("Can not open i2c bus %s, init fail\n", GH3018_I2C_BUS);
    //     return -1;
    // }
    // LOG_D("Open i2c bus device %s\n", GH3018_I2C_BUS);
    // /* configure i2c bus device */
    // {
    //     struct rt_i2c_configuration configuration = {
    //         .mode = 0,
    //         .addr = gh3018_get_dev_addr(),
    //         .timeout = 500,
    //         .max_hz = 400000,
    //     };
    //     ret = rt_i2c_configure(gh3018_i2cbus, &configuration);
    //     if (ret != RT_EOK)
    //     {
    //         LOG_E("Can not configure i2c bus %s, init fail\n",
    //         GH3018_I2C_BUS); return ret;
    //     }
    // }
    return ret;
}

/// i2c for gh30x wrtie
uint8_t hal_gh30x_i2c_write(uint8_t device_id, const uint8_t write_buffer[],
                            uint16_t length)
{
    uint8_t ret = GH30X_EXAMPLE_OK_VAL;

    struct rt_i2c_msg msgs[2];
    uint32_t res;

    if (gh3018_i2cbus)
    {
        msgs[0].addr = device_id >> 1;         /* Slave address */
        msgs[0].flags = RT_I2C_WR;             /* Write flag */
        msgs[0].buf = (uint8_t *)write_buffer; /* Slave register address */
        msgs[0].len = length;                  /* Number of bytes sent */

        res = rt_i2c_transfer(gh3018_i2cbus, msgs, 1);
        if (res == 1)
        {
            // LOG_D("GH3018_I2C_Write OK: 0x%x, %d\n", device_id, length);
            ret = GH30X_EXAMPLE_OK_VAL;
        }
        else
        {
            LOG_E("hal_gh30x_i2c_write FAIL: 0x%x, %d,  %d", device_id, length,
                  res);
            ret = GH30X_EXAMPLE_ERR_VAL;
        }
    }

    return ret;
}

/// i2c for gh30x read
uint8_t hal_gh30x_i2c_read(uint8_t device_id, const uint8_t write_buffer[],
                           uint16_t write_length, uint8_t read_buffer[],
                           uint16_t read_length)
{
    uint8_t ret = GH30X_EXAMPLE_OK_VAL;

    struct rt_i2c_msg msgs[2];
    uint32_t res;

    if (gh3018_i2cbus)
    {
        msgs[0].addr = device_id >> 1;         /* Slave address */
        msgs[0].flags = RT_I2C_WR;             /* Write flag */
        msgs[0].buf = (uint8_t *)write_buffer; /* Slave register address */
        msgs[0].len = write_length;            /* Number of bytes sent */

        msgs[1].addr = device_id >> 1; /* Slave address */
        msgs[1].flags = RT_I2C_RD;     /* Read flag */
        msgs[1].buf = read_buffer;     /* Read data pointer */
        msgs[1].len = read_length;     /* Number of bytes read */

        res = rt_i2c_transfer(gh3018_i2cbus, msgs, 2);
        if (res == 2)
        {
            // LOG_D("GH3018_I2C_Read OK: 0x%x\n", msgs[0].addr);
            ret = GH30X_EXAMPLE_OK_VAL;
        }
        else
        {
            LOG_E("hal_gh30x_i2c_read FAIL: 0x%x,  %d", device_id, res);
            ret = GH30X_EXAMPLE_ERR_VAL;
        }
    }
    return ret;
}

/* gh30x spi interface */

/// spi for gh30x init
void hal_gh30x_spi_init(void)
{
    // code implement by user
}

/// spi for gh30x wrtie
uint8_t hal_gh30x_spi_write(const uint8_t write_buffer[], uint16_t length)
{
    uint8_t ret = 1;
    return ret;
}

/// spi for gh30x read
uint8_t hal_gh30x_spi_read(uint8_t read_buffer[], uint16_t length)
{
    uint8_t ret = 1;
    return ret;
}

/// spi cs set low for gh30x
void hal_gh30x_spi_cs_set_low(void)
{
    // code implement by user
}

/// spi cs set high for gh30x
void hal_gh30x_spi_cs_set_high(void)
{
    // code implement by user
}

/* delay */

/// delay us
void hal_gh30x_delay_us(uint16_t us_cnt)
{
    // code implement by user
    HAL_Delay_us(us_cnt);
}

/* gsensor driver */

/// gsensor motion detect mode flag
bool gsensor_drv_motion_det_mode = false;

/// gsensor init
int8_t gsensor_drv_init(void)
{
    int8_t ret = GH30X_EXAMPLE_OK_VAL;

    gsensor_drv_motion_det_mode = false;

    // code implement by user
    /* if enable all func equal 25Hz, should config > 25Hz;
    but if enable have 100hz, should config to > 100hz. if not, feeback to
    GOODIX!!!
    */
    // ret = gs_drv_init();

    return ret;
}

/// gsensor enter normal mode
void gsensor_drv_enter_normal_mode(void)
{
    // code implement by user
    gsensor_drv_motion_det_mode = false;
}

/// gsensor enter fifo mode
void gsensor_drv_enter_fifo_mode(void)
{
    // code implement by user
    gsensor_drv_motion_det_mode = false;
}

/// gsensor enter motion det mode
void gsensor_drv_enter_motion_det_mode(void)
{
    // code implement by user
    /* if enable motion det mode that call @ref hal_gsensor_drv_int1_handler
       when motion generate irq e.g. 1. (hardware) use gsensor motion detect
       module with reg config
             2. (software) gsensor enter normal mode, then define 30ms timer get
       gsensor rawdata, if now total acceleration(sqrt(x*x+y*y+z*z)) - last
       total acceleration >= 30 (0.05g @512Lsb/g) as motion generate that call
       @ref hal_gsensor_drv_int1_handler
    */
    // gs_drv_enter_motion_det_mode();
    gsensor_drv_motion_det_mode = true;
}

/// gsensor get fifo data
void gsensor_drv_get_fifo_data(ST_GS_DATA_TYPE gsensor_buffer[],
                               uint16_t *gsensor_buffer_index,
                               uint16_t gsensor_max_len)
{
    // code implement by user
    // EXAMPLE_DEBUG_LOG_L1("gsensor_drv_get_fifo_data");
#ifdef BSP_USING_BLOC_PERIPHERAL
    for (uint16_t i = 0; i < gsensor_max_len; i++)
    {
        uint16_t origin_buf_index =
            (gsensor_fifo_buffer_index + i) % gsensor_max_len;
        gsensor_buffer[i].sXAxisVal = gsensor_fifo_buffer[origin_buf_index][0];
        gsensor_buffer[i].sYAxisVal = gsensor_fifo_buffer[origin_buf_index][1];
        gsensor_buffer[i].sZAxisVal = gsensor_fifo_buffer[origin_buf_index][2];
    }
    *gsensor_buffer_index = gsensor_max_len;
#endif
    /**************************** WARNNING: DO NOT REMOVE OR MODIFY THIS CODE
     * ---START***************************************************/
    if ((*gsensor_buffer_index) > (gsensor_max_len))
    {
        EXAMPLE_DEBUG_LOG_L1(
            "[%s]: Fatal error! Gsensor buffer is accessed overrun !",
            __FUNCTION__);
        while (1)
            ; // Fatal error !!!
    }
    /**************************** WARNNING: DO NOT REMOVE OR MODIFY THIS CODE
     * ---END***************************************************/
}

/// gsensor clear buffer
void gsensor_drv_clear_buffer(ST_GS_DATA_TYPE gsensor_buffer[],
                              uint16_t *gsensor_buffer_index,
                              uint16_t gsensor_buffer_len)
{
    if ((gsensor_buffer != NULL) && (gsensor_buffer_index != NULL))
    {
        memset(gsensor_buffer, 0, sizeof(ST_GS_DATA_TYPE) * gsensor_buffer_len);
        *gsensor_buffer_index = 0;
    }
}

/// gsensor get data
void gsensor_drv_get_data(ST_GS_DATA_TYPE *gsensor_data_ptr)
{
    /* This interface is only used in newdata mode,
        and only one XYZ axis array is obtained each time
    */
    // code implement by user
#ifdef BSP_USING_BLOC_PERIPHERAL
    gsensor_data_ptr->sXAxisVal =
        gsensor_fifo_buffer[gsensor_fifo_buffer_index][0];
    gsensor_data_ptr->sYAxisVal =
        gsensor_fifo_buffer[gsensor_fifo_buffer_index][1];
    gsensor_data_ptr->sZAxisVal =
        gsensor_fifo_buffer[gsensor_fifo_buffer_index][2];
#endif
}

/* interrupt handler */
void hal_gh30x_int_handler_top_half(void)
{
    g_uchNewIntFlag++;
}

/* thread handler */
void hal_gh30x_int_handler_bottom_half(void)
{
    gh30x_int_msg_handler();
}

/* int */
#define PERIOD_IN_FREQUENCY_25HZ (40) * __HB_FIFO_THR_CNT_CONFIG__
// #define PERIOD_IN_FREQUENCY_100HZ (10) //10  * __HB_FIFO_THR_CNT_CONFIG__

#ifdef GH3018_USE_INT
static void gh30x_int_handle(void *args)
{
    // LOG_D("gh30x_int_handle\n");
    #ifdef GH3018_USE_INT

        #ifdef GH3018_INT_BIT
    rt_pin_irq_enable(GH3018_INT_BIT, 0);
        #else
    rt_pin_irq_enable(PPG_INT_PIN, 0);
        #endif

    #endif
    hal_gh30x_int_handler_top_half();
    rt_sem_release(&gh3018_int_sem);
    // LOG_I("GH3018 int\n");
}
#endif
// start a thread to process semphone
static void gh30x_sensor_task(void *params)
{
    while (1)
    {
#ifdef GH3018_USE_INT
        rt_sem_take(&gh3018_int_sem, RT_WAITING_FOREVER);
#else
        rt_thread_mdelay(PERIOD_IN_FREQUENCY_25HZ);
#endif
        gh30x_api_lock();
        // LOG_I("gh30x_sensor_task");
        hal_gh30x_int_handler_bottom_half();
        gh30x_api_unlock();
#ifdef GH3018_USE_INT
    #ifdef GH3018_INT_BIT
        rt_pin_irq_enable(GH3018_INT_BIT, 1);
    #else
        rt_pin_irq_enable(PPG_INT_PIN, 1);
    #endif
#endif
    }
}
/// gh30x int pin init, should config as rise edge trigger
void hal_gh30x_int_init(void)
{
    // code implement by user
    // must register func hal_gh30x_int_handler as callback
    struct rt_device_pin_mode m;
    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at GH3018\n");
        return;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
#ifdef GH3018_USE_INT
    // int pin cfg
    #ifdef PPG_INT_PIN
    m.pin = PPG_INT_PIN;
    #else
    m.pin = GH3018_INT_BIT;
    #endif
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable gh3018 int
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, gh30x_int_handle,
                      (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);

    rt_device_close(device);

    rt_sem_init(&gh3018_int_sem, "gh3018_int", 0, RT_IPC_FLAG_FIFO);
#endif
    // start a thread to check data available
    gh3018_thread =
        rt_thread_create("gh3018", gh30x_sensor_task, NULL, THREAD_STACK_SIZE,
                         THREAD_PRIORITY, THREAD_TIMESLICE);

    if (gh3018_thread != RT_NULL)
    {
        rt_err_t ret = rt_thread_startup(gh3018_thread);
        if (ret == RT_EOK)
        {
            LOG_D("Thread gh3018 started\n");
        }
        else
        {
            LOG_E("Start gh3018 thread fail\n");
        }
    }
    else
    {
        LOG_E("Create gh3018 thread fail\n");
    }
}

/// gsensor int handler
void hal_gsensor_drv_int1_handler(void)
{
    // code implement by user
    // LOG_D("[hal_gsensor_drv_int1_handler] gsensor_drv_motion_det_mode=%d\n",
    // gsensor_drv_motion_det_mode);
    if (gsensor_drv_motion_det_mode)
    {
        gsensor_motion_has_detect();
    }
    else
    {
        /* if using gsensor fifo mode, should get data by fifo int
         * e.g. gsensor_read_fifo_data();
         */
        gsensor_read_fifo_data();
    }
}

/// gsensor int1 init, should config as rise edge trigger
void hal_gsensor_int1_init(void)
{
    // code implement by user
    // must register func hal_gsensor_drv_int1_handler as callback

    /* if using gsensor fifo mode,
    and gsensor fifo depth is not enough to store 1 second data,
    please connect gsensor fifo interrupt to the host,
    or if using gsensor motion detect mode(e.g  motion interrupt response by
    0.5G * 5counts), and implement this function to receive gsensor interrupt.
    */
    // hal_gsensor_int1_init2();
}

#if (__USE_GOODIX_APP__)
/// handle algo status update
void handle_algo_result_update(GU32 function_id, GU16 value)
{
    switch (function_id)
    {
    case GH30X_FUNCTION_HR:
        break;

    case GH30X_FUNCTION_HRV:
        break;

    case GH30X_FUNCTION_SPO2:
        break;

    default:
        break;
    }
}
#endif

/// handle wear status result
void handle_wear_status_result(uint8_t wearing_state_val, GU8 uchLivingFlag)
{
    // code implement by user
#ifdef GOODIX_DEMO_PLANFORM
    if (WEAR_STATUS_UNWEAR == wearing_state_val)
    {
        EXAMPLE_DEBUG_LOG_L1("[handle_wear_status_result] wear off! 0x%x\r",
                             g_unGh30xDemoFuncMode);
        if (g_unGh30xDemoFuncMode &
            (~GH30X_FUNCTION_ADT)) // if have some function excluding ADT, stop
                                   // current function and restart adt
        {
            /********************* START: the code is optional, system will
             * force sample without the code**************/
            gh30x_module_stop();

            if (false == goodix_app_start_app_mode)
            {
                gh30x_module_start(GH30X_FUNCTION_ADT);
            }
            else
            {
                gh30x_module_start(GH30X_FUNCTION_HR);
            }
            /********************* END: the code is optional, system will force
             * sample without the code**************/
        }
    }
    else if (WEAR_STATUS_WEAR == wearing_state_val)
    {
        EXAMPLE_DEBUG_LOG_L1("[handle_wear_status_result] wear on! 0x%x\r",
                             g_unGh30xDemoFuncMode);
        if (GH30X_FUNCTION_ADT == g_unGh30xDemoFuncMode)
        {
            if (RUN_MODE_HARDWARE_ADT_DET != gh30x_mcu_start_mode_get())
            {
                gh30x_module_stop();
            }

            switch (gh30x_mcu_start_mode_get())
            {
            case RUN_MODE_ADT_HB_DET:
            {
                gh30x_module_start(GH30X_FUNCTION_SOFT_ADT | GH30X_FUNCTION_HR |
                                   GH30X_FUNCTION_ADT);
            }
            break;

            case RUN_MODE_HRV_DET:
            {
                gh30x_module_start(GH30X_FUNCTION_SOFT_ADT |
                                   GH30X_FUNCTION_HRV | GH30X_FUNCTION_HR |
                                   GH30X_FUNCTION_ADT);
            }
            break;

            case RUN_MODE_SPO2_DET:
            {
                gh30x_module_start(GH30X_FUNCTION_SPO2 | GH30X_FUNCTION_HRV |
                                   GH30X_FUNCTION_ADT);
            }
            break;

            default:
                break;
            }
        }
    }
#else
    if (WEAR_STATUS_UNWEAR == wearing_state_val)
    {
        EXAMPLE_DEBUG_LOG_L1(
            "[handle_wear_status_result] wear off! uchLivingFlag:%d\r",
            uchLivingFlag);
        soft_adt_callback(false);
        /// TODO: if hcpu suspend, close sensor, enter save power mode.
        /// Otherwise, keep sensor power on, and keep sensor running.
    }
    else if (WEAR_STATUS_WEAR == wearing_state_val)
    {
        EXAMPLE_DEBUG_LOG_L1(
            "[handle_wear_status_result] wear on! uchLivingFlag:%d\r",
            uchLivingFlag);
        soft_adt_callback(true);
    }
#endif
}

#if (__SYSTEM_TEST_SUPPORT__)
/// handle wear status result: {0-11}, led_num: {0-2};
/// test_res: <0=> ok , <1=> init err , <2=> order err , <3=> comm read err,<4=>
/// comm write err,<5=> otp read err,<6=> ctr not pass
///< 7=> rawdata not pass , <8=> noise not pass , <9=> leak not pass, <10=>
///< leakratio not pass,<11=> resource error
void handle_system_test_result(uint8_t test_res, uint8_t led_num)
{
    // code implement by user
    EXAMPLE_DEBUG_LOG_L1(
        "system test has complete,the result is %d,led is %d\n", test_res,
        led_num);
}

void handle_before_system_os_test(void)
{
    // code implement by user
    EXAMPLE_DEBUG_LOG_L1("begin new term test");
    #ifdef GOODIX_DEMO_PLANFORM
    vTaskDelay(4000);
    #endif
}
#endif

#if (__FACTORY_DET_SUPPORT__)
void handle_factory_mode_result(STGh30xFactoryRawdata *pstGh30xFactoryRawdata)
{
    EXAMPLE_DEBUG_LOG_L1("current 0x%x 0x%x 0x%x\r\n",
                         pstGh30xFactoryRawdata->gre1_led_curr,
                         pstGh30xFactoryRawdata->ir_led_curr,
                         pstGh30xFactoryRawdata->red_led_curr);
    EXAMPLE_DEBUG_LOG_L1("green num %d, rawdata = ",
                         pstGh30xFactoryRawdata->gre1_num);

    for (GU8 i = 0; i < pstGh30xFactoryRawdata->gre1_num; i++)
    {
        EXAMPLE_DEBUG_LOG_L1("%d,", pstGh30xFactoryRawdata->gre1Buf[i]);
    }
    EXAMPLE_DEBUG_LOG_L1("\r\n");

    EXAMPLE_DEBUG_LOG_L1("ir num %d, rawdata = ",
                         pstGh30xFactoryRawdata->ir_num);
    for (GU8 i = 0; i < pstGh30xFactoryRawdata->ir_num; i++)
    {
        printf("%d,", pstGh30xFactoryRawdata->IrBuf[i]);
    }
    EXAMPLE_DEBUG_LOG_L1("\r\n");

    EXAMPLE_DEBUG_LOG_L1("red num %d, rawdata = ",
                         pstGh30xFactoryRawdata->red_num);
    for (GU8 i = 0; i < pstGh30xFactoryRawdata->red_num; i++)
    {
        EXAMPLE_DEBUG_LOG_L1("%d,", pstGh30xFactoryRawdata->redBuf[i]);
    }
    EXAMPLE_DEBUG_LOG_L1("\r\n");

    // code implement by user
}
#endif

/* ble */
#if (__USE_GOODIX_APP__)
/// send string via through profile
uint8_t ble_module_send_data_via_gdcs(uint8_t string[], uint8_t length)
{
    uint8_t ret = GH30X_EXAMPLE_OK_VAL;

    #ifdef GOODIX_DEMO_PLANFORM
    uint8_t buffer[0xFF] = {0};
    uint8_t *ble_ptr = &buffer[MUTLI_PKG_HEADER_LEN - 1];
    uint8_t len =
        length <= __BLE_PKG_SIZE_MAX__ ? length : __BLE_PKG_SIZE_MAX__;

    ble_ptr[0] = 0xAA;
    ble_ptr[1] = 0x11;
    ble_ptr[2] = 0xA2;
    ble_ptr[3] = len;
    memcpy(&ble_ptr[4], string, len);
    buffer[0] = MUTLI_PKG_MAGIC_0_VAL;
    buffer[1] = MUTLI_PKG_MAGIC_1_VAL;
    buffer[2] = len + 4;
    buffer[4 + len + MUTLI_PKG_HEADER_LEN - 1] = MUTLI_PKG_MAGIC_2_VAL;

    ret = UartDataTransfer(buffer, (len + MUTLI_PKG_HEADER_LEN + 4));
    #endif

    return ret;
}

/// Recv data via through profile
void ble_module_recv_data_via_gdcs(uint8_t *data, uint8_t length)
{
    gh30x_app_cmd_parse(data, length);
}

/// Determine whether the APP command is a reset command
static bool gdcs_parse_is_reset_cmd(uint8_t *gdcs_rx_data, uint8_t gdcs_rx_len)
{
    bool cmd_flag = false;
    if ((gdcs_rx_len == 5) && (gdcs_rx_data[0] == 0xDA) &&
        (gdcs_rx_data[1] == 0xAD) && (gdcs_rx_data[2] == 0xEF) &&
        (gdcs_rx_data[3] == 0xFE) && (gdcs_rx_data[4] == 0xA5))
    {
        cmd_flag = true;
    }
    return cmd_flag;
}

/// Determine whether an APP command is a custom command
static bool gdcs_parse_is_custom_cmd(uint8_t *gdcs_rx_data, uint8_t gdcs_rx_len)
{
    bool cmd_flag = false;
    if ((gdcs_rx_len == 5) && (gdcs_rx_data[0] == 0xDA) &&
        (gdcs_rx_data[1] == 0xAD) && (gdcs_rx_data[2] == 0xEF) &&
        (gdcs_rx_data[3] == 0xFE) && (gdcs_rx_data[4] == 0xA6))
    {
        cmd_flag = true;
    }
    return cmd_flag;
}

/// Data processing sent by APP
static void gdcs_rx_data_msg_handler(uint8_t *p_event_data, uint16_t event_size)
{
    #ifdef GOODIX_DEMO_PLANFORM
    if (gdcs_parse_is_reset_cmd(p_event_data, event_size))
    {
        NVIC_SystemReset();
    }
    else if (gdcs_parse_is_custom_cmd(p_event_data, event_size))
    {
    }
    else
    {
        ble_module_recv_data_via_gdcs(p_event_data, event_size);
    }
    #else
    ble_module_recv_data_via_gdcs(p_event_data, event_size);
    #endif
}

/// Analysis of data sent by APP
void ble_module_gdcs_rx_parse(uint8_t *rx_data, uint16_t rx_length)
{
    gdcs_rx_data_msg_handler(rx_data, rx_length);
}

// APP_TIMER_DEF(m_gdcs_repeat_timer_id);
extern uint32_t m_ble_gdcs_repeat_timer_tick;

/// ble repeat send data timer init
void ble_module_repeat_send_timer_init(void)
{
    // code implement by user
    // must register func ble_module_repeat_send_timer_handler as callback
    /* should setup 50ms timer and ble connect interval should < 100ms
     */
    #ifdef GOODIX_DEMO_PLANFORM
    GOODIX_PLANFROM_CREAT_ADT_CONFIRM_ENTITY();
    #endif
}

/// ble repeat send data timer start
void ble_module_repeat_send_timer_start(void)
{
    // code implement by user
    #ifdef GOODIX_DEMO_PLANFORM
    GOODIX_PLANFROM_START_TIMER_ENTITY();
    #endif
}

/// ble repeat send data timer stop
void ble_module_repeat_send_timer_stop(void)
{
    // code implement by user
    #ifdef GOODIX_DEMO_PLANFORM
    GOODIX_PLANFROM_STOP_TIMER_ENTITY();
    #endif
}

/// ble repeat send data timer handler
void ble_module_repeat_send_timer_handler(void)
{
    send_mcu_rawdata_packet_repeat();
}

void gdcs_repeat_msg_handler(void *p_event_data, uint16_t event_size)
{
    ble_module_repeat_send_timer_handler();
}

void gdcs_repeat_timeout_handler(void *p_context)
{
    // code implement by user
    /* must call gdcs_repeat_msg_handler */
}

#endif

/* timer */

/// gh30x fifo int timeout timer handler
void hal_gh30x_fifo_int_timeout_timer_handler(void)
{
    gh30x_fifo_int_timeout_msg_handler();
}

/// gh30x fifo int timeout timer start
void hal_gh30x_fifo_int_timeout_timer_start(void)
{
    // code implement by user
}

/// gh30x fifo int timeout timer stop
void hal_gh30x_fifo_int_timeout_timer_stop(void)
{
    // code implement by user
}

/// gh30x fifo int timeout timer init
void hal_gh30x_fifo_int_timeout_timer_init(void)
{
    // code implement by user
    // must register func gh30x_fifo_int_timeout_timer_handler as callback
    /* should setup timer interval with fifo int freq(e.g. 1s fifo int setup
     * 1080ms timer)
     */
}

/* uart */
#if (__UART_WITH_GOODIX_TOOLS__)
/// recv data via uart
void uart_module_recv_data(uint8_t *data, uint8_t length)
{
    gh30x_uart_cmd_parse(data, length);
}

/// send value via uart
uint8_t uart_module_send_data(uint8_t string[], uint8_t length)
{
    uint8_t ret = GH30X_EXAMPLE_OK_VAL;

    // code implement by user
    return ret;
}
#endif

/* handle cmd with all ctrl cmd @ref EM_COMM_CMD_TYPE, cmd come from goodix APP
 */
void handle_goodix_communicate_cmd(EM_COMM_CMD_TYPE cmd_type)
{
}

/// print dbg log
void example_dbg_log(char *log_string)
{
    LOG_D("%s", log_string);
}

#if (__USER_DYNAMIC_ALGO_MEM_EN__)
void *hal_gh30x_memory_malloc(GU32 size)
{
    EXAMPLE_DEBUG_LOG_L1("malloc %d Btyes\n", size);
    #ifdef GOODIX_DEMO_PLANFORM
    return malloc(size);
    #else
    // code implement by user
    return NULL;
    #endif
}

void hal_gh30x_memory_free(void *pmem)
{
    EXAMPLE_DEBUG_LOG_L1("free mem\n");
    #ifdef GOODIX_DEMO_PLANFORM
    free(pmem);
    #else
        // code implement by user
    #endif
}
#endif

uint32_t gh3018_get_i2c_handle(void)
{
    return (uint32_t)gh3018_i2cbus;
}

uint8_t gh3018_get_dev_addr(void)
{
    return (uint8_t)0x14;
}

int gh3018_self_check(void)
{
// check register first, register 1 should init to 0x14
// init_err_flag = HBD_SimpleInit(&gh30x_init_config); // init gh30x
// if (HBD_RET_OK != init_err_flag)
//{
//    EXAMPLE_DEBUG_LOG_L1("gh30x init error[%s]\r\n",
//    dbg_ret_val_string[DEBUG_HBD_RET_VAL_BASE + init_err_flag]); return
//    GH30X_EXAMPLE_ERR_VAL;
//}

// check gpio
#ifdef PPG_RST_PIN
    if (rt_pin_read(PPG_RST_PIN) == 0)
        return -1;
#else
    if (rt_pin_read(GH3018_RST_PIN) == 0)
        return -1;
#endif

    return 0;
}

uint32_t gh3018_get_hr(void)
{
    return (uint32_t)loc_hb_value;
}

void gh3018_set_hr(uint32_t hr)
{
    if (hr != loc_hb_value)
    {
        loc_hb_value = hr;
    }
}

uint32_t *gh3018_get_ppg(void)
{
    return loc_ppg_buf;
}

uint32_t *gh3018_get_ppg2(void)
{
    return loc_ppg_buf2;
}

uint16_t gh3018_get_ppg_len(void)
{
    return loc_ppg_rawdata_len;
}
/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd.
 * ********/
