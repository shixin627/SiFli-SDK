/**
*********************************************************************************************************
*               Copyright(c) 2024, Skaiwalk Corporation. All rights reserved.
**********************************************************************************************************
* @file     virtual_console.c
* @brief
* @details
* @author   shixin
* @date     2024-08-13
* @version  v1.0
*********************************************************************************************************
*/
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_usart.h"
#include "virtual_console.h"

#ifdef SOC_BF0_HCPU
#define VUART1_CONFIG            \
    {                            \
        .name = "vuart1",        \
        .Instance = USART1,      \
        .irq_type = USART1_IRQn, \
    }
#else
#define VUART4_CONFIG            \
    {                            \
        .name = "vuart4",        \
        .Instance = USART4,      \
        .irq_type = USART4_IRQn, \
    }
#endif

static rt_err_t drv_uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg);
static rt_err_t drv_uart_control(struct rt_serial_device *serial, int cmd, void *arg);
static int drv_uart_putc(struct rt_serial_device *serial, char c);
static int drv_uart_getc(struct rt_serial_device *serial);
static rt_size_t drv_uart_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction);
const struct rt_uart_ops _uart_opss =
    {
        drv_uart_configure,
        drv_uart_control,
        drv_uart_putc,
        drv_uart_getc,
        drv_uart_dma_transmit};
rt_err_t drv_uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    return (RT_EOK);
}
rt_err_t drv_uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    return (RT_EOK);
}
int drv_uart_putc(struct rt_serial_device *serial, char c)
{
    return (1);
}
int drv_uart_getc(struct rt_serial_device *serial)
{
    return (-1);
}
rt_size_t drv_uart_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
    return (0);
}
static struct rt_serial_device serialv;

static struct sifli_uart_config uartv[] =
    {
#ifdef SOC_BF0_HCPU
        VUART1_CONFIG,
#else
        VUART4_CONFIG,
#endif
};

int virtul_uart_init(void)
{
    struct rt_serial_device *serial = &serialv;
    struct sifli_uart_config *uart = (struct sifli_uart_config *)&uartv;
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    serial->ops = &_uart_opss;
    serial->config = cfg;

#ifdef SOC_BF0_HCPU
    const char *name = "vuart1";
#else
    const char *name = "vuart4";
#endif

    rt_hw_serial_register(serial, name,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);

    /* drv_common.c sets the console to vuart1/vuart4 inside rt_hw_board_init,
     * which runs before INIT_BOARD callbacks. At that point this device is
     * not yet registered, so the call silently fails and the console stays
     * NULL. Re-bind it now that the device exists. */
    rt_console_set_device(name);
    return 0;
}
INIT_BOARD_EXPORT(virtul_uart_init);