#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"

/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("Hello world!\n");

    /* Infinite loop */
    while (1)
    {
        // Delay for 1000 ms to yield CPU time to other threads
        rt_thread_mdelay(1000);
    }
    return 0;
}

