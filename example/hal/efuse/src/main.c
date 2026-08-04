/*
 * SPDX-FileCopyrightText: 2025-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <rtdevice.h>
#include "log.h"

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Infinite loop */
    while (1)
    {
        /* Do nothing. */
        rt_thread_mdelay(1000);
    }

    return 0;
}

static int efuse_write(int argc, char *argv[])
{
    int pos;
    uint8_t *buf;
    int len;
    int32_t r;
    uint32_t byte_len;

    if (argc < 4)
    {
        rt_kprintf("wrong arguments\n");
        rt_kprintf("argument list:\n");
        rt_kprintf("  pos: bit position\n");
        rt_kprintf("  len: bit length\n");
        rt_kprintf("  data: data in hex, e.g. 0102, 0x01 is the first byte written to [pos], 0x02 is the second byte written to [pos+8]\n");
        return -1;
    }

    pos = atoi(argv[1]);
    len = atoi(argv[2]);
    byte_len = (len + 7) >> 3;
    buf = rt_malloc(byte_len);
    RT_ASSERT(buf);
    hex2data(argv[3], buf, byte_len);

    r = HAL_EFUSE_Write2(pos, (uint8_t *)buf, len);
    if (r != len)
    {
        rt_kprintf("write fail:%d\n", r);
    }

    return 0;
}
MSH_CMD_EXPORT(efuse_write, efuse_write)

static int efuse_read(int argc, char *argv[])
{
    int pos;
    int len;
    uint8_t data[32];
    int32_t r;

    if (argc < 3)
    {
        rt_kprintf("wrong arguments\n");
        return -1;

    }

    pos = atoi(argv[1]);
    len = atoi(argv[2]);

    r = HAL_EFUSE_Read2(pos, data, len);
    if (r != len)
    {
        rt_kprintf("read fail:%d\n", r);
    }
    else
    {
        LOG_HEX("", 8, data, (len + 7) / 8);
    }

    return 0;
}
MSH_CMD_EXPORT(efuse_read, efuse_read)

