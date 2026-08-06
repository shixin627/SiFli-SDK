/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co, Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtdevice.h"
#include "rtthread.h"
#include "stdio.h"
#include "usbd_core.h"
#include "usbd_msc.h"

#if defined(CHERRYUSB_MSC_BACKEND_SPI_MSD)
    #include "spi_msd.h"
#elif !defined(CHERRYUSB_MSC_BACKEND_SDIO)
    #error "Please select CHERRYUSB_MSC_BACKEND_SPI_MSD or CHERRYUSB_MSC_BACKEND_SDIO"
#endif

/*!< endpoint address */
#define MSC_IN_EP 0x85
#define MSC_OUT_EP 0x02

#define USBD_VID 0x38f4
#define USBD_PID 0xFFFF
#define USBD_MAX_POWER 100
#define USBD_LANGID_STRING 1033

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + MSC_DESCRIPTOR_LEN)

#define MSC_MAX_MPS 64
#define SDCARD_READY_RETRY_DELAY_MS 100
#define SDCARD_READY_LOG_INTERVAL_MS 1000

#ifndef CHERRYUSB_DEVICE_MSC_DEVNAME
    #define CHERRYUSB_DEVICE_MSC_DEVNAME "sd0"
#endif

#if defined(CHERRYUSB_MSC_BACKEND_SPI_MSD)
    #define MSC_BACKEND_NAME "SPI MSD"
#else
    #define MSC_BACKEND_NAME "SDIO/MMC"
#endif

static const uint8_t device_descriptor[] =
{
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID,
                               0x0200, 0x01),
};

static const uint8_t config_descriptor_fs[] =
{
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01,
                               USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, MSC_MAX_MPS, 0x02),
};

static const uint8_t device_quality_descriptor[] =
{
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, 0x01),
};

static const char *string_descriptors[] =
{
    (const char[]){0x09, 0x04}, /* Langid */
    "SiFli",                    /* Manufacturer */
    "SiFli MSC DEMO",           /* Product */
    "2025090317",               /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return config_descriptor_fs;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (sizeof(string_descriptors) / sizeof(char *)))
    {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor msc_device_descriptor =
{
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;

    switch (event)
    {
    case USBD_EVENT_RESET:
        break;
    case USBD_EVENT_CONNECTED:
        break;
    case USBD_EVENT_DISCONNECTED:
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

static rt_device_t g_msc_dev = RT_NULL;

static uint32_t g_block_size = 0;
static uint32_t g_block_num = 0;
static volatile int g_msc_ready = 0;
static rt_bool_t g_bad_geometry_logged = RT_FALSE;

#if defined(CHERRYUSB_MSC_BACKEND_SPI_MSD)
extern int rt_spi_msd_init(void);
#endif

static void msc_backend_probe(void)
{
#if defined(CHERRYUSB_MSC_BACKEND_SPI_MSD)
    if (rt_device_find(CHERRYUSB_DEVICE_MSC_DEVNAME) == RT_NULL)
    {
        (void)rt_spi_msd_init();
    }
#endif
}

/* Try a single, non-blocking probe/open. Return 0 on success, -1 otherwise. */
static int msc_backend_try_open_once(void)
{
    struct rt_device_blk_geometry geometry;
    rt_err_t result;

    if (g_msc_ready)
    {
        return 0;
    }

    if (g_msc_dev == RT_NULL)
    {
        g_msc_dev = rt_device_find(CHERRYUSB_DEVICE_MSC_DEVNAME);
        if (g_msc_dev == RT_NULL)
        {
            return -1;
        }
    }

    result = rt_device_open(g_msc_dev, RT_DEVICE_OFLAG_RDWR);
    if (result != RT_EOK)
    {
        g_msc_dev = RT_NULL;
        return -1;
    }

    result = rt_device_control(g_msc_dev, RT_DEVICE_CTRL_BLK_GETGEOME,
                               &geometry);
    if (result != RT_EOK)
    {
        rt_device_close(g_msc_dev);
        g_msc_dev = RT_NULL;
        return -1;
    }

    g_block_size = geometry.bytes_per_sector;
    g_block_num = geometry.sector_count;
    if (g_block_size == 0 || g_block_num == 0 ||
            (CONFIG_USBDEV_MSC_MAX_BUFSIZE % g_block_size) != 0)
    {
        if (!g_bad_geometry_logged)
        {
            rt_kprintf("MSC: invalid '%s' geometry: block %d bytes, total %d blocks\n",
                       CHERRYUSB_DEVICE_MSC_DEVNAME, g_block_size, g_block_num);
            g_bad_geometry_logged = RT_TRUE;
        }
        rt_device_close(g_msc_dev);
        g_msc_dev = RT_NULL;
        return -1;
    }

    g_bad_geometry_logged = RT_FALSE;
    g_msc_ready = 1;
    rt_kprintf("MSC: %s disk '%s' ready (%d blocks * %d bytes = %d MB)\n",
               MSC_BACKEND_NAME, CHERRYUSB_DEVICE_MSC_DEVNAME,
               g_block_num, g_block_size,
               (g_block_num / 2048) * (g_block_size / 512));
    return 0;
}

/* Wait until the block device is ready before exposing MSC to the host. */
static void msc_backend_wait_ready(void)
{
    rt_tick_t last_log_tick = rt_tick_get();
    rt_tick_t last_retry_tick = last_log_tick;
    rt_tick_t retry_interval =
        rt_tick_from_millisecond(SDCARD_READY_LOG_INTERVAL_MS);
    rt_bool_t retry_now = RT_TRUE;

    rt_kprintf("MSC: waiting for %s disk '%s' ...\n", MSC_BACKEND_NAME,
               CHERRYUSB_DEVICE_MSC_DEVNAME);
    while (!g_msc_ready)
    {
        if (msc_backend_try_open_once() == 0)
        {
            return;
        }

        if (retry_now || (rt_tick_get() - last_retry_tick) >= retry_interval)
        {
            msc_backend_probe();
            retry_now = RT_FALSE;
            last_retry_tick = rt_tick_get();
        }

        rt_thread_mdelay(SDCARD_READY_RETRY_DELAY_MS);
        if ((rt_tick_get() - last_log_tick) >= retry_interval)
        {
            rt_kprintf("MSC: waiting for %s disk '%s' ...\n",
                       MSC_BACKEND_NAME, CHERRYUSB_DEVICE_MSC_DEVNAME);
            last_log_tick = rt_tick_get();
        }
    }
}

void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num,
                      uint32_t *block_size)
{
    (void)busid;
    (void)lun;

    if (!g_msc_ready)
    {
        /* Try a quick lazy probe (non-blocking) */
        (void)msc_backend_try_open_once();
    }

    if (!g_msc_ready)
    {
        *block_num = 0;  /* Report no media to host */
        *block_size = 0; /* Host will poll periodically */
    }
    else
    {
        *block_num = g_block_num;
        *block_size = g_block_size;
    }
}

int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector,
                         uint8_t *buffer, uint32_t length)
{
    (void)busid;
    (void)lun;

    rt_size_t sectors_read;
    uint32_t num_sectors_to_read;

    if (!g_msc_ready)
    {
        /* Attempt lazy open; still fail fast if not ready */
        if (msc_backend_try_open_once() != 0)
        {
            return -1; /* Not ready */
        }
    }

    if (g_block_size == 0 || (length % g_block_size) != 0)
    {
        return -1;
    }

    num_sectors_to_read = length / g_block_size;

    /* rt_device_read's `pos` is the sector number and `size` is the number of
     * sectors */
    sectors_read = rt_device_read(g_msc_dev, sector, buffer, num_sectors_to_read);

    if (sectors_read == num_sectors_to_read)
    {
        return 0; /* Success */
    }

    rt_kprintf("E: sector_read failed, expected %d, got %d\n",
               num_sectors_to_read, sectors_read);
    return -1; /* Error */
}

int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector,
                          uint8_t *buffer, uint32_t length)
{
    (void)busid;
    (void)lun;

    rt_size_t sectors_written;
    uint32_t num_sectors_to_write;

    if (!g_msc_ready)
    {
        if (msc_backend_try_open_once() != 0)
        {
            return -1; /* Not ready */
        }
    }

    if (g_block_size == 0 || (length % g_block_size) != 0)
    {
        return -1;
    }

    num_sectors_to_write = length / g_block_size;

    /* rt_device_write's `pos` is the sector number and `size` is the number of
     * sectors */
    sectors_written =
        rt_device_write(g_msc_dev, sector, buffer, num_sectors_to_write);

    if (sectors_written == num_sectors_to_write)
    {
        return 0; /* Success */
    }

    rt_kprintf("E: sector_write failed, expected %d, got %d\n",
               num_sectors_to_write, sectors_written);
    return -1; /* Error */
}

static struct usbd_interface intf0;

void msc_device_init(uint8_t busid, uint32_t reg_base)
{

    msc_backend_wait_ready();

    usbd_desc_register(busid, &msc_device_descriptor);
    usbd_add_interface(busid,
                       usbd_msc_init_intf(busid, &intf0, MSC_OUT_EP, MSC_IN_EP));

    usbd_initialize(busid, reg_base, usbd_event_handler);
}
