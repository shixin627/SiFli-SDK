/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co, Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"
#include "board.h"
#include "button.h"
#include "rtdevice.h"
#include "rtthread.h"
#include "usbd_core.h"
#include "usbd_hid.h"
#include <math.h>

/*!< endpoint address */
#define HID_INT_EP 0x81
#define HID_INT_EP_SIZE 8
#define HID_INT_EP_INTERVAL 10
#define HID_BUSID 0

#define USBD_VID 0x38f4
#define USBD_PID 0xFFFF
#define USBD_MAX_POWER 100

/*!< config descriptor size */
#define USB_HID_CONFIG_DESC_SIZ 34
/*!< report descriptor size */
#define HID_MOUSE_REPORT_DESC_SIZE 74

static const uint8_t device_descriptor[] = {USB_DEVICE_DESCRIPTOR_INIT(
        USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0002, 0x01)
                                           };

static const uint8_t config_descriptor[] =
{
    USB_CONFIG_DESCRIPTOR_INIT(USB_HID_CONFIG_DESC_SIZ, 0x01, 0x01,
                               USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),

    /************** Descriptor of HID mouse interface ****************/
    /* 09 */
    0x09,                          /* bLength: Interface Descriptor size */
    USB_DESCRIPTOR_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type
                                    */
    0x00,                          /* bInterfaceNumber: Number of Interface */
    0x00,                          /* bAlternateSetting: Alternate setting */
    0x01,                          /* bNumEndpoints */
    0x03,                          /* bInterfaceClass: HID */
    0x01,                          /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x01, /* nInterfaceProtocol : 0=none, 1=mouse, 2=mouse */
    0,    /* iInterface: Index of string descriptor */
    /******************** Descriptor of HID ********************/
    /* 18 */
    0x09,                    /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE_HID, /* bDescriptorType: HID */
    0x11,                    /* bcdHID: HID Class Spec release number */
    0x01, 0x00,              /* bCountryCode: Hardware target country */
    0x01, /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22, /* bDescriptorType */
    HID_MOUSE_REPORT_DESC_SIZE, /* wItemLength: Total length of Report
                                   descriptor */
    0x00,
    /******************** Descriptor of endpoint ********************/
    /* 27 */
    0x07,                         /* bLength: Endpoint Descriptor size */
    USB_DESCRIPTOR_TYPE_ENDPOINT, /* bDescriptorType: */
    HID_INT_EP,                   /* bEndpointAddress: Endpoint Address (IN) */
    0x03,                         /* bmAttributes: Interrupt endpoint */
    HID_INT_EP_SIZE,              /* wMaxPacketSize: 8 Byte max */
    0x00, HID_INT_EP_INTERVAL,    /* bInterval: Polling Interval */
};

static const uint8_t device_quality_descriptor[] =
{
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, 0x01),
};

static const char *string_descriptors[] =
{
    (const char[]){0x09, 0x04}, /* Langid */
    "SiFli",                    /* Manufacturer */
    "SiFli HID DEMO",           /* Product */
    "2025101418",               /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    if (index >= (sizeof(string_descriptors) / sizeof(char *)))
    {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor hid_descriptor =
{
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback
};

static const uint8_t hid_mouse_report_desc[HID_MOUSE_REPORT_DESC_SIZE] =
{
    0x05, 0x01, /*     USAGE_PAGE (Generic Desktop) */
    0x09, 0x02, /*     USAGE (Mouse) */
    0xA1, 0x01, /*     COLLECTION (Application) */
    0x09, 0x01, /*     USAGE (Pointer) */

    0xA1, 0x00, /*     COLLECTION (Physical) */
    0x05, 0x09, /*     USAGE_PAGE (Button) */
    0x19, 0x01, /*     USAGE_MINIMUM (Button 1) */
    0x29, 0x03, /*     USAGE_MAXIMUM (Button 3) */

    0x15, 0x00, /*     LOGICAL_MINIMUM (0) */
    0x25, 0x01, /*     LOGICAL_MAXIMUM (1) */
    0x95, 0x03, /*     REPORT_COUNT (3) */
    0x75, 0x01, /*     REPORT_SIZE (1) */

    0x81, 0x02, /*     INPUT (Data,Var,Abs) */
    0x95, 0x01, /*     REPORT_COUNT (1) */
    0x75, 0x05, /*     REPORT_SIZE (5) */
    0x81, 0x01, /*     INPUT (Cnst,Var,Abs) */

    0x05, 0x01, /*     USAGE_PAGE (Generic Desktop) */
    0x09, 0x30, /*     USAGE (X) */
    0x09, 0x31, /*     USAGE (Y) */
    0x09, 0x38,

    0x15, 0x81, /*     LOGICAL_MINIMUM (-127) */
    0x25, 0x7F, /*     LOGICAL_MAXIMUM (127) */
    0x75, 0x08, /*     REPORT_SIZE (8) */
    0x95, 0x03, /*     REPORT_COUNT (2) */

    0x81, 0x06, /*     INPUT (Data,Var,Rel) */
    0xC0, 0x09, 0x3c, 0x05, 0xff, 0x09,

    0x01, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95,

    0x02, 0xb1, 0x22, 0x75, 0x06, 0x95, 0x01, 0xb1,

    0x01, 0xc0 /*   END_COLLECTION */
};

/*!< mouse report struct */
typedef struct
{
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheel;
} hid_mouse_t;

/*!< mouse report */
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX hid_mouse_t mouse_cfg;

/* Byte corresponding to mouse button */
#define left_button 0x01
#define right_button 0x02
#define middle_button 0x04

/* HID state */
#define HID_STATE_IDLE 0
#define HID_STATE_BUSY 1

/* packet size */
#define PACKET_SIZE 4

static volatile uint8_t hid_state = HID_STATE_IDLE;

static volatile bool device_configured = false;
static rt_thread_t mouse_thread = RT_NULL;
static rt_sem_t mouse_key_sem = RT_NULL;

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[8];

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event)
    {
    case USBD_EVENT_RESET:
        hid_state = HID_STATE_IDLE;
        break;
    case USBD_EVENT_CONNECTED:
        break;
    case USBD_EVENT_DISCONNECTED:
        device_configured = false;
        hid_state = HID_STATE_IDLE;
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        device_configured = true;
        hid_state = HID_STATE_IDLE;
        rt_kprintf("USB HID Mouse configured!\n");
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;
    default:
        break;
    }
}

/* function ------------------------------------------------------------------*/
void usbd_hid_int_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    hid_state = HID_STATE_IDLE;
    mouse_cfg.x = 0;
    mouse_cfg.y = 0;
    mouse_cfg.wheel = 0;
}

/*!< endpoint call back */
static struct usbd_endpoint hid_in_ep = {.ep_cb = usbd_hid_int_callback,
           .ep_addr = HID_INT_EP
};

static void send_hid_report(uint8_t buttons, int8_t x, int8_t y, int8_t wheel)
{
    if (!device_configured || hid_state != HID_STATE_IDLE)
        return;

    mouse_cfg.buttons = buttons;
    mouse_cfg.x = x;
    mouse_cfg.y = y;
    mouse_cfg.wheel = wheel;

    hid_state = HID_STATE_BUSY;
    usbd_ep_start_write(HID_BUSID, HID_INT_EP, (uint8_t *)&mouse_cfg,
                        PACKET_SIZE);
}

static void demo_left_button_click(void)
{
    rt_kprintf("[MOUSE] left button click\n");
    send_hid_report(left_button, 0, 0, 0); /*left button down*/
    rt_thread_mdelay(10);
    send_hid_report(0, 0, 0, 0); /*left button up*/
    rt_thread_mdelay(10);
}

static void demo_right_button_click(void)
{
    rt_kprintf("[MOUSE] right button click\n");
    send_hid_report(right_button, 0, 0, 0); /*right button down*/
    rt_thread_mdelay(10);
    send_hid_report(0, 0, 0, 0); /*right button up*/
    rt_thread_mdelay(10);
}

static void demo_middle_button_click(void)
{
    rt_kprintf("[MOUSE] middle button click\n");
    send_hid_report(middle_button, 0, 0, 0); /*middle button down*/
    rt_thread_mdelay(10);
    send_hid_report(0, 0, 0, 0); /*middle button up*/
    rt_thread_mdelay(10);
}

static void demo_scroll_wheel(void)
{
    rt_kprintf("[MOUSE] scroll wheel\n");
    send_hid_report(0, 0, 0, 1); /* scroll wheel up */
    rt_thread_mdelay(100);
    send_hid_report(0, 0, 0, 1);
    rt_thread_mdelay(100);
    send_hid_report(0, 0, 0, -1); /* scroll wheel down */
    rt_thread_mdelay(100);
    send_hid_report(0, 0, 0, -1);
    rt_thread_mdelay(100);
}

static void demo_move_in_circle(void)
{
    rt_kprintf("[MOUSE] move in circle\n");
    for (int i = 0; i < 36; i++)
    {
        int8_t x = (int8_t)(10 * cos(i * 10 * 3.14 / 180));
        int8_t y = (int8_t)(10 * sin(i * 10 * 3.14 / 180));
        send_hid_report(0, x, y, 0);
        rt_thread_mdelay(20);
    }
    send_hid_report(0, 0, 0, 0);
    rt_thread_mdelay(10);
}

static void demo_drag_left_button(void)
{
    rt_kprintf("[MOUSE] Left click and drag\n");

    send_hid_report(0x01, 0, 0, 0); /* left button down */

    for (int i = 0; i < 50; i++)
    {
        send_hid_report(0x01, 2, 0, 0);
        rt_thread_mdelay(10);
    }

    send_hid_report(0x00, 0, 0, 0);
}

static void key_button_handler(int32_t pin, button_action_t action)
{
    if ((pin != BSP_KEY2_PIN) || (action != BUTTON_CLICKED))
    {
        return;
    }

    if (!device_configured)
    {
        rt_kprintf("USB HID mouse is not ready yet\n");
        return;
    }

    if (mouse_key_sem != RT_NULL)
    {
        rt_sem_release(mouse_key_sem);
    }
}

static void key_button_init(void)
{
    button_cfg_t cfg = {0};

    cfg.pin = BSP_KEY2_PIN;
    cfg.mode = PIN_MODE_INPUT;
#ifdef BSP_KEY2_ACTIVE_HIGH
    cfg.active_state = BUTTON_ACTIVE_HIGH;
#else
    cfg.active_state = BUTTON_ACTIVE_LOW;
#endif
    cfg.button_handler = key_button_handler;

    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));

    rt_kprintf("KEY2 button initialized, pin=%d\n", BSP_KEY2_PIN);
}

static void mouse_thread_entry(void *parameter)
{
    rt_kprintf("mouse thread started\n");

    rt_thread_mdelay(3000);

    uint8_t demo_index = 0;

    while (1)
    {
        rt_sem_take(mouse_key_sem, RT_WAITING_FOREVER);

        while (device_configured && (hid_state != HID_STATE_IDLE))
        {
            rt_thread_mdelay(5);
        }

        if (!device_configured)
        {
            continue;
        }

        switch (demo_index)
        {
        case 0:
            demo_left_button_click();
            break;
        case 1:
            demo_right_button_click();
            break;
        case 2:
            demo_middle_button_click();
            break;
        case 3:
            demo_scroll_wheel();
            break;
        case 4:
            demo_move_in_circle();
            break;
        case 5:
            demo_drag_left_button();
            break;
        default:
            break;
        }

        demo_index = (demo_index + 1) % 6;
    }
    return;
}

struct usbd_interface intf0;

static void hid_mouse_init(uint8_t busid, uintptr_t reg_base)
{

    usbd_desc_register(busid, &hid_descriptor);

    usbd_add_interface(busid,
                       usbd_hid_init_intf(busid, &intf0, hid_mouse_report_desc,
                                          HID_MOUSE_REPORT_DESC_SIZE));

    usbd_add_endpoint(busid, &hid_in_ep);

    usbd_initialize(busid, reg_base, usbd_event_handler);

    /*!< init mouse report data */
    mouse_cfg.buttons = 0;
    mouse_cfg.wheel = 0;
    mouse_cfg.x = 0;
    mouse_cfg.y = 0;
}

int main(void)
{
    rt_kprintf("========================================\n");
    rt_kprintf("CherryUSB HID mouse Demo\n");
    rt_kprintf("========================================\n");

    hid_mouse_init(0, (uintptr_t)USBC_BASE);

    mouse_key_sem = rt_sem_create("mkey", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mouse_key_sem != RT_NULL);
    key_button_init();

    mouse_thread = rt_thread_create("mouse_thread", mouse_thread_entry, RT_NULL,
                                    2048, RT_THREAD_PRIORITY_MAX / 2, 20);

    if (mouse_thread != RT_NULL)
    {
        rt_thread_startup(mouse_thread);
        rt_kprintf("mouse thread created successfully!\n");
    }
    else
    {
        rt_kprintf("Failed to create mouse thread!\n");
        return -1;
    }

    while (1)
    {
        rt_thread_mdelay(1000);
    }

    return 0;
}
