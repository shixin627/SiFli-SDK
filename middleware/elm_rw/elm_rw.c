/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _WIN32

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "bf0_hal.h"
#include "drv_flash.h"
#include <drv_log.h>
#include <dfs_fs.h>
#include <dfs_file.h>
#include <dfs_posix.h>
#include "finsh.h"
//#include <app_fat.h>
//#include <app_manager.h>
//#include <app_clock_comm.h>

#define WF_INSTAL_TEST  0

#define MAX_BLOCK_LEN   (8*1024)
//uint8_t elm_data[MAX_BLOCK_LEN];
static int elm_fptr = -1;
static uint8_t *p_buf = RT_NULL;

#define CLOSE_ELM_FILE \
        if(elm_fptr != -1) \
        { \
            close(elm_fptr); \
            elm_fptr = -1; \
        }

#define FREE_ELM_BUF \
        if(p_buf != RT_NULL) \
        { \
            rt_free(p_buf); \
            p_buf = RT_NULL; \
        }

unsigned int getCrc(unsigned char *pSrc, int len, unsigned int crc)
{
    //unsigned int crc = 0xffffffff;

    for (int m = 0; m < len; m++)
    {
        crc ^= ((unsigned int)pSrc[m]) << 24;

        for (int n = 0; n < 8; n++)
        {
            if (crc & 0x80000000)
            {
                crc = (crc << 1) ^ 0x04c11db7;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

RT_WEAK void log_pause(rt_bool_t pause)
{

}

RT_WEAK void ulog_pause(int pause)
{

}

void local_log_pause(int pause)
{
    log_pause(pause);
    //ulog_pause(pause);
}

static int elm_trans_in(char *file_path, char *file_size, char *crc_str)
{
    uint32_t len, off, size, crc1, crc2, start, end;
    int delta;
    int res;
    int cnt;
    int pathlen = strlen(file_path);
    char *pTemp;
#if RT_USING_CONSOLE
    rt_device_t pDev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
#else
    rt_device_t pDev = rt_device_find("uart1");
#endif

    size = strtoul(file_size, 0, 16);
    crc1 = strtoul(crc_str, 0, 16);
    cnt = (size + MAX_BLOCK_LEN - 1) / MAX_BLOCK_LEN;
    crc2 = 0xffffffff;

    CLOSE_ELM_FILE
    FREE_ELM_BUF

    pTemp = (char *)rt_malloc(pathlen + 1);
    if (pTemp == RT_NULL)
    {
        rt_kprintf("pTemp malloc err: %d\n", pathlen);
        rt_kprintf("elm_trans_in FAIL\n");
        return RT_ERROR;
    }
    memset(pTemp, 0, pathlen + 1);
    for (int m = 1; m < pathlen; m++)
    {
        if (file_path[m] == '/')
        {
            memcpy(pTemp, file_path, m);
            if (0 != access(pTemp, 0) && 0 != mkdir(pTemp, 0))
            {
                rt_kprintf("create folder error: %s\n", pTemp);
                rt_kprintf("elm_trans_in FAIL\n");
                free(pTemp);
                return RT_ERROR;
            }
        }
    }
    free(pTemp);

    elm_fptr =  open(file_path, O_RDWR | O_TRUNC | O_CREAT | O_BINARY, 0);
    if (elm_fptr < 0)
    {
        rt_kprintf("open file err: %s\n", file_path);
        rt_kprintf("elm_trans_in FAIL\n");
        return RT_ERROR;
    }

    /*
        if (0 != ioctl(elm_fptr, F_RESERVE_CONT_SPACE, size))
        {
            CLOSE_ELM_FILE

            rt_kprintf("set continue space error:%d\n", size);
            rt_kprintf("elm_trans_in FAIL\n");

            return RT_ERROR;
        }
    */
    p_buf = (uint8_t *)rt_malloc(MAX_BLOCK_LEN);
    if (p_buf == RT_NULL)
    {
        CLOSE_ELM_FILE

        rt_kprintf("p_buf malloc err: %d\n", MAX_BLOCK_LEN);
        rt_kprintf("elm_trans_in FAIL\n");

        return RT_ERROR;
    }

    local_log_pause(1);

    for (int m = 0; m < cnt; m++)
    {
        //rt_kprintf("elm_trans_in_waitrx\n");
        rt_device_write(pDev, 0, "elm_trans_in_waitrx\n", strlen("elm_trans_in_waitrx\n"));
        off = 0;
        len = MAX_BLOCK_LEN;
        if ((m == cnt - 1) && (size % MAX_BLOCK_LEN))
        {
            len = size % MAX_BLOCK_LEN;
        }

        start = rt_tick_get();
        while (1)
        {
            delta = rt_device_read(pDev, off, &p_buf[off], len - off);
            off += delta;
            if (off >= len)
            {
                crc2 = getCrc(p_buf, len, crc2);
                res = write(elm_fptr, p_buf, len);
                if (res != len)
                {
                    local_log_pause(0);

                    CLOSE_ELM_FILE
                    FREE_ELM_BUF

                    rt_kprintf("write file err: %d != %d\n", res, len);
                    rt_kprintf("elm_trans_in FAIL\n");

                    return RT_ERROR;
                }
                break;
            }
            end = rt_tick_get();
            if (end - start > 2000)
            {
                local_log_pause(0);

                CLOSE_ELM_FILE
                FREE_ELM_BUF

                rt_kprintf("rx data outof timer 2s\n");
                rt_kprintf("elm_trans_in FAIL\n");

                return RT_ERROR;
            }
        }
    }

    local_log_pause(0);

    CLOSE_ELM_FILE
    FREE_ELM_BUF

    if (crc2 == crc1)
    {
        rt_kprintf("crc2(0x%08x) == crc1(0x%08x)\n", crc2, crc1);
        rt_kprintf("elm_trans_in OK\n");
    }
    else
    {
        rt_kprintf("crc2(0x%08x) != crc1(0x%08x)\n", crc2, crc1);
        rt_kprintf("elm_trans_in FAIL\n");
    }


    return RT_EOK;
}

static int elm_trans_info(char *file_path)
{
    int res;
    struct stat file_stat;

    CLOSE_ELM_FILE
    FREE_ELM_BUF

    res = stat(file_path, &file_stat);
    if (0 != res)
    {
        rt_kprintf("stat error\n");
        rt_kprintf("elm_trans_info FAIL\n");
        return RT_ERROR;
    }
    rt_kprintf("elm_trans_len 0x%08x\n", file_stat.st_size);
    rt_kprintf("elm_trans_info OK\n");

    return 0;
}

static int elm_check_size(char *file_path, char *size1, char *size2)
{
    int result;
    struct statfs buffer;

    if (0 != access(file_path, 0) && 0 != mkdir(file_path, 0))
    {
        rt_kprintf("create folder error\n");
        rt_kprintf("elm_check_size FAIL\n");
        return -1;
    }

    result = dfs_statfs(file_path, &buffer);
    if (result != 0)
    {
        rt_kprintf("dfs_statfs error\n");
        rt_kprintf("elm_check_size FAIL\n");
        return -1;
    }

    if (buffer.f_bsize <= 0x800)
    {
        rt_kprintf("f_bsize:0x%x f_bfree:0x%x wfsize:%s\n", buffer.f_bsize, buffer.f_bfree, size1);
        if (buffer.f_bfree * buffer.f_bsize >= strtoul(size1, 0, 16))
        {
            rt_kprintf("elm_check_size OK\n");
        }
        else
        {
            rt_kprintf("elm_check_size FAIL\n");
        }
    }
    else
    {
        rt_kprintf("f_bsize:0x%x f_bfree:0x%x wfsize:%s\n", buffer.f_bsize, buffer.f_bfree, size2);
        if (buffer.f_bfree * buffer.f_bsize >= strtoul(size2, 0, 16))
        {
            rt_kprintf("elm_check_size OK\n");
        }
        else
        {
            rt_kprintf("elm_check_size FAIL\n");
        }
    }

    return 0;
}

static int elm_trans_out(char *file_path, char *file_size)
{
    int delta, res, cnt;
    int flag = 0;
    int match = 0;
    uint32_t len, off, size, crc;
    int cmdlen = strlen("elm_trans_out_waitrx");

#if RT_USING_CONSOLE
    rt_device_t pDev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
#else
    rt_device_t pDev = rt_device_find("uart1");
#endif
    int oflag = pDev->open_flag;

    if (oflag & RT_DEVICE_FLAG_DMA_TX)
    {
        rt_device_close(pDev);
        rt_device_open(pDev, oflag & (~RT_DEVICE_FLAG_DMA_TX));
    }


    size = strtoul(file_size, 0, 16);
    cnt = (size + MAX_BLOCK_LEN - 1) / MAX_BLOCK_LEN;
    crc = 0xffffffff;

    CLOSE_ELM_FILE
    FREE_ELM_BUF

    elm_fptr = open(file_path, O_RDWR | O_BINARY, 0);
    if (elm_fptr < 0)
    {
        rt_kprintf("open error\n");
        rt_kprintf("elm_trans_out FAIL\n");
        //return RT_ERROR;
        goto FUNC_END;
    }

    p_buf = (uint8_t *)rt_malloc(MAX_BLOCK_LEN);
    if (p_buf == RT_NULL)
    {
        CLOSE_ELM_FILE

        rt_kprintf("p_buf malloc err: %d\n", MAX_BLOCK_LEN);
        rt_kprintf("elm_trans_out FAIL\n");

        //return RT_ERROR;
        goto FUNC_END;
    }

    local_log_pause(1);

    rt_device_write(pDev, 0, "elm_trans_out START\n", strlen("elm_trans_out START\n"));

    if (pDev->open_flag & RT_DEVICE_FLAG_STREAM)
    {
        pDev->open_flag &= ~RT_DEVICE_FLAG_STREAM;
        flag = 1;
    }

    off = 0;
    while (1)
    {
        delta = rt_device_read(pDev, off, &p_buf[off], cmdlen - off);
        off += delta;
        if (off >= cmdlen)
        {
            for (int m = 0; m < cmdlen / 2; m++)
            {
                if (memcmp(&p_buf[m], "elm_trans_out_waitrx", cmdlen - m) == 0)
                {
                    if (m != 0)
                    {
                        rt_device_read(pDev, off, &p_buf[off], m);
                    }
                    break;
                }
            }
            break;
        }
    }

    for (int m = 0; m < cnt; m++)
    {

        len = MAX_BLOCK_LEN;
        off = 0;
        if ((m == cnt - 1) && (size % MAX_BLOCK_LEN))
        {
            len = size % MAX_BLOCK_LEN;
        }

        res = read(elm_fptr, &p_buf[0], len);
        crc = getCrc(p_buf, res, crc);
        if (res != len)
        {
            local_log_pause(0);

            CLOSE_ELM_FILE
            FREE_ELM_BUF

            rt_kprintf("read err: res(%d) > len(%d)\n", res, len);
            rt_kprintf("elm_trans_out FAIL\n");

            //return RT_ERROR;
            goto FUNC_END;

        }

        rt_device_write(pDev, 0, &p_buf[0], len);

    }

    local_log_pause(0);

    CLOSE_ELM_FILE
    FREE_ELM_BUF

    rt_kprintf("elm_trans_crc 0x%08x\n", crc);
    rt_kprintf("elm_trans_out OK\n");

FUNC_END:
    if (flag == 1)
    {
        pDev->open_flag |= RT_DEVICE_FLAG_STREAM;
    }

    if (oflag & RT_DEVICE_FLAG_DMA_TX)
    {
        rt_device_close(pDev);
        rt_device_open(pDev, oflag);
    }

    return 0;
}

extern void *get_disp_buf(uint32_t size);
//extern rt_err_t app_lv_task_mutex_take(void);
//extern rt_err_t app_lv_task_mutex_release(void);
#ifdef LVGL_V8
#include "lvsf.h"

static int framebuf_trans_out()
{
    int delta, res, cnt;
    int flag = 0;
    int match = 0;
    uint32_t len, off, size, crc;
    int cmdlen = strlen("framebuf_trans_out_waitrx");
    lv_coord_t hor_res = lv_disp_get_hor_res(NULL);
    lv_coord_t ver_res = lv_disp_get_ver_res(NULL);

    lv_disp_t *disp = _lv_refr_get_disp_refreshing();
    if (disp && disp->driver && disp->driver->wait_cb) disp->driver->wait_cb(disp->driver);

    uint8_t *frame_buf = get_disp_buf(hor_res * ver_res * LV_COLOR_DEPTH / 8);

    if (frame_buf == NULL)
    {
        rt_kprintf("open error\n");
        rt_kprintf("framebuf_trans_out FAIL\n");
        return RT_ERROR;
    }

#if RT_USING_CONSOLE
    rt_device_t pDev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
#else
    rt_device_t pDev = rt_device_find("uart1");
#endif

    size = hor_res * ver_res * LV_COLOR_DEPTH / 8;
    cnt = (size + MAX_BLOCK_LEN - 1) / MAX_BLOCK_LEN;
    crc = getCrc(frame_buf, size, 0xffffffff);

    p_buf = (uint8_t *)rt_malloc(MAX_BLOCK_LEN);
    if (p_buf == RT_NULL)
    {
        rt_kprintf("p_buf malloc err: %d\n", MAX_BLOCK_LEN);
        rt_kprintf("framebuf_trans_out FAIL\n");
        //app_lv_task_mutex_release();
        return RT_ERROR;
    }

    rt_kprintf("framebuf_trans_len 0x%08x\n", size);

    if (pDev->open_flag & RT_DEVICE_FLAG_STREAM)
    {
        pDev->open_flag &= ~RT_DEVICE_FLAG_STREAM;
        flag = 1;
    }

    local_log_pause(1);

    off = 0;
    while (1)
    {
        delta = rt_device_read(pDev, off, &p_buf[off], cmdlen - off);
        off += delta;
        if (off >= cmdlen)
        {
            for (int m = 0; m < cmdlen / 2; m++)
            {
                if (memcmp(&p_buf[m], "framebuf_trans_out_waitrx", cmdlen - m) == 0)
                {
                    if (m != 0)
                    {
                        rt_device_read(pDev, off, &p_buf[off], m);
                    }
                    break;
                }
            }
            break;
        }
    }

    off = 0;
    for (int m = 0; m < cnt; m++)
    {
        len = MAX_BLOCK_LEN;

        if ((m == cnt - 1) && (size % MAX_BLOCK_LEN))
        {
            len = size % MAX_BLOCK_LEN;
        }

        rt_device_write(pDev, 0, &frame_buf[off], len);
        off += len;
    }

    local_log_pause(0);

    if (flag == 1)
    {
        pDev->open_flag |= RT_DEVICE_FLAG_STREAM;
    }

    rt_kprintf("framebuf_trans_crc 0x%08x\n", crc);
    rt_kprintf("framebuf_trans_out OK\n");

    FREE_ELM_BUF;
    //app_lv_task_mutex_release();

    return 0;
}
#endif/*LVGL_V8 */
#if WF_INSTAL_TEST
static int elm_install_app_auto(const char *name)
{
    app_fat_load_watchface_pack();
    apm_app_state_t state;
    apm_app_info_t app_info;

    if (0 == apm_query_app(watchface_db, name, &app_info, &state))
    {
        app_dynamic_watchface_insert(&app_info);
        rt_kprintf("elm_install_app_auto OK\n");
        return 0;
    }
    rt_kprintf("elm_install_app_auto FAIL\n");
    return 1;
}
#endif

static int elm_trans_test(int argc, char **argv)
{
    int res;
    struct stat file_stat;
    uint8_t temp_pri = RT_THREAD_PRIORITY_HIGH - 1;
    uint8_t cur_pri = rt_thread_self()->current_priority;

    rt_thread_control(rt_thread_self(), RT_THREAD_CTRL_CHANGE_PRIORITY, &temp_pri);

    if (argc < 2)
    {
        rt_kprintf("para num %d < 2\n", argc);
        rt_kprintf("eg1: elm_trans_test 0 (check if support new trans api)\n");
        rt_kprintf("eg1: elm_trans_test 1 /test.bin (get test.bin file size)\n");
        rt_kprintf("eg1: elm_trans_test 2 /test.bin 0x20000 (trans test.bin file out)\n");
        rt_kprintf("eg1: elm_trans_test 3 /test.bin 0x20000 0x12345678(trans test.bin file in)\n");
        rt_kprintf("elm_trans_test FAIL\n");
        //return RT_ERROR;
    }

    if (argv[1][0] == '0')
    {
        rt_kprintf("elm_trans_test NEW\n");
    }
    else if (argv[1][0] == '1')
    {
        if (argc != 3)
        {
            rt_kprintf("para num %d != 3\n", argc);
            rt_kprintf("elm_trans_info FAIL\n");
            //return RT_ERROR;
        }
        else
        {
            elm_trans_info(argv[2]);
        }
    }
    else if (argv[1][0] == '2')
    {
        if (argc != 4)
        {
            rt_kprintf("para num %d != 4\n", argc);
            rt_kprintf("elm_trans_out FAIL\n");
            //return RT_ERROR;
        }
        else
        {
            elm_trans_out(argv[2], argv[3]);
        }
    }
    else if (argv[1][0] == '3')
    {
        if (argc != 5)
        {
            rt_kprintf("para num %d != 5\n", argc);
            rt_kprintf("elm_trans_in FAIL\n");
            return RT_ERROR;
        }
        else
        {
            elm_trans_in(argv[2], argv[3], argv[4]);
        }
    }
    else if (argv[1][0] == '4')
    {
        if (argc != 2)
        {
            rt_kprintf("para num %d != 2\n", argc);
            rt_kprintf("framebuf_trans_info FAIL\n");
            return RT_ERROR;
        }
        else
        {
#ifdef LVGL_V8
            framebuf_trans_out();
#endif/*LVGL_V8*/
        }
    }
    else if (argv[1][0] == '5')
    {
        if (argc != 5)
        {
            rt_kprintf("para num %d != 5\n", argc);
            rt_kprintf("elm_check_size FAIL\n");
            return RT_ERROR;
        }
        else
        {
            elm_check_size(argv[2], argv[3], argv[4]);
        }
    }
    else if (argv[1][0] == '6')
    {
        if (argc != 3)
        {
            rt_kprintf("para num %d != 2\n", argc);
            rt_kprintf("elm_install_app_auto FAIL\n");
            return RT_ERROR;
        }
        else
        {
#if WF_INSTAL_TEST
            elm_install_app_auto(argv[2]);
#else
            rt_kprintf("elm_install_app_auto not support\n", argc);
            rt_kprintf("elm_install_app_auto FAIL\n");
#endif
        }
    }

    rt_thread_control(rt_thread_self(), RT_THREAD_CTRL_CHANGE_PRIORITY, &cur_pri);

    return 0;
}
MSH_CMD_EXPORT(elm_trans_test, elm_trans_test);

static int shell_speed(int argc, char **argv)
{
    if (argc != 2)
    {
        rt_kprintf("Fail\n");
        return -1;
    }

    uint32_t baund = strtoul(argv[1], 0, 10);
    //uint32_t timelen = strtoul(argv[2], 0, 10);

    //rt_device_t logDev;
    struct serial_configure *pCfg; //= RT_SERIAL_CONFIG_DEFAULT;
    struct rt_serial_device *pLogDev = (struct rt_serial_device *)rt_device_find(RT_CONSOLE_DEVICE_NAME);

    pCfg = &(pLogDev->config);
    pCfg->baud_rate = baund;
    //rt_device_control(logDev, RT_DEVICE_CTRL_CONFIG, pCfg);
    rt_device_control((rt_device_t)pLogDev, RT_DEVICE_CTRL_CONFIG, pCfg);

    //rt_thread_mdelay(timelen);
    rt_kprintf("OK\n");

    return 0;
}
MSH_CMD_EXPORT(shell_speed, shell_speed 3000000);
#endif

