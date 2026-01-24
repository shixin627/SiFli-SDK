/*
 * Copyright (c) 2018-2024, Skaiwalk Development Team
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2024-02-16     jack         first version
 */

#include "board.h"
#ifdef RT_USING_DFS
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"
#include "lvgl.h"
#include "app_mem.h"

#define DBG_TAG "utest.fs"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

int fs_read_test(char *path, char *name)
{
    int fd, size, r = 0;
    lv_img_header_t header;
    uint8_t *data;
    struct stat file_stat;
    static char fname[80];

    strcpy(fname, path);
    strcat(fname, "/");
    strcat(fname, name);

    // Read header only
    fd = open(fname, O_RDONLY);
    if (fd < 0)
    {
        LOG_E("Cannot open file: %s", fname);
        r = -1;
        return r;
    }
    size = read(fd, &header, sizeof(lv_img_header_t));
    if (size != sizeof(lv_img_header_t))
    {
        LOG_E("Get header error: %s, %d", name, size);
        r = -1;
        return r;
    }
    if (header.cf > LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED)
    {
        LOG_E("Format error???: %s, %d", name, header.cf);
    }
    close(fd);

    // Read content
    if (stat(fname, &file_stat))
    {
        LOG_E("Cannot Get file size: %s", fname);
        r = -1;
        return r;
    }

    fd = open(fname, O_RDONLY);
    if (fd < 0)
    {
        LOG_E("Cannot open file: %s", fname);
        r = -1;
        return r;
    }

    size = file_stat.st_size - sizeof(lv_img_header_t);
    data = app_cache_alloc(size, IMAGE_CACHE_PSRAM);
    if (data == NULL)
    {
        LOG_E("Cannot allocate: %s,%d", name, size);
        close(fd);
        r = -1;
        return r;
    }
    lseek(fd, sizeof(lv_img_header_t), SEEK_SET);
    if (size != read(fd, data, size))
    {
        LOG_E("Get content error: %s, %d", name, size);
        r = -1;
    }
    app_cache_free(data);
    LOG_D("Check %s done, size=%d", fname, size);
    close(fd);
    return r;
}

int fs_read(char *path)
{
    struct dfs_fd fd;
    struct dirent dirent;

    int length;

    if (dfs_file_open(&fd, path, O_DIRECTORY) != 0)
        return -1;

    memset(&dirent, 0, sizeof(struct dirent));

    length = dfs_file_getdents(&fd, &dirent, sizeof(struct dirent));
    while (length)
    {
        if (strstr(dirent.d_name, ".bin")) // EZip file
        {
            fs_read_test(path, dirent.d_name);
        }
        else if (strstr(dirent.d_name, ".") == NULL) // Assume directory do not have . in name
        {
            char name[80];
            strcpy(name, path);
            strcat(name, "/");
            strcat(name, dirent.d_name);
            if (fs_read(name) < 0)
                RT_ASSERT(0);
        }
        else
        {
            LOG_D("Found file %s in %s", dirent.d_name, path);
        }
        length = dfs_file_getdents(&fd, &dirent, sizeof(struct dirent));
    }
    dfs_file_close(&fd);
    return RT_EOK;
}

int fsread(int argc, char *argv[])
{
    int iter = 1;

    if (argc > 1)
        iter = atoi(argv[1]);

    while (iter-- > 0)
        fs_read("/");

    return 0;
}
MSH_CMD_EXPORT(fsread, Test file system read);

#endif /* RT_USING_DFS */