/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>

#ifdef RT_USING_DFS
#include <dfs_fs.h>

#ifdef _WIN32
/* Two distinct host-side directories that production code expects:
 *
 *   ./disk       — backing folder for dfs_win32 (WIN32_DIRDISK_ROOT, see
 *                  rtos/rtthread/bsp/sifli/drivers_pc/dfs_win32.c). Any DFS
 *                  POSIX call that hits "/" on the watch translates to
 *                  ./disk/... on Windows. Created so DFS file ops have a
 *                  real backing tree.
 *
 *   ./prefdb     — FlashDB share_prefs LIBC-mode storage. The dfs_win32_ops
 *                  vtable has no mkdir op, so share_prefs_open()'s
 *                  mkdir("prefdb",0) goes through Windows libc directly
 *                  (CWD-relative), NOT through DFS. We pre-create here so
 *                  the access()-then-mkdir() check at the top of
 *                  share_prefs_open() short-circuits cleanly. Without
 *                  this, fdb_kvdb_init returns FDB_INIT_FAILED and every
 *                  prefs read on PC silently returns NULL.
 *
 * Both _mkdir calls are idempotent (EEXIST is fine).
 *
 * Plus: dualcore project ships /assets/emoji and /assets/fonts on the
 * watch's NAND root; the source-of-truth on disk is
 *   <repo>/example/get-started/dualcore/project/jsroot/assets/{emoji,fonts}
 * To make the PC sim see the same paths, mnt_init also creates junctions
 *   ./disk/assets/emoji -> <jsroot>/assets/emoji
 *   ./disk/assets/fonts -> <jsroot>/assets/fonts
 * Junctions (not symlinks) so no admin / dev-mode required.
 */
#include <direct.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>

static void ensure_host_dir(const char *path)
{
    if (_mkdir(path) == 0)
    {
        rt_kprintf("PC sim: created host dir %s\n", path);
    }
    else if (errno != EEXIST)
    {
        rt_kprintf("PC sim: WARN _mkdir(%s) failed errno=%d\n", path, errno);
    }
}

/* Create a Windows directory junction at `link_path` pointing to `target_abs`.
 * Junctions don't require admin or Developer Mode (unlike symlinks).
 * Idempotent: if `link_path` already exists (as junction OR regular dir OR
 * file) we leave it alone — let the user clean up if they want to re-point. */
static void ensure_junction(const char *link_path, const char *target_abs)
{
    DWORD attr = GetFileAttributesA(link_path);
    if (attr != INVALID_FILE_ATTRIBUTES)
    {
        /* Already exists (junction, dir, or file). Don't touch. */
        return;
    }
    if (GetFileAttributesA(target_abs) == INVALID_FILE_ATTRIBUTES)
    {
        rt_kprintf("PC sim: WARN junction target missing: %s\n", target_abs);
        return;
    }
    /* `mklink /J` makes a directory junction. Quote both args -- paths
     * may contain spaces. Swallow cmd's own output so we don't spam the
     * console; rely on rt_kprintf below for status. */
    char cmd[1024];
    snprintf(cmd, sizeof(cmd),
             "cmd /c mklink /J \"%s\" \"%s\" >nul 2>&1",
             link_path, target_abs);
    int rc = system(cmd);
    if (rc == 0)
        rt_kprintf("PC sim: junction %s -> %s\n", link_path, target_abs);
    else
        rt_kprintf("PC sim: WARN mklink rc=%d for %s -> %s\n",
                   rc, link_path, target_abs);
}

/* main.exe lives at <repo>/example/get-started/dualcore/project/hcpu/build_pc_hcpu/main.exe
 * jsroot lives at <repo>/example/get-started/dualcore/project/jsroot/
 * Walk up two directory levels from the exe path to reach .../project, then
 * descend into jsroot/assets/. Returns 0 on success, nonzero on failure
 * (e.g. exe path malformed). Writes the absolute path of <project>/jsroot
 * into `out`. */
static int resolve_jsroot_dir(char *out, size_t out_size)
{
    char exe_path[MAX_PATH];
    DWORD n = GetModuleFileNameA(NULL, exe_path, (DWORD)sizeof(exe_path));
    if (n == 0 || n >= sizeof(exe_path)) return -1;

    /* strip exe filename, then strip "build_pc_hcpu", then strip "hcpu" */
    for (int strip = 0; strip < 3; strip++)
    {
        char *slash = strrchr(exe_path, '\\');
        if (!slash) return -1;
        *slash = '\0';
    }
    /* exe_path is now <repo>/example/get-started/dualcore/project */
    int written = snprintf(out, out_size, "%s\\jsroot", exe_path);
    if (written < 0 || (size_t)written >= out_size) return -1;
    return 0;
}
#endif /* _WIN32 */

int mnt_init(void)
{
    dfs_init();

    extern int dfs_win32_init(void);
    extern rt_err_t rt_win_sharedir_init(const char *name);

#ifdef _WIN32
    ensure_host_dir("./disk");           /* dfs_win32 backing */
    ensure_host_dir("./disk/assets");    /* parent for the junctions below */
    ensure_host_dir("./prefdb");         /* FlashDB share_prefs (libc-mode) */

    /* Mirror the watch's /assets/{emoji,fonts} from dualcore's jsroot. */
    char jsroot[MAX_PATH];
    if (resolve_jsroot_dir(jsroot, sizeof(jsroot)) == 0)
    {
        char target[MAX_PATH];
        snprintf(target, sizeof(target), "%s\\assets\\emoji", jsroot);
        ensure_junction(".\\disk\\assets\\emoji", target);
        snprintf(target, sizeof(target), "%s\\assets\\fonts", jsroot);
        ensure_junction(".\\disk\\assets\\fonts", target);
    }
    else
    {
        rt_kprintf("PC sim: WARN couldn't resolve jsroot dir from exe path\n");
    }
#endif

    dfs_win32_init();
    rt_win_sharedir_init("wshare");

    if (dfs_mount("wshare", "/", "wdir", 0, 0) == 0)
    {
        rt_kprintf("File System on root initialized!\n");
    }
    else
    {
        rt_kprintf("File System on root initialization failed!\n");
    }

    return 0;
}

#endif
