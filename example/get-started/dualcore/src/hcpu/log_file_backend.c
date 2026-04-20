/**
 ******************************************************************************
 * @file   log_file_backend.c
 * @author Skaiwalk software development team
 * @brief  ulog backend that persists logs to the filesystem under /logs.
 *
 * Behavior:
 *   - Every boot opens a NEW file named by RTC time (log_YYYYMMDD_HHMMSS.log).
 *     Previous files are sealed and never written again. If the device
 *     crashes, the pre-crash file keeps the last messages (periodic fsync
 *     caps the loss at a few seconds of logs).
 *   - When the current file reaches LOG_FILE_MAX_SIZE (256 KB) a new
 *     timestamped file is opened and the old one is sealed mid-boot.
 *   - After every rotation the /logs directory is pruned down to at most
 *     LOG_DIR_MAX_SIZE (1 MB) by deleting the oldest files (by mtime).
 *
 * Thread model (single-producer / single-consumer ring buffer):
 *   - Producer: ulog's output callback (runs under ulog's output lock, may
 *     be called from thread or ISR context when ULOG_USING_ISR_LOG is on).
 *     Only copies bytes into the ring buffer.
 *   - Consumer: a dedicated low-priority thread that drains the ring buffer
 *     into the current file, rotates when full, and fsyncs periodically.
 ******************************************************************************
 */
#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <dfs_file.h>
#include <dfs_posix.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>
#include <ulog.h>

extern void rt_assert_set_hook(void (*hook)(const char *ex, const char *func,
                                            rt_size_t line));
extern void rt_hw_exception_install(rt_err_t (*handler)(void *context));

#ifndef LOG_FILE_BACKEND_DISABLE

#define LOG_DIR                 "/logs"
#define LOG_FILE_EXT            ".log"

#define LOG_FILE_MAX_SIZE       (256 * 1024)   /* per file cap: 256 KB */
#define LOG_DIR_MAX_SIZE        (1024 * 1024)  /* total folder cap: 1 MB */

#define LOG_RING_BUF_SIZE       8192           /* MUST be power of 2 */
#define LOG_RING_MASK           (LOG_RING_BUF_SIZE - 1)
#define LOG_FLUSH_THRESHOLD     512
#define LOG_FLUSH_INTERVAL_MS   100
#define LOG_FSYNC_EVERY_N       1              /* fsync on every non-empty flush */

#define LOG_FLUSH_STACK         2048
#define LOG_FLUSH_PRIO          28
#define LOG_FLUSH_TIMESLICE     10

#define LOG_PATH_MAX            96
#define LOG_DIR_SCAN_MAX        32             /* max files considered in prune */

static struct ulog_backend log_file_be;
static char ring_buf[LOG_RING_BUF_SIZE];
static volatile rt_uint32_t ring_head;     /* written by producer */
static volatile rt_uint32_t ring_tail;     /* written by flush thread */
static volatile rt_uint32_t dropped_bytes;

static rt_sem_t flush_sem;
static rt_thread_t flush_tid;
static int current_fd = -1;
static rt_uint32_t current_file_size;
static char current_file_path[LOG_PATH_MAX];
static volatile int backend_ready;
static volatile int in_emergency_flush;

static void build_log_filename(char *out, size_t out_sz, int seq)
{
    time_t now = 0;
    time(&now);
    struct tm *tmi = localtime(&now);

    int year = (tmi != NULL) ? tmi->tm_year + 1900 : 0;

    if (tmi != NULL && year >= 2020)
    {
        if (seq == 0)
        {
            rt_snprintf(out, out_sz,
                        "%s/log_%04d%02d%02d_%02d%02d%02d" LOG_FILE_EXT,
                        LOG_DIR, year, tmi->tm_mon + 1, tmi->tm_mday,
                        tmi->tm_hour, tmi->tm_min, tmi->tm_sec);
        }
        else
        {
            rt_snprintf(out, out_sz,
                        "%s/log_%04d%02d%02d_%02d%02d%02d_%02d" LOG_FILE_EXT,
                        LOG_DIR, year, tmi->tm_mon + 1, tmi->tm_mday,
                        tmi->tm_hour, tmi->tm_min, tmi->tm_sec, seq);
        }
    }
    else
    {
        /* RTC not set yet: fall back to monotonic tick so names stay unique */
        rt_snprintf(out, out_sz, "%s/log_boot_%010u_%02d" LOG_FILE_EXT,
                    LOG_DIR, (unsigned)rt_tick_get(), seq);
    }
}

/* Scan /logs, delete oldest .log files (by mtime) until total bytes
 * are <= LOG_DIR_MAX_SIZE. Never deletes the file at `keep_path`. */
static void prune_log_dir(const char *keep_path)
{
    struct entry_info
    {
        char name[64];
        time_t mtime;
        off_t size;
    };
    struct entry_info entries[LOG_DIR_SCAN_MAX];
    int count = 0;

    DIR *dir = opendir(LOG_DIR);
    if (dir == NULL)
    {
        return;
    }

    struct dirent *de;
    while ((de = readdir(dir)) != NULL && count < LOG_DIR_SCAN_MAX)
    {
        const char *ext = strrchr(de->d_name, '.');
        if (ext == NULL || strcmp(ext, LOG_FILE_EXT) != 0)
        {
            continue;
        }
        char full[LOG_PATH_MAX];
        rt_snprintf(full, sizeof(full), "%s/%s", LOG_DIR, de->d_name);
        struct stat st;
        if (stat(full, &st) != 0)
        {
            continue;
        }
        rt_strncpy(entries[count].name, de->d_name,
                   sizeof(entries[count].name) - 1);
        entries[count].name[sizeof(entries[count].name) - 1] = '\0';
        entries[count].mtime = st.st_mtime;
        entries[count].size = st.st_size;
        count++;
    }
    closedir(dir);

    off_t total = 0;
    for (int i = 0; i < count; i++)
    {
        total += entries[i].size;
    }

    while (total > (off_t)LOG_DIR_MAX_SIZE && count > 0)
    {
        /* find oldest by mtime */
        int oldest = 0;
        for (int i = 1; i < count; i++)
        {
            if (entries[i].mtime < entries[oldest].mtime)
            {
                oldest = i;
            }
        }

        char path[LOG_PATH_MAX];
        rt_snprintf(path, sizeof(path), "%s/%s", LOG_DIR, entries[oldest].name);

        if (keep_path != NULL && strcmp(path, keep_path) == 0)
        {
            /* Cannot delete the currently-open file. Remove from list and
             * continue pruning others. */
            entries[oldest] = entries[count - 1];
            count--;
            continue;
        }

        if (unlink(path) == 0)
        {
            total -= entries[oldest].size;
            rt_kprintf("[log_file_be] pruned %s (%u bytes)\n", path,
                       (unsigned)entries[oldest].size);
        }
        entries[oldest] = entries[count - 1];
        count--;
    }
}

/* Close current fd, open a new timestamped file, prune folder. */
static int open_new_log_file(void)
{
    if (current_fd >= 0)
    {
        fsync(current_fd);
        close(current_fd);
        current_fd = -1;
    }

    char path[LOG_PATH_MAX];
    int fd = -1;
    for (int seq = 0; seq < 100; seq++)
    {
        build_log_filename(path, sizeof(path), seq);
        fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0666);
        if (fd >= 0)
        {
            break;
        }
    }
    if (fd < 0)
    {
        rt_kprintf("[log_file_be] open new log file failed\n");
        return -RT_ERROR;
    }

    rt_strncpy(current_file_path, path, sizeof(current_file_path) - 1);
    current_file_path[sizeof(current_file_path) - 1] = '\0';
    current_fd = fd;
    current_file_size = 0;

    prune_log_dir(current_file_path);

    rt_kprintf("[log_file_be] opened %s\n", current_file_path);
    return RT_EOK;
}

/* Drain the ring buffer into current_fd. Returns number of bytes written.
 * Callable from the flush thread (normal case) or an assert / exception
 * context (emergency case) — in the latter interrupts are already disabled
 * by the caller. The rotation path is skipped in emergency so we don't
 * touch the file system more than necessary right before reset. */
static rt_uint32_t drain_ring_to_fd(int allow_rotate)
{
    rt_uint32_t total_written = 0;
    if (current_fd < 0)
    {
        return 0;
    }

    for (;;)
    {
        rt_base_t flags = rt_hw_interrupt_disable();
        rt_uint32_t h = ring_head;
        rt_uint32_t t = ring_tail;
        rt_hw_interrupt_enable(flags);

        rt_uint32_t used = h - t;
        if (used == 0)
        {
            break;
        }

        rt_uint32_t t_idx = t & LOG_RING_MASK;
        rt_uint32_t contig = LOG_RING_BUF_SIZE - t_idx;
        rt_uint32_t chunk = used < contig ? used : contig;

        int w = write(current_fd, &ring_buf[t_idx], chunk);
        if (w <= 0)
        {
            ring_tail = t + chunk;
            break;
        }
        ring_tail = t + (rt_uint32_t)w;
        current_file_size += (rt_uint32_t)w;
        total_written += (rt_uint32_t)w;

        if (allow_rotate && current_file_size >= LOG_FILE_MAX_SIZE)
        {
            open_new_log_file();
            if (current_fd < 0)
            {
                break;
            }
        }
    }

    rt_uint32_t dropped = dropped_bytes;
    if (dropped > 0 && current_fd >= 0)
    {
        char msg[64];
        int n = rt_snprintf(msg, sizeof(msg),
                            "[log_file_be] dropped %u bytes\n",
                            (unsigned)dropped);
        if (n > 0)
        {
            int w = write(current_fd, msg, n);
            if (w > 0)
            {
                current_file_size += (rt_uint32_t)w;
                total_written += (rt_uint32_t)w;
            }
        }
        rt_base_t flags = rt_hw_interrupt_disable();
        dropped_bytes -= dropped;
        rt_hw_interrupt_enable(flags);
    }

    return total_written;
}

static void flush_once(int *fsync_ctr)
{
    rt_uint32_t wrote = drain_ring_to_fd(1 /* allow rotate */);
    if (wrote == 0)
    {
        /* Nothing to persist this cycle — skip fsync to avoid needless
         * NAND churn when the system is idle. */
        return;
    }
    if (++(*fsync_ctr) >= LOG_FSYNC_EVERY_N)
    {
        *fsync_ctr = 0;
        if (current_fd >= 0)
        {
            fsync(current_fd);
        }
    }
}

/* Best-effort synchronous flush for use from rt_assert_handler / HardFault /
 * shell. Interrupts may already be disabled by the caller; DFS / NAND
 * drivers that rely on IRQ may hang here, but that is no worse than not
 * attempting — WDT will reset the device and the most recent periodic
 * fsync (at most LOG_FLUSH_INTERVAL_MS old) is already on disk. */
static void log_file_emergency_flush(void)
{
    if (!backend_ready || current_fd < 0)
    {
        return;
    }
    if (in_emergency_flush)
    {
        return;
    }
    in_emergency_flush = 1;
    drain_ring_to_fd(0 /* no rotation during emergency */);
    fsync(current_fd);
    in_emergency_flush = 0;
}

static void flush_thread_entry(void *param)
{
    (void)param;
    int fsync_ctr = 0;
    rt_tick_t interval = rt_tick_from_millisecond(LOG_FLUSH_INTERVAL_MS);

    while (1)
    {
        rt_sem_take(flush_sem, interval);
        flush_once(&fsync_ctr);
    }
}

static void log_file_output(struct ulog_backend *backend, rt_uint32_t level,
                            const char *tag, rt_bool_t is_raw, const char *log,
                            size_t len)
{
    (void)backend;
    (void)level;
    (void)tag;
    (void)is_raw;

    if (!backend_ready || len == 0 || log == NULL)
    {
        return;
    }

    rt_base_t flags = rt_hw_interrupt_disable();

    rt_uint32_t h = ring_head;
    rt_uint32_t t = ring_tail;
    rt_uint32_t used = h - t;
    rt_uint32_t avail = LOG_RING_BUF_SIZE - used;

    if (len > avail)
    {
        dropped_bytes += (rt_uint32_t)len;
        rt_hw_interrupt_enable(flags);
        return;
    }

    rt_uint32_t h_idx = h & LOG_RING_MASK;
    rt_uint32_t first = LOG_RING_BUF_SIZE - h_idx;
    if (len <= first)
    {
        rt_memcpy(&ring_buf[h_idx], log, len);
    }
    else
    {
        rt_memcpy(&ring_buf[h_idx], log, first);
        rt_memcpy(&ring_buf[0], log + first, len - first);
    }
    ring_head = h + (rt_uint32_t)len;
    used += (rt_uint32_t)len;

    rt_hw_interrupt_enable(flags);

    if (used >= LOG_FLUSH_THRESHOLD && flush_sem != RT_NULL)
    {
        rt_sem_release(flush_sem);
    }
}

static void log_file_flush_cb(struct ulog_backend *backend)
{
    (void)backend;
    /* ulog_flush() is called from rt_assert_handler with interrupts disabled
     * and the scheduler frozen, so the periodic flush thread cannot run.
     * Do a best-effort synchronous drain here so asserts land on disk. */
    log_file_emergency_flush();
    if (flush_sem != RT_NULL)
    {
        rt_sem_release(flush_sem);
    }
}

static rt_err_t log_file_exception_hook(void *context)
{
    (void)context;
    log_file_emergency_flush();
    return RT_EOK;
}

static void log_file_assert_hook(const char *ex, const char *func,
                                 rt_size_t line)
{
    (void)ex;
    (void)func;
    (void)line;
    log_file_emergency_flush();
}

static int log_file_backend_init(void)
{
    struct stat st;
    if (stat(LOG_DIR, &st) != 0)
    {
        if (mkdir(LOG_DIR, 0777) != 0)
        {
            rt_kprintf("[log_file_be] mkdir %s failed\n", LOG_DIR);
            return -RT_ERROR;
        }
    }

    if (open_new_log_file() != RT_EOK)
    {
        return -RT_ERROR;
    }

    flush_sem = rt_sem_create("logfe", 0, RT_IPC_FLAG_FIFO);
    if (flush_sem == RT_NULL)
    {
        close(current_fd);
        current_fd = -1;
        return -RT_ERROR;
    }

    flush_tid = rt_thread_create("logfe", flush_thread_entry, RT_NULL,
                                 LOG_FLUSH_STACK, LOG_FLUSH_PRIO,
                                 LOG_FLUSH_TIMESLICE);
    if (flush_tid == RT_NULL)
    {
        rt_sem_delete(flush_sem);
        flush_sem = RT_NULL;
        close(current_fd);
        current_fd = -1;
        return -RT_ERROR;
    }
    rt_thread_startup(flush_tid);

    ulog_init();
    log_file_be.output = log_file_output;
    log_file_be.flush = log_file_flush_cb;

    backend_ready = 1;
    ulog_backend_register(&log_file_be, "filebe", RT_FALSE);

    /* Flush pending logs to disk on RT_ASSERT / HardFault so the last
     * messages before reset survive in the current log file. */
    rt_assert_set_hook(log_file_assert_hook);
    rt_hw_exception_install(log_file_exception_hook);

    return RT_EOK;
}
/* Level "6" (INIT_PRE_ENV_EXPORT region isn't defined; use INIT_APP_EXPORT
 * which runs after INIT_ENV_EXPORT where the root fs is mounted) */
INIT_APP_EXPORT(log_file_backend_init);

/* Shell: force the ring buffer to drain and fsync so the current file on
 * disk is up to date (useful before copying it off via file browser). */
static void logfe_sync(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (flush_sem != RT_NULL)
    {
        rt_sem_release(flush_sem);
        rt_thread_mdelay(100);
    }
    if (current_fd >= 0)
    {
        fsync(current_fd);
        rt_kprintf("[log_file_be] synced %s (size=%u)\n", current_file_path,
                   (unsigned)current_file_size);
    }
}
MSH_CMD_EXPORT(logfe_sync, flush log file backend to disk);

/* Shell: force an immediate rotation to a new file (seals the current one). */
static void logfe_rotate(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (flush_sem != RT_NULL)
    {
        rt_sem_release(flush_sem);
        rt_thread_mdelay(100);
    }
    open_new_log_file();
}
MSH_CMD_EXPORT(logfe_rotate, seal current log file and open a new one);

#endif /* !LOG_FILE_BACKEND_DISABLE */
