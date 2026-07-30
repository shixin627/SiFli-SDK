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
#include <stdarg.h>
#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>
#include <ulog.h>
#include <board.h>
#include "communicate_update_image.h"
#include "bloc_filesystem.h"   /* watch -> phone file push (0x52/0x53/0x54) */

extern void rt_assert_set_hook(void (*hook)(const char *ex, const char *func,
                                            rt_size_t line));
extern void rt_hw_exception_install(rt_err_t (*handler)(void *context));

#define LOG_DIR                 "/logs"
#define LOG_FILE_EXT            ".log"
/* A crash log renamed to this has already been handed to the phone; it is
 * kept (not deleted) so it can still be pulled manually, and is reclaimed by
 * the normal prune along with ordinary .log files. */
#define LOG_SENT_EXT            ".sent"

#define LOG_FILE_MAX_SIZE       (256 * 1024)   /* per file cap: 256 KB */
#define LOG_DIR_MAX_SIZE        (1024 * 1024)  /* total folder cap: 1 MB */

#define LOG_RING_BUF_SIZE       8192           /* MUST be power of 2 */
#define LOG_RING_MASK           (LOG_RING_BUF_SIZE - 1)
#define LOG_FLUSH_THRESHOLD     512

#if kReleaseMode
/* Release units run this backend 24/7 on a sealed device, so the flush
 * cadence is a durability/evidence trade-off rather than a dev convenience:
 *   - too fast: the NAND gets hammered for the product's whole service life,
 *     and frequent flash writes can stall XIP -> GUI starves -> WDT reboot,
 *     i.e. the logger becomes a crash cause of its own.
 *   - too slow: a WDT reset loses whatever is still in the ring buffer, and
 *     unlike assert/HardFault there is no hook to rescue it.
 * 500 ms x fsync-every-2 lands physical writes ~1 s apart while capping the
 * WDT-case loss at ~1 s. Idle cycles cost nothing either way — flush_once()
 * returns early when the drain came back empty. */
#define LOG_FLUSH_INTERVAL_MS   500
#define LOG_FSYNC_EVERY_N       2
#else
#define LOG_FLUSH_INTERVAL_MS   100
#define LOG_FSYNC_EVERY_N       1              /* fsync on every non-empty flush */
#endif

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
    /* Static, not on-stack: LOG_DIR_SCAN_MAX entries (~80 B each) is ~2.5 KB,
       which overflows the flush thread's LOG_FLUSH_STACK (2048 B) when a
       rotation triggers a prune. Pruning only runs on the flush thread (and
       rare shell rotate), so a single shared buffer is safe. */
    static struct entry_info entries[LOG_DIR_SCAN_MAX];
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
        /* .sent files count too — they are pushed crash logs kept for manual
         * retrieval, and would otherwise sit outside the folder cap forever. */
        if (ext == NULL || (strcmp(ext, LOG_FILE_EXT) != 0 &&
                            strcmp(ext, LOG_SENT_EXT) != 0))
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

/* Crash-log auto-push state, consumed by the flush thread below and defined
 * in full further down (next to the /logs scanning helpers). */
static char pending_crash_path[LOG_PATH_MAX];
static rt_tick_t crash_push_due_tick;
static void push_pending_crash_log(void);

/* Live crash-evidence push (no reboot needed). The FreeType double-free guard
 * (ft_sfree in app_mem.c) survives the fault instead of asserting, so the
 * boot-time crash scan never fires for it — the evidence would just sit on the
 * watch. ft_sfree calls log_file_report_crash_evidence() to raise this flag;
 * the flush thread then seals the current /logs file (which already holds the
 * [FT-DBLFREE] line) and pushes it to the phone. Latched to fire at most once
 * per boot so a burst of double-frees in one whole-clean doesn't push a storm. */
static volatile int crash_evidence_pending;
static int crash_evidence_handled;
static void push_ft_evidence_now(void);

/* Black-box recorder mode: the ulog backend IS registered so the continuous
 * log stream flows into the RAM ring (circular, overwrite-oldest), but the
 * flush thread does NOT drain it to NAND periodically — zero flash writes in
 * normal operation. The ring (the last LOG_RING_BUF_SIZE bytes of log = the
 * lead-up context) is written to /logs only at crash time by the assert /
 * HardFault hooks, then auto-pushed. This gives pre-crash context for
 * autonomous field crashes without the write load that starved the GUI into
 * the 2026-07-27 mailbox crash-loop. Set before set_enabled(1) in release.
 * Declared here (before flush_thread_entry / ring_write) so those earlier
 * users see it. */
static int blackbox_mode;

static void flush_thread_entry(void *param)
{
    (void)param;
    int fsync_ctr = 0;
    rt_tick_t interval = rt_tick_from_millisecond(LOG_FLUSH_INTERVAL_MS);

    while (1)
    {
        /* When the backend is disabled, block indefinitely with no periodic
         * wakeups. The producer naturally releases flush_sem once logs start
         * flowing again after re-enable. */
        if (!backend_ready)
        {
            rt_sem_take(flush_sem, RT_WAITING_FOREVER);
            continue;
        }

        rt_sem_take(flush_sem, interval);

        if (!backend_ready)
        {
            continue;
        }
        /* Black-box mode keeps the log in the RAM ring only — no periodic NAND
         * drain (that continuous write load starved the GUI into the 2026-07-27
         * mailbox crash-loop). The ring is dumped to /logs only at crash time
         * (log_file_assert_hook / _exception_hook -> emergency_flush). */
        if (!blackbox_mode)
        {
            flush_once(&fsync_ctr);
        }

        /* A crash log inherited from the previous boot waits here until BLE
         * has had time to reconnect — pushing it at boot just fails. Runs at
         * most once per boot: push_pending_crash_log() clears the path. */
        if (pending_crash_path[0] != '\0' &&
            rt_tick_get() >= crash_push_due_tick)
        {
            push_pending_crash_log();
        }

        /* A live FreeType double-free was caught this session. Seal + push the
         * current file now — but only once a boot-inherited push (if any) has
         * cleared, so we don't clobber pending_crash_path mid-transfer. */
        if (crash_evidence_pending && !crash_evidence_handled &&
            pending_crash_path[0] == '\0')
        {
            crash_evidence_handled = 1;
            crash_evidence_pending = 0;
            push_ft_evidence_now();
        }
    }
}

/* Copy `len` bytes into the ring buffer. Returns bytes used after the copy,
 * or 0 if the payload did not fit (counted into dropped_bytes instead).
 * Shared by the ulog backend callback and the crash-report writer. */
static rt_uint32_t ring_write(const char *log, size_t len)
{
    rt_base_t flags = rt_hw_interrupt_disable();

    rt_uint32_t h = ring_head;
    rt_uint32_t t = ring_tail;
    rt_uint32_t used = h - t;
    rt_uint32_t avail = LOG_RING_BUF_SIZE - used;

    if (len > avail)
    {
        if (!blackbox_mode)
        {
            dropped_bytes += (rt_uint32_t)len;
            rt_hw_interrupt_enable(flags);
            return 0;
        }
        /* Black-box recorder: never drop — keep the MOST RECENT bytes by
         * overwriting the oldest. A single chunk larger than the whole ring
         * keeps only its tail. The buffer therefore always holds the last
         * LOG_RING_BUF_SIZE bytes of log, ready to dump at crash time. */
        if (len > LOG_RING_BUF_SIZE)
        {
            log += (len - LOG_RING_BUF_SIZE);
            len = LOG_RING_BUF_SIZE;
        }
        ring_tail = t + (len - avail);   /* drop oldest to make exactly room */
        t = ring_tail;
        used = h - t;
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
    return used;
}

static void log_file_output(struct ulog_backend *backend, rt_uint32_t level,
                            const char *tag, rt_bool_t is_raw, const char *log,
                            size_t len)
{
    (void)backend;
    (void)level;
    (void)is_raw;

    if (!backend_ready || len == 0 || log == NULL)
    {
        return;
    }

    /* Drop chatty BLE stack event logs (e.g. "sibles KE_EVT2: ...") —
     * they fire on every BLE packet and would otherwise fill the file
     * with transport-layer noise during BLE-heavy operations. Console
     * output is unaffected. */
    if (tag != NULL && rt_strncmp(tag, "sibles", 6) == 0)
    {
        return;
    }

    rt_uint32_t used = ring_write(log, len);

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

/* ARM v8-M architectural exception stack frame. The SDK hands the exception
 * hook `&exception_info->stack_frame.exception_stack_frame`; the layout is
 * fixed by hardware, so mirror it here rather than reaching into cpuport.c's
 * private struct definitions. */
struct crash_exc_frame
{
    rt_uint32_t r0, r1, r2, r3, r12, lr, pc, psr;
};

#define CRASH_LINE_MAX 128

/* Write one formatted line straight into the ring buffer, bypassing ulog.
 * The SDK dumps the crash context with rt_kprintf, which reaches the console
 * only — on a sealed unit there is no console, so the registers would be lost
 * entirely. Called with interrupts already disabled by the fault path. */
static void crash_printf(const char *fmt, ...)
{
    /* static, not on-stack: the faulting thread's stack may be nearly
     * exhausted (stack overflow is itself a common crash cause). Only ever
     * reached from assert / exception context, which fires once. */
    static char line[CRASH_LINE_MAX];
    va_list args;

    va_start(args, fmt);
    rt_int32_t n = rt_vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    if (n <= 0)
    {
        return;
    }
    if (n > (rt_int32_t)sizeof(line) - 1)
    {
        n = (rt_int32_t)sizeof(line) - 1;   /* truncated */
    }
    ring_write(line, (size_t)n);
}

/* Tail shared by both crash kinds: who was running and how much heap was
 * left. Heap exhaustion shows up as an allocation-failure assert somewhere
 * unrelated, so the numbers matter more than they look. */
static void crash_report_tail(void)
{
    rt_thread_t self = rt_thread_self();
    rt_uint32_t total = 0, used = 0, max_used = 0;

    crash_printf("thread : %s\n", (self != RT_NULL) ? self->name : "(none)");
    rt_memory_info(&total, &used, &max_used);
    crash_printf("heap   : total=%u used=%u max=%u\n",
                 (unsigned)total, (unsigned)used, (unsigned)max_used);
    crash_printf("=== CRASH END ===\n");
}

static rt_err_t log_file_exception_hook(void *context)
{
    const struct crash_exc_frame *f = (const struct crash_exc_frame *)context;

    if (f != RT_NULL)
    {
        crash_printf("\n=== CRASH: HARDFAULT ===\n");
        crash_printf("pc=%08x lr=%08x psr=%08x\n", f->pc, f->lr, f->psr);
        crash_printf("r0=%08x r1=%08x r2=%08x r3=%08x r12=%08x\n",
                     f->r0, f->r1, f->r2, f->r3, f->r12);
        crash_printf("cfsr=%08x hfsr=%08x mmfar=%08x bfar=%08x\n",
                     (unsigned)SCB->CFSR, (unsigned)SCB->HFSR,
                     (unsigned)SCB->MMFAR, (unsigned)SCB->BFAR);
        crash_report_tail();
    }

    log_file_emergency_flush();

    /* MUST NOT return RT_EOK. The SDK's handle_exception() bails out the
     * moment the hook reports RT_EOK (cortex-m33/cpuport.c), skipping the
     * whole register / thread / fault-status dump that follows — which
     * silently blinds the UART console too, not just this backend. */
    return -RT_ERROR;
}

static void log_file_assert_hook(const char *ex, const char *func,
                                 rt_size_t line)
{
    crash_printf("\n=== CRASH: ASSERT ===\n");
    if (func != RT_NULL)
    {
        crash_printf("at     : %s:%d (%s)\n", func, (int)line,
                     (ex != RT_NULL) ? ex : "?");
    }
    else
    {
        /* ASSERT_OPTIMIZE_1 builds pass the return address in `line` and
         * no function name — resolve the address against the .axf/.map. */
        crash_printf("at     : addr 0x%08x\n", (unsigned)line);
    }
    crash_report_tail();

    log_file_emergency_flush();
}

/* ---- crash log auto-push -------------------------------------------------
 * A crash the user hits in the field is only worth recording if it comes
 * back to us — a sealed unit has no console and no developer app. On boot we
 * look for a previous session's log whose tail carries a crash marker and
 * hand it to the phone over the existing file-sync path (0x52/0x53/0x54).
 *
 * The file is renamed to .sent *before* the push, not after: sync_file()
 * only queues the transfer, so renaming afterwards would pull the file out
 * from under the transfer thread. The rename is also what stops the next
 * boot from pushing the same crash again. */
#define CRASH_MARKER            "=== CRASH:"
#define CRASH_TAIL_SCAN         512      /* marker is always at end of file */
#define CRASH_PUSH_DELAY_MS     60000    /* give BLE time to reconnect */

/* Read the tail of `path` and report whether an assert / HardFault marker is
 * in it. Only the last CRASH_TAIL_SCAN bytes are read — the device reset
 * immediately after the hooks wrote it, so it can only be at the very end. */
static int log_tail_has_crash_marker(const char *path)
{
    /* static: called from the flush thread, whose stack is only 2 KB. */
    static char tail[CRASH_TAIL_SCAN + 1];
    struct stat st;

    if (stat(path, &st) != 0 || st.st_size <= 0)
    {
        return 0;
    }

    int fd = open(path, O_RDONLY);
    if (fd < 0)
    {
        return 0;
    }

    off_t from = (st.st_size > CRASH_TAIL_SCAN)
                 ? (st.st_size - CRASH_TAIL_SCAN) : 0;
    lseek(fd, from, SEEK_SET);
    int n = read(fd, tail, CRASH_TAIL_SCAN);
    close(fd);

    if (n <= 0)
    {
        return 0;
    }
    tail[n] = '\0';
    return (strstr(tail, CRASH_MARKER) != NULL) ? 1 : 0;
}

/* Scan /logs for a sealed file that ended in a crash. `keep_path` is the file
 * this boot just opened and is never a candidate. */
static void find_crashed_log(const char *keep_path)
{
    DIR *dir = opendir(LOG_DIR);
    if (dir == NULL)
    {
        return;
    }

    struct dirent *de;
    while ((de = readdir(dir)) != NULL)
    {
        const char *ext = strrchr(de->d_name, '.');
        if (ext == NULL || strcmp(ext, LOG_FILE_EXT) != 0)
        {
            continue;   /* already-pushed files carry LOG_SENT_EXT */
        }

        char full[LOG_PATH_MAX];
        rt_snprintf(full, sizeof(full), "%s/%s", LOG_DIR, de->d_name);
        if (keep_path != NULL && strcmp(full, keep_path) == 0)
        {
            continue;
        }
        if (!log_tail_has_crash_marker(full))
        {
            continue;
        }

        rt_strncpy(pending_crash_path, full, sizeof(pending_crash_path) - 1);
        pending_crash_path[sizeof(pending_crash_path) - 1] = '\0';
        rt_kprintf("[log_file_be] crash log found: %s\n", pending_crash_path);
        break;
    }
    closedir(dir);
}

static void push_pending_crash_log(void)
{
    char sent[LOG_PATH_MAX];
    char *ext = strrchr(pending_crash_path, '.');

    if (ext != NULL)
    {
        int stem = (int)(ext - pending_crash_path);
        rt_snprintf(sent, sizeof(sent), "%.*s" LOG_SENT_EXT, stem,
                    pending_crash_path);
    }
    else
    {
        rt_snprintf(sent, sizeof(sent), "%s" LOG_SENT_EXT, pending_crash_path);
    }

    if (rename(pending_crash_path, sent) != 0)
    {
        /* Can't mark it — skip rather than risk pushing it on every boot. */
        rt_kprintf("[log_file_be] rename %s failed, skip push\n",
                   pending_crash_path);
        pending_crash_path[0] = '\0';
        return;
    }

    /* delete_after_sync = false on purpose: bloc_filesystem deletes the file
     * whether or not the transfer succeeded, which would throw the only copy
     * of the evidence away whenever BLE happened to be down. The .sent file
     * stays for manual retrieval and is reclaimed by the normal prune. */
    if (bloc_file_system.sync_file != NULL)
    {
        rt_kprintf("[log_file_be] pushing crash log %s\n", sent);
        bloc_file_system.sync_file(sent, false);
    }
    pending_crash_path[0] = '\0';
}

/* Seal the CURRENT log file (it already contains the [FT-DBLFREE] line the FT
 * guard printed) and push it to the phone, then resume logging into a fresh
 * file. Runs on the flush thread. Order matters: close + hand the sealed path
 * to push_pending_crash_log (which renames it to .sent and syncs) BEFORE
 * opening the new file, so the subsequent prune can't target it (the .sent
 * copy is the newest by mtime and is skipped anyway). */
static void push_ft_evidence_now(void)
{
    if (current_fd < 0)
    {
        return;
    }
    drain_ring_to_fd(0);
    fsync(current_fd);
    close(current_fd);
    current_fd = -1;

    rt_strncpy(pending_crash_path, current_file_path,
               sizeof(pending_crash_path) - 1);
    pending_crash_path[sizeof(pending_crash_path) - 1] = '\0';

    push_pending_crash_log();   /* rename -> .sent + sync now */
    open_new_log_file();        /* keep logging after the evidence is sealed */
}

/* Raise the live-evidence flag and wake the flush thread. Safe from any
 * context: it only sets a flag and releases a semaphore (no file / BLE I/O
 * here — that all happens on the flush thread). Called by ft_sfree's
 * double-free guard via a weak symbol, so builds without this backend get a
 * no-op instead of a link error. */
void log_file_report_crash_evidence(void)
{
    crash_evidence_pending = 1;
    if (flush_sem != RT_NULL)
    {
        rt_sem_release(flush_sem);
    }
}

/* Called from the HCPU WDT1 timeout handler (main.c: wdt_store_exception_
 * information) just before drv_reboot(), when a thread has monopolised the CPU
 * past the watchdog deadline. This is the SILENT reboot class: no RT_ASSERT and
 * no HardFault fires, so neither crash hook runs and — until now — nothing was
 * captured (the handler's own rt_kprintf reaches the UART console only, which a
 * sealed field unit cannot return). `detail` carries the culprit the handler
 * already identified (spinning thread, stack high-water / overflow flag, stacked
 * PC/LR). We write it into the current /logs file behind the same "=== CRASH:"
 * marker the assert/HardFault hooks use, and emergency_flush drains the RAM ring
 * with it — so the file also holds the pre-hang log lead-up. The boot-time
 * find_crashed_log + push path then delivers it to the phone, no new plumbing.
 *
 * Runs in WDT-IRQ context: crash_printf uses a static line buffer (no heap) and
 * emergency_flush is reentry-guarded and no-ops when no file is open (dev builds
 * with the logger off), so it is safe to call unconditionally. Only stage-1 WDT
 * timeouts that let this IRQ run are caught; a hang hard enough to force the
 * stage-2 reset before the IRQ dispatches still escapes (would need RAM-ring
 * survival across the reset — deferred until evidence shows it is needed). */
void log_file_report_wdt(const char *detail)
{
    crash_printf("\n=== CRASH: WATCHDOG ===\n");
    if (detail != RT_NULL)
    {
        crash_printf("%s", detail);
    }
    crash_report_tail();
    log_file_emergency_flush();
}

static volatile int infra_ready;   /* dir + sem + flush thread + hooks ready */

int log_file_backend_set_enabled(int enable);   /* defined below */

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

    flush_sem = rt_sem_create("logfe", 0, RT_IPC_FLAG_FIFO);
    if (flush_sem == RT_NULL)
    {
        return -RT_ERROR;
    }

    flush_tid = rt_thread_create("logfe", flush_thread_entry, RT_NULL,
                                 LOG_FLUSH_STACK, LOG_FLUSH_PRIO,
                                 LOG_FLUSH_TIMESLICE);
    if (flush_tid == RT_NULL)
    {
        rt_sem_delete(flush_sem);
        flush_sem = RT_NULL;
        return -RT_ERROR;
    }
    rt_thread_startup(flush_tid);

    ulog_init();
    log_file_be.output = log_file_output;
    log_file_be.flush = log_file_flush_cb;

    /* Flush pending logs to disk on RT_ASSERT / HardFault so the last
     * messages before reset survive in the current log file. */
    rt_assert_set_hook(log_file_assert_hook);
    rt_hw_exception_install(log_file_exception_hook);

    infra_ready = 1;

#if kReleaseMode
    /* ── Minimal blackbox (2026-07-28) ──────────────────────────────────
     * The 2026-07-27 field build (full logger auto-on at DBG_INFO) held a
     * send_msg_to_gui_app_task mailbox-full assert crash-loop; the continuous
     * /logs NAND writes starved the GUI (a known failure mode on this HW).
     * The 2026-07-28 minimal build (no ulog capture) then ran crash-free but
     * caught autonomous HardFaults whose CAUSE it couldn't show — only the
     * corrupted register frame, no lead-up. Black-box mode fixes both: capture
     * the log stream into the RAM ring (zero NAND in normal operation) and
     * dump the last LOG_RING_BUF_SIZE bytes only at crash time -> pre-crash
     * context for autonomous field crashes, no GUI-starving write load. */
    blackbox_mode = 1;
    log_file_backend_set_enabled(1);
#else
    /* File log is DISABLED by default; user turns it on via the developer
     * app switch. Until then, no file is opened and the backend is not
     * registered with ulog — zero overhead. */
#endif
    return RT_EOK;
}
/* Runs after INIT_ENV_EXPORT where the root fs is mounted */
INIT_APP_EXPORT(log_file_backend_init);

int log_file_backend_is_enabled(void)
{
    return backend_ready ? 1 : 0;
}

int log_file_backend_set_enabled(int enable)
{
    if (!infra_ready)
    {
        return -RT_ERROR;
    }
    if (enable)
    {
        if (backend_ready)
        {
            return RT_EOK;
        }
        if (current_fd < 0)
        {
            if (open_new_log_file() != RT_EOK)
            {
                return -RT_ERROR;
            }
        }
        backend_ready = 1;
        /* Register the ulog backend in BOTH modes so the log stream is
         * captured. In black-box mode it flows into the RAM ring only (the
         * flush thread never drains it to NAND); in full mode the flush thread
         * also persists it periodically. */
        ulog_backend_register(&log_file_be, "filebe", RT_FALSE);
        rt_kprintf("[log_file_be] enabled (%s)\n",
                   blackbox_mode ? "blackbox/RAM-ring" : "full");

        /* First enable of this boot: look for a log the previous session
         * left behind with a crash marker in its tail. */
        static int crash_scan_done;
        if (!crash_scan_done)
        {
            crash_scan_done = 1;
            find_crashed_log(current_file_path);
            crash_push_due_tick = rt_tick_get() +
                                  rt_tick_from_millisecond(CRASH_PUSH_DELAY_MS);
        }
    }
    else
    {
        if (!backend_ready)
        {
            return RT_EOK;
        }
        /* Unregister first so ulog stops routing logs to us, then drain
         * whatever is already in the ring buffer and seal the file. */
        ulog_backend_unregister(&log_file_be);
        backend_ready = 0;
        if (current_fd >= 0)
        {
            drain_ring_to_fd(0);
            fsync(current_fd);
            close(current_fd);
            current_fd = -1;
        }
        rt_kprintf("[log_file_be] disabled\n");
    }
    return RT_EOK;
}

#if !kReleaseMode
/* Shell: force the ring buffer to drain and fsync so the current file on
 * disk is up to date (useful before copying it off via file browser). */
static void logfe_sync(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!backend_ready)
    {
        rt_kprintf("[log_file_be] disabled\n");
        return;
    }
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
    if (!backend_ready)
    {
        rt_kprintf("[log_file_be] disabled\n");
        return;
    }
    if (flush_sem != RT_NULL)
    {
        rt_sem_release(flush_sem);
        rt_thread_mdelay(100);
    }
    open_new_log_file();
}
MSH_CMD_EXPORT(logfe_rotate, seal current log file and open a new one);

/* Shell: toggle file logging on/off at runtime. Mirrors the developer
 * app switch. Usage: logfe_enable [0|1] */
static void logfe_enable(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("[log_file_be] %s\n",
                   log_file_backend_is_enabled() ? "enabled" : "disabled");
        return;
    }
    log_file_backend_set_enabled(atoi(argv[1]));
}
MSH_CMD_EXPORT(logfe_enable, enable or disable file log backend);

#endif /* !kReleaseMode — MSH dev commands only; the backend itself ships */
