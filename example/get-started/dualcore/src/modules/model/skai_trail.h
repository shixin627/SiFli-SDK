/**
 ******************************************************************************
 * @file   skai_trail.h
 * @brief  Pre-reboot event trail.
 *
 * The crash black box (hcpu/log_file_backend.c) only keeps the last 8 KB of
 * WARN/ERROR log text, and a chatty transfer (DFU / file sync) can evict
 * everything else. It also cannot say what the user or the system was DOING
 * right before a silent reboot, because ordinary actions (page switch, gesture,
 * screen wake) are INFO-level and never reach it.
 *
 * skai_trail() fills a second, tiny ring (128 x 16 bytes) that lives in the same
 * retained PSRAM region, so it survives a warm reset. On a silent reboot the
 * ring is written, oldest first, at the end of the dumped crash file.
 *
 * Cost: one IRQ-off 16-byte copy. Safe from any thread or ISR. HCPU only.
 ******************************************************************************
 */
#ifndef SKAI_TRAIL_H
#define SKAI_TRAIL_H

#include <stdint.h>

/* evt ids. gui_app_fwk.c (middleware) hard-codes TR_APP = 2 -- keep in sync. */
enum
{
    TR_BOOT    = 1,   /* arg 1 = previous session ended in a silent reboot, 0 = normal boot */
    TR_APP     = 2,   /* tag = app id; arg = 'S'tart 'R'esume 'P'ause 'X'=stop */
    TR_GESTURE = 3,   /* tag = event name; arg = GESTURE_EVENT_* mask */
    TR_BLE     = 4,   /* arg low byte 1 = connected, 0 = disconnected; high byte = reason */
    TR_SCREEN  = 5,   /* arg 1 = on, 0 = off */
    TR_DFU     = 6,   /* arg high byte = image id; low byte 1 start, 2 abort, 3 done, 4 failed */
    TR_FILE    = 7,   /* tag = file name; arg 1 = start, 2 = done, 3 = failed */
    TR_HOME    = 8,   /* main-screen state flip: bit0 = on watch face; bits1..8 = which
                         overlay/page holds it: inst msg cc mouse ai speech dev hosted */
};

/* tag: up to 8 chars are kept (may be NULL). */
void skai_trail(uint8_t evt, uint16_t arg, const char *tag);

#endif /* SKAI_TRAIL_H */
