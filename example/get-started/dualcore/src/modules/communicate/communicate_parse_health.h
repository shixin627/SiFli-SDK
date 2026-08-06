/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_parse_health.h
* @brief
* @details
* @author
* @date
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef __COMMUNICATE_PARSE_HEALTH_H__
#define __COMMUNICATE_PARSE_HEALTH_H__


#ifdef __cplusplus
extern "C" {
#endif



#include <stdbool.h>
#include "stdint.h"

typedef enum
{
    KEY_REQUEST_DATA   = 0x01,
    KEY_RETURN_SPORTS_DATA  = 0x02,
    KEY_RETURN_SLEEP_DATA  = 0x03,
    KEY_DAILY_DATA_SYNC = 0x09,
    KEY_LATEST_DATA_SYNC = 0x0A,
    KEY_REQUEST_HEART_DATA = 0x0D,
    KEY_HEART_DATA_RETURN = 0x0F,
    /* Background daily HR-curve sample {timestamp:u32 LE, bpm:u8}. Distinct
       from KEY_HEART_DATA_RETURN (0x0F, live single value during exercise):
       this is the timestamped, sparse (~15 min) ambient HR that the phone
       accumulates into a daily heart-rate curve. */
    KEY_HEART_CURVE_SAMPLE = 0x10,
    /* Why an HR point was NOT recorded for a 5-min bucket {timestamp:u32 LE,
       reason:u8}. Same command group + persistence path as 0x10; the phone
       draws it as a coloured gap label on the HR curve. Reason wire codes
       1..7 are frozen in ADR-0011 D2 (do not reorder). */
    KEY_HEART_CURVE_SKIP = 0x11,
    /* Wear-detect diagnostic record (watch -> phone), 14 bytes LE:
       {ts:u32, evt:u8, status:u8, dc/4:u16, PI*1e6:u16, PIrange*1e6:u16,
       IMUvar*1e4:u16}. evt codes 1..8 frozen (see watch_sys_wear_diag_evt_t).
       Cable-less units have no serial console; this is their only way to
       expose nightly wear-detect internals (phone appends to a daily CSV). */
    KEY_WEAR_DIAG = 0x12,
    /* Per-minute sleep-fusion diagnostic {ts:u32, score:u16, hr:u8, hr_std:u8,
       stage:u8, veto:u8, rhr:u8, worn:u8, rest:u8, fresh:u8, total:u16, deep:u16,
       rem:u16, light:u16, pi_e3:u16, frame_pct:u16, rate_info:u16} = 28B LE. Phone appends to a
       daily CSV for offline SQI-threshold + missed-night analysis. frame_pct is
       the last PPG burst's delivered frames as a % of 25 Hz * burst seconds —
       the test for whether the nightly 2x HR episodes are lost-frame timebase
       errors (~50%) rather than physiology (~100%). rate_info packs the HR
       frame divider (high byte) with the algo-side frame % (low byte): the chip
       runs 25 Hz for HR but 100 Hz for HRV, so the driver decimates by
       chip_rate/25 -- a divider of 2 while the chip is at 25 Hz feeds the
       algorithm 12.5 Hz while it believes 25 and it reports exactly DOUBLE.
       Temporary diagnostic stream (mirrors KEY_WEAR_DIAG). */
    KEY_SLEEP_DIAG = 0x13,
    /* PHONE -> WATCH. HR-curve backfill request {since_ts:u32 BE}: "I already
       hold every HR point up to and INCLUDING since_ts — replay what is newer."
       Sent by the phone right after it (re)connects, using MAX(ts_epoch) from
       its own hr_curve table (0 = phone has nothing, watch replays its cap).
       The watch answers by re-emitting ordinary KEY_HEART_CURVE_SAMPLE (0x10) /
       KEY_HEART_CURVE_SKIP (0x11) frames from its own /health/hr_*.json store,
       so the phone needs no new parsing path and its ts_epoch PRIMARY KEY
       de-dups any overlap. Closes the store-and-forward loop: the watch has
       always persisted these points, but nothing ever read them back, so a
       disconnected stretch (e.g. a whole night) was lost for good.
       NOTE endianness: u32 BE here to match every other inbound key parsed in
       communicate_parse_health.c (read_be32); the REPLY frames stay LE because
       0x10/0x11 are already defined that way. */
    KEY_HR_BACKFILL_REQ = 0x14,
    /* WATCH -> PHONE. Continuous-HR diagnostic batch, 30 s of raw 1 Hz algorithm
       output, sent only while the Settings toggle is on. Layout LE:
         base_ts:u32 | interval_s:u8 | count:u8 | bpm[count]:u8
                                               | qscore[count]:u8
                                               | qlevel[count]:u8
                                               | accst[count]:u8
                                               | accel[count]:u8
                                               | pi_e3[count]:u16
       sample[i] is at base_ts + i*interval_s; bpm 0 = the algo emitted nothing
       that second. accst = (algo motion state << 5) | algo scene id — the
       ALGORITHM's own classification, which decides how aggressively it tracks;
       accel = wrist motion that second; pi_e3 = perfusion index x1000 that
       second, the only in-band signal-quality measure this lib still populates.
       Deliberately NOT KEY_HEART_CURVE_SAMPLE: raw unfiltered output at 60x the
       curve's density must not pollute the user-facing hr_curve. Phone writes it
       to its own CSV. Temporary — remove with the experiment. */
    KEY_HR_CONT_DIAG = 0x15,
    /* WATCH -> PHONE. One 10.24 s window of detrended raw PPG, captured at the
       moment hr_autocorr produced an implausible estimate. Layout LE:
         ts:u32 | bpm:u8 | conf:u8 | count:u8 | reserved:u8 | int8[count]
       Sent at most once per burst, so a night costs a few hundred bytes.

       Exists because the offline suite cannot reproduce the field failures:
       sweeping the tie tolerance and accept threshold over nine combinations
       left all 81 synthetic cases passing, which means uniform noise plus a
       sine wander is not what this sensor actually produces. Tuning against
       that model would be tuning against imagination — the failing window has
       to come back and go into the suite. Temporary; remove once the estimator
       is settled. */
    KEY_HR_WINDOW_DUMP = 0x16,
} HEALTH_KEY;

void resolve_HealthData_command(uint8_t key, const uint8_t *pValue, uint16_t length);



#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PARSE_HEALTH_H__
