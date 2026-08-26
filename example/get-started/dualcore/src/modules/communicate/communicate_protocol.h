/**
*********************************************************************************************************
*               Copyright(c) 2018, Skaiwalk Corporation. All rights reserved.
*********************************************************************************************************
* @file         communicate_protocol.h
* @brief        斯凱沃克通信協議 — L2 framing + BLE TX wrappers
*********************************************************************************************************
*/

#ifndef __COMMUNICATE_PROTOCOL_H__
#define __COMMUNICATE_PROTOCOL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "communicate_update_image.h"

/* ──────────────────────────────────────────────────────────────────────────
   L2 frame layout (big-endian on the wire):

     byte[0]: command id        (cmd_id)
     byte[1]: header version    (0x00)
     byte[2]: first key         (key)
     byte[3]: bit 7 = MORE_FRAGMENTS, bit 0 = length high bit
     byte[4]: length low 8 bits
     byte[5..]: payload (up to 511 bytes per fragment)

   The legacy length decoder masks with 0x1FF, so bits 1..7 of byte[3] are
   transparently ignored by old peers — we use bit 7 to flag "more fragments
   coming" for the same (cmd, key) chain.
   ────────────────────────────────────────────────────────────────────────── */
#define L2_HEADER_VERSION       (0x00)
#define L2_FIRST_VALUE_POS      (5)
#define L2_MORE_FRAGMENTS_BIT   (0x80)

/* Hard cap on the reassembled L2 payload size (across all fragments of one
   logical command). Anything bigger gets dropped with an error log. */
/* 4096 撞到了(founder 2026-08-26):手機每次把**整份 backlog** 塞進一包 conv_state,
   一段 300 字的中文回覆就 ~1450 bytes,三則加上 JSON 結構 ≈ 4573 bytes → 整包在這裡
   被丟掉,手錶永遠停在倒數第二則、看不到最新的 AI 回覆(症狀完全不像傳輸問題)。
   拉到 8192 是止血;真正的解是手機端別每次重送整份對話(見 pushSiFliConvState)。
   代價 = 這個 static 緩衝 +4KB。 */
#define L2_REASSEMBLY_MAX_BYTES (8192)

/* ATT NTF/WRT carries its own 3-byte header, so the largest L2 frame we can
   put on the wire is mtu - 3. */
#define L2_ATT_OVERHEAD         (3)
#define BLE_DEFAULT_MTU         (23)

/* Upper bound for one BLE TX packet. BLE 5.0 caps ATT MTU at 517, giving an
   ATT NTF payload of at most 514 bytes; round up a touch for headroom. Used
   to size the per-fragment scratch buffer in skaiwatch_ble_send_l2. */
#define L2_TX_PKT_BUF_SIZE      (520)

/* Bluetooth Core Spec (Vol 3 Part F §3.2.9): a GATT attribute value is at
   most 512 octets, so a single notify/write payload tops out at 512 even
   when ATT_MTU is larger. Long Write/Read can exceed this but isn't used. */
#define BLE_MAX_ATTR_VALUE_LEN  (512)

/* The max_len declared on our BLE TX/RX characteristics. Phone-side writes
   and our own notify payloads are both bounded by this — exceeding it gets
   GATT_INVALID_ATTRIBUTE_LENGTH on the receiver. Used both in the GATT
   service declaration (main.c) and the L2 send path. The phone keeps its
   own copy (`watchCharValueMaxLen` in dart) that must match. */
#define BLE_APP_CHAR_MAX_LEN    (512)

    typedef enum
    {
        NONE = 0x00,
        IOS = 0x01,
        ANDROID = 0x02,
    } PHONE_OS_VERSION;

    /* BLE link performance tiers. Drives TX power + connection interval + PHY.
       Ordered by aggressiveness (SLOW < MEDIUM < FAST < ULTRA) — the guards in
       skaiwatch_ble_set_performance compare with '<', so keep them monotonic.
         SLOW   — screen-off idle; TPC owns TX power, 100-120 ms interval,
                  latency 9 (~1.2 s effective wake), 1M PHY (best margin).
         MEDIUM — screen-on idle; TPC owns TX power, 30-50 ms interval,
                  latency 2 (~150 ms effective wake), 1M PHY. Responsive while
                  the display is on, where BLE current is dwarfed by the panel.
                  Callers never request this directly: set_performance remaps a
                  SLOW request to MEDIUM whenever the screen is on.
         FAST   — interactive transfers (V2T / file sync / HID): pinned to
                  +10 dBm, 15-35 ms interval, 2M PHY, Apple-compliant window.
         ULTRA  — OTA only. Pushes the interval below the Apple guideline
                  (Min 7.5 ms, Max-Min < 20 ms) + 2M PHY to maximize throughput;
                  the peer may renegotiate. Avoid for non-OTA paths — iOS can
                  reject the param update entirely. */
    typedef enum
    {
        BLE_PERF_SLOW   = 0,
        BLE_PERF_MEDIUM = 1,
        BLE_PERF_FAST   = 2,
        BLE_PERF_ULTRA  = 3,
    } ble_perf_level_t;

    /******************* Function definition **********************************/
    extern void skaiwatch_ble_set_performance(ble_perf_level_t level);

    /* Send a payload as one logical L2 command. The function builds the L2
       header internally and splits the payload across multiple BLE writes if
       it exceeds the negotiated MTU; the caller does not need to allocate a
       buffer big enough for the whole frame. Prefer this over
       skaiwatch_ble_notify for new callers, especially when payload sizes
       can grow large (e.g. sensor batches). */
    extern bool skaiwatch_ble_send_l2(uint8_t cmd_id, uint8_t key,
                                       const uint8_t *payload, uint16_t payload_len);

    /* Legacy entry point: caller pre-builds the full L2 frame in `buf`. Kept
       wire-identical to old code on the fast path; auto-fragments only when
       the pre-built frame doesn't fit in one BLE write. */
    extern bool skaiwatch_ble_notify(uint8_t *buf, uint16_t length);

    extern bool skaiwatch_ble_audio_send(uint8_t *buf, uint16_t length);
    extern void L1_receive_data_without_crc_check(uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //__COMMUNICATE_PROTOCOL_H__
