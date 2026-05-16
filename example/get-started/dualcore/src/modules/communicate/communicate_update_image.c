/**
 ******************************************************************************
 * @file   communicate_update_image.c
 * @author Skaiwalk software development team
 ******************************************************************************
 */
/**
 * Copyright (c) 2018 - 2024, Skaiwalk Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Skaiwalk integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of Skaiwalk or its contributors may not be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Skaiwalk integrated circuit.
 *
 * 5. Any binary form of this software must not be reverse engineered, decompiled, modified,
 *    or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SKAIWALK TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SKAIWALK TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include <string.h>
#include "watch_global_data.h"
#include "communicate_parse.h"
#include "communicate_protocol.h"
#include "communicate_task.h"
#include "communicate_update_image.h"
#include "gui_app_fwk.h"
#include "watch_system_interact.h"
#include "ui_helper.h"
#include "bloc_setting.h"
#include "bloc_flash.h"
#include "bloc_peripheral.h"
#include "bloc_notification.h"

#ifdef PKG_USING_LZ4
#include "lz4.h"
#endif

#define DBG_TAG "commu.update.image"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define PAGE_SIZE 2048
#define BINARY_FILE_PREPARED_ERASE_SECTOR 8191

/* Little-endian 32-bit reader for the LZ4 fragment-size header. The phone
   writes this field LE (the LZ4 framing-format convention) — different from
   the BE used by the rest of the L2 protocol, hence a local helper rather
   than the shared read_be32. */
static inline uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0]        | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

#ifdef PKG_USING_LZ4
#define LZ4_DECOMPRESS_BUFFER_SIZE (8 * 1024)  // 8KB decompression buffer
#define LZ4_COMPRESS_PACKET_MAX_SIZE (5 * 1024)  // 5KB max compressed packet size (increased from 4KB to handle edge cases)
#endif

typedef struct
{
	uint8_t msg_type;
	uint32_t size;
	uint8_t *data;
} flash_write_t;

typedef enum _DFU_FLASH_MSG_TYPE
{
	DFU_FLASH_MSG_TYPE_DATA = 0,
	DFU_FLASH_MSG_TYPE_EXIT,
	DFU_FLASH_MSG_TYPE_VERIFY,
	DFU_FLASH_MSG_TYPE_ENTER,
#ifdef PKG_USING_LZ4
	DFU_FLASH_MSG_TYPE_COMPRESSED_DATA,
#endif
} DFU_FLASH_MSG_TYPE;

#ifdef PKG_USING_LZ4
// LZ4 compression context
typedef struct
{
	bool enabled;                          // Whether compression is enabled
	uint32_t original_total_size;          // Original uncompressed total size
	uint32_t compressed_total_size;        // Total compressed size
	uint32_t decompressed_size;            // Accumulated decompressed size
	uint8_t *compress_buffer;              // Buffer for compressed data
	uint32_t compress_buf_offset;          // Current offset in compress buffer
	uint32_t expected_packet_size;         // Expected compressed packet size
} lz4_decompress_ctx_t;

static lz4_decompress_ctx_t g_lz4_ctx = {0};
#endif

volatile uint32_t watch_image_write_addr;
volatile uint32_t watch_image_cur_addr;
volatile uint32_t watch_image_size;
volatile uint32_t watch_image_end_addr;
volatile bool is_end_addr_equal_to_last_writed_addr = false;
volatile dfu_img_id_t watch_img_id;
static uint32_t binary_image_size_max;
static bool dfu_started_mark = false;

/* Page-aligned write buffer for the flash storer. Moved to file scope so a
   failed session can be aborted and the next session reset the offset cleanly
   — when these were function-static, a mid-image terminate left stale partial
   bytes that the next OTA attempt would write to the wrong flash address. */
static uint8_t  s_write_buffer[PAGE_SIZE];
static uint16_t s_write_buffer_count = 0;
static uint8_t calculate_ota_progress(void);
static bool watch_binary_image_update_handler(uint8_t *buf, uint16_t len);

void mark_ota_started(void)
{
	LOG_I("[OTA]mark OTA start");
	dfu_started_mark = true;
	watch_system_wakeup();
	setting_provider.set_power_save_mode(0);
}

uint32_t get_cur_watch_image_size(void)
{
	return watch_image_size;
}

static rt_mailbox_t watch_image_mailbox;

bool WristBandBinaryImageStore(uint8_t *buf, uint16_t len)
{
	bool res = flash_provider.write_page_sector(watch_image_write_addr, buf, len);
	if (!res)
	{
		LOG_E("write image failed");
		return false;
	}
	watch_image_write_addr += len;
	return true;
}

bool is_firmware_transmitted(void)
{
	return is_end_addr_equal_to_last_writed_addr;
}

static bool verify_binary_image(dfu_img_id_t id)
{
	uint32_t image_update_start_address;
	switch (id)
	{
	case DFU_IMG_ID_NAND_HCPU:
		image_update_start_address = HCPU_CODE_DOWNLOAD_START_ADDRESS;
		binary_image_size_max = HCPU_CODE_MAX_SIZE;
		break;
	case DFU_IMG_ID_NAND_LCPU:
		image_update_start_address = LCPU_CODE_DOWNLOAD_START_ADDRESS;
		binary_image_size_max = LCPU_CODE_MAX_SIZE;
		break;
	case DFU_IMG_ID_NAND_FTAB:
		image_update_start_address = FTAB_START_DOWNLOAD_ADDRESS;
		binary_image_size_max = FTAB_MAX_SIZE;
		break;
	case DFU_IMG_ID_NAND_BOOTLOADER:
		image_update_start_address = BOOTLOADER_DOWNLOAD_START_ADDRESS;
		binary_image_size_max = BOOTLOADER_MAX_SIZE;
		break;
	case DFU_IMG_ID_NAND_LCPU_PATCH:
		image_update_start_address = LCPU_PATCH_DOWNLOAD_START_ADDRESS;
		binary_image_size_max = LCPU_PATCH_MAX_SIZE;
		break;
	case DFU_IMG_ID_NAND_IMAGE:
		image_update_start_address = FS_START_DOWNLOAD_ADDRESS;
		binary_image_size_max = FS_MAX_SIZE;
		break;
	default:
		LOG_E("Invalid image id");
		return false;
	}

	return flash_provider.check_image_write_address(watch_image_write_addr, watch_image_size, image_update_start_address, binary_image_size_max);
}

/* Chunked erase: a single flash_erase(binary_image_size_max) call holds the
   SPI bus and blocks every other flash user (share_prefs, log backend, file
   sync) for the whole duration. For the FS image region this can be many
   seconds. Slicing into 1 MB batches with a yield between each lets the
   peripheral task drain its queued SAVE_SHARE_PREFS / other events between
   sub-erases. Correctness is identical — flash_erase is idempotent over
   adjacent ranges. */
#define DFU_ERASE_BATCH_BYTES (1024 * 1024)

static void erase_image_region_chunked(uint32_t addr, uint32_t total_size)
{
	uint32_t erased = 0;
	while (erased < total_size)
	{
		uint32_t this_chunk = total_size - erased;
		if (this_chunk > DFU_ERASE_BATCH_BYTES) this_chunk = DFU_ERASE_BATCH_BYTES;
		flash_provider.erase_sector(addr + erased, this_chunk);
		erased += this_chunk;
		/* Give the peripheral task / BLE rx thread a chance to run before
		   we grab the SPI bus again for the next batch. */
		rt_thread_yield();
	}
}

static bool verify_ble_dfu_image(dfu_img_id_t id)
{
	bool isValid = verify_binary_image(id);
	if (isValid)
	{
		LOG_I("Binary Image[%d]is valid, max image size: %d", id, binary_image_size_max);
		erase_image_region_chunked(watch_image_write_addr, binary_image_size_max);
		return true;
	}
	else
	{
		LOG_E("Binary Image is invalid");
		return false;
	}
}

#ifdef PKG_USING_LZ4
// Initialize LZ4 decompression context
static void lz4_decompress_init(uint32_t original_size, uint32_t compressed_size)
{
	memset(&g_lz4_ctx, 0, sizeof(lz4_decompress_ctx_t));
	g_lz4_ctx.enabled = true;
	g_lz4_ctx.original_total_size = original_size;
	g_lz4_ctx.compressed_total_size = compressed_size;
	g_lz4_ctx.decompressed_size = 0;
	g_lz4_ctx.compress_buf_offset = 0;

	// Allocate compress buffer
	g_lz4_ctx.compress_buffer = rt_malloc(LZ4_COMPRESS_PACKET_MAX_SIZE);
	if (g_lz4_ctx.compress_buffer == NULL)
	{
		LOG_E("Failed to allocate LZ4 compress buffer");
		g_lz4_ctx.enabled = false;
	}
	else
	{
		LOG_I("LZ4 decompress init: orig_size=%d, comp_size=%d", original_size, compressed_size);
	}
}

// Clean up LZ4 decompression context
static void lz4_decompress_cleanup(void)
{
	if (g_lz4_ctx.compress_buffer != NULL)
	{
		rt_free(g_lz4_ctx.compress_buffer);
		g_lz4_ctx.compress_buffer = NULL;
	}
	memset(&g_lz4_ctx, 0, sizeof(lz4_decompress_ctx_t));
}

// Handle compressed data packet and decompress
static bool watch_compressed_image_update_handler(uint8_t *buf, uint16_t len)
{
	if (!g_lz4_ctx.enabled || g_lz4_ctx.compress_buffer == NULL)
	{
		LOG_E("LZ4 context not initialized");
		return false;
	}

	// Copy compressed data to buffer
	if (g_lz4_ctx.compress_buf_offset + len > LZ4_COMPRESS_PACKET_MAX_SIZE)
	{
		LOG_E("Compressed packet too large: %d + %d > %d",
			  g_lz4_ctx.compress_buf_offset, len, LZ4_COMPRESS_PACKET_MAX_SIZE);
		return false;
	}

	memcpy(g_lz4_ctx.compress_buffer + g_lz4_ctx.compress_buf_offset, buf, len);
	g_lz4_ctx.compress_buf_offset += len;

	// Check if we have received a complete compressed packet
	// The first 4 bytes of each packet should contain the compressed size
	if (g_lz4_ctx.compress_buf_offset >= 4)
	{
		// Extract compressed packet size from header if not already done
		if (g_lz4_ctx.expected_packet_size == 0)
		{
			g_lz4_ctx.expected_packet_size = read_le32(g_lz4_ctx.compress_buffer);

			// Validate packet size is reasonable (between 1 byte and buffer capacity - 4)
			if (g_lz4_ctx.expected_packet_size == 0 ||
			    g_lz4_ctx.expected_packet_size > LZ4_COMPRESS_PACKET_MAX_SIZE - 4)
			{
				LOG_E("Invalid compressed packet size in header: %d (valid range: 1-%d)",
					  g_lz4_ctx.expected_packet_size, LZ4_COMPRESS_PACKET_MAX_SIZE - 4);
				LOG_E("Header bytes: [0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X",
					  g_lz4_ctx.compress_buffer[0], g_lz4_ctx.compress_buffer[1],
					  g_lz4_ctx.compress_buffer[2], g_lz4_ctx.compress_buffer[3]);
				g_lz4_ctx.expected_packet_size = 0;
				return false;
			}

			LOG_D("Expecting compressed packet: %d bytes", g_lz4_ctx.expected_packet_size);
		}

		uint32_t packet_compressed_size = g_lz4_ctx.expected_packet_size;

		// Check if we have received the complete packet (header + data)
		if (g_lz4_ctx.compress_buf_offset >= packet_compressed_size + 4)
		{
			// Allocate decompression buffer
			uint8_t *decompress_buf = rt_malloc(LZ4_DECOMPRESS_BUFFER_SIZE);
			if (decompress_buf == NULL)
			{
				LOG_E("Failed to allocate decompress buffer");
				return false;
			}

			// Decompress using LZ4
			int decompressed_bytes = LZ4_decompress_safe(
				(const char *)(g_lz4_ctx.compress_buffer + 4),  // Skip 4-byte header
				(char *)decompress_buf,
				packet_compressed_size,
				LZ4_DECOMPRESS_BUFFER_SIZE
			);

			if (decompressed_bytes < 0)
			{
				LOG_E("LZ4 decompression failed: %d", decompressed_bytes);
				rt_free(decompress_buf);
				return false;
			}

			LOG_D("Decompressed %d bytes from %d bytes (ratio: %.2f%%)",
				  decompressed_bytes, packet_compressed_size,
				  (float)packet_compressed_size * 100.0f / decompressed_bytes);

			// Write decompressed data to flash using existing handler
			bool result = watch_binary_image_update_handler(decompress_buf, decompressed_bytes);

			rt_free(decompress_buf);

			// Update decompressed size counter
			g_lz4_ctx.decompressed_size += decompressed_bytes;

			// Reset buffer and expected size for next packet
			g_lz4_ctx.compress_buf_offset = 0;
			g_lz4_ctx.expected_packet_size = 0;

			return result;
		}
	}

	// Still waiting for more data
	return true;
}
#endif

static bool watch_binary_image_update_handler(uint8_t *buf, uint16_t len)
{
	// Copy data to write buffer
	for (uint16_t i = 0; i < len; i++)
	{
		watch_image_cur_addr++;
		s_write_buffer[s_write_buffer_count++] = buf[i];
		// If write buffer is full, write to flash
		if (s_write_buffer_count == PAGE_SIZE)
		{
			if (!WristBandBinaryImageStore(s_write_buffer, PAGE_SIZE))
			{
				LOG_E("Failed to write to flash");
				return false;
			}
			s_write_buffer_count = 0;
		}
	}

	if (watch_image_cur_addr == watch_image_end_addr)
	{
		is_end_addr_equal_to_last_writed_addr = true;
		LOG_I("Firmware image received completely");
		if (s_write_buffer_count > 0)
		{
			LOG_D("Write remaining data");
			if (s_write_buffer_count < PAGE_SIZE)
			{
				memset(s_write_buffer + s_write_buffer_count, 0xFF, PAGE_SIZE - s_write_buffer_count);
			}
			if (!WristBandBinaryImageStore(s_write_buffer, PAGE_SIZE))
			{
				LOG_E("Failed to write remaining data to flash");
				return false;
			}
			s_write_buffer_count = 0;
		}
	}
	else
	{
		LOG_W("Current address(0x%08X) not equal to end address(0x%08X)", watch_image_cur_addr, watch_image_end_addr);
	}
	return true;
}

static image_header_t temp_header[DFU_IMG_ID_NAND_MAX];
static int dfu_completed_handler(void)
{
	int ret = RT_EOK;
	if (is_firmware_transmitted())
	{
		image_header_t header;
		header.id = watch_img_id;
		header.length = get_cur_watch_image_size();
		temp_header[watch_img_id] = header;
		LOG_I("firmware id(%d) transmission(%d bytes) success", watch_img_id, header.length);
	}
	else
	{
		ret = -1;
		/* Show how short the upload came so it's clear whether the phone sent
		   END too early or our pipeline dropped bytes. Most often this is the
		   phone-side OTA logic giving up — single-frame chunks arrive in order
		   on this end, so missing bytes here means missing bytes on the wire. */
		uint32_t received = watch_image_size -
		                    (watch_image_end_addr - watch_image_cur_addr);
		LOG_E("firmware transmission failed: id=%d got %u of %u bytes (%u%%)",
		      watch_img_id,
		      (unsigned)received,
		      (unsigned)watch_image_size,
		      watch_image_size ? (unsigned)(received * 100u / watch_image_size) : 0u);
	}
	return ret;
}

static int ble_dfu_thread_run = 0;

bool is_ble_dfu_thread_running(void)
{
	return ble_dfu_thread_run;
}

void set_ble_dfu_thread_run(int run)
{
	ble_dfu_thread_run = run;
}

static void notify_update_status_to_client(uint8_t status)
{
	commu_send_ota_status(status);
}

static int ota_progress = -1;

/* Abort the current DFU session and clean up all per-session state so the
   next attempt starts from a known-good baseline. Does NOT reboot — for
   silent OTA the user is using the watch and a reboot would interrupt
   them. The partially-written flash region is the OTA target (separate from
   the currently-running firmware), and ftab is only swapped on
   DFU_FLASH_MSG_TYPE_VERIFY success, so the running image is unaffected
   by an aborted upload. */
static void terminate_ble_dfu_process(void)
{
	if (!ble_dfu_thread_run) return;

	notify_update_status_to_client(0);
	set_ble_dfu_thread_run(0);
	ota_progress = -1;
	handle_download_progress_update(ota_progress);

	/* Drop stale state so a retry can erase + write cleanly. */
	s_write_buffer_count = 0;
	is_end_addr_equal_to_last_writed_addr = false;
#ifdef PKG_USING_LZ4
	lz4_decompress_cleanup();
#endif
}

#define DFU_INACTIVITY_TIMEOUT_TICKS (5 * RT_TICK_PER_SECOND)

static void ble_dfu_flash_write()
{
	ble_dfu_thread_run = 1;
	watch_image_mailbox = rt_mb_create("dfu_flash", 12, RT_IPC_FLAG_FIFO);

	if (watch_image_mailbox == NULL)
	{
		LOG_E("Failed to create mailbox");
		return;
	}
	skaiwatch_ble_set_performance(BLE_PERF_ULTRA);
	peripheral_provider.subscribe_accelerometer_sensor(false);

	LOG_I("ble_dfu_flash_write thread started");

	while (ble_dfu_thread_run)
	{
		flash_write_t *fwrite;
		uint32_t p;
		/* Inactivity timeout: auto terminate if no DFU message within 5 seconds */
		int ret = rt_mb_recv(watch_image_mailbox, (rt_uint32_t *)&p, DFU_INACTIVITY_TIMEOUT_TICKS);
		if (ret == RT_EOK)
		{
			fwrite = (flash_write_t *)p;
			switch (fwrite->msg_type)
			{
			case DFU_FLASH_MSG_TYPE_ENTER:
			{
				LOG_D("DFU_FLASH_MSG_TYPE_ENTER, id: %d", watch_img_id);
				bool success = verify_ble_dfu_image(watch_img_id);
				if (!success)
				{
					terminate_ble_dfu_process();
				}
				break;
			}
			case DFU_FLASH_MSG_TYPE_DATA:
			{
				// LOG_D("DFU_FLASH_MSG_TYPE_DATA, size: %d", fwrite->size);
				bool res = watch_binary_image_update_handler(fwrite->data, fwrite->size);
				if (!res)
				{
					LOG_E("watch_binary_image_update_handler failed");
					terminate_ble_dfu_process();
				}
				else
				{
					uint8_t current_progress = calculate_ota_progress();
					if (ota_progress != current_progress)
					{
						ota_progress = current_progress;
						handle_download_progress_update(ota_progress);
					}
				}
			}
			break;
#ifdef PKG_USING_LZ4
			case DFU_FLASH_MSG_TYPE_COMPRESSED_DATA:
			{
				// LOG_D("DFU_FLASH_MSG_TYPE_COMPRESSED_DATA, size: %d", fwrite->size);
				bool res = watch_compressed_image_update_handler(fwrite->data, fwrite->size);
				if (!res)
				{
					LOG_E("watch_compressed_image_update_handler failed");
					terminate_ble_dfu_process();
				}
				else
				{
					uint8_t current_progress = calculate_ota_progress();
					if (ota_progress != current_progress)
					{
						ota_progress = current_progress;
						handle_download_progress_update(ota_progress);
					}
				}
			}
			break;
#endif
			case DFU_FLASH_MSG_TYPE_EXIT:
			{
				LOG_D("DFU_FLASH_MSG_TYPE_EXIT");
#ifdef PKG_USING_LZ4
				lz4_decompress_cleanup();
#endif
				int ret2 = dfu_completed_handler();
				if (ret2 == RT_EOK)
				{
					notify_update_status_to_client(1);
				}
				else
				{
					terminate_ble_dfu_process();
				}
			}
			break;
			case DFU_FLASH_MSG_TYPE_VERIFY:
			{
				LOG_D("DFU_FLASH_MSG_TYPE_VERIFY");
				int ret2 = RT_EOK;
				for (int i = 0; i < DFU_IMG_ID_NAND_MAX; i++)
				{
					if (temp_header[i].length > 0)
					{
						ret2 = dfu_check_fw_upgrade(temp_header[i]);
						if (ret2 == RT_EOK)
						{
							LOG_D("dfu_check_fw_upgrade success id(%d)", i);
						}
						else
						{
							LOG_E("dfu_check_fw_upgrade failed id(%d)", i);
							break;
						}
					}
				}
				if (ret2 == RT_EOK)
				{
					/* OTA verified -- about to reboot into the new image.
					 * Flush the dismissed-notification ring so notifications
					 * the user already dealt with don't re-pop after the
					 * BLE reconnects post-reboot. We deliberately do NOT
					 * call this on regular reboots / crashes -- the ring
					 * staying RAM-only there is by design. */
					bloc_notification_save_dismissed_to_flash();
					drv_reboot();
				}
				break;
			}
			default:
				LOG_E("Unknown message type: %d", fwrite->msg_type);
				break;
			}
			if (fwrite->data != NULL)
			{
				rt_free(fwrite->data);
				fwrite->data = NULL;
			}
			rt_free(fwrite);
		}
		else
		{
			/* Timeout (no DFU command) */
			LOG_W("DFU inactivity >5s, terminating");
			terminate_ble_dfu_process();
		}
	}

	if (watch_image_mailbox)
	{
		LOG_D("mb delete");
		rt_mb_delete(watch_image_mailbox);
		watch_image_mailbox = NULL;
	}

	skaiwatch_ble_set_performance(BLE_PERF_SLOW);
	peripheral_provider.subscribe_accelerometer_sensor(true);
	setting_provider.set_power_save_mode(1);
}
static rt_thread_t ble_dfu_flash_write_thread_start()
{
#ifndef BSP_USING_PC_SIMULATOR
	rt_thread_t tid;
	tid = rt_thread_create("ble_dfu_flash", ble_dfu_flash_write, NULL, 4096, 17, 10);
	rt_thread_startup(tid);
	return tid;
#else
	return NULL;
#endif
}

void init_ble_dfu_thread(dfu_img_id_t id, uint32_t dest_addr, uint32_t size)
{
	if (!dfu_started_mark)
	{
		notification_t notification;
		notification.sec_time = SkaiWatchSys.SecondCountRTC;
		notification.type = Notify_others;
		notification.state = true;
		strcpy(notification.id, "ota_start");
		strcpy(notification.title, "OTA Update hint");
		strcpy(notification.message, "Please update your smartphone app to the latest version to ensure compatibility.");
		interact_with_notification(&notification);
		return;
	}
	watch_img_id = id;
	is_end_addr_equal_to_last_writed_addr = false;
	watch_image_write_addr = dest_addr;
	watch_image_cur_addr = dest_addr;
	watch_image_size = size;
	watch_image_end_addr = dest_addr + size;
	ota_progress = -1;
	/* Defensive reset: if a previous session aborted mid-image without going
	   through terminate_ble_dfu_process (e.g., process crash before that ran),
	   the page buffer could carry stale bytes that would land at the wrong
	   address in the new image. Zeroing the count guarantees a clean start. */
	s_write_buffer_count = 0;

	/* verify_ble_dfu_image() is run later from the DFU thread on receipt of
	   DFU_FLASH_MSG_TYPE_ENTER, not here — running it on the BLE rx thread
	   serializes flash-erase against incoming packets. */
	if (ble_dfu_thread_run == 0)
	{
		ble_dfu_thread_run = 1;
#ifndef BSP_USING_PC_SIMULATOR
		ble_dfu_flash_write_thread_start();
#endif
	}
	else
	{
		LOG_D("dfu thread is already running");
	}
	rt_thread_mdelay(50);
	set_ble_dfu_start();
}

static uint8_t calculate_ota_progress(void)
{
	uint32_t total_size = watch_image_size;
	uint32_t current_size = watch_image_cur_addr - flash_provider.get_ota_address(watch_img_id);
	uint8_t prograss = (current_size * 100) / total_size;
	return prograss;
}

/* Backpressure: if the DFU thread is behind (flash write slower than BLE rx),
   wait up to this long for a mailbox slot before declaring failure. The BLE
   stack's link-layer flow control then naturally throttles the phone — no
   chunks dropped. Tuned long enough to cover one PAGE_SIZE flash_write
   (≈250 µs on NAND, plus the SPI bus serializes against prefs writes that
   can take tens of ms), short enough that a wedged DFU thread still aborts
   the session via the 5 s inactivity timeout above. */
#define DFU_MAILBOX_SEND_TIMEOUT_TICKS (rt_tick_from_millisecond(200))

static bool send_dfu_flash_msg(uint8_t msg_type, uint32_t size, uint8_t *data)
{
	/* Mailbox NULL is now an expected state after the DFU thread tears down
	   (no reboot path). The high-frequency caller, handle_ble_dfu_data, is
	   gated separately so we should normally never reach here with NULL —
	   but ENTER/EXIT/VERIFY can still arrive during the small window between
	   thread startup creating the mailbox and the BLE rx producing those
	   control messages. Log at debug only to avoid spamming the trace. */
	if (watch_image_mailbox == NULL)
	{
		LOG_D("dfu mailbox not ready, drop msg_type=%d", msg_type);
		return false;
	}

	flash_write_t *fwrite = rt_malloc(sizeof(flash_write_t));
	if (fwrite == NULL)
	{
		LOG_E("[%s]malloc failed", __func__);
		return false;
	}

	fwrite->msg_type = msg_type;
	if (size != 0 && data != NULL)
	{
		fwrite->size = size;
		fwrite->data = rt_malloc(size);
		if (fwrite->data == NULL)
		{
			LOG_E("[%s]malloc failed for data", __func__);
			rt_free(fwrite);
			return false;
		}
		memcpy(fwrite->data, data, size);
	}
	else
	{
		fwrite->size = 0;
		fwrite->data = NULL;
	}

	/* rt_mb_send is non-blocking; on -RT_EFULL the fwrite + data would leak
	   AND the chunk would be silently dropped (then image verification fails
	   without a clear cause). Use the _wait variant for real backpressure;
	   on persistent failure, free everything and surface the error so the
	   caller can terminate the DFU session cleanly. */
	rt_err_t mb_ret = rt_mb_send_wait(watch_image_mailbox,
	                                  (rt_uint32_t)fwrite,
	                                  DFU_MAILBOX_SEND_TIMEOUT_TICKS);
	if (mb_ret != RT_EOK)
	{
		LOG_E("[%s]mailbox full after timeout (ret=%d), drop msg_type=%d size=%u",
		      __func__, mb_ret, msg_type, (unsigned)size);
		if (fwrite->data) rt_free(fwrite->data);
		rt_free(fwrite);
		return false;
	}
	return true;
}

void handle_ble_dfu_data(uint8_t *buf, uint16_t len)
{
	/* Drop chunks that arrive after the DFU session has been torn down. The
	   phone may still have packets in flight (or retry blindly) for a moment
	   after we abort — without this gate, send_dfu_flash_msg below would fire
	   a LOG_D per chunk and call terminate_ble_dfu_process repeatedly. Once
	   the next SYNC_START arrives we'll re-init and start accepting again. */
	if (!ble_dfu_thread_run) return;

	if (len == 0 || buf == NULL)
	{
		LOG_E("[%s]size is 0 or data is NULL", __func__);
		terminate_ble_dfu_process();
		return;
	}

#ifdef PKG_USING_LZ4
	// If LZ4 compression is enabled, use compressed data handler
	if (g_lz4_ctx.enabled)
	{
		if (!send_dfu_flash_msg(DFU_FLASH_MSG_TYPE_COMPRESSED_DATA, len, buf))
		{
			terminate_ble_dfu_process();
		}
		return;
	}
#endif

	// Otherwise use normal uncompressed data handler
	if (!send_dfu_flash_msg(DFU_FLASH_MSG_TYPE_DATA, len, buf))
	{
		terminate_ble_dfu_process();
	}
}

void stop_ble_dfu_thread(void)
{
	if (!send_dfu_flash_msg(DFU_FLASH_MSG_TYPE_EXIT, 0, NULL))
	{
		terminate_ble_dfu_process();
	}
}

void verify_and_upgrade_dfu_image(void)
{
	if (!send_dfu_flash_msg(DFU_FLASH_MSG_TYPE_VERIFY, 0, NULL))
	{
		terminate_ble_dfu_process();
	}
}

void set_ble_dfu_start(void)
{
	if (!send_dfu_flash_msg(DFU_FLASH_MSG_TYPE_ENTER, 0, NULL))
	{
		terminate_ble_dfu_process();
	}
}

#ifdef PKG_USING_LZ4
/**
 * @brief Initialize LZ4 compressed DFU session
 * @param id Image ID
 * @param dest_addr Destination flash address
 * @param original_size Original uncompressed size
 * @param compressed_size Total compressed size
 */
void init_ble_dfu_thread_compressed(dfu_img_id_t id, uint32_t dest_addr, uint32_t original_size, uint32_t compressed_size)
{
	// Initialize normal DFU session first
	init_ble_dfu_thread(id, dest_addr, original_size);

	// Initialize LZ4 decompression context
	lz4_decompress_init(original_size, compressed_size);
}

/**
 * @brief Check if LZ4 compression is enabled
 * @return true if LZ4 is enabled, false otherwise
 */
bool is_lz4_compression_enabled(void)
{
	return g_lz4_ctx.enabled;
}
#endif

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/