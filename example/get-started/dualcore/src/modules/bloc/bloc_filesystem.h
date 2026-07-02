/*********************************************************************************************************
 *               Copyright(c) 2023, Skaiwalk Corporation. All rights reserved.
 **********************************************************************************************************
 * @file     bloc_filesystem.h
 * @brief    Business logic of voice to text page
 * @details
 * @author   jack
 * @date     2023-02-08
 * @version  v1.0
 *********************************************************************************************************
 */

#ifndef __BLOC_FILE_SYSTEMM_H__
#define __BLOC_FILE_SYSTEMM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#define FILE_PATH_MAX_LEN 128

	// 同步进度信息结构体
	typedef struct
	{
		bool sync_status; // 当前同步状态
		char sync_in_file_path[FILE_PATH_MAX_LEN];
		uint32_t total_bytes;		// 文件总大小（字节）
		uint32_t transferred_bytes; // 已传输大小（字节）
		uint8_t percent_complete;	// 完成百分比 (0-100)
	} sync_progress_t;

	extern sync_progress_t *get_sync_progress(void);
	extern char *get_sync_in_file_path(void);

	typedef struct
	{
		int (*sync_file)(const char *file_path, bool delete_after_sync);
		int (*sync_folder_files)(const char *folder_path, bool delete_after_sync);
		int (*delete_file)(const char *file_path);
	} BLocFileSystem;
	extern BLocFileSystem bloc_file_system;

	/* File receive functions */
	extern int bloc_start_receive_file(const char *file_path, uint32_t total_size);
	extern int bloc_receive_file_data(const uint8_t *data, uint16_t length);
	extern int bloc_end_receive_file(void);
	extern void received_file_handler(const char *path);

	typedef struct
	{
		char file_path[FILE_PATH_MAX_LEN];
		int fd;
		uint32_t total_size;
		uint32_t received_size;
		bool is_active;
		uint32_t last_receive_tick;  /* Timestamp of last data received */
	} file_receive_state_t;

	/* Instruction image request tracking */
	extern void set_pending_instruction_img_id(const char *id);

#ifdef __cplusplus
}
#endif

#endif //__BLOC_FILE_SYSTEMM_H__
