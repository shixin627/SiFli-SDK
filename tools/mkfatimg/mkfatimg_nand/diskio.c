/*-----------------------------------------------------------------------*/
/* RAM disk control module for Win32              (C)ChaN, 2014          */
/*-----------------------------------------------------------------------*/

#include <windows.h>
#include <assert.h>
#include "stdio.h"
#include "diskio.h"
#include "ff.h"
#include "map.h"

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/


extern BYTE *RamDisk;		/* Poiter to the active RAM disk (main.c) */
extern DWORD RamDiskSize;	/* Size of RAM disk in unit of sector */
extern int gc_ratio;
extern int block_size;

static struct dhara_map map;
static BYTE page_buffer[8192];
static struct dhara_nand nand = {
	.log2_page_size = 11,
	.log2_ppb = 6,
	.num_blocks = 16,
	.user_data = 0
};



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv		/* Physical drive nmuber */
)
{
	if (pdrv) return STA_NOINIT;

	int ret;
	int page_num_per_block;

	if (!RamDisk) {
		RamDisk = VirtualAlloc(0, RamDiskSize * FF_MIN_SS, MEM_COMMIT, PAGE_READWRITE);

		FillMemory(RamDisk, RamDiskSize * FF_MIN_SS, 0xFF);

		nand.user_data = (void *)RamDisk;

		page_num_per_block = block_size / FF_MIN_SS;

		if (512 == FF_MIN_SS)
		{
			nand.log2_page_size = 9;
		}
		else if (1024 == FF_MIN_SS)
		{
			nand.log2_page_size = 10;
		}
		else if (2048 == FF_MIN_SS)
		{
			nand.log2_page_size = 11;
		}
		else if (4096 == FF_MIN_SS)
		{
			nand.log2_page_size = 12;
		}
		else if (8192 == FF_MIN_SS)
		{
			nand.log2_page_size = 13;
		}
		else
		{
			printf("Invalid page size:%d\n", FF_MIN_SS);
			assert(0);
		}

		if (64 == page_num_per_block)
		{
			nand.log2_ppb = 6;
		}
		else if (32 == page_num_per_block)
		{
			nand.log2_ppb = 5;
		}
		else if (16 == page_num_per_block)
		{
			nand.log2_ppb = 4;
		}
		else
		{
			printf("Invalid ppb:%d,%d\n", page_num_per_block, block_size);
			assert(0);
		}

		nand.num_blocks = RamDiskSize * FF_MIN_SS / block_size;

		assert(FF_MIN_SS <= 8192);

		// init flash translation layer
		dhara_map_init(&map, &nand, page_buffer, gc_ratio);
		dhara_error_t err = DHARA_E_NONE;
		ret = dhara_map_resume(&map, &err);
		printf("init\n");
	}

    return 0;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0) */
)
{
	if (pdrv) return STA_NOINIT;

	return RamDisk ? 0 : STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,			/* Physical drive nmuber (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	DWORD sector,		/* Start sector number (LBA) */
	UINT count			/* Number of sectors to read */
)
{
	dhara_error_t err;
	uint32_t sector_size = (1 << nand.log2_page_size);

	//printf("disk_read:%d,%d\n", sector, count);

	// read *count* consecutive sectors
	for (int i = 0; i < count; i++) {
		int ret = dhara_map_read(&map, sector, buff, &err);
		if (ret) {
			printf("dhara read failed: %d, error: %d\n", ret, err);
			return RES_ERROR;
		}
		buff += sector_size; // sector size == page size
		sector++;
	}

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0) */
	const BYTE *buff,	/* Pointer to the data to be written */
	DWORD sector,		/* Start sector number (LBA) */
	UINT count			/* Number of sectors to write */
)
{
	dhara_error_t err;
	int ret;
	uint32_t sector_size = (1 << nand.log2_page_size);

	//printf("disk_write:%d,%d\n", sector, count);


	// write *count* consecutive sectors
	for (int i = 0; i < count; i++)
	{
		ret = dhara_map_write(&map, sector, buff, &err);
		if (ret)
		{
			printf("dhara write failed: %d, error: %d\n", ret, err);
			return RES_ERROR;
		}
		buff += sector_size; // sector size == page size
		sector++;
	}

	//ret = dhara_map_sync(&map, &err);
	//if (ret)
	{
	//	printf("dhara write failed: %d, error: %d\n", ret, err);
	//	return RES_ERROR;
	}

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Re-run the FTL resume on the current RamDisk                          */
/*-----------------------------------------------------------------------*/
/* Rebuilds the dhara map and resumes from whatever is currently in RamDisk,
   exactly as the device firmware does on cold boot. main.c's -pack verify
   calls this after erasing the buffer and reading the stripped image back,
   to prove dhara can resume from a GC'd + 0xFF-stripped image
   (ADR-0010 Open Q3). Returns 0 on success, non-zero if resume rejects it. */

int disk_resume (void)
{
	dhara_error_t err = DHARA_E_NONE;

	if (!RamDisk) return -1;

	dhara_map_init(&map, &nand, page_buffer, gc_ratio);
	return dhara_map_resume(&map, &err);
}


/*-----------------------------------------------------------------------*/
/* Re-pack live sectors into a fresh FTL at the front of a new buffer    */
/*-----------------------------------------------------------------------*/
/* dhara_map_gc_all() frees dead blocks but does NOT relocate live pages, so
   after the build's rewrite churn the ~5 MB of live data is scattered across
   the whole region and a strip of the GC'd image still spans ~full size.
   Here we read every live logical sector out of the current map and write it
   ONCE into a fresh dhara on a new buffer -- with no rewrites, that journal
   packs sequentially from page 0, so the stripped image is small. Caller
   VirtualFree()s *out_buf. Returns 0 on success. */

int disk_compact_to_front (BYTE **out_buf, DWORD *out_len)
{
	static struct dhara_map map2;
	static BYTE page_buffer2[8192];
	static BYTE sector_tmp[8192];
	struct dhara_nand nand2 = nand;          /* same geometry as the live device */
	dhara_error_t err = DHARA_E_NONE;
	BYTE *buf2;
	DWORD total_bytes = RamDiskSize * FF_MIN_SS;
	uint32_t page_size = (1 << nand.log2_page_size);
	dhara_sector_t cap, s;
	DWORD last;
	uint32_t i;

	if (!RamDisk) return -1;

	buf2 = (BYTE *)VirtualAlloc(0, total_bytes, MEM_COMMIT, PAGE_READWRITE);
	if (!buf2) return -1;
	FillMemory(buf2, total_bytes, 0xFF);     /* erased NAND */
	nand2.user_data = (void *)buf2;

	/* Fresh, empty FTL on the new buffer. */
	dhara_map_init(&map2, &nand2, page_buffer2, gc_ratio);
	dhara_map_resume(&map2, &err);

	cap = dhara_map_capacity(&map);
	for (s = 0; s < cap; s++) {
		int blank = 1;
		if (dhara_map_read(&map, s, sector_tmp, &err) != 0)
			continue;                        /* unmapped -> leave erased */
		for (i = 0; i < page_size; i++) {
			if (sector_tmp[i] != 0xFF) { blank = 0; break; }
		}
		if (blank) continue;                 /* all-0xFF reads back as 0xFF anyway */
		if (dhara_map_write(&map2, s, sector_tmp, &err) != 0) {
			VirtualFree(buf2, 0, MEM_RELEASE);
			return -2;
		}
	}
	dhara_map_sync(&map2, &err);

	/* Strip the trailing 0xFF, rounded up to a whole erase block. */
	last = total_bytes;
	while (last > 0 && buf2[last - 1] == 0xFF) last--;
	last = ((last + block_size - 1) / block_size) * block_size;
	if (last == 0) last = block_size;

	*out_buf = buf2;
	*out_len = last;
	return 0;
}


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0) */
	BYTE ctrl,		/* Control code */
	void* buff		/* Buffer to send/receive data block */
)
{
	DRESULT dr;
	dhara_error_t err;
	int ret;

    //printf("ioctrl:%d\n", ctrl);

	dr = RES_ERROR;
	if (!pdrv && RamDisk) {
		switch (ctrl) {
		case CTRL_SYNC:
			ret = dhara_map_sync(&map, &err);
			if (ret)
			{
				printf("dhara sync failed: %d, error: %d\n", ret, err);
				dr = RES_ERROR;
			}
			else
			{
				dr = RES_OK;
			}
			break;

		case GET_SECTOR_COUNT:
		{
			dhara_sector_t sector_count = dhara_map_capacity(&map);

            printf("sec_count:%d\n", sector_count);

			*(DWORD*)buff = sector_count;
			dr = RES_OK;
			break;
		}
		case GET_BLOCK_SIZE:
			*(DWORD*)buff = 1;
			dr = RES_OK;
			break;
		case GET_SECTOR_SIZE:
			*(DWORD*)buff = (1 << nand.log2_page_size);;
			dr = RES_OK;
			break;
		case FS_CLEAN_GARBAGE:
			ret = dhara_map_gc_all(&map, &err);
			if (ret)
			{
				printf("gc error\n");
				dr = RES_ERROR;
			}
			else
			{
				printf("map:%d,%d\n", dhara_journal_size(&map.journal), map.count);
				dr = RES_OK;
			}
			break;
		}

	}
	return dr;
}


