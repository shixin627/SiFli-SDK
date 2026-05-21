#!/usr/bin/env python3
"""Truncate a FAT image to its used extent while keeping the declared volume size.

Why: mkfatimg sizes the *output file* equal to the *volume size* (it writes the
whole RAM disk). So a full-region image (~87.5 MB) is a ~87.5 MB file that is
mostly empty -- flashing it would balloon flash time.

This tool keeps the device-visible volume size intact (BPB_TotSec is NOT
modified, so the watch still mounts the whole partition) but drops the trailing
*free* data area from the file. The flashed image then tracks the size of the
actual content (~6 MB), not the volume size, so flash time stays flat.

Correctness: the cut point is computed from the FAT (the highest allocated
cluster), never by scanning for trailing zeros -- so an asset whose final bytes
happen to be 0x00 is never clipped. The complete FAT table and root directory
sit before the data area and are always fully retained, so the device reads a
valid filesystem whose high clusters are marked free; their (un-flashed) data
bytes are don't-care on blank NAND.

Usage:
    truncate_fatimg.py <in.bin> <out.bin>
"""
import struct
import sys


def u16(b, off):
    return struct.unpack_from("<H", b, off)[0]


def u32(b, off):
    return struct.unpack_from("<I", b, off)[0]


def main():
    if len(sys.argv) != 3:
        print("usage: truncate_fatimg.py <in.bin> <out.bin>")
        return 1

    with open(sys.argv[1], "rb") as f:
        data = f.read()

    if len(data) < 512:
        print("error: file too small to be a FAT image")
        return 2

    # --- BIOS Parameter Block (BPB) ---
    bps = u16(data, 11)          # bytes per sector
    spc = data[13]               # sectors per cluster
    rsvd = u16(data, 14)         # reserved sector count (incl. boot sector)
    nfat = data[16]              # number of FAT copies
    rootent = u16(data, 17)      # root dir entries (FAT12/16; 0 on FAT32)
    totsec16 = u16(data, 19)
    fatsz16 = u16(data, 22)
    totsec32 = u32(data, 32)
    fatsz32 = u32(data, 36)

    if bps == 0 or (bps & (bps - 1)) != 0 or spc == 0:
        print("error: invalid BPB (bytes/sector=%d, sec/clus=%d) -- not a FAT image?"
              % (bps, spc))
        return 2
    if data[510] != 0x55 or data[511] != 0xAA:
        print("warning: boot signature 0x55AA not found at offset 510")

    totsec = totsec16 if totsec16 != 0 else totsec32
    fatsz = fatsz16 if fatsz16 != 0 else fatsz32

    root_dir_sectors = ((rootent * 32) + (bps - 1)) // bps
    first_data_sec = rsvd + nfat * fatsz + root_dir_sectors
    data_sec = totsec - first_data_sec
    nclust = data_sec // spc                     # count of data clusters

    if nclust < 4085:
        fat_type = 12
    elif nclust < 65525:
        fat_type = 16
    else:
        fat_type = 32

    # --- highest allocated cluster (scan the FAT; 0 == free) ---
    fat_off = rsvd * bps
    highest = 1                                  # clusters number from 2
    for c in range(2, nclust + 2):
        if fat_type == 16:
            v = u16(data, fat_off + c * 2)
        elif fat_type == 32:
            v = u32(data, fat_off + c * 4) & 0x0FFFFFFF
        else:  # FAT12: 1.5 bytes per entry
            byte_off = fat_off + (c * 3) // 2
            pair = u16(data, byte_off)
            v = (pair >> 4) if (c & 1) else (pair & 0x0FFF)
        if v != 0:
            highest = c

    # --- byte offset just past the last used cluster ---
    if highest >= 2:
        used_clusters = highest - 2 + 1
        cut = (first_data_sec + used_clusters * spc) * bps
    else:
        cut = first_data_sec * bps               # empty volume: keep metadata

    cut = max(cut, first_data_sec * bps)         # never clip FAT/root metadata
    cut = min(cut, len(data))                    # never exceed the source file

    with open(sys.argv[2], "wb") as f:
        f.write(data[:cut])

    vol_b = totsec * bps
    print("FAT%d  volume=%.1f MiB (%d sectors x %d B)  highest_used_cluster=%d"
          % (fat_type, vol_b / 1048576.0, totsec, bps, highest))
    print("input=%.2f MiB  ->  flashed=%.2f MiB (saved %.2f MiB)"
          % (len(data) / 1048576.0, cut / 1048576.0,
             (len(data) - cut) / 1048576.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
