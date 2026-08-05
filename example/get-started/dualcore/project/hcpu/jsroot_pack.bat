@echo off
REM ADR-0010 Route B: GC + strip -> ~5.5 MB image, FAT formatted for the full 87.5 MB region.
REM Flash build\jsroot_packed.bin with release_gui.py (0x64300000).
if not exist build mkdir build
..\..\..\..\..\tools\mkfatimg\mkfatimg_nand\Release\mkfatimg.exe -pack ..\jsroot build\jsroot_packed.bin 44800 2048
