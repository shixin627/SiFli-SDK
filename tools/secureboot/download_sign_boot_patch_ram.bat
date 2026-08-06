@echo Download Boot patch sigkey
@python download.py boot_patch_addr --port=%1
@echo Download SEC BOOT patch image part 1 to flash id 9
@python download.py img --eimg=boot_ram_patch_sign.bin --bksize=512 --flashid=9 --verbose=2 --port=%1