
# USB Mass Storage Device Example (NOR FLASH)

Source Path: example\cherryusb\device\nor_flash_disk

## Supported Platforms
<!-- Supported boards and chip platforms -->
+ sf32lb52-lcd_n16r8

## Overview
This example demonstrates exporting the NOR Flash `FS_REGION` area as a USB Mass Storage block device through CherryUSB MSC. It includes:
+ The PC can enumerate a USB Mass Storage device.
+ If the NOR Flash area already contains a valid FAT file system, the PC file manager can show the corresponding drive letter.
+ On first use, or after the area has been erased, the PC may prompt to format the disk. FAT32 is recommended. The drive letter is normally shown after formatting completes.

## How to Use the Example
<!-- Describe how to use the example. For rt_device based samples, list required config switches if any. -->

### Hardware Requirements
Prepare the following before running the example:
+ A supported development board ([Supported Platforms](quick_start)).
+ A USB-A to Type-C data cable that supports data transfer.
+ A USB-capable host (e.g., PC).

### menuconfig Configuration
This example does not enable DFS/Elm-FAT on the device side by default, and it does not pre-create a FAT file system. The USB host sees the exported NOR Flash area as a raw block device, and the file system is created by formatting it on the PC side.

### Build and Flash
In the example `project` directory, run the following `scons` command to build:

> scons --board=sf32lb52-lcd_n16r8 -j32

Then switch to the generated `project/build_xx` directory, run `uart_download.bat`, and follow the prompt to select the serial port for downloading:

> .\uart_download.bat

Example interaction:
> Uart Download

> please input the serial port num:5

For detailed build and download steps, refer to the [Quick Start Guide](quick_start).

## Expected Results
After startup:
1. Connect the board to the host using the USB data cable.
2. In the OS Device Manager (under Universal Serial Bus controllers), a new "USB Mass Storage Device" entry appears.
3. If the NOR Flash `FS_REGION` area has not been formatted, the PC may not immediately show a drive letter in the file manager, or it may prompt to format the disk. This is expected. Format it as FAT32 on the PC side.
4. After formatting, the PC file manager shows the corresponding drive letter. On later reconnects, the drive letter should appear directly as long as the area has not been erased or overwritten.

## Troubleshooting
| Issue | Possible Cause | Suggested Action |
|-------|----------------|------------------|
| USB disk not enumerated | Cable is power-only | Use a proper data cable |
| USB Mass Storage device is visible but no drive letter appears | The exported NOR Flash block device does not contain a valid file system | Format the disk as FAT32 on the PC side |
| The disk needs formatting after every reboot or reflash | The download/erase flow overwrote `FS_REGION`, or Flash write-back failed | Check that the download/erase range does not destroy `FS_REGION`, and inspect serial logs for Flash read/write errors |
| Flash read/write errors | NOR Flash init failure | Verify Flash wiring and initialization sequence |

## Reference Documents
Add related RT-Thread or CherryUSB documentation links here if needed.

## Revision History
| Version | Date   | Release Notes |
|:-------|:-------|:--------------|
| 0.0.1  | 09/2025 | Initial version |
|        |         |                |
|        |         |                |
