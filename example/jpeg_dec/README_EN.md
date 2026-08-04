# JPEG Hardware Decoding Example

Source code path: example/jpeg_dec


## Supported Platforms
<!-- Which chips are supported -->
+ sf32lb58x

> This example relies on the chip's on-die hardware JPEG decoder; only chips with this module can run it. Other series (LB52x / LB55x / LB56x) are not supported.


## Overview
<!-- Example introduction -->
This example demonstrates using the on-chip hardware JPEG decoder to decode an embedded 100×100 JPEG image (`src/100x100_jpeg.dat`) into RGB565 and draw it to the top-left corner of the LCD.


## Example Usage

After boot, the example performs:

1. `jpeg_decode_init()` — initialize the hardware JPEG decoder
2. `jpeg_decode_get_dimension()` — parse JPEG header, get image width/height
3. Allocate a `width * height * 2`-byte output buffer, call `jpeg_decode2()` to decode into RGB565
4. Use RT-Thread graphics device API (`rt_device_find("lcd")` + `set_window` + `draw_rect`) to draw the decoded image to the LCD
5. `jpeg_decode_deinit()` — release the hardware
6. Main thread enters a loop and prints `__main loop__` every 5 seconds


### Hardware Requirements
+ One development board supported by this example (see [Supported Platforms](#supported-platforms)), with an LCD


### menuconfig Configuration

The default `proj.conf` already enables the hardware JPEG decoder:

```
CONFIG_USING_JPEG_DEC=y
```

To reconfigure via menuconfig:

```bash
scons --board=<board_name> --menuconfig
```

Path: `External libraries → Enable Jpeg Accelerator Decoder`


### Compilation and Flashing
Switch to the example's `project` directory and run scons:

```c
> scons --board=<board_name> -j10
```

For example, using an LB58 series board:

```c
> scons --board=ec-lb587 -j10
```

Switch to `project/build_<board_name>` and run `uart_download.bat`; pick the serial port as prompted:

```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```

For detailed compile and flash steps, refer to [Quick Start](/quickstart/get-started.md).


## Expected Results
<!-- Describe the example execution results -->
After boot, the serial console prints LCD info, the decoded image dimensions, and a `__main loop__` heartbeat every 5 seconds:

```
Lcd info w:..., h:..., bits_per_pixel:..., draw_align:...
w=100, h=100
__main loop__
__main loop__
...
```

The LCD shows the decoded 100×100 RGB565 image at the top-left (0, 0)~(99, 99).

On decode failure the console prints:

| Output | Cause |
|---|---|
| `get dimension fail, err=<code>` | JPEG header parsing failed; input data format issue |
| `decode fail, err=<code>` | Decode error (corrupt data, alignment, etc.) |
| `Lcd open error!` | LCD device not registered or driver not loaded |


## Exception Diagnosis

- No `w=100, h=100` printed → verify `CONFIG_USING_JPEG_DEC=y` is effective and the target chip has the hardware JPEG decoder
- LCD blank or shows garbage → check board-level LCD configuration in `customer/boards/<board>/board.conf`
- Linker error on `jpeg_decode_init` etc. → confirm `external/jpeg_codec/` is included in the build

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.


## Reference Documentation
- JPEG decode API header: `external/jpeg_codec/include/jpeg_dec.h`
- HAL JPEG driver: `drivers/hal/bf0_hal_jpegd.c`


## Update History
| Version | Date | Release Notes |
|:---|:---|:---|
| 0.0.1 | 05/2026 | Initial version: fleshed out README with usage details |
| | | |
