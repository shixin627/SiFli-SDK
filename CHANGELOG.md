# SiFli SDK change Log v2.4
## Major Changes
- Update toolchain to support Linux and macOS. Legacy env tool (upgraded to v1.1.2) could still be used on Windows. But it's recommended to use native terminal tool (i.e. PowerShell for Windows) instead of env for development on Windows. Please refer to [install guide](https://docs.sifli.com/projects/sdk/v2.4/sf32lb52x/quickstart/install/script/index.html). New terminal toolchain doesn't support SDK version `<v2.4`. Windows env tool could support all SDK versions.
- Upgrade GCC to GCC 14. Windows env tool contains both GCC 8 and GCC 14. GCC 8 is used by SDK version `<v2.4`. GCC 14 is used by SDK version `>=v2.4`.
- Upgrade Python to Python 3.11. Windows env tool contains both Python 2.7 and Python 3.11. Python 2.7 is used by SDK version `<v2.4`. Python 3.11 is used by SDK version `>=v2.4`
- `middleware/bluetooth` is integrated as submodule. Downloaded zip file doesn't contain the complete code anymore
- Support SF32LB58 A2 chip. SF32LB58 EVB board is configured with A1 chip. SF32LB58-LCD board is configured with A2 chip.


## Change log since v2.3
### Bluetooth
#### Fixed
- Fix sf32lb58 BT sniff disconnect issue
- Reset 56x bt lcpu audio buffer for cvsd current sound
- Fixed some controller bugs
- Fixed BLE write packet wrong order issue
- BT: Increase stack ready wait time from 5s to 8s
- Fixed connection fail with some phones on sf32lb52 

#### Changed
- Optimize 3sco
- Disable sf32lb58 BTC warning assert
- Update RF driver for sf32lb58 and sf32lb52
- Fix userbin nonsignaling test power issue
- rf: Fix 6dBm power issue


#### Added
- Add 587 PTA for coexistence with WIFI
- Add BT RF CW test interface for sf32lb56x and sf32lb58x

### Drivers
#### Fixed
- usb: Fix DP still in PULLUP state after USB is disabled
- lcd: Fixed drv_lcd_fb lock error
- aon: Fix pin mode is not configured correctly for wakeup pin greater than PIN15 on sf32lb52x
- adc: Disable ADC timer trigger by default
- adc: Fixed crash when clock change on sf32lb52x if no adc instance is enabled
- pmu: Fix the problem of inconsistent parameter passing types for the `round` function
- spi: Add 3wire support of SPI master
- pwm: Optimize PWM  pulse percentage 0% and 100% waveform
- pinmux: Fix PBR PULLDOWN set failure on sf32lb56x
- spi: Fix SPI set clk div fail sometimes on sf32lb56x
- gpio: Fix memory overwritten as i is not initialized
- sdhci: Add eMMC low power mode support
- mailbox: Fix HAL_MAILBOX_LockEx timeout doesn't work on LCPU
- usb: Fix misalignment
- lcd: Fix the first frame doesn't appear on LCD sometimes
- lcd: Enable LCDC default layer
- epic: Fixed 'cont_blend_reset' error
- epic: Fixed 'drv_epic_fill' fore layer is not initialized
- audcodec: Fix PLL calibration of sf32lb58
- nand: Fixed using wrong NAND address by mtd device if RT_DFS_ELM_DHARA_ENABLED is not enabled
- aon: Fixed HAL_LPAON_DisableWakeupSrc not work on LCPU of sf32lb55x
- nor: Fix XT25F128F's type and delete duplicate BY25Q128ES in type 2
- wdt: Fix wdt start/stop fail.
- spi_tf: Fix some card cannot be detected in SPI mode
- ld: Fix DPI screen flicker of sf32lb56-lcd board
- nor: Fix sf32lb52 SIP flash read id error
- epic: Fix the drv_epic D-cache cleaning issues.
- epic: Fixed rotation3d abnormal



#### Changed
- dma: Add all DMA request definition
- lcd: Optimize drv_lcd_fb management 2 buffer
- lcd: Add an console cmd to skip LCD draw_core
- epic: Support customized viewpoint in epic-adv-type2
- lcd: Fixed DCX output error while LCDC reading SPI data
- epic: Update drv_epic statistics
- lcd: Move Dcache-clean out of HAL_LCDC
- pinmux: Unify pad numbering of hpsys and lpsys pad and no need to specify core in API `HAL_PIN_Set`
- audprc Add dual MIC support 
- lcd: Change DPI RGB565 output pins
- lcd: Use EXTDMA in preference to AES when they are both available
- adc: Add VBAT channel in multi-channel sampling mode for sf32lb52


#### Added
- mtd: Add mtd_dhara device to make fatfs support both NAND and NOR at run time
- epic: Add render_list support
- epic: Optimize EPIC filling translucent color

### Middleware
#### Fixed
- audio: start rx after tx interrupt immediately
- flashdb: Fixed fwrite in libc file mode
- flashdb: Add flash4 support
- agif: Fixed file mode bug
- lvgl: Fix mask_cf is not set
- sys: Fixed sifli_memset write data error
- audio: Fix wav parse error : wav_read_header return -1
- audio: Fix mp3 deocde error.
- ffmpeg: Fixed network error code
- lvgl(v8): Invalidate whole screen if using 2 PSRAM screen sized buffer
- lvgl: Fixed 24bit LVGL simulator screen messed up
- audio: Fix ffmpeg play mp3 file crash
- lvgl: Fixed 16bit LVGL simulator black screen
- lvgl: Fixed LV_LV_USE_L8_GPU not defined
- audio: Fix audio 3a open/close policy
- audio: Close audio 3a after client closed
- audio: Fix some audio server bugs
- lvgl: Fix LVGL 9 simulator wrong screen size
- media: Fix media player eixt memory leak

#### Changed
- audio: Fix simulator compile issue
- acpu_ctrl: Add new API acpu_send_result2 to report error_code
- coremark: Coremark package can be used without rt-thread enabled
- freetype: Open source lv_freetype.c to solve GCC build problem
- freetype: Add tiny_font lib for GCC and support big size font
- dfu: Update for NOR/NAND Flash support
- graphics: Gaussian blur supports A8 format
- audio: Support mp3 seek function
- audio: Make rx/tx device can be selected by user within i2s, pdm and audprc
- audio: Not use msbc for non-bt_voice
- lvgl: TWO_LCD_FRAMEBUFFER mode support partial update
- media: Optimize ezip memory usage
- lvgl: Use HW blending for A1/A2/A4 fonts
- rpmsg_lite: Make rpmsg-lite independent to RT_EVENT

#### Added
- secboot: Add secboot module for bootloader
- audio: Add DRC support
- audio: Add BLE talk
- audio: Add VBE(Virtual Bass Enhancement) support
- mbedtls: Add mbedtls v2.28.1
- audio: Add mix support
- lvgl(v8): Use render_list mechanism to improve performance
- lwip: Add lwip v2.1.2
- audio: Enable ogg and vorbis in ffmpeg
- audio: Add LE Audio device
- graphics: Add AGIF support
- graphics: Add JpegNanoD(JPEG Decoder) for sf32lb58
- graphics: Add JpegNanoE(JPEG Encoder) for sf32lb58
- graphics: Add vglite for sf32lb58
- audio: Support MICBIAS pin is used as power pin


### RTOS
#### Fixed
- Fix saved_stack_pointer if it's not aligned to 8byte

#### Changed
- timer: Increase timer task stack size from 512byte to 1024byte for HCPU
- pm: Use deep sleep mode by default if it is 52x


### Examples
#### Fixed
- `storage/flashdb`: Fixed print error
- `multimedia/lvgl/watch`: Support board `eh-lb551` and `eh-lb555`
- `multimedia/lvgl/watch`: Watch example support GCC and use tiny_full font
- `multimedia/lvgl/watch`: Support GCC toolchain and fixed cube rotation UI abnormal
- `multimedia/lvgl/watch`: Support GCC toolchain and fixed cube rotation UI abnormal
- `multimedia/lvgl/watch`: Fixed rotation3d abnormal
- `pm/coremark`: Fixed whileloop current abnormal on sf32lb52x when built by GCC
- `pm/coremark`: Fixed crash if building using GCC on sf32lb58x board
- boot: Fix 2nd bootloader crash on board 567-evb
- `bt/pan`: Fixed reconnect timer may trigger PAN connection even PAN already connected in `bt/pan` project.
- `pwm/pwm`: Fix pwm duty cycle 100%
- `hal_example`: Fix flash test fail on 583/587 evb
- `rt_device/pdm`: Fix pdm gain configuration error.

#### Changed
- bootloader: sf32lb58x 2nd boot not use rt-thread
- bootloader: Add DFU support
- `bt/music_source`: Support music file saved in NOR Flash filesystem
- `bt/central_and_peripheral`: Optimize project structure
- `multimedia/lvgl/watch`: Add simulator support
- `multimedia/lvgl/watch`: Optimize fps
- `multimedia/lvgl/watch`: Fixed mainmenu layout error on sf32lb55x
- `bt/pan`: Add OTA support


#### Added
- Added ble throughput example  in `ble/throughput`
- Added OTA example for sf32lb56x/sf32lb52x by `ble/peripheral_with_ota`
- Added BLE periodic adv example in `ble/periodic_adv`
- Added BT music sink example in `bt/music_sink`
- Added BT PAN example in `bt/pan`
- Add BT music source example in `bt/music_source`
- Add BT hands-free profile example in `bt/hfp`
- Added HCI over UART example in `bt/HCI_over_uart`
- Added pm example in `rt_device/pm`
- Added audprc example in `rt_device/audprc`
- Add I2S example in `rt_device/i2s`
- Add SPI TF card example in `rt_device/spi_tf`
- Add PDM example in `rt_device/pdm`
- Add RT_DEVICE GPIO example in `rt_device/gpio`
- Add PWM DMA example in `rt_device/pwm/pwm_dma`
- Add eMMC example in `rt_device/emmc`
- Added local music example in `multimedia/audio/local_music`
- Added record example in `multimedia/audio/record`
- Add JPEG decoder example in `jpeg_nanod`
- Add LCD stress test example in `lcd_stress_test`
- Add fatfs example in `storage/fatfs` for NOR and NAND Flash
- Added FlashDB example in `storage/flashdb`
- Add finsh example in `system/finsh`
- Add ulog example in `system/ulog`



### Tools
#### Fixed
- svd: Fix parsing error by some tool
- svd: Add sf32lb55/sf32lb58 svd and update for sf32lb52/sf32lb56
- build: Fixed build error if `.git` folder is present in application folder
- build: Fixed `GenDownloadScript` typo


#### Changed
- build: Support acpu image embedded in hcpu image
- build: Fix get SIFLI SDK build sha path
- Add crash_dump_analyzser install guide
- jlink_drv: Fix all zero data fail to be programmed into SD/eMMC
- jlink_drv: Add 58x emmc/sd1 jlink drv and project
- build: Print hint if board path is not present
- ImgDownUart: Update to support sf32lb58x
- FsrwTool: Update to support upload/download file by UART
- Add Trace32 download link
- bin2bmp: Supports changing the foreground color of Ax
- build: Add `buildlib` argument in `PrepareBuilding` and `board` argument in `PrepareEnv` to make lib build command simple
- build: Move environment construct to `PrepareBuilding` to unify SConstruct for simulator and other board
- jlink_drv: Add PMIC and customized control pin support
- font2c: Upgrade to support multiple freetype fonts register
- uart_download: Add verification step
- build: Add ftab support for sf32lb55 build
- uartServer: Upgrade SifliUsartServer to V2.8
- build: Add board search path to support custom boards



### BSP
#### Fixed
- board: Fix flash5 is not re-initialized by LCPU of board sf32lb58-lcd
- board: Fix wrong sf32lb58 board config causing deepwfi crash
- board: Fix flash1 of 585-evb board is not initialized
- board: Fix ipc_queue example doesn't work on 567-evb as LCPU_RAM_DATA_SIZE is wrong
- board: Update pinmux for eMMC of board sf32lb58-lcd
- board: Fixed PA config of board 567-evb

#### Changed
- board: Rename board `em-lb525` to `sf32lb52-lcd_n16r8`
- board: Change default LCD of sf32lb52-lcd board to LCD_1P85_390*450_DevKit_Adapter_V1.0.0 
- board: Change board `sf32lb52-lcd` touch pin to PA31 to support SF32LB52-DevKit-LCD v1.2 
- board: Change lcd backlight PWM to PWM2 on sf32lb52-lcd board
- board: Rename board `em-lb587` to `sf32lb58-lcd_n16r64n4`
- board: Change board sf32lb56-lcd HCPU default console uart to uart4
- rgbled: Update rgbled driver
- board: Update pc board for simulator build
- board: Optimize LCDC SoftSPI pin IO implementation of 56x-hdk board
- aw8155: move control pin config to kconfig

#### Added
- board: Add dummy board for board independent library build
- sensor: Add LTR303 LSM6DSL MMC56x3 Sensor
- lcd/tp: Added driver for LCD_1P85_390*450_DevKit_Adapter including CO5300 lcd driver and FT6146 touch driver
- board: Add board `sf32lb58-lcd_a128r32n1_dsi`, `sf32lb58-lcd_n16r32n1_dpi`, `sf32lb58-lcd_n16r32n1_dsi`
- board: Add board `sf32lb56-lcd_a128r12n1` and `sf32lb56-lcd_n16r12n1`
- board: Add board `sf32lb52-nano_52b` and `sf32lb52-nano_52j`
- board: Add board `sf32lb52-lchspi-ulp`
- board: Add board `sf32lb52-lcd_52d`
- lcd: Add jdi387a driver
