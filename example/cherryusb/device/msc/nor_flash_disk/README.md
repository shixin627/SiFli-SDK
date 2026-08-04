# USB 大容量存储设备示例（NOR FLASH）

源码路径：example/cherryusb/device/msc/nor_flash_disk

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd_n16r8

## 概述
<!-- 例程简介 -->
本例程演示基于 CherryUSB MSC 将 NOR Flash 的 `FS_REGION` 区域导出为 USB 大容量存储块设备，包含：
+ PC 可识别到一个 USB 大容量存储设备。
+ NOR Flash 区域已有有效 FAT 文件系统时，PC 文件管理器可看到对应盘符。
+ 首次使用或该区域被擦除后，PC 侧可能提示需要格式化；建议格式化为 FAT32 格式，格式化完成后才能正常显示盘符并进行文件读写。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](quick_start)）。
+ 带数据传输功能的USB-A转type-c数据线。
+ 支持usb的主机设备。

### menuconfig配置

本例程默认不在板端启用 DFS/Elm-FAT，也不会预置或自动创建 FAT 文件系统。USB 主机看到的是 NOR Flash 上的原始块设备，文件系统由 PC 侧格式化生成。

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：

> scons --board=sf32lb52-lcd_n16r8 -j32

切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：

 >./uart_download.bat

>Uart Download

>please input the serial port num:5

关于编译、下载的详细步骤，请参考[快速上手](quick_start)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
主机通过数据线连接板子，设备管理器中通用串行总线控制器一项出现新添设备，USB 大容量存储设备。

如果 NOR Flash 的 `FS_REGION` 区域尚未格式化，PC 侧可能不会立即在文件管理器中显示盘符，或会提示需要格式化磁盘。这是正常现象。建议在 PC 侧格式化为 FAT32 格式；完成格式化后，PC 文件管理器中会出现对应盘符，后续重新连接时只要该区域未被擦除或覆盖，就可以直接看到盘符。

## 异常诊断

|现象 |可能原因 |处理方法 |
|:---|:---|:---|
| 设备管理器可见 USB 大容量存储设备，但文件管理器无盘符 | NOR Flash 导出的块设备还没有有效文件系统 | 在 PC 侧将该磁盘格式化为 FAT32 格式 |
| 每次重新上电或重新烧录后都需要格式化 | 下载脚本或擦除流程覆盖了 `FS_REGION` 区域，或 Flash 写回失败 | 确认下载/擦除范围没有破坏 `FS_REGION`，并检查串口日志中的 Flash 读写错误 |

## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |09/2025 |初始版本 |
| | | |
| | | |
