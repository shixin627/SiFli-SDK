# JPEG 硬件解码示例程序

源码路径：example/jpeg_dec


## 支持的平台
<!-- 支持哪些芯片 -->
+ sf32lb58x

> 本例程依赖芯片内置 JPEG 硬件解码器，仅在带该模块的芯片上可用；其他系列（LB52x/LB55x/LB56x）暂不支持。


## 概述
<!-- 例程简介 -->
本例程演示用芯片硬件 JPEG 解码器把内嵌的 100×100 JPEG 图片（`src/100x100_jpeg.dat`）解码成 RGB565 格式，并通过 LCD 设备绘制到屏幕左上角。


## 例程的使用

例程开机后流程如下：

1. 调用 `jpeg_decode_init()` 初始化硬件 JPEG 解码器
2. 调用 `jpeg_decode_get_dimension()` 解析 JPEG 头，得到图片宽高
3. 申请 `width * height * 2` 字节的输出缓冲，调用 `jpeg_decode2()` 解码为 RGB565
4. 通过 RT-Thread 图形设备 API（`rt_device_find("lcd")` + `set_window` + `draw_rect`）把解码结果绘制到 LCD
5. 调用 `jpeg_decode_deinit()` 释放硬件资源
6. 主线程进入循环，每 5 秒打印一次 `__main loop__`


### 硬件需求
+ 一块本例程支持的开发板（[支持的平台](#支持的平台)），且开发板带 LCD


### menuconfig 配置

例程默认 `proj.conf` 已开启硬件 JPEG 解码器：

```
CONFIG_USING_JPEG_DEC=y
```

如需通过 menuconfig 重新配置：

```bash
scons --board=<board_name> --menuconfig
```

对应路径：`External libraries → Enable Jpeg Accelerator Decoder`


### 编译和烧录
切换到例程 `project` 目录，运行 scons 命令编译：

```c
> scons --board=<board_name> -j10
```

例如使用 LB58 系列开发板：

```c
> scons --board=ec-lb587 -j10
```

切换到例程 `project/build_<board_name>` 目录，运行 `uart_download.bat`，按提示选择端口下载：

```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```

关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。


## 例程的预期结果
<!-- 说明例程运行结果 -->
启动后串口会依次输出 LCD 信息、解码出的图片尺寸，随后每 5 秒一行 main loop 心跳：

```
Lcd info w:..., h:..., bits_per_pixel:..., draw_align:...
w=100, h=100
__main loop__
__main loop__
...
```

LCD 屏幕左上角 (0, 0)~(99, 99) 区域会显示解码后的 100×100 RGB565 图像。

如果解码失败，串口会打印对应错误：

| 现象 | 原因 |
|---|---|
| `get dimension fail, err=<错误码>` | JPEG 头解析失败，输入数据格式有问题 |
| `decode fail, err=<错误码>` | 解码过程出错（数据损坏、内存对齐等） |
| `Lcd open error!` | 板级未注册 lcd 设备或 LCD 驱动未加载 |


## 异常诊断

- 串口看不到 `w=100, h=100` → 检查 `CONFIG_USING_JPEG_DEC=y` 是否生效；确认所选芯片带 JPEG 硬件解码器
- LCD 不亮或显示异常 → 检查板级 LCD 配置是否正确加载，可在 `customer/boards/<board>/board.conf` 中确认
- 编译报 `jpeg_decode_init` 之类未定义符号 → 确认 `external/jpeg_codec/` 已编入工程

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。


## 参考文档
- JPEG 解码 API 头：`external/jpeg_codec/include/jpeg_dec.h`
- HAL JPEG 驱动：`drivers/hal/bf0_hal_jpegd.c`


## 更新记录
| 版本 | 日期 | 发布说明 |
|:---|:---|:---|
| 0.0.1 | 05/2026 | 初始版本：完善 README，补全用法说明 |
| | | |
