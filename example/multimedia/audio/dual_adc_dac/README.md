# Local music示例

源码路径：example/multimedia/audio/dual_adc_dac

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ 在SiFli的模组板上无法运行, 需要用sf32lb58系列芯片做的板子才行，而且要有两个模拟麦。

## 概述
<!-- 例程简介 -->
本例程演示如何同时录制两个模拟麦的声音，如何选择其中一个麦录音：


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### 硬件需求
运行该例程前，需要准备：
+ 一块能接两个模拟麦的使用sf32lb58芯片的板子（[支持的平台](quick_start)）。
+ 喇叭。

### menuconfig配置

1. 本例程需要读写文件，所以需要用到文件系统，配置`FAT`文件系统：
![RT_USING_DFS_ELMFAT](./assets/mc_fat.png)

     ```{tip}
     mnt_init 中mount root分区。
     ```
2. 使能AUDIO CODEC 和 AUDIO PROC：
![AUDIO CODEC & PROC](./assets/mc_audcodec_audprc.png)
3. 使能AUDIO(`AUDIO`)：
![AUDIO](./assets/mc_audio.png)
4. 使能AUDIO MANAGER.(`AUDIO_USING_MANAGER`)
![AUDIO_USING_MANAGER](./assets/mc_audio_manager.png)
5. (`AUDIO_LOCAL_MUSIC`)
![AUDIO_LOCAL_MUSIC](./assets/mc_local_music.png)


### 编译和烧录
因为需要客户自定义的板子，实际上需要用自定义的board编译，切换到例程project目录，运行scons命令执行编译：
```c
> scons --board=sf32lb58-lcd_n16r32n1_a1_dpi_hcpu -j32
```
切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[快速上手](quick_start)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
命令说明：

1. mic2file

10秒后会有两个文件生成，分别是两个麦的录音PCM数据
mic0_16k.pcm
mic1_16k.pcm

2. mic2speaker 0
   用第一个模拟麦采集数据并在喇叭播放 

3. mic2speaker 1
   用第二个模拟麦采集数据并在喇叭播放 


## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |05/2026 |初始版本 |
| | | |
| | | |
