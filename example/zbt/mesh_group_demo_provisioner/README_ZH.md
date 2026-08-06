# zbt mesh_provisioner示例

源码路径：example/zbt/mesh_group_demo_provisioner

## 支持的平台
<!-- 支持哪些平台 -->
全平台

## 概述
本示例演示了如何使用与配网及配置数据库（CDB）相关的蓝牙Mesh API。它需要与 `example/zbt/mesh_group_demo` 示例，用于配网和控制mesh_group_demo中的一个或多个节点。

该应用会自行完成配网，并在CDB中加载应用密钥，然后等待接收来自设备的未配网信标。如果开发板具有通过GPIO连接的按键，那么应用将等待用户按下按键，从而触发使用PB-ADV协议的配网流程。如果开发板没有此按键，使用'mesh_btn 1'命令对检测到的设备进行配网。配网完成后，节点会出现在CDB中，但尚未标记为"已配置"。应用会检测到这个未配置的节点并开始对其进行配置。如果没有遇到错误，节点将被标记为已配置。

节点的配置过程包括：添加应用密钥、获取其组成数据，以及将其所有模型绑定到该应用密钥。

## 例程的使用
见`example/zbt/mesh_group_demo/README_ZH.md`

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板([支持的平台](#Platform_peri))。
+ 另一块或多块运行mesh_group_demo工程的开发板。

### menuconfig配置
1. 使能蓝牙(`BLUETOOTH`)：
    - 路径：Sifli middleware → Bluetooth
    - 开启：Enable bluetooth
        - 宏开关：`CONFIG_BLUETOOTH`
        - 作用：使能蓝牙功能
    - 开启：Zephyr Bluetooth
        - 宏开关：`CONFIG_ZBT`
        - 作用：使能zephyr蓝牙协议栈
    - 开启：disable sifli bt lib
        - 宏开关：`CONFIG_DISABLE_SF_BT_LIB`
        - 作用：禁用sifli蓝牙库
2. 使能MESH：
    - 路径：Sifli middleware → Bluetooth → Bluetooth → Zephyr Bluetooth → Bluetooth Host → Bluetooth Mesh support
        - 使用工程默认已开启的配置即可
3. 使能NVDS：
    - 路径：Sifli middleware → Bluetooth → Bluetooth service → Common service
    - 开启：Enable NVDS synchronous
        - 宏开关：`CONFIG_BSP_BLE_NVDS_SYNC`
        - 作用：蓝牙NVDS同步。当蓝牙被配置到HCPU时，BLE NVDS可以同步访问，打开该选项；蓝牙被配置到LCPU时，需要关闭该选项。

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```c
> scons --board=eh-lb525 -j8
```
切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
1. 可以扫描到未配网设备的信标并可以配网。
2. 配网后的设备可以发送并接受同一网络的消息。

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
| 版本  | 日期    | 发布说明 |
| :---- | :------ | :------- |
| 0.0.1 | 04/2026 | 初始版本 |
|       |         |          |
|       |         |          |