# zbt mesh示例

源码路径：example/zbt/mesh_group_demo

## 支持的平台
<!-- 支持哪些平台 -->
全平台

## 概述
本例程演示了蓝牙 Mesh 功能。它包含多个标准的 Mesh 模型，并支持通过广播和 GATT 配网承载（即 PB-ADV 和 PB-GATT）进行配网。

该应用程序还需要一个功能正常的串行控制台，因为这是用于带外配网过程和模拟按键所必需的。

在带有 LED 的开发板上，通用 OnOff 服务器模型提供了通过网络控制板上第一个 LED 的功能。

最多可以有96个从机分为 4 个大组（Group 1-4），每个大组包含 3 行（Row 1-3），每行 8 列

支持断电记忆，开机自动恢复 Mesh 网络，无需重新配网。
## 例程的使用
1. 需要使用mesh_group_demo_provisioner工程进行配网，finsh命令输入mesh_start_target X Y进行配网。其中X为大组号，取值范围1-4；Y为行号，取值范围1-3。每行8列，每列分配一个单播地址，配网时会自动分配一个单播地址给目标设备，当配网数量达到该行上限时，自动停止配网。
2. 在provisioner端，输入mesh_list_nodes可以查看已配网设备和所在位置。
3. provisioner可以输入mesh_send_onoff X Y来控制各个node的LED ON/OFF状态，其中X是组地址，Y取值为0或者1

| 逻辑分组 | 分配的组地址 | 描述说明 | 对应包含的节点数量 |
| :---- | :---- | :---- | :---- |
| **All Nodes** | 0xFFFF | 蓝牙 Mesh 固定全网广播地址 | 96 |
| **Group A (总)** | 0xC001 | 第一大组全部节点的订阅地址 | 24 |
| **Group B (总)** | 0xC002 | 第二大组全部节点的订阅地址 | 24 |
| **Group C (总)** | 0xC003 | 第三大组全部节点的订阅地址 | 24 |
| **Group D (总)** | 0xC004 | 第四大组全部节点的订阅地址 | 24 |
| **Group A \- Row 1** | 0xC111 | 第 A 组第 1 行专有订阅地址 | 8 |
| **Group A \- Row 2** | 0xC112 | 第 A 组第 2 行专有订阅地址 | 8 |
| **...以此类推...** | ... | ... | 8 |
| **Group D \- Row 3** | 0xC143 | 第 D 组第 3 行专有订阅地址 | 8 |

4. node或者provisioner已经配网的情况下，设备重启后会自动恢复mesh网络连接，无需重新配网,。

5. 重置网络：provisioner通过mesh_reset_all_nodes发起全局擦除，向所有已知单播地址发送 Config Node Reset 指令，随后擦除本地 Flash 数据库。如果有节点错过了重置，可以在节点端输入mesh_reset reboot进行重置。

### 硬件需求
运行该例程前，需要准备：
+ 一块或多块本例程支持的开发板([支持的平台](#Platform_peri))。
+ 另一块运行mesh_group_demo_provisioner工程的开发板。

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
1. mesh node启动后，自动进入配网状态。需要mesh provisioner配网，也可以使用手机上的mesh app进行发现和配网
2. mesh provisioner启动，需要使用finsh命令进行配网

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
| 版本  | 日期    | 发布说明 |
| :---- | :------ | :------- |
| 0.0.1 | 04/2026 | 初始版本 |
|       |         |          |
|       |         |          |