# 双核综合例程
源码路径：example/get-started/dualcore

## 支持的平台
例程可以运行在以下开发板
* sf32lb52-lcd_n16r8
* eh-lb561
* ec-lb587
* eh-lb523

## 概述
本例程演示如何使用双核开发应用，项目已支持蓝牙和低功耗，基于LVGL v8构建用户界面（参考了`multimedia/lvgl/watch`），大小核运行各自的程序（SF32LB52系列芯片小核为蓝牙专用，不运行用户程序）。

> 注：`SF32LB55X` 系列暂不支持 `3D旋转(rotation3d)` 演示，因此在 55x 平台上对应蜂窝入口无法进入画面。

## 目录结构
```
.
└── hmi_demo
    ├── project
    │   ├── hcpu    // HCPU 工程目录
    │   └── lcpu    // LCPU 工程目录
    └── src
        ├── hcpu    // HCPU 代码目录
        └── lcpu    // LCPU 代码目录
```        

## 例程的使用
### 编译和烧录

切换到`project/hcpu`目录，运行scons命令编译：
```
scons --board=sf32lb52-lcd_n16r8 -j32
```

运行`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`，按提示选择端口即可进行下载：


在HCPU工程目录下执行`scons`命令会自动编译LCPU的工程，下载脚本会下载包括小核在内的所有的固件。

### 图片/字库资源（jsroot）

资源来源目录 `project/jsroot`（图片 + 字库），打包成 FAT 映像后单独刷入。

**关键参数：资源分区起始位址 = `0x64280000`**（= `FS_REGION_START_ADDR`，见 `ptab.json`）。
所有刷 jsroot 的脚本都必须用这个位址；旧文档里的 `0x64400000` 是错的。

产生映像（ADR-0010 Route B：GC + strip，约 5.5 MB，FAT 格式化为整个 87.5 MB 分区）：

```
project\hcpu\jsroot_pack.bat        REM → project\hcpu\build\jsroot_packed.bin
```

刷入方式二选一：

1. **UART（推荐）**：跑 `project\hcpu\release_gui.py`，按「刷入圖片資源」。
   内部用 `tools\uart_download\ImgDownUart.exe` 把 `build\jsroot_packed.bin` 烧到 `0x64280000`。
2. **JLink**：`jlink.exe -device SF32LB56X_NAND -if SWD -speed 10000 -autoconnect 1 -CommandFile tools\mkfatimg\mkfatimg_nand\_pack_flash.jlink`

`project\hcpu\jsroot.bat` 是旧的 8 MB 流程（产生 `jsroot.bin` 并直接用 JLink 烧录），已被 `jsroot_pack.bat` 取代。

### 内存/分区配置（ptab.json）

分区表：`customer/boards/sf32lb56-watch/ptab.json`。两块内存各有一个 `main` 区，
必须同时改，改一个不改另一个会在链接期或启动期出问题。

| mem | region | offset | max_size | 用途 |
| --- | --- | --- | --- | --- |
| psram1 | `main` | 0x00000000 | 0x00280000 | 代码执行区 |
| psram1 | (PSRAM_DATA) | 0x00280000 | 0x00580000 | 图片缓存等运行期数据 |
| flash3 | `main` | 0x00000000 | 0x00280000 | HCPU_FLASH_CODE_LOAD_REGION |
| flash3 | (FS_REGION) | 0x00280000 | 0x05780000 | 文件系统 + jsroot 资源 |

代码区上限 `0x280000` = 2,621,440 byte。参考占用：

| 固件 | main.bin | 余量 |
| --- | --- | --- |
| 基线 | 2,389,600 | 231,840（8.8%）|
| 合并上游后 | 2,509,472 | 111,968（4.3%）|
| 合并后 + `RT_USING_FINSH` | 2,603,120 | 18,320（0.7%）|

链接超出时的报错是 `Error: L6406E: No space in execution regions`，不会指出是分区不够，
只会列一堆放不下的 symbol，容易误判成代码问题。

**扩充步骤**（以 0x280000 → 0x300000，即 +512 KB 为例）：

1. `ptab.json`：psram1 `main` 与 flash3 `main` 的 `max_size` 同时改成 `0x00300000`。
2. `ptab.json`：PSRAM_DATA 的 `offset` 改 `0x00300000`、`max_size` 减 512 KB（`0x00500000`）；
   FS_REGION 的 `offset` 改 `0x00300000`、`max_size` 减 512 KB（`0x05700000`）。
3. FS_REGION 起始位址跟着变，**jsroot 的刷入位址必须同步改**（`0x64280000` → `0x64300000`），
   共三处硬编码：`project/hcpu/jsroot.bat`、`project/hcpu/jsroot_pack.bat`、
   `project/hcpu/release_gui.py` 的 `JSROOT_FLASH_ADDRESS`。
4. 改完必须 **clean build**（删掉 `build_<board>_hcpu` 目录），分区改动不会触发增量重建。
5. 分区一变，设备上原有的文件系统与 jsroot 都会错位，**必须重刷 jsroot**。

代价是 PSRAM 运行期数据少 512 KB，与 `CONFIG_IMAGE_CACHE_IN_PSRAM_SIZE` 抢空间，改完要回归图片相关功能。

### 烧录

下载脚本会按提示要 COM 口编号，可以直接用管道喂进去，不需要人工输入：

```
echo <port> | build_sf32lb56-watch_hcpu\uart_download.bat
```

`<port>` 只填数字（COM4 就填 `4`）。在 PowerShell 里跑要写成
`echo 4 | cmd /c ".\uart_download.bat"`，少了 `.\` 会报「不是内部或外部命令」。

**端口分两个，别接错**：一个是固件日志口（HCPU console，`uart1`），另一个是 boot ROM
下载口，`uart_download.bat` 用的是后者。两个口在不同机器上编号不同，插上后用
设备管理器或 `python -c "import serial.tools.list_ports as l;[print(p.device,p.description) for p in l.comports()]"` 确认。

第一次下载偶尔会卡在 `EnterDebugMode` 或 `WriteMem` 失败，立即重跑一次通常就过。
文件系统（含 jsroot）不会被这个脚本擦掉。

### 调试用固件（带 shell）

发布配置没有串口 shell，`uart1` 上只有 Warning 以上的日志，平时是安静的——
静默不代表接线有问题。要用串口下命令，改 `project/hcpu/proj.conf`：

```
CONFIG_RT_USING_FINSH=y
# CONFIG_BSP_USING_VIRTUAL_CONSOLE is not set
```

虚拟控制台会占住 `uart1`，必须一起关掉。这样能拿到 `msh />`，可用 `app_run <id>` /
`app_exit <id>` / `list_app` 驱动应用切换（共 435 条内建命令）。

`project/hcpu/set_build_mode.py dev` 是完整的 DEV 档，但它同时会改
`CUSTOMER_BOARD_VER`（BOARD_VER_29 → 28）与 LCPU 的传感器电源脚位，板型不符会开不了机；
只想要 shell 的话按上面两行手改，不要动硬件值。另外完整 DEV 档还会关掉
`ULOG_OUTPUT_LVL_W`（保留所有 D/I 字符串），在当前占用下会直接超出代码区。

### 代码解析
#### 编译脚本
由于用到了小核，需要在`project/hcpu/SConstruct`增加如下代码将小核工程编译进来，对于`SF32LB52X`，由于小核为蓝牙专用，不能使用自定义工程，所以直接使用命令`AddLCPU`添加小核的公共工程，
其他芯片系列则调用添加子工程的命令`AddChildProj`，将`lcpu`目录下的工程作为子工程加入编译。

```python
# Add LCPU project
if not GetDepend('SOC_SF32LB52X'):
    lcpu_proj_path = '../lcpu'   
    lcpu_proj_name = 'lcpu'
    AddChildProj(lcpu_proj_name, lcpu_proj_path, True, core="LCPU")
else:
    # use common LCPU project
    AddLCPU(SIFLI_SDK, rtconfig.CHIP)
```

#### 大核

大核的`main`函数在`src/hcpu/main.c`中，完成蓝牙初始化
```c
int main(void)
{
    int count = 0;

    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();
    env->time_handle  = rt_timer_create("app", app_timeout_handler,  NULL,
                                        rt_tick_from_millisecond(BLE_APP_TIMEOUT_INTERVAL), RT_TIMER_FLAG_SOFT_TIMER);
#ifdef SF32LB52X
    env->rc10k_time_handle  = rt_timer_create("rc10", rc10k_timeout_handler,  NULL,
                              rt_tick_from_millisecond(15 * 1000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER); // 15s
    rt_timer_start(env->rc10k_time_handle);
#endif

    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;
            env->conn_para.mtu = 23; /* Default value. */
            ble_app_service_init();
            /* First enable connectable adv then enable non-connectable. */
            ble_app_advertising_start();
            LOG_I("receive BLE power on!\r\n");
        }
    }
    return RT_EOK;
}
```

图形的初始化由`src/hcpu/gui_apps/watch_demo.c`中的`app_watch_init`触发，他被注册为`APP`级别的[自动初始化函数](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/basic/basic?id=rt-thread-%e8%87%aa%e5%8a%a8%e5%88%9d%e5%a7%8b%e5%8c%96%e6%9c%ba%e5%88%b6)，执行时间早于`main.c`中的`main`函数，在`app_watch_init`中创建了`app_watch`线程，图形的初始化实际是在`app_watch`线程完成。

```c
INIT_APP_EXPORT(app_watch_init);
```

#### 小核
小核的`main`函数在`src/lcpu/main.c`中，如下

```c
int main(void)
{
    if (HAL_LXT_DISABLED())
    {
        rc10k_time_handle  = rt_timer_create("rc10", rc10k_timeout_handler,  NULL,
                                            rt_tick_from_millisecond(15 * 1000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER); // 15s
        RT_ASSERT(rc10k_time_handle);
        rt_timer_start(rc10k_time_handle);
    }

    while (1)
    {
        rt_kprintf("main loop\n");
        rt_thread_mdelay(3000);
    }
    return RT_EOK;
}
```

#### 按键
如果按键Key1接在小核的管脚上，则由小核处理按键事件，再通过data service将按键事件转发给大核。
小核的按键初始化代码在`src/lcpu/main.c`的`init_pin`函数中，同时小核工程配置`proj.conf`使能了`CONFIG_BUTTON_SERVICE_ENABLED`，也就是注册了按键服务，大核会订阅该服务以接收按键事件。

````{note}
由于大部分板子小核的配置文件（即板子lcpu目录下的`board.conf`里没有定义按键的管脚，因此在lcpu工程目录下针对每一款支持的板子，都添加一个`proj.conf`用于定义按键的管脚编号，例如在`project/lcpu/eh-lb561_lcpu`目录下的`proj.conf`有如下的KEY1的配置参数，该配置针对eh-lb561这块开发板有效
```c
CONFIG_BSP_USING_KEY1=y
CONFIG_BSP_KEY1_PIN=128
CONFIG_BSP_KEY1_ACTIVE_HIGH=y
```

````

大核的按键初始化函数是`src/hcpu/gui_apps/watch_demo.c`中的`init_pin`，可以看到只有在按键对应的GPIO管脚为PA的管脚时，才会在大核程序中初始化按键，否则就只是大约按键的数据服务。
```c
#if (SLEEP_CTRL_PIN < GPIO1_PIN_NUM)
    button_cfg_t cfg;
#if defined(BSP_USING_PM) && !defined(SF32LB52X)
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;
#endif /* BSP_USING_PM && !SF32LB52X */
    
    cfg.pin = SLEEP_CTRL_PIN;
    cfg.active_state = BUTTON_ACTIVE_POL;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));
    key1_button_handle = id;

#endif /* SLEEP_CTRL_PIN < GPIO1_PIN_NUM */

    button_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != button_handle);
    datac_subscribe(button_handle, "btn0", button_service_callback, SLEEP_CTRL_PIN);    
```



对于SF32LB52系列的芯片，小核为蓝牙专用核，不运行用户程序，按键接在了大核的管脚上，由大核直接处理GPIO中断，但仍旧可以通过订阅按键服务的方式统一处理按键事件，这时按键服务也位于大核。

```{note}
SF32LB52系列需要使能`DATA_SVC_MBOX_THREAD_DISABLED`关闭跨核服务
```

## 例程的预期结果
上电后出现蜂窝界面，点击时钟图标可以打开表盘，如果10秒没有操作界面或者按键，会自动灭屏，此时大核睡眠进入低功耗模式，按键(Key1)唤醒大核并亮屏。

## 异常诊断

  
## 参考文档
