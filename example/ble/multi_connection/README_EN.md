# BLE Multi-Connection Example

Source path: example/ble/multi_connection

## Supported Platforms
All platforms

## Overview
This example demonstrates BLE multi-connection on this platform based on the GAP central, GAP peripheral, and GATT server roles.

## Usage
1. When the board acts as a peripheral, it automatically starts advertising at boot and can be connected from a mobile BLE app.
2. After a connection is established, the board automatically starts advertising again so that other phones can also connect.
3. The board can also act as a central device and use Finsh commands to scan for and connect to other peripheral devices. The steps are shown below.

#### Step 1: Scan for nearby devices
Enter the following command in the serial terminal to start scanning nearby BLE devices:
```
diss scan start 0 100 30 5000 -90
```

Parameter description, in order:

| Parameter | Example | Description |
|:----------|:--------|:------------|
| dup | 0 | Duplicate filtering policy, 0 = do not filter duplicate advertisements, 1 = filter duplicate advertisements |
| interval | 100 | Scan interval in ms |
| window | 30 | Scan window in ms |
| duration | 5000 | Scan duration in ms; scanning stops automatically when the time expires |
| rssi | -90 | RSSI threshold in dBm; devices weaker than this value are not displayed |

Scan results are printed in the serial terminal. Each device shows its index, address type, MAC address, advertising type, and RSSI.

To stop scanning early, enter:
```
diss scan stop
```

#### Step 2: Start a connection
Start a connection using the device index shown in the scan results. For example, if the target device index is 4, use:
```
diss conn_idx start 4 0 5000 30 100 30
```

![](./assets/image1.png)

Parameter description, in order:

| Parameter | Example | Description |
|:----------|:--------|:------------|
| idx | 1 | Device index in the scan results, starting from 1 |
| own_addr_type | 0 | Local address type, 0 = Public/Random Static, 1 = Resolvable |
| super_timeout | 5000 | Connection supervision timeout in ms |
| conn_itv | 30 | Connection interval in ms |
| scan_itv | 100 | Connection scan interval in ms |
| scan_wd | 30 | Connection scan window in ms |

After the connection succeeds, the peer device also prints connection information:

![](./assets/image2.png)

#### Step 3: Operations after connection
After the connection is established, you can use the following commands:

**View the connected device list:**
```
diss show_dev
```

![](./assets/image3.png)

`dev_idx` is the index number of a connected device. After running `diss show_dev`, the currently connected devices are listed with their indexes, usually starting from 0. This index is the `dev_idx` used by later commands to select which connected device to operate on.

**Search remote GATT services:**
```
diss search_svc <dev_idx> <uuid_len> <uuid_hex>
```

For example, to search for the service with 16-bit UUID 0x180A:
```
diss search_svc 0 2 0A18
```

**Show the details of the discovered services:**
```
diss show_rmt_service <dev_idx>
```

![](./assets/image4.png)

`handle` is the GATT attribute handle. It is required by the commands below.

**Read a remote characteristic value:**
```
diss read_val <dev_idx> <handle>
```

![](./assets/image5.png)

**Write a remote characteristic value:**
```
diss write_val <dev_idx> <handle> <data_len> <data_hex>
```

For example, to write 2 bytes of data to handle 15 on device 0:
```
diss write_val 0 15 2 0100
```

![](./assets/image6.png)

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example. ([Supported platforms](#Platform_peri))
+ A mobile phone.

### menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):
   - Path: Sifli middleware → Bluetooth
   - Enable: Enable bluetooth
     - Macro: `CONFIG_BLUETOOTH`
     - Description: Enables Bluetooth functionality
2. Enable GAP, GATT Client, and BLE connection manager:
   - Path: Sifli middleware → Bluetooth → Bluetooth service → BLE service
   - Enable: Enable BLE GAP central role
     - Macro: `CONFIG_BLE_GAP_CENTRAL`
     - Description: Enables the BLE CENTRAL role. When enabled, it provides scanning and active connection initiation with peripherals.
   - Enable: Enable BLE GATT client
     - Macro: `CONFIG_BLE_GATT_CLIENT`
     - Description: Enables the GATT CLIENT role. When enabled, it can actively discover services, read and write data, and receive notifications.
   - Enable: Enable BLE connection manager
     - Macro: `CONFIG_BSP_BLE_CONNECTION_MANAGER`
     - Description: Provides BLE connection control management, including multi-connection management, BLE pairing, and link connection parameter updates.
3. Enable NVDS:
   - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
   - Enable: Enable NVDS synchronous
     - Macro: `CONFIG_BSP_BLE_NVDS_SYNC`
     - Description: BLE NVDS synchronization. When Bluetooth is configured for HCPU, BLE NVDS can be accessed synchronously, so enable this option; when Bluetooth is configured for LCPU, this option must be disabled.

### Build and Flash
Switch to the example project/common directory and run the scons command to build:
```c
> scons --board=eh-lb525 -j8
```

Then switch to the example project/common/build_xx directory and run uart_download.bat, then select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```

For detailed build and download steps, please refer to the [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- Describe the runtime results of the example, such as which LEDs will light up and which logs will be printed, so users can tell whether the example is running correctly. The results can be explained step by step together with the code. -->
After the example starts:
1. It can be discovered and connected by multiple mobile phones through BLE apps, allowing corresponding GATT characteristic read/write operations.
2. It can actively connect to other devices.

## Troubleshooting

## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides more detailed guidance. Add web links here if needed, for example, refer to the RT-Thread [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc). -->

## Update History
| Version | Date | Release Notes |
|:--------|:-----|:--------------|
| 0.0.1 | 02/2025 | Initial version |