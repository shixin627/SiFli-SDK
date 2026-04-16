---
name: dualcore-ipc
description: >
  Automatically enforces the HCPU/LCPU dual-core data_service IPC pattern in this SiFli SDK project.
  TRIGGER: Any time code needs to pass data, state, or events between HCPU and LCPU —
  including reading LCPU-side sensor/algorithm state from HCPU GUI code,
  or sending HCPU commands to LCPU peripherals/algorithms.
  Signs: #include of an LCPU-only header in HCPU code (or vice versa),
  direct function calls across core boundaries, or user says "pass X between cores".
globs:
  - example/get-started/dualcore/**
---

# Dual-Core IPC: HCPU ↔ LCPU Communication via data_service

## Architecture Overview

This project runs on a **dual-core SoC** (SiFli SF32LB56x). Code is split:

| Core | Role | Key directories |
|------|------|----------------|
| **HCPU** | GUI (LVGL), BLE, app logic | `src/hcpu/`, `src/modules/client/`, `src/modules/bloc/`, `src/modules/model/` |
| **LCPU** | Sensors, algorithms, motor, LED | `src/lcpu/`, `src/modules/service/`, `src/modules/algorithm/` |

**HCPU code CANNOT directly `#include` or call LCPU-side headers/functions** (and vice versa).
All cross-core communication goes through the `data_service` framework (provider/subscriber pattern).

## Critical Files (read these before modifying IPC)

| File | Side | Purpose |
|------|------|---------|
| `src/modules/service/watch_sys_service.h` | Shared | Message IDs, data structs, `watch_sys_sync_t`, `client_msg_t` enums |
| `src/modules/service/watch_system_service.c` | LCPU | Service provider — pushes indications to HCPU, handles HCPU requests |
| `src/modules/client/watch_system_client.c` | HCPU | Service subscriber — receives indications, sends requests to LCPU |

## Existing Message IDs

```
MSG_SERVICE_SYS_DATA_REQ           = 0x30        (HCPU→LCPU requests, opcode in body[0])
MSG_SERVICE_BATTERY_DATA_RSP       = 0x8030      (LCPU→HCPU)
MSG_SERVICE_BATTERY_DATA_IND       = 0x8031
MSG_SERVICE_CHARGE_STATE_IND       = 0x8032
MSG_SERVICE_IMU_STATE_IND          = 0x8033
MSG_SERVICE_MAG_STATE_IND          = 0x8034
MSG_SERVICE_PPG_STATE_IND          = 0x8035
MSG_SERVICE_LIFT_IND               = 0x8036
MSG_SERVICE_SOFT_ADT_IND           = 0x8037      (wear detection)
MSG_SERVICE_GESTURE_IND            = 0x8038
MSG_SERVICE_GESTURE_DATASET_IND    = 0x8039
MSG_SERVICE_GESTURE_PPG_DATASET_IND = 0x803A
MSG_SERVICE_HEALTH_INFO_IND        = 0x803B
MSG_SERVICE_SLEEP_STATE_IND        = 0x803C
MSG_SERVICE_DEBUG_LOG_IND          = 0x803D
MSG_SERVICE_MINUTE_ACTIVITY_IND    = 0x803E
```

Next available offset: **+15** (i.e. `MSG_SERVICE_SYS_DATA_REQ + 15`).

## Existing HCPU→LCPU Request Opcodes (`client_msg_t`)

```
SysStandBy=0, SysWakeUp=1, SysSyncApiLock=2, SysRequestBattery=3,
SysRequestChargeStatus=4, PpgSensorPowerManage=5, UserTapDetected=6,
ImuDataCollection=7, ImuRawdataCollection=8, CalibrateGlobalAttitude=9,
UserProfileUpdate=10, SysRequestPedometerData=11, MotorControl=12,
DebugMode=13, DebugParamUpdate=14, MultiGestureMode=15,
TapAndHoldMode=16, RgbLedControl=17
```

Next available opcode value: **18**.

## How to Access LCPU State from HCPU

**NEVER** `#include` LCPU algorithm headers on HCPU side. Instead, use one of these patterns:

### Pattern A: State already synced via existing IPC

Many LCPU states are already mirrored to HCPU global state. Check first:

| LCPU state | HCPU accessor | Set by |
|------------|---------------|--------|
| Wear detection | `SkaiWatchSys.flag_field.is_wearing` | `MSG_SERVICE_SOFT_ADT_IND` |
| Battery level | `SkaiWatchSys.battery_level_value` | `MSG_SERVICE_BATTERY_DATA_IND` |
| Battery voltage | `SkaiWatchSys.battery_vol_value` | `MSG_SERVICE_BATTERY_DATA_IND` |
| Charge status | `SkaiWatchSys.charger_status` | `MSG_SERVICE_CHARGE_STATE_IND` |
| IMU enabled | `is_imu_enabled()` | `MSG_SERVICE_IMU_STATE_IND` |
| PPG enabled | `is_ppg_enabled()` | `MSG_SERVICE_PPG_STATE_IND` |
| Step count | `SkaiWatchSys.gPedoData.global_steps` | `MSG_SERVICE_HEALTH_INFO_IND` |
| Distance | `SkaiWatchSys.gPedoData.global_distance` | `MSG_SERVICE_HEALTH_INFO_IND` |
| Calories | `SkaiWatchSys.gPedoData.global_calories` | `MSG_SERVICE_HEALTH_INFO_IND` |

**Always check this table first.** If the state is already available, just read the global — no new IPC needed.

### Pattern B: Add new LCPU→HCPU indication (push notification)

Use this when LCPU needs to notify HCPU of a state change or event. Follow these steps **in order**:

#### Step 1: Define message ID in `watch_sys_service.h`

Add after the last existing `MSG_SERVICE_*_IND`:
```c
MSG_SERVICE_YOUR_NEW_IND = ((MSG_SERVICE_SYS_DATA_REQ + <next_offset>) | RSP_MSG_TYPE),
```

#### Step 2: Define data structure in `watch_sys_service.h` (if needed)

For simple bool/uint32 data, reuse `watch_sys_service_data_ind_t`. For complex data:
```c
typedef struct {
    uint32_t field1;
    uint16_t field2;
} watch_sys_your_data_t;
```

**IMPORTANT:** Total struct size must fit in `data_msg_t`. For inline: max ~12 bytes. Larger structs are heap-allocated automatically by the framework.

#### Step 3: Add callback function pointer to `watch_sys_sync_t` in `watch_sys_service.h`

In the `#else` (LCPU) section:
```c
void (*your_new_callback)(your_param_type param);
```

#### Step 4: Implement callback in `watch_system_service.c` (LCPU)

```c
static void your_new_callback(your_param_type param)
{
    if (watch_sys_service_env.service == NULL)
        return;
    watch_sys_service_data_ind_t data_ind;  // or your custom struct
    data_ind.data = (uint32_t)param;
    int32_t result = datas_push_msg_to_client(
        watch_sys_service_env.service,
        MSG_SERVICE_YOUR_NEW_IND,
        sizeof(data_ind), (uint8_t *)&data_ind);
    RT_ASSERT(0 == result);
}
```

#### Step 5: Register callback in `register_watch_sys_service_funs()` (LCPU)

```c
watch_sys_sync.your_new_callback = your_new_callback;
```

#### Step 6: Handle in `watch_sys_service_callback()` in `watch_system_client.c` (HCPU)

```c
case MSG_SERVICE_YOUR_NEW_IND:
{
    watch_sys_service_data_ind_t *data_ind;
    RT_ASSERT(arg->data_len == sizeof(watch_sys_service_data_ind_t));
    data_ind = (watch_sys_service_data_ind_t *)arg->data;
    RT_ASSERT(data_ind);
    // Update global state or trigger UI refresh
    SkaiWatchSys.your_field = data_ind->data;
    break;
}
```

#### Step 7: Call from LCPU algorithm

```c
if (watch_sys_sync.your_new_callback)
    watch_sys_sync.your_new_callback(value);
```

### Pattern C: Add new HCPU→LCPU request (command)

Use this when HCPU needs to tell LCPU to do something.

#### Step 1: Add opcode to `client_msg_t` enum in `watch_sys_service.h`

```c
YourNewCommand,   // = next available value
```

#### Step 2: Add function pointer to `watch_sys_sync_t` HCPU section

```c
int (*your_new_command)(param_type param);
```

#### Step 3: Implement in `watch_system_client.c` (HCPU)

```c
static int your_new_command(param_type param)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err = RT_EOK;
    msg.body[0] = YourNewCommand;
    msg.body[1] = param;  // pack parameters into body[1..N]
    body = data_service_init_msg(&msg, MSG_SERVICE_SYS_DATA_REQ, 0);
    err = send_watch_sys_msg_with_retry(&msg, 50, 1);
    return err;
}
```

#### Step 4: Register in `register_watch_sys_sync_funs()` (HCPU)

```c
watch_sys_sync.your_new_command = your_new_command;
```

#### Step 5: Handle in `watch_sys_service_msg_handler()` (LCPU)

```c
case YourNewCommand:
{
    param_type param = msg->body[1];
    // Execute on LCPU side
    break;
}
```

## Common Mistakes to Avoid

1. **NEVER `#include` LCPU headers in HCPU code** — use `SkaiWatchSys` globals or add new IPC
2. **NEVER call LCPU functions directly from HCPU** — always go through `watch_sys_sync` function pointers
3. **Check if state is already synced** before adding new messages (see Pattern A table)
4. **Keep message offsets sequential** — check the last used offset before adding
5. **Both sides must be modified** — header (shared), service (LCPU), client (HCPU)
6. **Use `datas_push_msg_to_client` on LCPU, `datac_send_msg` on HCPU** — don't mix them up
7. **Guard callbacks with NULL check** — `if (watch_sys_sync.callback) watch_sys_sync.callback(val);`
8. **Forward declarations** — if you reference a function before its definition in C, add a forward declaration near the top of the file

## Reference: Wear Detection as Complete Example

This shows the full flow for `LCPU wear_detect.c` → `HCPU SkaiWatchSys.flag_field.is_wearing`:

```
wear_detect.c (LCPU algorithm)
  → set_status() calls notify_wear_status(bool wearing)
  → watch_sys_sync.soft_adt_status_callback(wearing)

watch_system_service.c (LCPU service)
  → soft_adt_status_callback() packs into watch_sys_service_data_ind_t
  → datas_push_msg_to_client(..., MSG_SERVICE_SOFT_ADT_IND, ...)

[data_service framework cross-core IPC]

watch_system_client.c (HCPU client)
  → watch_sys_service_callback(), case MSG_SERVICE_SOFT_ADT_IND
  → SkaiWatchSys.flag_field.is_wearing = status

HCPU app code reads: SkaiWatchSys.flag_field.is_wearing
```
