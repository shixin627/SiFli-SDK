# Pre-fusion IMU measurement v1

Measurement only: no bias subtraction, recognition threshold change, training, model update or automatic upload. Android and iOS expose one extra toggle in the existing IMU recording card. Off preserves the frozen linear-ACC CSV contract. On explicitly negotiates measurement support and waits for a real frame before enabling Record. The chart then displays fusion-input ACC, including gravity; a readout shows GYRO. Labels clearly distinguish requesting, accepted/waiting, ready, recording, stopped, unsupported and interrupted states.

Companion: `shixin627/SiFli-SDK`, branch `feat/raw-imu-measurement`, based on main `d9dd3193a9de3bc7656ae9619f8817d238ab2269`. Install the matched HCPU and LCPU binaries only after separate approval. The mobile app safely times out with unsupported firmware; neither version strings nor legacy 0x50 samples imply support.

## BLE contract

Use the existing BWPS L2 transport/fragmenter. Existing factory open command 0x06/0x01 and linear notify 0x04/0x50 retain their semantics. New factory key 0x06/0x02 carries exactly 8 bytes: `49 4d 01 mode session_u32_le`, where mode=0 stops and mode=1 starts/renews a nonzero session. New notify key 0x04/0x51 has an 8-byte header `49 4d 01 kind session_u32_le`. kind=0 status adds mode byte and zero result byte (10 bytes total). kind=1 adds 1..5 fixed 76-byte samples. Unknown versions, malformed sizes, nonfinite floats, wrong source flags/rates and stale session IDs are rejected. An accepted status alone is not data success.

| Sample offset | Field / units |
|---|---|
| 0 | source sequence u32 LE, modulo 2^32 |
| 4 | LCPU monotonic milliseconds u32 LE, modulo 2^32; not UTC or hardware sensor time |
| 8 | sample rate u16 LE, currently 100 Hz |
| 10 | source flags u16 LE, currently 1=driver-remapped fusion inputs |
| 12..23 | ACC xyz float32 LE, m/s², includes gravity |
| 24..35 | GYRO xyz float32 LE, degrees/s |
| 36..47 | computed linear ACC xyz float32 LE, m/s² |
| 48..59 | computed gravity direction xyz float32 LE, dimensionless |
| 60..75 | quaternion wxyz float32 LE, dimensionless |

These are the actual ACC/GYRO values copied at `handle_imu_data` entry, before Mahony, alongside that same frame's computed results. They are not untouched BMI270 register values: Bosch remapping and gyro cross-axis compensation, board/runtime axis redirection and current hardware offset state already apply. ACC uses +/-4g, 8192LSB/g and 9.80665m/s²; GYRO uses +/-2000°/s and 16.384LSB/(°/s). No new calibration is applied. Hardware offset-register values and the 24-bit BMI270 hardware clock are not captured by v1.

Sequence advances in LCPU acquisition/fusion, before the existing shared IPC handoff; missing sequence IDs expose downstream acquisition-to-phone loss/coalescing. This does not identify which transport stage lost them or detect missing initial/final samples. The watch groups 5 samples (388-byte payload); up to 4 buffered tail samples can be discarded at stop. Duplicates/backward frames are rejected from the recording; sequence wrap is supported. Device-time wrap is retained explicitly in CSV for offline unwrapping.

## Lifecycle and local durability

The watch lease lasts 15 seconds, renewed by the phone every 4 seconds, and has a 120-second absolute cap per session. The phone also stops at 120 seconds, fails negotiation after 8 seconds and stops after 5 seconds without data or on disconnect. Reconnect requires a fresh user-requested session, never automatic resumption or mixing. Stop/cancel/card close ends measurement and closes the local partial file. Toggle is locked during recording. The extension never changes sensor range/rate/power or recognition flags; existing factory-test app behavior remains, so there is no new sensor configuration to restore.

Android writes under private `files/imu_measurements`; iOS under Application Support `imu_measurements`. Each received packet is synced to the local file. Raw files use the new `imu_measurement_v1_*` filename and explicit unit-named columns, a firmware/session/source metadata line, phone receipt time separate from device time, and loss/rejection counts. They are not passed to the old training CSV, OSS upload, sensor relay or pressure predictor. Android can copy a closed file to Downloads; iOS offers the existing ShareLink. Copies happen only on user action. Interrupted/partially saved recordings remain locally recoverable by filename even after app restart; in-app history browsing is not added.

## Minimal stationary capture

1. After approved installation of matching watch firmware, connect the phone and expand the existing IMU card. Turn on **Measure pre-fusion ACC / GYRO**.
2. Wait for **Raw measurement confirmed**; if unsupported or no samples, stop and check firmware/mode rather than recording legacy data as raw.
3. Press Record, keep the watch stationary for 10–20 seconds, then Stop. Inspect saved count and missing/rejected indicators.
4. Export/share the closed CSV. Compare ACC norm, gyro baseline, gravity/quaternion and `linear = ACC − gravity×9.80665` using device-time/sequence; record wrist/orientation separately.

No physical device validation has been performed. BLE throughput, battery impact, source cadence, disconnect/lease behavior and normal recognition timing need bench verification before release. A draft PR is not a flashed or production-tested feature.
