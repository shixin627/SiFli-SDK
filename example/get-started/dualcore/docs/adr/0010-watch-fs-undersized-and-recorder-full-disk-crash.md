# ADR-0010: Watch NAND filesystem is undersized; recorder crashed on a full disk

- agent: Claude Code (claude-opus-4-7, 1M context)
- model: claude-opus-4-7
- trigger: founder report 2026-05-25 — recorder crashes on record start
  (`Assertion failed at function:dfs_elm_write, line number:636`,
  `fatal error on thread: audio_st`); follow-up: "為什麼 storage 會 full,
  file_browser 看不到足以塞爆 128MB flash 的大檔?"
- status: **Partially accepted** — D1 (crash fix) landed (`02539846a`);
  D2 (FS-undersize root cause) confirmed on hardware; **D3 (how to grow the FS)
  is OPEN — fork between Route A and Route B, needs an eng/founder decision**
- scope: watch firmware only (SiFli SF32LB56W, RT-Thread, NAND + dhara + FatFs).
  No phone/SkaiLink code involved.
- relates: first ADR in the dualcore (watch firmware) repo. Continues the shared
  Skaiwalk ADR numbering (SkaiLink holds 0005–0009; watch code references the
  cross-repo ADR-0008 device registry).

## Context

Tapping **record** crashed the watch. Investigation found two distinct problems,
one stacked on the other:

1. **The crash** — on a full filesystem, the first Opus write panics the system.
2. **Why the disk was full** — `/` is a **5.8 MB** filesystem on an **87.5 MB**
   flash partition. ~82 MB is stranded. The volume fills almost immediately, and
   `file_browser` shows no large files because system data (fonts, models,
   assets) already consumes most of the tiny volume.

## Decisions

### D1 — file writes must fail gracefully, not panic (ACCEPTED, landed `02539846a`)

Root cause of the crash: `dfs_elm_write()` (vendor RT-Thread DFS,
`rtos/rtthread/components/dfs/filesystems/elmfat/dfs_elm.c`) called
`RT_ASSERT(0)` whenever FatFs reported a short write — which is exactly how FatFs
signals a full disk (`f_write` returns `FR_OK` with `bw < btw`). SiFli added this
assert in the FatFs-0.14 upgrade (commit `bdc0f4a56`, 2025-08-17); upstream
RT-Thread simply returns the byte count. The panic fired **inside** the `write()`
syscall on the `audio_st` thread, before the recorder's own
`if (write(...) != len)` handling could run — so the application-level error
handling never got a chance.

**Decision:** restore upstream behaviour (`return byte_write;` / errno) and harden
the recorder so a full disk degrades gracefully:

- `start_voice_recording()` now returns `int`; `mkdir`/`open` failures fail
  gracefully instead of `RT_ASSERT`.
- a pre-flight free-space check rotates out the oldest recordings; aborts if the
  disk is still full.
- the audio thread raises a `rec_disk_full` flag on write failure; the UI timer
  owns teardown (so the file/encoder are released exactly once) and shows
  "storage full".

Files: `dfs_elm.c`, `src/modules/bloc/bloc_v2t.c`, `bloc_v2t.h`,
`src/hcpu/gui_apps/recorder/app_recorder.c`. Verified: PC sim builds, links,
boots clean.

*Caveat:* the oldest-recording rotation is a band-aid for the 5.8 MB volume.
Revisit (relax or remove) once D3 lands and the volume is full-size.

### D2 — the FS is undersized: small FAT volume on a large partition (CONFIRMED)

The partition table provisions `/` at the full region; the on-device FAT does not
use it.

- `customer/boards/sf32lb56w-watch/ptab.json` → `FS_REGION max_size = 0x05780000`
  = **87.5 MiB**, NAND.
- Device `statfs` via the dev-page FS test (added `c17db696a`): **5.8 MB usable,
  region 87.5 MB → UNDERSIZED!** (confirmed on hardware by the founder).
- `mnt_init()` (`src/hcpu/main.c`) mounts an existing FAT as-is and only runs
  `dfs_mkfs` on mount **failure** — so a small FAT, once present, never grows,
  even though `FS_REGION` has been 88–92 MB since 2026-02-10 (git history of
  `ptab.json`).
- Origin of the small FAT: `project/hcpu/jsroot.bat` builds the asset image with
  `mkfatimg_nand ..\jsroot jsroot.bin 4096 2048` → a **4096-sector (8 MB) image**
  flashed to the 87.5 MB region. The FAT's declared volume size is baked in at
  image-creation time and the device honours it.

**Decision:** the FS must be formatted for the full region. *How* is D3.

### D3 — how to grow the FS to the full region (OPEN — fork)

**Constraint (founder, 2026-05-25): flash time must not increase.** The asset
image is flashed to every new watch; we cannot flash ~80 MB of empty space.

Naive "pass more sectors to `mkfatimg`" violates the constraint, and **truncation
does not rescue it**: `mkfatimg_nand` runs FatFs **through dhara** (a
log-structured FTL — the same one on the device) and **never runs garbage
collection** (`//f_gc()` is commented out in the tool's `main.c`). Writing 1911
files churns the FAT/directory sectors; every rewrite appends a new physical page
and leaves the old one as dead journal garbage. Measured on a 44032-sector build:
**61 MB used / 25 MB erased (0xFF) tail** for ~3.4 MB of real content. So neither
FAT-logical truncation nor a trailing-0xFF strip can shrink the image.

Two viable routes, **not yet decided**:

**Route A — device-side mkfs + phone provisioning.** Flash code only (or a tiny
image); `mnt_init` formats `/` to the full region on first boot; fonts/assets/
models are pushed from the phone (the file-sync protocol already delivers
`/model/*.tflite`).
- (+) no large flash; biggest possible volume; no vendor-tool work.
- (−) **boot bootstrap problem**: the CJK font is a file on the FAT
  (`/assets/fonts/tiny55_full.ttf`, opened by FreeType `FT_New_Face`;
  `src/hcpu/resource/fonts/freetype/font_partition_dsc.c`), and the closed-source
  `lvsf_font_inital` asserts if it is missing. A reformat with no font = broken /
  asserting UI until the phone provisions. Needs a **built-in fallback font** plus
  confirmation the phone pushes fonts/assets, not just models.

**Route B — GC + strip in `mkfatimg_nand`.** Make the tool run
`dhara_map_gc_all` (exposed via the `FS_CLEAN_GARBAGE` ioctl,
`tools/mkfatimg/mkfatimg_nand/diskio.c:264`) before writing, then strip the
trailing 0xFF; format the FAT for the full region. The device's dhara resumes
from the compacted ~5 MB prefix and manages the erased tail as free space.
- (+) keeps the current flash flow; flash stays ~5 MB; fonts/assets remain in the
  image, so there is no boot bootstrap problem.
- (−) modify and rebuild the vendored `mkfatimg.exe` (MSVC); must validate on
  hardware that on-device dhara resumes cleanly from a GC'd + stripped image.

## Open questions (resolve before committing to D3)

1. **Flash offset.** `jsroot.bat` flashes at `0x64400000`, but
   `FS_REGION_START_ADDR = 0x64280000` (a 1.5 MB gap). `register_nand_device`
   builds the MTD from `0x64280000`. The current 8 MB image mounts, so either the
   gap is dhara/FTL metadata or the address is effectively translated —
   unresolved by static analysis. It affects the correct full sector count and
   where the image must land. **Action:** capture the boot
   `[mnt_init] enter ... size=0x...` line and the dhara mount offset on hardware.
2. **Route A.** Does the phone push fonts/assets (not just `/model`)? Is there a
   built-in fallback font so the watch boots with an empty `/`?
3. **Route B.** Does on-device dhara `dhara_map_resume` accept a GC'd +
   0xFF-stripped prefix as a full-capacity volume?

## Consequences

- D1 stops the crash and makes recording degrade gracefully on any full disk —
  independent of D3.
- Until D3 lands, the watch still has only 5.8 MB; recordings/logs/assets
  contend, and the D1 rotation deletes oldest recordings to make room.
- D3 (either route) recovers ~80 MB and removes the space pressure; the D1
  rotation can then be relaxed or removed.

## Next step

Open a `/plan-eng-review` session to choose Route A vs Route B against the open
questions, then implement and validate on hardware. The dev-page FS test
(`c17db696a`) is the go/no-go check: after the fix it must read ~64–87 MB
**GROWN**, and Chinese text must still render.

## Evidence (so the next session does not re-derive it)

- FAT volume: crash-dump `df` → `total block 3012 × 2048 = 6,168,576 B`
  = 5.8 MiB usable, 0 free.
- Region: `FS_REGION_SIZE = 0x05780000` = 91,750,400 B = 87.5 MiB
  (`ptab.json` + generated `ptab.h`).
- `mkfatimg <src> <out> <image_size_sectors> <sector_size>`
  (`tools/mkfatimg/mkfatimg_nand/main.c:320`); current `4096 2048` = 8 MB image.
  Reserves 1/4 for dhara GC (`gc_ratio = 4`).
- 44032-sector test build: file 86 MiB, used prefix 61 MiB, 0xFF tail 25 MiB,
  live content 3.4 MB across 1911 files.
- The fatal assert was introduced by SiFli in `bdc0f4a56` (2025-08-17,
  "Upgrade fatfs to 0.14").

## Commits

- `02539846a` fix: survive recording on a full filesystem instead of crashing (D1)
- `c17db696a` feat(developer): show FS capacity vs flash region with
  GROWN/UNDERSIZED verdict (D2 diagnostic tool)
- `eb0cc5455` chore: add `truncate_fatimg.py` (WIP — FAT-logical truncation, the
  wrong approach for a dhara image; superseded by Route B's GC + strip)
