---
name: release-watch
description: >
  Trigger the watch firmware release via GitHub Actions on the bench PC
  (self-hosted runner): RELEASE mode + version → Keil build → package →
  flash the watch → 板級篩檢（UART）→ BLE 配號與連線測試 → 上傳阿里雲 OSS.
  Use when the user
  says "發布手錶韌體", "release 1.2.3", "跑發布流程", "出一版", "release
  watch firmware", or wants the release_gui.py flow run from CI instead of
  clicking through the Tk window.
---

# Watch release via GitHub Actions

Headless equivalent of `release_gui.py`. Same code path — the workflow only
calls [`release_ci.py`](../../../example/get-started/dualcore/project/hcpu/release_ci.py),
which imports `release_gui` and sequences its functions.

## Trigger

Every input has a default, so a plain release needs no arguments at all — the
version comes from OSS:

```bash
gh workflow run watch-release.yml --repo shixin627/SiFli-SDK
```

Inputs: `version` (**留空 = 讀 OSS `skaiwatch/<board>/info.json` 的版號 +1**),
`board` (`sf32lb56-watch` 量產 / `sf32lb56w-watch` 開發機), `notes`,
`with_watchface`, `flash` (預設 true；`-f flash=false` = 只編譯打包),
`flash_port` (default COM4), `hcpu_port` (default COM3), `skip_hwtest`,
`upload`.

**A failed flash wedges the watch, and a wedged watch lies about which port is
which.** After `DownLoadUart fail`, the next attempt fails too — sometimes on
the *other* port, and `EnterDebugMode success` can even appear on the HCPU port.
So never diagnose port order from a failed run: power-cycle the watch first,
then test once. The bench wiring is 燒錄 COM4 / 指令 COM3 (confirmed by
flashing from `release_gui.bat`).

Never express "build only" by blanking `flash_port` — `gh` drops empty `-f`
values and the workflow default (COM4) takes over, i.e. it flashes anyway.
Use `-f flash=false`.

Flashing and OSS upload change hardware / publish externally — **confirm both
with the user before triggering**.

Then follow the run:

```bash
gh run watch --repo shixin627/SiFli-SDK $(gh run list --repo shixin627/SiFli-SDK --workflow=watch-release.yml -L1 --json databaseId -q '.[0].databaseId')
```

Failures: read the step log with `gh run view --log-failed`. Build errors are
also in `_watch_build.log`, flashing errors in
`build_<board>_hcpu\ImgBurn.log` — both are attached to the run as artifacts.

## What it does not do

- **No `actions/checkout`.** The job builds `C:\skaiwalk\SiFli-SDK` as it sits
  on disk, uncommitted changes included, because the Keil toolchain, the watch
  fixture and `oss_credentials.json` all live there. The run summary records
  `git rev-parse HEAD` and any dirty files — check it before publishing.
- **No `set_build_mode.py dev` afterwards.** The tree is left in RELEASE mode;
  switch back in the GUI (「切換到開發模式」) before resuming development.
- **No RSSI gate.** The BLE step assigns a new MAC and verifies advertising +
  GATT, but signal strength is not a pass/fail criterion — the threshold is
  pinned to -100 dBm. The calibrated 治具 threshold stays in `release_gui.py`.
- **The BLE step writes a new MAC on every flashed run.** That is intended for
  bench/產線 use; it is not something to run against a watch already in a
  user's hands.

## The bench runner

`C:\actions-runner-sifli` — runner `skaiwalk-watch-bench`, labels
`self-hosted, Windows, X64, watch-bench`. (`C:\actions-runner` is a separate
runner bound to `shixin627/SkaiLink`; one runner serves one repo.)

Installed as a Windows service, start type Automatic, running as
`DESKTOP-QNDMPS1\skaiwalk` — **not** the default NETWORK SERVICE account, which
cannot reach the Keil licence or `C:\dev\env_latest`. Nothing to start by hand,
and it survives a reboot.

```bash
powershell -Command "Get-Service actions.runner.shixin627-SiFli-SDK.skaiwalk-watch-bench"
```

BLE works from this service despite it running in session 0 — verified end to
end (advertising + GATT read), so there is no reason to move it back to a
desktop session. If the runner shows `offline`, the service is stopped; a
dispatched workflow will just queue forever.

`workflow_dispatch` only appears once `watch-release.yml` is on the default
branch, so commit and push it before the first trigger.
