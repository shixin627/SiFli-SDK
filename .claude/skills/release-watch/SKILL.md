---
name: release-watch
description: >
  Trigger the watch firmware release via GitHub Actions on the bench PC
  (self-hosted runner): RELEASE mode + version → Keil build → package →
  flash the watch → 板級篩檢（UART）→ 上傳阿里雲 OSS. Use when the user
  says "發布手錶韌體", "release 1.2.3", "跑發布流程", "出一版", "release
  watch firmware", or wants the release_gui.py flow run from CI instead of
  clicking through the Tk window.
---

# Watch release via GitHub Actions

Headless equivalent of `release_gui.py`. Same code path — the workflow only
calls [`release_ci.py`](../../../example/get-started/dualcore/project/hcpu/release_ci.py),
which imports `release_gui` and sequences its functions.

## Trigger

Ask the user for the version (required). Everything else has a default; only
ask if they mention it. Then:

```bash
gh workflow run watch-release.yml --repo shixin627/SiFli-SDK -f version=1.2.3
```

Inputs: `version`, `board` (`sf32lb56-watch` 量產 / `sf32lb56w-watch` 開發機),
`notes`, `with_watchface`, `flash_port` (燒錄，default COM4, **empty = build
only, no flashing**), `hcpu_port` (下指令/篩檢，default COM3), `skip_hwtest`,
`upload`.

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
- **No BLE test, no MAC 配號.** A release verifies firmware; re-provisioning
  would burn a new BLE MAC on a watch that already has one. Production 配號
  and the RSSI/GATT test stay in `release_gui.py`.

## The bench runner

`C:\actions-runner-sifli` — runner `skaiwalk-watch-bench`, labels
`self-hosted, Windows, X64, watch-bench`. (`C:\actions-runner` is a separate
runner bound to `shixin627/SkaiLink`; one runner serves one repo.)

Not installed as a Windows service — start it and leave the window open:

```bash
cd /c/actions-runner-sifli && ./run.cmd
```

If the runner shows `offline` in the Actions settings page, that window is not
running — a dispatched workflow will just queue forever.

`workflow_dispatch` only appears once `watch-release.yml` is on the default
branch, so commit and push it before the first trigger.
