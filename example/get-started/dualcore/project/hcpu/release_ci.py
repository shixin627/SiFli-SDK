#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
release_ci.py — headless release runner for GitHub Actions (self-hosted).

Same flow as release_gui.py, no Tk: set RELEASE mode + version -> build ->
package -> info.json -> watchOS.zip -> (optional) flash + board screening ->
(optional) OSS upload. No BLE test and no MAC provisioning: those belong to
production 配號 and stay in the GUI.

All logic is imported from release_gui / set_build_mode; this file only
sequences the steps and turns any failure into a non-zero exit code.

  py -3 release_ci.py --version 1.2.3 --board sf32lb56-watch \
      --flash-port COM13 --hcpu-port COM14 --upload

Omit --flash-port to skip flashing and hardware tests. Omit --upload to keep
the artifacts local.
"""

import os
import sys
import json
import argparse
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

import release_gui as rg  # noqa: E402  (sibling module; all the real logic)

PY = sys.executable


def emit(text):
    print(text, flush=True)


def run(cmd, cwd=None, extra_env=None):
    """Run a child process, streaming its output into the CI log."""
    emit("$ " + " ".join(cmd))
    env = rg._child_env()
    if extra_env:
        env.update(extra_env)
    return subprocess.run(cmd, cwd=cwd or SCRIPT_DIR, env=env).returncode


def fail(message):
    emit("!! %s" % message)
    return 1


def next_version_from_oss():
    """Published version on OSS + 1 revision. Errors out rather than guessing:
    silently falling back to the local header could re-publish a version that
    already exists on OSS."""
    published = rg.oss.get_object(
        rg.oss.load_credentials(), rg.oss.info_json_key(rg.read_board()))
    if published is None:
        raise RuntimeError(
            "OSS 上還沒有 info.json，第一次發布請用 --version 明確指定版號。")
    version = json.loads(published.decode("utf-8")).get("version", "")
    if not rg.VER_RE.match(version):
        raise RuntimeError("OSS info.json 的版號格式無法解析：%r" % version)
    major, minor, revision = version.split(".")
    following = "%s.%s.%d" % (major, minor, int(revision) + 1)
    emit("OSS 上已發布 %s，本次發布 %s" % (version, following))

    _, local, _ = rg.read_status()
    if local != following:
        emit("（本機目前版號 %s，將被覆寫為 %s）" % (local, following))
    return following


def step_release_mode(version):
    emit("=== 切換到發布模式，版號 %s ===" % version)
    if run([PY, "set_build_mode.py", "release", "--version", version]) != 0:
        return fail("切換發布模式失敗。")
    mode, ver, is_release = rg.read_status()
    emit("目前狀態：%s ‧ 版號 %s" % (mode, ver))
    if not is_release or ver != version:
        return fail("發布模式/版號未套用（讀回 %s / %s）。" % (mode, ver))
    return 0


def step_build(board, notes, with_watchface):
    if notes:
        rg.write_description(notes)
        emit("已更新發布介紹：%s" % notes)

    emit("=== 編譯韌體（hcpu + lcpu），晶片 %s ===" % board)
    # Absolute path on purpose: NoDefaultCurrentDirectoryInExePath=1 (set by
    # some parent shells, including CI agents) stops cmd from searching cwd,
    # and a bare "_watch_build.cmd" then fails with "is not recognized".
    rc = run(["cmd", "/c", rg.WATCH_BUILD_CMD, "-j8"],
             extra_env={"WATCH_BOARD": board})
    # Exit code alone is not enough: scons has returned 0 on logs that contain
    # error lines, which is exactly what make_release.bat greps for.
    if rc != 0 or rg.build_failed():
        return fail("編譯失敗，請看 _watch_build.log。")
    emit("編譯成功。")

    wf = ["--with-watchface"] if with_watchface else []
    emit("=== 打包韌體到 watchOS\\sys ===")
    if run([PY, "package_watch_firmware.py", "--board", board] + wf) != 0:
        return fail("打包失敗。")

    emit("=== 更新 info.json ===")
    if run([PY, "update_info.py"] + wf) != 0:
        return fail("更新 info.json 失敗。")

    emit("=== 產生 watchOS.zip ===")
    ok, message, count = rg.make_watchos_zip()
    if not ok:
        return fail("產生 watchOS.zip 失敗：%s" % message)
    emit("已產生 %s（%d 個檔案）。" % (message, count))
    return 0


def step_flash_and_test(board, flash_port, hcpu_port, skip_hwtest):
    preflight = rg.flash_preflight_error(board)
    if preflight:
        return fail(preflight.replace("\n\n", " "))

    build_dir = rg.flash_build_dir(board)
    if not os.path.isfile(os.path.join(build_dir, "ImgBurnList.ini")):
        return fail("找不到 %s\\ImgBurnList.ini，請確認編譯用的是同一顆晶片。"
                    % build_dir)

    emit("=== 燒錄韌體：%s → %s ===" % (board, flash_port))
    if run(rg.flash_command(flash_port), cwd=build_dir) != 0:
        return fail("刷機失敗，請看 %s\\ImgBurn.log。" % build_dir)
    emit("=== 刷機完成 ===")

    if skip_hwtest:
        emit("（依參數跳過板級篩檢）")
        return 0

    rg.wait_for_system_boot(rg.POST_FLASH_BOOT_WAIT_SECONDS, emit)
    _, version, _ = rg.read_status()
    emit("=== 板級篩檢（CPU/RAM、Flash、電池、IMU、PPG）：%s ===" % hcpu_port)
    ok, detail = rg.run_hardware_selftest(
        hcpu_port, emit, expected_version=version,
        timeout=rg.HWTEST_SCREEN_TIMEOUT_SECONDS, screening_only=True)
    if not ok:
        return fail("板級篩檢失敗：%s" % detail)
    emit("=== 板級篩檢通過 ===")
    # No BLE test and no MAC provisioning here on purpose: a release is a
    # firmware check, and re-provisioning would burn a new BLE MAC on a watch
    # that already has one. Production 配號 stays in release_gui.py.
    return 0


def step_upload():
    creds = rg.oss.load_credentials()
    board = rg.read_board()
    _, version, _ = rg.read_status()

    key = rg.oss.info_json_key(board)
    emit("=== 上傳 info.json → %s ===" % key)
    emit("OK: %s" % rg.oss.put_object(creds, key, rg.INFO_JSON))

    key = rg.oss.watchos_zip_key(board, version)
    emit("=== 上傳 watchOS.zip → %s ===" % key)
    emit("OK: %s" % rg.oss.put_object(creds, key, rg.WATCHOS_ZIP))
    emit("=== 上傳完成 ===")
    return 0


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--version", default="", help="發布版號 X.Y.Z，"
                   "留空 = 讀 OSS 上已發布的版號 +1")
    p.add_argument("--board", default=rg.DEFAULT_BOARD,
                   choices=[b for _, b in rg.BOARD_CHOICES])
    p.add_argument("--notes", default="", help="info.json description")
    p.add_argument("--with-watchface", action="store_true")
    p.add_argument("--flash-port", default="", help="燒錄 COM，空白=不刷機")
    p.add_argument("--hcpu-port", default="", help="下指令/板級篩檢 COM")
    p.add_argument("--skip-hwtest", action="store_true",
                   help="刷機後不跑板級篩檢")
    p.add_argument("--upload", action="store_true", help="上傳到阿里雲 OSS")
    return p.parse_args(argv)


def resolve_ports(args):
    """Validate the CLI input. Returns (flash_port, hcpu_port, error).

    Pure on purpose — nothing here writes to the tree or touches a COM port,
    so selftest() can exercise it without running a release.
    """
    # An empty --version is resolved from OSS later; only a typo'd one is bad.
    if args.version and not rg.VER_RE.match(args.version):
        return None, None, "版號格式錯誤：%s（需 X.Y.Z）" % args.version
    if not args.flash_port:
        return "", "", None

    flash_port = rg.normalize_serial_port(args.flash_port)
    if not flash_port:
        return None, None, "燒錄 COM 格式錯誤：%s" % args.flash_port

    if args.hcpu_port:
        # A typo must not silently fall through to flash_port - 1: that would
        # talk to whatever else is on that COM port.
        hcpu_port = rg.normalize_serial_port(args.hcpu_port)
        if not hcpu_port:
            return None, None, "HCPU 篩檢 COM 格式錯誤：%s" % args.hcpu_port
    else:
        hcpu_port = rg.hcpu_port_from_flash(flash_port)
        if not hcpu_port and not args.skip_hwtest:
            return None, None, "無法從燒錄 COM %s 推導 HCPU COM，請指定 --hcpu-port" % flash_port
    return flash_port, hcpu_port, None


def selftest():
    """Guard the input validation: everything after it touches real hardware."""
    def check(*argv):
        return resolve_ports(parse_args(list(argv)))

    assert check("--version", "1.2")[2], "bad version must not pass"
    assert check("--version", "1.2.3", "--flash-port", "COM4 & whoami")[2]
    assert check("--version", "1.2.3", "--flash-port", "COM4",
                 "--hcpu-port", "nope")[2], "typo'd HCPU COM must not pass"
    assert check("--version", "1.2.3") == ("", "", None), "build-only is valid"
    # COM4 with no --hcpu-port derives COM3.
    assert check("--version", "1.2.3", "--flash-port", "com4") == (
        "COM4", "COM3", None)
    # Empty --version is legal: it means "resolve from OSS".
    assert check() == ("", "", None), "auto version must pass validation"
    print("release_ci selftest: OK")
    return 0


def main(argv=None):
    args = parse_args(argv)
    flash_port, hcpu_port, error = resolve_ports(args)
    if error:
        return fail(error)

    try:
        version = args.version or next_version_from_oss()
        rc = step_release_mode(version)
        if rc:
            return rc
        rc = step_build(args.board, args.notes, args.with_watchface)
        if rc:
            return rc
        if flash_port:
            rc = step_flash_and_test(
                args.board, flash_port, hcpu_port, args.skip_hwtest)
            if rc:
                return rc
        else:
            emit("（未指定 --flash-port，跳過刷機與實機測試）")
        if args.upload:
            rc = step_upload()
            if rc:
                return rc
        else:
            emit("（未指定 --upload，產物留在本機）")
    except Exception as e:
        return fail("%s: %s" % (e.__class__.__name__, e))

    emit("=== 發布流程完成：%s %s ===" % (args.board, version))
    return 0


if __name__ == "__main__":
    sys.exit(selftest() if "--selftest" in sys.argv else main())
