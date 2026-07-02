#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Switch the watch firmware between DEV and RELEASE build profiles.

A "release" flips several knobs scattered across four files. Doing it by hand
is error-prone (e.g. lcpu power pins were repeatedly left in release state).
This script is the single source of truth for the dev <-> release delta.

User-facing messages are in Traditional Chinese (the team's language). Code,
comments, and the Kconfig/#define identifiers stay ASCII.

Profile delta (derived from release commit dd100ed74):

  file                       key                         DEV            RELEASE
  -------------------------  --------------------------  -------------  -------------
  eh-lb56xu/bsp_board.h      kReleaseMode                0              1
                             CUSTOMER_BOARD_VER          BOARD_VER_28   BOARD_VER_29
  watch_global_data.h        VERSION                     (untouched)    user-entered (default +1)
  hcpu/proj.conf             RT_USING_FINSH              =y             # is not set
                             BSP_USING_VIRTUAL_CONSOLE   commented out  =y
                             BSP_PM_DEBUG                =y             # is not set
                             RT_USING_MEMTRACE           =y             # is not set
                             ULOG_OUTPUT_LVL_W           # is not set   =y  (strip D/I strings)
  lcpu/proj.conf             GH3018_POW_PIN              161            0
                             BMI270_POW_PIN              118            0
                             RT_USING_MEMTRACE           =y             # is not set
                             RT_USING_FINSH              off  (fixed both profiles)
                             BSP_USING_VIRTUAL_CONSOLE   on   (fixed both profiles)

BSP_PM_DEBUG / MEMTRACE are pure dev/debug overhead in a release build
(per-wake rt_kprintf, per-alloc tracking) -- dropped in release, kept in dev.
NOTE: BT_FINSH is deliberately left ON in both profiles -- see HCPU_BOOLS.

Only customer/boards/eh-lb56xu/bsp_board.h is touched: the production build
(scons --board=sf32lb56w-watch) pulls that board's drivers via
customer/boards/sf32lb56w-watch/SConscript. The pc/hcpu (simulator) and
sf32lb56-watch_base headers are separate targets with their own board versions
and are intentionally left alone.

Usage:
    python set_build_mode.py status                          # show current mode, no changes
    python set_build_mode.py release [--version X.Y.Z] [--dry-run]
                                                             # switch to RELEASE; prompts for the
                                                             # version (Enter = current +1), or pass
                                                             # --version to set it non-interactively
    python set_build_mode.py dev     [--dry-run]             # switch back to DEV
"""

import os
import re
import sys


def _force_utf8_console():
    """Make stdout/stdin handle CJK on Windows regardless of console code page.

    This machine's console is a Western (cp1252) code page, so naive printing
    of Chinese would turn into '?'/mojibake or raise UnicodeEncodeError. Switch
    the console to UTF-8 (65001) and reconfigure the Python streams to match.
    """
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
            ctypes.windll.kernel32.SetConsoleCP(65001)
        except Exception:
            pass
    for stream in (sys.stdout, sys.stdin, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")  # Python 3.7+
        except Exception:
            pass


_force_utf8_console()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# SCRIPT_DIR = .../example/get-started/dualcore/project/hcpu
BSP_BOARD_H = os.path.normpath(os.path.join(
    SCRIPT_DIR, "..", "..", "..", "..", "..",
    "customer", "boards", "eh-lb56xu", "bsp_board.h"))
HEADER_FILE = os.path.normpath(os.path.join(
    SCRIPT_DIR, "..", "..", "src", "modules", "model", "watch_global_data.h"))
HCPU_CONF = os.path.join(SCRIPT_DIR, "proj.conf")
LCPU_CONF = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "lcpu", "proj.conf"))

# --- per-profile values --------------------------------------------------

VALID_MODES = ("dev", "release")

# bsp_board.h #define values per profile.
# CUSTOMER_BOARD_VER: dev=28, release=29. Verified against history -- every commit
# that set kReleaseMode=1 (a real release build) also carried BOARD_VER_29, e.g.
# 02c41a14d (1.1.52) and 1f3fe6331 (1.1.49). The dev tips (kReleaseMode=0) sit on
# BOARD_VER_28. The "andrew_v28.x" branch names are just version labels, not the
# board ver of the shipped artifact.
BSP_DEFINES = {
    "release": {"kReleaseMode": "1", "CUSTOMER_BOARD_VER": "BOARD_VER_29"},
    "dev":     {"kReleaseMode": "0", "CUSTOMER_BOARD_VER": "BOARD_VER_28"},
}

# lcpu sensor power pins per profile (release board has no HR/IMU power switch).
LCPU_PINS = {
    "release": {"GH3018_POW_PIN": "0",   "BMI270_POW_PIN": "0"},
    "dev":     {"GH3018_POW_PIN": "161", "BMI270_POW_PIN": "118"},
}

# Kconfig bools, one row per knob:  (config_key, dev_enabled, release_enabled)
HCPU_BOOLS = [
    ("RT_USING_FINSH",            True,  False),  # shell off in release
    ("BSP_USING_VIRTUAL_CONSOLE", False, True),   # route console over virtual port
    ("BSP_PM_DEBUG",              True,  False),  # per-wake rt_kprintf (DBG_LVL can't mute it)
    ("RT_USING_MEMTRACE",         True,  False),  # per-alloc tracking overhead
    # ULOG static level: dev keeps the Kconfig default (Debug); release picks
    # Warning, which strips every LOG_D/LOG_I format string + call site out of
    # ROM at compile time (~60-80 KB). Release units therefore only emit W/E
    # over COM12 -- flash a dev build when I/D-level tracing is needed.
    ("ULOG_OUTPUT_LVL_W",         False, True),
    # BT_FINSH is intentionally NOT toggled. Despite the name it isn't just a
    # BT shell: middleware/bluetooth/service/bt/bt_finsh/ also holds the only
    # definitions of the HFP-HF call control (bt_hfp_hf_answer_call_send /
    # _hangup_call_send) that app_incoming_call.c links against. Disabling it
    # breaks the release link ("Undefined symbol bt_hfp_hf_answer_call_send").
    # Must stay =y in both profiles.
]
LCPU_BOOLS = [
    ("RT_USING_FINSH",            False, False),  # fixed: lcpu has no shell
    ("BSP_USING_VIRTUAL_CONSOLE", True,  True),   # fixed
    ("RT_USING_MEMTRACE",         True,  False),  # per-alloc tracking overhead
]


# --- low-level edit helpers ---------------------------------------------

class EditError(Exception):
    pass


def _read(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _write(path, text):
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


def set_define(text, name, value, path):
    """Replace `#define <name> <value>` in a C header. Errors if not found."""
    pattern = re.compile(r"(#define\s+" + re.escape(name) + r"\s+)(\S+)")
    m = pattern.search(text)
    if not m:
        raise EditError("在 %s 找不到 #define %s" % (path, name))
    old = m.group(2)
    new_text = pattern.sub(lambda mm: mm.group(1) + value, text, count=1)
    return new_text, old, value


def read_define(text, name):
    m = re.search(r"#define\s+" + re.escape(name) + r"\s+(\S+)", text)
    return m.group(1) if m else None


def parse_version(text):
    """Return (major, minor, revision) strings from the header."""
    return (read_define(text, "VERSION_MAJOR"),
            read_define(text, "VERSION_MINOR"),
            read_define(text, "VERSION_REVISION"))


def set_version(text, version_str, path):
    """Set VERSION_MAJOR/MINOR/REVISION from an 'X.Y.Z' string.
    Returns (new_text, old_version_str, new_version_str)."""
    parts = version_str.split(".")
    if len(parts) != 3 or not all(p.isdigit() for p in parts):
        raise EditError("版號格式錯誤 '%s'(需要 X.Y.Z,三段整數)" % version_str)
    old = "%s.%s.%s" % parse_version(text)
    out = text
    for name, val in zip(("VERSION_MAJOR", "VERSION_MINOR", "VERSION_REVISION"), parts):
        out, _, _ = set_define(out, name, val, path)
    return out, old, version_str


def _prompt_version(default_ver, cur_ver):
    """Ask the user for the release version. Empty input keeps the default."""
    print("")
    print("  目前版號      : %s" % cur_ver)
    print("  預設(按 Enter): %s" % default_ver)
    while True:
        try:
            raw = input("  請輸入發布版號 X.Y.Z [%s]: " % default_ver).strip()
        except EOFError:
            return default_ver
        if raw == "":
            return default_ver
        if re.match(r"^\d+\.\d+\.\d+$", raw):
            return raw
        print("  格式不對。請用 X.Y.Z(例如 %s),或直接按 Enter 用預設。" % default_ver)


def _kconfig_line_re(key):
    # Matches `CONFIG_KEY=...`, `# CONFIG_KEY is not set`, `# CONFIG_KEY=...`
    return re.compile(r"^\s*#?\s*CONFIG_" + re.escape(key) + r"\b")


def set_kconfig_bool(lines, key, enabled):
    """Force a Kconfig bool to enabled/disabled. Returns (lines, old, new)."""
    target = ("CONFIG_%s=y" % key) if enabled else ("# CONFIG_%s is not set" % key)
    rx = _kconfig_line_re(key)
    for i, line in enumerate(lines):
        if rx.match(line):
            old = line.rstrip("\n")
            lines[i] = target
            return lines, old, target
    # not present -> insert just after the first line (matches release-commit placement)
    insert_at = 1 if len(lines) >= 1 else 0
    lines.insert(insert_at, target)
    return lines, "(無此行)", target


def set_kconfig_value(lines, key, value):
    """Force `CONFIG_KEY=value`. Leaves trailing `# nnn` annotation lines alone."""
    target = "CONFIG_%s=%s" % (key, value)
    rx = re.compile(r"^\s*CONFIG_" + re.escape(key) + r"=")
    for i, line in enumerate(lines):
        if rx.match(line):
            old = line.rstrip("\n")
            lines[i] = target
            return lines, old, target
    lines.append(target)
    return lines, "(無此行)", target


def _split_lines(text):
    """Split keeping track of whether the file ended with a newline."""
    had_trailing_nl = text.endswith("\n")
    lines = text.split("\n")
    if had_trailing_nl:
        lines = lines[:-1]  # drop the empty tail produced by the final \n
    return lines, had_trailing_nl


def _join_lines(lines, had_trailing_nl):
    text = "\n".join(lines)
    if had_trailing_nl:
        text += "\n"
    return text


# --- reporting -----------------------------------------------------------

_changes = []


def record(path, key, old, new):
    rel = os.path.relpath(path, SCRIPT_DIR)
    changed = "" if str(old) == str(new) else "  <-- 已變更"
    _changes.append((rel, key, old, new, changed))


def print_changes(dry_run):
    if not _changes:
        print("  (沒有需要變更的項目)")
        return
    width_file = max(len(c[0]) for c in _changes)
    width_key = max(len(c[1]) for c in _changes)
    for rel, key, old, new, changed in _changes:
        print("  %-*s  %-*s  %s -> %s%s"
              % (width_file, rel, width_key, key, old, new, changed))
    if dry_run:
        print("\n  [預覽模式] 沒有寫入任何檔案。")


# --- main operations -----------------------------------------------------

def show_status():
    bsp = _read(BSP_BOARD_H)
    hdr = _read(HEADER_FILE)
    mode = "RELEASE(發布)" if read_define(bsp, "kReleaseMode") == "1" else "DEV(開發)"
    major = read_define(hdr, "VERSION_MAJOR")
    minor = read_define(hdr, "VERSION_MINOR")
    rev = read_define(hdr, "VERSION_REVISION")
    print("目前建置模式: %s" % mode)
    print("  kReleaseMode       = %s" % read_define(bsp, "kReleaseMode"))
    print("  CUSTOMER_BOARD_VER = %s" % read_define(bsp, "CUSTOMER_BOARD_VER"))
    print("  版號               = %s.%s.%s" % (major, minor, rev))


def apply_profile(mode, dry_run, cli_version=None):
    if mode not in VALID_MODES:
        raise EditError("未知模式: %s" % mode)
    is_release = (mode == "release")
    mode_label = "RELEASE(發布)" if is_release else "DEV(開發)"

    # 1) bsp_board.h: kReleaseMode + CUSTOMER_BOARD_VER
    bsp = _read(BSP_BOARD_H)
    cur_release = read_define(bsp, "kReleaseMode")
    for name, value in BSP_DEFINES[mode].items():
        bsp, o, n = set_define(bsp, name, value, BSP_BOARD_H)
        record(BSP_BOARD_H, name, o, n)

    # 2) watch_global_data.h: set VERSION on release (dev never touches it).
    #    The user types the version; default (Enter) is current +1 on a fresh
    #    dev->release switch, or the current version if already in release.
    hdr = _read(HEADER_FILE)
    if is_release:
        maj, minr, rev = parse_version(hdr)
        cur_ver = "%s.%s.%s" % (maj, minr, rev)
        if cur_release != "1":
            default_ver = "%s.%s.%d" % (maj, minr, int(rev) + 1)
        else:
            default_ver = cur_ver  # already release: keep unless overridden
        if cli_version:
            hdr, o, n = set_version(hdr, cli_version, HEADER_FILE)
            record(HEADER_FILE, "VERSION (--version)", o, n)
        elif dry_run:
            hdr, o, n = set_version(hdr, default_ver, HEADER_FILE)
            record(HEADER_FILE, "VERSION (預設值;實際執行時會詢問)", o, n)
        else:
            target_ver = _prompt_version(default_ver, cur_ver)
            hdr, o, n = set_version(hdr, target_ver, HEADER_FILE)
            record(HEADER_FILE, "VERSION", o, n)
    else:
        cur_ver = "%s.%s.%s" % parse_version(hdr)
        record(HEADER_FILE, "VERSION (dev - 不變動)", cur_ver, cur_ver)

    # 3) hcpu/proj.conf: profile-dependent Kconfig bools
    hcpu_text = _read(HCPU_CONF)
    hlines, h_nl = _split_lines(hcpu_text)
    for key, dev_en, rel_en in HCPU_BOOLS:
        hlines, o, n = set_kconfig_bool(hlines, key, rel_en if is_release else dev_en)
        record(HCPU_CONF, key, o, n)
    hcpu_out = _join_lines(hlines, h_nl)

    # 4) lcpu/proj.conf: sensor power pins (toggle) + Kconfig bools
    lcpu_text = _read(LCPU_CONF)
    llines, l_nl = _split_lines(lcpu_text)
    for key, value in LCPU_PINS[mode].items():
        llines, o, n = set_kconfig_value(llines, key, value)
        record(LCPU_CONF, key, o, n)
    for key, dev_en, rel_en in LCPU_BOOLS:
        llines, o, n = set_kconfig_bool(llines, key, rel_en if is_release else dev_en)
        record(LCPU_CONF, key, o, n)
    lcpu_out = _join_lines(llines, l_nl)

    print("切換到 %s 模式:" % mode_label)
    print_changes(dry_run)

    if not dry_run:
        _write(BSP_BOARD_H, bsp)
        _write(HEADER_FILE, hdr)
        _write(HCPU_CONF, hcpu_out)
        _write(LCPU_CONF, lcpu_out)
        print("\n完成。目前模式為 %s。" % mode_label)
        if is_release:
            print("接著用 _watch_build.cmd 編譯,再用 update_info.bat 打包")
            print("(或直接跑 make_release.bat 一次做完)。")
            print("之後要回到開發,執行:  python set_build_mode.py dev")


def _opt_value(argv, name):
    """Return the value of `--name VALUE` or `--name=VALUE`, else None."""
    for i, a in enumerate(argv):
        if a == name:
            return argv[i + 1] if i + 1 < len(argv) else None
        if a.startswith(name + "="):
            return a.split("=", 1)[1]
    return None


def main():
    argv = sys.argv[1:]
    dry_run = "--dry-run" in argv
    cli_version = _opt_value(argv, "--version")

    # Positional command = first bare token, skipping flags and the --version value.
    args = []
    skip = False
    for a in argv:
        if skip:
            skip = False
            continue
        if a == "--version":
            skip = True  # next token is its value, not a positional
            continue
        if a.startswith("-"):
            continue
        args.append(a)

    cmd = args[0] if args else None
    try:
        if cmd == "status":
            show_status()
        elif cmd in ("dev", "release"):
            apply_profile(cmd, dry_run, cli_version)
        else:
            print(__doc__)
            print("錯誤: 指令需為 status | dev | release 其中之一")
            return 2
    except EditError as e:
        print("錯誤: %s" % e)
        return 1
    except FileNotFoundError as e:
        print("錯誤: 找不到檔案: %s" % e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
