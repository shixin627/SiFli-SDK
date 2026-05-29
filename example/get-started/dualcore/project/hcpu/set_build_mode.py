#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Switch the watch firmware between DEV and RELEASE build profiles.

A "release" flips several knobs scattered across four files. Doing it by hand
is error-prone (e.g. lcpu power pins were repeatedly left in release state).
This script is the single source of truth for the dev <-> release delta.

Profile delta (derived from release commit dd100ed74):

  file                       key                         DEV            RELEASE
  -------------------------  --------------------------  -------------  -------------
  eh-lb56xu/bsp_board.h      kReleaseMode                0              1
                             CUSTOMER_BOARD_VER          BOARD_VER_28   BOARD_VER_29
  watch_global_data.h        VERSION_REVISION            (untouched)    +1 on dev->release
  hcpu/proj.conf             RT_USING_FINSH              =y             # is not set
                             BSP_USING_VIRTUAL_CONSOLE   commented out  =y
                             BSP_PM_DEBUG                =y             # is not set
                             RT_USING_MEMTRACE           =y             # is not set
                             BT_FINSH                    =y             # is not set
  lcpu/proj.conf             GH3018_POW_PIN              161            0
                             BMI270_POW_PIN              118            0
                             RT_USING_MEMTRACE           =y             # is not set
                             RT_USING_FINSH              off  (fixed both profiles)
                             BSP_USING_VIRTUAL_CONSOLE   on   (fixed both profiles)

BSP_PM_DEBUG / MEMTRACE / BT_FINSH are pure dev/debug overhead in a release
build (per-wake rt_kprintf, per-alloc tracking, BT shell cmds) — dropped in
release, kept in dev.

Only customer/boards/eh-lb56xu/bsp_board.h is touched: the production build
(scons --board=sf32lb56w-watch) pulls that board's drivers via
customer/boards/sf32lb56w-watch/SConscript. The pc/hcpu (simulator) and
sf32lb56-watch_base headers are separate targets with their own board versions
and are intentionally left alone.

Usage:
    python set_build_mode.py status              # show current mode, no changes
    python set_build_mode.py release [--dry-run]  # switch to RELEASE (+ bump on transition)
    python set_build_mode.py dev     [--dry-run]  # switch back to DEV
"""

import os
import re
import sys

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
# CUSTOMER_BOARD_VER: dev=28, release=29. Verified against history — every commit
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
    ("BT_FINSH",                  True,  False),  # BT shell cmds, useless without FINSH
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
        raise EditError(f"#define {name} not found in {path}")
    old = m.group(2)
    new_text = pattern.sub(lambda mm: mm.group(1) + value, text, count=1)
    return new_text, old, value


def read_define(text, name):
    m = re.search(r"#define\s+" + re.escape(name) + r"\s+(\S+)", text)
    return m.group(1) if m else None


def bump_revision(text, path):
    """VERSION_REVISION += 1."""
    pattern = re.compile(r"(#define\s+VERSION_REVISION\s+)(\d+)")
    m = pattern.search(text)
    if not m:
        raise EditError(f"#define VERSION_REVISION not found in {path}")
    old = int(m.group(2))
    new = old + 1
    new_text = pattern.sub(lambda mm: mm.group(1) + str(new), text, count=1)
    return new_text, str(old), str(new)


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
    return lines, "(absent)", target


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
    return lines, "(absent)", target


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
    changed = "" if str(old) == str(new) else "  <-- changed"
    _changes.append((rel, key, old, new, changed))


def print_changes(dry_run):
    if not _changes:
        print("  (nothing to change)")
        return
    width_file = max(len(c[0]) for c in _changes)
    width_key = max(len(c[1]) for c in _changes)
    for rel, key, old, new, changed in _changes:
        print("  %-*s  %-*s  %s -> %s%s"
              % (width_file, rel, width_key, key, old, new, changed))
    if dry_run:
        print("\n  [dry-run] no files were written.")


# --- main operations -----------------------------------------------------

def show_status():
    bsp = _read(BSP_BOARD_H)
    hdr = _read(HEADER_FILE)
    mode = "RELEASE" if read_define(bsp, "kReleaseMode") == "1" else "DEV"
    major = read_define(hdr, "VERSION_MAJOR")
    minor = read_define(hdr, "VERSION_MINOR")
    rev = read_define(hdr, "VERSION_REVISION")
    print("Current build profile: %s" % mode)
    print("  kReleaseMode       = %s" % read_define(bsp, "kReleaseMode"))
    print("  CUSTOMER_BOARD_VER = %s" % read_define(bsp, "CUSTOMER_BOARD_VER"))
    print("  version            = %s.%s.%s" % (major, minor, rev))


def apply_profile(mode, dry_run):
    if mode not in VALID_MODES:
        raise EditError("unknown mode: %s" % mode)
    is_release = (mode == "release")

    # 1) bsp_board.h: kReleaseMode + CUSTOMER_BOARD_VER
    bsp = _read(BSP_BOARD_H)
    cur_release = read_define(bsp, "kReleaseMode")
    for name, value in BSP_DEFINES[mode].items():
        bsp, o, n = set_define(bsp, name, value, BSP_BOARD_H)
        record(BSP_BOARD_H, name, o, n)

    # 2) watch_global_data.h: bump VERSION_REVISION only on dev -> release
    hdr = _read(HEADER_FILE)
    do_bump = (is_release and cur_release != "1")
    if do_bump:
        hdr, o, n = bump_revision(hdr, HEADER_FILE)
        record(HEADER_FILE, "VERSION_REVISION", o, n)
    else:
        rev = read_define(hdr, "VERSION_REVISION")
        note = "already release - not re-bumped" if is_release else "dev - untouched"
        record(HEADER_FILE, "VERSION_REVISION (%s)" % note, rev, rev)

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

    print("Switching to %s profile:" % mode.upper())
    print_changes(dry_run)

    if not dry_run:
        _write(BSP_BOARD_H, bsp)
        _write(HEADER_FILE, hdr)
        _write(HCPU_CONF, hcpu_out)
        _write(LCPU_CONF, lcpu_out)
        print("\nDone. Profile is now %s." % mode.upper())
        if mode == "release":
            print("Build with _watch_build.cmd, then package with update_info.bat")
            print("(or just run make_release.bat to do everything).")
            print("Resume dev work later with:  python set_build_mode.py dev")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    dry_run = "--dry-run" in sys.argv

    cmd = args[0] if args else None
    try:
        if cmd == "status":
            show_status()
        elif cmd in ("dev", "release"):
            apply_profile(cmd, dry_run)
        else:
            print(__doc__)
            print("ERROR: expected one of: status | dev | release")
            return 2
    except EditError as e:
        print("ERROR: %s" % e)
        return 1
    except FileNotFoundError as e:
        print("ERROR: file not found: %s" % e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
