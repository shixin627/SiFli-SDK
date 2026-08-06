#!/usr/bin/env python3
"""How far is each built-in app from being reproducible as a JS app?

Counts the LVGL API each gui_apps/ app actually calls and splits it into what
the curated skai_ui capabilities already cover and what they do not. The point
is to turn "reproduce every app in JS" from a wish into a number before anyone
writes the first one.

    python tools/sdk/app_gap.py            # per-app summary + global ranking
    python tools/sdk/app_gap.py --app main # one app, every missing symbol

ponytail: the LV->capability map below is hand-written and deliberately
incomplete. Anything not in it counts as MISSING, so the number errs pessimistic
and the ranking (which is what decides work order) stays right. Extend the map
when a capability lands, not before.
"""
import argparse
import collections
import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
APPS = ROOT / "example/get-started/dualcore/src/hcpu/gui_apps"
REGISTRY = ROOT / "example/get-started/dualcore/src/modules/sdk/generated/capability-registry.json"

# lv_* symbol -> the ui.* capability that already covers it.
COVERED = {
    "lv_label_create": "ui.label",
    "lv_btn_create": "ui.button",
    "lv_arc_create": "ui.arc",
    "lv_bar_create": "ui.bar",
    "lv_btnmatrix_create": "ui.keypad",
    "lv_line_create": "ui.divider",
    "lv_img_create": "ui.image",
    "lv_img_set_src": "ui.image",
    "lv_label_set_text": "ui.set_text",
    "lv_label_set_text_fmt": "ui.set_text",
    "lv_arc_set_value": "ui.set_arc",
    "lv_bar_set_value": "ui.bar",   # no setter yet; creation carries the value
    "lv_obj_set_style_text_color": "ui.set_color",
    "lv_obj_set_style_bg_color": "ui.set_bg",
    "lv_obj_set_style_text_font": "ui.set_font",
    "lv_obj_set_size": "ui.set_size",
    "lv_obj_set_width": "ui.set_size",
    "lv_obj_set_height": "ui.set_size",
    "lv_obj_align": "ui.align",
    "lv_obj_align_to": "ui.align",
    "lv_obj_center": "ui.align",
    "lv_btnmatrix_set_map": "ui.keypad",
}

# Calling these IS the app framework, not drawing -- a JS app gets the same
# behaviour from being hosted, so counting them as gaps would inflate the number.
IGNORED = {
    "lv_obj_create",       # skai_ui owns the container
    "lv_scr_act",
    "lv_obj_del",
    "lv_obj_clean",
    "lv_obj_remove_style_all",
    "lv_color_hex",
    "lv_color_black",
    "lv_color_white",
    "lv_color_make",
    "lv_pct",
}

CALL = re.compile(r"\b(lv_[a-z0-9_]+)\s*\(")


def scan(app_dir):
    counts = collections.Counter()
    files = 0
    for path in sorted(app_dir.rglob("*.c")):
        files += 1
        counts.update(CALL.findall(path.read_text(errors="ignore")))
    for name in IGNORED:
        counts.pop(name, None)
    return files, counts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--app", help="only this app, and list every missing symbol")
    args = ap.parse_args()

    if not APPS.is_dir():
        sys.exit(f"no gui_apps at {APPS}")

    caps = set()
    if REGISTRY.exists():
        caps = {c["name"] for c in json.loads(REGISTRY.read_text())["capabilities"]}
    unknown = {v for v in COVERED.values() if caps and v not in caps}
    if unknown:
        print(f"warning: map names capabilities that do not exist: {sorted(unknown)}\n")

    missing_apps = collections.Counter()   # symbol -> how many apps need it
    missing_calls = collections.Counter()  # symbol -> total call sites
    rows = []

    for app_dir in sorted(p for p in APPS.iterdir() if p.is_dir()):
        if args.app and app_dir.name != args.app:
            continue
        files, counts = scan(app_dir)
        if not counts:
            continue
        covered = {s: n for s, n in counts.items() if s in COVERED}
        missing = {s: n for s, n in counts.items() if s not in COVERED}
        for s, n in missing.items():
            missing_apps[s] += 1
            missing_calls[s] += n
        total = sum(counts.values())
        rows.append((app_dir.name, files, len(counts), len(missing),
                     100.0 * sum(covered.values()) / total if total else 100.0))
        if args.app:
            print(f"{app_dir.name}: {files} files, {total} LVGL calls, "
                  f"{len(counts)} distinct APIs\n")
            print("  missing, most used first:")
            for s, n in sorted(missing.items(), key=lambda kv: -kv[1]):
                print(f"    {n:4d}  {s}")
            return

    rows.sort(key=lambda r: r[4])
    print(f"{'app':<24}{'files':>6}{'APIs':>6}{'missing':>9}{'covered':>9}")
    for name, files, apis, miss, pct in rows:
        print(f"{name:<24}{files:>6}{apis:>6}{miss:>9}{pct:>8.0f}%")

    print(f"\n{len(rows)} apps. Most-wanted missing APIs "
          f"(apps needing it x call sites):")
    for s, napps in missing_apps.most_common(25):
        print(f"  {napps:3d} apps {missing_calls[s]:5d} calls  {s}")


if __name__ == "__main__":
    main()
