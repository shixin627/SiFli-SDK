#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Emit the filesystem copies of the logos that no longer live in the image.

Why: see the note above UI_FS_ICON in src/hcpu/gui_apps/common/ui_img_helper.h.
The watch reads these as LVGL v8 image files: a 4-byte lv_img_header_t followed
by exactly the bytes the resource build already produces for the C array, so the
file is byte-for-byte the same picture the firmware used to embed.

Source of truth for WHICH logos: every `UI_FS_ICON("<name>")` in ui_img_helper.h.
Source of the pixels: the resource build's generated `<name>.tmp.c` (run a watch
build first so they exist).

Writes the same files to two places:
  project/jsroot/assets/icons/        -> packed into the FS image for new watches
  SkaiLink android assets watch_icons/ -> copied down by the phone to watches
                                          already in the field

usage: python gen_fs_icons.py [--build build_sf32lb56-watch_hcpu] [--check]
"""
import argparse
import glob
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DUALCORE = os.path.abspath(os.path.join(HERE, "..", ".."))
HELPER = os.path.join(DUALCORE, "src", "hcpu", "gui_apps", "common", "ui_img_helper.h")
JSROOT_ICONS = os.path.join(DUALCORE, "project", "jsroot", "assets", "icons")
MONOREPO = os.path.abspath(os.path.join(DUALCORE, "..", "..", "..", ".."))
PHONE_ICONS = os.path.join(MONOREPO, "SkaiLink", "android-native", "app", "src", "main",
                           "assets", "watch_icons")

# lv_img_cf_t values (LVGL v8.3)
CF = {"LV_IMG_CF_TRUE_COLOR": 4, "LV_IMG_CF_TRUE_COLOR_ALPHA": 5,
      "LV_IMG_CF_RAW": 1, "LV_IMG_CF_RAW_ALPHA": 2}


def icon_names():
    text = open(HELPER, encoding="utf-8").read()
    return sorted(set(re.findall(r'UI_FS_ICON\("([a-z0-9_]+)"\)', text)))


def lv_header(cf, w, h):
    # uint32 bitfield, LSB first: cf:5, always_zero:3, reserved:2, w:11, h:11
    return struct.pack("<I", (cf & 0x1F) | ((w & 0x7FF) << 10) | ((h & 0x7FF) << 21))


def convert(tmp_c):
    src = open(tmp_c, encoding="utf-8", errors="replace").read()
    body = re.search(r"_map\[\][^{]*\{(.*?)\};", src, re.S)
    cf = re.search(r"\.header\.cf\s*=\s*(LV_IMG_CF_\w+)", src)
    w = re.search(r"\.header\.w\s*=\s*(\d+)", src)
    h = re.search(r"\.header\.h\s*=\s*(\d+)", src)
    size = re.search(r"\.data_size\s*=\s*(\d+)", src)
    if not (body and cf and w and h and size):
        raise ValueError("unrecognised resource file: " + tmp_c)
    data = bytes(int(b, 16) for b in re.findall(r"0x([0-9a-fA-F]{2})", body.group(1)))
    if len(data) != int(size.group(1)):
        raise ValueError("%s: %d bytes but data_size=%s" % (tmp_c, len(data), size.group(1)))
    if cf.group(1) not in CF:
        raise ValueError("%s: unsupported cf %s" % (tmp_c, cf.group(1)))
    return lv_header(CF[cf.group(1)], int(w.group(1)), int(h.group(1))) + data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", default="build_sf32lb56-watch_hcpu")
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if any output is missing or stale, write nothing")
    args = ap.parse_args()

    build = os.path.join(HERE, args.build)
    names = icon_names()
    if not names:
        sys.exit("no UI_FS_ICON entries found in " + HELPER)

    stale = []
    for out_dir in (JSROOT_ICONS, PHONE_ICONS):
        if not args.check:
            os.makedirs(out_dir, exist_ok=True)
    for name in names:
        hits = glob.glob(os.path.join(build, "src", "resource", "images", "**", name + ".tmp.c"),
                         recursive=True)
        if not hits:
            sys.exit("missing %s.tmp.c under %s — build the watch first" % (name, build))
        blob = convert(hits[0])
        for out_dir in (JSROOT_ICONS, PHONE_ICONS):
            out = os.path.join(out_dir, name + ".bin")
            if os.path.exists(out) and open(out, "rb").read() == blob:
                continue
            if args.check:
                stale.append(out)
            else:
                open(out, "wb").write(blob)
                print("wrote", os.path.relpath(out, MONOREPO), len(blob))
    if args.check and stale:
        print("stale:\n  " + "\n  ".join(stale))
        sys.exit(1)
    print("%d icons ok" % len(names))


if __name__ == "__main__":
    main()
