#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 2026-08-28 tap 測試為了拿到 shell 而做的暫時改動還原成 RELEASE 組態。

為什麼不是用 `set_build_mode.py release`:那支會連 `CUSTOMER_BOARD_VER` 一起翻,
而這次**刻意沒動板號**(founder 確認開發錶也是 BOARD_VER_29,dev/release 是同一塊
實體板,翻成 VER_28 會把 GH3018/BMI270 的電源腳位錯綁)。所以這裡只還原那三個
真正被改掉的旗標,板號與 LCPU 腳位完全不碰。

  bsp_board.h   kReleaseMode                0  -> 1
  hcpu/proj.conf RT_USING_FINSH             =y -> is not set
                 BSP_USING_VIRTUAL_CONSOLE  off -> =y

用法: python _restore_release_config.py [--check]
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
BSP = os.path.join(REPO, "customer", "boards", "eh-lb56xu", "bsp_board.h")
PROJ = os.path.join(HERE, "proj.conf")

EDITS = [
    (BSP, "#define kReleaseMode 0", "#define kReleaseMode 1"),
    (PROJ, "CONFIG_RT_USING_FINSH=y", "# CONFIG_RT_USING_FINSH is not set"),
    (PROJ, "# CONFIG_BSP_USING_VIRTUAL_CONSOLE is not set",
     "CONFIG_BSP_USING_VIRTUAL_CONSOLE=y"),
]


def main():
    check = "--check" in sys.argv
    changed = 0
    for path, dev, rel in EDITS:
        with open(path, encoding="utf-8", errors="surrogateescape") as fh:
            s = fh.read()
        if rel in s and dev not in s:
            print(f"  OK   already release: {os.path.basename(path)}  {rel}")
            continue
        if dev not in s:
            print(f"  WARN neither form found in {os.path.basename(path)}: {dev!r}")
            continue
        print(f"  {'WOULD FIX' if check else 'FIXED'}  {os.path.basename(path)}"
              f"  {dev}  ->  {rel}")
        changed += 1
        if not check:
            with open(path, "w", encoding="utf-8", errors="surrogateescape") as fh:
                fh.write(s.replace(dev, rel, 1))
    if changed and not check:
        print("\n改完要重編才會生效。板號 CUSTOMER_BOARD_VER 與 LCPU 電源腳位未被觸碰。")
    elif not changed:
        print("\n已經是 release 組態。")


if __name__ == "__main__":
    main()
