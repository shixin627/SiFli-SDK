#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
release_gui.py — Traditional-Chinese GUI front-end for the watch release flow.

Why a GUI: the release prompts are Chinese, and on a Western (cp1252) Windows
console the text is dropped/garbled no matter what code page we set. A Tk window
renders Unicode via system fonts and ignores the console code page entirely, so
this side-steps the whole encoding problem.

It is a thin front-end: all *writes* still go through set_build_mode.py (the
single source of truth). The GUI's DEV action keeps the production hardware
pin/board values while applying the rest of the DEV profile. Reads (current
mode / version) reuse set_build_mode's helpers directly.

Run:  python release_gui.py        (or double-click release_gui.bat)
Self-test (headless):  python release_gui.py --selftest
"""

import os
import re
import sys
import json
import queue
import zipfile
import threading
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

import set_build_mode as sbm  # noqa: E402  (sibling module; reuse paths + helpers)
import oss_upload as oss       # noqa: E402  (sibling module; Aliyun OSS client)

PY = sys.executable
REPO_ROOT = os.path.normpath(os.path.join(
    SCRIPT_DIR, "..", "..", "..", "..", ".."))
INFO_JSON = os.path.join(SCRIPT_DIR, "info.json")
BUILD_LOG = os.path.join(SCRIPT_DIR, "_watch_build.log")
WATCH_BUILD_CMD = os.path.join(SCRIPT_DIR, "_watch_build.cmd")
WATCHOS_DIR = os.path.join(SCRIPT_DIR, "watchOS")
WATCHOS_ZIP = os.path.join(SCRIPT_DIR, "watchOS.zip")
UART_DOWNLOAD_EXE = os.path.join(
    REPO_ROOT, "tools", "uart_download", "ImgDownUart.exe")

# Chip / scons board choices. The board name is both the scons --board value
# (via WATCH_BOARD for _watch_build.cmd) and the build-output dir suffix that
# package_watch_firmware.py reads (build_<board>_hcpu). Production first so it
# is the default selection.
BOARD_CHOICES = (
    ("量產機 — sf32lb563 (sf32lb56-watch)", "sf32lb56-watch"),
    ("開發機 — sf32lb563w (sf32lb56w-watch)", "sf32lb56w-watch"),
)
DEFAULT_BOARD = "sf32lb56-watch"  # 量產正式機


def read_board():
    """Return the board number as a string, e.g. '29', from CUSTOMER_BOARD_VER
    (which is 'BOARD_VER_29'). Used as the OSS key prefix skaiwatch/<board>/."""
    bsp = sbm._read(sbm.BSP_BOARD_H)
    raw = sbm.read_define(bsp, "CUSTOMER_BOARD_VER") or ""
    m = re.search(r"(\d+)", raw)
    return m.group(1) if m else raw

# Same failure patterns make_release.bat greps for.
BUILD_ERROR_PATTERNS = (" error:", "undefined reference", "cannot find", "scons: ***")

VER_RE = re.compile(r"^\d+\.\d+\.\d+$")
SERIAL_PORT_RE = re.compile(r"^COM([1-9]\d*)$", re.IGNORECASE)


# --- read-only helpers (no subprocess) ----------------------------------

def read_status():
    """Return (mode_label, version_str) by reading the headers directly."""
    bsp = sbm._read(sbm.BSP_BOARD_H)
    hdr = sbm._read(sbm.HEADER_FILE)
    is_release = sbm.read_define(bsp, "kReleaseMode") == "1"
    mode = "RELEASE(發布)" if is_release else "DEV(開發)"
    ver = "%s.%s.%s" % sbm.parse_version(hdr)
    return mode, ver, is_release


def suggested_version():
    """Default version to pre-fill: current+1 from dev, or current if already release."""
    bsp = sbm._read(sbm.BSP_BOARD_H)
    hdr = sbm._read(sbm.HEADER_FILE)
    maj, minr, rev = sbm.parse_version(hdr)
    if sbm.read_define(bsp, "kReleaseMode") == "1":
        return "%s.%s.%s" % (maj, minr, rev)
    return "%s.%s.%d" % (maj, minr, int(rev) + 1)


def normalize_serial_port(value):
    """Return a normalized COM port name, or None for invalid input."""
    port = value.strip().upper()
    return port if SERIAL_PORT_RE.fullmatch(port) else None


def list_serial_ports():
    """Return Windows serial ports such as COM3, sorted numerically."""
    if os.name != "nt":
        return []
    try:
        import winreg
        key = winreg.OpenKey(
            winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
    except (ImportError, FileNotFoundError, OSError):
        return []

    ports = set()
    try:
        index = 0
        while True:
            try:
                value = winreg.EnumValue(key, index)[1]
            except OSError:
                break
            port = normalize_serial_port(str(value))
            if port:
                ports.add(port)
            index += 1
    finally:
        winreg.CloseKey(key)

    return sorted(
        ports, key=lambda port: int(SERIAL_PORT_RE.fullmatch(port).group(1)))


def flash_build_dir(board):
    return os.path.join(SCRIPT_DIR, "build_%s_hcpu" % board)


def flash_command(port):
    """Build the non-interactive equivalent of generated uart_download.bat."""
    return [
        UART_DOWNLOAD_EXE,
        "--func", "0",
        "--port", port.lower(),
        "--baund", "1000000",
        "--loadram", "1",
        "--postact", "1",
        "--compare",
        "--verify",
        "--device", "SF32LB56X_NAND",
        "--file", "ImgBurnList.ini",
        "--log", "ImgBurn.log",
    ]


def write_description(notes):
    """Set info.json 'description' (release notes), preserving the rest."""
    data = {}
    if os.path.exists(INFO_JSON):
        with open(INFO_JSON, "r", encoding="utf-8") as f:
            txt = f.read().strip()
            if txt:
                data = json.loads(txt)
    data["description"] = notes
    with open(INFO_JSON, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)


def build_failed():
    """True if the watch build log contains any known error marker."""
    if not os.path.exists(BUILD_LOG):
        return True  # no log => build didn't run
    with open(BUILD_LOG, "r", encoding="utf-8", errors="replace") as f:
        log = f.read()
    return any(p in log for p in BUILD_ERROR_PATTERNS)


def make_watchos_zip():
    """Zip the watchOS/ folder into watchOS.zip. Entries keep the leading
    'watchOS/' prefix to match the existing tracked artifact (e.g.
    watchOS/sys/hcpu.bin). Returns (ok, message, count)."""
    if not os.path.isdir(WATCHOS_DIR):
        return False, "找不到 watchOS 資料夾,請先完成打包。", 0
    count = 0
    # arcnames are relative to SCRIPT_DIR so they include the 'watchOS/' prefix.
    with zipfile.ZipFile(WATCHOS_ZIP, "w", zipfile.ZIP_DEFLATED) as z:
        for root, dirs, files in os.walk(WATCHOS_DIR):
            dirs.sort()
            for name in sorted(files):
                full = os.path.join(root, name)
                arc = os.path.relpath(full, SCRIPT_DIR).replace("\\", "/")
                z.write(full, arc)
                count += 1
    return True, os.path.basename(WATCHOS_ZIP), count


def _child_env():
    env = dict(os.environ)
    env["PYTHONUTF8"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    return env


def apply_gui_dev_profile():
    """Apply DEV settings without reverting production hardware values."""
    original_bsp = sbm.BSP_DEFINES["dev"]
    original_pins = sbm.LCPU_PINS["dev"]
    sbm.BSP_DEFINES["dev"] = dict(
        original_bsp, CUSTOMER_BOARD_VER="BOARD_VER_29")
    sbm.LCPU_PINS["dev"] = dict(
        original_pins, GH3018_POW_PIN="0", BMI270_POW_PIN="0")
    try:
        sbm.apply_profile("dev", dry_run=False)
    finally:
        sbm.BSP_DEFINES["dev"] = original_bsp
        sbm.LCPU_PINS["dev"] = original_pins


# --- self-test (headless, no Tk window) ---------------------------------

def selftest():
    out_path = os.path.join(SCRIPT_DIR, "_gui_selftest.txt")
    lines = []

    def add(s):
        lines.append(str(s))

    try:
        mode, ver, is_rel = read_status()
        add("read_status: mode=%s ver=%s is_release=%s" % (mode, ver, is_rel))
    except Exception as e:
        add("read_status FAILED: %r" % e)

    try:
        add("suggested_version: %s" % suggested_version())
    except Exception as e:
        add("suggested_version FAILED: %r" % e)

    add("scripts_present: set_build_mode=%s watch_build_cmd=%s info_json=%s" % (
        os.path.exists(os.path.join(SCRIPT_DIR, "set_build_mode.py")),
        os.path.exists(WATCH_BUILD_CMD),
        os.path.exists(INFO_JSON)))
    add("flash_tool_present: %s" % os.path.isfile(UART_DOWNLOAD_EXE))
    add("serial_port_regex_ok: %s / %s" % (
        normalize_serial_port("com12") == "COM12",
        normalize_serial_port("COM12 & whoami") is None))
    add("detected_serial_ports: %s" % (", ".join(list_serial_ports()) or "none"))

    try:
        import tkinter  # noqa: F401
        add("tkinter_importable: True")
    except Exception as e:
        add("tkinter_importable: False (%r)" % e)

    add("version_regex_ok: %s / %s" % (bool(VER_RE.match("1.2.3")),
                                       bool(VER_RE.match("1.2"))))

    # Chip/board selector: default must be the production board, and every
    # choice should have its scons build-output dir present (build_<board>_hcpu).
    boards = [b for _, b in BOARD_CHOICES]
    add("default_board: %s (production=%s)" % (
        DEFAULT_BOARD, DEFAULT_BOARD == "sf32lb56-watch" and DEFAULT_BOARD in boards))
    for b in boards:
        add("board_build_dir[%s]: %s" % (
            b, os.path.isdir(os.path.join(SCRIPT_DIR, "build_%s_hcpu" % b))))

    add("watchOS_dir_present: %s" % os.path.isdir(WATCHOS_DIR))
    if "--zip" in sys.argv:
        try:
            ok, msg, n = make_watchos_zip()
            add("make_watchos_zip: ok=%s msg=%s files=%d" % (ok, msg, n))
        except Exception as e:
            add("make_watchos_zip FAILED: %r" % e)
    else:
        add("make_watchos_zip: skipped (pass --zip to actually build the zip)")

    try:
        board = read_board()
        add("read_board: %s" % board)
        add("oss_info_key: %s" % oss.info_json_key(board))
        add("oss_zip_key: %s" % oss.watchos_zip_key(board, "%s.%s.%s"
                                                     % sbm.parse_version(sbm._read(sbm.HEADER_FILE))))
    except Exception as e:
        add("oss key build FAILED: %r" % e)
    try:
        oss.load_credentials()
        add("oss_credentials: loaded")
    except Exception:
        add("oss_credentials: not configured (expected until oss_credentials.json exists)")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    # also echo (may be garbled on console, but the file is authoritative)
    print("\n".join(lines))
    return 0


# --- GUI ----------------------------------------------------------------

def run_gui():
    import tkinter as tk
    from tkinter import ttk, messagebox

    # Fonts that include CJK glyphs on Windows.
    UI_FONT = ("Microsoft JhengHei UI", 10)
    UI_BOLD = ("Microsoft JhengHei UI", 11, "bold")
    LOG_FONT = ("MingLiU", 10)  # fixed-width CJK so the change report aligns

    SIG_LOG = "log"
    SIG_DONE = "done"
    SIG_REFRESH = "refresh"

    class App:
        def __init__(self, root):
            self.root = root
            self.q = queue.Queue()
            self.busy = False
            root.title("Skaiwalk 手錶韌體發布與燒錄工具")
            root.geometry("760x650")
            root.minsize(640, 560)

            pad = {"padx": 8, "pady": 4}

            # --- status row ---
            top = ttk.Frame(root)
            top.pack(fill="x", **pad)
            self.status_var = tk.StringVar(value="讀取中…")
            ttk.Label(top, text="目前狀態：", font=UI_BOLD).pack(side="left")
            ttk.Label(top, textvariable=self.status_var, font=UI_FONT).pack(side="left")
            self.btn_refresh = ttk.Button(top, text="重新整理", command=self.refresh)
            self.btn_refresh.pack(side="right")

            ttk.Separator(root).pack(fill="x", padx=8, pady=2)

            # --- release row ---
            rel = ttk.LabelFrame(root, text="切換到發布(RELEASE)")
            rel.pack(fill="x", **pad)
            ttk.Label(rel, text="發布版號 X.Y.Z：", font=UI_FONT).grid(
                row=0, column=0, sticky="w", padx=6, pady=6)
            self.ver_var = tk.StringVar()
            self.ver_entry = ttk.Entry(rel, textvariable=self.ver_var, width=16, font=UI_FONT)
            self.ver_entry.grid(row=0, column=1, sticky="w", pady=6)
            self.btn_release = ttk.Button(rel, text="切換到發布模式", command=self.do_release)
            self.btn_release.grid(row=0, column=2, padx=10, pady=6)
            ttk.Label(rel, text="(版號會寫進韌體與 info.json)", font=UI_FONT,
                      foreground="#666").grid(row=1, column=0, columnspan=3, sticky="w", padx=6)

            # --- dev row ---
            dev = ttk.LabelFrame(root, text="切換回開發(DEV)")
            dev.pack(fill="x", **pad)
            self.btn_dev = ttk.Button(dev, text="切換到開發模式", command=self.do_dev)
            self.btn_dev.pack(side="left", padx=6, pady=6)
            ttk.Label(dev, text="(還原 shell / log；心率‧IMU 電源腳位與板號維持量產設定)",
                      font=UI_FONT, foreground="#666").pack(side="left", padx=6)

            # --- chip / board row ---
            chip = ttk.LabelFrame(root, text="晶片型號(芯片)")
            chip.pack(fill="x", **pad)
            self.board_var = tk.StringVar(value=DEFAULT_BOARD)
            for i, (label, board) in enumerate(BOARD_CHOICES):
                ttk.Radiobutton(chip, text=label, value=board,
                                variable=self.board_var).grid(
                    row=0, column=i, sticky="w", padx=6, pady=6)
            ttk.Label(chip, text="(量產正式機請選 sf32lb56-watch;sf32lb56w-watch 只在開發機用)",
                      font=UI_FONT, foreground="#666").grid(
                row=1, column=0, columnspan=2, sticky="w", padx=6)

            # --- build row ---
            bld = ttk.LabelFrame(root, text="編譯並打包(發布版)")
            bld.pack(fill="x", **pad)
            self.wf_var = tk.BooleanVar(value=False)
            ttk.Checkbutton(bld, text="包含錶面(watchface)", variable=self.wf_var).grid(
                row=0, column=0, sticky="w", padx=6, pady=4)
            ttk.Label(bld, text="發布介紹：", font=UI_FONT).grid(
                row=1, column=0, sticky="w", padx=6)
            self.notes_var = tk.StringVar()
            ttk.Entry(bld, textvariable=self.notes_var, width=50, font=UI_FONT).grid(
                row=1, column=1, sticky="we", padx=6, pady=4)
            self.btn_build = ttk.Button(bld, text="編譯 + 打包", command=self.do_build)
            self.btn_build.grid(row=0, column=1, sticky="e", padx=6)
            bld.columnconfigure(1, weight=1)
            ttk.Label(bld, text="(請先切換到發布模式;編譯需數分鐘,過程顯示於下方)",
                      font=UI_FONT, foreground="#666").grid(
                row=2, column=0, columnspan=2, sticky="w", padx=6)

            # --- flash row ---
            flash = ttk.LabelFrame(root, text="燒錄韌體(UART)")
            flash.pack(fill="x", **pad)
            ttk.Label(flash, text="COM Port：", font=UI_FONT).grid(
                row=0, column=0, sticky="w", padx=6, pady=6)
            ports = list_serial_ports()
            self.port_var = tk.StringVar(value=ports[0] if ports else "")
            self.port_combo = ttk.Combobox(
                flash, textvariable=self.port_var, values=ports,
                width=14, font=UI_FONT)
            self.port_combo.grid(row=0, column=1, sticky="w", pady=6)
            self.btn_port_refresh = ttk.Button(
                flash, text="重新偵測", command=self.refresh_ports)
            self.btn_port_refresh.grid(row=0, column=2, padx=6, pady=6)
            self.btn_flash = ttk.Button(
                flash, text="刷入手錶", command=self.do_flash)
            self.btn_flash.grid(row=0, column=3, padx=6, pady=6)
            ttk.Label(
                flash,
                text="(使用上方所選晶片的 build_<board>_hcpu 產物；刷機前會再次確認)",
                font=UI_FONT, foreground="#666").grid(
                    row=1, column=0, columnspan=4, sticky="w", padx=6)

            # --- upload row ---
            up = ttk.LabelFrame(root, text="上傳到雲端(阿里雲 OSS)")
            up.pack(fill="x", **pad)
            self.up_hint = tk.StringVar(value="")
            self.btn_upload = ttk.Button(up, text="上傳 info.json + watchOS.zip",
                                         command=self.do_upload)
            self.btn_upload.pack(side="left", padx=6, pady=6)
            ttk.Label(up, textvariable=self.up_hint, font=UI_FONT,
                      foreground="#666").pack(side="left", padx=6)

            # --- log pane ---
            logf = ttk.LabelFrame(root, text="訊息")
            logf.pack(fill="both", expand=True, **pad)
            self.log = tk.Text(logf, wrap="none", height=12, font=LOG_FONT,
                               state="disabled", background="#111", foreground="#ddd")
            yscroll = ttk.Scrollbar(logf, orient="vertical", command=self.log.yview)
            self.log.configure(yscrollcommand=yscroll.set)
            yscroll.pack(side="right", fill="y")
            self.log.pack(side="left", fill="both", expand=True)

            self._buttons = [self.btn_refresh, self.btn_release, self.btn_dev,
                             self.btn_build, self.btn_port_refresh,
                             self.btn_flash, self.btn_upload]

            self.refresh()
            self.root.after(80, self._pump)

        # --- ui helpers ---
        def append(self, text):
            self.log.configure(state="normal")
            self.log.insert("end", text + "\n")
            self.log.see("end")
            self.log.configure(state="disabled")

        def set_busy(self, busy):
            self.busy = busy
            state = "disabled" if busy else "normal"
            for b in self._buttons:
                b.configure(state=state)

        def refresh(self):
            try:
                mode, ver, _ = read_status()
                self.status_var.set("%s ‧ 版號 %s" % (mode, ver))
                if not self.ver_var.get().strip():
                    self.ver_var.set(suggested_version())
            except Exception as e:
                self.status_var.set("讀取失敗: %r" % e)
            # upload-target hint (board/version derived; keys shown for clarity)
            try:
                board = read_board()
                _, ver, _ = read_status()
                self.up_hint.set("→ skaiwatch/%s/info.json、skaiwatch/%s/%s/watchOS.zip"
                                 % (board, board, ver))
            except Exception:
                self.up_hint.set("")

        def refresh_ports(self):
            ports = list_serial_ports()
            current = self.port_var.get().strip()
            self.port_combo.configure(values=ports)
            if not current and ports:
                self.port_var.set(ports[0])
            if ports:
                self._emit("偵測到 COM Port: %s" % ", ".join(ports))
            else:
                self._emit("未自動偵測到 COM Port；可手動輸入，例如 COM12。")

        # --- queue pump (runs on UI thread) ---
        def _pump(self):
            try:
                while True:
                    kind, payload = self.q.get_nowait()
                    if kind == SIG_LOG:
                        self.append(payload)
                    elif kind == SIG_REFRESH:
                        self.refresh()
                    elif kind == SIG_DONE:
                        self.set_busy(False)
            except queue.Empty:
                pass
            self.root.after(80, self._pump)

        # --- worker plumbing ---
        def _emit(self, text):
            self.q.put((SIG_LOG, text))

        def _stream(self, cmd, cwd=None, extra_env=None):
            """Run cmd, push each output line to the log. Returns exit code."""
            self._emit("$ " + " ".join(cmd))
            env = _child_env()
            if extra_env:
                env.update(extra_env)
            try:
                p = subprocess.Popen(
                    cmd, cwd=cwd or SCRIPT_DIR, env=env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding="utf-8", errors="replace", bufsize=1)
            except Exception as e:
                self._emit("[無法啟動] %r" % e)
                return -1
            for line in p.stdout:
                self._emit(line.rstrip("\n"))
            p.wait()
            return p.returncode

        def _start(self, target):
            if self.busy:
                return
            self.set_busy(True)
            threading.Thread(target=target, daemon=True).start()

        # --- actions ---
        def do_release(self):
            ver = self.ver_var.get().strip()
            if not VER_RE.match(ver):
                messagebox.showerror("版號格式錯誤", "請輸入 X.Y.Z 格式(三段整數),例如 1.1.61")
                return
            self._start(lambda: self._w_release(ver))

        def _w_release(self, ver):
            self._emit("=== 切換到發布模式,版號 %s ===" % ver)
            rc = self._stream([PY, "set_build_mode.py", "release", "--version", ver])
            self._emit("完成。" if rc == 0 else "切換失敗(結束碼 %d)。" % rc)
            self.q.put((SIG_REFRESH, None))
            self.q.put((SIG_DONE, None))

        def do_dev(self):
            self._start(self._w_dev)

        def _w_dev(self):
            self._emit("=== 切換回開發模式 ===")
            rc = self._stream([PY, "release_gui.py", "--apply-dev-profile"])
            self._emit("完成。" if rc == 0 else "切換失敗(結束碼 %d)。" % rc)
            self.q.put((SIG_REFRESH, None))
            self.q.put((SIG_DONE, None))

        def do_build(self):
            _, _, is_rel = read_status()
            if not is_rel:
                if not messagebox.askyesno(
                        "尚未切換到發布模式",
                        "目前不是發布模式,編出來的會是開發版。\n要繼續嗎?"):
                    return
            with_wf = self.wf_var.get()
            notes = self.notes_var.get().strip()
            board = self.board_var.get()
            self._start(lambda: self._w_build(with_wf, notes, board))

        def _w_build(self, with_wf, notes, board):
            try:
                if notes:
                    write_description(notes)
                    self._emit("已更新發布介紹: %s" % notes)
            except Exception as e:
                self._emit("更新發布介紹失敗: %r" % e)

            self._emit("=== 編譯韌體(hcpu + lcpu),晶片 %s,請稍候… ===" % board)
            self._stream(["cmd", "/c", "_watch_build.cmd", "-j8"],
                         extra_env={"WATCH_BOARD": board})
            if build_failed():
                self._emit("!! 編譯失敗,請查看 _watch_build.log。發布參數仍維持,"
                           "修正後可重試,或切回開發模式。")
                self.q.put((SIG_DONE, None))
                return
            self._emit("編譯成功。")

            wf = ["--with-watchface"] if with_wf else []
            self._emit("=== 打包韌體到 watchOS\\sys ===")
            if self._stream([PY, "package_watch_firmware.py", "--board", board] + wf) != 0:
                self._emit("!! 打包失敗。")
                self.q.put((SIG_DONE, None))
                return

            self._emit("=== 更新 info.json ===")
            if self._stream([PY, "update_info.py"] + wf) != 0:
                self._emit("!! 更新 info.json 失敗。")
                self.q.put((SIG_DONE, None))
                return

            self._emit("=== 產生 watchOS.zip ===")
            try:
                ok, msg, n = make_watchos_zip()
                if ok:
                    self._emit("已產生 %s(%d 個檔案)。" % (msg, n))
                else:
                    self._emit("!! 產生 watchOS.zip 失敗: %s" % msg)
            except Exception as e:
                self._emit("!! 產生 watchOS.zip 發生例外: %r" % e)

            self._emit("=== 發布完成! 產物: watchOS\\sys\\、watchOS.zip、info.json ===")
            self._emit("提醒: 可按「刷入手錶」測試或「上傳到雲端」上傳;"
                       "繼續開發請按「切換到開發模式」。")
            self.q.put((SIG_REFRESH, None))
            self.q.put((SIG_DONE, None))

        def do_flash(self):
            port = normalize_serial_port(self.port_var.get())
            if not port:
                messagebox.showerror(
                    "COM Port 格式錯誤",
                    "請選擇或輸入有效的 COM Port，例如 COM12。")
                return

            board = self.board_var.get()
            build_dir = flash_build_dir(board)
            burn_list = os.path.join(build_dir, "ImgBurnList.ini")
            if not os.path.isfile(UART_DOWNLOAD_EXE):
                messagebox.showerror(
                    "缺少燒錄工具", "找不到：\n%s" % UART_DOWNLOAD_EXE)
                return
            if not os.path.isfile(burn_list):
                messagebox.showerror(
                    "缺少燒錄產物",
                    "找不到：\n%s\n\n請先用相同晶片型號完成編譯。" % burn_list)
                return

            self.port_var.set(port)
            if not messagebox.askyesno(
                    "確認刷機",
                    "晶片型號：%s\nCOM Port：%s\n\n"
                    "即將覆寫手錶韌體，確定要繼續嗎？" % (board, port)):
                return
            self._start(lambda: self._w_flash(board, port, build_dir))

        def _w_flash(self, board, port, build_dir):
            self._emit("=== 燒錄韌體：%s → %s ===" % (board, port))
            rc = self._stream(flash_command(port), cwd=build_dir)
            if rc == 0:
                self._emit("=== 刷機完成：%s ===" % port)
            else:
                self._emit("!! 刷機失敗(結束碼 %d)。請查看 %s" % (
                    rc, os.path.join(build_dir, "ImgBurn.log")))
            self.q.put((SIG_DONE, None))

        def do_upload(self):
            # Pre-flight: artifacts present + credentials configured.
            if not os.path.exists(INFO_JSON):
                messagebox.showerror("缺少檔案", "找不到 info.json,請先編譯打包。")
                return
            if not os.path.exists(WATCHOS_ZIP):
                messagebox.showerror("缺少檔案", "找不到 watchOS.zip,請先編譯打包。")
                return
            try:
                creds = oss.load_credentials()
            except oss.OssError as e:
                messagebox.showerror("尚未設定 OSS 認證", str(e))
                return
            board = read_board()
            _, ver, _ = read_status()
            if not messagebox.askyesno(
                    "確認上傳",
                    "板號 %s ‧ 版號 %s\n\n"
                    "info.json   → skaiwatch/%s/info.json\n"
                    "watchOS.zip → skaiwatch/%s/%s/watchOS.zip\n\n要上傳嗎?"
                    % (board, ver, board, board, ver)):
                return
            self._start(lambda: self._w_upload(creds, board, ver))

        def _w_upload(self, creds, board, ver):
            try:
                k_info = oss.info_json_key(board)
                self._emit("=== 上傳 info.json → %s ===" % k_info)
                url = oss.put_object(creds, k_info, INFO_JSON)
                self._emit("OK: %s" % url)

                k_zip = oss.watchos_zip_key(board, ver)
                self._emit("=== 上傳 watchOS.zip → %s ===" % k_zip)
                url = oss.put_object(creds, k_zip, WATCHOS_ZIP)
                self._emit("OK: %s" % url)

                self._emit("=== 上傳完成! ===")
            except oss.OssError as e:
                self._emit("!! 上傳失敗: %s" % e)
            except Exception as e:
                self._emit("!! 上傳發生例外: %r" % e)
            self.q.put((SIG_DONE, None))

    root = tk.Tk()
    try:
        App(root)
    except Exception as e:
        try:
            from tkinter import messagebox
            messagebox.showerror("啟動失敗", repr(e))
        except Exception:
            pass
        raise
    root.mainloop()
    return 0


def main():
    if "--apply-dev-profile" in sys.argv:
        try:
            apply_gui_dev_profile()
        except (sbm.EditError, FileNotFoundError) as e:
            sys.stderr.write("切換開發模式失敗: %s\n" % e)
            return 1
        return 0
    if "--selftest" in sys.argv:
        return selftest()
    try:
        return run_gui()
    except ImportError as e:
        sys.stderr.write("無法載入 GUI(缺少 tkinter?): %r\n" % e)
        return 1


if __name__ == "__main__":
    sys.exit(main())
