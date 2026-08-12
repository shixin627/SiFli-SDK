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
import time
import queue
import asyncio
import secrets
import tempfile
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
HWTEST_SOURCE = os.path.join(
    REPO_ROOT, "example", "get-started", "dualcore", "src", "modules",
    "model", "hardware_selftest.c")
TEST_APP_SOURCE = os.path.join(
    REPO_ROOT, "example", "get-started", "dualcore", "src", "hcpu",
    "gui_apps", "test", "app_test.c")
UART_DOWNLOAD_EXE = os.path.join(
    REPO_ROOT, "tools", "uart_download", "ImgDownUart.exe")
JSROOT_PACKED_BIN = os.path.join(SCRIPT_DIR, "build", "jsroot_packed.bin")
JSROOT_FLASH_ADDRESS = "0x64280000"
BLE_BATTERY_LEVEL_UUID = "00002a19-0000-1000-8000-00805f9b34fb"
BLE_SCAN_SECONDS = 12
BLE_CONNECT_TIMEOUT_SECONDS = 15
BLE_PROVISION_BOOT_WAIT_SECONDS = 8
DEFAULT_BLE_RSSI_THRESHOLD = -75
MAC_REGISTRY_FILE = os.path.join(
    os.environ.get("LOCALAPPDATA", os.path.expanduser("~")),
    "Skaiwalk", "release_gui", "ble_mac_registry.json")

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
BLE_MAC_RE = re.compile(
    r"^[0-9A-F]{2}(?::[0-9A-F]{2}){5}$", re.IGNORECASE)
HWTEST_TIMEOUT_SECONDS = 180
HWTEST_SCREEN_TIMEOUT_SECONDS = 45
POST_FLASH_BOOT_WAIT_SECONDS = 8


class HardwareSelftestCancelled(Exception):
    """Raised when the operator cancels the PC-side hardware test."""


def wait_for_system_boot(seconds, emit, cancel_event=None):
    """Wait after flashing before opening HCPU UART and sending commands."""
    emit("=== 等待系統完整開機（%d 秒） ===" % seconds)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if cancel_event is not None and cancel_event.is_set():
            raise HardwareSelftestCancelled("板級篩檢已取消。")
        time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
    emit("=== 系統開機等待完成 ===")


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


def hcpu_port_from_flash(port):
    """Return the usual HCPU port: download COM number minus one."""
    normalized = normalize_serial_port(port)
    if not normalized:
        return ""
    number = int(SERIAL_PORT_RE.fullmatch(normalized).group(1))
    return "COM%d" % (number - 1) if number > 1 else ""


def suggested_flash_port(ports):
    """Production fixtures normally enumerate download UART last."""
    return ports[-1] if ports else ""


def suggested_hcpu_port(ports, flash_port=""):
    """Prefer download COM-1; the operator can always override."""
    derived = hcpu_port_from_flash(flash_port)
    if derived:
        return derived
    return next((port for port in ports if port.upper() != "COM1"),
                ports[0] if ports else "")


def flash_build_dir(board):
    return os.path.join(SCRIPT_DIR, "build_%s_hcpu" % board)


def hardware_selftest_object(board):
    return os.path.join(
        flash_build_dir(board), "modules", "model", "hardware_selftest.o")


def interactive_test_app_object(board):
    return os.path.join(
        flash_build_dir(board), "src", "gui_apps", "test", "app_test.o")


def flash_preflight_error(board):
    """Return why the selected image is stale/failed, or None if usable."""
    if os.path.isfile(BUILD_LOG) and build_failed():
        return ("上次編譯失敗，不能刷入舊的建置產物。\n\n"
                "請先查看 _watch_build.log、修正錯誤並重新編譯。")

    required_objects = (
        (hardware_selftest_object(board), HWTEST_SOURCE, "UART 篩檢服務"),
        (interactive_test_app_object(board), TEST_APP_SOURCE,
         "test app（UART 篩檢與互動測試）"),
    )
    for obj, source, label in required_objects:
        if not os.path.isfile(source):
            continue
        if not os.path.isfile(obj):
            return ("目前建置產物未包含%s。\n\n"
                    "請先按『編譯 + 打包』重新編譯，再進行刷機。" % label)
        if os.path.getmtime(obj) < os.path.getmtime(source):
            return ("%s已修改，但建置產物尚未更新。\n\n"
                    "請先按『編譯 + 打包』重新編譯，再進行刷機。" % label)
    return None


def flash_command(port, burn_list="ImgBurnList.ini"):
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
        "--file", burn_list,
        "--log", "ImgBurn.log",
    ]


def create_resource_burn_list(build_dir):
    """Create a one-file UART burn list for the packed JS/image resources."""
    relative_resource = os.path.relpath(
        JSROOT_PACKED_BIN, build_dir).replace("/", "\\")
    handle = tempfile.NamedTemporaryFile(
        mode="w", suffix=".ini", prefix="_jsroot_burn_",
        dir=build_dir, delete=False, encoding="ascii", newline="\n")
    try:
        handle.write("[FILEINFO]\n")
        handle.write("FILE0=%s\n" % relative_resource)
        handle.write("ADDR0=%s\n" % JSROOT_FLASH_ADDRESS)
        handle.write("NUM=1\n")
    finally:
        handle.close()
    return handle.name


def normalize_ble_mac(value):
    mac = value.strip().upper().replace("-", ":")
    return mac if BLE_MAC_RE.fullmatch(mac) else None


def normalize_rssi_threshold(value):
    try:
        threshold = int(str(value).strip())
    except (TypeError, ValueError):
        return None
    return threshold if -100 <= threshold <= -20 else None


def _parse_provision_reply(line, action, nonce):
    prefix = "SKAI_PROVISION %s %s " % (action, nonce)
    if not line.startswith(prefix):
        return None
    status = line[len(prefix):].split(None, 1)[0]
    if status != "OK":
        return {
            "ok": False,
            "detail": line,
        }
    fields = dict(re.findall(r"\b([a-z_]+)=([^\s]+)", line))
    mac = normalize_ble_mac(fields.get("mac", ""))
    device = fields.get("device", "").upper()
    if not mac or not re.fullmatch(r"[0-9A-F]{12}", device):
        return {
            "ok": False,
            "detail": "韌體回傳的 BLE 配號資料格式錯誤：%s" % line,
        }
    return {
        "ok": True,
        "mac": mac,
        "device": device,
        "advertising": fields.get("advertising") == "1",
        "reboot": fields.get("reboot") == "1",
        "detail": line,
    }


def uart_provision_request(port, action, emit, timeout=20):
    """Request current/provisioned BLE identity over the HCPU production UART."""
    try:
        import serial
    except ImportError as e:
        raise RuntimeError(
            "缺少 pyserial，請執行：py -3 -m pip install pyserial") from e

    # The leading newline is load-bearing. The firmware's line buffer only
    # resets on '\n', so if a partial line is left in it — a truncated command
    # from a previous connection, line noise — the next command is appended to
    # that junk, the prefix no longer matches, and it is dropped in silence.
    # GET survives because it is resent every 0.8s; MAC is sent exactly once
    # and would simply never be answered. Verified on hardware: junk + MAC =
    # no reply, junk + "\n" + MAC = reply in 0.2s.
    nonce = secrets.token_hex(4).upper()
    command = ("\nSKAI_PROVISION %s %s\n" % (action, nonce)).encode("ascii")
    deadline = time.monotonic() + timeout
    next_send = 0.0
    # GET is idempotent and may be retried.  MAC must be sent exactly once:
    # a repeated command would assign a second address before reboot.
    allow_retry = action == "GET"
    sent = False

    try:
        with serial.Serial(port=port, baudrate=1000000, timeout=0.25,
                           write_timeout=2) as uart:
            uart.reset_input_buffer()
            while time.monotonic() < deadline:
                now = time.monotonic()
                if not sent or (allow_retry and now >= next_send):
                    uart.write(command)
                    uart.flush()
                    sent = True
                    next_send = now + 0.8

                raw = uart.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                protocol_at = line.find("SKAI_PROVISION ")
                if protocol_at < 0:
                    continue
                line = line[protocol_at:]
                parsed = _parse_provision_reply(line, action, nonce)
                if parsed is not None:
                    emit(line)
                    return parsed
    except serial.SerialException as e:
        raise RuntimeError("無法開啟 %s：%s" % (port, e)) from e

    raise RuntimeError(
        "%s 沒有收到 BLE %s 回應，請確認韌體已更新且 COM port 未被占用。"
        % (port, action))


def _load_mac_registry():
    if not os.path.isfile(MAC_REGISTRY_FILE):
        return {"records": {}}
    with open(MAC_REGISTRY_FILE, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict) or not isinstance(data.get("records"), dict):
        raise RuntimeError("BLE MAC 登錄檔格式錯誤：%s" % MAC_REGISTRY_FILE)
    return data


def _check_mac_unique(mac, device_identity):
    data = _load_mac_registry()
    previous = data["records"].get(mac)
    if previous and previous.get("device") != device_identity:
        raise RuntimeError(
            "BLE MAC %s 已登錄給另一塊板子（device=%s），不可出貨。"
            % (mac, previous.get("device", "unknown")))
    return data


def _record_mac(data, mac, device_identity, board, version, rssi):
    os.makedirs(os.path.dirname(MAC_REGISTRY_FILE), exist_ok=True)
    data["records"][mac] = {
        "device": device_identity,
        "board": board,
        "version": version,
        "rssi": rssi,
        "tested_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    temp_path = MAC_REGISTRY_FILE + ".tmp"
    with open(temp_path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False, sort_keys=True)
    os.replace(temp_path, MAC_REGISTRY_FILE)


def _advertisement_matches_mac(device, advertisement, mac):
    canonical = bytes.fromhex(mac.replace(":", ""))
    stack_order = canonical[::-1]
    if normalize_ble_mac(getattr(device, "address", "") or "") == mac:
        return True
    manufacturer_data = getattr(advertisement, "manufacturer_data", {}) or {}
    for payload in manufacturer_data.values():
        raw = bytes(payload)
        if raw[:6] in (canonical, stack_order):
            return True
    return False


async def _ble_scan_connect_async(
        mac, rssi_threshold, emit, cancel_event=None):
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError as e:
        raise RuntimeError(
            "缺少 bleak，無法執行 BLE 掃描/連線。請執行："
            "py -3 -m pip install bleak") from e

    matches = {}

    def detected(device, advertisement):
        if _advertisement_matches_mac(device, advertisement, mac):
            rssi = getattr(advertisement, "rssi", None)
            previous = matches.get(device.address)
            if previous is None or (
                    rssi is not None and
                    (previous[1] is None or rssi > previous[1])):
                matches[device.address] = (device, rssi)

    scanner = BleakScanner(detection_callback=detected)
    try:
        await scanner.start()
    except Exception as e:
        if e.__class__.__name__ == "BleakBluetoothNotAvailableError":
            raise RuntimeError(
                "Windows 找不到可用的 Bluetooth Low Energy 介面。"
                "請啟用藍牙或插入支援 BLE 的 USB 治具/接收器。") from e
        raise
    try:
        scan_deadline = time.monotonic() + BLE_SCAN_SECONDS
        while time.monotonic() < scan_deadline:
            if cancel_event is not None and cancel_event.is_set():
                raise HardwareSelftestCancelled("BLE 測試已取消。")
            await asyncio.sleep(min(0.2, scan_deadline - time.monotonic()))
    finally:
        await scanner.stop()

    if not matches:
        raise RuntimeError(
            "%d 秒內找不到 MAC %s 的 BLE advertising。"
            % (BLE_SCAN_SECONDS, mac))
    device, rssi = max(
        matches.values(),
        key=lambda item: item[1] if item[1] is not None else -999)
    if rssi is None:
        raise RuntimeError("找到 %s，但 Windows 未回報 RSSI。" % mac)
    emit("BLE advertising：%s RSSI=%d dBm（門檻 %d dBm）" % (
        mac, rssi, rssi_threshold))
    if rssi < rssi_threshold:
        raise RuntimeError(
            "BLE RSSI %d dBm 低於固定治具門檻 %d dBm。" %
            (rssi, rssi_threshold))

    emit("正在連線 BLE 並讀取 GATT Battery Level…")
    async with BleakClient(
            device, timeout=BLE_CONNECT_TIMEOUT_SECONDS) as client:
        if not client.is_connected:
            raise RuntimeError("BLE advertising 可見，但無法建立連線。")
        battery = await client.read_gatt_char(BLE_BATTERY_LEVEL_UUID)
        if len(battery) != 1:
            raise RuntimeError(
                "BLE 已連線，但 Battery Level 回應長度異常：%d" % len(battery))
        battery_level = int(battery[0])
        emit("BLE GATT 通訊通過：Battery Level=%d%%" % battery_level)
    return {
        "rssi": rssi,
        "battery_level": battery_level,
    }


def run_ble_provisioning_test(port, rssi_threshold, emit, board, version,
                              generate_new_mac, cancel_event=None):
    """Provision/read MAC, verify uniqueness, scan RSSI, and perform GATT I/O."""
    if cancel_event is not None and cancel_event.is_set():
        raise HardwareSelftestCancelled("BLE 測試已取消。")
    old_info = uart_provision_request(port, "GET", emit)
    if not old_info["ok"]:
        raise RuntimeError(old_info["detail"])

    if generate_new_mac:
        emit("=== 透過 HCPU UART 產生並寫入新的 BLE static random MAC ===")
        provisioned = uart_provision_request(port, "MAC", emit)
        if not provisioned["ok"]:
            raise RuntimeError(provisioned["detail"])
        wait_for_system_boot(
            BLE_PROVISION_BOOT_WAIT_SECONDS, emit, cancel_event)
        current = uart_provision_request(port, "GET", emit)
        if not current["ok"]:
            raise RuntimeError(current["detail"])
        if current["mac"] == old_info["mac"]:
            raise RuntimeError("BLE MAC 寫入後沒有改變，配號失敗。")
        if provisioned.get("mac") and current["mac"] != provisioned["mac"]:
            raise RuntimeError(
                "重啟後 MAC 與寫入回應不一致：%s != %s" %
                (current["mac"], provisioned["mac"]))
    else:
        current = old_info

    registry = _check_mac_unique(current["mac"], current["device"])
    emit("BLE MAC 唯一性通過：%s（device=%s）" % (
        current["mac"], current["device"]))
    result = asyncio.run(_ble_scan_connect_async(
        current["mac"], rssi_threshold, emit, cancel_event))
    _record_mac(
        registry, current["mac"], current["device"],
        board, version, result["rssi"])
    emit("BLE 配號/測試紀錄：%s" % MAC_REGISTRY_FILE)
    return current["mac"], result


def run_hardware_selftest(port, emit, expected_version=None,
                          timeout=HWTEST_TIMEOUT_SECONDS, cancel_event=None,
                          screening_only=False):
    """Run the release-firmware UART hardware smoke test."""
    try:
        import serial
    except ImportError as e:
        raise RuntimeError("缺少 pyserial；請執行：py -3 -m pip install pyserial") from e

    nonce = secrets.token_hex(4).upper()
    action = "SCREEN" if screening_only else "RUN"
    # Leading '\n' flushes any partial line left in the firmware's parser —
    # see the note in uart_provision_request.
    command = ("\nSKAI_HWTEST %s %s\n" % (action, nonce)).encode("ascii")
    end_prefix = "SKAI_HWTEST END %s " % nonce
    begin_prefix = "SKAI_HWTEST BEGIN %s " % nonce
    deadline = time.monotonic() + timeout
    next_send = 0.0
    began = False
    version_ok = True
    device_version = None
    rx_bytes = 0
    raw_lines = 0
    raw_lines_logged = 0

    if cancel_event is not None and cancel_event.is_set():
        raise HardwareSelftestCancelled("硬體自測已取消。")

    try:
        with serial.Serial(port=port, baudrate=1000000, timeout=0.25,
                           write_timeout=2) as uart:
            uart.reset_input_buffer()
            while time.monotonic() < deadline:
                if cancel_event is not None and cancel_event.is_set():
                    cancel_command = (
                        "\nSKAI_HWTEST CANCEL %s\n" % nonce).encode("ascii")
                    uart.write(cancel_command)
                    uart.flush()
                    raise HardwareSelftestCancelled("硬體自測已取消。")

                now = time.monotonic()
                # Keep requesting with the same nonce until END arrives.
                # Firmware ignores it while running and replays cached END
                # after completion, so a lost final UART line is recoverable.
                if now >= next_send:
                    uart.write(command)
                    uart.flush()
                    next_send = now + 1.0

                raw = uart.readline()
                if not raw:
                    continue
                rx_bytes += len(raw)
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                protocol_at = line.find("SKAI_HWTEST ")
                if protocol_at < 0:
                    raw_lines += 1
                    if raw_lines_logged < 12:
                        preview = re.sub(
                            r"[\x00-\x08\x0b\x0c\x0e-\x1f]", ".",
                            line)[:180]
                        emit("UART RX（非協定）：%s" % preview)
                        raw_lines_logged += 1
                    elif raw_lines_logged == 12:
                        emit("UART RX（非協定）：後續內容已省略")
                        raw_lines_logged += 1
                    continue
                # Console/debug output can share the UART and occasionally
                # precede a protocol packet on the same physical line.
                line = line[protocol_at:].strip()
                if nonce not in line:
                    if raw_lines_logged < 12:
                        emit("UART RX（其他測試識別碼）：%s" % line[:180])
                        raw_lines_logged += 1
                    continue
                emit(line)
                if line.startswith(begin_prefix):
                    began = True
                    match = re.search(r"\bversion=(\d+\.\d+\.\d+)\b", line)
                    device_version = match.group(1) if match else None
                    if expected_version and device_version != expected_version:
                        version_ok = False
                        emit("版本不符：GUI=%s，手錶=%s" % (
                            expected_version, device_version or "unknown"))
                if line.startswith(end_prefix):
                    status = line[len(end_prefix):].split(None, 1)[0]
                    ok = (status == "PASS" and version_ok)
                    if not version_ok:
                        return False, "版本不符：GUI=%s，手錶=%s" % (
                            expected_version, device_version or "unknown")
                    return ok, line
    except serial.SerialException as e:
        raise RuntimeError("無法開啟 %s：%s" % (port, e)) from e

    if began:
        return False, (
            "硬體自測逾時：已收到 BEGIN 與 %d bytes，但未收到相同識別碼的 "
            "END。" % rx_bytes)
    if rx_bytes:
        return False, (
            "硬體自測逾時：%s 共收到 %d bytes（%d 行非協定資料），"
            "但沒有收到 SKAI_HWTEST BEGIN。請查看訊息區的 UART RX 原始內容。"
            % (port, rx_bytes, raw_lines))
    return False, ("硬體自測逾時：%s 沒有回應。請確認刷入的是包含硬體自測服務的"
                   "最新發布韌體、在開機後 300 秒內執行，且沒有其他程式占用 "
                   "COM port。" % port)


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
    add("screening_sources_present: uart=%s test_app=%s" % (
        os.path.isfile(HWTEST_SOURCE),
        os.path.isfile(TEST_APP_SOURCE)))
    add("flash_tool_present: %s" % os.path.isfile(UART_DOWNLOAD_EXE))
    add("jsroot_resource: present=%s address=%s path=%s" % (
        os.path.isfile(JSROOT_PACKED_BIN),
        JSROOT_FLASH_ADDRESS, JSROOT_PACKED_BIN))
    add("ble_helpers: mac=%s rssi=%s registry=%s" % (
        normalize_ble_mac("c0:12:34:56:78:9a"),
        normalize_rssi_threshold("-75"),
        MAC_REGISTRY_FILE))
    add("serial_port_regex_ok: %s / %s" % (
        normalize_serial_port("com12") == "COM12",
        normalize_serial_port("COM12 & whoami") is None))
    add("hcpu_port_derivation_ok: %s" % (
        hcpu_port_from_flash("COM4") == "COM3"))
    add("detected_serial_ports: %s" % (", ".join(list_serial_ports()) or "none"))

    try:
        import tkinter  # noqa: F401
        add("tkinter_importable: True")
    except Exception as e:
        add("tkinter_importable: False (%r)" % e)
    try:
        import bleak  # noqa: F401
        add("bleak_importable: True")
    except Exception as e:
        add("bleak_importable: False (%r)" % e)

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
    SIG_HWTEST_RESULT = "hwtest_result"
    SIG_HWTEST_ACTIVE = "hwtest_active"
    SIG_BLE_RESULT = "ble_result"
    SIG_RESOURCE_RESULT = "resource_result"

    class App:
        def __init__(self, root):
            self.root = root
            self.q = queue.Queue()
            self.busy = False
            self.hwtest_cancel_event = None
            root.title("Skaiwalk 手錶韌體發布與燒錄工具")
            root.geometry("900x800")
            root.minsize(760, 650)

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
            default_flash_port = suggested_flash_port(ports)
            self.port_var = tk.StringVar(value=default_flash_port)
            self.port_combo = ttk.Combobox(
                flash, textvariable=self.port_var, values=ports,
                width=14, font=UI_FONT)
            self.port_combo.grid(row=0, column=1, sticky="w", pady=6)
            self.port_combo.bind(
                "<<ComboboxSelected>>", self.sync_hcpu_from_flash)
            self.port_combo.bind(
                "<FocusOut>", self.sync_hcpu_from_flash, add="+")
            self.port_combo.bind(
                "<Return>", self.sync_hcpu_from_flash, add="+")
            self.btn_port_refresh = ttk.Button(
                flash, text="重新偵測", command=self.refresh_ports)
            self.btn_port_refresh.grid(row=0, column=2, padx=6, pady=6)
            self.btn_flash = ttk.Button(
                flash, text="刷入手錶", command=self.do_flash)
            self.btn_flash.grid(row=0, column=3, padx=6, pady=6)
            self.btn_resource = ttk.Button(
                flash, text="刷入圖片資源", command=self.do_flash_resource)
            self.btn_resource.grid(row=0, column=4, padx=6, pady=6)

            ttk.Label(flash, text="HCPU 篩檢 COM：", font=UI_FONT).grid(
                row=1, column=0, sticky="w", padx=6, pady=6)
            self.hcpu_port_var = tk.StringVar(
                value=suggested_hcpu_port(ports, default_flash_port))
            self.hcpu_port_combo = ttk.Combobox(
                flash, textvariable=self.hcpu_port_var, values=ports,
                width=14, font=UI_FONT)
            self.hcpu_port_combo.grid(row=1, column=1, sticky="w", pady=6)
            self.btn_hwtest = ttk.Button(
                flash, text="板級篩檢", command=self.do_hwtest)
            self.btn_hwtest.grid(row=1, column=2, padx=6, pady=6)
            self.btn_hwtest_cancel = ttk.Button(
                flash, text="取消篩檢", command=self.cancel_hwtest,
                state="disabled")
            self.btn_hwtest_cancel.grid(row=1, column=3, padx=6, pady=6)
            self.btn_ble_test = ttk.Button(
                flash, text="BLE 配號＋連線測試",
                command=self.do_ble_provisioning_test)
            self.btn_ble_test.grid(row=1, column=4, padx=6, pady=6)
            ttk.Label(flash, text="固定治具 RSSI 門檻：",
                      font=UI_FONT).grid(
                row=2, column=0, sticky="w", padx=6, pady=4)
            self.rssi_var = tk.StringVar(
                value=str(DEFAULT_BLE_RSSI_THRESHOLD))
            ttk.Entry(
                flash, textvariable=self.rssi_var,
                width=8, font=UI_FONT).grid(
                row=2, column=1, sticky="w", pady=4)
            ttk.Label(flash, text="dBm（請用良品校正）",
                      font=UI_FONT, foreground="#666").grid(
                row=2, column=2, columnspan=2, sticky="w", padx=6)
            ttk.Label(
                flash,
                text=("(刷入手錶後自動：板級篩檢 → BLE 配號 → "
                      "advertising/RSSI → 連線讀取 Battery Level；"
                      "圖片資源獨立刷入 0x64280000)"),
                font=UI_FONT, foreground="#666").grid(
                    row=3, column=0, columnspan=6, sticky="w", padx=6)

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
                             self.btn_flash, self.btn_resource,
                             self.btn_hwtest, self.btn_ble_test,
                             self.btn_upload]

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
            if not busy:
                self.btn_hwtest_cancel.configure(state="disabled")
                self.hwtest_cancel_event = None

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
            current_hcpu = self.hcpu_port_var.get().strip()
            self.port_combo.configure(values=ports)
            self.hcpu_port_combo.configure(values=ports)
            if not current and ports:
                current = suggested_flash_port(ports)
                self.port_var.set(current)
            if not current_hcpu and ports:
                self.hcpu_port_var.set(suggested_hcpu_port(ports, current))
            if ports:
                self._emit("偵測到 COM Port: %s" % ", ".join(ports))
            else:
                self._emit("未自動偵測到 COM Port；可手動輸入，例如 COM12。")

        def sync_hcpu_from_flash(self, _event=None):
            hcpu_port = hcpu_port_from_flash(self.port_var.get())
            if hcpu_port:
                self.hcpu_port_var.set(hcpu_port)
                self._emit("已依燒錄 COM 自動選擇 HCPU COM：%s" % hcpu_port)

        def selected_rssi_threshold(self):
            threshold = normalize_rssi_threshold(self.rssi_var.get())
            if threshold is None:
                messagebox.showerror(
                    "RSSI 門檻格式錯誤",
                    "請輸入 -100 到 -20 之間的整數，例如 -75。")
            return threshold

        # --- queue pump (runs on UI thread) ---
        def _pump(self):
            try:
                while True:
                    kind, payload = self.q.get_nowait()
                    if kind == SIG_LOG:
                        self.append(payload)
                    elif kind == SIG_REFRESH:
                        self.refresh()
                    elif kind == SIG_HWTEST_RESULT:
                        ok, detail, cancelled = payload
                        if cancelled:
                            messagebox.showinfo("板級篩檢已取消", detail)
                        elif ok:
                            messagebox.showinfo("板級篩檢通過", detail)
                        else:
                            messagebox.showerror("板級篩檢失敗", detail)
                    elif kind == SIG_HWTEST_ACTIVE:
                        self.hwtest_cancel_event = payload
                        self.btn_hwtest_cancel.configure(state="normal")
                    elif kind == SIG_BLE_RESULT:
                        ok, detail = payload
                        if ok:
                            messagebox.showinfo("BLE 配號與連線測試通過", detail)
                        else:
                            messagebox.showerror("BLE 配號與連線測試失敗", detail)
                    elif kind == SIG_RESOURCE_RESULT:
                        ok, detail = payload
                        if ok:
                            messagebox.showinfo("圖片資源刷入完成", detail)
                        else:
                            messagebox.showerror("圖片資源刷入失敗", detail)
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
            build_rc = self._stream(
                ["cmd", "/c", "_watch_build.cmd", "-j8"],
                extra_env={"WATCH_BOARD": board})
            if build_rc != 0 or build_failed():
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
            flash_port = normalize_serial_port(self.port_var.get())
            if not flash_port:
                messagebox.showerror(
                    "燒錄 COM 格式錯誤",
                    "請選擇或輸入有效的燒錄 COM，例如 COM4。")
                return
            hcpu_port = normalize_serial_port(self.hcpu_port_var.get())
            if not hcpu_port:
                hcpu_port = hcpu_port_from_flash(flash_port)
                if hcpu_port:
                    self.hcpu_port_var.set(hcpu_port)
            if not hcpu_port:
                messagebox.showerror(
                    "HCPU COM 格式錯誤",
                    "請選擇或輸入有效的 HCPU 篩檢 COM，例如 COM3。")
                return
            rssi_threshold = self.selected_rssi_threshold()
            if rssi_threshold is None:
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

            preflight_error = flash_preflight_error(board)
            if preflight_error:
                messagebox.showerror("韌體尚未準備完成", preflight_error)
                return

            self.port_var.set(flash_port)
            self.hcpu_port_var.set(hcpu_port)
            if not messagebox.askyesno(
                    "確認刷機",
                    "晶片型號：%s\n燒錄 COM：%s\nHCPU 篩檢 COM：%s\n\n"
                    "刷機完成後會執行板級篩檢、BLE 配號、RSSI 與連線測試。\n"
                    "固定治具 RSSI 門檻：%d dBm\n\n"
                    "確定要繼續嗎？" %
                    (board, flash_port, hcpu_port, rssi_threshold)):
                return
            cancel_event = threading.Event()
            self._start(lambda: self._w_flash(
                board, flash_port, hcpu_port, build_dir, cancel_event,
                rssi_threshold))

        def _w_flash(self, board, flash_port, hcpu_port, build_dir,
                     cancel_event, rssi_threshold):
            self._emit("=== 燒錄韌體：%s → %s ===" % (
                board, flash_port))
            rc = self._stream(flash_command(flash_port), cwd=build_dir)
            if rc == 0:
                self._emit("=== 刷機完成：%s ===" % flash_port)
                self.q.put((SIG_HWTEST_ACTIVE, cancel_event))
                cancelled = False
                try:
                    wait_for_system_boot(
                        POST_FLASH_BOOT_WAIT_SECONDS,
                        self._emit, cancel_event)
                    self._emit(
                        "=== 切換至 HCPU 篩檢 COM：%s ===" % hcpu_port)
                    self._emit(
                        "=== 開始 UART 板級篩檢"
                        "（CPU/RAM、Flash、電池、IMU、PPG） ===")
                    _, expected_version, _ = read_status()
                    ok, detail = run_hardware_selftest(
                        hcpu_port, self._emit,
                        expected_version=expected_version,
                        timeout=HWTEST_SCREEN_TIMEOUT_SECONDS,
                        cancel_event=cancel_event, screening_only=True)
                    if ok:
                        self._emit(
                            "=== 板級篩檢通過，開始 BLE 配號與連線測試 ===")
                        mac, ble_result = run_ble_provisioning_test(
                            hcpu_port, rssi_threshold, self._emit,
                            board, expected_version,
                            generate_new_mac=True,
                            cancel_event=cancel_event)
                        detail = (
                            "板級篩檢與 BLE 測試通過\n"
                            "MAC=%s\nRSSI=%d dBm\nBattery Level=%d%%"
                            % (mac, ble_result["rssi"],
                               ble_result["battery_level"]))
                except HardwareSelftestCancelled as e:
                    cancelled = True
                    ok, detail = False, str(e)
                except Exception as e:
                    ok, detail = False, str(e)
                if cancelled:
                    self._emit("=== 板級篩檢已取消 ===")
                elif ok:
                    self._emit("=== 板級篩檢通過：此板可進入互動測試 ===")
                else:
                    self._emit("=== 板級篩檢失敗：請隔離此板 ===")
                    self._emit("失敗原因：%s" % detail)
                self.q.put((SIG_HWTEST_RESULT, (ok, detail, cancelled)))
            else:
                self._emit("!! 刷機失敗(結束碼 %d)。請查看 %s" % (
                    rc, os.path.join(build_dir, "ImgBurn.log")))
            self.q.put((SIG_DONE, None))

        def do_flash_resource(self):
            flash_port = normalize_serial_port(self.port_var.get())
            if not flash_port:
                messagebox.showerror(
                    "燒錄 COM 格式錯誤",
                    "請選擇或輸入有效的燒錄 COM，例如 COM4。")
                return
            if not os.path.isfile(UART_DOWNLOAD_EXE):
                messagebox.showerror(
                    "缺少燒錄工具", "找不到：\n%s" % UART_DOWNLOAD_EXE)
                return
            if not os.path.isfile(JSROOT_PACKED_BIN):
                messagebox.showerror(
                    "缺少圖片資源",
                    "找不到：\n%s\n\n請先產生 jsroot_packed.bin。"
                    % JSROOT_PACKED_BIN)
                return
            build_dir = flash_build_dir(self.board_var.get())
            if not os.path.isdir(build_dir):
                messagebox.showerror(
                    "缺少建置目錄", "找不到：\n%s" % build_dir)
                return
            if not messagebox.askyesno(
                    "確認刷入圖片資源",
                    "檔案：%s\n位址：%s\n燒錄 COM：%s\n\n確定刷入嗎？"
                    % (JSROOT_PACKED_BIN, JSROOT_FLASH_ADDRESS, flash_port)):
                return
            self._start(lambda: self._w_flash_resource(
                flash_port, build_dir))

        def _w_flash_resource(self, flash_port, build_dir):
            burn_list = None
            try:
                burn_list = create_resource_burn_list(build_dir)
                self._emit(
                    "=== 刷入圖片資源：%s → %s（%s） ===" %
                    (JSROOT_PACKED_BIN, JSROOT_FLASH_ADDRESS, flash_port))
                rc = self._stream(
                    flash_command(flash_port, os.path.basename(burn_list)),
                    cwd=build_dir)
                if rc == 0:
                    detail = "%s 已刷入 %s" % (
                        os.path.basename(JSROOT_PACKED_BIN),
                        JSROOT_FLASH_ADDRESS)
                    self._emit("=== 圖片資源刷入完成 ===")
                    self.q.put((SIG_RESOURCE_RESULT, (True, detail)))
                else:
                    detail = (
                        "燒錄工具結束碼 %d，請查看 %s" %
                        (rc, os.path.join(build_dir, "ImgBurn.log")))
                    self._emit("!! 圖片資源刷入失敗：%s" % detail)
                    self.q.put((SIG_RESOURCE_RESULT, (False, detail)))
            except Exception as e:
                detail = str(e)
                self._emit("!! 圖片資源刷入例外：%s" % detail)
                self.q.put((SIG_RESOURCE_RESULT, (False, detail)))
            finally:
                if burn_list and os.path.isfile(burn_list):
                    try:
                        os.remove(burn_list)
                    except OSError:
                        pass
                self.q.put((SIG_DONE, None))

        def do_ble_provisioning_test(self):
            hcpu_port = normalize_serial_port(self.hcpu_port_var.get())
            if not hcpu_port:
                messagebox.showerror(
                    "HCPU COM 格式錯誤",
                    "請選擇或輸入有效的 HCPU COM，例如 COM3。")
                return
            rssi_threshold = self.selected_rssi_threshold()
            if rssi_threshold is None:
                return
            if not messagebox.askyesno(
                    "確認 BLE 配號與連線測試",
                    "HCPU COM：%s\nRSSI 門檻：%d dBm\n\n"
                    "這會寫入新的 BLE MAC 並重新啟動手錶，確定繼續嗎？"
                    % (hcpu_port, rssi_threshold)):
                return
            board = self.board_var.get()
            _, version, _ = read_status()
            self._start(lambda: self._w_ble_provisioning_test(
                hcpu_port, rssi_threshold, board, version))

        def _w_ble_provisioning_test(
                self, hcpu_port, rssi_threshold, board, version):
            try:
                mac, result = run_ble_provisioning_test(
                    hcpu_port, rssi_threshold, self._emit,
                    board, version, generate_new_mac=True)
                detail = (
                    "MAC=%s\nRSSI=%d dBm\nBattery Level=%d%%" %
                    (mac, result["rssi"], result["battery_level"]))
                self._emit("=== BLE 配號與連線測試通過 ===")
                self.q.put((SIG_BLE_RESULT, (True, detail)))
            except Exception as e:
                detail = str(e)
                self._emit("=== BLE 配號與連線測試失敗 ===")
                self._emit("失敗原因：%s" % detail)
                self.q.put((SIG_BLE_RESULT, (False, detail)))
            self.q.put((SIG_DONE, None))

        def do_hwtest(self):
            hcpu_port = normalize_serial_port(self.hcpu_port_var.get())
            if not hcpu_port:
                messagebox.showerror(
                    "HCPU COM 格式錯誤",
                    "請選擇或輸入有效的 HCPU 篩檢 COM，例如 COM3。")
                return
            rssi_threshold = self.selected_rssi_threshold()
            if rssi_threshold is None:
                return
            self.hcpu_port_var.set(hcpu_port)
            board = self.board_var.get()
            cancel_event = threading.Event()
            self.hwtest_cancel_event = cancel_event
            self._start(lambda: self._w_hwtest(
                hcpu_port, cancel_event, rssi_threshold, board))
            self.btn_hwtest_cancel.configure(state="normal")

        def cancel_hwtest(self):
            if self.hwtest_cancel_event is None:
                return
            self.hwtest_cancel_event.set()
            self.btn_hwtest_cancel.configure(state="disabled")
            self._emit("=== 正在取消板級篩檢… ===")

        def _w_hwtest(self, port, cancel_event, rssi_threshold, board):
            self._emit("=== 板級篩檢：%s（最長 %d 秒） ===" % (
                port, HWTEST_SCREEN_TIMEOUT_SECONDS))
            cancelled = False
            try:
                _, expected_version, _ = read_status()
                ok, detail = run_hardware_selftest(
                    port, self._emit, expected_version=expected_version,
                    timeout=HWTEST_SCREEN_TIMEOUT_SECONDS,
                    cancel_event=cancel_event, screening_only=True)
                if ok:
                    self._emit(
                        "=== 驗證現有 BLE MAC、advertising、RSSI 與 GATT ===")
                    mac, ble_result = run_ble_provisioning_test(
                        port, rssi_threshold, self._emit,
                        board, expected_version,
                        generate_new_mac=False,
                        cancel_event=cancel_event)
                    detail = (
                        "板級篩檢與 BLE 測試通過\n"
                        "MAC=%s\nRSSI=%d dBm\nBattery Level=%d%%"
                        % (mac, ble_result["rssi"],
                           ble_result["battery_level"]))
            except HardwareSelftestCancelled as e:
                cancelled = True
                ok, detail = False, str(e)
            except Exception as e:
                ok, detail = False, str(e)
            if cancelled:
                self._emit("=== 板級篩檢已取消 ===")
            else:
                self._emit("=== 板級篩檢%s ===" % ("通過" if ok else "失敗"))
                if not ok:
                    self._emit("失敗原因：%s" % detail)
            self.q.put((SIG_HWTEST_RESULT, (ok, detail, cancelled)))
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
