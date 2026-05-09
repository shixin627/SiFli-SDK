"""Send keystrokes to running main.exe console (PC sim) for tshell automation.

Uses AttachConsole + WriteConsoleInputW so we don't need to focus the window
or steal foreground. Safe to call from non-console contexts.

Usage:
    python _send_to_main.py "onboarding\r"
    python _send_to_main.py "help\r"
"""

import ctypes
from ctypes import wintypes
import sys
import time

kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

INVALID_HANDLE_VALUE = wintypes.HANDLE(-1).value
GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 1
FILE_SHARE_WRITE = 2
OPEN_EXISTING = 3


class KEY_EVENT_RECORD(ctypes.Structure):
    _fields_ = [
        ("bKeyDown", wintypes.BOOL),
        ("wRepeatCount", wintypes.WORD),
        ("wVirtualKeyCode", wintypes.WORD),
        ("wVirtualScanCode", wintypes.WORD),
        ("UnicodeChar", wintypes.WCHAR),
        ("dwControlKeyState", wintypes.DWORD),
    ]


class INPUT_RECORD(ctypes.Structure):
    class _U(ctypes.Union):
        _fields_ = [("KeyEvent", KEY_EVENT_RECORD)]

    _anonymous_ = ("u",)
    _fields_ = [("EventType", wintypes.WORD), ("u", _U)]


def find_main_pid():
    import subprocess

    out = subprocess.check_output(
        ["wmic", "process", "where", "name='main.exe'", "get", "ProcessId"],
        text=True,
    )
    for line in out.splitlines():
        line = line.strip()
        if line.isdigit():
            return int(line)
    return None


user32 = ctypes.WinDLL("user32", use_last_error=True)
MAPVK_VK_TO_VSC = 0
SHIFT_PRESSED = 0x10


def send(pid: int, text: str) -> None:
    kernel32.FreeConsole()
    if not kernel32.AttachConsole(pid):
        raise OSError(f"AttachConsole({pid}) failed: {ctypes.get_last_error()}")
    h = kernel32.CreateFileW(
        "CONIN$",
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        None,
        OPEN_EXISTING,
        0,
        None,
    )
    if h == INVALID_HANDLE_VALUE:
        raise OSError(f"CreateFile CONIN$ failed: {ctypes.get_last_error()}")
    try:
        for ch in text:
            # VkKeyScanW returns: low byte = VK code, high byte = shift state
            vk_scan = user32.VkKeyScanW(ord(ch))
            vk = vk_scan & 0xFF
            shift = (vk_scan >> 8) & 0xFF
            ctrl_state = SHIFT_PRESSED if (shift & 1) else 0
            scan = user32.MapVirtualKeyW(vk, MAPVK_VK_TO_VSC) & 0xFFFF
            # Special case for \r: VK_RETURN = 0x0D
            if ch == "\r":
                vk = 0x0D
                scan = user32.MapVirtualKeyW(0x0D, MAPVK_VK_TO_VSC) & 0xFFFF
            recs = (INPUT_RECORD * 2)()
            recs[0].EventType = 1
            recs[0].KeyEvent.bKeyDown = True
            recs[0].KeyEvent.wRepeatCount = 1
            recs[0].KeyEvent.wVirtualKeyCode = vk
            recs[0].KeyEvent.wVirtualScanCode = scan
            recs[0].KeyEvent.UnicodeChar = ch
            recs[0].KeyEvent.dwControlKeyState = ctrl_state
            recs[1].EventType = 1
            recs[1].KeyEvent.bKeyDown = False
            recs[1].KeyEvent.wRepeatCount = 1
            recs[1].KeyEvent.wVirtualKeyCode = vk
            recs[1].KeyEvent.wVirtualScanCode = scan
            recs[1].KeyEvent.UnicodeChar = ch
            recs[1].KeyEvent.dwControlKeyState = ctrl_state
            written = wintypes.DWORD()
            ok = kernel32.WriteConsoleInputW(h, recs, 2, ctypes.byref(written))
            if not ok:
                raise OSError(
                    f"WriteConsoleInputW failed at {ch!r}: {ctypes.get_last_error()}"
                )
            time.sleep(0.03)
    finally:
        kernel32.CloseHandle(h)
        kernel32.FreeConsole()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('usage: python _send_to_main.py "<text>"')
        sys.exit(1)
    text = sys.argv[1].encode().decode("unicode_escape")  # process \r etc.
    pid = find_main_pid()
    if pid is None:
        print("main.exe not running")
        sys.exit(2)
    print(f"sending {text!r} to PID {pid}")
    send(pid, text)
    print("done")
