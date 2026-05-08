# Drive the LVGL sim window via SendInput (real system mouse events).
# Argument: direction = "left" or "right"
param([string]$direction = "left")

Add-Type @'
using System;
using System.Runtime.InteropServices;
public class W {
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll", SetLastError=true)] public static extern bool GetWindowRect(IntPtr hWnd, out RECT lpRect);
    [DllImport("user32.dll", SetLastError=true)] public static extern bool GetClientRect(IntPtr hWnd, out RECT lpRect);
    [DllImport("user32.dll", SetLastError=true)] public static extern bool ClientToScreen(IntPtr hWnd, ref POINT lpPoint);
    [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
    [DllImport("user32.dll")] public static extern void mouse_event(uint dwFlags, uint dx, uint dy, uint dwData, IntPtr dwExtraInfo);
    [StructLayout(LayoutKind.Sequential)]
    public struct RECT { public int Left, Top, Right, Bottom; }
    [StructLayout(LayoutKind.Sequential)]
    public struct POINT { public int X, Y; }
    public const uint MOUSEEVENTF_LEFTDOWN = 0x0002;
    public const uint MOUSEEVENTF_LEFTUP   = 0x0004;
}
'@

$proc = Get-Process -Name "main" -ErrorAction Stop
$hwnd = $proc.MainWindowHandle
[W]::SetForegroundWindow($hwnd) | Out-Null
Start-Sleep -Milliseconds 400

# Get client area in screen coords
$cr = New-Object W+RECT
[W]::GetClientRect($hwnd, [ref]$cr) | Out-Null
$tl = New-Object W+POINT
$tl.X = 0; $tl.Y = 0
[W]::ClientToScreen($hwnd, [ref]$tl) | Out-Null
$cw = $cr.Right - $cr.Left
$ch = $cr.Bottom - $cr.Top
"client: top-left=($($tl.X),$($tl.Y)) size=$($cw)x$($ch)"

# Pick start/end in screen coords. Mid-vertical, swipe horizontally.
$cy = $tl.Y + [int]($ch / 2)
if ($direction -eq "left") {
    $sx = $tl.X + [int]($cw * 0.85); $ex = $tl.X + [int]($cw * 0.15)
} else {
    $sx = $tl.X + [int]($cw * 0.15); $ex = $tl.X + [int]($cw * 0.85)
}
"swipe screen: ($sx,$cy) -> ($ex,$cy)"

# Move to start, mouse down
[W]::SetCursorPos($sx, $cy) | Out-Null
Start-Sleep -Milliseconds 100
[W]::mouse_event([W]::MOUSEEVENTF_LEFTDOWN, 0, 0, 0, [IntPtr]::Zero)
Start-Sleep -Milliseconds 50

# Drag in 30 small steps
for ($i = 1; $i -le 30; $i++) {
    $x = [int]($sx + ($ex - $sx) * $i / 30)
    [W]::SetCursorPos($x, $cy) | Out-Null
    Start-Sleep -Milliseconds 20
}
Start-Sleep -Milliseconds 50
[W]::mouse_event([W]::MOUSEEVENTF_LEFTUP, 0, 0, 0, [IntPtr]::Zero)
"done"
