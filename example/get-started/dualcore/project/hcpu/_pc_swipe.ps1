# Drive the LVGL sim window with synthetic mouse events via SendInput.
#
# Usage:
#   ./_pc_swipe.ps1 -dir up        # swipe finger up    (bottom→top)
#   ./_pc_swipe.ps1 -dir down      # swipe finger down  (top→bottom)
#   ./_pc_swipe.ps1 -dir left      # swipe finger left  (right→left)
#   ./_pc_swipe.ps1 -dir right     # swipe finger right (left→right)
#   ./_pc_swipe.ps1 -tap -x 50 -y 50          # tap at client% (x,y) ∈ [0,100]
#   ./_pc_swipe.ps1 -tap -px 250 -py 250      # tap at LVGL pixel (0..465)
#   ./_pc_swipe.ps1 -dir right -edge          # hug the very left edge
#                                              (lvsf_gesture back-bar)
#
# Coords are computed from the LVGL window's GetWindowRect (physical
# screen) so it works on hi-DPI / scaled displays.

param(
    [string]$dir,
    [switch]$tap,
    [int]$x = -1, [int]$y = -1,
    [int]$px = -1, [int]$py = -1,
    [switch]$edge,
    [int]$steps = 25,
    [int]$stepDelayMs = 18
)

Add-Type @'
using System;
using System.Runtime.InteropServices;
public class PcSwipe {
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint lpdwProcessId);
    [DllImport("user32.dll")] public static extern bool AttachThreadInput(uint idAttach, uint idAttachTo, bool fAttach);
    [DllImport("kernel32.dll")] public static extern uint GetCurrentThreadId();
    [DllImport("user32.dll", SetLastError=true)] public static extern bool GetWindowRect(IntPtr hWnd, out RECT lpRect);
    [DllImport("user32.dll")] public static extern int GetSystemMetrics(int nIndex);
    [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
    [DllImport("user32.dll")] public static extern void mouse_event(uint dwFlags, uint dx, uint dy, uint dwData, IntPtr dwExtraInfo);
    /* DPI awareness: PowerShell.exe is per-monitor-aware-v2 actually (Win11) but
     * .NET runtime can revert. Force per-monitor-v2 so coords are physical px. */
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("shcore.dll")] public static extern int SetProcessDpiAwareness(int v);

    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }

    public const uint INPUT_MOUSE = 0;
    public const uint MOUSEEVENTF_MOVE        = 0x0001;
    public const uint MOUSEEVENTF_LEFTDOWN    = 0x0002;
    public const uint MOUSEEVENTF_LEFTUP      = 0x0004;
    public const uint MOUSEEVENTF_ABSOLUTE    = 0x8000;
    public const uint MOUSEEVENTF_VIRTUALDESK = 0x4000;
    public const int  SW_RESTORE = 9;
    public const int  SM_XVIRTUALSCREEN = 76;
    public const int  SM_YVIRTUALSCREEN = 77;
    public const int  SM_CXVIRTUALSCREEN = 78;
    public const int  SM_CYVIRTUALSCREEN = 79;
}
'@

# Make this PowerShell process DPI aware so SetCursorPos uses physical
# screen pixels, matching what GetWindowRect returned.
try { [PcSwipe]::SetProcessDpiAwareness(2) | Out-Null } catch {}
[PcSwipe]::SetProcessDPIAware() | Out-Null

function Force-ToForeground([IntPtr]$hwnd) {
    [PcSwipe]::ShowWindow($hwnd, [PcSwipe]::SW_RESTORE) | Out-Null
    $junk = [uint32]0
    $fg = [PcSwipe]::GetForegroundWindow()
    $tidFg = [PcSwipe]::GetWindowThreadProcessId($fg, [ref]$junk)
    $tidMe = [PcSwipe]::GetCurrentThreadId()
    [PcSwipe]::AttachThreadInput($tidMe, $tidFg, $true) | Out-Null
    [PcSwipe]::SetForegroundWindow($hwnd) | Out-Null
    [PcSwipe]::AttachThreadInput($tidMe, $tidFg, $false) | Out-Null
    Start-Sleep -Milliseconds 250
}

# Use GetWindowRect for screen coords (physical pixels). LVGL window has
# a small title bar (~CYCAPTION) — subtract roughly to get client area.
function Get-WindowGeometry([IntPtr]$hwnd) {
    $wr = New-Object PcSwipe+RECT
    [PcSwipe]::GetWindowRect($hwnd, [ref]$wr) | Out-Null
    # Title bar: small window has thin caption ~22-32px (DPI-dependent).
    # Border ~1-8px each side. Approximate: 25px caption, 8px sides.
    $cap = 25; $side = 8
    return @{
        X = $wr.Left + $side
        Y = $wr.Top + $cap
        W = ($wr.Right - $wr.Left) - 2 * $side
        H = ($wr.Bottom - $wr.Top) - $cap - $side
    }
}

# Convert physical screen pixel → 0..65535 absolute coord (virtual desktop)
$script:vsx = [PcSwipe]::GetSystemMetrics([PcSwipe]::SM_XVIRTUALSCREEN)
$script:vsy = [PcSwipe]::GetSystemMetrics([PcSwipe]::SM_YVIRTUALSCREEN)
$script:vsw = [PcSwipe]::GetSystemMetrics([PcSwipe]::SM_CXVIRTUALSCREEN)
$script:vsh = [PcSwipe]::GetSystemMetrics([PcSwipe]::SM_CYVIRTUALSCREEN)
function To-Absolute([int]$x, [int]$y) {
    $ax = [int]((($x - $script:vsx) * 65535.0) / $script:vsw)
    $ay = [int]((($y - $script:vsy) * 65535.0) / $script:vsh)
    return @{ X = $ax; Y = $ay }
}
function Send-MouseInput([int]$x, [int]$y, [uint32]$flags) {
    if ($flags -band [PcSwipe]::MOUSEEVENTF_MOVE) {
        # For moves we need cursor at the target before pressing/releasing.
        [PcSwipe]::SetCursorPos($x, $y) | Out-Null
        if ($flags -eq [PcSwipe]::MOUSEEVENTF_MOVE) { return }
    }
    [PcSwipe]::SetCursorPos($x, $y) | Out-Null
    [PcSwipe]::mouse_event($flags, 0, 0, 0, [IntPtr]::Zero)
}
function Drag-FromTo([int]$sx, [int]$sy, [int]$ex, [int]$ey, [int]$nSteps, [int]$delayMs) {
    Send-MouseInput $sx $sy ([PcSwipe]::MOUSEEVENTF_MOVE)
    Start-Sleep -Milliseconds 80
    Send-MouseInput $sx $sy ([PcSwipe]::MOUSEEVENTF_LEFTDOWN)
    Start-Sleep -Milliseconds 30
    for ($i = 1; $i -le $nSteps; $i++) {
        $cx = [int]($sx + ($ex - $sx) * $i / $nSteps)
        $cy = [int]($sy + ($ey - $sy) * $i / $nSteps)
        Send-MouseInput $cx $cy ([PcSwipe]::MOUSEEVENTF_MOVE)
        Start-Sleep -Milliseconds $delayMs
    }
    Start-Sleep -Milliseconds 50
    Send-MouseInput $ex $ey ([PcSwipe]::MOUSEEVENTF_LEFTUP)
    Start-Sleep -Milliseconds 50
}

# ---- main ----
$proc = Get-Process -Name "main" -ErrorAction Stop
$hwnd = $proc.MainWindowHandle
Force-ToForeground $hwnd

$script:g = Get-WindowGeometry $hwnd
"client area: top-left=($($g.X),$($g.Y)) size=$($g.W)x$($g.H)"

function ClientPercent-To-Screen([int]$xpct, [int]$ypct) {
    return @{ X = $g.X + [int]($g.W * $xpct / 100); Y = $g.Y + [int]($g.H * $ypct / 100) }
}
function LvglPx-To-Screen([int]$lx, [int]$ly) {
    return @{ X = $g.X + [int]($g.W * $lx / 466); Y = $g.Y + [int]($g.H * $ly / 466) }
}

if ($tap) {
    if ($px -ge 0 -and $py -ge 0) { $p = LvglPx-To-Screen $px $py }
    elseif ($x -ge 0 -and $y -ge 0) { $p = ClientPercent-To-Screen $x $y }
    else { $p = ClientPercent-To-Screen 50 50 }
    "tap screen ($($p.X),$($p.Y))"
    Send-MouseInput $p.X $p.Y ([PcSwipe]::MOUSEEVENTF_MOVE)
    Start-Sleep -Milliseconds 80
    Send-MouseInput $p.X $p.Y ([PcSwipe]::MOUSEEVENTF_LEFTDOWN)
    Start-Sleep -Milliseconds 80
    Send-MouseInput $p.X $p.Y ([PcSwipe]::MOUSEEVENTF_LEFTUP)
    return
}

switch ($dir.ToLower()) {
    "up"    { $a = ClientPercent-To-Screen 50 80; $b = ClientPercent-To-Screen 50 30 }
    "down"  { $a = ClientPercent-To-Screen 50 20; $b = ClientPercent-To-Screen 50 75 }
    "left"  {
        if ($edge) { $a = ClientPercent-To-Screen 95 50; $b = ClientPercent-To-Screen 25 50 }
        else       { $a = ClientPercent-To-Screen 85 50; $b = ClientPercent-To-Screen 15 50 }
    }
    "right" {
        if ($edge) { $a = ClientPercent-To-Screen 2  50; $b = ClientPercent-To-Screen 70 50 }
        else       { $a = ClientPercent-To-Screen 15 50; $b = ClientPercent-To-Screen 85 50 }
    }
    default { Write-Error "invalid -dir '$dir' (want: up|down|left|right)"; exit 1 }
}
"swipe $dir : ($($a.X),$($a.Y)) -> ($($b.X),$($b.Y))"
Drag-FromTo $a.X $a.Y $b.X $b.Y $steps $stepDelayMs
"done"
