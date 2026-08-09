# Dualcore PC simulator dev-loop helper.
#
# Usage:
#   ./_dev_test.ps1                              rebuild + restart sim, leave it running
#   ./_dev_test.ps1 -nobuild                     just restart sim (skip scons)
#   ./_dev_test.ps1 -cmd "goto_app weather"      send one MSH command
#   ./_dev_test.ps1 -script path\to\file.txt     read newline-separated cmds from a file
#                                                ('#' = comment, blank lines ignored)
#   ./_dev_test.ps1 -screenshot out.png          take a screenshot of the sim window
#   ./_dev_test.ps1 -snapshot apps               capture every builtin app into
#                                                _snapshots/<name>/<app_id>.png
#
# For multiple commands in one run, use -script. Passing multiple loose
# args through cmd /c hits PowerShell positional-binding quirks.

param(
    [switch]$nobuild,
    [string]$cmd,
    [string]$script,
    [string]$screenshot,
    [string]$snapshot
)

$ErrorActionPreference = 'Stop'
$proj = $PSScriptRoot
$exe  = Join-Path $proj 'build_pc_hcpu\main.exe'
# env install dir differs per machine (env_latest / env) -- probe, don't pin.
$py = @($env:ENV_ROOT, 'C:\dev\env_latest', 'C:\dev\env') |
    Where-Object { $_ } |
    ForEach-Object { Join-Path $_ 'tools\python-3.11.9-amd64\python.exe' } |
    Where-Object { Test-Path $_ } |
    Select-Object -First 1
if (-not $py) { throw 'SiFli env python not found. Set ENV_ROOT to the env install dir.' }
$send = Join-Path $proj '_send_to_main.py'

function Kill-Sim {
    Get-Process -Name 'main' -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Milliseconds 500
}

function Build-Sim {
    Write-Host '[build] scons --board=pc -j8' -ForegroundColor Cyan
    cmd /c "$proj\_pc_build.cmd -j8" 2>&1 | Out-Null
    $err = Select-String -Path "$proj\_pc_build.log" -Pattern 'error LNK|error C\d|fatal error' |
           Select-Object -First 5
    if ($err) { Write-Host '[build] FAILED:' -ForegroundColor Red; $err | ForEach-Object { Write-Host $_.Line }; throw 'build failed' }
    if (-not (Select-String -Path "$proj\_pc_build.log" -Pattern 'scons: done building')) {
        throw 'build did not complete'
    }
    Write-Host '[build] OK' -ForegroundColor Green
}

function Start-Sim {
    if (-not (Test-Path $exe)) { throw "no main.exe at $exe -- build first" }
    $p = Start-Process -FilePath $exe -PassThru
    Write-Host "[sim] launched pid=$($p.Id)" -ForegroundColor Cyan
    # Wait for the LVGL window + tshell to come up. 5 s is enough on this
    # machine; bump if you see "main.exe not running" from _send_to_main.py.
    Start-Sleep -Seconds 5
    if (-not (Get-Process -Id $p.Id -ErrorAction SilentlyContinue)) {
        throw "sim died at boot (pid=$($p.Id))"
    }
    return $p
}

function Send-Cmd($line) {
    if (-not $line.EndsWith("`r")) { $line += "`r" }
    & $py $send $line | Out-Null
    Start-Sleep -Milliseconds 600
}

function Read-Console {
    # Attach to main.exe's console and dump the visible screen buffer. The
    # python helper is inline so the function is self-contained.
    $reader = @'
import ctypes, sys, subprocess
from ctypes import wintypes
out = subprocess.check_output(["wmic","process","where","name='main.exe'","get","ProcessId"], text=True)
pid = next((int(l.strip()) for l in out.splitlines() if l.strip().isdigit()), None)
if not pid: sys.exit(1)
k = ctypes.WinDLL("kernel32", use_last_error=True)
k.FreeConsole()
if not k.AttachConsole(pid): sys.exit(2)
h = k.GetStdHandle(-11)
class COORD(ctypes.Structure):  _fields_ = [("X",wintypes.SHORT),("Y",wintypes.SHORT)]
class SR(ctypes.Structure):     _fields_ = [("L",wintypes.SHORT),("T",wintypes.SHORT),("R",wintypes.SHORT),("B",wintypes.SHORT)]
class CSBI(ctypes.Structure):   _fields_ = [("dwSize",COORD),("dwCursorPosition",COORD),("wAttributes",wintypes.WORD),("srWindow",SR),("dwMaximumWindowSize",COORD)]
csbi = CSBI(); k.GetConsoleScreenBufferInfo(h, ctypes.byref(csbi))
w = csbi.dwSize.X; top = csbi.srWindow.T; bot = csbi.srWindow.B
n = bot - top + 1
buf = ctypes.create_unicode_buffer(w*n); read = wintypes.DWORD()
k.ReadConsoleOutputCharacterW(h, buf, w*n, COORD(0,top), ctypes.byref(read))
k.FreeConsole()
t = buf.value
for i in range(n):
    s = t[i*w:(i+1)*w].rstrip()
    if s: print(s)
'@
    $tmp = Join-Path $env:TEMP '_read_console.py'
    $reader | Out-File $tmp -Encoding ascii
    & $py $tmp
}

function Screenshot($outPath) {
    Add-Type -AssemblyName System.Drawing
    $u = Add-Type -Name UD -Namespace W -PassThru -MemberDefinition '
[DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out R r);
[DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
[StructLayout(LayoutKind.Sequential)] public struct R { public int L,T,Ri,B; }'
    # Pick the sim process that actually owns a top-level window. A bare
    # Get-Process -Name 'main' returns an array when leftover sims linger,
    # which used to blow up the handle calls below.
    $proc = @(Get-Process -Name 'main' -ErrorAction SilentlyContinue |
              Where-Object { $_.MainWindowHandle -ne 0 })[0]
    if (-not $proc) { throw 'sim not running (no main.exe window)' }
    $hwnd = $proc.MainWindowHandle
    $r = New-Object W.UD+R
    [W.UD]::GetWindowRect($hwnd, [ref]$r) | Out-Null
    $w = $r.Ri - $r.L; $h = $r.B - $r.T
    $bmp = New-Object System.Drawing.Bitmap $w, $h
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $hdc = $g.GetHdc()
    # PrintWindow + PW_RENDERFULLCONTENT(2): grab the window's own pixels even
    # when it's covered or off-screen. CopyFromScreen grabbed whatever desktop
    # region overlapped it (e.g. the editor on top), so headless/agent runs got
    # the wrong window.
    $ok = [W.UD]::PrintWindow($hwnd, $hdc, 2)
    $g.ReleaseHdc($hdc)
    $bmp.Save($outPath, [System.Drawing.Imaging.ImageFormat]::Png)
    $g.Dispose(); $bmp.Dispose()
    if ($ok) { Write-Host "[screenshot] $outPath" -ForegroundColor Green }
    else     { Write-Host "[screenshot] PrintWindow returned false -> $outPath" -ForegroundColor Yellow }
}

# ---------------------------------------------------------------- main

Kill-Sim
if (-not $nobuild) { Build-Sim }
$null = Start-Sim

# Collect commands from -cmd and/or -script
$todo = @()
if ($cmd)    { $todo += $cmd }
if ($script) {
    if (-not (Test-Path $script)) { throw "script not found: $script" }
    $todo += (Get-Content $script | Where-Object { $_ -and -not $_.StartsWith('#') })
}
foreach ($c in $todo) {
    Write-Host "[send] $c" -ForegroundColor Yellow
    Send-Cmd $c
}

if ($todo.Count -gt 0) {
    Write-Host '[console] ----' -ForegroundColor Cyan
    Read-Console
}

if ($screenshot) { Screenshot $screenshot }

if ($snapshot) {
    $dir = Join-Path $proj "_snapshots\$snapshot"
    New-Item -ItemType Directory -Path $dir -Force | Out-Null
    # Pull the app list from list_apps. Give the sim ~2 s after sending the
    # cmd before reading the console so the response actually lands in the
    # buffer -- 600 ms (Send-Cmd default) is sometimes too tight.
    Send-Cmd 'list_apps'
    Start-Sleep -Seconds 2
    $screen = Read-Console
    # Output looks like:
    #   builtin apps:
    #     alarm
    #     battery
    #     ...
    #   (15 apps)
    # Lines start with two spaces; strip them and filter to looks-like-id.
    $apps = $screen |
            Where-Object { $_ -match '^\s{2}[a-zA-Z][a-zA-Z0-9_]+\s*$' } |
            ForEach-Object { $_.Trim() }
    # Fall back to a hardcoded list if parsing failed -- the PC build is
    # deterministic so this matches what list_apps emits today.
    if ($apps.Count -eq 0) {
        Write-Host '[snapshot] list_apps parse empty; using hardcoded fallback' -ForegroundColor Yellow
        # Verified against list_apps 2026-08-03. The previous list still had
        # 'sleep', which is NOT launchable — goto_app silently did nothing and
        # every snapshot for it came out as the watchface, which is worse than
        # a missing file because it looks like a real capture.
        $apps = @('alarm','battery','calculator','proto_pager','file_browser',
                  'flashlight','mouse','interact','Main','message','photo',
                  'qrcode','setting','skaiapp','skaijs','test','timer','weather')
    }
    Write-Host "[snapshot] $($apps.Count) apps to capture into $dir" -ForegroundColor Cyan
    foreach ($id in $apps) {
        Send-Cmd "goto_app $id"
        Start-Sleep -Milliseconds 1800   # page resume + render
        $out = Join-Path $dir "$id.png"
        try   { Screenshot $out }
        catch { Write-Host "[snapshot] $id failed: $_" -ForegroundColor Red }
        Send-Cmd 'goto_app Main'
        Start-Sleep -Milliseconds 1000
    }
    Write-Host "[snapshot] done -> $dir" -ForegroundColor Green
}
