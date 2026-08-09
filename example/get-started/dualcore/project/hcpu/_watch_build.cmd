@echo off
REM Wrapper to run scons --board=sf32lb56w-watch with full SiFli env.
REM
REM Repo root auto-detected from %~dp0 (do not reintroduce hardcoded paths).
REM
REM Toolchain: KEIL (armclang + microlib) -- production setting.
REM   * Smaller LCPU binary (~159 KB vs GCC ~234 KB) because microlib is much
REM     leaner than newlib (printf/scanf/timezone DB stripping).
REM   * GCC alternative exists for verification only -- change `keil` -> `gcc`
REM     below. GCC build pulls in full newlib (localtime/mktime in hr_service.c
REM     and alarm_manager_service.c drags timezone DB ~14 KB + printf-float
REM     family ~22 KB).

for %%I in ("%~dp0..\..\..\..\..") do set REPO_ROOT=%%~fI

REM ENV_ROOT is probed, not pinned: the SiFli env install dir differs per
REM machine (env_latest / env), and a stale hardcode makes set_env.bat report
REM the misleading "Please upgrate env to v1.2.0" (really: dir not found).
REM An ENV_ROOT already set in the environment always wins.
if not exist "%ENV_ROOT%\tools\ConEmu\ConEmu\CmdInit.cmd" set "ENV_ROOT=C:\dev\env_latest"
if not exist "%ENV_ROOT%\tools\ConEmu\ConEmu\CmdInit.cmd" set "ENV_ROOT=C:\dev\env"
if not exist "%ENV_ROOT%\tools\ConEmu\ConEmu\CmdInit.cmd" (
    echo ERROR: SiFli env not found. Set ENV_ROOT to the env install dir.
    exit /b 1
)
REM Read the version from the installed env instead of pinning it here:
REM a hardcoded value silently blocks every SDK that raises the minimum.
for /f "tokens=2 delims==" %%V in ('findstr /b /c:"set ENV_VER=" "%ENV_ROOT%\tools\ConEmu\ConEmu\CmdInit.cmd"') do set ENV_VER=%%V
if "%ORG_PATH%"=="" set ORG_PATH=%PATH%
set PYTHONPATH=%ENV_ROOT%\tools\python-3.11.9-amd64
set PYTHONHOME=%ENV_ROOT%\tools\python-3.11.9-amd64
set SCONS=%PYTHONPATH%\Scripts
set PATH=%ENV_ROOT%\tools\bin;%PATH%
set PATH=%PYTHONHOME%;%PATH%
set PATH=%PYTHONPATH%;%PATH%
set PATH=%SCONS%;%PATH%

call "%REPO_ROOT%\set_env.bat" keil
if errorlevel 1 exit /b %errorlevel%

REM Board defaults to sf32lb56-watch (sf32lb563 production chip = the eh-lb56xu
REM watch this repo targets). Build output: build_sf32lb56-watch_hcpu\; flash it
REM with that dir's uart_download.bat (UART download, COM port = the watch's
REM ImgDownUart channel). Override WATCH_BOARD before calling to build a different
REM chip (e.g. the older sf32lb56w-watch dev board, or release_gui.py's target).
if "%WATCH_BOARD%"=="" set WATCH_BOARD=sf32lb56-watch

cd /d "%~dp0"
scons --board=%WATCH_BOARD% %* > _watch_build.log 2>&1
set "WATCH_BUILD_RC=%ERRORLEVEL%"
type _watch_build.log
exit /b %WATCH_BUILD_RC%
