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

set ENV_ROOT=C:\dev\env_latest
set ENV_VER=1.1.4
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

REM Board defaults to sf32lb56w-watch (sf32lb563w, dev machines) for manual /
REM make_release.bat use. release_gui.py overrides WATCH_BOARD to build the
REM production chip (sf32lb563 = sf32lb56-watch).
if "%WATCH_BOARD%"=="" set WATCH_BOARD=sf32lb56w-watch

cd /d "%~dp0"
scons --board=%WATCH_BOARD% %* > _watch_build.log 2>&1
type _watch_build.log
