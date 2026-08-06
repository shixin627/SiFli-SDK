@echo off
REM Generate Keil MDK5 project (.uvprojx) for sf32lb56w-watch.
REM
REM Repo root auto-detected from %~dp0 (do not reintroduce hardcoded paths).
REM
REM Must run under Keil/armclang env because the project-local template.uvprojx
REM is TargetArmAds/Cads (armclang) structure -- keil.py with PLATFORM=gcc would
REM look for TargetArm/Carm and fail with AttributeError.
REM
REM Output: project.uvprojx (+ project.uvoptx) in this dir, open with Keil MDK 5.

for %%I in ("%~dp0..\..\..\..\..") do set REPO_ROOT=%%~fI

set ENV_ROOT=C:\dev\env_latest
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

cd /d "%~dp0"
scons --board=sf32lb56w-watch_hcpu --target=mdk5 %* > _watch_mdk5.log 2>&1
type _watch_mdk5.log
