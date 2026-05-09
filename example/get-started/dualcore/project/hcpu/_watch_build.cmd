@echo off
REM Temporary wrapper to run scons --board=sf32lb56w-watch with full SiFli env
REM (replicates what ConEmu CmdInit.cmd + set_env.bat would do interactively)

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

call C:\work\SiFli-SDK\set_env.bat keil
if errorlevel 1 exit /b %errorlevel%

cd /d C:\work\SiFli-SDK\example\get-started\dualcore\project\hcpu
scons --board=sf32lb56w-watch %* > _watch_build.log 2>&1
type _watch_build.log
