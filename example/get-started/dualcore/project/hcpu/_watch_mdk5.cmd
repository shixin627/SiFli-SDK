@echo off
REM Generate Keil MDK5 project (.uvprojx) for sf32lb56w-watch.
REM Must run under Keil/armclang env because the project-local template.uvprojx
REM is TargetArmAds/Cads (armclang) structure — keil.py with PLATFORM=gcc would
REM look for TargetArm/Carm and fail with AttributeError.
REM
REM Output: project.uvprojx (+ project.uvoptx) in this dir, open with Keil MDK 5.

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
scons --board=sf32lb56w-watch_hcpu --target=mdk5 %* > _watch_mdk5.log 2>&1
type _watch_mdk5.log
