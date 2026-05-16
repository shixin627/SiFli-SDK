@echo off
REM Wrapper to run scons --board=pc with full SiFli env (replicates what ConEmu
REM CmdInit.cmd + set_env.bat would do interactively).
REM
REM Repo root is auto-detected from this wrapper's own location (%~dp0) so the
REM file works whether the SDK is at C:\work\SiFli-SDK, C:\Users\...\GitHub\SiFli-SDK,
REM or anywhere else. Do NOT reintroduce hardcoded C:\work paths.

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

call "%REPO_ROOT%\set_env.bat" gcc
if errorlevel 1 exit /b %errorlevel%

cd /d "%~dp0"
scons --board=pc %* > _pc_build.log 2>&1
type _pc_build.log
