@echo off
REM Wrapper to run scons --board=pc with full SiFli env (replicates what ConEmu
REM CmdInit.cmd + set_env.bat would do interactively).
REM
REM Repo root is auto-detected from this wrapper's own location (%~dp0) so the
REM file works whether the SDK is at C:\work\SiFli-SDK, C:\Users\...\GitHub\SiFli-SDK,
REM or anywhere else. Do NOT reintroduce hardcoded C:\work paths.

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

call "%REPO_ROOT%\set_env.bat" gcc
if errorlevel 1 exit /b %errorlevel%

cd /d "%~dp0"
scons --board=pc %* > _pc_build.log 2>&1
type _pc_build.log
