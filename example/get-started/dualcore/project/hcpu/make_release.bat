@echo off
REM ==========================================================================
REM One-key RELEASE builder for the Skaiwalk dualcore watch.
REM
REM   [1] set_build_mode.py release  - flip dev->release knobs + ask for version
REM   [2] _watch_build.cmd           - build hcpu + lcpu (Keil / production)
REM   [3] edit release notes         - optional, opens info.json in notepad
REM   [4] package_watch_firmware.py  - copy bins into watchOS/sys
REM   [5] update_info.py             - sync version + fileList into info.json
REM
REM Order matters: the version entry (step 1) must precede the build because the
REM version is compiled into the firmware. Everything that touches info.json /
REM the watchOS folder (steps 3-5) runs only AFTER a successful build, so a
REM failed build never leaves half-written release notes or stale bins.
REM
REM After flashing/packaging, return to dev with:  python set_build_mode.py dev
REM ==========================================================================

setlocal

echo ============================================================
echo Step 1: Switching to RELEASE profile (you'll be asked for the version)...
echo ============================================================
python "%~dp0set_build_mode.py" release
if errorlevel 1 (
    echo.
    echo Failed to switch profile. Aborting.
    pause
    exit /b 1
)

echo.
echo ============================================================
echo Step 2: Building firmware (hcpu + lcpu)...
echo ============================================================
call "%~dp0_watch_build.cmd" -j8

REM _watch_build.cmd ends with `type log`, so its exit code is useless here.
REM Detect build failure by scanning the log (same patterns as the lint grep).
findstr /C:" error:" /C:"undefined reference" /C:"cannot find" /C:"scons: ***" "%~dp0_watch_build.log" >nul
if not errorlevel 1 (
    echo.
    echo ============================================================
    echo BUILD FAILED - see _watch_build.log
    echo Profile is still RELEASE. Fix the error and re-run, or switch
    echo back with:  python set_build_mode.py dev
    echo ============================================================
    pause
    exit /b 1
)

echo.
echo Build OK. Now packaging the release.
set /p "EDIT_DESC=Step 3: Edit release notes (info.json description) now? (y/N): "
if /i "%EDIT_DESC%"=="y" (
    echo Opening info.json - edit the "description" field, save, then close the editor.
    start /wait notepad "%~dp0info.json"
)

echo.
set "WATCHFACE_FLAG="
set /p "INCLUDE_WF=Include watchface folder in package? (y/N): "
if /i "%INCLUDE_WF%"=="y" (
    set "WATCHFACE_FLAG=--with-watchface"
    echo Watchface will be included.
) else (
    echo Watchface will NOT be included.
)

echo.
echo ============================================================
echo Step 4: Packaging watch firmware into watchOS\sys...
echo ============================================================
python "%~dp0package_watch_firmware.py" %WATCHFACE_FLAG%
if errorlevel 1 (
    echo.
    echo Package firmware failed! Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo ============================================================
echo Step 5: Updating info.json...
echo ============================================================
python "%~dp0update_info.py" %WATCHFACE_FLAG%
if errorlevel 1 (
    echo.
    echo Update info.json failed! Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo ============================================================
echo RELEASE BUILD DONE
echo   profile : RELEASE (kReleaseMode=1, board=29, FINSH off,
echo             virtual console on, HR/IMU power pins = 0)
echo   output  : watchOS\sys\ + info.json
echo.
echo Resume dev work later with:  python set_build_mode.py dev
echo ============================================================

endlocal
pause
