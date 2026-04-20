@echo off
REM Batch script to update info.json and package watch firmware

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
echo Step 1: Packaging watch firmware...
echo ============================================================

python "%~dp0package_watch_firmware.py" %WATCHFACE_FLAG%

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ============================================================
    echo Package firmware failed! Error code: %ERRORLEVEL%
    echo ============================================================
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo ============================================================
echo Step 2: Updating info.json...
echo ============================================================

python "%~dp0update_info.py" %WATCHFACE_FLAG%

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ============================================================
    echo Update info.json failed! Error code: %ERRORLEVEL%
    echo ============================================================
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo ============================================================
echo All tasks completed successfully!
echo ============================================================

pause
