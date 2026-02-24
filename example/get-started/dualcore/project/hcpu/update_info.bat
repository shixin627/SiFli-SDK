@echo off
REM Batch script to update info.json and package watch firmware

echo ============================================================
echo Step 1: Packaging watch firmware...
echo ============================================================

python "%~dp0package_watch_firmware.py"

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

python "%~dp0update_info.py"

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
