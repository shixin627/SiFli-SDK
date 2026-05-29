@echo off
REM Launch the Traditional-Chinese release GUI (release_gui.py).
REM A GUI window renders Chinese via system fonts, so it does not depend on
REM the console code page. This launcher is ASCII (the .cmd ASCII rule holds).
chcp 65001 >nul
set "PYTHONUTF8=1"
cd /d "%~dp0"
python release_gui.py
if errorlevel 1 (
    echo.
    echo GUI failed to start. If the error mentions tkinter, your Python lacks
    echo the Tk bindings - use a standard python.org build or install tkinter.
    pause
)
