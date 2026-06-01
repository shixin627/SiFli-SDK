@echo off
REM Launch the Traditional-Chinese release GUI (release_gui.py).
REM A GUI window renders Chinese via system fonts, so it does not depend on
REM the console code page. This launcher is ASCII (the .cmd ASCII rule holds).
chcp 65001 >nul
set "PYTHONUTF8=1"
cd /d "%~dp0"

REM This GUI is Python 3 (set_build_mode.py uses py3-only open(encoding=),
REM subprocess text=, etc). The SiFli toolchain puts Python 2.7 first on PATH,
REM so a bare "python" runs the wrong interpreter. Prefer the py launcher, then
REM fall back to the user-scope python.org install.
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 release_gui.py
) else (
    "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" release_gui.py
)
if errorlevel 1 (
    echo.
    echo GUI failed to start. If the error mentions tkinter, your Python lacks
    echo the Tk bindings - use a standard python.org build or install tkinter.
    echo If it mentions "encoding" / "invalid keyword", Python 3 was not found -
    echo install it from python.org (or: winget install Python.Python.3.12).
    pause
)
