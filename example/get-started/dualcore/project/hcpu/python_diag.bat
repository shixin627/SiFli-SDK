@echo off
echo === USERPROFILE / APPDATA ===
echo USERPROFILE=%USERPROFILE%
echo APPDATA=%APPDATA%
echo.
echo === where python (and python3 / python.exe) ===
where python 2>&1
where python.exe 2>&1
where python3 2>&1
echo.
echo === ~/.local/bin contents ===
dir /b "%USERPROFILE%\.local\bin" 2>&1
echo.
echo === uv python dir contents ===
dir /b "%USERPROFILE%\AppData\Roaming\uv\python" 2>&1
echo.
echo === direct python.exe exists? ===
if exist "%USERPROFILE%\AppData\Roaming\uv\python\cpython-3.12.13-windows-x86_64-none\python.exe" (
    echo YES: direct python.exe is present
) else (
    echo NO: direct python.exe NOT FOUND
)
echo.
echo === PATH ===
echo %PATH%
echo.
pause
