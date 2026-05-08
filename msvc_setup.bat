@REM Setup MSVC PATH
@REM Clear any previous subst mappings (ignore errors if not mapped)
subst x: /D 2>nul
subst y: /D 2>nul
subst l: /D 2>nul
@REM MSVC root folder (VS2017 toolset bundled inside VS2022 Community)
subst x: "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.16.27023"
@REM Windows SDK Include folder
subst y: "C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0"
@REM Windows SDK Lib folder
subst l: "C:\Program Files (x86)\Windows Kits\10\Lib\10.0.22621.0"
