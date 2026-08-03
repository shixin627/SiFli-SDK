@echo off
REM Regenerate the signed-package fixtures the simulator's skai_pkg_test reads
REM from /pkgsrc. Each one is signed correctly and then broken in exactly one
REM way, so a failing check names the property that was violated.
REM
REM Keys are generated fresh every run except for `update`, which reuses good's
REM key on purpose: that is what makes it an update rather than an impostor.
setlocal
set ROOT=%~dp0..\..\..\..\..
set SIGN=node "%ROOT%\tools\sdk\sign_pkg.js"
set OUT=%~dp0disk\pkgsrc
set SRC=%OUT%\hello.js

if not exist "%OUT%" mkdir "%OUT%"
if not exist "%SRC%" (
    > "%SRC%" echo var t = skai.ui.label^('installed app'^);
    >>"%SRC%" echo skai.ui.set_color^(t, 0x8E8E93^);
)

set COMMON=--payload "%SRC%" --id hello --name Hello --caps ui.label,ui.set_color

%SIGN% %COMMON% --version 1.0.0 --out "%OUT%\good"
%SIGN% %COMMON% --version 1.0.0 --out "%OUT%\t_payload" --tamper payload
%SIGN% %COMMON% --version 1.0.0 --out "%OUT%\t_caps"    --tamper caps
%SIGN% %COMMON% --version 1.0.0 --out "%OUT%\t_keyid"   --tamper keyid
%SIGN% %COMMON% --version 1.0.0 --out "%OUT%\t_sig"     --tamper sig
%SIGN% %COMMON% --version 1.0.1 --out "%OUT%\update"    --key "%OUT%\good\key.json"
%SIGN% %COMMON% --version 1.0.0 --out "%OUT%\impostor"
%SIGN% %COMMON% --version 1.0.0 --out "%OUT%\future"    --skai ">=9.9"

echo.
echo fixtures regenerated in %OUT%
endlocal
