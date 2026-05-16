@echo off
REM Thin wrapper for _dev_test.ps1 — bypasses the default PowerShell
REM execution policy that blocks running .ps1 files for normal users.
REM
REM Usage:
REM   _dev_test.cmd                                  rebuild + relaunch sim
REM   _dev_test.cmd -nobuild                         just relaunch sim (skip scons)
REM   _dev_test.cmd -nobuild -cmd "list_apps"        send one MSH command
REM   _dev_test.cmd -script my_cmds.txt              run a newline-separated script
REM   _dev_test.cmd -screenshot foo.png              snap a screenshot
REM   _dev_test.cmd -snapshot apps                   capture every app to disk
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0_dev_test.ps1" %*
