@echo off
for %%i in (*.png) do (
    ..\..\..\..\..\..\tools\png2ezip\ezip -convert %%i -rgb565 -binfile 2
)