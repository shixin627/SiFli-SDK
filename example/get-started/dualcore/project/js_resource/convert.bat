@echo off
# find subfolders in the current folder
for /d %%a in (*) do (
    # change to the subfolder
    cd %%a
    # call the convert.bat in the subfolder
    call convert.bat
    # change back to the parent folder
    cd ..
)
