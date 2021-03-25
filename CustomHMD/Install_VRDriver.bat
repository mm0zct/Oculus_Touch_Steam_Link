@echo off

IF EXIST "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\CustomHMD\bin\win32" GOTO SKIP1
mkdir "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\CustomHMD\bin\win32"
:SKIP1

IF EXIST "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\CustomHMD\bin\win64" GOTO SKIP2
mkdir "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\CustomHMD\bin\win64"
:SKIP2

set check = 0

echo ********************
xcopy /y /q "%cd%\steamvr.vrsettings"               "C:\Program Files (x86)\Steam\config"
if errorlevel 0 set /a check += 1

xcopy /y /q "%cd%\Release\x86\driver_CustomHMD.dll" "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\CustomHMD\bin\win32"
if errorlevel 0 set /a check += 1

xcopy /y /q "%cd%\Release\x64\driver_CustomHMD.dll" "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\CustomHMD\bin\win64"
if errorlevel 0 set /a check += 1

echo ********************
echo %check%/3 Files copied
echo ********************

pause