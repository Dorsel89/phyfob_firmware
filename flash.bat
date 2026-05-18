@echo off
setlocal enabledelayedexpansion

if "%~2"=="" (
    echo Usage: %0 ^<ascii_char^> ^<2stellige_zahl^>
    echo Beispiel: %0 D 42
    pause
    exit /b 1
)

set ASCII_CHAR=%1
set ZAHL=%2

REM Validiere 2-stellige Zahl
if not "%ZAHL:~2%"=="" (
    echo Fehler: Zweite Zahl muss genau 2 Ziffern sein ^(00-99^)
    pause
    exit /b 1
)

REM Bytes als Hex extrahieren (PowerShell)
for /f %%i in ('powershell -command "[convert]::ToString([int][char]'%ASCII_CHAR%',16).PadLeft(2,'0').ToUpper()"') do set BYTE1=%%i
for /f %%i in ('powershell -command "[convert]::ToString([int]'%ZAHL%',16).PadLeft(4,'0').ToUpper()"') do set ZAHL_HEX=%%i

REM Kombiniere: ASCII_BYTE << 16 | uint16_ZAHL (little-endian angepasst)
set VAL=0x%BYTE1%%ZAHL_HEX%

echo Bytes: %ASCII_CHAR% ^(0x%BYTE1%^) + '%ZAHL%' ^(0x%BYTE2%%BYTE3%^) ^-^> VAL %VAL%

echo 1/5: Erase all...
nrfjprog --family NRF52 --eraseall
if %errorlevel% neq 0 goto :error

echo 3/5: Write custom 0x10001080...
nrfjprog --family NRF52 --memwr 0x10001080 --val %VAL%
if %errorlevel% neq 0 goto :error

echo 2/5: Program ^& Verify HEX...
nrfjprog --family NRF52 --program build_3v3/merged.hex --verify
if %errorlevel% neq 0 goto :error

echo 4/4: Reset...
nrfjprog --family NRF52 --reset
echo Fertig! Geschrieben: %VAL% an 0x10001080
pause
exit /b 0

:error
echo FEHLER in Schritt %errorlevel%!

exit /b %errorlevel%
