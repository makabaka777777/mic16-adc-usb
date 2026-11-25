@echo off
chcp 65001 >nul
cd /d "%~dp0"

rem ============================================================
rem  MIC16_ADC123_USB  one-click capture
rem
rem  Usage:
rem    double-click          -> uses COM5, 5 seconds
rem    capture.bat COM6      -> uses COM6, 5 seconds
rem
rem  Output (in this folder):
rem    wave.png   16-channel waveform image
rem    ch1.wav    channel 1 audio
rem    out.csv    raw codes, one frame per line
rem ============================================================

set PORT=%~1
if "%PORT%"=="" set PORT=COM5

echo.
echo ============================================================
echo   MIC16 USB capture
echo   port     : %PORT%
echo   duration : 5 s
echo   files    : wave.png  ch1.wav  out.csv
echo   folder   : %CD%
echo ============================================================
echo.

python tools\raw_recv.py %PORT% --start --png wave.png --png-ms 50 --wav ch1.wav --wav-ch 1 --csv out.csv --seconds 5

if errorlevel 1 goto failed

echo.
echo [OK] done. files are in:
echo      %CD%
echo.
dir /b wave.png ch1.wav out.csv 2>nul
goto end

:failed
echo.
echo [ERROR] capture failed.
echo  1. is the board plugged in and powered? (check Device Manager)
echo  2. is the COM number right?  try:  capture.bat COM6
echo  3. is pyserial installed?    try:  pip install pyserial
echo  4. is another program using the port? (close serial assistant)

:end
echo.
pause
