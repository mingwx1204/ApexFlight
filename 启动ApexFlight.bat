@echo off
rem ==========================================
rem  ApexFlight Launcher
rem  Double-click this file to run the app.
rem ==========================================
cd /d "%~dp0"

rem Prefer a known local Python install, fall back to PATH.
set "PYEXE=C:\Users\Administrator\AppData\Local\Python\bin\python3.exe"
if not exist "%PYEXE%" set "PYEXE=python"

"%PYEXE%" "src\main.py"
if errorlevel 1 (
    echo.
    echo App exited with an error. Check logs\crash.log for details.
    echo If Python is missing, install Python 3.10+ from python.org
    echo then run:  pip install -r requirements.txt
    pause
)
