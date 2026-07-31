@echo off
cd /d D:\KimiData\kimi\workspace\ApexFlight
set QT_QPA_PLATFORM=offscreen
set FAILED=0
for %%T in (test_core test_sweep smoke_v07 check_startup_log check_autotune_dialog check_motor_tab check_sweep_viz check_map_anchor check_task_dispatch check_extras_v099 check_ollama_dl check_ai_determinism) do (
  echo === %%T ===
  "C:\Users\Administrator\AppData\Local\Python\bin\python3.exe" -X utf8 tests\%%T.py > tests\out\%%T.log 2>&1
  if %ERRORLEVEL%==0 (echo PASS %%T) else (echo FAIL %%T rc=%ERRORLEVEL% & set FAILED=1)
)
echo SUITE_DONE FAILED=%FAILED%
