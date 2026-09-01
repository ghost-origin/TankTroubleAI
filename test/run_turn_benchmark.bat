@echo off
setlocal
set "ROOT=%~dp0.."
cd /d "%ROOT%"
set "PY=python"
if exist "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" set "PY=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if not exist test\results\current\turn mkdir test\results\current\turn
"%PY%" test\scripts\turn_benchmark.py --out-dir test\results\current\turn
if errorlevel 1 pause & exit /b 1
echo.
echo Turn Benchmark finished: test\results\current\turn\turn_summary.csv
pause
