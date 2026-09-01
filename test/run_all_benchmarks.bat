@echo off
setlocal
set "ROOT=%~dp0.."
cd /d "%ROOT%"
set "PY=python"
if exist "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" set "PY=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if not exist test\results\current\turn mkdir test\results\current\turn
if not exist test\results\current\arena mkdir test\results\current\arena
"%PY%" test\scripts\turn_benchmark.py --out-dir test\results\current\turn || goto :fail
"%PY%" test\scripts\navigation_arena.py --repo . --maze-index 0 --seed 1001 --goals 20 --out-dir test\results\current\arena || goto :fail
"%PY%" test\scripts\benchmark_report.py --root test\results\current || goto :fail
echo.
echo All navigation benchmarks completed.
echo Results: test\results\current
pause
exit /b 0
:fail
echo Benchmark failed.
pause
exit /b 1
