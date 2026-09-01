@echo off
setlocal
set "ROOT=%~dp0.."
cd /d "%ROOT%"
set "PY=python"
if exist "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" set "PY=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if not exist test\results\current\arena mkdir test\results\current\arena
"%PY%" test\scripts\navigation_arena.py --repo . --maze-index 0 --seed 1001 --goals 20 --out-dir test\results\current\arena
if errorlevel 1 pause & exit /b 1
echo.
echo Navigation Arena finished: test\results\current\arena\arena_summary.json
pause
