@echo off
setlocal
set "ROOT=%~dp0.."
cd /d "%ROOT%"
set "PY=python"
if exist "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" set "PY=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if not exist test\results\current\arena_long mkdir test\results\current\arena_long
"%PY%" test\scripts\navigation_arena.py --repo . --maze-index 0 --seed 1001 --goals 20 --profile long --out-dir test\results\current\arena_long
pause
