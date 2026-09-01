@echo off
setlocal
cd /d "%~dp0"
set "PY=python"
if exist "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" set "PY=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if not defined JSDOM_PATH set "JSDOM_PATH=%TEMP%\tt3jsdom"
if not exist "%JSDOM_PATH%\node_modules\jsdom" npm install jsdom --prefix "%JSDOM_PATH%" || exit /b 1
"%PY%" run_navigation_headless.py --repo . --matches 10 --out nav_combat_results --engine jsdom
endlocal
