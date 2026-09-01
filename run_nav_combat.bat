@echo off
setlocal
cd /d "%~dp0"
if not defined JSDOM_PATH set "JSDOM_PATH=%TEMP%\tt3jsdom"
if not exist "%JSDOM_PATH%\node_modules\jsdom" npm install jsdom --prefix "%JSDOM_PATH%" || exit /b 1
python run_navigation_headless.py --repo . --matches 10 --out nav_combat_results --engine jsdom
endlocal
