@echo off
setlocal
cd /d "%~dp0"
if not defined JSDOM_PATH set "JSDOM_PATH=%TEMP%\tt3jsdom"
if not exist "%JSDOM_PATH%\node_modules\jsdom" (
  echo Installing jsdom to %JSDOM_PATH% ...
  npm install jsdom --prefix "%JSDOM_PATH%" || exit /b 1
)
python run_navigation_headless.py --repo . --matches 10
endlocal
