@echo off
setlocal
cd /d "%~dp0"
set "PY=python"
if exist "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" set "PY=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"

echo ========================================
echo TankTrouble Navigation Benchmark
 echo 10 matches, navigation-only, existing AI baseline = 1
 echo ========================================

if not defined JSDOM_PATH set "JSDOM_PATH=%TEMP%\tt3jsdom"
if not exist "%JSDOM_PATH%\node_modules\jsdom" (
  echo Installing jsdom to %JSDOM_PATH% ...
  npm install jsdom --prefix "%JSDOM_PATH%" || goto :chromium
)

"%PY%" run_navigation_headless.py --repo . --matches 10 --out nav_headless_results --engine jsdom --navigation-only
goto :end

:chromium
echo.
echo jsdom install failed. Trying Chromium/Playwright fallback...
"%PY%" -c "import playwright" >nul 2>nul || "%PY%" -m pip install playwright
"%PY%" -m playwright install chromium
"%PY%" run_navigation_headless.py --repo . --matches 10 --duration 12 --game-speed 2 --out nav_headless_results --engine chromium --navigation-only

:end
echo.
echo Results: nav_headless_results\analysis.json
echo Scores : nav_headless_results\scores.csv
echo Plots  : nav_headless_results\match_XXX\track_*.png
pause
endlocal
