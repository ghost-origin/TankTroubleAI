@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title TankTrouble Autonomous AI

echo ============================================================
echo   TankTrouble Autonomous AI Launcher
echo ============================================================
echo.

set "PYMODE="
where py >nul 2>nul
if not errorlevel 1 (
  py -3 -c "import sys; sys.exit(0 if sys.version_info.major==3 else 1)" >nul 2>nul
  if not errorlevel 1 set "PYMODE=PYLAUNCHER"
)

if not defined PYMODE (
  where python >nul 2>nul
  if not errorlevel 1 (
    python -c "import sys; sys.exit(0 if sys.version_info.major==3 else 1)" >nul 2>nul
    if not errorlevel 1 set "PYMODE=PYTHON"
  )
)

if not defined PYMODE (
  where python3 >nul 2>nul
  if not errorlevel 1 (
    python3 -c "import sys; sys.exit(0 if sys.version_info.major==3 else 1)" >nul 2>nul
    if not errorlevel 1 set "PYMODE=PYTHON3"
  )
)

if not defined PYMODE goto NO_PYTHON

if "%PYMODE%"=="PYLAUNCHER" goto RUN_PYLAUNCHER
if "%PYMODE%"=="PYTHON" goto RUN_PYTHON
if "%PYMODE%"=="PYTHON3" goto RUN_PYTHON3

goto NO_PYTHON

:RUN_PYLAUNCHER
py -3 bootstrap_ai.py
set "RC=%ERRORLEVEL%"
goto FINISH

:RUN_PYTHON
python bootstrap_ai.py
set "RC=%ERRORLEVEL%"
goto FINISH

:RUN_PYTHON3
python3 bootstrap_ai.py
set "RC=%ERRORLEVEL%"
goto FINISH

:NO_PYTHON
echo [ERROR] Python 3 was not found.
echo Install Python 3.10 or newer and enable "Add Python to PATH".
echo.
echo Details are also written to startup_error.log.
> startup_error.log echo Python 3 was not found. Install Python 3 and enable Add Python to PATH.
set "RC=1"

:FINISH
echo.
if not "%RC%"=="0" (
  echo [FAILED] The launcher returned error code %RC%.
  echo Open startup_error.log in this folder for details.
) else (
  echo Game server stopped normally.
)
echo.
pause
exit /b %RC%
