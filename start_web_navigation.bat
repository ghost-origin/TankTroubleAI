@echo off
setlocal
cd /d "%~dp0"
set "PYCMD="
where py >nul 2>nul && set "PYCMD=py -3"
if not defined PYCMD where python >nul 2>nul && set "PYCMD=python"
if not defined PYCMD where python3 >nul 2>nul && set "PYCMD=python3"
if not defined PYCMD (
  echo [ERROR] Python 3 was not found.
  echo Please install Python 3 and add it to PATH.
  pause
  exit /b 1
)
if not exist "%~dp0web_nav_logs" mkdir "%~dp0web_nav_logs"
%PYCMD% "%~dp0launcher.py" --ai-mode navigation --nav-log-dir "%~dp0web_nav_logs"

echo.
echo Web server stopped.
pause
endlocal
