@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"

set "PYCMD="
where py >nul 2>nul && set "PYCMD=py -3"
if not defined PYCMD where python >nul 2>nul && set "PYCMD=python"
if not defined PYCMD where python3 >nul 2>nul && set "PYCMD=python3"
if not defined PYCMD (
  echo [错误] 未找到 Python 3。
  echo 请先安装 Python 3.10/3.11/3.12，并勾选 Add Python to PATH。
  pause
  exit /b 1
)

echo [1/2] 检查自主导航依赖...
%PYCMD% -c "import numpy, PIL, matplotlib" >nul 2>nul
if errorlevel 1 (
  echo 正在安装 numpy / Pillow / matplotlib ...
  %PYCMD% -m pip install -r "%~dp0requirements_autonomous.txt"
  if errorlevel 1 (
    echo [错误] Python 依赖安装失败。
    pause
    exit /b 1
  )
)

echo [2/2] 启动 TankTrouble 自主导航 AI...
%PYCMD% "%~dp0launcher_autonomous.py"

echo.
echo 游戏服务已停止。
pause
endlocal
