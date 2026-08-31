@echo off
cd /d "%~dp0"

rem ============================================================
rem  Find a Python 3 interpreter on this machine.
rem  Search order:
rem    1. py launcher (official Windows Python launcher)
rem    2. python  on PATH
rem    3. python3 on PATH
rem    4. common install folders (LocalAppData / Program Files /
rem       Program Files (x86) / C:\Python*)
rem  If nothing usable is found, print an error and exit.
rem ============================================================
set "PYCMD="

where py >nul 2>nul && set "PYCMD=py -3"
if not defined PYCMD where python >nul 2>nul && set "PYCMD=python"
if not defined PYCMD where python3 >nul 2>nul && set "PYCMD=python3"

if not defined PYCMD (
  for /d %%D in ("%LOCALAPPDATA%\Programs\Python\Python*") do (
    if not defined PYCMD if exist "%%D\python.exe" set "PYCMD="%%D\python.exe""
  )
)

if not defined PYCMD (
  for /d %%D in ("%ProgramFiles%\Python*") do (
    if not defined PYCMD if exist "%%D\python.exe" set "PYCMD="%%D\python.exe""
  )
)

set "PFX86=%ProgramFiles(x86)%"
if not defined PYCMD (
  for /d %%D in ("%PFX86%\Python*") do (
    if not defined PYCMD if exist "%%D\python.exe" set "PYCMD="%%D\python.exe""
  )
)

if not defined PYCMD (
  for /d %%D in ("C:\Python*") do (
    if not defined PYCMD if exist "%%D\python.exe" set "PYCMD="%%D\python.exe""
  )
)

rem validate the candidate: must actually run and be Python 3
if defined PYCMD (
  %PYCMD% -c "import sys; sys.exit(0 if sys.version_info[0] == 3 else 1)" >nul 2>&1
  if errorlevel 1 set "PYCMD="
)

if not defined PYCMD (
  echo.
  echo [ERROR] Python 3 was not found on this machine.
  echo.
  echo Please install Python 3 from https://www.python.org/downloads/
  echo Tick "Add Python to PATH" during installation, then run this file again.
  echo.
  pause
  exit /b 1
)

echo Using Python: %PYCMD%
echo.
%PYCMD% "%~dp0launcher.py"

echo.
echo Server stopped.
pause
