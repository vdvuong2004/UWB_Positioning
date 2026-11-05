@echo off
rem Links Libraries\Dw3000 and Libraries\Dw1000 into the Arduino libraries
rem folder, so the Arduino IDE builds this repo's copy and its examples show up
rem under File > Examples. Run it again after moving the repo.
rem Usage: install_libraries.bat [libraries folder]
rem        default: %USERPROFILE%\Documents\Arduino\libraries
setlocal
set "LIBDIR=%USERPROFILE%\Documents\Arduino\libraries"
if not "%~1"=="" set "LIBDIR=%~1"
if not exist "%LIBDIR%" mkdir "%LIBDIR%"

set "FAILED="
for %%L in (Dw3000 Dw1000) do call :link %%L
if defined FAILED exit /b 1
echo Done. Restart the Arduino IDE.
exit /b 0

:link
set "DST=%LIBDIR%\%1"
if exist "%DST%" (
  rem An old link is replaced; a real folder is left alone.
  dir "%LIBDIR%" /AL /B 2>nul | findstr /X /I "%1" >nul
  if errorlevel 1 (
    echo %DST% is a real folder, not a link. Move or delete it, then run this again.
    set "FAILED=1"
    exit /b 0
  )
  rmdir "%DST%"
)
mklink /J "%DST%" "%~dp0Libraries\%1"
exit /b 0
