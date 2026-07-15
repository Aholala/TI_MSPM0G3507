@echo off
setlocal
cd /d "%~dp0"

echo Building model firmware...
set TRY=0
:build_retry
set /a TRY+=1
cmake --build cmake-debug -j1
if not errorlevel 1 goto build_ok
if %TRY% GEQ 3 goto build_failed
echo Build directory was busy. Retrying...
timeout /t 1 /nobreak >nul
goto build_retry

:build_ok
copy /y "cmake-debug\model.out" "Debug\model.out" >nul
if errorlevel 1 goto copy_failed
echo.
echo BUILD OK
echo Firmware: %~dp0Debug\model.out
goto finish

:build_failed
echo.
echo BUILD FAILED. Check the messages above.
goto finish

:copy_failed
echo.
echo BUILD OK, but copying Debug\model.out failed.

:finish
echo.
pause
endlocal
