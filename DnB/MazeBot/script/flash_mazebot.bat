@echo off
setlocal

set CLI=D:\Repository\Embedded Development\Arduino\arduino-cli_1.4.1\arduino-cli.exe
set SKETCH=D:\QMUL Modules\Y3_S2\Microprocessor Systems Design\EoM\MazeBot\DnB\MazeBot
set LIBS=D:\Repository\Embedded Development\Arduino\libraries
set FQBN=STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_F446RE,upload_method=swdMethod
set BUILD=%TEMP%\mazebot_build

echo [1/2] Compiling MazeBot...
"%CLI%" compile --fqbn %FQBN% --libraries "%LIBS%" --build-path "%BUILD%" "%SKETCH%"
if errorlevel 1 goto :fail

echo.
echo [2/2] Uploading via ST-LINK SWD...
"%CLI%" upload --fqbn %FQBN% --input-dir "%BUILD%" "%SKETCH%"
if errorlevel 1 goto :fail

echo.
echo Flash complete.
pause
goto :end

:fail
echo.
echo Flash failed.
pause
exit /b 1

:end
endlocal
