@echo off
set sceneFileRelativePath=%cd%\%1
cd /d "%~dp0..\build\bin\RelWithDebInfo\"
runSofa.exe -lSofaPython3 %sceneFileRelativePath% -g batch -n 90
if errorlevel 1 (
    echo.
    echo Process exited with error code %errorlevel%
)
pause