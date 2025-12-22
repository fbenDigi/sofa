@echo off
set sceneFileRelativePath=%cd%\%1
start /d "../build/bin/RelWithDebInfo/" runSofa.exe -lSofaPython3 %sceneFileRelativePath% -g batch -n 90