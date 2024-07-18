@echo off
set sceneFileRelativePath=%cd%\%1
echo %sceneFileRelativePath%
start /d "../build/bin/RelWithDebInfo/" runSofa.exe -lSofaPython3 %sceneFileRelativePath%