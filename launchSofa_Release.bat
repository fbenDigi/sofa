@echo off
set sceneFileRelativePath=%cd%\%1
echo %sceneFileRelativePath%
start /d "../build/bin/Release/" runSofa.exe -lSofaPython3 %sceneFileRelativePath%