@echo off
set sceneFileRelativePath=%cd%\%1
start /d "../build/bin/Release/" runSofa.exe -lSofaPython3 %sceneFileRelativePath%