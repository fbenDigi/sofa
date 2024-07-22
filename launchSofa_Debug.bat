@echo off
set sceneFileRelativePath=%cd%\%1
start /d "../build/bin/Debug/" runSofa.exe -lSofaPython3 %sceneFileRelativePath%