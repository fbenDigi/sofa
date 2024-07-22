@echo off
set sceneFileRelativePath=%cd%\%1
start /d "../build/bin/RelWithDebInfo/" runSofa.exe -lSofaPython3 %sceneFileRelativePath%