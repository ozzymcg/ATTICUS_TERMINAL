@echo off
powershell -ExecutionPolicy Bypass -File "%~dp0build_exe.ps1" %*
pause