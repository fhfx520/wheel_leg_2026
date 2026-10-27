@echo off
setlocal
powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -File "%~dp0sync_dma_memory.ps1" -IocPath "%~dp0..\..\rm_main.ioc"
exit /b %errorlevel%
