@echo off
title NavSim Telemetry Monitor
cd /d "%~dp0\..\.."
echo ========================================================
echo Iniciando Monitor de Telemetria MAVLink v2 para NavSim
echo ========================================================
echo.
python scripts\telemetry_monitor\telemetry_monitor.py %*
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo El monitor se ha cerrado o ha ocurrido un error.
    pause
)
