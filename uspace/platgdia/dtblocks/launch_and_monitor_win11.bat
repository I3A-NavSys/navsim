@echo off
setlocal enabledelayedexpansion

:: Navigate to the directory containing this script
cd /d "%~dp0"

:: 1. Launch the Docker Compose project in detached mode and build if necessary
echo Starting Docker Compose environment...
docker compose up -d --build

:: 2. Wait briefly to allow all containers to spin up and register
timeout /t 3 /nobreak >nul

:: 3. Retrieve service names and launch a new terminal window for each
for /f "tokens=*" %%S in ('docker compose ps --services') do (
    if /i "%%S"=="mqtt_dtblock" (
        echo Skipping log window for service: %%S
    ) else (
        echo Opening log window for service: %%S
        
        :: 'start "Title"' sets the window title
        :: 'cmd /k' keeps the window open even if the process stops or fails
        start "%%S" cmd /k "title %%S && docker compose logs -f %%S"
    )
)

echo All log windows successfully opened.

:: Script directory and optional validator calls
set "SCRIPT_DIR=%~dp0"
:: rem start "Validator Script" cmd /k "title Validator Script && python "%SCRIPT_DIR%validation\validation_dtb.py""

endlocal