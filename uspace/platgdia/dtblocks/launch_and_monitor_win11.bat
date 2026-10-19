@echo off
setlocal enabledelayedexpansion

:: Navigate to the directory containing this script
cd /d "%~dp0"

:: 1. Launch the Docker Compose project in detached mode and build if necessary
echo Starting Docker Compose environment...
docker compose up -d --build

:: 2. Wait briefly to allow all containers to spin up and register
timeout /t 3 /nobreak >nul

:: 3. Retrieve service names and launch a dedicated Windows Terminal window for each
for /f "tokens=*" %%S in ('docker compose ps --services') do (
    if /i "%%S"=="mqtt_dtblock" (
        echo Skipping log window for service: %%S
    ) else (
        echo Opening log window for service: %%S
        
        :: Use '-d .' instead of '-d "%~dp0"' to prevent trailing backslash escaping issues
        wt.exe -w new nt --title "%%S" -d . cmd /k "docker compose logs -f %%S"
    )
)

echo All log windows successfully opened.

:: Script directory and optional validator calls
set "SCRIPT_DIR=%~dp0"
:: rem wt.exe -w new nt --title "Validator Script" -d . cmd /k "python validation\validation_dtb.py"

endlocal