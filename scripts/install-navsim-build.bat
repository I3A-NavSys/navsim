@echo off
setlocal enabledelayedexpansion

@echo off
echo === NAVSIM Interactive Deployment ===
echo.

if "%1"=="" (
    echo ❌ Missing required arguments.
    echo.
    echo Usage: %0 ^<docker_compose_yml^> ^<isaac_sim_path^>
    echo Example: %0 docker-compose.yml C:\isaacsim
    pause
    exit /b 1
)

if "%2"=="" (
    echo ❌ Missing required arguments.
    echo.
    echo Usage: %0 ^<docker_compose_yml^> ^<isaac_sim_path^>
    echo Example: %0 docker-compose.yml C:\isaacsim
    pause
    exit /b 1
)

set COMPOSE_FILE=%1
set ISAAC_SIM_PATH=%2

@REM Validate Compose file
if not exist "%COMPOSE_FILE%" (
    echo ❌ Configuration file not found: %COMPOSE_FILE%
    echo Please check the configuration name.
    pause
    exit /b 1
)

REM Validate Isaac Sim path
if not exist "%ISAAC_SIM_PATH%" (
    echo ❌ Isaac Sim path does not exist: %ISAAC_SIM_PATH%
    echo Please check the path and try again.
    pause
    exit /b 1
)

REM Additional Isaac Sim validation
if not exist "%ISAAC_SIM_PATH%\isaac-sim.bat" (
    if not exist "%ISAAC_SIM_PATH%\isaac-sim.fabric.bat" (
        echo ❌ This doesn't appear to be a valid Isaac Sim installation.
        echo Looking for isaac-sim.bat or isaac-sim.fabric.bat in: %ISAAC_SIM_PATH%
        echo.
        echo Contents of directory:
        dir "%ISAAC_SIM_PATH%" /B
        echo.
        echo Please provide a valid Isaac Sim installation path.
        pause
        exit /b 1
    )
)

docker-compose -f %COMPOSE_FILE% up --build

echo.
echo Cleaning up...
docker-compose -f %COMPOSE_FILE% down

pause