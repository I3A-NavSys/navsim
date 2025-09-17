@echo off
setlocal enabledelayedexpansion

echo === NAVSIM Installation System ===
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

echo Configuration: %CONFIG%
echo Isaac Sim Path: %ISAAC_SIM_PATH%
echo Compose File: %COMPOSE_FILE%
echo.

REM Check if Docker is running
docker version >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo ❌ Docker is not running. Please start Docker Desktop and try again.
    pause
    exit /b 1
)

echo ✅ Docker is running
echo.

echo === Pulling Required Images ===
echo Downloading latest installation images from registry...
docker-compose -f %COMPOSE_FILE% pull

if %ERRORLEVEL% NEQ 0 (
    echo ❌ Failed to pull Docker images. Please check your internet connection and try again.
    pause
    exit /b 1
)

echo ✅ All images downloaded successfully
echo.

echo All services will run in parallel. NAVSIM will be installed at: %ISAAC_SIM_PATH%\navsim
echo Extensions will be installed at: %ISAAC_SIM_PATH%\navsim\extensions\
echo.

echo === Running Installation Services ===
docker-compose -f %COMPOSE_FILE% up

echo.
echo === Cleaning Up ===
echo Removing containers...
docker-compose -f %COMPOSE_FILE% down