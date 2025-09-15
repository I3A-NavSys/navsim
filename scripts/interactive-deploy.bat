@echo off
echo === NAVSIM Interactive Deployment ===
echo.
echo This tool will help you deploy the Grid Planner extension to your Isaac Sim installation.
echo Please follow the interactive prompts.
echo.

docker-compose -f docker-compose.interactive-deploy.yml build --no-cache

docker-compose -f docker-compose.interactive-deploy.yml run --rm interactive-deploy

echo.
echo Cleaning up...
docker-compose -f docker-compose.interactive-deploy.yml down

pause