#!/bin/bash

# 1. Launch the Docker Compose project in detached mode and build if necessary
echo "Starting Docker Compose environment..."
docker-compose up -d --build

# 2. Wait briefly to allow all containers to spin up and register
sleep 3

# 3. Retrieve the list of service names from the current docker-compose.yml
SERVICES=$(docker-compose ps --services)

# 4. Iterate over each service and launch a new terminal window
for SERVICE in $SERVICES; do
    echo "Opening log window for service: $SERVICE"
    
    # Launch a new terminal window natively using gnome-terminal
    # 'docker-compose logs -f' streams the logs in real-time
    # 'exec bash' ensures the terminal window remains open even if the container crashes or stops
    # gnome-terminal -- bash -c "echo '=== Real-time Logs: $SERVICE ==='; echo ''; docker-compose logs -f $SERVICE; exec bash"
    gnome-terminal -- bash -c "echo '=== Real-time Logs: $SERVICE ==='; echo ''; docker-compose logs -f $SERVICE"
done

echo "All log windows successfully opened."

gnome-terminal -- bash -c "echo '=== Validator Script ==='; echo ''; /bin/python3.14 /home/tetemo/Escritorio/UCLM/TRABAJO/navsim/uspace/platgdia/dtblocks/validation/validator.py; exec bash"