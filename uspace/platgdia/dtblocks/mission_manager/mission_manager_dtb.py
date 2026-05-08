import time
import os
import json
import sys
from os import path


current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../../../.."))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

from uspace.mission_manager.mission_manager import MissionManager


def parse_str_list(str_list: str) -> list:
    '''
    Parses a string representation of a list into an actual list.
    Args:
        str_list (str): A string representation of a list, e.g., "[1, 2, 3]" or '[["DELIVERY", "PASSENGER_TRANSPORT"]]'.
    Returns:
        list: The parsed list.
    Raises:
        ValueError: If the string cannot be parsed into a list.
    '''
    try:
        return json.loads(str_list)
    
    except json.JSONDecodeError:
        raise ValueError(f"Failed to parse list: {str_list}")


# Get Configuration Variables
MAX_MISSION_MNG = int(os.getenv("MAX_MISSION_MNG", 1))
MAX_REQUEST_TIME = parse_str_list(os.getenv("MAX_REQUEST_TIME", '[20]'))
MISSION_MNG_SERVICE_TYPES = parse_str_list(os.getenv("MISSION_MNG_SERVICE_TYPES", '[["DELIVERY", "PASSENGER_TRANSPORT"]]'))
MQTT_HOST_ADDRESS = os.getenv("MQTT_HOST_ADDRESS")
MQTT_HOST_PORT = int(os.getenv("MQTT_HOST_PORT", "1883"))

# Control variables
current_time = 0

# Build Mission Managers
mission_managers = [
    MissionManager(
        id=f"MSSN_MNG_{i}", 
        name=f"Mission Manager {i}", 
        verbose=True
    )
    for i in range(MAX_MISSION_MNG)
]

# Wait a few seconds to ensure that the USpace Manager and UAV Operator DTBlocks
# are up and running before connecting to MQTT and making requests
time.sleep(3)

# Connect to MQTT Broker
for mng in mission_managers:
    mng.connect_mqtt_client(MQTT_HOST_ADDRESS, MQTT_HOST_PORT)

# Request UAV and Vertiport operators list
for mng in mission_managers:
    mng.request_uav_operator_list()
    mng.request_vertiport_operator_list()

# Main Loop
try:
    while True:
        time.sleep(1)
        current_time += 1

        print(f"\nMission Manager DTBlock: Current Time: {current_time}")

        for i in range(MAX_MISSION_MNG):
            if current_time % MAX_REQUEST_TIME[i] == 0:
                mission_managers[i].request_uav_mission(current_time)

except Exception as e:
    print(f"Mission Manager DTBlock: An error has occurred: {e}")