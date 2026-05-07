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
SERVICE_TYPES = parse_str_list(os.getenv("SERVICE_TYPES", '[["DELIVERY", "PASSENGER_TRANSPORT"]]'))
MQTT_HOST_ADDRESS = os.getenv("MQTT_HOST_ADDRESS", "127.0.0.1")
MQTT_HOST_PORT = int(os.getenv("MQTT_HOST_PORT", "1883"))

# Build Mission Managers
mission_managers = [
    MissionManager(f"MSSN_MNG_{i}", f"Mission Manager {i}") 
    for i in range(MAX_MISSION_MNG)
]

print(f"Initialized {len(mission_managers)} Mission Managers:")

# Connect to MQTT Broker
for mng in mission_managers:
    mng.connect_mqtt_client(MQTT_HOST_ADDRESS, MQTT_HOST_PORT)

# Testing connection:
# Subscribe to 'test' topic
for mng in mission_managers:
    mng.subscribe_mqtt_topic("test")
    mng.mqtt_client.message_callback_add(
        "test", 
        lambda client, userdata, msg: print(f"{mng.id}: {json.loads(msg.payload.decode())}")
    )
# Publish test message
for mng in mission_managers:
    print(f"{mng.id}: Publishing test message...")
    mng.send_mqtt_msg("test", json.dumps({"message": "Hello from Mission Manager!"}))

try:
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("Shutting down Mission Managers...")
    for mng in mission_managers:
        mng.disconnect_mqtt_client()