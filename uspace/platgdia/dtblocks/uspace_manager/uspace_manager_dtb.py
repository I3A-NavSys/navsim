import threading
import os
import json
import sys
from os import path


current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../../../.."))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

from uspace.uspace_manager.uspace_manager import USpaceManager


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
ROUTING_ALGORITHM = os.getenv("ROUTING_ALGORITHM", "GRID")
MQTT_HOST_ADDRESS = os.getenv("MQTT_HOST_ADDRESS")
MQTT_HOST_PORT = int(os.getenv("MQTT_HOST_PORT", "1883"))

# Build USpace Manager
uspace_manager = USpaceManager(
    id="USPACE_MNG_1",
    name="USpace Manager 1",
    verbose=True
)

# Connect to MQTT Broker
uspace_manager.connect_mqtt_client(MQTT_HOST_ADDRESS, MQTT_HOST_PORT)

# Main Loop
wait_event = threading.Event()
try:
    # Wait indefinitely until the event is set (which never happens in this case)
    wait_event.wait()
except KeyboardInterrupt:
    pass
except Exception as e:
    print(f"USpace DTBlock: An error has occurred: {e}")