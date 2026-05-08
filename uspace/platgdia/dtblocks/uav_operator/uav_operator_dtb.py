import time
import os
import json
import sys
from os import path


current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../../../.."))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

from uspace.uav_operator.uav_operator import UAVOperator, UAV
from uspace.uspace_manager.constants import MissionType, UAVStatus
from uspace.platgdia.dtblocks.uav_operator.time_manager import TimeManager

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

def build_uavs(uav_operator_id, uav_types):
    uavs = {
        MissionType.DELIVERY: {},
        MissionType.PASSENGER_TRANSPORT: {}
    }

    for i in range(len(uav_types)):
        uavs[uav_types[i]][f"UAV_{i}"] = UAV(
            id=f"UAV_{i}",
            type=uav_types[i],
            status=UAVStatus.AVAILABLE,
            battery_level=100,
            operator_id=uav_operator_id,
            location=(0, 0, 0),
            pad_id=""
        )

    return uavs


# Get Configuration Variables
MAX_UAV_OPS = int(os.getenv("MAX_UAV_OPS", 1))
UAV_OP_SERVICE_TYPES = parse_str_list(os.getenv("UAV_OP_SERVICE_TYPES", '[["DELIVERY", "PASSENGER_TRANSPORT"]]'))
UAV_FLEET = parse_str_list(os.getenv("UAV_FLEET", '[["DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT"]]'))
PRIVATE_VERTIPORT_ID = parse_str_list(os.getenv("PRIVATE_VERTIPORT_ID", '["VERT_OP_0"]'))
MQTT_HOST_ADDRESS = os.getenv("MQTT_HOST_ADDRESS")
MQTT_HOST_PORT = int(os.getenv("MQTT_HOST_PORT", "1883"))

# Control Variables
time_manager = TimeManager()
time_manager.start()
time_manager.current_sim_time -= 1 # Adjust to sync with mission manager's time

# Build UAV Operators
uav_operators = []

for i in range(MAX_UAV_OPS):
    uavs = build_uavs(f"UAV_OP_{i}", UAV_FLEET[i])

    uav_operators.append(
        UAVOperator(
            id=f"UAV_OP_{i}", 
            name=f"UAV Operator {i}", 
            service_types=UAV_OP_SERVICE_TYPES[i],
            private_vertiport_operator_id=PRIVATE_VERTIPORT_ID[i],
            uavs=uavs,
            verbose=True
         )
    )
    uav_operators[i].time_manager = time_manager

# Wait a few seconds to ensure that the USpace Manager DTBlock and the Vertiport Operator
# DTBlocks are up and running before connecting to MQTT and making requests
time.sleep(2)

# Connect to MQTT Broker
for uav_op in uav_operators:
    uav_op.connect_mqtt_client(MQTT_HOST_ADDRESS, MQTT_HOST_PORT)

# Register into USpace Manager
for uav_op in uav_operators:
    uav_op.register_into_airspace()

# Request private vertiport information (pads info)
for uav_op in uav_operators:
    uav_op.request_private_vertiport_information()

# Main Loop
try:
    while True:
        time.sleep(1)
        time_manager.current_sim_time += 1

except Exception as e:
    print(f"UAV Operator DTBlock: An error has occurred: {e}")