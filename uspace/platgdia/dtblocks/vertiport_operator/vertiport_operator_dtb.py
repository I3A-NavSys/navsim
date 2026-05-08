import threading
import time
import os
import json
import sys
from os import path


current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../../../.."))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

from uspace.vertiport_operator.vertiport_operator import VertiportOperator
from uspace.vertiport_operator.vertiport_pad import Pad
from uspace.uspace_manager.constants import MissionType, PadStatus


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
        compatible_str_list = str_list.replace("True", "true").replace("False", "false")
        return json.loads(compatible_str_list)
    
    except json.JSONDecodeError:
        raise ValueError(f"Failed to parse list: {str_list}")
    
def build_grid_connection(location):
    vert_op_grid_connection = {}

    i_grid = location[0] // 100
    heading = [1 if i_grid % 2 == 0 else -1, 0]

    takeff_pos = [
        location[0] + 50 * heading[0],
        location[1],
        grid_x_layer_height
    ]

    landing_pos = [
        location[0] - 50 * heading[0],
        location[1],
        grid_x_layer_height
    ]

    vert_op_grid_connection["takeoff"] = {
        "heading": heading,
        "position": takeff_pos
    }
    vert_op_grid_connection["landing"] = {
        "heading": heading,
        "position": landing_pos
    }

    return vert_op_grid_connection

def build_main_pad(vert_op_id, location):
    return Pad(
        id="MAIN_PAD",
        type=MissionType.PASSENGER_TRANSPORT,
        status=PadStatus.OPERATIVE,
        operator_id=vert_op_id,
        location=(location[0], location[1], 0)
    )
    
def build_pads(vert_op_id, amount, types, location):
    pad_pos_offset = 3
    current_x_pos = (amount // 2) * -pad_pos_offset
    pads = {}

    for i in range(amount):
        pads[f"PAD_{i}"] = Pad(
            id=f"PAD_{i}",
            type=types[i],
            status=PadStatus.OPERATIVE,
            operator_id=vert_op_id,
            location=(location[0] + current_x_pos, location[1] - 30, 0)
        )

        current_x_pos += pad_pos_offset

    return pads


# Get Configuration Variables
MAX_VERT_OPS = int(os.getenv("MAX_VERT_OPS", 2))
VERT_OP_ID = parse_str_list(os.getenv("VERT_OP_ID", '["VERT_OP_0", "VERT_OP_1"]'))
PRIVACY = parse_str_list(os.getenv("PRIVACY", '[true, false]'))
VERT_OP_SERVICE_TYPES = parse_str_list(os.getenv("VERT_OP_SERVICE_TYPES", '[["DELIVERY", "PASSENGER_TRANSPORT"], ["DELIVERY"]]'))
MAX_PADS = parse_str_list(os.getenv("MAX_PADS", '[10, 5]'))
PAD_SERVICE_TYPES = parse_str_list(os.getenv("PAD_SERVICE_TYPES", '[["DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT"], ["DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY"]]'))
LOCATION = parse_str_list(os.getenv("LOCATION", '[[100, 500], [-600, -200]]'))
MQTT_HOST_ADDRESS = os.getenv("MQTT_HOST_ADDRESS")
MQTT_HOST_PORT = int(os.getenv("MQTT_HOST_PORT", "1883"))

# Control Variables
grid_x_layer_height = 60

# Build Vertiport Operators
vertiport_operators = []

for i in range(MAX_VERT_OPS):
    vert_op_location = LOCATION[i]
    
    vert_op_grid_connection = build_grid_connection(vert_op_location)
    maind_pad = build_main_pad(VERT_OP_ID[i], vert_op_location)
    pads = build_pads(VERT_OP_ID[i], MAX_PADS[i], PAD_SERVICE_TYPES[i], vert_op_location)

    vertiport_operators.append(
        VertiportOperator(
            id=VERT_OP_ID[i], 
            name=f"Vertiport Operator {i}", 
            service_types=VERT_OP_SERVICE_TYPES[i], 
            is_private=PRIVACY[i], 
            grid_connection=vert_op_grid_connection,
            main_pad=maind_pad,
            pads=pads,
            verbose=True
        )
    )

# Wait a few seconds to ensure that the USpace Manager DTBlock are up and running before 
# connecting to MQTT and making requests
time.sleep(1)

# Connect to MQTT Broker
for vert_op in vertiport_operators:
    vert_op.connect_mqtt_client(MQTT_HOST_ADDRESS, MQTT_HOST_PORT)

# Register into USpace Manager
for vert_op in vertiport_operators:
    vert_op.register_into_airspace()

# Main Loop
wait_event = threading.Event()
try:
    # Wait indefinitely until the event is set (which never happens in this case)
    wait_event.wait()
except Exception as e:
    print(f"Vertiport Operator DTBlock: An error has occurred: {e}")