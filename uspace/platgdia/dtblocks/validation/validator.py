from dotenv import load_dotenv
from datetime import datetime
import os
import json
import sys
from os import path


current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../../../.."))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

from uspace.mqtt.mqtt_service import MQTTService
from uspace.uspace_manager.constants import Topics, MissionStatus


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
    
def on_mission_msg(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    mission_mng_id = data["mission_manager_id"]
    uav_operator_id = data.get("uav_operator_id")
    mission_id = data["mission_id"]
    mission_type = data.get("mission_type", "")
    mission_status = data.get("mission_status", "")
    stop_list = data.get("stop_list", [])
    stop_times = data.get("stop_times", [])
    landing_time = data.get("landing_time", None)

    start_msg = mission_status == ""

    if f"{mission_mng_id} - {mission_id}" not in missions:
        missions[f"{mission_mng_id} - {mission_id}"] = {
            "start_time": None, 
            "end_time": None, 
            "uav_operator_id": uav_operator_id,
            "mission_type": None,
            "stop_list": None,
            "stop_times": None,
            "landing_time": None,
            "status": None
        }

    if start_msg:
        missions[f"{mission_mng_id} - {mission_id}"]["start_time"] = datetime.now().strftime("%d-%m-%Y %H:%M:%S:%f")
        missions[f"{mission_mng_id} - {mission_id}"]["stop_list"] = stop_list
        missions[f"{mission_mng_id} - {mission_id}"]["mission_type"] = mission_type
        missions[f"{mission_mng_id} - {mission_id}"]["stop_times"] = stop_times
        missions[f"{mission_mng_id} - {mission_id}"]["landing_time"] = landing_time

    else:
        missions[f"{mission_mng_id} - {mission_id}"]["end_time"] = datetime.now().strftime("%d-%m-%Y %H:%M:%S:%f")
        missions[f"{mission_mng_id} - {mission_id}"]["status"] = MissionStatus.IN_PROGRESS

def on_mission_cancelled(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    mission_mng_id = data["mission_manager_id"]
    mission_id = data["mission_id"]

    if f"{mission_mng_id} - {mission_id}" in missions:
        missions[f"{mission_mng_id} - {mission_id}"]["end_time"] = datetime.now().strftime("%d-%m-%Y %H:%M:%S:%f")
        missions[f"{mission_mng_id} - {mission_id}"]["status"] = MissionStatus.CANCELLED

def on_route_requested(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    grid = data["grid"]
    raw_flightplan = data["flightplan"]

    airspace["grid"] = grid
    airspace["flightplans"].append(raw_flightplan)


# Load environment variables
env_path = "/home/tetemo/Escritorio/UCLM/TRABAJO/navsim/uspace/platgdia/dtblocks/.env"
load_dotenv(env_path)

# Get Configuration Variables
MQTT_HOST_ADDRESS = os.getenv("MQTT_HOST_ADDRESS")
MQTT_HOST_PORT = int(os.getenv("MQTT_HOST_PORT", "1883"))

MAX_MISSION_MNG = int(os.getenv("MAX_MISSION_MNG", 1))
MAX_REQUEST_TIME = parse_str_list(os.getenv("MAX_REQUEST_TIME", '[20]'))
MISSION_MNG_SERVICE_TYPES = parse_str_list(os.getenv("MISSION_MNG_SERVICE_TYPES", '[["DELIVERY", "PASSENGER_TRANSPORT"]]'))

MAX_UAV_OPS = int(os.getenv("MAX_UAV_OPS", 1))
UAV_OP_SERVICE_TYPES = parse_str_list(os.getenv("UAV_OP_SERVICE_TYPES", '[["DELIVERY", "PASSENGER_TRANSPORT"]]'))
UAV_FLEET = parse_str_list(os.getenv("UAV_FLEET", '[["DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT"]]'))
PRIVATE_VERTIPORT_ID = parse_str_list(os.getenv("PRIVATE_VERTIPORT_ID", '["VERT_OP_0"]'))

ROUTING_ALGORITHM = os.getenv("ROUTING_ALGORITHM", "GRID")

MAX_VERT_OPS = int(os.getenv("MAX_VERT_OPS", 2))
VERT_OP_ID = parse_str_list(os.getenv("VERT_OP_ID", '["VERT_OP_0", "VERT_OP_1"]'))
PRIVACY = parse_str_list(os.getenv("PRIVACY", '[true, false]'))
VERT_OP_SERVICE_TYPES = parse_str_list(os.getenv("VERT_OP_SERVICE_TYPES", '[["DELIVERY", "PASSENGER_TRANSPORT"], ["DELIVERY"]]'))
MAX_PADS = parse_str_list(os.getenv("MAX_PADS", '[10, 5]'))
PAD_SERVICE_TYPES = parse_str_list(os.getenv("PAD_SERVICE_TYPES", '[["DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT"], ["DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY"]]'))
LOCATION = parse_str_list(os.getenv("LOCATION", '[[100, 500], [-600, -200]]'))

# Control Variables
# {mission_id: {start_time: int, end_time: int, status: MissionStatus}}
missions = {}
# {grid: list[tuple(int, int, str, int)], flightplans: list[dict[str, Any]]}
airspace = {"grid": [], "flightplans": []}
results_file_path = "/home/tetemo/Escritorio/UCLM/TRABAJO/navsim/uspace/platgdia/dtblocks/validation/"

# Build MQTT Client
mqtt_client = MQTTService.build_client("VALIDATOR")

# Connect to MQTT Broker
result = mqtt_client.connect(MQTT_HOST_ADDRESS, MQTT_HOST_PORT)

# Set callbacks for topics
for i in range(MAX_MISSION_MNG):
    mqtt_client.message_callback_add(
        f"{Topics.MISSION_STATUS_UPDATE}/MSSN_MNG_{i}",
        on_mission_msg
    )
    mqtt_client.message_callback_add(
        f"{Topics.CANCEL_MISSION}/MSSN_MNG_{i}",
        on_mission_cancelled
    )

for i in range(MAX_UAV_OPS):
    mqtt_client.message_callback_add(
        f"{Topics.MISSION_UAV_SERVICE}/UAV_OP_{i}",
        on_mission_msg
    )
    mqtt_client.message_callback_add(
        f"{Topics.REQUEST_ROUTE}/UAV_OP_{i}",
        on_route_requested
    )


# Subscribe to necessary topics
# Mission Manager Topics
for i in range(MAX_MISSION_MNG):
    mqtt_client.subscribe(f"{Topics.MISSION_STATUS_UPDATE}/MSSN_MNG_{i}")
    mqtt_client.subscribe(f"{Topics.CANCEL_MISSION}/MSSN_MNG_{i}")

# UAV Operator Topics
for i in range(MAX_UAV_OPS):
    mqtt_client.subscribe(f"{Topics.MISSION_UAV_SERVICE}/UAV_OP_{i}")
    mqtt_client.subscribe(f"{Topics.REQUEST_ROUTE}/UAV_OP_{i}")

# Main loop
try:
    if result == 0:
        mqtt_client.loop_forever()

except KeyboardInterrupt as e:
    pass
except Exception as e:
    print(f"[VALIDATOR] - Something went wrong: {e}")

finally:
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

    print(f"\nSaving results to JSON in {results_file_path}")

    # Save results to a JSON file
    with open(f"{results_file_path}missions_validation.json", "w") as file:
        json.dump(missions, file, indent=4)

    with open(f"{results_file_path}airspace_validation.json", "w") as file:
        json.dump(airspace, file, indent=4)

    # Show results
    # print("\n--- Missions ---")
    # for mission_id, mission_data in missions.items():
    #     print(f"Mission ID: {mission_id}")
    #     print(f"  Start Time: {mission_data['start_time']}")
    #     print(f"  End Time: {mission_data['end_time']}")
    #     print(f"  Status: {mission_data['status']}")

    # print("\n--- Airspace ---")
    # print(f"Grid: {airspace['grid']}")
    # print(f"Flightplans: {airspace['flightplans']}")