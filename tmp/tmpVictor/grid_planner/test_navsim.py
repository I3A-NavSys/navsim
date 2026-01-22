import sys
from os import path
import time
 
current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../../.."))
 
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

from uspace.uav_operator.uav_operator import UAVOperator
from uspace.mission_manager.mission_manager import MissionManager
from uspace.vertiport_operator.vertiport_operator import VertiportOperator
from uspace.uspace_manager.uspace_manager import USpaceManager


mission_mgr = MissionManager()
uspace_mgr = USpaceManager()
uav_op1 = UAVOperator(id="UAV_OP_001", name="UAV Operator 1")
vert_op1 = VertiportOperator(
    id="VERT_OP_001", 
    name="Vertiport Operator 1",
    grid_connection=[0, 0, 0]
)
vert_op2 = VertiportOperator(
    id="VERT_OP_002",
    name="Vertiport Operator 2",
    grid_connection=[300, 400, 0]
)

mission_mgr.connect_mqtt_client()
uspace_mgr.connect_mqtt_client()


uav_op1.connect_mqtt_client()
vert_op1.connect_mqtt_client()
vert_op2.connect_mqtt_client()

uav_op1.register_into_airspace()
vert_op1.register_into_airspace()
vert_op2.register_into_airspace()

mission_mgr.request_uav_operator_list()
mission_mgr.request_vertiport_operator_list()

uav_op1.request_route(
    mission_manager_id=mission_mgr.id,
    mission_id="MISSION_001",
    origin_vertiport="VERT_OP_001",
    destination_vertiport="VERT_OP_002",
    takeoff_time=-2,
    landing_time=None,
    stop_time=30
)

time.sleep(5)