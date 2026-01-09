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
 
uav_op = UAVOperator(id="UAV_OP_001", name="Main UAV Operator")
vert_op = VertiportOperator(id="VERT_OP_001", name="Main Vertiport Operator")
mission_mgr = MissionManager()
uspace_mgr = USpaceManager()
 
uspace_mgr.connect_mqtt_client()
uspace_mgr.subcribe_mqtt_topic("airspace/operators/register/uav")
uspace_mgr.subcribe_mqtt_topic("airspace/operators/register/vertiport")
uav_op.connect_mqtt_client()
uav_op.register_into_airspace()
vert_op.connect_mqtt_client()
vert_op.register_into_airspace()

time.sleep(5)