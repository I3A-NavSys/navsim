import sys
from os import path
import time
 
current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../../.."))
 
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

from uspace.uav_operator.uav_operator import UAVOperator
from uspace.uav_operator.uav import UAV
from uspace.mission_manager.mission_manager import MissionManager
from uspace.vertiport_operator.vertiport_operator import VertiportOperator
from uspace.vertiport_operator.vertiport_pad import Pad
from uspace.uspace_manager.uspace_manager import USpaceManager
from uspace.uspace_manager.constants import MissionType, UAVStatus, PadStatus


mission_mgr = MissionManager(id="MISSION_MGR_0", name="Mission Manager 0")
uspace_mgr = USpaceManager(id="USPACE_MGR_0", name="USpace Manager 0")
uav_op1 = UAVOperator(
    id="UAV_OP_0", 
    name="UAV Operator 0",
    private_vertiport_operator_id="VERT_OP_0"
)
uav_op1.uavs = {
    MissionType.DELIVERY: {
        "UAV_1": UAV(
            id="UAV_1",
            operator_id=uav_op1.id,
            type=MissionType.DELIVERY,
            status=UAVStatus.AVAILABLE,
            battery_level=100.0,
            location=(0.0, 0.0, 0.0),
            pad_id="PAD_1"
        )
    },
    MissionType.PASSENGER_TRANSPORT: {
        "UAV_2": UAV(
            id="UAV_2",
            operator_id=uav_op1.id,
            type=MissionType.PASSENGER_TRANSPORT,
            status=UAVStatus.AVAILABLE,
            battery_level=100.0,
            location=(0.0, 50.0, 0.0),
            pad_id="PAD_2",
        )
    }
}

veriport_operators = []
for j in range(-5, 6, 1):
    direction = 1 if j % 2 == 0 else -1
    vert_op = VertiportOperator(
        id=f"VERT_OP_{j}",
        name=f"Vertiport Operator {j}",
        grid_connection={
            "takeoff": {
                "heading": [direction, 0],
                "position": [50 * direction + j * 100, j * 100, 0]
            },
            "landing": {
                "heading": [direction, 0],
                "position": [j * 100 - 50 * direction, j * 100, 0]
            }
        },
        main_pad=Pad(
            id=f"MAIN_PAD_{j}",
            type=MissionType.PASSENGER_TRANSPORT,
            status=PadStatus.OPERATIVE,
            operator_id=f"VERT_OP_{j}",
            location=(j * 100, j * 100, 0)
        ),
        pads={
            "PAD_1": Pad(
                id="PAD_1",
                type=MissionType.DELIVERY,
                status=PadStatus.OPERATIVE,
                operator_id=f"VERT_OP_{j}",
                location=(j * 100, j * 100 - 30, 0)
            ),
            "PAD_2": Pad(
                id="PAD_2",
                type=MissionType.PASSENGER_TRANSPORT,
                status=PadStatus.OPERATIVE,
                operator_id=f"VERT_OP_{j}",
                location=(j * 100 + 15, j * 100 - 30, 0)
            ),
            "PAD_3": Pad(
                id="PAD_3",
                type=MissionType.PASSENGER_TRANSPORT,
                status=PadStatus.OPERATIVE,
                operator_id=f"VERT_OP_{j}",
                location=(j * 100 - 15, j * 100 - 30, 0)
            ),
        },
        security_pad_booking_buffer=40
    )

    veriport_operators.append(vert_op)

mission_mgr.connect_mqtt_client()
uspace_mgr.connect_mqtt_client()

uav_op1.connect_mqtt_client()
uav_op1.register_into_airspace()

for vert_op in veriport_operators:
    vert_op.connect_mqtt_client()
    vert_op.register_into_airspace()

mission_mgr.request_uav_operator_list()
mission_mgr.request_vertiport_operator_list()

# while True:
time.sleep(2)
mission_mgr.request_uav_mission()
time.sleep(10)