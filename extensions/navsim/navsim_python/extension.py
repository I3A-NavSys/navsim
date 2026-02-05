import omni.ext
import logging
import omni.timeline

from navsim_utils.sim_utils import *
from .ui_builder import UIBuilder
from uspace.uav_operator.uav_operator import UAVOperator
from uspace.uav_operator.uav import UAV
from uspace.mission_manager.mission_manager import MissionManager
from uspace.vertiport_operator.vertiport_operator import VertiportOperator
from uspace.vertiport_operator.vertiport_pad import Pad
from uspace.uspace_manager.uspace_manager import USpaceManager
from uspace.uspace_manager.constants import MissionType, UAVStatus, PadStatus

class NavSim(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()

    def on_shutdown(self) -> None:
        pass

    def on_timeline_play(self, event):
        # Resume time manager from pause
        if self.is_simulation_running:
            self.time_manager.resume()
            return
        
        # Set simulation as running
        self.is_simulation_running = True

        # Start NavSim simulation
        self.temporal_scan_scene()
        self.connect_entities_to_mqtt()
        self.request_operator_lists()
        self.request_missions()

    def on_timeline_stop(self, event):

        # Set simulation as not running
        self.is_simulation_running = False

        # Stop NavSim simulation
        self.disconnect_entities_from_mqtt()

    def on_timeline_pause(self, event):
        self.time_manager.pause()


    def init_vars(self):
        # UI Builder
        self.ui_builder = UIBuilder()

        # Logger
        self.logger = logging.getLogger("NavSim")

        # Control
        self.time_manager = TimeManager()
        self.is_simulation_running = False

        # NavSim parameters
        self.mission_manager_amount = 1
        self.uav_operator_amount = 1
        self.vertiport_operator_amount = 10
        self.uspace_manager_amount = 1

        self.mission_managers = []
        self.uav_operators = []
        self.vertiport_operators = []
        self.uspace_managers = []

        # IsaacSim parameters
        self.timeline = omni.timeline.get_timeline_interface()
        timeline_stream = self.timeline.get_timeline_event_stream()
        self.on_stop_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), 
            self.on_timeline_stop
        )
        self.on_play_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), 
            self.on_timeline_play
        )
        self.on_pause_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PAUSE), 
            self.on_timeline_pause
        )

    def build_ui(self) -> None:
        self.ui_builder.build_ui()


    def connect_entities_to_mqtt(self):
        for mission_mgr in self.mission_managers:
            mission_mgr.connect_mqtt_client()
            
        for uspace_mgr in self.uspace_managers:
            uspace_mgr.connect_mqtt_client()
        
        for uav_op in self.uav_operators:
            uav_op.connect_mqtt_client()
            uav_op.register_into_airspace()

        for vert_op in self.vertiport_operators:
            vert_op.connect_mqtt_client()
            vert_op.register_into_airspace()

    def disconnect_entities_from_mqtt(self):
        for mission_mgr in self.mission_managers:
            mission_mgr.disconnect_mqtt_client()
            
        for uspace_mgr in self.uspace_managers:
            uspace_mgr.disconnect_mqtt_client()
        
        for uav_op in self.uav_operators:
            uav_op.disconnect_mqtt_client()

        for vert_op in self.vertiport_operators:
            vert_op.disconnect_mqtt_client()

    def request_operator_lists(self):
        for mission_mgr in self.mission_managers:
            mission_mgr.request_uav_operator_list()
            mission_mgr.request_vertiport_operator_list()

    def request_missions(self):
        for mission_mgr in self.mission_managers:
            mission_mgr.request_uav_mission()

    def temporal_scan_scene(self):
        self.mission_managers = [
            MissionManager(id=f"MISSION_MGR_{i}", name=f"Mission Manager {i}") 
            for i in range(self.mission_manager_amount)
        ]
        self.uav_operators = [
            UAVOperator(
                id=f"UAV_OP_{i}", 
                name=f"UAV Operator {i}",
                private_vertiport_operator_id=f"VERT_OP_{i}",
                uavs={
                    MissionType.DELIVERY: {
                        f"UAV_{j}_DELIVERY": UAV(
                            id=f"UAV_{j}_DELIVERY",
                            operator_id=f"UAV_OP_{i}",
                            type=MissionType.DELIVERY,
                            status=UAVStatus.AVAILABLE,
                            battery_level=100.0,
                            location=(0.0, 0.0, 0.0),
                            pad_id=f"VERT_OP_{i}_PAD_{j}"
                        )
                        for j in range(3)
                    },
                    MissionType.PASSENGER_TRANSPORT: {
                        f"UAV_{j}_PASSENGER": UAV(
                            id=f"UAV_{j}_PASSENGER",
                            operator_id=f"UAV_OP_{i}",
                            type=MissionType.PASSENGER_TRANSPORT,
                            status=UAVStatus.AVAILABLE,
                            battery_level=100.0,
                            location=(0.0, 5.0, 0.0),
                            pad_id=f"VERT_OP_{i}_PAD_{j}",
                        )
                        for j in range(3)
                    }
                }
            ) 
            for i in range(self.uav_operator_amount)
        ]
        self.vertiport_operators = [
            VertiportOperator(
                id=f"VERT_OP_{i}",
                name=f"Vertiport Operator {i}",
                grid_connection={
                    "takeoff": {
                        "heading": [1 if i % 2 == 0 else -1, 0],
                        "position": [50 * (1 if i % 2 == 0 else -1) + i * 100, i * 100, 0]
                    },
                    "landing": {
                        "heading": [1 if i % 2 == 0 else -1, 0],
                        "position": [i * 100 - 50 * (1 if i % 2 == 0 else -1), i * 100, 0]
                    }
                },
                main_pad=Pad(
                    id=f"VERT_OP_{i}_MAIN_PAD",
                    type=MissionType.PASSENGER_TRANSPORT,
                    status=PadStatus.OPERATIVE,
                    operator_id=f"VERT_OP_{i}",
                    location=(i * 100, i * 100, 0)
                ),
                pads={
                    f"VERT_OP_{i}_PAD_{j}": Pad(
                        id=f"VERT_OP_{i}_PAD_{j}",
                        type=MissionType.DELIVERY if j % 2 == 0 else MissionType.PASSENGER_TRANSPORT,
                        status=PadStatus.OPERATIVE,
                        operator_id=f"VERT_OP_{i}",
                        location=(i * 100 + j * 10, i * 100 + j * 10, 0)
                    )
                    for j in range(3)
                },
                security_pad_booking_buffer=40
            )
            for i in range(self.vertiport_operator_amount)
        ]
        self.uspace_managers = [
            USpaceManager(id=f"USPACE_MGR_{i}", name=f"U-Space Manager {i}") 
            for i in range(self.uspace_manager_amount)
        ]

