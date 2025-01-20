import omni.ext
import omni.ui as ui
from omni.isaac.ui.element_wrappers import DropDown
import carb.events
import omni.timeline
import omni.physx

import pickle
import base64
import sys, os

from navsim_utils.extensions_utils import ExtensionUtils
from uspace.grid_planner.grid_planner import GridPlanner
from uspace.flight_plan.flight_plan import FlightPlan

file_path = os.path.dirname(__file__)

project_root_path = os.path.abspath(os.path.join(file_path, '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

class GridPlannerExt(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()

    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.on_stop_sub = None

    def on_physics_step(self, step_size:int):
        self.current_time += step_size

    def on_timeline_stop(self, event):
        self.current_time = 0

    def init_vars(self):
        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(self.on_physics_step, True, 0)

        self.timeline = omni.timeline.get_timeline_interface()
        self.on_stop_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop)
        
        self.navsim_utils = ExtensionUtils()
        self.gp = GridPlanner()
        self.fp = FlightPlan()

        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()
        self.operator_event = carb.events.type_from_string("NavSim.Operator")
        self.eventSub = self.event_stream.create_subscription_to_push_by_type(self.operator_event, self.event_listener)

        self.current_time = 0
        self.uavs = {}
        self.clients_requests = {}

    def event_listener(self, event):
        sender = event.payload["sender"]

        match sender:
            case "uav":
                uav_id = event.payload["id"]
                uav_state = event.payload["state"]
                uav_time = event.payload["time"]
                uav_pos = event.payload["pos"]
                uav_flightplan = event.payload["flightplan"]

                self.uavs[uav_id] = {
                    "id": uav_id,
                    "state": uav_state,
                    "time": uav_time,
                    "pos": uav_pos,
                    "flightplan": uav_flightplan
                }

            case "client":
                client_id = event.payload["id"]
                client_request_time = event.payload["request_time"]
                client_request_origin = event.payload["request_origin"]
                client_request_destination = event.payload["request_destination"]

                self.clients_requests[client_id] = {
                    "id": client_id,
                    "request_time": client_request_time,
                    "request_origin": client_request_origin,
                    "request_destination": client_request_destination
                }

    def build_ui(self):
        pass