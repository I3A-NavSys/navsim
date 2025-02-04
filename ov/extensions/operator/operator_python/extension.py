import omni.ext
import omni.ui as ui
from omni.isaac.ui.element_wrappers import DropDown
import carb.events
import omni.timeline
import omni.physx

import pickle
import base64
import sys, os
import numpy as np
import math

from navsim_utils.extensions_utils import ExtensionUtils
from uspace.grid_planner.grid_planner import GridPlanner
from uspace.flight_plan.flight_plan import FlightPlan

file_path = os.path.dirname(__file__)

project_root_path = os.path.abspath(os.path.join(file_path, '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

class UAVState:
    IDLE = "idle"
    BUSY = "busy"
    DEAD = "dead"

class RequestState:
    PENDING = "pending"
    COMPLETED = "completed"

class Operator(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()

    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.on_stop_sub = None
        self.on_play_sub = None

    def on_physics_step(self, step_size:int):
        self.current_time += step_size

    def on_timeline_stop(self, event):
        self.current_time = 0
        
    def on_timeline_play(self, event):
        self.gp.clear_grid()
        self.clients_requests = {}
        self.uavs = {}
        self.uav_plots = {}
        self.vertiports_from_id, self.vertiports_from_pos = self.find_vertiports()
        self.print_vertiports()

        self.print_clients()

    def init_vars(self):
        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(self.on_physics_step, True, 0)

        self.timeline = omni.timeline.get_timeline_interface()
        self.on_stop_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop)
        self.on_play_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self.on_timeline_play)
        
        self.navsim_utils = ExtensionUtils()
        self.gp = GridPlanner()

        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()
        self.operator_event = carb.events.type_from_string("NavSim.Operator")
        self.uspace_clients_event = carb.events.type_from_string("NavSim.USpaceClients")
        self.event_sub = self.event_stream.create_subscription_to_push_by_type(self.operator_event, self.event_listener)

        self.current_time = 0
        self.clients_requests = {}
        self.uavs = {}
        self.uav_plots = {}     # {uav_id: {"amazon_request_1": {"fp": fp, "track_info": track_info}, "amazon_request_2": {"fp": fp, "track_info": track_info}} }
        self.vertiports_from_id = {}
        self.vertiports_from_pos = {}

    def find_vertiports(self):
        vertiports_prims = self.navsim_utils.get_vertiport_prims()
        vertiports_from_id = {}
        vertiports_from_pos = {}
        
        for prim in vertiports_prims:
            id = prim.GetAttribute("NavSim:id").Get()
            position = prim.GetAttribute("xformOp:translate").Get()
            model = prim.GetAttribute("NavSim:model").Get()

            vertiports_from_id[id] = {"position": position, "model": model}
            vertiports_from_pos[position] = id

        return vertiports_from_id, vertiports_from_pos

    def event_listener(self, event):
        sender = event.payload["sender"]

        match sender:
            case "uav":
                uav_id = event.payload["id"]
                uav_state = event.payload["state"]
                uav_time = event.payload["time"]
                uav_pos = pickle.loads(base64.b64decode(event.payload["pos"]))
                uav_flightplan = pickle.loads(base64.b64decode(event.payload["flightplan"]))

                if uav_id in self.uavs:
                    self.check_request_completed(uav_id, uav_state, uav_flightplan, event)

                    self.uavs[uav_id]["id"] = uav_id
                    self.uavs[uav_id]["state"] = uav_state
                    self.uavs[uav_id]["time"] = uav_time
                    self.uavs[uav_id]["pos"] = uav_pos
                    self.uavs[uav_id]["flightplan"] = uav_flightplan

                else:
                    self.uavs[uav_id] = {
                        "id": uav_id,
                        "state": uav_state,
                        "time": uav_time,
                        "pos": uav_pos,
                        "flightplan": uav_flightplan,
                        "request": None
                    }

                    self.ui_select_uav_to_plot.repopulate()

                self.print_uavs()

            case "client":
                client_id = event.payload["client_id"]
                request_id = event.payload["request_id"]
                init_time = event.payload["init_time"]
                end_time = event.payload["end_time"]
                origin = event.payload["origin"]
                destination = event.payload["destination"]

                if client_id not in self.clients_requests:
                    self.clients_requests[client_id] = {}

                self.clients_requests[client_id][request_id] = {
                    "init_time": init_time,
                    "end_time": end_time,
                    "origin": origin,
                    "destination": destination
                }

                self.print_clients()
                self.process_request(client_id, request_id)

    def check_request_completed(self, uav_id, uav_state, uav_flightplan, event):
        # Check if uav has completed the request
        if self.uavs[uav_id]["state"] == UAVState.BUSY and uav_state == UAVState.IDLE:
            # Inform the client that the request was completed
            self.inform_client(self.uavs[uav_id]["request"]["client_id"], 
                                self.uavs[uav_id]["request"]["request_id"])
            
            # Store uav tracked info
            tracked_info = pickle.loads(base64.b64decode(event.payload["tracked_info"]))
            client_id = self.uavs[uav_id]["request"]["client_id"]
            request_id = self.uavs[uav_id]["request"]["request_id"]

            if uav_id in self.uav_plots:
                self.uav_plots[uav_id][f"{client_id}_{request_id}"] = {
                    "fp": uav_flightplan, 
                    "tracked_info": tracked_info
                }

            else:
                self.uav_plots[uav_id] = {}
                self.uav_plots[uav_id][f"{client_id}_{request_id}"] = {
                    "fp": uav_flightplan, 
                    "tracked_info": tracked_info
                }

            # Reset uav request
            self.uavs[uav_id]["request"] = None

            # If the current selected uav to see its plots is the one which finished the request, we update the list
            if self.ui_select_uav_to_plot.get_selection() == uav_id:
                self.update_uav_plots_frame(uav_id)

    def print_uavs(self):
        final_string = ""
        for value in self.uavs.values():
            string = "ID: " + value["id"] + "\n"
            string += "State: " + value["state"] + "\n"
            string += "Time: "+ str(value["time"]) + "\n"
            string += "Position: " + str(value["pos"]) + "\n"
            if value["request"] is None:
                string += "Request: None\n"
            else:
                string += "Request: " + value["request"]["client_id"] + " - " + value["request"]["request_id"] + "\n"

            string += "\n"

            final_string += string

        self.ui_uavs_label.text = final_string

    def print_clients(self):
        final_string = ""
        for key, values in self.clients_requests.items():
            client_id = key
            for key, value in values.items():
                string = "Client ID: " + client_id + "\n"
                string += "Request ID: " + key + "\n"
                string += "Init time: " + str(value["init_time"]) + "\n"
                string += "End time: " + str(value["end_time"]) + "\n"
                string += "Origin: " + str(value["origin"]) + "\n"              # Given by a vertiport id
                string += "Destination: " + str(value["destination"]) + "\n"    # Given by a vertiport id
                string += "\n"

                final_string += string

        self.ui_clients_label.text = final_string
                    
    def print_vertiports(self):
        final_string = ""
        for key, value in self.vertiports_from_id.items():
            string = "ID: " + key + "\n"
            string += "Position: " + str(value["position"]) + "\n"
            string += "Model: " + value["model"] + "\n"
            string += "\n"

            final_string += string

        self.ui_vertiports_label.text = final_string

    def plot_uav_pos(self, uav_id, key):
        fp: FlightPlan = self.uav_plots[uav_id][key]["fp"]
        tracked_info = self.uav_plots[uav_id][key]["tracked_info"]

        fp.position_figure(f"{uav_id}: POSITION", 0.01)
        fp.add_UAV_track_pos(f"{uav_id}: POSITION", tracked_info)

    def plot_uav_vel(self, uav_id, key):
        fp: FlightPlan = self.uav_plots[uav_id][key]["fp"]
        tracked_info = self.uav_plots[uav_id][key]["tracked_info"]

        fp.velocity_figure(f"{uav_id}: VELOCITY", 0.01)
        fp.add_UAV_track_vel(f"{uav_id}: VELOCITY", tracked_info)

    def plot_uav_acc(self, uav_id, key):
        fp: FlightPlan = self.uav_plots[uav_id][key]["fp"]
        tracked_info = self.uav_plots[uav_id][key]["tracked_info"]

        fp.acceleration_figure(f"{uav_id}: ACCELERATION", 0.01)
        # fp.add_UAV_track_pos(f"{uav_id}: ACCELERATION", tracked_info)

    def process_request(self, client_id, request_id):
        request = self.clients_requests[client_id][request_id]

        request_origin = self.vertiports_from_id[request["origin"]]["position"]
        request_destination = self.vertiports_from_id[request["destination"]]["position"]

        idle_uavs = [uav for uav in self.uavs.values() if uav["state"] == UAVState.IDLE]
        if not idle_uavs:   return
        uav_distances_to_origin = [np.linalg.norm(abs(uav["pos"] - request_origin)) for uav in idle_uavs]
        closest_uav_i = np.argmin(uav_distances_to_origin)
        closest_uav = idle_uavs[closest_uav_i]

        if uav_distances_to_origin[closest_uav_i] < 2:
            # Get node pos from closest uav as initial node
            i = request_origin[0] // self.gp.cell_side
            j = request_origin[1] // self.gp.cell_side
            # Get node pos from destination as final node
            i2 = request_destination[0] // self.gp.cell_side
            j2 = request_destination[1] // self.gp.cell_side
            # Round up the initial time
            init_time_slot = math.ceil(request["init_time"] / self.gp.slot_time)
            # Round down the end time
            end_time_slot = math.floor(request["end_time"] / self.gp.slot_time)

            route, _ = self.gp.get_best_route(2, (i, j), (i2, j2), init_time_slot, end_time_slot)

            if route:
                self.gp.reserve_nodes(route)

            fp = self.gp.get_flightplan_from_route(route)

            self.add_takeoff_landing_wps(fp, request_origin[:2], request_destination[:2])

            self.send_flightplan(closest_uav["id"], fp)

            self.uavs[closest_uav["id"]]["request"] = {"client_id": client_id, "request_id": request_id}
            self.print_uavs()

        # The closest uav is not at the origin
        else:
            i = closest_uav["pos"][0] // self.gp.cell_side
            j = closest_uav["pos"][1] // self.gp.cell_side

            i2 = request_origin[0] // self.gp.cell_side
            j2 = request_origin[1] // self.gp.cell_side

            init_time_slot = math.ceil(self.current_time / self.gp.slot_time)

            route, _ = self.gp.get_best_route(2, (i, j), (i2, j2), init_time_slot, init_time_slot)

    def add_takeoff_landing_wps(self, fp: FlightPlan, init_pos, end_pos):
        init_time = fp.init_time() - 2 * self.gp.slot_time
        end_time_1 = fp.finish_time() + 2 * self.gp.slot_time
        end_time_2 = fp.finish_time() + 3 * self.gp.slot_time
        end_time_3 = fp.finish_time() + 4 * self.gp.slot_time

        init_pos = [init_pos[0], init_pos[1], 1.75]
        end_pos_1 = [end_pos[0], end_pos[1], 20]
        end_pos_2 = [end_pos[0], end_pos[1], 3]
        end_pos_3 = [end_pos[0], end_pos[1], 1.75]

        init_vel = [0, 0, 0]
        end_vel_1 = [0, 0, -3]
        end_vel_2 = [0, 0, -0.2]
        end_vel_3 = [0, 0, 0]

        if ((init_pos[1] // self.gp.cell_side) % 2 == 0):      heading = [1, 0]
        else:                           heading = [-1, 0]

        # Initial takeoff waypoint
        fp.set_waypoint(time=init_time, pos=init_pos, vel=init_vel, heading=heading)
        # Final landing waypoints
        fp.set_waypoint(time=end_time_1, pos=end_pos_1, vel=end_vel_1, heading=heading)
        fp.set_waypoint(time=end_time_2, pos=end_pos_2, vel=end_vel_2, heading=heading)
        fp.set_waypoint(time=end_time_3, pos=end_pos_3, vel=end_vel_3, heading=heading)

        fp.connect_waypoints()
        # fp.remove_negative_time()
        # fp.postpone(self.current_time + 1 * self.gp.slot_time)
        # fp.postpone(self.current_time + 0.1)

    def send_flightplan(self, uav_id, fp):
        uav_event = carb.events.type_from_string("NavSim." + uav_id)
        serialized_fp = base64.b64encode(pickle.dumps(fp)).decode('utf-8')
        self.event_stream.push(uav_event, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})
        
    def inform_client(self, client_id, request_id):
        self.event_stream.push(self.uspace_clients_event, payload={"client_id": client_id, "request_id": request_id, 
                                                                   "state": RequestState.COMPLETED})

    def build_ui(self):
        self.window = ui.Window("OP: NavSim - Operator", width=300, height=300)
        self.window.deferred_dock_in("Layers")
        self.window.frame.set_style(self.navsim_utils.Window_dark_style)

        with self.window.frame:
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED):
                with ui.VStack(spacing=self.navsim_utils.SPACING_S, height=0):
                    # UAVs collapsable
                    self.ui_uavs_collapsable = ui.CollapsableFrame("UAVs", collapsed=False,
                                                                   style=self.navsim_utils.CollapsableFrame_style)
                    with self.ui_uavs_collapsable:
                        self.ui_uavs_label = ui.Label("", padding=self.navsim_utils.LABEL_PADDING)

                    # Clients collapsable
                    self.ui_clients_collapsable = ui.CollapsableFrame("Clients", collapsed=False,
                                                                        style=self.navsim_utils.CollapsableFrame_style)
                    with self.ui_clients_collapsable:
                        self.ui_clients_label = ui.Label("")

                    self.ui_vertiports_collapsable = ui.CollapsableFrame("Vertiports", collapsed=False,
                                                                        style=self.navsim_utils.CollapsableFrame_style)
                    with self.ui_vertiports_collapsable:
                        self.ui_vertiports_label = ui.Label("")

                    self.ui_uav_plots_collapsable = ui.CollapsableFrame("UAV plots", collapsed=False,
                                                                        style=self.navsim_utils.CollapsableFrame_style)
                    with self.ui_uav_plots_collapsable:
                        with ui.VStack(height=0):
                            self.ui_select_uav_to_plot = DropDown("Select UAV", 
                                                                  populate_fn=self.populate_select_usv_to_plot,
                                                                  on_selection_fn=self.update_uav_plots_frame)
                            self.ui_select_uav_to_plot.repopulate()

                            with ui.ZStack(style={"margin":20}):
                                ui.Rectangle(height=150, style={"background_color": 0xFF5b5b5b, 
                                            "border_radius": 10, 
                                            "corner_flag": ui.CornerFlag.ALL,})
                                
                                self.ui_uav_plots_frame = ui.ScrollingFrame(
                                            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                            style={"background_color": 0xFF5b5b5b, "margin":5}, height=150)
                                

    def populate_select_usv_to_plot(self):
        return list(self.uavs.keys())

    def update_uav_plots_frame(self, uav_id):
        self.ui_uav_plots_frame.clear()

        if uav_id in self.uav_plots:
            uav_plots = self.uav_plots[uav_id]
        
            with self.ui_uav_plots_frame:
                with ui.VStack(heigth=0):
                    for key in uav_plots.keys():
                        ui.Label(key, alignment=ui.Alignment.CENTER)
                        with ui.HStack(spacing=self.navsim_utils.SPACING_S):
                            ui.Button(text="PLOT POS", 
                                    clicked_fn=lambda uav_id=uav_id, key=key: self.plot_uav_pos(uav_id, key),
                                    style={"background_color": 0xFF5b5b5b})
                            ui.Button(text="PLOT VEL", 
                                    clicked_fn=lambda uav_id=uav_id, key=key: self.plot_uav_vel(uav_id, key))
                            ui.Button(text="PLOT ACC", 
                                    clicked_fn=lambda uav_id=uav_id, key=key: self.plot_uav_acc(uav_id, key))
                        
                        ui.Separator()