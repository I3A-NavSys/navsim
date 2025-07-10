import omni.ext
from isaacsim.gui.components import ui
from isaacsim.gui.components.element_wrappers import DropDown
import carb.events
import omni.timeline
import omni.physx
from omni.isaac.core.prims import RigidPrimView

import pickle
import base64
import sys, os
import numpy as np
import math
import matplotlib.pyplot as plt
import torch

from navsim_utils.extensions_utils import ExtensionUtils
from uspace.grid_planner.grid_planner import GridPlanner
from uspace.flight_plan.flight_plan import FlightPlan
from fleet.uav_ia_control import UAVcontrol
# from fleet.uav_matrix_control import UAVcontrol

file_path = os.path.dirname(__file__)

project_root_path = os.path.abspath(os.path.join(file_path, '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

project_root_path = project_root_path.replace("\\", "/")

class UAVState:
    IDLE = "idle"
    BUSY = "busy"
    DEAD = "dead"

class RequestState:
    PENDING = "pending"
    COMPLETED = "completed"

class Operator(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()

    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.on_stop_sub = None
        self.on_play_sub = None
        self.event_sub = None

    def on_physics_step(self, step_size:int):
        if self.is_sim_played:
            self.current_time += step_size
            self.uav_control.update(self.current_time, self.uavs, step_size)

    def on_timeline_stop(self, event):
        self.current_time = 0
        self.is_sim_played = False
        self.rigid_prim_view = None
        self.uav_control = None
        
    def on_timeline_play(self, event):
        if self.is_extension_on and not self.is_sim_played:
            # Reset all variables
            self.gp.clear_grid()
            self.clients_requests = {}
            self.uavs = {}
            self.uav_plots = {}
            self.ui_uav_plots_frame.clear()
            self.vertiports_from_id, self.vertiports_from_pos = self.find_vertiports()
            self.print_vertiports()
            self.ui_requests_scrolling_frame.style = {
                "background_color": 0xFF5b5b5b, 
                "margin": 7,
                "height": 150
            }
            self.ui_requests_container.clear()
            self.torch_device = "cuda:0" if torch.cuda.is_available() else "cpu"

            # Get UAVs rigid prim view each time simulation is played, as stage could be modified
            self.rigid_prim_view = RigidPrimView(["/World/UAVs/UAV_*",])
            self.rigid_prim_view.initialize()

            self.init_uavs()
            self.uav_control = UAVcontrol(self.rigid_prim_view, self.torch_device, self.uavs, self.operator_event, 
                                          self.event_stream)
            self.print_uavs()
            self.ui_select_uav_to_plot.repopulate()

            self.is_sim_played = True

    def set_grid_parameters(self):
        self.gp.cell_side = self.ui_grid_cell_size.model.get_value_as_int()
        self.gp.slot_time = self.ui_grid_slot_time.model.get_value_as_int()
        self.gp.x_height = self.ui_grid_x_level_height.model.get_value_as_int()
        self.gp.y_height = self.ui_grid_y_level_height.model.get_value_as_int()

    # ----------------------------------
    # -------- INITIALIZATION ----------
    # ----------------------------------
    def init_vars(self):
        self.rigid_prim_view = None
        self.uav_control = None

        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(self.on_physics_step, True, 0)

        self.timeline = omni.timeline.get_timeline_interface()
        self.on_stop_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop
        )
        self.on_play_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self.on_timeline_play
        )
        
        self.navsim_utils = ExtensionUtils()
        self.gp = GridPlanner()
        self.plot_time_steps = 0.01

        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()
        self.operator_event = carb.events.type_from_string("NavSim.Operator")
        self.uspace_clients_event = carb.events.type_from_string("NavSim.USpaceClients")
        self.event_sub = self.event_stream.create_subscription_to_push_by_type(self.operator_event, self.event_listener)

        self.is_extension_on = False
        self.is_sim_played = False
        self.current_time = 0
        self.clients_requests = {}
        self.uavs = {}
        self.uav_plots = {}     # {uav_id: {"amazon_request_1": {"fp": fp, "track_info": track_info}, "amazon_request_2": {"fp": fp, "track_info": track_info}} }
        self.vertiports_from_id = {}
        self.vertiports_from_pos = {}

    def init_uavs(self):
        pos, _ = self.rigid_prim_view.get_world_poses(indices=range(self.rigid_prim_view.count))

        for i in range(self.rigid_prim_view.count):
            uav_id = f"UAV_{i}"
            uav_state = UAVState.IDLE
            uav_time = 0
            uav_pos = pos[i]
            uav_flightplan = None

            self.uavs[uav_id] = {
                "id": uav_id,
                "state": uav_state,
                "time": uav_time,
                "pos": uav_pos,
                "flightplan": uav_flightplan,
                "request": None
            }

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

    # ----------------------------------
    # ------ TRACKED INFORMATION -------
    # ----------------------------------
    def plot_uav_pos(self, uav_id, key):
        fp: FlightPlan = self.uav_plots[uav_id][key]["fp"]
        tracked_info = self.uav_plots[uav_id][key]["tracked_info"]

        fp.position_figure(f"{key}: POSITION", self.plot_time_steps)
        fp.add_UAV_track_pos(f"{key}: POSITION", tracked_info)

    def plot_uav_vel(self, uav_id, key):
        fp: FlightPlan = self.uav_plots[uav_id][key]["fp"]
        tracked_info = self.uav_plots[uav_id][key]["tracked_info"]

        fp.velocity_figure(f"{key}: VELOCITY", self.plot_time_steps)
        fp.add_UAV_track_vel(f"{key}: VELOCITY", tracked_info)

    def plot_uav_acc(self, uav_id, key):
        fp: FlightPlan = self.uav_plots[uav_id][key]["fp"]
        tracked_info = self.uav_plots[uav_id][key]["tracked_info"]

        fp.acceleration_figure(f"{key}: ACCELERATION", self.plot_time_steps)
        # fp.add_UAV_track_pos(f"{uav_id}: ACCELERATION", tracked_info)

    def save_figures(self, uav_id, key):
        pos_fig_name = f"{key}: POSITION"
        vel_fig_name = f"{key}: VELOCITY"
        acc_fig_name = f"{key}: ACCELERATION"
        
        id = uav_id.replace("/", "_")
        path = project_root_path + "/sims/figures" + f"/{id}_{key}"
        
        if plt.fignum_exists(pos_fig_name):     plt.figure(pos_fig_name).savefig(fname=path + "_pos.svg")
        if plt.fignum_exists(vel_fig_name):     plt.figure(vel_fig_name).savefig(fname=path + "_vel.svg")
        if plt.fignum_exists(acc_fig_name):     plt.figure(acc_fig_name).savefig(fname=path + "_acc.svg")

    def export_request_tracking_data(self, uav_id, key):
        fp: FlightPlan = self.uav_plots[uav_id][key]["fp"]
        tracked_info = self.uav_plots[uav_id][key]["tracked_info"]
        tracked_info_trace_rows = len(tracked_info)
        tracked_info_trace_cols = 7

        waypoints = [[wp.t, wp.pos[0], wp.pos[1], wp.pos[2]] for wp in fp.waypoints]

        fp_trace = fp.trace(self.plot_time_steps)
        tracked_info_trace = np.zeros((tracked_info_trace_rows, tracked_info_trace_cols))
        for i in range(tracked_info_trace_rows):
            wp = tracked_info[i]

            tracked_info_trace[i, 0] = wp.t
            tracked_info_trace[i, 1] = wp.pos[0]
            tracked_info_trace[i, 2] = wp.pos[1]
            tracked_info_trace[i, 3] = wp.pos[2]
            tracked_info_trace[i, 4] = wp.vel[0]
            tracked_info_trace[i, 5] = wp.vel[1]
            tracked_info_trace[i, 6] = wp.vel[2]

        id = uav_id.replace("/", "_")
        waypoints_path = project_root_path + "/sims/exported_data" + f"/{id}_{key}_waypoints.csv"
        fp_path = project_root_path + "/sims/exported_data" + f"/{id}_{key}_flightplan.csv"
        tracked_info_path = project_root_path + "/sims/exported_data" + f"/{id}_{key}_tracked_info.csv"

        np.savetxt(waypoints_path, waypoints, delimiter=", ", fmt="%s")
        np.savetxt(fp_path, fp_trace, delimiter=", ", fmt="%s")
        np.savetxt(tracked_info_path, tracked_info_trace, delimiter=", ", fmt="%s")

    # ----------------------------------
    # -- EVENTS AND REQUESTS HANDLING --
    # ----------------------------------
    def event_listener(self, event):
        if not event.payload["is_request"]:
            self.switch_on_off(event.payload["state"], is_from_event=True)
            return

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

                self.print_new_request(client_id, request_id, init_time, end_time, origin, destination)
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
            pass
            # TODO: Esto no funciona
            # i = closest_uav["pos"][0] // self.gp.cell_side
            # j = closest_uav["pos"][1] // self.gp.cell_side

            # i2 = request_origin[0] // self.gp.cell_side
            # j2 = request_origin[1] // self.gp.cell_side

            # init_time_slot = math.ceil(self.current_time / self.gp.slot_time)

            # route, _ = self.gp.get_best_route(2, (i, j), (i2, j2), init_time_slot, init_time_slot)

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

        if ((init_pos[1] // self.gp.cell_side) % 2 == 0):       init_heading = [1, 0]
        else:                                                   init_heading = [-1, 0]

        if ((end_pos_1[1] // self.gp.cell_side) % 2 == 0):      end_heading = [1, 0]
        else:                                                   end_heading = [-1, 0]

        # Initial takeoff waypoint
        fp.set_waypoint(time=init_time, pos=init_pos, vel=init_vel, heading=init_heading)
        # Final landing waypoints
        fp.set_waypoint(time=end_time_1, pos=end_pos_1, vel=end_vel_1, heading=end_heading)
        fp.set_waypoint(time=end_time_2, pos=end_pos_2, vel=end_vel_2, heading=end_heading)
        fp.set_waypoint(time=end_time_3, pos=end_pos_3, vel=end_vel_3, heading=end_heading)

        fp.connect_waypoints()
        # fp.remove_negative_time()
        # fp.postpone(self.current_time + 1 * self.gp.slot_time)
        # fp.postpone(self.current_time + 0.1)

    def send_flightplan(self, uav_id, fp):
        # uav_event = carb.events.type_from_string("NavSim." + uav_id)
        # serialized_fp = base64.b64encode(pickle.dumps(fp)).decode('utf-8')
        # self.event_stream.push(uav_event, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})
        self.uavs[uav_id]["flightplan"] = fp
        
    def inform_client(self, client_id, request_id):
        self.event_stream.push(
            self.uspace_clients_event, 
            payload={
                "is_request": True,
                "client_id": client_id, 
                "request_id": request_id, 
                "state": RequestState.COMPLETED
            }
        )

    # ----------------------------------
    # ---- UI BUILDING AND HANDLING ----
    # ----------------------------------
    def build_ui(self):
        self.window = ui.Window("OP: NavSim - Operator", width=300, height=300)
        self.window.deferred_dock_in("Layers")
        self.window.frame.set_style(self.navsim_utils.Window_dark_style)

        with self.window.frame:
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED):
                with ui.VStack(spacing=self.navsim_utils.SPACING_S, height=0):
                    # Title
                    ui.Spacer(height=10)
                    ui.Label("NAVSIM - OPERATOR", alignment=ui.Alignment.CENTER, style={"font_size": 20, "font_weight": "bold"})
                    ui.Spacer(height=5)

                    # On/Off button
                    self.on_off_button = ui.ToolButton(
                        text="ON", 
                        height=30, 
                        clicked_fn=lambda state=False, is_from_event=False: self.switch_on_off(state, is_from_event), 
                        style={"background_color": ui.color("#6f9523")}
                    )

                    # GridPlanner parameters
                    ui.Label("GRID PARAMETERS", alignment=ui.Alignment.CENTER)
                    
                    with ui.HStack():
                        ui.Label("Cell size")
                        self.ui_grid_cell_size = ui.IntField()
                        self.ui_grid_cell_size.model.set_value(100)
                    with ui.HStack():
                        ui.Label("Slot time")
                        self.ui_grid_slot_time = ui.IntField()
                        self.ui_grid_slot_time.model.set_value(10)
                    with ui.HStack():
                        ui.Label("X level height")
                        self.ui_grid_x_level_height = ui.IntField()
                        self.ui_grid_x_level_height.model.set_value(60)
                    with ui.HStack():
                        ui.Label("Y level height")
                        self.ui_grid_y_level_height = ui.IntField()
                        self.ui_grid_y_level_height.model.set_value(100)

                    self.ui_grid_set_params = ui.Button("SET PARAMETERS", height=50, clicked_fn=self.set_grid_parameters)

                    ui.Separator()

                    # UAVs collapsable
                    self.ui_uavs_collapsable = ui.CollapsableFrame("UAVs", collapsed=False,
                                                                   style=self.navsim_utils.CollapsableFrame_style)
                    with self.ui_uavs_collapsable:
                        self.ui_uavs_scrolling_frame = ui.ScrollingFrame(
                            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                            style={"background_color": 0xFF5b5b5b, "margin":7}, 
                            height=150
                        )
                        
                        with self.ui_uavs_scrolling_frame:
                            self.ui_uavs_container = ui.VStack(height=0)

                    # Client requests collapsable
                    self.ui_requests_collapsable = ui.CollapsableFrame("Requests", collapsed=False,
                                                                        style=self.navsim_utils.CollapsableFrame_style)
                    with self.ui_requests_collapsable:
                        self.ui_requests_scrolling_frame = ui.ScrollingFrame(
                            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                            style={"background_color": 0xFF5b5b5b, "margin":7}, 
                            height=150
                        )

                        with self.ui_requests_scrolling_frame:
                            self.ui_requests_container = ui.VStack(height=0)
                        
                    # Vertiports collapsable
                    self.ui_vertiports_collapsable = ui.CollapsableFrame("Vertiports", collapsed=False,
                                                                        style=self.navsim_utils.CollapsableFrame_style)
                    with self.ui_vertiports_collapsable:
                        self.ui_vertiports_scrolling_frame = ui.ScrollingFrame(
                            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                            style={"background_color": 0xFF5b5b5b, "margin":7}, 
                            height=150
                        )
                        
                        with self.ui_vertiports_scrolling_frame:
                            self.ui_vertiports_container = ui.VStack(height=0)

                    # UAV plots collapsable
                    self.ui_uav_plots_collapsable = ui.CollapsableFrame("UAV plots", collapsed=False,
                                                                        style=self.navsim_utils.CollapsableFrame_style)
                    with self.ui_uav_plots_collapsable:
                        with ui.VStack(height=0):
                            self.ui_select_uav_to_plot = DropDown("Select UAV", 
                                                                  populate_fn=self.populate_select_uav_to_plot,
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

    def switch_on_off(self, state, is_from_event):
        model = self.on_off_button.model
        model_value = model.get_value_as_bool()

        if is_from_event:
            internal_state = state
            model.set_value(state)
        else:
            internal_state = model_value

        if internal_state:
            self.switch_extension_state(on=True, is_from_event=is_from_event)

            style={"background_color": ui.color("#952323")}
            self.on_off_button.set_style(style)
            self.on_off_button.text = "OFF"

        else:
            self.switch_extension_state(on=False, is_from_event=is_from_event)
            
            style={"background_color": ui.color("#6f9523")}
            self.on_off_button.set_style(style)
            self.on_off_button.text = "ON"

    def switch_extension_state(self, on, is_from_event):
        # Update internal state
        self.is_extension_on = on

        if not is_from_event:
            # Update Clients extension state
            self.event_stream.push(
                self.uspace_clients_event, 
                payload={
                    "is_request": False, 
                    "state": on
                }
            )

    def populate_select_uav_to_plot(self):
        return list(self.uavs.keys())

    def update_uav_plots_frame(self, uav_id):
        self.ui_uav_plots_frame.clear()

        if uav_id in self.uav_plots:
            uav_plots = self.uav_plots[uav_id]
        
            with self.ui_uav_plots_frame:
                with ui.VStack(heigth=0):
                    for key in uav_plots.keys():
                        with ui.HStack(spacing=self.navsim_utils.SPACING_S):
                            ui.Label(key)

                            ui.Button(text="PLOT POS", 
                                    clicked_fn=lambda uav_id=uav_id, key=key: self.plot_uav_pos(uav_id, key))
                            
                            ui.Button(text="PLOT VEL", 
                                    clicked_fn=lambda uav_id=uav_id, key=key: self.plot_uav_vel(uav_id, key))
                            
                            ui.Button(text="PLOT ACC", 
                                    clicked_fn=lambda uav_id=uav_id, key=key: self.plot_uav_acc(uav_id, key))
                        
                        with ui.HStack(spacing=self.navsim_utils.SPACING_S):
                            ui.Button(text="SAVE ACTIVE FIGURES", clicked_fn=lambda uav_id=uav_id, key=key: self.save_figures(uav_id, key))
                            ui.Button(text="EXPORT DATA", clicked_fn=lambda uav_id=uav_id, key=key: self.export_request_tracking_data(uav_id, key))

                        ui.Separator()

    def print_uavs(self):
        self.ui_uavs_container.clear()

        for uav_id, value in self.uavs.items():
            if value["request"]:    request = f"{value['request']['client_id']} - {value['request']['request_id']}"  
            else:                   request = "None"
            self.print_new_uav(uav_id, value["state"], value["time"], value["pos"], request)

    def print_new_uav(self, uav_id, state, time, pos, request):        
        id_label = ui.Label(f"ID: {uav_id}\n")
        state_label = ui.Label(f"State: {state}\n")
        time_label = ui.Label(f"Time: {time}\n")
        pos_label = ui.Label(f"Position: {pos}\n")
        request_label = ui.Label(f"Request: {request}\n")
        spacer = ui.Spacer(height=10)

        self.ui_uavs_container.add_child(id_label)
        self.ui_uavs_container.add_child(state_label)
        self.ui_uavs_container.add_child(time_label)
        self.ui_uavs_container.add_child(pos_label)
        self.ui_uavs_container.add_child(request_label)
        self.ui_uavs_container.add_child(spacer)

    def print_requests(self):
        self.ui_requests_container.clear()

        for client_id, requests in self.clients_requests.items():
            for request_id, request in requests.items():
                self.print_new_request(
                    client_id, 
                    request_id, 
                    request["init_time"], 
                    request["end_time"], 
                    request["origin"], 
                    request["destination"]
                )

    def print_new_request(self, client_id, request_id, init_time, end_time, origin, destination):        
        client_id_label = ui.Label(f"Client ID: {client_id}\n")
        request_id_label = ui.Label(f"Request ID: {request_id}\n")
        init_time_label = ui.Label(f"Init time: {init_time}\n")
        end_time_label = ui.Label(f"End time: {end_time}\n")
        origin_label = ui.Label(f"Origin: {origin}\n")
        destination_label = ui.Label(f"Destination: {destination}\n")
        spacer = ui.Spacer(height=10)

        self.ui_requests_container.add_child(client_id_label)
        self.ui_requests_container.add_child(request_id_label)
        self.ui_requests_container.add_child(init_time_label)
        self.ui_requests_container.add_child(end_time_label)
        self.ui_requests_container.add_child(origin_label)
        self.ui_requests_container.add_child(destination_label)
        self.ui_requests_container.add_child(spacer)
                    
    def print_vertiports(self):
        self.ui_vertiports_container.clear()

        for key, value in self.vertiports_from_id.items():
            self.print_new_vertiport(key, value["position"], value["model"])

    def print_new_vertiport(self, id, position, model):
        id_label = ui.Label(f"ID: {id}\n")
        position_label = ui.Label(f"Position: {position}\n")
        model_label = ui.Label(f"Model: {model}\n")
        spacer = ui.Spacer(height=10)

        self.ui_vertiports_container.add_child(id_label)
        self.ui_vertiports_container.add_child(position_label)
        self.ui_vertiports_container.add_child(model_label)
        self.ui_vertiports_container.add_child(spacer)
