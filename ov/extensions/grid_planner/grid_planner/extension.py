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

project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

class GridPlannerExt(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()

        self.physx_interface = omni.physx.get_physx_interface()
        self.physics_timer_callback = self.physx_interface.subscribe_physics_step_events(self.on_physics_step)

        self.timeline = omni.timeline.get_timeline_interface()
        self.event_timer_callback = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop)

    def on_shutdown(self):
        pass

    def on_physics_step(self, step_size:int):
        self.current_time += step_size

    def on_timeline_stop(self, event):
        self.current_time = 0

    def init_vars(self):
        self.navsim_utils = ExtensionUtils()
        self.gp = GridPlanner()
        self.fp = FlightPlan()

        self.begin_end_info = {"Takeoff": {"pos": [], "dropdown": None}, "Landing": {"pos": [], "dropdown": None}}
        self.current_time = 0
        self.route = None
        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

    def build_ui(self):
        colors = {"X": 0xFF5555AA, "Y": 0xFF76A371}
        options = ["Takeoff", "Landing"]

        self.window = ui.Window("My Window", width=300, height=300)
        self.window.deferred_dock_in("Layers")
        # self.window.setPosition(25, 25)
        self.window.frame.set_style(self.navsim_utils.Window_dark_style)

        with self.window.frame:
            with ui.VStack(spacing=self.navsim_utils.SPACING_S, style=self.navsim_utils.VStack_B, height=0):
                # UAV selector
                self.UAV_selector_dropdown = self.navsim_utils.build_uav_selector()
                ui.Spacer(height=10)

                # Takeoff and Landing options
                for option in options:
                    with ui.HStack(spacing=self.navsim_utils.SPACING_S):
                        # with ui.HStack():
                        #     ui.Label(option)

                        self.begin_end_info[option]["dropdown"] = DropDown(label=option, 
                                                                           populate_fn=self.populate_dropdown)
                        self.begin_end_info[option]["dropdown"].repopulate()

                        with ui.HStack():
                            with ui.ZStack(width=15):
                                ui.Rectangle(width=15, height=20, style={"background_color": colors["X"], 
                                                                            "border_radius": 3, 
                                                                            "corner_flag": ui.CornerFlag.LEFT})

                                # Axis letter label
                                ui.Label("X", style=self.navsim_utils.Label_A, alignment=ui.Alignment.CENTER)

                            self.begin_end_info[option]["pos"].append(ui.FloatDrag(min=-1000000, max=1000000, step=0.1))

                        with ui.HStack():
                            with ui.ZStack(width=15):
                                ui.Rectangle(width=15, height=20, style={"background_color": colors["Y"], 
                                                                            "border_radius": 3, 
                                                                            "corner_flag": ui.CornerFlag.LEFT})

                                # Axis letter label
                                ui.Label("Y", style=self.navsim_utils.Label_A, alignment=ui.Alignment.CENTER)

                            self.begin_end_info[option]["pos"].append(ui.FloatDrag(min=-1000000, max=1000000, step=0.1))

                with ui.HStack():
                    ui.Label("Time to go:")
                    self.time_to_go = ui.IntDrag(min=0, max=1000000, step=1)

                ui.Spacer(height=10)

                # Buttons
                ui.Button("CLEAR GRID", clicked_fn=self.clear_grid, height=50)
                ui.Button("COMPUTE ROUTE", clicked_fn=self.compute_route, height=50)
                ui.Button("SEND FLIGHTPLAN", clicked_fn=self.send_flightplan, height=50)

    def populate_dropdown(self):
        return ["0", "1"]

    def compute_route(self):
        x = self.begin_end_info["Takeoff"]["pos"][0].model.get_value_as_float()
        y = self.begin_end_info["Takeoff"]["pos"][1].model.get_value_as_float()
        time = self.time_to_go.model.get_value_as_int()
        takeoff_nodes = self.gp.get_take_off_nodes((x, y), time)

        x = self.begin_end_info["Landing"]["pos"][0].model.get_value_as_float()
        y = self.begin_end_info["Landing"]["pos"][1].model.get_value_as_float()
        landing_nodes = self.gp.get_landing_nodes((x, y))

        takeoff_node = int(self.begin_end_info["Takeoff"]["dropdown"].get_selection())
        landing_node = int(self.begin_end_info["Landing"]["dropdown"].get_selection())

        self.route, _, _ = self.gp.get_route(takeoff_nodes[takeoff_node], landing_nodes[landing_node], is_cost=False)
        
        if self.route:
            self.gp.reserve_nodes(self.route)
            self.gp.print_route(self.route)

    def clear_grid(self):
        self.gp.clear_grid()

    def send_flightplan(self):
        if self.UAV_selector_dropdown.get_selection() is None:
            raise Exception("ERROR: No drone selected")
        
        uav = self.navsim_utils.get_prim_by_name(self.UAV_selector_dropdown.get_selection())
        uav_event = carb.events.type_from_string("NavSim." + str(uav.GetPath()))

        fp = self.gp.get_flightplan_from_route(self.route)

        # Add waypoints for the vertiports
        fp.set_waypoint(time=fp.init_time()-15, pos=[-500, 0, 1.75], vel=[0, 0, 0], orientation=[10, 0])
        # fp.set_waypoint(time=5, pos=[-500, 0, 1.75], vel=[0, 0, 0])
        fp.set_waypoint(time=fp.finish_time() + 15, pos=[0, 0, 1.75], vel=[0, 0, 0])

        fp.waypoints[-2].orientation = [10, 0]

        fp.connect_waypoints()

        # fp.smooth_waypoint_speed(1, 1)
        # fp.smooth_waypoint_speed(-2, 1)

        fp.postpone(self.current_time + 5)

        serialized_fp = base64.b64encode(pickle.dumps(fp)).decode('utf-8')

        # Push to the event stream the serialized command
        self.event_stream.push(uav_event, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})

        # fp.position_figure("PosFig", 0.1)
        # fp.velocity_figure("VelFig", 0.1)