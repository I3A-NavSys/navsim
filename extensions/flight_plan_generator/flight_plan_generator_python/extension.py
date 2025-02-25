# Standard library imports
import sys, os

# Related third party imports
import omni.ext
import omni.ui as ui
import carb.events
import pickle
import base64
from omni.isaac.ui.element_wrappers import *
import omni.timeline
import omni.physx


project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
# Add root 'ov' folder to sys.path
if project_root_path not in sys.path:
    sys.path.append(project_root_path)


# Local application/library specific imports
from uspace.flight_plan.flight_plan import FlightPlan
from uspace.flight_plan.waypoint import Waypoint
from navsim_utils.extensions_utils import ExtensionUtils


# Color
KIT_GREEN = 0xFF8A8777

# Label
LABEL_PADDING = 120

# Spacing
SPACING_S = 8
SPACING_M = SPACING_S * 2
SPACING_L = SPACING_M * 2
SPACING_XL = SPACING_L * 2

# Height
MINIMAL_HEIGHT = 0

# Width
MINIMAL_WIDTH = 0

Window_dark_style = {
    "Window": {"background_color": 0xFF444444}
}


VStack_A = {
    "VStack": {
        "margin_width": 10, 
        "margin_height": 0
    }
}


VStack_B = {
    "VStack": {
        "margin_width": 10,
        "margin_height": 5
    }
}


HStack_A = {
    "HStack": {
        "margin_width": 10,
        "margin_height": 5
    }
}


Label_A = {
    "Label": {
        "font_size": 12,
        "color": 0xFFDDDDDD
    }
}


colors = {
    "R": 0xFF5555AA,
    "G": 0xFF76A371,
    "B": 0xFFA07D4F
}


CollapsableFrame_style = {
    "CollapsableFrame": {
        "background_color": 0xFF343432,
        "secondary_color": 0xFF343432,
        "color": 0xFFAAAAAA,
        "border_radius": 4.0,
        "border_color": 0x0,
        "border_width": 0,
        "font_size": 14,
        "padding": 0,
    },
    "HStack::header": {"margin": 5},
    "CollapsableFrame:hovered": {"secondary_color": 0xFF3A3A3A},
    "CollapsableFrame:pressed": {"secondary_color": 0xFF343432},
}

ScrollingFrame_style = {
    "ScrollingFrame": {
        "background_color": 0xFF343432,
        "secondary_color": 0xFF343432,
        "color": 0xFFAAAAAA,
        "border_radius": 4.0,
        "border_color": 0x0,
        "border_width": 0,
        "font_size": 14,
        "padding": 0,
    },
    "HStack::header": {"margin": 5},
    "ScrollingFrame:hovered": {"secondary_color": 0xFF3A3A3A},
    "ScrollingFrame:pressed": {"secondary_color": 0xFF343432},
}

class FlightPlanGenerator(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.

    def on_startup(self, ext_id):
        # Initialize utils instance
        self.extension_utils = ExtensionUtils()
        
        # Field variables
        self.position = [0, 0, 0]
        self.velocity = [0, 0, 0]
        self.time = 0
        self.fly_over = None

        # Drone selector handle
        self.UAV_selector_dropdown = None

        # UI field handles
        self.position_handles : list[ui.FloatDrag] = []
        self.velocity_handles : list[ui.FloatDrag] = []
        self.time_handle = None

        # Checkbox handles
        self.position_check_handle = None
        self.velocity_check_handle = None
        self.time_check_handle = None
        self.fly_over_check_handle = None

        # UI widgets
        self.waypoint_list_collapsable_frame = None
        self.waypoint_list_scrolling_frame = None
        self.waypoint_list = None
        self.empty_waypoint_list_label = None

        # Initialize flight plan
        self.flight_plan = FlightPlan()

        # Get bus event stream
        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

        # Get simulation current time
        self.current_time = 0
        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_step_events(self.on_physics_step)

        self.timeline = omni.timeline.get_timeline_interface()
        self.on_timeline_stop_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop)

        # Build graphical interface
        self.build_window()
                    
    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.on_timeline_stop_sub = None

    def on_physics_step(self, step_size):
        # print(self.current_time)
        self.current_time += step_size

    def on_timeline_stop(self, event):
        self.current_time = 0

    def build_waypoint_frame(self):
        with ui.CollapsableFrame(title="Waypoint", style=CollapsableFrame_style):
            with ui.VStack(spacing=SPACING_S, style=VStack_B):
                ui.Spacer(height=MINIMAL_HEIGHT)

                # Add time field
                with ui.HStack(spacing=SPACING_S):
                    # Time label
                    ui.Label("Time", style=Label_A, width=LABEL_PADDING)
                    self.time_handle = ui.IntDrag(min=0, max=1000000, step=1)

                    # Time field checkbox
                    self.time_check_handle = ui.CheckBox(width=MINIMAL_WIDTH)
                    self.time_check_handle.model.set_value(True)

                # Position fields
                with ui.HStack(spacing=SPACING_S):
                    with ui.HStack(width=LABEL_PADDING):
                        ui.Label("Position", style=Label_A, width=50)
                        ui.Spacer()

                    # Axis fields
                    all_axis = ["X", "Y", "Z"]
                    colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
                    for axis in all_axis:
                        with ui.HStack():
                            with ui.ZStack(width=15):

                                # Colored rectangles
                                ui.Rectangle(width=15, height=20, style={"background_color": colors[axis], 
                                                                            "border_radius": 3, 
                                                                            "corner_flag": ui.CornerFlag.LEFT})

                                # Axis letter label
                                ui.Label(axis, style=Label_A, alignment=ui.Alignment.CENTER)

                            # FloatDrag widgets
                            self.position_handles.append(ui.FloatDrag(min=-1000000, max=1000000, step=0.1))

                    # Position field checkboxes
                    self.position_check_handle = ui.CheckBox(width=MINIMAL_WIDTH)
                    self.position_check_handle.model.set_value(True)

                # Velocity fields
                with ui.HStack(spacing=SPACING_S):
                    with ui.HStack(width=LABEL_PADDING):
                        ui.Label("Velocity", style=Label_A, width=50)
                        ui.Spacer()

                    # Axis fields
                    all_axis = ["X", "Y", "Z"]
                    colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
                    for axis in all_axis:
                        with ui.HStack():
                            with ui.ZStack(width=15):

                                # Colored rectangles
                                ui.Rectangle(width=15, height=20, style={"background_color": colors[axis], 
                                                                            "border_radius": 3, 
                                                                            "corner_flag": ui.CornerFlag.LEFT})

                                # Axis letter label
                                ui.Label(axis, style=Label_A, alignment=ui.Alignment.CENTER)

                            # FloatDrag widgets
                            self.velocity_handles.append(ui.FloatDrag(min=-1000000, max=1000000, step=0.1))

                    # Velocity field checkboxes
                    self.velocity_check_handle = ui.CheckBox(width=MINIMAL_WIDTH)
                    self.velocity_check_handle.model.set_value(True)

                # Fly over field
                with ui.HStack(spacing=SPACING_S):
                    # Fly over label
                    ui.Label("Fly over", style=Label_A, width=LABEL_PADDING)

                    # Fly over field checkbox
                    self.fly_over_check_handle = ui.CheckBox(width=MINIMAL_WIDTH)
                    self.fly_over_check_handle.model.set_value(True)

                # Add waypoint id field
                with ui.HStack(spacing=SPACING_S):
                    # Waypoint id label
                    ui.Label("Waypoint ID", style=Label_A, width=LABEL_PADDING)
                    self.waypoind_id = ui.StringField()

                    # Enabled field checkbox
                    self.check_waypoind_id = ui.CheckBox(width=MINIMAL_WIDTH)
                    self.check_waypoind_id.model.set_value(True)

                # Add buttons
                with ui.VStack(height=MINIMAL_HEIGHT):
                    # Add waypoint button
                    ui.Button("Add Waypoint", height=MINIMAL_HEIGHT, clicked_fn=self.add_waypoint)

                    # Reset flight plan button
                    ui.Button("Reset Waypoints", height=MINIMAL_HEIGHT, clicked_fn=self.reset_waypoints)

    def build_waypoint_list(self):
        self.waypoint_list_collapsable_frame = ui.CollapsableFrame(title="Waypoint list", style=CollapsableFrame_style)
        with self.waypoint_list_collapsable_frame:
            with ui.VStack(spacing=SPACING_S, style=VStack_B):
                # Column labels
                with ui.HStack(style=HStack_A):
                    ui.Label("Label", alignment=ui.Alignment.LEFT)
                    ui.Label("Time", alignment=ui.Alignment.CENTER)
                    ui.Label("Position", alignment=ui.Alignment.LEFT)
                    ui.Label("Velocity", alignment=ui.Alignment.LEFT)
                    ui.Label("Fly over", alignment=ui.Alignment.LEFT)

                ui.Line()

                self.waypoint_list_scrolling_frame = ui.ScrollingFrame(
                    height=250,
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    style=ScrollingFrame_style
                )
                with self.waypoint_list_scrolling_frame:
                    self.waypoint_list = ui.VStack(height=MINIMAL_HEIGHT, spacing=SPACING_S)
                    with self.waypoint_list:
                        ui.Spacer(height=MINIMAL_HEIGHT)
                        self.empty_waypoint_list_label = ui.Label("Waypoint list is empty", style=Label_A, 
                                                                alignment=ui.Alignment.CENTER_TOP)

    def send_flight_plan(self):
        # Check if an UAV has been selected
        if self.UAV_selector_dropdown.get_selection() is None:
            raise Exception("ERROR: No drone selected")
        
        # Get the UAV prim
        selected_UAV = self.extension_utils.get_prim_by_name(self.UAV_selector_dropdown.get_selection())

        # Create the event to send the flightplan to the UAV
        self.UAV_event = carb.events.type_from_string("NavSim." + str(selected_UAV.GetPath()))

        # Create a copy of the flightplan to avoid modifying waypoints' time and be printed in the UI
        flightplan_to_send = self.flight_plan.copy()

        # Set the waypoints' uniform vector velocities
        # flightplan_to_send.set_uniform_velocity()

        # Delay a bit the flightplan
        flightplan_to_send.postpone(self.current_time + 5.0)

        # Serialize flight plan
        serialized_flightplan = base64.b64encode(pickle.dumps(flightplan_to_send)).decode('utf-8')

        # Push to the event stream the serialized command
        self.event_stream.push(self.UAV_event, payload={"method": "eventFn_FlightPlan", "fp": serialized_flightplan})

    def reset_waypoints(self):
        self.waypoint_list.clear()
        self.flight_plan.waypoints = []
        with self.waypoint_list:
            ui.Spacer(height=MINIMAL_HEIGHT)
            self.empty_waypoint_list_label = ui.Label("Waypoint list is empty", style=Label_A, 
                                                        alignment=ui.Alignment.CENTER_TOP)
                    
    def update_variables(self):
        # Time
        if self.time_check_handle.model.get_value_as_bool():
            self.time = self.time_handle.model.as_int
        else:
            self.time = None
        
        # Position
        if self.position_check_handle.model.get_value_as_bool():
            self.position = [0,0,0]
            for i in range(3):
                self.position[i] = self.position_handles[i].model.as_float
        else:
            self.position = None

        # Velocity
        if self.velocity_check_handle.model.get_value_as_bool():
            self.velocity = [0,0,0]
            for i in range(3):
                self.velocity[i] = self.velocity_handles[i].model.as_float
        else:
            self.velocity = None

        # Fly over
        self.fly_over = self.fly_over_check_handle.model.get_value_as_bool()
                
    def add_waypoint(self):
        # Remove waypoints from UI
        self.waypoint_list.clear()
        
        # Update variables from UI fields
        self.update_variables()

        # Create the new waypoint variables
        label = self.waypoind_id.model.get_value_as_string()
        time = self.time
        pos = self.position
        vel = self.velocity
        fly_over = self.fly_over

        # Add waypoint to the list
        self.flight_plan.set_waypoint(label=label, time=time, pos=pos, vel=vel)
        self.flight_plan.set_uniform_velocity()
        
        # Add waypoint to the UI
        with self.waypoint_list:                        
            for waypoint in self.flight_plan.waypoints:
                with ui.HStack():
                    # Label
                    ui.Label(waypoint.label, alignment=ui.Alignment.LEFT_CENTER, word_wrap=True)
                    
                    # Time
                    ui.Label(f"{waypoint.t:.2f}", alignment=ui.Alignment.CENTER)

                    # Position data
                    with ui.VStack():
                        ui.Label(f"{waypoint.pos[0]:.2f}", style={'color': colors["R"]}, alignment=ui.Alignment.LEFT)
                        ui.Label(f"{waypoint.pos[1]:.2f}", style={'color': colors["G"]}, alignment=ui.Alignment.LEFT)
                        ui.Label(f"{waypoint.pos[2]:.2f}", style={'color': colors["B"]}, alignment=ui.Alignment.LEFT)

                    # Velocity data
                    with ui.VStack():
                        ui.Label(f"{waypoint.vel[0]:.2f}", style={'color': colors["R"]}, alignment=ui.Alignment.LEFT)
                        ui.Label(f"{waypoint.vel[1]:.2f}", style={'color': colors["G"]}, alignment=ui.Alignment.LEFT)
                        ui.Label(f"{waypoint.vel[2]:.2f}", style={'color': colors["B"]}, alignment=ui.Alignment.LEFT)

                    # Fly over
                    if waypoint.fly_over:
                        ui.Label(str(waypoint.fly_over), style={'color' : colors["G"]}, 
                                alignment=ui.Alignment.CENTER)
                    else:
                        ui.Label(str(waypoint.fly_over), style={'color' : colors["R"]}, 
                                alignment=ui.Alignment.CENTER)
                ui.Separator()

    def build_window(self):
        # Create extension main window
        self.window = ui.Window("FP: NavSim - Flight Plan Generator", width=500, height=800)
        self.window.deferred_dock_in("Layers")
        self.window.setPosition(25, 25)
        self.window.frame.set_style(Window_dark_style)

        # Populate window frame
        with self.window.frame:
            self.main_window_scrolling_frame = ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                )
            with self.main_window_scrolling_frame:
                with ui.VStack(height=MINIMAL_HEIGHT, style=VStack_A, spacing=SPACING_S):
                    ui.Spacer(height=MINIMAL_HEIGHT)
                    # Drone selector widget
                    self.UAV_selector_dropdown = self.extension_utils.build_uav_selector()
                    # Create transform frame
                    self.build_waypoint_frame()
                    # Create waypoint list
                    self.build_waypoint_list()
                    # Send flight plan button
                    ui.Button("Send Flight Plan", height=50, clicked_fn=self.send_flight_plan)