# Standard library imports
import sys, os

# Related third party imports
import omni.ext
import omni.ui as ui
from omni.isaac.ui.element_wrappers import *


project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
# Add root 'ov' folder to sys.path
if project_root_path not in sys.path:
    sys.path.append(project_root_path)


# Local application/library specific imports
from uspace.flight_plan.flight_plan import FlightPlan
from uspace.flight_plan.waypoint import Waypoint


KIT_GREEN = 0xFF8A8777
LABEL_PADDING = 120


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
    "VStack": {"margin_width": 15}
}


Label_A = {
    "Label": {
        "font_size": 12,
        "color": 0xFFDDDDDD
    }
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
        # Waypoint variables
        self.waypoints = []
        self.n_waypoints = 0
        
        # Field variables
        self.position = [0, 0, 0]
        self.velocity = [0, 0, 0]
        self.time = 0
        self.fly_over = None

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
        self.waypoint_list_frame = None
        self.waypoint_list = None
        self.empty_waypoint_list_label = None

        # Initialize flight plan
        self.fp = FlightPlan()

        # Get bus event stream
        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

        # Build graphical interface
        self.build_window()
                    
    def on_shutdown(self):
        pass

    def build_waypoint_frame(self):
        with ui.CollapsableFrame(title="Waypoint", style=CollapsableFrame_style):
            with ui.VStack(spacing=8, style=VStack_B):
                ui.Spacer(height=0)

                # Position fields
                with ui.HStack(spacing=8):
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
                            self.position_handles.append(ui.FloatDrag(min=-1000000, max=1000000, step=0.01))

                    # Enabled field checkboxes
                    self.position_check_handle = ui.CheckBox(width=0)
                    self.position_check_handle.model.set_value(True)

                # Velocity fields
                with ui.HStack(spacing=8):
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
                            self.velocity_handles.append(ui.FloatDrag(min=-1000000, max=1000000, step=0.01))

                    # Enabled field checkboxes
                    self.velocity_check_handle = ui.CheckBox(width=0)
                    self.velocity_check_handle.model.set_value(True)

                # Add time field
                with ui.HStack(spacing=8):
                    ui.Label("Time", style=Label_A, width=LABEL_PADDING)
                    self.time_handle = ui.IntDrag(min=0, max=1000000, step=1)

                    # Enabled field checkbox
                    self.time_check_handle = ui.CheckBox(width=0)
                    self.time_check_handle.model.set_value(True)

                with ui.HStack(spacing=8):
                    # Fly over label
                    ui.Label("Fly over", style=Label_A, width=LABEL_PADDING)

                    # Fly over field checkbox
                    self.fly_over_check_handle = ui.CheckBox(width=0)
                    self.fly_over_check_handle.model.set_value(True)

                # Add waypoint button
                ui.Button("Add waypoint", clicked_fn=self.add_waypoint)
                ui.Spacer()

    def build_waypoint_list(self):
        with ui.CollapsableFrame(title="Waypoint list", style=CollapsableFrame_style):
            self.waypoint_list_frame = ui.ScrollingFrame(
                height=250,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                style=ScrollingFrame_style
            )
            with self.waypoint_list_frame:
                self.waypoint_list = ui.VStack(spacing=5)
                with self.waypoint_list:
                    ui.Spacer(height=0)
                    self.empty_waypoint_list_label = ui.Label("Waypoint list is empty", style=Label_A, 
                                                              alignment=ui.Alignment.CENTER_TOP)
                    
    def update_variables(self):
        # Position and velocity
        for i in range(3):
            self.position[i] = self.position_handles[i].model.as_float
            self.velocity[i] = self.velocity_handles[i].model.as_float

        # Time
        self.time = self.time_handle.model.as_float

        # Fly over
        if self.fly_over_check_handle.model.get_value_as_bool():
            self.fly_over = True
        else:
            self.fly_over = False
                
    def add_waypoint(self):
        # Update variables from UI fields
        self.update_variables()

        # Create the new waypoint
        new_waypoint = Waypoint(
            label="wp"+str(self.n_waypoints),
            t=self.time,
            pos=self.position,
            vel=self.velocity,
            fly_over=self.fly_over
        )

        # Add waypoint to the list
        self.waypoints.append(new_waypoint)
        self.n_waypoints = self.n_waypoints + 1
        
        # Modify empty list label visibility
        if len(self.waypoints) > 0:
            self.empty_waypoint_list_label.visible = False
        else:
            self.empty_waypoint_list_label.visible = True

        # Add waypoint to the UI
        colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
        with self.waypoint_list_frame:
            with self.waypoint_list:
                with ui.HStack(height=15, spacing=30):
                    # Waypoint label
                    ui.Label(new_waypoint.label, width=30, alignment=ui.Alignment.LEFT_TOP)

                    # Position data
                    with ui.HStack(width=0, spacing=5):
                        ui.Label("Position {", width=0, alignment=ui.Alignment.LEFT_TOP)
                        ui.Label("X:", width=0, style={'color': colors["X"]}, alignment=ui.Alignment.LEFT_TOP)
                        ui.Label(f"{new_waypoint.pos[0]:.2f}", alignment=ui.Alignment.LEFT_TOP)
                        ui.Label("Y:", width=0, style={'color': colors["Y"]}, alignment=ui.Alignment.LEFT_TOP)
                        ui.Label(f"{new_waypoint.pos[1]:.2f}", alignment=ui.Alignment.LEFT_TOP)
                        ui.Label("Z:", width=0, style={'color': colors["Z"]}, alignment=ui.Alignment.LEFT_TOP)
                        ui.Label(f"{new_waypoint.pos[2]:.2f}", alignment=ui.Alignment.LEFT_TOP)
                        ui.Label("}", width=0, alignment=ui.Alignment.LEFT_TOP)

                    # Velocity data
                    with ui.HStack(width=0, spacing=5):
                        ui.Label("Velocity {", width=0, alignment=ui.Alignment.LEFT_TOP)
                        ui.Label("X:", width=0, style={'color': colors["X"]}, alignment=ui.Alignment.LEFT_TOP)
                        ui.Label(f"{new_waypoint.vel[0]:.2f}", alignment=ui.Alignment.LEFT_TOP)
                        ui.Label("Y:", width=0, style={'color': colors["Y"]}, alignment=ui.Alignment.LEFT_TOP)
                        ui.Label(f"{new_waypoint.vel[1]:.2f}", alignment=ui.Alignment.LEFT_TOP)
                        ui.Label("Z:", width=0, style={'color': colors["Z"]}, alignment=ui.Alignment.LEFT_TOP)
                        ui.Label(f"{new_waypoint.vel[2]:.2f}", alignment=ui.Alignment.LEFT_TOP)
                        ui.Label("}", width=0, alignment=ui.Alignment.LEFT_TOP)

                    # Time
                    with ui.HStack(width=0, spacing=5):
                        ui.Label("Time:", width=0, alignment=ui.Alignment.LEFT_TOP)
                        ui.Label(f"{new_waypoint.t:.2f}", alignment=ui.Alignment.LEFT_TOP)

                    # Fly over
                    with ui.HStack(spacing=5):
                        ui.Label("Fly over:", width=0, alignment=ui.Alignment.LEFT_TOP)
                        ui.Label(str(new_waypoint.fly_over), alignment=ui.Alignment.LEFT_TOP)
                ui.Separator()

    def build_window(self):
        # Create extension main window
        self.window = ui.Window("NavSim - Flight Plan Generator", width=450, height=600)
        self.window.deferred_dock_in("Layers")
        self.window.setPosition(100, 100)
        self.window.frame.set_style(Window_dark_style)

        # Populate window frame
        with self.window.frame:
            with ui.VStack(height=0, style=VStack_A, spacing=6):
                ui.Spacer(height=0)
                # Create transform frame
                self.build_waypoint_frame()
                # Create waypoint list
                self.build_waypoint_list()