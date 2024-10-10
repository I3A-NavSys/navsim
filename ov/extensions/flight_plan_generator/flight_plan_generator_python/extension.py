import sys, os

# Adding root 'ov' folder to sys.path
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

import omni.ext
import omni.ui as ui

from omni.isaac.ui.element_wrappers import *

from uspace.flight_plan.flight_plan import FlightPlan

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.

KIT_GREEN = 0xFF8A8777
LABEL_PADDING = 120

Window_dark_style = {
    "Window": {"background_color": 0xFF444444}
}

VStack_A = {
    "VStack": {
        "secondary_color": 0x0, 
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

Label_list = {
    "Label": {
        "font_size": 12,
        "color": 0xFFDDDDDD,
        "margin_width": 10,
        "margin_height": 5
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
        # Field variables
        self.waypoint_list = []
        self.position = []
        self.velocity = []
        self.time = 0
        self.priority = 0

        # Field validity variables
        self.position_check = True
        self.velocity_check = True
        self.time_check = True
        self.fly_over_check = True
        self.priority_check = True

        # UI field handles
        self.position_handles : list[ui.FloatDrag] = []
        self.velocity_handles : list[ui.FloatDrag] = []
        self.time_handle = None
        self.priority_handle = None

        # UI widgets
        self.waypoint_list_frame = None
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
                    self.position_check = ui.CheckBox(width=0).model.set_value(True)

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
                    self.velocity_check = ui.CheckBox(width=0).model.set_value(True)

                # Add time field
                with ui.HStack(spacing=8):
                    ui.Label("Time", style=Label_A, width=LABEL_PADDING)
                    self.time_handle = ui.IntDrag(min=0, max=1000000, step=1)

                    # Enabled field checkbox
                    self.time_check = ui.CheckBox(width=0).model.set_value(True)

                with ui.HStack(spacing=8):
                    # Fly over label
                    ui.Label("Fly over", style=Label_A, width=LABEL_PADDING)

                    # Fly over field checkbox
                    self.fly_over_check = ui.CheckBox(width=0).model.set_value(True)

                with ui.HStack(spacing=8):
                    # Priority label
                    ui.Label("Priority", style=Label_A, width=LABEL_PADDING)

                    # Priority field checkbox
                    self.priority_handle = ui.IntDrag(min=0, max=1000000, step=1)

                    # Enabled field checkbox
                    self.priority_check = ui.CheckBox(width=0).model.set_value(True)

                # Add waypoint button
                ui.Button("Add waypoint", clicked_fn=self.add_waypoint)
                ui.Spacer()

    def build_waypoint_list(self):
        with ui.CollapsableFrame(title="Waypoint list", style=CollapsableFrame_style):
            self.waypoint_list_frame = ui.ScrollingFrame(
                height=250,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                style=ScrollingFrame_style
            )
            with self.waypoint_list_frame:
                with ui.VStack(spacing=8):
                    ui.Spacer(height=0)
                    self.empty_waypoint_list_label = ui.Label("Waypoint list is empty", style=Label_A, 
                                                              alignment=ui.Alignment.CENTER_TOP)
                
    def add_waypoint(self):
        with self.waypoint_list_frame:
            with ui.VStack(height = 0, style = VStack_B):
                with ui.HStack(spacing=0, width=20):
                    for _ in range(9):
                        ui.Label("Test", style=Label_list, alignment=ui.Alignment.LEFT_TOP)
    
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