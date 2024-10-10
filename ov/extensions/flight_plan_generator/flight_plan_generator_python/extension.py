import sys, os

# Adding root 'ov' folder to sys.path
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

import omni.ext
import omni.ui as ui

import carb.events

from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.ui.element_wrappers import *

from uspace.flight_plan.flight_plan import FlightPlan

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.

KIT_GREEN = 0xFF8A8777
LABEL_PADDING = 120

DARK_WINDOW_STYLE = {
    "Window": {"background_color": 0xFF444444},
    "Button": {"background_color": 0xFF292929, "margin": 3, "padding": 3, "border_radius": 2},
    "Button.Label": {"color": 0xFFCCCCCC},
    "Button:hovered": {"background_color": 0xFF9E9E9E},
    "Button:pressed": {"background_color": 0xC22A8778},
    "VStack::main_v_stack": {"secondary_color": 0x0, "margin_width": 10, "margin_height": 0},
    "VStack::frame_v_stack": {"margin_width": 15},
    "Rectangle::frame_background": {"background_color": 0xFF343432, "border_radius": 5},
    "Field::models": {"background_color": 0xFF23211F, "font_size": 14, "color": 0xFFAAAAAA, "border_radius": 4.0},
    "Frame": {"background_color": 0xFFAAAAAA},
    "Label::transform": {"font_size": 12, "color": 0xFF8A8777},
    "Circle::transform": {"background_color": 0x558A8777},
    "Field::transform": {
        "background_color": 0xFF23211F,
        "border_radius": 3,
        "corner_flag": ui.CornerFlag.RIGHT,
        "font_size": 12,
    }
}

Label_style = {
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

class NavsimOperatorCmdExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.

    def on_startup(self, ext_id):
        # List of manipulable prims
        self.manipulable_prims = []

        # Initialize flight plan
        self.fp = FlightPlan()

        # Get bus event stream
        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

        # Build graphical interface
        self.build_window()
                    
    def get_manipulable_prims(self, prim=None):
            # Needed list containing the names of the manipulable prims for the dropdown selector
            manipulable_prims = []

            # First iteration
            if prim is None:
                stage = get_current_stage()
                prim = stage.GetPseudoRoot()

                self.manipulable_prims.clear()

            # Check current prim
            if prim.GetAttribute("NavSim:Manipulable").IsValid() and prim.GetAttribute("NavSim:Manipulable").Get():
                manipulable_prims.append(prim.GetName())
                self.manipulable_prims.append(prim)
            
            else:
                # Check prim's children
                for child in prim.GetAllChildren():
                    # First check children
                    if len(child.GetAllChildren()) > 0:
                        manipulable_prims += self.get_manipulable_prims(child)
                    
                    # Then check current child
                    elif child.GetAttribute("NavSim:Manipulable").IsValid() and child.GetAttribute("NavSim:Manipulable").Get():
                        manipulable_prims.append(child.GetName())
                        self.manipulable_prims.append(prim)

            return manipulable_prims

    def refresh_drone_selector(self):
        self.drone_selector_dropdown.enabled = True
        self.drone_selector_dropdown.repopulate()

    def on_click(self):
        # Get the selected drone
        try:
            drone = self.manipulable_prims[self.drone_selector_dropdown.get_selection_index()]
        except:
            print("[REMOTE COMMAND ext] No drone selected")
            return
        
        print(f"[REMOTE COMMAND ext] Command sent at simulation time: {self.current_time:.2f}")

        # Create the event to send commands to the UAV
        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))

    def on_shutdown(self):
        pass

    def build_waypoint_frame(self):
        with ui.CollapsableFrame(title="Waypoint", style=CollapsableFrame_style):
            with ui.VStack(spacing=8, name="frame_v_stack"):
                ui.Spacer(height=0)

                # Pose and linear velocity fields
                components = ["Position", "Velocity"]

                # Handles to each component FloatDrag
                self.component_handles : list[ui.FloatDrag] = []
                for component in components:

                    # Field labels
                    with ui.HStack(spacing=8):
                        with ui.HStack(width=LABEL_PADDING):
                            ui.Label(component, style=Label_style, width=50)
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
                                    ui.Label(axis, style=Label_style, alignment=ui.Alignment.CENTER)

                                # FloatDrag widgets
                                self.component_handles.append(ui.FloatDrag(name="transform", min=-1000000, max=1000000, 
                                                                           step=0.01))

                        # Enabled field checkboxes
                        ui.CheckBox(width=0).model.set_value(True)

                # Add time field
                with ui.HStack(spacing=8):
                    ui.Label("Time", style=Label_style, width=LABEL_PADDING)
                    ui.IntDrag(min=0, max=1000000, step=1)

                    # Enabled field checkbox
                    ui.CheckBox(width=0).model.set_value(True)

                with ui.HStack(spacing=8):
                    # Fly over label
                    ui.Label("Fly over", style=Label_style, width=LABEL_PADDING)

                    # Fly over field checkbox
                    ui.CheckBox(width=0).model.set_value(True)

                with ui.HStack(spacing=8):
                    # Priority label
                    ui.Label("Priority", style=Label_style, width=LABEL_PADDING)

                    # Priority field checkbox
                    ui.IntDrag(min=0, max=1000000, step=1)

                    # Enabled field checkbox
                    ui.CheckBox(width=0).model.set_value(True)

                # Add waypoint button
                ui.Button("Add waypoint", clicked_fn=self.add_waypoint)
                ui.Spacer()

    def build_waypoint_list(self):
        with ui.CollapsableFrame(title="Waypoint list", style=CollapsableFrame_style):
            self.waypoint_list = ui.ScrollingFrame(
                height=250,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                style=ScrollingFrame_style
            )
            with self.waypoint_list:
                with ui.VStack(spacing=8):
                    ui.Spacer(height=0)
                    self.empty_waypoint_list = ui.Label("Waypoint list is empty", style=Label_style, 
                                                        alignment=ui.Alignment.CENTER_TOP)
                
    def add_waypoint(self):
        new_waypoint = []
        for handle in self.component_handles:
            new_waypoint.append(handle.model.floatValue())
    
    def build_window(self):
        # Create extension main window
        self.window = ui.Window("NavSim - Flight Plan Generator", width=450, height=800)
        self.window.deferred_dock_in("Layers")
        self.window.setPosition(0, 0)
        self.window.frame.set_style(DARK_WINDOW_STYLE)

        # Populate window frame
        with self.window.frame:
            with ui.VStack(height=0, name="main_v_stack", spacing=6):
                ui.Spacer(height=0)
                # Create transform frame
                self.build_waypoint_frame()
                # Create waypoint list
                self.build_waypoint_list()