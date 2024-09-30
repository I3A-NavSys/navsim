import sys, os

# Adding root 'ov' folder to sys.path
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

import omni.ext
import omni.ui as ui

import carb.events

from omni.ui import color as cl
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.ui.element_wrappers import *

from uspace.flightplan.FlightPlan import FlightPlan

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
    },
    "Slider::transform": {
        "background_color": 0xFF23211F,
        "border_radius": 3,
        "draw_mode": ui.SliderDrawMode.DRAG,
        "corner_flag": ui.CornerFlag.RIGHT,
        "font_size": 14,
    },
    "Label::transform_label": {"font_size": 12, "color": 0xFFDDDDDD},
    "Label": {"font_size": 12, "color": 0xFF8A8777},
    "Label::label": {"font_size": 14, "color": 0xFF8A8777},
    "Label::title": {"font_size": 14, "color": 0xFFAAAAAA},
    "Triangle::title": {"background_color": 0xFFAAAAAA},
    "ComboBox::path": {"font_size": 12, "secondary_color": 0xFF23211F, "color": 0xFFAAAAAA},
    "ComboBox::choices": {
        "font_size": 12,
        "color": 0xFFAAAAAA,
        "background_color": 0xFF23211F,
        "secondary_color": 0xFF23211F,
    },
    "ComboBox:hovered:choices": {"background_color": 0xFF33312F, "secondary_color": 0xFF33312F},
    "Slider::value_less": {
        "font_size": 12,
        "color": 0x0,
        "border_radius": 5,
        "background_color": 0xFF23211F,
        "secondary_color": KIT_GREEN,
        "border_color": 0xFFAAFFFF,
        "border_width": 0,
    },
    "Slider::value": {
        "font_size": 14,
        "color": 0xFFAAAAAA,
        "border_radius": 5,
        "background_color": 0xFF23211F,
        "secondary_color": KIT_GREEN,
    },
    "Rectangle::add": {"background_color": 0xFF23211F},
    "Rectangle:hovered:add": {"background_color": 0xFF73414F},
    "CheckBox::greenCheck": {"font_size": 10, "background_color": KIT_GREEN, "color": 0xFF23211F},
    "CheckBox::whiteCheck": {"font_size": 10, "background_color": 0xFFDDDDDD, "color": 0xFF23211F},
    "Slider::colorField": {"background_color": 0xFF23211F, "font_size": 14, "color": 0xFF8A8777},
    # Frame
    "CollapsableFrame::standard_collapsable": {
        "background_color": 0xFF343432,
        "secondary_color": 0xFF343432,
        "font_size": 16,
        "border_radius": 2.0,
        "border_color": 0x0,
        "border_width": 0,
    },
    "CollapsableFrame:hovered:standard_collapsable": {"secondary_color": 0xFFFBF1E5},
    "CollapsableFrame:pressed:standard_collapsable": {"secondary_color": 0xFFF7E4CC},
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

class NavsimOperatorCmdExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.

    def on_startup(self, ext_id):
        # List of manipulable prims
        self.manipulable_prims = []

        # Initialize flightplan
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
                components = ["Position", "Rotation", "Linear Velocity"]
                for component in components:
                    # Field labels
                    with ui.HStack(spacing=8):
                        with ui.HStack(width=LABEL_PADDING):
                            ui.Label(component, name="transform", width=50)
                            ui.Spacer()
                        # Axis fields
                        all_axis = ["X", "Y", "Z"]
                        colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
                        for axis in all_axis:
                            with ui.HStack():
                                with ui.ZStack(width=15):
                                    # Colored rectangle
                                    ui.Rectangle(
                                        width=15,
                                        height=20,
                                        style={
                                            "background_color": colors[axis],
                                            "border_radius": 3,
                                            "corner_flag": ui.CornerFlag.LEFT
                                        })
                                    # Axis letter label
                                    ui.Label(axis, name="transform_label", alignment=ui.Alignment.CENTER)
                                # FloatDrag widget
                                ui.FloatDrag(name="transform", min=-1000000, max=1000000, step=0.01)
                        # Null field checkbox
                        ui.CheckBox(width=0).model.set_value(True)
                ui.Spacer(height=0)
                
    def build_window(self):
        # Create extension main window
        self.window = ui.Window("Flightplan Generator", width=450, height=800)
        self.window.deferred_dock_in("Layers")
        self.window.setPosition(0, 0)
        self.window.frame.set_style(DARK_WINDOW_STYLE)

        # Populate window frame
        with self.window.frame:
            with ui.VStack(height=0, name="main_v_stack", spacing=6):
                ui.Spacer(height=0)
                # create build transform Frame
                self.build_waypoint_frame()