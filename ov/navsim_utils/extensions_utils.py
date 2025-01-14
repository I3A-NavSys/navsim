# Standard library imports
import sys
import os

# Related third party imports
import omni.ui as ui
from omni.isaac.ui.element_wrappers import DropDown
from omni.isaac.core.utils.stage import get_current_stage


project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add to the python list of paths to look for modules the path to the project root
if project_root_path not in sys.path:
    sys.path.append(project_root_path)


class ExtensionUtils:

    def __init__(self):
         # Color
        self.KIT_GREEN = 0xFF8A8777

        # Label
        self.LABEL_PADDING = 120

        # Spacing
        self.SPACING_S = 8
        self.SPACING_M = self.SPACING_S * 2
        self.SPACING_L = self.SPACING_M * 2
        self.SPACING_XL = self.SPACING_L * 2

        # Height
        self.MINIMAL_HEIGHT = 0

        # Width
        self.MINIMAL_WIDTH = 0

        self.Window_dark_style = {
            "Window": {"background_color": 0xFF444444}
        }


        self.VStack_A = {
            "VStack": {
                "margin_width": 10, 
                "margin_height": 0
            }
        }


        self.VStack_B = {
            "VStack": {
                "margin_width": 10,
                "margin_height": 5
            }
        }


        self.HStack_A = {
            "HStack": {
                "margin_width": 10,
                "margin_height": 5
            }
        }


        self.Label_A = {
            "Label": {
                "font_size": 12,
                "color": 0xFFDDDDDD
            }
        }


        self.colors = {
            "R": 0xFF5555AA,
            "G": 0xFF76A371,
            "B": 0xFFA07D4F
        }


        self.CollapsableFrame_style = {
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

        self.ScrollingFrame_style = {
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

    #------------------------------------------------------------------------------------------------------------------
    # USER INTERFACE

    def build_uav_selector(self):
        with ui.HStack(spacing=5):
            # Dropdown selector
            self.UAV_selector_dropdown = DropDown("Select Drone", "Select the drone you want to control", 
                                            self.get_navsim_UAV_names)
            self.UAV_selector_dropdown.enabled = False

            # Button to refresh manipulable UAVs
            ui.Button("REFRESH", clicked_fn=self.refresh_drone_selector, width=100)

        return self.UAV_selector_dropdown

    def get_navsim_UAV_names(self):
        manipulable_UAV_names = []
        stage = get_current_stage()
        
        if stage is not None:
            for prim in stage.Traverse():
                att = prim.GetAttribute("NavSim:type")
                if att.IsValid() and att.Get() == "UAV":
                        manipulable_UAV_names.append(prim.GetName())

        return manipulable_UAV_names

    def refresh_drone_selector(self):
            self.UAV_selector_dropdown.enabled = True
            self.UAV_selector_dropdown.repopulate()

    #------------------------------------------------------------------------------------------------------------------
    # MISCELLANEOUS UTILS

    def get_prim_by_name(self, name):
        stage = get_current_stage()
        for prim in stage.Traverse():
            if prim.GetName() == name:
                    return prim
            
        return None