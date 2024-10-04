# Adding root 'project' folder to sys.path
import sys
import os
# This line will add to the python list of paths to look for modules the path to the project root
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

if project_root_path not in sys.path:
    sys.path.append(project_root_path)

import omni.ui as ui
from omni.ui import color as cl
from omni.isaac.ui.element_wrappers import DropDown
from omni.isaac.core.utils.stage import get_current_stage


# -- USER INTERFACE --

#  -> START -| Build NavSim:Manipulable UAV selector |-
UAV_selector_dropdown: DropDown = None

def build_uav_selector():
    global UAV_selector_dropdown

    with ui.HStack(spacing=5):
        # Dropdown selector
        UAV_selector_dropdown = DropDown("Select Drone", "Select the drone you want to control", 
                                            get_navsim_UAV_names)
        UAV_selector_dropdown.enabled = False

        # Button to refresh manipulable UAVs
        ui.Button("REFRESH", clicked_fn=refresh_drone_selector, width=100)

    return UAV_selector_dropdown



def get_navsim_UAV_names():
    manipulable_UAV_names = []
    stage = get_current_stage()
    
    for prim in stage.Traverse():
           att = prim.GetAttribute("NavSim:UAV")
           if att.IsValid() and att.Get():
                  manipulable_UAV_names.append(prim.GetName())

    return manipulable_UAV_names



def refresh_drone_selector():
        global UAV_selector_dropdown
        UAV_selector_dropdown.enabled = True
        UAV_selector_dropdown.repopulate()

#  -> END -| Build NavSim:Manipulable UAV selector |-





# -- GENERAL UTILS --

def get_prim_by_name(name):
    stage = get_current_stage()
    for prim in stage.Traverse():
        if prim.GetName() == name:
                return prim
        
    return None