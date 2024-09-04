import omni.ext
import omni.ui as ui

from omni.isaac.ui.ui_utils import dropdown_builder
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.ui.element_wrappers import *

import carb.events



# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class NavsimOperatorCmdExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[navsim.operator.cmd] startup")

        self.velX = 0


        # List of manipulable prims (usually drones)
        self.manipulable_prims = []

        # Get the bus event stream
        self.msg_bus_event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()




        self._window = ui.Window("Operator CMD", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                label = ui.Label("")


                ui.Spacer(height=10)

                # Drone selector dropdown
                with ui.HStack(spacing=5):
                    # Dropdown selector
                    self.drone_selector_dropdown = DropDown("Select Drone", "Select the drone you want to control", self.get_manipulable_prims)
                    self.drone_selector_dropdown.enabled = False

                    # Button to refresh manipulable drones
                    self.refresh_selector_button = ui.Button("REFRESH", clicked_fn=self.refresh_drone_selector, width=100)
                    pass


                ui.Spacer(height=10)





                LvelX = ui.FloatField(tooltip="velX parameter")




                def on_click():


                    # Get the selected drone
                    drone = self.manipulable_prims[self.drone_selector_dropdown.get_selection_index()]

                    # Create the event to have a communication between the UAV and the joystick
                    self.CONTROL_JOYSTICK_EVENT = carb.events.type_from_string("omni.NavSim.ExternalControl." + str(drone.GetPath()))
                           
                    # Set inputs data structure
                    inputs = {"x_vel": 2, "y_vel": 0, "z_vel": 0.1, "z_rot": 1}
                    # Push CONTROL_JOYSTICK_EVENT with the inputs
                    self.msg_bus_event_stream.push(self.CONTROL_JOYSTICK_EVENT, payload={"method": "set_flight_inputs", "inputs": inputs})





                    self.velX = 1
                    label.text = f"velX: {self.velX}"
                    print (f"velX: {self.velX}")

                with ui.HStack():
                    ui.Button("Send CMD", clicked_fn = on_click)



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





















    def on_shutdown(self):
        print("[navsim.operator.cmd] shutdown")
