import omni.ext
import omni.ui as ui
from omni.ui import color as cl

from omni.isaac.ui.ui_utils import dropdown_builder
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.ui.element_wrappers import *

import carb.events

# Adding root 'ov' folder to sys.path
import sys, os
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

from uspace.flightplan.command import Command
import pickle   # Serialization
import base64   # Parsing to string

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.

class NavsimOperatorCmdExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.




    def on_startup(self, ext_id):
        # print("[REMOTE COMMAND ext] startup")

        import omni.timeline
        self.timeline = omni.timeline.get_timeline_interface()


        # List of manipulable prims (usually drones)
        self.manipulable_prims = []

        # Get the bus event stream
        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

        # Create the window
        self._window = ui.Window("REMOTE COMMAND", width=400, height=200)
        with self._window.frame:

            with ui.VStack(spacing=10):
                ui.Spacer(height=10)

                



                # UAV selector dropdown
                with ui.HStack(height=25):
                    # Dropdown selector
                    self.drone_selector_dropdown = DropDown(
                        "UAV", "Select the drone you want to send commands", 
                        self.get_manipulable_prims)
                    self.drone_selector_dropdown.enabled = False

                    # Button to refresh manipulable drones
                    self.refresh_selector_button = ui.Button(
                        "REFRESH", 
                        clicked_fn=self.refresh_drone_selector, 
                        height=15,
                        width=80)

                with ui.HStack(height=20, spacing=10):


                    with ui.HStack(spacing=5):
                        ui.Button(
                            "ON", enable=False,
                            width=30,
                            style={
                                "background_color":cl.grey, 
                                "color":cl.white, 
                                "margin": 0})
                        self.rotors_CB = ui.CheckBox(tooltip="Rotors activation")
                        self.rotors_CB.model.set_value(True)

                    with ui.HStack():
                        ui.Button(
                            "duration", enable=False,
                            width=50,
                            style={
                                "background_color":cl.grey, 
                                "color":cl.white, 
                                "margin": 0})
                        self.duration_FF = ui.FloatField(tooltip="time executing this command")
                        self.duration_FF.model.set_value(1.0)
                        self.duration_FF.precision = 2

                with ui.HStack(height=20, spacing=10):

                    with ui.HStack():
                        ui.Button(
                            "velX", enable=False,
                            width=30,
                            style={
                                "background_color":cl.red, 
                                "color":cl.white, 
                                "margin": 0})
                        self.velX_FF = ui.FloatField(tooltip="velX parameter")
                        self.velX_FF.precision = 2


                    with ui.HStack():
                        ui.Button(
                            "velY", enable=False,
                            width=30,
                            style={
                                "background_color":cl.green, 
                                "color":cl.white, 
                                "margin": 0})
                        self.velY_FF = ui.FloatField(tooltip="velY parameter")
                        self.velY_FF.precision = 2

                    with ui.HStack():
                        ui.Button(
                            "velZ", enable=False,
                            width=30,
                            style={
                                "background_color":cl.blue, 
                                "color":cl.white, 
                                "margin": 0})
                        self.velZ_FF = ui.FloatField(tooltip="velZ parameter")
                        self.velZ_FF.precision = 2

                    with ui.HStack():
                        ui.Button(
                            "rotZ", enable=False,
                            width=20,
                            style={
                                "background_color":cl.orange, 
                                "color":cl.white, 
                                "margin": 0})
                        self.rotZ_FF = ui.FloatField(tooltip="rotZ parameter")
                        self.rotZ_FF.precision = 2

                with ui.HStack(height=50):
                    #send button
                    ui.Button(
                        "SEND", 
                        clicked_fn = self.on_click)




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
        # print("[REMOTE COMMAND ext] SEND button clicked")

        # Get the selected drone
        try:
            drone = self.manipulable_prims[self.drone_selector_dropdown.get_selection_index()]
        except:
            print("[REMOTE COMMAND ext] No drone selected")
            return
        
        # Check if the simulation is running
        if not self.timeline.is_playing():
            print(f"[REMOTE COMMAND ext] Simulation is not running!")
            return
        simulation_time = self.timeline.get_current_time()
        print(f"[REMOTE COMMAND ext] Command sent at simulation time: {simulation_time:.2f}")

        # Create the event to send commands to the UAV
        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))
                
        # Set command data structure
        command = Command(
                        on = self.rotors_CB.checked, 
                        velX = self.velX_FF.model.get_value_as_float(), 
                        velY = self.velY_FF.model.get_value_as_float(), 
                        velZ = self.velZ_FF.model.get_value_as_float(),
                        rotZ = self.rotZ_FF.model.get_value_as_float(),
                        duration = self.duration_FF.model.get_value_as_float())

        serialized_command = base64.b64encode(pickle.dumps(command)).decode('utf-8')
        self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_RemoteCommand", "command": serialized_command})




    def on_shutdown(self):
        # print("[REMOTE COMMAND ext] shutdown")
        pass
