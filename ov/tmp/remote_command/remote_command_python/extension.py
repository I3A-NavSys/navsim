# Adding root 'project' folder to sys.path
import sys
import os
# This line will add to the python list of paths to look for modules the path to the project root
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))

if project_root_path not in sys.path:
    sys.path.append(project_root_path)

for path in sys.path:
    print(path)

import omni.ext
import omni.ui as ui
from omni.ui import color as cl

from omni.isaac.ui.ui_utils import dropdown_builder
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.ui.element_wrappers import *

import carb.events

from uspace.flightplan.command import Command
import pickle   # Serialization
import base64   # Parsing to string

import omni.kit.app

class NavsimOperatorCmdExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        # print("[REMOTE COMMAND ext] startup")

        # List of manipulable prims (usually drones)
        self.manipulable_prims = []

        # Get the bus event stream
        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

        self.app_interface = omni.kit.app.get_app_interface()

        # Create the window
        self._window = ui.Window("REMOTE COMMAND", width=400, height=200)
        with self._window.frame:

            with ui.VStack(spacing=10):
                ui.Spacer(height=10)

                def on_click():
                    # print("[REMOTE COMMAND ext] SEND button clicked")

                    # Get the selected drone
                    try:
                        drone = self.manipulable_prims[self.drone_selector_dropdown.get_selection_index()]
                    except:
                        print("[REMOTE COMMAND ext] No drone selected")
                        return
                    
                    # Create the event to send commands to the UAV
                    self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))
                           
                    # Set command data structure
                    command = Command(
                                    on = rotors_CB.checked, 
                                    velX = velX_FF.model.get_value_as_float(), 
                                    velY = velY_FF.model.get_value_as_float(), 
                                    velZ = velZ_FF.model.get_value_as_float(),
                                    rotZ = rotZ_FF.model.get_value_as_float(),
                                    duration = duration_FF.model.get_value_as_float())

                    serialized_command = base64.b64encode(pickle.dumps(command)).decode('utf-8')
                    self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_RemoteCommand", "command": serialized_command})


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
                        rotors_CB = ui.CheckBox(tooltip="Rotors activation")
                        rotors_CB.model.set_value(True)

                    with ui.HStack():
                        ui.Button(
                            "duration", enable=False,
                            width=50,
                            style={
                                "background_color":cl.grey, 
                                "color":cl.white, 
                                "margin": 0})
                        duration_FF = ui.FloatField(tooltip="time executing this command")
                        duration_FF.model.set_value(1.0)
                        duration_FF.precision = 2

                with ui.HStack(height=20, spacing=10):

                    with ui.HStack():
                        ui.Button(
                            "velX", enable=False,
                            width=30,
                            style={
                                "background_color":cl.red, 
                                "color":cl.white, 
                                "margin": 0})
                        velX_FF = ui.FloatField(tooltip="velX parameter")
                        velX_FF.precision = 2


                    with ui.HStack():
                        ui.Button(
                            "velY", enable=False,
                            width=30,
                            style={
                                "background_color":cl.green, 
                                "color":cl.white, 
                                "margin": 0})
                        velY_FF = ui.FloatField(tooltip="velY parameter")
                        velY_FF.precision = 2

                    with ui.HStack():
                        ui.Button(
                            "velZ", enable=False,
                            width=30,
                            style={
                                "background_color":cl.blue, 
                                "color":cl.white, 
                                "margin": 0})
                        velZ_FF = ui.FloatField(tooltip="velZ parameter")
                        velZ_FF.precision = 2

                    with ui.HStack():
                        ui.Button(
                            "rotZ", enable=False,
                            width=20,
                            style={
                                "background_color":cl.orange, 
                                "color":cl.white, 
                                "margin": 0})
                        rotZ_FF = ui.FloatField(tooltip="rotZ parameter")
                        rotZ_FF.precision = 2

                with ui.HStack(height=50):
                    #send button
                    ui.Button(
                        "SEND", 
                        clicked_fn = on_click)
                    
                with ui.HStack(height=50):
                    # fp button
                    ui.Button(
                        "FP",
                        clicked_fn = self.flightPlan
                    )




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
        # print("[REMOTE COMMAND ext] shutdown")
        self.stop_update_plot = True
        pass



    def flightPlan(self):
        from uspace.flightplan.Waypoint   import Waypoint
        from uspace.flightplan.FlightPlan import FlightPlan


        # Crear waypoints
        wp1L = Waypoint(label="wp1L", pos=[-190, -119, 48.1])
        wp1P = Waypoint(label="wp1P", pos=[-190, -119, 48.1])
        wp1 = Waypoint(label="wp1", pos=[-190, -119, 70])
        wp2 = Waypoint(label="wp2", pos=[-90, -19, 70])
        wp3 = Waypoint(label="wp3", pos=[-90, 181, 70])
        wp4 = Waypoint(label="wp4", pos=[110, 181, 70])
        wp5 = Waypoint(label="wp5", pos=[110, -119, 70])
        wp6L = Waypoint(label="wp6L", pos=[-152, -106, 49.1])
        wp6P = Waypoint(label="wp6P", pos=[-152, -106, 49.1])
        wp6 = Waypoint(label="wp6", pos=[-152, -106, 70])

        self.fp = FlightPlan()
        self.fp.radius = 2
        self.fp.AppendWaypoint(wp1L)
        wp1P.t = self.fp.FinishTime() + 5
        self.fp.SetWaypoint(wp1P)
        self.fp.AppendWaypoint(wp1)
        self.fp.AppendWaypoint(wp2)
        self.fp.AppendWaypoint(wp3)
        self.fp.AppendWaypoint(wp4)
        self.fp.AppendWaypoint(wp5)
        self.fp.AppendWaypoint(wp6)
        self.fp.AppendWaypoint(wp6L)
        wp6P.t = self.fp.FinishTime() + 5
        self.fp.SetWaypoint(wp6P)

        self.fp.SetTimeFromVel("wp1", 1)
        self.fp.SetTimeFromVel("wp2", 3)
        self.fp.SetTimeFromVel("wp3", 3)
        self.fp.SetTimeFromVel("wp4", 3)
        self.fp.SetTimeFromVel("wp5", 3)
        self.fp.SetTimeFromVel("wp6", 3)
        self.fp.SetTimeFromVel("wp6L", 1)

        self.fp.SetV0000()

        try:
            drone = self.manipulable_prims[self.drone_selector_dropdown.get_selection_index()]
        except:
            print("[REMOTE COMMAND ext] No drone selected")
            return

        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))

        serialized_fp = base64.b64encode(pickle.dumps(self.fp)).decode('utf-8')
        self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})