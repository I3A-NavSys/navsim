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

from uspace.flight_plan.command import Command
import pickle   # Serialization
import base64   # Parsing to string

from navsim_utils.extensions_utils import ExtensionUtils

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.

class CommandGenerator(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.

    def on_startup(self, ext_id):
        # Buidl ExtUtils instance
        self.ext_utils = ExtensionUtils()

        # Getting the simulation current time
        self.current_time = 0
        self.physx_interface = omni.physx.get_physx_interface()
        self.physics_timer_callback = self.physx_interface.subscribe_physics_step_events(self.physics_timer_callback_fn)

        self.timeline = omni.timeline.get_timeline_interface()
        self.event_timer_callback = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.timeline_timer_callback_fn)

        # Get the bus event stream
        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

        # Create the window
        self._window = ui.Window("NavSim - Command Generator", width=400, height=200)
        with self._window.frame:

            with ui.VStack(spacing=10, height=0):
                ui.Spacer(height=10)

                # UAV selector dropdown                    
                self.UAV_selector_dropdown = self.ext_utils.build_uav_selector()

                with ui.HStack(spacing=10):


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

                with ui.HStack(spacing=10):

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

                with ui.HStack():
                    #send button
                    ui.Button(
                        "SEND", 
                        height=50,
                        clicked_fn = self.on_click)



    def physics_timer_callback_fn(self, step_size:int):
        self.current_time += step_size



    def timeline_timer_callback_fn(self, event):
        self.current_time = 0



    def on_click(self):   
        if self.UAV_selector_dropdown.get_selection() is None:
            raise Exception("[REMOTE COMMAND ext] No drone selected")
        
        drone = self.ext_utils.get_prim_by_name(self.UAV_selector_dropdown.get_selection())
        
        print(f"[REMOTE COMMAND ext] Command sent at simulation time: {self.current_time:.2f}")

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