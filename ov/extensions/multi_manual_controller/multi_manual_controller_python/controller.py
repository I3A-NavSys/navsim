import pickle   # Serialization
import base64   # Parsing to string

import numpy as np
import asyncio
import carb.events
import omni.kit.app
import omni.kit.viewport.utility
from pxr import UsdGeom, Gf, PhysxSchema
import omni.physx

from uspace.flight_plan.command import Command
from navsim_utils.extensions_utils import ExtensionUtils
from .joysticks import Joysticks

class Controller:
    def __init__(self):
        self.is_running = False
        self.linear_vel_limit = 3.5
        self.ang_vel_limit = 1
        self.invert_camera_movement = False
        self.camera_path = "/manual_controller_CAM"
        self.perspective_camera_path = "/OmniverseKit_Persp"
        self.joysticks = Joysticks()
        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()
        self.uav_events = []
        self.ext_utils = ExtensionUtils()

    def start(self):
        if not self.is_running:
            self.is_running = True
            self.uavs = self.get_uavs()
            self.uavs_rottors_on_change = [False for i in range(len(self.uavs))]
            self.uavs_current_rottors_on = [False for i in range(len(self.uavs))]

            for uav in self.uavs:
                event = carb.events.type_from_string("NavSim." + str(uav.GetPath()))
                self.uav_events.append(event)
            
            self.joysticks.start()

            # Attach control updates to physics steps
            self.physx_interface = omni.physx.get_physx_interface()
            self.physics_sub = self.physx_interface.subscribe_physics_step_events(self.control)

    def stop(self):
        if self.is_running:
            self.is_running = False
            self.physics_sub = None
            self.joysticks.stop()

    def control(self, event):
        joysticks_ids, joysticks_inputs = self.joysticks.get_inputs()

        for i in range(len(joysticks_ids)):
            # Get required info
            joystick_id = joysticks_ids[i]
            joystick_input = joysticks_inputs[joystick_id]
            uav_event = self.uav_events[i]
            rottors_on_change = self.uavs_rottors_on_change[i]
            current_rottors_on = self.uavs_current_rottors_on[i]

            # Get velocities and rotation
            vel = joystick_input[:3] * -self.linear_vel_limit
            rot = joystick_input[3] * -self.ang_vel_limit

            # Get rottors on/off
            rottors_on = joystick_input[4]

            # Evaluate whether rottors should be on off according to same button
            if rottors_on == 1 and not rottors_on_change:
                self.uavs_rottors_on_change[i] = True
                current_rottors_on = not current_rottors_on
                self.uavs_current_rottors_on[i] = current_rottors_on

            elif rottors_on == 0:
                self.uavs_rottors_on_change[i] = False

            # Set command
            command = Command(
                            on = current_rottors_on,
                            velX = vel[0],
                            velY = vel[1],
                            velZ = vel[2],
                            rotZ = rot,
                            duration = None)

            serialized_command = base64.b64encode(pickle.dumps(command)).decode('utf-8')

            # Push uav_event with the inputs
            self.event_stream.push(uav_event, payload={"method": "eventFn_RemoteCommand", 
                                                                    "command": serialized_command})
        
    def get_uavs(self):
        uavs = []
        uav_names = self.ext_utils.get_navsim_UAV_names()
        for name in uav_names:
            uavs.append(self.ext_utils.get_prim_by_name(name))

        return uavs