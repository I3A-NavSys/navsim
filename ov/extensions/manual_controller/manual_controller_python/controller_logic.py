from .joystick_input import JoystickInput
from .keyboard_input import KeyboardInput

import numpy as np
import asyncio

import carb.events
import omni.kit.app

from omni.isaac.core.utils.stage import get_current_stage
import omni.kit.viewport.utility

from uspace.flight_plan.command import Command
import pickle   # Serialization
import base64   # Parsing to string

class ControllerLogic:
    def __init__(self):
        # Loop condition
        self._stop = True

        # For manual gravity computation
        self.time_step = 1/100

        # Velocity limits
        self.linear_vel_limit = 0.5
        self.ang_vel_limit = 0.5

        # External inputs
        self.joystick = JoystickInput()
        self.keyboard = KeyboardInput()

        # Get the bus event stream
        self.msg_bus_event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

    
    def start(self, prim, controller = None):
        if self._stop:
            self._stop = False

            # Stage
            self.stage = get_current_stage()

            # Cameras
            self.cameras = self.get_cameras()
            self.active_camera = 0
            self.amount_cameras = len(self.cameras)

            # Create the event to have a communication between the UAV and the joystick
            self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(prim.GetPath()))

            # Get the controller
            self.controller = controller

            # Selected drone
            self.prim = prim

            # Needed variables
            self.inputs = [0,0,0,0,0,0,0,0]
            self.changed_on = False
            self.current_on = False
            
            # Check if a specific controller must be used
            if self.controller != None:
                # Check if it is joystick
                if self.controller == 0:
                    self.joystick.start()

                # Check if it is keyboard
                elif self.controller == 1:
                    self.keyboard.start()

            else:
                self.joystick.start()
                self.keyboard.start()

            # Start control coroutine
            asyncio.ensure_future(self.control())


    def stop(self):
        if not self._stop:
            self._stop = True

            # Check if a specific controller must be used
            if self.controller != None:
                # Check if it is joystick
                if self.controller == 0:
                    self.joystick.stop()

                # Check if it is keyboard
                elif self.controller == 1:
                    self.keyboard.stop()

            else:
                self.joystick.stop()
                self.keyboard.stop()


    # -- FUNCTION control --------------------------------------------------------------------------------
    # This method simply gets the inputs from both the joystick and the keyboard, then decide which one to use
    # Afterwards, it calls the corresponding methods to control the UAV
    # -----------------------------------------------------------------------------------------------------------
    async def control(self):
        while not self._stop:
            self.inputs = self.get_inputs()

            # Get velocities and rotation
            vel = self.inputs[:3] * -self.linear_vel_limit
            rot = self.inputs[3] * -self.ang_vel_limit

            # Get rottors on/off
            input_on = self.inputs[4]

            if input_on == 1 and not self.changed_on:
                self.changed_on = True
                self.current_on = not self.current_on

            elif input_on == 0:
                self.changed_on = False

            # Get camera switch index
            if self.inputs[5] != 0:     camera_slice = self.inputs[5]
            else:                       camera_slice = self.inputs[6]

            # Switch camera
            if self.amount_cameras > 0:
                self.active_camera = int(self.active_camera + camera_slice)%self.amount_cameras
                self.switch_active_camera(self.active_camera)

            # Update camera distance & height
            follow_distance = self.inputs[7]
            follow_height = self.inputs[8]
            self.update_camera_dist_height(follow_distance, follow_height)

            # Set command
            command = Command(
                            on = self.current_on,
                            velX = vel[0],
                            velY = vel[1],
                            velZ = vel[2],
                            rotZ = rot,
                            duration = None)

            serialized_command = base64.b64encode(pickle.dumps(command)).decode('utf-8')

            # Push UAV_EVENT with the inputs
            self.msg_bus_event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_RemoteCommand", "command": serialized_command})

            await asyncio.sleep(0.1)


    # -- FUNCTION get_inputs --------------------------------------------------------------------------------
    # This method is in charge of returning the valid inputs.
    # It must check which controllers are being used and get the corresponding inputs
    # -----------------------------------------------------------------------------------------------------------
    def get_inputs(self):
        # Chek if both controllers are running
        if self.controller == None:
            # Get the corresponding inputs
            self.joystick.ask_input()
            joy_inputs = np.array(self.joystick.inputs)
            key_inputs = np.array(self.keyboard.inputs)

            # Joystick input has priority
            if self.check_inputs(joy_inputs):
                return joy_inputs
            else:
                return key_inputs

        else:
            # Check if running controller is joystick
            if self.controller == 0:
                self.joystick.ask_input()
                return np.array(self.joystick.inputs)

            # Check if running controller is keyboard
            elif self.controller == 1:
                return np.array(self.keyboard.inputs)


    # -- FUNCTION get_grav_deacc --------------------------------------------------------------------------------
    # This method just checks if we have any input from the receiving parameter (joystick as it has priority)
    # -----------------------------------------------------------------------------------------------------------
    def check_inputs(self, inputs):
        for value in inputs:
            if value != 0:
                return True
            
        return False
    

    # -- FUNCTION get_cameras -----------------------------------------------------------------------------------
    # This function returns the cameras' prim path
    # The only searched cameras are the ones which have physics to follow the prim
    # -----------------------------------------------------------------------------------------------------------
    def get_cameras(self, prim=None):
        cameras = []

        # First iteration
        if prim is None:
            prim = self.stage.GetPseudoRoot()

        # Check current prim
        if (("Camera" in prim.GetName()) and (("Drone" in prim.GetName()) or ("Look" in prim.GetName()) or ("Velocity" in prim.GetName()))):
            cameras.append(prim.GetPath())
        
        else:
            # Check prim's children
            for child in prim.GetChildren():
                # First check children
                if len(child.GetChildren()) > 0:
                    cameras += self.get_cameras(child)
                
                # Then check current child
                elif (("Camera" in child.GetName()) and (("Drone" in child.GetName()) or ("Look" in child.GetName()) or ("Velocity" in child.GetName()))):
                    cameras.append(child.GetPath())

        return cameras

    # -- FUNCTION switch_active_camera --------------------------------------------------------------------------
    # This function change the active camera
    # -----------------------------------------------------------------------------------------------------------
    def switch_active_camera(self, camera_index):
        camera = self.cameras[camera_index]
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        viewport_api.set_active_camera(camera)

    
    # -- FUNCTION switch_active_camera --------------------------------------------------------------------------
    # This method changes the follow distance and height of the active based on the received parameters
    # Highlight that it only support cameras of type Drone (neither Look nor Velocity)
    # -----------------------------------------------------------------------------------------------------------
    def update_camera_dist_height(self, follow_distance, follow_height):
        active_camera_path = self.cameras[self.active_camera]
        active_camera_prim = self.stage.GetPrimAtPath(active_camera_path)

        if "Drone" in active_camera_prim.GetName():
            distance_att = active_camera_prim.GetAttribute("physxDroneCamera:followDistance")
            height_att = active_camera_prim.GetAttribute("physxDroneCamera:followHeight")

            distance_att.Set(distance_att.Get() + follow_distance)
            height_att.Set(height_att.Get() + follow_height)
