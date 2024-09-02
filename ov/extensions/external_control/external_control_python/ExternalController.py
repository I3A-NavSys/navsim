from .JoystickInput import JoystickInput
from .KeyboardInput import KeyboardInput

import numpy as np
import asyncio

import carb.events
import omni.kit.app

from pxr import UsdGeom, Gf, Usd

class ExternalController:
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

        # Create the event have a communication between the UAV and the joystick
        self.CONTROL_JOYSTICK_EVENT = carb.events.type_from_string("omni.NavSim.ExternalControl.CONTROL_JOYSTICK_EVENT")
        # Get the bus event stream
        self.msg_bus_event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

    
    def start(self, prim, controller = None):
        if self._stop:
            self._stop = False

            # Get the controller
            self.controller = controller

            # Selected drone
            self.prim = prim

            # Needed variables
            self.inputs = [0,0,0,0]
            
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

            # Set inputs data structure
            inputs = {"x_vel": vel[0], "y_vel": vel[1], "z_vel": vel[2], "z_rot": rot}
            # Push CONTROL_JOYSTICK_EVENT with the inputs
            self.msg_bus_event_stream.push(self.CONTROL_JOYSTICK_EVENT, payload={"method": "set_flight_inputs", "inputs": inputs})

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