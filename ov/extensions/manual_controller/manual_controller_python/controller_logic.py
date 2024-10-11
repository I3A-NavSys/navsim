import pickle   # Serialization
import base64   # Parsing to string

import numpy as np
import asyncio
import carb.events
import omni.kit.app
from omni.isaac.core.utils.stage import get_current_stage
import omni.kit.viewport.utility
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema

from uspace.flight_plan.command import Command
from .joystick_input import JoystickInput
from .keyboard_input import KeyboardInput

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

    def start(self, prim):
        if self._stop:
            self._stop = False

            # Stage
            self.stage = get_current_stage()

            # Selected drone
            self.prim = prim

            # Build follow velocity camera
            self.camera = self.build_camera()
            self.camera_setted = False

            # Create the event to have a communication between the UAV and the joystick
            self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(self.prim.GetPath()))

            # Needed variables
            self.inputs = [0,0,0,0,0,0,0,0]
            self.changed_on = False
            self.current_on = False
            
            self.joystick.start()
            self.keyboard.start()

            # Start control coroutine
            asyncio.ensure_future(self.control())

    def stop(self):
        if not self._stop:
            self._stop = True

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

            # Set velocity camera to active if first time
            if not self.camera_setted:
                self.camera_setted = True
                self.set_camera_to_active()

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
        # Get the corresponding inputs
        self.joystick.ask_input()
        joy_inputs = np.array(self.joystick.inputs)
        key_inputs = np.array(self.keyboard.inputs)

        # Joystick input has priority
        if self.check_inputs(joy_inputs):
            return joy_inputs
        else:
            return key_inputs

    # -- FUNCTION check_inputs --------------------------------------------------------------------------------
    # This method just checks if we have any input from the receiving parameter (joystick as it has priority)
    # -----------------------------------------------------------------------------------------------------------
    def check_inputs(self, inputs):
        for value in inputs:
            if value != 0:
                return True
            
        return False

    def build_camera(self):
        # Prim subject (target) path
        subject_path = self.prim.GetPath()

        # Build the camera prim
        camera_path = "/FollowVelocityCamera"
        usdCamera = UsdGeom.Camera.Define(self.stage, camera_path)
        follow_camera_prim = self.stage.GetPrimAtPath(camera_path)

        # Apply the physics API to the camera
        PhysxSchema.PhysxCameraFollowVelocityAPI.Apply(follow_camera_prim)

        # Set the subject target
        subject_rel = follow_camera_prim.GetRelationship("physxCamera:subject")
        subject_rel.SetTargets([subject_path])
        
        # Set the parameters
        position_offset = Gf.Vec3f(0, 0, 0)
        camera_position_tc = Gf.Vec3f(0.1, 0.1, 0.1)
        look_position_tc = Gf.Vec3f(0.2, 0.2, 0.2)

        follow_camera_prim.GetAttribute("clippingRange").Set((0.5, 1000000.0))
        follow_camera_prim.GetAttribute("alwaysUpdateEnabled").Set(True)
        follow_camera_prim.GetAttribute("physxFollowCamera:cameraPositionTimeConstant").Set(camera_position_tc)
        follow_camera_prim.GetAttribute("physxFollowCamera:followMaxDistance").Set(10.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:followMaxSpeed").Set(30.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:followMinDistance").Set(7.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:followMinSpeed").Set(3.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:followTurnRateGain").Set(0.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:lookAheadMaxSpeed").Set(20.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:lookAheadMinDistance").Set(0.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:lookAheadMinSpeed").Set(0.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:lookAheadTurnRateGain").Set(0.2)
        follow_camera_prim.GetAttribute("physxFollowCamera:lookPositionHeight").Set(0.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:lookAheadMinDistance").Set(0.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:lookPositionTimeConstant").Set(look_position_tc)
        follow_camera_prim.GetAttribute("physxFollowCamera:pitchAngle").Set(15.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:pitchAngleTimeConstant").Set(0.2)
        follow_camera_prim.GetAttribute("physxFollowCamera:positionOffset").Set(position_offset)
        follow_camera_prim.GetAttribute("physxFollowCamera:slowPitchAngleSpeed").Set(1000.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:slowSpeedPitchAngleScale").Set(0.5)
        follow_camera_prim.GetAttribute("physxFollowCamera:velocityNormalMinSpeed").Set(600.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:yawAngle").Set(0.0)
        follow_camera_prim.GetAttribute("physxFollowCamera:yawRateTimeConstant").Set(0.0)
        follow_camera_prim.GetAttribute("physxFollowFollowCamera:lookAheadMaxDistance").Set(0.0)

        return follow_camera_prim

    # -- FUNCTION switch_active_camera --------------------------------------------------------------------------
    # This function change the active camera
    # -----------------------------------------------------------------------------------------------------------
    def set_camera_to_active(self):
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        viewport_api.set_active_camera(self.camera)

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
