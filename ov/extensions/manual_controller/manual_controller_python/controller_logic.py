import pickle   # Serialization
import base64   # Parsing to string

import numpy as np
import asyncio
import carb.events
import omni.kit.app
from omni.isaac.core.utils.stage import get_current_stage
import omni.kit.viewport.utility
from pxr import UsdGeom, Gf, PhysxSchema

from uspace.flight_plan.command import Command
from .joystick_input import JoystickInput
from .keyboard_input import KeyboardInput

class ControllerLogic:
    def __init__(self):
        # Loop condition
        self._stop = True

        # Needed variables
        self.linear_vel_limit = 0.5
        self.ang_vel_limit = 0.5
        self.current_on = False
        self.invert_camera_movement = False
        self.camera_path = "/manual_controller_CAM"
        self.perspective_camera_path = "/OmniverseKit_Persp"

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
            self.camera = self.stage.GetPrimAtPath(self.camera_path)
            if not self.camera.IsValid():
                self.camera = self.build_camera()
                self.camera_setted = False
                self.camera_distance_attr = self.camera.GetAttribute("physxFollowCamera:followMinDistance")
                self.camera_yaw_attr = self.camera.GetAttribute("physxFollowCamera:yawAngle")
                self.camera_pitch_attr = self.camera.GetAttribute("physxFollowCamera:pitchAngle")

            # Create the event to have a communication between the UAV and the joystick
            self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(self.prim.GetPath()))

            # Needed variables
            self.inputs = [0,0,0,0,0,0,0,0]
            self.changed_on = False
            
            self.joystick.start()
            self.keyboard.start()

            # Start control coroutine
            asyncio.ensure_future(self.control())

    def stop(self):
        if not self._stop:
            self._stop = True

            self.joystick.stop()
            self.keyboard.stop()

    # -- FUNCTION control ---------------------------------------------------------------------------------------
    # This function simply gets the inputs from both the joystick and the keyboard, then decide which one to use
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
                self.set_camera_to_active(self.camera_path)

            # Update camera distance & height
            derease_distance = self.inputs[5]
            increase_distance = self.inputs[6]
            yaw = self.inputs[7]
            pitch = self.inputs[8]
            self.move_camera(derease_distance, increase_distance, yaw, pitch)

            # print(self.current_on)

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
            self.msg_bus_event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_RemoteCommand", 
                                                                    "command": serialized_command})

            await asyncio.sleep(0.1)

    # -- FUNCTION get_inputs ------------------------------------------------------------------------------------
    # This function is in charge of returning the valid inputs.
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

    # -- FUNCTION check_inputs ----------------------------------------------------------------------------------
    # This function just checks if we have any input from the receiving parameter (joystick as it has priority)
    # -----------------------------------------------------------------------------------------------------------
    def check_inputs(self, inputs):
        for value in inputs:
            if value != 0:
                return True
            
        return False

    # -- FUNCTION build_camera ----------------------------------------------------------------------------------
    # This function builds or overwrite a FollowVelocityCamera, which will be used to follow the UAV
    # -----------------------------------------------------------------------------------------------------------
    def build_camera(self):
        # Prim subject (target) path
        subject_path = self.prim.GetPath()

        # Build the camera prim
        usdCamera = UsdGeom.Camera.Define(self.stage, self.camera_path)
        follow_camera_prim = self.stage.GetPrimAtPath(self.camera_path)

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

    # -- FUNCTION destroy_camera --------------------------------------------------------------------------------
    # This function just sets the active camera as the perspective one  and deletes our camera from the stage
    # -----------------------------------------------------------------------------------------------------------
    def destroy_camera(self, stage):
        self.set_camera_to_active(self.perspective_camera_path)
        stage.RemovePrim(self.camera_path)
        self.camera = None

    # -- FUNCTION set_camera_to_active --------------------------------------------------------------------------
    # This function set the built camera as active each time the controller starts running
    # -----------------------------------------------------------------------------------------------------------
    def set_camera_to_active(self, camera_path):
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        viewport_api.set_active_camera(camera_path)

    # -- FUNCTION move_camera -----------------------------------------------------------------------------------
    # This function moves the camera around the UAV according to the received parameters
    # Highlight that it only support cameras of type followVelocity (neither Look nor Drone)
    # -----------------------------------------------------------------------------------------------------------
    def move_camera(self, derease_distance, increase_distance, yaw, pitch):
        if self.invert_camera_movement:
            yaw *= -1
            pitch *= -1

        distance_increment = derease_distance + increase_distance

        current_distance = self.camera_distance_attr.Get()
        current_yaw = self.camera_yaw_attr.Get()
        current_pitch = self.camera_pitch_attr.Get()

        new_distance = current_distance + distance_increment
        new_yaw = current_yaw + yaw + yaw
        new_pitch = current_pitch + pitch + pitch

        self.camera_distance_attr.Set(new_distance)
        self.camera_yaw_attr.Set(new_yaw)
        self.camera_pitch_attr.Set(new_pitch)

    # -- FUNCTION change_camera_subject -------------------------------------------------------------------------
    # This function modifies the subject property from our camera to look to the corresponding UAV
    # -----------------------------------------------------------------------------------------------------------
    def change_camera_subject(self, new_target_path):
        if hasattr(self, "camera") and (self.camera is not None):
            subject_rel = self.camera.GetRelationship("physxCamera:subject")
            subject_rel.SetTargets([new_target_path])
            self.camera.Unload()
            self.camera.Load()