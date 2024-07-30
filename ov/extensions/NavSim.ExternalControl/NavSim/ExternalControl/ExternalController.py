from .JoystickInput import JoystickInput
from .KeyboardInput import KeyboardInput

import numpy as np
import asyncio

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

    
    def start(self, prim):
        if self._stop:
            self._stop = False

            # Selected drone
            self.prim = prim

            # Attributes required
            self._vel_att = self.prim.GetAttribute("physics:velocity")
            self._ang_vel_att = self.prim.GetAttribute("physics:angularVelocity")
            self._rb_gravity_att = self.prim.GetAttribute("physxRigidBody:disableGravity")

            # New values for those attributes
            self._vel_to_apply = Gf.Vec3f(0,0,0)
            self._ang_vel_to_apply = Gf.Vec3f(0,0,0)

            # Needed variables
            self.last_rotation = 0
            self.last_z_neg_force = 0
            self.inputs = [0,0,0,0]

            # Reset attributes
            self._vel_att.Set(self._vel_to_apply)
            self._ang_vel_att.Set(self._ang_vel_to_apply)
            self.prim.GetAttribute("physxForce:force").Set(Gf.Vec3f(0,0,0))
            
            # Start external inputs
            self.joystick.start()
            self.keyboard.start()

            # Start control coroutine
            asyncio.ensure_future(self.control())


    def stop(self):
        if not self._stop:
            self._stop = True

            self.joystick.stop()
            self.keyboard.stop()


    # -- FUNCTION get_grav_deacc --------------------------------------------------------------------------------
    # This method simply gets the inputs from both the joystick and the keyboard, then decide which one to use
    # Afterwards, it calls the corresponding methods to control the UAV
    # -----------------------------------------------------------------------------------------------------------
    async def control(self):
        while not self._stop:
            # Get the corresponding inputs
            self.joystick.ask_input()
            joy_inputs = np.array(self.joystick.inputs)
            key_inputs = np.array(self.keyboard.inputs)

            # Joystick input has priority
            if self.check_inputs(joy_inputs):
                self.inputs = joy_inputs
            else:
                self.inputs = key_inputs

            # Compute velocities
            self.compute_ang_vel()
            self.compute_lin_vel()

            # Update velocities
            self._vel_att.Set(self._vel_to_apply)
            self._ang_vel_att.Set(self._ang_vel_to_apply)

            await asyncio.sleep(0.1)


    # -- FUNCTION get_grav_deacc --------------------------------------------------------------------------------
    # This method just checks if we have any input from the receiving parameter (joystick as it has priority)
    # -----------------------------------------------------------------------------------------------------------
    def check_inputs(self, inputs):
        for value in inputs:
            if value != 0:
                return True
            
        return False
    

    # -- FUNCTION get_grav_deacc --------------------------------------------------------------------------------
    # This method is used to simulate gravity when the default one is not working properly
    # -----------------------------------------------------------------------------------------------------------
    def get_grav_deacc(self, current_vel):
        return current_vel - 9.8 * self._time_step
    

    # -- FUNCTION stop_inertia ----------------------------------------------------------------------------------
    # This method just smooths the rotation of the drone
    # -----------------------------------------------------------------------------------------------------------
    def stop_inertia(self, last_rotate):
        if last_rotate != 0:
            if last_rotate > 0 and last_rotate < 8: return 0
            if last_rotate < 0 and last_rotate > -8: return 0

            if last_rotate > 0: return last_rotate - 8
            if last_rotate < 0: return last_rotate + 8

        return 0
    
    
    # -- FUNCTION get_fordward_vector ---------------------------------------------------------------------------
    # This method is in charge of computing the fordward vector of the drone, so that linear velocities are
    # properly setted
    # -----------------------------------------------------------------------------------------------------------
    def get_fordward_vector(self, axis):
        xformable = UsdGeom.Xformable(self.prim)
        transform_matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        match axis:
            # X is fordward
            case 0:
                fordward_vector = Gf.Vec3f(transform_matrix[0][0], transform_matrix[0][1], transform_matrix[0][2])                

            # Y is fordward
            case 1:
                fordward_vector = Gf.Vec3f(transform_matrix[1][0], transform_matrix[1][1], transform_matrix[1][2])

        return fordward_vector
    

    # -- FUNCTION get_orthogonal_vectors ------------------------------------------------------------------------
    # This method is similar to get_fordward_vector
    # It gets the orthogonal vectors to the fordward one, so that the drone can move among the three axis
    # -----------------------------------------------------------------------------------------------------------
    def get_orthogonal_vectors(self, forward_axis, fordward_vector):
        match forward_axis:
            # X is fordward
            case 0:
                up_vector = Gf.Vec3f(0,0,1)

                left_vector = Gf.Cross(up_vector, fordward_vector)
                up_vector = Gf.Cross(fordward_vector, left_vector)

                return left_vector, up_vector
            
            # Y is fordward
            case 1:
                up_vector = Gf.Vec3f(0,0,1)

                right_vector = Gf.Cross(fordward_vector, up_vector)
                up_vector = Gf.Cross(right_vector, fordward_vector)

                return right_vector, up_vector
            

    # -- FUNCTION compute_ang_vel -------------------------------------------------------------------------------
    # This method is in charge of controlling the angular velocity, and hence the rotation of the UAV
    # -----------------------------------------------------------------------------------------------------------
    def compute_ang_vel(self):
        rotation = self.inputs[3] * -self.ang_vel_limit

        # If rotation is applied, accumulate it for a simulated inertia
        if rotation != 0:
            self.last_rotation += rotation
            
        # Simulate inertia
        else:
            self.last_rotation = self.stop_inertia(self.last_rotation)

        # Update orienation
        self._ang_vel_to_apply = Gf.Vec3f(0, 0, self.last_rotation)


    # -- FUNCTION compute_lin_vel -------------------------------------------------------------------------------
    # This method is similar to compute_ang_vel, but it controls the linear velocity
    # -----------------------------------------------------------------------------------------------------------
    def compute_lin_vel(self):
        # Boolean smoothing variables
        if self.inputs[0] == 0.0 and self.inputs[1] == 0.0: 
            smooth_xy = True
        else: 
            smooth_xy = False


        if self.inputs[2] >= 0.9: 
            gravity = True
            self.last_z_neg_force = self._vel_att.Get()[2]
        else: 
            gravity = False

        # if joy_force[2] > 0: joy_force[2] = 0     # Avoid down force
        inputs = self.inputs * -self.linear_vel_limit          # Revert axis and set maximum limit

        # Get vector directions
        fordward_vector = self.get_fordward_vector(0)
        left_vector, up_vector = self.get_orthogonal_vectors(0, fordward_vector)

        # Moving and going up
        if not smooth_xy and not gravity:
            # Disable gravity to avoid flickering
            self._rb_gravity_att.Set(True)

            x_dir_vel = fordward_vector * inputs[0]
            y_dir_vel = left_vector * inputs[1]
            z_dir_vel = up_vector * inputs[2]

            if self.last_z_neg_force < inputs[2]:
                self.last_z_neg_force += 0.8
                z_dir_vel = Gf.Vec3f(0,0, self.last_z_neg_force)
            
            self._vel_to_apply = x_dir_vel + y_dir_vel + z_dir_vel

        # Not moving
        elif smooth_xy:
            current_x_vel = self._vel_att.Get()[0]
            current_y_vel = self._vel_att.Get()[1]
            current_z_vel = self._vel_att.Get()[2]

            # Smooth X
            if current_x_vel >= 0 and current_x_vel < 0.5: current_x_vel = 0
            elif current_x_vel <= 0 and current_x_vel > -0.5: current_x_vel = 0

            if current_x_vel >= 0.5: current_x_vel -= 0.5
            elif current_x_vel <= -0.5: current_x_vel += 0.5

            # Smooth Y
            if current_y_vel >= 0 and current_y_vel < 0.5: current_y_vel = 0
            elif current_y_vel <= 0 and current_y_vel > -0.5: current_y_vel = 0

            if current_y_vel >= 0.5: current_y_vel -= 0.5
            elif current_y_vel <= -0.5: current_y_vel += 0.5

            self._vel_to_apply = Gf.Vec3f(current_x_vel, current_y_vel, current_z_vel)

            # Not moving and going down
            if gravity:
                # Uncomment this if nvidia gravity is not working properly, and comment the gravity attribute
                # current_z_vel = self.get_grav_deacc(current_z_vel)
                # self._vel_to_apply = Gf.Vec3f(current_x_vel, current_y_vel, current_z_vel)
                
                # Enable gravity
                self._rb_gravity_att.Set(False)

            # Not moving and going up
            else:
                # Disable gravity to avoid flickering
                self._rb_gravity_att.Set(True)

                if self.last_z_neg_force < inputs[2]:
                    self.last_z_neg_force += 0.8
                    self._vel_to_apply = Gf.Vec3f(current_x_vel, current_y_vel, self.last_z_neg_force)

                else:
                    z_dir_vel = up_vector * inputs[2]
                    z_dir_vel[0] = current_x_vel
                    z_dir_vel[1] = current_y_vel

                    self._vel_to_apply = z_dir_vel

        # Moving and going down
        else:
            # Uncomment this if nvidia gravity is not working properly, and comment the gravity attribute
            # current_z_vel = self._vel_att.Get()[2]
            # current_z_vel = self.get_grav_deacc(current_z_vel)

            # Enable gravity
            self._rb_gravity_att.Set(False)

            x_dir_vel = fordward_vector * inputs[0]
            y_dir_vel = left_vector * inputs[1]
            z_dir_vel = Gf.Vec3f(0,0, self._vel_att.Get()[2])

            self._vel_to_apply = x_dir_vel + y_dir_vel + z_dir_vel