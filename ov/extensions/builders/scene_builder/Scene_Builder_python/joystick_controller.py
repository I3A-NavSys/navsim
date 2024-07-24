import pygame
import asyncio
import numpy as np

from omni.isaac.core.utils.stage import get_current_stage

from pxr import UsdGeom, Gf, UsdLux, Sdf, Usd

class Joystick_controller:
    def __init__(self):
        self._run = False
        self._time_step = 1/100

    def start(self, prim):
        if not self._run:
            print("-- STARTED --------------------------------------")
            # Current drone
            self.prim = prim

            # Attributes required
            self._vel_att = self.prim.GetAttribute("physics:velocity")
            self._ang_vel_att = self.prim.GetAttribute("physics:angularVelocity")
            self._rb_gravity_att = self.prim.GetAttribute("physxRigidBody:disableGravity")

            # New values for those attributes
            self._vel_to_apply = Gf.Vec3f(0,0,0)
            self._ang_vel_to_apply = Gf.Vec3f(0,0,0)

            # Needed variables
            self._run = False
            self.last_rotate = 0
            self.last_z_neg_force = 0

            # Reset attributes
            self._vel_att.Set(self._vel_to_apply)
            self._ang_vel_att.Set(self._ang_vel_to_apply)
            self.prim.GetAttribute("physxForce:force").Set(Gf.Vec3f(0,0,0))
            
            # Initialize pygame environment
            pygame.init()
            pygame.joystick.init()
            
            # Start joystick controller coroutine
            self._run = True
            asyncio.ensure_future(self.new_control())


    def stop(self):
        if self._run:
            print("-- STOPPED --------------------------------------")
            # Reset variables and attributes
            self._vel_to_apply = Gf.Vec3f(0,0,0)
            self._ang_vel_to_apply = Gf.Vec3f(0,0,0)

            self._run = False
            self.last_rotate = 0
            self.last_z_neg_force = 0

            # Finish pygame environment
            pygame.joystick.quit()
            pygame.quit()


    async def new_control(self):
        while self._run:
            # Check if joystick is connected
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    joystick = pygame.joystick.Joystick(event.device_index)

            # Joystick axis
            x_force = joystick.get_axis(1)    # Left - Right
            y_force = joystick.get_axis(0)    # Fordward - Backward
            z_force = joystick.get_axis(3)    # Slider
            z_rot = joystick.get_axis(2)    # Rotation Left - Right


            # Get joystick rotation
            joy_rotate = round(z_rot, 2) * -5

            # If rotation is applied, accumulate it for a simulated inertia
            if joy_rotate != 0:
                self.last_rotate += joy_rotate
                
            # Simulate inertia
            else:
                self.last_rotate = self.stop_inertia(self.last_rotate)

            # Update orienation
            self._ang_vel_to_apply = Gf.Vec3f(0, 0, self.last_rotate)



            # Boolean smoothing variables
            if x_force == 0.0 and y_force == 0.0: 
                smooth_xy = True

            else: 
                smooth_xy = False


            if z_force >= 0.9: 
                gravity = True
                self.last_z_neg_force = self._vel_att.Get()[2]

            else: 
                gravity = False

            # Get force vector
            joy_force = np.array([round(x_force, 2), round(y_force, 2), round(z_force, 2)])

            # if joy_force[2] > 0: joy_force[2] = 0   # Avoid down force
            joy_force *= -4                         # Revert axis and increase force

            # Get vector directions
            fordward_vector = self.get_fordward_vector(0)
            left_vector, up_vector = self.get_orthogonal_vectors(0, fordward_vector)

            # Moving and going up
            if not smooth_xy and not gravity:
                # Disable gravity to avoid flickering
                self._rb_gravity_att.Set(True)

                x_dir_vel = fordward_vector * joy_force[0]
                y_dir_vel = left_vector * joy_force[1]
                z_dir_vel = up_vector * joy_force[2]

                if self.last_z_neg_force < joy_force[2]:
                    self.last_z_neg_force += 0.8
                    z_dir_vel = Gf.Vec3f(0,0, self.last_z_neg_force)
                
                self._vel_to_apply = x_dir_vel + y_dir_vel + z_dir_vel

            # Not moving
            elif smooth_xy:
                current_x_vel = self._vel_att.Get()[0]
                current_y_vel = self._vel_att.Get()[1]
                current_z_vel = self._vel_att.Get()[2]

                # Smooth X
                if current_x_vel >= 0 and current_x_vel < 0.1: current_x_vel = 0
                elif current_x_vel <= 0 and current_x_vel > -0.1: current_x_vel = 0

                if current_x_vel >= 0.1: current_x_vel -= 0.1
                elif current_x_vel <= -0.1: current_x_vel += 0.1

                # Smooth Y
                if current_y_vel >= 0 and current_y_vel < 0.1: current_y_vel = 0
                elif current_y_vel <= 0 and current_y_vel > -0.1: current_y_vel = 0

                if current_y_vel >= 0.1: current_y_vel -= 0.1
                elif current_y_vel <= -0.1: current_y_vel += 0.1

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

                    if self.last_z_neg_force < joy_force[2]:
                        self.last_z_neg_force += 0.8
                        self._vel_to_apply = Gf.Vec3f(current_x_vel, current_y_vel, self.last_z_neg_force)

                    else:
                        z_dir_vel = up_vector * joy_force[2]
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

                x_dir_vel = fordward_vector * joy_force[0]
                y_dir_vel = left_vector * joy_force[1]
                z_dir_vel = Gf.Vec3f(0,0, self._vel_att.Get()[2])

                self._vel_to_apply = x_dir_vel + y_dir_vel + z_dir_vel


            # Update drone control
            self._vel_att.Set(self._vel_to_apply)
            self._ang_vel_att.Set(self._ang_vel_to_apply)

            await asyncio.sleep(0.1)

    
    def get_grav_deacc(self, current_vel):
        return current_vel - 9.8 * self._time_step
    

    def stop_inertia(self, last_rotate):
        if last_rotate != 0:
            if last_rotate > 0 and last_rotate < 8: return 0
            if last_rotate < 0 and last_rotate > -8: return 0

            if last_rotate > 0: return last_rotate - 8
            if last_rotate < 0: return last_rotate + 8

        return 0
    

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