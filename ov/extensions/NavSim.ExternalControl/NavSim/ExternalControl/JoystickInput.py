import pygame
import asyncio
import numpy as np

from omni.isaac.core.utils.stage import get_current_stage

from pxr import UsdGeom, Gf, Usd

class JoystickInput:
    def __init__(self):
        self.inputs = [0,0,0,0]
        self.event = asyncio.Event()

    def start(self):
        # Loop condition
        self._stop = False

        # Initialize pygame environment
        pygame.init()
        pygame.joystick.init()
        
        # Start input coroutine
        asyncio.ensure_future(self.input())


    def stop(self):
        self._stop = True
        self.event.set()

        # Finish pygame environment
        pygame.joystick.quit()
        pygame.quit()


    async def input(self):
        joystick = None

        while not self._stop:
            await self.event.wait()

            # Avoid exception when saving file while running simulation
            if self._stop:
                break

            # Check if joystick is connected
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    joystick = pygame.joystick.Joystick(event.device_index)

            if joystick != None:
                # Joystick axis
                self.inputs[0] = round(joystick.get_axis(1), 2)    # Left - Right
                self.inputs[1] = round(joystick.get_axis(0), 2)    # Fordward - Backward
                self.inputs[2] = round(joystick.get_axis(3), 2)    # Slider
                self.inputs[3] = round(joystick.get_axis(2), 2)    # Rotation Left - Right

            self.event.clear()

    
    def ask_input(self):
        self.event.set()

    
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