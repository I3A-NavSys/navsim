import numpy as np

import omni.physx

from navsim_utils.sim_utils import *
from .joysticks import Joysticks

class Controller:
    def __init__(self):
        self.is_running = False
        self.linear_vel_limit = 6.0
        self.ang_vel_limit = 0.7
        self.joysticks = Joysticks()
        
    def start(self, uav_control):
        if not self.is_running:
            self.uav_control = uav_control

            self.uavs_rottors_on_change = np.zeros(self.uav_control.rigid_prim_view.count, dtype=bool)
            self.uavs_current_rottors_on = np.zeros(self.uav_control.rigid_prim_view.count, dtype=bool)
            # self.uavs_rottors_on_change = [False for i in range(self.uav_control.rigid_prim_view.count)]
            # self.uavs_current_rottors_on = [False for i in range(self.uav_control.rigid_prim_view.count)]
            
            self.joysticks.start()

            self.is_running = True

    def stop(self):
        if self.is_running:
            self.joysticks.stop()

            self.is_running = False

    def control(self):
        joysticks_ids, joysticks_inputs = self.joysticks.get_inputs()
        amount_joysticks = len(joysticks_ids)

        cmd_local_linear_vel = np.zeros((amount_joysticks, 3))
        cmd_yaw_rotation = np.zeros(amount_joysticks)

        for i in range(amount_joysticks):
            # Get required info
            joystick_id = joysticks_ids[i]
            joystick_input = joysticks_inputs[joystick_id]
            rottors_on_change = self.uavs_rottors_on_change[i]
            current_rottors_on = self.uavs_current_rottors_on[i]

            # Get velocities and rotation
            vel = joystick_input[:3] * -self.linear_vel_limit
            rot = joystick_input[3] * -self.ang_vel_limit

            # Get rottors on/off
            rottors_on = joystick_input[4]

            # Get position reseting
            reset_position = joystick_input[5]

            # Reset position if that is the case
            if reset_position == 1:
                self.uav_control.reset_pose([i])

            # Evaluate whether rottors should be on off according to same button
            if rottors_on == 1 and not rottors_on_change:
                self.uavs_rottors_on_change[i] = True
                current_rottors_on = not current_rottors_on
                self.uavs_current_rottors_on[i] = current_rottors_on

            elif rottors_on == 0:
                self.uavs_rottors_on_change[i] = False

            # Set command
            cmd_local_linear_vel[i] = vel
            cmd_yaw_rotation[i] = rot

        return cmd_local_linear_vel, cmd_yaw_rotation, amount_joysticks

    def check_joysticks(self):
        _, joysticks_inputs = self.joysticks.get_inputs()
        return joysticks_inputs
    
