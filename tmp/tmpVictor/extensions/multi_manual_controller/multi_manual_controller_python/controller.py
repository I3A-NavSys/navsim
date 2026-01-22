import numpy as np
import pickle
import base64


import omni.physx


from uspace.flight_plan.command import Command
from navsim_utils.sim_utils import *
from .joysticks import Joysticks

class Controller:
    def __init__(self):
        self.current_time = 0
        self.is_running = False
        self.linear_vel_limit = 6.0
        self.ang_vel_limit = 0.7
        self.joysticks = Joysticks()
        self.physics_sub = None
        
    def start(self, uav_control):
        if not self.is_running:
            self.uav_control = uav_control
            self.uav_names = list(self.uav_control.uavs.keys())
            self.is_running = True

            self.uavs_rottors_on_change = [False for i in range(self.uav_control.rigid_prim_view.count)]
            self.uavs_current_rottors_on = [False for i in range(self.uav_control.rigid_prim_view.count)]
            
            self.joysticks.start()

            if self.physics_sub is None:
                # Attach control updates to physics steps
                self.physx_interface = omni.physx.get_physx_interface()
                self.physics_sub = self.physx_interface.subscribe_physics_step_events(self.control)

    def stop(self):
        if self.is_running:
            self.is_running = False

            if self.physics_sub is not None:
                self.physics_sub.unsubscribe()
                self.physics_sub = None
                
            self.joysticks.stop()
            self.current_time = 0

    def control(self, step_size):
        joysticks_ids, joysticks_inputs = self.joysticks.get_inputs()

        for i in range(len(joysticks_ids)):
            # Get required info
            uav_name = self.uav_names[i]
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
                self.uav_control.x[i]  = np.zeros((8, 1))  # model state
                self.uav_control.y[i]  = np.zeros((4, 1))  # model output
                self.uav_control.u[i]  = np.zeros((4, 1))  # input (rotors speeds)
                self.uav_control.r[i]  = np.zeros((4, 1))  # model reference
                self.uav_control.e[i]  = np.zeros((4, 1))  # model error
                self.uav_control.E[i]  = np.zeros((4, 1))  # model accumulated error

                self.uav_control.rigid_prim_view.set_world_poses(
                    positions=[self.uav_control.rigid_prim_view.get_world_poses([i])[0][0]],
                    orientations=[(1, 0, 0, 0)],
                    indices=i,
                    usd=False
                )
                self.uav_control.rigid_prim_view.set_velocities(
                    velocities=np.zeros(6),
                    indices=i,
                )

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
                            duration = 0.1)

            uav_i = int(uav_name.removeprefix("UAV_"))
            self.uav_control.commands[uav_i] = command
            self.uav_control.cmd_exp_time[uav_i] = self.current_time + command.duration
            # serialized_cmd = base64.b64encode(pickle.dumps(command)).decode('utf-8')
            # self.inform_operator(TypeMessage.CMD_FP_REQUEST, uav_i, serialized_cmd)

            # print(self.uav_control.commands[uav_i].print_command())

    def check_joysticks(self):
        _, joysticks_inputs = self.joysticks.get_inputs()
        return joysticks_inputs
    
    def inform_operator(self, type_message, uav_id, cmd):
        """Inform the operator about the command to be sent to the UAV"""
        
        payload = {"type_message": type_message}
        msg = {"sender": TypeSender.COMMAND_GENERATOR}

        match type_message:
            case TypeMessage.CMD_FP_REQUEST:
                msg["request"] = {
                    "uav_id": uav_id,
                    "cmd": cmd
                }

        payload["msg"] = msg

        self.event_stream.push(self.operator_uav_event, payload=payload)