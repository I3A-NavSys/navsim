import carb
import numpy as np
import omni.physx
import omni.timeline
from omni.kit.scripting import BehaviorScript
from omni.isaac.core.prims import RigidPrimView


class FuerzasRpv(BehaviorScript):
    def on_physics_step(self, step_size):
        self.current_time += step_size
        self.delta_time = step_size

        self.update()

    def reset_current_time(self, event):
        self.current_time = 0

    def on_init(self):
        self.timeline_sub_stop = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.reset_current_time)

        self.physx_interface = omni.physx.get_physx_interface()
        self.physics_sub = self.physx_interface.subscribe_physics_step_events(self.on_physics_step)

        self.current_time = 0
        self.delta_time = 0

        self.body_link_path = "/World/test_2/cube_rpv/body_link"
        self.r_left_link_path = "/World/test_2/cube_rpv/r_left_link"
        self.r_right_link_path = "/World/test_2/cube_rpv/r_right_link"

        self.rigid_prim_view = RigidPrimView([self.body_link_path, self.r_left_link_path, self.r_right_link_path])
        self.rpv_initialized = False

    def on_destroy(self):
        self.timeline_sub_stop = None
        self.physics_sub = None

    def on_play(self):
        self.bl_force = np.array([0,0,0])
        self.bl_torque = np.array([0,0,1])
        self.rl_force = np.array([0,0,4.95])
        self.rl_torque = np.array([0,0,0])

        self.forces = [self.bl_force, self.rl_force, self.rl_force]
        self.torques = [self.bl_torque, self.rl_torque, self.rl_torque]

    def on_pause(self):
        pass

    def on_stop(self):
        self.rpv_initialized = False
        self.counter = 0

    # def on_update(self, current_time: float, delta_time: float):
    def update(self):
        if not self.rpv_initialized:
            self.rigid_prim_view.initialize()
            # self.rigid_prim_view.set_masses(np.array(self.mass))
            self.rpv_initialized = True

        bl_ang_vel = self.rigid_prim_view.get_angular_velocities()[0]
        bl_lin_vel = self.rigid_prim_view.get_linear_velocities()[0]
        print(f" {self.current_time} - RPV_AV: {bl_ang_vel}")
        # print("LV:", bl_lin_vel)

        if self.current_time < 2:
            self.rigid_prim_view.apply_forces_and_torques_at_pos(forces=self.forces, torques=self.torques, is_global=False)
        elif self.current_time < 4:
            self.bl_torque[2] = -1
            self.torques = np.array([self.bl_torque, self.rl_torque, self.rl_torque])
            self.rigid_prim_view.apply_forces_and_torques_at_pos(forces=self.forces, torques=self.torques, is_global=False)
        else:
            if bl_ang_vel[2] > -0.01 and bl_ang_vel[2] < 0.01:
                self.bl_torque[2] = 0
            else:
                self.bl_torque[2] = 1
            self.torques = np.array([self.bl_torque, self.rl_torque, self.rl_torque])
            self.rigid_prim_view.apply_forces_and_torques_at_pos(forces=self.forces, torques=self.torques, is_global=False)             