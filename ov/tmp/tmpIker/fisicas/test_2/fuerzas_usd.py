import carb

from omni.kit.scripting import BehaviorScript
from pxr import Gf
import omni.physx
import omni.timeline
import numpy as np


class Fuerzas2Usd(BehaviorScript):
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

        self.body_link = self.prim.GetChild("body_link")
        self.r_left_link = self.prim.GetChild("r_left_link")
        self.r_right_link = self.prim.GetChild("r_right_link")

        self.bl_ang_vel_atr = self.body_link.GetAttribute("physics:angularVelocity")

        self.bl_force_atr = self.body_link.GetAttribute("physxForce:force")
        self.bl_torque_atr = self.body_link.GetAttribute("physxForce:torque")
        self.rll_force_atr = self.r_left_link.GetAttribute("physxForce:force")
        self.rll_torque_atr = self.r_left_link.GetAttribute("physxForce:torque")
        self.rrl_force_atr = self.r_right_link.GetAttribute("physxForce:force")
        self.rrl_torque_atr = self.r_right_link.GetAttribute("physxForce:torque")

    def on_destroy(self):
        self.timeline_sub_stop = None
        self.physics_sub = None

    def on_play(self):
        pass

    def on_pause(self):
        pass

    def on_stop(self):
        self.bl_force_atr.Set(Gf.Vec3f(0,0,0))
        self.bl_torque_atr.Set(Gf.Vec3f(0,0,0))
        self.rll_force_atr.Set(Gf.Vec3f(0,0,0))
        self.rll_torque_atr.Set(Gf.Vec3f(0,0,0))
        self.rrl_force_atr.Set(Gf.Vec3f(0,0,0))
        self.rrl_torque_atr.Set(Gf.Vec3f(0,0,0))

    # def on_update(self, current_time: float, delta_time: float):
    def update(self):
        bl_ang_vel = self.bl_ang_vel_atr.Get()
        bl_ang_vel = np.radians(bl_ang_vel)
        print(f"{self.current_time} - USD_AV: {bl_ang_vel}")

        if self.current_time < 2:
            self.bl_torque_atr.Set(Gf.Vec3f(0,0,1))
            self.rll_force_atr.Set(Gf.Vec3f(0,0,4.95))
            self.rrl_force_atr.Set(Gf.Vec3f(0,0,4.95))
        elif self.current_time < 4:
            self.bl_torque_atr.Set(Gf.Vec3f(0,0,-1))
        else:
            if bl_ang_vel[2] > -0.5 and bl_ang_vel[2] < 0.5:
                self.bl_torque_atr.Set(Gf.Vec3f(0,0,0))