import carb

from omni.kit.scripting import BehaviorScript
from pxr import Gf
import omni.physx


class FuerzasUsd(BehaviorScript):
    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")
        self.physx_interface = omni.physx.get_physx_interface()
        self.physx_interface_sub = self.physx_interface.subscribe_physics_step_events(self.on_physics_step)
        self.force_atr = self.prim.GetAttribute("physxForce:force")
        self.torque_atr = self.prim.GetAttribute("physxForce:torque")
        self.time_steps = 0

    def on_destroy(self):
        pass

    def on_play(self):
        pass

    def on_pause(self):
        pass

    def on_stop(self):
        self.time_steps = 0

    def on_update(self, current_time: float, delta_time: float):
        pass

    def on_physics_step(self, dt):
        self.force_atr.Set(Gf.Vec3f(0,0,9.81))
        self.torque_atr.Set(Gf.Vec3f(0,0,0.1))
        carb.log_info(f"(usd_cube) time step = {self.time_steps}")
