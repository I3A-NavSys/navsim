import carb

from omni.kit.scripting import BehaviorScript
from pxr import Gf
import omni.physx


class FuerzasUsd(BehaviorScript):
    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")
        self.force_atr = self.prim.GetAttribute("physxForce:force")
        self.force_atr.Set(Gf.Vec3f(0,0,0))
        self.torque_atr = self.prim.GetAttribute("physxForce:torque")
        self.torque_atr.Set(Gf.Vec3f(0,0,0))

    def on_destroy(self):
        pass

    def on_play(self):
        pass

    def on_pause(self):
        pass

    def on_stop(self):
        pass

    def on_update(self, current_time: float, delta_time: float):
        self.force_atr.Set(Gf.Vec3f(0,-0.1,9.81))
        self.torque_atr.Set(Gf.Vec3f(0,0,0))
