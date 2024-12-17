import carb
import numpy as np

from omni.kit.scripting import BehaviorScript
from omni.isaac.core.prims import RigidPrimView


class FuerzasRpv(BehaviorScript):
    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")
        self.rigid_prim_view = RigidPrimView("/World/cube_rpv")
        self.rpv_initialized = False
        self.force = np.array([0,0,9.81])
        self.torque = np.array([0,0,1.0])
        self.mass = np.array([1.0])

    def on_destroy(self):
        pass

    def on_play(self):
        pass

    def on_pause(self):
        pass

    def on_stop(self):
        self.rpv_initialized = False

    def on_update(self, current_time: float, delta_time: float):
        if not self.rpv_initialized:
            self.rigid_prim_view.initialize()
            self.rigid_prim_view.set_masses(np.array(self.mass))
            self.rpv_initialized = True

        self.rigid_prim_view.apply_forces_and_torques_at_pos(forces=np.array(self.force), torques=np.array(self.torque))
