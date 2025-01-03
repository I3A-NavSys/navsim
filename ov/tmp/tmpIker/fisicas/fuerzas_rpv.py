import carb
import numpy as np

from omni.kit.scripting import BehaviorScript
from omni.isaac.core.prims import RigidPrimView
import omni.physx


class FuerzasRpv(BehaviorScript):
    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")
        self.physx_interface = omni.physx.get_physx_interface()
        self.physx_interface_sub = self.physx_interface.subscribe_physics_on_step_events(self.on_physics_step, True, 0)
        self.rigid_prim_view = RigidPrimView("/World/cube_rpv")
        self.rpv_initialized = False
        self.force = np.array([0,0,9.81])
        self.torque = np.array([0,0,0.1])

    def on_destroy(self):
        pass

    def on_play(self):
        pass

    def on_pause(self):
        pass

    def on_stop(self):
        self.rpv_initialized = False

    def on_update(self, current_time: float, delta_time: float):
        pass

    def on_physics_step(self, dt):
        if not self.rpv_initialized:
            self.rigid_prim_view.initialize()
            self.rpv_initialized = True

        self.rigid_prim_view.apply_forces_and_torques_at_pos(forces=np.array(self.force), torques=np.array(self.torque))
        carb.log_info("on_physics_step!!!\n")
