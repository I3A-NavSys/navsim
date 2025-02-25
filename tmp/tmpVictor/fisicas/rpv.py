import carb
import numpy as np
import omni.physx

from omni.kit.scripting import BehaviorScript
from omni.isaac.core.prims import RigidPrimView


class FuerzasRpv(BehaviorScript):
    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")
        self.physx_interface = omni.physx.get_physx_interface()
        self.physx_interface_sub = self.physx_interface.subscribe_physics_on_step_events(self.on_physics_step, True, 0)
        self.rigid_prim_view = RigidPrimView(["/World/cubo/chasis", "/World/cubo/rotor_NW", "/World/cubo/rotor_NE", 
                                              "/World/cubo/rotor_SW", "/World/cubo/rotor_SE"])
        self.rpv_initialized = False
        self.time_steps = 0

    def on_destroy(self):
        self.physx_interface_sub = None

    def on_play(self):
        self.forces = np.array([[0.1,0,0], [0,0,4.905], [0,0,4.905], [0,0,4.905], [0,0,4.905]])
        self.torques = np.array([[0,0,0.1], [0,0,0], [0,0,0], [0,0,0], [0,0,0]])

    def on_pause(self):
        pass

    def on_stop(self):
        self.rpv_initialized = False
        self.time_steps = 0

    def on_update(self, current_time: float, delta_time: float):
        pass

    def on_physics_step(self, dt):
        self.time_steps = self.time_steps + 1

        if not self.rpv_initialized:
            self.rigid_prim_view.initialize()
            self.rpv_initialized = True

        self.rigid_prim_view.apply_forces_and_torques_at_pos(forces=np.array(self.forces), torques=np.array(self.torques),
                                                             is_global=False)
