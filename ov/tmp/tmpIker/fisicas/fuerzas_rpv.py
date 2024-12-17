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

        self.rigid_prim_view = RigidPrimView("/World/test_1/cube_rpv")
        self.rpv_initialized = False
        self.force = np.array([0,0,9.81])
        self.torque = np.array([0,0,1])
        # self.mass = np.array([1.0])

    def on_destroy(self):
        self.timeline_sub_stop = None
        self.physics_sub = None

    def on_play(self):
        pass

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

        self.rigid_prim_view.apply_forces_and_torques_at_pos(forces=np.array(self.force), torques=np.array(self.torque), is_global=False)
