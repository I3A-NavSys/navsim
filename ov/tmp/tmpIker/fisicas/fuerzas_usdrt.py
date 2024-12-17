import carb
from omni.kit.scripting import BehaviorScript
import omni.physx
import omni.timeline
import omni.usd
from usdrt import Usd, Gf, Sdf

class FuerzasUsdrt(BehaviorScript):
    def on_physics_step(self, step_size):
        self.current_time += step_size
        self.delta_time = step_size

        if not self.are_attributes_initialized:
            self.init_attributes()
            self.are_attributes_initialized = True

        self.update()

    def reset_current_time(self, event):
        self.current_time = 0

    def init_attributes(self):
        self.force_atr = self.usdrt_prim.GetAttribute("physxForce:force")
        self.force_atr.Set(Gf.Vec3f(0,0,0))
        self.torque_atr = self.usdrt_prim.GetAttribute("physxForce:torque")
        self.torque_atr.Set(Gf.Vec3f(0,0,0))

    def on_init(self):
        self.timeline_sub_stop = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.reset_current_time)

        self.physx_interface = omni.physx.get_physx_interface()
        self.physics_sub = self.physx_interface.subscribe_physics_step_events(self.on_physics_step)

        self.current_time = 0
        self.delta_time = 0

        self.usdrt_stage_id = omni.usd.get_context().get_stage_id()
        self.usdrt_stage = Usd.Stage.Attach(self.usdrt_stage_id)
        self.usdrt_prim_path = Sdf.Path("/World/cube_usdrt")
        self.usdrt_prim = self.usdrt_stage.GetPrimAtPath(self.usdrt_prim_path)

        self.are_attributes_initialized = False

    def on_destroy(self):
        self.timeline_sub_stop = None
        self.physics_sub = None

    def on_play(self):
        pass

    def on_pause(self):
        pass

    def on_stop(self):
        pass

    # def on_update(self, current_time: float, delta_time: float):
    def update(self):
        self.force_atr.Set(Gf.Vec3f(0,0,9.81))
        self.torque_atr.Set(Gf.Vec3f(0,0,1.0))
