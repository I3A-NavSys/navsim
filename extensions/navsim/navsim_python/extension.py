import numpy as np

import omni.ext
import logging
import omni.timeline
import omni.usd
from pxr import Tf, Usd
import omni.physx
from omni.isaac.core.prims import RigidPrimView

from navsim_utils.sim_utils import *
from .ui_builder import UIBuilder
from .navsim import NavSimManager

class NavSim(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.initialize_variables()
        self.build_ui()
        self.navsim_manager.startup()

    def on_shutdown(self) -> None:
        self.reset_control_variables()
        self.disconnect_callbacks()
        self.navsim_manager.shutdown()

    def on_timeline_play(self, event):
        # Recover from pause
        if self.is_simulation_running:
            self.time_manager.resume()
            self.navsim_manager.start_simulation()
            return
        
        # if self.has_stage_been_modified:
        #     self.navsim_manager.shutdown()
        #     self.navsim_manager.startup()
        #     self.has_stage_been_modified = False

        # Start RigidPrimView
        self.rigid_prim_view = RigidPrimView(
            prim_paths_expr=["/World/*/*/*/UAV_*"],
            # prim_paths_expr=["/World/*/*/*/UAV_01", "/World/*/*/*/UAV_02"],
            name="NavSimRigidPrimView",
        )
        self.rigid_prim_view.initialize()

        # Start NavSim simulation
        self.navsim_manager.start_simulation()

        # Set simulation as running
        self.is_simulation_running = True

    def on_timeline_stop(self, event):
        # Reset control variables
        self.reset_control_variables()

        # Stop NavSim simulation
        self.navsim_manager.stop_simulation()

        # Reset RigidPrimView
        self.rigid_prim_view = None
        
    def on_timeline_pause(self, event):
        self.time_manager.pause()
        self.navsim_manager.pause_simulation()

    def on_stage_event(self, event):
        # If a new stage is opened while the simulation is running, 
        # stop the simulation and scan the new scene for NavSim entities
        if event.type == int(omni.usd.StageEventType.OPENED):
            self.reset_control_variables()
            self.navsim_manager.shutdown()
            self.navsim_manager.startup()

    def on_stage_modification(self, notice, sender):
        # Check addition/removal of prims in the stage to update NavSim entities lists
        if notice.GetResyncedPaths():
            self.has_stage_been_modified = True

    def on_physics_step(self, step_size: float):
        if self.is_simulation_running:
            forces = np.tile(np.array([[0, 0, -9.81 * 2000]]), (self.rigid_prim_view.count, 1))
            self.rigid_prim_view.apply_forces_and_torques_at_pos(
                forces=forces,
                torques=np.zeros_like(forces),
                indices=np.array(range(self.rigid_prim_view.count)),
                is_global=False
            )


    def initialize_variables(self):
        # UI Builder
        self.ui_builder = UIBuilder()

        # Logger
        self.logger = logging.getLogger("NavSim")

        # Control
        self.is_simulation_running = False
        self.has_stage_been_modified = False

        # Managers
        self.time_manager = TimeManager()
        self.navsim_manager = NavSimManager()

        # Timeline callbacks
        self.timeline = omni.timeline.get_timeline_interface()
        timeline_stream = self.timeline.get_timeline_event_stream()

        self.on_stop_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), 
            self.on_timeline_stop
        )

        self.on_play_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), 
            self.on_timeline_play
        )

        self.on_pause_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PAUSE), 
            self.on_timeline_pause
        )

        # Stage event callbacks
        self.usd_context = omni.usd.get_context()

        self.on_stage_event_sub = self.usd_context.get_stage_event_stream().create_subscription_to_pop(
            self.on_stage_event
        )

        self.on_stage_modification_sub = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged,
            self.on_stage_modification, 
            None
        )

        # Physx callbacks
        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_step_events(
            self.on_physics_step,
        )

    def build_ui(self) -> None:
        self.ui_builder.build_ui()


    def reset_control_variables(self):
        self.is_simulation_running = False
        self.has_stage_been_modified = False

    def disconnect_callbacks(self):
        self.on_stop_sub = None
        self.on_play_sub = None
        self.on_pause_sub = None
        self.on_stage_event_sub = None
        self.on_stage_modification_sub.Revoke()