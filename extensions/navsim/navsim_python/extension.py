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
from .uav_control import UAVControl

class NavSim(omni.ext.IExt):
    # ---------------
    # -- Callbacks --
    # ---------------
    def on_startup(self, ext_id):
        self.initialize_variables()
        self.build_ui()
        self.navsim_manager.startup()

    def on_shutdown(self) -> None:
        self.reset_control_variables()
        self.disconnect_callbacks()
        self.navsim_manager.shutdown()

    def on_physics_step(self, step_size: float):
        if self.is_simulation_running:
            # Get current time
            current_time = self.timeline.get_current_time()

            # Update time manager current times
            self.time_manager.current_sim_time = current_time
            self.time_manager.current_real_time = self.time_manager.sim_to_real(current_time)

            # Get current flightplans to update UAV control
            uav_physics_indices, current_flightplans = (
                self.navsim_manager.get_all_current_flitghplans(current_time)
            )

            # self.ui_builder.current_flightplans = current_flightplans

            # Update UAV control
            if uav_physics_indices:
                self.uav_control.update(
                    uav_physics_indices, 
                    current_flightplans, 
                    current_time, 
                    step_size
                )
            
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

        # Start time manager
        self.time_manager.start()

        # Start RigidPrimView
        self.rigid_prim_view = RigidPrimView(
            prim_paths_expr=["/World/*/*/*/UAV_*"],
            # prim_paths_expr=["/World/*/*/*/UAV_01", "/World/*/*/*/UAV_02"],
            name="NavSimRigidPrimView",
        )
        self.rigid_prim_view.initialize()

        # Relate UAV ids to physics buffer indices
        self.relate_uav_ids_to_physics_buffer()

        # Start UAVControl
        self.uav_control = UAVControl(
            self.rigid_prim_view, 
            self.uav_ids_to_physics_buffer
        )

        # Start NavSim simulation
        self.navsim_manager.start_simulation()
        self.navsim_manager.uav_ids_to_physics_buffer = self.uav_ids_to_physics_buffer

        # Set simulation as running
        self.is_simulation_running = True

    def on_timeline_stop(self, event):
        # Reset control variables
        self.reset_control_variables()

        # Stop NavSim simulation
        self.navsim_manager.stop_simulation()

        # Reset RigidPrimView
        self.rigid_prim_view = None

        # Reset UAVControl
        self.uav_control = None

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

    # -----------------------
    # -- Startup Functions --
    # -----------------------
    def initialize_variables(self):
        # Logger
        self.logger = logging.getLogger("NavSim")

        # Control
        self.is_simulation_running = False
        self.has_stage_been_modified = False
        self.uav_control = None
        self.uav_ids_to_physics_buffer = {} # {operator_id: {uav_id: physics_buffer_index}}

        # Managers
        self.time_manager = TimeManager()
        self.navsim_manager = NavSimManager(self.time_manager)

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
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(
            fn=lambda step_size: self.on_physics_step(step_size),
            pre_step=True,
            order=10
        )

        # UI Builder
        self.ui_builder = UIBuilder(self.navsim_manager)

    def build_ui(self) -> None:
        self.ui_builder.build_ui()

    # -------------------------
    # -- Auxiliary Functions --
    # -------------------------
    def reset_control_variables(self):
        self.is_simulation_running = False
        self.has_stage_been_modified = False

    def disconnect_callbacks(self):
        self.on_stop_sub = None
        self.on_play_sub = None
        self.on_pause_sub = None
        self.on_stage_event_sub = None
        self.on_stage_modification_sub.Revoke()

    def relate_uav_ids_to_physics_buffer(self):
        # Get UAV prims
        uavs = self.rigid_prim_view.prims

        for idx, uav in enumerate(uavs):
            # Get UAV operator id and UAV id from prim attributes
            uav_operator_id = uav.GetAttribute("NavSim:operator_id").Get()
            uav_id = uav.GetAttribute("NavSim:id").Get()

            # Relate UAV id to physics buffer index for each UAV
            if uav_operator_id not in self.uav_ids_to_physics_buffer:
                self.uav_ids_to_physics_buffer[uav_operator_id] = {}

            self.uav_ids_to_physics_buffer[uav_operator_id][uav_id] = idx
            