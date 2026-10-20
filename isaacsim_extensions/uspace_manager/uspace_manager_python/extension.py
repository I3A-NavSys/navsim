
import numpy as np

import omni.ext
from isaacsim.gui.components.ui_utils import ui
import omni.timeline
import omni.physx
from isaacsim.core.prims import RigidPrim, Articulation


class USpaceManager(omni.ext.IExt):
    def on_startup(self, ext_id) -> None:
        self.init_vars()
        self.build_ui()
        
    def on_shutdown(self) -> None:
        self.on_physics_step_sub = None
        self.on_stop_sub = None
        self.event_sub = None
        
    def on_physics_step(self, step_size:int) -> None:
        if self.is_simulation_running:
            for rigid_prim, force_to_apply in zip(self.rigid_prims, self.forces_to_apply):
                rigid_prim.apply_forces(
                    forces=force_to_apply,
                    is_global=False
                )

                print(rigid_prim.prim_paths[0])
                print(rigid_prim.get_world_poses()[0])

            for articulation, rotor_target, blade_target in zip(self.articulations, self.rotor_targets, self.blade_targets):
                articulation.set_joint_position_targets(
                    positions=rotor_target,
                    joint_names=articulation.joint_names[:6]
                )
                
                articulation.set_joint_velocity_targets(
                    velocities=blade_target,
                    joint_names=articulation.joint_names[6:]
                )

    def on_timeline_stop(self, event) -> None:
        if self.is_simulation_running:
            self.is_simulation_running = False

            self.rigid_prims = []
            self.forces_to_apply = []

            self.articulations = []
            self.rotor_targets = []
            self.blade_targets = []

    def on_timeline_play(self, event) -> None:
        if not self.is_simulation_running:
            for path in self.rigid_prim_paths:
                rigid_prim = RigidPrim(prim_paths_expr=path)
                rigid_prim.initialize()
                self.rigid_prims.append(rigid_prim)

                forces_to_apply = np.zeros((rigid_prim.count, 3), dtype=np.float32)
                forces_to_apply[:, 2] = self.hover_force / rigid_prim.count

                if "blades4" in path or "jovi" in path:
                    forces_to_apply[:, 2] -= self.hover_force / rigid_prim.count

                self.forces_to_apply.append(forces_to_apply)

            for path in self.articulation_paths:
                articulation = Articulation(prim_paths_expr=path)
                articulation.initialize()
                self.articulations.append(articulation)

                self.rotor_targets.append(np.zeros(6))
                self.blade_targets.append(np.zeros(6))

            self.is_simulation_running = True
    
    def init_vars(self) -> None:
        self.rigid_prim_paths = [
            # "/World/blades0/body", 
            # "/World/blades4/body", 
            # "/World/blades4_1rb",
            # "/World/jovi/body",
            "/World/jovi/blade*",
        ]
        self.rigid_prims = []
        self.forces_to_apply = []

        self.articulation_paths = [
            "/World/jovi",
        ]
        self.articulations = []
        self.rotor_targets = []
        self.blade_targets = []

        self.is_simulation_running = False
        self.gravity = 9.81
        self.total_mass = 2002
        self.hover_force = self.total_mass * self.gravity

        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(
            fn=self.on_physics_step,
            pre_step=True,
            order=0
        )

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

    def build_ui(self) -> None:
        self.window = ui.Window(
            "Prueba Fuerzas", 
            width=300, 
            height=300,
        )

        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(height=0):
                    with ui.HStack():
                        ui.Label("Rigid Prim")
                        ui.Label("Articulation Root")

                    with ui.HStack():
                        with ui.VStack():
                            ui.Button(
                                text="Upward",
                                clicked_fn=self.set_upward_force
                            )
                            ui.Button(
                                text="Forward",
                                clicked_fn=self.set_forward_force
                            )
                        
                        with ui.VStack():
                            with ui.VStack():
                                ui.Label("Rotors angle (0-90)")
                                self.rotors_angle_field = ui.IntField()
                                ui.Button(
                                    text="Apply",
                                    clicked_fn=self.set_rotors_angle
                                )
                            with ui.VStack():
                                ui.Label("Thrust")
                                self.blades_thrust_field = ui.IntField()
                                ui.Button(
                                    text="Apply",
                                    clicked_fn=self.set_thrust
                                )

    def set_upward_force(self) -> None:
        for force, path in zip(self.forces_to_apply, self.rigid_prim_paths):
            force[:, 2] = (self.hover_force / force.shape[0]) * 1.1
            
            if "blades4" in path or "jovi" in path:
                force[:, 2] -= self.hover_force / force.shape[0]

    def set_forward_force(self) -> None:
        for force in self.forces_to_apply:
            force[:, 0] = 100

    def set_rotors_angle(self) -> None:
        for rotor_target in self.rotor_targets:
            value = self.rotors_angle_field.model.get_value_as_int()
            rotor_target[:] = np.deg2rad(value)

    def set_thrust(self) -> None:
        for blade_target in self.blade_targets:
            value = self.blades_thrust_field.model.get_value_as_int()
            blade_target[[0,3,4]] = np.deg2rad(value)
            blade_target[[1,2,5]] = np.deg2rad(-value)
