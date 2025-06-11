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
        self.rpv_initialized = False

        self.individual_positions = np.array([[0,0,0], [0.5,0.5,0.5], [0.5,-0.5,0.5], [-0.5,0.5,0.5], [-0.5,-0.5,0.5]])

    def on_destroy(self):
        self.physx_interface_sub = None

    def on_play(self):
        self.individual_forces = np.array([[1000,0,0], [0,0,5000], [0,0,5000], [0,0,5000], [0,0,5000]])
        self.forces = np.array([[0,0,0]])
        self.torques = np.array([[0,0,0]])
        self.rigid_prim_view = RigidPrimView([str(self.prim_path)])
        
        if not self.rpv_initialized:
            self.rigid_prim_view.initialize()
            self.rpv_initialized = True

    def on_pause(self):
        pass

    def on_stop(self):
        self.rpv_initialized = False

    def on_update(self, current_time: float, delta_time: float):
        # if current_time >= 5.0:
        #     self.individual_forces = np.array([[0,0,0], [0,0,2.6], [0,0,2.6], [0,0,2.6], [0,0,2.6]])

        # if current_time >= 10.0:
        #     self.individual_forces = np.array([[1,0,0], [0,0,2.5], [0,0,2.5], [0,0,2.5], [0,0,2.5]])
        pass

    def on_physics_step(self, dt):
        if self.rpv_initialized:
            self.forces = np.sum(self.individual_forces, axis=0)
            self.torques = self.compute_torques(self.individual_forces, self.individual_positions)
            self.rigid_prim_view.apply_forces_and_torques_at_pos(forces=self.forces, torques=self.torques, is_global=False)

    def compute_torques(self, forces, positions):
        """
        Compute torques about each axis (X, Y, Z) given forces applied at offset positions.
        
        Parameters:
        - forces: List of force vectors [Fx, Fy, Fz] in Newtons
        - positions: List of position vectors [x, y, z] in meters relative to COM
        
        Returns:
        - Total torque vector [τx, τy, τz] in Newton-meters
        """
        total_torque = np.zeros(3)
        
        for force, position in zip(forces, positions):
            # Convert to numpy arrays for vector operations
            # f = np.array(force)
            # r = np.array(position)
            
            # Compute cross product (r × F)
            # torque = np.cross(r, f)
            torque = np.cross(position, force)
            
            # Add to total torque
            total_torque += torque
        
        return total_torque
