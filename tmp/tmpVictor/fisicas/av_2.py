import numpy as np
import omni.physx


from omni.kit.scripting import BehaviorScript
from omni.isaac.core.prims import RigidPrimView
import omni.physics.tensors as omni_tensors
from scipy.spatial.transform import Rotation


class FuerzasAv(BehaviorScript):
    def on_init(self):
        self.physx_interface = omni.physx.get_physx_interface()
        self.physx_interface_sub = self.physx_interface.subscribe_physics_on_step_events(self.on_physics_step, True, 0)

        self.view_created = False
        self.delta_time = 0.02
        self.inidividual_pos = np.array([
            [0, 0, 0],
            [0.5, 1.95, 0.5],
            [0.5, -1.95, 0.5],
            [-2.5, 1.55, 0.5],
            [-2.5, -1.55, 0.5]
        ])

    def on_destroy(self):
        self.physx_interface_sub = None

    def on_play(self):
        self.rigid_prim_view = RigidPrimView(["/World/UAVs/UAV_*"])
        
        
        if not self.view_created:
            self.view_created = True

            self.rigid_prim_view.initialize()

            self.individual_forces = np.array([
                [0, 0, 0],
                [0, 0, 8170],
                [0, 0, 8170],
                [0, 0, 1634],
                [0, 0, 1634]
            ])
            self.individual_torque = np.zeros(3)
            # self.forces = np.zeros(3)
            self.forces = np.array([0, 0, 19900])
            self.torques = np.array([100, 0, 0])

    def on_pause(self):
        pass

    def on_stop(self):
        self.view_created = False

    def on_update(self, current_time: float, delta_time: float):
        self.current_time = current_time
        self.delta_time = delta_time

        if self.current_time >= 5:
            self.individual_forces[1, 2] = 8400
            self.individual_forces[2, 2] = 8400
            self.individual_forces[3, 2] = 1680
            self.individual_forces[4, 2] = 1680

        if self.current_time >= 10:
            self.individual_forces[1, 2] = 8200
            self.individual_forces[2, 2] = 8200
            self.individual_forces[3, 2] = 1680
            self.individual_forces[4, 2] = 1680

        # if self.current_time >= 15:
        #     self.individual_forces[1, 2] = 8000
        #     self.individual_forces[2, 2] = 8300
        #     self.individual_forces[3, 2] = 1800
        #     self.individual_forces[4, 2] = 1400
        #     self.individual_torque[2] = 1000

    def on_physics_step(self, dt):
        if self.view_created:
            # self.forces = np.sum(self.individual_forces, axis=0)
            # self.torques = self.compute_torques(self.individual_forces, self.inidividual_pos)
            # self.torques += self.individual_torque

            self.pos, _ = self.rigid_prim_view.get_world_poses()
            _, self.ori = self.rigid_prim_view.get_local_poses()
            self.pos = self.pos[0]
            self.ori = self.ori[0]
            self.rot = Rotation.from_quat([self.ori[1], self.ori[2], self.ori[3], self.ori[0]])
            self.roll, self.pitch, self.yaw = self.rot.as_euler('xyz', degrees=False)
            self.linear_vel = self.rigid_prim_view.get_linear_velocities()[0]
            self.angular_vel = self.rigid_prim_view.get_angular_velocities()[0]

            self.linear_vel = self.rot.inv().apply(self.linear_vel)
            self.angular_vel = self.rot.inv().apply(self.angular_vel)

            print()
            print(f"[{self.current_time}] - pos: {np.round(self.pos, 2)}")
            # print(f"ori: {np.round(self.ori, 2)}")
            print(f"[{self.current_time}] - roll: {np.round(self.roll, 2)}")
            print(f"[{self.current_time}] - pitch: {np.round(self.pitch, 2)}")
            print(f"[{self.current_time}] - yaw: {np.round(self.yaw, 2)}")
            print(f"[{self.current_time}] - linear_vel: {np.round(self.linear_vel, 2)}")
            print(f"[{self.current_time}] - angular_vel: {np.round(self.angular_vel, 2)}")

            self.rigid_prim_view.apply_forces_and_torques_at_pos(
                forces=self.forces,
                torques=self.torques,
                is_global=False
            )


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

   