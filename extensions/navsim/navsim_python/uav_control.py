import numpy as np
from scipy.spatial.transform import Rotation

from omni.isaac.core.prims import RigidPrimView

class UAVControl:
    def __init__(self, rigid_prim_view: RigidPrimView, uav_ids_to_physics_buffer: dict):
        # Runtime variables
        self.rigid_prim_view = rigid_prim_view
        self.uav_ids_to_physics_buffer = uav_ids_to_physics_buffer
        self.amount_uavs = self.rigid_prim_view.count

        # UAVs parameters
        self.positions = None
        self.orientations = None
        self.eulers = None
        self.global_linear_vels = None
        self.global_angular_vels = None
        self.local_linear_vels = None
        self.local_angular_vels = None

        # UAV classic modern control
        self.w_max = 62.8319
        self.w_min = 0.0
        self.w_hov = 41.8879

        self.rotors_vel = np.zeros((4))

        self.thrust_coeffs = np.array([4.6544, 4.6544, 0.9309, 0.9309])
        self.drag_force_coeffs = -np.array([3.0625, 4.0000, 7.8400])
        self.drag_moment_coeffs = np.array([5.9683, 5.9683, 1.4921, 1.4921])
        self.friction_moment_coeffs = -np.array([37.4010, 25.8580, 20.2514])
        
        self.Kx = np.array([
            [-14.6551, -45.5032, -4.1872, -13.0009, 3.8871, -6.6331, 2.1363, 6.4114],
            [14.6551, -45.5032, 4.1872, -13.0009, -3.8871, -6.6331, -2.1363, 6.4114],
            [-58.6206, 227.5161, -16.7487, 65.0046, -24.4514, 33.1656, 8.5453, 6.4114],
            [58.6206, 227.5161, 16.7487, 65.0046, 24.4514, 33.1656, -8.5453, 6.4114]
        ])
        self.Ky = np.array([
            [-3.1839, 1.0254, 4.2743, 2.5914],
            [-3.1839, -1.0254, 4.2743, -2.5914],
            [15.9195, 4.1017, 4.2743, -16.3009],
            [15.9195, -4.1017, 4.2743, 16.3009]
        ])

        self.Hs = np.full((4, 1), self.w_hov)
        self.r = np.zeros((4, 1))
        self.x = np.zeros((8, 1))
        self.y = np.zeros((4, 1))
        self.e = np.zeros((4, 1))
        self.E = np.zeros((self.amount_uavs, 4, 1))
        self.u = np.zeros((4, 1))
        self.E_max = 150

        # Physics
        self.bodies_positions = np.array([
            [0, 0, 0], [0.5, 1.95, 0.5], [0.5, -1.95, 0.5],
            [-2.5, 1.55, 0.5], [-2.5, -1.55, 0.5]
        ])
        self.forces_to_apply = np.zeros((self.rigid_prim_view.count, 3))
        self.torques_to_apply = np.zeros((self.rigid_prim_view.count, 3))

    # -----------------------
    # -- Control Functions --
    # -----------------------
    def imu(self):
        self.positions, self.orientations = self.rigid_prim_view.get_world_poses()
        self.eulers = self.quaternion_to_euler(self.orientations)
        self.world_linear_vels = self.rigid_prim_view.get_linear_velocities(clone=True)
        self.world_angular_vels = self.rigid_prim_view.get_angular_velocities(clone=True)
        self.local_linear_vels = self.global_to_local_velocity(
            self.world_linear_vels, 
            self.orientations
        )
        self.local_angular_vels = self.global_to_local_velocity(
            self.world_angular_vels, 
            self.orientations
        )
    
    def servo_control(self, command_linear_vel, command_angular_vel, uav_idx, step_size):
        # Assign the model reference to be followed
        self.r[0, 0] = command_linear_vel[0]  # bXdot
        self.r[1, 0] = command_linear_vel[1]  # bYdot
        self.r[2, 0] = command_linear_vel[2]  # bZdot
        self.r[3, 0] = command_angular_vel     # hZdot

        # Assign model state
        self.x[0, 0] = self.eulers[uav_idx, 0]
        self.x[1, 0] = self.eulers[uav_idx, 1]
        self.x[2:5, 0] = self.local_angular_vels[uav_idx] # bWx, bWy, bWz
        self.x[5:8, 0] = self.local_linear_vels[uav_idx]  # bXdot, bYdot, bZdot

        # Assign model output
        self.y[0, 0] = self.x[5, 0]        # bXdot
        self.y[1, 0] = self.x[6, 0]        # bYdot
        self.y[2, 0] = self.x[7, 0]        # bZdot
        self.y[3, 0] = self.x[4, 0]        # bWz

        # Error between the output and the reference 
        self.e = self.y - self.r

        # Cumulative error
        self.E[uav_idx] += self.e * step_size
        self.E[uav_idx] = np.clip(self.E[uav_idx], -self.E_max, self.E_max)

        # Dynamic system control
        self.u = self.Hs - np.dot(self.Kx, self.x) - np.dot(self.Ky, self.E[uav_idx])

        # Rotor speed saturation
        self.u = np.clip(self.u, self.w_min, self.w_max)

        # Assign rotor speed
        self.rotors_vel[0] = self.u[1, 0]
        self.rotors_vel[1] = self.u[0, 0]
        self.rotors_vel[2] = self.u[3, 0]
        self.rotors_vel[3] = self.u[2, 0]

    def compute_dynamics(self, uav_idx):
        square_rotors_vel = self.rotors_vel**2

        # Thrust forces
        thrust_z = self.thrust_coeffs * square_rotors_vel
        
        # Drag forces
        fd = (
            self.drag_force_coeffs * 
            np.abs(self.local_linear_vels[uav_idx]) * 
            self.local_linear_vels[uav_idx]
        )
        
        # Drag moments
        mdr_z = self.drag_moment_coeffs * square_rotors_vel
        mdr = np.zeros((3))
        mdr[2] = mdr_z[1] - mdr_z[0] - mdr_z[3] + mdr_z[2]
        
        # Friction moments
        md = (
            self.friction_moment_coeffs * 
            np.abs(self.local_angular_vels[uav_idx]) * 
            self.local_angular_vels[uav_idx]
        )
        
        # Force and torque
        bodies_forces = np.zeros((5, 3))
        bodies_forces[0] = fd
        bodies_forces[1:, 2] = thrust_z
        
        self.forces_to_apply[uav_idx] = np.sum(bodies_forces, axis=0)
        
        torques = np.sum(np.cross(self.bodies_positions, bodies_forces), axis=0)
        
        self.torques_to_apply[uav_idx] = mdr + md + torques

    def apply_dynamics(self):
        self.rigid_prim_view.apply_forces_and_torques_at_pos(
            forces=self.forces_to_apply,
            torques=self.torques_to_apply,
            is_global=False
        )
    
    def update(self, flightplans, current_time, step_size):
        self.imu()

        for operator_id, uav_id, flightplan in flightplans:
            uav_idx = self.uav_ids_to_physics_buffer[operator_id][uav_id]
            current_pos = self.positions[uav_idx]
            current_world_linear_vel = self.world_linear_vels[uav_idx]
            current_yaw = self.eulers[uav_idx, 2]
            current_waypoint_heading = flightplan.get_running_waypoint(current_time).heading

            cmd_world_linear_vel, cmd_yaw_rotation = flightplan.get_isaacsim_command(
                current_time,
                current_pos,
                current_world_linear_vel,
                current_yaw,
                current_waypoint_heading,
                2
            )

            cmd_local_linear_vel = self.global_to_local_velocity(
                cmd_world_linear_vel,
                self.orientations[uav_idx]
            )

            # print(f"POS: {current_pos}")
            # print(f"CURRENT WORLD LINEAR VEL: {current_world_linear_vel}")
            # print(f"CURRENT YAW: {current_yaw}")
            # print(f"CMD WORLD LINEAR VEL: {cmd_world_linear_vel}")
            # print(f"CMD YAW ROTATION: {cmd_yaw_rotation}")
            # print(f"CMD LOCAL LINEAR VEL: {cmd_local_linear_vel}")
            # print("----")

            self.servo_control(cmd_local_linear_vel, cmd_yaw_rotation, uav_idx, step_size)
            self.compute_dynamics(uav_idx)

        self.apply_dynamics()

    # -------------------------
    # -- Auxiliary Functions --
    # -------------------------
    def rotate_vector_by_quaternion(self, vector, quaternion, conjugate):
        # Ensure working with a 2D array
        is_1d_array = False

        if len(vector.shape) == 1:
            vector = vector[np.newaxis, :]
            is_1d_array = True

        if len(quaternion.shape) == 1:
            quaternion = quaternion[np.newaxis, :]

        # Extract scalar (w) and vector (u) parts from the quaternion [W, X, Y, Z]
        w = quaternion[:, 0:1]
        if conjugate:
            u = -quaternion[:, 1:4]
        else:
            u = quaternion[:, 1:4]

        # Apply the vector rotation formula
        t = 2.0 * np.cross(u, vector)
        vector_rotated = vector + w * t + np.cross(u, t)

        if is_1d_array:
            vector_rotated = vector_rotated[0]

        return np.round(vector_rotated, decimals=4)

    def local_to_global_velocity(self, vector, quaternion):
        return self.rotate_vector_by_quaternion(vector, quaternion, conjugate=False)

    def global_to_local_velocity(self, vector, quaternion):
        return self.rotate_vector_by_quaternion(vector, quaternion, conjugate=True)

    def quaternion_to_euler(self, quaternion):
        # Ensure it's a 2D array even if a single quaternion is passed
        is_1d_array = False

        if len(quaternion.shape) == 1:
            quaternion = quaternion[np.newaxis, :]
            is_1d_array = True

        # Extract individual components
        w, x, y, z = quaternion[:, 0], quaternion[:, 1], quaternion[:, 2], quaternion[:, 3]
        
        # 1. Roll (X-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x**2 + y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # 2. Pitch (Y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        # Clip to avoid NaNs caused by floating point inaccuracies (e.g., 1.00000000001)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = np.arcsin(sinp)
        
        # 3. Yaw (Z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # Combine back into a single array (N, 3)
        euler = np.column_stack((roll, pitch, yaw))

        if is_1d_array:
            euler = euler[0]

        return euler

