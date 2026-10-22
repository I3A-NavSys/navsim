import numpy as np
import carb
try:
    from isaacsim.core.prims import RigidPrim
except ImportError:
    try:
        from isaacsim.core.experimental.prims import RigidPrim
    except ImportError:
        RigidPrim = object


# Precomputed Taylor series inverse factorials for trajectory evaluation
INV_FACTORIAL_3 = 1.0 / 6.0
INV_FACTORIAL_4 = 1.0 / 24.0
INV_FACTORIAL_5 = 1.0 / 120.0


class UAVControl:
    """Controls a fleet of multirotor UAVs using physics-based aerodynamics and modern state-space control."""

    def __init__(self, rigid_prim_view: RigidPrim):
        self.rigid_prim_view = rigid_prim_view
        self.amount_uavs = self.rigid_prim_view.count
        self.rigid_prim_uav_indices = np.arange(self.amount_uavs)

        # UAV telemetry and kinematics
        self.positions = None
        self.orientations = None
        self.eulers = None
        self.world_linear_vels = None
        self.world_angular_vels = None
        self.local_linear_vels = None
        self.local_angular_vels = None

        # Multirotor limits and operating points
        self.w_max = 62.8319
        self.w_min = 0.0
        self.w_hov = 41.8879

        self.rotors_vel = np.zeros((self.amount_uavs, 4), dtype=np.float32)

        # Aerodynamic parameters
        self.thrust_coeffs = np.array([4.6544, 4.6544, 0.9309, 0.9309], dtype=np.float32)
        self.drag_force_coeffs = -np.array([3.0625, 4.0000, 7.8400], dtype=np.float32)
        self.drag_moment_coeffs = np.array([5.9683, 5.9683, 1.4921, 1.4921], dtype=np.float32)
        self.friction_moment_coeffs = -np.array([37.4010, 25.8580, 20.2514], dtype=np.float32)

        # State-space control gain matrices
        self.Kx = np.array([
            [-14.6551, -45.5032, -4.1872, -13.0009, 3.8871, -6.6331, 2.1363, 6.4114],
            [14.6551, -45.5032, 4.1872, -13.0009, -3.8871, -6.6331, -2.1363, 6.4114],
            [-58.6206, 227.5161, -16.7487, 65.0046, -24.4514, 33.1656, 8.5453, 6.4114],
            [58.6206, 227.5161, 16.7487, 65.0046, 24.4514, 33.1656, -8.5453, 6.4114],
        ], dtype=np.float32)

        self.Ky = np.array([
            [-3.1839, 1.0254, 4.2743, 2.5914],
            [-3.1839, -1.0254, 4.2743, -2.5914],
            [15.9195, 4.1017, 4.2743, -16.3009],
            [15.9195, -4.1017, 4.2743, 16.3009],
        ], dtype=np.float32)

        self.Hs = np.full((4, 1), self.w_hov, dtype=np.float32)
        self.r = np.zeros((self.amount_uavs, 4, 1), dtype=np.float32)
        self.x = np.zeros((self.amount_uavs, 8, 1), dtype=np.float32)
        self.y = np.zeros((self.amount_uavs, 4, 1), dtype=np.float32)
        self.e = np.zeros((self.amount_uavs, 4, 1), dtype=np.float32)
        self.E = np.zeros((self.amount_uavs, 4, 1), dtype=np.float32)
        self.u = np.zeros((self.amount_uavs, 4, 1), dtype=np.float32)
        self.E_max = 150.0

        # Rotor positions relative to body frame [x, y, z]
        self.bodies_positions = np.array([
            [0.0, 0.0, 0.0],
            [0.5, 1.95, 0.5],
            [0.5, -1.95, 0.5],
            [-2.5, 1.55, 0.5],
            [-2.5, -1.55, 0.5],
        ], dtype=np.float32)

        # Precompute rotor lever arms for analytical torque computation (eliminating np.cross and 3D tensors)
        self.rotor_rx = self.bodies_positions[1:, 0]
        self.rotor_ry = self.bodies_positions[1:, 1]

        self.forces_to_apply = np.zeros((self.amount_uavs, 3), dtype=np.float32)
        self.torques_to_apply = np.zeros((self.amount_uavs, 3), dtype=np.float32)

    # -----------------------
    # -- Control Functions --
    # -----------------------
    def imu(self):
        """
        Reads UAV state from the physics simulation view without unnecessary cloning.
        Queries transforms and combined velocities in single operations.
        """
        self.positions, self.orientations = self.rigid_prim_view.get_world_poses(clone=False)
        self.eulers = self.quaternion_to_euler(self.orientations)

        # Fetch both linear and angular velocities in a single physics view query
        velocities = self.rigid_prim_view.get_velocities(clone=False)
        self.world_linear_vels = velocities[:, :3]
        self.world_angular_vels = velocities[:, 3:6]

        self.local_linear_vels = self.global_to_local_velocity(
            self.world_linear_vels,
            self.orientations,
        )
        self.local_angular_vels = self.global_to_local_velocity(
            self.world_angular_vels,
            self.orientations,
        )

    def navigation(
        self,
        uav_physics_indices,
        current_time,
        fp_times,
        fp_positions,
        fp_velocities,
        fp_accelerations,
        fp_jerks,
        fp_snaps,
        fp_crackels,
        fp_headings,
    ):
        """Evaluates trajectory polynomials and calculates target guidance velocities."""
        target_indices = self.get_running_idx(fp_times, current_time)

        running_data = self.extract_running_data(
            fp_times,
            fp_positions,
            fp_velocities,
            fp_accelerations,
            fp_jerks,
            fp_snaps,
            fp_crackels,
            fp_headings,
            target_indices,
        )

        expected_pos, expected_vel = self.status_at_time(
            current_time,
            running_data[0],
            running_data[1],
            running_data[2],
            running_data[3],
            running_data[4],
            running_data[5],
            running_data[6],
        )

        cmd_world_linear_vel, cmd_yaw_rotation = self.get_command(
            expected_pos,
            expected_vel,
            self.positions[uav_physics_indices],
            self.world_linear_vels[uav_physics_indices],
            self.eulers[uav_physics_indices, 2],
            running_data[7],
            2.0,
        )

        cmd_local_linear_vel = self.global_to_local_velocity(
            cmd_world_linear_vel,
            self.orientations[uav_physics_indices],
        )

        return cmd_local_linear_vel, cmd_yaw_rotation

    def servo_control(self, uav_physics_indices, command_linear_vel, command_angular_vel, step_size: float):
        """Computes feedback control inputs and saturates rotor angular velocities in-place."""
        if len(uav_physics_indices) == 0:
            return

        # Reference trajectory inputs
        self.r[uav_physics_indices, 0, 0] = command_linear_vel[:, 0]
        self.r[uav_physics_indices, 1, 0] = command_linear_vel[:, 1]
        self.r[uav_physics_indices, 2, 0] = command_linear_vel[:, 2]
        self.r[uav_physics_indices, 3, 0] = command_angular_vel

        # Current state feedback: [roll, pitch, bWx, bWy, bWz, bXdot, bYdot, bZdot]
        self.x[uav_physics_indices, :2, 0] = self.eulers[uav_physics_indices, :2]
        self.x[uav_physics_indices, 2:5, 0] = self.local_angular_vels[uav_physics_indices]
        self.x[uav_physics_indices, 5:8, 0] = self.local_linear_vels[uav_physics_indices]

        # Model output
        self.y[uav_physics_indices, 0, 0] = self.x[uav_physics_indices, 5, 0]
        self.y[uav_physics_indices, 1, 0] = self.x[uav_physics_indices, 6, 0]
        self.y[uav_physics_indices, 2, 0] = self.x[uav_physics_indices, 7, 0]
        self.y[uav_physics_indices, 3, 0] = self.x[uav_physics_indices, 4, 0]

        # Output tracking error
        self.e[uav_physics_indices] = self.y[uav_physics_indices] - self.r[uav_physics_indices]

        # Integrated cumulative error updated and saturated in-place
        self.E[uav_physics_indices] += self.e[uav_physics_indices] * step_size
        np.clip(
            self.E[uav_physics_indices],
            -self.E_max,
            self.E_max,
            out=self.E[uav_physics_indices],
        )

        # Full-state feedback control law
        self.u[uav_physics_indices] = (
            self.Hs
            - (self.Kx @ self.x[uav_physics_indices])
            - (self.Ky @ self.E[uav_physics_indices])
        )

        # Saturate commanded rotor velocities in-place
        np.clip(
            self.u[uav_physics_indices],
            self.w_min,
            self.w_max,
            out=self.u[uav_physics_indices],
        )

        # Vectorized assignment of rotor speeds: u indices [1, 0, 3, 2] map to rotors [0, 1, 2, 3]
        self.rotors_vel[uav_physics_indices] = self.u[uav_physics_indices, :, 0][:, [1, 0, 3, 2]]

    def compute_dynamics(self, uav_physics_indices):
        """
        Computes aerodynamic forces and moments using analytical torque derivation.
        Eliminates intermediate 3D tensor allocations and general cross products.
        """
        if len(uav_physics_indices) == 0:
            return

        active_rotors = self.rotors_vel[uav_physics_indices]
        square_rotors_vel = active_rotors * active_rotors

        # Rotor thrusts along body Z-axis: shape (M, 4)
        thrust_z = self.thrust_coeffs * square_rotors_vel

        # Body aerodynamic drag forces with velocity bounding to prevent quadratic divergence
        active_lin_vel = np.clip(self.local_linear_vels[uav_physics_indices], -100.0, 100.0)
        fd = self.drag_force_coeffs * np.abs(active_lin_vel) * active_lin_vel

        # Rotor drag moments
        mdr_z = self.drag_moment_coeffs * square_rotors_vel
        mdr_torque_z = mdr_z[:, 1] - mdr_z[:, 0] - mdr_z[:, 3] + mdr_z[:, 2]

        # Body rotational friction moments with angular velocity bounding
        active_ang_vel = np.clip(self.local_angular_vels[uav_physics_indices], -50.0, 50.0)
        md = self.friction_moment_coeffs * np.abs(active_ang_vel) * active_ang_vel

        # Total body-frame forces: fd + sum of rotor thrusts along Z
        self.forces_to_apply[uav_physics_indices, :2] = fd[:, :2]
        self.forces_to_apply[uav_physics_indices, 2] = fd[:, 2] + np.sum(thrust_z, axis=1)

        # Analytical body torques: r_i x [0, 0, T_i] = [r_iy * T_i, -r_ix * T_i, 0]
        torque_thrust_x = np.sum(thrust_z * self.rotor_ry, axis=1)
        torque_thrust_y = -np.sum(thrust_z * self.rotor_rx, axis=1)

        self.torques_to_apply[uav_physics_indices, 0] = torque_thrust_x + md[:, 0]
        self.torques_to_apply[uav_physics_indices, 1] = torque_thrust_y + md[:, 1]
        self.torques_to_apply[uav_physics_indices, 2] = mdr_torque_z + md[:, 2]

        # Numerical safety clamps: protect against NaN/Inf and extreme physics impulses
        np.nan_to_num(self.forces_to_apply[uav_physics_indices], copy=False)
        np.nan_to_num(self.torques_to_apply[uav_physics_indices], copy=False)
        np.clip(
            self.forces_to_apply[uav_physics_indices],
            -50000.0,
            50000.0,
            out=self.forces_to_apply[uav_physics_indices],
        )
        np.clip(
            self.torques_to_apply[uav_physics_indices],
            -10000.0,
            10000.0,
            out=self.torques_to_apply[uav_physics_indices],
        )

    def apply_dynamics(self, uav_to_update_idx):
        """Applies computed forces and torques directly to the simulation rigid body view."""
        if len(uav_to_update_idx) == 0:
            return

        self.rigid_prim_view.apply_forces_and_torques_at_pos(
            forces=self.forces_to_apply[uav_to_update_idx],
            torques=self.torques_to_apply[uav_to_update_idx],
            is_global=False,
            indices=self.rigid_prim_uav_indices[uav_to_update_idx],
        )

    def reset_dynamics(self, uav_to_update_idx):
        """Zeroes forces, torques, and error integral for inactive UAVs."""
        mask = np.ones(self.amount_uavs, dtype=bool)
        mask[uav_to_update_idx] = False

        self.forces_to_apply[mask] = 0.0
        self.torques_to_apply[mask] = 0.0
        self.E[mask, :, 0] = 0.0

    def update(self, uav_physics_indices, flightplans, current_time: float, step_size: float):
        """Executes full autonomous flight trajectory loop for the specified UAVs."""
        fp_times = self.adjust_2d_array_shape(flightplans[0])
        fp_positions = self.adjust_3d_array_shape(flightplans[1])
        fp_velocities = self.adjust_3d_array_shape(flightplans[2])
        fp_accelerations = self.adjust_3d_array_shape(flightplans[3])
        fp_jerks = self.adjust_3d_array_shape(flightplans[4])
        fp_snaps = self.adjust_3d_array_shape(flightplans[5])
        fp_crackels = self.adjust_3d_array_shape(flightplans[6])
        fp_headings = self.adjust_3d_array_shape(flightplans[7])

        self.imu()

        cmd_local_linear_vel, cmd_yaw_rotation = self.navigation(
            uav_physics_indices,
            current_time,
            fp_times,
            fp_positions,
            fp_velocities,
            fp_accelerations,
            fp_jerks,
            fp_snaps,
            fp_crackels,
            fp_headings,
        )

        self.servo_control(uav_physics_indices, cmd_local_linear_vel, cmd_yaw_rotation, step_size)
        self.compute_dynamics(uav_physics_indices)
        self.apply_dynamics(uav_physics_indices)
        self.reset_dynamics(uav_physics_indices)

    def update_command(
        self,
        uav_physics_indices,
        cmd_local_linear_vel,
        cmd_yaw_rotation,
        step_size: float,
    ):
        """Executes manual teleoperation loop for active joysticks."""
        self.imu()

        if len(uav_physics_indices) > 0:
            self.servo_control(uav_physics_indices, cmd_local_linear_vel, cmd_yaw_rotation, step_size)
            self.compute_dynamics(uav_physics_indices)
            self.apply_dynamics(uav_physics_indices)

    # -------------------------
    # -- Auxiliary Functions --
    # -------------------------
    @staticmethod
    def _fast_cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Fast 3D cross-product on 2D arrays avoiding np.cross overhead."""
        return np.column_stack((
            a[:, 1] * b[:, 2] - a[:, 2] * b[:, 1],
            a[:, 2] * b[:, 0] - a[:, 0] * b[:, 2],
            a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0],
        ))

    def rotate_vector_by_quaternion(self, vector: np.ndarray, quaternion: np.ndarray, conjugate: bool) -> np.ndarray:
        """Rotates 3D vector(s) using unit quaternion with Rodrigues' formulation."""
        is_1d_array = vector.ndim == 1
        if is_1d_array:
            vector = vector[np.newaxis, :]
        if quaternion.ndim == 1:
            quaternion = quaternion[np.newaxis, :]

        w = quaternion[:, 0:1]
        u = -quaternion[:, 1:4] if conjugate else quaternion[:, 1:4]

        t = 2.0 * self._fast_cross(u, vector)
        vector_rotated = vector + w * t + self._fast_cross(u, t)

        return vector_rotated[0] if is_1d_array else vector_rotated

    def local_to_global_velocity(self, vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
        """Transforms body velocity vector to world frame."""
        return self.rotate_vector_by_quaternion(vector, quaternion, conjugate=False)

    def global_to_local_velocity(self, vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
        """Transforms world velocity vector to body frame."""
        return self.rotate_vector_by_quaternion(vector, quaternion, conjugate=True)

    def quaternion_to_euler(self, quaternion: np.ndarray) -> np.ndarray:
        """Converts quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw] in-place."""
        is_1d_array = quaternion.ndim == 1
        if is_1d_array:
            quaternion = quaternion[np.newaxis, :]

        w = quaternion[:, 0]
        x = quaternion[:, 1]
        y = quaternion[:, 2]
        z = quaternion[:, 3]

        euler = np.empty((quaternion.shape[0], 3), dtype=quaternion.dtype)

        # Roll (X-axis)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        euler[:, 0] = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (Y-axis) with in-place clipping against numerical precision overflow
        sinp = 2.0 * (w * y - z * x)
        np.clip(sinp, -1.0, 1.0, out=sinp)
        euler[:, 1] = np.arcsin(sinp)

        # Yaw (Z-axis)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        euler[:, 2] = np.arctan2(siny_cosp, cosy_cosp)

        return euler[0] if is_1d_array else euler

    def adjust_2d_array_shape(self, array: list[list[float]]) -> np.ndarray:
        """Pre-allocates and converts jagged 2D lists into a contiguous NumPy array padded with np.inf."""
        num_rows = len(array)
        max_length = max(len(row) for row in array)
        matrix = np.full((num_rows, max_length), np.inf, dtype=np.float32)

        for i, row in enumerate(array):
            matrix[i, : len(row)] = row

        return matrix

    def adjust_3d_array_shape(self, array: list[list[np.ndarray]]) -> np.ndarray:
        """Pre-allocates and converts jagged lists of coordinate vectors into a 3D NumPy array padded with np.inf."""
        num_rows = len(array)
        max_length = max(len(path) for path in array)
        num_dimensions = len(array[0][0])

        matrix = np.full((num_rows, max_length, num_dimensions), np.inf, dtype=np.float32)

        for i, path in enumerate(array):
            matrix[i, : len(path), :] = path

        return matrix

    def get_running_idx(self, array: np.ndarray, value: float) -> np.ndarray:
        """Vectorized lookup of active waypoint index across all flight paths."""
        indices = np.sum(array <= value, axis=1) - 1
        return np.clip(indices, 0, None)

    def extract_running_data(
        self,
        times: np.ndarray,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray,
        jerks: np.ndarray,
        snaps: np.ndarray,
        crackels: np.ndarray,
        headings: np.ndarray,
        target_indices: np.ndarray,
    ) -> list[np.ndarray]:
        """Extracts active trajectory setpoints using integer array indexing."""
        path_indices = np.arange(times.shape[0])

        return [
            times[path_indices, target_indices],
            positions[path_indices, target_indices],
            velocities[path_indices, target_indices],
            accelerations[path_indices, target_indices],
            jerks[path_indices, target_indices],
            snaps[path_indices, target_indices],
            crackels[path_indices, target_indices],
            headings[path_indices, target_indices],
        ]

    def status_at_time(
        self,
        current_time: float,
        running_times: np.ndarray,
        running_pos: np.ndarray,
        running_vel: np.ndarray,
        running_accel: np.ndarray,
        running_jerk: np.ndarray,
        running_snap: np.ndarray,
        running_crackel: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Computes current position and velocity along minimum-snap trajectory polynomials."""
        dt = np.clip(current_time - running_times, 0.0, None)[:, np.newaxis]

        dt_2 = dt * dt
        dt_3 = dt_2 * dt
        dt_4 = dt_3 * dt
        dt_5 = dt_4 * dt

        pos = (
            running_pos
            + running_vel * dt
            + 0.5 * running_accel * dt_2
            + INV_FACTORIAL_3 * running_jerk * dt_3
            + INV_FACTORIAL_4 * running_snap * dt_4
            + INV_FACTORIAL_5 * running_crackel * dt_5
        )

        vel = (
            running_vel
            + running_accel * dt
            + 0.5 * running_jerk * dt_2
            + INV_FACTORIAL_3 * running_snap * dt_3
            + INV_FACTORIAL_4 * running_crackel * dt_4
        )

        return pos, vel

    def get_command(
        self,
        expected_pos: np.ndarray,
        expected_vel: np.ndarray,
        current_pos: np.ndarray,
        current_lin_vel: np.ndarray,
        current_yaw: np.ndarray,
        headings: np.ndarray,
        t_to_solve: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Calculates guidance linear velocity and yaw rotation command."""
        correction_vel = (expected_pos - current_pos) / t_to_solve
        command_linear_vel = expected_vel + correction_vel

        # Work on a copy of headings to avoid mutating input trajectory array
        headings_copy = np.copy(headings)
        zero_mask = (headings_copy[:, 0] == 0.0) & (headings_copy[:, 1] == 0.0)
        headings_copy[zero_mask] = expected_vel[zero_mask, :2]

        target_yaw = np.arctan2(headings_copy[:, 1], headings_copy[:, 0])
        target_yaw[zero_mask] = current_yaw[zero_mask]

        yaw_error = target_yaw - current_yaw
        yaw_error = (yaw_error + np.pi) % (2.0 * np.pi) - np.pi
        command_yaw_rotation = yaw_error / t_to_solve

        return command_linear_vel, command_yaw_rotation

    def reset_pose(self, uav_indices: list[int]):
        """Resets pose, velocity, dynamic states, and forces for the requested UAVs."""
        uav_amount = len(uav_indices)
        if uav_amount == 0:
            return

        # Reset controller internal states
        self.r[uav_indices] = 0.0
        self.x[uav_indices] = 0.0
        self.y[uav_indices] = 0.0
        self.e[uav_indices] = 0.0
        self.E[uav_indices] = 0.0
        self.u[uav_indices] = 0.0
        self.rotors_vel[uav_indices] = 0.0
        self.forces_to_apply[uav_indices] = 0.0
        self.torques_to_apply[uav_indices] = 0.0

        current_positions, _ = self.rigid_prim_view.get_world_poses(
            indices=uav_indices,
            clone=False,
            usd=False,
        )

        new_orientations = np.zeros((uav_amount, 4), dtype=np.float32)
        new_orientations[:, 0] = 1.0  # Unit quaternion identity [1, 0, 0, 0]
        new_velocities = np.zeros((uav_amount, 6), dtype=np.float32)

        carb.log_info(f"[UAVControl] Resetting UAV pose for indices: {uav_indices}")

        self.rigid_prim_view.set_world_poses(
            positions=current_positions,
            orientations=new_orientations,
            indices=uav_indices,
            usd=False,
        )

        self.rigid_prim_view.set_velocities(
            velocities=new_velocities,
            indices=uav_indices,
        )
