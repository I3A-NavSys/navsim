import io
import os
import numpy as np
import torch

try:
    import carb
except ImportError:
    class _MockCarb:
        @staticmethod
        def log_warn(msg: str):
            pass
        @staticmethod
        def log_info(msg: str):
            pass
    carb = _MockCarb()

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
    """Controls a fleet of multirotor UAVs using physics-based aerodynamics and an AI neural network policy."""

    def __init__(
        self,
        rigid_prim_view: RigidPrim,
        uav_ids_to_physics_buffer: dict | None = None,
        policy_path: str | None = None,
        policy_actions_scale: float = 10.0,
        torch_device: str | None = None,
    ):
        self.rigid_prim_view = rigid_prim_view
        self.uav_ids_to_physics_buffer = uav_ids_to_physics_buffer
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
        self.thrust_scale = 0.5
        self.thrust_coeffs = np.array([4.6544, 4.6544, 0.9309, 0.9309], dtype=np.float32)
        self.drag_force_coeffs = -np.array([3.0625, 4.0000, 7.8400], dtype=np.float32)
        self.drag_moment_coeffs = np.array([5.9683, 5.9683, 1.4921, 1.4921], dtype=np.float32)
        self.drag_moment_coeffs_u = np.array([5.9683, -5.9683, 1.4921, -1.4921], dtype=np.float32)
        self.friction_moment_coeffs = -np.array([37.4010, 25.8580, 20.2514], dtype=np.float32)

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

        # ---------------------------
        # -- AI Policy Configuration -
        # ---------------------------
        if torch_device is not None:
            self.torch_device = torch_device
        else:
            self.torch_device = "cuda:0" if torch.cuda.is_available() else "cpu"

        if policy_path is not None:
            self.policy_path = policy_path
        else:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            # self.policy_path = os.path.join(current_dir, "policy.pt")
            self.policy_path = os.path.join(current_dir, "policy2.pt")

        self.policy_actions_scale = policy_actions_scale
        self.policy = None

        if os.path.exists(self.policy_path):
            try:
                with open(self.policy_path, "rb") as f:
                    file_bytes = io.BytesIO(f.read())
                self.policy = torch.jit.load(file_bytes, map_location=self.torch_device).eval()
                carb.log_info(
                    f"[UAVControl IA] Policy successfully loaded from {self.policy_path} on {self.torch_device}."
                )
            except Exception as e:
                carb.log_warn(f"[UAVControl IA] Failed to load TorchScript policy from {self.policy_path}: {e}")
        else:
            carb.log_warn(f"[UAVControl IA] Policy file not found at {self.policy_path}.")

        self.omega_hover = self.compute_omega_hover()
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

        cmd_world_linear_vel, yaw_error = self.get_command(
            expected_pos,
            expected_vel,
            self.positions[uav_physics_indices],
            self.eulers[uav_physics_indices, 2],
            running_data[7],
            2.0,
        )

        cmd_local_linear_vel = self.global_to_local_velocity(
            cmd_world_linear_vel,
            self.orientations[uav_physics_indices],
        )

        lookahead_pos = expected_pos + 0.65 * expected_vel

        return cmd_local_linear_vel, yaw_error, lookahead_pos

    def servo_control(self, uav_physics_indices, cmd_local_linear_vel, yaw_error, lookahead_pos):
        """
        Computes rotor angular velocities using an AI TorchScript policy model.
        Prepares a 16-element observation batch from telemetry and navigation commands.
        """
        num_uavs = len(uav_physics_indices)
        if num_uavs == 0:
            return

        if self.policy is not None:
            # Linear and angular velocity tracking errors
            # lin_vel_errors = local_lin_vels_active - commands_batch[:, :3]
            # ang_vel_errors = local_ang_vels_active[:, 2] - commands_batch[:, 3]

            # Construct 16-element observation batch:
            # [0:3]     local_lin_vel (3)
            # [3:6]     local_ang_vel (3)
            # [6:9]     projected_gravity (3)
            # [9:12]    target_vel (3)
            # [12:15]   target_future (3)
            # [15]      yaw_error (1)
            observations_batch = np.empty((num_uavs, 16), dtype=np.float32)
            
            observations_batch[:, 0:3] = self.local_linear_vels[uav_physics_indices]
            observations_batch[:, 3:6] = self.local_angular_vels[uav_physics_indices]
            observations_batch[:, 6:9] = self.global_to_local_velocity(
                np.array([0.0, 0.0, -1.0], dtype=np.float32),
                self.orientations[uav_physics_indices],
            )
            observations_batch[:, 9:12] = cmd_local_linear_vel
            observations_batch[:, 12:15] = self.global_to_local_velocity(
                lookahead_pos - self.positions[uav_physics_indices],
                self.orientations[uav_physics_indices],
            )
            observations_batch[:, 15] = yaw_error

            # Forward pass through AI policy
            obs_tensor = torch.as_tensor(observations_batch, device=self.torch_device)
            with torch.no_grad():
                raw_actions = self.policy(obs_tensor)

            # Process actions
            actions = torch.nan_to_num(torch.clamp(raw_actions, -1.0, 1.0), nan=0.0)
            rotors_vel_batch = torch.clamp(self.omega_hover * (1.0 + self.thrust_scale * actions), min=0.0).cpu().numpy()

            # Saturate commanded rotor velocities in-place [w_min, w_max]
            np.clip(
                rotors_vel_batch,
                self.w_min,
                self.w_max,
                out=rotors_vel_batch,
            )

            # Assign rotor velocities: model output [NW, NE, SW, SE] maps to rotors [0, 1, 2, 3]
            self.rotors_vel[uav_physics_indices] = rotors_vel_batch
        else:
            # Fallback when policy model is not available
            self.rotors_vel[uav_physics_indices] = self.w_hov

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

        # Body aerodynamic drag forces with realistic velocity bounding to prevent quadratic feedback loop
        active_lin_vel = np.clip(self.local_linear_vels[uav_physics_indices], -25.0, 25.0)
        fd = self.drag_force_coeffs * np.abs(active_lin_vel) * active_lin_vel

        # Rotor drag moments
        mdr_z = self.drag_moment_coeffs * square_rotors_vel
        mdr_torque_z = mdr_z[:, 1] - mdr_z[:, 0] - mdr_z[:, 3] + mdr_z[:, 2]

        # Body rotational friction moments with realistic angular velocity bounding
        active_ang_vel = np.clip(self.local_angular_vels[uav_physics_indices], -20.0, 20.0)
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

        # Numerical safety clamps: protect against NaN/Inf and extreme physics impulse spikes
        np.nan_to_num(self.forces_to_apply[uav_physics_indices], copy=False)
        np.nan_to_num(self.torques_to_apply[uav_physics_indices], copy=False)
        np.clip(
            self.forces_to_apply[uav_physics_indices, :2],
            -10000.0,
            10000.0,
            out=self.forces_to_apply[uav_physics_indices, :2],
        )
        np.clip(
            self.forces_to_apply[uav_physics_indices, 2],
            -5000.0,
            45000.0,
            out=self.forces_to_apply[uav_physics_indices, 2],
        )
        np.clip(
            self.torques_to_apply[uav_physics_indices],
            -5000.0,
            5000.0,
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
        """Zeroes forces and torques for inactive UAVs."""
        mask = np.ones(self.amount_uavs, dtype=bool)
        mask[uav_to_update_idx] = False

        self.forces_to_apply[mask] = 0.0
        self.torques_to_apply[mask] = 0.0

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

        cmd_local_linear_vel, yaw_error, lookahead_pos = self.navigation(
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

        self.servo_control(uav_physics_indices, cmd_local_linear_vel, yaw_error, lookahead_pos)
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

        self.reset_dynamics(uav_physics_indices)

    # -------------------------
    # -- Auxiliary Functions --
    # -------------------------
    def compute_omega_hover(self):
        mass = 2000
        mg = mass * 9.81

        # Construir matriz A para cada env
        A = torch.zeros(self.amount_uavs, 4, 4, device=self.torch_device)

        # Fuerza total
        A[:, 0, :] = torch.as_tensor(self.thrust_coeffs, device=self.torch_device)

        # Momento X (roll)
        A[:, 1, :] = torch.as_tensor(self.rotor_ry, device=self.torch_device) * torch.as_tensor(self.thrust_coeffs, device=self.torch_device)

        # Momento Y (pitch)
        A[:, 2, :] = -torch.as_tensor(self.rotor_rx, device=self.torch_device) * torch.as_tensor(self.thrust_coeffs, device=self.torch_device)

        # Momento Z (yaw)
        A[:, 3, :] = torch.as_tensor(self.drag_moment_coeffs_u, device=self.torch_device)

        # Vector objetivo
        b = torch.zeros(self.amount_uavs, 4, device=self.torch_device)
        b[:, 0] = mg

        # Resolver sistema
        u_hover = torch.linalg.solve(A, b)

        # Asegurar positivo
        u_hover = torch.clamp(u_hover, min=0.0)

        return torch.sqrt(u_hover)

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
        headings_copy[zero_mask] = command_linear_vel[zero_mask, :2]
        zero_mask = (headings_copy[:, 0] == 0.0) & (headings_copy[:, 1] == 0.0)

        target_yaw = np.arctan2(headings_copy[:, 1], headings_copy[:, 0])
        target_yaw[zero_mask] = current_yaw[zero_mask]

        yaw_error = target_yaw - current_yaw
        yaw_error = (yaw_error + np.pi) % (2.0 * np.pi) - np.pi
        yaw_error = yaw_error / np.pi

        return command_linear_vel, yaw_error

