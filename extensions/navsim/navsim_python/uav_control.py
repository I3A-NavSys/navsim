import numpy as np

from omni.isaac.core.prims import RigidPrimView

class UAVControl:
    def __init__(self, rigid_prim_view: RigidPrimView, uav_ids_to_physics_buffer: dict):
        # Runtime variables
        self.rigid_prim_view = rigid_prim_view
        self.uav_ids_to_physics_buffer = uav_ids_to_physics_buffer
        self.amount_uavs = self.rigid_prim_view.count
        self.rigid_prim_uav_indices = np.arange(self.amount_uavs)

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

        self.rotors_vel = np.zeros((self.amount_uavs, 4))

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
        self.r = np.zeros((self.amount_uavs, 4, 1))
        self.x = np.zeros((self.amount_uavs, 8, 1))
        self.y = np.zeros((self.amount_uavs, 4, 1))
        self.e = np.zeros((self.amount_uavs, 4, 1))
        self.E = np.zeros((self.amount_uavs, 4, 1))
        self.u = np.zeros((self.amount_uavs, 4, 1))
        self.E_max = 150

        # Physics
        self.bodies_positions = np.array([
            [0, 0, 0], [0.5, 1.95, 0.5], [0.5, -1.95, 0.5],
            [-2.5, 1.55, 0.5], [-2.5, -1.55, 0.5]
        ])
        self.forces_to_apply = np.zeros((self.amount_uavs, 3))
        self.torques_to_apply = np.zeros((self.amount_uavs, 3))

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
        fp_headings
    ):
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
            target_indices
        )

        expected_pos, expected_vel = self.status_at_time(
            current_time,
            running_data[0],  # times
            running_data[1],  # positions
            running_data[2],  # velocities
            running_data[3],  # accelerations
            running_data[4],  # jerks
            running_data[5],  # snaps
            running_data[6],  # crackels
        )

        cmd_world_linear_vel, cmd_yaw_rotation = self.get_command(
            expected_pos,
            expected_vel,
            self.positions[uav_physics_indices],
            self.world_linear_vels[uav_physics_indices],
            self.eulers[uav_physics_indices, 2],
            running_data[7],  # headings
            2
        )

        cmd_local_linear_vel = self.global_to_local_velocity(
            cmd_world_linear_vel,
            self.orientations[uav_physics_indices]
        )

        return cmd_local_linear_vel, cmd_yaw_rotation

    def servo_control(self, uav_physics_indices, command_linear_vel, command_angular_vel, step_size):
        # Assign the model reference to be followed
        self.r[uav_physics_indices, 0, 0] = command_linear_vel[:, 0]  # bXdot
        self.r[uav_physics_indices, 1, 0] = command_linear_vel[:, 1]  # bYdot
        self.r[uav_physics_indices, 2, 0] = command_linear_vel[:, 2]  # bZdot
        self.r[uav_physics_indices, 3, 0] = command_angular_vel     # hZdot

        # Assign model state
        self.x[uav_physics_indices, :2, 0] = self.eulers[uav_physics_indices, :2]
        self.x[uav_physics_indices, 2:5, 0] = self.local_angular_vels[uav_physics_indices] # bWx, bWy, bWz
        self.x[uav_physics_indices, 5:8, 0] = self.local_linear_vels[uav_physics_indices]  # bXdot, bYdot, bZdot

        # Assign model output
        self.y[uav_physics_indices, 0, 0] = self.x[uav_physics_indices, 5, 0]        # bXdot
        self.y[uav_physics_indices, 1, 0] = self.x[uav_physics_indices, 6, 0]        # bYdot
        self.y[uav_physics_indices, 2, 0] = self.x[uav_physics_indices, 7, 0]        # bZdot
        self.y[uav_physics_indices, 3, 0] = self.x[uav_physics_indices, 4, 0]        # bWz

        # Error between the output and the reference 
        self.e[uav_physics_indices] = self.y[uav_physics_indices] - self.r[uav_physics_indices]

        # Cumulative error
        self.E[uav_physics_indices] += self.e[uav_physics_indices] * step_size
        self.E = np.clip(self.E, -self.E_max, self.E_max)

        # Dynamic system control
        self.u[uav_physics_indices] = (
            self.Hs - 
            (self.Kx @ self.x[uav_physics_indices]) - 
            (self.Ky @ self.E[uav_physics_indices])
        )

        # Rotor speed saturation
        self.u = np.clip(self.u, self.w_min, self.w_max)

        # Assign rotor speed
        self.rotors_vel[uav_physics_indices, 0] = self.u[uav_physics_indices, 1, 0]
        self.rotors_vel[uav_physics_indices, 1] = self.u[uav_physics_indices, 0, 0]
        self.rotors_vel[uav_physics_indices, 2] = self.u[uav_physics_indices, 3, 0]
        self.rotors_vel[uav_physics_indices, 3] = self.u[uav_physics_indices, 2, 0]

    def compute_dynamics(self, uav_physics_indices):
        square_rotors_vel = self.rotors_vel * self.rotors_vel

        # Thrust forces
        thrust_z = self.thrust_coeffs * square_rotors_vel
        
        # Drag forces
        fd = (
            self.drag_force_coeffs * 
            np.abs(self.local_linear_vels) * 
            self.local_linear_vels
        )
        
        # Drag moments
        mdr_z = self.drag_moment_coeffs * square_rotors_vel
        mdr = np.zeros((self.amount_uavs, 3))
        mdr[:, 2] = mdr_z[:, 1] - mdr_z[:, 0] - mdr_z[:, 3] + mdr_z[:, 2]
        
        # Friction moments
        md = (
            self.friction_moment_coeffs * 
            np.abs(self.local_angular_vels) * 
            self.local_angular_vels
        )
        
        # Force and torque
        bodies_forces = np.zeros((self.amount_uavs, 5, 3))
        bodies_forces[uav_physics_indices, 0] = fd[uav_physics_indices]
        bodies_forces[uav_physics_indices, 1:, 2] = thrust_z[uav_physics_indices]
        
        self.forces_to_apply[uav_physics_indices] = np.sum(
            bodies_forces[uav_physics_indices], 
            axis=1
        )
        
        torques = np.sum(
            np.cross(self.bodies_positions, bodies_forces[uav_physics_indices]), axis=1
        )
        
        self.torques_to_apply[uav_physics_indices] = (
            mdr[uav_physics_indices] + md[uav_physics_indices] + torques
        )

    def apply_dynamics(self, uav_to_update_idx):
        self.rigid_prim_view.apply_forces_and_torques_at_pos(
            forces=self.forces_to_apply[uav_to_update_idx],
            torques=self.torques_to_apply[uav_to_update_idx],
            is_global=False,
            indices=self.rigid_prim_uav_indices[uav_to_update_idx]
        )
    
    def reset_dynamics(self, uav_to_update_idx):
        mask = np.ones(self.amount_uavs, dtype=bool)
        mask[uav_to_update_idx] = False

        self.forces_to_apply[mask] = [0,0,0]
        self.torques_to_apply[mask] = [0,0,0]

        self.E[mask, :, 0] = 0

    def update(self, uav_physics_indices, flightplans, current_time, step_size):
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
            fp_headings
        )

        # uav_idxs = []
        # for operator_id, uav_id, flightplan in flightplans:
        #     uav_idx = self.uav_ids_to_physics_buffer[operator_id][uav_id]
        #     uav_idxs.append(uav_idx)

        #     current_pos = self.positions[uav_idx]
        #     current_world_linear_vel = self.world_linear_vels[uav_idx]
        #     current_yaw = self.eulers[uav_idx, 2]
        #     current_waypoint_heading = flightplan.get_running_waypoint(current_time).heading

        #     cmd_world_linear_vel, cmd_yaw_rotation = flightplan.get_isaacsim_command(
        #         current_time,
        #         current_pos,
        #         current_world_linear_vel,
        #         current_yaw,
        #         current_waypoint_heading,
        #         2
        #     )

        #     cmd_local_linear_vel = self.global_to_local_velocity(
        #         cmd_world_linear_vel,
        #         self.orientations[uav_idx]
        #     )

            # if uav_id == "UAV_03":
            #     print(f"[{uav_id}] - POS: {current_pos}")
                # print(f"CURRENT WORLD LINEAR VEL: {current_world_linear_vel}")
                # print(f"CURRENT YAW: {current_yaw}")
                # print(f"CMD WORLD LINEAR VEL: {cmd_world_linear_vel}")
                # print(f"CMD YAW ROTATION: {cmd_yaw_rotation}")
                # print(f"CMD LOCAL LINEAR VEL: {cmd_local_linear_vel}")
                # print()

        self.servo_control(
            uav_physics_indices, 
            cmd_local_linear_vel, 
            cmd_yaw_rotation, 
            step_size
        )
        self.compute_dynamics(uav_physics_indices)

        self.apply_dynamics(uav_physics_indices)
        self.reset_dynamics(uav_physics_indices)
        

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

    def adjust_2d_array_shape(self, array: list[list[float]]):
        """
        Converts a list of varying-length lists into a 2D NumPy array 
        padded with np.inf.
        
        This method uses memory pre-allocation for maximum performance.
        
        Args:
            jagged_list: A list containing lists of numbers.
            
        Returns:
            A 2D numpy array of shape (num_rows, max_length) filled with np.inf 
            in the empty spots.
        """
        # 1. Find the dimensions needed for the final matrix
        num_rows = len(array)
        max_length = max(len(row) for row in array)
        
        # 2. Pre-allocate the entire matrix filled with infinity
        # np.full is faster than creating an array of zeros and replacing them
        matrix = np.full((num_rows, max_length), np.inf)
        
        # 3. Overwrite the infinities with the actual data using slicing
        # This loop is in Python, but the row assignment is executed in C by NumPy
        for i, row in enumerate(array):
            matrix[i, :len(row)] = row
            
        return matrix

    def adjust_3d_array_shape(self, array: list[list[np.ndarray]]):
        """
        Converts a jagged list of NumPy arrays (e.g., 3D coordinates) 
        into a uniform 3D NumPy array padded with np.inf.
        
        Args:
            jagged_list: A list containing lists of 1D NumPy arrays.
            
        Returns:
            A 3D numpy array of shape (num_paths, max_waypoints, num_dimensions).
        """
        num_rows = len(array)
        max_length = max(len(path) for path in array)
        
        # Dynamically find the number of dimensions (e.g., 3 for [x, y, z])
        # Assumes the first path has at least one waypoint
        num_dimensions = len(array[0][0])
        
        # Pre-allocate a 3D block of memory filled with infinity
        matrix = np.full((num_rows, max_length, num_dimensions), np.inf)
        
        # Inject the actual coordinates using hardware-level slicing
        for i, path in enumerate(array):
            # NumPy automatically converts the list of 1D arrays into a 2D block
            # and slots it perfectly into the 3D tensor
            matrix[i, :len(path), :] = path
            
        return matrix

    def get_running_idx(self, array, value):
        """
        Finds the left-side index for a given value across multiple arrays simultaneously.
        Uses pure boolean vectorization, avoiding Python loops entirely.
        
        Args:
            padded_matrix: A 2D numpy array where shorter sub-arrays are padded with np.inf.
            value: The float value to search for.
            
        Returns:
            A 1D numpy array with the corresponding indices for each row.
        """
        # 1. matrix <= value: Creates a boolean matrix (True where elements are <= value)
        # 2. np.sum(..., axis=1): Counts the True values per row. 
        # 3. Subtract 1: Converts the count to a 0-based index.
        indices = np.sum(array <= value, axis=1) - 1
        
        # 4. np.clip: Forces any -1 (when value is smaller than the first element) to become 0.
        return np.clip(indices, 0, None)

    def extract_running_data(
        self, 
        times, 
        positions, 
        velocities, 
        accelerations, 
        jerks, 
        snaps, 
        crackels, 
        headings,
        target_indices
    ):
        """
        Extracts a specific 3D coordinate from each path in a 3D tensor 
        using advanced integer indexing. Extremely fast, zero loops.
        
        Args:
            tensor_3d: A 3D NumPy array of shape (num_paths, max_waypoints, 3).
            target_indices: A list or 1D array of indices to extract for each path.
            
        Returns:
            A 2D NumPy array of shape (num_paths, 3) with the extracted coordinates.
        """
        # 1. Create an array representing the path indices: [0, 1, 2, ..., N]
        # This guarantees we pick one element per path (row)
        path_indices = np.arange(times.shape[0])
        
        # 2. Use advanced indexing to extract the elements. 
        # NumPy will pair path_indices[0] with target_indices[0], 
        # path_indices[1] with target_indices[1], etc.
        running_times = times[path_indices, target_indices]
        running_positions = positions[path_indices, target_indices]
        running_velocities = velocities[path_indices, target_indices]
        running_accelerations = accelerations[path_indices, target_indices]
        running_jerks = jerks[path_indices, target_indices]
        running_snaps = snaps[path_indices, target_indices]
        running_crackels = crackels[path_indices, target_indices]
        running_headings = headings[path_indices, target_indices]
        
        return [
            running_times,
            running_positions,
            running_velocities,
            running_accelerations,
            running_jerks,
            running_snaps,
            running_crackels,
            running_headings,
        ]

    def status_at_time(
        self,
        current_time, 
        running_times, 
        running_pos, 
        running_vel, 
        running_accel, 
        running_jerk, 
        running_snap, 
        running_crackel, 
    ):
        running_times_diff = current_time - running_times
        running_times_diff = np.clip(running_times_diff, 0, None)  # Ensure non-negative
        running_times_diff = running_times_diff[:, np.newaxis]  # Reshape for broadcasting

        running_times_diff_2 = running_times_diff * running_times_diff
        running_times_diff_3 = running_times_diff_2 * running_times_diff
        running_times_diff_4 = running_times_diff_3 * running_times_diff
        running_times_diff_5 = running_times_diff_4 * running_times_diff

        pos =  running_pos + running_vel * running_times_diff + 0.5 * running_accel * running_times_diff_2 + (1/6) * running_jerk * running_times_diff_3 + (1/24) * running_snap * running_times_diff_4 + (1/120) * running_crackel * running_times_diff_5
        vel =  running_vel + running_accel * running_times_diff + 0.5 * running_jerk * running_times_diff_2 + (1/6) * running_snap * running_times_diff_3 + (1/24) * running_crackel * running_times_diff_4
        # accel = running_accel + running_jerk * current_time + 0.5 * running_snap * current_time**2 + (1/6) * running_crackel * current_time**3
        # jerk =  running_jerk + running_snap * current_time + 0.5 * running_crackel * current_time**2
        # snap =  running_snap + running_crackel * current_time
        # crackel = running_crackel

        return pos, vel
        
    def get_command(
        self, 
        expected_pos, 
        expected_vel, 
        current_pos, 
        current_lin_vel, 
        current_yaw, 
        headings, 
        t_to_solve
    ):
        correction_vel = (expected_pos - current_pos) / t_to_solve

        command_linear_vel = expected_vel + correction_vel

        variation_vel = command_linear_vel - current_lin_vel
        command_linear_vel = current_lin_vel + variation_vel

        # Replace any [0,0] heading with the expected velocity direction
        mask = (headings[:, 0] == 0) & (headings[:, 1] == 0)
        headings[mask] = expected_vel[mask, :2]
        mask = (headings[:, 0] == 0) & (headings[:, 1] == 0)

        target_yaw = np.arctan2(headings[:, 1], headings[:, 0])
        
        # If the target_yaw is 0, replace it with the current_yaw to avoid a calculation error
        target_yaw[mask] = current_yaw[mask]

        yaw_error = target_yaw - current_yaw

        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

        command_yaw_rotation = yaw_error / t_to_solve

        return command_linear_vel, command_yaw_rotation

