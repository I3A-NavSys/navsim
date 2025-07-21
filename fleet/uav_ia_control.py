import pickle   # Serialization
import base64   # Parsing to string
import GPUtil
import torch
import numpy as np
import io
from scipy.spatial.transform import Rotation


from omni.isaac.core.prims import RigidPrimView


from uspace.flight_plan.waypoint import Waypoint
from uspace.flight_plan.command import Command
from navsim_utils.paths_utils import get_navsim_root_path
from navsim_utils.sim_utils import *


project_root_path = get_navsim_root_path()

class UAVState:
    IDLE = "idle"
    BUSY = "busy"
    DEAD = "dead"

class TypeMessage:
    EXTENSSION_ON_OFF = "extension_on_off"
    CMD_FP_REQUEST = "cmd_fp_request"
    USPACE = "uspace"

class UAVcontrol:
    def __init__(self, rigid_prim_view, torch_device, uavs, operator_event, event_stream):
        self.rigid_prim_view: RigidPrimView = rigid_prim_view
        self.torch_device = torch_device
        self.uavs = uavs
        self.operator_event = operator_event
        self.event_stream = event_stream
        
        # Load policy for IA control
        self.policy_path = f"{project_root_path}/tmp/tmpVictor/IA_control_UAV/policy.pt"
        self.policy_actions_scale = 10

        with open(self.policy_path, 'rb') as f:
            file_bytes = io.BytesIO(f.read())
        self.policy = torch.jit.load(file_bytes, map_location=self.torch_device).eval()

        # Navigation parameters
        self.n = self.rigid_prim_view.count
        self.current_time = 0.0
        self.step_size = 0.0
        
        # Pre-compute UAV mappings for faster lookup
        self.uav_ids = np.array(list(self.uavs.keys()))
        self.uav_id_to_idx = {uav_id: i for i, uav_id in enumerate(self.uav_ids)}
        
        # Command management - use object arrays for efficient batch operations
        self.commands = np.array([Command() for _ in range(self.n)], dtype=object)
        self.current_wp = np.full(self.n, -1, dtype=np.int32)
        self.target_wp = np.full(self.n, -1, dtype=np.int32)
        self.cmd_exp_time = np.zeros(self.n, dtype=np.float64)
        
        # Batch state arrays
        self.poses = np.zeros((self.n, 3), dtype=np.float64)
        self.oris = np.zeros((self.n, 4), dtype=np.float64)
        self.rots = [None] * self.n
        self.eulers = np.zeros((self.n, 3), dtype=np.float64)
        self.world_lin_vels = np.zeros((self.n, 3), dtype=np.float64)
        self.world_ang_vels = np.zeros((self.n, 3), dtype=np.float64)
        self.local_lin_vels = np.zeros((self.n, 3), dtype=np.float64)
        self.local_ang_vels = np.zeros((self.n, 3), dtype=np.float64)
        
        # Navigation state arrays
        self.has_flightplan = np.zeros(self.n, dtype=bool)
        self.has_command = np.zeros(self.n, dtype=bool)
        self.active_mask = np.zeros(self.n, dtype=bool)
        self.flightplans = np.array([None] * self.n, dtype=object)
        
        # Pre-allocated navigation computation arrays
        self.nav_distances = np.zeros(self.n, dtype=np.float64)
        self.nav_status_changed = np.zeros(self.n, dtype=bool)
        self.nav_obsolete = np.zeros(self.n, dtype=bool)
        self.nav_starting = np.zeros(self.n, dtype=bool)
        self.nav_in_progress = np.zeros(self.n, dtype=bool)
        self.nav_completed = np.zeros(self.n, dtype=bool)
        self.nav_well_positioned = np.zeros(self.n, dtype=bool)
        
        # Control system matrices and arrays
        self.rotors_vel = np.zeros((self.n, 4), dtype=np.float64)
        
        # Dynamics coefficients
        self.thrust_coeffs = np.array([4.6544, 4.6544, 0.9309, 0.9309])
        self.drag_force_coeffs = -np.array([3.0625, 4.0000, 7.8400])
        self.drag_moment_coeffs = np.array([5.9683, 5.9683, 1.4921, 1.4921])
        self.friction_moment_coeffs = -np.array([37.4010, 25.8580, 20.2514])
        
        # Physics arrays
        self.individual_pos = np.array([
            [0, 0, 0], 
            [0.5, 1.95, 0.5], 
            [0.5, -1.95, 0.5],
            [-2.5, 1.55, 0.5], 
            [-2.5, -1.55, 0.5]
        ])
        self.forces_to_apply = np.zeros((self.n, 3))
        self.torques_to_apply = np.zeros((self.n, 3))
        
        # Tracking
        self.tracked_info = {uav_id: [] for uav_id in self.uav_ids}
        self.is_tracking = np.zeros(self.n, dtype=bool)
        self.refresh_rate = 1.0
        self.last_time_track = np.zeros(self.n)
        self.completed_fps = np.zeros(self.n)
        self.compare_controls = np.zeros(self.n, dtype=bool)
        
        # Performance tracking arrays
        self.track_arrays = {
            'z_ang_vel': [[] for _ in range(self.n)],
            'roll': [[] for _ in range(self.n)],
            'pitch': [[] for _ in range(self.n)],
            'servo_time': [[] for _ in range(self.n)],
            'cpu_usage': [[] for _ in range(self.n)],
            'mem_usage': [[] for _ in range(self.n)],
            'gpu_usage': [[] for _ in range(self.n)]
        }
        
        self.gpus = GPUtil.getGPUs()

    def update(self, current_time, uavs, step_size):
        """Main update loop with maximum batching"""
        self.current_time = current_time
        self.step_size = step_size
        self.uavs = uavs
        
        # 1. Batch IMU data collection
        self._batch_collect_imu_data()
        
        # 2. Batch navigation state assessment
        self._batch_assess_navigation_state()
        
        # 3. Batch navigation processing (heavily optimized)
        if np.any(self.has_flightplan):
            self._batch_process_navigation()
        
        # 4. Batch servo control
        if np.any(self.active_mask):
            self._batch_servo_control()
        
        # 5. Batch telemetry
        self._batch_telemetry()
        
        # 6. Batch platform dynamics
        self._batch_platform_dynamics()
        
        # 7. Apply physics
        if np.any(self.active_mask):
            self._apply_physics()

    def _batch_collect_imu_data(self):
        """Optimized IMU data collection with minimal conversions"""
        # Get all poses and orientations at once
        self.poses, self.oris = self.rigid_prim_view.get_world_poses()
        
        # Vectorized invalid orientation handling
        ori_invalid = (np.sum(self.oris, axis=1) == 0) | np.isnan(self.oris).any(axis=1)
        self.oris[ori_invalid] = [1, 0, 0, 0]
        
        # Batch conversion to rotations and euler angles
        self.rots = [Rotation.from_quat([ori[1], ori[2], ori[3], ori[0]]) for ori in self.oris]
        self.eulers = np.array([rot.as_euler('xyz', degrees=False) for rot in self.rots])
        
        # Get velocities
        self.world_lin_vels = self.rigid_prim_view.get_linear_velocities()
        self.world_ang_vels = self.rigid_prim_view.get_angular_velocities()
        
        # Vectorized local frame conversion
        self.local_lin_vels = np.array([rot.inv().apply(vel) for rot, vel in zip(self.rots, self.world_lin_vels)])
        self.local_ang_vels = np.array([rot.inv().apply(vel) for rot, vel in zip(self.rots, self.world_ang_vels)])

    def _batch_assess_navigation_state(self):
        """Vectorized navigation state assessment"""
        # Reset state arrays
        self.has_flightplan.fill(False)
        self.has_command.fill(False)
        self.flightplans.fill(None)
        
        # Batch assessment of flight plan and command status
        for i, uav_id in enumerate(self.uav_ids):
            fp = self.uavs[uav_id].get("flightplan")
            if fp is not None:
                self.has_flightplan[i] = True
                self.flightplans[i] = fp
                # Pre-compute target waypoint
                self.target_wp[i] = fp.get_target_index_from_time(self.current_time)
            
            self.has_command[i] = self.commands[i].on
        
        self.active_mask = self.has_flightplan | self.has_command

    def _batch_process_navigation(self):
        """Highly optimized batch navigation processing"""
        # Get indices of UAVs with flight plans
        fp_indices = np.where(self.has_flightplan)[0]
        if len(fp_indices) == 0:
            return
        
        # Vectorized status change detection
        self.nav_status_changed[fp_indices] = self.current_wp[fp_indices] != self.target_wp[fp_indices]
        self.nav_obsolete[fp_indices] = (self.current_wp[fp_indices] == -1) & (self.target_wp[fp_indices] != 0)
        
        # Batch process obsolete flight plans
        obsolete_indices = fp_indices[self.nav_obsolete[fp_indices]]
        for i in obsolete_indices:
            uav_id = self.uav_ids[i]
            print(f"[{self.current_time:3.2f}] {uav_id}: discarding FP due to obsolete status")
            self.uavs[uav_id]["flightplan"] = None
            self.has_flightplan[i] = False
        
        # Update fp_indices after removing obsolete plans
        fp_indices = np.where(self.has_flightplan)[0]
        if len(fp_indices) == 0:
            return
        
        # Vectorized flight plan status classification
        changed_indices = fp_indices[self.nav_status_changed[fp_indices]]
        
        if len(changed_indices) > 0:
            # Vectorized status determination
            fp_lengths = np.array([len(self.flightplans[i].waypoints) for i in changed_indices])
            target_wps = self.target_wp[changed_indices]
            
            self.nav_starting[changed_indices] = (target_wps == 0)
            self.nav_in_progress[changed_indices] = (target_wps < fp_lengths) & (target_wps > 0)
            self.nav_completed[changed_indices] = (target_wps >= fp_lengths)
            
            # Vectorized position checking for starting flight plans
            starting_indices = changed_indices[self.nav_starting[changed_indices]]
            if len(starting_indices) > 0:
                start_positions = np.array([self.flightplans[i].waypoints[0].pos for i in starting_indices])
                distances = np.linalg.norm(self.poses[starting_indices] - start_positions, axis=1)
                radii = np.array([self.flightplans[i].radius for i in starting_indices])
                self.nav_well_positioned[starting_indices] = distances <= radii
            
            # Process state changes in batches
            self._batch_process_starting_plans(starting_indices)
            self._batch_process_in_progress_plans(changed_indices[self.nav_in_progress[changed_indices]])
            self._batch_process_completed_plans(changed_indices[self.nav_completed[changed_indices]])
        
        # Batch update navigation state and generate commands
        self.current_wp[fp_indices] = self.target_wp[fp_indices]
        self._batch_generate_commands(fp_indices)

    def _batch_process_starting_plans(self, indices):
        """Process starting flight plans in batch"""
        well_positioned = indices[self.nav_well_positioned[indices]]
        poorly_positioned = indices[~self.nav_well_positioned[indices]]
        
        # Handle well-positioned UAVs
        for i in well_positioned:
            uav_id = self.uav_ids[i]
            print(f"[{self.current_time:3.2f}] {uav_id}: waiting to start flightplan")
            self.inform_operator(
                TypeMessage.USPACE,
                uav_id,
                UAVState.BUSY, 
                self.current_time, 
                self.poses[i], 
                self.flightplans[i], 
                ""
            )
        
        # Handle poorly positioned UAVs
        for i in poorly_positioned:
            uav_id = self.uav_ids[i]
            print(f"[{self.current_time:3.2f}] {uav_id}: discarding FP - incorrect position")
            self.uavs[uav_id]["flightplan"] = None
            self.has_flightplan[i] = False

    def _batch_process_in_progress_plans(self, indices):
        """Process in-progress flight plans"""
        self.is_tracking[indices] = True
        
        for i in indices:
            uav_id = self.uav_ids[i]
            target_wp = self.target_wp[i]
            fp = self.flightplans[i]
            
            label = fp.waypoints[target_wp].label if fp.waypoints[target_wp].label else str(target_wp)
            print(f"[{self.current_time:3.2f}] {uav_id}: flying to {label}")
            self.inform_operator(
                TypeMessage.USPACE,
                uav_id, 
                UAVState.BUSY, 
                self.current_time, 
                self.poses[i], 
                fp, 
                ""
            )

    def _batch_process_completed_plans(self, indices):
        """Process completed flight plans"""
        self.is_tracking[indices] = False
        
        for i in indices:
            uav_id = self.uav_ids[i]
            fp = self.flightplans[i]
            
            print(f"[{self.current_time:3.2f}] {uav_id}: flight plan completed")
            self.inform_operator(
                TypeMessage.USPACE,
                uav_id, 
                UAVState.IDLE, 
                self.current_time, 
                self.poses[i], 
                fp, 
                self.tracked_info[uav_id], 
                is_request_completed=True
            )
            
            # Reset state
            self.tracked_info[uav_id] = []
            self.uavs[uav_id]["flightplan"] = None
            self.commands[i].off()
            self.has_flightplan[i] = False
            
            if self.compare_controls[i]:
                self.completed_fps[i] += 1
                self._export_tracking_data(i, uav_id)
                self._reset_tracking_data(i)

    def _batch_generate_commands(self, indices):
        """Generate commands for active flight plans in batch"""
        for i in indices:
            if not self.has_flightplan[i]:
                continue
                
            fp = self.flightplans[i]
            uav_id = self.uav_ids[i]
            target_wp = self.target_wp[i]
            
            # Determine heading
            if target_wp == 0:
                heading = fp.waypoints[0].heading
            else:
                heading = fp.waypoints[target_wp - 1].heading
            
            # Generate command
            self.commands[i] = fp.get_command(
                self.current_time, self.poses[i], self.world_lin_vels[i],
                self.rots[i], heading, 2.0
            )
            self.cmd_exp_time[i] = self.current_time + self.commands[i].duration

    def _batch_servo_control(self):
        """Batched AI policy-based servo control"""
        active_indices = np.where(self.active_mask)[0]
        if len(active_indices) == 0:
            return
        
        # Prepare command data arrays
        cmd_on = np.array([self.commands[i].on for i in range(self.n)])
        cmd_expired = self.current_time > self.cmd_exp_time
        
        # Handle expired commands
        expired_indices = active_indices[cmd_expired[active_indices] & cmd_on[active_indices]]
        for i in expired_indices:
            self.commands[i].hover()
        
        # Filter for active control
        control_active = cmd_on & self.active_mask
        if not np.any(control_active):
            return
        
        control_indices = np.where(control_active)[0]
        n_active = len(control_indices)
        
        if n_active == 0:
            return
        
        # Batch prepare command references
        commands_batch = np.zeros((n_active, 4), dtype=np.float32)
        for idx, i in enumerate(control_indices):
            commands_batch[idx] = [
                self.commands[i].velX,
                self.commands[i].velY, 
                self.commands[i].velZ,
                self.commands[i].rotZ
            ]
        
        # Batch compute velocity and angular velocity errors
        local_lin_vels_active = self.local_lin_vels[control_indices]
        local_ang_vels_active = self.local_ang_vels[control_indices]
        eulers_active = self.eulers[control_indices]
        
        # Vectorized error computation
        lin_vel_errors = local_lin_vels_active - commands_batch[:, :3]
        ang_vel_errors = local_ang_vels_active[:, 2] - commands_batch[:, 3]
        
        # Batch prepare observations for policy (16 components total)
        observations_batch = np.zeros((n_active, 16), dtype=np.float32)
        
        # Vectorized observation assembly
        observations_batch[:, 0:3] = local_lin_vels_active          # local_lin_vel (3 components)
        observations_batch[:, 3:6] = local_ang_vels_active          # local_ang_vel (3 components)
        observations_batch[:, 6] = eulers_active[:, 0]              # roll (1 component)
        observations_batch[:, 7] = eulers_active[:, 1]              # pitch (1 component)
        observations_batch[:, 8:12] = commands_batch                # command (4 components)
        observations_batch[:, 12:15] = lin_vel_errors               # lin_vel_error (3 components)
        observations_batch[:, 15] = ang_vel_errors                  # ang_vel_error (1 component)
        
        # Convert to tensor and move to device
        observations_tensor = torch.tensor(observations_batch, device=self.torch_device)
        
        # Batch policy inference
        with torch.no_grad():
            rotors_vel_tensor = self.policy(observations_tensor) * self.policy_actions_scale
            rotors_vel_batch = rotors_vel_tensor.cpu().numpy()
        
        # Assign rotor velocities to active UAVs
        self.rotors_vel[control_indices] = rotors_vel_batch
        
        # Turn off rotors for inactive UAVs
        inactive_indices = np.where(~self.active_mask)[0]
        self.rotors_vel[inactive_indices] = 0.0

    def _batch_telemetry(self):
        """Vectorized telemetry processing"""
        track_due = (self.current_time - self.last_time_track) >= self.refresh_rate
        should_track = self.is_tracking & track_due
        
        if not np.any(should_track):
            return
        
        # Update tracking timestamps
        self.last_time_track[should_track] = self.current_time
        
        # Batch update tracking data
        track_indices = np.where(should_track)[0]
        for i in track_indices:
            uav_id = self.uav_ids[i]
            self.tracked_info[uav_id].append(
                Waypoint(t=self.current_time, pos=self.poses[i], vel=self.world_lin_vels[i])
            )
            
            if self.compare_controls[i]:
                self.track_arrays['z_ang_vel'][i].append(self.local_ang_vels[i, 2])
                self.track_arrays['roll'][i].append(self.eulers[i, 0])
                self.track_arrays['pitch'][i].append(self.eulers[i, 1])

    def _batch_platform_dynamics(self):
        """Vectorized platform dynamics computation"""
        # Vectorized thrust computation
        thrust_z = self.thrust_coeffs * self.rotors_vel**2
        
        # Vectorized drag forces
        fd = self.drag_force_coeffs * np.abs(self.local_lin_vels) * self.local_lin_vels
        
        # Vectorized drag moments
        mdr_z = self.drag_moment_coeffs * self.rotors_vel**2
        mdr = np.zeros((self.n, 3))
        mdr[:, 2] = mdr_z[:, 1] - mdr_z[:, 0] - mdr_z[:, 3] + mdr_z[:, 2]
        
        # Vectorized friction moments
        md = self.friction_moment_coeffs * np.abs(self.local_ang_vels) * self.local_ang_vels
        
        # Vectorized force and torque computation
        individual_forces = np.zeros((self.n, 5, 3))
        individual_forces[:, 0] = fd
        individual_forces[:, 1:, 2] = thrust_z
        
        self.forces_to_apply = np.sum(individual_forces, axis=1)
        
        # Vectorized torque computation
        torques = np.zeros((self.n, 3))
        for j in range(5):
            torques += np.cross(self.individual_pos[j], individual_forces[:, j])
        
        self.torques_to_apply = mdr + md + torques

    def _apply_physics(self):
        """Apply forces and torques to all UAVs"""

        self.rigid_prim_view.apply_forces_and_torques_at_pos(
            forces=self.forces_to_apply[self.active_mask],
            torques=self.torques_to_apply[self.active_mask],
            indices=np.where(self.active_mask)[0],
            is_global=False
        )

    def _export_tracking_data(self, i, uav_id):
        """Export tracking data to CSV"""
        if not self.tracked_info[uav_id]:
            return
        
        # Vectorized data extraction
        data_points = self.tracked_info[uav_id]
        times = np.array([wp.t for wp in data_points])
        positions = np.array([wp.pos for wp in data_points])
        velocities = np.array([wp.vel for wp in data_points])
        
        # Calculate errors if flight plan exists
        uav = self.uavs[uav_id]
        if uav.get("flightplan"):
            fp_positions = np.array([uav["flightplan"].status_at_time(t).pos for t in times])
            errors = np.linalg.norm(positions - fp_positions, axis=1)
        else:
            errors = np.zeros_like(times)
        
        # Vectorized acceleration computation
        vel_diffs = np.diff(velocities, axis=0, prepend=velocities[:1])
        
        # Prepare data array
        min_len = min(len(times), len(errors), len(vel_diffs))
        data = np.column_stack([
            times[:min_len], errors[:min_len], vel_diffs[:min_len]
        ])
        
        path = f"{project_root_path}/sims/exported_data/{uav_id}_{int(self.completed_fps[i])}_optimized.csv"
        np.savetxt(path, data, delimiter=",", fmt="%.6f")

    def _reset_tracking_data(self, i):
        """Reset tracking arrays for UAV i"""
        for key in self.track_arrays:
            self.track_arrays[key][i] = []

    def inform_operator(
        self, 
        type_message, 
        id=None, 
        state=None, 
        time=None, 
        pos=None, 
        flightplan=None, 
        tracked_info=None, 
        is_request_completed=False
    ):
        payload = {"type_message": type_message}
        msg = {"sender": TypeSender.SINGLE_UAV}

        match type_message:
            case TypeMessage.USPACE:    
                serialized_fp = base64.b64encode(
                    pickle.dumps(flightplan)
                ).decode('utf-8')
                serialized_pos = base64.b64encode(
                    pickle.dumps(pos)
                ).decode('utf-8')
                
                if is_request_completed:
                    tracked_info = base64.b64encode(
                        pickle.dumps(tracked_info)
                    ).decode('utf-8')

                msg["request"] = {
                    "id": id,
                    "state": state,
                    "time": time,
                    "pos": serialized_pos,
                    "flightplan": serialized_fp,
                    "tracked_info": tracked_info
                }

        payload["msg"] = msg

        self.event_stream.push(self.operator_event, payload=payload)

    def rotors_off(self):
        """Turn off all rotors"""
        self.rotors_vel.fill(0)