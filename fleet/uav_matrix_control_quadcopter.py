import sys
import os
import pickle   # Serialization
import base64   # Parsing to string
import time
import psutil
import GPUtil
import torch
import numpy as np
import io


from scipy.spatial.transform import Rotation
from omni.isaac.core.prims import RigidPrimView


from uspace.flight_plan.flight_plan import FlightPlan
from uspace.flight_plan.waypoint import Waypoint
from uspace.flight_plan.command import Command

project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.append(project_root_path)
project_root_path = project_root_path.replace("\\", "/")

class UAVState:
    IDLE = "idle"
    BUSY = "busy"
    DEAD = "dead"

class UAVcontrol:
    def __init__(self, rigid_prim_view, torch_device, uavs, operator_event, event_stream):
        self.rigid_prim_view : RigidPrimView = rigid_prim_view
        self.torch_device = torch_device
        self.uavs = uavs
        self.operator_event = operator_event
        self.event_stream = event_stream
        
        # Navigation parameters
        self.current_time = 0.0
        self.step_size = 0.0
        self.w_max = 628.3185  # Maximum rotor speed
        self.w_min = 0.0      # Minimum rotor speed
        self.w_hov = 291.45       # rad/s = 400rpm
        self.commands = {}
        self.current_wp = np.zeros(self.rigid_prim_view.count)
        self.cmd_exp_time = np.zeros(self.rigid_prim_view.count)
        self.rotors_vel = np.zeros(4)   # [NW, NE, SW, SE]
        
        # Tracking parameters
        self.tracked_info = {}
        self.is_tracking = np.zeros(self.rigid_prim_view.count, dtype=bool)
        self.refresh_rate = 1.0  # seconds
        self.last_time_track = np.zeros(self.rigid_prim_view.count)

        self.completed_fps = np.zeros(self.rigid_prim_view.count)
        self.compare_controls = np.zeros(self.rigid_prim_view.count, dtype=bool)
        self.track_z_ang_vel = np.zeros(self.rigid_prim_view.count)
        self.track_roll = np.zeros(self.rigid_prim_view.count)
        self.track_pitch = np.zeros(self.rigid_prim_view.count)
        self.track_servo_control_time = np.zeros(self.rigid_prim_view.count)
        self.track_cpu_usage = np.zeros(self.rigid_prim_view.count)
        self.track_mem_usage = np.zeros(self.rigid_prim_view.count)
        self.track_gpu_usage = np.zeros(self.rigid_prim_view.count)
        self.gpus = GPUtil.getGPUs()

        # Dynamics parameters
        kFT = 1.7197e-05
        kFDx = 1.1902e-04
        kFDy = 1.1902e-04
        kFDz = 36.4437e-4
        kMDR = 3.6714e-08
        kMDx = 1.1078e-04
        kMDy = 1.1078e-04
        kMDz = 7.8914e-05
        
        self.thrust_coeffs = np.array([kFT, kFT, kFT, kFT])
        self.drag_force_coeffs = -np.array([kFDx, kFDy, kFDz])
        self.drag_moment_coeffs = np.array([kMDR, kMDR, kMDR, kMDR])
        self.friction_moment_coeffs = -np.array([kMDx, kMDy, kMDz])
        self.mdr = np.zeros(3)

        self.Kx = np.array([        # state control matrix
            [-47.4820, -47.4820, -9.3626, -9.3626,  413.1508, -10.5091,  10.5091,  132.4440],
            [ 47.4820, -47.4820,  9.3626, -9.3626, -413.1508, -10.5091, -10.5091,  132.4440],
            [-47.4820,  47.4820, -9.3626,  9.3626, -413.1508,  10.5091,  10.5091,  132.4440],
            [ 47.4820,  47.4820,  9.3626,  9.3626,  413.1508,  10.5091, -10.5091,  132.4440]
        ])

        self.Ky = np.array([        # error control matrix
            [-8.1889,  8.1889,  294.3201,  918.1130],
            [-8.1889, -8.1889,  294.3201, -918.1130],
            [ 8.1889,  8.1889,  294.3201, -918.1130],
            [ 8.1889, -8.1889,  294.3201,  918.1130]
        ])

        self.Hs = np.array([        # hovering speed
            [self.w_hov], 
            [self.w_hov], 
            [self.w_hov], 
            [self.w_hov]
        ])

        self.r = np.zeros((self.rigid_prim_view.count, 4, 1))  # Reference
        self.x = np.zeros((self.rigid_prim_view.count, 8, 1))  # State
        self.y = np.zeros((self.rigid_prim_view.count, 4, 1))  # Output
        self.e = np.zeros((self.rigid_prim_view.count, 4, 1))  # Error
        self.E = np.zeros((self.rigid_prim_view.count, 4, 1))  # Cumulative error
        self.u = np.zeros((self.rigid_prim_view.count, 4, 1))  # Control input
        self.E_max = 15            # maximum model accumulated error

        self.individual_forces = np.zeros((5, 3))
        self.individual_pos = np.array([
            [0, 0, 0],
            [0.075, 0.075, 0.01],
            [0.075, -0.075, 0.01],
            [-0.075, 0.075, 0.01],
            [-0.075, -0.075, 0.01]
        ])
        self.forces_to_apply = np.zeros((self.rigid_prim_view.count, 3))
        self.torques_to_apply = np.zeros((self.rigid_prim_view.count, 3))

        # Initialize needed variables
        for i, uav_id in enumerate(self.uavs.keys()):
            self.commands[uav_id] = Command()
            self.current_wp[i] = -1
            self.tracked_info[uav_id] = []

    def update(self, current_time, uavs, step_size):
        self.current_time = current_time
        self.step_size = step_size
        self.uavs = uavs

        # Compute navigation information from all UAVs
        for i, (uav_id, uav) in enumerate(self.uavs.items()):
            # Skip UAV if it has no flight plan assigned
            has_flightplan = uav["flightplan"] is not None
            has_command = self.commands[uav_id].on
            if not has_flightplan and not has_command:  continue

            # Compute the UAV control
            self.imu(i)
            self.navigation(i, uav_id, uav)
            # Temporal, as we will select a specific control finally
            if self.compare_controls[i]:    self.servo_control_track(i)
            else:                           self.servo_control(i, uav_id)
            self.telemetry(i, uav_id)
            self.platform_dynamics(i)

        # Apply forces and torques to the UAVs
        self.apply_physics()

    def inform_operator(self, id, state, time, pos, flightplan, tracked_info, is_request_completed=False):
        serialized_fp = base64.b64encode(pickle.dumps(flightplan)).decode('utf-8')
        serialized_pos = base64.b64encode(pickle.dumps(pos)).decode('utf-8')
        if is_request_completed:
            tracked_info = base64.b64encode(pickle.dumps(tracked_info)).decode('utf-8')

        payload = {
            "is_request": True,
            "sender": "uav",
            "id": id,
            "state": state,
            "time":time,
            "pos": serialized_pos,
            "flightplan": serialized_fp,
            "tracked_info": tracked_info
        }

        self.event_stream.push(self.operator_event, payload=payload)

    def rotors_off(self):
        self.rotors_vel = np.zeros(4)

    def imu(self, i):
        # Get positions and orientations
        self.pos, self.ori = self.rigid_prim_view.get_world_poses(indices=[i])
        self.pos = self.pos[0]
        self.ori = self.ori[0]
        
        # Set identity orientation if it is not a valid quaternion
        are_all_0 = self.ori.sum() == 0
        is_any_nan = np.isnan(self.ori).any()
        
        if are_all_0 or is_any_nan:     self.ori = np.array([1, 0, 0, 0])

        # Convert quaternion to rotation object and extract roll, pitch, yaw
        self.rot = Rotation.from_quat([self.ori[1], self.ori[2], self.ori[3], self.ori[0]])
        self.roll, self.pitch, self.yaw = self.rot.as_euler('xyz', degrees=False)
        
        # Get linear and angular velocities (world/local frame)
        self.world_lin_vel = self.rigid_prim_view.get_linear_velocities(indices=[i])[0]
        self.world_ang_vel = self.rigid_prim_view.get_angular_velocities(indices=[i])[0]
        self.local_lin_vel = self.rot.inv().apply(self.world_lin_vel)
        self.local_ang_vel = self.rot.inv().apply(self.world_ang_vel)

    def navigation(self, i, uav_id, uav):
        has_flightplan = uav["flightplan"] is not None
        if not has_flightplan:  return
        
        uav_flightplan: FlightPlan = uav["flightplan"]
        target_wp = uav_flightplan.get_target_index_from_time(self.current_time)
        num_wps = len(uav_flightplan.waypoints)

        is_flightplan_obsolete = self.current_wp[i] == -1 and target_wp != 0
        has_status_changed = self.current_wp[i] != target_wp

        if is_flightplan_obsolete:
            print(f"[{self.current_time:3.2f}] {uav_id}: discarding FP due to it is obsolete")
            uav["flightplan"] = None
            return

        if has_status_changed:
            is_flightplan_starting = target_wp == 0
            is_flightplan_in_progress = target_wp < num_wps

            if is_flightplan_starting:
                is_well_positioned = np.linalg.norm(self.pos - uav_flightplan.waypoints[0].pos) <= uav_flightplan.radius

                if is_well_positioned:
                    print(f"[{self.current_time:3.2f}] {uav_id}: waiting to start the flightplan")
                    self.inform_operator(uav_id, UAVState.BUSY, self.current_time, self.pos, uav_flightplan, "")

                else:
                    print(f"[{self.current_time:3.2f}] {uav_id}: discarding FP due to an incorrect starting position")
                    uav["flightplan"] = None
                    return

            elif is_flightplan_in_progress:
                self.is_tracking[i] = True
                has_waypoint_label = uav_flightplan.waypoints[target_wp].label != ""
                
                if has_waypoint_label:      waypoint_label = uav_flightplan.waypoints[target_wp].label
                else:                       waypoint_label = target_wp

                print(f"[{self.current_time:3.2f}] {uav_id}: flying to {waypoint_label} waypoint")
                self.inform_operator(uav_id, UAVState.BUSY, self.current_time, self.pos, uav_flightplan, "")

            else:
                print(f"[{self.current_time:3.2f}] {uav_id}: flight plan completed")
                self.inform_operator(
                    uav_id, 
                    UAVState.IDLE, 
                    self.current_time, 
                    self.pos, 
                    uav_flightplan,
                    self.tracked_info[uav_id], 
                    is_request_completed=True
                )
                
                if self.compare_controls[i]:
                    self.completed_fps[i] += 1
                    self.tracked_data_to_csv(i, uav_id, uav)
                
                # Reset uav state
                self.is_tracking[i] = False
                self.tracked_info[uav_id] = []
                uav["flightplan"] = None
                self.commands[uav_id].off()

                if self.compare_controls[i]:
                    self.track_z_ang_vel[i] = []
                    self.track_roll[i] = []
                    self.track_pitch[i] = []
                    self.track_servo_control_time[i] = []
                    self.track_cpu_usage[i] = []
                    self.track_mem_usage[i] = []
                    self.track_gpu_usage[i] = []

                return
        
        # Update navigation state
        self.current_wp[i] = target_wp

        is_flightplan_start = target_wp == 0
        if is_flightplan_start: heading = uav_flightplan.waypoints[0].heading
        else:                   heading = uav_flightplan.waypoints[target_wp - 1].heading

        self.commands[uav_id] = uav_flightplan.get_command(
            self.current_time, 
            self.pos, 
            self.world_lin_vel, 
            self.rot, 
            heading, 
            2
        )
        self.cmd_exp_time[i] = self.current_time + self.commands[uav_id].duration

    def servo_control(self, i, uav_id):
        # Evaluate command status
        is_command_on = self.commands[uav_id].on
        is_cmd_duration_none = self.commands[uav_id].duration is None
        has_cmd_time_expired = self.current_time > self.cmd_exp_time[i]

        if not is_command_on:
            self.rotors_off()
            return

        if not is_cmd_duration_none and has_cmd_time_expired:
            self.commands[uav_id].hover()

        # Assign the model reference to be followed
        self.r[i, 0, 0] = self.commands[uav_id].velX       # bXdot
        self.r[i, 1, 0] = self.commands[uav_id].velY       # bYdot
        self.r[i, 2, 0] = self.commands[uav_id].velZ       # bZdot
        self.r[i, 3, 0] = self.commands[uav_id].rotZ       # hZdot

        # Assign model state
        self.x[i, 0, 0] = self.roll             # ePhi
        self.x[i, 1, 0] = self.pitch            # eTheta
        self.x[i, 2, 0] = self.local_ang_vel[0]       # bWx
        self.x[i, 3, 0] = self.local_ang_vel[1]       # bWy
        self.x[i, 4, 0] = self.local_ang_vel[2]       # bWz
        self.x[i, 5, 0] = self.local_lin_vel[0]       # bXdot
        self.x[i, 6, 0] = self.local_lin_vel[1]       # bYdot
        self.x[i, 7, 0] = self.local_lin_vel[2]       # bZdot

        # Assign model output
        self.y[i, 0, 0] = self.x[i, 5, 0]        # bXdot
        self.y[i, 1, 0] = self.x[i, 6, 0]        # bYdot
        self.y[i, 2, 0] = self.x[i, 7, 0]        # bZdot
        self.y[i, 3, 0] = self.x[i, 4, 0]        # bWz

        # Error between the output and the reference 
        # (between the commanded velocity and the drone velocity)
        self.e[i] = self.y[i] - self.r[i]

        # Cumulative error
        self.E[i] = self.E[i] + (self.e[i] * self.step_size)

        # Error saturation
        # if self.E[0, 0] >  self.E_max : self.E[0, 0] =  self.E_max
        # if self.E[0, 0] < -self.E_max : self.E[0, 0] = -self.E_max
        # if self.E[1, 0] >  self.E_max : self.E[1, 0] =  self.E_max
        # if self.E[1, 0] < -self.E_max : self.E[1, 0] = -self.E_max
        # if self.E[2, 0] >  self.E_max : self.E[2, 0] =  self.E_max
        # if self.E[2, 0] < -self.E_max : self.E[2, 0] = -self.E_max
        # if self.E[3, 0] >  self.E_max : self.E[3, 0] =  self.E_max
        # if self.E[3, 0] < -self.E_max : self.E[3, 0] = -self.E_max

        # Dynamic system control
        self.u[i] = self.Hs - self.Kx @ self.x[i] - self.Ky @ self.E[i]

        # Rotor speed saturation
        self.u[i].clip(min=self.w_min, max=self.w_max, out=self.u[i])

        # Assign rotor speed
        self.rotors_vel[0] = self.u[i, 1, 0]  # NW
        self.rotors_vel[1] = self.u[i, 0, 0]  # NE
        self.rotors_vel[2] = self.u[i, 3, 0]  # SW
        self.rotors_vel[3] = self.u[i, 2, 0]  # SE

    def platform_dynamics(self, i):
        # Compute thrust forces
        thrust_z = self.thrust_coeffs * self.rotors_vel**2

        # Compute drag force
        fd = self.drag_force_coeffs * np.abs(self.local_lin_vel) * self.local_lin_vel
        
        # Compute the drag moment
        mdr_z = self.drag_moment_coeffs * self.rotors_vel**2
        self.mdr[2] = mdr_z[1] - mdr_z[0] - mdr_z[3] + mdr_z[2]

        # Compute friction moments
        md = self.friction_moment_coeffs * np.abs(self.local_ang_vel) * self.local_ang_vel

        self.individual_forces[0] = fd
        self.individual_forces[1:, 2] = thrust_z[:]

        self.forces_to_apply[i] = np.sum(self.individual_forces, axis=0)
        self.torques_to_apply[i] = self.mdr + md + self.compute_torques(self.individual_forces, self.individual_pos)

    def apply_physics(self):
        self.rigid_prim_view.apply_forces_and_torques_at_pos(
            forces=self.forces_to_apply,
            torques=self.torques_to_apply,
            is_global=False
        )

    def compute_torques(self, forces, positions):
        """
        Compute torques about each axis (X, Y, Z) given forces applied at offset positions.
        
        Args:
            forces: List of force vectors [Fx, Fy, Fz] in Newtons
        - positions: List of position vectors [x, y, z] in meters relative to COM
        
        Returns:
        - Total torque vector [τx, τy, τz] in Newton-meters
        """
        total_torque = np.zeros(3)
        
        for force, position in zip(forces, positions):
            total_torque += np.cross(position, force)
        
        return total_torque
  
    def telemetry(self, i, uav_id):
        must_track = self.current_time - self.last_time_track[i] >= self.refresh_rate
        
        if self.is_tracking[i] and must_track:
            # Update last_time_track
            self.last_time_track[i] = self.current_time

            # Get tracking information
            self.tracked_info[uav_id].append(Waypoint(
                t=self.current_time, 
                pos=self.pos, 
                vel=self.world_lin_vel)
            )

            if self.compare_controls[i]:
                self.track_z_ang_vel[i].append(self.local_ang_vel[2])
                self.track_roll[i].append(self.roll)
                self.track_pitch[i].append(self.pitch)
                self.track_servo_control_time[i].append(self.servo_total_time)
                self.track_cpu_usage[i].append(self.cpu_increment)
                self.track_mem_usage[i].append(self.mem_increment)
                self.track_gpu_usage[i].append(self.gpu_increment)

    def servo_control_track(self, i):
        must_track = self.current_time - self.last_time_track[i] >= self.refresh_rate

        if self.is_tracking[i] and must_track:
            before_gpu_usage = self.gpus[0].load * 100
            before_mem_usage = psutil.virtual_memory().percent
            before_cpu_usage = psutil.cpu_percent(interval=0.1)
            before_time = time.time()

        self.servo_control()

        if self.is_tracking[i] and must_track:
            after_time = time.time()
            after_cpu_usage = psutil.cpu_percent(interval=0.1)
            after_mem_usage = psutil.virtual_memory().percent
            after_gpu_usage = self.gpus[0].load * 100

            self.servo_total_time = after_time - before_time
            self.cpu_increment = after_cpu_usage - before_cpu_usage
            self.mem_increment = after_mem_usage - before_mem_usage
            self.gpu_increment = after_gpu_usage - before_gpu_usage

    def tracked_data_to_csv(self, i, uav_id):
        """This functions is in charge of creating a csv with all the data collected for the UAV control comparison
        
        Returns:
            .csv file with all the information needed
        """

        times = []
        error = []
        x_lin_vel = []
        y_lin_vel = []
        z_lin_vel = []

        for wp in self.tracked_info[uav_id]:
            times.append(wp.t)
            status = self.fp.status_at_time(wp.t)
            error.append(np.linalg.norm(wp.pos - status.pos))
            x_lin_vel.append(wp.vel[0])
            y_lin_vel.append(wp.vel[1])
            z_lin_vel.append(wp.vel[2])

        x_lin_acc = np.insert(np.diff(x_lin_vel), 0, 0)
        y_lin_acc = np.insert(np.diff(y_lin_vel), 0, 0)
        z_lin_acc = np.insert(np.diff(z_lin_vel), 0, 0)
        z_ang_acc = np.insert(np.diff(self.track_z_ang_vel[i]), 0, 0)

        data = np.column_stack((times, error, x_lin_acc, y_lin_acc, z_lin_acc, z_ang_acc, self.track_roll[i], 
                                self.track_pitch[i], self.track_servo_control_time[i], self.track_cpu_usage[i], 
                                self.track_mem_usage[i], self.track_gpu_usage[i]))
            
        path = project_root_path + "/sims/exported_data" + f"/{uav_id}_{self.completed_fps[i]}_modern_control.csv"
        np.savetxt(path, data, delimiter=", ", fmt="%s")

