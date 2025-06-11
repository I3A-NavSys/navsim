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
        
        self.current_time = 0.0
        self.delta_time = 0.0
        self.w_max = 62.8319  # Maximum rotor speed
        self.w_min = 0.0      # Minimum rotor speed
        self.w_hov = 41.8879       # rad/s = 400rpm
        
        self.uav_amount = self.rigid_prim_view.count//5
        self.commands = {}
        self.current_wp = np.zeros(self.uav_amount)
        self.cmd_exp_time = np.zeros(self.uav_amount)
        
        self.tracked_info = {}
        self.is_tracking = np.zeros(self.uav_amount)
        self.refresh_rate = 1.0  # seconds
        self.last_time_track = np.zeros(self.uav_amount)

        self.completed_fps = np.zeros(self.uav_amount)
        self.compare_controls = np.zeros(self.uav_amount)
        self.track_z_ang_vel = np.zeros(self.uav_amount)
        self.track_roll = np.zeros(self.uav_amount)
        self.track_pitch = np.zeros(self.uav_amount)
        self.track_servo_control_time = np.zeros(self.uav_amount)
        self.track_cpu_usage = np.zeros(self.uav_amount)
        self.track_mem_usage = np.zeros(self.uav_amount)
        self.track_gpu_usage = np.zeros(self.uav_amount)

        self.kFT_N = 4.6544
        self.kFT_S = 0.9309
        self.kFDx = 3.0625
        self.kFDy = 4.0000
        self.kFDz = 7.8400
        self.kMDR_N = 5.9683
        self.kMDR_S = 1.4921
        self.kMDx = 37.4010
        self.kMDy = 25.8580
        self.kMDz = 20.2514
        self.Kx = np.array([        # state control matrix
            [ -14.6551,  -45.5032,   -4.1872,  -13.0009,    3.8871,   -6.6331,    2.1363,    6.4114 ],
            [  14.6551,  -45.5032,    4.1872,  -13.0009,   -3.8871,   -6.6331,   -2.1363,    6.4114 ],
            [ -58.6206,  227.5161,  -16.7487,   65.0046,  -24.4514,   33.1656,    8.5453,    6.4114 ],
            [  58.6206,  227.5161,   16.7487,   65.0046,   24.4514,   33.1656,   -8.5453,    6.4114 ]
        ])

        self.Ky = np.array([        # error control matrix
            [ -3.1839,    1.0254,    4.2743,    2.5914 ],
            [ -3.1839,   -1.0254,    4.2743,   -2.5914 ],
            [ 15.9195,    4.1017,    4.2743,  -16.3009 ],
            [ 15.9195,   -4.1017,    4.2743,   16.3009 ]           
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
        self.E_max = 150            # maximum model accumulated error

        self.forces = np.zeros((self.rigid_prim_view.count, 3))
        self.torques = np.zeros((self.rigid_prim_view.count, 3))

        for i, uav_id in enumerate(self.uavs.keys()):
            self.commands[uav_id] = Command()
            self.current_wp[i] = -1
            self.tracked_info[uav_id] = []

    def update(self, current_time, uavs, delta_time):
        self.current_time = current_time
        self.uavs = uavs
        self.delta_time = delta_time

        for i, (uav_id, uav) in enumerate(self.uavs.items()):
            self.imu(i)
            self.navigation(i, uav_id, uav)
            self.servo_control(i, uav_id)
            self.telemetry(i, uav_id)
            self.platform_dynamics(i)
        self.apply_physics()

    def inform_operator(self, id, state, time, pos, flightplan, tracked_info, is_request_completed=False):
        serialized_fp = base64.b64encode(pickle.dumps(flightplan)).decode('utf-8')
        serialized_pos = base64.b64encode(pickle.dumps(pos)).decode('utf-8')
        if is_request_completed:
            tracked_info = base64.b64encode(pickle.dumps(np.array(tracked_info))).decode('utf-8')

        payload = {
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
        self.w_rotor_NW = 0.0
        self.w_rotor_NE = 0.0
        self.w_rotor_SW = 0.0
        self.w_rotor_SE = 0.0

    def imu(self, i):
        self.pos, self.ori = self.rigid_prim_view.get_world_poses(indices=[i])
        self.pos = self.pos[0]
        self.ori = self.ori[0]
        self.rot = Rotation.from_quat([self.ori[1], self.ori[2], self.ori[3], self.ori[0]])
        self.roll, self.pitch, self.yaw = self.rot.as_euler('xyz', degrees=False)
        self.lin_vel = self.rigid_prim_view.get_linear_velocities(indices=[i])[0]
        self.ang_vel = self.rigid_prim_view.get_angular_velocities(indices=[i])[0]

        self.lin_vel = self.rot.inv().apply(self.lin_vel)
        self.ang_vel = self.rot.inv().apply(self.ang_vel)

    def navigation(self, i, uav_id, uav):
        has_flightplan = uav["flightplan"] is not None
        
        if has_flightplan:
            uav_flightplan = uav["flightplan"]
            WP = uav_flightplan.get_target_index_from_time(self.current_time)
            num_wps = len(uav_flightplan.waypoints)

            is_flightplan_obsolete = self.current_wp[i] == -1 and WP != 0
            has_status_changed = self.current_wp[i] != WP

            if is_flightplan_obsolete:
                print(f"[{self.current_time:3.2f}] {uav_id}: discarding FP due to it is obsolete")
                uav["flightplan"] = None
                return

            if has_status_changed:
                is_flightplan_start = WP == 0
                is_flightplan_in_progress = WP < num_wps

                if is_flightplan_start:
                    is_well_positioned = np.linalg.norm(self.pos - uav_flightplan.waypoints[0].pos) < uav_flightplan.radius

                    if is_well_positioned:
                        print(f"[{self.current_time:3.2f}] {uav_id}: waiting to start the flightplan")
                        self.inform_operator(uav_id, UAVState.BUSY, self.current_time, self.pos, uav_flightplan, "")

                    else:
                        print(f"[{self.current_time:3.2f}] {uav_id}: discarding FP due to an incorrect starting position")
                        uav["flightplan"] = None
                        return

                elif is_flightplan_in_progress:
                    self.is_tracking[i] = True
                    has_waypoint_label = uav_flightplan.waypoints[WP].label != ""
                    waypoint = uav_flightplan.waypoints[WP].label if has_waypoint_label else WP

                    print(f"[{self.current_time:3.2f}] {uav_id}: flying to {waypoint} waypoint")
                    self.inform_operator(uav_id, UAVState.BUSY, self.current_time, self.pos, uav_flightplan, "")

                else:
                    print(f"[{self.current_time:3.2f}] {uav_id}: flight plan completed")
                    self.inform_operator(uav_id, UAVState.IDLE, self.current_time, self.pos, 
                                         uav_flightplan, self.tracked_info[uav_id], is_request_completed=True)
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
            self.current_wp[i] = WP
            abs_lin_vel = self.rot.apply(self.lin_vel)

            is_flightplan_start = WP == 0
            if is_flightplan_start: heading = uav_flightplan.waypoints[0].heading
            else:                   heading = uav_flightplan.waypoints[WP - 1].heading

            self.commands[uav_id] = uav_flightplan.get_command(self.current_time, self.pos, 
                                                               abs_lin_vel, self.rot, heading, 2)
            self.cmd_exp_time[i] = self.current_time + self.commands[uav_id].duration

    def servo_control(self, i, uav_id):
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
        # print(f"r: {np.round(self.r.T, 2)}")

        # Assign model state
        self.x[i, 0, 0] = self.roll           # ePhi
        self.x[i, 1, 0] = self.pitch          # eTheta
        self.x[i, 2, 0] = self.ang_vel[0] # bWx
        self.x[i, 3, 0] = self.ang_vel[1] # bWy
        self.x[i, 4, 0] = self.ang_vel[2] # bWz
        self.x[i, 5, 0] = self.lin_vel[0]  # bXdot
        self.x[i, 6, 0] = self.lin_vel[1]  # bYdot
        self.x[i, 7, 0] = self.lin_vel[2]  # bZdot
        # print(f"x: {np.round(self.x.T, 2)}")

        # Assign model output
        self.y[i, 0, 0] = self.x[i, 5, 0]        # bXdot
        self.y[i, 1, 0] = self.x[i, 6, 0]        # bYdot
        self.y[i, 2, 0] = self.x[i, 7, 0]        # bZdot
        self.y[i, 3, 0] = self.x[i, 4, 0]        # bWz
        # print(f"y: {np.round(self.y.T, 2)}")

        # Error between the output and the reference 
        # (between the commanded velocity and the drone velocity)
        self.e[i] = self.y[i] - self.r[i]
        # print(f"e: {np.round(self.e.T, 2)}")

        # Cumulative error
        self.E[i] = self.E[i] + (self.e[i] * self.delta_time)
        # print(f"E: {np.round(self.E.T, 2)}")

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
        # print(f"u: {np.round(self.u.T, 2)}")

        # Rotor speed saturation
        if self.u[i, 0, 0] > self.w_max : self.u[i, 0, 0] = self.w_max
        if self.u[i, 0, 0] < self.w_min : self.u[i, 0, 0] = self.w_min
        if self.u[i, 1, 0] > self.w_max : self.u[i, 1, 0] = self.w_max
        if self.u[i, 1, 0] < self.w_min : self.u[i, 1, 0] = self.w_min
        if self.u[i, 2, 0] > self.w_max : self.u[i, 2, 0] = self.w_max
        if self.u[i, 2, 0] < self.w_min : self.u[i, 2, 0] = self.w_min
        if self.u[i, 3, 0] > self.w_max : self.u[i, 3, 0] = self.w_max
        if self.u[i, 3, 0] < self.w_min : self.u[i, 3, 0] = self.w_min

        # Assign rotor speed
        self.w_rotor_NE = self.u[i, 0, 0]
        self.w_rotor_NW = self.u[i, 1, 0]
        self.w_rotor_SE = self.u[i, 2, 0]
        self.w_rotor_SW = self.u[i, 3, 0]

    def platform_dynamics(self, i):
        # Compute thrust forces
        ft_nw = np.array([0, 0, self.kFT_N * self.w_rotor_NW**2])
        ft_ne = np.array([0, 0, self.kFT_N * self.w_rotor_NE**2])
        ft_sw = np.array([0, 0, self.kFT_S * self.w_rotor_SW**2])
        ft_se = np.array([0, 0, self.kFT_S * self.w_rotor_SE**2])
        
        # Compute drag forces
        fd = np.array([
            -self.kFDx * self.lin_vel[0] * abs(self.lin_vel[0]),
            -self.kFDy * self.lin_vel[1] * abs(self.lin_vel[1]),
            -self.kFDz * self.lin_vel[2] * abs(self.lin_vel[2])
        ])
        
        # Compute drag moments
        mdr_nw = self.kMDR_N * self.w_rotor_NW**2
        mdr_ne = self.kMDR_N * self.w_rotor_NE**2
        mdr_sw = self.kMDR_S * self.w_rotor_SW**2
        mdr_se = self.kMDR_S * self.w_rotor_SE**2
        mdr = np.array([0, 0, mdr_ne - mdr_nw - mdr_se + mdr_sw])
        
        # Compute friction moments
        md = np.array([
            -self.kMDx * self.ang_vel[0] * abs(self.ang_vel[0]),
            -self.kMDy * self.ang_vel[1] * abs(self.ang_vel[1]),
            -self.kMDz * self.ang_vel[2] * abs(self.ang_vel[2])
        ])

        self.forces[i] = fd
        self.forces[i + self.uav_amount] = ft_nw
        self.forces[i + self.uav_amount * 2] = ft_ne
        self.forces[i + self.uav_amount * 3] = ft_sw
        self.forces[i + self.uav_amount * 4] = ft_se
        self.torques[i] = mdr + md

    def telemetry(self, i, uav_id):
        # Update every self.refresh_rate seconds
        if self.is_tracking[i] and self.current_time - self.last_time_track[i] >= self.refresh_rate:
            # Update last_time_track
            self.last_time_track[i] = self.current_time

            # Change relative vel to absolute
            linear_vel = self.rot.apply(self.lin_vel)

            # Get tracking information
            self.tracked_info[uav_id].append(Waypoint(t= self.current_time, pos=self.pos, vel=linear_vel))

            if self.compare_controls[i]:
                self.track_z_ang_vel[i].append(self.ang_vel[2])
                self.track_roll[i].append(self.roll)
                self.track_pitch[i].append(self.pitch)
                self.track_servo_control_time[i].append(self.servo_total_time)
                self.track_cpu_usage[i].append(self.cpu_increment)
                self.track_mem_usage[i].append(self.mem_increment)
                self.track_gpu_usage[i].append(self.gpu_increment)

    def servo_control_track(self, i):
        if self.is_tracking[i] and self.current_time - self.last_time_track[i] >= self.refresh_rate:
            before_gpu_usage = GPUtil.getGPUs()[0].load * 100
            before_mem_usage = psutil.virtual_memory().percent
            before_cpu_usage = psutil.cpu_percent(interval=0.1)
            before_time = time.time()

        self.servo_control()

        if self.is_tracking[i] and self.current_time - self.last_time_track[i] >= self.refresh_rate:
            after_time = time.time()
            after_cpu_usage = psutil.cpu_percent(interval=0.1)
            after_mem_usage = psutil.virtual_memory().percent
            after_gpu_usage = GPUtil.getGPUs()[0].load * 100

            self.servo_total_time = after_time - before_time
            self.cpu_increment = after_cpu_usage - before_cpu_usage
            self.mem_increment = after_mem_usage - before_mem_usage
            self.gpu_increment = after_gpu_usage - before_gpu_usage

    def tracked_data_to_csv(self, i, uav_id):
        """This functions is in charge of creating a csv with all the data collected for the UAV control comparison
        
        Output:
        - .csv file with all the information needed
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

    def apply_physics(self):
        self.rigid_prim_view.apply_forces_and_torques_at_pos(
            forces=self.forces,
            torques=self.torques,
            is_global=False
        )
