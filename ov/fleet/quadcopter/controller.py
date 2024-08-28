from omni.kit.scripting import BehaviorScript
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.utils import rotations

from pxr import UsdGeom, Gf

from .msgs.flight_plan_msg import FlightPlanMsg
from .msgs.waypoint_msg import WaypointMsg
from .msgs.navigation_report_msg import NavigationReportMsg

import numpy as np

import math

class Quadcopter(BehaviorScript):
    def on_init(self):
        # Get rigid body prims
        self.drone_rbp = RigidPrimView("/quadcopter")
        self.prims_initialized = False

        # Get xformable from prim
        self.xform = UsdGeom.Xformable(self.prim)

        # Navigation parameters
        self.max_var_lin_vel = 5 # Maximum variation in linear  velocity   [  m/s]
        self.max_var_ang_vel = 2 # Maximum variation in angular velocity   [rad/s]
        self.target_step = 2 # Compute targetPos targetStep seconds later  [s]

        # Navigation command
        self.cmd_on = False
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_z = 0.0
        self.cmd_rot_z = 0.0

        # Navigation aux
        self.active_wp = -1

        # Quadcopter parameters
        g = 9.8
        mass = 0.595

        # Rotor speed (rad/s)
        self.w_rotor_NE : float
        self.w_rotor_NW : float
        self.w_rotor_SE : float
        self.w_rotor_SW : float

        # Linear & angular velocities
        self.linear_vel = Gf.Vec3d(0, 0, 0)
        self.angular_vel = Gf.Vec3d(0, 0, 0)

        # Pose variables
        self.position = Gf.Vec3d(0, 0, 0)
        self.orientation = Gf.Vec3d(0, 0, 0)
        self.orientation_qt = Gf.Vec4d(0, 0, 0, 0)

        # Angular speed limits
        self.w_max = 628.3185
        self.w_min = 0

        # Rotor position 15cm from the center of mass, with 45º of inclination respect to the axes
        self.pos_CM = np.array([0.0,     0.0,   0.0])
        self.pos_NE = np.array([0.075,  -0.075, 0.0])
        self.pos_NW = np.array([0.075,   0.075, 0.0])
        self.pos_SE = np.array([-0.075, -0.075, 0.0])
        self.pos_SW = np.array([-0.075,  0.075, 0.0])

        # Aero-dynamic thrust force constant
        self.kFT = 1.7179e-05
        self.w_hov = math.sqrt(mass * g / 4 / self.kFT)

        # Aero-dynamic drag force constant
        self.kMDR = 3.6714e-08

        # Aero-dynamic drag force constant per axis
        self.kFDx = 1.1902e-04
        self.kFDy = 1.1902e-04
        self.kFDz = 36.4437e-4

        # Aero-dynamic drag moment constant per axis
        self.kMDx = 1.1078e-04
        self.kMDy = 1.1078e-04
        self.kMDz = 7.8914e-05

        # Control matrices variables
        self.x = np.zeros((8, 1)) # Model state
        self.y = np.zeros((4, 1)) # Model output
        self.u = np.zeros((4, 1)) # Input (rotor speed)
        self.r = np.zeros((4, 1)) # Model reference
        self.e = np.zeros((4, 1)) # Model error
        self.E = np.zeros((4, 1)) # Model acumulated error

        # State control matrix
        self.Kx = [[-47.4820, -47.4820, -9.3626, -9.3626,  413.1508, -10.5091,  10.5091,  132.4440],
                   [ 47.4820, -47.4820,  9.3626, -9.3626, -413.1508, -10.5091, -10.5091,  132.4440],
                   [-47.4820,  47.4820, -9.3626,  9.3626, -413.1508,  10.5091,  10.5091,  132.4440],
                   [ 47.4820,  47.4820,  9.3626,  9.3626,  413.1508,  10.5091, -10.5091,  132.4440]]
        
        # Error control matrix
        self.Ky = [[-8.1889,  8.1889,  294.3201,  918.1130],
                   [-8.1889, -8.1889,  294.3201, -918.1130],
                   [ 8.1889,  8.1889,  294.3201, -918.1130],
                   [ 8.1889, -8.1889,  294.3201,  918.1130]]
        
        # Linearization point
        self.Hs = [self.w_hov, self.w_hov, self.w_hov, self.w_hov]

        # Misc
        self.E_max = 15 # Maximum acumulated error
        self.prev_control_time = 0 # Time of last low level control update
        self.rotors_on = False

        # Flight plan example
        self.fp = FlightPlanMsg()
        self.fp.plan_id = 0
        self.fp.uav_id = 0
        self.fp.operator_id = 0
        self.fp.radius = 2
        self.fp.priority = 0

        waypoint_data = [[10, 5, 0, 1],
                         [20, 5, 5, 1],   
                         [30, 0, 5, 1],  
                         [40, 0, 0, 1]]

        for data in waypoint_data:
            pos = Gf.Vec3d(data[1], data[2], data[3])
            vel = Gf.Vec3d(0, 0, 0)
            accel = Gf.Vec3d(0, 0, 0)
            jerk = Gf.Vec3d(0, 0, 0)
            snap = Gf.Vec3d(0, 0, 0)
            crkl = Gf.Vec3d(0, 0, 0)
            waypoint = WaypointMsg(pos, vel, accel, jerk, snap, crkl, data[0])
            self.fp.route.append(waypoint)

    def on_destroy(self):
        pass

    def on_play(self):
        pass

    def on_pause(self):
        pass

    def on_stop(self):
        self.prims_initialized = False

    def on_update(self, current_time: float, delta_time: float):
        # TEST
        self.navigation(current_time)
        self.servo_control(current_time, delta_time)
        self.platform_dynamics(current_time, delta_time)
    
    def navigation(self, time):
        if self.fp == None:
            return
        
        # Create navigation report message
        nav_rep = NavigationReportMsg(self.fp.plan_id, "Abejorro00", self.fp.operator_id, False, False, False, 0, time)
        
        # Check flightplan vigency
        wp_i = self.get_current_wp_index(time)

        # If flight plan is obsolete
        if self.active_wp == -1 and wp_i != 0:
            nav_rep.fp_aborted = True
            self.fp = None
            return

        # Current UAV status
        self.position, self.orientation_qt = self.drone_rbp.get_world_poses()

        # Transform orientation from quaternions to Euler angles
        self.orientation = rotations.quat_to_euler_angles(self.orientation_qt[0])

        # Get linear and angular velocity
        self.linear_vel = self.drone_rbp.get_linear_velocities()[0]
        self.angular_vel = self.drone_rbp.get_angular_velocities()[0]

        # Check if navigation status has changed
        if self.active_wp != wp_i:
            if wp_i == 0:
                wp_0 = self.fp.route[0]
                initial_position = Gf.Vec3d(wp_0.pos[0], wp_0.pos[1], wp_0.pos[2])

    # Get the index of the route list where the current waypoint is stored
    def get_current_wp_index(self, time: float) -> int:
        for i in range(len(self.fp.route)):
            if time < self.fp.route[i].time:
                break
        return i
    
    def get_wp_pos_at_time(self, time: float) -> Gf.Vec3d:
        wp_i = self.get_current_wp_index(time)

        # If the drone is on the first waypoint
        if wp_i == 0:
            return Gf.Vec3d(self.fp.route[0].pos[0], self.fp.route[0].pos[1], self.fp.route[0].pos[2])

        # If the drone has completed the flightplan
        elif wp_i == len(self.fp.route)-1:
             return Gf.Vec3d(self.fp.route[wp_i].pos[0], self.fp.route[wp_i].pos[1], self.fp.route[wp_i].pos[2])
        
        # If the drone is executing the flightplan
        else:
            wp = self.fp.route[wp_i]
            dt = time - wp.time
            r1 = Gf.Vec3d(wp.pos[0], wp.pos[1], wp.pos[2])
            v1 = Gf.Vec3d(wp.vel[0], wp.vel[1], wp.vel[2])
            a1 = Gf.Vec3d(wp.accel[0], wp.accel[1], wp.accel[2])
            j1 = Gf.Vec3d(wp.jerk[0], wp.jerk[1], wp.jerk[2])
            s1 = Gf.Vec3d(wp.snap[0], wp.snap[1], wp.snap[2])
            c1 = Gf.Vec3d(wp.crkl[0], wp.crkl[1], wp.crkl[2])

            return r1 + v1*dt + a1*pow(dt,2)/2 + j1*pow(dt,3)/6 + s1*pow(dt,4)/24 + c1*pow(dt,5)/120
        
    def get_wp_vel_at_time(self, time: float) -> Gf.Vec3d:
        wp_i = self.get_current_wp_index(time)

        # If the drone is on the first waypoint
        if wp_i == 0:
            return Gf.Vec3d(0, 0, 0)

        # If the drone has completed the flightplan
        elif wp_i == len(self.fp.route)-1:
             return Gf.Vec3d(0, 0, 0)
        
        # If the drone is executing the flightplan
        else:
            wp = self.fp.route[wp_i]
            dt = time - wp.time
            v1 = Gf.Vec3d(wp.vel[0], wp.vel[1], wp.vel[2])
            a1 = Gf.Vec3d(wp.accel[0], wp.accel[1], wp.accel[2])
            j1 = Gf.Vec3d(wp.jerk[0], wp.jerk[1], wp.jerk[2])
            s1 = Gf.Vec3d(wp.snap[0], wp.snap[1], wp.snap[2])
            c1 = Gf.Vec3d(wp.crkl[0], wp.crkl[1], wp.crkl[2])

            return v1 + a1*dt + j1*pow(dt,2)/2 + s1*pow(dt,3)/6 + c1*pow(dt,4)/24

    def command_off(self):
        self.cmd_on = False
        self.cmd_vel_x = 0
        self.cmd_vel_y = 0
        self.cmd_vel_z = 0
        self.cmd_rot_z = 0

    def hover(self):
        self.command_off()
        self.cmd_on = True

    def rotors_off(self):
        self.w_rotor_NE = 0
        self.w_rotor_NW = 0
        self.w_rotor_SE = 0
        self.w_rotor_SW = 0
        self.rotors_on = False

        # Control reset
        self.E = np.zeros((4, 1))

    def servo_control(self, current_time: float, delta_time: float):
        # Initialize rigid body prim
        if self.prims_initialized == False:
            self.drone_rbp.initialize()
            self.prims_initialized = True
            
        # Get position and orientation
        self.position, self.orientation_qt = self.drone_rbp.get_world_poses()

        # Transform orientation from quaternions to Euler angles
        self.orientation = rotations.quat_to_euler_angles(self.orientation_qt[0])

        # Get linear and angular velocity
        self.linear_vel = self.drone_rbp.get_linear_velocities()[0]
        self.angular_vel = self.drone_rbp.get_angular_velocities()[0]

        # Assign model state
        self.x[0, 0] = self.orientation[0] # ePhi
        self.x[1, 0] = self.orientation[1] # eTheta
        self.x[2, 0] = self.angular_vel[0] # bWx
        self.x[3, 0] = self.angular_vel[1] # bWy
        self.x[4, 0] = self.angular_vel[2] # bWz
        self.x[5, 0] = self.linear_vel[0] # bXdot
        self.x[6, 0] = self.linear_vel[1] # bYdot
        self.x[7, 0] = self.linear_vel[2] # bZdot

        # Assign model output
        self.y[0, 0] = self.linear_vel[0] # bXdot
        self.y[1, 0] = self.linear_vel[1] # bYdot
        self.y[2, 0] = self.linear_vel[2] # bZdot
        self.y[3, 0] = self.angular_vel[2] # bWz

        # Assign reference (m/s)
        self.r[0, 0] = 0
        self.r[1, 0] = 0
        self.r[2, 0] = 1
        self.r[3, 0] = 0

        # Error between output and reference
        self.e = self.y - self.r

        # Cumulative error
        self.E = self.E + self.e * delta_time

        # Error saturation
        if self.E[0, 0] > self.E_max : self.E[0, 0] = self.E_max
        if self.E[0, 0] < -self.E_max : self.E[0, 0] = -self.E_max

        if self.E[1, 0] > self.E_max : self.E[1, 0] = self.E_max
        if self.E[1, 0] < -self.E_max : self.E[1, 0] = -self.E_max

        if self.E[2, 0] > self.E_max : self.E[2, 0] = self.E_max
        if self.E[2, 0] < -self.E_max : self.E[2, 0] = -self.E_max

        if self.E[3, 0] > self.E_max : self.E[3, 0] = self.E_max
        if self.E[3, 0] < -self.E_max : self.E[3, 0] = -self.E_max

        # Dynamic system control
        self.u = self.Hs - self.Kx @ self.x - self.Ky @ self.E

        # Rotor speed saturation
        if self.u[0, 0] > self.w_max : self.u[0, 0] = self.w_max
        if self.u[0, 0] < self.w_min : self.u[0, 0] = self.w_min

        if self.u[1, 0] > self.w_max : self.u[1, 0] = self.w_max
        if self.u[1, 0] < self.w_min : self.u[1, 0] = self.w_min
        
        if self.u[2, 0] > self.w_max : self.u[2, 0] = self.w_max
        if self.u[2, 0] < self.w_min : self.u[2, 0] = self.w_min
        
        if self.u[3, 0] > self.w_max : self.u[3, 0] = self.w_max
        if self.u[3, 0] < self.w_min : self.u[3, 0] = self.w_min

        # Assign rotor speed
        self.w_rotor_NE = self.u[0, 0]
        self.w_rotor_NW = self.u[1, 0]
        self.w_rotor_SE = self.u[2, 0]
        self.w_rotor_SW = self.u[3, 0]

    def platform_dynamics(self, current_time: float, delta_time: float):
        # Apply thrust force
        thrust_rotor_NE = np.array([0.0, 0.0, self.kFT * math.pow(self.w_rotor_NE, 2)])
        thrust_rotor_NW = np.array([0.0, 0.0, self.kFT * math.pow(self.w_rotor_NW, 2)])
        thrust_rotor_SE = np.array([0.0, 0.0, self.kFT * math.pow(self.w_rotor_SE, 2)])
        thrust_rotor_SW = np.array([0.0, 0.0, self.kFT * math.pow(self.w_rotor_SW, 2)])

        self.drone_rbp.apply_forces_and_torques_at_pos(thrust_rotor_NE, positions=self.pos_NE, is_global=False)
        self.drone_rbp.apply_forces_and_torques_at_pos(thrust_rotor_NW, positions=self.pos_NW, is_global=False)
        self.drone_rbp.apply_forces_and_torques_at_pos(thrust_rotor_SE, positions=self.pos_SE, is_global=False)
        self.drone_rbp.apply_forces_and_torques_at_pos(thrust_rotor_SW, positions=self.pos_SW, is_global=False)

        # Apply drag moment
        drag_rotor_NE = np.array([0.0, 0.0, self.kMDR * math.pow(self.w_rotor_NE, 2)])
        drag_rotor_NW = np.array([0.0, 0.0, self.kMDR * math.pow(self.w_rotor_NW, 2)])
        drag_rotor_SE = np.array([0.0, 0.0, self.kMDR * math.pow(self.w_rotor_SE, 2)])
        drag_rotor_SW = np.array([0.0, 0.0, self.kMDR * math.pow(self.w_rotor_SW, 2)])

        drag_CM = drag_rotor_NE - drag_rotor_NW - drag_rotor_SE + drag_rotor_SW

        self.drone_rbp.apply_forces_and_torques_at_pos(torques=drag_CM, positions=self.pos_CM, is_global=False)

        # Apply air friction force
        friction_force_CM = np.array([-self.kFDx * self.linear_vel[0] * math.fabs(self.linear_vel[0]),
                                      -self.kFDy * self.linear_vel[1] * math.fabs(self.linear_vel[1]),
                                      -self.kFDz * self.linear_vel[2] * math.fabs(self.linear_vel[2])])
        
        self.drone_rbp.apply_forces_and_torques_at_pos(friction_force_CM, positions=self.pos_CM, is_global=False)

        # Apply air friction moment
        friction_moment_CM = np.array([-self.kMDx * self.angular_vel[0] * math.fabs(self.angular_vel[0]),
                                       -self.kMDy * self.angular_vel[1] * math.fabs(self.angular_vel[1]),
                                       -self.kMDz * self.angular_vel[2] * math.fabs(self.angular_vel[2])])
        
        self.drone_rbp.apply_forces_and_torques_at_pos(torques=friction_moment_CM, positions=self.pos_CM, 
                                                       is_global=False)