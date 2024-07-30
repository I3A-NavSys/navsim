from omni.kit.scripting import BehaviorScript
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.utils import rotations

from pxr import UsdGeom, Gf

import numpy as np

import math

import carb

class UamMinidrone(BehaviorScript):
    def on_init(self):
        # Get rigid body prims
        self.drone_rbp = RigidPrimView("/abejorro")
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

        # Quadcopter parameters
        g = 9.8
        mass = 0.595

        # Rotor speed (rad/s)
        self.w_rotor_NE : float
        self.w_rotor_NW : float
        self.w_rotor_SE : float
        self.w_rotor_SW : float

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

    def on_destroy(self):
        pass

    def on_play(self):
        pass

    def on_pause(self):
        pass

    def on_stop(self):
        self.prims_initialized = False

    def on_update(self, current_time: float, delta_time: float):
        # Initialize rigid body prim
        if self.prims_initialized == False:
            self.drone_rbp.initialize()
            self.prims_initialized = True
            
        # Get position and orientation
        position, orientation = self.drone_rbp.get_world_poses()

        # Transform orientation from quaternions to Euler angles
        orientation = rotations.quat_to_euler_angles(orientation[0])

        # Get linear and angular velocity
        linear_vel = self.drone_rbp.get_linear_velocities()[0]
        angular_vel = self.drone_rbp.get_angular_velocities()[0]

        # Assign model state
        self.x[0, 0] = orientation[0] # ePhi
        self.x[1, 0] = orientation[1] # eTheta
        self.x[2, 0] = angular_vel[0] # bWx
        self.x[3, 0] = angular_vel[1] # bWy
        self.x[4, 0] = angular_vel[2] # bWz
        self.x[5, 0] = linear_vel[0] # bXdot
        self.x[6, 0] = linear_vel[1] # bYdot
        self.x[7, 0] = linear_vel[2] # bZdot

        # Assign model output
        self.y[0, 0] = linear_vel[0] # bXdot
        self.y[1, 0] = linear_vel[1] # bYdot
        self.y[2, 0] = linear_vel[2] # bZdot
        self.y[3, 0] = angular_vel[2] # bWz

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

        self.drone_rbp.apply_forces_and_torques_at_pos(drag_rotor_NE, positions=self.pos_NE, is_global=False)
        self.drone_rbp.apply_forces_and_torques_at_pos(drag_rotor_NW, positions=self.pos_NW, is_global=False)
        self.drone_rbp.apply_forces_and_torques_at_pos(drag_rotor_SE, positions=self.pos_SE, is_global=False)
        self.drone_rbp.apply_forces_and_torques_at_pos(drag_rotor_SW, positions=self.pos_SW, is_global=False)

        # Apply air friction force
        friction_force_CM = np.array([-self.kFDx * linear_vel[0] * math.fabs(linear_vel[0]),
                                      -self.kFDy * linear_vel[1] * math.fabs(linear_vel[1]),
                                      -self.kFDz * linear_vel[2] * math.fabs(linear_vel[2])])
        
        self.drone_rbp.apply_forces_and_torques_at_pos(friction_force_CM, positions=self.pos_CM, is_global=False)

        # Apply air friction moment
        friction_moment_CM = np.array([-self.kMDx * angular_vel[0] * math.fabs(angular_vel[0]),
                                       -self.kMDy * angular_vel[1] * math.fabs(angular_vel[1]),
                                       -self.kMDz * angular_vel[2] * math.fabs(angular_vel[2])])
        
        self.drone_rbp.apply_forces_and_torques_at_pos(friction_moment_CM, positions=self.pos_CM, is_global=False)
        
    def rotors_off(self):
        # Turn off rotors
        self.w_rotor_NE = 0
        self.w_rotor_NW = 0
        self.w_rotor_SE = 0
        self.w_rotor_SW = 0
        self.rotors_on = False

        # Reset control
        self.E = [0, 0, 0, 0]