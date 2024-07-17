from omni.kit.scripting import BehaviorScript
from omni.isaac.dynamic_control import _dynamic_control
from pxr import UsdGeom, Gf

import numpy as np

import math

class UamMinidrone(BehaviorScript):
    def on_init(self):
        # Get dynamic control interface
        self.dc = _dynamic_control.acquire_dynamic_control_interface()

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
        self.pos_CM = [   0.0,    0.0, 0.0]
        self.pos_NE = [ 0.075, -0.075, 0.0]
        self.pos_NW = [ 0.075,  0.075, 0.0]
        self.pos_SE = [-0.075, -0.075, 0.0]
        self.pos_SW = [-0.075,  0.075, 0.0]

        # Aero-dynamic thrust force constant
        self.kFT = 1.7179e-05
        self.w_hov = math.sqrt(mass * g / 4 / self.kFT)

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
        pass

    def on_update(self, current_time: float, delta_time: float):
        # Get pose
        drone = self.dc.get_rigid_body("/abejorro")
        pose = self.dc.get_rigid_body_pose(drone)

        # Get linear and angular velocity
        linear_vel = self.prim.GetAttribute("physics:velocity").Get()
        angular_vel = self.prim.GetAttribute("physics:angularVelocity").Get()

        # Assign model state
        self.x[0, 0] = pose.r[0] # ePhi
        self.x[1, 0] = pose.r[1] # eTheta
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

        # Assign reference
        self.r[0, 0] = 1
        self.r[1, 0] = 0
        self.r[2, 0] = 0
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

    def rotors_off(self):
        # Turn off rotors
        self.w_rotor_NE = 0
        self.w_rotor_NW = 0
        self.w_rotor_SE = 0
        self.w_rotor_SW = 0
        self.rotors_on = False

        # Reset control
        self.E = [0, 0, 0, 0]