import numpy as np
import omni.physx


from omni.kit.scripting import BehaviorScript
from omni.isaac.core.prims import RigidPrimView
import omni.physics.tensors as omni_tensors
from scipy.spatial.transform import Rotation


class FuerzasAv(BehaviorScript):
    def on_init(self):
        self.init_control()

        self.physx_interface = omni.physx.get_physx_interface()
        self.physx_interface_sub = self.physx_interface.subscribe_physics_on_step_events(self.on_physics_step, True, 0)

        self.view_created = False
        self.delta_time = 0.02
        self.inidividual_pos = np.array([
            [0, 0, 0],
            [0.5, 1.95, 0.5],
            [0.5, -1.95, 0.5],
            [-2.5, 1.55, 0.5],
            [-2.5, -1.55, 0.5]
        ])

    def on_destroy(self):
        self.physx_interface_sub = None

    def on_play(self):
        self.rigid_prim_view = RigidPrimView(
            ["/World/UAVs/UAV_*"],
            reset_xform_properties=False)
        
        
        if not self.view_created:
            self.view_created = True

            self.rigid_prim_view.initialize()

            self.individual_forces = np.zeros((5, 3))
            self.forces = np.zeros((3))
            self.torques = np.zeros((3))
            self.command = np.zeros((4))

    def on_pause(self):
        pass

    def on_stop(self):
        self.view_created = False

    def on_update(self, current_time: float, delta_time: float):
        self.current_time = current_time
        # self.delta_time = delta_time

        if self.current_time >= 5:
            self.command[2] = 1

        if self.current_time >= 10:
            self.command[0] = 1
            self.command[2] = 0

        if self.current_time >= 15:
            self.command[2] = 1
            self.command[3] = 0.5

    def on_physics_step(self, dt):
        self.delta_time = dt

        if self.view_created:
            self.imu()
            self.servo_control()
            self.dynamics()

            self.rigid_prim_view.apply_forces_and_torques_at_pos(
                forces=self.forces,
                torques=self.torques,
                # torques=None,
                is_global=False
            )

    def imu(self):
        self.pos, self.ori = self.rigid_prim_view.get_world_poses()
        self.pos = self.pos[0]
        self.ori = self.ori[0]
        self.rot = Rotation.from_quat([self.ori[1], self.ori[2], self.ori[3], self.ori[0]])
        self.roll, self.pitch, self.yaw = self.rot.as_euler('xyz', degrees=False)
        self.linear_vel = self.rigid_prim_view.get_linear_velocities()[0]
        self.angular_vel = self.rigid_prim_view.get_angular_velocities()[0]

        self.linear_vel = self.rot.inv().apply(self.linear_vel)
        self.angular_vel = self.rot.inv().apply(self.angular_vel)

        print()
        print(f"[{self.current_time}] - pos: {np.round(self.pos, 2)}")
        # print(f"ori: {np.round(self.ori, 2)}")
        print(f"[{self.current_time}] - roll: {np.round(self.roll, 2)}")
        print(f"[{self.current_time}] - itch: {np.round(self.pitch, 2)}")
        print(f"[{self.current_time}] - yaw: {np.round(self.yaw, 2)}")
        print(f"[{self.current_time}] - linear_vel: {np.round(self.linear_vel, 2)}")
        print(f"[{self.current_time}] - angular_vel: {np.round(self.angular_vel, 2)}")

    def servo_control(self):

        # Assign the model reference to be followed
        self.r[0, 0] = self.command[0]       # bXdot
        self.r[1, 0] = self.command[1]       # bYdot
        self.r[2, 0] = self.command[2]       # bZdot
        self.r[3, 0] = self.command[3]       # hZdot
        # print(f"r: {np.round(self.r.T, 2)}")

        # Assign model state
        self.x[0, 0] = self.roll           # ePhi
        self.x[1, 0] = self.pitch          # eTheta
        self.x[2, 0] = self.angular_vel[0] # bWx
        self.x[3, 0] = self.angular_vel[1] # bWy
        self.x[4, 0] = self.angular_vel[2] # bWz
        self.x[5, 0] = self.linear_vel[0]  # bXdot
        self.x[6, 0] = self.linear_vel[1]  # bYdot
        self.x[7, 0] = self.linear_vel[2]  # bZdot
        # print(f"x: {np.round(self.x.T, 2)}")

        # Assign model output
        self.y[0, 0] = self.x[5, 0]        # bXdot
        self.y[1, 0] = self.x[6, 0]        # bYdot
        self.y[2, 0] = self.x[7, 0]        # bZdot
        self.y[3, 0] = self.x[4, 0]        # bWz
        # print(f"y: {np.round(self.y.T, 2)}")

        # Error between the output and the reference 
        # (between the commanded velocity and the drone velocity)
        self.e = self.y - self.r
        # print(f"e: {np.round(self.e.T, 2)}")

        # Cumulative error
        self.E = self.E + (self.e * self.delta_time)
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
        self.u = self.Hs - self.Kx @ self.x - self.Ky @ self.E
        # print(f"u: {np.round(self.u.T, 2)}")

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

    def dynamics(self):
        FT_NE = np.array([0, 0, self.kFT_N * self.w_rotor_NE**2])
        FT_NW = np.array([0, 0, self.kFT_N * self.w_rotor_NW**2])
        FT_SE = np.array([0, 0, self.kFT_S * self.w_rotor_SE**2])
        FT_SW = np.array([0, 0, self.kFT_S * self.w_rotor_SW**2])

        # Apply the air friction force to the drone
        FD = np.array([
            -self.kFDx * self.linear_vel[0] * abs(self.linear_vel[0]),
            -self.kFDy * self.linear_vel[1] * abs(self.linear_vel[1]),
            -self.kFDz * self.linear_vel[2] * abs(self.linear_vel[2])])
      
        # Compute the drag moment
        MDR_NE = self.kMDR_N * self.w_rotor_NE**2
        MDR_NW = self.kMDR_N * self.w_rotor_NW**2
        MDR_SE = self.kMDR_S * self.w_rotor_SE**2
        MDR_SW = self.kMDR_S * self.w_rotor_SW**2
        MDR = np.array([0, 0, MDR_NE - MDR_NW - MDR_SE + MDR_SW])

        # Compute the air friction moment
        MD = np.array([
            -self.kMDx * self.angular_vel[0] * abs(self.angular_vel[0]),
            -self.kMDy * self.angular_vel[1] * abs(self.angular_vel[1]),
            -self.kMDz * self.angular_vel[2] * abs(self.angular_vel[2])])
        
        self.individual_forces[0] = FD
        self.individual_forces[1] = FT_NW
        self.individual_forces[2] = FT_NE
        self.individual_forces[3] = FT_SW
        self.individual_forces[4] = FT_SE

        self.forces = np.sum(self.individual_forces, axis=0)
        self.torques = MDR + MD + self.compute_torques(self.individual_forces, self.inidividual_pos)
        # self.torques = self.compute_torques(self.individual_forces, self.inidividual_pos)

        # print(f"i: {i}, forces: {np.round(self.forces[i], 2)}, torques: {np.round(self.torques[i], 2)}")
        # print()
        # print(f"forces: {np.round(self.forces, 2)}")
        # print(f"torques: {np.round(self.torques, 2)}")

    def compute_torques(self, forces, positions):
        """
        Compute torques about each axis (X, Y, Z) given forces applied at offset positions.
        
        Parameters:
        - forces: List of force vectors [Fx, Fy, Fz] in Newtons
        - positions: List of position vectors [x, y, z] in meters relative to COM
        
        Returns:
        - Total torque vector [τx, τy, τz] in Newton-meters
        """
        total_torque = np.zeros(3)
        
        for force, position in zip(forces, positions):
            # Convert to numpy arrays for vector operations
            # f = np.array(force)
            # r = np.array(position)
            
            # Compute cross product (r × F)
            # torque = np.cross(r, f)
            torque = np.cross(position, force)
            
            # Add to total torque
            total_torque += torque
        
        return total_torque

    def init_control(self):
        self.w_rotor_NE = 0.0
        self.w_rotor_NW = 0.0
        self.w_rotor_SE = 0.0
        self.w_rotor_SW = 0.0
    
        # Max and minimum angular velocity of the motors
        self.w_max = 62.8319       # rad/s = 600rpm
        self.w_min = 0             # rad/s =   0rpm

        # Aerodynamic thrust force constant
        # Force generated by the rotors is FT = kFT * w²
        self.kFT_N =  4.6544       # north side
        self.kFT_S =  0.9309       # south side
        self.w_hov = 41.8879       # rad/s = 400rpm

        # Aerodynamic drag force constant
        # Moment generated by the rotors is MDR = kMDR * w²
        self.kMDR_N = 5.9683
        self.kMDR_S = 1.4921

        # Aerodynamic drag force constant per axis
        # Drag force generated by the air friction, opposite to the velocity is FD = -kFD * r_dot*|r_dot| (depends on 
        # the shape of the object in each axis).
        # Horizontal axis:
        self.kFDx = 3.0625
        self.kFDy = 4.0000
        # Vertical axis:
        self.kFDz = 7.8400

        # Aerodynamic drag moment constant per axis
        # Drag moment generated by the air friction, opposite to the angular velocity is MD = -kMD * rpy_dot*|rpy_dot| 
        # (depends on the shape of the object in each axis).
        self.kMDx = 37.4010
        self.kMDy = 25.8580
        self.kMDz = 20.2514

        #--------------------------------------------------------------------------------------------------------------
        # LOW LEVEL CONTROL

        self.x  = np.zeros((8, 1))  # model state
        self.y  = np.zeros((4, 1))  # model output
        self.u  = np.zeros((4, 1))  # input (rotors speeds)
        self.r  = np.zeros((4, 1))  # model reference
        self.e  = np.zeros((4, 1))  # model error
        self.E  = np.zeros((4, 1))  # model accumulated error
        
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
        
        self.E_max = 150