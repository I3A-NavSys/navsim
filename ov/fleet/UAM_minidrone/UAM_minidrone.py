# Description: This script is used to control the minidrone in the UAM environment.

from omni.kit.scripting import BehaviorScript
from pxr import Sdf
# from omni.isaac.core.prims import RigidPrimView
from pxr import Gf
# from pxr import Usd, UsdGeom, Gf

import numpy as np
# import math
from scipy.spatial.transform import Rotation

import carb.events
import omni.kit.app


class UAM_minidrone(BehaviorScript):


    def on_init(self):
        print(f"\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        print(f"INIT    {self.prim_path}")

        ########################################################################
        ## Omniverse

        self.current_time = 0
        self.delta_time = 0

        self.pos_atr    = self.prim.GetAttribute("xformOp:translate")
        self.ori_atr    = self.prim.GetAttribute("xformOp:orient")

        self.linVel_atr = self.prim.GetAttribute("physics:velocity")
        self.angVel_atr = self.prim.GetAttribute("physics:angularVelocity")

        self.force_atr = self.prim.GetAttribute("physxForce:force")
        self.force_atr.Set(Gf.Vec3f(0,0,0))
        self.torque_atr = self.prim.GetAttribute("physxForce:torque")
        self.torque_atr.Set(Gf.Vec3f(0,0,0))

        primNE = self.prim.GetChild("rotor_NE")
        self.forceNE_atr = primNE.CreateAttribute("physxForce:force", Sdf.ValueTypeNames.Float3)
        self.forceNE_atr.Set(Gf.Vec3f(0,0,0))

        primNW = self.prim.GetChild("rotor_NW")
        self.forceNW_atr = primNW.CreateAttribute("physxForce:force", Sdf.ValueTypeNames.Float3)
        self.forceNW_atr.Set(Gf.Vec3f(0,0,0))

        primSE = self.prim.GetChild("rotor_SE")
        self.forceSE_atr = primSE.CreateAttribute("physxForce:force", Sdf.ValueTypeNames.Float3)
        self.forceSE_atr.Set(Gf.Vec3f(0,0,0))

        primSW = self.prim.GetChild("rotor_SW")
        self.forceSW_atr = primSW.CreateAttribute("physxForce:force", Sdf.ValueTypeNames.Float3)
        self.forceSW_atr.Set(Gf.Vec3f(0,0,0))        
        
        self.primRotStatic = self.prim.GetChild("rotors_spinning")

        # Create the omniverse event associated to this UAV
        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(self.prim.GetPath()))
        bus = omni.kit.app.get_app().get_message_bus_event_stream()
        self.eventSub = bus.create_subscription_to_push_by_type(self.UAV_EVENT, self.push_subscripted_event_method)



        ########################################################################
        ## Navigation parameters

        # AutoPilot navigation command
        self.cmd_on = False          # (bool) motores activos 
        self.cmd_velX = 0.0          # (m/s)  velocidad lineal  deseada en eje X
        self.cmd_velY = 0.0          # (m/s)  velocidad lineal  deseada en eje Y
        self.cmd_velZ = 0.0          # (m/s)  velocidad lineal  deseada en eje Z
        self.cmd_rotZ = 0.0          # (m/s)  velocidad angular deseada en eje Z
        self.cmd_exp_time = None     # (s)    tiempo de expiracion del comando


        ########################################################################
        ## quadcopter parameters

        self.g    = 9.81

        mass_attr = self.prim.GetAttribute("physics:mass")
        self.mass = mass_attr.Get()
        self.mass = 0.595
        # print(f"mass: {self.mass}")

        inertia_attr = self.prim.GetAttribute("physics:diagonalInertia")
        self.inertia = inertia_attr.Get()
        # print(f"inertia: {self.inertia}")

        self.pos   = Gf.Vec3f(0, 0, 0)
        self.roll  = 0
        self.pitch = 0
        self.yaw   = 0
        self.linear_vel  = Gf.Vec3f(0, 0, 0)
        self.angular_vel = Gf.Vec3f(0, 0, 0)

        # Rotor speed (rad/s)
        # asumimos velocidad máxima de 6000rpm = 628.3185 rad/s
        self.w_rotor_NE = 0.0
        self.w_rotor_NW = 0.0
        self.w_rotor_SE = 0.0
        self.w_rotor_SW = 0.0
    
        # Max and minimum angular velocity of the motors
        self.w_max = 628.3185      # rad/s = 15000rpm
        self.w_min = 0             # rad/s =     0rpm

        # Aerodynamic thrust force constant
        # Force generated by the rotors is FT = kFT * w²
        self.kFT = 1.7197e-05                                    # assuming that FT_max = 0.692kg
        self.w_hov = (self.mass * self.g / 4 / self.kFT) ** 0.5  # 291.45 rad/s

        # Aerodynamic drag force constant
        # Moment generated by the rotors is MDR = kMDR * w²
        self.kMDR = 3.6714e-08

        # Aerodynamic drag force constant per axis
        # Drag force generated by the air friction, opposite to the velocity is FD = -kFD * r_dot*|r_dot| (depends on the shape of the object in each axis).
        # Horizontal axis:
        self.kFDx = 1.1902e-04
        self.kFDy = 1.1902e-04
        # Vertical axis:
        self.kFDz = 36.4437e-4

        # Aerodynamic drag moment constant per axis
        # Drag moment generated by the air friction, opposite to the angular velocity is MD = -kMD * rpy_dot*|rpy_dot| (depends on the shape of the object in each axis).

        # Horizontal axis:
        # Assuming similar drag in both axes (although the fuselage is not equal), no gravity and the drone is propulsed by two rotors of the same side at maximum speed the maximum angular velocity is Vrp_max = 2 * 2*pi;
        # operating kMDxy =  2 * FT_max * sin(deg2rad(45))^2 / Vrp_max^2 we get that...
        self.kMDx = 1.1078e-04
        self.kMDy = 1.1078e-04

        # Vertical axis:
        # Must be verified at maximum yaw velocity
        # Assuming that Vyaw_max = 4*pi rad/s (max yaw velocity of 2rev/s) and w_hov2 (rotor speed to maintain the hovering)
        # And taking into account that MDR = kMDR * w² and MDz = kMDz * Vyaw²
        # operating MDz  = MDR (the air friction compensates the effect of the rotors) and kMDz = kMDR* (2 * w_hov2²) / Vyaw_max² we get that...
        self.kMDz = 7.8914e-05


        ########################################################################
        # low-level control

        self.x  = np.zeros((8, 1))  # model state
        self.y  = np.zeros((4, 1))  # model output
        self.u  = np.zeros((4, 1))  # input (rotors speeds)
        self.r  = np.zeros((4, 1))  # model reference
        self.e  = np.zeros((4, 1))  # model error
        self.E  = np.zeros((4, 1))  # model accumulated error
        
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
        
        self.E_max = 15  # previous value 5, maximum model accumulated error




########################################################################
# Event handlers



    def on_play(self):
        print(f"PLAY    {self.prim_path}")
        # print(f"\t {self.current_time} \t {self.delta_time}")

        # Asumimos un comando ---------------------------------------TEMPORAL
        self.hover()
        # self.cmd_on = True
        # self.cmd_velX = 0.0
        # self.cmd_velY = 0.0
        # self.cmd_velZ = 0.0
        # self.cmd_rotZ = 0.0
        # print(f"UAV command: ON: {self.cmd_on:.0f} velX: {self.cmd_velX:.1f} velY: {self.cmd_velY:.1f} velZ: {self.cmd_velZ:.1f} rotZ: {self.cmd_rotZ:.1f}")

        # Update the drone status
        self.IMU()
        self.servo_control()
        self.platform_dynamics()
        pass


    
    def on_pause(self):
        print(f"PAUSE   {self.prim_path}")



    def on_stop(self):
        print(f"STOP    {self.prim_path}")

        self.current_time = 0
        self.delta_time = 0

        self.command_off()
        self.rotors_off()

        self.force_atr.Set(Gf.Vec3f(0,0,0))
        self.torque_atr.Set(Gf.Vec3f(0,0,0))
        self.forceNE_atr.Set(Gf.Vec3f(0,0,0))
        self.forceNW_atr.Set(Gf.Vec3f(0,0,0))
        self.forceSE_atr.Set(Gf.Vec3f(0,0,0))
        self.forceSW_atr.Set(Gf.Vec3f(0,0,0))        



    def on_update(self, current_time: float, delta_time: float):
        # print(f"UPDATE  {self.prim_path} \t {current_time:.3f} \t {delta_time:.3f}")

        # Get current simulation time
        self.current_time = current_time
        self.delta_time = delta_time

        # if self.current_time > 5:
        #     self.command_off()
        #     self.rotors_off()

        # Update the drone status
        self.IMU()
        self.servo_control()
        self.platform_dynamics()
        pass



    def push_subscripted_event_method(self, e):       

        try:
            method = getattr(self, e.payload["method"]) 
        except:
            print(f"Method {e.payload['method']} not found")
            return
        
        match e.payload["method"]:
            case "eventFn_RemoteCommand":
                method(e.payload["command"])
            case "eventFn_FlightPlan":
                method(e.payload["fp"])
            case _:
                method()
            


    def eventFn_RemoteCommand(self, command):
        # print(command)
        self.cmd_on   = command["on"]
        self.cmd_velX = command["velX"]
        self.cmd_velY = command["velY"]
        self.cmd_velZ = command["velZ"]
        self.cmd_rotZ = command["rotZ"]
        self.cmd_exp_time = self.current_time + command["duration"]       
        


    def eventFn_FlightPlan(self, fp):
        print(fp)
        pass



########################################################################



    def command_off(self):
        self.cmd_on   = False
        self.cmd_velX = 0
        self.cmd_velY = 0
        self.cmd_velZ = 0
        self.cmd_rotZ = 0
        self.cmd_exp_time = None



    def hover(self):
        self.command_off()
        self.cmd_on = True
        self.cmd_exp_time = None     # Never expires



    def rotors_off(self):

        # Apagamos motores
        self.w_rotor_NE = 0
        self.w_rotor_NW = 0
        self.w_rotor_SE = 0
        self.w_rotor_SW = 0
        self.primRotStatic.SetActive(False)

        # Reset del control
        self.E = np.zeros((4, 1))



    def servo_control(self):
        # This function converts 
        # a navigation command (desired velocity vector and rotation)
        # to speeds of the four rotors

        if not self.cmd_on:
            self.rotors_off()
            return

        if self.cmd_exp_time is not None:
            if self.current_time > self.cmd_exp_time:
                self.hover()

        self.primRotStatic.SetActive(True)

        # Assign the model reference to be followed
        self.r[0, 0] = self.cmd_velX       # bXdot
        self.r[1, 0] = self.cmd_velY       # bYdot
        self.r[2, 0] = self.cmd_velZ       # bZdot
        self.r[3, 0] = self.cmd_rotZ       # hZdot
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

        pass



    def IMU(self):

        self.pos  = self.pos_atr.Get()
        # print(f"\nposition:  {self.pos}")

        self.ori  = self.ori_atr.Get()
        # print(f"\norientation:  {self.ori}")
     
        W = self.ori.real
        X = self.ori.imaginary[0]
        Y = self.ori.imaginary[1]
        Z = self.ori.imaginary[2]
        # print(f"Orientation:  {W:.4f} {X:.4f} {Y:.4f} {Z:.4f}")

        rot = Rotation.from_quat([X,Y,Z,W])     
        self.roll, self.pitch, self.yaw = rot.as_euler('xyz', degrees=False)
        # print(f"Euler RPY (rad):  {self.roll:.2f} {self.pitch:.2f} {self.yaw:.2f}")
        # roll, pitch, yaw = rot.as_euler('xyz', degrees=True)
        # print(f"Euler RPY (deg):  {roll:.0f} {pitch:.0f} {yaw:.0f}")

        self.linear_vel  = self.linVel_atr.Get()
        # print(f"linear velocity  (local):  {self.linear_vel}")

        self.angular_vel = self.angVel_atr.Get() * np.pi / 180
        # print(f"angular velocity (local):  {self.angular_vel}")



    def platform_dynamics(self):
        # Esta función traduce 
        # la velocidad de rotación de los 4 motores
        # a fuerzas y torques del sólido libre
        
        # Con esto simulamos rotación de sustentación
        # self.w_rotor_NE = self.w_hov
        # self.w_rotor_NW = self.w_rotor_NE
        # self.w_rotor_SE = self.w_rotor_NE
        # self.w_rotor_SW = self.w_rotor_NE

        # # Con esto simulamos rotación de giro
        # w_hov2 = (self.mass * self.g / 2.0 / self.kFT) ** 0.5
        # self.w_rotor_NE = w_hov2
        # self.w_rotor_NW = 0
        # self.w_rotor_SE = 0
        # self.w_rotor_SW = self.w_rotor_NE

        # Apply thrust force
        FT_NE = Gf.Vec3f(0, 0, self.kFT * self.w_rotor_NE**2)
        FT_NW = Gf.Vec3f(0, 0, self.kFT * self.w_rotor_NW**2)
        FT_SE = Gf.Vec3f(0, 0, self.kFT * self.w_rotor_SE**2)
        FT_SW = Gf.Vec3f(0, 0, self.kFT * self.w_rotor_SW**2)
        self.forceNE_atr.Set(FT_NE)
        self.forceNW_atr.Set(FT_NW)
        self.forceSE_atr.Set(FT_SE)
        self.forceSW_atr.Set(FT_SW)


        # Apply the air friction force to the drone
        FD = Gf.Vec3f(
            -self.kFDx * self.linear_vel[0] * abs(self.linear_vel[0]),
            -self.kFDy * self.linear_vel[1] * abs(self.linear_vel[1]),
            -self.kFDz * self.linear_vel[2] * abs(self.linear_vel[2]))
        self.force_atr.Set(FD)
      
      
        # Compute the drag moment
        MDR_NE = self.kMDR * self.w_rotor_NE**2
        MDR_NW = self.kMDR * self.w_rotor_NW**2
        MDR_SE = self.kMDR * self.w_rotor_SE**2
        MDR_SW = self.kMDR * self.w_rotor_SW**2
        MDR = Gf.Vec3f(0, 0, MDR_NE - MDR_NW - MDR_SE + MDR_SW)
        # print(f"MDR  = {MDR}")

        # Compute the air friction moment
        MD = Gf.Vec3f(
            -self.kMDx * self.angular_vel[0] * abs(self.angular_vel[0]),
            -self.kMDy * self.angular_vel[1] * abs(self.angular_vel[1]),
            -self.kMDz * self.angular_vel[2] * abs(self.angular_vel[2]))
        # print(f"MD  = {MD}")

        # Apply the moments to the drone
        self.torque_atr.Set(MDR + MD)
        # print(f"Torque Z => \t {MDR[2]:.4f} + {MD[2]:.4f} = {MDR[2] + MD[2]:.4f}")


