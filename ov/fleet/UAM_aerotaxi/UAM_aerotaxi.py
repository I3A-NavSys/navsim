# Standard library imports
import sys
import os
import pickle   # Serialization
import base64   # Parsing to string

# Related third party imports
import omni.kit.app
import carb.events
import numpy as np
import matplotlib.pyplot as plt
from omni.kit.scripting import BehaviorScript
from pxr import Sdf, Gf
from scipy.spatial.transform import Rotation



##############################################################################
# Adding root 'ov' folder to sys.path

current_path = os.path.abspath(os.path.dirname(__file__))
while True:
    if os.path.basename(current_path) == 'ov':
        project_root_path = current_path
        break
    parent_path = os.path.dirname(current_path)
    if parent_path == current_path:
        raise RuntimeError("No se encontró el directorio 'ov' en la ruta.")
    current_path = parent_path
# print(f"Directorio raíz del proyecto: {project_root_path}")

if project_root_path not in sys.path:
    sys.path.append(project_root_path)



##############################################################################
# Local application/library specific imports
from uspace.flight_plan.flight_plan import FlightPlan
from uspace.flight_plan.waypoint import Waypoint
from uspace.flight_plan.command import Command


class UAM_minidrone(BehaviorScript):

    def on_init(self):
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
        
        self.primRotSpinning = self.prim.GetChild("rotors_spinning")
        self.primRotStatic   = self.prim.GetChild("rotors_static")
        
        self.prim_rotor_NE_rot_att = self.primRotStatic.GetChild("rotor_NE").GetAttribute("xformOp:orient")
        self.prim_rotor_NW_rot_att = self.primRotStatic.GetChild("rotor_NW").GetAttribute("xformOp:orient")
        self.prim_rotor_SE_rot_att = self.primRotStatic.GetChild("rotor_SE").GetAttribute("xformOp:orient")
        self.prim_rotor_SW_rot_att = self.primRotStatic.GetChild("rotor_SW").GetAttribute("xformOp:orient")
        self.prim_rotors_rot_vel = 179
        self.are_rotors_on = False

        # Create the omniverse event associated to this UAV
        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(self.prim.GetPath()))
        bus = omni.kit.app.get_app().get_message_bus_event_stream()
        self.eventSub = bus.create_subscription_to_push_by_type(self.UAV_EVENT, self.push_subscripted_event_method)

        #--------------------------------------------------------------------------------------------------------------
        # NAVIGATION PARAMETERS

        self.fp = None               
        self.currentWP = None  

        # AutoPilot navigation command
        self.command = Command()
        self.cmd_exp_time = 0

        # Tracking
        self.tracking_figure_builded = False
        self.refresh_rate = 1
        self.last_time_track = 0
        self.track_info = []

        #--------------------------------------------------------------------------------------------------------------
        # QUADCOPTER PARAMETERS

        self.g    = 9.81

        mass_attr = self.prim.GetAttribute("physics:mass")
        self.mass = mass_attr.Get()
        self.mass = 2000
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
        self.w_max = 62.8319       # rad/s = 600rpm
        self.w_min = 0             # rad/s =   0rpm

        # Aerodynamic thrust force constant
        # Force generated by the rotors is FT = kFT * w²
        self.kFT_N = 3.9895        # north side
        self.kFT_S = 1.5958        # south side
        self.w_hov = 41.888        # rad/s = 400rpm
        
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
        self.kMDx = 34.0439
        self.kMDy = 44.3280
        self.kMDz = 20.5427

        #--------------------------------------------------------------------------------------------------------------
        # LOW LEVEL CONTROL

        self.x  = np.zeros((8, 1))  # model state
        self.y  = np.zeros((4, 1))  # model output
        self.u  = np.zeros((4, 1))  # input (rotors speeds)
        self.r  = np.zeros((4, 1))  # model reference
        self.e  = np.zeros((4, 1))  # model error
        self.E  = np.zeros((4, 1))  # model accumulated error
        
        self.Kx = np.array([        # state control matrix
            [ -5.1944,  -18.2013,   -2.4117,   -8.4506,    2.8571,   -1.5256,    0.4354,    3.2057],
            [  5.1944,  -18.2013,    2.4117,   -8.4506,   -2.8571,   -1.5256,   -0.4354,    3.2057],
            [-20.7777,   45.5032,   -9.6468,   21.1265,   -8.5714,    3.8140,    1.7416,    3.2057],
            [ 20.7777,   45.5032,    9.6468,   21.1265,    8.5714,    3.8140,   -1.7416,    3.2057]
        ])


        self.Ky = np.array([        # error control matrix
            [ -0.3980,   0.1136,   1.0686,   0.9524],
            [ -0.3980,  -0.1136,   1.0686,  -0.9524],
            [  0.9950,   0.4543,   1.0686,  -2.8571],
            [  0.9950,  -0.4543,   1.0686,   2.8571]
        ])

        self.Hs = np.array([        # hovering speed
            [self.w_hov], 
            [self.w_hov], 
            [self.w_hov], 
            [self.w_hov]
        ])
        
        self.E_max = 150            # maximum model accumulated error

    #------------------------------------------------------------------------------------------------------------------
    # EVENT HANDLERS

    def on_play(self):
        # print(f"PLAY    {self.prim_path}")
        # print(f"\t {self.current_time} \t {self.delta_time}")

        # Tracking
        self.show_tracking = False
        self.refresh_rate = 1
        self.last_time_track = 0

        # Update the drone status
        self.imu()
        self.navigation()
        self.servo_control()
        self.platform_dynamics()
        self.telemetry()

    def on_pause(self):
        # print(f"PAUSE   {self.prim_path}")
        pass

    def on_stop(self):
        # print(f"STOP    {self.prim_path}")
        self.current_time = 0
        self.delta_time = 0

        self.command.off()
        self.rotors_off()

        self.force_atr.Set(Gf.Vec3f(0,0,0))
        self.torque_atr.Set(Gf.Vec3f(0,0,0))
        self.forceNE_atr.Set(Gf.Vec3f(0,0,0))
        self.forceNW_atr.Set(Gf.Vec3f(0,0,0))
        self.forceSE_atr.Set(Gf.Vec3f(0,0,0))
        self.forceSW_atr.Set(Gf.Vec3f(0,0,0))
    
        self.prim_rotor_NE_rot_att.Set(Gf.Quatd(1,0,0,0))
        self.prim_rotor_NW_rot_att.Set(Gf.Quatd(1,0,0,0))
        self.prim_rotor_SE_rot_att.Set(Gf.Quatd(1,0,0,0))
        self.prim_rotor_SW_rot_att.Set(Gf.Quatd(1,0,0,0))

        self.fp = None

        self.tracking_figure_builded = False
        self.track_info = []

    def on_update(self, current_time: float, delta_time: float):
        # print(f"UPDATE  {self.prim_path} \t {current_time:.3f} \t {delta_time:.3f}")

        # Get current simulation time
        self.current_time = current_time
        self.delta_time = delta_time

        self.animate_rotors()

        # Update the drone status
        self.imu()
        self.navigation()
        self.servo_control()
        self.platform_dynamics()
        self.telemetry()

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
        self.command : Command = pickle.loads(base64.b64decode(command))
        if self.command.duration is not None:
            self.cmd_exp_time = self.current_time + self.command.duration
            print(f"[{self.current_time}] {self.prim_path}: command received")
        
    def eventFn_FlightPlan(self, fp):
        self.fp :FlightPlan = pickle.loads(base64.b64decode(fp))
        self.currentWP = None
        print(f"[{self.current_time}] {self.prim_path}: flightplan received")
            
    #------------------------------------------------------------------------------------------------------------------
    # FLYING FUNCTIONS

    def rotors_off(self):
        # Apagamos motores
        self.w_rotor_NE = 0
        self.w_rotor_NW = 0
        self.w_rotor_SE = 0
        self.w_rotor_SW = 0
        # self.primRotStatic.SetActive(True)
        # self.primRotSpinning.SetActive(False)
        self.are_rotors_on = False

        # Reset del control
        self.E = np.zeros((4, 1))

    def animate_rotors(self):
        if self.are_rotors_on:
            ori = self.prim_rotor_NW_rot_att.Get()

            W = ori.real
            X = ori.imaginary[0]
            Y = ori.imaginary[1]
            Z = ori.imaginary[2]
            quaternion = Rotation.from_quat([X,Y,Z,W])

            rotation = Rotation.from_euler("z", self.prim_rotors_rot_vel, degrees=True)
            q_rot = rotation * quaternion
            
            new_quat = q_rot.as_quat()
            W = new_quat[3]
            X = new_quat[0]
            Y = new_quat[1]
            Z = new_quat[2]

            new_quat_NWSE = Gf.Quatd(W, X, Y, Z)
            new_quat_NESW = Gf.Quatd(W, X, Y, Z * -1)

            self.prim_rotor_NE_rot_att.Set(new_quat_NESW)
            self.prim_rotor_NW_rot_att.Set(new_quat_NWSE)
            self.prim_rotor_SE_rot_att.Set(new_quat_NWSE)
            self.prim_rotor_SW_rot_att.Set(new_quat_NESW)

    def imu(self):
        self.pos  = self.pos_atr.Get()
        # print(f"\nposition:  {self.pos}")

        ori  = self.ori_atr.Get()
        # print(f"\norientation:  {self.ori}")
     
        W = ori.real
        X = ori.imaginary[0]
        Y = ori.imaginary[1]
        Z = ori.imaginary[2]
        # print(f"Orientation:  {W:.4f} {X:.4f} {Y:.4f} {Z:.4f}")

        self.rot = Rotation.from_quat([X,Y,Z,W])     
        self.roll, self.pitch, self.yaw = self.rot.as_euler('xyz', degrees=False)
        # print(f"Euler RPY (rad):  {self.roll:.2f} {self.pitch:.2f} {self.yaw:.2f}")
        # roll, pitch, yaw = rot.as_euler('xyz', degrees=True)
        # print(f"Euler RPY (deg):  {roll:.0f} {pitch:.0f} {yaw:.0f}")

        self.linear_vel  = self.linVel_atr.Get()
        # print(f"linear velocity  (local):  {self.linear_vel}")

        self.angular_vel = self.angVel_atr.Get() * np.pi / 180
        # print(f"angular velocity (local):  {self.angular_vel}")

    def navigation(self):
        # This function converts a flight plan position at certain time
        # to a navigation command (desired velocity vector and rotation)
        if self.fp is None:
            return

        # Check FP vigency
        WP = self.fp.GetTargetIndexFromTime(self.current_time)
        numWPs = len(self.fp.waypoints)

        if self.currentWP is None and WP != 0:
            # This flight plan is obsolete
            print(f"[{self.current_time:3.2f}] {self.prim_path} discarding FP due to it is obsolete")
            self.fp = None
            return
        
        # Navigation status has changed?
        if self.currentWP != WP:
            if WP == 0:
                initPos = self.fp.waypoints[0].pos.copy()
                initPos -= self.pos

                if np.linalg.norm(initPos) < self.fp.radius:
                    # Drone waiting to start the flight
                    print(f"[{self.current_time:3.2f}] {self.prim_path} waiting to start a FP")

                else:
                    # Drone in an incorrect starting position
                    print(f"[{self.current_time:3.2f}] {self.prim_path} discarding FP due to an incorrect starting position")
                    self.fp = None
                    return

            elif WP < numWPs:
                print(f"[{self.current_time:3.2f}] {self.prim_path} flying to {self.fp.waypoints[WP].label}")

            else:
                print(f"[{self.current_time:3.2f}] {self.prim_path} has completed its flight plan")

                # Uncomment this to show the corresponding plots
                # plt.close(plt.gcf())
                # self.fp.PositionFigure("FP1: POSITION", 0.01)
                # self.fp.VelocityFigure("FP1: VELOCITY", 0.01)
                
                # self.fp.AddUAVTrackPos("FP1: POSITION", self.track_info)
                # self.fp.AddUAVTrackVel("FP1: VELOCITY", self.track_info)

                self.fp = None
                return

        self.currentWP = WP
        
        # Change relative vel to absolute
        linear_vel = self.rot.apply(self.linear_vel)
        self.command = self.fp.get_command(self.current_time, self.pos, linear_vel, self.rot, 2)
        self.cmd_exp_time = self.current_time + self.command.duration

    def servo_control(self):
        # This function converts 
        # a navigation command (desired velocity vector and rotation)
        # to speeds of the four rotors

        if not self.command.on:
            self.rotors_off()
            return

        if self.command.duration is not None:
            if self.current_time > self.cmd_exp_time:
                self.command.hover()

        # self.primRotStatic.SetActive(False)
        # self.primRotSpinning.SetActive(True)
        self.are_rotors_on = True

        # Assign the model reference to be followed
        self.r[0, 0] = self.command.velX       # bXdot
        self.r[1, 0] = self.command.velY       # bYdot
        self.r[2, 0] = self.command.velZ       # bZdot
        self.r[3, 0] = self.command.rotZ       # hZdot
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

    def platform_dynamics(self):
        # Esta función traduce 
        # la velocidad de rotación de los 4 motores
        # a fuerzas y torques del sólido libre
        # Con esto simulamos rotación de sustentación
        # self.w_rotor_NE = self.w_hov
        # self.w_rotor_NW = self.w_rotor_NE
        # self.w_rotor_SE = self.w_rotor_NE
        # self.w_rotor_SW = self.w_rotor_NE

        # Apply thrust force
        FT_NE = Gf.Vec3f(0, 0, self.kFT_N * self.w_rotor_NE**2)
        FT_NW = Gf.Vec3f(0, 0, self.kFT_N * self.w_rotor_NW**2)
        FT_SE = Gf.Vec3f(0, 0, self.kFT_S * self.w_rotor_SE**2)
        FT_SW = Gf.Vec3f(0, 0, self.kFT_S * self.w_rotor_SW**2)
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
        MDR_NE = self.kMDR_N * self.w_rotor_NE**2
        MDR_NW = self.kMDR_N * self.w_rotor_NW**2
        MDR_SE = self.kMDR_S * self.w_rotor_SE**2
        MDR_SW = self.kMDR_S * self.w_rotor_SW**2
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

    #------------------------------------------------------------------------------------------------------------------
    # TRACKING FUNCTIONS

    def telemetry(self):
        # Update every self.refresh_rate seconds
        if self.current_time - self.last_time_track >= self.refresh_rate:
            # Update last_time_track
            self.last_time_track = self.current_time

            # Change relative vel to absolute
            linear_vel = self.rot.apply(self.linear_vel)

            # Get tracking information
            self.track_info.append(Waypoint(t= self.current_time, pos=self.pos, vel=linear_vel))

            # Check if we want to show tracking while simulating
            # if self.show_tracking:
            #     # Check if we already built the figure
            #     if not self.tracking_figure_builded:
            #         # Build the ploting figure just once
            #         self.tracking_figure_builded = True

            #         # Create the matplotlib figure and the corresponding plot
            #         track_fig = plt.figure("Tracking Plot")
            #         self.track_plot = track_fig.add_subplot(projection="3d")

            #         # Indicate the axes name
            #         self.track_plot.set_xlabel("x [m]")
            #         self.track_plot.set_ylabel("y [m]")
            #         self.track_plot.set_zlabel("z [m]")

            #         # Write a title for the plot
            #         self.track_plot.set_title("Position 3D")

            #         # Stablish the initial limits
            #         # self.track_plot.set_xlim3d(-15, 15)
            #         # self.track_plot.set_ylim3d(-15, 15)
            #         # self.track_plot.set_zlim3d(-15, 15)

            #         # Plot initial position
            #         self.line, = self.track_plot.plot(self.xPos_track, self.yPos_track, self.zPos_track, 
            #                                           linestyle="dashed", linewidth=1, color="black")

            #     self.line.set_data(self.xPos_track, self.yPos_track)
            #     self.line.set_3d_properties(self.zPos_track)
            #     self.track_plot.scatter(self.xPos_track[-1], self.yPos_track[-1], self.zPos_track[-1], color="black", 
            #                             s=10)

            #     # Update plot limits
            #     track_plot_lims = self.get_matplotlib_plot_limits(self.track_plot)
            #     new_limits = self.update_track_plot_lims(track_plot_lims, self.pos)

            #     self.track_plot.set_xlim3d(*new_limits[0])
            #     self.track_plot.set_ylim3d(*new_limits[1])
            #     self.track_plot.set_zlim3d(*new_limits[2])

            #     plt.pause(0.01)

    def get_matplotlib_plot_limits(self, plot):
        try:
            z_lim = plot.get_zlim3d()
            x_lim = plot.get_xlim3d()
            y_lim = plot.get_ylim3d()

            return [x_lim, y_lim, z_lim]
        
        except:
            x_lim = plot.get_xlim3d()
            y_lim = plot.get_ylim3d()

            return (x_lim, y_lim)
        
    def update_track_plot_lims(self, track_plot_lims, new_pos):
        if new_pos[0] < track_plot_lims[0][0]:
            track_plot_lims[0] = [new_pos[0], track_plot_lims[0][1]]

        if new_pos[0] > track_plot_lims[0][1]:
            track_plot_lims[0] = [track_plot_lims[0][0], new_pos[0]]

        if new_pos[1] < track_plot_lims[1][0]:
            track_plot_lims[1] = [new_pos[1], track_plot_lims[1][1]]

        if new_pos[1] > track_plot_lims[1][1]:
            track_plot_lims[1] = [track_plot_lims[1][0], new_pos[1]]

        if new_pos[2] < track_plot_lims[2][0]:
            track_plot_lims[2] = [new_pos[2], track_plot_lims[2][0]]

        if new_pos[2] > track_plot_lims[2][1]:
            track_plot_lims[2] = [track_plot_lims[2][0], new_pos[2]]

        return track_plot_lims