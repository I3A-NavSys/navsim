# Description: This script is used to control the minidrone in the UAM environment.

import omni.kit.app
from omni.kit.scripting import BehaviorScript
from pxr import Sdf, Gf
import carb.events

import numpy as np
from scipy.spatial.transform import Rotation
import pickle   # Serialization
import base64   # Parsing to string
import matplotlib.pyplot as plt



# Adding root 'project' folder to sys.path
import sys, os
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)
# # Verificar que la ruta se ha añadido correctamente
# print("Rutas en sys.path:")
# for path in sys.path:
#     print(path)

from uspace.flightplan.FlightPlan import FlightPlan
from uspace.flightplan.Waypoint   import Waypoint
from uspace.flightplan.command    import Command






########################################################################



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
        
        self.primRotSpinning = self.prim.GetChild("rotors_spinning")
        self.primRotStatic   = self.prim.GetChild("rotors_static")

        # Create the omniverse event associated to this UAV
        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(self.prim.GetPath()))
        bus = omni.kit.app.get_app().get_message_bus_event_stream()
        self.eventSub = bus.create_subscription_to_push_by_type(self.UAV_EVENT, self.push_subscripted_event_method)

        # Subscribe to the physics step event
        # self._stepping_sub = get_physx_interface().subscribe_physics_step_events(self._on_physics_step)
        # esto tiene el problema de que se suscribe múltiples veces al evento, por lo que se ejecuta varias veces el método _on_physics_step

        ########################################################################
        ## Navigation parameters

        self.fp = None               
        self.currentWP = None  

        # AutoPilot navigation command
        self.command = Command()
        self.cmd_exp_time = 0

        # Tracking
        self.show_tracking = True
        self.tracking_figure_builded = False
        self.refresh_rate = 1
        self.last_time_track = 0

        self.time_track = []

        self.xPos_track = []
        self.yPos_track = []
        self.zPos_track = []
        
        self.xVel_track = []
        self.yVel_track = []
        self.zVel_track = []

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

        # Tracking
        self.show_tracking = False
        self.refresh_rate = 1
        self.last_time_track = 0


        # Update the drone status
        self.IMU()
        self.navigation()
        self.servo_control()
        self.platform_dynamics()
        self.telemetry()


    
    def on_pause(self):
        print(f"PAUSE   {self.prim_path}")



    def on_stop(self):
        print(f"STOP    {self.prim_path}")

        self.current_time = 0
        self.delta_time = 0

        self.command.Off()
        self.rotors_off()

        self.force_atr.Set(Gf.Vec3f(0,0,0))
        self.torque_atr.Set(Gf.Vec3f(0,0,0))
        self.forceNE_atr.Set(Gf.Vec3f(0,0,0))
        self.forceNW_atr.Set(Gf.Vec3f(0,0,0))
        self.forceSE_atr.Set(Gf.Vec3f(0,0,0))
        self.forceSW_atr.Set(Gf.Vec3f(0,0,0))

        self.fp = None

        self.tracking_figure_builded = False
        self.time_track = []
        
        self.xPos_track = []
        self.yPos_track = []
        self.zPos_track = []
        
        self.xVel_track = []
        self.yVel_track = []
        self.zVel_track = []



    def on_update(self, current_time: float, delta_time: float):
        # print(f"UPDATE  {self.prim_path} \t {current_time:.3f} \t {delta_time:.3f}")

        # Get current simulation time
        self.current_time = current_time
        self.delta_time = delta_time

        # Update the drone status
        self.IMU()
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
        


    def eventFn_FlightPlan(self, fp):
        self.fp :FlightPlan = pickle.loads(base64.b64decode(fp))
        self.currentWP = None
        print(f"{self.prim_path}: flightplan received")
            


########################################################################
# Flying functions



    def rotors_off(self):

        # Apagamos motores
        self.w_rotor_NE = 0
        self.w_rotor_NW = 0
        self.w_rotor_SE = 0
        self.w_rotor_SW = 0
        self.primRotStatic.SetActive(True)
        self.primRotSpinning.SetActive(False)

        # Reset del control
        self.E = np.zeros((4, 1))



    def IMU(self):

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
        WP = self.fp.GetIndexFromTime(self.current_time)
        numWPs = len(self.fp.waypoints)

        if self.currentWP is None and WP != 0:
            # This flight plan is obsolete
            print(f"{self.prim_path} discarding FP due to it is obsolete")
            self.fp = None
            return
        
        # Navigation status has changed?
        if self.currentWP != WP:
            if WP == 0:
                initPos = self.fp.waypoints[0].pos.copy()
                initPos -= self.pos

                if np.linalg.norm(initPos) < self.fp.radius:
                    # Drone waiting to start the flight
                    print(f"{self.prim_path} waiting to start a FP")

                else:
                    # Drone in an incorrect starting position
                    print(f"{self.prim_path} discarding FP due to an incorrect starting position")
                    self.fp = None
                    return

            elif WP < numWPs:
                print(f"{self.prim_path} flying to {self.fp.waypoints[WP].label}")

            else:
                print(f"{self.prim_path} has completed its flight plan")

                # Uncomment this to show the corresponding plots
                # plt.close(plt.gcf())
                # self.fp.PositionFigure("FP1: POSITION", 0.01)
                # self.fp.VelocityFigure("FP1: VELOCITY", 0.01)
                
                # self.fp.AddUAVTrackPos("FP1: POSITION", self.xPos_track, self.yPos_track, self.zPos_track, self.time_track)
                # self.fp.AddUAVTrackVel("FP1: VELOCITY", np.array(self.xVel_track), np.array(self.yVel_track), np.array(self.zVel_track), self.time_track)

                self.fp = None
                return

        self.currentWP = WP
        
        self.command = self.fp.GetCommand(self.current_time, self.pos, self.linear_vel, self.rot, 2)
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
                self.command.Hover()

        self.primRotStatic.SetActive(False)
        self.primRotSpinning.SetActive(True)

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


        
########################################################################
# Tracking functions



    def telemetry(self):
        # Update every self.refresh_rate seconds
        if self.current_time - self.last_time_track >= self.refresh_rate:
            # Update last_time_track
            self.last_time_track = self.current_time

            # Get tracking information
            self.time_track.append(self.current_time)
            
            self.xPos_track.append(self.pos[0])
            self.yPos_track.append(self.pos[1])
            self.zPos_track.append(self.pos[2])

            self.xVel_track.append(self.linear_vel[0])
            self.yVel_track.append(self.linear_vel[1])
            self.zVel_track.append(self.linear_vel[2])

            # Check if we want to show tracking while simulating
            if self.show_tracking:
                # Check if we already built the figure
                if not self.tracking_figure_builded:
                    # Build the ploting figure just once
                    self.tracking_figure_builded = True

                    # Create the matplotlib figure and the corresponding plot
                    track_fig = plt.figure("Tracking Plot")
                    self.track_plot = track_fig.add_subplot(projection="3d")

                    # Indicate the axes name
                    self.track_plot.set_xlabel("x [m]")
                    self.track_plot.set_ylabel("y [m]")
                    self.track_plot.set_zlabel("z [m]")

                    # Write a title for the plot
                    self.track_plot.set_title("Position 3D")

                    # Stablish the initial limits
                    # self.track_plot.set_xlim3d(-15, 15)
                    # self.track_plot.set_ylim3d(-15, 15)
                    # self.track_plot.set_zlim3d(-15, 15)

                    # Plot initial position
                    self.line, = self.track_plot.plot(self.xPos_track, self.yPos_track, self.zPos_track, linestyle="dashed", linewidth=1, color="black")

                self.line.set_data(self.xPos_track, self.yPos_track)
                self.line.set_3d_properties(self.zPos_track)
                self.track_plot.scatter(self.xPos_track[-1], self.yPos_track[-1], self.zPos_track[-1], color="black", s=10)

                # Update plot limits
                track_plot_lims = self.get_matplotlib_plot_limits(self.track_plot)
                new_limits = self.update_track_plot_lims(track_plot_lims, self.pos)

                self.track_plot.set_xlim3d(*new_limits[0])
                self.track_plot.set_ylim3d(*new_limits[1])
                self.track_plot.set_zlim3d(*new_limits[2])

                plt.pause(0.01)


            
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


