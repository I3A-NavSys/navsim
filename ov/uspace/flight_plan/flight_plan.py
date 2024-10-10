# Standard library imports
import copy
from re import T
from typing import List, Optional

# Related third party imports
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# Local application/library specific imports
from uspace.flight_plan.waypoint import Waypoint
from uspace.flight_plan.command import Command


class FlightPlan:

    def __init__(self):
        self.id: int = 0
        self.priority: int = 0
        self.radius: float = 1
        self.maxVarLinVel = 5        # maximum variation in linear  velocity   [  m/s]
        self.maxVarAngVel = 1        # maximum variation in angular velocity   [rad/s]
        self.waypoints: List[Waypoint] = []
        self.targetYaw = None

    def set_waypoint(self, wp=None, label="", time=None, pos=None, vel=None):
        numWPs = len(self.waypoints)

        if wp is None:
            if time is None:
                if numWPs == 0:
                    time = 0
                else:
                    time = self.FinishTime() + 1

            if numWPs > 0:      status = self.status_at_time(time)

            if pos is None:
                if numWPs == 0:
                    pos = [0,0,0]
                else:
                    pos = status.pos

            if vel is None:
                if numWPs == 0:
                    vel = [0,0,0]
                else:
                    if time <= self.init_time() or time >= self.FinishTime():
                        vel = [0,0,0]
                    else:
                        vel = status.vel

            wp = Waypoint(label=label, t=time, pos=pos, vel=vel)
        
        index = self.get_target_index_from_time(wp.t)

        # Replace if the same time already exists
        if index < numWPs and self.waypoints[index].t == wp.t:
            self.waypoints[index] = wp
        else:
            self.waypoints.insert(index, wp)

    def remove_waypoint_at_time(self, t: float) -> None:
        """Removes the waypoint at a specific time `t` from the flight plan."""
        self.waypoints = list(filter(lambda wp: wp.t != t, self.waypoints))

    def get_index_from_label(self, label: str) -> Optional[int]:
        for i, wp in enumerate(self.waypoints):
            if wp.label == label:
                return i
        return None
    
    def get_running_index_from_time(self, t: float):
        # It returns the WP the UAV is currently executing
        index = self.get_target_index_from_time(t)
        return index - 1
    
    def get_target_index_from_time(self, t: float):
        # It returns the WP the UAV is flying to
        # Note: if t == wp.t, that wp is also considered as target, although they are at the same instant
        if not self.waypoints:
            return 0

        for i, wp in enumerate(self.waypoints):
            if t <= wp.t:
                return i
        return i + 1
    
    def copy(self):
        """Realiza una copia profunda de la instancia actual de FlightPlan."""
        return FlightPlan(waypoints=copy.deepcopy(self.waypoints))

    #------------------------------------------------------------------------------------------------------------------
    # TIME MANAGEMENT

    def init_time(self):
        # time of the first waypoint
        if not self.waypoints:
            return 0
        else:
            return self.waypoints[0].t

    def finish_time(self):
        # time of the last waypoint
        if not self.waypoints:
            return 0
        else:
            return self.waypoints[-1].t

    def set_uniform_velocity(self, wp=None, vel=None):
        # Compute MRU velocity for all WPs
        if wp is None and vel is None:
            for i in range(len(self.waypoints)-1):
                self.set_uniform_velocity(wp=i)

            # Stop last WP
            self.waypoints[-1].Stop()

        # Set vel velocity to all WPs
        elif wp is None:
            for i in range(len(self.waypoints)-1):
                self.set_uniform_velocity(wp=i, vel=vel)

            # Stop last WP
            self.waypoints[-1].Stop()

        # Compute MRU velocity just for wp WP
        elif vel is None:
            # Get WP index
            if isinstance(wp, str):     index = self.get_index_from_label(wp)
            else:                       index = wp

            # Return if it is the last WP or it is not found
            if (index == len(self.waypoints)-1) or (index is None): return

            # Get specified WP and next one
            wp1 = self.waypoints[index]
            wp2 = self.waypoints[index+1]

            # Update wp1.vel
            wp1.set_uniform_velocity(wp2)        

        # Set vel velocity just to wp WP
        else:
            # Get WP index
            if isinstance(wp, str):     index = self.get_index_from_label(wp)
            else:                       index = wp

            # Return if it is the last WP or it is not found
            if (index == len(self.waypoints)-1) or (index is None): return

            # Get specified WP and next one
            wp1 = self.waypoints[index]
            wp2 = self.waypoints[index+1]

            # New time for wp2 according to vel
            t2 = wp1.t + wp1.DistanceTo(wp2) / vel
            # Update wp2.t and postpone following WPs
            self.postpone_from(wp2.t, t2 - wp2.t)
            # Update wp1.vel
            wp1.set_uniform_velocity(wp2)

    def postpone_from(self, startTime: float, timeStep: float):
        # Postpone a portion of the Flight Plan a given time_delta,
        # starting from a given startTime
        if self.FinishTime() < startTime:
            return

        # Find the first waypoint with time greater than or equal to startTime
        index = self.get_target_index_from_time(startTime)

        if index > 0 and timeStep < self.waypoints[index].TimeTo(self.waypoints[index-1]):
            return  # Not enough time in the past

        for i in range(index, len(self.waypoints)):
            self.waypoints[i].postpone(timeStep)

    def postpone(self, timeStep: float) -> None:
        # Postpone the Flight Plan by a given timeStep
        self.postpone_from(self.init_time(), timeStep)

    def reschedule_at(self, time: float) -> None:
        # Perform a temporal translation of the Flight Plan to begin at a given time.
        self.postpone(time - self.init_time())

    #------------------------------------------------------------------------------------------------------------------
    # ROUTE MANAGEMENT

    def connect_waypoints(self):
        """
        Para cada waypoint con tiempo, posición, velocidad y aceleración determinados,
        obtiene las 3 derivadas siguientes que ejecutan dicho movimiento.
        """
        for i in range(len(self.waypoints) - 1):
            wpA = self.waypoints[i]
            wpB = self.waypoints[i + 1]
            wpA.connect_to(wpB)

    def smooth_waypoint_speed(self, label, angVel):
        # Curva el vertice entre dos rectas
        # manteniendo velocidad y acortando el tiempo de vuelo
        # Para ello descompone dicho waypoint en dos
        i : int = self.get_index_from_label(label)
        if (i== 0) or (i == len(self.waypoints) - 1) or (i is None):
            raise RuntimeError(f"Trying to smooth invalid WP (received label: {label})")
      
        wp1 = self.waypoints[i-1]
        wp2 = self.waypoints[i]
        wp3 = self.waypoints[i+1]

        if wp2.fly_over:
            raise RuntimeError(f"Trying to smooth a fly over WP (waypoint: {wp2.label})")
        
        angle = wp1.angle_with(wp2)

        v1 = np.linalg.norm(wp1.vel)
        v2 = np.linalg.norm(wp2.vel)
        v = np.mean([v1, v2])

        r = v / angVel              # Radius of the curve
        d = r * np.tan(angle/2)     # Distance to the new waypoints
        step = d / v                # Time to the decomposed waypoints

        if step == 0:
            raise RuntimeError(f"Trying to smooth with step=0")

        label = wp2.label + "_A"
        pos = wp2.pos - wp1.vel * step
        vel = wp1.vel
        t = wp2.t - step
        wp2A = Waypoint(label=label, t=t, pos=pos, vel=vel)
        # wp2A.t = wp2.t - step

        label = wp2.label + "_B"
        pos = wp2.pos + wp2.vel * step
        vel = wp2.vel
        wp2B = Waypoint(label=label, pos=pos, vel=vel)
        wp2BTinit = wp2.t + step

        T2Min = wp2A.t + angle/angVel
        T2Max = wp2BTinit

        while T2Max - T2Min > 0.05:
            wp2B.t = np.round(np.mean([T2Min, T2Max]), 2)
            wp2A.connect_to(wp2B)

            tABmed = np.mean([wp2A.t, wp2B.t])
            status = wp2A.interpolation(tABmed)
            vMed = np.linalg.norm(status.vel)

            if vMed < v:
                T2Max = wp2B.t
            else:
                T2Min = wp2B.t

        if wp2A.t <= wp1.t or wp3.t <= wp2B.t or (wp3.t + wp2B.t - wp2BTinit) <= wp2B.t:
            raise RuntimeError(f"There is not time enough to include the curve in waypoint {wp2.label}")
        
        self.RemoveWaypointAtTime(wp2.t)
        self.SetWaypoint(wp2A)
        self.SetWaypoint(wp2B)

        self.postpone_from(wp2B.t + 0.001, wp2B.t - wp2BTinit)

    def smooth_waypoint_duration(self, label, angVel, linAcel):
        # Curva el vertice entre dos rectas
        # reduciendo velocidad y manteniendo el tiempo de vuelo
        # Para ello descompone dicho waypoint en dos
        i : int = self.get_index_from_label(label)
        if (i== 0) or (i == len(self.waypoints) - 1) or (i is None):
            return
      
        wp1 = self.waypoints[i-1]
        wp2 = self.waypoints[i]

        if wp2.fly_over:
            return
        
        angle = wp1.angle_with(wp2)
        tc = angle / angVel         # Time spent in the curve

        v1 = np.linalg.norm(wp1.vel)
        v2 = np.linalg.norm(wp2.vel)
        ts = np.abs(v2-v1) / linAcel

        interval = np.max([tc, ts])
        self.expand_waypoint(i, interval)

    def expand_waypoint(self, index, interval):
        # Decompone un waypoint en dos, separados un intervalo dado
        i = index
      
        wp1 = self.waypoints[i-1]
        wp2 = self.waypoints[i]
        wp3 = self.waypoints[i+1]
        
        step = interval / 2
        if (step >= wp2.t - wp1.t) or (step >= wp3.t - wp2.t):
            # There is not time enough to include the curve
            return

        wp2A = Waypoint()
        wp2A.label = wp2.label + "_A"
        wp2A.pos = wp2.pos - wp1.vel * step
        wp2A.vel = wp1.vel
        wp2A.t = wp2.t - step

        wp2B = Waypoint()
        wp2B.label = wp2.label + "_B"
        wp2B.pos = wp2.pos + wp2.vel * step
        wp2B.vel = wp2.vel
        wp2B.t = wp2.t + step

        wp2A.connect_to(wp2B)

        self.RemoveWaypointAtTime(wp2.t)
        self.SetWaypoint(wp2A)
        self.SetWaypoint(wp2B)

    #------------------------------------------------------------------------------------------------------------------
    # FLIGHT PLAN BEHAVIOUR

    def status_at_time(self, t: float) -> Waypoint:
        """
        Check the status of the flight plan at a given time.
        
        Args:
        t (float): The time at which to check the status.
        
        Returns:
        Waypoint: The interpolated waypoint status at time t.
        """
        # Check if t is outside the flight plan schedule
        if t <= self.init_time(): 
            return self.waypoints[0]
        
        if t >= self.FinishTime():
            return self.waypoints[-1]

        # Get the current waypoint
        for i in range(1, len(self.waypoints)):
            if t < self.waypoints[i].t:
                wp1 = self.waypoints[i - 1]
                wp2 = wp1.interpolation(t)
                return wp2
            
        # index = self.GetRunningIndexFromTime(t)
        # wp2 = self.waypoints[index].interpolation(t)
        # return wp2
    
    def trace(self, timeStep):
        # This method expands the flight plan behavior over time
        instants = np.arange(self.init_time(), self.FinishTime() + timeStep, timeStep)
        tr = np.zeros((len(instants), 7))
        tr[:, 0] = instants
        
        # Get position at each time instant
        for i in range(len(instants)):
            wp = self.status_at_time(tr[i, 0])
            tr[i, 1:4] = wp.pos
            tr[i, 4:7] = wp.vel
        
        tr[-1, 4:7] = [0, 0, 0]  # Set velocity to zero at the last time instant
        
        return tr

    #------------------------------------------------------------------------------------------------------------------
    # UAV NAVIGATION

    def get_command(self, currentTime, UAVpos, UAVvel, UAVrot : Rotation, tToSolve) -> Command:
        # UAVvel = current UAV vel in global system
        # This function converts a flight plan position at certain time
        # to a navigation command (desired velocity vector and rotation)        

        # CURRENT UAV YAW
        _, _, UAVyaw = UAVrot.as_euler('xyz', degrees=False)

        # EXPECTED UAV POSE
        expected = self.status_at_time(currentTime)

        # COMPUTING CORRECTION VELOCITY (to achieve status.pos in 'tToSolve' seconds)
        crVel = (expected.pos - UAVpos) / tToSolve

        # COMPUTING COMMANDED VELOCITY
        cmdVel = expected.vel + crVel
        # print("cmdVel1:", cmdVel)

        # SMOOTHING COMMANDED VELOCITY
        varVel = cmdVel - UAVvel
        varVelMagnitude = np.linalg.norm(varVel)
        if varVelMagnitude > self.maxVarLinVel:
            varVel /= varVelMagnitude # Normalize
            varVel *= self.maxVarLinVel
        # print("varVel:", varVel)

        cmdVel = UAVvel + varVel
        # print("cmdVel2:", cmdVel)

        # COMPUTING DRONE RELATIVE LINEAR VELOCITY
        cmdRelVel = UAVrot.inv().apply(cmdVel)
        # print("cmdRelVel:", cmdRelVel)

        # COMPUTING TARGET ERROR YAW
        targetDir = expected.vel.copy()
        targetDir[2] = 0

        if np.linalg.norm(targetDir) > 0:
            self.targetYaw = np.arctan2(targetDir[1], targetDir[0])
        elif self.targetYaw is None:
            self.targetYaw = UAVyaw

        errorYaw = self.targetYaw - UAVyaw
        while errorYaw < -np.pi:
            errorYaw += 2*np.pi

        while np.pi < errorYaw:
            errorYaw -= 2*np.pi

        # COMPUTING TARGET ANGULAR VELOCITY
        currentWel = errorYaw / tToSolve
        
        if currentWel < -self.maxVarAngVel:
            currentWel = -self.maxVarAngVel
        
        if self.maxVarAngVel < currentWel:
            currentWel = self.maxVarAngVel

        # CREATING COMMANDED RELATIVE VELOCITY VECTOR
        cmd = Command(
            on=True,
            velX=cmdRelVel[0],
            velY=cmdRelVel[1],
            velZ=cmdRelVel[2],
            rotZ=currentWel,
            duration=tToSolve
        )

        return cmd
    
    #------------------------------------------------------------------------------------------------------------------
    # CONFLICT DETECTION

    #------------------------------------------------------------------------------------------------------------------
    # INFORMATION AND FIGURES

    def print_waypoints(self) -> None:
        """Prints all waypoints in the flight plan with their time, position, and velocity."""
        for wp in self.waypoints:
            print(f"{wp.label} \t  {wp.t} pos{wp.pos} vel{wp.vel}")

    def __repr__(self):
        return f"FlightPlan(id: {self.id}, waypoints: {len(self.waypoints)})"

    def position_figure(self, figName, timeStep):
        # Display the flight plan trajectory
        
        # Check if the flight plan is empty
        if not self.waypoints:
            print('The flight plan is empty')
            return
        
        # Create matplolib figure (window)
        posFig = plt.figure(figName)

        # Figure settings
        color = [0, 0.7, 1]

        # Get the trace
        tr = self.trace(timeStep)

        # POSITION 3D
        # Create plot
        xyzPosPlot = posFig.add_subplot(3, 5, (1, 13), projection="3d")
        
        # Indicate axes' name
        xyzPosPlot.set_xlabel("x [m]")
        xyzPosPlot.set_ylabel("y [m]")
        xyzPosPlot.set_zlabel("z [m]")
        
        # Set title
        xyzPosPlot.set_title("Position 3D")
        
        # Set grid to True
        xyzPosPlot.grid(True)
        
        # Set plot info
        xyzPosPlot.plot(tr[:,1], tr[:,2], tr[:,3], linewidth=2, color=color)

        # POSITIONS VERSUS TIME
        # Create plots
        xPosTimePlot = posFig.add_subplot(3, 5, (4, 5))
        yPosTimePlot = posFig.add_subplot(3, 5, (9, 10))
        zPosTimePlot = posFig.add_subplot(3, 5, (14, 15))
        
        # Indicate axes' names
        xPosTimePlot.set_ylabel("x [m]")
        yPosTimePlot.set_ylabel("y [m]")
        zPosTimePlot.set_ylabel("z [m]")
        zPosTimePlot.set_xlabel("t [s]")
        
        # Set title
        xPosTimePlot.set_title("Position versus time")
        
        # Set grid to True
        xPosTimePlot.grid(True)
        yPosTimePlot.grid(True)
        zPosTimePlot.grid(True)
        
        # Set plots info
        xPosTimePlot.plot(tr[:,0], tr[:,1], linewidth=2, color=color)
        yPosTimePlot.plot(tr[:,0], tr[:,2], linewidth=2, color=color)
        zPosTimePlot.plot(tr[:,0], tr[:,3], linewidth=2, color=color)

        # Get waypoints positions to highlight
        xPos = []
        yPos = []
        zPos = []
        t = []

        for wp in self.waypoints:
            xPos.append(wp.pos[0])
            yPos.append(wp.pos[1])
            zPos.append(wp.pos[2])
            t.append(wp.t)

        # Highlight waypoints positions
        xyzPosPlot.scatter(xPos, yPos, zPos, marker="o", color="blue", s=20)
        xPosTimePlot.scatter(t, xPos, marker="o", color="blue", s=20)
        yPosTimePlot.scatter(t, yPos, marker="o", color="blue", s=20)
        zPosTimePlot.scatter(t, zPos, marker="o", color="blue", s=20)

        # Update limits to maintain scale in all axes
        xLim = max(np.abs(xyzPosPlot.get_xlim3d()))
        yLim = max(np.abs(xyzPosPlot.get_ylim3d()))
        zLim = max(np.abs(xyzPosPlot.get_zlim3d()))
        maxLim = max(xLim, yLim, zLim)

        xyzPosPlot.set_xlim3d(-maxLim, maxLim)
        xyzPosPlot.set_ylim3d(-maxLim, maxLim)
        xyzPosPlot.set_zlim3d(-maxLim, maxLim)

        xLim = max(np.abs(xPosTimePlot.get_ylim()))
        yLim = max(np.abs(yPosTimePlot.get_ylim()))
        zLim = max(np.abs(zPosTimePlot.get_ylim()))
        maxLim = max(xLim, yLim, zLim)

        xPosTimePlot.set_ylim(-maxLim, maxLim)
        yPosTimePlot.set_ylim(-maxLim, maxLim)
        zPosTimePlot.set_ylim(-maxLim, maxLim)

        # Show the plots
        plt.show(block=False)

    def velocity_figure(self, figName, timeStep):
        # Display the flight plan instant velocity

        # Check if the flight plan is empty
        if not self.waypoints:
            print('The flight plan is empty')
            return
        
        # Create matplolib figure (window)
        velFig = plt.figure(figName)

        # Figure settings
        color = [0, 0.7, 1]

        # Get the trace
        tr = self.trace(timeStep)

        # VELOCITY 3D
        # Create plot
        velPlot3D = velFig.add_subplot(4, 2, (1, 2))
        
        # Indicate axes' name
        velPlot3D.set_ylabel("3D [m/s]")
        
        # Set title
        velPlot3D.set_title("Velocity versus time")
        
        # Set grid to True
        velPlot3D.grid(True)
        
        # Set plot info
        velPlot3D.plot(tr[:,0], np.sqrt(tr[:,4]**2 + tr[:,5]**2 + tr[:,6]**2), linewidth=2, color=color)

        # VELOCITIES VERSUS TIME
        # Create plots
        xVelTimePlot = velFig.add_subplot(4, 2, (3, 4))
        yVelTimePlot = velFig.add_subplot(4, 2, (5, 6))
        zVelTimePlot = velFig.add_subplot(4, 2, (7, 8))
        
        # Indicate axes' names
        xVelTimePlot.set_ylabel("vx [m/s]")
        yVelTimePlot.set_ylabel("vy [m/s]")
        zVelTimePlot.set_ylabel("vz [m/s]")
        zVelTimePlot.set_xlabel("t [s]")
        
        # Set grid to True
        xVelTimePlot.grid(True)
        yVelTimePlot.grid(True)
        zVelTimePlot.grid(True)
        
        # Set plots info
        xVelTimePlot.plot(tr[:,0], tr[:,4], linewidth=2, color=color)
        yVelTimePlot.plot(tr[:,0], tr[:,5], linewidth=2, color=color)
        zVelTimePlot.plot(tr[:,0], tr[:,6], linewidth=2, color=color)

        # Get waypoints velocities to highlight
        xVel = []
        yVel = []
        zVel = []
        t = []

        for wp in self.waypoints:
            xVel.append(wp.vel[0])
            yVel.append(wp.vel[1])
            zVel.append(wp.vel[2])
            t.append(wp.t)

        # Highlight waypoints positions
        xVelTimePlot.scatter(t, xVel, marker="o", color="blue", s=20)
        yVelTimePlot.scatter(t, yVel, marker="o", color="blue", s=20)
        zVelTimePlot.scatter(t, zVel, marker="o", color="blue", s=20)

        # Update limits to maintain scale in all axes
        lim3D = max(np.abs(velPlot3D.get_ylim()))
        xLim = max(np.abs(xVelTimePlot.get_ylim()))
        yLim = max(np.abs(yVelTimePlot.get_ylim()))
        zLim = max(np.abs(zVelTimePlot.get_ylim()))
        maxLim = max(lim3D, xLim, yLim, zLim)

        velPlot3D.set_ylim(-maxLim, maxLim)
        xVelTimePlot.set_ylim(-maxLim, maxLim)
        yVelTimePlot.set_ylim(-maxLim, maxLim)
        zVelTimePlot.set_ylim(-maxLim, maxLim)

        # Show the plots
        plt.show(block=False)

    def add_UAV_track_pos(self, figName, UAVinfo : List[Waypoint]):
        posFig = plt.figure(figName)
        subplots = posFig.get_axes()
        xyzPosPlot = subplots[0]
        xPosTimePlot = subplots[1]
        yPosTimePlot = subplots[2]
        zPosTimePlot = subplots[3]

        xPosUAV = []
        yPosUAV = []
        zPosUAV = []
        timeUAV = []

        for wp in UAVinfo:
            xPosUAV.append(wp.pos[0])
            yPosUAV.append(wp.pos[1])
            zPosUAV.append(wp.pos[2])
            timeUAV.append(wp.t)

        # Plot UAV route
        xyzPosPlot.plot(xPosUAV, yPosUAV, zPosUAV, linestyle="dashed", linewidth=1, color="black")
        xPosTimePlot.plot(timeUAV, xPosUAV, linestyle="dashed", linewidth=1, color="black")
        yPosTimePlot.plot(timeUAV, yPosUAV, linestyle="dashed", linewidth=1, color="black")
        zPosTimePlot.plot(timeUAV, zPosUAV, linestyle="dashed", linewidth=1, color="black")

        # Highlight UAV route positions
        xyzPosPlot.scatter(xPosUAV, yPosUAV, zPosUAV, color="black", s=10)
        xPosTimePlot.scatter(timeUAV, xPosUAV, color="black", s=10)
        yPosTimePlot.scatter(timeUAV, yPosUAV, color="black", s=10)
        zPosTimePlot.scatter(timeUAV, zPosUAV, color="black", s=10)

    def add_UAV_track_vel(self, figName, UAVinfo : List[Waypoint]):
        posFig = plt.figure(figName)
        subplots = posFig.get_axes()
        velPlot3D = subplots[0]
        xVelTimePlot = subplots[1]
        yVelTimePlot = subplots[2]
        zVelTimePlot = subplots[3]

        xVelUAV = []
        yVelUAV = []
        zVelUAV = []
        timeUAV = []

        for wp in UAVinfo:
            xVelUAV.append(wp.vel[0])
            yVelUAV.append(wp.vel[1])
            zVelUAV.append(wp.vel[2])
            timeUAV.append(wp.t)

        xVelUAV = np.array(xVelUAV)
        yVelUAV = np.array(yVelUAV)
        zVelUAV = np.array(zVelUAV)
        
        # Plot UAV route
        velPlot3D.plot(timeUAV, np.sqrt(xVelUAV**2 + yVelUAV**2 + zVelUAV**2), linestyle="dashed", linewidth=1, color="black")
        xVelTimePlot.plot(timeUAV, xVelUAV, linestyle="dashed", linewidth=1, color="black")
        yVelTimePlot.plot(timeUAV, yVelUAV, linestyle="dashed", linewidth=1, color="black")
        zVelTimePlot.plot(timeUAV, zVelUAV, linestyle="dashed", linewidth=1, color="black")

        # Highlight UAV route positions
        velPlot3D.scatter(timeUAV, np.sqrt(xVelUAV**2 + yVelUAV**2 + zVelUAV**2), color="black", s=10)
        xVelTimePlot.scatter(timeUAV, xVelUAV, color="black", s=10)
        yVelTimePlot.scatter(timeUAV, yVelUAV, color="black", s=10)
        zVelTimePlot.scatter(timeUAV, zVelUAV, color="black", s=10)