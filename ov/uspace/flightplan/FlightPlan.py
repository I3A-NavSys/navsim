from re import T
from typing import List, Optional
import numpy as np
import copy
from bisect import bisect_left
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D  # Necesario para gráficos 3D

from uspace.flightplan.Waypoint import   Waypoint
from uspace.flightplan.command import Command

from scipy.spatial.transform import Rotation


class FlightPlan:



    def __init__(self, waypoints: Optional[List[Waypoint]] = None):
        self.id: int = 0
        self.priority: int = 0
        self.radius: float = 1
        self.maxVarLinVel = 5        # maximum variation in linear  velocity   [  m/s]
        self.maxVarAngVel = 1        # maximum variation in angular velocity   [rad/s]
        self.waypoints: List[Waypoint] = []

        if waypoints is not None:
            for waypoint in waypoints:
                self.SetWaypoint(waypoint)



    def SetWaypoint(self, waypoint):
        """Adds or updates a waypoint in the flight plan, maintaining sorted order by time."""

        if not isinstance(waypoint, Waypoint):
            raise ValueError('The waypoint must be a Waypoint object')

        times = [wp.t for wp in self.waypoints]
        index = bisect_left(times, waypoint.t)

        # Replace if the same time already exists
        if index < len(self.waypoints) and self.waypoints[index].t == waypoint.t:
            self.waypoints[index] = waypoint
        else:
            self.waypoints.insert(index, waypoint)



    def AppendWaypoint(self, waypoint: Waypoint) -> None:
        """Introduces a waypoint at the end (1s later) of the flight plan."""
        if not isinstance(waypoint, Waypoint):
            raise ValueError('The waypoint must be a Waypoint object')

        if not self.waypoints:
            waypoint.t = 0
        else:
            waypoint.t = self.FinishTime() + 1

        self.SetWaypoint(waypoint)



    def RemoveWaypointAtTime(self, t: float) -> None:
        """Removes the waypoint at a specific time `t` from the flight plan."""
        self.waypoints = list(filter(lambda wp: wp.t != t, self.waypoints))



    def GetIndexFromLabel(self, label: str) -> Optional[int]:
        for i, wp in enumerate(self.waypoints):
            if wp.label == label:
                return i
        return None
    
    

    def GetIndexFromTime(self, t: float) -> Optional[int]:
        # It returns the WP we are going to
        for i, wp in enumerate(self.waypoints):
            if t < wp.t:
                return i
        return i + 1



    def Copy(self):
        """Realiza una copia profunda de la instancia actual de FlightPlan."""
        return FlightPlan(
            waypoints=copy.deepcopy(self.waypoints),
        )



#-------------------------------------------------------------------
# TIME MANAGEMENT



    def InitTime(self):
        # time of the first waypoint
        if not self.waypoints:
            return 0
        else:
            return self.waypoints[0].t



    def FinishTime(self):
        # time of the last waypoint
        if not self.waypoints:
            return 0
        else:
            return self.waypoints[-1].t



    def SetTimeFromVel(self, label: str, vel: float):
        index = self.GetIndexFromLabel(label)
        if index <= 1:
            return

        wp1 = self.waypoints[index - 1]
        wp2 = self.waypoints[index]

        # Calcula el nuevo tiempo para wp2 basado en la velocidad
        t = wp1.t + wp1.DistanceTo(wp2) / vel
        self.PostponeFrom(wp2.t, t - wp2.t)



    def PostponeFrom(self, startTime: float, timeStep: float):
        # Postpone a portion of the Flight Plan a given time_delta,
        # starting from a given startTime

        if self.FinishTime() < startTime:
            return

        # Find the first waypoint with time greater than or equal to startTime
        index = bisect_left([wp.t for wp in self.waypoints], startTime)

        if index > 0 and timeStep < self.waypoints[index].TimeTo(self.waypoints[index-1]):
            return  # Not enough time in the past

        for i in range(index, len(self.waypoints)):
            self.waypoints[i].Postpone(timeStep)



    def Postpone(self, timeStep: float) -> None:
        # Postpone the Flight Plan by a given timeStep
        self.PostponeFrom(self.InitTime(), timeStep)



    def RescheduleAt(self, time: float) -> None:
        # Perform a temporal translation of the Flight Plan to begin at a given time.
        self.Postpone(time - self.InitTime())



#-------------------------------------------------------------------
# ROUTE MANAGEMENT


    def SetV0000(self):
        # Para cada waypoint, establece su velocidad uniforme asumiendo movimiento rectilíneo y uniforme
        for i in range(len(self.waypoints) - 1):
            wpA = self.waypoints[i]
            wpB = self.waypoints[i + 1]
            wpA.SetV0000(wpB)
        
        # Detiene la última velocidad (wpB)
        self.waypoints[-1].Stop()



    def SetJSC(self):
        """
        Para cada waypoint con tiempo, posición, velocidad y aceleración determinados,
        obtiene las 3 derivadas siguientes que ejecutan dicho movimiento.
        """
        for i in range(len(self.waypoints) - 1):
            wpA = self.waypoints[i]
            wpB = self.waypoints[i + 1]
            wpA.SetJSC(wpB)


    
    def SmoothVertexMaintainingSpeed(self, label, angVel):
        # Curva el vertice entre dos rectas
        # manteniendo velocidad y acortando el tiempo de vuelo
        # Para ello descompone dicho waypoint en dos

        i : int = self.GetIndexFromLabel(label)
        if i <= 0 or len(self.waypoints) <= i:
            return
      
        wp1 = self.waypoints[i-1]
        wp2 = self.waypoints[i]
        wp3 = self.waypoints[i+1]

        if wp2.fly_over:
            return
        
        angle = wp1.AngleWith(wp2)

        v1 = np.linalg.norm(wp1.vel)
        v2 = np.linalg.norm(wp2.vel)
        v = np.mean([v1, v2])

        r = v / angVel              # Radius of the curve
        d = r * np.tan(angle/2)     # Distance to the new waypoints
        step = d / v                # Time to the decomposed waypoints

        wp2A = Waypoint()
        wp2A.label = wp2.label + "_A"
        wp2A.pos = wp2.pos - wp1.vel * step
        wp2A.vel = wp1.vel
        wp2A.t = wp2.t - step

        wp2B = Waypoint()
        wp2B.label = wp2.label + "_B"
        wp2B.pos = wp2.pos + wp2.vel * step
        wp2B.vel = wp2.vel
        wp2BTinit = wp2.t + step

        T2Min = wp2A.t + angle/angVel
        T2Max = wp2BTinit

        while T2Max - T2Min > 0.05:
            wp2B.t = np.mean([T2Min, T2Max])
            wp2A.SetJSC(wp2B)

            tABmed = np.mean([wp2A.t, wp2B.t])
            status = wp2A.Interpolation(tABmed)
            vMed = np.linalg.norm(status.vel)

            if vMed < v:
                T2Max = wp2B.t
            else:
                T2Min = wp2B.t

        if wp2A.t < wp1.t or wp3.t < wp2B.t:
            return "There is not time enough to include the curve"
        
        self.RemoveWaypointAtTime(wp2.t)
        self.SetWaypoint(wp2A)
        self.SetWaypoint(wp2B)

        self.PostponeFrom(wp2B.t + 0.001, wp2B.t - wp2BTinit)

        

    def SmoothVertexMaintainingDuration(self, label, angVel, linAcel):
        # Curva el vertice entre dos rectas
        # reduciendo velocidad y manteniendo el tiempo de vuelo
        # Para ello descompone dicho waypoint en dos

        i : int = self.GetIndexFromLabel(label)
        if i <= 0 or len(self.waypoints) <= i:
            return
      
        wp1 = self.waypoints[i-1]
        wp2 = self.waypoints[i]

        if wp2.fly_over:
            return
        
        angle = wp1.AngleWith(wp2)
        tc = angle / angVel         # Time spent in the curve

        v1 = np.linalg.norm(wp1.vel)
        v2 = np.linalg.norm(wp2.vel)
        ts = np.abs(v2-v1) / linAcel

        interval = np.max([tc, ts])
        self.ExpandWaypoint(label, interval)

        

    def ExpandWaypoint(self, label, interval):
        # Decompone un waypoint en dos, separados un intervalo dado

        i : int = self.GetIndexFromLabel(label)
        if i <= 0 or len(self.waypoints) <= i:
            return
      
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

        wp2A.SetJSC(wp2B)

        self.RemoveWaypointAtTime(wp2.t)
        self.SetWaypoint(wp2A)
        self.SetWaypoint(wp2B)


#-------------------------------------------------------------------
# FLIGHT PLAN BEHAVIOUR



    def StatusAtTime(self, t: float) -> Waypoint:
        """
        Check the status of the flight plan at a given time.
        
        Args:
        t (float): The time at which to check the status.
        
        Returns:
        Waypoint: The interpolated waypoint status at time t.
        """

        numWPs = len(self.waypoints)
        
        # Check if t is outside the flight plan schedule
        if t < self.InitTime(): 
            wp2 = Waypoint(pos=self.waypoints[0].pos)
            return wp2
        
        if t > self.FinishTime():
            wp2 = Waypoint(pos=self.waypoints[numWPs-1].pos)
            return wp2

        # Search for the current waypoint
        for i in range(1, len(self.waypoints)):
            if t < self.waypoints[i].t:
                wp1 = self.waypoints[i - 1]
                wp2 = wp1.Interpolation(t)
                return wp2
    


    def Trace(self, timeStep):
        # This method expands the flight plan behavior over time
        
        instants = np.arange(self.InitTime(), self.FinishTime() + timeStep, timeStep)
        tr = np.zeros((len(instants), 7))
        tr[:, 0] = instants
        
        # Get position at each time instant
        for i in range(len(instants)):
            wp = self.StatusAtTime(tr[i, 0])
            tr[i, 1:4] = wp.pos
            tr[i, 4:7] = wp.vel
        
        tr[-1, 4:7] = [0, 0, 0]  # Set velocity to zero at the last time instant
        
        return tr



#-------------------------------------------------------------------
# UAV NAVIGATION


    def GetCommand(self, currentTime, UAVpos, UAVvel, UAVrot : Rotation, tToSolve) -> Command:
        # UAVvel = current UAV vel in global system
        # This function converts a flight plan position at certain time
        # to a navigation command (desired velocity vector and rotation)        

        _, _, UAVyaw = UAVrot.as_euler('xyz', degrees=False)

        # EXPECTED UAV POSE
        expected = self.StatusAtTime(currentTime)


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

        if np.linalg.norm(targetDir) >= 1:
            targetYaw = np.arctan2(targetDir[1], targetDir[0])
        else:
            targetYaw = UAVyaw

        errorYaw = targetYaw - UAVyaw
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
    


#-------------------------------------------------------------------
# CONFLICT DETECTION



#-------------------------------------------------------------------
# INFORMATION AND FIGURES

 

    def Print(self) -> None:
        """Prints all waypoints in the flight plan with their time, position, and velocity."""
        for wp in self.waypoints:
            print(f"{wp.label} \t  {wp.t} pos{wp.pos} vel{wp.vel}")



    def __repr__(self):
        return f"FlightPlan(id: {self.id}, waypoints: {len(self.waypoints)})"



    def PositionFigure(self, figName, timeStep):
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
        tr = self.Trace(timeStep)

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



    def VelocityFigure(self, figName, timeStep):
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
        tr = self.Trace(timeStep)

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



    def AddUAVTrackPos(self, figName, UAVinfo : List[Waypoint]):
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



    def AddUAVTrackVel(self, figName, UAVinfo : List[Waypoint]):
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