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
        self.waypoints: List[Waypoint] = []
        self.currentWP = None

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



    def PostponeFrom(self, time: float, timeStep: float):
        # Postpone a portion of the Flight Plan a given time_delta,
        # starting from a given start_time

        if self.FinishTime() < time:
            return

        # Find the first waypoint with time greater than or equal to start_time
        index = bisect_left([wp.t for wp in self.waypoints], time)

        if index > 0 and timeStep < self.waypoints[index].t - self.waypoints[index - 1].t:
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



    def SetJLS(self):
        """
        Para cada waypoint con tiempo, posición, velocidad y aceleración determinados,
        obtiene las 3 derivadas siguientes que ejecutan dicho movimiento.
        """
        for i in range(len(self.waypoints) - 1):
            wpA = self.waypoints[i]
            wpB = self.waypoints[i + 1]
            wpA.SetJLS(wpB)



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
        # Check if t is outside the flight plan schedule
        if t < self.InitTime() or t > self.FinishTime():
            wp2 = Waypoint()
            wp2.pos = np.array([np.nan, np.nan, np.nan])
            return wp2

        # Search for the current waypoints
        for i in range(1, len(self.waypoints)):
            if t < self.waypoints[i].t:
                break
        
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

 

    def Navigate(self, UAVname, UAVori : Rotation, UAVyaw, currentStatus : Waypoint, tToSolve):
        # This function converts a flight plan position at certain time
        # to a navigation command (desired velocity vector and rotation)        
        # if self.fp is None:
        #     return

        # No sé donde ponerlo
        maxVarLinVel = 5
        maxVarAngVel = 1
        targetStep = 2

        # Check FP vigency
        WP = self.GetIndexFromTime(currentStatus.t)

        # Avoid exception when currentStatus.t is greater than fp.FinishTime() (solved as now it returns the last WP)
        if WP is not None:
        
            if self.currentWP is None and WP != 0:
                # This flight plan is obsolete
                print(f"{UAVname} discarding FP due to it is obsolete")
                # self.fp = None
                return None
            
            # Current UAV status (la tiene wp)
            
            # Navigation status has changed?
            if self.currentWP != WP:
                if WP == 0:
                    initPos = self.waypoints[0].pos.copy()
                    initPos -= currentStatus.pos

                    if np.linalg.norm(initPos) < self.radius:
                        # Drone waiting to start the flight
                        print(f"{UAVname} waiting to start a FP")

                    else:
                        # Drone in an incorrect starting position
                        print(f"{UAVname} discarding FP due to an incorrect starting position")
                        # self.fp = None
                        return None

                elif WP < len(self.waypoints):
                    print(f"{UAVname} flying to WP {WP}")

                else:
                    print(f"{UAVname} has completed its flight plan")
                    # self.fp = None

                    cmd = Command(on=True, duration=0)
                    return cmd

            self.currentWP = WP

            # Time to analyze movement
            step = self.waypoints[self.currentWP].t - currentStatus.t

            if 0 < self.currentWP and self.currentWP < len(self.waypoints):
                step = targetStep

            if step == 0:
                step = 1    # Avoid division by zero

            # COMPUTING DESIRED POSITION / VELOCITY
            # correctStatus = self.StatusAtTime(currentStatus.t)
            # desPos = correctStatus.pos
            # desVel = correctStatus.vel
            desPos, desVel = self.PosVelAtTime(currentStatus.t)
            # print("currentPos:", currentStatus.pos)
            # print("desPos:", desPos)
            # print("desVel:", desVel)

            # COMPUTING CORRECTION VELOCITY (to achieve dtPos in 'targetStep' seconds)
            crVel = (desPos - currentStatus.pos) / step
            # print("crVel:", crVel)

            # COMPUTING COMMANDED VELOCITY
            cmdVel = desVel + crVel
            # print("cmdVel1:", cmdVel)

            # TODO: Implement this by means of a dynamic control
            # SMOOTHING COMMANDED VELOCITY
            varVel = cmdVel - currentStatus.vel
            varVelMagnitude = np.linalg.norm(varVel)
            if varVelMagnitude > maxVarLinVel:
                varVel /= varVelMagnitude # Normalize
                varVel *= maxVarLinVel
            # print("varVel:", varVel)

            cmdVel = currentStatus.vel + varVel
            # print("cmdVel2:", cmdVel)

            # COMPUTING DRONE RELATIVE LINEAR VELOCITY
            cmdRelVel = UAVori.inv().apply(cmdVel)
            # print("cmdRelVel:", cmdRelVel)

            # COMPUTING TARGET ERROR YAW
            targetDir = cmdVel.copy()
            targetDir[2] = 0

            if np.linalg.norm(targetDir) == 0:
                targetYaw = UAVyaw
            else:
                targetYaw = np.arctan2(targetDir[1], targetDir[0])

            errorYaw = targetYaw - UAVyaw
            while errorYaw < -np.pi:
                errorYaw += 2*np.pi

            while np.pi < errorYaw:
                errorYaw -= 2*np.pi

            # COMPUTING TARGET ANGULAR VELOCITY
            currentWel = errorYaw / targetStep
            
            if currentWel < -maxVarAngVel:
                currentWel = -maxVarAngVel
            
            if maxVarAngVel < currentWel:
                currentWel = maxVarAngVel

            # CREATING COMMANDED RELATIVE VELOCITY VECTOR
            cmd = Command(
                on=True,
                velX=cmdRelVel[0],
                velY=cmdRelVel[1],
                velZ=cmdRelVel[2],
                # velY=cmdRelVel[1] * -1,
                # velZ=cmdRelVel[2] * -1,
                rotZ=currentWel,
                duration=tToSolve
            )

            return cmd
        
        return None



    def PosVelAtTime(self, t2):
        r2 = None
        v2 = None

        numWps = len(self.waypoints)
        i = self.GetIndexFromTime(t2)

        if i == 0:
            # The drone is waiting to start the FP
            r2 = self.waypoints[0].pos
            v2 = np.array([0.0, 0.0, 0.0])
        
        elif i == numWps:
            # The drone has completed the FP
            r2 = self.waypoints[i-1]
            v2 = np.array([0.0, 0.0, 0.0])
        
        else:
            # The drone is executing the FP
            wp1 = self.waypoints[i-1]
            t1 = wp1.t
            t = t2-t1

            r1 = wp1.pos
            v1 = wp1.vel
            a1 = wp1.acel
            j1 = wp1.jerk
            s1 = wp1.snap
            c1 = wp1.crkl

            r2 = r1 + v1*t + a1*pow(t,2)/2 + j1*pow(t,3)/6 + s1*pow(t,4)/24 + c1*pow(t,5)/120
            v2 = v1 + a1*t + j1*pow(t,2)/2 + s1*pow(t,3)/6 + c1*pow(t,4)/24

        return r2, v2
    


    # def VelocityAtTime(self, t2):
    #     v2 = None

    #     numWps = len(self.waypoints)
    #     i = self.GetIndexFromTime(t2)

    #     if i == 0:
    #         # The drone is waiting to start the FP
    #         v2 = np.array([0.0, 0.0, 0.0])
        
    #     elif i == numWps:
    #         # The drone has completed the FP
    #         v2 = np.array([0.0, 0.0, 0.0])
        
    #     else:
    #         # The drone is executing the FP
    #         wp1 = self.waypoints[i-1]
    #         t1 = wp1.t
    #         t = t2-t1

    #         v1 = wp1.vel
    #         a1 = wp1.acel
    #         j1 = wp1.jerk
    #         s1 = wp1.snap
    #         c1 = wp1.crkl

    #         v2 = v1 + a1*t + j1*pow(t,2)/2 + s1*pow(t,3)/6 + c1*pow(t,4)/24

    #     return v2



    def YawAtTime(self, t, currentYaw):
        i = self.GetIndexFromTime(t)
        numWPs = len(self.waypoints)
        if i == numWPs:
            i -= 1

        wp1 = self.waypoints[i-1]
        preWPpos = wp1.pos

        wp2 = self.waypoints[i]
        currentWPpos = wp2.pos

        targetDir = currentWPpos - preWPpos
        targetDir[2] = 0

        targetYaw = None
        if np.linalg.norm(targetDir) < 1:
            targetYaw = currentYaw
        else:
            targetYaw = np.arctan2(targetDir[1], targetDir[0])

        return targetYaw
    

    
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



    # def PositionFigure(self, figName, timeStep, xPosUAV, yPosUAV, zPosUAV, timeUAV):
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

        # # Plot UAV route
        # xyzPosPlot.plot(xPosUAV, yPosUAV, zPosUAV, linestyle="dashed", linewidth=1, color="black")
        # xPosTimePlot.plot(timeUAV, xPosUAV, linestyle="dashed", linewidth=1, color="black")
        # yPosTimePlot.plot(timeUAV, yPosUAV, linestyle="dashed", linewidth=1, color="black")
        # zPosTimePlot.plot(timeUAV, zPosUAV, linestyle="dashed", linewidth=1, color="black")

        # # Highlight UAV route positions
        # xyzPosPlot.scatter(xPosUAV, yPosUAV, zPosUAV, color="black", s=10)
        # xPosTimePlot.scatter(timeUAV, xPosUAV, color="black", s=10)
        # yPosTimePlot.scatter(timeUAV, yPosUAV, color="black", s=10)
        # zPosTimePlot.scatter(timeUAV, zPosUAV, color="black", s=10)

        # Update limits to maintain scale in all axes
        # xLim = max(np.abs(xyzPosPlot.get_xlim3d()))
        # yLim = max(np.abs(xyzPosPlot.get_ylim3d()))
        # zLim = max(np.abs(xyzPosPlot.get_zlim3d()))
        # maxLim = max(xLim, yLim, zLim)

        # xyzPosPlot.set_xlim3d(-maxLim, maxLim)
        # xyzPosPlot.set_ylim3d(-maxLim, maxLim)
        # xyzPosPlot.set_zlim3d(-maxLim, maxLim)

        # xLim = max(np.abs(xPosTimePlot.get_ylim()))
        # yLim = max(np.abs(yPosTimePlot.get_ylim()))
        # zLim = max(np.abs(zPosTimePlot.get_ylim()))
        # maxLim = max(xLim, yLim, zLim)

        # xPosTimePlot.set_ylim(-maxLim, maxLim)
        # yPosTimePlot.set_ylim(-maxLim, maxLim)
        # zPosTimePlot.set_ylim(-maxLim, maxLim)

        # Show the plots
        plt.show(block=False)



    # def VelocityFigure(self, figName, timeStep, xVelUAV, yVelUAV, zVelUAV, timeUAV):
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

        # # Plot UAV route
        # velPlot3D.plot(timeUAV, np.sqrt(xVelUAV**2 + yVelUAV**2 + zVelUAV**2), linestyle="dashed", linewidth=1, color="black")
        # xVelTimePlot.plot(timeUAV, xVelUAV, linestyle="dashed", linewidth=1, color="black")
        # yVelTimePlot.plot(timeUAV, yVelUAV, linestyle="dashed", linewidth=1, color="black")
        # zVelTimePlot.plot(timeUAV, zVelUAV, linestyle="dashed", linewidth=1, color="black")

        # # Highlight UAV route positions
        # velPlot3D.scatter(timeUAV, np.sqrt(xVelUAV**2 + yVelUAV**2 + zVelUAV**2), color="black", s=10)
        # xVelTimePlot.scatter(timeUAV, xVelUAV, color="black", s=10)
        # yVelTimePlot.scatter(timeUAV, yVelUAV, color="black", s=10)
        # zVelTimePlot.scatter(timeUAV, zVelUAV, color="black", s=10)

        # Update limits to maintain scale in all axes
        # lim3D = max(np.abs(velPlot3D.get_ylim()))
        # xLim = max(np.abs(xVelTimePlot.get_ylim()))
        # yLim = max(np.abs(yVelTimePlot.get_ylim()))
        # zLim = max(np.abs(zVelTimePlot.get_ylim()))
        # maxLim = max(lim3D, xLim, yLim, zLim)

        # velPlot3D.set_ylim(-maxLim, maxLim)
        # xVelTimePlot.set_ylim(-maxLim, maxLim)
        # yVelTimePlot.set_ylim(-maxLim, maxLim)
        # zVelTimePlot.set_ylim(-maxLim, maxLim)

        # Show the plots
        plt.show(block=False)


