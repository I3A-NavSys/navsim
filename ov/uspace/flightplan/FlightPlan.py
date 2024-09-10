from re import T
from typing import List, Optional
import numpy as np
import copy
from bisect import bisect_left
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Necesario para gráficos 3D





from uspace.flightplan.Waypoint import   Waypoint



class FlightPlan:



    def __init__(self, waypoints: Optional[List[Waypoint]] = None):
        self.id: int = 0
        self.priority: int = 0
        self.radius: float = 1
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



    def GetIndexFromLabel(self, label: str) -> int:
        for i, wp in enumerate(self.waypoints):
            if wp.label == label:
                return i
        return -1



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
        
        plt.figure(figName)

        # Figure settings
        color = [0, 0.7, 1]

        tr = self.Trace(timeStep)
        plt.plot(tr[:, 1], tr[:, 2], '-', linewidth=2, color=color)
        plt.grid(True)
        plt.show()
