from typing import List, Optional
import numpy as np
import copy
from bisect import bisect_left


from planners.Waypoint import Waypoint



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
            return None
        else:
            return self.waypoints[0].t



    def FinishTime(self):
        # time of the last waypoint
        if not self.waypoints:
            return None
        else:
            return self.waypoints[-1].t




#-------------------------------------------------------------------
# FLIGHT PLAN BEHAVIOUR


#-------------------------------------------------------------------
# ROUTE MANAGEMENT



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
