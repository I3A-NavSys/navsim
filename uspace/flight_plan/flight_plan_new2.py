from os import times

from tabulate import tabulate
from typing import List, Optional, Any, Union
import math
import numpy as np
import matplotlib
import mplcursors
import matplotlib.pyplot as plt
from matplotlib.backend_tools import ToolToggleBase
from matplotlib.collections import PathCollection
import multiprocessing as mp
# import bisect
from sortedcontainers import SortedList


from uspace.flight_plan.waypoint_new2 import Waypoint
from uspace.flight_plan.command import Command


matplotlib.use("Qt5Agg")
plt.rcParams["toolbar"] = "toolmanager"

class FlightPlan:
    def __init__(
        self, 
        id: str="", 
        priority: int=0, 
        radius: float=1, 
        max_var_lin_vel: float=5, 
        max_var_ang_vel: float=1, 
        target_yaw=None
    ):
        self.id: str = id
        self.priority: int = priority
        self.radius: float = radius
        self.max_var_lin_vel = max_var_lin_vel        # maximum variation in linear  velocity   [  m/s]
        self.max_var_ang_vel = max_var_ang_vel        # maximum variation in angular velocity   [rad/s]
        self.target_yaw = target_yaw
        self.waypoints: List[Waypoint] = []
        self.time_waypoints: SortedList[float] = SortedList([])
        self.labels_to_idx = {}
        self.length: int = 0
        self.figure_processes = []

    def __repr__(self):
        waypoints_info = [[wp.label, wp.time, wp.pos, wp.vel, wp.acel, wp.jerk, wp.snap, wp.crakle] for wp in self.waypoints]
        waypoints_headers = ('label', 'time', 'position', 'velocity', 'acceleration', 'jerk', 'snap', 'crakle')
        return f"FlightPlan with id '{self.id}':\n{tabulate(waypoints_info, headers=waypoints_headers, tablefmt='grid')}"

    # ----------------------------------
    # -------- BASE FUNCTIONS ----------
    # ----------------------------------
    def add_waypoint(self, wp=None, label=None, time=None, pos=None, vel=None, heading=[0,0]):
        if wp is None:
            if time is None:
                if self.length == 0:
                    time = 0
                else:
                    time = self.finish_time() + 1

            if self.length > 0:  status = self.status_at_time(time)

            if pos is None:
                if self.length == 0:
                    pos = [0,0,0]
                else:
                    pos = status.pos

            if vel is None:
                if self.length == 0:
                    vel = [0,0,0]
                else:
                    if time <= self.init_time():
                        vel = [0,0,0]
                    else:
                        vel = status.vel

            if label == None:
                if self.length == 0:
                    label = "default_label_0"
                else:
                    label = f"default_label_{self.length}"

            wp = Waypoint(label=label, time=time, pos=pos, vel=vel, heading=heading)
        
        # Determine the index to insert the new waypoint based on its time
        index = self.time_waypoints.bisect_left(wp.time)

        # Add label to the dictionary for quick access
        self.labels_to_idx[wp.label] = index

        # Replace if the same time already exists
        update_existing_wp = (
            (index < self.length and self.time_waypoints[index] == wp.time) or
            (index > 0 and index == self.length and self.time_waypoints[index - 1] == wp.time)
        )
        
        if update_existing_wp:
            old_wp = self.waypoints.pop(index)
            self.time_waypoints.pop(index)
            self.labels_to_idx.pop(old_wp.label, None)

            self.length -= 1

        # Insert the new waypoint and its time into the sorted lists
        self.waypoints.insert(index, wp)
        self.time_waypoints.add(wp.time)

        self.length += 1

    def remove_waypoint(
        self, 
        idx: Optional[int] = None,  
        # label: Optional[str], 
        time: Optional[float] = None
    ) -> None:
        """
        Remove a waypoint from the flight plan by index, or time.
        If no index or time is provided, the last waypoint will be removed.

        Args:
            - idx (Optional[int]) : The index of the waypoint to remove. If None, the waypoint will be removed by time.
            
            - time (Optional[float]) : The time of the waypoint to remove. If None, the waypoint will be removed by index.
        """

        # Check if there is any waypoint to remove
        if self.length == 0:
            return

        remove_by_idx = idx is not None
        remove_by_time = time is not None

        # Remove the last waypoint
        if not remove_by_idx and not remove_by_time:
            wp = self.waypoints.pop()
            self.time_waypoints.pop()
            self.labels_to_idx.pop(wp.label, None)
            self.length -= 1
            return
        
        # Remove by index
        if remove_by_idx:
            # Check if the index is valid
            is_not_valid = (
                idx >= self.length or 
                idx < -self.length
            )
            if is_not_valid:
                raise IndexError(f"Index {idx} is out of range for FlightPlan of length {self.length}.")
            
            wp = self.waypoints.pop(idx)
            self.time_waypoints.pop(idx)
            self.labels_to_idx.pop(wp.label, None)
            self.length -= 1
            return
        
        # Remove by time
        if time >= self.time_waypoints[-1]:
            pointed_wp_idx = self.length - 1
        else:
            pointed_wp_idx = self.time_waypoints.bisect_left(time)
        
        wp = self.waypoints.pop(pointed_wp_idx)
        self.time_waypoints.pop(pointed_wp_idx)
        self.labels_to_idx.pop(wp.label, None)
        self.length -= 1
        return

    def get_idx_by_label(self, label: str) -> Optional[int]:
        """
        Get the index of a waypoint by its label.

        Args:
            - label (str) : The label of the waypoint to find.

        Returns:
            Optional[int] : The index of the waypoint if found, otherwise None.
        """

        if label in self.labels_to_idx:
            return self.labels_to_idx[label]
        return None

    def get_running_waypoint(self, time: float) -> Optional[Waypoint]:
        """
        Get the waypoint that the UAV is executing at a given time.

        Args:
            - time (float) : The time to check for the executing waypoint. 
        
        Returns:
            Optional[Waypoint] : The waypoint being executed at the given time, or None if no waypoint is found.
        """

        waypoint_idx = self.get_running_waypoint_idx(time)
        
        if waypoint_idx is None:
            return None
        
        return self.waypoints[waypoint_idx]

    def get_running_waypoint_idx(self, time: float) -> Optional[int]:
        """
        Get the index of the waypoint that the UAV is executing at a given time.

        Args:
            - time (float) : The time to check for the executing waypoint. 

        Returns:
            Optional[int] : The index of the waypoint being executed at the given time, or None if no waypoint is found.
        """

        if self.length == 0:
            return None
        
        waypoint_idx = self.time_waypoints.bisect_right(time) - 1
        return waypoint_idx

    def get_target_waypoint(self, time: float) -> Optional[Waypoint]:
        """
        Get the waypoint that the UAV is flying to at a given time.

        Args:
            - time (float) : The time to check for the target waypoint. 

        Returns:
            Optional[Waypoint] : The waypoint being targeted at the given time, or None if no waypoint is found.
        """

        waypoint_idx = self.get_target_waypoint_idx(time)
        
        if waypoint_idx is None:
            return None
        
        return self.waypoints[waypoint_idx]
    
    def get_target_waypoint_idx(self, time: float) -> Optional[int]:
        """
        Get the index of the waypoint that the UAV is flying to at a given time.

        Args:
            - time (float) : The time to check for the target waypoint. 

        Returns:
            Optional[int] : The index of the waypoint being targeted at the given time, or None if no waypoint is found.
        """

        if self.length == 0:
            return None
        
        waypoint_idx = self.time_waypoints.bisect_right(time)
        
        if waypoint_idx >= self.length:
            return None
        return waypoint_idx
    
    # ----------------------------------
    # -------- AUXILIARY FUNCTIONS -----
    # ----------------------------------
    def copy(self) -> FlightPlan:
        """
        Makes a deep copy of the flight plan

        Returns:
            FlightPlan: A new instance of FlightPlan with the same waypoints.
        """
        flight_plan = FlightPlan.__new__(FlightPlan)
        
        flight_plan.id = self.id
        flight_plan.priority = self.priority
        flight_plan.radius = self.radius
        flight_plan.max_var_lin_vel = self.max_var_lin_vel
        flight_plan.max_var_ang_vel = self.max_var_ang_vel
        flight_plan.target_yaw = self.target_yaw
        flight_plan.length = self.length
        flight_plan.waypoints = [wp.copy() for wp in self.waypoints]
        flight_plan.time_waypoints = SortedList(self.time_waypoints)
        flight_plan.labels_to_idx = dict(self.labels_to_idx)
            
        return flight_plan

    def to_dict(self) -> dict[str, Any]:
        """
        Convert the flight plan to a dictionary.

        Returns:
            dict: A dictionary representing the flight plan.
        """
        
        return {
            "id": self.id,
            "priority": self.priority,
            "radius": self.radius,
            "max_var_lin_vel": self.max_var_lin_vel,
            "max_var_ang_vel": self.max_var_ang_vel,
            "target_yaw": self.target_yaw,
            "waypoints": {
                "labels":   [wp.label    for wp in self.waypoints],
                "fly_over": [wp.fly_over for wp in self.waypoints],
                "times":    [wp.time        for wp in self.waypoints],
                "headings": np.array([wp.heading  for wp in self.waypoints]).tolist(),
                "pos":    np.array([wp.pos    for wp in self.waypoints]).tolist(),
                "vel":    np.array([wp.vel    for wp in self.waypoints]).tolist(),
                "acel":   np.array([wp.acel   for wp in self.waypoints]).tolist(),
                "jerk":   np.array([wp.jerk   for wp in self.waypoints]).tolist(),
                "snap":   np.array([wp.snap   for wp in self.waypoints]).tolist(),
                "crakle": np.array([wp.crakle for wp in self.waypoints]).tolist(),
            }
        }
    
    def from_dict(self, data: dict[str, Any]) -> None:
        """
        Load the flight plan from a dictionary.

        Args:
            - data (dict) : A dictionary representing the flight plan.
        """

        # 1. Load basic attributes
        self.id = data.get("id", 0)
        self.priority = data.get("priority", 0)
        self.radius = data.get("radius", 1)
        self.max_var_lin_vel = data.get("max_var_lin_vel", 5)
        self.max_var_ang_vel = data.get("max_var_ang_vel", 1)
        self.target_yaw = data.get("target_yaw", None)

        # 2. Reset waypoints and related attributes
        self.waypoints = []
        self.time_waypoints = SortedList([])
        self.labels_to_idx = {}
        self.length = 0

        # 3. Load waypoints
        wps_data = data.get("waypoints", {})
        if not wps_data:
            return

        # 4. Extract waypoint attributes from the dictionary
        labels   = wps_data.get("labels",   [])
        fly_over = wps_data.get("fly_over", [])
        times    = wps_data.get("times",    [])
        headings = wps_data.get("headings", [])
        pos      = wps_data.get("pos",      [])
        vel      = wps_data.get("vel",      [])
        acel     = wps_data.get("acel",     [])
        jerk     = wps_data.get("jerk",     [])
        snap     = wps_data.get("snap",     [])
        crakle   = wps_data.get("crakle",   [])

        # 5. Create and add waypoints to the flight plan
        for i in range(len(labels)):
            wp = Waypoint(
                label=labels[i],
                time=times[i],
                pos=pos[i],
                vel=vel[i],
                acel=acel[i],
                jerk=jerk[i],
                snap=snap[i],
                crakle=crakle[i],
                fly_over=fly_over[i],
                heading=headings[i]
            )
            self.waypoints.append(wp)
            self.time_waypoints.add(wp.time)
            self.labels_to_idx[wp.label] = i
            self.length += 1

    def waypoints_to_arrays(self) -> List[np.ndarray]:
        """
        Convert the waypoints to separate numpy arrays for each attribute.

        Returns:
            - List[np.ndarray] : A list of numpy arrays containing the attributes of each waypoint,
            in this order: [times, positions, velocities, accelerations, jerks, snaps, crackles, headings].

        Example::

            flight_plan = FlightPlan()
            flight_plan.add_waypoint(label="WP1", time=0, pos=[0, 0, 0], vel=[1, 0, 0])
            flight_plan.add_waypoint(label="WP2", time=1, pos=[1, 0, 0], vel=[1, 0, 0])
            arrays = flight_plan.waypoints_to_arrays()
            # arrays[0] -> array([0., 1.])              shape (N,)    # times
            # arrays[1] -> array([[0,0,0],[1,0,0]])     shape (N, 3)  # positions
            # arrays[2] -> array([[1,0,0],[1,0,0]])     shape (N, 3)  # velocities
            # arrays[3] -> array([[0,0,0],[0,0,0]])     shape (N, 3)  # accelerations
            # arrays[4] -> array([[0,0,0],[0,0,0]])     shape (N, 3)  # jerks
            # arrays[5] -> array([[0,0,0],[0,0,0]])     shape (N, 3)  # snaps
            # arrays[6] -> array([[0,0,0],[0,0,0]])     shape (N, 3)  # crackles
            # arrays[7] -> array([[0,0],[0,0]])         shape (N, 2)  # headings
        """

        labels        = np.empty(self.length, dtype=object)
        times         = np.empty(self.length)
        positions     = np.empty((self.length, 3))
        velocities    = np.empty((self.length, 3))
        accelerations = np.empty((self.length, 3))
        jerks         = np.empty((self.length, 3))
        snaps         = np.empty((self.length, 3))
        crackles      = np.empty((self.length, 3))
        headings      = np.empty((self.length, 2))

        for i, wp in enumerate(self.waypoints):
            labels[i]        = wp.label
            times[i]         = wp.time
            positions[i]     = wp.pos
            velocities[i]    = wp.vel
            accelerations[i] = wp.acel
            jerks[i]         = wp.jerk
            snaps[i]         = wp.snap
            crackles[i]      = wp.crakle
            headings[i]      = wp.heading

        return [labels, times, positions, velocities, accelerations, jerks, snaps, crackles, headings]
    
    # ----------------------------------
    # -------- TIME MANAGEMENT ---------
    # ----------------------------------
    def start_time(self) -> Optional[float]:
        """
        Returns the time of the first waypoint in the flight plan.

        Returns:
            Optional[float]: The time of the first waypoint, or None if there are no waypoints.
        """

        if self.length == 0:
            return None
        else:
            return self.waypoints[0].time

    def finish_time(self) -> Optional[float]:
        """
        Returns the time of the last waypoint in the flight plan.

        Returns:
            Optional[float]: The time of the last waypoint, or None if there are no waypoints.
        """

        if self.length == 0:
            return None
        else:
            return self.waypoints[-1].time

    def remove_negative_time(self) -> None:
        """
        Remove any negative time from the flight plan by postponing all waypoints
        to ensure the first waypoint starts at positive time (>= 0).
        """

        if self.time_waypoints[0] < 0:
            self.postpone(-self.time_waypoints[0])

    def postpone_from(self, start_time: float, time_delta: float) -> None:
        """
        Postpone the Flight Plan from a given start_time by a given time_delta.

        Args:
            - start_time (float) : The time from which to start postponing the flight plan.
            - time_delta (float) : The amount of time to postpone the flight plan.
        """

        # Early exit conditions: 
        # if the flight plan is empty, 
        # if the time_delta is zero 
        # if the start_time is after the finish_time
        if self.length == 0 or time_delta == 0 or start_time >= self.time_waypoints[-1]:
            return

        # Find the index of the first waypoint whose time is greater than or equal to start_time
        index = self.time_waypoints.bisect_left(start_time)

        # Check if the time_delta is less than the gap between the found waypoint and the previous one
        # This ensures that we do not create overlapping waypoints in time as there is not enough time gap
        if index > 0 and time_delta <= (self.time_waypoints[index - 1] - self.time_waypoints[index]):
            raise ValueError(
                f"Time delta {time_delta} is too small to postpone from time {start_time}. "
                "Not enough time gap between waypoints."
            )

        # Postpone all waypoints from the found index onwards by the time_delta
        affected_wps = self.waypoints[index:]

        for wp in affected_wps:
            wp.time += time_delta

        # Rebuild time_waypoints for the affected range: O(k) delete + O(k log N) re-insert
        del self.time_waypoints[index:]
        self.time_waypoints.update(wp.time for wp in affected_wps)

    def postpone(self, time_delta: float) -> None:
        """
        Postpone the entire Flight Plan by a given time delta.

        Args:
            - time_delta (float) : The amount of time to postpone the flight plan. Can be negative.
        """
        
        self.postpone_from(self.time_waypoints[0], time_delta)

    def reschedule_at(self, time: float) -> None:
        """
        Perform a temporal translation of the Flight Plan to begin at a given time.

        Args:
            - time (float) : The new start time for the Flight Plan.
        """

        self.postpone(time - self.time_waypoints[0])

    # ----------------------------------
    # -------- PLAN MANAGEMENT ---------
    # ----------------------------------
    def make_plan_feasible(self, is_time_key_aspect: bool, reconnect_waypoints: bool) -> None:
        """
        Makes the flight plan feasible by adjusting the times and velocities of the waypoints.

        Args:
            - is_time_key_aspect (bool) : If True, time is the key aspect and velocities will be adjusted. 
                                           If False, velocities are the key aspect and times will be adjusted.
            - reconnect_waypoints (bool) : If True, waypoints will be reconnected after adjustment.
        """

        # 1. If there are less than 2 waypoints, there's nothing to make feasible
        if self.length < 2:
            return
        
        # 2. Convert waypoints to arrays for easier manipulation
        waypoint_arrays = self.waypoints_to_arrays()
        labels = waypoint_arrays[0]
        times = waypoint_arrays[1]
        positions = waypoint_arrays[2]
        velocities = waypoint_arrays[3]
        headings = waypoint_arrays[8]

        # 3. Calculate distances, delta times, and velocity magnitudes
        distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        delta_times = np.diff(times)
        vel_magnitudes = np.linalg.norm(velocities, axis=1)

        # 3.1 Avoid division by zero by replacing zero magnitudes with a small value
        vel_magnitudes[vel_magnitudes == 0] = 1e-6

        # 4. Initialize new velocities and times (default to current values)
        new_velocities = velocities
        new_times = times

        # 5. Adjust velocities and times based on whether time is a key aspect or not
        if is_time_key_aspect:
            # If time is the key aspect, we adjust the velocities to ensure that the UAV can reach the next waypoint 
            # in the given time.
            normed_velocities = velocities[:-1] / vel_magnitudes[:-1, np.newaxis]
            new_vel_magnitudes = distances / delta_times
            new_velocities = normed_velocities * new_vel_magnitudes[:, np.newaxis]
            new_velocities = np.vstack((new_velocities, velocities[-1]))
            new_velocities = np.round(new_velocities, 4)

            # 6. Update the velocities of the waypoints
            for i, wp in enumerate(self.waypoints):
                wp.vel = new_velocities[i]

        else:
            # If velocity is the key aspect, we adjust the times to ensure that the UAV can reach the next waypoint
            new_delta_times = distances / vel_magnitudes[:-1]
            new_times = np.cumsum(np.concatenate(([times[0]], new_delta_times)))
            new_times = np.round(new_times, 4)

            # 6. Update the times of the waypoints
            self.time_waypoints = SortedList(new_times)

            self.waypoints = [
                Waypoint(
                    label=labels[i],
                    time=new_times[i],
                    pos=positions[i],
                    vel=new_velocities[i],
                    heading=headings[i]
                ) 
                for i in range(self.length)
            ]
            
        # 7. Reconnect waypoints if specified
        if reconnect_waypoints:
            self.connect_waypoints()

    def connect_waypoints(self) -> None:
        """
        Connects each waypoint to the next one in the flight plan.
        This method iterates through the waypoints and establishes a connection from each waypoint to the subsequent 
        one, allowing for smooth transitions between waypoints during flight.        
        """

        for i in range(self.length - 1):
            self.waypoints[i].connect_to(self.waypoints[i + 1])

    def smooth_waypoint_speed(self, waypoint: Union[str, int, float], ang_vel: float) -> None:
        """
        Smooths the curve at a specified waypoint by creating two new waypoints to replace the original one 
        while maintaining the speed.

        Args:
            - waypoint (Union[str, int, float]) : The label, index, or time of the waypoint to smooth.
            - ang_vel (float) : The angular velocity to be used for smoothing the curve.
        """

        # 1. Determine the index of the waypoint to smooth based on the input type (label or index)
        if isinstance(waypoint, str):
            wp_idx: int = self.labels_to_idx.get(waypoint, None)
        elif isinstance(waypoint, int):
            wp_idx = waypoint
        elif isinstance(waypoint, float):
            wp_idx = self.get_running_waypoint_idx(waypoint)
        else:
            raise TypeError(f"Invalid type for waypoint: {type(waypoint)}. Expected str, int, or float.")

        # 2. Check if the waypoint index is valid for smoothing (not the first or last waypoint).
        is_invalid_wp = (
            wp_idx is None or
            wp_idx == 0 or
            wp_idx == self.length - 1
        )

        if is_invalid_wp:
            raise RuntimeError(f"Trying to smooth invalid WP: {wp_idx}")

        # 3. Retrieve the waypoints before, at, and after the specified index
        wp_1 = self.waypoints[wp_idx - 1]
        wp_2 = self.waypoints[wp_idx]
        wp_3 = self.waypoints[wp_idx + 1]

        # 4. Check if the waypoint to smooth is a fly-over waypoint, which cannot be smoothed
        if wp_2.fly_over:
            raise RuntimeError(f"Trying to smooth a fly over WP (waypoint: {wp_2.label})")

        # 5. Calculate the angle between the two segments formed by the waypoints
        angle = wp_1.angle_with(wp_2)

        # 5.1 Check if the angle is zero, which indicates a straight line and cannot be smoothed
        if angle == 0:
            raise RuntimeError(f"Trying to smooth a straight line (waypoint: {wp_2.label})")

        # 6. Calculate the average velocity between the two waypoints to determine the radius of the curve
        # 6.1 Calculate the magnitudes of the velocities of the two waypoints
        vel_1 = math.sqrt(wp_1.vel[0]**2 + wp_1.vel[1]**2 + wp_1.vel[2]**2)
        vel_2 = math.sqrt(wp_2.vel[0]**2 + wp_2.vel[1]**2 + wp_2.vel[2]**2)

        # 6.2 Calculate the average velocity
        avg_vel = (vel_1 + vel_2) / 2

        # 7. Calculate the time step to create the new waypoints for smoothing based on the angle and average velocity
        radius          = avg_vel / ang_vel
        distance_offset = radius * math.tan(angle / 2)
        time_step       = distance_offset / avg_vel
        wp_2B_init_time = wp_2.time + time_step

        # 8. Early feasibility checks:
        # 8.1 Check if there is enough time in the past to create the first new waypoint (wp_2A) 
        # before the original waypoint (wp_2)
        if wp_2.time - time_step <= wp_1.time:
            raise Exception(f"There is not time enough in the past to smooth the curve (waypoint: {wp_2.label})")
        
        # 8.2 Check if there is enough time in the future to create the second new waypoint (wp_2B) 
        # after the original waypoint (wp_2)
        if wp_3.time <= wp_2B_init_time:
            raise Exception(f"There is not time enough in the future to smooth the curve (waypoint: {wp_2.label})")

        # 9. Create two new waypoints (wp_2A and wp_2B) to replace the original waypoint (wp_2)
        # 9.1 Create the first new waypoint (wp_2A) before the original waypoint (wp_2)
        wp_2A = Waypoint(
            label = f"{wp_2.label}_A",
            time  = wp_2.time - time_step,
            pos   = wp_2.pos - wp_1.vel * time_step,
            vel   = wp_1.vel
        )

        # 9.2 Create the second new waypoint (wp_2B) after the original waypoint (wp_2)
        wp_2B = Waypoint(
            label = f"{wp_2.label}_B",
            pos   = wp_2.pos + wp_2.vel * time_step,
            vel   = wp_2.vel
        )

        # 10. Find the optimal arc duration T = wp_2B.time - wp_2A.time analytically.
        #
        #     For a 5th-order polynomial arc with zero accelerations at both endpoints, solving
        #     the connect_to (Waypoint method) linear system symbolically gives the velocity at the midpoint as:
        #
        #         v(T/2) = (15/8)·(r₂−r₁)/T  −  (7/16)·(v₁+v₂)
        #
        #     Because r₂−r₁ = (v₁+v₂)·time_step by construction, v(T/2) is parallel to
        #     (v₁+v₂). Setting |v(T/2)| = avg_vel then yields the closed-form solution:
        #
        #         T = 30·time_step·|v₁+v₂| / (7·|v₁+v₂| + 16·avg_vel)

        w = wp_2A.vel + wp_2B.vel
        norm_w = math.sqrt(w[0]**2 + w[1]**2 + w[2]**2)

        if norm_w < 1e-12:
            raise RuntimeError(f"Cannot find arc duration: v1+v2 ≈ 0 at waypoint {wp_2.label}")

        T = 30.0 * time_step * norm_w / (7.0 * norm_w + 16.0 * avg_vel)
        wp_2B.time = wp_2A.time + T

        # 11. Connect new waypoints
        wp_1.connect_to(wp_2A)
        wp_2A.connect_to(wp_2B)
        wp_2B.connect_to(wp_3)

        # 12. Update the flight plan by removing the original waypoint (wp_2) and
        # adding the new waypoints (wp_2A and wp_2B)
        wp_2_idx = self.labels_to_idx[wp_2.label]

        self.remove_waypoint(wp_2_idx)
        self.add_waypoint(wp_2A)
        self.add_waypoint(wp_2B)

        # 13. Postpone the flight plan from wp_2B.time + 0.001 by the time difference between wp_2B.time and
        # its initial time to maintain the overall flight duration
        self.postpone_from(wp_2B.time + 0.001, wp_2B.time - wp_2B_init_time)

    def smooth_waypoint_duration(self, waypoint: Union[str, int], ang_vel: float, lin_acel: float) -> None:
        """
        Smooths the curve at a specified waypoint by creating two new waypoints to replace the original one
        while maintaining the duration of the curve.

        Args:
            - waypoint (Union[str, int]) : The label or index of the waypoint to smooth.
            - ang_vel (float) : The angular velocity to be used for smoothing the curve.
            - lin_acel (float) : The linear acceleration to be used for smoothing the curve
        """
        
        # 1. Check if ang_vel and lin_acel are valid (greater than zero)
        if ang_vel <= 0:
            raise ValueError("Angular velocity must be a positive value.")

        if lin_acel <= 0:
            raise ValueError("Linear acceleration must be a positive value.")

        # 2. Determine the index of the waypoint to smooth based on the input type (label or index)
        if isinstance(waypoint, str):
            wp_idx: int = self.labels_to_idx.get(waypoint, None)
        elif isinstance(waypoint, int):
            wp_idx = waypoint
        elif isinstance(waypoint, float):
            wp_idx = self.get_running_waypoint_idx(waypoint)
        else:
            raise TypeError(f"Invalid type for waypoint: {type(waypoint)}. Expected str, int, or float.")
            
        # 3. Check if the waypoint index is valid for smoothing (not the first or last waypoint).
        is_invalid_wp = (
            wp_idx is None or
            wp_idx == 0 or
            wp_idx == self.length - 1
        )

        if is_invalid_wp:
            raise RuntimeError(f"Trying to smooth invalid WP: {wp_idx}")
            
        # 4. Retrieve the waypoints before, at, and after the specified index
        wp_1 = self.waypoints[wp_idx - 1]
        wp_2 = self.waypoints[wp_idx]
        wp_3 = self.waypoints[wp_idx + 1]

        # 5. Check if the waypoint to smooth is a fly-over waypoint, which cannot be smoothed
        if wp_2.fly_over:
            raise RuntimeError(f"Trying to smooth a fly over WP (waypoint: {wp_2.label})")
        
        # 6. Calculate the angle between the two segments formed by the waypoints
        angle = wp_1.angle_with(wp_2)

        # 6.1 Check if the angle is zero, which indicates a straight line and cannot be smoothed
        if angle == 0:
            raise RuntimeError(f"Trying to smooth a straight line (waypoint: {wp_2.label})")

        # 7 Compute the time spent in the curve based on the linear acceleration and the difference in velocities
        # 7.1 Calculate the magnitudes of the velocities of the two waypoints
        vel_1 = math.sqrt(wp_1.vel[0]**2 + wp_1.vel[1]**2 + wp_1.vel[2]**2)
        vel_2 = math.sqrt(wp_2.vel[0]**2 + wp_2.vel[1]**2 + wp_2.vel[2]**2)
        
        # 7.2 Calculate the time spent in the curve based on the velocity and the  linear acceleration
        ts = abs(vel_2 - vel_1) / lin_acel

        # 8. Calculate the time spent in the curve based on the angle and the given angular velocity
        tc = angle / ang_vel

        # 9. Determine the time step to create the new waypoints for smoothing based on the maximum 
        # of the two calculated times
        time_step = max(tc, ts) / 2

        # 10. Check feasibility:
        # 10.1 Check if there is enough time in the past to create the first new waypoint (wp_2A)
        # before the original waypoint (wp_2)

        if time_step >= wp_2.time - wp_1.time:
            raise Exception(f"There is not time enough in the past to smooth the curve (waypoint: {wp_2.label})")
        
        # 10.2 Check if there is enough time in the future to create the second new waypoint (wp_2B)
        # after the original waypoint (wp_2)
        if time_step >= wp_3.time - wp_2.time:
            raise Exception(f"There is not time enough in the future to smooth the curve (waypoint: {wp_2.label})")
        
        # 11. Create two new waypoints (wp_2A and wp_2B) to replace the original waypoint (wp_2)
        # 11.1 Create the first new waypoint (wp_2A) before the original waypoint (wp_2)
        wp_2A = Waypoint(
            label = f"{wp_2.label}_A",
            time  = wp_2.time - time_step,
            pos   = wp_2.pos - wp_1.vel * time_step,
            vel   = wp_1.vel
        )

        # 11.2 Create the second new waypoint (wp_2B) after the original waypoint (wp_2)
        wp_2B = Waypoint(
            label = f"{wp_2.label}_B",
            time  = wp_2.time + time_step,
            pos   = wp_2.pos + wp_2.vel * time_step,
            vel   = wp_2.vel
        )

        # 12. Connect new waypoints
        wp_1.connect_to(wp_2A)
        wp_2A.connect_to(wp_2B)
        wp_2B.connect_to(wp_3)

        # 13. Update the flight plan by removing the original waypoint (wp_2) and
        # adding the new waypoints (wp_2A and wp_2B)
        wp_2_idx = self.labels_to_idx[wp_2.label]

        self.remove_waypoint(wp_2_idx)
        self.add_waypoint(wp_2A)
        self.add_waypoint(wp_2B)

    # ----------------------------------
    # -------- CORE FUNCTIONS ----------
    # ----------------------------------
    def status_at_time(self, time: float) -> Waypoint:
        """
        Check the status of the flight plan at a given time.
        
        Args:
            - time (float) : The time at which to check the status.
        
        Returns:
            Waypoint: The interpolated waypoint status at the given time.
        """

        # 1. Handle edge cases for time before the first waypoint and after the last waypoint
        if time <= self.time_waypoints[0]: 
            return self.waypoints[0]
        
        if time == self.time_waypoints[-1]:
            return self.waypoints[-1]

        if time > self.time_waypoints[-1]:
            return self.waypoints[-1].interpolation(time)

        # 2. Get the running waypoint for the given time and return its interpolated status
        running_wp = self.get_running_waypoint(time)

        return running_wp.interpolation(time)
    
    def trace(self, time_step: float) -> np.ndarray:
        """
        Compute the UAV's trace over time based on the flightplan's waypoints.

        Args:
            - time_step (float) : The time step to compute the trace.

        Returns:
            - np.ndarray : Array of position, velocity and acceleration at time instants sample with the given time step.
        """

        # 1. Get waypoints as arrays for easier manipulation
        waypoint_arrays = self.waypoints_to_arrays()
        times = waypoint_arrays[1]
        positions = waypoint_arrays[2]
        velocities = waypoint_arrays[3]
        accelerations = waypoint_arrays[4]
        jerks = waypoint_arrays[5]
        snaps = waypoint_arrays[6]
        crackles = waypoint_arrays[7]

        # 2. Compute time instants according to the given time_step
        instants = np.arange(times[0], times[-1] + time_step, time_step)

        # 3. Relate each instant to the last waypoint whose time <= t.
        interpolation_idxs = np.searchsorted(times, instants, side="right") - 1

        # 4. Compute the time difference between each instant and the corresponding waypoint time
        dt = (instants - times[interpolation_idxs])[:, np.newaxis]

        # 5. Compute the status at each instant using the Taylor series expansion
        traced_positions = (
            positions[interpolation_idxs] + 
            velocities[interpolation_idxs] * dt + 
            0.5 * accelerations[interpolation_idxs] * dt**2 + 
            (1/6) * jerks[interpolation_idxs] * dt**3 + 
            (1/24) * snaps[interpolation_idxs] * dt**4 + 
            (1/120) * crackles[interpolation_idxs] * dt**5
        )

        traced_velocities = (
            velocities[interpolation_idxs] + 
            accelerations[interpolation_idxs] * dt + 
            0.5 * jerks[interpolation_idxs] * dt**2 + 
            (1/6) * snaps[interpolation_idxs] * dt**3 + 
            (1/24) * crackles[interpolation_idxs] * dt**4
        )

        traced_accelerations = (
            accelerations[interpolation_idxs] + 
            jerks[interpolation_idxs] * dt + 
            0.5 * snaps[interpolation_idxs] * dt**2 + 
            (1/6) * crackles[interpolation_idxs] * dt**3
        )

        traced_jerks = (
            jerks[interpolation_idxs] + 
            snaps[interpolation_idxs] * dt + 
            0.5 * crackles[interpolation_idxs] * dt**2
        )

        traced_snaps = (
            snaps[interpolation_idxs] + 
            crackles[interpolation_idxs] * dt
        )

        traced_crackles = crackles[interpolation_idxs]

        # 6. Combine all traced data into a single array
        trace = np.column_stack((
            instants, 
            traced_positions, 
            traced_velocities, 
            traced_accelerations, 
            traced_jerks, 
            traced_snaps, 
            traced_crackles
        ))

        return trace

    # ----------------------------------
    # -------- NAVIGATION --------------
    # ----------------------------------
    def get_command(self, time, pos, lin_vel, yaw, heading, t_to_solve) -> Command:
        """
        Compute the navigation command for the UAV based on its current state and the flight plan.

        Args:
            - time (float) : The current time.
            - pos (np.ndarray) : The current position of the UAV.
            - lin_vel (np.ndarray) : The current linear velocity of the UAV.
            - yaw (float) : The current yaw of the UAV.
            - heading (np.ndarray) : The desired heading direction for the UAV.
            - t_to_solve (float) : The time duration over which to compute the command
        """
        
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
        # varVelMagnitude = np.linalg.norm(varVel)
        # if varVelMagnitude > self.max_var_lin_vel:
        #     varVel /= varVelMagnitude # Normalize
        #     varVel *= self.max_var_lin_vel
        # print("varVel:", varVel)

        cmdVel = UAVvel + varVel
        # print("cmdVel2:", cmdVel)

        # COMPUTING DRONE RELATIVE LINEAR VELOCITY
        cmdRelVel = UAVrot.inv().apply(cmdVel)
        # print("cmdRelVel:", cmdRelVel)

        # COMPUTING TARGET ERROR YAW
        if WPheading is None:
            targetDir = expected.vel.copy()
            targetDir[2] = 0

        else:
            targetDir = WPheading

        if np.linalg.norm(targetDir) > 0:
            self.target_yaw = np.arctan2(targetDir[1], targetDir[0])
        elif self.target_yaw is None:
            self.target_yaw = UAVyaw

        errorYaw = self.target_yaw - UAVyaw
        while errorYaw < -np.pi:
            errorYaw += 2*np.pi

        while np.pi < errorYaw:
            errorYaw -= 2*np.pi

        # COMPUTING TARGET ANGULAR VELOCITY
        currentWel = errorYaw / tToSolve
        
        # if currentWel < -self.max_var_ang_vel:
        #     currentWel = -self.max_var_ang_vel
        
        # if self.max_var_ang_vel < currentWel:
        #     currentWel = self.max_var_ang_vel

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
    
    def get_isaacsim_command(self, time, pos, lin_vel, yaw, heading, t_to_solve):
        # EXPECTED UAV POSE
        expected = self.status_at_time(time)

        # COMPUTING CORRECTION VELOCITY (to achieve status.pos in 'tToSolve' seconds)
        correction_vel = (expected.pos - pos) / t_to_solve

        # COMPUTING COMMANDED VELOCITY
        command_linear_vel = expected.vel + correction_vel

        # SMOOTHING COMMANDED VELOCITY
        variation_vel = command_linear_vel - lin_vel
        command_linear_vel = lin_vel + variation_vel

        # COMPUTING TARGET ERROR YAW
        if heading is None:
            target_heading = expected.vel[:2]
        else:
            target_heading = heading

        if np.linalg.norm(target_heading) > 0:
            self.target_yaw = np.arctan2(target_heading[1], target_heading[0])
        elif self.target_yaw is None:
            self.target_yaw = yaw

        yaw_error = self.target_yaw - yaw
        # Normalize to [-pi, pi]
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

        # COMPUTING TARGET ANGULAR VELOCITY
        command_yaw_rotation = yaw_error / t_to_solve

        return command_linear_vel, command_yaw_rotation

    # ----------------------------------
    # -------- CONFLICT DETECTION ------
    # ----------------------------------
    def compare_to(self, fp2, time_step):
        decimals = len(str(time_step).split(".")[1])

        trace_1 = self.trace(time_step)
        trace_2 = fp2.trace(time_step)

        trace_1_times = np.round(trace_1[:, 0], decimals)
        trace_2_times = np.round(trace_2[:, 0], decimals)

        # Early return when the two plans do not overlap in time
        overlap_start = max(trace_1_times[0], trace_2_times[0])
        overlap_end   = min(trace_1_times[-1], trace_2_times[-1])
        if overlap_start > overlap_end:
            return [], np.array([])

        init_trace_1 = [[0]]
        init_trace_2 = [[0]]
        end_trace_1 = [[len(trace_1_times) - 1]]
        end_trace_2 = [[len(trace_2_times) - 1]]

        if trace_1_times[0] < trace_2_times[0]:     init_trace_1 = np.where(trace_1_times == trace_2_times[0])
        else:                                       init_trace_2 = np.where(trace_2_times == trace_1_times[0])

        if trace_1_times[-1] < trace_2_times[-1]:   end_trace_2 = np.where(trace_2_times == trace_1_times[-1])
        else:                                       end_trace_1 = np.where(trace_1_times == trace_2_times[-1])

        i1, e1 = init_trace_1[0][0], end_trace_1[0][0]
        i2, e2 = init_trace_2[0][0], end_trace_2[0][0]

        # Vectorised Euclidean distance: no Python loop, no redundant np.abs
        diff = trace_1[i1:e1, 1:4] - trace_2[i2:e2, 1:4]
        distances = np.linalg.norm(diff, axis=1)

        return distances, trace_1_times[i1:e1]

    # ----------------------------------
    # -------- INFO & FIGURES ----------
    # ----------------------------------
    def print_waypoints(self) -> None:
        """Prints all waypoints in the flight plan with their time, position, and velocity."""
        table = [
            [wp.label, wp.time, wp.pos, wp.vel, wp.acel, wp.jerk, wp.snap, wp.crakle] 
            for wp in self.waypoints
        ]
        print(tabulate(
            table, 
            headers=[
                "Label", 
                "Time", 
                "Position", 
                "Velocity", 
                "Acceleration", 
                "Jerk", 
                "Snap", 
                "Crakle"
            ]
        ))

    def attach_cursor_annotations(self, cursor, waypoints):
        @cursor.connect("add")
        def on_add(sel):
            wp = waypoints[sel.index]
            text = f"T: {wp.time}\n"
            text += f"POS: {wp.pos}\n"
            text += f"VEL: {wp.vel}\n"
            text += f"ACEL: {wp.acel}"

            sel.annotation.set_text(text)

    def position_figure(self, figName, timeStep):
        # Display the flight plan trajectory
        
        # Check if the flight plan is empty
        if not self.waypoints:
            print('The flight plan is empty')
            return

        # Build a process to show the figure without blocking the main thread
        process = mp.Process(target=self.position_figure_process, args=(figName, timeStep, ))
        self.figure_processes.append(process)
        process.start()

    def position_figure_process(self, figName, timeStep):
        # Create matplolib figure (window)
        posFig = plt.figure(figName)
        posFig.canvas.manager.toolmanager.add_tool("ToogleUAV", ToggleUAVtracking, gid="UAVtracking")
        posFig.canvas.manager.toolbar.add_tool('ToogleUAV', 'navigation', 1)

        # Figure settings
        color = [0, 0.7, 1]

        # Get the trace
        tr = self.trace(timeStep)
        tr_t = tr[:, 0]
        tr_x = tr[:, 1]
        tr_y = tr[:, 2]
        tr_z = tr[:, 3]

        # POSITION ERROR VERSUS TIME
        # Create plot
        xyzPosErrorPlot = posFig.add_subplot(6, 5, (26, 28))

        # Indicate axes' name
        xyzPosErrorPlot.set_xlabel("t [s]")
        xyzPosErrorPlot.set_ylabel("Error [m]")

        # Set title
        xyzPosErrorPlot.set_title("Position error versus time")

        # Set grid to True
        xyzPosErrorPlot.grid(True)

        # POSITION 3D
        # Create plot
        xyzPosPlot = posFig.add_subplot(6, 5, (1, 23), projection="3d")
        
        # Indicate axes' name
        xyzPosPlot.set_xlabel("x [m]")
        xyzPosPlot.set_ylabel("y [m]")
        xyzPosPlot.set_zlabel("z [m]")
        
        # Set title
        xyzPosPlot.set_title("Position 3D")
        
        # Set grid to True
        xyzPosPlot.grid(True)
        
        # Set plot info
        xyzPosPlot.plot(tr_x, tr_y, tr_z, linewidth=2, color=color, zorder=1)

        # POSITIONS VERSUS TIME
        # Create plots
        xPosTimePlot = posFig.add_subplot(6, 5, (4, 10))
        yPosTimePlot = posFig.add_subplot(6, 5, (14, 20))
        zPosTimePlot = posFig.add_subplot(6, 5, (24, 30))
        
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
        xPosTimePlot.plot(tr_t, tr_x, linewidth=2, color=color, zorder=1)
        yPosTimePlot.plot(tr_t, tr_y, linewidth=2, color=color, zorder=1)
        zPosTimePlot.plot(tr_t, tr_z, linewidth=2, color=color, zorder=1)

        # Get waypoints positions to highlight
        xPos = []
        yPos = []
        zPos = []
        t = []

        for wp in self.waypoints:
            xPos.append(wp.pos[0])
            yPos.append(wp.pos[1])
            zPos.append(wp.pos[2])
            t.append(wp.time)

        # Highlight waypoints positions
        xyzPosPlot_scatter = xyzPosPlot.scatter(xPos, yPos, zPos, marker="o", color="blue", s=25, pickradius=30, zorder=3)
        xPosTimePlot_scatter = xPosTimePlot.scatter(t, xPos, marker="o", color="blue", s=25, pickradius=30, zorder=3)
        yPosTimePlot_scatter = yPosTimePlot.scatter(t, yPos, marker="o", color="blue", s=25, pickradius=30, zorder=3)
        zPosTimePlot_scatter = zPosTimePlot.scatter(t, zPos, marker="o", color="blue", s=25, pickradius=30, zorder=3)

        xyzPosPlot_cursor = mplcursors.cursor(xyzPosPlot_scatter, highlight=True)
        xPosTimePlot_cursor = mplcursors.cursor(xPosTimePlot_scatter, highlight=True)
        yPosTimePlot_cursor = mplcursors.cursor(yPosTimePlot_scatter, highlight=True)
        zPosTimePlot_cursor = mplcursors.cursor(zPosTimePlot_scatter, highlight=True)

        self.attach_cursor_annotations(xyzPosPlot_cursor, self.waypoints)
        self.attach_cursor_annotations(xPosTimePlot_cursor, self.waypoints)
        self.attach_cursor_annotations(yPosTimePlot_cursor, self.waypoints)
        self.attach_cursor_annotations(zPosTimePlot_cursor, self.waypoints)

        # Update limits to maintain scale in all axes
        xLim = max(np.abs(xyzPosPlot.get_xlim3d()))
        yLim = max(np.abs(xyzPosPlot.get_ylim3d()))
        zLim = max(np.abs(xyzPosPlot.get_zlim3d()))
        maxLim = max(xLim, yLim, zLim)

        xyzPosPlot.set_xlim3d(-maxLim, maxLim)
        xyzPosPlot.set_ylim3d(-maxLim, maxLim)
        xyzPosPlot.set_zlim3d(-maxLim, maxLim)

        xLim = xPosTimePlot.get_ylim()
        yLim = yPosTimePlot.get_ylim()
        xRange = xLim[1] - xLim[0]
        yRange = yLim[1] - yLim[0]
        
        maxRange = max(xRange, yRange)
        addition = maxRange / 2

        xMidValue = (xLim[1] + xLim[0]) / 2
        yMidValue = (yLim[1] + yLim[0]) / 2

        xPosTimePlot.set_ylim(xMidValue - addition, xMidValue + addition)
        yPosTimePlot.set_ylim(yMidValue - addition, yMidValue + addition)

        plt.show()

    def velocity_figure(self, figName, timeStep):
        # Display the flight plan instant velocity

        # Check if the flight plan is empty
        if not self.waypoints:
            print('The flight plan is empty')
            return
        
        # Build a process to show the figure without blocking the main thread
        process = mp.Process(target=self.velocity_figure_process, args=(figName, timeStep, ))
        self.figure_processes.append(process)
        process.start()

    def velocity_figure_process(self, figName, timeStep):
        # Create matplolib figure (window)
        velFig = plt.figure(figName)
        velFig.canvas.manager.toolmanager.add_tool("ToogleUAV", ToggleUAVtracking, gid="UAVtracking")
        velFig.canvas.manager.toolbar.add_tool('ToogleUAV', 'navigation', 1)

        # Figure settings
        color = [0, 0.7, 1]

        # Get the trace
        tr = self.trace(timeStep)
        tr_t = tr[:, 0]
        tr_x = tr[:, 4]
        tr_y = tr[:, 5]
        tr_z = tr[:, 6]

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
        velPlot3D.plot(tr_t, np.sqrt(tr_x**2 + tr_y**2 + tr_z**2), linewidth=2, color=color, zorder=1)

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
        xVelTimePlot.plot(tr_t, tr_x, linewidth=2, color=color, zorder=1)
        yVelTimePlot.plot(tr_t, tr_y, linewidth=2, color=color, zorder=1)
        zVelTimePlot.plot(tr_t, tr_z, linewidth=2, color=color, zorder=1)

        # Get waypoints velocities to highlight
        xVel = []
        yVel = []
        zVel = []
        t = []

        for wp in self.waypoints:
            xVel.append(wp.vel[0])
            yVel.append(wp.vel[1])
            zVel.append(wp.vel[2])
            t.append(wp.time)

        # Highlight waypoints positions
        xVelTimePlot_scatter = xVelTimePlot.scatter(t, xVel, marker="o", color="blue", s=25, pickradius=30, zorder=3)
        yVelTimePlot_scatter = yVelTimePlot.scatter(t, yVel, marker="o", color="blue", s=25, pickradius=30, zorder=3)
        zVelTimePlot_scatter = zVelTimePlot.scatter(t, zVel, marker="o", color="blue", s=25, pickradius=30, zorder=3)

        xVelTimePlot_cursor = mplcursors.cursor(xVelTimePlot_scatter, highlight=True)
        yVelTimePlot_cursor = mplcursors.cursor(yVelTimePlot_scatter, highlight=True)
        zVelTimePlot_cursor = mplcursors.cursor(zVelTimePlot_scatter, highlight=True)

        self.attach_cursor_annotations(xVelTimePlot_cursor, self.waypoints)
        self.attach_cursor_annotations(yVelTimePlot_cursor, self.waypoints)
        self.attach_cursor_annotations(zVelTimePlot_cursor, self.waypoints)

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

        plt.show()

    def acceleration_figure(self, figName, timeStep):
        # Display the flight plan instant velocity

        # Check if the flight plan is empty
        if not self.waypoints:
            print('The flight plan is empty')
            return

        # Build a process to show the figure without blocking the main thread
        process = mp.Process(target=self.acceleration_figure_process, args=(figName, timeStep))
        self.figure_processes.append(process)
        process.start()

    def acceleration_figure_process(self, figName, timeStep):
        # Create matplolib figure (window)
        accFig = plt.figure(figName)

        # Figure settings
        color = [0, 0.7, 1]

        # Get the trace
        tr = self.trace(timeStep)
        tr_t = tr[:, 0]
        tr_x = tr[:, 7]
        tr_y = tr[:, 8]
        tr_z = tr[:, 9]

        # ACCELERATION 3D
        # Create plot
        accPlot3D = accFig.add_subplot(4, 2, (1, 2))
        
        # Indicate axes' name
        accPlot3D.set_ylabel("3D [m/s2]")
        
        # Set title
        accPlot3D.set_title("Acceleration versus time")
        
        # Set grid to True
        accPlot3D.grid(True)
        
        # Set plot info
        accPlot3D.plot(tr_t, np.sqrt(tr_x**2 + tr_y**2 + tr_z**2), linewidth=2, color=color, zorder=1)

        # ACCELERATIONS VERSUS TIME
        # Create plots
        xAccTimePlot = accFig.add_subplot(4, 2, (3, 4))
        yAccTimePlot = accFig.add_subplot(4, 2, (5, 6))
        zAccTimePlot = accFig.add_subplot(4, 2, (7, 8))
        
        # Indicate axes' names
        xAccTimePlot.set_ylabel("ax [m/s2]")
        yAccTimePlot.set_ylabel("ay [m/s2]")
        zAccTimePlot.set_ylabel("az [m/s2]")
        zAccTimePlot.set_xlabel("t [s]")
        
        # Set grid to True
        xAccTimePlot.grid(True)
        yAccTimePlot.grid(True)
        zAccTimePlot.grid(True)
        
        # Set plots info
        xAccTimePlot.plot(tr_t, tr_x, linewidth=2, color=color, zorder=1)
        yAccTimePlot.plot(tr_t, tr_y, linewidth=2, color=color, zorder=1)
        zAccTimePlot.plot(tr_t, tr_z, linewidth=2, color=color, zorder=1)

        # Get waypoints velocities to highlight
        xAcc = []
        yAcc = []
        zAcc = []
        t = []

        for wp in self.waypoints:
            xAcc.append(wp.acel[0])
            yAcc.append(wp.acel[1])
            zAcc.append(wp.acel[2])
            t.append(wp.time)

        # Highlight waypoints positions
        xVelTimePlot_scatter = xAccTimePlot.scatter(t, xAcc, marker="o", color="blue", s=25, pickradius=30, zorder=2)
        yVelTimePlot_scatter = yAccTimePlot.scatter(t, yAcc, marker="o", color="blue", s=25, pickradius=30, zorder=2)
        zVelTimePlot_scatter = zAccTimePlot.scatter(t, zAcc, marker="o", color="blue", s=25, pickradius=30, zorder=2)

        xVelTimePlot_cursor = mplcursors.cursor(xVelTimePlot_scatter, highlight=True)
        yVelTimePlot_cursor = mplcursors.cursor(yVelTimePlot_scatter, highlight=True)
        zVelTimePlot_cursor = mplcursors.cursor(zVelTimePlot_scatter, highlight=True)

        self.attach_cursor_annotations(xVelTimePlot_cursor, self.waypoints)
        self.attach_cursor_annotations(yVelTimePlot_cursor, self.waypoints)
        self.attach_cursor_annotations(zVelTimePlot_cursor, self.waypoints)

        # Update limits to maintain scale in all axes
        lim3D = max(np.abs(accPlot3D.get_ylim()))
        xLim = max(np.abs(xAccTimePlot.get_ylim()))
        yLim = max(np.abs(yAccTimePlot.get_ylim()))
        zLim = max(np.abs(zAccTimePlot.get_ylim()))
        maxLim = max(lim3D, xLim, yLim, zLim)

        accPlot3D.set_ylim(-maxLim, maxLim)
        xAccTimePlot.set_ylim(-maxLim, maxLim)
        yAccTimePlot.set_ylim(-maxLim, maxLim)
        zAccTimePlot.set_ylim(-maxLim, maxLim)

        plt.show()

    def add_UAV_track_pos(self, figName, UAVinfo : List[Waypoint]):
        posFig = plt.figure(figName)
        subplots = posFig.get_axes()
        xyzPosErrorPlot = subplots[0]
        xyzPosPlot = subplots[1]
        xPosTimePlot = subplots[2]
        yPosTimePlot = subplots[3]
        zPosTimePlot = subplots[4]

        xPosUAV = []
        yPosUAV = []
        zPosUAV = []
        timeUAV = []
        errors = []

        for wp in UAVinfo:
            xPosUAV.append(wp.pos[0])
            yPosUAV.append(wp.pos[1])
            zPosUAV.append(wp.pos[2])
            timeUAV.append(wp.time)

            # Compute errors between UAV and flight plan
            status = self.status_at_time(wp.time)
            error = np.linalg.norm(wp.pos - status.pos)
            errors.append(error)

        # Plot UAV errors
        xyzPosErrorPlot.plot(timeUAV, errors, linestyle="solid", linewidth=1, color="red")

        # Plot UAV route
        xyzPosPlot.plot(xPosUAV, yPosUAV, zPosUAV, linestyle="dashed", linewidth=1, color="black", 
                                     gid="UAVtracking", zorder=2)
        xPosTimePlot.plot(timeUAV, xPosUAV, linestyle="dashed", linewidth=1, color="black", 
                                         gid="UAVtracking", zorder=2)
        yPosTimePlot.plot(timeUAV, yPosUAV, linestyle="dashed", linewidth=1, color="black", 
                                         gid="UAVtracking", zorder=2)
        zPosTimePlot.plot(timeUAV, zPosUAV, linestyle="dashed", linewidth=1, color="black", 
                                         gid="UAVtracking", zorder=2)

        # Highlight UAV route positions
        xyzPosPlot.scatter(xPosUAV, yPosUAV, zPosUAV, color="black", s=10, gid="UAVtracking", zorder=2)
        xPosTimePlot.scatter(timeUAV, xPosUAV, color="black", s=10, gid="UAVtracking", zorder=2)
        yPosTimePlot.scatter(timeUAV, yPosUAV, color="black", s=10, gid="UAVtracking", zorder=2)
        zPosTimePlot.scatter(timeUAV, zPosUAV, color="black", s=10, gid="UAVtracking", zorder=2)

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
            timeUAV.append(wp.time)

        xVelUAV = np.array(xVelUAV)
        yVelUAV = np.array(yVelUAV)
        zVelUAV = np.array(zVelUAV)
        
        # Plot UAV route
        velPlot3D.plot(timeUAV, np.sqrt(xVelUAV**2 + yVelUAV**2 + zVelUAV**2), linestyle="dashed", linewidth=1, 
                       color="black", gid="UAVtracking", zorder=2)
        xVelTimePlot.plot(timeUAV, xVelUAV, linestyle="dashed", linewidth=1, color="black", gid="UAVtracking", zorder=2)
        yVelTimePlot.plot(timeUAV, yVelUAV, linestyle="dashed", linewidth=1, color="black", gid="UAVtracking", zorder=2)
        zVelTimePlot.plot(timeUAV, zVelUAV, linestyle="dashed", linewidth=1, color="black", gid="UAVtracking", zorder=2)

        # Highlight UAV route positions
        velPlot3D.scatter(timeUAV, np.sqrt(xVelUAV**2 + yVelUAV**2 + zVelUAV**2), color="black", s=10, gid="UAVtracking", 
                          zorder=2)
        xVelTimePlot.scatter(timeUAV, xVelUAV, color="black", s=10, gid="UAVtracking", zorder=2)
        yVelTimePlot.scatter(timeUAV, yVelUAV, color="black", s=10, gid="UAVtracking", zorder=2)
        zVelTimePlot.scatter(timeUAV, zVelUAV, color="black", s=10, gid="UAVtracking", zorder=2)

    def add_UAV_track_acc(self, figName, UAVinfo : List[Waypoint]):
        posFig = plt.figure(figName)
        subplots = posFig.get_axes()
        accPlot3D = subplots[0]
        xAccTimePlot = subplots[1]
        yAccTimePlot = subplots[2]
        zAccTimePlot = subplots[3]

        xVelUAV = []
        yVelUAV = []
        zVelUAV = []
        timeUAV = []

        for wp in UAVinfo:
            xVelUAV.append(wp.vel[0])
            yVelUAV.append(wp.vel[1])
            zVelUAV.append(wp.vel[2])
            timeUAV.append(wp.time)

        xAccUAV = np.diff(xVelUAV, prepend=xVelUAV[:1])
        yAccUAV = np.diff(yVelUAV, prepend=yVelUAV[:1])
        zAccUAV = np.diff(zVelUAV, prepend=zVelUAV[:1])
        
        # Plot UAV route
        accPlot3D.plot(timeUAV, np.sqrt(xAccUAV**2 + yAccUAV**2 + zAccUAV**2), linestyle="dashed", linewidth=1, 
                       color="black", gid="UAVtracking", zorder=2)
        xAccTimePlot.plot(timeUAV, xAccUAV, linestyle="dashed", linewidth=1, color="black", gid="UAVtracking", zorder=2)
        yAccTimePlot.plot(timeUAV, yAccUAV, linestyle="dashed", linewidth=1, color="black", gid="UAVtracking", zorder=2)
        zAccTimePlot.plot(timeUAV, zAccUAV, linestyle="dashed", linewidth=1, color="black", gid="UAVtracking", zorder=2)

        # Highlight UAV route positions
        accPlot3D.scatter(timeUAV, np.sqrt(xAccUAV**2 + yAccUAV**2 + zAccUAV**2), color="black", s=10, gid="UAVtracking", 
                          zorder=2)
        xAccTimePlot.scatter(timeUAV, xAccUAV, color="black", s=10, gid="UAVtracking", zorder=2)
        yAccTimePlot.scatter(timeUAV, yAccUAV, color="black", s=10, gid="UAVtracking", zorder=2)
        zAccTimePlot.scatter(timeUAV, zAccUAV, color="black", s=10, gid="UAVtracking", zorder=2)

    def terminate_figure_processes(self):
        for process in self.figure_processes:
            process.terminate()
        self.figure_processes = []

class ToggleUAVtracking(ToolToggleBase):
    default_keymap = 'S'
    description = 'Show by gid'
    default_toggled = True

    def __init__(self, *args, gid, **kwargs):
        self.gid = gid
        super().__init__(*args, **kwargs)

    def enable(self, *args):
        self.set_lines_visibility(True)

    def disable(self, *args):
        self.set_lines_visibility(False)

    def set_lines_visibility(self, state):
        for ax in self.figure.get_axes():
            for line in ax.get_lines():
                if line.get_gid() == self.gid:
                    line.set_visible(state)

            scatter_plots = [coll for coll in ax.collections if isinstance(coll, PathCollection)]
            for scatter in scatter_plots:
                if scatter.get_gid() == self.gid:
                    scatter.set_visible(state)
        self.figure.canvas.draw()
