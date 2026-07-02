from tabulate import tabulate
from typing import List, Optional, Any, Union, Iterable
import math
import numpy as np
import matplotlib
import mplcursors
import matplotlib.pyplot as plt
from matplotlib.backend_tools import ToolToggleBase
from matplotlib.collections import PathCollection
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
    ):
        self.id: str = id
        self.priority: int = priority
        self.radius: float = radius
        self.max_var_lin_vel = max_var_lin_vel        # maximum variation in linear  velocity   [  m/s]
        self.max_var_ang_vel = max_var_ang_vel        # maximum variation in angular velocity   [rad/s]
        self.waypoints: List[Waypoint] = []
        self.time_waypoints: SortedList[float] = SortedList([])
        self.ids_to_idx = {}
        self.length: int = 0
        self.time_decimals = 3
        self.position_decimals = 3
        self.velocity_decimals = 3

    def __repr__(self):
        waypoints_info = [
            [wp.id, wp.time, wp.pos, wp.vel, wp.acel, wp.jerk, wp.snap, wp.crakle] 
            for wp in self.waypoints
        ]
        waypoints_headers = (
            'ID', 
            'Time', 
            'Position', 
            'Velocity', 
            'Acceleration', 
            'Jerk', 
            'Snap', 
            'Crakle'
        )
        return (
            f"FlightPlan with id '{self.id}':"
            f"\n\tPriority: {self.priority}"
            f"\n\tRadius: {self.radius}"
            f"\n\tMax Var Lin Vel: {self.max_var_lin_vel}"
            f"\n\tMax Var Ang Vel: {self.max_var_ang_vel}"
            f"\n\tWaypoints ({self.length}):"
            f"\n\n{tabulate(waypoints_info, headers=waypoints_headers)}"
        )

    # ----------------------------------
    # -------- BASE FUNCTIONS ----------
    # ----------------------------------
    def add_waypoint(
        self, 
        wp: Optional[Waypoint] = None, 
        id: Optional[str] = None, 
        time: float = None, 
        pos: Optional[Iterable[float]] = None, 
        vel: Optional[Iterable[float]] = None, 
        heading: Iterable[float] = [0,0]
    ) -> None:
        """
        Add a waypoint to the flight plan.
        In case of missing parameters (pos or vel), the function will attempt to infer 
        them based on the existing waypoints and their properties.
        If a waypoint object is provided, it will be added directly.

        Args:
            - wp (Waypoint, optional) : A Waypoint object to be added. If provided, other parameters are ignored.
            - id (str, optional) : The identifier for the waypoint. If not provided, a default id will be generated.
            - time (float) : The time at which the waypoint should be reached. This is a required parameter.
            - pos (Iterable[float], optional) : The position of the waypoint as a list of [x, y, z]. If not provided, it will be inferred.
            - vel (Iterable[float], optional) : The velocity at the waypoint as a list of [vx, vy, vz]. If not provided, it will be inferred.
            - heading (Iterable[float], optional) : The heading of the waypoint as a list of [1, 0.5]. Default value is [0, 0].
        """

        # 1. Check if the time parameter is provided, 
        #    as it is essential for adding a waypoint
        if time is None:
            raise ValueError("Time must be provided for the waypoint.")
        
        # 2. If a Waypoint object is not provided, create one using the given parameters
        if wp is None:
            # 2.1 Stablish the conditions to infer missing parameters (id, pos, vel)
            is_first_wp         =   self.length == 0
            is_id_none          =   id is None
            is_pos_none         =   pos is None
            is_vel_none         =   vel is None
            is_previous_time   =   not is_first_wp and time <= self.time_waypoints[0]

            # 2.2 If the waypoint is not the first one, 
            #     get the expected status at the given time
            if not is_first_wp:
                expected_status = self.status_at_time(time)

            # 2.3 Infer missing parameters based on the conditions and expected status
            if is_pos_none:
                if is_first_wp:
                    pos = [0, 0, 0]
                else:
                    pos = expected_status.pos

            if is_vel_none:
                if is_first_wp or is_previous_time:
                    vel = [0, 0, 0]
                else:
                    vel = expected_status.vel

            if is_id_none:
                if is_first_wp:
                    id = "default_id_0"
                else:
                    id = f"default_id_{self.length}"

            # 2.4 Create a new Waypoint object with the provided or inferred parameters
            wp = Waypoint(id=id, time=time, pos=pos, vel=vel, heading=heading)
        
        # 3. Determine the index to insert the new waypoint based on its time
        index = self.time_waypoints.bisect_left(wp.time)

        # 4. Add id to the dictionary for quick access
        self.ids_to_idx[wp.id] = index

        # 5. Replace the waypoint if the same time already exists
        update_existing_wp = (
            (index < self.length and self.time_waypoints[index] == wp.time) or
            (index > 0 and index == self.length and self.time_waypoints[index - 1] == wp.time)
        )
        
        if update_existing_wp:
            old_wp = self.waypoints.pop(index)
            self.time_waypoints.pop(index)
            self.ids_to_idx.pop(old_wp.id, None)

            self.length -= 1

        # 6. Insert the new waypoint and its time into the corresponding data structures
        self.waypoints.insert(index, wp)
        self.time_waypoints.add(wp.time)

        # 7. Update the length of the flight plan to reflect the addition of the new waypoint
        self.length += 1

    def remove_waypoint(
        self, 
        idx: Optional[int] = None,  
        # id: Optional[str], 
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
            self.ids_to_idx.pop(wp.id, None)
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
            self.ids_to_idx.pop(wp.id, None)
            self.length -= 1
            return
        
        # Remove by time
        if time >= self.time_waypoints[-1]:
            pointed_wp_idx = self.length - 1
        else:
            pointed_wp_idx = self.time_waypoints.bisect_left(time)
        
        wp = self.waypoints.pop(pointed_wp_idx)
        self.time_waypoints.pop(pointed_wp_idx)
        self.ids_to_idx.pop(wp.id, None)
        self.length -= 1
        return

    def get_idx_by_id(self, id: str) -> Optional[int]:
        """
        Get the index of a waypoint by its id.

        Args:
            - id (str) : The id of the waypoint to find.

        Returns:
            Optional[int] : The index of the waypoint if found, otherwise None.
        """

        if id in self.ids_to_idx:
            return self.ids_to_idx[id]
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
        flight_plan.ids_to_idx = dict(self.ids_to_idx)
            
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
                "ids":   [wp.id    for wp in self.waypoints],
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
        self.ids_to_idx = {}
        self.length = 0

        # 3. Load waypoints
        wps_data = data.get("waypoints", {})
        if not wps_data:
            return

        # 4. Extract waypoint attributes from the dictionary
        ids   = wps_data.get("ids",   [])
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
        for i in range(len(ids)):
            wp = Waypoint(
                id=ids[i],
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
            self.ids_to_idx[wp.id] = i
            self.length += 1

    def waypoints_to_arrays(self) -> List[np.ndarray]:
        """
        Convert the waypoints to separate numpy arrays for each attribute.

        Returns:
            - List[np.ndarray] : A list of numpy arrays containing the attributes of each waypoint,
            in this order: [times, positions, velocities, accelerations, jerks, snaps, crackles, headings].

        Example::

            flight_plan = FlightPlan()
            flight_plan.add_waypoint(id="WP1", time=0, pos=[0, 0, 0], vel=[1, 0, 0])
            flight_plan.add_waypoint(id="WP2", time=1, pos=[1, 0, 0], vel=[1, 0, 0])
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

        ids = np.empty(self.length, dtype=object)
        times         = np.empty(self.length)
        positions     = np.empty((self.length, 3))
        velocities    = np.empty((self.length, 3))
        accelerations = np.empty((self.length, 3))
        jerks         = np.empty((self.length, 3))
        snaps         = np.empty((self.length, 3))
        crackles      = np.empty((self.length, 3))
        headings      = np.empty((self.length, 2))

        for i, wp in enumerate(self.waypoints):
            ids[i]        = wp.id
            times[i]         = wp.time
            positions[i]     = wp.pos
            velocities[i]    = wp.vel
            accelerations[i] = wp.acel
            jerks[i]         = wp.jerk
            snaps[i]         = wp.snap
            crackles[i]      = wp.crakle
            headings[i]      = wp.heading

        return [ids, times, positions, velocities, accelerations, jerks, snaps, crackles, headings]
    
    def rotate_vector_by_quaternion(self, vector: np.ndarray, quaternion: np.ndarray, conjugate: bool) -> np.ndarray:
        """
        Rotates a vector according to a given quaternion.

        Args:
            - vector (np.ndarray) : The vector to rotate.
            - quaternion (np.ndarray) : The quaternion representing the rotation.
            - conjugate (bool) : Whether to use the conjugate of the quaternion.

        Returns:
            - np.ndarray: The rotated vector.
        """

        # 1. Extract the scalar and vector parts of the quaternion (assuming form q = [w, x, y, z])
        w = quaternion[0]

        # 1.1 If 'conjugate' is True, use the conjugate of the quaternion for the extraction
        if conjugate:
            u = -quaternion[1:4]
        else:
            u = quaternion[1:4]

        # 2. Use Rodrigues' rotation formula to rotate the vector by the quaternion
        t = 2.0 * np.cross(u, vector)
        vector_rotated = vector + w * t + np.cross(u, t)

        return np.round(vector_rotated, decimals=self.velocity_decimals)

    def local_to_global_velocity(self, vector, quaternion):
        return self.rotate_vector_by_quaternion(vector, quaternion, conjugate=False)

    def global_to_local_velocity(self, vector, quaternion):
        return self.rotate_vector_by_quaternion(vector, quaternion, conjugate=True)

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
    def make_plan_feasible(
        self, 
        is_time_key_aspect: bool, 
        reconnect_waypoints: bool = False
    ) -> None:
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
        ids = waypoint_arrays[0]
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
            new_velocities = np.round(new_velocities, self.velocity_decimals)

            # 6. Update the velocities of the waypoints
            for i, wp in enumerate(self.waypoints):
                wp.vel = new_velocities[i]

        else:
            # If velocity is the key aspect, we adjust the times to ensure that the UAV can reach the next waypoint
            new_delta_times = distances / vel_magnitudes[:-1]
            new_times = np.cumsum(np.concatenate(([times[0]], new_delta_times)))
            new_times = np.round(new_times, self.time_decimals)

            # 6. Update the times of the waypoints
            self.time_waypoints = SortedList(new_times)

            self.waypoints = [
                Waypoint(
                    id=ids[i],
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
            - waypoint (Union[str, int, float]) : The id, index, or time of the waypoint to smooth.
            - ang_vel (float) : The angular velocity to be used for smoothing the curve.
        """

        # 1. Determine the index of the waypoint to smooth based on the input type (id or index)
        if isinstance(waypoint, str):
            wp_idx: int = self.ids_to_idx.get(waypoint, None)
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
            raise RuntimeError(f"Trying to smooth a fly over WP (waypoint: {wp_2.id})")

        # 5. Calculate the angle between the two segments formed by the waypoints
        angle = wp_1.angle_with(wp_2)

        # 5.1 Check if the angle is zero, which indicates a straight line and cannot be smoothed
        if angle == 0:
            raise RuntimeError(f"Trying to smooth a straight line (waypoint: {wp_2.id})")

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
            raise Exception(f"There is not time enough in the past to smooth the curve (waypoint: {wp_2.id})")
        
        # 8.2 Check if there is enough time in the future to create the second new waypoint (wp_2B) 
        # after the original waypoint (wp_2)
        if wp_3.time <= wp_2B_init_time:
            raise Exception(f"There is not time enough in the future to smooth the curve (waypoint: {wp_2.id})")

        # 9. Create two new waypoints (wp_2A and wp_2B) to replace the original waypoint (wp_2)
        # 9.1 Create the first new waypoint (wp_2A) before the original waypoint (wp_2)
        wp_2A = Waypoint(
            id = f"{wp_2.id}_A",
            time  = wp_2.time - time_step,
            pos   = wp_2.pos - wp_1.vel * time_step,
            vel   = wp_1.vel
        )

        # 9.2 Create the second new waypoint (wp_2B) after the original waypoint (wp_2)
        wp_2B = Waypoint(
            id = f"{wp_2.id}_B",
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
            raise RuntimeError(f"Cannot find arc duration: v1+v2 ≈ 0 at waypoint {wp_2.id}")

        T = 30.0 * time_step * norm_w / (7.0 * norm_w + 16.0 * avg_vel)
        wp_2B.time = wp_2A.time + T

        # 11. Connect new waypoints
        wp_1.connect_to(wp_2A)
        wp_2A.connect_to(wp_2B)
        wp_2B.connect_to(wp_3)

        # 12. Update the flight plan by removing the original waypoint (wp_2) and
        # adding the new waypoints (wp_2A and wp_2B)
        wp_2_idx = self.ids_to_idx[wp_2.id]

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
            - waypoint (Union[str, int]) : The id or index of the waypoint to smooth.
            - ang_vel (float) : The angular velocity to be used for smoothing the curve.
            - lin_acel (float) : The linear acceleration to be used for smoothing the curve
        """
        
        # 1. Check if ang_vel and lin_acel are valid (greater than zero)
        if ang_vel <= 0:
            raise ValueError("Angular velocity must be a positive value.")

        if lin_acel <= 0:
            raise ValueError("Linear acceleration must be a positive value.")

        # 2. Determine the index of the waypoint to smooth based on the input type (id or index)
        if isinstance(waypoint, str):
            wp_idx: int = self.ids_to_idx.get(waypoint, None)
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
            raise RuntimeError(f"Trying to smooth a fly over WP (waypoint: {wp_2.id})")
        
        # 6. Calculate the angle between the two segments formed by the waypoints
        angle = wp_1.angle_with(wp_2)

        # 6.1 Check if the angle is zero, which indicates a straight line and cannot be smoothed
        if angle == 0:
            raise RuntimeError(f"Trying to smooth a straight line (waypoint: {wp_2.id})")

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
            raise Exception(f"There is not time enough in the past to smooth the curve (waypoint: {wp_2.id})")
        
        # 10.2 Check if there is enough time in the future to create the second new waypoint (wp_2B)
        # after the original waypoint (wp_2)
        if time_step >= wp_3.time - wp_2.time:
            raise Exception(f"There is not time enough in the future to smooth the curve (waypoint: {wp_2.id})")
        
        # 11. Create two new waypoints (wp_2A and wp_2B) to replace the original waypoint (wp_2)
        # 11.1 Create the first new waypoint (wp_2A) before the original waypoint (wp_2)
        wp_2A = Waypoint(
            id = f"{wp_2.id}_A",
            time  = wp_2.time - time_step,
            pos   = wp_2.pos - wp_1.vel * time_step,
            vel   = wp_1.vel
        )

        # 11.2 Create the second new waypoint (wp_2B) after the original waypoint (wp_2)
        wp_2B = Waypoint(
            id = f"{wp_2.id}_B",
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
        wp_2_idx = self.ids_to_idx[wp_2.id]

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
            return self.waypoints[-1].interpolate(time)

        # 2. Get the running waypoint for the given time and return its interpolated status
        running_wp = self.get_running_waypoint(time)

        return running_wp.interpolate(time)
    
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
    def get_command(self, current_time, uav_pos, uav_lin_vel, uav_ori_quaternion, target_heading, t_to_solve=2) -> Command:
        """
        Compute the navigation command for the UAV based on its current state and the flight plan.

        Args:
            - current_time (float) : The current time.
            - uav_pos (np.ndarray) : The current position of the UAV.
            - uav_lin_vel (np.ndarray) : The current linear velocity of the UAV in the world frame.
            - uav_ori_quaternion (np.ndarray) : The current orientation quaternion of the UAV.
            - target_heading (np.ndarray) : The desired heading direction for the UAV.
            - t_to_solve (float) : The time duration over which to compute the command.

        Returns:
            - Command : The computed command for the UAV.
        """

        # 1. Get the uav's current yaw from its orientation quaternion
        w, x, y, z = uav_ori_quaternion
        uav_yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))

        # 2. Get the expected status of the UAV at the current time
        expected_status = self.status_at_time(current_time)

        # 3. Compute the correction velocity needed to reach the expected position in the given 't_to_solve' time
        correction_vel = (expected_status.pos - uav_pos) / t_to_solve

        # 4. Compute the commanded velocity by adding the expected velocity and the correction velocity
        commanded_vel = expected_status.vel + correction_vel

        # 5. Get the UAV's commanded velocity in the UAV's body frame
        commanded_vel = self.global_to_local_velocity(commanded_vel, uav_ori_quaternion)
        
        # 6. Compute the target yaw based on the expected velocity or the target heading
        # 6.1 If the target heading is zero, use the expected velocity to determine the target direction
        target_direction = target_heading

        if np.linalg.norm(target_heading) < 1e-9:
            target_direction = expected_status.vel[:2]

        target_yaw = math.atan2(target_direction[1], target_direction[0])

        # 6.2 If target_yaw is 0, use the UAV's current yaw as the target yaw
        if target_yaw == 0:
            target_yaw = uav_yaw

        # 7. Compute the error in yaw and normalize it to the range [-pi, pi]
        error_yaw = target_yaw - uav_yaw
        error_yaw = (error_yaw + math.pi) % (2 * math.pi) - math.pi

        # 8. Compute the commanded angular velocity based on the error in yaw and the given 'time to solve'
        commanded_ang_vel = round(error_yaw / t_to_solve, self.velocity_decimals)

        # 9. Return the computed command as a Command object
        return Command(
            on=True,
            vel_x=commanded_vel[0],
            vel_y=commanded_vel[1],
            vel_z=commanded_vel[2],
            rot_z=commanded_ang_vel,
            duration=t_to_solve
        )

    # ----------------------------------
    # -------- COMPARISON ANALYSIS -----
    # ----------------------------------    
    def compare_to(self, fp2: FlightPlan, time_step: float) -> dict:
        """
        Compare two FlightPlans and return a comprehensive analysis of their interaction.

        Args:
            - fp2 (FlightPlan) : The second flight plan to compare against.
            - time_step (float) : The time step used to sample both traces.

        Returns:
            dict with the following keys:

            - "overlap" (dict | None) : Temporal overlap info, or None if there is no overlap.
                - "start" (float)    : Start time of the overlap.
                - "end"   (float)    : End time of the overlap.
                - "duration" (float) : Duration of the overlap.

            - "times" (np.ndarray) : Time instants for the overlap period, shape (N,).

            - "separation" (dict) : Distance statistics between the two plans during the overlap.
                - "distances" (np.ndarray) : Separation distance at each instant, shape (N,).
                - "min"      (float)       : Minimum separation distance.
                - "max"      (float)       : Maximum separation distance.
                - "mean"     (float)       : Mean separation distance.
                - "std"      (float)       : Standard deviation of separation distance.
                - "min_time" (float)       : Time of minimum separation (closest approach).
                - "max_time" (float)       : Time of maximum separation.

            - "relative_velocity" (dict) : Relative motion analysis.
                - "closing_speed"      (np.ndarray) : Closing speed at each instant (positive = approaching), shape (N,).
                - "angle_between"      (np.ndarray) : Angle between velocity vectors [rad], shape (N,).
                - "mean_closing_speed" (float)      : Mean closing speed.
                - "max_closing_speed"  (float)      : Maximum closing speed (peak approach rate).
                - "mean_angle_between" (float)      : Mean angle between velocity vectors [rad].

            - "conflicts" (dict) : Conflict detection based on combined safety radii.
                - "detected"       (bool)        : Whether any conflict was detected.
                - "combined_radius"(float)        : Sum of both plans' radii.
                - "count"          (int)          : Number of time samples in conflict.
                - "total_duration" (float)        : Total conflict duration [s].
                - "times"          (np.ndarray)   : Time instants in conflict, shape (K,).
                - "positions_fp1"  (np.ndarray)   : Positions of fp1 during conflict, shape (K, 3).
                - "positions_fp2"  (np.ndarray)   : Positions of fp2 during conflict, shape (K, 3).
                - "distances"      (np.ndarray)   : Separation distances during conflict, shape (K,).

            - "closest_approach" (dict) : Details of the closest approach instant.
                - "time"     (float)      : Time of closest approach.
                - "distance" (float)      : Separation distance at closest approach.
                - "pos_fp1"  (np.ndarray) : Position of fp1 at closest approach, shape (3,).
                - "pos_fp2"  (np.ndarray) : Position of fp2 at closest approach, shape (3,).
        """

        trace_1 = self.trace(time_step)
        trace_2 = fp2.trace(time_step)

        times_1 = np.round(trace_1[:, 0], decimals=self.time_decimals)
        times_2 = np.round(trace_2[:, 0], decimals=self.time_decimals)

        # 1. Compute temporal overlap
        overlap_start = max(times_1[0], times_2[0])
        overlap_end   = min(times_1[-1], times_2[-1])

        if overlap_start > overlap_end:
            return {"overlap": None}

        # 2. Find matching index ranges for the overlap period
        i1 = int(np.searchsorted(times_1, overlap_start, side='left'))
        e1 = int(np.searchsorted(times_1, overlap_end,   side='right'))
        i2 = int(np.searchsorted(times_2, overlap_start, side='left'))
        e2 = int(np.searchsorted(times_2, overlap_end,   side='right'))

        # 2.1 Trim to the same length to guard against floating-point edge cases
        n    = min(e1 - i1, e2 - i2)
        seg1 = trace_1[i1:i1 + n]
        seg2 = trace_2[i2:i2 + n]
        overlap_times = times_1[i1:i1 + n]

        # 3. Extract position and velocity arrays
        pos1 = seg1[:, 1:4]
        pos2 = seg2[:, 1:4]
        vel1 = seg1[:, 4:7]
        vel2 = seg2[:, 4:7]

        # 4. Separation distances
        diff      = pos1 - pos2
        distances = np.linalg.norm(diff, axis=1)
        min_idx   = int(np.argmin(distances))
        max_idx   = int(np.argmax(distances))

        # 5. Closing speed: projection of relative velocity onto the separation unit vector
        #    Positive value  → plans are approaching each other
        #    Negative value  → plans are diverging
        sep_norm       = np.linalg.norm(diff, axis=1, keepdims=True)
        sep_norm_safe  = np.where(sep_norm < 1e-12, 1e-12, sep_norm)
        sep_unit       = diff / sep_norm_safe
        rel_vel        = vel2 - vel1
        closing_speed  = -np.sum(rel_vel * sep_unit, axis=1)

        # 6. Angle between velocity vectors at each instant
        vel1_norm     = np.linalg.norm(vel1, axis=1, keepdims=True)
        vel2_norm     = np.linalg.norm(vel2, axis=1, keepdims=True)
        vel1_norm_safe = np.where(vel1_norm < 1e-12, 1e-12, vel1_norm)
        vel2_norm_safe = np.where(vel2_norm < 1e-12, 1e-12, vel2_norm)
        cos_angle     = np.sum((vel1 / vel1_norm_safe) * (vel2 / vel2_norm_safe), axis=1)
        cos_angle     = np.clip(cos_angle, -1.0, 1.0)
        angle_between = np.arccos(cos_angle)

        # 7. Conflict detection based on combined safety radii
        combined_radius  = self.radius + fp2.radius
        conflict_mask    = distances < combined_radius
        conflict_times   = overlap_times[conflict_mask]
        conflict_pos1    = pos1[conflict_mask]
        conflict_pos2    = pos2[conflict_mask]
        conflict_dist    = distances[conflict_mask]

        return {
            "base_info": {
                "fp1_id": self.id,
                "fp2_id": fp2.id,
                "fp1_priority": self.priority,
                "fp2_priority": fp2.priority,
                "fp1_radius": float(self.radius),
                "fp2_radius": float(fp2.radius),
            },
            "overlap": {
                "start":    float(overlap_start),
                "end":      float(overlap_end),
                "duration": float(overlap_end - overlap_start),
            },
            "times": overlap_times,
            "separation": {
                "distances": distances,
                "min":       float(distances[min_idx]),
                "max":       float(distances[max_idx]),
                "mean":      float(np.mean(distances)),
                "std":       float(np.std(distances)),
                "min_time":  float(overlap_times[min_idx]),
                "max_time":  float(overlap_times[max_idx]),
            },
            "relative_velocity": {
                "closing_speed":       closing_speed,
                "angle_between":       angle_between,
                "mean_closing_speed":  float(np.mean(closing_speed)),
                "max_closing_speed":   float(np.max(closing_speed)),
                "mean_angle_between":  float(np.mean(angle_between)),
            },
            "conflicts": {
                "detected":        bool(np.any(conflict_mask)),
                "combined_radius": float(combined_radius),
                "count":           int(np.sum(conflict_mask)),
                "total_duration":  float(int(np.sum(conflict_mask)) * time_step),
                "times":           conflict_times,
                "positions_fp1":   conflict_pos1,
                "positions_fp2":   conflict_pos2,
                "distances":       conflict_dist,
            },
            "closest_approach": {
                "time":     float(overlap_times[min_idx]),
                "distance": float(distances[min_idx]),
                "pos_fp1":  pos1[min_idx],
                "pos_fp2":  pos2[min_idx],
            },
        }

    def print_comparison(self, comparison_data) -> None:
        """
        Print a human-readable comparison between this FlightPlan and fp2.

        Args:
            - comparison_data (dict) : The comparison data returned by the compare_to method.
        """

        SEP   = "=" * 60
        SEP_S = "-" * 60
        RAD   = math.degrees

        fp2_id = comparison_data["base_info"]["fp2_id"]
        fp2_radius = comparison_data["base_info"]["fp2_radius"]

        print(SEP)
        print(f"  FLIGHT PLAN COMPARISON")
        print(f"  '{self.id}'  vs  '{fp2_id}'")
        print(SEP)

        # ── Overlap ──────────────────────────────────────────────
        overlap = comparison_data.get("overlap")
        if overlap is None:
            print("  No temporal overlap between the two flight plans.")
            print(SEP)
            return

        print(f"\n{'TEMPORAL OVERLAP':^60}")
        print(SEP_S)
        overlap_table = [
            ["Start time",  f"{overlap['start']:.3f} s"],
            ["End time",    f"{overlap['end']:.3f} s"],
            ["Duration",    f"{overlap['duration']:.3f} s"],
        ]
        print(tabulate(overlap_table, tablefmt="simple"))

        # ── Separation ───────────────────────────────────────────
        sep = comparison_data["separation"]
        print(f"\n{'SEPARATION DISTANCE':^60}")
        print(SEP_S)
        sep_table = [
            ["Minimum",  f"{sep['min']:.3f} m",  f"at t = {sep['min_time']:.3f} s"],
            ["Maximum",  f"{sep['max']:.3f} m",  f"at t = {sep['max_time']:.3f} s"],
            ["Mean",     f"{sep['mean']:.3f} m", ""],
            ["Std dev",  f"{sep['std']:.3f} m",  ""],
        ]
        print(tabulate(sep_table, headers=["Stat", "Value", ""], tablefmt="simple"))

        # ── Relative velocity ─────────────────────────────────────
        rv = comparison_data["relative_velocity"]
        print(f"\n{'RELATIVE VELOCITY':^60}")
        print(SEP_S)
        rv_table = [
            ["Mean closing speed",  f"{rv['mean_closing_speed']:+.3f} m/s",
             "(+ approaching / - diverging)"],
            ["Max closing speed",   f"{rv['max_closing_speed']:+.3f} m/s", ""],
            ["Mean angle between",  f"{RAD(rv['mean_angle_between']):.1f} deg", ""],
        ]
        print(tabulate(rv_table, headers=["Metric", "Value", "Note"], tablefmt="simple"))

        # ── Closest approach ─────────────────────────────────────
        ca = comparison_data["closest_approach"]
        print(f"\n{'CLOSEST APPROACH':^60}")
        print(SEP_S)
        ca_table = [
            ["Time",        f"{ca['time']:.3f} s"],
            ["Distance",    f"{ca['distance']:.3f} m"],
            [f"Pos '{self.id}'", f"[{ca['pos_fp1'][0]:.2f}, {ca['pos_fp1'][1]:.2f}, {ca['pos_fp1'][2]:.2f}] m"],
            [f"Pos '{fp2_id}'",  f"[{ca['pos_fp2'][0]:.2f}, {ca['pos_fp2'][1]:.2f}, {ca['pos_fp2'][2]:.2f}] m"],
        ]
        print(tabulate(ca_table, tablefmt="simple"))

        # ── Conflict detection ────────────────────────────────────
        cf = comparison_data["conflicts"]
        print(f"\n{'CONFLICT DETECTION':^60}")
        print(SEP_S)
        print(f"  Combined safety radius : {cf['combined_radius']:.3f} m  "
              f"({self.radius:.3f} + {fp2_radius:.3f})")
        if cf["detected"]:
            print(f"  {'[!] CONFLICT DETECTED':}")
            cf_table = [
                ["Samples in conflict",  cf["count"]],
                ["Total duration",       f"{cf['total_duration']:.3f} s"],
                ["Min dist in conflict", f"{cf['distances'].min():.3f} m"],
                ["Max dist in conflict", f"{cf['distances'].max():.3f} m"],
                ["First conflict time",  f"{cf['times'][0]:.3f} s"],
                ["Last conflict time",   f"{cf['times'][-1]:.3f} s"],
            ]
            print(tabulate(cf_table, headers=["Metric", "Value"], tablefmt="simple"))
        else:
            print("  No conflicts detected.")

        print("\n" + SEP)

    # ----------------------------------
    # -------- INFO & FIGURES ----------
    # ----------------------------------
    def attach_cursor_annotations(self, cursor, waypoints) -> None:
        """
        Attach annotations to a cursor for displaying waypoint information.

        Args:
            - cursor : The mplcursors cursor object to attach annotations to.
            - waypoints : The list of waypoints corresponding to the cursor's data points.
        """

        @cursor.connect("add")
        def on_add(sel):
            wp = waypoints[sel.index]
            text = f"TIME: {wp.time}\n"
            text += f"POS: {wp.pos}\n"
            text += f"VEL: {wp.vel}\n"
            text += f"ACEL: {wp.acel}"

            sel.annotation.set_text(text)

    def plot_position(self, fig_name, time_step) -> None:
        """
        Launch a matplotlib window to visualize the flight plan's position data over time.
        It plots the 3D trajectory, position (X, Y, Z) versus time, and position error versus time.

        Args:
            - fig_name (str) : The name of the matplotlib figure window.
            - time_step (float) : The time step used to sample the flight plan's trace
        """

        # 1. Create a new figure for plotting
        figure = plt.figure(fig_name)
        figure.canvas.manager.toolmanager.add_tool("Toogle_UAV", ToggleUAVtracking, gid="UAV_tracking")
        figure.canvas.manager.toolbar.add_tool('Toogle_UAV', 'navigation', 1)

        # 2. Define the color for the plots
        color = [0, 0.7, 1]

        # 3. Compute the trace of the flight plan at the specified time step
        trace = self.trace(time_step)
        trace_times = trace[:, 0]
        trace_x = trace[:, 1]
        trace_y = trace[:, 2]
        trace_z = trace[:, 3]

        # 4. Add a new subplot for the position error versus time plot
        pos_error_vs_time_plot = figure.add_subplot(6, 5, (26, 28))

        # 4.1 Set the axes' labels
        pos_error_vs_time_plot.set_xlabel("t [s]")
        pos_error_vs_time_plot.set_ylabel("Error [m]")

        # 4.2 Set the title
        pos_error_vs_time_plot.set_title("Position error versus time")

        # 4.3 Set the grid to True
        pos_error_vs_time_plot.grid(True)

        # 5. Add a new subplot for the 3D position plot
        position_3D_plot = figure.add_subplot(6, 5, (1, 23), projection="3d")
        
        # 5.1 Set the axes' labels
        position_3D_plot.set_xlabel("x [m]")
        position_3D_plot.set_ylabel("y [m]")
        position_3D_plot.set_zlabel("z [m]")
        
        # 5.2 Set the title
        position_3D_plot.set_title("Position 3D")
        
        # 5.3 Set the grid to True
        position_3D_plot.grid(True)
        
        # 5.4 Plot the 3D trajectory of the flight plan
        position_3D_plot.plot(trace_x, trace_y, trace_z, linewidth=2, color=color, zorder=1)

        # 6. Add new subplots for the position versus time plots for each axis (X, Y, Z)
        x_pos_vs_time_plot = figure.add_subplot(6, 5, (4, 10))
        y_pos_vs_time_plot = figure.add_subplot(6, 5, (14, 20))
        z_pos_vs_time_plot = figure.add_subplot(6, 5, (24, 30))
        
        # 6.1 Set the axes' labels for subplot
        x_pos_vs_time_plot.set_ylabel("x [m]")
        y_pos_vs_time_plot.set_ylabel("y [m]")
        z_pos_vs_time_plot.set_ylabel("z [m]")
        z_pos_vs_time_plot.set_xlabel("t [s]")
        
        # 6.2 Set the title for the X position versus time plot
        x_pos_vs_time_plot.set_title("Position versus time")
        
        # 6.3 Set the grid to True for each subplot
        x_pos_vs_time_plot.grid(True)
        y_pos_vs_time_plot.grid(True)
        z_pos_vs_time_plot.grid(True)
        
        # 6.4 Plot the position data for each axis versus time
        x_pos_vs_time_plot.plot(trace_times, trace_x, linewidth=2, color=color, zorder=1)
        y_pos_vs_time_plot.plot(trace_times, trace_y, linewidth=2, color=color, zorder=1)
        z_pos_vs_time_plot.plot(trace_times, trace_z, linewidth=2, color=color, zorder=1)

        # 7. Extract the positions and times of the waypoints for highlighting
        waypoint_idxs = np.where(np.isin(trace_times, self.time_waypoints))[0]
        x_waypoint_pos = trace_x[waypoint_idxs]
        y_waypoint_pos = trace_y[waypoint_idxs]
        z_waypoint_pos = trace_z[waypoint_idxs]

        # 7.1 Highlight the waypoints on the plots using scatter plots
        position_3D_plot_scatter = position_3D_plot.scatter(
            x_waypoint_pos, 
            y_waypoint_pos, 
            z_waypoint_pos, 
            marker="o", 
            color="blue", 
            s=25, 
            pickradius=30, 
            zorder=3
        )
        x_pos_vs_time_plot_scatter = x_pos_vs_time_plot.scatter(
            self.time_waypoints, 
            x_waypoint_pos, 
            marker="o", 
            color="blue", 
            s=25, 
            pickradius=30, 
            zorder=3
        )
        y_pos_vs_time_plot_scatter = y_pos_vs_time_plot.scatter(
            self.time_waypoints, 
            y_waypoint_pos, 
            marker="o", 
            color="blue", 
            s=25, 
            pickradius=30, 
            zorder=3
        )
        z_pos_vs_time_plot_scatter = z_pos_vs_time_plot.scatter(
            self.time_waypoints, 
            z_waypoint_pos, 
            marker="o", 
            color="blue", 
            s=25, 
            pickradius=30, 
            zorder=3
        )

        # 8. Attach cursor annotations to the scatter plots for displaying waypoint information
        position_3D_plot_cursor = mplcursors.cursor(position_3D_plot_scatter, highlight=True)
        x_pos_vs_time_plot_cursor = mplcursors.cursor(x_pos_vs_time_plot_scatter, highlight=True)
        y_pos_vs_time_plot_cursor = mplcursors.cursor(y_pos_vs_time_plot_scatter, highlight=True)
        z_pos_vs_time_plot_cursor = mplcursors.cursor(z_pos_vs_time_plot_scatter, highlight=True)

        self.attach_cursor_annotations(position_3D_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(x_pos_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(y_pos_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(z_pos_vs_time_plot_cursor, self.waypoints)

        # 9. Update limits to maintain scale in all axes
        # 9.1 3D position plot limits
        x_limit = max(np.abs(position_3D_plot.get_xlim3d()))
        y_limit = max(np.abs(position_3D_plot.get_ylim3d()))
        z_limit = max(np.abs(position_3D_plot.get_zlim3d()))
        max_limit = max(x_limit, y_limit, z_limit)

        position_3D_plot.set_xlim3d(-max_limit, max_limit)
        position_3D_plot.set_ylim3d(-max_limit, max_limit)
        position_3D_plot.set_zlim3d(-max_limit, max_limit)

        # 9.2 Position versus time plot limits
        x_limit = x_pos_vs_time_plot.get_ylim()
        y_limit = y_pos_vs_time_plot.get_ylim()
        xRange = x_limit[1] - x_limit[0]
        yRange = y_limit[1] - y_limit[0]
        
        maxRange = max(xRange, yRange)
        addition = maxRange / 2

        xMidValue = (x_limit[1] + x_limit[0]) / 2
        yMidValue = (y_limit[1] + y_limit[0]) / 2

        x_pos_vs_time_plot.set_ylim(xMidValue - addition, xMidValue + addition)
        y_pos_vs_time_plot.set_ylim(yMidValue - addition, yMidValue + addition)

        # 10. Show the figure without blocking the execution of the program
        plt.show(block=False)

    def plot_velocity(self, fig_name, time_step) -> None:
        """
        Launch a matplotlib window to visualize the flight plan's velocity data over time.
        It plots the 3D velocity, and the individual velocity components (X, Y, Z) versus time.

        Args:
            - fig_name (str) : The name of the matplotlib figure window.
            - time_step (float) : The time step used to sample the flight plan's trace
        """

        # 1. Create a new figure for plotting
        figure = plt.figure(fig_name)
        figure.canvas.manager.toolmanager.add_tool("Toogle_UAV", ToggleUAVtracking, gid="UAV_tracking")
        figure.canvas.manager.toolbar.add_tool('Toogle_UAV', 'navigation', 1)

        # 2. Define the color for the plots
        color = [0, 0.7, 1]

        # 3. Compute the trace of the flight plan at the specified time step
        trace = self.trace(time_step)
        trace_times = trace[:, 0]
        trace_x = trace[:, 4]
        trace_y = trace[:, 5]
        trace_z = trace[:, 6]

        # 4. Add a new subplot for the 3D velocity plot
        velocity_3D_plot = figure.add_subplot(4, 2, (1, 2))
        
        # 4.1 Set the axes' labels
        velocity_3D_plot.set_ylabel("3D [m/s]")
        
        # 4.2 Set the title
        velocity_3D_plot.set_title("Velocity versus time")
        
        # 4.3 Set the grid to True
        velocity_3D_plot.grid(True)
        
        # 4.4 Set plot info
        velocity_3D_plot.plot(
            trace_times, 
            np.sqrt(trace_x**2 + trace_y**2 + trace_z**2), 
            linewidth=2, 
            color=color, 
            zorder=1
        )

        # 5. Add new subplots for the velocity versus time plots for each axis (X, Y, Z)
        x_vel_vs_time_plot = figure.add_subplot(4, 2, (3, 4))
        y_vel_vs_time_plot = figure.add_subplot(4, 2, (5, 6))
        z_vel_vs_time_plot = figure.add_subplot(4, 2, (7, 8))
        
        # 5.1 Set the axes' labels for subplot
        x_vel_vs_time_plot.set_ylabel("vel x [m/s]")
        y_vel_vs_time_plot.set_ylabel("vel y [m/s]")
        z_vel_vs_time_plot.set_ylabel("vel z [m/s]")
        z_vel_vs_time_plot.set_xlabel("time [s]")
        
        # 5.2 Set the grid to True
        x_vel_vs_time_plot.grid(True)
        y_vel_vs_time_plot.grid(True)
        z_vel_vs_time_plot.grid(True)
        
        # 5.3 Set plots info
        x_vel_vs_time_plot.plot(trace_times, trace_x, linewidth=2, color=color, zorder=1)
        y_vel_vs_time_plot.plot(trace_times, trace_y, linewidth=2, color=color, zorder=1)
        z_vel_vs_time_plot.plot(trace_times, trace_z, linewidth=2, color=color, zorder=1)

        # 6. Extract the positions and times of the waypoints for highlighting
        waypoint_idxs = np.where(np.isin(trace_times, self.time_waypoints))[0]
        x_waypoint_vel = trace_x[waypoint_idxs]
        y_waypoint_vel = trace_y[waypoint_idxs]
        z_waypoint_vel = trace_z[waypoint_idxs]

        # 6.1 Highlight the waypoints on the plots using scatter plots
        x_vel_vs_time_plot_scatter = x_vel_vs_time_plot.scatter(
            self.time_waypoints, 
            x_waypoint_vel, 
            marker="o", 
            color="blue", 
            s=25, 
            pickradius=30, 
            zorder=3
        )
        y_vel_vs_time_plot_scatter = y_vel_vs_time_plot.scatter(
            self.time_waypoints, 
            y_waypoint_vel, 
            marker="o", 
            color="blue", 
            s=25, 
            pickradius=30, 
            zorder=3
        )
        z_vel_vs_time_plot_scatter = z_vel_vs_time_plot.scatter(
            self.time_waypoints, 
            z_waypoint_vel, 
            marker="o", 
            color="blue", 
            s=25, 
            pickradius=30, 
            zorder=3
        )

        # 7. Attach cursor annotations to the scatter plots for displaying waypoint information
        x_vel_vs_time_plot_cursor = mplcursors.cursor(x_vel_vs_time_plot_scatter, highlight=True)
        y_vel_vs_time_plot_cursor = mplcursors.cursor(y_vel_vs_time_plot_scatter, highlight=True)
        z_vel_vs_time_plot_cursor = mplcursors.cursor(z_vel_vs_time_plot_scatter, highlight=True)

        self.attach_cursor_annotations(x_vel_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(y_vel_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(z_vel_vs_time_plot_cursor, self.waypoints)

        # 8. Update limits to maintain scale in all axes
        # 8.1 3D velocity plot limits
        limit_3d = max(np.abs(velocity_3D_plot.get_ylim()))
        x_limit = max(np.abs(x_vel_vs_time_plot.get_ylim()))
        y_limit = max(np.abs(y_vel_vs_time_plot.get_ylim()))
        z_limit = max(np.abs(z_vel_vs_time_plot.get_ylim()))
        max_limit = max(limit_3d, x_limit, y_limit, z_limit)

        # 8.2 Position versus time plot limits
        velocity_3D_plot.set_ylim(-max_limit, max_limit)
        x_vel_vs_time_plot.set_ylim(-max_limit, max_limit)
        y_vel_vs_time_plot.set_ylim(-max_limit, max_limit)
        z_vel_vs_time_plot.set_ylim(-max_limit, max_limit)

        # 9. Show the figure without blocking the execution of the program
        plt.show(block=False)

    def plot_acceleration(self, fig_name, time_step) -> None:
        """
        Launch a matplotlib window to visualize the flight plan's acceleration data over time.
        It plots the 3D acceleration, and the individual acceleration components (X, Y, Z) versus time.

        Args:
            - fig_name (str) : The name of the matplotlib figure window.
            - time_step (float) : The time step used to sample the flight plan's trace
        """        

        # 1. Create a new figure for plotting
        figure = plt.figure(fig_name)

        # 2. Define the color for the plots
        color = [0, 0.7, 1]

        # 3. Compute the trace of the flight plan at the specified time step
        trace = self.trace(time_step)
        trace_times = trace[:, 0]
        trace_x = trace[:, 7]
        trace_y = trace[:, 8]
        trace_z = trace[:, 9]

        # 4. Add a new subplot for the 3D acceleration versus time plot
        acceleration_3D_plot = figure.add_subplot(4, 2, (1, 2))
        
        # 4.1 Set the axes' labels
        acceleration_3D_plot.set_ylabel("3D [m/s2]")
        
        # 4.2 Set the title
        acceleration_3D_plot.set_title("Acceleration versus time")
        
        # 4.3 Set the grid to True
        acceleration_3D_plot.grid(True)
        
        # 4.4 Set plot info
        acceleration_3D_plot.plot(
            trace_times, 
            np.sqrt(trace_x**2 + trace_y**2 + trace_z**2), 
            linewidth=2, 
            color=color, 
            zorder=1
        )

        # 5. Add new subplots for the acceleration versus time plots for each axis (X, Y, Z)
        x_acc_vs_time_plot = figure.add_subplot(4, 2, (3, 4))
        y_acc_vs_time_plot = figure.add_subplot(4, 2, (5, 6))
        z_acc_vs_time_plot = figure.add_subplot(4, 2, (7, 8))
        
        # 5.1 Set the axes' labels for subplot
        x_acc_vs_time_plot.set_ylabel("accel x [m/s2]")
        y_acc_vs_time_plot.set_ylabel("accel y [m/s2]")
        z_acc_vs_time_plot.set_ylabel("accel z [m/s2]")
        x_acc_vs_time_plot.set_xlabel("time [s]")
        
        # 5.2 Set the grid to True
        x_acc_vs_time_plot.grid(True)
        y_acc_vs_time_plot.grid(True)
        z_acc_vs_time_plot.grid(True)
        
        # 5.3 Set plots info
        x_acc_vs_time_plot.plot(trace_times, trace_x, linewidth=2, color=color, zorder=1)
        y_acc_vs_time_plot.plot(trace_times, trace_y, linewidth=2, color=color, zorder=1)
        z_acc_vs_time_plot.plot(trace_times, trace_z, linewidth=2, color=color, zorder=1)

        # 6. Extract the positions and times of the waypoints for highlighting
        waypoint_idxs = np.where(np.isin(trace_times, self.time_waypoints))[0]
        x_waypoint_acc = trace_x[waypoint_idxs]
        y_waypoint_acc = trace_y[waypoint_idxs]
        z_waypoint_acc = trace_z[waypoint_idxs]

        # 6.1 Highlight the waypoints on the plots using scatter plots
        x_acc_vs_time_plot_scatter = x_acc_vs_time_plot.scatter(
            self.time_waypoints, 
            x_waypoint_acc, 
            marker="o", 
            color="blue", 
            s=25, 
            pickradius=30, 
            zorder=2
        )
        y_acc_vs_time_plot_scatter = y_acc_vs_time_plot.scatter(
            self.time_waypoints, 
            y_waypoint_acc, 
            marker="o", 
            color="blue", 
            s=25, 
            pickradius=30, 
            zorder=2
        )
        z_acc_vs_time_plot_scatter = z_acc_vs_time_plot.scatter(
            self.time_waypoints, 
            z_waypoint_acc, 
            marker="o", 
            color="blue", 
            s=25, 
            pickradius=30, 
            zorder=2
        )

        # 7. Attach cursor annotations to the scatter plots for displaying waypoint information
        x_acc_vs_time_plot_cursor = mplcursors.cursor(x_acc_vs_time_plot_scatter, highlight=True)
        y_acc_vs_time_plot_cursor = mplcursors.cursor(y_acc_vs_time_plot_scatter, highlight=True)
        z_acc_vs_time_plot_cursor = mplcursors.cursor(z_acc_vs_time_plot_scatter, highlight=True)

        self.attach_cursor_annotations(x_acc_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(y_acc_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(z_acc_vs_time_plot_cursor, self.waypoints)

        # 8. Update limits to maintain scale in all axes
        limit_3D = max(np.abs(acceleration_3D_plot.get_ylim()))
        x_limit = max(np.abs(x_acc_vs_time_plot.get_ylim()))
        y_limit = max(np.abs(y_acc_vs_time_plot.get_ylim()))
        z_limit = max(np.abs(z_acc_vs_time_plot.get_ylim()))
        max_limit = max(limit_3D, x_limit, y_limit, z_limit)

        acceleration_3D_plot.set_ylim(-max_limit, max_limit)
        x_acc_vs_time_plot.set_ylim(-max_limit, max_limit)
        y_acc_vs_time_plot.set_ylim(-max_limit, max_limit)
        z_acc_vs_time_plot.set_ylim(-max_limit, max_limit)

        # 9. Show the figure without blocking the execution of the program
        plt.show(block=False)

    def plot_jerk(self, fig_name, time_step) -> None:
        """
        Launch a matplotlib window to visualize the flight plan's jerk data over time.
        It plots the 3D jerk magnitude, and the individual jerk components (X, Y, Z) versus time.

        Args:
            - fig_name (str) : The name of the matplotlib figure window.
            - time_step (float) : The time step used to sample the flight plan's trace
        """

        # 1. Create a new figure for plotting
        figure = plt.figure(fig_name)

        # 2. Define the color for the plots
        color = [0, 0.7, 1]

        # 3. Compute the trace of the flight plan at the specified time step
        trace = self.trace(time_step)
        trace_times = trace[:, 0]
        trace_x = trace[:, 10]
        trace_y = trace[:, 11]
        trace_z = trace[:, 12]

        # 4. Add a new subplot for the 3D jerk versus time plot
        jerk_3D_plot = figure.add_subplot(4, 2, (1, 2))

        # 4.1 Set the axes' labels
        jerk_3D_plot.set_ylabel("3D [m/s3]")

        # 4.2 Set the title
        jerk_3D_plot.set_title("Jerk versus time")

        # 4.3 Set the grid to True
        jerk_3D_plot.grid(True)

        # 4.4 Set plot info
        jerk_3D_plot.plot(
            trace_times,
            np.sqrt(trace_x**2 + trace_y**2 + trace_z**2),
            linewidth=2,
            color=color,
            zorder=1
        )

        # 5. Add new subplots for the jerk versus time plots for each axis (X, Y, Z)
        x_jerk_vs_time_plot = figure.add_subplot(4, 2, (3, 4))
        y_jerk_vs_time_plot = figure.add_subplot(4, 2, (5, 6))
        z_jerk_vs_time_plot = figure.add_subplot(4, 2, (7, 8))

        # 5.1 Set the axes' labels for subplot
        x_jerk_vs_time_plot.set_ylabel("jerk x [m/s3]")
        y_jerk_vs_time_plot.set_ylabel("jerk y [m/s3]")
        z_jerk_vs_time_plot.set_ylabel("jerk z [m/s3]")
        x_jerk_vs_time_plot.set_xlabel("time [s]")

        # 5.2 Set the grid to True
        x_jerk_vs_time_plot.grid(True)
        y_jerk_vs_time_plot.grid(True)
        z_jerk_vs_time_plot.grid(True)

        # 5.3 Set plots info
        x_jerk_vs_time_plot.plot(trace_times, trace_x, linewidth=2, color=color, zorder=1)
        y_jerk_vs_time_plot.plot(trace_times, trace_y, linewidth=2, color=color, zorder=1)
        z_jerk_vs_time_plot.plot(trace_times, trace_z, linewidth=2, color=color, zorder=1)

        # 6. Extract the jerk and times of the waypoints for highlighting
        waypoint_idxs = np.where(np.isin(trace_times, self.time_waypoints))[0]
        x_waypoint_jerk = trace_x[waypoint_idxs]
        y_waypoint_jerk = trace_y[waypoint_idxs]
        z_waypoint_jerk = trace_z[waypoint_idxs]

        # 6.1 Highlight the waypoints on the plots using scatter plots
        x_jerk_vs_time_plot_scatter = x_jerk_vs_time_plot.scatter(
            self.time_waypoints,
            x_waypoint_jerk,
            marker="o",
            color="blue",
            s=25,
            pickradius=30,
            zorder=2
        )
        y_jerk_vs_time_plot_scatter = y_jerk_vs_time_plot.scatter(
            self.time_waypoints,
            y_waypoint_jerk,
            marker="o",
            color="blue",
            s=25,
            pickradius=30,
            zorder=2
        )
        z_jerk_vs_time_plot_scatter = z_jerk_vs_time_plot.scatter(
            self.time_waypoints,
            z_waypoint_jerk,
            marker="o",
            color="blue",
            s=25,
            pickradius=30,
            zorder=2
        )

        # 7. Attach cursor annotations to the scatter plots for displaying waypoint information
        x_jerk_vs_time_plot_cursor = mplcursors.cursor(x_jerk_vs_time_plot_scatter, highlight=True)
        y_jerk_vs_time_plot_cursor = mplcursors.cursor(y_jerk_vs_time_plot_scatter, highlight=True)
        z_jerk_vs_time_plot_cursor = mplcursors.cursor(z_jerk_vs_time_plot_scatter, highlight=True)

        self.attach_cursor_annotations(x_jerk_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(y_jerk_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(z_jerk_vs_time_plot_cursor, self.waypoints)

        # 8. Update limits to maintain scale in all axes
        limit_3D = max(np.abs(jerk_3D_plot.get_ylim()))
        x_limit = max(np.abs(x_jerk_vs_time_plot.get_ylim()))
        y_limit = max(np.abs(y_jerk_vs_time_plot.get_ylim()))
        z_limit = max(np.abs(z_jerk_vs_time_plot.get_ylim()))
        max_limit = max(limit_3D, x_limit, y_limit, z_limit)

        jerk_3D_plot.set_ylim(-max_limit, max_limit)
        x_jerk_vs_time_plot.set_ylim(-max_limit, max_limit)
        y_jerk_vs_time_plot.set_ylim(-max_limit, max_limit)
        z_jerk_vs_time_plot.set_ylim(-max_limit, max_limit)

        # 9. Show the figure without blocking the execution of the program
        plt.show(block=False)

    def plot_snap(self, fig_name, time_step) -> None:
        """
        Launch a matplotlib window to visualize the flight plan's snap data over time.
        It plots the 3D snap magnitude, and the individual snap components (X, Y, Z) versus time.

        Args:
            - fig_name (str) : The name of the matplotlib figure window.
            - time_step (float) : The time step used to sample the flight plan's trace
        """

        # 1. Create a new figure for plotting
        figure = plt.figure(fig_name)

        # 2. Define the color for the plots
        color = [0, 0.7, 1]

        # 3. Compute the trace of the flight plan at the specified time step
        trace = self.trace(time_step)
        trace_times = trace[:, 0]
        trace_x = trace[:, 13]
        trace_y = trace[:, 14]
        trace_z = trace[:, 15]

        # 4. Add a new subplot for the 3D snap versus time plot
        snap_3D_plot = figure.add_subplot(4, 2, (1, 2))

        # 4.1 Set the axes' labels
        snap_3D_plot.set_ylabel("3D [m/s4]")

        # 4.2 Set the title
        snap_3D_plot.set_title("Snap versus time")

        # 4.3 Set the grid to True
        snap_3D_plot.grid(True)

        # 4.4 Set plot info
        snap_3D_plot.plot(
            trace_times,
            np.sqrt(trace_x**2 + trace_y**2 + trace_z**2),
            linewidth=2,
            color=color,
            zorder=1
        )

        # 5. Add new subplots for the snap versus time plots for each axis (X, Y, Z)
        x_snap_vs_time_plot = figure.add_subplot(4, 2, (3, 4))
        y_snap_vs_time_plot = figure.add_subplot(4, 2, (5, 6))
        z_snap_vs_time_plot = figure.add_subplot(4, 2, (7, 8))

        # 5.1 Set the axes' labels for subplot
        x_snap_vs_time_plot.set_ylabel("snap x [m/s4]")
        y_snap_vs_time_plot.set_ylabel("snap y [m/s4]")
        z_snap_vs_time_plot.set_ylabel("snap z [m/s4]")
        x_snap_vs_time_plot.set_xlabel("time [s]")

        # 5.2 Set the grid to True
        x_snap_vs_time_plot.grid(True)
        y_snap_vs_time_plot.grid(True)
        z_snap_vs_time_plot.grid(True)

        # 5.3 Set plots info
        x_snap_vs_time_plot.plot(trace_times, trace_x, linewidth=2, color=color, zorder=1)
        y_snap_vs_time_plot.plot(trace_times, trace_y, linewidth=2, color=color, zorder=1)
        z_snap_vs_time_plot.plot(trace_times, trace_z, linewidth=2, color=color, zorder=1)

        # 6. Extract the snap and times of the waypoints for highlighting
        waypoint_idxs = np.where(np.isin(trace_times, self.time_waypoints))[0]
        x_waypoint_snap = trace_x[waypoint_idxs]
        y_waypoint_snap = trace_y[waypoint_idxs]
        z_waypoint_snap = trace_z[waypoint_idxs]

        # 6.1 Highlight the waypoints on the plots using scatter plots
        x_snap_vs_time_plot_scatter = x_snap_vs_time_plot.scatter(
            self.time_waypoints,
            x_waypoint_snap,
            marker="o",
            color="blue",
            s=25,
            pickradius=30,
            zorder=2
        )
        y_snap_vs_time_plot_scatter = y_snap_vs_time_plot.scatter(
            self.time_waypoints,
            y_waypoint_snap,
            marker="o",
            color="blue",
            s=25,
            pickradius=30,
            zorder=2
        )
        z_snap_vs_time_plot_scatter = z_snap_vs_time_plot.scatter(
            self.time_waypoints,
            z_waypoint_snap,
            marker="o",
            color="blue",
            s=25,
            pickradius=30,
            zorder=2
        )

        # 7. Attach cursor annotations to the scatter plots for displaying waypoint information
        x_snap_vs_time_plot_cursor = mplcursors.cursor(x_snap_vs_time_plot_scatter, highlight=True)
        y_snap_vs_time_plot_cursor = mplcursors.cursor(y_snap_vs_time_plot_scatter, highlight=True)
        z_snap_vs_time_plot_cursor = mplcursors.cursor(z_snap_vs_time_plot_scatter, highlight=True)

        self.attach_cursor_annotations(x_snap_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(y_snap_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(z_snap_vs_time_plot_cursor, self.waypoints)

        # 8. Update limits to maintain scale in all axes
        limit_3D = max(np.abs(snap_3D_plot.get_ylim()))
        x_limit = max(np.abs(x_snap_vs_time_plot.get_ylim()))
        y_limit = max(np.abs(y_snap_vs_time_plot.get_ylim()))
        z_limit = max(np.abs(z_snap_vs_time_plot.get_ylim()))
        max_limit = max(limit_3D, x_limit, y_limit, z_limit)

        snap_3D_plot.set_ylim(-max_limit, max_limit)
        x_snap_vs_time_plot.set_ylim(-max_limit, max_limit)
        y_snap_vs_time_plot.set_ylim(-max_limit, max_limit)
        z_snap_vs_time_plot.set_ylim(-max_limit, max_limit)

        # 9. Show the figure without blocking the execution of the program
        plt.show(block=False)

    def plot_crackle(self, fig_name, time_step) -> None:
        """
        Launch a matplotlib window to visualize the flight plan's crackle data over time.
        It plots the 3D crackle magnitude, and the individual crackle components (X, Y, Z) versus time.

        Args:
            - fig_name (str) : The name of the matplotlib figure window.
            - time_step (float) : The time step used to sample the flight plan's trace
        """

        # 1. Create a new figure for plotting
        figure = plt.figure(fig_name)

        # 2. Define the color for the plots
        color = [0, 0.7, 1]

        # 3. Compute the trace of the flight plan at the specified time step
        trace = self.trace(time_step)
        trace_times = trace[:, 0]
        trace_x = trace[:, 16]
        trace_y = trace[:, 17]
        trace_z = trace[:, 18]

        # 4. Add a new subplot for the 3D crackle versus time plot
        crackle_3D_plot = figure.add_subplot(4, 2, (1, 2))

        # 4.1 Set the axes' labels
        crackle_3D_plot.set_ylabel("3D [m/s5]")

        # 4.2 Set the title
        crackle_3D_plot.set_title("Crackle versus time")

        # 4.3 Set the grid to True
        crackle_3D_plot.grid(True)

        # 4.4 Set plot info
        crackle_3D_plot.plot(
            trace_times,
            np.sqrt(trace_x**2 + trace_y**2 + trace_z**2),
            linewidth=2,
            color=color,
            zorder=1
        )

        # 5. Add new subplots for the crackle versus time plots for each axis (X, Y, Z)
        x_crackle_vs_time_plot = figure.add_subplot(4, 2, (3, 4))
        y_crackle_vs_time_plot = figure.add_subplot(4, 2, (5, 6))
        z_crackle_vs_time_plot = figure.add_subplot(4, 2, (7, 8))

        # 5.1 Set the axes' labels for subplot
        x_crackle_vs_time_plot.set_ylabel("crackle x [m/s5]")
        y_crackle_vs_time_plot.set_ylabel("crackle y [m/s5]")
        z_crackle_vs_time_plot.set_ylabel("crackle z [m/s5]")
        x_crackle_vs_time_plot.set_xlabel("time [s]")

        # 5.2 Set the grid to True
        x_crackle_vs_time_plot.grid(True)
        y_crackle_vs_time_plot.grid(True)
        z_crackle_vs_time_plot.grid(True)

        # 5.3 Set plots info
        x_crackle_vs_time_plot.plot(trace_times, trace_x, linewidth=2, color=color, zorder=1)
        y_crackle_vs_time_plot.plot(trace_times, trace_y, linewidth=2, color=color, zorder=1)
        z_crackle_vs_time_plot.plot(trace_times, trace_z, linewidth=2, color=color, zorder=1)

        # 6. Extract the crackle and times of the waypoints for highlighting
        waypoint_idxs = np.where(np.isin(trace_times, self.time_waypoints))[0]
        x_waypoint_crackle = trace_x[waypoint_idxs]
        y_waypoint_crackle = trace_y[waypoint_idxs]
        z_waypoint_crackle = trace_z[waypoint_idxs]

        # 6.1 Highlight the waypoints on the plots using scatter plots
        x_crackle_vs_time_plot_scatter = x_crackle_vs_time_plot.scatter(
            self.time_waypoints,
            x_waypoint_crackle,
            marker="o",
            color="blue",
            s=25,
            pickradius=30,
            zorder=2
        )
        y_crackle_vs_time_plot_scatter = y_crackle_vs_time_plot.scatter(
            self.time_waypoints,
            y_waypoint_crackle,
            marker="o",
            color="blue",
            s=25,
            pickradius=30,
            zorder=2
        )
        z_crackle_vs_time_plot_scatter = z_crackle_vs_time_plot.scatter(
            self.time_waypoints,
            z_waypoint_crackle,
            marker="o",
            color="blue",
            s=25,
            pickradius=30,
            zorder=2
        )

        # 7. Attach cursor annotations to the scatter plots for displaying waypoint information
        x_crackle_vs_time_plot_cursor = mplcursors.cursor(x_crackle_vs_time_plot_scatter, highlight=True)
        y_crackle_vs_time_plot_cursor = mplcursors.cursor(y_crackle_vs_time_plot_scatter, highlight=True)
        z_crackle_vs_time_plot_cursor = mplcursors.cursor(z_crackle_vs_time_plot_scatter, highlight=True)

        self.attach_cursor_annotations(x_crackle_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(y_crackle_vs_time_plot_cursor, self.waypoints)
        self.attach_cursor_annotations(z_crackle_vs_time_plot_cursor, self.waypoints)

        # 8. Update limits to maintain scale in all axes
        limit_3D = max(np.abs(crackle_3D_plot.get_ylim()))
        x_limit = max(np.abs(x_crackle_vs_time_plot.get_ylim()))
        y_limit = max(np.abs(y_crackle_vs_time_plot.get_ylim()))
        z_limit = max(np.abs(z_crackle_vs_time_plot.get_ylim()))
        max_limit = max(limit_3D, x_limit, y_limit, z_limit)

        crackle_3D_plot.set_ylim(-max_limit, max_limit)
        x_crackle_vs_time_plot.set_ylim(-max_limit, max_limit)
        y_crackle_vs_time_plot.set_ylim(-max_limit, max_limit)
        z_crackle_vs_time_plot.set_ylim(-max_limit, max_limit)

        # 9. Show the figure without blocking the execution of the program
        plt.show(block=False)

    def add_UAV_track_pos(self, fig_name, uav_info: np.ndarray) -> None:
        """
        Adds the UAV's tracked position to the existing position plots in the specified figure.

        Args:
            - fig_name (str) : The name of the matplotlib figure window.
            - uav_info (np.ndarray) : An array of UAV information containing time and position data.
        """

        # 1. Get the existing figure and its subplots
        figure = plt.figure(fig_name)
        subplots = figure.get_axes()

        # 2. Unpack the subplots for easier access
        pos_error_vs_time_plot = subplots[0]
        position_3D_plot = subplots[1]
        x_pos_vs_time_plot = subplots[2]
        y_pos_vs_time_plot = subplots[3]
        z_pos_vs_time_plot = subplots[4]

        # 3. Unpack the UAV information into separate arrays for time and position
        times_uav = uav_info[:, 0]
        x_uav_pos = uav_info[:, 1]
        y_uav_pos = uav_info[:, 2]
        z_uav_pos = uav_info[:, 3]
    
        # 4. Compute the error between the UAV's tracked position and the expected position from the flight plan
        expected_status = np.array([self.status_at_time(time).pos for time in times_uav])
        errors = np.linalg.norm(uav_info[:, 1:4] - expected_status, axis=1)

        # 5. Plot the error on the position error versus time subplot
        pos_error_vs_time_plot.plot(times_uav, errors, linestyle="solid", linewidth=1, color="red")

        # 6. Plot the UAV's tracked position on the 3D position plot and the individual position versus time plots
        position_3D_plot.plot(
            x_uav_pos, 
            y_uav_pos, 
            z_uav_pos, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )
        x_pos_vs_time_plot.plot(
            times_uav, 
            x_uav_pos, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )
        y_pos_vs_time_plot.plot(
            times_uav, 
            y_uav_pos, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )
        z_pos_vs_time_plot.plot(
            times_uav, 
            z_uav_pos, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )

        # 7. Highlight the UAV's tracked positions on the plots using scatter plots
        position_3D_plot.scatter(x_uav_pos, y_uav_pos, z_uav_pos, color="black", s=10, gid="UAV_tracking", zorder=2)
        x_pos_vs_time_plot.scatter(times_uav, x_uav_pos, color="black", s=10, gid="UAV_tracking", zorder=2)
        y_pos_vs_time_plot.scatter(times_uav, y_uav_pos, color="black", s=10, gid="UAV_tracking", zorder=2)
        z_pos_vs_time_plot.scatter(times_uav, z_uav_pos, color="black", s=10, gid="UAV_tracking", zorder=2)

    def add_UAV_track_vel(self, fig_name, uav_info : np.ndarray) -> None:
        """
        Adds the UAV's tracked velocity to the existing velocity plots in the specified figure.

        Args:
            - fig_name (str) : The name of the matplotlib figure window.
            - uav_info (np.ndarray) : An array of UAV information containing time and velocity data.
        """

        # 1. Get the existing figure and its subplots
        figure = plt.figure(fig_name)
        subplots = figure.get_axes()

        # 2. Unpack the subplots for easier access
        velocity_3D_plot = subplots[0]
        x_vel_vs_time_plot = subplots[1]
        y_vel_vs_time_plot = subplots[2]
        z_vel_vs_time_plot = subplots[3]

        # 3. Unpack the UAV information into separate arrays for time and velocity
        times_uav = uav_info[:, 0]
        x_uav_vel = uav_info[:, 1]
        y_uav_vel = uav_info[:, 2]
        z_uav_vel = uav_info[:, 3]

        # 4. Plot the UAV's tracked velocity on the 3D velocity plot and the individual velocity versus time plots
        info_3d = np.sqrt(x_uav_vel**2 + y_uav_vel**2 + z_uav_vel**2)

        velocity_3D_plot.plot(
            times_uav, 
            info_3d, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )
        x_vel_vs_time_plot.plot(
            times_uav, 
            x_uav_vel, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )
        y_vel_vs_time_plot.plot(
            times_uav, 
            y_uav_vel, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )
        z_vel_vs_time_plot.plot(
            times_uav, 
            z_uav_vel, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )

        # 5. Highlight the UAV's tracked positions on the plots using scatter plots
        velocity_3D_plot.scatter(times_uav, info_3d, color="black", s=10, gid="UAV_tracking", zorder=2)
        x_vel_vs_time_plot.scatter(times_uav, x_uav_vel, color="black", s=10, gid="UAV_tracking", zorder=2)
        y_vel_vs_time_plot.scatter(times_uav, y_uav_vel, color="black", s=10, gid="UAV_tracking", zorder=2)
        z_vel_vs_time_plot.scatter(times_uav, z_uav_vel, color="black", s=10, gid="UAV_tracking", zorder=2)

    def add_UAV_track_acc(self, fig_name, uav_info : np.ndarray) -> None:
        """
        Adds the UAV's tracked acceleration to the existing velocity plots in the specified figure.

        Args:
            - fig_name (str) : The name of the matplotlib figure window.
            - uav_info (np.ndarray) : An array of UAV information containing time and acceleration data.
        """

        # 1. Get the existing figure and its subplots
        figure = plt.figure(fig_name)
        subplots = figure.get_axes()

        # 2. Unpack the subplots for easier access
        acceleration_3D_plot = subplots[0]
        x_acc_vs_time_plot = subplots[1]
        y_acc_vs_time_plot = subplots[2]
        z_acc_vs_time_plot = subplots[3]

        # 3. Unpack the UAV information into separate arrays for time and acceleration
        times_uav = uav_info[:, 0]
        x_uav_acc = uav_info[:, 1]
        y_uav_acc = uav_info[:, 2]
        z_uav_acc = uav_info[:, 3]

        # 4. Compute the acceleration from the velocity data using finite differences
        x_uav_acc = np.diff(x_uav_acc, prepend=x_uav_acc[:1])
        y_uav_acc = np.diff(y_uav_acc, prepend=y_uav_acc[:1])
        z_uav_acc = np.diff(z_uav_acc, prepend=z_uav_acc[:1])
        
        # 5. Plot the UAV's tracked acceleration on the 3D acceleration plot and the individual acceleration versus time plots
        info_3d = np.sqrt(x_uav_acc**2 + y_uav_acc**2 + z_uav_acc**2)

        acceleration_3D_plot.plot(
            times_uav, 
            info_3d, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )
        x_acc_vs_time_plot.plot(
            times_uav, 
            x_uav_acc, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )
        y_acc_vs_time_plot.plot(
            times_uav, 
            y_uav_acc, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )
        z_acc_vs_time_plot.plot(
            times_uav, 
            z_uav_acc, 
            linestyle="dashed", 
            linewidth=1, 
            color="black", 
            gid="UAV_tracking", 
            zorder=2
        )

        # 6. Highlight the UAV's tracked positions on the plots using scatter plots
        acceleration_3D_plot.scatter(times_uav, info_3d, color="black", s=10, gid="UAV_tracking", zorder=2)
        x_acc_vs_time_plot.scatter(times_uav, x_uav_acc, color="black", s=10, gid="UAV_tracking", zorder=2)
        y_acc_vs_time_plot.scatter(times_uav, y_uav_acc, color="black", s=10, gid="UAV_tracking", zorder=2)
        z_acc_vs_time_plot.scatter(times_uav, z_uav_acc, color="black", s=10, gid="UAV_tracking", zorder=2)


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
