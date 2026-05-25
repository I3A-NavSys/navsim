import sys
from os import path

current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../.."))

if project_root_path not in sys.path:
    sys.path.append(project_root_path)

from tabulate import tabulate
import copy
import logging
from typing import List, Optional
import numpy as np
import matplotlib
import mplcursors
import matplotlib.pyplot as plt
from matplotlib.backend_tools import ToolToggleBase
from matplotlib.collections import PathCollection
from scipy.spatial.transform import Rotation

from detection.conflictDetection import SweptBox_AABB, SweptBox_OBB
from core.config import UAV_MAX_SPEED, UAV_MAX_ACCEL, UAV_RADIUS


from .waypoint import Waypoint
from .command import Command


# matplotlib.use("TkAgg")
plt.rcParams["toolbar"] = "toolmanager"

log = logging.getLogger(__name__)

class FlightPlan:

    def __init__(self):
        self.id: int = 0
        self.priority: int = 0
        self.radius: float = UAV_RADIUS
        self.max_var_lin_vel = 5        # maximum variation in linear  velocity   [  m/s]
        self.max_var_ang_vel = 1        # maximum variation in angular velocity   [rad/s]
        self.target_yaw = None
        self.waypoints: List[Waypoint] = []

    def set_waypoint(self, wp=None, label="", time=None, pos=None, vel=None, heading=[0,0]):
        numWPs = len(self.waypoints)

        if wp is None:
            if time is None:
                if numWPs == 0:
                    time = 0
                else:
                    time = self.finish_time() + 1

            if numWPs > 0:  status = self.status_at_time(time)

            if pos is None:
                if numWPs == 0:
                    pos = [0,0,0]
                else:
                    pos = status.pos

            if vel is None:
                if numWPs == 0:
                    vel = [0,0,0]
                else:
                    if time <= self.init_time():
                        vel = [0,0,0]
                    else:
                        vel = status.vel

            wp = Waypoint(label=label, t=time, pos=pos, vel=vel, heading=heading)
        
        index = self.get_target_index_from_time(wp.t)

        # Replace if the same time already exists
        if index < numWPs and self.waypoints[index].t == wp.t:
            self.waypoints[index] = wp
        else:
            self.waypoints.insert(index, wp)

    def remove_negative_time(self):
        if self.init_time() < 0:
            self.postpone(-self.init_time())

    def remove_waypoint_at_time(self, t: float) -> None:
        """Removes the waypoint at a specific time `t` from the flight plan."""
        self.waypoints = list(filter(lambda wp: wp.t != t, self.waypoints))

    def get_index_from_label(self, label: str) -> Optional[int]:
        for i, wp in enumerate(self.waypoints):
            if wp.label == label:
                return i
        return None
    
    def get_running_index_from_time(self, t: float):
        """
        It returns the WP the UAV is currently executing
        """
        
        index = self.get_target_index_from_time(t)
        running_i = 0 if index == 0 else index - 1
        return running_i
    
    def get_target_index_from_time(self, t: float):
        """
        It returns the WP the UAV is flying to
        Note: if t == wp.t, that wp is also considered as target, although they are at the same instant
        """

        if not self.waypoints:
            return 0

        for i, wp in enumerate(self.waypoints):
            if t <= wp.t:
                return i
        return i + 1
    
    def copy(self):
        """Realiza una copia profunda de la instancia actual de FlightPlan."""
        fp = FlightPlan()
        fp.id = self.id
        fp.priority = self.priority
        fp.radius = self.radius
        fp.max_var_lin_vel = self.max_var_lin_vel
        fp.max_var_ang_vel = self.max_var_ang_vel
        fp.target_yaw = self.target_yaw
        fp.waypoints = copy.deepcopy(self.waypoints)
        return fp

    def to_dict(self):
        return {
            "id": self.id,
            "priority": self.priority,
            "radius": self.radius,
            "max_var_lin_vel": self.max_var_lin_vel,
            "max_var_ang_vel": self.max_var_ang_vel,
            "target_yaw": self.target_yaw,
            "waypoints": [
                {
                    "label": wp.label,
                    "t": wp.t,
                    "pos": wp.pos.tolist(),
                    "vel": wp.vel.tolist(),
                    "acel": wp.acel.tolist(),
                    "jerk": wp.jerk.tolist(),
                    "snap": wp.snap.tolist(),
                    "crakle": wp.crakle.tolist(),
                    "fly_over": wp.fly_over,
                    "heading": wp.heading if wp.heading is not None else None
                }
                for wp in self.waypoints
            ]
        }
    
    def from_dict(self, data: dict):
        self.id = data.get("id", 0)
        self.priority = data.get("priority", 0)
        self.radius = data.get("radius", UAV_RADIUS)
        self.max_var_lin_vel = data.get("max_var_lin_vel", 5)
        self.max_var_ang_vel = data.get("max_var_ang_vel", 1)
        self.target_yaw = data.get("target_yaw", None)
        self.waypoints = []
        for wp_data in data.get("waypoints", []):
            wp = Waypoint(
                label=wp_data.get("label", ""),
                t=wp_data.get("t", 0),
                pos=wp_data.get("pos", [0, 0, 0]),
                vel=wp_data.get("vel", [0, 0, 0]),
                acel=wp_data.get("acel", [0, 0, 0]),
                jerk=wp_data.get("jerk", [0, 0, 0]),
                snap=wp_data.get("snap", [0, 0, 0]),
                crakle=wp_data.get("crakle", [0, 0, 0]),
                fly_over=wp_data.get("fly_over", False),
                heading=wp_data.get("heading", None)
            )
            self.waypoints.append(wp)

    def to_lists(self):
        times = [wp.t for wp in self.waypoints]
        positions = [wp.pos for wp in self.waypoints]
        velocities = [wp.vel for wp in self.waypoints]
        accelerations = [wp.acel for wp in self.waypoints]
        jerks = [wp.jerk for wp in self.waypoints]
        snaps = [wp.snap for wp in self.waypoints]
        crackels = [wp.crakle for wp in self.waypoints]
        headings = [wp.heading if wp.heading is not None else [0, 0] for wp in self.waypoints]
        
        return [times, positions, velocities, accelerations, jerks, snaps, crackels, headings]

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
            # self.waypoints[-1].stop()

        # Set vel velocity to all WPs
        elif wp is None:
            for i in range(len(self.waypoints)-1):
                self.set_uniform_velocity(wp=i, vel=vel)

            # Stop last WP
            self.waypoints[-1].stop()

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

            # Update wp1.vel if has no velocity yet or positions are the same
            if np.sum(wp1.vel) == 0 or np.array_equal(wp1.pos, wp2.pos):
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
            t2 = wp1.t + wp1.distance_to(wp2) / vel
            # Update wp2.t and postpone following WPs
            self.postpone_from(wp2.t, t2 - wp2.t)
            # Update wp1.vel
            wp1.set_uniform_velocity(wp2)

    def postpone_from(self, startTime: float, timeStep: float):
        # Postpone a portion of the Flight Plan a given time_delta,
        # starting from a given startTime
        if self.finish_time() < startTime:
            return

        # Find the first waypoint with time greater than or equal to startTime
        index = self.get_target_index_from_time(startTime)

        # If the requested postponement would move the waypoint earlier than
        # the previous waypoint (overlap), abort. The original condition
        # mistakenly returned when `timeStep` was *smaller* than the existing
        # gap, producing a no-op and leaving subsequent waypoints too close
        # to newly inserted ones (see hover detour failures). Use a more
        # intuitive guard: if postponing by `timeStep` would make the target
        # waypoint earlier than or equal to the previous one, abort.
        if index > 0:
            gap = self.waypoints[index].time_to(self.waypoints[index-1])
            if timeStep < 0 and abs(timeStep) >= gap:
                # Trying to move waypoint earlier beyond the previous waypoint
                return

        for i in range(index, len(self.waypoints)):
            # Do not postpone resolver-inserted hover markers; these are
            # intentionally placed by the resolver and should keep their
            # scheduled times relative to the anchor. Skipping labels that
            # start with 'HOV_' avoids double-postponement when callers
            # insert hover waypoints before calling postpone_from().
            wp = self.waypoints[i]
            if isinstance(wp.label, str) and wp.label.startswith("HOV_"):
                continue
            self.waypoints[i].postpone(timeStep)

    def postpone(self, timeStep: float) -> None:
        # Postpone the Flight Plan by a given timeStep
        self.postpone_from(self.init_time(), timeStep)

    def reschedule_at(self, time: float) -> None:
        # Perform a temporal translation of the Flight Plan to begin at a given time.
        self.postpone(time - self.init_time())

    #------------------------------------------------------------------------------------------------------------------
    # ROUTE MANAGEMENT

    def connect_waypoints(
        self,
        v_max: float = UAV_MAX_SPEED,
        a_max: float | None = UAV_MAX_ACCEL,
        strict: bool = True,
    ):
        """
        Computes the quintic-polynomial coefficients (jerk, snap, crakle) for
        every segment in the flight plan so that the trajectory passes exactly
        through each waypoint with the prescribed position, velocity, and
        acceleration.

        Before interpolating each segment, an optional kinematic feasibility
        check is performed via Waypoint.check_kinematic_feasibility().
        If the check fails:
          - In *strict* mode  (strict=True)  a ValueError is raised immediately,
            aborting the whole plan.  Use this during plan construction to catch
            impossible schedules early.
          - In *lenient* mode (strict=False) a WARNING is logged and the
            polynomial is computed anyway.  The resulting trajectory will respect
            the boundary conditions mathematically but may violate hardware
            limits at runtime.  This is the default to preserve backward
            compatibility with existing callers.

        Args:
            v_max  (float):      Maximum drone speed [m/s].
                                 Defaults to UAV_MAX_SPEED from config.
            a_max  (float|None): Maximum linear acceleration [m/s²].
                                 Defaults to UAV_MAX_ACCEL from config.
                                 Pass None to skip Guard 3.
            strict (bool):       If True, raise ValueError on infeasible
                                 segments instead of warning.  Default: False.
        """
        for i in range(len(self.waypoints) - 1):
            wpA = self.waypoints[i]
            wpB = self.waypoints[i + 1]

            # ------------------------------------------------------------------
            # Kinematic feasibility check
            # Verifies that the drone can physically travel from wpA to wpB
            # within the allotted time window given the hardware limits.
            # ------------------------------------------------------------------
            ok, reason = wpA.check_kinematic_feasibility(wpB, v_max=v_max, a_max=a_max)
            if not ok:
                msg = (
                    f"[connect_waypoints] Infeasible segment "
                    f"{wpA.label!r} -> {wpB.label!r}: {reason}"
                )
                if strict:
                    raise ValueError(msg)
                else:
                    log.warning(msg)

            # Compute the polynomial regardless (lenient mode) or after the
            # check has already confirmed feasibility (strict mode).
            wpA.connect_to(wpB)

    def smooth_waypoint_speed(self, wp, angVel):
        # Curva el vertice entre dos rectas
        # manteniendo velocidad y acortando el tiempo de vuelo
        # Para ello descompone dicho waypoint en dos

        if type(wp) == str:
            i = self.get_index_from_label(wp)
        elif type(wp) == int:
            i = wp

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
        wp2A = Waypoint(label=label, t=t, pos=pos, vel=vel, heading=[0, 0])
        # wp2A.t = wp2.t - step

        label = wp2.label + "_B"
        pos = wp2.pos + wp2.vel * step
        vel = wp2.vel
        wp2B = Waypoint(label=label, pos=pos, vel=vel, heading=[0, 0])
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
        
        self.remove_waypoint_at_time(wp2.t)
        self.set_waypoint(wp2A)
        self.set_waypoint(wp2B)

        self.postpone_from(wp2B.t + 0.001, wp2B.t - wp2BTinit)

    def smooth_waypoint_duration(self, wp, angVel, linAcel):
        # Curva el vertice entre dos rectas
        # reduciendo velocidad y manteniendo el tiempo de vuelo
        # Para ello descompone dicho waypoint en dos
        if type(wp) == str:
            i = self.get_index_from_label(wp)
        elif type(wp) == int:
            i = wp
            
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

        self.remove_waypoint_at_time(wp2.t)
        self.set_waypoint(wp2A)
        self.set_waypoint(wp2B)

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
        
        if t == self.finish_time():
            return self.waypoints[-1]

        if t > self.finish_time():
            return self.waypoints[-1].interpolation(t)

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
        instants = np.arange(self.init_time(), self.finish_time() + timeStep, timeStep)
        tr = np.zeros((len(instants), 10))
        tr[:, 0] = instants
        
        # Get position at each time instant
        for i in range(len(instants)):
            wp = self.status_at_time(tr[i, 0])
            tr[i, 1:4] = wp.pos
            tr[i, 4:7] = wp.vel
            tr[i, 7:10] = wp.acel
        
        tr[-1, 4:7] = [0, 0, 0]  # Set velocity to zero at the last time instant
        
        return tr

    #------------------------------------------------------------------------------------------------------------------
    # UAV NAVIGATION

    def get_command(self, currentTime, UAVpos, UAVvel, UAVrot : Rotation, WPheading, tToSolve) -> Command:
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
    
    #------------------------------------------------------------------------------------------------------------------
    # CONFLICT DETECTION

    def compare_to(self, fp2, time_step):
        decimals = len(str(time_step).split(".")[1])

        trace_1 = self.trace(time_step)
        trace_2 = fp2.trace(time_step)

        trace_1_times = np.round(trace_1[:, 0], decimals)
        trace_2_times = np.round(trace_2[:, 0], decimals)

        init_trace_1 = [[0]]
        init_trace_2 = [[0]]
        end_trace_1 = [[len(trace_1_times) - 1]]
        end_trace_2 = [[len(trace_2_times) - 1]]

        if trace_1_times[0] < trace_2_times[0]:     init_trace_1 = np.where(trace_1_times == trace_2_times[0])
        else:                                       init_trace_2 = np.where(trace_2_times == trace_1_times[0])

        if trace_1_times[-1] < trace_2_times[-1]:   end_trace_2 = np.where(trace_2_times == trace_1_times[-1])
        else:                                       end_trace_1 = np.where(trace_1_times == trace_2_times[-1])

        distance_separation = np.abs(trace_1[init_trace_1[0][0]:end_trace_1[0][0], 1:4] - 
                                     trace_2[init_trace_2[0][0]:end_trace_2[0][0], 1:4])
        
        distances = [np.linalg.norm(dist) for dist in distance_separation]

        return distances, trace_1_times[init_trace_1[0][0]:end_trace_1[0][0]]

    def generate_swept_boxes(self, interval=1.0) -> List[SweptBox_AABB]:
        
        """
        Generate a sequence of Swept Volumes (CCD) along the trajectory.
        
        The length of each box is proportional to the UAV's speed.
        """

        boxes = []
        t = self.init_time()
        t_final = self.finish_time()
        
        while t < t_final:
            t_next = min(t + interval, t_final)
            
            # With status_at_time, we obtain the position of the UAV at time t and t_next, which are the start and end of the interval.
            p_start = self.status_at_time(t).pos
            p_end = self.status_at_time(t_next).pos
            
            # The swept box is defined by the minimum and maximum coordinates of the start and end positions, 
            # expanded by the safety radius ( This is why every box is overlapping another one,
            # and also to ensure that there are no gaps in the coverage of the trajectory).
            
           
            p_min = np.minimum(p_start, p_end) - self.radius
            p_max = np.maximum(p_start, p_end) + self.radius
            
            # We save the swept box with its corresponding time interval.
            boxes.append(SweptBox_AABB(p_min, p_max, t, t_next))
            t = t_next
            
        return boxes
    
    def generate_swept_boxes_obb(self, interval=1.0) -> List[SweptBox_OBB]:
        """
        Generate a sequence of Swept Volumes (CCD) using Oriented Bounding Boxes (OBB).
        
        OBBs are rotated to align with the movement direction, reducing false positives.
        """
        boxes = []
        t = self.init_time()
        t_final = self.finish_time()
        
        while t < t_final:
            t_next = min(t + interval, t_final)
            
            p_start = self.status_at_time(t).pos
            p_end = self.status_at_time(t_next).pos
            
            # Calculate movement vector
            direction = p_end - p_start
            distance = np.linalg.norm(direction)
            
            if distance < 1e-10:
                # No movement - create a sphere-like OBB
                center = p_start
                forward = np.array([1, 0, 0])
                right = np.array([0, 1, 0])
                up = np.array([0, 0, 1])
                half_extents = np.array([self.radius, self.radius, self.radius])
            else:
                # Normalize direction (forward axis)
                forward = direction / distance
                
                # Create perpendicular axes (right and up)
                # Find a vector not parallel to forward
                if abs(forward[0]) < 0.9:
                    temp = np.array([1, 0, 0])
                else:
                    temp = np.array([0, 1, 0])
                
                right = np.cross(forward, temp)
                right = right / (np.linalg.norm(right) + 1e-10)
                up = np.cross(right, forward)
                up = up / (np.linalg.norm(up) + 1e-10)
                
                # Center between start and end
                center = (p_start + p_end) / 2
                
                # Half-extents: movement distance/2 along forward, radius in other directions
                half_extents = np.array([distance / 2, self.radius, self.radius])
            
            # Create OBB with axes as rows of matrix
            axes = np.array([forward, right, up])
            boxes.append(SweptBox_OBB(center, axes, half_extents, t, t_next))
            t = t_next
        
        return boxes
    #------------------------------------------------------------------------------------------------------------------
    # INFORMATION AND FIGURES

    def print_waypoints(self) -> None:
        """Prints all waypoints in the flight plan with their time, position, and velocity."""
        table = [
            [wp.label, wp.t, wp.pos, wp.vel, wp.acel, wp.jerk, wp.snap, wp.crakle] 
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
            text = f"T: {wp.t}\n"
            text += f"POS: {wp.pos}\n"
            text += f"VEL: {wp.vel}\n"
            text += f"ACEL: {wp.acel}"

            sel.annotation.set_text(text)

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
            t.append(wp.t)

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

        # Show the plots
        plt.show(block=False)

    @staticmethod
    def compare_velocity_flight_plans(flight_plans: List['FlightPlan'], figName: str, timeStep: float = 0.1) -> None:
        """
        Visualize velocity (magnitude and components) for multiple flight plans in the same figure.

        Args:
            flight_plans (List[FlightPlan]): List of flight plans to visualize
            figName (str): Title of the figure
            timeStep (float): Time step for trajectory sampling (default: 0.1)
        """

        if not flight_plans or len(flight_plans) == 0:
            print("No flight plans to visualize")
            return

        for i, fp in enumerate(flight_plans):
            if not fp.waypoints:
                print(f'Flight plan {i} is empty')
                return

        colors = [
            [0, 0.7, 1],
            [1, 0, 0],
            [0, 1, 0],
            [1, 1, 0],
            [1, 0, 1],
            [0, 1, 1],
            [1, 0.5, 0],
            [0.5, 0, 1],
        ]

        fig = plt.figure(figName)
        # Top: speed magnitude
        speedPlot = fig.add_subplot(4, 1, 1)
        speedPlot.set_title("Speed magnitude versus time")
        speedPlot.set_ylabel("speed [m/s]")
        speedPlot.grid(True)

        # Components: vx, vy, vz
        vxPlot = fig.add_subplot(4, 1, 2)
        vyPlot = fig.add_subplot(4, 1, 3)
        vzPlot = fig.add_subplot(4, 1, 4)

        vxPlot.set_ylabel("vx [m/s]")
        vyPlot.set_ylabel("vy [m/s]")
        vzPlot.set_ylabel("vz [m/s]")
        vzPlot.set_xlabel("t [s]")

        vxPlot.grid(True)
        vyPlot.grid(True)
        vzPlot.grid(True)

        for idx, fp in enumerate(flight_plans):
            color = colors[idx % len(colors)]
            tr = fp.trace(timeStep)
            tr_t = tr[:, 0]
            vx = tr[:, 4]
            vy = tr[:, 5]
            vz = tr[:, 6]
            speed = np.sqrt(vx**2 + vy**2 + vz**2)

            speedPlot.plot(tr_t, speed, linewidth=2, color=color, zorder=1, label=f"FP {idx+1}")
            vxPlot.plot(tr_t, vx, linewidth=2, color=color, zorder=1, label=f"FP {idx+1}")
            vyPlot.plot(tr_t, vy, linewidth=2, color=color, zorder=1)
            vzPlot.plot(tr_t, vz, linewidth=2, color=color, zorder=1)

            # Highlight waypoints velocities
            wp_vx = [wp.vel[0] for wp in fp.waypoints]
            wp_vy = [wp.vel[1] for wp in fp.waypoints]
            wp_vz = [wp.vel[2] for wp in fp.waypoints]
            wp_t = [wp.t for wp in fp.waypoints]

            wp_speeds = [np.linalg.norm(wp.vel) for wp in fp.waypoints]
            speedPlot.scatter(wp_t, wp_speeds, marker="o", color=color, s=25, zorder=3)
            vxPlot.scatter(wp_t, wp_vx, marker="o", color=color, s=25, zorder=3)
            vyPlot.scatter(wp_t, wp_vy, marker="o", color=color, s=25, zorder=3)
            vzPlot.scatter(wp_t, wp_vz, marker="o", color=color, s=25, zorder=3)

        speedPlot.legend(loc='upper right')
        vxPlot.legend(loc='upper right')

        plt.show(block=False)

    def velocity_figure(self, figName, timeStep):
        # Display the flight plan instant velocity

        # Check if the flight plan is empty
        if not self.waypoints:
            print('The flight plan is empty')
            return
        
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
            t.append(wp.t)

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
        maxLim = float(max(lim3D, xLim, yLim, zLim))

        velPlot3D.set_ylim(-maxLim, maxLim)
        xVelTimePlot.set_ylim(-maxLim, maxLim)
        yVelTimePlot.set_ylim(-maxLim, maxLim)
        zVelTimePlot.set_ylim(-maxLim, maxLim)

        # Show the plots
        plt.show(block=False)

    def acceleration_figure(self, figName, timeStep):
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
        tr_t = tr[:, 0]
        tr_x = tr[:, 7]
        tr_y = tr[:, 8]
        tr_z = tr[:, 9]

        # ACCELERATION 3D
        # Create plot
        accPlot3D = velFig.add_subplot(4, 2, (1, 2))
        
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
        xAccTimePlot = velFig.add_subplot(4, 2, (3, 4))
        yAccTimePlot = velFig.add_subplot(4, 2, (5, 6))
        zAccTimePlot = velFig.add_subplot(4, 2, (7, 8))
        
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
            t.append(wp.t)

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
        maxLim = float(max(lim3D, xLim, yLim, zLim))

        accPlot3D.set_ylim(-maxLim, maxLim)
        xAccTimePlot.set_ylim(-maxLim, maxLim)
        yAccTimePlot.set_ylim(-maxLim, maxLim)
        zAccTimePlot.set_ylim(-maxLim, maxLim)

        # Show the plots
        plt.show(block=False)

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
            timeUAV.append(wp.t)

            # Compute errors between UAV and flight plan
            status = self.status_at_time(wp.t)
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
            timeUAV.append(wp.t)

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
            timeUAV.append(wp.t)

        xVelUAV = np.array(xVelUAV)
        yVelUAV = np.array(yVelUAV)
        zVelUAV = np.array(zVelUAV)

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

    @staticmethod
    def _draw_swept_box(ax, sweptbox, color, alpha=0.1):
        """
        Helper method to draw a swept box in 3D.
        Detects if it's AABB or OBB and draws accordingly.
        
        Args:
            ax: matplotlib 3D axis
            sweptbox: SweptBox_AABB or SweptBox_OBB object
            color: color for the box
            alpha: transparency level
        """
        # Detect type based on attributes
        if hasattr(sweptbox, 'center'):  # OBB
            FlightPlan._draw_swept_box_obb(ax, sweptbox, color, alpha)
        else:  # AABB
            FlightPlan._draw_swept_box_aabb(ax, sweptbox, color, alpha)

    @staticmethod
    def _draw_swept_box_aabb(ax, sweptbox, color, alpha=0.1):
        """Draw Axis-Aligned Bounding Box (AABB)."""
        # Define the 8 corners of the bounding box
        p_min = sweptbox.min
        p_max = sweptbox.max
        
        corners = [
            [p_min[0], p_min[1], p_min[2]],
            [p_max[0], p_min[1], p_min[2]],
            [p_max[0], p_max[1], p_min[2]],
            [p_min[0], p_max[1], p_min[2]],
            [p_min[0], p_min[1], p_max[2]],
            [p_max[0], p_min[1], p_max[2]],
            [p_max[0], p_max[1], p_max[2]],
            [p_min[0], p_max[1], p_max[2]],
        ]
        
        # Define the 12 edges of the cube
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
            [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
            [0, 4], [1, 5], [2, 6], [3, 7],  # Vertical edges
        ]
        
        corners = np.array(corners)
        
        # Draw edges
        for edge in edges:
            points = corners[edge]
            ax.plot3D(*points.T, color=color, linewidth=0.5, alpha=0.6)
        
        # Draw filled faces with transparency
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        
        # Define the 6 faces of the cube
        faces = [
            [corners[0], corners[1], corners[5], corners[4]],  # Front
            [corners[2], corners[3], corners[7], corners[6]],  # Back
            [corners[0], corners[3], corners[7], corners[4]],  # Left
            [corners[1], corners[2], corners[6], corners[5]],  # Right
            [corners[0], corners[1], corners[2], corners[3]],  # Bottom
            [corners[4], corners[5], corners[6], corners[7]],  # Top
        ]
        
        poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor=color, linewidth=0.5)
        ax.add_collection3d(poly)

    @staticmethod
    def _draw_swept_box_obb(ax, sweptbox, color, alpha=0.1):
        """Draw Oriented Bounding Box (OBB)."""
        # Get the 8 corners of the OBB
        corners = sweptbox.get_corners()
        corners = np.array(corners)
        
        # Define the 12 edges connecting the corners
        edges = [
            [0, 1], [1, 3], [3, 2], [2, 0],  # Bottom face
            [4, 5], [5, 7], [7, 6], [6, 4],  # Top face
            [0, 4], [1, 5], [2, 6], [3, 7],  # Vertical edges
        ]
        
        # Draw edges
        for edge in edges:
            points = corners[edge]
            ax.plot3D(*points.T, color=color, linewidth=1.0, alpha=0.8)
        
        # Draw filled faces with transparency
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        
        # Define the 6 faces using corner indices
        faces = [
            [corners[0], corners[1], corners[5], corners[4]],  # Face 1
            [corners[2], corners[3], corners[7], corners[6]],  # Face 2
            [corners[0], corners[2], corners[6], corners[4]],  # Face 3
            [corners[1], corners[3], corners[7], corners[5]],  # Face 4
            [corners[0], corners[1], corners[3], corners[2]],  # Face 5
            [corners[4], corners[5], corners[7], corners[6]],  # Face 6
        ]
        
        poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor=color, linewidth=0.8)
        ax.add_collection3d(poly)

    @staticmethod
    def compare_flight_plans(flight_plans: List['FlightPlan'], figName: str, timeStep: float = 0.1, 
                            show_swept_boxes: bool = False, box_interval: float = 1.0, box_type: str = "aabb") -> None:
        """
        Visualize multiple flight plans in the same figure with different colors.
        
        Args:
            flight_plans (List[FlightPlan]): List of flight plans to visualize
            figName (str): Title of the figure
            timeStep (float): Time step for trajectory sampling (default: 0.1)
            show_swept_boxes (bool): Whether to display swept boxes for collision detection (default: False)
            box_interval (float): Time interval for generating swept boxes (default: 1.0)
            box_type (str): Type of boxes - "aabb" or "obb" (default: "aabb")
        """
        
        if not flight_plans or len(flight_plans) == 0:
            print("No flight plans to visualize")
            return
        
        # Check if any flight plan is empty
        for i, fp in enumerate(flight_plans):
            if not fp.waypoints:
                print(f'Flight plan {i} is empty')
                return
        
        # Color palette for different flight plans (including various hues)
        colors = [
            [0, 0.7, 1],      # Blue
            [1, 0, 0],         # Red
            [0, 1, 0],         # Green
            [1, 1, 0],         # Yellow
            [1, 0, 1],         # Magenta
            [0, 1, 1],         # Cyan
            [1, 0.5, 0],       # Orange
            [0.5, 0, 1],       # Purple
        ]
        
        # Create matplotlib figure
        posFig = plt.figure(figName)
        posFig.canvas.manager.toolmanager.add_tool("ToogleUAV", ToggleUAVtracking, gid="UAVtracking")
        posFig.canvas.manager.toolbar.add_tool('ToogleUAV', 'navigation', 1)

        # POSITION 3D
        xyzPosPlot = posFig.add_subplot(6, 5, (1, 28), projection="3d")
        xyzPosPlot.set_xlabel("x [m]")
        xyzPosPlot.set_ylabel("y [m]")
        xyzPosPlot.set_zlabel("z [m]")
        xyzPosPlot.set_title("Position 3D - Multiple Flight Plans")
        xyzPosPlot.grid(True)

        # POSITION ERROR VERSUS TIME
        # xyzPosErrorPlot = posFig.add_subplot(6, 5, (26, 28))
        # xyzPosErrorPlot.set_xlabel("t [s]")
        # xyzPosErrorPlot.set_ylabel("Error [m]")
        # xyzPosErrorPlot.set_title("Position error versus time")
        # xyzPosErrorPlot.grid(True)

        # POSITIONS VERSUS TIME
        xPosTimePlot = posFig.add_subplot(6, 5, (4, 10))
        yPosTimePlot = posFig.add_subplot(6, 5, (14, 20))
        zPosTimePlot = posFig.add_subplot(6, 5, (24, 30))
        
        xPosTimePlot.set_ylabel("x [m]")
        yPosTimePlot.set_ylabel("y [m]")
        zPosTimePlot.set_ylabel("z [m]")
        zPosTimePlot.set_xlabel("t [s]")
        xPosTimePlot.set_title("Position versus time")
        
        xPosTimePlot.grid(True)
        yPosTimePlot.grid(True)
        zPosTimePlot.grid(True)

        # Determine box generation method
        if box_type.lower() == "obb":
            print("Using Oriented Bounding Boxes (OBB) for visualization")
            box_generator = lambda fp: fp.generate_swept_boxes_obb(interval=box_interval)
        else:
            print("Using Axis-Aligned Bounding Boxes (AABB) for visualization")
            box_generator = lambda fp: fp.generate_swept_boxes(interval=box_interval)

        # Plot each flight plan
        for idx, fp in enumerate(flight_plans):
            color = colors[idx % len(colors)]
            
            # Get trace
            tr = fp.trace(timeStep)
            tr_t = tr[:, 0]
            tr_x = tr[:, 1]
            tr_y = tr[:, 2]
            tr_z = tr[:, 3]
            
            # Plot trajectories
            xyzPosPlot.plot(tr_x, tr_y, tr_z, linewidth=2, color=color, zorder=1, label=f"FP {idx+1}")
            xPosTimePlot.plot(tr_t, tr_x, linewidth=2, color=color, zorder=1, label=f"FP {idx+1}")
            yPosTimePlot.plot(tr_t, tr_y, linewidth=2, color=color, zorder=1, label=f"FP {idx+1}")
            zPosTimePlot.plot(tr_t, tr_z, linewidth=2, color=color, zorder=1, label=f"FP {idx+1}")
            
            # Highlight waypoints
            xPos = [wp.pos[0] for wp in fp.waypoints]
            yPos = [wp.pos[1] for wp in fp.waypoints]
            zPos = [wp.pos[2] for wp in fp.waypoints]
            t = [wp.t for wp in fp.waypoints]
            
            xyzPosPlot.scatter(xPos, yPos, zPos, marker="o", color=color, s=25, pickradius=30, zorder=3)
            xPosTimePlot.scatter(t, xPos, marker="o", color=color, s=25, pickradius=30, zorder=3)
            yPosTimePlot.scatter(t, yPos, marker="o", color=color, s=25, pickradius=30, zorder=3)
            zPosTimePlot.scatter(t, zPos, marker="o", color=color, s=25, pickradius=30, zorder=3)
            
            # Draw swept boxes if requested
            if show_swept_boxes:
                boxes = box_generator(fp)
                for box in boxes:
                    FlightPlan._draw_swept_box(xyzPosPlot, box, color, alpha=0.05)
        
        # Add legends
        xyzPosPlot.legend(loc='upper right')
        xPosTimePlot.legend(loc='upper right')
        yPosTimePlot.legend(loc='upper right')
        zPosTimePlot.legend(loc='upper right')
        
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

        # Fix matplotlib 3D scroll zoom bug by wrapping _set_view_from_bbox
        # This prevents ValueError when unpacking 3 values into 4 variables
        original_set_view = xyzPosPlot._set_view_from_bbox
        
        def safe_set_view_from_bbox(bbox, *args, **kwargs):
            """Safely handle _set_view_from_bbox for 3D axes, preventing unpacking errors."""
            try:
                # Only call original if bbox has correct number of elements
                if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
                    return original_set_view(bbox, *args, **kwargs)
                # Otherwise, ignore and return (silently handle 3D zoom errors)
                return
            except (ValueError, TypeError):
                # Silently ignore the error
                return
        
        xyzPosPlot._set_view_from_bbox = safe_set_view_from_bbox

        # Show the plots
        plt.show(block=False)

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
                if scatter.get_gid() == self.gid:  # type: ignore
                    scatter.set_visible(state)  # type: ignore
        self.figure.canvas.draw()
