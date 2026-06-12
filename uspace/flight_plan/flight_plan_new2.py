from tabulate import tabulate
import copy
from typing import List, Optional
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

    def __init__(self):
        self.id: int = 0
        self.priority: int = 0
        self.radius: float = 1
        self.max_var_lin_vel = 5        # maximum variation in linear  velocity   [  m/s]
        self.max_var_ang_vel = 1        # maximum variation in angular velocity   [rad/s]
        self.target_yaw = None
        self.waypoints: SortedList[Waypoint] = SortedList([])
        self.time_waypoints: SortedList[float] = SortedList([])
        self.length: int = 0
        self.figure_processes = []

    def set_waypoint(self, wp=None, label="", time=None, pos=None, vel=None, heading=[0,0]):
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

            wp = Waypoint(label=label, t=time, pos=pos, vel=vel, heading=heading)
        
        index = self.time_waypoints.bisect_left(wp.t)

        # Replace if the same time already exists
        if index < self.length and self.time_waypoints[index] == wp.t:
            self.waypoints[index] = wp
        else:
            self.waypoints.add(wp)

        self.length += 1

    def remove_waypoint(
        self, 
        idx: Optional[int],  
        # label: Optional[str], 
        time: Optional[float]
    ) -> None:
        """
        Remove a waypoint from the flight plan by index, or time.
        If no index or time is provided, the last waypoint will be removed.

        Args:
            idx (Optional[int]): The index of the waypoint to remove. If None, the waypoint will be removed by time.
            time (Optional[float]): The time of the waypoint to remove. If None, the waypoint will be removed by index.
        """

        # Check if there is any waypoint to remove
        if self.length == 0:
            return

        remove_by_idx = idx is not None
        remove_by_time = time is not None

        # Remove the last waypoint
        if not remove_by_idx and not remove_by_time:
            self.waypoints.pop()
            self.time_waypoints.pop()
            self.length -= 1
            return
        
        # Remove by index
        if remove_by_idx:
            # Check if the index is valid
            if idx < 0 or idx >= self.length:
                raise IndexError(f"Index {idx} is out of range for FlightPlan of length {self.length}.")
            
            self.waypoints.pop(idx)
            self.time_waypoints.pop(idx)
            self.length -= 1
            return
        
        # Remove by time
        if time >= self.time_waypoints[-1]:
            pointed_wp_idx = self.length - 1
        else:
            pointed_wp_idx = self.time_waypoints.bisect_left(time)
        
        self.waypoints.pop(pointed_wp_idx)
        self.time_waypoints.pop(pointed_wp_idx)
        self.length -= 1
        

    def get_index_from_label(self, label: str) -> Optional[int]:
        for i, wp in enumerate(self.waypoints):
            if wp.label == label:
                return i
        return None
    
    def get_running_waypoint(self, t: float):
        return self.waypoints[self.get_running_index_from_time(t)]

    def get_running_index_from_time(self, t: float):
        """
        It returns the WP the UAV is currently executing
        """
        
        index = self.get_target_index_from_time(t)
        running_i = 0 if index == 0 else index - 1
        return running_i
    
    def get_target_index_from_time(self, t: float):
        """
        It returns the WP the UAV is flying to.
        Note: if t == wp.t, that wp is also considered as target.
        O(log n) binary search instead of O(n) linear scan.
        """
        if not self.waypoints:
            return 0
        return bisect.bisect_left(_TimesView(self.waypoints), t)
    
    def copy(self):
        """Realiza una copia profunda de la instancia actual de FlightPlan."""
        fp = FlightPlan()
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
        self.radius = data.get("radius", 1)
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
        headings = [wp.heading for wp in self.waypoints]
        
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

    def remove_negative_time(self) -> None:
        """
        Postpone the flight plan if it contains waypoints with negative time, 
        so that the first waypoint starts at time 0.
        """
        if self.time_waypoints[0] < 0:
            self.postpone(-self.time_waypoints[0])


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

        if index > 0 and timeStep < self.waypoints[index].time_to(self.waypoints[index-1]):
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

    def smooth_waypoint_speed(self, wp, angVel):
        # Curva el vertice entre dos rectas
        # manteniendo velocidad y acortando el tiempo de vuelo
        # Para ello descompone dicho waypoint en dos

        if type(wp) == str:
            i : int = self.get_index_from_label(wp)
        elif type(wp) == int:
            i = wp

        if (i== 0) or (i == len(self.waypoints) - 1) or (i is None):
            raise RuntimeError(f"Trying to smooth invalid WP: {i})")
      
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
        
        self.remove_waypoint_at_time(wp2.t)
        self.set_waypoint(wp2A)
        self.set_waypoint(wp2B)

        self.postpone_from(wp2B.t + 0.001, wp2B.t - wp2BTinit)

    def smooth_waypoint_duration(self, wp, angVel, linAcel):
        # Curva el vertice entre dos rectas
        # reduciendo velocidad y manteniendo el tiempo de vuelo
        # Para ello descompone dicho waypoint en dos
        if type(wp) == str:
            i : int = self.get_index_from_label(wp)
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

        # Binary search: find the last waypoint whose time <= t, O(log n).
        idx = bisect.bisect_right(_TimesView(self.waypoints), t) - 1
        return self.waypoints[max(idx, 0)].interpolation(t)
    
    def trace(self, timeStep):
        # This method expands the flight plan behavior over time.
        # Vectorised segment-by-segment Taylor expansion:
        #   1. searchsorted assigns every instant to its segment in O(n log n)
        #   2. per segment all instants are evaluated in a single numpy broadcast
        # Overall: O(n_wp + n_pts) instead of the previous O(n_pts * log(n_wp)).
        instants = np.arange(self.init_time(), self.finish_time() + timeStep, timeStep)
        tr = np.empty((len(instants), 10))
        tr[:, 0] = instants

        wp_times = np.array([wp.t for wp in self.waypoints])
        seg = np.searchsorted(wp_times, instants, side='right') - 1
        seg = np.clip(seg, 0, len(self.waypoints) - 1)

        for si in range(len(self.waypoints)):
            mask = seg == si
            if not np.any(mask):
                continue
            wp = self.waypoints[si]
            dt  = (instants[mask] - wp.t)[:, None]  # (k,1) — broadcasts over xyz
            dt2 = dt  * dt
            dt3 = dt2 * dt
            dt4 = dt3 * dt
            dt5 = dt4 * dt
            r, v, a = wp.pos, wp.vel, wp.acel
            j, sn, c = wp.jerk, wp.snap, wp.crakle
            tr[mask, 1:4]  = r + dt*v   + (dt2*0.5)*a  + (dt3/6)*j   + (dt4/24)*sn  + (dt5/120)*c
            tr[mask, 4:7]  = v + dt*a   + (dt2*0.5)*j  + (dt3/6)*sn  + (dt4/24)*c
            tr[mask, 7:10] = a + dt*j   + (dt2*0.5)*sn + (dt3/6)*c

        tr[-1, 4:7] = 0.0   # zero velocity at the last instant
        return tr

    #------------------------------------------------------------------------------------------------------------------
    # UAV NAVIGATION
    
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

    #------------------------------------------------------------------------------------------------------------------
    # CONFLICT DETECTION

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
