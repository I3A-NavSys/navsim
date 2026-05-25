import os

import numpy as np

import omni.ext
import omni.timeline
import carb.events
import omni.kit.app
import omni.physx
from isaacsim.core.prims import RigidPrim

from isaacsim.util.debug_draw import _debug_draw

from .uav_control import UAVControl
from uspace.uav_conflict_resolver.visualizers.viz_08_scenario_data import get_viz08_flightplans


VIZ08_COLORS = [
    (0x2e / 255.0, 0xcc / 255.0, 0x71 / 255.0, 1.0),
    (0xe7 / 255.0, 0x4c / 255.0, 0x3c / 255.0, 1.0),
    (0x34 / 255.0, 0x98 / 255.0, 0xdb / 255.0, 1.0),
    (0xf1 / 255.0, 0xc4 / 255.0, 0x0f / 255.0, 1.0),
    (0x9b / 255.0, 0x59 / 255.0, 0xb6 / 255.0, 1.0),
    (0x1a / 255.0, 0xbc / 255.0, 0x9c / 255.0, 1.0),
    (0xe6 / 255.0, 0x7e / 255.0, 0x22 / 255.0, 1.0),
    (0x34 / 255.0, 0x49 / 255.0, 0x5e / 255.0, 1.0),
    (0x7f / 255.0, 0x8c / 255.0, 0x8d / 255.0, 1.0),
    (0x16 / 255.0, 0xa0 / 255.0, 0x85 / 255.0, 1.0),
]

class UAVControlBridge(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.initialize_variables()

    def on_shutdown(self):
        if hasattr(self, "debug_draw") and self.debug_draw is not None:
            self.debug_draw.clear_lines()
        self.on_physics_step_sub = None
        self.op_stop_sub = None
        self.on_play_sub = None
        self.on_flightplan_request_event_sub = None
        self.on_command_request_event_sub = None

    def build_flightplans(self):
        # Build the bridge data from whatever UAV prims exist in the scene.
        # This removes the old two-UAV assumption and keeps the code aligned with
        # the number of UAVs that Isaac Sim actually spawned.
        ordered_uavs = self._get_ordered_uav_entries()
        uav_physics_indices = [entry[0] for entry in ordered_uavs]

        if len(ordered_uavs) != 9:
            raise ValueError(
                f"Viz 08 full cascade expects exactly 9 UAVs in the Isaac Sim scene, got {len(ordered_uavs)}."
            )

        selected_flightplans = get_viz08_flightplans(
            case=self.viz08_case,
            n_uavs=len(ordered_uavs),
        )
        self.viz08_flightplans = selected_flightplans

        flightplan_as_lists = []
        for _, _, uav_id in ordered_uavs:
            plan_id = self._get_viz08_plan_id_for_uav_id(uav_id)
            if plan_id not in selected_flightplans:
                raise KeyError(
                    f"No Viz 08 flight plan was generated for UAV id '{uav_id}' ({plan_id})."
                )
            start_pos = selected_flightplans[plan_id].waypoints[0].pos
            print(f"[UAVControlBridge] Mapped prim {uav_id} -> {plan_id} (start: {start_pos})")
            flightplan_as_lists.append(selected_flightplans[plan_id].to_lists())

        if len(flightplan_as_lists) != len(uav_physics_indices):
            raise ValueError(
                "The number of generated flight plans does not match the UAVs "
                "present in the Isaac Sim scene."
            )

        times = [fp[0] for fp in flightplan_as_lists]
        positions = [fp[1] for fp in flightplan_as_lists]
        velocities = [fp[2] for fp in flightplan_as_lists]
        accelerations = [fp[3] for fp in flightplan_as_lists]
        jerks = [fp[4] for fp in flightplan_as_lists]
        snaps = [fp[5] for fp in flightplan_as_lists]
        crackels = [fp[6] for fp in flightplan_as_lists]
        headings = [fp[7] for fp in flightplan_as_lists]

        current_flightplans = [
            times,
            positions,
            velocities,
            accelerations,
            jerks,
            snaps,
            crackels,
            headings,
        ]

        self._draw_viz08_flightplans(selected_flightplans, ordered_uavs)

        return uav_physics_indices, current_flightplans

    def _heading_to_quaternion(self, heading):
        if heading is None:
            return [1.0, 0.0, 0.0, 0.0]

        heading_x, heading_y = float(heading[0]), float(heading[1])
        if abs(heading_x) < 1e-9 and abs(heading_y) < 1e-9:
            return [1.0, 0.0, 0.0, 0.0]

        yaw = float(np.arctan2(heading_y, heading_x))
        half_yaw = yaw * 0.5
        return [float(np.cos(half_yaw)), 0.0, 0.0, float(np.sin(half_yaw))]

    def _draw_viz08_flightplans(self, selected_flightplans, ordered_uavs):
        if not hasattr(self, "debug_draw") or self.debug_draw is None:
            return

        self.debug_draw.clear_lines()

        for idx, (_, _, uav_id) in enumerate(ordered_uavs):
            plan_id = self._get_viz08_plan_id_for_uav_id(uav_id)
            flight_plan = selected_flightplans.get(plan_id)
            if flight_plan is None:
                continue

            fp_trace = flight_plan.trace(0.1)
            color = VIZ08_COLORS[idx % len(VIZ08_COLORS)]
            self.debug_draw.draw_lines_spline(
                fp_trace[:, 1:4],
                color,
                5,
                False,
            )

    def _sort_viz08_plan_id(self, plan_id: str):
        # Viz 08 plans are named UAV_0, UAV_1, ..., so we sort them numerically
        # before pairing them with the UAV prims found in the scene.
        return int(plan_id.split("_")[-1])

    def _get_viz08_plan_id_for_uav_id(self, uav_id):
        # The scene-side UAV IDs are zero-based, and the Viz 08 flight plans
        # use the same naming convention (UAV_0..UAV_8).
        if isinstance(uav_id, str):
            return uav_id

        return f"UAV_{int(uav_id)}"

    def _get_ordered_uav_entries(self):
        # Flatten the nested {operator_id: {uav_id: physics_index}} structure into
        # a single ordered list so the simulator-side UAV order is deterministic.
        entries = []
        for operator_id, uavs in self.uav_ids_to_physics_buffer.items():
            for uav_id, physics_index in uavs.items():
                entries.append((physics_index, operator_id, uav_id))

        entries.sort(key=lambda entry: entry[0])
        return entries

    def _get_viz08_case(self):
        # Keep the runtime selection lightweight: an environment variable is easy
        # to document and avoids adding extra UI plumbing for this bridge.
        case = os.getenv("UAV_CONTROL_BRIDGE_VIZ08_CASE", "resolved")
        case = case.strip().lower()
        if case not in {"original", "resolved"}:
            return "original"
        return case

    def on_physics_step(self, step_size: int):
        if self.is_simulation_running:
            # Get current time
            current_time = self.timeline.get_current_time()

            # Update UAV control
            if self.uav_physics_indices:
                self.uav_control.update(
                    self.uav_physics_indices, 
                    self.current_flightplans, 
                    current_time, 
                    step_size
                )

    def on_timeline_stop(self, event):
        if self.is_simulation_running:
            self.rigid_prim = None
            if hasattr(self, "debug_draw") and self.debug_draw is not None:
                self.debug_draw.clear_lines()

            self.is_simulation_running = False

    def on_timeline_play(self, event):
        if not self.is_simulation_running:
            # Start RigidPrimView
            self.rigid_prim = RigidPrim("/World/UAVs/UAV_*")
            self.rigid_prim.initialize()

            # Relate UAV ids to physics buffer indices
            self.relate_uav_ids_to_physics_buffer()

            # Start UAVControl
            self.uav_control = UAVControl(
                self.rigid_prim, 
                self.uav_ids_to_physics_buffer
            )

            # Load the selected Viz 08 scenario once, instead of rebuilding it on
            # every physics step. That keeps the control loop deterministic and
            # cheaper when the UAV count grows.
            self.uav_physics_indices, self.current_flightplans = self.build_flightplans()

            # Make sure the simulated UAVs start exactly at the first waypoint
            # of their assigned flight plans.
            self._reset_uav_poses_to_flightplan_starts()

            # Set simulation as running
            self.is_simulation_running = True

    # ----------------------------------
    # -------- INITIALIZATION ----------
    # ----------------------------------
    def initialize_variables(self):

        self.debug_draw = _debug_draw.acquire_debug_draw_interface()
        # Control state is initialized here so the bridge can be restarted cleanly
        # without keeping stale UAV mappings from a previous simulation run.
        self.is_simulation_running = False
        self.uav_control = None
        self.uav_physics_indices = None
        self.current_flightplans = None
        # Allow the Viz08 case to be selected via environment variable.
        self.viz08_case = self._get_viz08_case()
        self.uav_ids_to_physics_buffer = {} # {operator_id: {uav_id: physics_buffer_index}}

        # Event stream
        self.bus_event_stream = omni.kit.app.get_app().get_message_bus_event_stream()
        self.FLIGHTPLAN_REQUEST_EVENT = carb.events.type_from_string(
            "navsim.REQUEST_FLIGHTPLAN_EVENT"
        )
        self.COMMAND_REQUEST_EVENT = carb.events.type_from_string(
            "navsim.REQUEST_COMMAND_EVENT"
        )
        self.on_flightplan_request_event_sub = self.bus_event_stream.create_subscription_to_push_by_type(
            self.FLIGHTPLAN_REQUEST_EVENT, 
            self.on_flightplan_request_event
        )
        self.on_command_request_event_sub = self.bus_event_stream.create_subscription_to_push_by_type(
            self.COMMAND_REQUEST_EVENT, 
            self.on_command_request_event
        )

        # Timeline callbacks
        self.timeline = omni.timeline.get_timeline_interface()
        timeline_stream = self.timeline.get_timeline_event_stream()
        self.on_stop_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), 
            self.on_timeline_stop
        )
        self.on_play_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), 
            self.on_timeline_play
        )
        
        # Physx callbacks
        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(
            fn=lambda step_size: self.on_physics_step(step_size),
            pre_step=True,
            order=10
        )

    def relate_uav_ids_to_physics_buffer(self):
        # Get UAV prims
        uavs = self.rigid_prim.prims

        for idx, uav in enumerate(uavs):
            # Get UAV operator id and UAV id from prim attributes
            uav_operator_id = uav.GetAttribute("NavSim:operator_id").Get()
            uav_id = uav.GetAttribute("NavSim:id").Get()

            # Relate UAV id to physics buffer index for each UAV
            if uav_operator_id not in self.uav_ids_to_physics_buffer:
                self.uav_ids_to_physics_buffer[uav_operator_id] = {}

            self.uav_ids_to_physics_buffer[uav_operator_id][uav_id] = idx

    def _reset_uav_poses_to_flightplan_starts(self):
        if not getattr(self, "viz08_flightplans", None) or not self.uav_physics_indices:
            return

        current_time = self.timeline.get_current_time()

        ordered_uavs = self._get_ordered_uav_entries()
        start_positions = []
        start_headings = []

        for _, _, uav_id in ordered_uavs:
            plan_id = self._get_viz08_plan_id_for_uav_id(uav_id)
            flight_plan = self.viz08_flightplans.get(plan_id)
            if flight_plan is None:
                continue

            current_wp = flight_plan.status_at_time(current_time)
            start_positions.append(current_wp.pos)
            if current_wp.heading is None:
                start_headings.append(current_wp.vel[:2])
            else:
                start_headings.append(current_wp.heading)

        start_orientations = [self._heading_to_quaternion(heading) for heading in start_headings]
        self.rigid_prim.set_world_poses(
            positions=start_positions,
            orientations=start_orientations,
            indices=self.uav_physics_indices,
        )
            

    # -----------------------------
    # -------- CALLBACKS ----------
    # ----------------------------- 
    def on_flightplan_request_event(self, event):
        # Here we would normally get the flight plan for the UAV that is being requested, but for this example we will just print the event payload
        print("Flight plan request event received with payload:", event.payload)

    def on_command_request_event(self, event):
        # Here we would normally get the command for the UAV that is being requested, but for this example we will just print the event payload
        print("Command request event received with payload:", event.payload)
