import numpy as np

import omni.ext
import omni.timeline
import carb.events
import omni.kit.app
import omni.physx
from isaacsim.core.prims import RigidPrim

from .uav_control import UAVControl
from uspace.flight_plan.flight_plan import FlightPlan

class UAVControlBridge(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.initialize_variables()

    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.op_stop_sub = None
        self.on_play_sub = None
        self.on_flightplan_request_event_sub = None
        self.on_command_request_event_sub = None

    def build_flightplans(self):
        flightplan_as_lists = []

        uav_physics_indices = [
            self.uav_ids_to_physics_buffer["UAV_OPERATOR_0"]["UAV_0"],
            self.uav_ids_to_physics_buffer["UAV_OPERATOR_0"]["UAV_1"]
        ]

        # UAV_0 FlightPlan
        positions = np.array([
            [-0.60193, -0.51592, 9.99345],
            [-0.60193, -0.51592, 14.34305],
            [75.11271, -43.3532, 19.27086],
            [131.01348, 0, 19.27086],
            [318.85296, 0, 19.27086],
            [410.57898, -96.92693, -11.36428],
            [436.30316, -157.16426, -48.94219],
            [436.30316, -157.16426, -48.9422],
        ])
        magnitudes = np.array([
            np.linalg.norm(positions[1] - positions[0]),
            np.linalg.norm(positions[2] - positions[1]),
            np.linalg.norm(positions[3] - positions[2]),
            np.linalg.norm(positions[4] - positions[3]),
            np.linalg.norm(positions[5] - positions[4]),
            np.linalg.norm(positions[6] - positions[5]),
            np.linalg.norm(positions[7] - positions[6])
        ])
        velocities = np.array([
            [0, 0, 0],
            (positions[2] - positions[1]) / magnitudes[1],
            (positions[3] - positions[2]) / magnitudes[2],
            (positions[4] - positions[3]) / magnitudes[3],
            (positions[5] - positions[4]) / magnitudes[4],
            (positions[6] - positions[5]) / magnitudes[5],
            (positions[7] - positions[6]) / magnitudes[6],
            [0, 0, 0]
        ])

        velocities[1] *= np.array([3, 3, 2])
        velocities[2] *= np.array([3, 3, 1])
        velocities[3] += np.array([5, 5, 0])
        velocities[4] *= np.array([3.5, 3, 1])
        velocities[5] *= np.array([1, 2, 1])

        wp_amount = len(positions)
        times = np.zeros(wp_amount)
        times[0] = 10
        times[1] = 15
        for i in range(2, wp_amount):
            times[i] = times[i - 1] + magnitudes[i-1] / np.linalg.norm(velocities[i-1])

        fp0 = FlightPlan()
        fp0.set_waypoint(
            label="wp",
            time=times[0],
            pos=positions[0],
            vel=velocities[0],
            heading=[velocities[1][0], velocities[1][1]]
        )
        fp0.set_waypoint(
            label="wp_01",
            time=times[1],
            pos=positions[1],
            vel=velocities[1],
        )
        fp0.set_waypoint(
            label="wp_02",
            time=times[2],
            pos=positions[2],
            vel=velocities[2],
        )
        fp0.set_waypoint(
            label="wp_03",
            time=times[3],
            pos=positions[3],
            vel=velocities[3],
        )
        fp0.set_waypoint(
            label="wp_04",
            time=times[4],
            pos=positions[4],
            vel=velocities[4],
        )
        fp0.set_waypoint(
            label="wp_05",
            time=times[5],
            pos=positions[5],
            vel=velocities[5],
        )
        fp0.set_waypoint(
            label="wp_06",
            time=times[6],
            pos=positions[6],
            vel=velocities[6],
        )
        fp0.set_waypoint(
            label="wp_07",
            time=times[7],
            pos=positions[7],
            vel=velocities[7],
        )
        fp0.connect_waypoints()

        # UAV_1 FlightPlan
        positions = np.array([
            [436.30316, -157.16426, -48.94219],
            [436.30316, -157.16426, -48.9422],
            [394.76271, -142.86073, -11.36428],
            [37.57808, -19.87221, 19.27086],
            [-0.60193, -0.51592, 14.34305],
            [-0.60193, -0.51592, 9.99345],
        ])
        magnitudes = np.array([
            0,
            np.linalg.norm(positions[2] - positions[1]),
            np.linalg.norm(positions[3] - positions[2]),
            np.linalg.norm(positions[4] - positions[3]),
            np.linalg.norm(positions[5] - positions[4]),
            0
        ])
        velocities = np.array([
            [0, 0, 0],
            (positions[2] - positions[1]) / magnitudes[1],
            (positions[3] - positions[2]) / magnitudes[2],
            (positions[4] - positions[3]) / magnitudes[3],
            (positions[5] - positions[4]) / magnitudes[4],
            [0, 0, 0],
        ])

        velocities[1] *= np.array([1, 1, 7])
        velocities[2] *= np.array([5, 5, 1])
        velocities[4] *= np.array([1, 1, 0.5])

        times = np.zeros(6)
        times[0] = 10
        times[1] = 15
        for i in range(2, 6):
            times[i] = times[i - 1] + magnitudes[i-1] / np.linalg.norm(velocities[i-1])

        fp1 = FlightPlan()
        fp1.set_waypoint(
            label="wp",
            time=times[0],
            pos=positions[0],
            vel=velocities[0],
            heading=[velocities[1][0], velocities[1][1]]
        )
        fp1.set_waypoint(
            label="wp_01",
            time=times[1],
            pos=positions[1],
            vel=velocities[1],
        )
        fp1.set_waypoint(
            label="wp_02",
            time=times[2],
            pos=positions[2],
            vel=velocities[2],
        )
        fp1.set_waypoint(
            label="wp_05",
            time=times[3],
            pos=positions[3],
            vel=velocities[3],
        )
        fp1.set_waypoint(
            label="wp_06",
            time=times[4],
            pos=positions[4],
            vel=velocities[4],
        )
        fp1.set_waypoint(
            label="wp_07",
            time=times[5],
            pos=positions[5],
            vel=velocities[5],
        )
        fp1.connect_waypoints()

        flightplan_as_lists.append(fp0.to_lists())
        flightplan_as_lists.append(fp1.to_lists())

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
            headings
        ]

        return uav_physics_indices, current_flightplans

    def on_physics_step(self, step_size: int):
        if self.is_simulation_running:
            # Get current time
            current_time = self.timeline.get_current_time()

            uav_physics_indices, current_flightplans = self.build_flightplans()

            # Update UAV control
            if uav_physics_indices:
                self.uav_control.update(
                    uav_physics_indices, 
                    current_flightplans, 
                    current_time, 
                    step_size
                )

    def on_timeline_stop(self, event):
        if self.is_simulation_running:
            self.rigid_prim = None

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

            # Set simulation as running
            self.is_simulation_running = True

    # ----------------------------------
    # -------- INITIALIZATION ----------
    # ----------------------------------
    def initialize_variables(self):
        # Control
        self.is_simulation_running = False
        self.uav_control = None
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
            

    # -----------------------------
    # -------- CALLBACKS ----------
    # ----------------------------- 
    def on_flightplan_request_event(self, event):
        # Here we would normally get the flight plan for the UAV that is being requested, but for this example we will just print the event payload
        print("Flight plan request event received with payload:", event.payload)

    def on_command_request_event(self, event):
        # Here we would normally get the command for the UAV that is being requested, but for this example we will just print the event payload
        print("Command request event received with payload:", event.payload)
