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
        ]

        # UAV_0 FlightPlan
        wps = 6
        times = [0, 5, 40, 80, 140, 200]
        pos = np.array([
            [1042, 1028, 708],
            [1042, 1028, 708],
            [1098, 1305, 780],
            [1169, 1681, 780],
            [922, 2966, 810],
            [1112, 5347, 845]
        ])
        vel = np.array([
            np.array([0,0,0]),
            np.array([0,0,5]),
            (pos[2] - pos[1]) / np.linalg.norm(pos[2] - pos[1]) * 10,
            (pos[2] - pos[1]) / np.linalg.norm(pos[2] - pos[1]) * 20,
            (pos[4] - pos[3]) / np.linalg.norm(pos[4] - pos[3]) * 20,
            (pos[4] - pos[3]) / np.linalg.norm(pos[4] - pos[3]) * 20,
        ])
    
        fp = FlightPlan()
        for i in range(wps):
            fp.set_waypoint(
                time=times[i],
                pos=pos[i],
                vel=vel[i]
            )

        flightplan_as_lists.append(fp.to_lists())

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
