import asyncio

import omni.timeline
import omni.usd
from isaacsim.core.utils.prims import find_matching_prim_paths

from uspace.uav_operator.uav_operator import UAVOperator
from uspace.uav_operator.uav import UAV
from uspace.mission_manager.mission_manager import MissionManager
from uspace.vertiport_operator.vertiport_operator import VertiportOperator
from uspace.vertiport_operator.vertiport_pad import Pad
from uspace.uspace_manager.uspace_manager import USpaceManager
from uspace.uspace_manager.constants import MissionType, UAVStatus, PadStatus

class NavSimManager:
    def __init__(self, time_manager):
        # Runtime variables
        self.uav_ids_to_physics_buffer = {}

        # Control
        self.time_manager = time_manager
        self.is_simulation_running = False
        self.back_counter_time_max = 5
        self.back_counter_time = self.back_counter_time_max

        # Parameters
        self.mission_manager_amount = 1
        self.uav_operator_amount = 1
        self.vertiport_operator_amount = 10
        self.uspace_manager_amount = 1

        self.mission_managers = []
        self.uav_operators = []
        self.vertiport_operators = []
        self.uspace_managers = []

        self.mission_generation_task = None

        # IsaacSim parameters
        self.timeline = omni.timeline.get_timeline_interface()

    # ---------------------------
    # -- System Initialization --
    # ---------------------------
    def startup(self):
        self.scan_scene()
        self.connect_entities_to_mqtt()
        self.request_operators_list()

    def shutdown(self):
        self.disconnect_entities_from_mqtt()
        self.is_simulation_running = True
        self.stop_simulation()

    # ------------------------
    # -- Simulation Control --
    # ------------------------
    def start_simulation(self):
        # Recover from pause
        if self.is_simulation_running:
            self.mission_generation_task = asyncio.ensure_future(self.mission_generation_loop())
            return

        self.is_simulation_running = True
        self.mission_generation_task = asyncio.ensure_future(self.mission_generation_loop())
    
    def stop_simulation(self):
        if self.is_simulation_running:
            self.is_simulation_running = False
            self.back_counter_time = self.back_counter_time_max
            if self.mission_generation_task is not None:
                self.mission_generation_task.cancel()

            for mission_manager in self.mission_managers:
                mission_manager.last_mission_id = 0

            # for uav_operator in self.uav_operators:
            #     uav_operator.missions = {}

            for uspace_manager in self.uspace_managers:
                uspace_manager.airspace.clear_grid()
                # uspace_manager.missions = {}

            # for vertiport_operator in self.vertiport_operators:
            #     vertiport_operator.missions = {}


    def pause_simulation(self):
        self.mission_generation_task.cancel()

    async def mission_generation_loop(self):
        while self.is_simulation_running:
            print(f"Back counter time: {self.back_counter_time}")
            await asyncio.sleep(1)

            self.back_counter_time -= 1

            if self.back_counter_time == 0:
                self.request_missions()
                self.back_counter_time = self.back_counter_time_max

    # --------------------------
    # -- Simulation Functions --
    # --------------------------
    def connect_entities_to_mqtt(self):
        for mission_mgr in self.mission_managers:
            mission_mgr.connect_mqtt_client()
            
        for uspace_mgr in self.uspace_managers:
            uspace_mgr.connect_mqtt_client()
        
        for uav_op in self.uav_operators:
            uav_op.connect_mqtt_client()
            uav_op.register_into_airspace()

        for vert_op in self.vertiport_operators:
            vert_op.connect_mqtt_client()
            vert_op.register_into_airspace()

    def disconnect_entities_from_mqtt(self):
        for mission_mgr in self.mission_managers:
            mission_mgr.disconnect_mqtt_client()
            
        for uspace_mgr in self.uspace_managers:
            uspace_mgr.disconnect_mqtt_client()
        
        for uav_op in self.uav_operators:
            uav_op.disconnect_mqtt_client()

        for vert_op in self.vertiport_operators:
            vert_op.disconnect_mqtt_client()

    def request_operators_list(self):
        for mission_mgr in self.mission_managers:
            mission_mgr.request_uav_operator_list()
            mission_mgr.request_vertiport_operator_list()

    def request_missions(self):
        current_time = int(self.timeline.get_current_time())
        for mission_mgr in self.mission_managers:
            mission_mgr.request_uav_mission(current_time)

    def scan_mission_managers(self):
        self.mission_managers = [
            MissionManager(id=f"MISSION_MGR_{i}", name=f"Mission Manager {i}") 
            for i in range(self.mission_manager_amount)
        ]

    def scan_uav_operators(self, stage, prim_paths):
        # Build UAV Operators list
        for prim_path in prim_paths:
            # Get the prim for the UAV operator
            uav_operator_prim = stage.GetPrimAtPath(prim_path)
            
            # Extract attributes for the UAV operator
            uav_operator_id = uav_operator_prim.GetAttribute("NavSim:id").Get()
            uav_operator_name = uav_operator_prim.GetAttribute("NavSim:name").Get()
            uav_operator_service_types = uav_operator_prim.GetAttribute(
                "NavSim:service_types"
            ).Get()
            private_vertiport_operator_id = uav_operator_prim.GetAttribute(
                "NavSim:private_vertiport_operator_id"
            ).Get()

            # Initialize the UAV operator's UAVs dictionary
            uav_operator_uavs = {
                MissionType.DELIVERY: {},
                MissionType.PASSENGER_TRANSPORT: {}
            }

            # Build the UAVs for this UAV operator
            for uav_prim in uav_operator_prim.GetChildren()[0].GetChildren():
                # Get UAV attributes
                uav_id = uav_prim.GetAttribute("NavSim:id").Get()
                uav_type = uav_prim.GetAttribute("NavSim:type").Get()
                uav_status = uav_prim.GetAttribute("NavSim:status").Get()
                battery_level = uav_prim.GetAttribute("NavSim:battery_level").Get()
                operator_id = uav_prim.GetAttribute("NavSim:operator_id").Get()
                location = uav_prim.GetAttribute("xformOp:translate").Get()
                pad_id = uav_prim.GetAttribute("NavSim:pad_id").Get()

                # Store the UAV in the appropriate mission type category
                match uav_type:
                    case MissionType.DELIVERY:
                        uav_operator_uavs[MissionType.DELIVERY][uav_id] = UAV(
                            id=uav_id,
                            type=uav_type,
                            status=uav_status,
                            battery_level=float(battery_level),
                            operator_id=operator_id,
                            location=tuple(location),
                            pad_id=pad_id
                        )

                    case MissionType.PASSENGER_TRANSPORT:
                        uav_operator_uavs[MissionType.PASSENGER_TRANSPORT][uav_id] = UAV(
                            id=uav_id,
                            type=uav_type,
                            status=uav_status,
                            battery_level=float(battery_level),
                            operator_id=operator_id,
                            location=tuple(location),
                            pad_id=pad_id
                        )

                    case _:
                        raise ValueError(f"Unknown UAV type '{uav_type}' for UAV with ID '{uav_id}'")

            # Create the UAV operator object and add it to the list
            uav_operator = UAVOperator(
                id=uav_operator_id,
                name=uav_operator_name,
                service_types=list(uav_operator_service_types),
                private_vertiport_operator_id=private_vertiport_operator_id,
                uavs=uav_operator_uavs
            )

            uav_operator.time_manager = self.time_manager
            self.uav_operators.append(uav_operator)

    def scan_vertiport_operators(self, stage, prim_paths):
        # Build UAV Operators list
        for prim_path in prim_paths:
            # Get the prim for the vertiport operator
            vertiport_operator_prim = stage.GetPrimAtPath(prim_path)
            
            # Extract attributes for the vertiport operator
            vertiport_operator_id = vertiport_operator_prim.GetAttribute(
                "NavSim:id"
            ).Get()
            vertiport_operator_name = vertiport_operator_prim.GetAttribute(
                "NavSim:name"
            ).Get()
            vertiport_operator_service_types = vertiport_operator_prim.GetAttribute(
                "NavSim:service_types"
            ).Get()
            vertiport_is_private = vertiport_operator_prim.GetAttribute(
                "NavSim:is_private"
            ).Get()
            vertiport_location = vertiport_operator_prim.GetAttribute(
                "xformOp:translate"
            ).Get()

            # Initialize the vertiport operator's pads dictionary
            vertiport_operator_pads = {}

            # Build the pads for this vertiport operator
            for pad_prim in vertiport_operator_prim.GetChildren()[1].GetChildren():
                # Get pad attributes
                pad_id = pad_prim.GetAttribute("NavSim:id").Get()
                pad_type = pad_prim.GetAttribute("NavSim:type").Get()
                pad_status = pad_prim.GetAttribute("NavSim:status").Get()
                operator_id = pad_prim.GetAttribute("NavSim:operator_id").Get()
                location = pad_prim.GetAttribute("xformOp:translate").Get()

                # Identify the main pad and store it separately
                if pad_id == "MAIN_PAD":
                    main_pad = Pad(
                        id=pad_id,
                        type=pad_type,
                        status=pad_status,
                        operator_id=operator_id,
                        location=tuple(location + vertiport_location)
                    )
                    continue

                # Store the pad in the vertiport operator's pads dictionary
                vertiport_operator_pads[pad_id] = Pad(
                    id=pad_id,
                    type=pad_type,
                    status=pad_status,
                    operator_id=operator_id,
                    location=tuple(location + vertiport_location)
                )

            # Build grid connection information
            vertiport_grid_connection = {}

            i_grid = main_pad.location[1] // 100
            heading = [
                1 if i_grid % 2 == 0 else -1,
                0
            ]

            takeoff_position = [
                main_pad.location[0] + 50 * heading[0],
                main_pad.location[1],
                60  # TODO: Change this fixed values to adapt to grid requirements
            ]

            landing_position = [
                main_pad.location[0] - 50 * heading[0],
                main_pad.location[1],
                60  # TODO: Change this fixed values to adapt to grid requirements
            ]

            vertiport_grid_connection["takeoff"] = {
                "heading": heading,
                "position": takeoff_position
            }

            vertiport_grid_connection["landing"] = {
                "heading": heading,
                "position": landing_position
            }

            # Create the vertiport operator object and add it to the list
            vertiport_operator = VertiportOperator(
                id=vertiport_operator_id,
                name=vertiport_operator_name,
                service_types=list(vertiport_operator_service_types),
                is_private=vertiport_is_private,
                grid_connection=vertiport_grid_connection,
                main_pad=main_pad,
                pads=vertiport_operator_pads
            )

            self.vertiport_operators.append(vertiport_operator)

    def scan_uspace_managers(self):
        self.uspace_managers = [
            USpaceManager(id=f"USPACE_MGR_{i}", name=f"U-Space Manager {i}") 
            for i in range(self.uspace_manager_amount)
        ]

    def scan_scene(self):
        # Get the current stage
        stage = omni.usd.get_context().get_stage()

        # Find the prim paths for all vertiport operators and UAV operators in the stage
        uav_operator_prim_paths = find_matching_prim_paths("/*/*/UAV_OP_*")
        vertiport_operator_prim_paths = find_matching_prim_paths("/*/*/VERT_OP_*")

        # Clear existing lists
        self.mission_managers = []
        self.uav_operators = []
        self.vertiport_operators = []
        self.uspace_managers = []

        # Refill entity lists based on the current stage content
        self.scan_mission_managers()
        self.scan_uav_operators(stage, uav_operator_prim_paths)
        self.scan_vertiport_operators(stage, vertiport_operator_prim_paths)
        self.scan_uspace_managers()

        # Update control parameters based on the number of entities found
        self.mission_manager_amount = len(self.mission_managers)
        self.uav_operator_amount = len(self.uav_operators)
        self.vertiport_operator_amount = len(self.vertiport_operators)
        self.uspace_manager_amount = len(self.uspace_managers)

    def get_all_current_flitghplans(self, current_time):
        uav_physics_indices = []
        flightplan_as_lists = []

        # Get the current flightplan to be executed or being executed of all missions
        for uav_operator in self.uav_operators:
            for mission_manager_missions in uav_operator.missions.values():
                for mission in mission_manager_missions.values():
                    # Only consider missions that have been assigned to an UAV
                    assigned_uav_id = mission["assigned_uav_id"]
                    
                    if not assigned_uav_id:
                        continue

                    # Obtain the current flightplan to be or being executed
                    flightplan = None
                    
                    for fp in mission["flightplans"]:
                        if fp.init_time() - 5 <= current_time < fp.finish_time():
                            flightplan = fp
                            break

                    # Continue if no flightplan is being executed or about to be 
                    # executed in the next 5 seconds
                    if not flightplan:
                        continue

                    # Obtain the physics buffer index for the assigned UAV
                    uav_physics_indices.append(
                        self.uav_ids_to_physics_buffer[uav_operator.id][assigned_uav_id]
                    )

                    # Convert the flightplan to lists for vectorized processing
                    flightplan_as_lists.append(flightplan.to_lists())

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