import asyncio

import omni.timeline

from uspace.uav_operator.uav_operator import UAVOperator
from uspace.uav_operator.uav import UAV
from uspace.mission_manager.mission_manager import MissionManager
from uspace.vertiport_operator.vertiport_operator import VertiportOperator
from uspace.vertiport_operator.vertiport_pad import Pad
from uspace.uspace_manager.uspace_manager import USpaceManager
from uspace.uspace_manager.constants import MissionType, UAVStatus, PadStatus

class NavSimManager:
    def __init__(self):
        # Control
        self.is_simulation_running = False
        self.back_counter_time_max = 10
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
        self.temporal_scan_scene()
        self.connect_entities_to_mqtt()
        self.request_operators_list()

    def shutdown(self):
        self.disconnect_entities_from_mqtt()
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
            self.mission_generation_task.cancel()

            for uspace_manager in self.uspace_managers:
                uspace_manager.airspace.clear_grid()

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

    def scan_scene(self):
        stage = omni.usd.get_context().get_stage()

        vertiport_prims = []
        vertiport_OPs = []
        vertiport_pads = {}

        # Obtenemos todos los Prims del stage y filtramos aquellos que sean de tipo "vertiport"
        for prim in stage.TraverseAll():
            if prim.GetAttribute("NavSim:type").Get() == "vertiport":
                vertiport_prims.append(prim)

        # Obtenemos el operador de vertiport asociado al pad actual
        for vertiport in vertiport_prims:
            vertiport_OP = vertiport.GetAttribute("NavSim:VertOP").Get()
            id = vertiport.GetAttribute("NavSim:id").Get()
            type = vertiport.GetAttribute("NavSim:type").Get()
            location = vertiport.GetAttribute("xformOp:translate").Get()
            pad = Pad(id=id, type=type, status=PadStatus.OPERATIVE, operator_id=vertiport_OP, location=location)
            
            # Si el operador de vertiport no está en el diccionario, lo añadimos con una lista vacía y luego añadimos el pad a la lista de pads de ese operador
            if vertiport_OP not in vertiport_pads:
                vertiport_pads[vertiport_OP] = []
            vertiport_pads[vertiport_OP].append(pad)

        # Ahora creamos un objeto de la clase VertiportOperator para cada operador de vertiport y le asignamos la lista de pads correspondiente
        for vertiport_OP, pads in vertiport_pads.items():
            # Buscamos el main pad
            main_pad = None
            pad_dict = {}
            for pad in pads:
                if pad.id == "MAIN_PAD":
                    main_pad = pad
                    break

            # Creamos el diccionario de pads sin el main pad
            for pad in pads:
                if pad.id == "MAIN_PAD":
                    continue
                # Combinamos el id del operador de vertiport con el id del pad para crear un id único para cada pad
                pad_dict[vertiport_OP + "_" + pad.id] = pad

            vertiport_operator = VertiportOperator(
                id = vertiport_OP,
                name = vertiport_OP,
                main_pad = main_pad,
                pads = pad_dict,
                security_pad_booking_buffer = 40
            )

            vertiport_OPs.append(vertiport_operator)

    def temporal_scan_scene(self):
        self.mission_managers = [
            MissionManager(id=f"MISSION_MGR_{i}", name=f"Mission Manager {i}") 
            for i in range(self.mission_manager_amount)
        ]
        self.uav_operators = [
            UAVOperator(
                id=f"UAV_OP_{i}", 
                name=f"UAV Operator {i}",
                private_vertiport_operator_id=f"VERT_OP_{i}",
                uavs={
                    MissionType.DELIVERY: {
                        f"UAV_{j}_DELIVERY": UAV(
                            id=f"UAV_{j}_DELIVERY",
                            operator_id=f"UAV_OP_{i}",
                            type=MissionType.DELIVERY,
                            status=UAVStatus.AVAILABLE,
                            battery_level=100.0,
                            location=(0.0, 0.0, 0.0),
                            pad_id=f"VERT_OP_{i}_PAD_{j}"
                        )
                        for j in range(3)
                    },
                    MissionType.PASSENGER_TRANSPORT: {
                        f"UAV_{j}_PASSENGER": UAV(
                            id=f"UAV_{j}_PASSENGER",
                            operator_id=f"UAV_OP_{i}",
                            type=MissionType.PASSENGER_TRANSPORT,
                            status=UAVStatus.AVAILABLE,
                            battery_level=100.0,
                            location=(0.0, 5.0, 0.0),
                            pad_id=f"VERT_OP_{i}_PAD_{j}",
                        )
                        for j in range(3)
                    }
                }
            ) 
            for i in range(self.uav_operator_amount)
        ]
        self.vertiport_operators = [
            VertiportOperator(
                id=f"VERT_OP_{i}",
                name=f"Vertiport Operator {i}",
                grid_connection={
                    "takeoff": {
                        "heading": [1 if i % 2 == 0 else -1, 0],
                        "position": [50 * (1 if i % 2 == 0 else -1) + i * 100, i * 100, 0]
                    },
                    "landing": {
                        "heading": [1 if i % 2 == 0 else -1, 0],
                        "position": [i * 100 - 50 * (1 if i % 2 == 0 else -1), i * 100, 0]
                    }
                },
                main_pad=Pad(
                    id=f"VERT_OP_{i}_MAIN_PAD",
                    type=MissionType.PASSENGER_TRANSPORT,
                    status=PadStatus.OPERATIVE,
                    operator_id=f"VERT_OP_{i}",
                    location=(i * 100, i * 100, 0)
                ),
                pads={
                    f"VERT_OP_{i}_PAD_{j}": Pad(
                        id=f"VERT_OP_{i}_PAD_{j}",
                        type=MissionType.DELIVERY if j % 2 == 0 else MissionType.PASSENGER_TRANSPORT,
                        status=PadStatus.OPERATIVE,
                        operator_id=f"VERT_OP_{i}",
                        location=(i * 100 + j * 10, i * 100 + j * 10, 0)
                    )
                    for j in range(3)
                },
                security_pad_booking_buffer=40
            )
            for i in range(self.vertiport_operator_amount)
        ]
        self.uspace_managers = [
            USpaceManager(id=f"USPACE_MGR_{i}", name=f"U-Space Manager {i}") 
            for i in range(self.uspace_manager_amount)
        ]

    