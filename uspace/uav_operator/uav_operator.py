from tabulate import tabulate
import json
from typing import Any

from uspace.flight_plan.flight_plan import FlightPlan
from uspace.uspace_manager.constants import Topics, MissionStatus, UAVStatus, CancellationReason, MissionType
from uspace.mqtt.mqtt_service import MQTTService
from uspace.mission_manager.mission import Mission
from .uav import UAV


class UAVOperator:
    def __init__(self, id=None, name=None, private_vertiport_operator_id=None, uavs={}):
        self.id: str = id
        self.name: str = name
        self.private_vertiport_operator_id: str = private_vertiport_operator_id
        self.private_vertiport_operator_fp_time: int = 40
        # Keep track of UAVs: {mission_type: {uav_id: UAV}}
        self.uavs: dict[str, dict[str, UAV]] = uavs
        # Keep track of missions' processing status (used when requesting routes): 
        # {
        #   mission_manager_id: {
        #     mission_id: {
        #       mission_type: MissionType,
        #       stop_list: [str], 
        #       stop_time: [int], 
        #       current_destination_stop_index: int,
        #       last_destination_stop_index: int,
        #       uav_id: str
        #     }
        #   }
        # }
        self.missions_processing_status: dict[str, dict[str, tuple[list, list, int]]] = {}
        # Keep track of missions, specially for cancellation purposes:
        # {
        #   mission_manager_id: {
        #     mission_id: {
        #       mission: Mission,
        #       assigned_uav_id: uav_id,
        #       flightplans: [FlightPlan]
        #     }
        #   }
        # }
        self.missions: dict[str, dict[str, dict[str, Any]]] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.callback_topics = [
            f"{Topics.MISSION_UAV_SERVICE}/{self.id}",
            f"{Topics.REQUEST_ROUTE}/{self.id}",
            f"{Topics.CANCEL_MISSION}/{self.id}",
            Topics.PRIVATE_VERTIPORT_TAKEOFF,
            Topics.PRIVATE_VERTIPORT_LANDING
        ]

        self.mqtt_client.message_callback_add(
            f"{Topics.MISSION_UAV_SERVICE}/{self.id}", 
            self.on_request_uav_service
        )
        
        self.mqtt_client.message_callback_add(
            f"{Topics.REQUEST_ROUTE}/{self.id}", 
            self.on_request_route_response
        )

        self.mqtt_client.message_callback_add(
            f"{Topics.CANCEL_MISSION}/{self.id}",
            self.on_cancel_mission
        )

        self.mqtt_client.message_callback_add(
            Topics.PRIVATE_VERTIPORT_TAKEOFF,
            self.on_request_private_vertiport_takeoff_response
        )

        self.mqtt_client.message_callback_add(
            Topics.PRIVATE_VERTIPORT_LANDING,
            self.on_request_private_vertiport_landing_response
        )

    # ----------------------
    # --- MQTT Methods -----
    # ----------------------
    def connect_mqtt_client(self):
        if not self.mqtt_is_connected:
            success = MQTTService.connect_client(self.mqtt_client)
            if success:
                self.mqtt_is_connected = True
                
                for topic in self.callback_topics:
                    self.subscribe_mqtt_topic(topic)

    def disconnect_mqtt_client(self):
        if self.mqtt_is_connected:
            self.mqtt_is_connected = False
            MQTTService.disconnect_client(self.mqtt_client)
            self.mqtt_subscribed_topics.clear()

    def subscribe_mqtt_topic(self, topic):
        if topic in self.mqtt_subscribed_topics:
            return
        
        self.mqtt_client.subscribe(topic)
        self.mqtt_subscribed_topics.add(topic)

    def send_mqtt_msg(self, topic, msg):
        self.mqtt_client.publish(topic, msg)

    # -------------------------
    # --- Auxiliary Methods ---
    # -------------------------
    def get_available_uavs(self):
        supported_mission_types = self.uavs.keys()

        return {
            mission_type: [
                uav for uav in self.uavs[mission_type].values()
                if uav.status == UAVStatus.AVAILABLE
            ]
            for mission_type in supported_mission_types
        }

    def get_busy_uavs(self, mission_type):
        supported_mission_types = self.uavs.keys()
        
        return {
            mission_type: [
                uav for uav in self.uavs[mission_type].values()
                if uav.status == UAVStatus.BUSY
            ]
            for mission_type in supported_mission_types
        }

    def get_out_of_service_uavs(self):
        supported_mission_types = self.uavs.keys()
        
        return {
            mission_type: [
                uav for uav in self.uavs[mission_type].values()
                if uav.status == UAVStatus.OUT_OF_SERVICE
            ]
            for mission_type in supported_mission_types
        }

    def free_resources(
        self, 
        mission_manager_id, 
        mission_id, 
        mission_type, 
        cancellation_reason
    ):
        # Get mission entry
        mission_dict = self.missions[mission_manager_id][mission_id]
        
        # Get mission information
        mission = mission_dict["mission"]
        assigned_uav_id = mission_dict["assigned_uav_id"]

        # Update control variables' status
        mission.status = MissionStatus.CANCELLED
        mission.cancellation_reason = cancellation_reason

        # Free UAV booking if it was assigned
        if assigned_uav_id:
            # Get UAV
            uav = self.uavs[mission_type][assigned_uav_id]

            # Get start_time and end_time of the booking to cancel
            start_time = mission_dict["flightplans"][0].init_time()
            end_time = mission_dict["flightplans"][-1].finish_time()

            # Cancel UAV booking
            uav.cancel_booking(start_time, end_time)

        # Clear flightplans list as they are no longer relevant (memory optimization)
        mission_dict["flightplans"] = []

    def log_dict_table(self, data, headers):
        formatted_data = [
            [key, json.dumps(value, indent=2)] 
            for key, value in data.items()
        ]

        print(tabulate(formatted_data, headers=headers, tablefmt="grid"))
        print()

    # ----------------------
    # --- USpace Methods ---
    # ----------------------
    def register_into_airspace(self):
        topic = Topics.UAV_OPERATOR_REGISTER
        msg = {
            "id": self.id,
            "name": self.name
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def cancel_mission(self, mission_manager_id, mission_id, cancellation_reason):
        # Logging
        print(f"[{self.id}] - Cancelling mission:")
        print(f"  Mission Manager ID: {mission_manager_id}")
        print(f"  Mission ID: {mission_id}")
        print(f"  Cancellation Reason: {cancellation_reason}")
        print()

        # Select topics based on cancellation reason
        topics = [f"{Topics.CANCEL_MISSION}/{mission_manager_id}"]

        if cancellation_reason == CancellationReason.NO_AVAILABLE_UAV:
            topics.append(Topics.CANCEL_MISSION)
        
        # Build cancellation message
        msg = {
            "uav_operator_id": self.id,
            "mission_manager_id": mission_manager_id,
            "mission_id": mission_id,
            "cancellation_reason": cancellation_reason
        }
        
        # Send message to corresponding entities
        for topic in topics:
            self.send_mqtt_msg(topic, json.dumps(msg))

    def request_route(
        self, 
        uav_pad_id,
        mission_manager_id, 
        mission_id, 
        mission_type,
        origin_vertiport_id, 
        destination_vertiport_id, 
        takeoff_time, 
        landing_time, 
        stop_time,
        is_last_leg,
    ):
        topic = Topics.REQUEST_ROUTE
        msg = {
            "id": self.id,
            "uav_pad_id": uav_pad_id,
            "mission_manager_id": mission_manager_id,
            "mission_id": mission_id,
            "mission_type": mission_type,
            "origin_vertiport_id": origin_vertiport_id,
            "destination_vertiport_id": destination_vertiport_id,
            "takeoff_time": takeoff_time,
            "landing_time": landing_time,
            "stop_time": stop_time,
            "is_last_leg": is_last_leg
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def send_mission_status_update(self, missiom_manager_id, mission_id, mission_status):
        topic = f"{Topics.MISSION_STATUS_UPDATE}/{missiom_manager_id}"
        msg = {
            "id": self.id,
            "mission_id": mission_id,
            "mission_status": mission_status,
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def find_available_uav(self, mission_manager_id, mission_id):
        # Get corresponding mission entry
        mission_dict = self.missions[mission_manager_id][mission_id]

        # Get the times to start and end the takeoff and landing flightplans
        start_time = (
            mission_dict["flightplans"][0].init_time() - 
            self.private_vertiport_operator_fp_time
        )

        end_time = (
            mission_dict["flightplans"][-1].finish_time() + 
            self.private_vertiport_operator_fp_time
        )

        # Book first available UAV for the whole mission duration
        for uav in self.uavs[mission_dict["mission"].mission_type].values():
            if uav.book(start_time, end_time):
                # Store assigned UAV id in mission information
                mission_dict["assigned_uav_id"] = uav.id
                return True, uav.pad_id, start_time

        return False, None, None

    def request_private_vertiport_flightplan(
        self, 
        mission_manager_id, 
        mission_id,
        pad_id,
        is_landing,
        is_reversed,
        mission_type,
        time,
        stop_time
    ):
        topic = f"{Topics.MISSION_VERTIPORT_SERVICE}/{self.private_vertiport_operator_id}"
        msg = {
            "uav_operator_id": self.id,
            "mission_manager_id": mission_manager_id,
            "mission_id": mission_id,
            "pad_id": pad_id,
            "is_landing": is_landing,
            "is_reversed": is_reversed,
            "mission_type": mission_type,
            "time": time,
            "stop_time": stop_time
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    # ----------------------
    # --- MQTT Callbacks ---
    # ----------------------
    def on_cancel_mission(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract cancellation data
        mission_manager_id = data.get("mission_manager_id", "")
        mission_id = data.get("mission_id", "")
        mission_type = data.get("mission_type", "")
        cancellation_reason = data.get("cancellation_reason", "")

        # Inform entities involved in the mission for them to free resources
        self.cancel_mission(
            mission_manager_id, 
            mission_id, 
            cancellation_reason
        )

        # Free resources and update mission cancellation reason
        self.free_resources(
            mission_manager_id, 
            mission_id, 
            mission_type, 
            cancellation_reason
        )

    def on_request_uav_service(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract mission data
        mission_manager_id = data["id"]
        mission_id = data["mission_id"]
        mission_type = data["mission_type"]
        stop_list = data["stop_list"]
        stop_time = data["stop_times"]
        landing_time = data["landing_time"]

        # Logging
        print(f"[{self.id}] - Received request for mission:")
        print(f"  Mission ID: {mission_id}")
        print(f"  Mission Type: {mission_type}")
        print(f"  Stop List: {stop_list}")
        print(f"  Stop Times: {stop_time}")
        print(f"  Landing Time: {landing_time}")
        print()

        # Return if mission type is not supported
        if mission_type not in self.uavs or self.uavs[mission_type] == {}:
            self.cancel_mission(
                mission_manager_id, 
                mission_id, 
                CancellationReason.UNSUPPORTED_MISSION_TYPE
            )
            return
        
        # Initialize missions and missions processing status
        if mission_manager_id not in self.missions_processing_status:
            self.missions_processing_status[mission_manager_id] = {}
            self.missions[mission_manager_id] = {}

        # Build new stop_list with private vertiport included at the beginning and end
        stop_list = stop_list + [self.private_vertiport_operator_id]
        stop_time = stop_time + [float('inf')]

        # Store mission processing status information
        self.missions_processing_status[mission_manager_id][mission_id] = {
            "mission_type": mission_type,
            "stop_list": stop_list, 
            "stop_time": stop_time, 
            "current_destination_stop_index": 0,
            "last_destination_stop_index": len(stop_list) - 1,
            "uav_id": None
        }

        # Store mission information for cancellation purposes
        self.missions[mission_manager_id][mission_id] = {
            "mission": Mission(
                id=mission_id,
                mission_type=mission_type,
                stop_list=stop_list,
                stop_times=stop_time,
                uav_operator_id=self.id,
                landing_time=landing_time,
                status=MissionStatus.PENDING
            ),
            "assigned_uav_id": None,
            "flightplans": []
        }

        # Ask for first route (from private vertiport to first stop)
        self.request_route(
            uav_pad_id=None,    # None as we need to know the complete mission's fligtplans
            mission_manager_id=mission_manager_id,
            mission_id=mission_id,
            mission_type=mission_type,
            origin_vertiport_id=self.private_vertiport_operator_id,
            destination_vertiport_id=stop_list[0],
            takeoff_time=None,   # None as we don't know when to start to arrive on time
            landing_time=landing_time,
            stop_time=stop_time[0],
            is_last_leg=False,
        )
        
    def on_request_route_response(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract route response data
        uspace_manager_id = data["id"]
        mission_manager_id = data["mission_manager_id"]
        mission_id = data["mission_id"]
        raw_flightplan = data["flightplan"]
        landing_pad_id = data["landing_pad_id"]

        # Construct flightPlan object from raw data
        flightplan = FlightPlan()
        flightplan.from_dict(raw_flightplan)

        # Logging
        print(f"[{self.id}] - Received new leg flightplan:")
        print(f"  USpace Manager ID: {uspace_manager_id}")
        print(f"  Mission Manager ID: {mission_manager_id}")
        print(f"  Mission ID: {mission_id}")
        print(f"  Landing Pad ID: {landing_pad_id}")
        # print("  Flightplan waypoints:")
        # flightplan.print_waypoints()
        print()
        
        # Store flightplan
        self.missions[mission_manager_id][mission_id]["flightplans"].append(flightplan)

        # Check if it is the last leg of the mission
        mission = self.missions_processing_status[mission_manager_id][mission_id]
        mission_type = mission["mission_type"]
        stop_list = mission["stop_list"]
        stop_time = mission["stop_time"]
        current_stop = mission["current_destination_stop_index"]
        last_stop = mission["last_destination_stop_index"]

        if current_stop == last_stop:
            # Find the first available UAV
            is_any_available, pad_id, start_time = self.find_available_uav(mission_manager_id, mission_id)

            # Cancel mission if no UAV is available
            if not is_any_available:
                self.cancel_mission(
                    mission_manager_id, 
                    mission_id, 
                    CancellationReason.NO_AVAILABLE_UAV
                )
                return
            
            # Request private vertiport takeoff flightplan
            self.request_private_vertiport_flightplan(
                mission_manager_id=mission_manager_id,
                mission_id=mission_id,
                pad_id=pad_id,
                is_landing=False,
                is_reversed=False,
                mission_type=mission_type,
                time=start_time,
                stop_time=0
            )
            
            return

        # Logging
        print(("----------------------------------------------------------"))
        print(f"[{self.id}] - Leg completed ({current_stop + 1} / {last_stop + 1}) for mission {mission_id}")
        print(("----------------------------------------------------------"))
        print()

        # Ask for next route
        origin_vertiport_id = stop_list[current_stop]
        destination_vertiport_id = stop_list[current_stop + 1]
        destination_stop_time = stop_time[current_stop + 1]
        takeoff_time = flightplan.finish_time() + stop_time[current_stop]

        self.request_route(
            uav_pad_id=landing_pad_id,
            mission_manager_id=mission_manager_id,
            mission_id=mission_id,
            mission_type=mission_type,
            origin_vertiport_id=origin_vertiport_id,
            destination_vertiport_id=destination_vertiport_id,
            takeoff_time=takeoff_time,
            landing_time=None,  # None as we don't know when uav will arrive
            stop_time=destination_stop_time,
            is_last_leg=(destination_vertiport_id == self.private_vertiport_operator_id)
        )

        # Update mission processing status by incrementing current_destination_stop_index
        mission["current_destination_stop_index"] += 1
        
    def on_request_private_vertiport_takeoff_response(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract mission details
        mission_manager_id = data.get("mission_manager_id", "")
        mission_id = data.get("mission_id", "")
        raw_flightplan = data.get("flightplan", None)

        # Build flightplan object from raw data
        takeoff_flightplan = FlightPlan()
        takeoff_flightplan.from_dict(raw_flightplan)

        # Get mission entry
        mission_dict = self.missions[mission_manager_id][mission_id]

        # Get first airspace flightplan
        first_flightplan = mission_dict["flightplans"][0]
        
        # Remove first waypoint to avoid duplicates
        first_flightplan.waypoints.pop(0)
        
        # Build new flightplan
        new_flightplan = FlightPlan()
        new_flightplan.waypoints = takeoff_flightplan.waypoints + first_flightplan.waypoints

        # Smooth waypoints
        new_flightplan.connect_waypoints()

        # Store updated flightplan
        mission_dict["flightplans"][0] = new_flightplan

        # Get pad_id
        assigned_uav_id = mission_dict["assigned_uav_id"]
        mission_type = mission_dict["mission"].mission_type
        pad_id = self.uavs[mission_type][assigned_uav_id].pad_id

        # Request private vertiport landing flightplan
        self.request_private_vertiport_flightplan(
            mission_manager_id=mission_manager_id,
            mission_id=mission_id,
            pad_id=pad_id,
            is_landing=True,
            is_reversed=False,
            mission_type=mission_type,
            time=mission_dict["flightplans"][-1].finish_time(),
            stop_time=float('inf')
        )

    def on_request_private_vertiport_landing_response(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract mission details
        mission_manager_id = data.get("mission_manager_id", "")
        mission_id = data.get("mission_id", "")
        raw_flightplan = data.get("flightplan", None)

        # Build flightplan object from raw data
        landing_flightplan = FlightPlan()
        landing_flightplan.from_dict(raw_flightplan)

        # Get mission entry
        mission_dict = self.missions[mission_manager_id][mission_id]

        # Get last airspace flightplan
        last_flightplan = mission_dict["flightplans"][-1]
        
        # Remove last waypoint to avoid duplicates
        last_flightplan.waypoints.pop(-1)
        
        # Build new flightplan
        new_flightplan = FlightPlan()
        new_flightplan.waypoints =  last_flightplan.waypoints + landing_flightplan.waypoints

        # Smooth waypoints
        new_flightplan.connect_waypoints()

        # Store updated flightplan
        mission_dict["flightplans"][-1] = new_flightplan

        # Logging
        print(("----------------------------------------------------"))
        print(f"[{self.id}] - Mission {mission_id} completed all legs")
        print(("----------------------------------------------------"))
        print()
        for i, fp in enumerate(self.missions[mission_manager_id][mission_id]["flightplans"]):
            print(f"Leg {i+1}:")
            fp.print_waypoints()
        print()

        # All legs completed, send mission status update to mission manager
        self.send_mission_status_update(
            mission_manager_id, 
            mission_id, 
            MissionStatus.IN_PROGRESS
        )
        # TODO: send flightplan to UAV for execution