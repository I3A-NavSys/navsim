from tabulate import tabulate
import json

from uspace.flight_plan.flight_plan import FlightPlan
from uspace.uspace_manager.constants import Topics, MissionStatus, UAVStatus
from uspace.mqtt.mqtt_service import MQTTService
from .uav import UAV


class UAVOperator:
    def __init__(self, id=None, name=None, private_vertiport_operator_id=None):
        self.id: str = id
        self.name: str = name
        self.private_vertiport_operator_id: str = private_vertiport_operator_id
        # Keep track of UAVs: {mission_type: {uav_id: UAV}}
        self.uavs: dict[str, dict[str, UAV]] = {}
        # Keep track of missions' processing status (used when requesting routes): 
        # {
        #   manager_id: {
        #     mission_id: [
        #       mission_type,
        #       stop_list, 
        #       stop_time, 
        #       current_destination_stop_index,
        #       last_destination_stop_index,
        #       uav_id
        #     ]
        #   }
        # }
        self.missions_processing_status: dict[str, dict[str, tuple[list, list, int]]] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.callback_topics = [
            f"{Topics.MISSION_UAV_SERVICE}/{self.id}",
            f"{Topics.REQUEST_ROUTE}/{self.id}"
        ]
        self.mqtt_client.message_callback_add(
            f"{Topics.MISSION_UAV_SERVICE}/{self.id}", 
            self.on_request_uav_service
        )
        self.mqtt_client.message_callback_add(
            f"{Topics.REQUEST_ROUTE}/{self.id}", 
            self.on_request_route_response
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
    def get_available_uavs(self, mission_type):
        if mission_type not in self.uavs:
            return []
        
        available_uavs = [
            uav for uav in self.uavs[mission_type].values()
            if uav.status == UAVStatus.AVAILABLE
        ]
        return available_uavs

    def free_resources(self):
        pass

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

    def cancel_mission(self, mission_manager_id, mission_id):
        # Logging
        print(f"[{self.id}] - Cancelling mission:")
        print(f"  Mission Manager ID: {mission_manager_id}")
        print(f"  Mission ID: {mission_id}")
        print()

        self.send_mission_status_update(
            mission_manager_id, 
            mission_id, 
            MissionStatus.CANCELLED
        )
        self.free_resources()

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
        stop_time
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

    # ----------------------
    # --- MQTT Callbacks ---
    # ----------------------
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
        if mission_type not in self.uavs:
            self.cancel_mission(mission_manager_id, mission_id)
            return
        
        # Return if there are no available UAVs (temporal)
        available_uavs = self.get_available_uavs(mission_type)
        if not available_uavs:
            self.cancel_mission(mission_manager_id, mission_id)
            return
        assigned_uav = available_uavs[0]
        
        # Initialize mission processing status
        if mission_manager_id not in self.missions_processing_status:
            self.missions_processing_status[mission_manager_id] = {}

        self.missions_processing_status[mission_manager_id][mission_id] = [
            mission_type,
            stop_list, 
            stop_time, 
            0,
            len(stop_list) - 1,
            assigned_uav.id
        ]
        
        # Ask for first route (from private vertiport to first stop)
        # assigned_uav.status = UAVStatus.OCCUPIED    # Reserve UAV
        self.request_route(
            uav_pad_id=assigned_uav.pad_id,
            mission_manager_id=mission_manager_id,
            mission_id=mission_id,
            mission_type=mission_type,
            origin_vertiport_id=self.private_vertiport_operator_id,
            destination_vertiport_id=stop_list[0],
            takeoff_time=None,   # None as we don't know when to start to arrive on time
            landing_time=landing_time,
            stop_time=stop_time[0],
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
        print("  Flightplan waypoints:")
        flightplan.print_waypoints()
        print()

        # Cancel mission if no route is found
        if not flightplan.waypoints:
            self.cancel_mission(mission_manager_id, mission_id)
            return
        
        # Check if it is the last leg of the mission
        mission = self.missions_processing_status[mission_manager_id][mission_id]
        mission_type = mission[0]
        stop_list = mission[1]
        stop_time = mission[2]
        current_stop = mission[3]
        last_stop = mission[4]

        if current_stop == last_stop:
            # Logging
            print(f"[{self.id}] - Mission {mission_id} completed all legs")
            print()

            # All legs completed, send mission status update to mission manager
            self.send_mission_status_update(
                mission_manager_id, 
                mission_id, 
                MissionStatus.IN_PROGRESS
            )
            # TODO: send flightplan to UAV for execution
            return

        # Logging
        print(f"[{self.id}] - Leg completed ({current_stop + 1} / {last_stop + 1}) for mission {mission_id}")
        print()

        # Ask for next route
        origin_vertiport_id = stop_list[current_stop]
        destination_vertiport_id = stop_list[current_stop + 1]
        destination_stop_time = stop_time[current_stop + 1]
        takeoff_time = flightplan.finish_time() + destination_stop_time

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
        )

        # Update mission processing status by incrementing current_destination_stop_index
        mission[3] += 1
        


