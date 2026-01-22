import json

from uspace.uspace_manager.constants import Topics, MissionStatus, UAVStatus, MissionType
from uspace.mqtt.mqtt_service import MQTTService
from .uav import UAV


class UAVOperator:
    def __init__(self, id=None, name=None, private_vertiport_operator_id=None):
        self.id: str = id
        self.name: str = name
        self.private_vertiport_operator_id: str = private_vertiport_operator_id
        # Keep track of UAVs: {mission_type: {uav_id: UAV}}
        self.uavs: dict[MissionType, dict[str, UAV]] = {}
        # Keep track of missions' processing status (used when requesting routes): 
        # {manager_id: {mission_id: (stop_list, stop_time, current_destination_stop_index)}}
        self.missions_processing_status: dict[str, dict[str, tuple[list, list, int]]] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.callback_topics = [
            f"{Topics.MISSION_UAV_SERVICE.value}/{self.id}",
            f"{Topics.REQUEST_ROUTE.value}/{self.id}"
        ]
        self.mqtt_client.message_callback_add(
            f"{Topics.MISSION_UAV_SERVICE.value}/{self.id}", 
            self.on_request_uav_service
        )
        self.mqtt_client.message_callback_add(
            f"{Topics.REQUEST_ROUTE.value}/{self.id}", 
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

    # ----------------------
    # --- USpace Methods ---
    # ----------------------
    def register_into_airspace(self):
        topic = Topics.UAV_OPERATOR_REGISTER.value
        msg = {
            "id": self.id,
            "name": self.name
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def cancel_mission(self, mission_manager_id, mission_id):
        topic = f"{Topics.MISSION_STATUS_UPDATE.value}/{mission_manager_id}"
        msg = {
            "id": self.id,
            "mission_id": mission_id,
            "mission_status": MissionStatus.CANCELLED,
            # uav_id: None
            # eta: None
        }
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
        stop_time
    ):
        topic = Topics.REQUEST_ROUTE.value
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

        # Return if this UAV Operator does not support the requested mission type
        if mission_type not in self.uavs:
            self.cancel_mission(mission_manager_id, mission_id)
            return
        
        # Return if there are no available UAVs (temporal)
        available_uavs = self.get_available_uavs(mission_type)
        if not available_uavs:
            self.cancel_mission(mission_manager_id, mission_id)
            return
        
        # Initialize mission processing status
        self.missions_processing_status[mission_manager_id][mission_id] = (stop_list, stop_time, 0)
        
        # Ask for first route (from private vertiport to first stop)
        assigned_uav = available_uavs[0]
        assigned_uav.status = UAVStatus.OCCUPIED    # Reserve UAV
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
        route = data["route"]

        print(f"[UAV Operator] - Received route for mission {mission_id}:\n\t{route}")

        # Cancel mission if no route is found
        if not route:
            self.cancel_mission(mission_manager_id, mission_id)
            return
        
        

        


