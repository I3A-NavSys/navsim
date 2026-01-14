import json

from uspace.uspace_manager.constants import Topics, MissionStatus, UAVStatus, MissionType
from uspace.mqtt.mqtt_service import MQTTService
from .uav import UAV


class UAVOperator:
    def __init__(self, id=None, name=None, private_vertiport_operator_id=None):
        self.id: str = id
        self.name: str = name
        self.private_vertiport_operator_id: str = private_vertiport_operator_id
        self.uavs: dict[MissionType, dict[str, UAV]] = {}
        # Keep track of missions' processing status: 
        # {manager_id: {mission_id: (stop_list, stop_time, current_destination_stop_index)}}
        self.missions_processing_status: dict[str, dict[str, tuple[list, list, int]]] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.mqtt_client.message_callback_add(
            f"{Topics.MISSION_UAV_SERVICE.value}/{self.id}", 
            self.on_request_uav_service
        )

    # ----------------------
    # --- MQTT Methods -----
    # ----------------------
    def connect_mqtt_client(self):
        if not self.mqtt_is_connected:
            success = MQTTService.connect_client(self.mqtt_client)
            if success:
                self.mqtt_is_connected = True

    def disconnect_mqtt_client(self):
        if self.mqtt_is_connected:
            self.mqtt_is_connected = False
            MQTTService.disconnect_client(self.mqtt_client)

    def subscribe_mqtt_topic(self, topic):
        if topic in self.mqtt_subscribed_topics:
            return
        
        self.mqtt_client.subscribe(topic)
        self.mqtt_subscribed_topics.add(topic)

    def send_mqtt_msg(self, topic, msg):
        self.mqtt_client.publish(topic, msg)

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

    def cancel_mission(self, manager_id, mission_id):
        topic = Topics.MISSION_UAV_SERVICE.value + f"/{manager_id}"
        msg = {
            "id": self.id,
            "mission_id": mission_id,
            "mission_status": MissionStatus.CANCELLED,
            # uav_id: None
            # eta: None
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def get_available_uavs(self, mission_type):
        if mission_type not in self.uavs:
            return []
        
        available_uavs = [
            uav for uav in self.uavs[mission_type].values()
            if uav.status == UAVStatus.AVAILABLE
        ]
        return available_uavs
    
    def request_route(self, origin, destination, takeoff_time, landing_time, stop_time):
        topic = Topics.REQUEST_ROUTE.value
        msg = {
            "id": self.id,
            "origin_vertiport": origin,
            "destination_vertiport": destination,
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

        # Will be obtained from data
        manager_id = None
        mission_id = None
        mission_type = None
        stop_list = None
        stop_time = None
        landing_time = None

        # Return if this UAV Operator does not support the requested mission type
        if mission_type not in self.uavs:
            self.cancel_mission(manager_id, mission_id)
            return
        
        # Return if there are no available UAVs (temporal)
        available_uavs = self.get_available_uavs(mission_type)
        if not available_uavs:
            self.cancel_mission(manager_id, mission_id)
            return
        
        # Initialize mission processing status
        self.missions_processing_status[manager_id][mission_id] = (stop_list, stop_time, 0)
        
        # Ask for first route (from private vertiport to first stop)
        assigned_uav = available_uavs[0]
        assigned_uav.status = UAVStatus.OCCUPIED    # Reserve UAV
        self.request_route(
            origin=self.private_vertiport_operator_id,
            destination=stop_list[0],
            takeoff_time=None,   # None as we don't know when to start to arrive on time
            landing_time=landing_time,
            stop_time=stop_time[0]
        )




