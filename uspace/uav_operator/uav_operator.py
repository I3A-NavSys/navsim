import json

from uspace.uspace_manager.constants import Topics, MissionStatus, UAVStatus, MissionType
from uspace.mqtt.mqtt_service import MQTTService
from .uav import UAV


class UAVOperator:
    def __init__(self, id=None, name=None):
        self.id: str = id
        self.name: str = name
        self.uavs: dict[MissionType, dict[str, UAV]] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.mqtt_client.message_callback_add(
            Topics.MISSION_UAV_SERVICE.value, 
            self.on_request_uav_service
        )


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

    # ----------------------
    # --- MQTT Callbacks ---
    # ----------------------
    def on_request_uav_service(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Will be obtained from data
        manager_id = None
        mission_id = None
        misstion_type = None

        # Return if this UAV Operator does not support the requested mission type
        if misstion_type not in self.uavs:
            self.cancel_mission(manager_id, mission_id)
            return
        
        available_uavs = self.get_available_uavs(misstion_type)
        # Return if there are no available UAVs
        if not available_uavs:
            self.cancel_mission(manager_id, mission_id)
            return


