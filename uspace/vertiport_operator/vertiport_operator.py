import json

from uspace.uspace_manager.constants import Topics
from .vertiport_pad import VertiportPad
from uspace.mqtt.mqtt_service import MQTTService


class VertiportOperator:
    def __init__(self, id=None, name=None, grid_connection=None):
        self.id: str = id
        self.name: str = name
        self.grid_connection: tuple[float, float, float] = grid_connection
        self.pads: dict[str, VertiportPad] = {}

        # MQTT client
        self.callback_topics = [

        ]
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

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

    def disconnect_client(self):
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

    # ----------------------
    # --- USpace Methods ---
    # ----------------------
    def register_into_airspace(self):
        topic = Topics.VERTIPORT_OPERATOR_REGISTER.value
        msg = {
            "id": self.id,
            "name": self.name,
            "grid_connection": self.grid_connection
        }
        self.send_mqtt_msg(topic, json.dumps(msg))
        