import json

from uspace.uspace_manager.constants import Topics
from .vertiport_pad import VertiportPad
from uspace.mqtt.mqtt_service import MQTTService


class VertiportOperator:
    def __init__(self, id=None, name=None):
        self.id: str = id
        self.name: str = name
        self.pads: dict[str, VertiportPad] = {}

        # MQTT client
        self.mqtt_client_id = "MQTT_Vertiport_Operator"
        self.mqtt_client = MQTTService.build_client(self.mqtt_client_id)
        self.mqtt_client.on_message = self.listen_mqtt
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()


    def connect_mqtt_client(self):
        if not self.mqtt_is_connected:
            success = MQTTService.connect_client(self.mqtt_client)
            if success:
                self.mqtt_is_connected = True

    def disconnect_client(self):
        if self.mqtt_is_connected:
            self.mqtt_is_connected = False
            MQTTService.disconnect_client(self.mqtt_client)

    def listen_mqtt(self, client, userdata, msg):
        print(f"[VertiportOperator] - Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    def subcribe_mqtt_topic(self, topic):
        if topic in self.mqtt_subscribed_topics:
            return
        
        self.mqtt_client.subscribe(topic)
        self.mqtt_subscribed_topics.add(topic)

    def send_mqtt_msg(self, topic, msg):
        self.mqtt_client.publish(topic, msg)

    def register_into_airspace(self):
        topic = Topics.VERTIPORT_REGISTER.value
        msg = {
            "id": self.id,
            "name": self.name
        }
        self.send_mqtt_msg(topic, json.dumps(msg))
        