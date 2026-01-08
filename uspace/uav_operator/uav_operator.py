from uspace.mqtt.mqtt_service import MQTTService
from .uav import UAV


class UAVOperator:
    def __init__(self, id=None, name=None):
        self.id: str = id
        self.name: str = name
        self.uavs = dict[str, UAV]

        # MQTT client
        self.mqtt_client_id = "MQTT_UAV_Operator"
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
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    def subcribe_mqtt_topic(self, topic):
        if topic in self.mqtt_subscribed_topics:
            return
        
        self.mqtt_client.subscribe(topic)
        self.mqtt_subscribed_topics.add(topic)

    def send_msg(self, msg, topic):
        self.mqtt_client.publish(topic, msg)