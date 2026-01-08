from uspace.uav_operator.uav_operator import UAVOperator
from uspace.vertiport_operator.vertiport_operator import VertiportOperator
from uspace.grid_planner.grid_planner import GridPlanner
from uspace.mqtt.mqtt_service import MQTTService


class USpaceManager:
    def __init__(self):
        self.id: str
        self.name:str
        self.uspace: GridPlanner
        self.uav_operators: dict[str, UAVOperator]
        self.vertiport_operators: dict[str, VertiportOperator]

        # MQTT client
        self.mqtt_client_id = "MQTT_USpaceManager"
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
        print(f"[USpaceManager] - Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    def subcribe_mqtt_topic(self, topic):
        if topic in self.mqtt_subscribed_topics:
            return
        
        self.mqtt_client.subscribe(topic)
        self.mqtt_subscribed_topics.add(topic)

    def send_mqtt_msg(self, msg, topic):
        self.mqtt_client.publish(topic, msg)
