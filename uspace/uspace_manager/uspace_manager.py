import json
from uspace.uav_operator.uav_operator import UAVOperator
from uspace.vertiport_operator.vertiport_operator import VertiportOperator
#from uspace.grid_planner.grid_planner import GridPlanner
from uspace.mqtt.mqtt_service import MQTTService


class USpaceManager:
    def __init__(self, id=None, name=None):
        self.id: str = id
        self.name: str = name
        #self.uspace: GridPlanner
        self.uav_operators: dict[str, str] = {}
        self.vertiport_operators: dict[str, str] = {}

        # MQTT client
        self.mqtt_client_id = "MQTT_USpaceManager"
        self.mqtt_client = MQTTService.build_client(self.mqtt_client_id)
        self.mqtt_client.on_message = self.listen_mqtt
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.mqtt_client.message_callback_add("airspace/operator/uav/register", self.on_uav_operator_register)
        self.mqtt_client.message_callback_add("airspace/operator/vertiport/register", self.on_vertiport_operator_register)


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

    def on_uav_operator_register(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.uav_operators[data["id"]] = data["name"]
        print(self.uav_operators)

    def on_vertiport_operator_register(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.vertiport_operators[data["id"]] = data["name"]
        print(self.vertiport_operators)