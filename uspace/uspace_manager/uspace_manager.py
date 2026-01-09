import json


from uspace.uav_operator.uav_operator import UAVOperator
from uspace.vertiport_operator.vertiport_operator import VertiportOperator
#from uspace.grid_planner.grid_planner import GridPlanner
from uspace.mqtt.mqtt_service import MQTTService
from .constants import Topics


class USpaceManager:
    def __init__(self, id=None, name=None):
        self.id: str = id
        self.name: str = name
        #self.uspace: GridPlanner
        self.uav_operators: dict[str, str] = {}
        self.vertiport_operators: dict[str, str] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.mqtt_client.message_callback_add(
            Topics.UAV_OPERATOR_REGISTER.value, 
            self.on_uav_operator_register
        )
        self.mqtt_client.message_callback_add(
            Topics.VERTIPORT_OPERATOR_REGISTER.value, 
            self.on_vertiport_operator_register
        )
        self.mqtt_client.message_callback_add(
            Topics.REQUEST_UAV_OPERATOR_LIST.value,
            self.on_request_uav_operator_list
        )
        self.mqtt_client.message_callback_add(
            Topics.REQUEST_VERTIPORT_OPERATOR_LIST.value,
            self.on_request_vertiport_operator_list
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

    # ----------------------
    # --- MQTT Callbacks ---
    # ----------------------
    def on_uav_operator_register(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.uav_operators[data["id"]] = data["name"]
        print(self.uav_operators)

    def on_vertiport_operator_register(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.vertiport_operators[data["id"]] = data["name"]
        print(self.vertiport_operators)

    def on_request_uav_operator_list(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        topic = Topics.REQUEST_UAV_OPERATOR_LIST.value + f"/{data["id"]}"
        msg = {
            "id": self.id,
            "uav_operators": self.uav_operators
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def on_request_vertiport_operator_list(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        topic = Topics.REQUEST_VERTIPORT_OPERATOR_LIST.value + f"/{data["id"]}"
        msg = {
            "id": self.id,
            "vertiport_operators": self.vertiport_operators
        }
        self.send_mqtt_msg(topic, json.dumps(msg))
        