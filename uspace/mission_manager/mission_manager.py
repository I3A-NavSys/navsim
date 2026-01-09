import json
from uspace.mqtt.mqtt_service import MQTTService
from msgs.mission_manager_msgs import MissionMsg
from uspace.uspace_manager.constants import Topics


class MissionManager:
    def __init__(self, id=None, name=None):
        self.id: str = id
        self.name: str = name
        self.missions = []
        self.msg: MissionMsg
        self.uav_operators: dict[str, str] = {}
        self.vertiport_operators: dict[str, str] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.mqtt_client.message_callback_add(
            Topics.REQUEST_VERTIPORT_OPERATOR_LIST.value,
            self.on_receive_vertiport_operator_list
        )
        self.mqtt_client.message_callback_add(
            Topics.REQUEST_UAV_OPERATOR_LIST.value,
            self.on_receive_uav_operator_list
        )


    def connect_mqtt_client(self):
        if not self.mqtt_is_connected:
            success = MQTTService.connect_client(self.mqtt_client)
            if success:
                self.mqtt_is_connected = True

    def disconnect_client(self):
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

    def request_vertiport_operator_list(self):
        topic = Topics.REQUEST_VERTIPORT_OPERATOR_LIST.value
        msg = {
            "id": self.id
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def request_uav_operator_list(self):
        topic = Topics.REQUEST_UAV_OPERATOR_LIST.value
        msg = {
            "id": self.id
        }
        self.send_mqtt_msg(topic, msg)

    # ----------------------
    # --- MQTT Callbacks ---
    # ----------------------
    def on_receive_vertiport_operator_list(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.vertiport_operators = data["vertiport_operators"]

    def on_receive_uav_operator_list(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.uav_operators = data["uav_operators"]