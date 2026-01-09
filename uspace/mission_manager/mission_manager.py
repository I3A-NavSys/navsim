from uspace.mqtt.mqtt_service import MQTTService
from msgs.mission_manager_msgs import MissionMsg


class MissionManager:
    def __init__(self):
        self.id: str
        self.name:str
        self.missions = []
        self.msg: MissionMsg
        
        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
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

    def subscribe_mqtt_topic(self, topic):
        if topic in self.mqtt_subscribed_topics:
            return
        
        self.mqtt_client.subscribe(topic)
        self.mqtt_subscribed_topics.add(topic)

    def send_mqtt_msg(self, topic, msg):
        self.mqtt_client.publish(topic, msg)

    def request_vertiport_operator_list(self):
        pass

    def request_uav_operator_list(self):
        pass