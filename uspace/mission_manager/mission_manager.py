from tabulate import tabulate
import json
import random

from uspace.mqtt.mqtt_service import MQTTService
from uspace.uspace_manager.constants import Topics, MissionStatus, MissionType
from .mission import Mission


class MissionManager:
    def __init__(self, id=None, name=None):
        # Test
        random.seed(1)

        self.id: str = id
        self.name: str = name
        self.missions: dict[str, Mission] = {}
        self.last_mission_id: int = 0
        self.uav_operators: dict[str, str] = {}
        self.vertiport_operators: dict[str, str] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.callback_topics = [
            f"{Topics.REQUEST_VERTIPORT_OPERATOR_LIST}/{self.id}",
            f"{Topics.REQUEST_UAV_OPERATOR_LIST}/{self.id}",
            f"{Topics.MISSION_STATUS_UPDATE}/{self.id}",
            Topics.CANCEL_MISSION,
        ]

        self.mqtt_client.message_callback_add(
            f"{Topics.REQUEST_VERTIPORT_OPERATOR_LIST}/{self.id}",
            self.on_receive_vertiport_operator_list
        )

        self.mqtt_client.message_callback_add(
            f"{Topics.REQUEST_UAV_OPERATOR_LIST}/{self.id}",
            self.on_receive_uav_operator_list
        )

        self.mqtt_client.message_callback_add(
            f"{Topics.MISSION_STATUS_UPDATE}/{self.id}",
            self.on_receive_uav_mission_update
        )

        self.mqtt_client.message_callback_add(
            Topics.CANCEL_MISSION,
            self.on_cancel_mission
        )

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

    def disconnect_mqtt_client(self):
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

    # -------------------------
    # --- Auxiliary Methods ---
    # -------------------------
    def log_dict_table(self, data, headers):
        formatted_data = [
            [key, json.dumps(value, indent=2)] 
            for key, value in data.items()
        ]

        print(tabulate(formatted_data, headers=headers, tablefmt="grid"))
        print()

    # ----------------------
    # --- USpace Methods ---
    # ----------------------
    def request_vertiport_operator_list(self):
        topic = Topics.REQUEST_VERTIPORT_OPERATOR_LIST
        msg = {
            "id": self.id
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def request_uav_operator_list(self):
        topic = Topics.REQUEST_UAV_OPERATOR_LIST
        msg = {
            "id": self.id
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def request_uav_mission(self):
        self.last_mission_id += 1
        amount_stops = random.randint(2, 5)

        mission_id = f"MISSION_{self.last_mission_id}"
        mission_type = random.choice([MissionType.DELIVERY, MissionType.PASSENGER_TRANSPORT])
        stop_list = random.sample(list(self.vertiport_operators.keys()), amount_stops)
        stop_times = [random.randint(10, 60) for _ in range(amount_stops)]
        uav_operator_id = random.choice(list(self.uav_operators.keys()))
        landing_time = random.randint(100, 1000)

        self.missions[mission_id] = Mission(
            id=mission_id,
            mission_type=mission_type,
            stop_list=stop_list,
            stop_times=stop_times,
            uav_operator_id=uav_operator_id,
            landing_time=landing_time,
            status=MissionStatus.PENDING
        )

        # Logging
        print(f"[{self.id}] - Requesting new UAV mission:")
        print(f"  Mission ID: {mission_id}")
        print(f"  Mission Type: {mission_type}")
        print(f"  UAV Operator ID: {uav_operator_id}")
        print(f"  Stop List: {stop_list}")
        print(f"  Stop Times: {stop_times}")
        print(f"  Landing Time: {landing_time}")
        print()
        
        topic = f"{Topics.MISSION_UAV_SERVICE}/{uav_operator_id}"
        msg = {
            "id": self.id,
            "mission_id": mission_id,
            "mission_type": mission_type,
            "stop_list": stop_list,
            "stop_times": stop_times,
            "landing_time": landing_time,
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    # ----------------------
    # --- MQTT Callbacks ---
    # ----------------------
    def on_cancel_mission(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract cancellation data
        mission_manager_id = data.get("mission_manager_id", "")
        mission_id = data["mission_id"]
        cancellation_reason = data["cancellation_reason"]

        # Only process cancellation if it is for this mission manager
        if mission_manager_id != self.id:
            return
        
        # Logging
        print(f"[{self.id}] - Received mission cancellation:")
        print(f"  Mission ID: {mission_id}")
        print(f"  Cancellation Reason: {cancellation_reason}")
        print()

        # Update mission status and cancellation reason
        self.missions[mission_id].status = MissionStatus.CANCELLED
        self.missions[mission_id].cancellation_reason = cancellation_reason

    def on_receive_vertiport_operator_list(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.vertiport_operators = data["vertiport_operators"]

        # Logging
        print(f"[{self.id}] - Received Vertiport operator list:")
        self.log_dict_table(self.vertiport_operators, ["ID", "Data"])

    def on_receive_uav_operator_list(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.uav_operators = data["uav_operators"]

        # Logging
        print(f"[{self.id}] - Received UAV operator list:")
        self.log_dict_table(self.uav_operators, ["ID", "Data"])

    def on_receive_uav_mission_update(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        uav_operator_id = data["id"]
        mission_id = data["mission_id"]
        mission_status = data["mission_status"]

        # Update mission status
        mission = self.missions[mission_id]
        mission.status = mission_status

        # Logging
        print(f"[{self.id}] - Received mission status update:")
        print(f"  UAV Operator ID: {uav_operator_id}")
        print(f"  Mission ID: {mission_id}")
        print(f"  Mission Status: {mission_status}")
        print()
