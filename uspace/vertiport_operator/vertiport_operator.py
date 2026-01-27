import json

from uspace.flight_plan.flight_plan import FlightPlan
from uspace.uspace_manager.constants import Topics
from .vertiport_pad import Pad
from uspace.mqtt.mqtt_service import MQTTService


class VertiportOperator:
    def __init__(self, id=None, name=None, grid_connection=None):
        self.id: str = id
        self.name: str = name
        self.grid_connection: dict[str, tuple[float, float, float]] = grid_connection
        self.pads: dict[str, Pad] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.callback_topics = [
            f"{Topics.MISSION_VERTIPORT_SERVICE}/{self.id}"
        ]
        self.mqtt_client.message_callback_add(
            f"{Topics.MISSION_VERTIPORT_SERVICE}/{self.id}",
            self.on_request_vertiport_service
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
        topic = Topics.VERTIPORT_OPERATOR_REGISTER
        msg = {
            "id": self.id,
            "name": self.name,
            "grid_connection": self.grid_connection
        }
        self.send_mqtt_msg(topic, json.dumps(msg))
        
    def build_takeoff_flightplan(self, pad_id, time):
        flightplan = FlightPlan()

        takeoff_grid_pos = self.grid_connection["takeoff"]
        start_pos = (takeoff_grid_pos[0], takeoff_grid_pos[1], 0.0)
        direction = 1 if (takeoff_grid_pos[1] // 100) % 2 == 0 else -1

        flightplan.set_waypoint(time=time - 20, pos=start_pos, vel=[0, 0, 2])
        flightplan.set_waypoint(time=time, pos=takeoff_grid_pos, vel=[direction * 10, 0, 0])
        flightplan.connect_waypoints()

        return flightplan

    def build_landing_flightplan(self, pad_id, time):
        flightplan = FlightPlan()
        
        landing_grid_pos = self.grid_connection["landing"]
        end_pos = (landing_grid_pos[0], landing_grid_pos[1], 0.0)
        
        flightplan.set_waypoint(time=time, pos=landing_grid_pos, vel=[0, 0, -2])
        flightplan.set_waypoint(time=time + 20, pos=end_pos, vel=[0, 0, 0])
        flightplan.connect_waypoints()

        return flightplan

    def send_flightplan(
        self,
        uav_operator_id,
        mission_manager_id,
        mission_id,
        is_landing,
        flightplan,
        pad_id
    ):
        topic = f"{Topics.RECEIVE_ROUTE}"
        msg = {
            "id": self.id,
            "uav_operator_id": uav_operator_id,
            "mission_manager_id": mission_manager_id,
            "mission_id": mission_id,
            "is_landing": is_landing,
            "flightplan": flightplan.to_dict(),
            "pad_id": pad_id
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    # ----------------------
    # --- MQTT Callbacks ---
    # ----------------------
    def on_request_vertiport_service(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract mission details
        uspace_manager_id = data["id"]
        uav_operator_id = data["uav_operator_id"]
        mission_manager_id = data["mission_manager_id"]
        mission_id = data["mission_id"]
        pad_id = data["pad_id"]
        is_landing = data["is_landing"]
        mission_type = data["mission_type"]
        time = data["time"]
        stop_time = data["stop_time"]

        # Logging
        print(f"[{self.id}] - Received request for mission:")
        print(f"  USpace Manager ID: {uspace_manager_id}")
        print(f"  UAV Operator ID: {uav_operator_id}")
        print(f"  Mission Manager ID: {mission_manager_id}")
        print(f"  Mission ID: {mission_id}")
        print(f"  Is Landing: {is_landing}")
        print(f"  Mission Type: {mission_type}")
        print(f"  Time: {time}")
        print()

        if is_landing:
            flightplan = self.build_landing_flightplan(pad_id, time)
        else:
            flightplan = self.build_takeoff_flightplan(pad_id, time)

        self.send_flightplan(
            uav_operator_id=uav_operator_id,
            mission_manager_id=mission_manager_id,
            mission_id=mission_id,
            is_landing=is_landing,
            flightplan=flightplan,
            pad_id=pad_id
        )