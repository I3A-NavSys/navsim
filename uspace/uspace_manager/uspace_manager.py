import json
from typing import Any


from uspace.uav_operator.uav_operator import UAVOperator
from uspace.vertiport_operator.vertiport_operator import VertiportOperator
from uspace.grid_planner.grid_planner import GridPlanner
from uspace.mqtt.mqtt_service import MQTTService
from .constants import Topics


class USpaceManager:
    def __init__(self, id=None, name=None):
        self.id: str = id
        self.name: str = name
        self.airspace = GridPlanner()
        # Dictionary of uav operators: {id: name}
        self.uav_operators: dict[str, str] = {}
        # Dictionary of vertiport operators: {id: {name: V0, grid_conn: [1,1,1]}}
        self.vertiport_operators: dict[str, dict[str, Any]] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.callback_topics = [
            Topics.UAV_OPERATOR_REGISTER.value,
            Topics.VERTIPORT_OPERATOR_REGISTER.value,
            Topics.REQUEST_ROUTE.value,
            Topics.RECEIVE_ROUTE.value,
            Topics.REQUEST_UAV_OPERATOR_LIST.value,
            Topics.REQUEST_VERTIPORT_OPERATOR_LIST.value
        ]
        self.mqtt_client.message_callback_add(
            Topics.UAV_OPERATOR_REGISTER.value, 
            self.on_uav_operator_register
        )
        self.mqtt_client.message_callback_add(
            Topics.VERTIPORT_OPERATOR_REGISTER.value, 
            self.on_vertiport_operator_register
        )
        self.mqtt_client.message_callback_add(
            Topics.REQUEST_ROUTE.value,
            self.on_request_route
        )
        self.mqtt_client.message_callback_add(
            Topics.RECEIVE_ROUTE.value,
            self.on_receive_route
        )
        self.mqtt_client.message_callback_add(
            Topics.REQUEST_UAV_OPERATOR_LIST.value,
            self.on_request_uav_operator_list
        )
        self.mqtt_client.message_callback_add(
            Topics.REQUEST_VERTIPORT_OPERATOR_LIST.value,
            self.on_request_vertiport_operator_list
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

    # ----------------------
    # --- USpace Methods ---
    # ----------------------
    def request_vertiport_route(self, vertiport_operator_id, time):
        pass

    # ----------------------
    # --- MQTT Callbacks ---
    # ----------------------
    def on_uav_operator_register(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract operator data
        operator_id = data["id"]
        operator_name = data["name"]

        # Store UAV operator data
        self.uav_operators[operator_id] = operator_name
        print(f"[USpace Manager] - UAV Operator registered: {operator_id} - {operator_name}")

    def on_vertiport_operator_register(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract operator data
        operator_id = data["id"]
        operator_name = data["name"]
        grid_connection = data["grid_connection"]

        # Store vertiport operator data
        self.vertiport_operators[operator_id] = {}
        self.vertiport_operators[operator_id]["name"] = operator_name
        self.vertiport_operators[operator_id]["grid_connection"] = grid_connection
        print(f"[USpace Manager] - Vertiport Operator registered: {operator_id} - {operator_name}")

    def on_request_uav_operator_list(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract mission manager data
        manager_id = data["id"]

        # Send UAV Operator list
        topic = Topics.REQUEST_UAV_OPERATOR_LIST.value + f"/{manager_id}"
        msg = {
            "id": self.id,
            "uav_operators": self.uav_operators
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def on_request_vertiport_operator_list(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract mission manager data
        manager_id = data["id"]

        # Send Vertiport Operator list
        topic = Topics.REQUEST_VERTIPORT_OPERATOR_LIST.value + f"/{manager_id}"
        msg = {
            "id": self.id,
            "vertiport_operators": self.vertiport_operators
        }
        self.send_mqtt_msg(topic, json.dumps(msg))
        
    def on_request_route(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract route request data
        uav_operator_id = data["id"]
        mission_manager_id = data["mission_manager_id"]
        mission_id = data["mission_id"]
        origin_vertiport = data["origin_vertiport"]
        destination_vertiport = data["destination_vertiport"]
        takeoff_time = data["takeoff_time"]
        landing_time = data["landing_time"]
        stop_time = data["stop_time"]

        # Determine if the route computing is reversed
        is_reversed = landing_time != None

        # Get origin and destination grid connections
        origin = self.vertiport_operators[origin_vertiport]["grid_connection"]
        destination = self.vertiport_operators[destination_vertiport]["grid_connection"]

        # Compute the route
        if is_reversed:
            route = self.airspace.get_route(
                origin=destination, 
                destination=origin,
                start_time=landing_time,
                end_time=0,
                reverse=True
            )

        else:
            route = self.airspace.get_route(
                origin=origin, 
                destination=destination,
                start_time=takeoff_time,
                end_time=0,
                reverse=False
            )

        topic = f"{Topics.REQUEST_ROUTE.value}/{uav_operator_id}"
        msg = {
            "id": self.id,
            "mission_manager_id": mission_manager_id,
            "mission_id": mission_id,
            "route": route
        }
        self.send_mqtt_msg(topic, json.dumps(msg))
    
    def on_receive_route(self, client, userdata, msg):
        pass
