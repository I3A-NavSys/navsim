from tabulate import tabulate
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
        # Dictionary of missions' processing status (used when requesting routes):
        # {uav_operator_id: {
        #   manager_id: {
        #       mission_id: {
        #           "takeoff_flightplan": FP, 
        #           "landing_flightplan": FP, 
        #           "grid_flightplan": FP}
        #       }
        #   }
        # }
        self.missions_processing_status: dict[str, dict[str, tuple[list, list, int]]] = {}

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
    def request_vertiport_flightplan(
        self, 
        vertiport_operator_id, 
        pad_id,
        is_landing, 
        mission_type, 
        time, 
        stop_time
    ):
        topic = f"{Topics.MISSION_VERTIPORT_SERVICE.value}/{vertiport_operator_id}"
        msg = {
            "id": self.id,
            "pad_id": pad_id,
            "is_landing": is_landing,
            "mission_type": mission_type,
            "time": time,
            "stop_time": stop_time
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

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

        # Logging
        print("[USpace Manager] - UAV Operator registered:")
        print(tabulate([[operator_id, operator_name]], headers=["ID", "Name"]))
        print()

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

        # Logging
        print("[USpace Manager] - Vertiport Operator registered:")
        print(tabulate([[operator_id, operator_name]], headers=["ID", "Name"], showindex=True))
        print()

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
        uav_pad_id = data["uav_pad_id"]
        mission_manager_id = data["mission_manager_id"]
        mission_id = data["mission_id"]
        mission_type = data["mission_type"]
        origin_vertiport_id = data["origin_vertiport_id"]
        destination_vertiport_id = data["destination_vertiport_id"]
        takeoff_time = data["takeoff_time"]
        landing_time = data["landing_time"]
        stop_time = data["stop_time"]

        # Determine if the route computing is reversed
        is_reversed = landing_time != None

        # Get origin and destination grid connections
        origin_grid_connection = self.vertiport_operators[origin_vertiport_id]["grid_connection"]
        destination_grid_connection = self.vertiport_operators[destination_vertiport_id]["grid_connection"]

        # Compute the route
        if is_reversed:
            route = self.airspace.get_route(
                origin=destination_grid_connection["landing"], 
                destination=origin_grid_connection["takeoff"],
                start_time=landing_time,
                end_time=0,
                reverse=True
            )

        else:
            route = self.airspace.get_route(
                origin=origin_grid_connection["takeoff"], 
                destination=destination_grid_connection["landing"],
                start_time=takeoff_time,
                end_time=0,
                reverse=False
            )

        # Cancel mission if no route was found
        if not route:
            topic = f"{Topics.REQUEST_ROUTE.value}/{uav_operator_id}"
            msg = {
                "id": self.id,
                "mission_manager_id": mission_manager_id,
                "mission_id": mission_id,
                "flightplan": None
            }
            self.send_mqtt_msg(topic, json.dumps(msg))
            return

        grid_flightplan = self.airspace.get_flightplan_from_route(route)
        
        grid_flightplan_init_time = grid_flightplan.init_time()
        grid_flightplan_finish_time = grid_flightplan.finish_time()
        #TODO: Check begin time is in the past

        # Store mission processing status
        self.missions_processing_status[uav_operator_id] = {}
        self.missions_processing_status[uav_operator_id][mission_manager_id] = {}
        self.missions_processing_status[uav_operator_id][mission_manager_id][mission_id] = {
            "takeoff_flightplan": None,
            "landing_flightplan": None,
            "grid_flightplan": grid_flightplan
        }

        # Request takeoff flightplan 
        self.request_vertiport_flightplan(
            vertiport_operator_id=origin_vertiport_id,
            pad_id=uav_pad_id,
            is_landing=False,
            mission_type=mission_type,
            time=grid_flightplan_init_time,
            stop_time=None
        )

        # Request landing flightplan
        self.request_vertiport_flightplan(
            vertiport_operator_id=destination_vertiport_id,
            pad_id=None,
            is_landing=True,
            mission_type=mission_type,
            time=grid_flightplan_finish_time,
            stop_time=stop_time
        )
    
    def on_receive_route(self, client, userdata, msg):
        pass
