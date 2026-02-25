from tabulate import tabulate
import json
from typing import Any

from uspace.flight_plan.flight_plan import FlightPlan
from uspace.uav_operator.uav_operator import UAVOperator
from uspace.vertiport_operator.vertiport_operator import VertiportOperator
from uspace.grid_planner.grid_planner import GridPlanner
from uspace.mqtt.mqtt_service import MQTTService
from .constants import Topics, CancellationReason


class USpaceManager:
    def __init__(self, id=None, name=None):
        self.id: str = id
        self.name: str = name
        self.airspace = GridPlanner(max_route_length=1000)
        # Dictionary of uav operators: 
        # {
        #   id: {
        #       name: str,
        #       service_types: [str],
        #   }
        # }
        self.uav_operators: dict[str, dict[str, Any]] = {}
        # Dictionary of vertiport operators: 
        # {
        #   id: {
        #       name: V0, 
        #       grid_conn: [1,1,1], 
        #       is_private: False,
        #       service_types: [str],
        #   }
        # }
        self.vertiport_operators: dict[str, dict[str, Any]] = {}
        # Dictionary of missions' processing status (used when requesting routes):
        # {
        #   uav_operator_id: {
        #     manager_id: {
        #       mission_id: {
        #         takeoff_flightplan: FlightPlan, 
        #         landing_flightplan: FlightPlan, 
        #         grid_flightplan: FlightPlan,
        #         landing_pad_id: str
        #       }
        #     }
        #   }
        # }
        self.missions_processing_status: dict[str, dict[str, tuple[list, list, int]]] = {}
        # Keep track of missions, specially for cancellation purposes
        # {
        #   uav_operator_id: {
        #     mission_manager_id: {
        #       mission_id: {
        #         mission_type: MissionType,
        #         vertiport_operator_ids: set(str),
        #         routes: [grid_route]
        #         cancellation_reason: CancellationReason (None if not cancelled)
        #       }
        #     }
        #   }
        # }
        self.missions: dict[str, dict[str, dict[str, dict[str, Any]]]] = {}

        # MQTT client
        self.mqtt_client = MQTTService.build_client(self.id)
        self.mqtt_is_connected = False
        self.mqtt_subscribed_topics = set()

        # MQTT Callbacks
        self.callback_topics = [
            Topics.UAV_OPERATOR_REGISTER,
            Topics.VERTIPORT_OPERATOR_REGISTER,
            Topics.REQUEST_ROUTE,
            Topics.RECEIVE_TAKEOFF_FLIGHTPLAN,
            Topics.RECEIVE_LANDING_FLIGHTPLAN,
            Topics.REQUEST_UAV_OPERATOR_LIST,
            Topics.REQUEST_VERTIPORT_OPERATOR_LIST,
            Topics.CANCEL_MISSION,
        ]

        self.mqtt_client.message_callback_add(
            Topics.UAV_OPERATOR_REGISTER, 
            self.on_uav_operator_register
        )
        
        self.mqtt_client.message_callback_add(
            Topics.VERTIPORT_OPERATOR_REGISTER, 
            self.on_vertiport_operator_register
        )
        
        self.mqtt_client.message_callback_add(
            Topics.REQUEST_ROUTE,
            self.on_request_route
        )
        
        self.mqtt_client.message_callback_add(
            Topics.RECEIVE_TAKEOFF_FLIGHTPLAN,
            self.on_receive_takeoff_flightplan
        )
        
        self.mqtt_client.message_callback_add(
            Topics.RECEIVE_LANDING_FLIGHTPLAN,
            self.on_receive_landing_flightplan
        )
        
        self.mqtt_client.message_callback_add(
            Topics.REQUEST_UAV_OPERATOR_LIST,
            self.on_request_uav_operator_list
        )
        
        self.mqtt_client.message_callback_add(
            Topics.REQUEST_VERTIPORT_OPERATOR_LIST,
            self.on_request_vertiport_operator_list
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
    def free_resources(
        self, 
        uav_operator_id, 
        mission_manager_id, 
        mission_id, 
        cancellation_reason
    ):
        # Get mission entry
        mission = self.missions[uav_operator_id][mission_manager_id][mission_id]

        # Update cancellation reason
        mission["cancellation_reason"] = cancellation_reason

        # Clear vertiport operators ids involved in the mission for memory optimization
        mission["vertiport_operator_ids"] = []

        # Free reserved routes in the airspace
        for route in mission["routes"]:
            self.airspace.free_route(route)
        
    def track_mission_route(
        self, 
        vertiport_operator_id, 
        uav_operator_id, 
        mission_manager_id, 
        mission_id, 
        route
    ):
        # Store reserved route for cancellation purposes
        mission = self.missions[uav_operator_id][mission_manager_id][mission_id]
        mission["vertiport_operator_ids"].add(vertiport_operator_id)
        mission["routes"].append(route)

    def adjust_time_to_slot(self, time):
        return time + (-time % self.airspace.slot_time)

    # ----------------------
    # --- USpace Methods ---
    # ----------------------
    def cancel_mission(
        self, 
        uav_operator_id, 
        mission_manager_id, 
        mission_id,
        cancellation_reason
    ):
        # Logging
        print(f"[{self.id}] - Cancelling mission:")
        print(f"  Mission Manager ID: {mission_manager_id}")
        print(f"  Mission ID: {mission_id}")
        print(f"  Cancellation Reason: {cancellation_reason}")
        print()

        # Get mission entry
        mission = self.missions[uav_operator_id][mission_manager_id][mission_id]

        # Select topics based on cancellation reason
        topics = [
            f"{Topics.CANCEL_MISSION}/{vertiport_operator_id}"
            for vertiport_operator_id in mission["vertiport_operator_ids"]
        ]

        inform_uav_operator = (
            cancellation_reason == CancellationReason.NO_AVAILABLE_ROUTE or
            cancellation_reason == CancellationReason.NO_AVAILABLE_PAD
        )

        if inform_uav_operator:
            topics.append(f"{Topics.CANCEL_MISSION}/{uav_operator_id}")

        # Build cancellation message
        msg = {
            "uav_operator_id": uav_operator_id,
            "mission_manager_id": mission_manager_id,
            "mission_id": mission_id,
            "mission_type": mission["mission_type"],
            "cancellation_reason": cancellation_reason
        }

        # Send message to corresponding entities
        for topic in topics:
            self.send_mqtt_msg(topic, json.dumps(msg))

    def request_vertiport_flightplan(
        self, 
        uav_operator_id, 
        mission_manager_id, 
        mission_id, 
        vertiport_operator_id, 
        pad_id,
        is_landing, 
        is_reversed, 
        mission_type, 
        time, 
        stop_time
    ):
        topic = f"{Topics.MISSION_VERTIPORT_SERVICE}/{vertiport_operator_id}"
        msg = {
            "id": self.id,
            "uav_operator_id": uav_operator_id,
            "mission_manager_id": mission_manager_id,
            "mission_id": mission_id,
            "pad_id": pad_id,
            "is_landing": is_landing,
            "is_reversed": is_reversed,
            "mission_type": mission_type,
            "time": time,
            "stop_time": stop_time
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def check_and_send_complete_flightplan(
        self, 
        uav_operator_id, 
        mission_manager_id, 
        mission_id,
        takeoff_flightplan,
        landing_flightplan,
        grid_flightplan,
        landing_pad_id,
        is_reversed,
        is_last_leg
    ):
        # Check if all flightplans have been computed
        takeoff_fp_exists = takeoff_flightplan is not None
        landing_fp_exists = landing_flightplan is not None
        grid_fp_exists = grid_flightplan is not None
        
        # Return complete flightplan if all parts are ready
        if takeoff_fp_exists and landing_fp_exists and grid_fp_exists:
            # Remove first and last waypoint of grid flightplan to avoid duplicates
            if not is_reversed:
                grid_flightplan.waypoints.pop(0)

            if not is_last_leg:
                grid_flightplan.waypoints.pop(-1)

            # Build complete flightplan
            complete_flightplan = FlightPlan()
            complete_flightplan.waypoints = (
                takeoff_flightplan.waypoints + 
                grid_flightplan.waypoints + 
                landing_flightplan.waypoints
            )

            # Send complete flightplan to UAV Operator
            self.send_mission_flightplan(
                uav_operator_id,
                mission_manager_id,
                mission_id,
                complete_flightplan,
                landing_pad_id
            )

    def send_mission_flightplan(
        self, 
        uav_operator_id, 
        mission_manager_id, 
        mission_id, 
        flightplan,
        landing_pad_id
    ):
        topic = f"{Topics.REQUEST_ROUTE}/{uav_operator_id}"
        msg = {
            "id": self.id,
            "mission_manager_id": mission_manager_id,
            "mission_id": mission_id,
            "flightplan": flightplan.to_dict(),
            "landing_pad_id": landing_pad_id
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    # ----------------------
    # --- MQTT Callbacks ---
    # ----------------------
    def on_cancel_mission(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract cancellation data
        uav_operator_id = data.get("uav_operator_id", "")
        mission_manager_id = data.get("mission_manager_id", "")
        mission_id = data.get("mission_id", "")
        cancellation_reason = data.get("cancellation_reason", "")

        # Inform entities involved in the mission for them to free resources
        self.cancel_mission(
            uav_operator_id, 
            mission_manager_id, 
            mission_id,
            cancellation_reason
        )

        # Free resources and update cancellation reason
        self.free_resources(
            uav_operator_id, 
            mission_manager_id, 
            mission_id, 
            cancellation_reason
        )

    def on_uav_operator_register(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract operator data
        operator_id = data["id"]
        operator_name = data["name"]
        operator_service_types = data["service_types"]

        # Store UAV operator data
        self.uav_operators[operator_id] = {}
        self.uav_operators[operator_id]["name"] = operator_name
        self.uav_operators[operator_id]["service_types"] = operator_service_types

        # Logging
        print(f"[{self.id}] - UAV Operator registered:")
        print(tabulate([[operator_id, operator_name]], headers=["ID", "Name"]))
        print()

    def on_vertiport_operator_register(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract operator data
        operator_id = data["id"]
        operator_name = data["name"]
        operator_service_types = data["service_types"]
        grid_connection = data["grid_connection"]
        is_private = data["is_private"]

        # Store vertiport operator data
        self.vertiport_operators[operator_id] = {}
        self.vertiport_operators[operator_id]["name"] = operator_name
        self.vertiport_operators[operator_id]["service_types"] = operator_service_types
        self.vertiport_operators[operator_id]["grid_connection"] = grid_connection
        self.vertiport_operators[operator_id]["is_private"] = is_private

        # Logging
        print(f"[{self.id}] - Vertiport Operator registered:")
        print(tabulate([[operator_id, operator_name]], headers=["ID", "Name"]))
        print()

    def on_request_uav_operator_list(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract mission manager data
        mission_manager_id = data["id"]

        # Send UAV Operator list
        topic = f"{Topics.REQUEST_UAV_OPERATOR_LIST}/{mission_manager_id}"
        msg = {
            "id": self.id,
            "uav_operators": self.uav_operators
        }
        self.send_mqtt_msg(topic, json.dumps(msg))

    def on_request_vertiport_operator_list(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract mission manager data
        mission_manager_id = data["id"]

        # Get only not private vertiport operators for the mission manager
        vertiport_operators = {
            vertiport_operator_id: vertiport_operator_data
            for vertiport_operator_id, vertiport_operator_data in self.vertiport_operators.items()
            if not vertiport_operator_data["is_private"]
        }

        # Send Vertiport Operator list
        topic = f"{Topics.REQUEST_VERTIPORT_OPERATOR_LIST}/{mission_manager_id}"
        msg = {
            "id": self.id,
            "vertiport_operators": vertiport_operators
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
        is_last_leg = data["is_last_leg"]

        # Logging
        print(f"[{self.id}] - Request received:")
        print(f"  UAV Operator ID: {uav_operator_id}")
        print(f"  Mission ID: {mission_id}")
        print(f"  Origin Vertiport ID: {origin_vertiport_id}")
        print(f"  Destination Vertiport ID: {destination_vertiport_id}")
        print()

        # Initialize mission processing status and missions dictionaries
        if uav_operator_id not in self.missions_processing_status:
            self.missions_processing_status[uav_operator_id] = {}
            self.missions[uav_operator_id] = {}
        
        if mission_manager_id not in self.missions_processing_status[uav_operator_id]:
            self.missions_processing_status[uav_operator_id][mission_manager_id] = {}
            self.missions[uav_operator_id][mission_manager_id] = {}

        if mission_id not in self.missions[uav_operator_id][mission_manager_id]:
            self.missions[uav_operator_id][mission_manager_id][mission_id] = {
                "mission_type": mission_type,
                "vertiport_operator_ids": set(),
                "routes": [],
                "cancellation_reason": None
            }

        self.missions_processing_status[uav_operator_id][mission_manager_id][mission_id] = {
            "origin_vertiport_id": origin_vertiport_id,
            "destination_vertiport_id": destination_vertiport_id,
            "takeoff_pad_id": uav_pad_id,
            "mission_type": mission_type,
            "stop_time": stop_time,
            "is_last_leg": is_last_leg,
            "takeoff_flightplan": None,
            "landing_flightplan": None,
            "grid_flightplan": None,
            "landing_pad_id": None
        }

        # Determine if the route computing is reversed
        is_reversed = landing_time != None

        if is_reversed:
            # Ask first for landing flightplan
            self.request_vertiport_flightplan(
                uav_operator_id=uav_operator_id,
                mission_manager_id=mission_manager_id,
                mission_id=mission_id,
                vertiport_operator_id=destination_vertiport_id,
                pad_id=None,
                is_landing=True,
                is_reversed=True,
                mission_type=mission_type,
                time=self.adjust_time_to_slot(landing_time),
                stop_time=stop_time
            )

        else:
            # Ask first for takeoff flightplan
            self.request_vertiport_flightplan(
                uav_operator_id=uav_operator_id,
                mission_manager_id=mission_manager_id,
                mission_id=mission_id,
                vertiport_operator_id=origin_vertiport_id,
                pad_id=uav_pad_id,
                is_landing=False,
                is_reversed=False,
                mission_type=mission_type,
                time=self.adjust_time_to_slot(takeoff_time),
                stop_time=None
            )
    
    def on_receive_takeoff_flightplan(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract response data
        vertiport_operator_id = data["id"]
        uav_operator_id = data["uav_operator_id"]
        mission_manager_id = data["mission_manager_id"]
        mission_id = data["mission_id"]
        is_landing = data["is_landing"]
        is_reversed = data["is_reversed"]
        raw_flightplan = data["flightplan"]
        pad_id = data["pad_id"]

        # Build flightplan object from raw data
        flightplan = FlightPlan()
        flightplan.from_dict(raw_flightplan)

        # Logging
        print(f"[{self.id}] - Received takeofff flightplan:")
        print(f"  Vertiport Operator ID: {vertiport_operator_id}")
        print(f"  UAV Operator ID: {uav_operator_id}")
        print(f"  Mission Manager ID: {mission_manager_id}")
        print(f"  Mission ID: {mission_id}")
        print(f"  Is Landing: {is_landing}")
        print(f"  Is Reversed: {is_reversed}")
        print(f"  Pad ID: {pad_id}")
        # print("  Flightplan waypoints:")
        # flightplan.print_waypoints()
        print()
        
        # Store flightplan in mission processing status
        mission = self.missions_processing_status[uav_operator_id][mission_manager_id][mission_id]
        mission["takeoff_flightplan"] = flightplan

        # Request now the grid flightplan
        # Get origin and destination grid connections
        origin_vertiport_id = mission["origin_vertiport_id"]
        destination_vertiport_id = mission["destination_vertiport_id"]

        origin_grid_connection = self.vertiport_operators[origin_vertiport_id]["grid_connection"]
        destination_grid_connection = self.vertiport_operators[destination_vertiport_id]["grid_connection"]

        # Compute the route
        route = self.airspace.get_route(
            origin=origin_grid_connection["takeoff"]["position"], 
            destination=destination_grid_connection["landing"]["position"],
            start_time=flightplan.finish_time(),
            end_time=0,
            reverse=False
        )

        # Track reserved route for cancellation purposes
        self.track_mission_route(
            vertiport_operator_id,
            uav_operator_id, 
            mission_manager_id, 
            mission_id, 
            route
        )

        # Cancel mission if no route was found
        if not route:
            self.cancel_mission(
                uav_operator_id, 
                mission_manager_id, 
                mission_id,
                CancellationReason.NO_AVAILABLE_ROUTE
            )
            return
        
        # Reserve route in the airspace
        self.airspace.reserve_route(route)

        # Build grid flightplan from route
        grid_flightplan = self.airspace.get_flightplan_from_route(route)
    
        grid_flightplan_finish_time = grid_flightplan.finish_time()
        # TODO: Check begin time is not in the past

        # Logging
        print(f"[{self.id}] - Computed route:")
        print(f"  UAV Operator ID: {uav_operator_id}")
        print(f"  Mission ID: {mission_id}")
        print(f"  Origin Vertiport ID: {origin_vertiport_id}")
        print(f"  Destination Vertiport ID: {destination_vertiport_id}")
        # print("  Flightplan waypoints:")
        # grid_flightplan.print_waypoints()
        print()
        
        # Store grid flightplan in mission processing status
        mission["grid_flightplan"] = grid_flightplan

        # Check if landing flightplan is not needed (last leg of the mission), 
        # if so, send complete flightplan to UAV operator
        if mission["is_last_leg"]:
            # As it is the last leg of the mission, we know that the landing flightplan 
            # must be computed by the UAV operator, so set a blank landing flightplan 
            # to be able to send the complete flightplan to the UAV Operator
            mission["landing_flightplan"] = FlightPlan()

            self.check_and_send_complete_flightplan(
                uav_operator_id=uav_operator_id,
                mission_manager_id=mission_manager_id,
                mission_id=mission_id,
                takeoff_flightplan=mission["takeoff_flightplan"],
                landing_flightplan=mission["landing_flightplan"],
                grid_flightplan=mission["grid_flightplan"],
                landing_pad_id=mission["landing_pad_id"],
                is_reversed=False,
                is_last_leg=True
            )

        else:
            # Request landing flightplan 
            self.request_vertiport_flightplan(
                uav_operator_id=uav_operator_id,
                mission_manager_id=mission_manager_id,
                mission_id=mission_id,
                vertiport_operator_id=destination_vertiport_id,
                pad_id=None,
                is_landing=True,
                is_reversed=False,
                mission_type=mission["mission_type"],
                time=grid_flightplan_finish_time,
                stop_time=mission["stop_time"]
            )

    def on_receive_landing_flightplan(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract response data
        vertiport_operator_id = data["id"]
        uav_operator_id = data["uav_operator_id"]
        mission_manager_id = data["mission_manager_id"]
        mission_id = data["mission_id"]
        is_landing = data["is_landing"]
        is_reversed = data["is_reversed"]
        raw_flightplan = data["flightplan"]
        pad_id = data["pad_id"]

        # Build flightplan object from raw data
        flightplan = FlightPlan()
        flightplan.from_dict(raw_flightplan)

        # Logging
        print(f"[{self.id}] - Received landing flightplan:")
        print(f"  Vertiport Operator ID: {vertiport_operator_id}")
        print(f"  UAV Operator ID: {uav_operator_id}")
        print(f"  Mission Manager ID: {mission_manager_id}")
        print(f"  Mission ID: {mission_id}")
        print(f"  Is Landing: {is_landing}")
        print(f"  Is Reversed: {is_reversed}")
        print(f"  Pad ID: {pad_id}")
        # print("  Flightplan waypoints:")
        # flightplan.print_waypoints()
        print()

        # Store flightplan in mission processing status
        mission = self.missions_processing_status[uav_operator_id][mission_manager_id][mission_id]
        mission["landing_flightplan"] = flightplan
        mission["landing_pad_id"] = pad_id

        # If the route computing is reversed, request now the grid flightplan
        if is_reversed:
            # Get origin and destination grid connections
            origin_vertiport_id = mission["origin_vertiport_id"]
            destination_vertiport_id = mission["destination_vertiport_id"]

            origin_grid_connection = self.vertiport_operators[origin_vertiport_id]["grid_connection"]
            destination_grid_connection = self.vertiport_operators[destination_vertiport_id]["grid_connection"]

            # Compute the route
            route = self.airspace.get_route(
                origin=destination_grid_connection["landing"]["position"], 
                destination=origin_grid_connection["takeoff"]["position"],
                start_time=flightplan.init_time(),
                end_time=0,
                reverse=True
            )

            # Track reserved route for cancellation purposes
            self.track_mission_route(
                vertiport_operator_id,
                uav_operator_id, 
                mission_manager_id, 
                mission_id, 
                route
            )

            # Cancel mission if no route was found
            if not route:
                self.cancel_mission(
                    uav_operator_id, 
                    mission_manager_id, 
                    mission_id,
                    CancellationReason.NO_AVAILABLE_ROUTE
                )
                return
            
            # Reserve route in the airspace
            self.airspace.reserve_route(route)

            # Build grid flightplan from route
            grid_flightplan = self.airspace.get_flightplan_from_route(route)

            # Logging
            print(f"[{self.id}] - Computed route:")
            print(f"  UAV Operator ID: {uav_operator_id}")
            print(f"  Mission ID: {mission_id}")
            print(f"  Origin Vertiport ID: {origin_vertiport_id}")
            print(f"  Destination Vertiport ID: {destination_vertiport_id}")
            # print("  Flightplan waypoints:")
            # grid_flightplan.print_waypoints()
            print()
            
            # Store grid flightplan in mission processing status
            mission["grid_flightplan"] = grid_flightplan

            # As it is reversed, we know that the takeoff flightplan must be computed by
            # the UAV operator, so set a blank takeoff flightplan to be able to send the 
            # complete flightplan to the UAV Operator
            mission["takeoff_flightplan"] = FlightPlan()

        # If the route computing is not reversed, the grid and takeoff flightplans should 
        # have been already computed and stored, so we can check if all flightplans are 
        # ready to be sent to the UAV Operator
        self.check_and_send_complete_flightplan(
            uav_operator_id=uav_operator_id,
            mission_manager_id=mission_manager_id,
            mission_id=mission_id,
            takeoff_flightplan=mission["takeoff_flightplan"],
            landing_flightplan=mission["landing_flightplan"],
            grid_flightplan=mission["grid_flightplan"],
            landing_pad_id=mission["landing_pad_id"],
            is_reversed=is_reversed,
            is_last_leg=False
        )