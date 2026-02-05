import json

from uspace.flight_plan.flight_plan import FlightPlan
from uspace.uspace_manager.constants import Topics, PadStatus
from .vertiport_pad import Pad
from uspace.mqtt.mqtt_service import MQTTService


class VertiportOperator:
    def __init__(
        self, 
        id=None, 
        name=None, 
        grid_connection=None, 
        main_pad=None, 
        pads=None,
        security_pad_booking_buffer=40
    ):
        self.id: str = id
        self.name: str = name
        # Connection points in the grid for takeoff and landing
        # {
        #   "takeoff": {
        #     "heading": [x, y],
        #     "position": [x, y, z]
        #   }
        #   "landing": {
        #     "heading": [x, y],
        #     "position": [x, y, z]
        #   }
        # }
        self.grid_connection: dict[str, dict[str, tuple]] = grid_connection
        self.main_pad: Pad = main_pad
        self.pads: dict[str, Pad] = pads
        self.security_pad_booking_buffer: float = security_pad_booking_buffer  # in seconds

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
    def get_pads_by_status(self, status, pads=None):
        # If no pads specified, check all pads
        if pads is None:
            pads = self.pads.values()

        # Filter operative pads
        operative_pads = [
            pad for pad in pads
            if pad.status == status
        ]

        return operative_pads

    def get_pads_by_type(self, type, pads=None):
        # If no pads specified, check all pads
        if pads is None:
            pads = self.pads.values()

        # Filter pads by type
        type_pads = [
            pad for pad in pads
            if pad.type == type
        ]

        return type_pads

    def get_available_pads(self, start_time, end_time, pads=None):
        # If no pads specified, check all pads
        if pads is None:
            pads = self.pads.values()

        # Filter available pads
        available_pads = [
            pad for pad in pads
            if pad.is_available(start_time, end_time, self.security_pad_booking_buffer)
        ]

        return available_pads        

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
        
    def cancel_mission(
        self, 
        uav_operator_id, 
        mission_manager_id, 
        mission_id, 
        is_landing,
        is_reversed
    ):
        self.send_flightplan(
            uav_operator_id,
            mission_manager_id,
            mission_id,
            is_landing,
            is_reversed,
            FlightPlan(),
            None
        )

    def build_takeoff_flightplan(self, pad_id, time, is_reversed):
        # Initialize flightplan
        flightplan = FlightPlan()

        # Get parameters' information
        pad_pos = self.pads[pad_id].location
        takeoff_grid_pos = self.grid_connection["takeoff"]["position"]
        takeoff_grid_heading = self.grid_connection["takeoff"]["heading"]
        x_direction = takeoff_grid_heading[0]
        y_direction = takeoff_grid_heading[1]
        counter_pad_heading = [
            self.main_pad.location[0] - pad_pos[0],
            self.main_pad.location[1] - pad_pos[1]
        ]

        # Determine time offset based on is_reversed
        offset = 0
        if is_reversed:
            offset = 40

        # Set waypoints
        # Wait 5 seconds at assigned pad
        flightplan.set_waypoint(
            time=time - offset, 
            pos=pad_pos, 
            vel=[0, 0, 0], 
            heading=counter_pad_heading
        )
        # UAV pad -> main pad
        flightplan.set_waypoint(
            time=time + 5 - offset, 
            pos=pad_pos, 
            vel=[0, 0, 0], 
            heading=counter_pad_heading
        )
        # Wait 5 seconds at main pad to be properly oriented
        flightplan.set_waypoint(
            time=time + 15 - offset, 
            pos=self.main_pad.location, 
            vel=[0, 0, 0],
            heading=takeoff_grid_heading
        )
        # main pad -> grid connection point
        flightplan.set_waypoint(
            time=time + 20 - offset, 
            pos=self.main_pad.location, 
            vel=[0, 0, 0],
            heading=takeoff_grid_heading
        )
        # UAV in the grid
        flightplan.set_waypoint(
            time=time + 40 - offset, 
            pos=takeoff_grid_pos, 
            vel=[x_direction * 10, y_direction * 10, 0]
        )
        
        # Smooth waypoints
        flightplan.connect_waypoints()

        return flightplan

    def build_landing_flightplan(self, pad, time, is_reversed):
        # Initialize flightplan
        flightplan = FlightPlan()
        
        # Get parameters' information
        pad_pos = pad.location
        landing_grid_pos = self.grid_connection["landing"]["position"]
        landing_grid_heading = self.grid_connection["landing"]["heading"]
        x_direction = landing_grid_heading[0]
        y_direction = landing_grid_heading[1]
        counter_pad_heading = [
            self.main_pad.location[0] - pad_pos[0],
            self.main_pad.location[1] - pad_pos[1]
        ]
        
        # Determine time offset based on is_reversed
        offset = 0
        if is_reversed:
            offset = 40

        # Set waypoints
        # UAV in the grid -> main pad
        flightplan.set_waypoint(
            time=time - offset, 
            pos=landing_grid_pos, 
            vel=[x_direction * 5, y_direction * 5, 0],
            heading=landing_grid_heading
        )
        # main pad
        flightplan.set_waypoint(
            time=time + 20 - offset,
            pos=self.main_pad.location,
            vel=[0, 0, -1],
        )
        # wait 5 seconds at main pad to be properly oriented
        flightplan.set_waypoint(
            time=time + 25 - offset,
            pos=self.main_pad.location,
            vel=[0, 0, 0],
            heading=counter_pad_heading
        )
        # main pad -> assigned pad
        flightplan.set_waypoint(
            time=time + 35 - offset,
            pos=pad.location,
            vel=[0, 0, 0],
            heading=counter_pad_heading
        )
        # wait 5 seconds at assigned pad
        flightplan.set_waypoint(
            time=time + 40 - offset,
            pos=pad.location,
            vel=[0, 0, 0],
            heading=counter_pad_heading
        )

        # Smooth waypoints
        flightplan.connect_waypoints()

        return flightplan

    def send_flightplan(
        self,
        uav_operator_id,
        mission_manager_id,
        mission_id,
        is_landing,
        is_reversed,
        flightplan,
        pad_id
    ):
        if is_landing:
            topic = f"{Topics.RECEIVE_LANDING_FLIGHTPLAN}"
        else:
            topic = f"{Topics.RECEIVE_TAKEOFF_FLIGHTPLAN}"
            
        msg = {
            "id": self.id,
            "uav_operator_id": uav_operator_id,
            "mission_manager_id": mission_manager_id,
            "mission_id": mission_id,
            "is_landing": is_landing,
            "is_reversed": is_reversed,
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
        is_reversed = data["is_reversed"]
        mission_type = data["mission_type"]
        time = data["time"]
        stop_time = data["stop_time"]

        # Logging
        print(f"[{self.id}] - Received mission request:")
        print(f"  USpace Manager ID: {uspace_manager_id}")
        print(f"  UAV Operator ID: {uav_operator_id}")
        print(f"  Mission Manager ID: {mission_manager_id}")
        print(f"  Mission ID: {mission_id}")
        print(f"  Pad ID: {pad_id}")
        print(f"  Is Landing: {is_landing}")
        print(f"  Is Reversed: {is_reversed}")
        print(f"  Mission Type: {mission_type}")
        print(f"  Time: {time}")
        print(f"  Stop Time: {stop_time}")
        print()

        if is_landing:
            # Get operative and correct type pads
            operative_pads = self.get_pads_by_status(status=PadStatus.OPERATIVE)
            operative_correct_type_pads = self.get_pads_by_type(
                type=mission_type,
                pads=operative_pads
            )

            # Cancel mission if no operative pads of the correct type are available
            if not operative_correct_type_pads:
                self.cancel_mission(
                    uav_operator_id,
                    mission_manager_id,
                    mission_id,
                    is_landing,
                    is_reversed
                )
                return
            
            # Get available pads in the requested time window
            end_time = time + stop_time

            available_pads = self.get_available_pads(
                start_time=time,
                end_time=end_time,
                pads=operative_correct_type_pads
            )

            # Cancel mission if no pads are available
            if not available_pads:
                self.cancel_mission(
                    uav_operator_id,
                    mission_manager_id,
                    mission_id,
                    is_landing,
                    is_reversed
                )
                return

            # Assign the first available pad
            assigned_pad = available_pads[0]
            assigned_pad.book(
                start_time=time,
                end_time=end_time,
                buffer=self.security_pad_booking_buffer,
                availability_checked=True
            )
            pad_id = assigned_pad.id

            # Build landing flightplan
            flightplan = self.build_landing_flightplan(assigned_pad, time, is_reversed)
        else:
            # Build takeoff flightplan
            flightplan = self.build_takeoff_flightplan(pad_id, time, is_reversed)

        self.send_flightplan(
            uav_operator_id=uav_operator_id,
            mission_manager_id=mission_manager_id,
            mission_id=mission_id,
            is_landing=is_landing,
            is_reversed=is_reversed,
            flightplan=flightplan,
            pad_id=pad_id
        )