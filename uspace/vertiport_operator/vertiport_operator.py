import json
from typing import Any

from uspace.flight_plan.flight_plan import FlightPlan
from uspace.uspace_manager.constants import Topics, PadStatus, CancellationReason
from .vertiport_pad import Pad
from uspace.mqtt.mqtt_service import MQTTService


class VertiportOperator:
    def __init__(
        self, 
        id="", 
        name="", 
        service_types=None, 
        is_private=False,
        grid_connection=None, 
        main_pad=None, 
        pads=None,
        verbose=False
    ):
        self.verbose = verbose
        self.id: str = id
        self.name: str = name
        self.service_types: list[str] = service_types
        self.is_private = is_private
        self.takeoff_duration = 50
        self.landing_duration = 60
        self.main_pad: Pad = main_pad
        self.pads: dict[str, Pad] = pads
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
        # Keep track of missions, specially for cancellation purposes
        # {
        #   uav_operator_id: {
        #     mission_manager_id: {
        #       mission_id: {
        #         main_pad_bookings: [tuple[float, float]],
        #         pad_bookings: {
        #          pad_id: [tuple[float, float]],
        #         },
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
            f"{Topics.MISSION_VERTIPORT_SERVICE}/{self.id}",
            f"{Topics.CANCEL_MISSION}/{self.id}",
        ]

        self.mqtt_client.message_callback_add(
            f"{Topics.MISSION_VERTIPORT_SERVICE}/{self.id}",
            self.on_request_vertiport_service
        )

        self.mqtt_client.message_callback_add(
            f"{Topics.CANCEL_MISSION}/{self.id}",
            self.on_cancel_mission
        )

    # ----------------------
    # --- MQTT Methods -----
    # ----------------------
    def connect_mqtt_client(self, host, port):
        if not self.mqtt_is_connected:
            success = MQTTService.connect_client(self.mqtt_client, host, port)
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

        # Free main pad's time slots
        for start_time, end_time in mission["main_pad_bookings"]:
            self.main_pad.cancel_booking(start_time, end_time)

        # Free booked pads' time slots
        for pad_id, bookings in mission["pad_bookings"].items():
            for start_time, end_time in bookings:
                self.pads[pad_id].cancel_booking(start_time, end_time)

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
            if pad.is_available(start_time, end_time)
        ]

        return available_pads        

    def check_main_pad_availability(self, time, is_landing, is_reversed):
        if is_landing:
            offset_time = self.landing_duration
        else:
            offset_time = self.takeoff_duration

        start_time = time
        end_time = time + offset_time

        if is_reversed:
            start_time = time - offset_time
            end_time = time

        is_available = self.main_pad.book(
            start_time=start_time,
            end_time=end_time,
        )

        return is_available, start_time, end_time

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
        if self.verbose:
            print(f"[{self.id}] - Cancelling mission:")
            print(f"  Mission Manager ID: {mission_manager_id}")
            print(f"  Mission ID: {mission_id}")
            print(f"  Cancellation Reason: {cancellation_reason}")
            print()

        # Inform USpace manager about the cancellation
        topic = Topics.CANCEL_MISSION
        
        # Build cancellation message
        msg = {
            "uav_operator_id": uav_operator_id,
            "mission_manager_id": mission_manager_id,
            "mission_id": mission_id,
            "cancellation_reason": cancellation_reason
        }

        # Send cancellation message
        self.send_mqtt_msg(topic, json.dumps(msg))

        # Free resources and update cancellation reason
        self.free_resources(
            uav_operator_id, 
            mission_manager_id, 
            mission_id, 
            cancellation_reason
        )

    def register_into_airspace(self):
        topic = Topics.VERTIPORT_OPERATOR_REGISTER
        msg = {
            "id": self.id,
            "name": self.name,
            "service_types": self.service_types,
            "grid_connection": self.grid_connection,
            "is_private": self.is_private
        }
        self.send_mqtt_msg(topic, json.dumps(msg))
        
    def build_takeoff_flightplan(self, pad_id, time, is_reversed):
        # Initialize flightplan
        flightplan = FlightPlan()

        # Get parameters' information
        pad_pos = self.pads[pad_id].location
        takeoff_grid_pos = self.grid_connection["takeoff"]["position"]
        takeoff_grid_heading = self.grid_connection["takeoff"]["heading"]
        x_direction = takeoff_grid_heading[0]
        y_direction = takeoff_grid_heading[1]
        pad_to_main_pad_heading = [
            self.main_pad.location[0] - pad_pos[0],
            self.main_pad.location[1] - pad_pos[1]
        ]

        # Determine time offset based on is_reversed
        offset = 0
        if is_reversed:
            offset = self.takeoff_duration

        # Set waypoints
        flightplan.set_waypoint(
            label="PAD",
            time=time - offset, 
            pos=[pad_pos[0], pad_pos[1], pad_pos[2] + 1.75], 
            vel=[0, 0, 0], 
            heading=pad_to_main_pad_heading
        )

        flightplan.set_waypoint(
            label="PAD_TO_MAIN_PAD",
            time=time + 5 - offset, 
            pos=[pad_pos[0], pad_pos[1], pad_pos[2] + 3], 
            vel=[0,0,0],
            heading=pad_to_main_pad_heading
        )

        flightplan.set_waypoint(
            label="MAIN_PAD",
            time=time + 25 - offset, 
            pos=[
                self.main_pad.location[0], 
                self.main_pad.location[1], 
                self.main_pad.location[2] + 3
            ], 
            vel=[0, 0, 0],
            heading=takeoff_grid_heading
        )

        flightplan.set_waypoint(
            label="MAIN_PAD_TO_GRID",
            time=time + 30 - offset, 
            pos=[
                self.main_pad.location[0], 
                self.main_pad.location[1], 
                self.main_pad.location[2] + 3
            ], 
            vel=[0, 0, 1],
            heading=takeoff_grid_heading
        )

        flightplan.set_waypoint(
            label="GRID",
            time=time + 50 - offset, 
            pos=takeoff_grid_pos, 
            vel=[x_direction * 10, y_direction * 10, 0]
        )
        
        # Smooth waypoints
        flightplan.connect_waypoints()

        return flightplan

    def build_landing_flightplan(self, pad_id, time, is_reversed):
        # Initialize flightplan
        flightplan = FlightPlan()
        
        # Get parameters' information
        pad_pos = self.pads[pad_id].location
        landing_grid_pos = self.grid_connection["landing"]["position"]
        landing_grid_heading = self.grid_connection["landing"]["heading"]
        x_direction = landing_grid_heading[0]
        y_direction = landing_grid_heading[1]
        pad_to_main_pad_heading = [
            self.main_pad.location[0] - pad_pos[0],
            self.main_pad.location[1] - pad_pos[1]
        ]
        
        # Determine time offset based on is_reversed
        offset = 0
        if is_reversed:
            offset = self.landing_duration

        # Set waypoints
        flightplan.set_waypoint(
            label="GRID",
            time=time - offset, 
            pos=landing_grid_pos, 
            vel=[x_direction * 5.5, y_direction * 5.5, 0]
        )

        flightplan.set_waypoint(
            label="GRID_TO_MAIN_PAD_1",
            time=time + 20 - offset,
            pos=[
                self.main_pad.location[0], 
                self.main_pad.location[1], 
                self.main_pad.location[2] + 20
            ], 
            vel=[0, 0, -3]
        )

        flightplan.set_waypoint(
            label="GRID_TO_MAIN_PAD_2",
            time=time + 25 - offset,
            pos=[
                self.main_pad.location[0], 
                self.main_pad.location[1], 
                self.main_pad.location[2] + 10
            ], 
            vel=[0, 0, -0.2],
        )

        flightplan.set_waypoint(
            label="GRID_TO_MAIN_PAD_3",
            time=time + 30 - offset,
            pos=[
                self.main_pad.location[0], 
                self.main_pad.location[1], 
                self.main_pad.location[2] + 5
            ], 
            vel=[0, 0, 0],
            heading=pad_to_main_pad_heading
        )

        flightplan.set_waypoint(
            label="MAIN_PAD",
            time=time + 35 - offset,
            pos=[
                self.main_pad.location[0], 
                self.main_pad.location[1], 
                self.main_pad.location[2] + 3
            ], 
            vel=[0, 0, 0],
            heading=pad_to_main_pad_heading
        )

        flightplan.set_waypoint(
            label="MAIN_PAD_TO_PAD",
            time=time + 55 - offset,
            pos=[pad_pos[0], pad_pos[1], pad_pos[2] + 3], 
            vel=[0, 0, 0],
            heading=pad_to_main_pad_heading
        )

        flightplan.set_waypoint(
            label="PAD",
            time=time + 60 - offset,
            pos=[pad_pos[0], pad_pos[1], pad_pos[2] + 1.75], 
            vel=[0, 0, 0]
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
        if self.is_private:
            if is_landing:
                topic = f"{Topics.PRIVATE_VERTIPORT_LANDING}/{uav_operator_id}"
            else:
                topic = f"{Topics.PRIVATE_VERTIPORT_TAKEOFF}/{uav_operator_id}"

        else:
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
    def on_cancel_mission(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract cancellation data
        uav_operator_id = data.get("uav_operator_id", "")
        mission_manager_id = data.get("mission_manager_id", "")
        mission_id = data.get("mission_id", "")
        cancellation_reason = data.get("cancellation_reason", "")
        
        # Logging
        if self.verbose:
            print(f"[{self.id}] - Cancelling mission:")
            print(f"  Mission Manager ID: {mission_manager_id}")
            print(f"  Mission ID: {mission_id}")
            print(f"  Cancellation Reason: {cancellation_reason}")
            print()

        # Free resources and update cancellation reason
        self.free_resources(
            uav_operator_id, 
            mission_manager_id, 
            mission_id, 
            cancellation_reason
        )

    def on_request_vertiport_service(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())

        # Extract mission details
        uspace_manager_id = data.get("id", "")
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
        if self.verbose:
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

        # Initialize mission entry if it doesn't exist
        if uav_operator_id not in self.missions:
            self.missions[uav_operator_id] = {}

        if mission_manager_id not in self.missions[uav_operator_id]:
            self.missions[uav_operator_id][mission_manager_id] = {}

        if mission_id not in self.missions[uav_operator_id][mission_manager_id]:
            self.missions[uav_operator_id][mission_manager_id][mission_id] = {
                "main_pad_bookings": [],
                "pad_bookings": {},
                "cancellation_reason": None
            }

        # Book main pad for the mission duration as a security measure
        is_main_pad_available, main_pad_start_time, main_pad_end_time = (
            self.check_main_pad_availability(
                time=time, 
                is_landing=is_landing, 
                is_reversed=is_reversed
            )
        )

        # If main pad's airspace is not available, cancel the mission immediately
        if not is_main_pad_available:
            self.cancel_mission(
                uav_operator_id,
                mission_manager_id,
                mission_id,
                CancellationReason.MAIN_PAD_OCCUPIED
            )
            return
        
        # Store main pad booking for potential future cancellation
        mission = self.missions[uav_operator_id][mission_manager_id][mission_id]
        mission["main_pad_bookings"].append((main_pad_start_time, main_pad_end_time))

        # If it's a landing mission, check for available pads and book one.
        # If it's a takeoff mission, directly build the flightplan since the pad is 
        # already reserved for the UAV.
        if is_landing:
            # If the vertiport is private, a pad is always reserved for the UAV, 
            # so no need to check for availability
            if not self.is_private:
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
                        CancellationReason.NO_AVAILABLE_PAD
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
                        CancellationReason.NO_AVAILABLE_PAD
                    )
                    return

                # Assign the first available pad
                assigned_pad = available_pads[0]

                # Book the pad for the mission duration
                assigned_pad.book(
                    start_time=time,
                    end_time=end_time,
                    availability_checked=True,
                )

                # Get pad id for the response
                pad_id = assigned_pad.id

                # Store booking index for potential future cancellation
                mission = self.missions[uav_operator_id][mission_manager_id][mission_id]

                if pad_id not in mission["pad_bookings"]:
                    mission["pad_bookings"][pad_id] = []

                mission["pad_bookings"][pad_id].append((time, end_time))

            # Build landing flightplan
            flightplan = self.build_landing_flightplan(pad_id, time, is_reversed)
        else:
            # Build takeoff flightplan
            flightplan = self.build_takeoff_flightplan(pad_id, time, is_reversed)

        # Send back flightplan
        self.send_flightplan(
            uav_operator_id=uav_operator_id,
            mission_manager_id=mission_manager_id,
            mission_id=mission_id,
            is_landing=is_landing,
            is_reversed=is_reversed,
            flightplan=flightplan,
            pad_id=pad_id
        )