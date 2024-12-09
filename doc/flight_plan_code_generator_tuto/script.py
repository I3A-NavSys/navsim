# Import necessary modules
from omni.isaac.core.utils.stage import get_current_stage
import carb.events
from uspace.flight_plan.flight_plan import FlightPlan
import pickle
import base64
import omni.kit.app as app

# Get the current stage
stage = get_current_stage()
# Get the recently added drone
uav = stage.GetPrimAtPath("/Fleet/UAV_01")
# Get its associated event
uav_event = carb.events.type_from_string("NavSim." + str(uav.GetPath()))

# Create the FlightPlan
fp = FlightPlan()
# Build the waypoints
fp.set_waypoint(label="wp0", time=0, pos=[1071.0, 1039.0, 216.1])
fp.set_waypoint(label="wp0_P", time=5, pos=[1071.0, 1039.0, 216.1])
fp.set_waypoint(label="wp1", time=20, pos=[1071.0, 1039.0, 230.0])
fp.set_waypoint(label="wp2", time=35, pos=[1110.0, 1039.0, 216.1])
fp.set_waypoint(label="wp3", time=50, pos=[1150.0, 1039.0, 125.0])
fp.set_waypoint(label="wp4", time=60, pos=[1200.0, 1039.0, 125.0])
fp.set_waypoint(label="wp5", time=70, pos=[1200.0, 990.0, 125.0])
fp.set_waypoint(label="wp6", time=80, pos=[1150.0, 950.0, 125.0])
fp.set_waypoint(label="wp7", time=125, pos=[1150.0, 720.0, 125.0])
fp.set_waypoint(label="wp8", time=150, pos=[1290.0, 720.0, 125.0])
fp.set_waypoint(label="wp9", time=180, pos=[1470.0, 600.0, 125])
fp.set_waypoint(label="wp10", time=200, pos=[1590.0, 520.0, 125.0])
fp.set_waypoint(label="wp11", time=220, pos=[1590.0, 520.0, 100])
fp.set_waypoint(label="wp12", time=240, pos=[1590.0, 520.0, 88.7])
# Set linear uniform velocities
fp.set_uniform_velocity()
# Smooth some corners of the flight plan
fp.smooth_waypoint_speed("wp1", 0.15)
fp.smooth_waypoint_speed("wp2", 0.2)
fp.smooth_waypoint_speed("wp3", 0.2)
fp.smooth_waypoint_speed("wp4", 0.15)
fp.smooth_waypoint_speed("wp5", 0.15)
fp.smooth_waypoint_speed("wp6", 0.12)
fp.smooth_waypoint_speed("wp7", 0.12)
fp.smooth_waypoint_speed("wp8", 0.05)
fp.smooth_waypoint_duration("wp9", 0.05, 0.2)
fp.smooth_waypoint_duration("wp10", 0.1, 0.2)
fp.smooth_waypoint_duration("wp11", 0.1, 0.2)

# Postpone 10 seconds the flightplan
fp.postpone(10)

# Serialize the flightplan
serialized_fp = base64.b64encode(pickle.dumps(fp)).decode("utf-8")

# Get the bus event stream
bus_event_stream = app.get_app_interface().get_message_bus_event_stream()
# Send the flightplan
bus_event_stream.push(uav_event, payload={"method": "eventFn_FlightPlan",
                                                "fp": serialized_fp})