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
uav = stage.GetPrimAtPath("/UAM_aerotaxi")
# Get its associated event
uav_event = carb.events.type_from_string("NavSim." + str(uav.GetPath()))

# Create the FlightPlan
fp = FlightPlan()
# Build the waypoints
fp.set_waypoint(label="wp1", time=0, pos=[28,15,16])
fp.set_waypoint(label="wp2", time=5, pos=[28,15,16])
fp.set_waypoint(label="wp3", time=15, pos=[28,15,30])
fp.set_waypoint(label="wp4", time=20, pos=[28,15,30])
fp.set_waypoint(label="wp5", time=30, pos=[28,25,30])
fp.set_waypoint(label="wp6", time=40, pos=[18,25,30])
fp.set_waypoint(label="wp7", time=50, pos=[18,15,30])
fp.set_waypoint(label="wp8", time=60, pos=[28,15,30])
fp.set_waypoint(label="wp9", time=70, pos=[28,15,30])
fp.set_waypoint(label="wp10", time=75, pos=[28,15,16])
# Set linear uniform velocities
fp.set_uniform_velocity()
# Postpone 10 seconds the flightplan
fp.postpone(10)

# Serialize the flightplan
serialized_fp = base64.b64encode(pickle.dumps(fp)).decode("utf-8")

# Get the bus event stream
bus_event_stream = app.get_app_interface().get_message_bus_event_stream()
# Send the flightplan
bus_event_stream.push(uav_event, payload={"method": "eventFn_FlightPlan",
                                          "fp": serialized_fp})
