# Import necessary modules
from omni.isaac.core.utils.stage import get_current_stage
import carb.events
from uspace.flight_plan.command import Command
import pickle
import base64
import omni.kit.app as app

# Get the current stage
stage = get_current_stage()
# Get the recently added drone
uav = stage.GetPrimAtPath("/UAM_minidrone")
# Get its associated event
uav_event = carb.events.type_from_string("NavSim." + str(uav.GetPath()))

# Build the command
command = Command(
    on = True,
    velX = 1,
    velY = 0,
    velZ = 0,
    rotZ = 1.57,
    duration = 4
)

# Serialize the command
serialized_cmd = base64.b64encode(pickle.dumps(command)).decode("utf-8")

# Get the bus event stream
bus_event_stream = app.get_app_interface().get_message_bus_event_stream()
# Send the command
bus_event_stream.push(uav_event, payload={"method": "eventFn_RemoteCommand",
                                          "command": serialized_cmd})
