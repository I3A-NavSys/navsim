import carb
from omni.kit.scripting import BehaviorScript
import carb.events
import omni.kit.app
from pxr import Gf


class ConeBehavior(BehaviorScript):
    def on_play(self):
        # Get the message bus event stream
        # This will let us subscribe to a specific type of event and then call a function whenever it detects that event 
        self.bus = omni.kit.app.get_app().get_message_bus_event_stream()

        # Create our own events
        # This must be a unique id, got from the string argument once it is hashed
        COMMON_EVENT = carb.events.type_from_string("omni.stream_event_example.COMMON_EVENT")
        CONE_EVENT = carb.events.type_from_string("omni.stream_event_example.CONE_EVENT")

        # Then we subscribe to that specific type of event and indicate the callback to run
        # We also specify that we only want to know whenever this event is pushed
        # In order the subscription process to work, the subscription object created must be alive
        self.common_event_push_subs = self.bus.create_subscription_to_push_by_type(COMMON_EVENT, self.common_event_callback)
        self.cone_event_push_subs = self.bus.create_subscription_to_push_by_type(CONE_EVENT, self.cone_event_callback)


    def common_event_callback(self, event):
        # event has the property 'payload' which can store some information
        match event.payload["state"]:
            case "UP":
                self.prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(-2, 0, 1.5))

            case "DOWN":
                self.prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(-2, 0, 0.5))


    def cone_event_callback(self, event):
        # In this case, there is no need to use the event
        self.prim.GetAttribute("physics:angularVelocity").Set(Gf.Vec3f(0, 0, 50))


    def on_stop(self):
        # Once we want to end the subscription, we just have to set to None the subscription objects
        self.common_event_push_subs = None
        self.cone_event_push_subs = None


    def on_init(self):
        pass

    def on_pause(self):
        pass

    def on_update(self, current_time: float, delta_time: float):
        pass
