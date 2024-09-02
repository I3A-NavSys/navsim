import omni.ext
import omni.ui as ui
import carb.events
import omni.kit.app
import omni.usd

class Stream_events_example_pythonExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._window = ui.Window("Stream event example", width=300, height=300)

        self.init_event_streams()
        self.build_ui()
        

    def init_event_streams(self):
        # Get the message bus event stream
        # This will let us subscribe to a specific type of event and then call a function whenever it detects that event 
        self.bus = omni.kit.app.get_app().get_message_bus_event_stream()

        # Create our own events
        # This must be a unique id, got from the string argument once it is hashed
        self.COMMON_EVENT = carb.events.type_from_string("omni.stream_event_example.COMMON_EVENT")
        self.CUBE_EVENT = carb.events.type_from_string("omni.stream_event_example.CUBE_EVENT")
        self.CONE_EVENT = carb.events.type_from_string("omni.stream_event_example.CONE_EVENT")

        # Varible to play with
        self.common_event_state = "DOWN"


    def build_ui(self):
        with self._window.frame:
            with ui.VStack():
                ui.Label("NOTE: Have a look at README file.")

                # The simulation must be running in order these buttons' callbacks to work
                ui.Button("COMMON EVENT", clicked_fn=self.common_event)
                ui.Button("CUBE EVENT", clicked_fn=self.cube_event)
                ui.Button("CONE EVENT", clicked_fn=self.cone_event)


    def common_event(self):
        # Alternate common event state
        match self.common_event_state:
            case "UP":
                self.common_event_state = "DOWN"

            case "DOWN":
                self.common_event_state = "UP"

        # Once we have everything setted up, we can push the common event with custom information in the payload
        self.bus.push(self.COMMON_EVENT, payload={"state": self.common_event_state})


    def cube_event(self):
        # Here we just push the cube event
        self.bus.push(self.CUBE_EVENT)


    def cone_event(self):
        # Here we just push the cone event
        self.bus.push(self.CONE_EVENT)


    def on_shutdown(self):
        pass
