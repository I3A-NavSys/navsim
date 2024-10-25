import omni.ext
import omni.ui as ui
import omni.physx
import omni.timeline


from .controller import Controller

class Multi_manual_controllerExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.create_vars()
        self.build_ui()

    def on_shutdown(self):
        self.stop()

    def create_vars(self):
        self.controller = Controller()

        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline_stop_event_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop)

    def build_ui(self):
        self.window = ui.Window("NavSim - Multi Manual Controller", width=600, height=600)
        
        with self.window.frame:
            with ui.VStack(spacing=10):
                ui.Button(text="START", clicked_fn=self.start)
                ui.Button(text="STOP", clicked_fn=self.stop)

    def start(self):
        self.controller.start()

    def stop(self):
        self.controller.stop()

    def on_timeline_stop(self, event):
        self.controller.stop()