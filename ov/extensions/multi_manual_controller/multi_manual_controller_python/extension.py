import omni.ext
import omni.ui as ui
from .controller import Controller

class Multi_manual_controllerExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.create_vars()
        self.build_ui()

    def on_shutdown(self):
        self.stop()

    def create_vars(self):
        self.controller = Controller()

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