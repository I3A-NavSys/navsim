import omni.ext
import omni.ui as ui
import omni.physx
import omni.timeline
import omni.kit.viewport.window


from .controller import Controller

class MultiManualController(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.create_vars()
        self.build_ui()

    def on_shutdown(self):
        self.stop()

    def create_vars(self):
        self.controller = Controller()
        self.max_joysticks = 4

        self.perspective_camera_path = "/OmniverseKit_Persp"
        self.follow_UAV_camera_path = "/manual_controller_CAM"
        self.camera_1 = "/UAM_minidrone/UAM_minidrone_cam"
        self.camera_2 = "/UAM_minidrone_01/UAM_minidrone_01_cam"
        self.camera_3 = "/UAM_minidrone_02/UAM_minidrone_02_cam"
        self.camera_4 = "/UAM_minidrone_03/UAM_minidrone_03_cam"
        self.viewports = {}

        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline_stop_event_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop)

    def build_ui(self):
        self.window = ui.Window("NavSim - Multi Manual Controller", width=600, height=600)
        
        with self.window.frame:
            with ui.VStack(spacing=10, height=0):
                self.add_viewports_button = ui.Button("ADD VIEWPORT", clicked_fn=self.add_viewports)

                with ui.HStack(spacing=10):
                    ui.Button(text="START", clicked_fn=self.start)
                    ui.Button(text="STOP", clicked_fn=self.stop)

                for i in range(self.max_joysticks):
                    # with ui.HStack(spacing=5):
                    ui.Label("User" + str(i+1))
                    ui.Separator()

    def start(self):
        self.controller.start()

    def stop(self):
        self.controller.stop()

    def on_timeline_stop(self, event):
        self.controller.stop()

    def add_viewports(self):
        amount_viewports = 0
        for viewport_window in omni.kit.viewport.window.get_viewport_window_instances():
            if not viewport_window.visible:
                viewport_window.viewport_widget.destroy()
                viewport_window.destroy()
                continue

            amount_viewports += 1
        
        viewport_width = ui.Workspace.get_main_window_width()/4
        viewport_height = ui.Workspace.get_main_window_height()/3
        render_resolution_scale = 0.5

        viewport_window = omni.kit.viewport.window.ViewportWindow(name=f"User {amount_viewports}", 
                                                                  width=viewport_width, height=viewport_height)

        viewport_window.setPosition(viewport_width*(amount_viewports - 1), viewport_height*2)
        viewport_window.viewport_api.set_active_camera(self.follow_UAV_camera_path)
        viewport_window.set_visibility_changed_fn(self.viewport_on_visibility_change)

        viewport_api = viewport_window.viewport_api
        viewport_api.set_texture_resolution((viewport_width, viewport_height))

    def viewport_on_visibility_change(self, visible):
        for viewport_window in omni.kit.viewport.window.get_viewport_window_instances():
            if not viewport_window.visible:
                # self.viewports.pop(viewport_window.title)
                viewport_window.viewport_widget.destroy()
                viewport_window.destroy()