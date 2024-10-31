import asyncio

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
        self.on_timeline_stop(None)

    def create_vars(self):
        self.controller = Controller()
        self.max_joysticks = 4
        self.is_running = False
        self.is_checking = False
        self.joystick_checkers = []

        self.perspective_camera_path = "/OmniverseKit_Persp"
        self.follow_UAV_camera_path = "/manual_controller_CAM"
        self.camera_1 = "/UAM_minidrone/UAM_minidrone_cam"
        self.camera_2 = "/UAM_minidrone_01/UAM_minidrone_01_cam"
        self.camera_3 = "/UAM_minidrone_02/UAM_minidrone_02_cam"
        self.camera_4 = "/UAM_minidrone_03/UAM_minidrone_03_cam"
        self.viewports = {}

        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline_start_event_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self.on_timeline_play)
        self.timeline_stop_event_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop)

    def build_ui(self):
        self.window = ui.Window("NavSim - Multi Manual Controller", width=0, height=0, 
                                raster_policy=ui.RasterPolicy.NEVER)
        
        with self.window.frame:
            with ui.VStack(height=0, spacing=10):
                # Add viewport button
                self.add_viewports_button = ui.Button("ADD VIEWPORT", height=50, clicked_fn=self.add_viewports)

                # Checking joystick part
                with ui.HStack(spacing=20):
                    # Start/Stop button
                    self.toggle_on_checking_button = ui.ToolButton(text="CHECK", width=80, height=100, style={"background_color": ui.color("#6f9523"),
                            "border_radius": 5, ":hovered": {"background_color": ui.color("#939393")}},
                            clicked_fn=self.toggle_checking)
                    
                    ui.Spacer(width=0)

                    # Checking part
                    for i in range(self.max_joysticks):
                        user_checker = []
                        x_checker = []
                        y_checker = []
                        z_checker = []

                        with ui.VStack(spacing=5):
                            with ui.ZStack():
                                # User container
                                ui.Rectangle(width=75, height=75, style={"background_color": ui.color("#787878"), 
                                                "border_color": ui.color.white, "border_width": 1, "border_radius": 5})
                                
                                with ui.Frame(width=75, height=75):
                                    with ui.VStack(height=0, spacing=5):
                                        ui.Spacer(height=10)

                                        with ui.HStack():
                                            # +X
                                            ui.Spacer(width=20)
                                            x_checker.append(ui.Rectangle(width=10, height=10, style={"background_color": ui.color("#952323"), 
                                                        "border_color": ui.color.white, "border_width": 0, "border_radius": 5}))
                                            # +Z
                                            ui.Spacer(width=25)
                                            z_checker.append(ui.Rectangle(width=10, height=10, style={"background_color": ui.color("#952323"), 
                                                        "border_color": ui.color.white, "border_width": 0, "border_radius": 5}))
                                            
                                        with ui.HStack():
                                            # +Y
                                            ui.Spacer(width=5)
                                            y_checker.append(ui.Rectangle(width=10, height=10, style={"background_color": ui.color("#952323"), 
                                                        "border_color": ui.color.white, "border_width": 0, "border_radius": 5}))
                                            # -Y
                                            ui.Spacer(width=20)
                                            y_checker.append(ui.Rectangle(width=10, height=10, style={"background_color": ui.color("#952323"), 
                                                        "border_color": ui.color.white, "border_width": 0, "border_radius": 5}))
                                        
                                        with ui.HStack():
                                            # -X
                                            ui.Spacer(width=20)
                                            x_checker.append(ui.Rectangle(width=10, height=10, style={"background_color": ui.color("#952323"), 
                                                        "border_color": ui.color.white, "border_width": 0, "border_radius": 5}))
                                            # -Z
                                            ui.Spacer(width=25)
                                            z_checker.append(ui.Rectangle(width=10, height=10, style={"background_color": ui.color("#952323"), 
                                                        "border_color": ui.color.white, "border_width": 0, "border_radius": 5}))

                            # Username
                            with ui.ZStack():
                                ui.Rectangle(width=75, height=20, style={"background_color": ui.color("#5b5b5b"), 
                                        "border_color": ui.color.white, "border_width": 1, "border_radius": 5})
                                with ui.Frame(width=75, height=20):
                                    ui.Label("User" + str(i+1), alignment=ui.Alignment.CENTER)

                            user_checker.append(x_checker)
                            user_checker.append(y_checker)
                            user_checker.append(z_checker)
                            self.joystick_checkers.append(user_checker)

                ui.Spacer(height=0)

    def toggle_checking(self):
        model = self.toggle_on_checking_button.model

        if model.get_value_as_bool():
            style={"background_color": ui.color("#952323"),"border_radius": 5, 
                   ":hovered": {"background_color": ui.color("#939393")}}
            self.toggle_on_checking_button.set_style(style)
            self.toggle_on_checking_button.text = "CHECKING"

            self.is_checking = True
            asyncio.ensure_future(self.start_checking())

        else:
            style={"background_color": ui.color("#6f9523"),"border_radius": 5, 
                   ":hovered": {"background_color": ui.color("#939393")}}
            self.toggle_on_checking_button.set_style(style)
            self.toggle_on_checking_button.text = "CHECK"

            self.is_checking = False
            self.stop_checking()

    async def start_checking(self):
        while self.is_checking:
            self.controller.joysticks.start()
            joystick_inputs = self.controller.check_joysticks()

            for i in range(len(joystick_inputs)):
                input = joystick_inputs[i]
                x_value = input[0]
                y_value = input[1]
                z_value = input[2]

                pos_x_checker: ui.Rectangle = self.joystick_checkers[i][0][0]
                neg_x_checker: ui.Rectangle = self.joystick_checkers[i][0][1]
                pos_y_checker: ui.Rectangle = self.joystick_checkers[i][1][0]
                neg_y_checker: ui.Rectangle = self.joystick_checkers[i][1][1]
                pos_z_checker: ui.Rectangle = self.joystick_checkers[i][2][0]
                neg_z_checker: ui.Rectangle = self.joystick_checkers[i][2][1]

                style={"background_color": ui.color("#6f9523"), "border_color": ui.color.white, "border_width": 0, 
                    "border_radius": 5}

                if x_value < 0:
                    pos_x_checker.set_style(style)
                if x_value > 0:
                    neg_x_checker.set_style(style)

                if y_value < 0:
                    pos_y_checker.set_style(style)
                if y_value > 0:
                    neg_y_checker.set_style(style)

                if z_value < 0:
                    pos_z_checker.set_style(style)
                if z_value > 0:
                    neg_z_checker.set_style(style)

            await asyncio.sleep(0.1)

    def stop_checking(self):
        self.controller.joysticks.stop()

        style={"background_color": ui.color("#952323"), "border_color": ui.color.white, "border_width": 0, 
                   "border_radius": 5}
        
        for checker in self.joystick_checkers:
            for axis in checker:
                for rectangle in axis:
                    rectangle.set_style(style)

    def on_timeline_play(self, event):
        if not self.is_running:
            self.is_running = True
            self.controller.start()

    def on_timeline_stop(self, event):
        self.controller.stop()
        self.is_running = False

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
        viewport_window.set_visibility_changed_fn(self.viewport_on_visibility_change)

    def viewport_on_visibility_change(self, visible):
        for viewport_window in omni.kit.viewport.window.get_viewport_window_instances():
            if not viewport_window.visible:
                # self.viewports.pop(viewport_window.title)
                viewport_window.viewport_widget.destroy()
                viewport_window.destroy()