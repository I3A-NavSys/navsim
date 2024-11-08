import asyncio

import omni.ext
import omni.ui as ui
import omni.physx
import omni.timeline
import omni.kit.viewport.window as vp_window
# Adding root 'ov' folder to sys.path
import sys, os
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)
    
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
        self.is_controlling = False
        self.joystick_checkers = []

        self.perspective_camera_path = "/OmniverseKit_Persp"
        self.follow_UAV_camera_path = "/manual_controller_CAM"
        self.camera_1 = "/Drones/drone1/drone1_camera"
        self.camera_2 = "/Drones/drone2/drone2_camera"
        self.camera_3 = "/Drones/drone3/drone3_camera"
        self.camera_4 = "/Drones/drone4/drone4_camera"
        self.viewports = {}

        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline_start_event_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self.on_timeline_play)
        self.timeline_stop_event_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop)

    def build_ui(self):
        toggle_checking_button_style = {"background_color": ui.color("#db8f26"),
                            "border_radius": 5, ":hovered": {"background_color": ui.color("#939393")}}
        
        checker_container_style = {"background_color": ui.color("#787878"), "border_color": ui.color.white, 
                                    "border_width": 1, "border_radius": 5}
        
        checker_style = {"background_color": ui.color("#db8f26"), "border_color": ui.color.white,
                        "border_width": 0, "border_radius": 5}
        
        username_container_style = {"background_color": ui.color("#5b5b5b"), "border_color": ui.color.white, 
                        "border_width": 1, "border_radius": 5}
        
        add_viewport_button_style = {"border_radius": 5}

        toggle_control_button_style = {"background_color": ui.color("#952323"),
                            "border_radius": 5, ":hovered": {"background_color": ui.color("#939393")}}

        self.window = ui.Window("NavSim - Multi Manual Controller", width=0, height=0, 
                                raster_policy=ui.RasterPolicy.NEVER)
        
        with self.window.frame:
            with ui.VStack(height=0, spacing=10):
                # Checking joystick part
                # Check button
                self.toggle_checking_button = ui.ToolButton(text="CHECK", height=50, 
                        style=toggle_checking_button_style, clicked_fn=self.toggle_checking)
                    
                with ui.HStack(spacing=500):
                    # Checking part
                    for i in range(self.max_joysticks):
                        user_checker = []
                        x_checker = []
                        y_checker = []
                        z_checker = []

                        with ui.VStack(spacing=5):
                            with ui.ZStack():
                                # User container
                                ui.Rectangle(width=75, height=75, style=checker_container_style)
                                
                                with ui.Frame(width=75, height=75):
                                    with ui.VStack(height=0, spacing=5):
                                        ui.Spacer(height=10)

                                        with ui.HStack():
                                            # +X
                                            ui.Spacer(width=20)
                                            x_checker.append(ui.Rectangle(width=10, height=10, style=checker_style))
                                            # +Z
                                            ui.Spacer(width=25)
                                            z_checker.append(ui.Rectangle(width=10, height=10, style=checker_style))
                                            
                                        with ui.HStack():
                                            # +Y
                                            ui.Spacer(width=5)
                                            y_checker.append(ui.Rectangle(width=10, height=10, style=checker_style))
                                            # -Y
                                            ui.Spacer(width=20)
                                            y_checker.append(ui.Rectangle(width=10, height=10, style=checker_style))
                                        
                                        with ui.HStack():
                                            # -X
                                            ui.Spacer(width=20)
                                            x_checker.append(ui.Rectangle(width=10, height=10, style=checker_style))
                                            # -Z
                                            ui.Spacer(width=25)
                                            z_checker.append(ui.Rectangle(width=10, height=10, style=checker_style))

                            # Username
                            with ui.ZStack():
                                ui.Rectangle(width=75, height=20, style=username_container_style)
                                with ui.Frame(width=75, height=20):
                                    ui.Label("Jugador " + str(i+1), alignment=ui.Alignment.CENTER)

                            user_checker.append(x_checker)
                            user_checker.append(y_checker)
                            user_checker.append(z_checker)
                            self.joystick_checkers.append(user_checker)

                ui.Spacer(height=50)

                # Add viewport button
                self.add_viewports_button = ui.Button("ADD VIEWPORT", height=40, style= add_viewport_button_style,
                                                       clicked_fn=self.add_viewports)

                # Control/Not control
                self.toggle_control_button = ui.ToolButton(text="NO CONTROL", height=40,
                                                    style=toggle_control_button_style, clicked_fn=self.toggle_control)
                self.toggle_control_button.model.set_value(True)

    def toggle_control(self):
        model = self.toggle_control_button.model

        if model.get_value_as_bool():
            style={"background_color": ui.color("#952323"),"border_radius": 5, 
                   ":hovered": {"background_color": ui.color("#939393")}}
            self.toggle_control_button.set_style(style)
            self.toggle_control_button.text = "NO CONTROL"

            self.is_controlling = False

        else:
            style={"background_color": ui.color("#6f9523"),"border_radius": 5, 
                   ":hovered": {"background_color": ui.color("#939393")}}
            self.toggle_control_button.set_style(style)
            self.toggle_control_button.text = "CONTROL"

            self.is_controlling = True

    def toggle_checking(self):
        model = self.toggle_checking_button.model

        if model.get_value_as_bool():
            style={"background_color": ui.color("#e14e27"),"border_radius": 5, 
                   ":hovered": {"background_color": ui.color("#939393")}}
            self.toggle_checking_button.set_style(style)
            self.toggle_checking_button.text = "CHECKING"

            self.is_checking = True
            asyncio.ensure_future(self.start_checking())

        else:
            style={"background_color": ui.color("#db8f26"),"border_radius": 5, 
                   ":hovered": {"background_color": ui.color("#939393")}}
            self.toggle_checking_button.set_style(style)
            self.toggle_checking_button.text = "CHECK"

            self.is_checking = False
            self.stop_checking()

    async def start_checking(self):
        while self.is_checking:
            self.controller.joysticks.start()
            joystick_inputs = self.controller.check_joysticks()

            style={"background_color": ui.color("#6f9523"), "border_color": ui.color.white, "border_width": 0, 
                    "border_radius": 5}

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

        style={"background_color": ui.color("#db8f26"), "border_color": ui.color.white, "border_width": 0, 
                   "border_radius": 5}
        
        for checker in self.joystick_checkers:
            for axis in checker:
                for rectangle in axis:
                    rectangle.set_style(style)

    def on_timeline_play(self, event):
        if not self.is_running and self.is_controlling:
            self.is_running = True
            self.controller.start()

    def on_timeline_stop(self, event):
        if self.is_running:
            self.controller.stop()
            self.is_running = False

    def add_viewports(self):
        amount_viewports = 0
        for viewport_window in vp_window.get_viewport_window_instances():
            if not viewport_window.visible:
                viewport_window.viewport_widget.destroy()
                viewport_window.destroy()
                continue

            amount_viewports += 1
        
        window_name = f"User {amount_viewports}"
        viewport_width = ui.Workspace.get_main_window_width()/4
        viewport_height = ui.Workspace.get_main_window_height()/3

        viewport_window = vp_window.ViewportWindow(name=window_name, width=viewport_width, height=viewport_height)
        viewport_api = viewport_window.viewport_api

        viewport_window.setPosition(viewport_width*(amount_viewports - 1), viewport_height*2)
        viewport_window.set_visibility_changed_fn(self.viewport_on_visibility_change)

        match(window_name):
            case "User 1":
                viewport_api.set_active_camera(self.camera_1)
                # viewport_window.setPosition(0, 0)
            case "User 2":
                viewport_api.set_active_camera(self.camera_2)
                # viewport_window.setPosition(viewport_width, 0)
            case "User 3":
                viewport_api.set_active_camera(self.camera_3)
                # viewport_window.setPosition(0, viewport_height)
            case "User 4":
                viewport_api.set_active_camera(self.camera_4)
                # viewport_window.setPosition(viewport_width, viewport_height)
            case _:
                pass

    def viewport_on_visibility_change(self, visible):
        for viewport_window in omni.kit.viewport.window.get_viewport_window_instances():
            if not viewport_window.visible:
                # self.viewports.pop(viewport_window.title)
                viewport_window.viewport_widget.destroy()
                viewport_window.destroy()