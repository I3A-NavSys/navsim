import omni.ext
import omni.ui as ui

from omni.isaac.ui.element_wrappers import *
from omni.isaac.core.utils.stage import get_current_stage

import asyncio
import omni.kit.app
from .ExternalController import ExternalController
from omni.ui import color as cl


class NavsimExternalcontrolExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[NavSim.ExternalControl] NavSim ExternalControl startup")

        self.create_vars()
        self.build_ui()


    def on_shutdown(self):
        print("[NavSim.ExternalControl] NavSim ExternalControl shutdown")

        self.stop_update()


    def create_vars(self):
        # Plot data
        self.x_lv_plot_data = [0.0, 0.0]
        self.y_lv_plot_data = [0.0, 0.0]
        self.z_lv_plot_data = [0.0, 0.0]
        self.z_av_plot_data = [0.0, 0.0]

        # Control variable
        self.stop_update_plot = True
        
        # List of manipulable prims (usually drones)
        self.manipulable_prims = []

        # Instance of ExternalController
        self.external_control = ExternalController()

    
    def build_ui(self):
        self._window = ui.Window("NavSim - External UAV control", width=300, height=300)

        with self._window.frame:

            with ui.ScrollingFrame():

                with ui.VStack(spacing=10, height=0):
                    ui.Spacer(height=10)

                    # Drone selector dropdown
                    with ui.HStack(spacing=5):
                        # Dropdown selector
                        self.drone_selector_dropdown = DropDown("Select Drone", "Select the drone you want to control", self.get_manipulable_prims)
                        self.drone_selector_dropdown.enabled = False

                        # Button to refresh manipulable drones
                        self.refresh_selector_button = ui.Button("REFRESH", clicked_fn=self.refresh_drone_selector, width=100)

                    ui.Spacer(height=10)

                    # Controls power
                    self.controls_power = CollapsableFrame(title="Controls power", collapsed=False)

                    with self.controls_power:
                        with ui.VStack():
                            with ui.HStack():
                                # Linear velocity (m/s)
                                ui.Label("Linear velocity (m/s)")
                                self.linear_vel_power = ui.FloatSlider(min=0.5, max=4, step=0.5, precision=1)
                                self.linear_vel_power.model.set_value(1)

                            ui.Spacer(height=10)

                            with ui.HStack():
                                # Angular velocity (rad/s)
                                ui.Label("Angular velocity (rad/s)")
                                self.angular_vel_power = ui.FloatSlider(min=0.5, max=3, step=0.5, precision=1)
                                self.angular_vel_power.model.set_value(1)

                    # Controls ploting
                    self.controls_ploting = CollapsableFrame(title="Drone control visualization", collapsed=False)

                    with self.controls_ploting:
                        with ui.VStack():
                            with ui.HStack(spacing=5):
                                # Control buttons
                                ui.Button("START", clicked_fn=self.start_update, height=5)
                                ui.Button("STOP", clicked_fn=self.stop_update, height=5)
                                ui.Button("RESET", clicked_fn=self.reset_plot, height=5)

                            ui.Spacer(height=10)
                            
                            # Plots
                            # self.first_way()
                            self.second_way()
                            # self.third_way()


    def get_manipulable_prims(self, prim=None):
        # Needed list containing the names of the manipulable prims for the dropdown selector
        manipulable_prims = []

        # First iteration
        if prim is None:
            stage = get_current_stage()
            prim = stage.GetPrimAtPath("/World")

            self.manipulable_prims.clear()

        # Check current prim
        if prim.GetAttribute("NavSim:Manipulable").IsValid() and prim.GetAttribute("NavSim:Manipulable").Get():
            manipulable_prims.append(prim.GetName())
            self.manipulable_prims.append(prim)

        # Check prim's children
        for child in prim.GetAllChildren():
            # First check children
            if len(child.GetAllChildren()) > 0:
                manipulable_prims += self.get_manipulable_prims(child)
            
            # Then check current child
            elif child.GetAttribute("NavSim:Manipulable").IsValid() and child.GetAttribute("NavSim:Manipulable").Get():
                manipulable_prims.append(child.GetName())
                self.manipulable_prims.append(prim)

        return manipulable_prims
    

    def refresh_drone_selector(self):
        self.drone_selector_dropdown.enabled = True
        self.drone_selector_dropdown.repopulate()


    async def update_plot(self):
        while not self.stop_update_plot:
            # Get external inputs
            self.x_lv_plot_data.append(self.external_control.inputs[0] * -self.linear_vel_limit)
            self.y_lv_plot_data.append(self.external_control.inputs[1] * -self.linear_vel_limit)
            self.z_lv_plot_data.append(self.external_control.inputs[2] * -self.linear_vel_limit)
            self.z_av_plot_data.append(self.external_control.inputs[3] * -self.angular_vel_limit)
            
            # To have a continuous plot line
            if len(self.x_lv_plot_data) > 50:
                self.x_lv_plot_data.pop(0)

            if len(self.y_lv_plot_data) > 50:
                self.y_lv_plot_data.pop(0)

            if len(self.z_lv_plot_data) > 50:
                self.z_lv_plot_data.pop(0)

            if len(self.z_av_plot_data) > 50:
                self.z_av_plot_data.pop(0)

            # Update plot data
            self.x_lv_plot.set_data(*self.x_lv_plot_data)
            self.y_lv_plot.set_data(*self.y_lv_plot_data)
            self.z_lv_plot.set_data(*self.z_lv_plot_data)
            self.z_av_plot.set_data(*self.z_av_plot_data)

            # Update 10 times per second
            await asyncio.sleep(0.1)

            # Avoid exception when saving file while running simulation
            if not hasattr(self, "stop_update_plot"):
                break


    def start_update(self):
        if len(self.manipulable_prims) == 0:
            raise Exception("No drone selected")

        # Start just once
        if self.stop_update_plot:
            print("-- START --")
            self.stop_update_plot = False

            # Set controls power to controller
            self.linear_vel_limit = self.linear_vel_power.model.get_value_as_float()
            self.angular_vel_limit = self.angular_vel_power.model.get_value_as_float()

            self.external_control.ang_vel_limit = self.angular_vel_limit
            self.external_control.linear_vel_limit = self.linear_vel_limit

            # Update linear velocity limit labels
            self.x_max_lvl.text = str(self.linear_vel_limit)
            if not self.TRDW:
                self.y_max_lvl.text = str(self.linear_vel_limit)
                self.z_max_lvl.text = str(self.linear_vel_limit)

            self.x_min_lvl.text = str(-self.linear_vel_limit)
            if not self.TRDW:
                self.y_min_lvl.text = str(-self.linear_vel_limit)
                self.z_min_lvl.text = str(-self.linear_vel_limit)

            # Update angular velocity limit labels
            self.z_max_avl.text = str(self.angular_vel_limit)
            self.z_min_avl.text = str(-self.angular_vel_limit)
            
            # Adjust scale to corresponding control power
            self.x_lv_plot.scale_min = -self.linear_vel_limit
            self.x_lv_plot.scale_max = self.linear_vel_limit
            
            self.y_lv_plot.scale_min = -self.linear_vel_limit
            self.y_lv_plot.scale_max = self.linear_vel_limit

            self.z_lv_plot.scale_min = -self.linear_vel_limit
            self.z_lv_plot.scale_max = self.linear_vel_limit

            # Get the selected drone
            drone = self.manipulable_prims[self.drone_selector_dropdown.get_selection_index()]

            # Start external control
            self.external_control.start(drone)

            # Start the coroutine that updates the plots
            asyncio.ensure_future(self.update_plot())

        else:
            print("ALREADY STARTED")


    def stop_update(self):
        # Stop just once
        if not self.stop_update_plot:
            print("-- STOP --")
            self.stop_update_plot = True
            self.external_control.stop()

        else:
            print("ALREADY STOPPED")


    def reset_plot(self):
        self.x_lv_plot_data = [0.0, 0.0]
        self.y_lv_plot_data = [0.0, 0.0]
        self.z_lv_plot_data = [0.0, 0.0]
        self.z_av_plot_data = [0.0, 0.0]

        self.x_lv_plot.set_data(*self.x_lv_plot_data)
        self.y_lv_plot.set_data(*self.y_lv_plot_data)
        self.z_lv_plot.set_data(*self.z_lv_plot_data)
        self.z_av_plot.set_data(*self.z_av_plot_data)

    
    def first_way(self):
        self.TRDW = False

        # X linear vel
        self.x_linear_vel_label = ui.Label("X linear velocity (m/s)", alignment=ui.Alignment.CENTER)
        self.x_max_lvl = ui.Label("1.0")
        self.x_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.x_lv_plot_data, height=50, alignment=ui.Alignment.CENTER, style={"color": cl("#B13333")})
        self.x_min_lvl = ui.Label("-1.0")

        ui.Spacer(height=10)
        ui.Separator()

        # Y linear vel
        self.y_linear_vel_label = ui.Label("Y linear velocity (m/s)", alignment=ui.Alignment.CENTER)
        self.y_max_lvl = ui.Label("1.0")
        self.y_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.y_lv_plot_data, height=50, alignment=ui.Alignment.CENTER, style={"color": cl("#54B133")})
        self.y_min_lvl = ui.Label("-1.0")

        ui.Spacer(height=10)
        ui.Separator()

        # Z linear vel
        self.z_linear_vel_label = ui.Label("Z linear velocity (m/s)", alignment=ui.Alignment.CENTER)
        self.z_max_lvl = ui.Label("1.0")
        self.z_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_lv_plot_data, height=50, alignment=ui.Alignment.CENTER, style={"color": cl("#4C73E2")})
        self.z_min_lvl = ui.Label("-1.0")
            
        ui.Spacer(height=10)

        # Z angular vel
        self.z_angular_vel_label = ui.Label("Z angular velocity (rad/s)", alignment=ui.Alignment.CENTER)
        self.z_max_avl = ui.Label("1.0")
        self.z_av_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_av_plot_data, height=50, alignment=ui.Alignment.CENTER, style={"color": cl("#4C73E2")})
        self.z_min_avl = ui.Label("-1.0")


    def second_way(self):
        self.TRDW = False

        with ui.HStack(spacing=5):
            with ui.VStack():
                # X linear vel
                self.x_linear_vel_label = ui.Label("X linear velocity (m/s)", alignment=ui.Alignment.CENTER)
                self.x_max_lvl = ui.Label("1.0")
                self.x_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.x_lv_plot_data, height=50, alignment=ui.Alignment.CENTER, style={"color": cl("#B13333")})
                self.x_min_lvl = ui.Label("-1.0")

            with ui.VStack():
                # Y linear vel
                self.y_linear_vel_label = ui.Label("Y linear velocity (m/s)", alignment=ui.Alignment.CENTER)
                self.y_max_lvl = ui.Label("1.0")
                self.y_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.y_lv_plot_data, height=50, alignment=ui.Alignment.CENTER, style={"color": cl("#54B133")})
                self.y_min_lvl = ui.Label("-1.0")

            with ui.VStack():
                # Z linear vel
                self.z_linear_vel_label = ui.Label("Z linear velocity (m/s)", alignment=ui.Alignment.CENTER)
                self.z_max_lvl = ui.Label("1.0")
                self.z_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_lv_plot_data, height=50, alignment=ui.Alignment.CENTER, style={"color": cl("#4C73E2")})
                self.z_min_lvl = ui.Label("-1.0")
            
        ui.Spacer(height=10)

        # Z angular vel
        self.z_angular_vel_label = ui.Label("Z angular velocity (rad/s)", alignment=ui.Alignment.CENTER)
        self.z_max_avl = ui.Label("1.0")
        self.z_av_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_av_plot_data, height=50, alignment=ui.Alignment.CENTER, style={"color": cl("#4C73E2")})
        self.z_min_avl = ui.Label("-1.0")


    def third_way(self):
        self.TRDW = True

        self.x_linear_vel_label = ui.Label("Linear velocity (m/s)", alignment=ui.Alignment.CENTER)
        self.x_max_lvl = ui.Label("1.0")
        
        with ui.ZStack():
            ui.Rectangle(style={"background_color": 0xFF555555, "border_color": 0xFF000000, "border_width": 1})
            frame = ui.Frame(width=ui.Percent(100), height=50, opaque_for_mouse_events=True, style={"color": 0xFFFFFFFF})

            with frame:
                with ui.ZStack():
                    # X linear vel
                    self.x_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.x_lv_plot_data, width=ui.Percent(100), height=50, style={"color": cl("#B13333"), "background_color": 0x00000000})

                    # Y linear vel
                    self.y_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.y_lv_plot_data, width=ui.Percent(100), height=50, style={"color": cl("#54B133"), "background_color": 0x00000000})

                    # Z linear vel
                    self.z_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_lv_plot_data, width=ui.Percent(100), height=50, style={"color": cl("#4C73E2"), "background_color": 0x00000000})
            
        self.x_min_lvl = ui.Label("-1.0")

        # Z angular vel
        self.z_angular_vel_label = ui.Label("Z angular velocity (rad/s)", alignment=ui.Alignment.CENTER)
        self.z_max_avl = ui.Label("1.0")
        self.z_av_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_av_plot_data, height=50, alignment=ui.Alignment.CENTER, style={"color": cl("#4C73E2")})
        self.z_min_avl = ui.Label("-1.0")