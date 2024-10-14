# Adding root 'project' folder to sys.path
import sys
import os
# This line will add to the python list of paths to look for modules the path to the project root
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))

if project_root_path not in sys.path:
    sys.path.append(project_root_path)

import omni.ext
import omni.ui as ui
from omni.ui import color as cl
import omni.kit.app
import carb.events
from omni.isaac.ui.element_wrappers import DropDown
from omni.isaac.core.utils.stage import get_current_stage

import asyncio
from .controller_logic import ControllerLogic

from navsim_utils.extensions_utils import ExtensionUtils

try:
    import matplotlib
except:
    raise Exception("ERROR: 'matplotlib' package is not installed. Copy and paste in the Script Editor the " + 
                    "folllowing code\n\n" + 
                    "# -- START CODE ------------------------------\n" +
                    "import omni.kit.pipapi\n" +
                    "omni.kit.pipapi.install(\"matplotlib\")\n" +
                    "# -- END CODE --------------------------------\n")

try:
    matplotlib.use("Qt5Agg")
except:
    raise Exception("ERROR: 'PyQt5' package is not installed. Copy and paste in the Script Editor the " +
                    "folllowing code\n\n" + 
                    "# -- START CODE ------------------------------\n" +
                    "import omni.kit.pipapi\n" +
                    "omni.kit.pipapi.install(\"PyQt5\")\n" +
                    "# -- END CODE --------------------------------\n")

import matplotlib.pyplot as plt

class ManualController(omni.ext.IExt):

    def on_startup(self, ext_id):
        self.create_vars()
        self.build_ui()

    def on_shutdown(self):
        self.stop_update()
        self.manual_control.destroy_camera(get_current_stage())

    def create_vars(self):
        # Build ExtensionUtils instance
        self.ext_utils = ExtensionUtils()

        # UI window
        # The ui.RasterPolicy.NEVER is to always update plots line drawing
        self.window = ui.Window("NavSim - Manual Controller", width=600, height=600, raster_policy=ui.RasterPolicy.NEVER)

        # Plot data
        self.x_lv_plot_data = [0.0, 0.0]
        self.y_lv_plot_data = [0.0, 0.0]
        self.z_lv_plot_data = [0.0, 0.0]
        self.z_av_plot_data = [0.0, 0.0]

        # Plots appereance
        self.plots_appearance = 1
        self.build_plots = False
        self.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                                                                                self.build_plots_container, 
                                                                                name="Plots_building")

        # Control variable
        self.stop_update_plot = True

        # Manual control
        self.manual_control = ControllerLogic()

        # App interface
        self.app_interface = omni.kit.app.get_app_interface()

    def build_ui(self):
        with self.window.frame:

            with ui.ScrollingFrame():
                with ui.VStack(spacing=10, height=0):
                    ui.Spacer(height=10)

                    # UAV selector dropdown
                    self.UAV_selector_dropdown: DropDown = self.ext_utils.build_uav_selector()
                    self.UAV_selector_dropdown.set_on_selection_fn(self.change_uav_subject)

                    ui.Spacer(height=10)

                    # Controls power
                    self.controls_power = ui.CollapsableFrame(title="Control", collapsed=False)

                    with self.controls_power:
                        with ui.VStack(style={"margin": 1}, height=0):
                            with ui.HStack():
                                # Linear velocity (m/s)
                                ui.Label("Linear velocity Max")
                                self.linear_vel_power = ui.FloatSlider(min=0.5, max=10, step=0.5, precision=1, 
                                                                       style={"background_color": cl(0.13), 
                                                                              "secondary_color": cl(0.3), 
                                                                              "draw_mode": ui.SliderDrawMode.FILLED})
                                self.linear_vel_power.model.set_value(1)
                                self.linear_vel_power.model.add_value_changed_fn(self.on_linear_vel_power_change)

                            ui.Spacer(height=10)

                            with ui.HStack():
                                # Angular velocity (rad/s)
                                ui.Label("Angular velocity Max")
                                self.angular_vel_power = ui.FloatSlider(min=0.5, max=6, step=0.5, precision=1, 
                                                                        style={"background_color": cl(0.13), 
                                                                               "secondary_color": cl(0.3), 
                                                                               "draw_mode": ui.SliderDrawMode.FILLED})
                                self.angular_vel_power.model.set_value(1)
                                self.angular_vel_power.model.add_value_changed_fn(self.on_angular_vel_power_change)

                            ui.Spacer(height=25)

                            with ui.HStack(spacing=5):
                                # On/Off init rotors
                                ui.Label("Start with rotors on")
                                self.start_rotors_on_off_checkbox = ui.CheckBox()

                            ui.Spacer(height=5)

                            # Start/Stop manual control
                            self.start_stop_tool_button = ui.ToolButton(text="START", height=30, 
                                                                        clicked_fn=self.start_stop_update, 
                                                                        style={"background_color": cl("#6f9523")})


                    # Controls ploting
                    self.controls_ploting = ui.CollapsableFrame(title="Visualization", collapsed=False)

                    with self.controls_ploting:
                        self.plots_container = ui.VStack(height=0)

                        self.build_plot_container_content()

    def change_uav_subject(self, uav_name):
        # Switch the camera subject target to the selected UAV
        uav = self.ext_utils.get_prim_by_name(uav_name)
        self.manual_control.change_camera_subject(uav.GetPath())
        
        # Check if the controller is running (START button clicked) to change UAV control
        if not self.stop_update_plot:
            # Get if rotors should be on/off
            self.rotors_on = self.start_rotors_on_off_checkbox.model.get_value_as_bool()
            self.manual_control.current_on = self.rotors_on

            # Restart manual control
            self.manual_control.stop()
            self.manual_control.start(uav)

    def on_linear_vel_power_change(self, model):
        # Get and set new limit
        self.linear_vel_limit = model.get_value_as_float()
        self.manual_control.linear_vel_limit = self.linear_vel_limit

        self.update_ui_linear_vel_limits()

    def on_angular_vel_power_change(self, model):
        self.angular_vel_limit = model.get_value_as_float()
        self.manual_control.ang_vel_limit = self.angular_vel_limit

        self.update_ui_angular_vel_limits()
    
    def change_plot_distribution(self, model, index):        
        item = model.get_item_value_model(index).as_int

        match item:
            case 0:
                self.plots_appearance = 0

            case 1:
                self.plots_appearance = 1

            case 2:
                self.plots_appearance = 2

        self.build_plots = True

    def build_plots_container(self, e: carb.events.IEvent):
        if self.build_plots:
            self.build_plots = False

            self.plots_container.clear()

            self.build_plot_container_content()

    def build_plot_container_content(self):
        with self.plots_container:
            ui.Spacer(height=10)
            
            # Plots appereance section
            with ui.HStack():
                ui.Label("Plots appereance", alignment=ui.Alignment.LEFT)
                self.plots_appearance_combo_box = ui.ComboBox(self.plots_appearance, "Rows", "Group", "Stack")
                self.plots_appearance_combo_box.model.add_item_changed_fn(self.change_plot_distribution)

            # Make UI beauty
            ui.Spacer(height=10)

            self.reset_plots_button = ui.Button("RESET PLOTS", clicked_fn=self.reset_plot, height=35, 
                                                style={"Button":{"background_color": cl("#952323")}})

            # Make UI beauty
            ui.Spacer(height=30)

            # Plots section
            match self.plots_appearance:
                case 0:
                    self.first_way()

                case 1:
                    self.second_way()

                case 2:
                    self.third_way()

    def start_stop_update(self):
        model = self.start_stop_tool_button.model

        if model.get_value_as_bool():            
            self.start_update()

            style={"background_color": cl("#952323")}
            self.start_stop_tool_button.set_style(style)
            self.start_stop_tool_button.text = "STOP"

        else:
            self.stop_update()
            
            style={"background_color": cl("#6f9523")}
            self.start_stop_tool_button.set_style(style)
            self.start_stop_tool_button.text = "START"

    async def update_plot(self):
        while not self.stop_update_plot:
            # Get external inputs
            self.x_lv_plot_data.append(self.manual_control.inputs[0] * -self.linear_vel_limit)
            self.y_lv_plot_data.append(self.manual_control.inputs[1] * -self.linear_vel_limit)
            self.z_lv_plot_data.append(self.manual_control.inputs[2] * -self.linear_vel_limit)
            self.z_av_plot_data.append(self.manual_control.inputs[3] * -self.angular_vel_limit)
            
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
        if self.UAV_selector_dropdown.get_selection() is None:
            # Reset start_stop_toolbutton as it changed its model state
            self.start_stop_tool_button.model.set_value(False)

            raise Exception("No UAV selected")

        # Change loop variable value to startloop
        self.stop_update_plot = False

        # Set controls power to controller
        self.linear_vel_limit = self.linear_vel_power.model.get_value_as_float()
        self.angular_vel_limit = self.angular_vel_power.model.get_value_as_float()
        self.rotors_on = self.start_rotors_on_off_checkbox.model.get_value_as_bool()

        self.manual_control.ang_vel_limit = self.angular_vel_limit
        self.manual_control.linear_vel_limit = self.linear_vel_limit
        self.manual_control.current_on = self.rotors_on

        self.update_ui_linear_vel_limits()
        self.update_ui_angular_vel_limits()

        # Get the selected UAV
        uav = self.ext_utils.get_prim_by_name(self.UAV_selector_dropdown.get_selection())

        self.manual_control.start(uav)

        # Start the coroutine that updates the plots
        asyncio.ensure_future(self.update_plot())

    def update_ui_linear_vel_limits(self):
        # Update linear velocity limit labels
        self.x_max_lvl.text = str(self.linear_vel_limit)
        if not self.TRDW:
            self.y_max_lvl.text = str(self.linear_vel_limit)
            self.z_max_lvl.text = str(self.linear_vel_limit)

        self.x_min_lvl.text = str(-self.linear_vel_limit)
        if not self.TRDW:
            self.y_min_lvl.text = str(-self.linear_vel_limit)
            self.z_min_lvl.text = str(-self.linear_vel_limit)

        # Adjust scale to corresponding control power
        self.x_lv_plot.scale_min = -self.linear_vel_limit
        self.x_lv_plot.scale_max = self.linear_vel_limit
        
        self.y_lv_plot.scale_min = -self.linear_vel_limit
        self.y_lv_plot.scale_max = self.linear_vel_limit

        self.z_lv_plot.scale_min = -self.linear_vel_limit
        self.z_lv_plot.scale_max = self.linear_vel_limit

    def update_ui_angular_vel_limits(self):
        # Update angular velocity limit labels
        self.z_max_avl.text = str(self.angular_vel_limit)
        self.z_min_avl.text = str(-self.angular_vel_limit)

        # Adjust scale to corresponding control power
        self.z_av_plot.scale_min = -self.angular_vel_limit
        self.z_av_plot.scale_max = self.angular_vel_limit

    def stop_update(self):
        # Reset loop variable
        self.stop_update_plot = True

        # Stop asking for inputs
        self.manual_control.stop()

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
        ui.Spacer(height=5)
        self.x_max_lvl = ui.Label("1.0")
        self.x_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.x_lv_plot_data, height=50, alignment=ui.Alignment.CENTER, 
                                 style={"color": cl("#B13333")})
        self.x_min_lvl = ui.Label("-1.0")

        ui.Spacer(height=10)

        # Y linear vel
        self.y_linear_vel_label = ui.Label("Y linear velocity (m/s)", alignment=ui.Alignment.CENTER)
        ui.Spacer(height=5)
        self.y_max_lvl = ui.Label("1.0")
        self.y_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.y_lv_plot_data, height=50, alignment=ui.Alignment.CENTER, 
                                 style={"color": cl("#54B133")})
        self.y_min_lvl = ui.Label("-1.0")

        ui.Spacer(height=10)

        # Z linear vel
        self.z_linear_vel_label = ui.Label("Z linear velocity (m/s)", alignment=ui.Alignment.CENTER)
        ui.Spacer(height=5)
        self.z_max_lvl = ui.Label("1.0")
        self.z_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_lv_plot_data, height=50, alignment=ui.Alignment.CENTER, 
                                 style={"color": cl("#4C73E2")})
        self.z_min_lvl = ui.Label("-1.0")
            
        ui.Spacer(height=10)

        # Z angular vel
        self.z_angular_vel_label = ui.Label("Z angular velocity (rad/s)", alignment=ui.Alignment.CENTER)
        ui.Spacer(height=5)
        self.z_max_avl = ui.Label("1.0")
        self.z_av_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_av_plot_data, height=50, alignment=ui.Alignment.CENTER, 
                                 style={"color": cl.orange})
        self.z_min_avl = ui.Label("-1.0")

    def second_way(self):
        self.TRDW = False

        with ui.HStack(spacing=5):
            with ui.VStack():
                # X linear vel
                self.x_linear_vel_label = ui.Label("X linear velocity (m/s)", alignment=ui.Alignment.CENTER)
                ui.Spacer(height=5)
                self.x_max_lvl = ui.Label("1.0")
                self.x_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.x_lv_plot_data, height=50, 
                                         alignment=ui.Alignment.CENTER, style={"color": cl("#B13333")})
                self.x_min_lvl = ui.Label("-1.0")

            with ui.VStack():
                # Y linear vel
                self.y_linear_vel_label = ui.Label("Y linear velocity (m/s)", alignment=ui.Alignment.CENTER)
                ui.Spacer(height=5)
                self.y_max_lvl = ui.Label("1.0")
                self.y_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.y_lv_plot_data, height=50, 
                                         alignment=ui.Alignment.CENTER, style={"color": cl("#54B133")})
                self.y_min_lvl = ui.Label("-1.0")

            with ui.VStack():
                # Z linear vel
                self.z_linear_vel_label = ui.Label("Z linear velocity (m/s)", alignment=ui.Alignment.CENTER)
                ui.Spacer(height=5)
                self.z_max_lvl = ui.Label("1.0")
                self.z_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_lv_plot_data, height=50, 
                                         alignment=ui.Alignment.CENTER, style={"color": cl("#4C73E2")})
                self.z_min_lvl = ui.Label("-1.0")
            
        ui.Spacer(height=20)

        # Z angular vel
        self.z_angular_vel_label = ui.Label("Z angular velocity (rad/s)", alignment=ui.Alignment.CENTER)
        ui.Spacer(height=5)
        self.z_max_avl = ui.Label("1.0")
        self.z_av_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_av_plot_data, height=50, 
                                 alignment=ui.Alignment.CENTER, style={"color": cl.orange})
        self.z_min_avl = ui.Label("-1.0")

    def third_way(self):
        self.TRDW = True

        self.x_linear_vel_label = ui.Label("Linear velocity (m/s)", alignment=ui.Alignment.CENTER)
        ui.Spacer(height=5)
        self.x_max_lvl = ui.Label("1.0")
        
        with ui.ZStack():
            ui.Rectangle(style={"background_color": 0xFF555555, "border_color": 0xFF000000, "border_width": 1})
            frame = ui.Frame(width=ui.Percent(100), height=50, opaque_for_mouse_events=True, style={"color": 0xFFFFFFFF})

            with frame:
                with ui.ZStack():
                    # X linear vel
                    self.x_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.x_lv_plot_data, width=ui.Percent(100), 
                                             height=50, style={"color": cl("#B13333"), "background_color": 0x00000000})

                    # Y linear vel
                    self.y_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.y_lv_plot_data, width=ui.Percent(100), 
                                             height=50, style={"color": cl("#54B133"), "background_color": 0x00000000})

                    # Z linear vel
                    self.z_lv_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_lv_plot_data, width=ui.Percent(100), 
                                             height=50, style={"color": cl("#4C73E2"), "background_color": 0x00000000})
            
        self.x_min_lvl = ui.Label("-1.0")

        # Z angular vel
        self.z_angular_vel_label = ui.Label("Z angular velocity (rad/s)", alignment=ui.Alignment.CENTER)
        ui.Spacer(height=5)
        self.z_max_avl = ui.Label("1.0")
        self.z_av_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_av_plot_data, height=50, alignment=ui.Alignment.CENTER, 
                                 style={"color": cl.orange})
        self.z_min_avl = ui.Label("-1.0")