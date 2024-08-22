import omni.ext
import omni.ui as ui

from omni.isaac.ui.ui_utils import dropdown_builder
from omni.isaac.ui.element_wrappers import *
from omni.isaac.core.utils.stage import get_current_stage

import asyncio
import omni.kit.app
from .ExternalController import ExternalController
from omni.ui import color as cl

import carb.events

import matplotlib
try:
    matplotlib.use("Qt5Agg")

except ModuleNotFoundError:
    import omni.kit.pipapi
    omni.kit.pipapi.install("PyQt5")

import matplotlib.pyplot as plt

class NavsimExternalcontrolExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[NavSim.ExternalControl] NavSim ExternalControl startup")

        self.create_vars()
        self.build_ui()


    def on_shutdown(self):
        print("[NavSim.ExternalControl] NavSim ExternalControl shutdown")

        self.stop_update()


    def create_vars(self):
        # UI window
        self.window = ui.Window("NavSim - External UAV control", width=300, height=300, raster_policy=ui.RasterPolicy.NEVER)   # The ui.RasterPolicy.NEVER is to always update plots line drawing

        # Plot data
        self.x_lv_plot_data = [0.0, 0.0]
        self.y_lv_plot_data = [0.0, 0.0]
        self.z_lv_plot_data = [0.0, 0.0]
        self.z_av_plot_data = [0.0, 0.0]

        # Plots appereance
        self.plots_appearance = 1
        self.build_plots = False
        self.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self.build_plots_container, name="Plots_building")

        # Control variable
        self.stop_update_plot = True
        
        # List of manipulable prims (usually drones)
        self.manipulable_prims = []

        # Instance of ExternalController
        self.external_control = ExternalController()


    def build_ui(self):
        with self.window.frame:

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
                    self.controls_power = ui.CollapsableFrame(title="Controls power", collapsed=False)

                    with self.controls_power:
                        with ui.VStack(style={"margin": 1}):
                            with ui.HStack():
                                # Linear velocity (m/s)
                                ui.Label("Linear velocity (m/s)")
                                self.linear_vel_power = ui.FloatSlider(min=0.5, max=4, step=0.5, precision=1, style={"background_color": cl(0.13), "secondary_color": cl(0.3), "draw_mode": ui.SliderDrawMode.FILLED})
                                self.linear_vel_power.model.set_value(1)

                            ui.Spacer(height=10)

                            with ui.HStack():
                                # Angular velocity (rad/s)
                                ui.Label("Angular velocity (rad/s)")
                                self.angular_vel_power = ui.FloatSlider(min=0.5, max=3, step=0.5, precision=1, style={"background_color": cl(0.13), "secondary_color": cl(0.3), "draw_mode": ui.SliderDrawMode.FILLED})
                                self.angular_vel_power.model.set_value(1)

                    # Controls ploting
                    self.controls_ploting = ui.CollapsableFrame(title="Drone control visualization", collapsed=False)

                    with self.controls_ploting:
                        self.plots_container = ui.VStack()

                        self.build_plot_container_content()
                            

    def get_manipulable_prims(self, prim=None):
        # Needed list containing the names of the manipulable prims for the dropdown selector
        manipulable_prims = []

        # First iteration
        if prim is None:
            stage = get_current_stage()
            prim = stage.GetPseudoRoot()

            self.manipulable_prims.clear()

        # Check current prim
        if prim.GetAttribute("NavSim:Manipulable").IsValid() and prim.GetAttribute("NavSim:Manipulable").Get():
            manipulable_prims.append(prim.GetName())
            self.manipulable_prims.append(prim)
        
        else:
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
            
            with ui.HStack():
                ui.Label("Plots appearance", alignment=ui.Alignment.LEFT)
                self.plots_appearance_combo_box = ui.ComboBox(self.plots_appearance, "Rows", "Group", "Stack")
                self.plots_appearance_combo_box.model.add_item_changed_fn(self.change_plot_distribution)

            match self.plots_appearance:
                case 0:
                    ui.Spacer(height=6)

                case 1:
                    pass

                case 2:
                    ui.Spacer(height=6)

            with ui.HStack():
                ui.Label("Track position", alignment=ui.Alignment.LEFT)
                self.track_checkbox = ui.CheckBox()

            match self.plots_appearance:
                case 0:
                    ui.Spacer(height=14)

                case 1:
                    pass

                case 2:
                    ui.Spacer(height=9)

            ui.Separator(height=10)

            with ui.HStack(spacing=5):
                # Control buttons
                ui.Button("START", clicked_fn=self.start_update, height=5)
                ui.Button("STOP", clicked_fn=self.stop_update, height=5)
                ui.Button("RESET", clicked_fn=self.reset_plot, height=5)

            match self.plots_appearance:
                case 0:
                    self.first_way()

                case 1:
                    self.second_way()

                case 2:
                    self.third_way()


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

            # Check if prim's pos must be tracked
            if self.track_checkbox.model.get_value_as_bool():
                asyncio.ensure_future(self.track_prim_pos(drone))

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

    
    async def track_prim_pos(self, drone):
        # Create the matplotlib figure and the corresponding plot
        plt_fig = plt.figure()
        track_plot = plt_fig.add_subplot(111, projection="3d")

        # Indicate the axes name
        track_plot.set_xlabel("X AXES")
        track_plot.set_ylabel("Y AXES")
        track_plot.set_zlabel("Z AXES")

        # Write a title for the plot
        track_plot.set_title("Drone Position")

        # Stablish the initial limits
        track_plot.set_xlim3d(-1, 1)
        track_plot.set_ylim3d(-1, 1)
        track_plot.set_zlim3d(-1, 1)

        # Create the position lists to show over time
        x_pos_track = []
        y_pos_track = []
        z_pos_track = []

        # Store initial position
        drone_pos = drone.GetAttribute("xformOp:translate").Get()
        x_pos_track.append(drone_pos[0])
        y_pos_track.append(drone_pos[1])
        z_pos_track.append(drone_pos[2])
        

        line, = track_plot.plot(x_pos_track, y_pos_track, z_pos_track)

        while not self.stop_update_plot:
            # Get the current position
            drone_pos = drone.GetAttribute("xformOp:translate").Get()

            # If position (all coordinates) is changed
            if drone_pos[0] != x_pos_track[-1] and drone_pos[1] != y_pos_track[-1] and drone_pos[2] != z_pos_track[-1]:
                # Add the position components to the corresponding list
                x_pos_track.append(drone_pos[0])
                y_pos_track.append(drone_pos[1])
                z_pos_track.append(drone_pos[2])

                # Update plot data
                line.set_data(x_pos_track, y_pos_track)
                line.set_3d_properties(z_pos_track)

                # Update plot limits
                track_plot_lims = self.get_matplotlib_plot_limits(track_plot)
                new_limits = self.update_track_plot_lims(track_plot_lims, drone_pos)

                track_plot.set_xlim3d(*new_limits[0])
                track_plot.set_ylim3d(*new_limits[1])
                track_plot.set_zlim3d(*new_limits[2])

                plt.draw()
                plt.pause(0.01)
            
            await asyncio.sleep(0.2)

        # TODO
        # Once the tracking has finished, we can save the plot, present it versus time, etc
        
        track_plot.cla()
        plt.close(plt_fig)

    
    def get_matplotlib_plot_limits(self, plot):
        try:
            z_lim = plot.get_zlim3d()
            x_lim = plot.get_xlim3d()
            y_lim = plot.get_ylim3d()

            return [x_lim, y_lim, z_lim]
        except:
            x_lim = plot.get_xlim3d()
            y_lim = plot.get_ylim3d()

            return (x_lim, y_lim)
        

    def update_track_plot_lims(self, track_plot_lims, new_pos):
        if new_pos[0] < track_plot_lims[0][0]:
            track_plot_lims[0] = [new_pos[0], track_plot_lims[0][1]]

        if new_pos[0] > track_plot_lims[0][1]:
            track_plot_lims[0] = [track_plot_lims[0][0], new_pos[0]]

        if new_pos[1] < track_plot_lims[1][0]:
            track_plot_lims[1] = [new_pos[1], track_plot_lims[1][1]]

        if new_pos[1] > track_plot_lims[1][1]:
            track_plot_lims[1] = [track_plot_lims[1][0], new_pos[1]]

        if new_pos[2] < track_plot_lims[2][0]:
            track_plot_lims[2] = [new_pos[2], track_plot_lims[2][0]]

        if new_pos[2] > track_plot_lims[2][1]:
            track_plot_lims[2] = [track_plot_lims[2][0], new_pos[2]]

        return track_plot_lims

    def first_way(self):
        self.TRDW = False

        ui.Spacer(height=16)

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

        ui.Spacer(height=14)

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
            
        ui.Spacer(height=20)

        # Z angular vel
        self.z_angular_vel_label = ui.Label("Z angular velocity (rad/s)", alignment=ui.Alignment.CENTER)
        self.z_max_avl = ui.Label("1.0")
        self.z_av_plot = ui.Plot(ui.Type.LINE, -1, 1, *self.z_av_plot_data, height=50, alignment=ui.Alignment.CENTER, style={"color": cl("#4C73E2")})
        self.z_min_avl = ui.Label("-1.0")


    def third_way(self):
        self.TRDW = True

        ui.Spacer(height=14)

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