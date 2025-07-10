import sys, os
import asyncio
import torch

import carb
import omni.ext
import omni.ui as ui
import omni.usd
from omni.kit.viewport.window import ViewportWindow
from omni.isaac.core.prims import RigidPrimView
import omni.kit.app

    
from .controller import Controller
from fleet.uav_matrix_control_quadcopter import UAVcontrol

project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)


class MultiManualController(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.create_vars()
        self.build_ui()

    def on_shutdown(self):
        self.on_timeline_stop(None)

    def on_physics_step(self, step_size:int):
        if self.is_running:
            self.current_time += step_size
            self.uav_control.update(self.current_time, self.uavs, step_size)
            self.controller.current_time = self.current_time

    def on_timeline_stop(self, event):
        if self.is_running:
            self.controller.stop()
            self.is_running = False
            self.current_time = 0
            self.rigid_prim_view = None
            self.uav_control = None

    def on_timeline_play(self, event):
        if not self.is_running and self.is_controlling:
            self.uavs = {}
            self.torch_device = "cuda:0" if torch.cuda.is_available() else "cpu"

            try:
                self.rigid_prim_view = RigidPrimView(["/World/*/UAV_*",])
                self.rigid_prim_view.initialize()
            except Exception as e:
                carb.log_warn(f"[REMOTE COMMAND ext] Error initializing RigidPrimView: {e}")
                return

            self.init_uavs()
            self.uav_control = UAVcontrol(
                self.rigid_prim_view, 
                self.torch_device, 
                self.uavs, 
                "", 
                self.event_stream
            )
            
            self.is_running = True
            self.controller.start(self.uav_control)

    def init_uavs(self):
        pos, _ = self.rigid_prim_view.get_world_poses(indices=range(self.rigid_prim_view.count))

        for i in range(self.rigid_prim_view.count):
            uav_id = f"UAV_{i}"
            uav_state = "idle"
            uav_time = 0
            uav_pos = pos[i]
            uav_flightplan = None

            self.uavs[uav_id] = {
                "id": uav_id,
                "state": uav_state,
                "time": uav_time,
                "pos": uav_pos,
                "flightplan": uav_flightplan,
                "request": None
            }

    def create_vars(self):
        pass
        self.rigid_prim_view = None
        self.uav_control = None
        self.current_time = 0
        self.uavs = {}

        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(
            self.on_physics_step, 
            True, 
            0
        )
        
        self.controller = Controller()
        self.max_joysticks = 4
        self.is_running = False
        self.is_checking = False
        self.is_controlling = False
        self.joystick_checkers = []

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
                                                       clicked_fn=self.add_viewports_nvidia_way)
                
                # Clean viewports button
                self.clear_viewports_button = ui.Button("CLEAR VIEWPORTS", height=40, style= add_viewport_button_style,
                                                        clicked_fn=self.clear_viewports)

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
        self.controller.joysticks.start()

        while self.is_checking:
            joystick_inputs = self.controller.check_joysticks()

            # Batch UI updates to reduce GPU calls
            await self.update_joystick_ui(joystick_inputs)
            await asyncio.sleep(0.2)  # Increase delay to 200ms

    async def update_joystick_ui(self, joystick_inputs):
        """Batch UI updates to reduce GPU stress"""
        style_active = {"background_color": ui.color("#6f9523"), "border_color": ui.color.white, 
                    "border_width": 0, "border_radius": 5}
        style_inactive = {"background_color": ui.color("#db8f26"), "border_color": ui.color.white, 
                        "border_width": 0, "border_radius": 5}
        
        for i in range(min(len(joystick_inputs), self.max_joysticks)):
            input_vals = joystick_inputs[i]
            x_value, y_value, z_value = input_vals[0], input_vals[1], input_vals[2]
            
            # Update all UI elements for this joystick at once
            checkers = self.joystick_checkers[i]
            
            # X-axis
            checkers[0][0].set_style(style_active if x_value < 0 else style_inactive)
            checkers[0][1].set_style(style_active if x_value > 0 else style_inactive)
            
            # Y-axis  
            checkers[1][0].set_style(style_active if y_value < 0 else style_inactive)
            checkers[1][1].set_style(style_active if y_value > 0 else style_inactive)
            
            # Z-axis
            checkers[2][0].set_style(style_active if z_value < 0 else style_inactive)
            checkers[2][1].set_style(style_active if z_value > 0 else style_inactive)

    def stop_checking(self):
        self.controller.joysticks.stop()

        style={"background_color": ui.color("#db8f26"), "border_color": ui.color.white, "border_width": 0, 
                   "border_radius": 5}
        
        for checker in self.joystick_checkers:
            for axis in checker:
                for rectangle in axis:
                    rectangle.set_style(style)

    def add_viewports(self):
        viewport_width = ui.Workspace.get_main_window_width()/4
        viewport_height = ui.Workspace.get_main_window_height()/3
        
        async def create_viewports_safely():
            """Create viewports one by one to avoid GPU overload"""
            try:
                for i in range(1):
                    window_name = f"User {i + 1}"
                    
                    # Check if viewport already exists
                    existing_viewports = ViewportWindow.get_viewport_window_instances()
                    if any(vp.name == window_name for vp in existing_viewports):
                        print(f"Viewport '{window_name}' already exists, skipping...")
                        continue
                    
                    # Create viewport with error handling
                    try:
                        new_viewport = ViewportWindow.ViewportWindow(
                            name=window_name, 
                            width=int(viewport_width), 
                            height=int(viewport_height)
                        )
                        
                        # Optional: Configure viewport settings to reduce GPU load
                        if hasattr(new_viewport, 'viewport_widget') and new_viewport.viewport_widget:
                            # Reduce rendering quality for additional viewports
                            pass
                        
                        print(f"Successfully created viewport: {window_name}")
                        
                        # Wait between viewport creations to prevent GPU overload
                        await asyncio.sleep(0.5)
                        
                    except Exception as e:
                        carb.log_error(f"Failed to create viewport '{window_name}': {e}")
                        break  # Stop creating more viewports if one fails
                        
            except Exception as e:
                carb.log_error(f"Error in viewport creation process: {e}")
        
        # Start the async viewport creation
        asyncio.ensure_future(create_viewports_safely())

    def clear_viewports(self):
        """Safely clear viewports one by one"""
        async def clear_viewports_safely():
            viewport_instances = ViewportWindow.get_instances()
            
            for viewport_window in viewport_instances:
                if viewport_window.name == "Viewport":  # Keep main viewport
                    continue
                    
                try:
                    if hasattr(viewport_window, 'viewport_widget') and viewport_window.viewport_widget:
                        viewport_window.viewport_widget.destroy()
                    viewport_window.destroy()
                    # print(f"Destroyed viewport: {viewport_window.name}")
                    
                    # Small delay between destructions
                    await asyncio.sleep(0.1)
                    
                except Exception as e:
                    carb.log_error(f"Error destroying viewport {viewport_window.name}: {e}")
        
        asyncio.ensure_future(clear_viewports_safely())

    def add_viewports_nvidia_way(self):
        """NVIDIA's recommended approach for multiple viewports"""
        
        # Calculate safe viewport dimensions
        window_width = ui.Workspace.get_main_window_width()
        window_height = ui.Workspace.get_main_window_height()
        
        # Create smaller viewports to reduce GPU load
        viewport_width = int(window_width / 3)  # Smaller than /4
        viewport_height = int(window_height / 4)  # Smaller than /3
        
        async def create_viewports_safely():
            created_viewports = []
            
            try:
                viewports = ViewportWindow.get_instances()
                viewports_names = [vp.name for vp in viewports]
                for i in range(4):
                    viewport_name = f"Camera_{i+1}"
                    
                    # Check if viewport already exists
                    if viewport_name in viewports_names:
                        carb.log_warn(f"Viewport {viewport_name} already exists")
                        continue
                    
                    # Create viewport with NVIDIA's method
                    viewport_window = ViewportWindow(
                        name=viewport_name,
                        width=viewport_width,
                        height=viewport_height,
                        dockPreference=ui.DockPreference.DISABLED  # Don't auto-dock
                    )
                    
                    # Configure viewport settings to reduce GPU load
                    if viewport_window.viewport_api:
                        # Reduce rendering quality
                        viewport_window.viewport_api.set_texture_resolution((512, 512))  # Lower resolution
                        
                        # Disable expensive features
                        # render_settings = viewport_window.viewport_api.get_render_settings()
                        # if render_settings:
                        #     render_settings.set_anti_aliasing_mode(0)  # Disable AA
                        #     render_settings.set_ambient_occlusion_enabled(False)
                        #     render_settings.set_screen_space_reflection_enabled(False)
                    
                    created_viewports.append(viewport_window)
                    carb.log_info(f"Created viewport: {viewport_name}")
                    
                    # Critical: Wait for GPU to process
                    await self.wait_for_gpu_ready()
                    
            except Exception as e:
                carb.log_error(f"Failed to create viewports: {e}")
                # Clean up any created viewports
                for vp in created_viewports:
                    try:
                        vp.destroy()
                    except:
                        pass
        
        # Execute async
        asyncio.ensure_future(create_viewports_safely())

    async def wait_for_gpu_ready(self):
        """Wait for GPU to be ready for next operation"""
        # Force a frame update
        await omni.kit.app.get_app().next_update_async()
        # Additional small delay
        await asyncio.sleep(0.2)