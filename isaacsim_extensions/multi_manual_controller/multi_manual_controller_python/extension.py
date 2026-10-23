import asyncio
import carb.settings
import omni.ext
import omni.kit.app
import omni.physx
import omni.timeline
import omni.ui as ui
from omni.kit.viewport.window import ViewportWindow

try:
    from isaacsim.core.prims import RigidPrim
except ImportError:
    try:
        from isaacsim.core.experimental.prims import RigidPrim
    except ImportError:
        RigidPrim = None

from .controller import Controller
from .uav_control import UAVControl


class MultiManualController(omni.ext.IExt):
    """Extension allowing multi-user manual joystick teleoperation for simulated UAV swarms."""

    def on_startup(self, ext_id):
        self.initialize_variables()
        self.build_ui()

    def on_shutdown(self):
        """Releases all callbacks, UI elements, async tasks, and viewports cleanly on shutdown."""
        # Stop simulation control if currently running
        self.on_timeline_stop(None)

        # Stop joystick checking task if active
        if self.is_checking:
            self.is_checking = False
            self.stop_checking()

        if self._checking_task and not self._checking_task.done():
            self._checking_task.cancel()
            self._checking_task = None

        # Unsubscribe physics step callback
        if self.on_physics_step_sub is not None:
            self.on_physics_step_sub = None

        # Unsubscribe timeline event callbacks
        if self.timeline_start_event_sub is not None:
            self.timeline_start_event_sub = None
        if self.timeline_stop_event_sub is not None:
            self.timeline_stop_event_sub = None

        # Clean up viewports created by this extension
        for vp in self.created_viewports:
            try:
                if hasattr(vp, "viewport_widget") and vp.viewport_widget:
                    vp.viewport_widget.destroy()
                vp.destroy()
            except Exception:
                pass
        self.created_viewports.clear()

        # Destroy UI window
        if self.window:
            self.window.destroy()
            self.window = None

    def on_physics_step(self, step_size: float):
        """Physics step event callback running on simulation update."""
        if not self.is_simulation_running or self.uav_control is None:
            return

        # Poll joystick inputs mapped to UAV buffer indices
        cmd_local_linear_vel, cmd_yaw_rotation, amount_joysticks = self.controller.control(
            self.uav_physics_indices
        )

        if amount_joysticks > 0:
            self.uav_control.update_command(
                self.uav_physics_indices[:amount_joysticks],
                cmd_local_linear_vel,
                cmd_yaw_rotation,
                step_size,
            )
        else:
            # Keep telemetry and watchdog active while clearing residual forces
            self.uav_control.imu()
            self.uav_control.reset_dynamics([])

    def on_timeline_stop(self, event):
        """Handles simulation stop event."""
        if self.is_simulation_running:
            self.controller.stop()
            self.is_simulation_running = False
            self.current_time = 0.0
            self.rigid_prim_view = None
            self.uav_control = None
            self.uav_physics_indices.clear()
            self.uav_ids_to_physics_buffer.clear()

    def on_timeline_play(self, event):
        """Initializes RigidPrim and UAV controller when timeline starts playing."""
        if not self.is_simulation_running and self.is_controlling:
            # Configure RTX transform history storage to accommodate multi-viewport rendering
            if self.carb_settings.get("/rtx/scenedb/maxHistoryTransformCount") < 256:
                self.carb_settings.set("/rtx/scenedb/maxHistoryTransformCount", 256)

            # Cap max depenetration velocity in physics engine
            self.carb_settings.set("/physics/maxDepenetrationVelocity", 8.0)
                
            self.uav_physics_indices.clear()
            self.uav_ids_to_physics_buffer.clear()

            # Initialize RigidPrim with optimized flags (disabling expensive contact sensor preparation)
            try:
                self.rigid_prim_view = RigidPrim(
                    prim_paths_expr="/World/UAVs/UAV_*",
                    reset_xform_properties=False,
                    track_contact_forces=False,
                    prepare_contact_sensors=False,
                )
                self.rigid_prim_view.initialize()
            except Exception as e:
                carb.log_error(f"[MultiManualController] Failed to initialize RigidPrim view: {e}")
                return

            if self.rigid_prim_view.count == 0:
                carb.log_warn("[MultiManualController] No UAV prims found at '/World/UAVs/UAV_*'.")
                return

            self.relate_uav_ids_to_physics_buffer()
            self.uav_control = UAVControl(self.rigid_prim_view)
            self.is_simulation_running = True
            self.controller.start(self.uav_control)

    def relate_uav_ids_to_physics_buffer(self):
        """Maps USD prim attributes to physics buffer array indices."""
        self.uav_ids_to_physics_buffer.clear()
        self.uav_physics_indices.clear()

        uavs = self.rigid_prim_view.prims
        for idx, uav in enumerate(uavs):
            op_attr = uav.GetAttribute("NavSim:operator_id")
            id_attr = uav.GetAttribute("NavSim:id")

            uav_operator_id = op_attr.Get() if (op_attr and op_attr.IsValid()) else "default"
            uav_id = id_attr.Get() if (id_attr and id_attr.IsValid()) else uav.GetName()

            if uav_operator_id not in self.uav_ids_to_physics_buffer:
                self.uav_ids_to_physics_buffer[uav_operator_id] = {}

            self.uav_ids_to_physics_buffer[uav_operator_id][uav_id] = idx
            self.uav_physics_indices.append(idx)

    def initialize_variables(self):
        """Initializes state variables and subscriptions."""
        self.is_simulation_running = False
        self.rigid_prim_view = None
        self.uav_control = None
        self.uav_ids_to_physics_buffer = {}
        self.uav_physics_indices = []
        self.created_viewports = []

        # Joysticks and controller
        self.controller = Controller()
        self.max_joysticks = 4
        self.is_checking = False
        self.is_controlling = False
        self.joystick_checkers = []
        self._checking_task = None

        # Pre-cached UI style dictionaries to prevent re-allocation during UI polling
        self.style_active = {
            "background_color": ui.color("#6f9523"),
            "border_color": ui.color.white,
            "border_width": 0,
            "border_radius": 5,
        }
        self.style_inactive = {
            "background_color": ui.color("#db8f26"),
            "border_color": ui.color.white,
            "border_width": 0,
            "border_radius": 5,
        }

        # Track previous UI indicator states (4 joysticks x 3 axes x 2 directions) to skip unchanged styles
        self._prev_checker_states = [[[False, False], [False, False], [False, False]] for _ in range(self.max_joysticks)]

        # PhysX step callback
        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(
            fn=self.on_physics_step,
            pre_step=True,
            order=10,
        )

        # Timeline callbacks
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline_start_event_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self.on_timeline_play
        )
        self.timeline_stop_event_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop
        )

        # Carb settings
        self.carb_settings = carb.settings.get_settings()

    def ui_build_player_checker(self, player_id, checker_container_style, checker_style, username_container_style):
        """Constructs UI indicators for a single joystick."""
        user_checker = []
        x_checker = []
        y_checker = []
        z_checker = []

        with ui.VStack(spacing=5):
            with ui.ZStack():
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

            with ui.ZStack():
                ui.Rectangle(width=75, height=20, style=username_container_style)
                with ui.Frame(width=75, height=20):
                    ui.Label(f"Jugador {player_id + 1}", alignment=ui.Alignment.CENTER)

            user_checker.append(x_checker)
            user_checker.append(y_checker)
            user_checker.append(z_checker)
            self.joystick_checkers.append(user_checker)

    def build_ui(self):
        """Constructs the extension UI window."""
        toggle_checking_button_style = {
            "background_color": ui.color("#db8f26"),
            "border_radius": 5,
            ":hovered": {"background_color": ui.color("#939393")},
        }

        checker_container_style = {
            "background_color": ui.color("#787878"),
            "border_color": ui.color.white,
            "border_width": 1,
            "border_radius": 5,
        }

        checker_style = {
            "background_color": ui.color("#db8f26"),
            "border_color": ui.color.white,
            "border_width": 0,
            "border_radius": 5,
        }

        username_container_style = {
            "background_color": ui.color("#5b5b5b"),
            "border_color": ui.color.white,
            "border_width": 1,
            "border_radius": 5,
        }

        add_viewport_button_style = {"border_radius": 5}

        toggle_control_button_style = {
            "background_color": ui.color("#952323"),
            "border_radius": 5,
            ":hovered": {"background_color": ui.color("#939393")},
        }

        self.window = ui.Window(
            "NavSim - Multi Manual Controller",
            width=0,
            height=0,
            raster_policy=ui.RasterPolicy.NEVER,
        )

        with self.window.frame:
            with ui.HStack(width=0, height=0, spacing=10):
                with ui.VStack(height=0):
                    self.toggle_checking_button = ui.Button(
                        text="CHECK",
                        height=50,
                        width=160,
                        style=toggle_checking_button_style,
                        clicked_fn=self.toggle_checking,
                    )

                    ui.Spacer(height=10)

                    with ui.HStack(width=0, spacing=10):
                        self.ui_build_player_checker(0, checker_container_style, checker_style, username_container_style)
                        self.ui_build_player_checker(1, checker_container_style, checker_style, username_container_style)

                    ui.Spacer(height=10)

                    with ui.HStack(width=0, spacing=10):
                        self.ui_build_player_checker(2, checker_container_style, checker_style, username_container_style)
                        self.ui_build_player_checker(3, checker_container_style, checker_style, username_container_style)

                    ui.Spacer(height=10)
                    ui.Separator(width=160, style={"color": ui.color("#939393")})
                    ui.Spacer(height=10)

                    self.add_viewports_button = ui.Button(
                        "ADD VIEWPORT",
                        height=40,
                        width=160,
                        style=add_viewport_button_style,
                        clicked_fn=self.add_viewports_nvidia_way,
                    )

                    self.clear_viewports_button = ui.Button(
                        "CLEAR VIEWPORTS",
                        height=40,
                        width=160,
                        style=add_viewport_button_style,
                        clicked_fn=self.clear_viewports,
                    )

                    self.toggle_control_button = ui.Button(
                        text="NO CONTROL",
                        height=40,
                        width=160,
                        style=toggle_control_button_style,
                        clicked_fn=self.toggle_control,
                    )

    def toggle_control(self):
        """Toggles active manual control state."""
        if self.is_controlling:
            style = {
                "background_color": ui.color("#952323"),
                "border_radius": 5,
                ":hovered": {"background_color": ui.color("#939393")},
            }
            self.toggle_control_button.set_style(style)
            self.toggle_control_button.text = "NO CONTROL"
            self.is_controlling = False
        else:
            style = {
                "background_color": ui.color("#6f9523"),
                "border_radius": 5,
                ":hovered": {"background_color": ui.color("#939393")},
            }
            self.toggle_control_button.set_style(style)
            self.toggle_control_button.text = "CONTROL"
            self.is_controlling = True

    def toggle_checking(self):
        """Toggles real-time joystick hardware state visualizer."""
        if self.is_checking:
            style = {
                "background_color": ui.color("#db8f26"),
                "border_radius": 5,
                ":hovered": {"background_color": ui.color("#939393")},
            }
            self.toggle_checking_button.set_style(style)
            self.toggle_checking_button.text = "CHECK"
            self.is_checking = False
            self.stop_checking()
        else:
            style = {
                "background_color": ui.color("#e14e27"),
                "border_radius": 5,
                ":hovered": {"background_color": ui.color("#939393")},
            }
            self.toggle_checking_button.set_style(style)
            self.toggle_checking_button.text = "CHECKING"
            self.is_checking = True
            self._checking_task = asyncio.ensure_future(self.start_checking())

    async def start_checking(self):
        """Background coroutine to poll joystick axes for UI testing."""
        self.controller.joysticks.start()

        while self.is_checking:
            joystick_inputs = self.controller.check_joysticks()
            self.update_joystick_ui(joystick_inputs)
            await asyncio.sleep(0.2)

    def update_joystick_ui(self, joystick_inputs: dict):
        """Updates indicator rectangles only when state changes occur, avoiding GPU redraw spikes."""
        joy_ids = self.controller.joysticks.joysticks_ids
        num_joysticks = min(len(joy_ids), self.max_joysticks)

        for i in range(num_joysticks):
            joy_id = joy_ids[i]
            input_vals = joystick_inputs.get(joy_id)
            if input_vals is None:
                continue

            x_val, y_val, z_val = input_vals[0], input_vals[1], input_vals[2]
            checkers = self.joystick_checkers[i]
            prev_states = self._prev_checker_states[i]

            # X-axis (-X and +X)
            neg_x = x_val < 0.0
            pos_x = x_val > 0.0
            if prev_states[0][0] != neg_x:
                checkers[0][0].set_style(self.style_active if neg_x else self.style_inactive)
                prev_states[0][0] = neg_x
            if prev_states[0][1] != pos_x:
                checkers[0][1].set_style(self.style_active if pos_x else self.style_inactive)
                prev_states[0][1] = pos_x

            # Y-axis (-Y and +Y)
            neg_y = y_val < 0.0
            pos_y = y_val > 0.0
            if prev_states[1][0] != neg_y:
                checkers[1][0].set_style(self.style_active if neg_y else self.style_inactive)
                prev_states[1][0] = neg_y
            if prev_states[1][1] != pos_y:
                checkers[1][1].set_style(self.style_active if pos_y else self.style_inactive)
                prev_states[1][1] = pos_y

            # Z-axis (-Z and +Z)
            neg_z = z_val < 0.0
            pos_z = z_val > 0.0
            if prev_states[2][0] != neg_z:
                checkers[2][0].set_style(self.style_active if neg_z else self.style_inactive)
                prev_states[2][0] = neg_z
            if prev_states[2][1] != pos_z:
                checkers[2][1].set_style(self.style_active if pos_z else self.style_inactive)
                prev_states[2][1] = pos_z

    def stop_checking(self):
        """Resets joystick polling and restores default UI indicator colors."""
        self.controller.joysticks.stop()

        for i, checker in enumerate(self.joystick_checkers):
            for axis_idx, axis in enumerate(checker):
                for dir_idx, rectangle in enumerate(axis):
                    rectangle.set_style(self.style_inactive)
                    self._prev_checker_states[i][axis_idx][dir_idx] = False

    def clear_viewports(self):
        """Safely destroys only the extra viewports instantiated by this extension."""
        async def clear_viewports_safely():
            for vp in list(self.created_viewports):
                try:
                    if hasattr(vp, "viewport_widget") and vp.viewport_widget:
                        vp.viewport_widget.destroy()
                    vp.destroy()
                    self.created_viewports.remove(vp)
                    await asyncio.sleep(0.05)
                except Exception as e:
                    carb.log_error(f"[MultiManualController] Error destroying viewport: {e}")

        asyncio.ensure_future(clear_viewports_safely())

    def add_viewports_nvidia_way(self):
        """Spawns camera viewports using recommended non-docking configuration."""
        window_width = ui.Workspace.get_main_window_width()
        window_height = ui.Workspace.get_main_window_height()

        viewport_width = max(int(window_width / 3), 320)
        viewport_height = max(int(window_height / 4), 240)

        async def create_viewports_safely():
            existing_names = [vp.name for vp in ViewportWindow.get_instances()]

            for i in range(3):
                viewport_name = f"Camera_{i + 1}"
                if viewport_name in existing_names:
                    carb.log_warn(f"[MultiManualController] Viewport '{viewport_name}' already exists.")
                    continue

                try:
                    viewport_window = ViewportWindow(
                        name=viewport_name,
                        width=viewport_width,
                        height=viewport_height,
                        dockPreference=ui.DockPreference.DISABLED,
                    )

                    if viewport_window.viewport_api:
                        # Cap texture resolution to 512x512 to preserve frame rates
                        viewport_window.viewport_api.set_texture_resolution((512, 512))

                    self.created_viewports.append(viewport_window)
                    carb.log_info(f"[MultiManualController] Created viewport: {viewport_name}")

                    await self.wait_for_gpu_ready()
                except Exception as e:
                    carb.log_error(f"[MultiManualController] Failed to create viewport '{viewport_name}': {e}")
                    break

        asyncio.ensure_future(create_viewports_safely())

    async def wait_for_gpu_ready(self):
        """Allows GPU frame update before next viewport initialization to prevent crashes."""
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(0.1)
