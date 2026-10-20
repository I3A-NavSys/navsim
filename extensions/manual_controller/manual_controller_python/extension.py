import asyncio
from pathlib import Path
import sys
from typing import List

import carb.events
try:
    import omni.ui as ui
except ImportError:
    from isaacsim.gui.components.ui_utils import ui

import omni.ext
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd

# Asegurar que la raíz del proyecto está en sys.path para importar dialecto MAVLink
_REPO_ROOT = str(Path(__file__).resolve().parents[3])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from .controller_logic import ControllerLogic

UAV_PRIM_PATTERNS = [
    "/World/*/*/*/UAV_*",
    "/World/*/*/UAV_*",
    "/World/*/UAV_*",
    "/World/UAV_*",
]


class ManualController(omni.ext.IExt):
    """
    Extensión Manual Controller de NavSim para Isaac Sim.
    Permite pilotar drones de la flota mediante joystick o teclado, transmitiendo
    comandos NAVSIM_MANUAL_CONTROL por MAVLink v2 al motor de físicas vectorizado.
    """

    def on_startup(self, ext_id: str) -> None:
        self.init_vars()
        self.build_ui()
        self.refresh_uav_list()

    def on_shutdown(self) -> None:
        self.stop_control()
        if hasattr(self, "manual_control") and self.manual_control:
            self.manual_control.destroy_camera()

        if hasattr(self, "window") and self.window:
            try:
                self.window.destroy()
            except Exception:
                pass
            self.window = None

        self.on_physics_step_sub = None
        self.on_stop_sub = None
        self.plots_update_sub = None
        self.on_stage_event_sub = None

    def init_vars(self) -> None:
        self.uav_paths: List[str] = []
        self.selected_uav_idx = 0
        self.selected_uav_path = ""

        self.linear_vel_limit = 2.0
        self.ang_vel_limit = 1.0
        self.is_controlling = False
        self.stop_plots_update = True

        self.manual_control = ControllerLogic(base_rx_port=14550)

        # Historial para gráficas en tiempo real
        self.plot_len = 50
        self.x_lv_data = [0.0] * self.plot_len
        self.y_lv_data = [0.0] * self.plot_len
        self.z_lv_data = [0.0] * self.plot_len
        self.z_av_data = [0.0] * self.plot_len

        # Suscripción a eventos de parada en la simulación
        self.timeline = omni.timeline.get_timeline_interface()
        timeline_stream = self.timeline.get_timeline_event_stream()
        self.on_stop_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop
        )

        # Suscripción a eventos de apertura/recarga de escenario USD
        usd_context = omni.usd.get_context()
        self.on_stage_event_sub = (
            usd_context.get_stage_event_stream().create_subscription_to_pop(
                self.on_stage_event
            )
        )

    def on_stage_event(self, event) -> None:
        if event.type == int(omni.usd.StageEventType.OPENED):
            self.stop_control()
            self.refresh_uav_list()
        elif event.type == int(omni.usd.StageEventType.CLOSING):
            self.stop_control()

    def on_timeline_stop(self, event) -> None:
        if self.is_controlling:
            self.stop_control()

    # ==========================================================================
    # Detección de UAVs en el Stage
    # ==========================================================================
    def scan_uavs(self) -> List[str]:
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return []

        paths = []
        try:
            from isaacsim.core.utils.prims import find_matching_prim_paths
            for pat in UAV_PRIM_PATTERNS:
                paths.extend(find_matching_prim_paths(pat))
        except Exception:
            pass

        if not paths:
            for prim in stage.Traverse():
                name = prim.GetName()
                if name.startswith("UAV_"):
                    paths.append(str(prim.GetPath()))

        return sorted(list(set(paths)))

    def refresh_uav_list(self) -> None:
        self.uav_paths = self.scan_uavs()
        if not hasattr(self, "uav_combo_frame") or self.uav_combo_frame is None:
            return

        self.uav_combo_frame.clear()
        with self.uav_combo_frame:
            if self.uav_paths:
                items = [p.split("/")[-1] for p in self.uav_paths]
                self.uav_combo = ui.ComboBox(0, *items)
                self.uav_combo.model.add_item_changed_fn(self.on_uav_selected)
                self.selected_uav_idx = 0
                self.selected_uav_path = self.uav_paths[0]
                self.manual_control.set_uav(self.selected_uav_path, self.selected_uav_idx)
                if hasattr(self, "status_label") and self.status_label:
                    self.status_label.text = (
                        f"UAV seleccionado: {self.selected_uav_path} (Puerto RX: 14550)"
                    )
            else:
                self.uav_combo = ui.ComboBox(0, "No se encontraron UAV_* en la escena")
                self.selected_uav_idx = 0
                self.selected_uav_path = ""
                if hasattr(self, "status_label") and self.status_label:
                    self.status_label.text = "Aviso: No se encontraron prims UAV_* en la escena activa."

    # ==========================================================================
    # Interfaz de Usuario (UI)
    # ==========================================================================
    def build_ui(self) -> None:
        self.window = ui.Window(
            "NavSim — Control Manual (Joystick)",
            width=520,
            height=660,
            raster_policy=ui.RasterPolicy.NEVER,
        )

        with self.window.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(spacing=8, height=0):
                    # --- Sección 1: Selección de UAV y Dispositivo ---
                    with ui.CollapsableFrame("1. Objetivo y Dispositivo", collapsed=False):
                        with ui.VStack(spacing=6):
                            with ui.HStack():
                                ui.Label("Seleccionar UAV:", width=120)
                                self.uav_combo_frame = ui.Frame()
                                ui.Button(
                                    "Refrescar",
                                    width=80,
                                    clicked_fn=self.refresh_uav_list,
                                )

                            with ui.HStack():
                                ui.Button(
                                    "Abrir / Enfocar Viewport de Seguimiento",
                                    clicked_fn=self.on_open_camera_viewport,
                                    style={"background_color": 0xFF2A427A},
                                )

                            self.status_label = ui.Label(
                                "Detectando UAVs...",
                                style={"color": 0xFF88DDFF},
                                word_wrap=True,
                            )
                            self.joystick_label = ui.Label(
                                "Joystick: Inicializando...",
                                style={"color": 0xFFAAAAAA},
                            )

                    # --- Sección 2: Conmutación de Modo Operativo ---
                    with ui.CollapsableFrame("2. Modos de Vuelo MAVLink", collapsed=False):
                        with ui.VStack(spacing=6):
                            self.mode_label = ui.Label(
                                "Modo Actual: INACTIVO (Pulsa INICIAR CONTROL)",
                                style={"font_size": 13, "color": 0xFFFFAA33},
                            )
                            with ui.HStack(spacing=6):
                                ui.Button(
                                    "Modo Manual (Joystick)",
                                    clicked_fn=lambda: self.switch_mode(1),
                                    style={"background_color": 0xFF2A5E2A},
                                )
                                ui.Button(
                                    "Modo Auto (Seguir Ruta)",
                                    clicked_fn=lambda: self.switch_mode(2),
                                    style={"background_color": 0xFF2A427A},
                                )
                            with ui.HStack(spacing=6):
                                ui.Button(
                                    "Mantener Posición (Hover)",
                                    clicked_fn=lambda: self.switch_mode(3),
                                    style={"background_color": 0xFF6A5A20},
                                )
                                ui.Button(
                                    "Parada / Desarmar (Disarm)",
                                    clicked_fn=lambda: self.switch_mode(0),
                                    style={"background_color": 0xFF7A2020},
                                )

                    # --- Sección 3: Parámetros de Control ---
                    with ui.CollapsableFrame("3. Ajustes de Velocidad", collapsed=False):
                        with ui.VStack(spacing=6):
                            with ui.HStack():
                                ui.Label("Velocidad Lineal Máx (m/s):", width=200)
                                self.lin_vel_slider = ui.FloatSlider(
                                    min=0.5, max=15.0, step=0.5
                                )
                                self.lin_vel_slider.model.set_value(self.linear_vel_limit)
                                self.lin_vel_slider.model.add_value_changed_fn(
                                    self.on_lin_vel_changed
                                )

                            with ui.HStack():
                                ui.Label("Velocidad Angular Máx (rad/s):", width=200)
                                self.ang_vel_slider = ui.FloatSlider(
                                    min=0.2, max=5.0, step=0.2
                                )
                                self.ang_vel_slider.model.set_value(self.ang_vel_limit)
                                self.ang_vel_slider.model.add_value_changed_fn(
                                    self.on_ang_vel_changed
                                )

                            with ui.HStack():
                                ui.Label("Invertir orientación de cámara:")
                                self.inv_cam_cb = ui.CheckBox(width=0)
                                self.inv_cam_cb.model.add_value_changed_fn(
                                    lambda m: setattr(
                                        self.manual_control,
                                        "invert_camera_movement",
                                        m.get_value_as_bool(),
                                    )
                                )

                            with ui.HStack():
                                ui.Label("Invertir sentido de guiñada (Yaw):")
                                self.inv_yaw_cb = ui.CheckBox(width=0)
                                self.inv_yaw_cb.model.set_value(False)
                                self.inv_yaw_cb.model.add_value_changed_fn(
                                    lambda m: setattr(
                                        self.manual_control,
                                        "invert_yaw",
                                        m.get_value_as_bool(),
                                    )
                                )

                            with ui.HStack():
                                ui.Label("Habilitar teclado (WASD / Flechas):")
                                self.keyboard_cb = ui.CheckBox(width=0)
                                self.keyboard_cb.model.set_value(True)
                                self.manual_control.use_keyboard = True
                                self.keyboard_cb.model.add_value_changed_fn(
                                    self.on_keyboard_toggle
                                )

                    # --- Sección 4: Visualización de Consignas ---
                    with ui.CollapsableFrame("4. Gráficas de Consigna en Tiempo Real", collapsed=False):
                        with ui.VStack(spacing=4):
                            ui.Label("Vx (Cuerpo - Adelante / Atrás) [m/s]:")
                            self.plot_vx = ui.Plot(
                                ui.Type.LINE,
                                -self.linear_vel_limit,
                                self.linear_vel_limit,
                                *self.x_lv_data,
                                height=45,
                                style={"color": 0xFFE04040},
                            )

                            ui.Label("Vy (Cuerpo - Izquierda / Derecha) [m/s]:")
                            self.plot_vy = ui.Plot(
                                ui.Type.LINE,
                                -self.linear_vel_limit,
                                self.linear_vel_limit,
                                *self.y_lv_data,
                                height=45,
                                style={"color": 0xFF40C040},
                            )

                            ui.Label("Vz (Vertical - Arriba / Abajo) [m/s]:")
                            self.plot_vz = ui.Plot(
                                ui.Type.LINE,
                                -self.linear_vel_limit,
                                self.linear_vel_limit,
                                *self.z_lv_data,
                                height=45,
                                style={"color": 0xFF4080E0},
                            )

                            ui.Label("Yaw Rate Wz (Rotación Guiñada) [rad/s]:")
                            self.plot_wz = ui.Plot(
                                ui.Type.LINE,
                                -self.ang_vel_limit,
                                self.ang_vel_limit,
                                *self.z_av_data,
                                height=45,
                                style={"color": 0xFFE0A020},
                            )

                    # --- Botón Principal de Inicio / Parada ---
                    ui.Spacer(height=4)
                    self.start_stop_button = ui.Button(
                        "INICIAR CONTROL MANUAL",
                        height=40,
                        clicked_fn=self.toggle_control,
                        style={"background_color": 0xFF357A35, "font_size": 14},
                    )

    # ==========================================================================
    # Lógica de Interacción
    # ==========================================================================
    def on_open_camera_viewport(self) -> None:
        if not self.selected_uav_path:
            self.refresh_uav_list()
        if self.selected_uav_path:
            self.manual_control.setup_camera_for_uav(self.selected_uav_path)
            self.manual_control.open_or_update_viewport()

    def on_uav_selected(self, model, item=None) -> None:
        val = 0
        try:
            val_model = model.get_item_value_model()
            if hasattr(val_model, "as_int"):
                val = val_model.as_int
            elif hasattr(val_model, "get_value_as_int"):
                val = val_model.get_value_as_int()
        except Exception:
            try:
                val = model.as_int
            except Exception:
                pass

        if 0 <= val < len(self.uav_paths):
            self.selected_uav_idx = val
            self.selected_uav_path = self.uav_paths[val]
            if hasattr(self, "status_label") and self.status_label:
                self.status_label.text = (
                    f"UAV seleccionado: {self.selected_uav_path} (Puerto RX: {14550 + val})"
                )
            self.manual_control.set_uav(self.selected_uav_path, self.selected_uav_idx)

    def on_lin_vel_changed(self, model) -> None:
        self.linear_vel_limit = model.get_value_as_float()
        self.manual_control.linear_vel_limit = self.linear_vel_limit
        if hasattr(self, "plot_vx"):
            self.plot_vx.set_range(-self.linear_vel_limit, self.linear_vel_limit)
            self.plot_vy.set_range(-self.linear_vel_limit, self.linear_vel_limit)
            self.plot_vz.set_range(-self.linear_vel_limit, self.linear_vel_limit)

    def on_ang_vel_changed(self, model) -> None:
        self.ang_vel_limit = model.get_value_as_float()
        self.manual_control.ang_vel_limit = self.ang_vel_limit
        if hasattr(self, "plot_wz"):
            self.plot_wz.set_range(-self.ang_vel_limit, self.ang_vel_limit)

    def on_keyboard_toggle(self, model) -> None:
        enabled = model.get_value_as_bool()
        self.manual_control.use_keyboard = enabled
        if self.is_controlling:
            if enabled:
                self.manual_control.keyboard.start()
            else:
                self.manual_control.keyboard.stop()

    def switch_mode(self, mode: int) -> None:
        mode_names = {
            0: "DESARMADO / PARADA (IDLE)",
            1: "MANUAL (Joystick Activo)",
            2: "AUTOMÁTICO (Siguiendo Ruta)",
            3: "HOVER (Mantenimiento de Posición)",
        }
        self.manual_control.set_mode(mode)
        name = mode_names.get(mode, "DESCONOCIDO")
        color = 0xFF40DD40 if mode == 1 else (0xFF4090FF if mode == 2 else 0xFFFFAA33)
        self.mode_label.text = f"Modo Actual: {name}"
        self.mode_label.style = {"color": color}

    def toggle_control(self) -> None:
        if self.is_controlling:
            self.stop_control()
        else:
            self.start_control()

    def start_control(self) -> None:
        if not self.uav_paths:
            self.refresh_uav_list()
        if not self.selected_uav_path:
            self.status_label.text = "Error: Selecciona un UAV válido antes de iniciar."
            return

        self.is_controlling = True
        self.stop_plots_update = False
        self.start_stop_button.text = "DETENER CONTROL"
        self.start_stop_button.style = {"background_color": 0xFF8A2525}

        self.manual_control.start(self.selected_uav_path, self.selected_uav_idx)
        self.switch_mode(1)

        # Iniciar actualización de gráficas
        asyncio.ensure_future(self.update_plots_loop())

    def stop_control(self) -> None:
        self.is_controlling = False
        self.stop_plots_update = True
        if hasattr(self, "start_stop_button"):
            self.start_stop_button.text = "INICIAR CONTROL MANUAL"
            self.start_stop_button.style = {"background_color": 0xFF357A35}

        self.manual_control.stop()
        if hasattr(self, "mode_label"):
            self.mode_label.text = "Modo Actual: DETENIDO"
            self.mode_label.style = {"color": 0xFFAAAAAA}

    async def update_plots_loop(self) -> None:
        while not self.stop_plots_update:
            # Actualizar etiqueta de joystick
            joy_name = self.manual_control.joystick.joystick_name
            self.joystick_label.text = f"Dispositivo Joystick: {joy_name}"

            # Obtener consignas enviadas
            vx, vy, vz = self.manual_control.latest_command_vel
            wz = self.manual_control.latest_command_yaw

            self.x_lv_data.append(float(vx))
            self.y_lv_data.append(float(vy))
            self.z_lv_data.append(float(vz))
            self.z_av_data.append(float(wz))

            if len(self.x_lv_data) > self.plot_len:
                self.x_lv_data.pop(0)
            if len(self.y_lv_data) > self.plot_len:
                self.y_lv_data.pop(0)
            if len(self.z_lv_data) > self.plot_len:
                self.z_lv_data.pop(0)
            if len(self.z_av_data) > self.plot_len:
                self.z_av_data.pop(0)

            try:
                self.plot_vx.set_data(*self.x_lv_data)
                self.plot_vy.set_data(*self.y_lv_data)
                self.plot_vz.set_data(*self.z_lv_data)
                self.plot_wz.set_data(*self.z_av_data)
            except Exception:
                pass

            await asyncio.sleep(0.05)