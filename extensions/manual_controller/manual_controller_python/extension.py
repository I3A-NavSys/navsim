import asyncio
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Tuple

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

# Estilos temáticos para los botones de disparo/evento (Triggers puntuales)
STYLE_TRIGGER_MANUAL = {
    "background_color": 0xFF233823,
    "border_color": 0xFF4CAF50,
    "border_width": 1.0,
    "color": 0xFFE0E0E0,
}

STYLE_TRIGGER_HOVER = {
    "background_color": 0xFF382E15,
    "border_color": 0xFFFFB300,
    "border_width": 1.0,
    "color": 0xFFE0E0E0,
}

STYLE_TRIGGER_AUTO = {
    "background_color": 0xFF182838,
    "border_color": 0xFF2196F3,
    "border_width": 1.0,
    "color": 0xFFE0E0E0,
}

STYLE_TRIGGER_DISARM = {
    "background_color": 0xFF381A1A,
    "border_color": 0xFFE53935,
    "border_width": 1.0,
    "color": 0xFFE0E0E0,
}

# Compatibilidad con referencias previas
STYLE_INACTIVE = STYLE_TRIGGER_DISARM
STYLE_ACTIVE_MANUAL = STYLE_TRIGGER_MANUAL
STYLE_ACTIVE_HOVER = STYLE_TRIGGER_HOVER
STYLE_ACTIVE_AUTO = STYLE_TRIGGER_AUTO
STYLE_ACTIVE_DISARM = STYLE_TRIGGER_DISARM
STYLE_ACTIVE_STOP = STYLE_TRIGGER_DISARM

# Definición visual de estados de la máquina de estados del dron (formato ABGR nativo de omni.ui)
# 0=DISARMED, 1=MANUAL, 2=AUTO, 3=HOVER, 4=SYSTEM_FAILURE
STATE_INFO = {
    0: ("DISARMED", 0xFFFF8CBA),       # Morado luminoso / legible (RGB: 186, 140, 255)
    1: ("MANUAL", 0xFF84C781),         # Verde (coincide con botón Manual: 0xFF84C781)
    2: ("AUTO", 0xFF2196F3),           # Anaranjado (coincide con botón Auto: 0xFF2196F3)
    3: ("HOVER", 0xFFFFB300),          # Azul (coincide con botón Hover: 0xFFFFB300)
    4: ("SYSTEM_FAILURE", 0xFF4417FF), # Rojo emergencia (R=255, G=23, B=68)
}

# Estilos para la matriz de 2x6 botones de test de joystick (1 al 12)
JOY_BTN_OFF_STYLE = {
    "background_color": 0xFF2A2A2A,
    "border_color": 0xFF424242,
    "border_width": 1.0,
    "border_radius": 4.0,
    "color": 0xFF9E9E9E,
    "font_size": 12,
}

JOY_BTN_ON_STYLE = {
    "background_color": 0xFF327D2E,  # Verde encendido / activo (R=46, G=125, B=50)
    "border_color": 0xFF6ABB66,      # Borde verde brillante (R=102, G=187, B=106)
    "border_width": 1.5,
    "border_radius": 4.0,
    "color": 0xFFFFFFFF,             # Texto blanco iluminado
    "font_size": 12,
}

# Estilos inspirados fielmente en el Property Window nativo de Isaac Sim
ISAAC_COLLAPSABLE_STYLE = {
    "CollapsableFrame": {
        "background_color": 0xFF343432,
        "secondary_color": 0xFF343432,
        "border_radius": 3.0,
        "border_color": 0x0,
        "border_width": 0,
        "padding": 3,
    },
    "CollapsableFrame:hovered": {
        "secondary_color": 0xFF2A2A28,
    },
    "CollapsableFrame:pressed": {
        "secondary_color": 0xFF20201E,
    },
    "CollapsableFrame.Header": {
        "font_size": 14.5,
        "color": 0xFFDDDDDD,
    },
}

LABEL_STYLE_GREY = {"color": 0xFF9E9E9E}


def build_isaac_header(collapsed: bool, title: str = "", **kwargs):
    """Encabezado de sección con la estética nativa de Isaac Sim Property Window."""
    if collapsed:
        alignment = ui.Alignment.RIGHT_CENTER
        w, h = 5, 7
    else:
        alignment = ui.Alignment.CENTER_BOTTOM
        w, h = 7, 5

    with ui.HStack(spacing=8, height=22):
        with ui.VStack(width=8):
            ui.Spacer()
            ui.Triangle(
                width=w,
                height=h,
                alignment=alignment,
                style={"background_color": 0xFFCCCCCC},
            )
            ui.Spacer()
        ui.Label(str(title), style={"color": 0xFFDDDDDD, "font_size": 14.5})


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
            self.manual_control.destroy()

        if hasattr(self, "window") and self.window:
            try:
                self.window.destroy()
            except Exception:
                pass
            self.window = None

        self.btn_manual = None
        self.btn_hover = None
        self.btn_auto = None
        self.btn_disarm = None
        self.btn_stop = None
        self.port_label = None
        self.state_label = None
        self.joy_device_label = None
        self.joystick_test_buttons = []
        self.app_update_sub = None

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
        self.stop_plots_update = False
        self._last_plot_update_time = 0.0

        self.btn_manual = None
        self.btn_hover = None
        self.btn_auto = None
        self.btn_disarm = None
        self.btn_stop = None
        self.port_label = None
        self.state_label = None
        self.joy_device_label = None
        self.joystick_test_buttons = []
        self._last_joy_btn_states = [False] * 12
        self._joy_btn1_prev = False
        self._joy_btn2_prev = False
        self.uav_modes = {}
        self.last_displayed_mode = -1

        self.manual_control = ControllerLogic(base_rx_port=15000)
        self.manual_control.on_mode_change_requested = self.switch_mode

        # Suscripción a eventos de refresco de interfaz por frame
        try:
            app = omni.kit.app.get_app()
            self.app_update_sub = (
                app.get_update_event_stream().create_subscription_to_pop(
                    self._on_app_update
                )
            )
        except Exception:
            self.app_update_sub = None

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
        self.uav_modes.clear()
        if hasattr(self, "update_state_display"):
            self.update_state_display(0)

    # ==========================================================================
    # Detección de UAVs en el Stage
    # ==========================================================================
    def scan_uavs(self) -> List[str]:
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return []

        paths = []
        try:
            from extensions.navsim.navsim_python.drone_identity import find_navsim_drone_prims
            paths = [str(p.GetPath()) for p in find_navsim_drone_prims(stage)]
        except Exception:
            try:
                from navsim_python.drone_identity import find_navsim_drone_prims
                paths = [str(p.GetPath()) for p in find_navsim_drone_prims(stage)]
            except Exception:
                pass

        if not paths:
            for prim in stage.Traverse():
                name = prim.GetName()
                if name.startswith(("UAV_", "uav_", "dron_", "Dron_")):
                    paths.append(str(prim.GetPath()))

        return sorted(list(set(paths)))

    def refresh_uav_list(self) -> None:
        self.uav_paths = self.scan_uavs()
        if not hasattr(self, "uav_combo_frame") or self.uav_combo_frame is None:
            return

        self.uav_combo_frame.clear()
        with self.uav_combo_frame:
            with ui.VStack():
                ui.Spacer(height=3)
                if self.uav_paths:
                    items = [self.manual_control.get_uas_id(p) for p in self.uav_paths]
                    self.uav_combo = ui.ComboBox(
                        0, *items,
                        height=24,
                    )
                    self.uav_combo.model.add_item_changed_fn(self.on_uav_selected)
                    self.selected_uav_idx = 0
                    self.selected_uav_path = self.uav_paths[0]
                    self.manual_control.set_uav(self.selected_uav_path, self.selected_uav_idx)
                    if hasattr(self, "port_label") and self.port_label:
                        self.port_label.text = "15000"
                    if hasattr(self, "update_state_display"):
                        self.update_state_display()
                else:
                    self.uav_combo = ui.ComboBox(
                        0, "No se encontraron aeronaves NavSim en la escena",
                        height=24,
                    )
                    self.selected_uav_idx = 0
                    self.selected_uav_path = ""
                    if hasattr(self, "port_label") and self.port_label:
                        self.port_label.text = "-"
                    if hasattr(self, "state_label") and self.state_label:
                        self.state_label.text = "-"
                        self.state_label.style = {"color": 0xFF888888}
                ui.Spacer()

    # ==========================================================================
    # Interfaz de Usuario (UI)
    # ==========================================================================
    def build_ui(self) -> None:
        self.window = ui.Window(
            "NavSim - Joystick",
            width=520,
            height=660,
            raster_policy=ui.RasterPolicy.NEVER,
        )

        with self.window.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(spacing=6, height=0):
                    # --- Desplegable: UAV ---
                    with ui.CollapsableFrame("UAV", collapsed=False, style=ISAAC_COLLAPSABLE_STYLE, build_header_fn=build_isaac_header):
                        with ui.VStack(spacing=4, style={"margin_left": 8, "margin_right": 8, "margin_top": 6, "margin_bottom": 8}):
                            with ui.HStack(height=26, spacing=6):
                                ui.Label("Aeronave", width=80, alignment=ui.Alignment.LEFT_CENTER, style=LABEL_STYLE_GREY)
                                self.uav_combo_frame = ui.Frame()
                                ui.Button(
                                    "Refrescar",
                                    width=80,
                                    height=24,
                                    clicked_fn=self.refresh_uav_list,
                                )

                            with ui.HStack(height=24, spacing=6):
                                ui.Label("Estado", width=80, alignment=ui.Alignment.LEFT_CENTER, style=LABEL_STYLE_GREY)
                                self.state_label = ui.Label(
                                    "DISARMED",
                                    alignment=ui.Alignment.LEFT_CENTER,
                                    style={"color": 0xFFFF8CBA, "font_size": 13},
                                )

                            with ui.HStack(height=24, spacing=6):
                                ui.Label("Puerto RX", width=80, alignment=ui.Alignment.LEFT_CENTER, style=LABEL_STYLE_GREY)
                                self.port_label = ui.Label("15000", alignment=ui.Alignment.LEFT_CENTER, style={"color": 0xFF88DD88})
                                ui.Spacer()
                                ui.Button(
                                    "Viewport",
                                    width=80,
                                    height=24,
                                    clicked_fn=self.on_open_camera_viewport,
                                )

                    # --- Desplegable: Modo de vuelo ---
                    with ui.CollapsableFrame("Modo de vuelo", collapsed=False, style=ISAAC_COLLAPSABLE_STYLE, build_header_fn=build_isaac_header):
                        with ui.VStack(spacing=6, style={"margin_left": 8, "margin_right": 8, "margin_top": 6, "margin_bottom": 8}):
                            with ui.HStack(height=32, spacing=6):
                                self.btn_auto = ui.Button(
                                    "Auto",
                                    clicked_fn=lambda: self.switch_mode(2),
                                    style=STYLE_TRIGGER_AUTO,
                                )
                                self.btn_manual = ui.Button(
                                    "Manual",
                                    clicked_fn=lambda: self.switch_mode(1),
                                    style=STYLE_TRIGGER_MANUAL,
                                )
                                self.btn_hover = ui.Button(
                                    "Hover",
                                    clicked_fn=lambda: self.switch_mode(3),
                                    style=STYLE_TRIGGER_HOVER,
                                )
                                self.btn_disarm = ui.Button(
                                    "Disarm",
                                    clicked_fn=lambda: self.switch_mode(0),
                                    style=STYLE_TRIGGER_DISARM,
                                )
                                self.btn_stop = self.btn_disarm

                    # --- Desplegable: Ajustes ---
                    with ui.CollapsableFrame("Ajustes", collapsed=False, style=ISAAC_COLLAPSABLE_STYLE, build_header_fn=build_isaac_header):
                        with ui.VStack(spacing=5, style={"margin_left": 8, "margin_right": 8, "margin_top": 6, "margin_bottom": 8}):
                            with ui.HStack(height=22):
                                ui.Label("Velocidad Lineal Máx (m/s)", width=220, style=LABEL_STYLE_GREY)
                                self.lin_vel_slider = ui.FloatSlider(
                                    min=0.5, max=15.0, step=0.5
                                )
                                self.lin_vel_slider.model.set_value(self.linear_vel_limit)
                                self.lin_vel_slider.model.add_value_changed_fn(
                                    self.on_lin_vel_changed
                                )

                            with ui.HStack(height=22):
                                ui.Label("Velocidad Angular Máx (rad/s)", width=220, style=LABEL_STYLE_GREY)
                                self.ang_vel_slider = ui.FloatSlider(
                                    min=0.2, max=5.0, step=0.2
                                )
                                self.ang_vel_slider.model.set_value(self.ang_vel_limit)
                                self.ang_vel_slider.model.add_value_changed_fn(
                                    self.on_ang_vel_changed
                                )

                            with ui.HStack(height=22):
                                ui.Label("Invertir cámara vertical", width=220, style=LABEL_STYLE_GREY)
                                self.inv_cam_cb = ui.CheckBox(width=0)
                                ui.Spacer()
                                self.inv_cam_cb.model.set_value(True)
                                self.manual_control.invert_camera_vertical = True
                                self.inv_cam_cb.model.add_value_changed_fn(
                                    lambda m: setattr(
                                        self.manual_control,
                                        "invert_camera_vertical",
                                        m.get_value_as_bool(),
                                    )
                                )

                            with ui.HStack(height=22):
                                ui.Label("Invertir cámara horizontal", width=220, style=LABEL_STYLE_GREY)
                                self.inv_yaw_cb = ui.CheckBox(width=0)
                                ui.Spacer()
                                self.inv_yaw_cb.model.set_value(False)
                                self.manual_control.invert_camera_horizontal = False
                                self.inv_yaw_cb.model.add_value_changed_fn(
                                    lambda m: setattr(
                                        self.manual_control,
                                        "invert_camera_horizontal",
                                        m.get_value_as_bool(),
                                    )
                                )

                            with ui.HStack(height=22):
                                ui.Label("Habilitar teclado (WASD / Flechas)", width=220, style=LABEL_STYLE_GREY)
                                self.keyboard_cb = ui.CheckBox(width=0)
                                ui.Spacer()
                                self.keyboard_cb.model.set_value(True)
                                self.manual_control.use_keyboard = True
                                self.keyboard_cb.model.add_value_changed_fn(
                                    self.on_keyboard_toggle
                                )

                    # --- Desplegable: Monitor ---
                    with ui.CollapsableFrame("Monitor", collapsed=False, style=ISAAC_COLLAPSABLE_STYLE, build_header_fn=build_isaac_header):
                        with ui.VStack(spacing=4, style={"margin_left": 8, "margin_right": 8, "margin_top": 6, "margin_bottom": 8}):
                            with ui.HStack(spacing=8, height=32):
                                ui.Label("Vx [m/s]", width=70, alignment=ui.Alignment.LEFT_CENTER, style={"color": 0xFF7878FF})
                                self.plot_vx = ui.Plot(
                                    ui.Type.LINE,
                                    -self.linear_vel_limit,
                                    self.linear_vel_limit,
                                    *self.x_lv_data,
                                    height=30,
                                    style={"color": 0xFF3643F4},
                                )

                            with ui.HStack(spacing=8, height=32):
                                ui.Label("Vy [m/s]", width=70, alignment=ui.Alignment.LEFT_CENTER, style={"color": 0xFF8CDC8C})
                                self.plot_vy = ui.Plot(
                                    ui.Type.LINE,
                                    -self.linear_vel_limit,
                                    self.linear_vel_limit,
                                    *self.y_lv_data,
                                    height=30,
                                    style={"color": 0xFF50AF4C},
                                )

                            with ui.HStack(spacing=8, height=32):
                                ui.Label("Vz [m/s]", width=70, alignment=ui.Alignment.LEFT_CENTER, style={"color": 0xFFFFB478})
                                self.plot_vz = ui.Plot(
                                    ui.Type.LINE,
                                    -self.linear_vel_limit,
                                    self.linear_vel_limit,
                                    *self.z_lv_data,
                                    height=30,
                                    style={"color": 0xFFF39621},
                                )

                            with ui.HStack(spacing=8, height=32):
                                ui.Label("Yz [rad/s]", width=70, alignment=ui.Alignment.LEFT_CENTER, style={"color": 0xFF4DB7FF})
                                self.plot_wz = ui.Plot(
                                    ui.Type.LINE,
                                    -self.ang_vel_limit,
                                    self.ang_vel_limit,
                                    *self.z_av_data,
                                    height=30,
                                    style={"color": 0xFF0098FF},
                                )

                            ui.Spacer(height=6)
                            ui.Line(height=1)
                            ui.Spacer(height=4)

                            # Matriz de 2x6 botones de test de joystick (1 al 12) directamente debajo de la línea
                            self.joystick_test_buttons = []
                            self._last_joy_btn_states = [False] * 12
                            with ui.VStack(spacing=4):
                                for r in range(2):
                                    with ui.HStack(spacing=4):
                                        for c in range(6):
                                            btn_idx = r * 6 + c
                                            btn_num = btn_idx + 1
                                            btn = ui.Button(
                                                str(btn_num),
                                                height=26,
                                                style=JOY_BTN_OFF_STYLE,
                                                clicked_fn=lambda idx=btn_idx: self._on_joy_btn_clicked(idx),
                                            )
                                            self.joystick_test_buttons.append(btn)



    # ==========================================================================
    # Lógica de Interacción
    # ==========================================================================
    def on_open_camera_viewport(self) -> None:
        if not self.selected_uav_path:
            self.refresh_uav_list()
        if self.selected_uav_path:
            self.manual_control.set_uav(self.selected_uav_path, self.selected_uav_idx)
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
            if hasattr(self, "port_label") and self.port_label:
                self.port_label.text = str(15000 + val)
            self.manual_control.set_uav(self.selected_uav_path, self.selected_uav_idx)
            if hasattr(self, "update_state_display"):
                self.update_state_display()

    def on_lin_vel_changed(self, model) -> None:
        self.linear_vel_limit = model.get_value_as_float()
        self.manual_control.linear_vel_limit = self.linear_vel_limit
        for p in (getattr(self, "plot_vx", None), getattr(self, "plot_vy", None), getattr(self, "plot_vz", None)):
            if p is not None:
                p.scale_min = -self.linear_vel_limit
                p.scale_max = self.linear_vel_limit

    def on_ang_vel_changed(self, model) -> None:
        self.ang_vel_limit = model.get_value_as_float()
        self.manual_control.ang_vel_limit = self.ang_vel_limit
        p = getattr(self, "plot_wz", None)
        if p is not None:
            p.scale_min = -self.ang_vel_limit
            p.scale_max = self.ang_vel_limit

    def on_keyboard_toggle(self, model) -> None:
        enabled = model.get_value_as_bool()
        self.manual_control.use_keyboard = enabled
        if self.is_controlling:
            if enabled:
                self.manual_control.keyboard.start()
            else:
                self.manual_control.keyboard.stop()

    def _get_navsim_instance(self):
        try:
            from extensions.navsim.navsim_python import extension as n_ext
            return getattr(n_ext, "_ACTIVE_NAVSIM_INSTANCE", None)
        except Exception:
            try:
                import sys
                for mod_name, mod in sys.modules.items():
                    if "navsim_python.extension" in mod_name and hasattr(mod, "_ACTIVE_NAVSIM_INSTANCE"):
                        return getattr(mod, "_ACTIVE_NAVSIM_INSTANCE", None)
            except Exception:
                pass
        return None

    def get_uav_mode(self, uav_idx: int) -> int:
        # Modo comandado / registrado localmente en el Joystick
        local_mode = self.uav_modes.get(uav_idx, 0)

        # Consultar si NavSim reporta un evento asíncrono relevante (misión AUTO o emergencia)
        navsim_inst = self._get_navsim_instance()
        if navsim_inst is not None and getattr(navsim_inst, "is_simulation_running", False):
            ctrl = getattr(navsim_inst, "uav_control", None)
            if ctrl is not None:
                try:
                    # 1. Si el dron está en fallo crítico en físicas:
                    uav_state = int(ctrl.uav_states[uav_idx])
                    if uav_state == 2:  # DEAD
                        self.uav_modes[uav_idx] = 4
                        return 4  # SYSTEM_FAILURE

                    # 2. Si el dron está ejecutando autónomamente una ruta 4D:
                    if uav_state == 1:  # BUSY
                        self.uav_modes[uav_idx] = 2
                        return 2  # AUTO

                    # 3. Si estaba en AUTO y el plan finalizó:
                    if local_mode == 2 and not bool(ctrl.has_plan[uav_idx]):
                        self.uav_modes[uav_idx] = 0  # Auto-Disarm al posarse en vertipuerto -> DISARMED
                        return 0

                    # 4. Si uav_control expone uav_modes directamente y reporta modo activo:
                    if hasattr(ctrl, "uav_modes") and 0 <= uav_idx < len(ctrl.uav_modes):
                        sim_mode = int(ctrl.uav_modes[uav_idx])
                        if sim_mode != 0:
                            self.uav_modes[uav_idx] = sim_mode
                            return sim_mode
                except Exception:
                    pass

        return local_mode

    def update_state_display(self, force_mode: Optional[int] = None) -> None:
        if not hasattr(self, "state_label") or self.state_label is None:
            return
        mode = force_mode if force_mode is not None else self.get_uav_mode(self.selected_uav_idx)
        self.last_displayed_mode = mode
        label_text, color = STATE_INFO.get(mode, ("DISARMED", 0xFFFF8CBA))
        self.state_label.text = label_text
        self.state_label.style = {"color": color, "font_size": 13}

    def _on_app_update(self, event) -> None:
        if hasattr(self, "state_label") and self.state_label is not None:
            current_mode = self.get_uav_mode(self.selected_uav_idx)
            if current_mode != getattr(self, "last_displayed_mode", -1):
                if hasattr(self, "update_state_display"):
                    self.update_state_display(current_mode)

        # Actualizar matriz de botones de test de joystick en tiempo real (1 al 12)
        if hasattr(self, "joystick_test_buttons") and len(self.joystick_test_buttons) == 12:
            states = self.manual_control.joystick.get_buttons_state(12)
            if not hasattr(self, "_last_joy_btn_states") or len(self._last_joy_btn_states) != 12:
                self._last_joy_btn_states = [False] * 12

            for idx, (btn, state) in enumerate(zip(self.joystick_test_buttons, states)):
                if state != self._last_joy_btn_states[idx]:
                    btn.style = JOY_BTN_ON_STYLE if state else JOY_BTN_OFF_STYLE
                    self._last_joy_btn_states[idx] = state


            # Botón 1 (Gatillo): Conmutar entre HOVER (3) y DISARM (0), actualizando el panel superior
            btn1 = states[0] if len(states) > 0 else False
            if btn1 and not getattr(self, "_joy_btn1_prev", False):
                self._joy_btn1_prev = True
                cur_mode = self.get_uav_mode(self.selected_uav_idx)
                target_mode = 0 if cur_mode != 0 else 3
                self.switch_mode(target_mode)
            elif not btn1:
                self._joy_btn1_prev = False

            # Botón 2 (Pulgar): Devolver la cámara de seguimiento a su posición inicial
            btn2 = states[1] if len(states) > 1 else False
            if btn2 and not getattr(self, "_joy_btn2_prev", False):
                self._joy_btn2_prev = True
                self.manual_control.reset_camera()
            elif not btn2:
                self._joy_btn2_prev = False

            # Si el lazo de vuelo no está corriendo, permitir navegación de cámara con el hat y botones 5/6
            if not self.is_controlling and hasattr(self, "manual_control") and self.manual_control:
                if hasattr(self.manual_control, "camera_prim") and self.manual_control.camera_prim:
                    joy_in = self.manual_control.joystick.inputs
                    z_in = joy_in[5]
                    z_out = joy_in[6]
                    c_yaw = joy_in[7]
                    c_pitch = joy_in[8]
                    if z_in != 0.0 or z_out != 0.0 or c_yaw != 0.0 or c_pitch != 0.0:
                        self.manual_control.move_camera(z_in, z_out, c_yaw, c_pitch)

        # Actualización continua de gráficas de velocidad (10 Hz = cada 100 ms)
        now = time.time()
        if now - getattr(self, "_last_plot_update_time", 0.0) >= 0.1:
            self._last_plot_update_time = now
            self._update_plots_step()

    def _on_joy_btn_clicked(self, idx: int) -> None:
        """Feedback visual temporal cuando se hace clic con el ratón sobre un botón de test."""
        if hasattr(self, "joystick_test_buttons") and 0 <= idx < len(self.joystick_test_buttons):
            async def _flash():
                self.joystick_test_buttons[idx].style = JOY_BTN_ON_STYLE
                await asyncio.sleep(0.18)
                last_states = getattr(self, "_last_joy_btn_states", [False] * 12)
                if not last_states[idx]:
                    self.joystick_test_buttons[idx].style = JOY_BTN_OFF_STYLE
            asyncio.ensure_future(_flash())

    def _update_mode_buttons(self, active_mode: int) -> None:
        # Los botones de modo son triggers de evento puntual: no se fijan con color activo
        if hasattr(self, "update_state_display"):
            self.update_state_display(active_mode)

    def switch_mode(self, mode: int) -> None:
        self.uav_modes[self.selected_uav_idx] = mode
        if hasattr(self, "update_state_display"):
            self.update_state_display(mode)

        if mode == 0:
            self.manual_control.set_mode(0)
            self.stop_control()
            return

        # Para modos operativos (1=MANUAL, 2=AUTO, 3=HOVER)
        if not self.is_controlling:
            self.start_control(initial_mode=mode)
        else:
            self.manual_control.set_mode(mode)

    def toggle_control(self) -> None:
        if self.is_controlling:
            self.switch_mode(0)
        else:
            self.switch_mode(1)

    def start_control(self, initial_mode: int = 1) -> None:
        if not self.uav_paths:
            self.refresh_uav_list()
        if not self.selected_uav_path:
            return

        self.is_controlling = True
        self.manual_control.start(self.selected_uav_path, self.selected_uav_idx)
        if initial_mode != 1:
            self.manual_control.set_mode(initial_mode)

        self.uav_modes[self.selected_uav_idx] = initial_mode
        if hasattr(self, "update_state_display"):
            self.update_state_display(initial_mode)

    def stop_control(self) -> None:
        self.is_controlling = False
        self.manual_control.stop()
        self.uav_modes[self.selected_uav_idx] = 0
        if hasattr(self, "update_state_display"):
            self.update_state_display(0)

    def _update_plots_step(self) -> None:
        """Actualiza el histórico y traza gráfica de los 4 ejes a 10 Hz de forma continua."""
        if not hasattr(self, "manual_control") or not self.manual_control:
            return
        if not hasattr(self, "plot_vx") or self.plot_vx is None:
            return

        vx, vy, vz, wz = self.manual_control.get_current_velocities()

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

    async def update_plots_loop(self) -> None:
        """Compatibilidad con bucle asíncrono si fuera invocado externamente."""
        while not getattr(self, "stop_plots_update", False):
            self._update_plots_step()
            await asyncio.sleep(0.1)