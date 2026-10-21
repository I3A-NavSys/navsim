from datetime import datetime, timezone, date

try:
    import omni.ui as ui
except ImportError:
    from isaacsim.gui.components.ui_utils import ui
from .georeference import (
    DEFAULT_CAMPUS_ALBACETE_LAT,
    DEFAULT_CAMPUS_ALBACETE_LON,
    DEFAULT_CAMPUS_ALBACETE_ALT,
)
from .time_reference import NavSimTimeReference

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
    "Button": {
        "margin": 0,
        "border_radius": 2,
    },
    "Button:disabled": {
        "background_color": 0xFF202020,
        "border_color": 0x0,
        "margin": 0,
    },
    "Button:disabled:hovered": {
        "background_color": 0xFF202020,
        "margin": 0,
    },
    "Button.Label:disabled": {
        "color": 0xFF606060,
    },
    "Field": {
        "alignment": ui.Alignment.CENTER,
    },
}

LABEL_STYLE_GREY = {"color": 0xFF9E9E9E}

STYLE_CENTERED_FIELD = {
    "alignment": ui.Alignment.CENTER,
    "Field": {
        "alignment": ui.Alignment.CENTER,
    },
    "color": 0xFFDDDDDD,
}

STYLE_ACTUAL_DATE = {
    "alignment": ui.Alignment.CENTER,
    "Field": {
        "alignment": ui.Alignment.CENTER,
    },
    "color": 0xFFE0E0E0,
}

STYLE_ACTUAL_TIME = {
    "alignment": ui.Alignment.CENTER,
    "Field": {
        "alignment": ui.Alignment.CENTER,
    },
    "color": 0xFF88DD88,
}


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


class UIBuilder:
    """
    Consola de control, georreferenciación y monitorización de flota para NavSim.
    Diseñada siguiendo las pautas de estilo y distribución compacta de Isaac Sim.
    """

    def __init__(self, extension):
        self.extension = extension
        self.window = None
        self._georef_source_label = None

        # Componentes de validación de identidad ASTM F3411
        self._validation_banner_label = None
        self._validation_details_label = None
        self._drone_combo_frame = None
        self._drone_combo = None
        self._drone_details_frame = None
        self._detail_title_label = None
        self._detail_prim_label = None
        self._detail_uas_id_label = None
        self._detail_id_type_label = None
        self._detail_ua_type_label = None
        self._detail_operator_id_label = None
        self._detail_operator_type_label = None
        self._detail_desc_label = None
        self._detail_model_label = None
        self._detail_ports_label = None
        self._detail_status_label = None
        self._detail_errors_label = None

        # Componentes del desplegable Comunicaciones
        self._comms_state_label = None
        self._comms_rate_label = None
        self._comms_rx_range_label = None
        self._comms_tx_range_label = None

        self._current_report = None
        self._selected_drone_idx = 0
        self._selected_drone_uas_id = None
        self._fleet_frame = None

        georef = getattr(self.extension, "georeference", None)
        lat0 = getattr(georef, "lat0", DEFAULT_CAMPUS_ALBACETE_LAT) if georef else DEFAULT_CAMPUS_ALBACETE_LAT
        lon0 = getattr(georef, "lon0", DEFAULT_CAMPUS_ALBACETE_LON) if georef else DEFAULT_CAMPUS_ALBACETE_LON
        alt0 = getattr(georef, "alt0", DEFAULT_CAMPUS_ALBACETE_ALT) if georef else DEFAULT_CAMPUS_ALBACETE_ALT

        # Componentes de Cronorreferenciación y Tiempo
        self._actual_date_field = None
        self._actual_time_field = None
        self._btn_system = None
        self._btn_play_pause = None
        self._btn_stop = None
        self._btn_plus_hour = None

        time_ref = getattr(self.extension, "time_reference", None)
        date0 = time_ref.get_date_str() if time_ref else "2026-10-21"
        time0 = time_ref.get_time_str() if time_ref else "12:00:00"
        end_date0 = time_ref.get_end_date_str() if time_ref else "2026-10-21"
        end_time0 = time_ref.get_end_time_str() if time_ref else "13:00:00"

        # Modelos reactivos para los campos de fecha y hora inicial, actual y final
        self._date_model = ui.SimpleStringModel(date0)
        self._time_model = ui.SimpleStringModel(time0)
        self._actual_date_model = ui.SimpleStringModel(date0)
        self._actual_time_model = ui.SimpleStringModel(f"{time0} UTC")
        self._end_date_model = ui.SimpleStringModel(end_date0)
        self._end_time_model = ui.SimpleStringModel(end_time0)

        # Modelos reactivos para los campos numéricos WGS-84 y comunicaciones
        self._lat_model = ui.SimpleFloatModel(lat0)
        self._lon_model = ui.SimpleFloatModel(lon0)
        self._alt_model = ui.SimpleFloatModel(alt0)

        rate0 = getattr(self.extension, "telemetry_rate_hz", 1.0) if self.extension else 1.0
        self._rate_model = ui.SimpleFloatModel(float(rate0))

        self._updating_models_internally = False
        self._date_model.add_value_changed_fn(lambda m: self._on_time_field_changed())
        self._time_model.add_value_changed_fn(lambda m: self._on_time_field_changed())
        self._date_model.add_end_edit_fn(lambda m: self._on_date_field_end_edit())
        self._time_model.add_end_edit_fn(lambda m: self._on_time_field_end_edit())
        self._end_date_model.add_end_edit_fn(lambda m: self._on_end_date_field_end_edit())
        self._end_time_model.add_end_edit_fn(lambda m: self._on_end_time_field_end_edit())
        self._lat_model.add_value_changed_fn(lambda m: self._on_georef_field_changed())
        self._lon_model.add_value_changed_fn(lambda m: self._on_georef_field_changed())
        self._alt_model.add_value_changed_fn(lambda m: self._on_georef_field_changed())
        self._rate_model.add_value_changed_fn(lambda m: self._on_rate_field_changed())

    def build_ui(self):
        self.window = ui.Window("NavSim - Consola", width=440, height=600)
        if getattr(self.window, "set_visibility_changed_listener", None):
            self.window.set_visibility_changed_listener(self._on_window_visibility_changed)
        elif hasattr(self.window, "set_visibility_changed_fn"):
            self.window.set_visibility_changed_fn(self._on_window_visibility_changed)

        with self.window.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(spacing=6, height=0):
                    # --- Desplegable 1: Mapa ---
                    with ui.CollapsableFrame("Mapa", height=0, collapsed=False, style=ISAAC_COLLAPSABLE_STYLE, build_header_fn=build_isaac_header):
                        with ui.VStack(spacing=3, height=0, style={"margin_left": 8, "margin_right": 8, "margin_top": 6, "margin_bottom": 8}):
                            with ui.HStack(height=22):
                                ui.Label("Origen", width=140, style={"color": 0xFF9E9E9E})
                                georef = getattr(self.extension, "georeference", None)
                                source_str = getattr(georef, "source", "WGS84") if georef else "WGS84"
                                self._georef_source_label = ui.Label(
                                    f"{source_str}",
                                    word_wrap=True,
                                    style={"color": 0xFFCCCCCC},
                                )

                            with ui.HStack(height=22):
                                ui.Label("Latitud [°]", width=140, style={"color": 0xFF9E9E9E})
                                ui.FloatField(model=self._lat_model, format="%.6f", step=0.0001)

                            with ui.HStack(height=22):
                                ui.Label("Longitud [°]", width=140, style={"color": 0xFF9E9E9E})
                                ui.FloatField(model=self._lon_model, format="%.6f", step=0.0001)

                            with ui.HStack(height=22):
                                ui.Label("Altitud [m]", width=140, style={"color": 0xFF9E9E9E})
                                ui.FloatField(model=self._alt_model, format="%.2f", step=1.0)

                            with ui.HStack(height=22, style={"margin_top": 4}):
                                ui.Spacer(width=140)
                                with ui.HStack(spacing=6):
                                    ui.Button(
                                        text="USD",
                                        height=22,
                                        clicked_fn=self._on_detect_georef_clicked,
                                    )
                                    ui.Button(
                                        text="Albacete",
                                        height=22,
                                        clicked_fn=self._on_reset_albacete_clicked,
                                    )

                    # --- Desplegable 2: Tiempo ---
                    with ui.CollapsableFrame("Tiempo", height=0, collapsed=False, style=ISAAC_COLLAPSABLE_STYLE, build_header_fn=build_isaac_header):
                        with ui.VStack(spacing=4, height=0, style={"margin_left": 8, "margin_right": 8, "margin_top": 6, "margin_bottom": 8}):
                            time_ref = getattr(self.extension, "time_reference", None)
                            init_d = time_ref.get_date_str() if time_ref else "2026-10-21"
                            init_t = time_ref.get_time_str() if time_ref else "12:00:00"

                            # 1. Fila Inicial: Día y Hora (centrados en sus campos)
                            with ui.HStack(height=22):
                                ui.Label("Inicial", width=140, style=LABEL_STYLE_GREY)
                                with ui.HStack(spacing=6):
                                    ui.StringField(
                                        model=self._date_model,
                                        alignment=ui.Alignment.CENTER,
                                        style=STYLE_CENTERED_FIELD,
                                    )
                                    ui.StringField(
                                        model=self._time_model,
                                        alignment=ui.Alignment.CENTER,
                                        style=STYLE_CENTERED_FIELD,
                                    )

                            # Botón Sistema (alineado exactamente con la columna de datos)
                            with ui.HStack(height=22, style={"margin_top": 3, "margin_bottom": 4}):
                                ui.Spacer(width=140)
                                self._btn_system = ui.Button(
                                    "Sistema",
                                    height=22,
                                    clicked_fn=self._on_reset_system_time_clicked,
                                )

                            # 2. Fila Actual: Día y Hora (centrados en sus campos, de solo lectura)
                            with ui.HStack(height=22):
                                ui.Label("Actual", width=140, style=LABEL_STYLE_GREY)
                                with ui.HStack(spacing=6):
                                    self._actual_date_field = ui.StringField(
                                        model=self._actual_date_model,
                                        read_only=True,
                                        alignment=ui.Alignment.CENTER,
                                        style=STYLE_ACTUAL_DATE,
                                        tooltip="Fecha UTC actual de simulación",
                                    )
                                    self._actual_time_field = ui.StringField(
                                        model=self._actual_time_model,
                                        read_only=True,
                                        alignment=ui.Alignment.CENTER,
                                        style=STYLE_ACTUAL_TIME,
                                        tooltip="Hora UTC actual de simulación",
                                    )

                            # Botones Play y Stop (alineados con los datos)
                            with ui.HStack(height=22, style={"margin_top": 3, "margin_bottom": 4}):
                                ui.Spacer(width=140)
                                with ui.HStack(spacing=6):
                                    self._btn_play_pause = ui.Button(
                                        "Play",
                                        height=22,
                                        clicked_fn=self._on_play_pause_clicked,
                                    )
                                    self._btn_stop = ui.Button(
                                        "Stop",
                                        height=22,
                                        clicked_fn=self._on_stop_clicked,
                                        enabled=False,
                                    )

                            # 3. Fila Final: Día y Hora (centrados en sus campos)
                            with ui.HStack(height=22, style={"margin_top": 2}):
                                ui.Label("Final", width=140, style=LABEL_STYLE_GREY)
                                with ui.HStack(spacing=6):
                                    ui.StringField(
                                        model=self._end_date_model,
                                        alignment=ui.Alignment.CENTER,
                                        style=STYLE_CENTERED_FIELD,
                                    )
                                    ui.StringField(
                                        model=self._end_time_model,
                                        alignment=ui.Alignment.CENTER,
                                        style=STYLE_CENTERED_FIELD,
                                    )

                            # Botón +1 hora (alineado exactamente con la columna de datos)
                            with ui.HStack(height=22, style={"margin_top": 3, "margin_bottom": 2}):
                                ui.Spacer(width=140)
                                self._btn_plus_hour = ui.Button(
                                    "+1 hora",
                                    height=22,
                                    clicked_fn=self._on_plus_one_hour_clicked,
                                )

                    # --- Desplegable 3: Flota ---
                    self._fleet_frame = ui.CollapsableFrame(
                        "Flota",
                        height=0,
                        collapsed=False,
                        style=ISAAC_COLLAPSABLE_STYLE,
                        build_header_fn=build_isaac_header,
                    )
                    self._fleet_frame.set_collapsed_changed_fn(self._on_fleet_frame_collapsed_changed)
                    with self._fleet_frame:
                        with ui.VStack(spacing=3, height=0, style={"margin_left": 8, "margin_right": 8, "margin_top": 6, "margin_bottom": 8}):
                            with ui.HStack(height=22):
                                ui.Label("Estado Flota", width=140, style={"color": 0xFF9E9E9E})
                                self._validation_banner_label = ui.Label(
                                    "[INFO] Sin verificar (pulsa PLAY)",
                                    word_wrap=True,
                                    style={"color": 0xFFAAAAAA},
                                )

                            self._validation_details_label = ui.Label(
                                "",
                                word_wrap=True,
                                style={"color": 0xFFFF6666},
                            )

                            with ui.HStack(height=26, spacing=6, style={"margin_top": 4, "margin_bottom": 2}):
                                ui.Label("Aeronave", width=65, alignment=ui.Alignment.LEFT_CENTER, style={"color": 0xFF9E9E9E})
                                self._drone_combo_frame = ui.Frame()
                                ui.Button(
                                    "Enfocar",
                                    width=75,
                                    height=24,
                                    clicked_fn=self._on_focus_drone_clicked,
                                )

                            # Contenedor de los detalles del dron seleccionado
                            self._drone_details_frame = ui.Frame(height=0, visible=False)
                            self._build_drone_details_widgets()

                    # --- Desplegable 4: Comunicaciones ---
                    with ui.CollapsableFrame("Comunicaciones", height=0, collapsed=False, style=ISAAC_COLLAPSABLE_STYLE, build_header_fn=build_isaac_header):
                        with ui.VStack(spacing=3, height=0, style={"margin_left": 8, "margin_right": 8, "margin_top": 6, "margin_bottom": 8}):
                            with ui.HStack(height=22):
                                ui.Label("Estado Canales", width=140, style={"color": 0xFF9E9E9E})
                                self._comms_state_label = ui.Label(
                                    "En espera (pulsa PLAY)",
                                    style={"color": 0xFFAAAAAA},
                                )

                            with ui.HStack(height=22):
                                ui.Label("Protocolo", width=140, style={"color": 0xFF9E9E9E})
                                ui.Label("MAVLink 2.0 / UDP", style={"color": 0xFFCCCCCC})

                            with ui.HStack(height=22):
                                ui.Label("Interfaz Local", width=140, style={"color": 0xFF9E9E9E})
                                ui.Label("127.0.0.1 (Localhost)", style={"color": 0xFFCCCCCC})

                            with ui.HStack(height=22):
                                ui.Label("Frecuencia Telemetría", width=140, style={"color": 0xFF9E9E9E})
                                ui.FloatField(model=self._rate_model, format="%.1f", step=0.5, width=60)
                                ui.Label(" Hz", width=30, style={"color": 0xFF88DD88})

                            with ui.HStack(height=22):
                                ui.Label("Rango Puertos RX", width=140, style={"color": 0xFF9E9E9E})
                                self._comms_rx_range_label = ui.Label("15000", style={"color": 0xFF88DD88})

                            with ui.HStack(height=22):
                                ui.Label("Rango Puertos TX", width=140, style={"color": 0xFF9E9E9E})
                                self._comms_tx_range_label = ui.Label("16000", style={"color": 0xFF88DD88})

                    # Spacer al final para absorber todo el espacio vertical sobrante de la ventana
                    ui.Spacer()

        # Si ya había un informe de validación previo en la extensión, inicializar la vista de flota
        prev_report = getattr(self.extension, "latest_validation_report", None)
        if prev_report is not None:
            self.update_validation_report(prev_report)
        else:
            self._schedule_rebuild_drone_list()

        # Inicializar panel de comunicaciones
        self._update_comms_labels()
        if self.extension and getattr(self.extension, "is_simulation_running", False):
            self.update_comms_simulation_state(True)

        # Inicializar panel de tiempo simulado
        self.update_sim_time(0.0, is_playing=False, is_paused=False)

    def update_status(self, text: str):
        """Registro de mensajes de estado en el subsistema de logging de NavSim."""
        if self.extension and getattr(self.extension, "logger", None):
            self.extension.logger.info(text)

    def _build_drone_details_widgets(self):
        """Construye una única vez la estructura visual del inspector de detalles del UAV.
        La actualización posterior se realiza modificando directamente las propiedades .text
        y .style de las etiquetas preexistentes, sin llamar a frame.clear() ni frame.addChild()
        durante callbacks de eventos (evitando errores de concurrencia en ImGui/omni.ui)."""
        with self._drone_details_frame:
            with ui.VStack(spacing=3, height=0, style={"margin_top": 6}):
                ui.Line(style={"color": 0x44FFFFFF})
                with ui.HStack(height=22, style={"margin_top": 4}):
                    ui.Label("Prim USD", width=140, style={"color": 0xFF9E9E9E})
                    self._detail_prim_label = ui.Label("", style={"color": 0xFFCCCCCC}, word_wrap=True)

                with ui.HStack(height=22):
                    ui.Label("Identificador (uas_id)", width=140, style={"color": 0xFF9E9E9E})
                    self._detail_uas_id_label = ui.Label("", style={"color": 0xFFFFFFFF})

                with ui.HStack(height=22):
                    ui.Label("Tipo Identificador", width=140, style={"color": 0xFF9E9E9E})
                    self._detail_id_type_label = ui.Label("", style={"color": 0xFFCCCCCC})

                with ui.HStack(height=22):
                    ui.Label("Categoría Aeronave", width=140, style={"color": 0xFF9E9E9E})
                    self._detail_ua_type_label = ui.Label("", style={"color": 0xFFCCCCCC})

                with ui.HStack(height=22):
                    ui.Label("Operador Civil", width=140, style={"color": 0xFF9E9E9E})
                    self._detail_operator_id_label = ui.Label("", style={"color": 0xFFCCCCCC})

                with ui.HStack(height=22):
                    ui.Label("Tipo Operador", width=140, style={"color": 0xFF9E9E9E})
                    self._detail_operator_type_label = ui.Label("", style={"color": 0xFFCCCCCC})

                with ui.HStack(height=22):
                    ui.Label("Descripción Reposo", width=140, style={"color": 0xFF9E9E9E})
                    self._detail_desc_label = ui.Label("", style={"color": 0xFFAAAAAA})

                with ui.HStack(height=22):
                    ui.Label("Modelo Físico", width=140, style={"color": 0xFF9E9E9E})
                    self._detail_model_label = ui.Label("", style={"color": 0xFFDDDDDD})

                with ui.HStack(height=22):
                    ui.Label("Puertos UDP MAVLink", width=140, style={"color": 0xFF9E9E9E})
                    self._detail_ports_label = ui.Label("", style={"color": 0xFF88DD88})

                with ui.HStack(height=22):
                    ui.Label("Estado Conforme", width=140, style={"color": 0xFF9E9E9E})
                    self._detail_status_label = ui.Label("", style={"color": 0xFF44FF44})

                self._detail_errors_label = ui.Label(
                    "",
                    style={"color": 0xFFFF6666},
                    word_wrap=True,
                    visible=False,
                )

    def update_validation_report(self, report):
        """Actualiza el panel de validación ASTM F3411 y los avisos de conflictos en pantalla."""
        self._current_report = report
        if self._validation_banner_label is None:
            return

        if report.uav_count == 0:
            self._validation_banner_label.text = "[AVISO] No se encontraron aeronaves en la escena."
            self._validation_banner_label.style = {"color": 0xFF888888}
            if self._validation_details_label is not None:
                self._validation_details_label.text = ""
            self._selected_drone_idx = -1
            self._schedule_rebuild_drone_list()
            return

        # 1. Caso Conflicto Crítico de Identificador Duplicado
        if report.has_conflicts:
            self._validation_banner_label.text = "[CONFLICTO CRÍTICO] ID DUPLICADO"
            self._validation_banner_label.style = {"color": 0xFFFF3333}
            if self._validation_details_label is not None:
                self._validation_details_label.text = (
                    f"¡PELIGRO DE COLISIÓN! {len(report.conflicts)} conflicto(s) de identificadores duplicados.\n"
                    f"Simulación bloqueada. Consulta el desglose en la Consola principal de Isaac Sim."
                )
                self._validation_details_label.style = {"color": 0xFFFF6666}

        # 2. Caso Incompleto / Atributos Faltantes (sin colisión de IDs)
        elif not report.is_valid:
            invalid_drones = [ident for ident in report.identities if not ident.is_valid]
            num_inv = len(invalid_drones)
            self._validation_banner_label.text = f"[AVISO] ATRIBUTOS INCOMPLETOS ({num_inv}/{report.uav_count})"
            self._validation_banner_label.style = {"color": 0xFFFFAA00}
            if self._validation_details_label is not None:
                self._validation_details_label.text = (
                    f"{len(report.incomplete_errors)} atributos faltantes en {num_inv} aeronaves.\n"
                    f"Desglose emitido a la Consola de Isaac Sim. Selecciona una aeronave en el desplegable para ver sus atributos."
                )
                self._validation_details_label.style = {"color": 0xFFFFCC44}

        # 3. Flota Conforme y Válida
        else:
            self._validation_banner_label.text = (
                f"{report.uav_count} aeronave(s) conformes (ASTM F3411)"
            )
            self._validation_banner_label.style = {"color": 0xFF44FF44}
            if self._validation_details_label is not None:
                self._validation_details_label.text = ""

        # Mantener selección válida
        if self._selected_drone_idx < 0 or self._selected_drone_idx >= len(report.identities):
            self._selected_drone_idx = 0

        self._schedule_rebuild_drone_list()

    def _schedule_rebuild_drone_list(self):
        """Programa la reconstrucción del selector en el siguiente frame para evitar
        modificar contenedores durante callbacks de eventos o renderizado."""
        import asyncio

        async def _deferred():
            try:
                import omni.kit.app
                await omni.kit.app.get_app().next_update_async()
            except Exception:
                await asyncio.sleep(0.01)
            if self.window is None or self._drone_combo_frame is None:
                return
            self._rebuild_drone_list()
            self._update_drone_details_labels()
            self._update_comms_labels()

        asyncio.ensure_future(_deferred())

    def _rebuild_drone_list(self):
        """Construye o actualiza el menú desplegable (ComboBox) con la lista de aeronaves."""
        if self._drone_combo_frame is None:
            return
        self._drone_combo_frame.clear()
        with self._drone_combo_frame:
            with ui.VStack():
                ui.Spacer(height=3)
                report = self._current_report
                if report is None or not report.identities:
                    self._drone_combo = ui.ComboBox(
                        0, "No se encontraron aeronaves en la escena",
                        height=24,
                    )
                    if self._drone_details_frame is not None:
                        self._drone_details_frame.visible = False
                    ui.Spacer()
                    return

                items = []
                for ident in report.identities:
                    mark = "" if ident.is_valid else "[ERR] "
                    items.append(f"{mark}{ident.uas_id}")

                if not items:
                    items = ["No se encontraron aeronaves en la escena"]

                # Preservar el dron previamente marcado según su identificador (uas_id)
                prev_uas_id = getattr(self, "_selected_drone_uas_id", None)
                sel_idx = 0
                if prev_uas_id and report and report.identities:
                    for i, ident in enumerate(report.identities):
                        if ident.uas_id == prev_uas_id:
                            sel_idx = i
                            break
                    else:
                        sel_idx = max(0, min(self._selected_drone_idx, len(items) - 1))
                else:
                    sel_idx = max(0, min(self._selected_drone_idx, len(items) - 1))

                self._selected_drone_idx = sel_idx
                if report and report.identities and 0 <= sel_idx < len(report.identities):
                    self._selected_drone_uas_id = report.identities[sel_idx].uas_id

                self._drone_combo = ui.ComboBox(
                    sel_idx, *items,
                    height=24,
                )
                self._drone_combo.model.add_item_changed_fn(self._on_drone_combo_selected)
                ui.Spacer()

    def _on_drone_combo_selected(self, model, item=None):
        """Callback cuando el usuario selecciona una aeronave en el ComboBox."""
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

        report = self._current_report
        if report and report.identities and 0 <= val < len(report.identities):
            self._selected_drone_idx = val
            self._selected_drone_uas_id = report.identities[val].uas_id
            self._update_drone_details_labels()

    def _update_drone_details_labels(self):
        """Actualiza el contenido de las etiquetas de detalle in-place."""
        report = self._current_report
        if report is None or not report.identities or self._selected_drone_idx < 0:
            if self._drone_details_frame is not None:
                self._drone_details_frame.visible = False
            return

        if self._selected_drone_idx >= len(report.identities):
            self._selected_drone_idx = 0

        ident = report.identities[self._selected_drone_idx]
        uav_idx = self._selected_drone_idx

        base_rx = getattr(self.extension, "base_rx_port", 15000) if self.extension else 15000
        base_tx = getattr(self.extension, "base_tx_port", 16000) if self.extension else 16000
        rx_port = base_rx + uav_idx
        tx_port = base_tx + uav_idx

        if self._drone_details_frame is not None:
            self._drone_details_frame.visible = True

        if self._detail_prim_label is not None:
            self._detail_prim_label.text = f"{ident.prim_path}"
        if self._detail_uas_id_label is not None:
            self._detail_uas_id_label.text = f"{ident.uas_id}"
        if self._detail_id_type_label is not None:
            self._detail_id_type_label.text = f"{ident.id_type_name} ({ident.id_type})"
        if self._detail_ua_type_label is not None:
            self._detail_ua_type_label.text = f"{ident.ua_type_name} ({ident.ua_type})"
        if self._detail_operator_id_label is not None:
            self._detail_operator_id_label.text = f"{ident.operator_id or 'NO ASIGNADO'}"
            self._detail_operator_id_label.style = {"color": 0xFFCCCCCC if ident.operator_id else 0xFFFF8888}
        if self._detail_operator_type_label is not None:
            op_type_name = "CAA (Registro Autoridad)" if ident.operator_id_type == 0 else f"Tipo {ident.operator_id_type}"
            self._detail_operator_type_label.text = f"{op_type_name}"
        if self._detail_desc_label is not None:
            desc_str = f"\"{ident.self_id_desc}\"" if ident.self_id_desc else "(cadena vacía)"
            self._detail_desc_label.text = f"{desc_str}"
        if self._detail_model_label is not None:
            self._detail_model_label.text = f"{ident.model} (fleet/)"
        if self._detail_ports_label is not None:
            self._detail_ports_label.text = f"RX: {rx_port}  |  TX: {tx_port}"
        if self._detail_status_label is not None:
            if ident.is_valid:
                self._detail_status_label.text = "Conforme a ASTM F3411"
                self._detail_status_label.style = {"color": 0xFF44FF44}
            else:
                self._detail_status_label.text = "[ERR] Atributos incompletos"
                self._detail_status_label.style = {"color": 0xFFFF4444}

        if self._detail_errors_label is not None:
            if not ident.is_valid and ident.errors:
                err_lines = ["Atributos faltantes:"] + [f"  * {err}" for err in ident.errors]
                self._detail_errors_label.text = "\n".join(err_lines)
                self._detail_errors_label.visible = True
            else:
                self._detail_errors_label.text = ""
                self._detail_errors_label.visible = False

    def update_comms_simulation_state(self, is_running: bool):
        """Actualiza el indicador de estado de los canales de comunicación según el Timeline."""
        if self._comms_state_label is None:
            return
        if is_running:
            self._comms_state_label.text = "Activo (escuchando sockets)"
            self._comms_state_label.style = {"color": 0xFF44FF44}
        else:
            self._comms_state_label.text = "En espera (pulsa PLAY)"
            self._comms_state_label.style = {"color": 0xFFAAAAAA}

    def _update_comms_labels(self):
        """Calcula y actualiza dinámicamente los rangos de puertos UDP y el mapeo por UAV."""
        if self._comms_rx_range_label is None or self._comms_tx_range_label is None:
            return

        base_rx = getattr(self.extension, "base_rx_port", 15000) if self.extension else 15000
        base_tx = getattr(self.extension, "base_tx_port", 16000) if self.extension else 16000
        rate_hz = getattr(self.extension, "telemetry_rate_hz", 1.0) if self.extension else 1.0

        if self._rate_model is not None and not self._updating_models_internally:
            self._updating_models_internally = True
            try:
                self._rate_model.set_value(float(rate_hz))
            finally:
                self._updating_models_internally = False

        report = self._current_report
        identities = report.identities if (report and report.identities) else []
        n = len(identities)

        if n == 0:
            self._comms_rx_range_label.text = f"{base_rx}"
            self._comms_tx_range_label.text = f"{base_tx}"
            return

        if n == 1:
            self._comms_rx_range_label.text = f"{base_rx} (1 canal RX)"
            self._comms_tx_range_label.text = f"{base_tx} (1 canal TX)"
        else:
            self._comms_rx_range_label.text = f"{base_rx} .. {base_rx + n - 1} ({n} canales RX)"
            self._comms_tx_range_label.text = f"{base_tx} .. {base_tx + n - 1} ({n} canales TX)"

    def _on_fleet_frame_collapsed_changed(self, collapsed: bool):
        """Al expandir la sección Flota, escanea la escena USD y actualiza la lista automáticamente."""
        if not collapsed and self.extension is not None:
            self._on_refresh_drones_clicked()

    def _on_refresh_drones_clicked(self):
        """Vuelve a escanear el escenario, valida la flota ASTM F3411 y actualiza el desplegable de aeronaves."""
        if self.extension is None:
            return
        report = self.extension.validate_current_fleet()
        if hasattr(self.extension, "latest_validation_report"):
            self.extension.latest_validation_report = report
        self.update_validation_report(report)
        if report.has_conflicts:
            self.update_status("[CONFLICTO] Dos aeronaves comparten el mismo ID.")
        elif not report.is_valid:
            self.update_status(f"[AVISO] {len(report.incomplete_errors)} atributos faltantes en drones.")
        else:
            self.update_status(f"Flota actualizada: {report.uav_count} aeronave(s) conformes a ASTM F3411.")

    def _on_validate_fleet_clicked(self):
        """Compatibilidad con llamadas legacy: redirige a _on_refresh_drones_clicked."""
        self._on_refresh_drones_clicked()

    def sync_georef_ui(self):
        """Sincroniza los campos de la interfaz con los valores actuales de self.extension.georeference."""
        if self.extension is None or self._lat_model is None:
            return
        georef = getattr(self.extension, "georeference", None)
        if georef is None:
            return
        self._updating_models_internally = True
        try:
            self._lat_model.set_value(georef.lat0)
            self._lon_model.set_value(georef.lon0)
            self._alt_model.set_value(georef.alt0)
            if self._georef_source_label is not None:
                self._georef_source_label.text = f"{georef.source}"
        finally:
            self._updating_models_internally = False

        is_running = bool(getattr(self.extension, "is_simulation_running", False))
        is_paused = False
        curr_time = 0.0
        if hasattr(self.extension, "timeline") and self.extension.timeline:
            curr_time = self.extension.timeline.get_current_time()
            if hasattr(self.extension.timeline, "is_paused"):
                is_paused = bool(self.extension.timeline.is_paused())
        self.update_sim_time(curr_time, is_playing=is_running and not is_paused, is_paused=is_paused)

    def _on_georef_field_changed(self):
        if self._updating_models_internally or self._lat_model is None or self.extension is None:
            return
        lat = self._lat_model.get_value_as_float()
        lon = self._lon_model.get_value_as_float()
        alt = self._alt_model.get_value_as_float()
        georef = getattr(self.extension, "georeference", None)
        if georef is not None:
            georef.set_origin(lat, lon, alt, source="Manual (Interfaz)")
        if self._georef_source_label is not None:
            self._georef_source_label.text = "Manual (Interfaz)"

    def _on_rate_field_changed(self):
        if self._updating_models_internally or self._rate_model is None or self.extension is None:
            return
        rate = max(0.1, min(100.0, float(self._rate_model.get_value_as_float())))
        self.extension.set_telemetry_rate(rate)

    def _on_detect_georef_clicked(self):
        if self.extension is None:
            return
        detected = self.extension.detect_and_apply_stage_georeference()
        if detected:
            self.sync_georef_ui()
        elif self._georef_source_label is not None:
            georef = getattr(self.extension, "georeference", None)
            source_str = getattr(georef, "source", "WGS84") if georef else "WGS84"
            self._georef_source_label.text = f"No detectado en USD ({source_str})"

    def _on_reset_albacete_clicked(self):
        if self.extension is None:
            return
        georef = getattr(self.extension, "georeference", None)
        if georef is not None:
            georef.set_origin(
                DEFAULT_CAMPUS_ALBACETE_LAT,
                DEFAULT_CAMPUS_ALBACETE_LON,
                DEFAULT_CAMPUS_ALBACETE_ALT,
                source="Campus Albacete (UCLM)",
            )
        self.sync_georef_ui()

    def sync_time_ui(self):
        """Sincroniza los campos de fecha y hora inicial y final con los valores de time_reference."""
        if self.extension is None or self._date_model is None or self._time_model is None:
            return
        time_ref = getattr(self.extension, "time_reference", None)
        if time_ref is None:
            return
        self._updating_models_internally = True
        try:
            self._date_model.set_value(time_ref.get_date_str())
            self._time_model.set_value(time_ref.get_time_str())
            if self._actual_date_model is not None:
                self._actual_date_model.set_value(time_ref.get_date_str())
            if self._actual_time_model is not None:
                self._actual_time_model.set_value(f"{time_ref.get_time_str()} UTC")
            if self._end_date_model is not None:
                self._end_date_model.set_value(time_ref.get_end_date_str())
            if self._end_time_model is not None:
                self._end_time_model.set_value(time_ref.get_end_time_str())
        finally:
            self._updating_models_internally = False

        is_running = bool(getattr(self.extension, "is_simulation_running", False))
        curr_time = 0.0
        if is_running and hasattr(self.extension, "timeline") and self.extension.timeline:
            curr_time = self.extension.timeline.get_current_time()
        self.update_sim_time(curr_time, is_playing=is_running, is_paused=False)

    def _on_time_field_changed(self):
        """Callback cuando cambia el modelo de fecha/hora (ej. desde popups interactivos)."""
        if self._updating_models_internally or self._date_model is None or self._time_model is None or self.extension is None:
            return
        date_str = self._date_model.get_value_as_string().strip()
        time_str = self._time_model.get_value_as_string().strip()
        time_ref = getattr(self.extension, "time_reference", None)
        if time_ref is not None:
            ok = time_ref.set_datetime(date_str, time_str, source="Manual (Interfaz)")
            if ok:
                if getattr(self.extension, "mavlink_manager", None) is not None:
                    self.extension.mavlink_manager.set_time_reference(time_ref)

    def _on_date_field_end_edit(self):
        """Callback al finalizar la edición manual de la fecha (Enter o pérdida de foco).
        Si la fecha ingresada es inválida o errónea, cancela la operación y restablece el valor anterior."""
        if self._updating_models_internally or self._date_model is None or self.extension is None:
            return
        time_ref = getattr(self.extension, "time_reference", None)
        if time_ref is None:
            return

        raw_date = self._date_model.get_value_as_string().strip()
        valid_date = NavSimTimeReference.parse_date_str(raw_date)

        if not valid_date:
            # Cancelar operación y revertir al valor anterior correcto
            prev_date = time_ref.get_date_str()
            self._updating_models_internally = True
            try:
                self._date_model.set_value(prev_date)
            finally:
                self._updating_models_internally = False

            msg = f"[ERROR] Fecha inválida '{raw_date}'. Formato requerido: AAAA-MM-DD. Operación cancelada; restaurado a '{prev_date}'."
            if self.extension and getattr(self.extension, "logger", None):
                self.extension.logger.warning(msg)
            self.update_status(msg)
            return

        # Fecha válida: aplicar conservando la hora actual de simulación
        time_str = time_ref.get_time_str()
        ok = time_ref.set_datetime(valid_date, time_str, source="Manual (Interfaz)")
        if ok:
            # Normalizar formato en el cuadro de texto
            self._updating_models_internally = True
            try:
                self._date_model.set_value(valid_date)
            finally:
                self._updating_models_internally = False

            if getattr(self.extension, "mavlink_manager", None) is not None:
                self.extension.mavlink_manager.set_time_reference(time_ref)
            self.sync_time_ui()
            self.update_status(f"Fecha configurada a {valid_date}.")
        else:
            prev_date = time_ref.get_date_str()
            self._updating_models_internally = True
            try:
                self._date_model.set_value(prev_date)
            finally:
                self._updating_models_internally = False

    def _on_time_field_end_edit(self):
        """Callback al finalizar la edición manual de la hora (Enter o pérdida de foco).
        Si la hora ingresada es inválida o errónea, cancela la operación y restablece el valor anterior."""
        if self._updating_models_internally or self._time_model is None or self.extension is None:
            return
        time_ref = getattr(self.extension, "time_reference", None)
        if time_ref is None:
            return

        raw_time = self._time_model.get_value_as_string().strip()
        valid_time = NavSimTimeReference.parse_time_str(raw_time)

        if not valid_time:
            # Cancelar operación y revertir al valor anterior correcto
            prev_time = time_ref.get_time_str()
            self._updating_models_internally = True
            try:
                self._time_model.set_value(prev_time)
            finally:
                self._updating_models_internally = False

            msg = f"[ERROR] Hora inválida '{raw_time}'. Formato requerido: HH:MM:SS. Operación cancelada; restaurado a '{prev_time}'."
            if self.extension and getattr(self.extension, "logger", None):
                self.extension.logger.warning(msg)
            self.update_status(msg)
            return

        # Hora válida: aplicar conservando la fecha actual de simulación
        date_str = time_ref.get_date_str()
        ok = time_ref.set_datetime(date_str, valid_time, source="Manual (Interfaz)")
        if ok:
            # Normalizar formato en el cuadro de texto
            self._updating_models_internally = True
            try:
                self._time_model.set_value(valid_time)
            finally:
                self._updating_models_internally = False

            if getattr(self.extension, "mavlink_manager", None) is not None:
                self.extension.mavlink_manager.set_time_reference(time_ref)
            self.sync_time_ui()
            self.update_status(f"Hora configurada a {valid_time}.")
        else:
            prev_time = time_ref.get_time_str()
            self._updating_models_internally = True
            try:
                self._time_model.set_value(prev_time)
            finally:
                self._updating_models_internally = False

    def _on_reset_system_time_clicked(self):
        if self.extension is None:
            return
        if getattr(self.extension, "is_simulation_running", False):
            return
        self.extension.reset_time_to_system_clock()
        self.sync_time_ui()

    def _on_window_visibility_changed(self, visible: bool = False):
        if getattr(self, "extension", None) is not None and hasattr(self.extension, "on_window_visibility_changed"):
            try:
                self.extension.on_window_visibility_changed(visible)
            except Exception:
                pass

    def _on_focus_drone_clicked(self):
        """Enfoca la cámara del Viewport activo sobre el UAV seleccionado actualmente en la lista."""
        report = self._current_report
        prim_path = None
        if report and report.identities and 0 <= self._selected_drone_idx < len(report.identities):
            ident = report.identities[self._selected_drone_idx]
            prim_path = getattr(ident, "prim_path", None)
        elif self.extension:
            rpv = getattr(self.extension, "rigid_prim_view", None)
            if rpv and hasattr(rpv, "prims") and 0 <= self._selected_drone_idx < len(rpv.prims):
                prim = rpv.prims[self._selected_drone_idx]
                if prim and prim.IsValid():
                    prim_path = prim.GetPath().pathString

        if prim_path and self.extension and hasattr(self.extension, "focus_prim_in_viewport"):
            self.extension.focus_prim_in_viewport(prim_path)

    def update_sim_time(self, sim_time: float, is_playing: bool = False, is_paused: bool = False):
        """Actualiza los campos informativos de fecha y hora del instante actual y los estados de los botones."""
        if self._actual_date_model is None or self._actual_time_model is None:
            return

        time_ref = getattr(self.extension, "time_reference", None) if self.extension else None

        if time_ref is not None:
            dt = time_ref.get_datetime(sim_time)
            self._actual_date_model.set_value(dt.strftime("%Y-%m-%d"))
            self._actual_time_model.set_value(f"{dt.strftime('%H:%M:%S')} UTC")

        self.update_time_buttons_state(is_running=is_playing, is_paused=is_paused)

    def update_time_buttons_state(self, is_running: bool, is_paused: bool = False):
        """
        Actualiza el estado habilitado/deshabilitado y texto de los botones del panel de Tiempo.
        - Detenido (Stop):
            * Sistema: Habilitado
            * +1 hora: Habilitado
            * Stop: Deshabilitado (fondo #202020, texto #606060, sin hover)
            * Play: Habilitado, texto 'Play'
        - En ejecución (Play):
            * Sistema: Deshabilitado (fondo #202020, texto #606060, sin hover)
            * +1 hora: Deshabilitado (fondo #202020, texto #606060, sin hover)
            * Stop: Habilitado
            * Play: Habilitado, texto 'Pause'
        - En pausa (Pause):
            * Sistema: Deshabilitado (fondo #202020, texto #606060, sin hover)
            * +1 hora: Deshabilitado (fondo #202020, texto #606060, sin hover)
            * Stop: Habilitado
            * Play: Habilitado, texto 'Play'
        """
        is_active = is_running or is_paused

        if self._btn_play_pause is not None:
            self._btn_play_pause.text = "Pause" if (is_running and not is_paused) else "Play"
            self._btn_play_pause.enabled = True

        if self._btn_stop is not None:
            self._btn_stop.enabled = is_active

        if self._btn_system is not None:
            self._btn_system.enabled = not is_active

        if self._btn_plus_hour is not None:
            self._btn_plus_hour.enabled = not is_active

    def update_play_pause_button(self, is_playing: bool):
        """Mantiene compatibilidad delegando en update_time_buttons_state."""
        is_paused = False
        if self.extension and hasattr(self.extension, "timeline") and self.extension.timeline:
            is_paused = bool(getattr(self.extension.timeline, "is_paused", lambda: False)())
        self.update_time_buttons_state(is_running=is_playing, is_paused=is_paused)

    def _on_play_pause_clicked(self):
        """Gestiona la pulsación del botón PLAY/PAUSE."""
        if self.extension and hasattr(self.extension, "play_or_pause_simulation"):
            self.extension.play_or_pause_simulation()

    def _on_stop_clicked(self):
        """Gestiona la pulsación del botón Stop."""
        if self.extension and hasattr(self.extension, "stop_simulation"):
            if not getattr(self.extension, "is_simulation_running", False):
                return
            self.extension.stop_simulation()

    def _on_plus_one_hour_clicked(self):
        """Suma 1 hora acumulativa al instante final y sincroniza la interfaz."""
        if self.extension is None:
            return
        if getattr(self.extension, "is_simulation_running", False):
            return
        time_ref = getattr(self.extension, "time_reference", None)
        if time_ref is None:
            return
        time_ref.set_end_plus_hours(1.0)
        self.sync_time_ui()
        dur_h = time_ref.get_target_duration_seconds() / 3600.0
        self.update_status(
            f"Instante final extendido a {time_ref.get_end_date_str()} {time_ref.get_end_time_str()} (duración: {dur_h:.1f} h)."
        )

    def _on_end_date_field_end_edit(self):
        """Valida y aplica la fecha del instante final."""
        if self._updating_models_internally or self._end_date_model is None or self.extension is None:
            return
        time_ref = getattr(self.extension, "time_reference", None)
        if time_ref is None:
            return
        raw_date = self._end_date_model.get_value_as_string().strip()
        raw_time = self._end_time_model.get_value_as_string().strip() if self._end_time_model else time_ref.get_end_time_str()
        ok = time_ref.set_end_datetime(raw_date, raw_time)
        if not ok:
            self._updating_models_internally = True
            try:
                self._end_date_model.set_value(time_ref.get_end_date_str())
            finally:
                self._updating_models_internally = False
        else:
            self.sync_time_ui()

    def _on_end_time_field_end_edit(self):
        """Valida y aplica la hora del instante final."""
        if self._updating_models_internally or self._end_time_model is None or self.extension is None:
            return
        time_ref = getattr(self.extension, "time_reference", None)
        if time_ref is None:
            return
        raw_date = self._end_date_model.get_value_as_string().strip() if self._end_date_model else time_ref.get_end_date_str()
        raw_time = self._end_time_model.get_value_as_string().strip()
        ok = time_ref.set_end_datetime(raw_date, raw_time)
        if not ok:
            self._updating_models_internally = True
            try:
                self._end_time_model.set_value(time_ref.get_end_time_str())
            finally:
                self._updating_models_internally = False
        else:
            self.sync_time_ui()

    def cleanup(self):
        """Libera todos los recursos visuales y rompe la referencia a NavSim para evitar fugas en Kit."""
        if self.window is not None:
            try:
                if getattr(self.window, "set_visibility_changed_listener", None):
                    self.window.set_visibility_changed_listener(None)
                elif hasattr(self.window, "set_visibility_changed_fn"):
                    self.window.set_visibility_changed_fn(None)
            except Exception:
                pass
            try:
                self.window.destroy()
            except Exception:
                pass
            self.window = None

        self._lat_model = None
        self._lon_model = None
        self._alt_model = None
        self._rate_model = None
        self._date_model = None
        self._time_model = None
        self._georef_source_label = None
        self._actual_date_field = None
        self._actual_time_field = None
        self._actual_date_model = None
        self._actual_time_model = None
        self._btn_system = None
        self._btn_play_pause = None
        self._btn_stop = None
        self._btn_plus_hour = None
        self._end_date_model = None
        self._end_time_model = None
        self._validation_banner_label = None
        self._validation_details_label = None
        self._drone_combo_frame = None
        self._drone_combo = None
        self._drone_details_frame = None
        self._detail_title_label = None
        self._detail_prim_label = None
        self._detail_uas_id_label = None
        self._detail_id_type_label = None
        self._detail_ua_type_label = None
        self._detail_operator_id_label = None
        self._detail_operator_type_label = None
        self._detail_desc_label = None
        self._detail_model_label = None
        self._detail_ports_label = None
        self._detail_status_label = None
        self._detail_errors_label = None
        self._comms_state_label = None
        self._comms_rate_label = None
        self._comms_rx_range_label = None
        self._comms_tx_range_label = None
        self._current_report = None
        self._selected_drone_uas_id = None
        self._fleet_frame = None
        self.extension = None
