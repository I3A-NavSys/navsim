from isaacsim.gui.components.ui_utils import ui
from .georeference import (
    DEFAULT_CAMPUS_ALBACETE_LAT,
    DEFAULT_CAMPUS_ALBACETE_LON,
    DEFAULT_CAMPUS_ALBACETE_ALT,
)

WINDOW_DARK_STYLE = {"Window": {"background_color": 0xFF383838}}
SPACING_S = 8


class UIBuilder:
    """
    Interfaz de monitorización y configuración para la extensión NavSim.
    Muestra:
      1. Catálogo de drones cargados en fleet/.
      2. Estado y endpoints MAVLink de la simulación activa.
      3. Georreferenciación WGS-84 (origen cartesiano 0,0,0 sobre el elipsoide).
      4. Validación de Flota ASTM F3411 / EUROCAE ED-282 (detección de conflictos en tiempo real).
    """

    def __init__(self, extension):
        self.extension = extension
        self.window = None
        self._status_label = None
        self._georef_source_label = None

        # Componentes de validación de identidad ASTM F3411
        self._validation_banner_label = None
        self._validation_details_label = None
        self._uav_roster_label = None

        georef = getattr(self.extension, "georeference", None)
        lat0 = getattr(georef, "lat0", DEFAULT_CAMPUS_ALBACETE_LAT) if georef else DEFAULT_CAMPUS_ALBACETE_LAT
        lon0 = getattr(georef, "lon0", DEFAULT_CAMPUS_ALBACETE_LON) if georef else DEFAULT_CAMPUS_ALBACETE_LON
        alt0 = getattr(georef, "alt0", DEFAULT_CAMPUS_ALBACETE_ALT) if georef else DEFAULT_CAMPUS_ALBACETE_ALT

        # Modelos reactivos para los campos numéricos WGS-84
        self._lat_model = ui.SimpleFloatModel(lat0)
        self._lon_model = ui.SimpleFloatModel(lon0)
        self._alt_model = ui.SimpleFloatModel(alt0)

        self._updating_models_internally = False
        self._lat_model.add_value_changed_fn(lambda m: self._on_georef_field_changed())
        self._lon_model.add_value_changed_fn(lambda m: self._on_georef_field_changed())
        self._alt_model.add_value_changed_fn(lambda m: self._on_georef_field_changed())

    def build_ui(self):
        self.window = ui.Window("NavSim — Control & MAVLink", width=500, height=520)
        self.window.frame.set_style(WINDOW_DARK_STYLE)

        with self.window.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(height=0, spacing=SPACING_S):
                    # --- Sección 1: Catálogo y Estado ---
                    fleet_cat = getattr(self.extension, "fleet_catalog", None)
                    models = list(fleet_cat.models.keys()) if (fleet_cat and hasattr(fleet_cat, "models")) else []
                    ui.Label(
                        f"Modelos en catálogo fleet/: {', '.join(models)}",
                        word_wrap=True,
                    )
                    self._status_label = ui.Label(
                        "Estado: Simulación detenida (pulsa PLAY para iniciar RigidPrim y MAVLink UDP).",
                        word_wrap=True,
                    )
                    ui.Button(
                        text="Recargar catálogo dynamics.yaml",
                        clicked_fn=self._on_reload_catalog,
                    )

                    ui.Spacer(height=4)
                    ui.Line(style={"color": 0x44FFFFFF})
                    ui.Spacer(height=4)

                    # --- Sección 2: Validación ASTM F3411 y Conflictos de Flota ---
                    ui.Label(
                        "Identificación de Flota ASTM F3411 / ED-282",
                        style={"font_size": 14, "color": 0xFF88DDFF},
                    )

                    self._validation_banner_label = ui.Label(
                        "Estado de Identificación: Sin verificar (pulsa el botón o PLAY)",
                        word_wrap=True,
                        style={"font_size": 13, "color": 0xFFAAAAAA},
                    )

                    self._validation_details_label = ui.Label(
                        "",
                        word_wrap=True,
                        style={"font_size": 12, "color": 0xFFFF6666},
                    )

                    self._uav_roster_label = ui.Label(
                        "",
                        word_wrap=True,
                        style={"font_size": 11, "color": 0xFFCCCCCC},
                    )

                    ui.Button(
                        text="🔍 Verificar Identificación de Flota (ASTM F3411)",
                        clicked_fn=self._on_validate_fleet_clicked,
                    )

                    ui.Spacer(height=4)
                    ui.Line(style={"color": 0x44FFFFFF})
                    ui.Spacer(height=4)

                    # --- Sección 3: Georreferenciación WGS-84 ---
                    ui.Label(
                        "Georreferenciación WGS-84 (Origen 0,0,0)",
                        style={"font_size": 14, "color": 0xFF88DDFF},
                    )
                    georef = getattr(self.extension, "georeference", None)
                    source_str = getattr(georef, "source", "WGS84") if georef else "WGS84"
                    self._georef_source_label = ui.Label(
                        f"Origen: {source_str}",
                        word_wrap=True,
                    )

                    with ui.VStack(spacing=4):
                        with ui.HStack():
                            ui.Label("Latitud [°]:", width=130)
                            ui.FloatField(model=self._lat_model, format="%.6f", step=0.0001)

                        with ui.HStack():
                            ui.Label("Longitud [°]:", width=130)
                            ui.FloatField(model=self._lon_model, format="%.6f", step=0.0001)

                        with ui.HStack():
                            ui.Label("Altitud WGS-84 [m]:", width=130)
                            ui.FloatField(model=self._alt_model, format="%.2f", step=1.0)

                    with ui.HStack(spacing=6):
                        ui.Button(
                            text="Detectar desde USD",
                            clicked_fn=self._on_detect_georef_clicked,
                        )
                        ui.Button(
                            text="Restablecer a Albacete",
                            clicked_fn=self._on_reset_albacete_clicked,
                        )

    def update_status(self, text: str):
        if self._status_label is not None:
            self._status_label.text = text

    def update_validation_report(self, report):
        """Actualiza el panel de validación ASTM F3411 y los avisos de conflictos en pantalla."""
        if self._validation_banner_label is None:
            return

        if report.uav_count == 0:
            self._validation_banner_label.text = "Estado: No se encontraron aeronaves (UAV_*) en la escena."
            self._validation_banner_label.style = {"color": 0xFF888888, "font_size": 12}
            if self._validation_details_label is not None:
                self._validation_details_label.text = ""
            if self._uav_roster_label is not None:
                self._uav_roster_label.text = ""
            return

        # 1. Caso Conflicto Crítico de Identificador Duplicado
        if report.has_conflicts:
            self._validation_banner_label.text = "🚨 CONFLICTO CRÍTICO: IDENTIFICADOR DUPLICADO EN EL AIRE"
            self._validation_banner_label.style = {"color": 0xFFFF3333, "font_size": 13}
            details = [
                "¡PELIGRO DE COLISIÓN DE IDENTIDAD AÉREA!",
                "El estándar internacional ASTM F3411 prohíbe que dos aeronaves compartan",
                "el mismo identificador de vuelo simultáneamente. Simulación BLOQUEADA.",
                "",
                "Detalles del conflicto:",
            ]
            for c in report.conflicts:
                details.append(f"  • {c}")
            if report.incomplete_errors:
                details.append("")
                details.append("Otros atributos incompletos detectados:")
                for e in report.incomplete_errors:
                    details.append(f"  - {e}")

            if self._validation_details_label is not None:
                self._validation_details_label.text = "\n".join(details)
                self._validation_details_label.style = {"color": 0xFFFF6666, "font_size": 11}

        # 2. Caso Incompleto / Atributos Faltantes (sin colisión de IDs)
        elif not report.is_valid:
            self._validation_banner_label.text = "⚠️ IDENTIFICACIÓN INCOMPLETA / ATRIBUTOS FALTANTES (ASTM F3411)"
            self._validation_banner_label.style = {"color": 0xFFFFAA00, "font_size": 13}
            details = [
                f"Detectados {len(report.incomplete_errors)} atributo(s) no conforme(s) al estándar ASTM F3411 / ED-282:",
            ]
            for e in report.incomplete_errors:
                details.append(f"  • {e}")
            for w in report.warnings:
                details.append(f"  (Aviso) {w}")
            if self._validation_details_label is not None:
                self._validation_details_label.text = "\n".join(details)
                self._validation_details_label.style = {"color": 0xFFFFCC44, "font_size": 11}

        # 3. Flota Conforme y Válida
        else:
            self._validation_banner_label.text = (
                f"✅ IDENTIFICACIÓN VÁLIDA: {report.uav_count} aeronave(s) conforme(s) a ASTM F3411"
            )
            self._validation_banner_label.style = {"color": 0xFF44FF44, "font_size": 13}
            if self._validation_details_label is not None:
                self._validation_details_label.text = (
                    "Todos los identificadores son únicos y todos los atributos obligatorios están presentes."
                )
                self._validation_details_label.style = {"color": 0xFF88FF88, "font_size": 11}

        # Resumen de aeronaves detectadas
        roster_lines = ["Aeronaves inspeccionadas en el escenario:"]
        for idx, ident in enumerate(report.identities, 1):
            mark = "✓" if ident.is_valid else "✗"
            roster_lines.append(
                f"  [{idx}] {mark} {ident.uas_id} ({ident.id_type_name}) | "
                f"{ident.ua_type_name} | Op: {ident.operator_id or 'NO ASIGNADO'}"
            )
        if self._uav_roster_label is not None:
            self._uav_roster_label.text = "\n".join(roster_lines)

    def _on_validate_fleet_clicked(self):
        if self.extension is None:
            return
        report = self.extension.validate_current_fleet()
        if report.has_conflicts:
            self.update_status("🚨 Conflicto detectado: Dos aeronaves comparten el mismo ID.")
        elif not report.is_valid:
            self.update_status(f"Aviso: {len(report.incomplete_errors)} atributos faltantes en drones.")
        else:
            self.update_status(f"Validación correcta: {report.uav_count} drones conformes a ASTM F3411.")

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
                self._georef_source_label.text = f"Origen: {georef.source}"
        finally:
            self._updating_models_internally = False

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
            self._georef_source_label.text = "Origen: Manual (Interfaz)"

    def _on_detect_georef_clicked(self):
        if self.extension is None:
            return
        detected = self.extension.detect_and_apply_stage_georeference()
        if not detected and self._georef_source_label is not None:
            georef = getattr(self.extension, "georeference", None)
            source_str = getattr(georef, "source", "WGS84") if georef else "WGS84"
            self._georef_source_label.text = (
                f"Origen: No detectado en USD (se mantiene {source_str})"
            )

    def _on_reset_albacete_clicked(self):
        if self.extension is None:
            return
        georef = getattr(self.extension, "georeference", None)
        if georef is not None:
            georef.set_origin(
                DEFAULT_CAMPUS_ALBACETE_LAT,
                DEFAULT_CAMPUS_ALBACETE_LON,
                DEFAULT_CAMPUS_ALBACETE_ALT,
                source="Predefinido (Campus Albacete)",
            )
        self.sync_georef_ui()

    def _on_reload_catalog(self):
        if self.extension is None:
            return
        fleet_cat = getattr(self.extension, "fleet_catalog", None)
        if fleet_cat is not None:
            fleet_cat.reload()
            models = list(fleet_cat.models.keys())
        else:
            models = []
        self.update_status(f"Catálogo recargado: {', '.join(models)}")

    def cleanup(self):
        """Libera todos los recursos visuales y rompe la referencia a NavSim para evitar fugas en Kit."""
        if self.window is not None:
            try:
                self.window.destroy()
            except Exception:
                pass
            self.window = None

        self._lat_model = None
        self._lon_model = None
        self._alt_model = None
        self._status_label = None
        self._georef_source_label = None
        self._validation_banner_label = None
        self._validation_details_label = None
        self._uav_roster_label = None
        self.extension = None
