from __future__ import annotations

import carb
import gc
import importlib
import logging
from pathlib import Path
import sys
import traceback
from typing import Any, Dict, List, Optional, Set, Tuple
import warnings

import omni.ext
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.prims import find_matching_prim_paths

# Garantizar que la raíz del repositorio está en sys.path
_REPO_ROOT = str(Path(__file__).resolve().parents[3])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from .drone_config import FleetCatalog
from .drone_identity import FleetValidationReport, inspect_prim_identity, validate_fleet_prims
from .georeference import WGS84Georeference
from .uav_control import UAVControl
from .ui_builder import UIBuilder


UAV_PRIM_PATTERNS = [
    "/World/*/*/*/UAV_*",
    "/World/*/*/UAV_*",
    "/World/*/UAV_*",
    "/World/UAV_*",
]


_ACTIVE_NAVSIM_INSTANCE: Optional[NavSim] = None


def _deduplicate_and_patch_navsim_instances(current: Optional[NavSim] = None) -> None:
    """Mantiene la referencia activa a la instancia NavSim actual."""
    global _ACTIVE_NAVSIM_INSTANCE
    if current is not None:
        _ACTIVE_NAVSIM_INSTANCE = current


class NavSim(omni.ext.IExt):
    """
    Extensión única de simulación NavSim en Isaac Sim.
    Responsabilidades exclusivas:
      1. Descubrir los UAVs en la escena USD y validar su identidad y unicidad según
         el estándar internacional ASTM F3411 / EUROCAE ED-282 (Open Remote ID).
      2. Configurar la dinámica de cada aeronave a partir de `fleet/<modelo>/dynamics.yaml`.
      3. Ejecutar el bucle vectorizado de control lineal y física de cuerpo rígido (`RigidPrim`).
      4. Recibir planes de vuelo polinómicos y emitir telemetría mediante MAVLink v2 UDP
         (un puerto UDP por dron: `14550 + idx`).
    """

    # Atributos por defecto a nivel de clase para garantizar que cualquier acceso
    # a nivel de instancia retorne None/valor por defecto en vez de un AttributeError
    ui_builder: Optional[UIBuilder] = None
    mavlink_manager: Optional[Any] = None
    uav_control: Optional[Any] = None
    rigid_prim_view: Optional[Any] = None
    latest_validation_report: Optional[Any] = None
    is_simulation_running: bool = False
    fleet_catalog: Optional[FleetCatalog] = None
    georeference: Optional[WGS84Georeference] = None
    timeline: Optional[Any] = None
    logger: Optional[logging.Logger] = None
    base_rx_port: int = 14550
    base_tx_port: int = 14650
    telemetry_rate_hz: float = 20.0

    def _ensure_initialized(self) -> None:
        """Garantiza la inicialización segura de componentes de la extensión."""
        # 1. Hot-reload opcional de submódulos de datos si están en sys.modules (NUNCA extension ni ui_builder)
        for mod_name in (
            "mavlink.dialect",
            "mavlink.udp_manager",
            "extensions.navsim.navsim_python.drone_identity",
            "extensions.navsim.navsim_python.georeference",
            "navsim_python.drone_identity",
            "navsim_python.georeference",
        ):
            if mod_name in sys.modules:
                try:
                    importlib.reload(sys.modules[mod_name])
                except Exception:
                    pass

        # 2. Inicialización segura de atributos esenciales
        if getattr(self, "logger", None) is None:
            self.logger = logging.getLogger("NavSim")
        if not hasattr(self, "is_simulation_running"):
            self.is_simulation_running = False
        if not hasattr(self, "latest_validation_report"):
            self.latest_validation_report = None
        if not hasattr(self, "base_rx_port"):
            self.base_rx_port = 14550
        if not hasattr(self, "base_tx_port"):
            self.base_tx_port = 14650
        if not hasattr(self, "telemetry_rate_hz"):
            self.telemetry_rate_hz = 20.0
        if getattr(self, "timeline", None) is None:
            self.timeline = omni.timeline.get_timeline_interface()
        if getattr(self, "fleet_catalog", None) is None:
            self.fleet_catalog = FleetCatalog()
        if getattr(self, "georeference", None) is None:
            self.georeference = WGS84Georeference()
        if getattr(self, "ui_builder", None) is None:
            try:
                self.ui_builder = UIBuilder(self)
                self.ui_builder.build_ui()
            except Exception as e:
                if self.logger:
                    self.logger.warning(f"No se pudo inicializar UIBuilder: {e}")
                self.ui_builder = None

    def on_startup(self, ext_id: str) -> None:
        global _ACTIVE_NAVSIM_INSTANCE
        _ACTIVE_NAVSIM_INSTANCE = self

        self.logger = logging.getLogger("NavSim")
        self.is_simulation_running = False
        self.latest_validation_report = None

        self.fleet_catalog = FleetCatalog()
        self.rigid_prim_view = None
        self.uav_control = None
        self.mavlink_manager = None

        self.base_rx_port = 14550
        self.base_tx_port = 14650
        self.telemetry_rate_hz = 20.0

        # Suscripciones al Timeline de Isaac Sim
        self.timeline = omni.timeline.get_timeline_interface()
        timeline_stream = self.timeline.get_timeline_event_stream()
        self.on_play_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self.on_timeline_play
        )
        self.on_pause_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PAUSE), self.on_timeline_pause
        )
        self.on_stop_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_timeline_stop
        )

        # Suscripción al paso de física de PhysX
        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(
            fn=lambda step_size: self.on_physics_step(step_size),
            pre_step=True,
            order=10,
        )

        # Suscripción a eventos de Stage USD (abrir/recargar/modificar escenas)
        self.usd_context = omni.usd.get_context()
        self.on_stage_event_sub = (
            self.usd_context.get_stage_event_stream().create_subscription_to_pop(
                self.on_stage_event
            )
        )

        # Georreferenciación WGS-84 (origen cartesiano 0,0,0)
        self.georeference = WGS84Georeference()
        try:
            self.detect_and_apply_stage_georeference()
        except Exception as e:
            if self.logger:
                self.logger.warning(f"Error detectando georreferenciación en startup: {e}")

        # UI de monitorización y georreferenciación
        try:
            self.ui_builder = UIBuilder(self)
            self.ui_builder.build_ui()
        except Exception as e:
            if self.logger:
                self.logger.warning(f"Error construyendo UIBuilder en startup: {e}")
            self.ui_builder = None

        # Inspección y validación inicial de flota
        try:
            self.validate_current_fleet()
        except Exception as e:
            if self.logger:
                self.logger.warning(f"Error validando flota en startup: {e}")

    def on_shutdown(self) -> None:
        global _ACTIVE_NAVSIM_INSTANCE
        if _ACTIVE_NAVSIM_INSTANCE is self:
            _ACTIVE_NAVSIM_INSTANCE = None
        self._teardown_simulation()
        if hasattr(self, "ui_builder") and self.ui_builder is not None:
            try:
                self.ui_builder.cleanup()
            except Exception:
                pass
            self.ui_builder = None
        for sub_name in ("on_play_sub", "on_pause_sub", "on_stop_sub", "on_physics_step_sub", "on_stage_event_sub"):
            sub = getattr(self, sub_name, None)
            if sub is not None:
                try:
                    if hasattr(sub, "unsubscribe"):
                        sub.unsubscribe()
                except Exception:
                    pass
                setattr(self, sub_name, None)
        self.physx_interface = None
        self.usd_context = None
        self.timeline = None

    def post_notification(self, message: str, is_error: bool = False, duration: int = 8) -> None:
        """Muestra una notificación en la interfaz de Omniverse Kit y registra en el log de carb."""
        if is_error:
            carb.log_error(f"[NavSim] {message}")
        else:
            carb.log_warn(f"[NavSim] {message}")
        try:
            import omni.kit.notification_manager as nm
            status = nm.NotificationStatus.ERROR if is_error else nm.NotificationStatus.WARNING
            nm.post_notification(message, status=status, duration=duration)
        except Exception:
            pass

    def validate_current_fleet(self) -> FleetValidationReport:
        """
        Inspecciona todos los Prims de UAV en el escenario actual y valida:
          1. Duplicidad de identificadores UAS (conflicto crítico en el aire).
          2. Cumplimiento de atributos requeridos por ASTM F3411 / ED-282.
        Actualiza el reporte en la interfaz y emite notificaciones si corresponde.
        """
        self._ensure_initialized()
        if self.fleet_catalog is not None:
            self.fleet_catalog.reload()

        stage = omni.usd.get_context().get_stage()
        prims = []
        if stage is not None:
            found_paths = set()
            for pat in UAV_PRIM_PATTERNS:
                for p in find_matching_prim_paths(pat):
                    found_paths.add(p)

            for p in sorted(found_paths):
                prim = stage.GetPrimAtPath(p)
                if prim and prim.IsValid():
                    prims.append(prim)

        report = validate_fleet_prims(prims, fleet_catalog=self.fleet_catalog)
        self.latest_validation_report = report

        # Notificar en interfaz gráfica de NavSim
        ui = getattr(self, "ui_builder", None)
        if ui is not None:
            ui.update_validation_report(report)

        # Si hay conflictos críticos, emitir notificación visual inmediata
        if report.has_conflicts:
            conflict_summary = " | ".join(report.conflicts)
            self.post_notification(
                f"🚨 CONFLICTO ASTM F3411: Identificador duplicado en {len(report.conflicts)} caso(s). "
                f"Dos aeronaves no pueden emitir el mismo ID.",
                is_error=True,
                duration=10,
            )
            if self.logger:
                self.logger.error(f"[NavSim ASTM F3411 CONFLICTO] {conflict_summary}")
        elif not report.is_valid and report.uav_count > 0:
            self.post_notification(
                f"⚠️ AVISO ASTM F3411: {len(report.incomplete_errors)} error(es) de atributos en drones de la flota.",
                is_error=False,
                duration=6,
            )

        return report

    def on_stage_event(self, event) -> None:
        global _ACTIVE_NAVSIM_INSTANCE
        if _ACTIVE_NAVSIM_INSTANCE is not None and self is not _ACTIVE_NAVSIM_INSTANCE:
            return

        if event.type == int(omni.usd.StageEventType.OPENED):
            if self.logger:
                self.logger.info("Escenario USD cargado/abierto. Limpiando estado de extensión.")
            self._teardown_simulation()
            self.detect_and_apply_stage_georeference()
            self.validate_current_fleet()
            ui = getattr(self, "ui_builder", None)
            if ui is not None:
                ui.update_status("Estado: Escena cargada. Simulación lista (pulsa PLAY).")
        elif event.type in (int(omni.usd.StageEventType.SAVED), int(omni.usd.StageEventType.DIRTY_STATE_CHANGED)):
            self.validate_current_fleet()
        elif event.type == int(omni.usd.StageEventType.CLOSING):
            self._teardown_simulation()

    # ==========================================================================
    # Callbacks de Simulación y Física
    # ==========================================================================
    def on_physics_step(self, step_size: float) -> None:
        global _ACTIVE_NAVSIM_INSTANCE
        if _ACTIVE_NAVSIM_INSTANCE is not None and self is not _ACTIVE_NAVSIM_INSTANCE:
            return

        if not self.is_simulation_running or self.uav_control is None or self.mavlink_manager is None:
            return

        try:
            current_time = self.timeline.get_current_time()

            # 1. Sondear recepción no bloqueante de nuevos planes de vuelo polinómicos por MAVLink
            new_plans = self.mavlink_manager.poll_new_flight_plans()
            if new_plans:
                self.uav_control.update_flight_plans_batch(new_plans)

            # 2. Sondear consignas de control manual por MAVLink (joystick / teclado)
            manual_cmds = self.mavlink_manager.poll_manual_controls()
            if manual_cmds:
                for uav_idx, cmd in manual_cmds.items():
                    self.uav_control.set_manual_control(
                        uav_idx=uav_idx,
                        vel_x=cmd.vel_x,
                        vel_y=cmd.vel_y,
                        vel_z=cmd.vel_z,
                        rot_z=cmd.rot_z,
                        mode=cmd.mode,
                        sim_time=current_time,
                    )

            # 3. Paso vectorizado de control (AUTO + MANUAL), control lineal y fuerzas en CoM
            self.uav_control.step(current_time, step_size)

            # 4. Emitir telemetría MAVLink de vuelta por cada dron
            self.mavlink_manager.maybe_send_telemetry(
                sim_time=current_time,
                uav_states=self.uav_control.uav_states,
                plan_ids=self.uav_control.active_plan_ids,
                current_wps=self.uav_control.current_wp_indices,
                positions=self.uav_control.positions,
                eulers=self.uav_control.eulers,
                orientations=self.uav_control.orientations,
                world_lin_vels=self.uav_control.world_linear_vels,
                local_ang_vels=self.uav_control.local_angular_vels,
                rotors_vel=self.uav_control.rotors_vel,
            )
        except Exception as e:
            carb.log_error(f"[NavSim] Error en on_physics_step: {e}")

    def on_timeline_play(self, event) -> None:
        global _ACTIVE_NAVSIM_INSTANCE
        if _ACTIVE_NAVSIM_INSTANCE is not None and self is not _ACTIVE_NAVSIM_INSTANCE:
            return

        if self.is_simulation_running:
            return

        try:
            self._ensure_initialized()

            # Limpiar cualquier residuo previo antes de inicializar la nueva escena
            self._teardown_simulation()

            from mavlink.udp_manager import FleetMavlinkManager

            if self.fleet_catalog is not None:
                self.fleet_catalog.reload()
            self.detect_and_apply_stage_georeference()

            # 1. Validación exhaustiva de Flota ASTM F3411 / EUROCAE ED-282
            report = self.validate_current_fleet()

            if report.uav_count == 0:
                carb.log_warn("[NavSim] Aviso: No se encontraron Prims UAV_* en el escenario.")
                ui = getattr(self, "ui_builder", None)
                if ui is not None:
                    ui.update_status("Aviso: No se encontraron Prims UAV_* en el escenario.")
                return

            # 2. Bloquear simulación si existe conflicto crítico por ID duplicado en el aire
            if report.has_conflicts:
                if self.timeline is not None:
                    self.timeline.stop()
                err_msg = "🚨 DESPEGUE BLOQUEADO: Conflicto crítico de identificadores duplicados en el aire (ASTM F3411)."
                carb.log_error(f"[NavSim] {err_msg}")
                if self.logger:
                    self.logger.error(err_msg)
                ui = getattr(self, "ui_builder", None)
                if ui is not None:
                    ui.update_status(err_msg)
                return

            # Localizar patrones de Prims UAV existentes en el Stage actual
            active_patterns = [
                pat for pat in UAV_PRIM_PATTERNS if find_matching_prim_paths(pat)
            ]
            if not active_patterns:
                carb.log_warn("[NavSim] No se encontraron rutas activas de UAV.")
                return

            self.rigid_prim_view = RigidPrim(
                prim_paths_expr=active_patterns,
                name="NavSimRigidPrim",
            )
            self.rigid_prim_view.initialize()

            # Extraer identidades alineadas 1:1 con el orden de los prims en RigidPrim
            uav_identities = [
                inspect_prim_identity(prim, fleet_catalog=self.fleet_catalog)
                for prim in self.rigid_prim_view.prims
            ]
            uav_ids = [ident.uas_id for ident in uav_identities]
            model_names = [ident.model for ident in uav_identities]
            fleet_tensors = self.fleet_catalog.build_fleet_tensors(model_names)

            self.uav_control = UAVControl(
                rigid_prim_view=self.rigid_prim_view,
                fleet_tensors=fleet_tensors,
                uav_ids=uav_ids,
            )
            self.uav_control.apply_inertial_properties()

            self.mavlink_manager = FleetMavlinkManager(
                uav_ids=uav_ids,
                base_rx_port=self.base_rx_port,
                base_tx_port=self.base_tx_port,
                telemetry_rate_hz=self.telemetry_rate_hz,
                georeference=self.georeference,
                identities=uav_identities,
            )
            self.mavlink_manager.start()
            self.is_simulation_running = True

            rx_str = (
                f"{self.base_rx_port}"
                if len(uav_ids) == 1
                else f"{self.base_rx_port}..{self.base_rx_port + len(uav_ids) - 1}"
            )
            tx_str = (
                f"{self.base_tx_port}"
                if len(uav_ids) == 1
                else f"{self.base_tx_port}..{self.base_tx_port + len(uav_ids) - 1}"
            )
            status_text = f"Simulación activa: {len(uav_ids)} UAVs | RX: {rx_str} | TX Telemetría: {tx_str}"
            carb.log_info(f"[NavSim] {status_text}")
            ui = getattr(self, "ui_builder", None)
            if ui is not None:
                ui.update_status(status_text)
        except Exception as e:
            err = f"Error crítico iniciando simulación en on_timeline_play: {e}\n{traceback.format_exc()}"
            carb.log_error(f"[NavSim] {err}")
            if self.logger:
                self.logger.error(err)
            ui = getattr(self, "ui_builder", None)
            if ui is not None:
                ui.update_status(f"Error al iniciar: {e}")

    def on_timeline_pause(self, event) -> None:
        pass

    def on_timeline_stop(self, event) -> None:
        global _ACTIVE_NAVSIM_INSTANCE
        if _ACTIVE_NAVSIM_INSTANCE is not None and self is not _ACTIVE_NAVSIM_INSTANCE:
            return

        self._ensure_initialized()
        self._teardown_simulation()
        self.validate_current_fleet()
        carb.log_info("[NavSim] Simulación detenida.")
        ui = getattr(self, "ui_builder", None)
        if ui is not None:
            ui.update_status("Estado: Simulación detenida.")

    # ==========================================================================
    # Utilidades Internas
    # ==========================================================================
    def _inspect_uav_prims(self) -> Tuple[List[str], List[str]]:
        """
        Extrae el identificador y el tipo de modelo de cada Prim de UAV en la vista.
        Soporta los atributos USD `NavSim:model`, `NavSim:type`, `NavSim:id` o el nombre del Prim.
        """
        uav_ids: List[str] = []
        model_names: List[str] = []

        if self.rigid_prim_view is None or not hasattr(self.rigid_prim_view, "prims"):
            return uav_ids, model_names

        for idx, prim in enumerate(self.rigid_prim_view.prims):
            # 1. Identificador del UAV
            id_attr = prim.GetAttribute("NavSim:id")
            uav_id = id_attr.Get() if (id_attr and id_attr.IsValid()) else None
            if not uav_id:
                uav_id = prim.GetName() or f"UAV_{idx:02d}"
            uav_ids.append(str(uav_id))

            # 2. Modelo / tipo de dron para cargar su `dynamics.yaml`
            model_candidate = None
            for attr_name in ("NavSim:model", "NavSim:uav_model", "NavSim:type"):
                attr = prim.GetAttribute(attr_name)
                if attr and attr.IsValid() and attr.Get():
                    model_candidate = str(attr.Get())
                    break

            if not model_candidate:
                model_candidate = str(prim.GetPath())

            resolved_cfg = self.fleet_catalog.get(model_candidate)
            model_names.append(resolved_cfg.model_name)

        return uav_ids, model_names

    def _teardown_simulation(self) -> None:
        self.is_simulation_running = False
        if self.mavlink_manager is not None:
            self.mavlink_manager.stop()
            self.mavlink_manager = None
        self.uav_control = None
        self.rigid_prim_view = None

    def detect_and_apply_stage_georeference(self) -> bool:
        """
        Inspecciona el Stage USD actual buscando coordenadas geodésicas de origen.
        Si se detectan, actualiza self.georeference y la interfaz de usuario.
        """
        try:
            stage = omni.usd.get_context().get_stage()
        except Exception:
            stage = None

        lat, lon, alt, src, detected = WGS84Georeference.detect_from_stage(stage)
        if detected:
            if self.georeference is not None:
                self.georeference.set_origin(lat, lon, alt, source=src)
            if self.mavlink_manager is not None:
                self.mavlink_manager.set_georeference(self.georeference)
            ui = getattr(self, "ui_builder", None)
            if ui is not None:
                ui.sync_georef_ui()
        return detected