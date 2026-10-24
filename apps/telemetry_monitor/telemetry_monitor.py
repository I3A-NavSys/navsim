#!/usr/bin/env python3
"""
Monitor de Telemetría MAVLink v2 para NavSim (NVIDIA Isaac Sim).

Arquitectura Pub/Sub bajo demanda sobre un único puerto UDP:
- Abre un único socket UDP local de escucha (por defecto puerto 16000).
- Se suscribe periódicamente al puerto UDP único del simulador (14550) mediante comandos
  de infraestructura (SIM:SUB <puerto_retorno> [target]).
- Recibe y desmultiplexa en tiempo real las tramas MAVLink v2 (ASTM F3411 OpenDroneID y
  NavSim FlightPlan Status) de toda la flota de drones sobre ese único puerto.

Controles:
  ↑ / ↓       : Seleccionar dron en la lista interactiva
  q / Ctrl+C  : Salir limpiamente desuscribiéndose del simulador
"""

import argparse
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import select
import socket
import sys
import time
from typing import Dict, List, Optional, Set, Tuple

# Garantizar importación de mavlink desde la raíz del repositorio
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from libs.mavlink.dialect import (
    MAVLinkV2Codec,
    MavFlightPlanStatusMsg,
    MavOdidBasicIdMsg,
    MavOdidLocationMsg,
    MavOdidOperatorIdMsg,
    MavOdidSelfIdMsg,
    MavOdidStatus,
    NavSimUAVState,
    _decode_str,
)

DEFAULT_SIM_PORT = 14550
DEFAULT_LISTEN_PORT = 16000
HEARTBEAT_PERIOD_S = 1.0

ODID_STATUS_SHORT = {
    int(MavOdidStatus.UNDECLARED): "UNDECL",
    int(MavOdidStatus.GROUND): "GROUND",
    int(MavOdidStatus.AIRBORNE): "AIRBORNE",
    int(MavOdidStatus.EMERGENCY): "EMERGENCY",
    int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE): "FAIL/STOP",
}

FG_GREEN = "\033[92m"
FG_AMBER = "\033[93m"
FG_RED = "\033[91m"
FG_GREY = "\033[90m"
FG_WHITE = "\033[97m"
FG_RESET = "\033[39m"

ODID_STATUS_COLORS = {
    int(MavOdidStatus.UNDECLARED): FG_GREY,
    int(MavOdidStatus.GROUND): FG_AMBER,
    int(MavOdidStatus.AIRBORNE): FG_GREEN,
    int(MavOdidStatus.EMERGENCY): FG_RED,
    int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE): FG_RED,
}

UAV_STATE_NAMES = {
    int(NavSimUAVState.IDLE): "IDLE (En reposo)",
    int(NavSimUAVState.BUSY): "BUSY (Ruta 4D activa)",
    int(NavSimUAVState.DEAD): "DEAD (Fallo crítico)",
    int(NavSimUAVState.MANUAL): "MANUAL (Control directo)",
}

UA_TYPE_NAMES = {
    0: "Sin especificar",
    1: "Aeroplano (Ala fija)",
    2: "Helicóptero / Multirrotor",
    3: "Giroplano",
    4: "VTOL Híbrido",
    5: "Ornitóptero",
    6: "Planeador",
    7: "Cometa",
    8: "Globo libre",
}

ID_TYPE_NAMES = {
    0: "Ninguno",
    1: "Nº Serie (ANSI/CTA-2063-A)",
    2: "Registro CAA",
    3: "UUID U-Space/UTM",
    4: "ID de Sesión específica",
}

RESET_COLOR = "\033[0m"
CYAN = "\033[96m"
BOLD = "\033[1m"
REVERSE = "\033[7m"
ERA_LINE = "\033[K"
ERA_DISPLAY = "\033[J"

TABLE_WIDTH = 118


def cardinal_direction(deg: float) -> str:
    """Devuelve el punto cardinal aproximado (16 rumbos) para un rumbo en grados."""
    cardinals = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    idx = int(round((deg % 360.0) / 22.5)) % 16
    return cardinals[idx]


def format_utc_us(time_unix_usec: int) -> str:
    """Formatea microsegundos desde 1970 como 'AAAA-MM-DD HH:MM:SS.mmm' (UTC)."""
    dt = datetime.fromtimestamp(time_unix_usec / 1e6, tz=timezone.utc)
    return dt.strftime("%Y-%m-%d %H:%M:%S.") + f"{dt.microsecond // 1000:03d}"


def color_metric(text: str, is_active_change: bool, default_col: str = FG_WHITE) -> str:
    """Si el valor cambió en el último segundo, resalta en amarillo (FG_AMBER); si no, blanco."""
    col = FG_AMBER if is_active_change else default_col
    return f"{col}{text}{FG_RESET}"


# ==============================================================================
# Seguimiento de Estado por Dron
# ==============================================================================
class UAVTelemetryTracker:
    def __init__(self, uav_idx: int, host: str = "127.0.0.1", listen_port: int = 16000, sim_port: int = 14550):
        self.uav_idx = uav_idx
        self.host = host
        self.listen_port = listen_port
        self.sim_port = sim_port

        self.last_loc_msg: Optional[MavOdidLocationMsg] = None
        self.last_basic_msg: Optional[MavOdidBasicIdMsg] = None
        self.last_op_msg: Optional[MavOdidOperatorIdMsg] = None
        self.last_self_msg: Optional[MavOdidSelfIdMsg] = None
        self.last_fp_status_msg: Optional[MavFlightPlanStatusMsg] = None

        self.last_recv_time: float = 0.0
        self.msg_count: int = 0
        self.hz: float = 0.0
        self._hz_timer: float = time.time()
        self._hz_counter: int = 0
        self.is_stopped: bool = False
        # Historial de cambios para resaltado visual: field -> (val_str, last_change_time)
        self.field_changes: Dict[str, Tuple[str, float]] = {}

    def track_change(self, field: str, val_str: str, now: float) -> bool:
        """Determina si un dato cambió en el último segundo para resaltarlo en amarillo."""
        prev = self.field_changes.get(field)
        if prev is None:
            self.field_changes[field] = (val_str, now - 2.0)
            return False
        prev_val, change_time = prev
        if prev_val != val_str:
            self.field_changes[field] = (val_str, now)
            return True
        return (now - change_time) <= 1.0

    @property
    def uas_id(self) -> str:
        if self.last_basic_msg is not None and self.last_basic_msg.uas_id:
            return self.last_basic_msg.uas_id
        if self.last_loc_msg is not None:
            clean_id = _decode_str(self.last_loc_msg.id_or_mac)
            if clean_id:
                return clean_id
        return f"UAV_{self.uav_idx:02d}"

    def is_active(self, now: float, timeout: float = 2.0) -> bool:
        if self.is_stopped:
            return False
        if self.last_recv_time <= 0.0:
            return False
        return (now - self.last_recv_time) < timeout

    def reset_state(self) -> None:
        self.last_loc_msg = None
        self.last_basic_msg = None
        self.last_op_msg = None
        self.last_self_msg = None
        self.last_fp_status_msg = None
        self.last_recv_time = 0.0
        self.hz = 0.0
        self.is_stopped = True

    def feed_message(self, msg: object, now: float) -> Optional[Tuple[str, object]]:
        self.last_recv_time = now
        self.msg_count += 1
        self._hz_counter += 1

        if now - self._hz_timer >= 1.0:
            self.hz = self._hz_counter / (now - self._hz_timer)
            self._hz_counter = 0
            self._hz_timer = now

        if isinstance(msg, MavOdidLocationMsg):
            if msg.status == int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE):
                self.is_stopped = True
                self.last_loc_msg = None
            else:
                self.is_stopped = False
                self.last_loc_msg = msg
            return ("loc", msg)

        elif isinstance(msg, MavOdidBasicIdMsg):
            self.last_basic_msg = msg
            return ("basic", msg)

        elif isinstance(msg, MavOdidOperatorIdMsg):
            self.last_op_msg = msg
            return ("operator", msg)

        elif isinstance(msg, MavOdidSelfIdMsg):
            self.last_self_msg = msg
            return ("self_id", msg)

        elif isinstance(msg, MavFlightPlanStatusMsg):
            self.last_fp_status_msg = msg
            return ("fp_status", msg)

        return None


# ==============================================================================
# Lectura no bloqueante de teclado (multiplataforma)
# ==============================================================================
class KeyReader:
    def __init__(self):
        self._old_term = None
        self._fd = None

    def __enter__(self):
        if os.name != "nt" and sys.stdin.isatty():
            try:
                import termios
                import tty
                self._fd = sys.stdin.fileno()
                self._old_term = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
            except Exception:
                pass
        return self

    def __exit__(self, *args):
        if self._old_term is not None and self._fd is not None:
            try:
                import termios
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
            except Exception:
                pass

    def read_keys(self) -> List[str]:
        keys = []
        if os.name == "nt":
            import msvcrt
            while msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b"\x00", b"\xe0"):
                    if msvcrt.kbhit():
                        ch2 = msvcrt.getch()
                        if ch2 == b"H":
                            keys.append("up")
                        elif ch2 == b"P":
                            keys.append("down")
                else:
                    try:
                        c_str = ch.decode("utf-8", errors="ignore")
                        if c_str in ("q", "Q"):
                            keys.append("q")
                    except Exception:
                        pass
        else:
            r, _, _ = select.select([sys.stdin], [], [], 0.0)
            if r:
                seq = sys.stdin.read(1)
                if seq == "\x1b":
                    r2, _, _ = select.select([sys.stdin], [], [], 0.0)
                    if r2:
                        seq += sys.stdin.read(2)
                        if seq == "\x1b[A":
                            keys.append("up")
                        elif seq == "\x1b[B":
                            keys.append("down")
                elif seq in ("q", "Q"):
                    keys.append("q")
        return keys


# ==============================================================================
# Representación Gráfica en Consola
# ==============================================================================
def tracker_state(tr: UAVTelemetryTracker, now: float, timeout: float) -> Tuple[str, str]:
    if tr.last_recv_time <= 0.0 and not tr.is_stopped:
        return "ESPERA", FG_GREY
    if tr.is_stopped:
        return "DETENIDO", FG_RED
    if not tr.is_active(now, timeout):
        return "TIMEOUT", FG_RED
    if tr.last_loc_msg is not None:
        st = tr.last_loc_msg.status
        return ODID_STATUS_SHORT.get(st, f"ST({st})"), ODID_STATUS_COLORS.get(st, FG_WHITE)
    return "ESPERA", FG_GREY


def render_row(tr: UAVTelemetryTracker, now: float, timeout: float, selected: bool) -> str:
    state_txt, state_col = tracker_state(tr, now, timeout)
    loc = tr.last_loc_msg
    fp = tr.last_fp_status_msg
    recent = tr.is_active(now, timeout)

    if loc is not None and recent:
        raw_lat = f"{loc.lat_deg:+10.6f}"
        raw_lon = f"{loc.lon_deg:+11.6f}"
        raw_alt = f"{loc.altitude_geodetic:7.1f}"
        raw_agl = f"{loc.height:5.1f}"
        raw_vh = f"{loc.speed_h_ms:5.1f}"
        raw_vv = f"{loc.speed_v_ms:+5.1f}"
        raw_rumbo = f"{loc.direction_deg:03.0f}° {cardinal_direction(loc.direction_deg):<3s}"

        c_lat = color_metric(raw_lat, tr.track_change("r_lat", raw_lat, now))
        c_lon = color_metric(raw_lon, tr.track_change("r_lon", raw_lon, now))
        c_alt = color_metric(raw_alt, tr.track_change("r_alt", raw_alt, now))
        c_agl = color_metric(raw_agl, tr.track_change("r_agl", raw_agl, now))
        c_vh = color_metric(raw_vh, tr.track_change("r_vh", raw_vh, now))
        c_vv = color_metric(raw_vv, tr.track_change("r_vv", raw_vv, now))
        c_rumbo = color_metric(raw_rumbo, tr.track_change("r_rumbo", raw_rumbo, now))
    else:
        c_lat = f"{FG_GREY}{'--'.rjust(10)}{FG_RESET}"
        c_lon = f"{FG_GREY}{'--'.rjust(11)}{FG_RESET}"
        c_alt = f"{FG_GREY}{'--'.rjust(7)}{FG_RESET}"
        c_agl = f"{FG_GREY}{'--'.rjust(5)}{FG_RESET}"
        c_vh = f"{FG_GREY}{'--'.rjust(5)}{FG_RESET}"
        c_vv = f"{FG_GREY}{'--'.rjust(5)}{FG_RESET}"
        c_rumbo = f"{FG_GREY}{'--'.ljust(7)}{FG_RESET}"

    if fp is not None and fp.time_unix_usec and recent:
        raw_time = format_utc_us(fp.time_unix_usec)
        c_time = color_metric(raw_time, tr.track_change("r_time", raw_time, now))
    else:
        c_time = f"{FG_GREY}{'--'.ljust(23)}{FG_RESET}"

    raw_hz = f"{tr.hz:4.1f}" if recent else " 0.0"
    c_hz = color_metric(raw_hz, tr.track_change("r_hz", raw_hz, now))

    uas_id_raw = f"{tr.uas_id[:16]:<16s}"
    c_uas = color_metric(uas_id_raw, tr.track_change("r_uas", uas_id_raw, now))

    marker = ">" if selected else " "
    prefix = REVERSE if selected else ""
    suffix = RESET_COLOR if selected else ""
    state_cell = f"{state_col}{state_txt:<8s}{FG_RESET}"
    return (
        f"{prefix}{marker}{tr.uav_idx:>3d} {c_uas} {state_cell} {c_lat} {c_lon} {c_alt} {c_agl} "
        f"{c_vh} {c_vv} {c_rumbo} {c_time} {c_hz}{suffix}"
    )


def render_detail(tr: UAVTelemetryTracker, now: float, timeout: float) -> List[str]:
    lines: List[str] = []
    lines.append(f"{BOLD}{CYAN}── Detalle UAV #{tr.uav_idx:02d} · {tr.uas_id} " + "─" * 40 + RESET_COLOR)

    loc = tr.last_loc_msg
    basic = tr.last_basic_msg
    fp = tr.last_fp_status_msg
    state_txt, state_col = tracker_state(tr, now, timeout)

    if not tr.is_active(now, timeout) and loc is None:
        lines.append(
            f" Telemetría     : {FG_GREY}[Esperando datagramas de {tr.host}:{tr.sim_port}. "
            f"Pulsa PLAY en Isaac Sim]{FG_RESET}"
        )
        lines.append(f" Enlace         : Isaac Sim {tr.host}:{tr.sim_port}  Puerto Local Escucha: {tr.listen_port}")
        return lines

    if basic is not None:
        ua = UA_TYPE_NAMES.get(basic.ua_type, f"Tipo({basic.ua_type})")
        idt = ID_TYPE_NAMES.get(basic.id_type, f"TipoID({basic.id_type})")
        lines.append(f" Identificación : {tr.uas_id}   Tipo ID: {idt}   Aeronave: {ua}")

    op_txt = tr.last_op_msg.operator_id if (tr.last_op_msg and tr.last_op_msg.operator_id) else "--"
    mission = tr.last_self_msg.description if (tr.last_self_msg and tr.last_self_msg.description) else "--"
    lines.append(f" Operador       : {op_txt}   Misión: {mission}")

    if loc is not None:
        href = "Suelo (AGL)" if loc.height_reference == 1 else "Despegue"
        v_h, v_v = loc.speed_h_ms, loc.speed_v_ms
        trend = "Ascenso" if v_v > 0.05 else ("Descenso" if v_v < -0.05 else "Nivelado")

        d_lat = color_metric(f"Lat {loc.lat_deg:+.7f}°", tr.track_change("d_lat", f"{loc.lat_deg:+.7f}", now))
        d_lon = color_metric(f"Lon {loc.lon_deg:+.7f}°", tr.track_change("d_lon", f"{loc.lon_deg:+.7f}", now))
        d_alt = color_metric(f"Alt HAE {loc.altitude_geodetic:+.2f} m", tr.track_change("d_alt", f"{loc.altitude_geodetic:+.2f}", now))
        lines.append(f" Posición       : {d_lat}   {d_lon}   {d_alt}")

        d_agl = color_metric(f"AGL {loc.height:.2f} m", tr.track_change("d_agl", f"{loc.height:.2f}", now))
        d_baro = color_metric(f"Baro {loc.altitude_barometric:+.2f} m", tr.track_change("d_baro", f"{loc.altitude_barometric:+.2f}", now))
        lines.append(f" Alturas        : {d_agl}   {d_baro}   Ref: {href}")

        d_vh = color_metric(f"Horiz {v_h:.2f} m/s ({v_h * 3.6:.1f} km/h)", tr.track_change("d_vh", f"{v_h:.2f}", now))
        d_vv = color_metric(f"Vert {v_v:+.2f} m/s ({trend})", tr.track_change("d_vv", f"{v_v:+.2f}", now))
        d_rumbo = color_metric(f"Rumbo {loc.direction_deg:.1f}° ({cardinal_direction(loc.direction_deg)})", tr.track_change("d_rumbo", f"{loc.direction_deg:.1f}", now))
        lines.append(f" Velocidad      : {d_vh}   {d_vv}   {d_rumbo}")

    if fp is not None and fp.time_unix_usec:
        raw_utc = f"{format_utc_us(fp.time_unix_usec)} UTC"
        utc_txt = color_metric(raw_utc, tr.track_change("d_utc", raw_utc, now))
    else:
        utc_txt = "-- (sin NAVSIM_FLIGHT_PLAN_STATUS)"
    raw_odid = f"{loc.timestamp:.1f} s tras la hora en punto" if loc is not None else "--"
    odid_txt = color_metric(raw_odid, tr.track_change("d_odid", raw_odid, now))
    lines.append(f" Tiempo         : {utc_txt}   ODID: {odid_txt}   (precisión 0,1 s)")
    lines.append(" Exactitud      : Horiz <1 m | Vert <1 m | Vel <0,3 m/s")

    if fp is not None:
        mode = UAV_STATE_NAMES.get(fp.uav_state, f"STATE({fp.uav_state})")
        if fp.uav_state == int(NavSimUAVState.BUSY) and fp.plan_id:
            wp_idx = fp.current_wp + 1 if fp.current_wp >= 0 else 0
            lbl = f" '{fp.wp_label}'" if fp.wp_label else ""
            dur = max(fp.fp_finish_time - fp.fp_init_time, 1e-6)
            pct = min(100.0, max(0.0, (fp.sim_time - fp.fp_init_time) * 100.0 / dur))
            ep = math.sqrt(sum(c * c for c in fp.pos_error))
            ev = math.sqrt(sum(c * c for c in fp.vel_error))
            lines.append(
                f" Ruta 4D        : {mode}   Plan: {fp.plan_id}   WP {wp_idx}/{fp.total_wps}{lbl}   "
                f"Progreso {pct:.1f}%"
            )
            lines.append(
                f" Seguimiento 4D : Ref [{fp.desired_pos[0]:+.1f}, {fp.desired_pos[1]:+.1f}, {fp.desired_pos[2]:+.1f}] m   "
                f"|e_pos|={ep:.3f} m   |e_vel|={ev:.3f} m/s"
            )
        else:
            lines.append(f" Ruta 4D        : {mode}   Plan activo: Ninguno")

    lines.append(
        f" Enlace         : Isaac Sim {tr.host}:{tr.sim_port}  Puerto Escucha: {tr.listen_port}  "
        f"Tasa: {tr.hz:.1f} Hz"
    )
    return lines


def render_dashboard(
    trackers: List[UAVTelemetryTracker],
    now: float,
    timeout: float = 2.0,
    selected: int = 0,
    listen_port: int = 16000,
    sim_port: int = 14550,
    sim_host: str = "127.0.0.1",
) -> str:
    lines: List[str] = []
    active_count = len([t for t in trackers if t.is_active(now, timeout=timeout)])
    pc_utc = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
    lines.append(f"{BOLD}{CYAN}NavSim Telemetry Monitor — MAVLink v2 · ASTM F3411 / ED-282 (Pub/Sub){RESET_COLOR}")
    sub = (
        f"Hora PC (UTC): {pc_utc}   Drones activos: {active_count}/{len(trackers)}   "
        f"[Simulador {sim_host}:{sim_port} → Puerto Local {listen_port}]"
    )
    if active_count == 0:
        sub += f" {FG_AMBER}(Pulsa PLAY en Isaac Sim){FG_RESET}"
    lines.append(sub)

    lines.append("═" * TABLE_WIDTH)
    lines.append(
        f"{BOLD} {'#':>3s} {'UAS ID':<16s} {'Estado':<8s} {'Lat [°]':>10s} {'Lon [°]':>11s} {'Alt HAE':>7s} "
        f"{'AGL':>5s} {'Vh':>5s} {'Vv':>5s} {'Rumbo':<7s} {'Hora UTC (dron)':<23s} {'Hz':>4s}{RESET_COLOR}"
    )
    lines.append("─" * TABLE_WIDTH)

    selected = max(0, min(selected, len(trackers) - 1)) if trackers else 0
    total_trackers = len(trackers)
    max_visible_rows = 12

    if total_trackers <= max_visible_rows:
        visible_indices = list(range(total_trackers))
    else:
        half = max_visible_rows // 2
        start = max(0, min(selected - half, total_trackers - max_visible_rows))
        end = start + max_visible_rows
        visible_indices = list(range(start, end))

        if start > 0:
            lines.append(f"{FG_GREY}  ▲ ... {start} dron(es) más arriba (usa ↑ para navegar) ...{FG_RESET}")

    for i in visible_indices:
        lines.append(render_row(trackers[i], now, timeout, selected=(i == selected)))

    if total_trackers > max_visible_rows and visible_indices and visible_indices[-1] < (total_trackers - 1):
        remaining = total_trackers - 1 - visible_indices[-1]
        lines.append(f"{FG_GREY}  ▼ ... {remaining} dron(es) más abajo (usa ↓ para navegar) ...{FG_RESET}")

    lines.append("═" * TABLE_WIDTH)

    if trackers:
        lines.extend(render_detail(trackers[selected], now, timeout))
    else:
        lines.append(f" {FG_GREY}Esperando suscripción y detección automática de aeronaves...{FG_RESET}")
    lines.append("─" * TABLE_WIDTH)
    lines.append(f"{FG_GREY}↑/↓ seleccionar · q/Ctrl+C salir · {FG_AMBER}amarillo{FG_GREY}: dato actualizado (<=1s) · {FG_WHITE}blanco{FG_GREY}: estático{FG_RESET}")

    return "\n".join(f"{line}{ERA_LINE}" for line in lines) + ERA_DISPLAY


# ==============================================================================
# Bucle de Ejecución Principal
# ==============================================================================
def run_monitor(
    target_spec: str,
    listen_port: int = 16000,
    sim_port: int = 14550,
    sim_host: str = "127.0.0.1",
    bind_host: str = "0.0.0.0",
    stream_mode: bool = False,
    timeout: float = 2.0,
    subscribe: bool = True,
    known_count: int = 0,
) -> int:
    try:
        rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rx_sock.bind((bind_host, listen_port))
        rx_sock.setblocking(False)
    except OSError as e:
        print(f"Error: no se pudo abrir el puerto local de escucha {bind_host}:{listen_port} ({e}).")
        return 1

    codec = MAVLinkV2Codec(sysid=255, compid=190)
    trackers_map: Dict[int, UAVTelemetryTracker] = {}
    only_first = (target_spec.upper() in ("FIRST", "0"))
    first_uav_seen: Optional[int] = None

    if only_first:
        # Inicializar con UAV 0 por defecto mientras se espera conexión
        trackers_map[0] = UAVTelemetryTracker(0, host=sim_host, listen_port=listen_port, sim_port=sim_port)
    elif known_count > 0:
        for idx in range(known_count):
            trackers_map[idx] = UAVTelemetryTracker(idx, host=sim_host, listen_port=listen_port, sim_port=sim_port)

    last_heartbeat = 0.0
    selected = 0
    last_render_time = 0.0

    if not stream_mode:
        os.system("cls" if os.name == "nt" else "clear")
    else:
        print(f"Monitor escuchando en {bind_host}:{listen_port} → Solicitando suscripción a {sim_host}:{sim_port} ({target_spec})")

    try:
        with KeyReader() as keys:
            while True:
                now = time.time()

                # 1. Enviar latido de suscripción al simulador
                if subscribe and (now - last_heartbeat) >= HEARTBEAT_PERIOD_S:
                    sub_cmd = f"SIM:SUB {listen_port} {target_spec} 3.0".encode("utf-8")
                    try:
                        rx_sock.sendto(sub_cmd, (sim_host, sim_port))
                    except OSError:
                        pass
                    last_heartbeat = now

                # 2. Esperar datagramas MAVLink entrantes en el socket único
                readable, _, _ = select.select([rx_sock], [], [], 0.05)
                if rx_sock in readable:
                    while True:
                        try:
                            data, _ = rx_sock.recvfrom(65535)
                        except (BlockingIOError, InterruptedError):
                            break
                        except ConnectionResetError:
                            continue
                        except OSError:
                            break

                        if not data:
                            break

                        recv_now = time.time()
                        parsed = codec.parse_bytes(data)
                        for msg in parsed:
                            # Determinar índice del UAV
                            uav_idx = getattr(msg, "sysid", 1) - 1
                            if uav_idx < 0:
                                uav_idx = 0

                            # Si se solicitó solo el primero, bloquear la tabla al primer UAV recibido
                            if only_first:
                                if first_uav_seen is None:
                                    first_uav_seen = uav_idx
                                    if 0 in trackers_map and uav_idx != 0:
                                        del trackers_map[0]
                                elif uav_idx != first_uav_seen:
                                    continue

                            # Si el tracker aún no existe en el mapa, añadirlo dinámicamente
                            if uav_idx not in trackers_map:
                                trackers_map[uav_idx] = UAVTelemetryTracker(
                                    uav_idx=uav_idx,
                                    host=sim_host,
                                    listen_port=listen_port,
                                    sim_port=sim_port,
                                )

                            tr = trackers_map[uav_idx]
                            event = tr.feed_message(msg, recv_now)

                            if stream_mode and event:
                                ev_type, m = event
                                if ev_type == "loc" and isinstance(m, MavOdidLocationMsg):
                                    if m.status != int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE):
                                        fp = tr.last_fp_status_msg
                                        t_txt = (
                                            format_utc_us(fp.time_unix_usec) + "Z"
                                            if (fp and fp.time_unix_usec)
                                            else datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + "Z(PC)"
                                        )
                                        st_name = ODID_STATUS_SHORT.get(m.status, str(m.status))
                                        print(
                                            f"[{t_txt}] UAV #{tr.uav_idx:02d} ({tr.uas_id}) "
                                            f"Lat={m.lat_deg:+.7f}° Lon={m.lon_deg:+.7f}° "
                                            f"Alt={m.altitude_geodetic:.1f}m AGL={m.height:.1f}m "
                                            f"Vh={m.speed_h_ms:.1f}m/s Vz={m.speed_v_ms:+.1f}m/s "
                                            f"Dir={m.direction_deg:.1f}°({cardinal_direction(m.direction_deg)}) "
                                            f"Status={st_name}"
                                        )

                # Limpiar estado por timeout
                for tr in trackers_map.values():
                    if tr.last_recv_time > 0 and (now - tr.last_recv_time) >= timeout and not tr.is_stopped:
                        tr.reset_state()

                # 3. Interacción teclado
                if not stream_mode:
                    trackers_list = [trackers_map[k] for k in sorted(trackers_map.keys())]
                    dirty = False
                    for key in keys.read_keys():
                        if key == "up" and trackers_list:
                            selected = (selected - 1) % len(trackers_list)
                            dirty = True
                        elif key == "down" and trackers_list:
                            selected = (selected + 1) % len(trackers_list)
                            dirty = True
                        elif key == "q":
                            return 0

                    if dirty or (now - last_render_time) >= 0.15:
                        buf = render_dashboard(
                            trackers=trackers_list,
                            now=now,
                            timeout=timeout,
                            selected=selected,
                            listen_port=listen_port,
                            sim_port=sim_port,
                            sim_host=sim_host,
                        )
                        sys.stdout.write("\033[H" + buf)
                        sys.stdout.flush()
                        last_render_time = now

    except KeyboardInterrupt:
        pass
    finally:
        # Cancelar suscripción al salir
        try:
            unsub_cmd = f"SIM:UNSUB {listen_port} {target_spec}".encode("utf-8")
            rx_sock.sendto(unsub_cmd, (sim_host, sim_port))
        except Exception:
            pass
        rx_sock.close()
        print("\nCerrando monitor de telemetría...")

    return 0


# ==============================================================================
# CLI Entrypoint
# ==============================================================================
def main() -> int:
    if os.name == "nt":
        os.system("")

    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    parser = argparse.ArgumentParser(
        prog="telemetry_monitor.py",
        description="Monitor de telemetría MAVLink v2 Pub/Sub bajo demanda para NavSim (Isaac Sim).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "targets",
        nargs="*",
        metavar="IDENTIFICADOR",
        help="Identificadores de drones a monitorizar (ej. ESP-ABJ-2026-23). "
        "Sin argumentos: primer dron disponible.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Monitorizar todos los drones de la escena.",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Dirección IP de la máquina de Isaac Sim (por defecto: 127.0.0.1).",
    )
    parser.add_argument(
        "--sim-port",
        type=int,
        default=DEFAULT_SIM_PORT,
        help="Puerto UDP único de NavSim / Isaac Sim (por defecto: 14550).",
    )
    parser.add_argument(
        "--listen-port",
        type=int,
        default=DEFAULT_LISTEN_PORT,
        help="Puerto UDP local único para recibir la telemetría (por defecto: 16000).",
    )
    parser.add_argument(
        "--bind",
        default="0.0.0.0",
        help="IP local donde enlazar el socket de escucha UDP (por defecto: 0.0.0.0).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Segundos sin recibir datos para considerar un dron inactivo (por defecto: 2.0).",
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="Salida línea a línea continua en vez de la tabla interactiva de consola.",
    )
    parser.add_argument(
        "--no-subscribe",
        action="store_true",
        help="No enviar mensaje de suscripción SIM:SUB (escucha pasiva).",
    )

    args = parser.parse_args()

    # Procesar targets: --all, lista de identificadores o por defecto 0 (primer dron)
    if args.all or any(t.upper() in ("ALL", "*") for t in args.targets):
        target_spec = "ALL"
    elif not args.targets:
        target_spec = "0"
    else:
        # Resolver cada token: si contiene número (ej. ESP-ABJ-2026-23 -> 23),
        # enviar tanto el índice como el string completo para compatibilidad total
        expanded = []
        for t in args.targets:
            token = t.strip()
            expanded.append(token)
            # Extraer sufijo numérico final si existe (ej. 'ESP-ABJ-2026-03' -> '3', '03')
            m = re.search(r"(\d+)$", token)
            if m:
                num_str = str(int(m.group(1)))
                if num_str not in expanded:
                    expanded.append(num_str)
        target_spec = ",".join(expanded)

    return run_monitor(
        target_spec=target_spec,
        listen_port=args.listen_port,
        sim_port=args.sim_port,
        sim_host=args.host,
        bind_host=args.bind,
        stream_mode=args.stream,
        timeout=args.timeout,
        subscribe=not args.no_subscribe,
    )


if __name__ == "__main__":
    sys.exit(main())
