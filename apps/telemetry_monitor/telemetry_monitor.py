#!/usr/bin/env python3
"""
Monitor de Telemetría MAVLink v2 ASTM F3411 / EUROCAE ED-282 para NavSim (Isaac Sim).

Muestra en una tabla compacta (una fila por dron) el estado de los drones indicados y, debajo,
el detalle completo del dron seleccionado con las flechas arriba/abajo.

Uso rápido (ver `--help` para la descripción completa):
    python apps/telemetry_monitor/telemetry_monitor.py                         # 1 dron: 127.0.0.1:16000
    python apps/telemetry_monitor/telemetry_monitor.py 16003                   # 1 dron: 127.0.0.1:16003
    python apps/telemetry_monitor/telemetry_monitor.py 16000 16005             # lista de drones
    python apps/telemetry_monitor/telemetry_monitor.py 16000-16009             # rango de puertos
    python apps/telemetry_monitor/telemetry_monitor.py 192.168.1.20:16000-16003  # Isaac Sim en otra máquina
"""

import argparse
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import re
import select
import socket
import sys
import time
from typing import Dict, List, Optional, Tuple

# Garantizar importación de mavlink desde la raíz del repositorio
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from mavlink.dialect import (
    MAVLinkV2Codec,
    MavFlightPlanStatusMsg,
    MavOdidBasicIdMsg,
    MavOdidLocationMsg,
    MavOdidOperatorIdMsg,
    MavOdidSelfIdMsg,
    MavOdidStatus,
    NavSimUAVState,
)

DEFAULT_TX_PORT = 16000
DEFAULT_RX_PORT = 15000
HEARTBEAT_PERIOD_S = 1.0

ODID_STATUS_SHORT = {
    int(MavOdidStatus.UNDECLARED): "UNDECL",
    int(MavOdidStatus.GROUND): "GROUND",
    int(MavOdidStatus.AIRBORNE): "AIRBORNE",
    int(MavOdidStatus.EMERGENCY): "EMERGENC",
    int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE): "FAILURE",
}

# Colores de primer plano (se cierran con FG_RESET para no anular la fila resaltada)
FG_GREY = "\033[90m"
FG_YELLOW = "\033[93m"
FG_GREEN = "\033[92m"
FG_RED = "\033[91m"
FG_MAGENTA = "\033[95m"
FG_WHITE = "\033[97m"
FG_RESET = "\033[39m"

ODID_STATUS_COLORS = {
    int(MavOdidStatus.UNDECLARED): FG_GREY,
    int(MavOdidStatus.GROUND): FG_YELLOW,
    int(MavOdidStatus.AIRBORNE): FG_GREEN,
    int(MavOdidStatus.EMERGENCY): FG_RED,
    int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE): FG_MAGENTA,
}

UAV_STATE_NAMES = {
    int(NavSimUAVState.IDLE): "IDLE (En reposo)",
    int(NavSimUAVState.BUSY): "AUTO 4D (Ejecutando Plan)",
    int(NavSimUAVState.DEAD): "DEAD (Fuera de servicio)",
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


# ==============================================================================
# Interpretación de la lista de drones (IP:puerto)
# ==============================================================================
def parse_targets(tokens: List[str], default_host: str, base_tx: int, base_rx: int) -> List[Tuple[str, int, int, int]]:
    """
    Convierte la lista de argumentos en tuplas (host, tx_port, rx_port, uav_idx).

    Formatos admitidos por elemento:
        16003                       puerto de telemetría en el host por defecto
        16000-16009                 rango de puertos en el host por defecto
        192.168.1.20                IP (o nombre) con el puerto por defecto 16000
        192.168.1.20:16003          IP y puerto
        192.168.1.20:16000-16009    IP y rango de puertos
    Si la lista está vacía se monitoriza un único dron en default_host:16000.
    """
    if not tokens:
        tokens = [str(DEFAULT_TX_PORT)]

    result: List[Tuple[str, int, int, int]] = []
    for token in tokens:
        spec = token.strip()
        host = default_host
        port_spec = None

        if ":" in spec:
            host_part, port_spec = spec.rsplit(":", 1)
            if host_part:
                host = host_part
        elif re.fullmatch(r"\d+(-\d+)?", spec):
            port_spec = spec
        else:
            host = spec  # IP o nombre de máquina sin puerto

        if port_spec is None:
            ports = [DEFAULT_TX_PORT]
        else:
            m = re.fullmatch(r"(\d+)(?:-(\d+))?", port_spec)
            if not m:
                raise ValueError(f"Elemento no válido '{token}'. Usa PUERTO, PUERTO-PUERTO, IP o IP:PUERTO[-PUERTO].")
            first = int(m.group(1))
            last = int(m.group(2)) if m.group(2) else first
            if last < first:
                raise ValueError(f"Rango descendente en '{token}'.")
            if last - first >= 1000:
                raise ValueError(f"Rango demasiado grande en '{token}' (máximo 1000 drones).")
            ports = list(range(first, last + 1))

        for p in ports:
            if not 1 <= p <= 65535:
                raise ValueError(f"Puerto fuera de rango (1-65535) en '{token}'.")

        try:
            resolved_host = socket.gethostbyname(host)
        except OSError:
            raise ValueError(f"No se puede resolver la dirección '{host}' (elemento '{token}').")

        for p in ports:
            if p >= base_tx:
                uav_idx = p - base_tx
                rx_port = base_rx + uav_idx
            else:
                uav_idx = len(result)
                rx_port = base_rx + uav_idx
            result.append((resolved_host, p, rx_port, uav_idx))

    # Cada dron necesita su propio puerto local de escucha
    seen: Dict[int, str] = {}
    for host, tx_port, _, _ in result:
        if tx_port in seen:
            raise ValueError(
                f"El puerto {tx_port} aparece repetido ({seen[tx_port]} y {host}). "
                f"El monitor escucha cada dron en su propio puerto local."
            )
        seen[tx_port] = host
    return result


# ==============================================================================
# Seguimiento por dron
# ==============================================================================
class UAVTelemetryTracker:
    def __init__(self, uav_idx: int, tx_port: int, rx_port: int, host: str = "127.0.0.1"):
        self.uav_idx = uav_idx
        self.host = host  # Dirección de la máquina donde corre Isaac Sim
        self.tx_port = tx_port  # Puerto local de escucha = puerto TX de telemetría del dron
        self.rx_port = rx_port  # Puerto RX de comandos del dron en Isaac Sim (aquí se envía el latido)
        self.codec = MAVLinkV2Codec(sysid=255, compid=190)
        self.sock: Optional[socket.socket] = None

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
        self._last_heartbeat: float = 0.0

    @property
    def uas_id(self) -> str:
        if self.last_basic_msg is not None and self.last_basic_msg.uas_id:
            return self.last_basic_msg.uas_id
        return f"UAV_{self.uav_idx:02d}"

    def is_active(self, now: float, timeout: float = 2.0) -> bool:
        """Determina si este dron está emitiendo telemetría activamente."""
        if self.is_stopped:
            return False
        if self.last_recv_time <= 0.0:
            return False
        return (now - self.last_recv_time) < timeout

    def reset_state(self) -> None:
        """Limpia el estado y coordenadas almacenadas al detenerse o desaparecer el dron."""
        self.last_loc_msg = None
        self.last_basic_msg = None
        self.last_op_msg = None
        self.last_self_msg = None
        self.last_fp_status_msg = None
        self.last_recv_time = 0.0
        self.hz = 0.0
        self.is_stopped = True

    def open_socket(self, bind_host: str = "0.0.0.0") -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((bind_host, self.tx_port))
        sock.setblocking(False)
        self.sock = sock

    def close(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def send_subscribe(self, now: float, force: bool = False) -> None:
        """
        Envía un datagrama UDP de suscripción al puerto RX del dron (15000+).
        Isaac Sim registra la dirección de origen como destino de telemetría, de modo que el monitor
        recibe datos aunque se ejecute en otra máquina.
        """
        if self.sock is None:
            return
        if not force and (now - self._last_heartbeat) < HEARTBEAT_PERIOD_S:
            return
        self._last_heartbeat = now
        try:
            self.sock.sendto(b"NAVSIM_SUB", (self.host, self.rx_port))
        except OSError:
            pass

    def process_incoming(self) -> List[Tuple[str, object]]:
        if not self.sock:
            return []
        received_events = []
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except ConnectionResetError:
                # Windows: ICMP "puerto inalcanzable" por un envío previo; se ignora y se sigue leyendo
                continue
            except OSError:
                break

            now = time.time()
            self.last_recv_time = now
            self.msg_count += 1
            self._hz_counter += 1

            if now - self._hz_timer >= 1.0:
                self.hz = self._hz_counter / (now - self._hz_timer)
                self._hz_counter = 0
                self._hz_timer = now

            parsed = self.codec.parse_bytes(data)
            for msg in parsed:
                if isinstance(msg, MavOdidLocationMsg):
                    if msg.status == int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE):
                        self.is_stopped = True
                        self.last_loc_msg = None
                    else:
                        self.is_stopped = False
                        self.last_loc_msg = msg
                    received_events.append(("loc", msg))
                elif isinstance(msg, MavOdidBasicIdMsg):
                    self.last_basic_msg = msg
                    received_events.append(("basic", msg))
                elif isinstance(msg, MavOdidOperatorIdMsg):
                    self.last_op_msg = msg
                    received_events.append(("operator", msg))
                elif isinstance(msg, MavOdidSelfIdMsg):
                    self.last_self_msg = msg
                    received_events.append(("self_id", msg))
                elif isinstance(msg, MavFlightPlanStatusMsg):
                    self.last_fp_status_msg = msg
                    received_events.append(("fp_status", msg))
        return received_events


# ==============================================================================
# Teclado (flechas arriba/abajo, q) multiplataforma
# ==============================================================================
class KeyReader:
    """Lectura no bloqueante de teclas: devuelve 'up', 'down' o el carácter pulsado."""

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
                self._old_term = None
        return self

    def __exit__(self, *exc):
        if self._old_term is not None:
            try:
                import termios

                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
            except Exception:
                pass
        return False

    def read_keys(self) -> List[str]:
        keys: List[str] = []
        if os.name == "nt":
            try:
                import msvcrt
            except ImportError:
                return keys
            while msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch in ("\x00", "\xe0"):
                    ch2 = msvcrt.getwch()
                    if ch2 == "H":
                        keys.append("up")
                    elif ch2 == "P":
                        keys.append("down")
                else:
                    keys.append(ch.lower())
            return keys

        if self._old_term is None:
            return keys
        while select.select([sys.stdin], [], [], 0)[0]:
            data = os.read(self._fd, 32).decode("utf-8", errors="ignore")
            if not data:
                break
            i = 0
            while i < len(data):
                if data.startswith("\x1b[A", i):
                    keys.append("up")
                    i += 3
                elif data.startswith("\x1b[B", i):
                    keys.append("down")
                    i += 3
                else:
                    keys.append(data[i].lower())
                    i += 1
        return keys


# ==============================================================================
# Representación en pantalla
# ==============================================================================
def tracker_state(tr: UAVTelemetryTracker, now: float, timeout: float) -> Tuple[str, str]:
    """Retorna (texto_corto, color) del estado de un dron."""
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
        lat = f"{loc.lat_deg:+10.6f}"
        lon = f"{loc.lon_deg:+11.6f}"
        alt = f"{loc.altitude_geodetic:7.1f}"
        agl = f"{loc.height:5.1f}"
        vh = f"{loc.speed_h_ms:5.1f}"
        vv = f"{loc.speed_v_ms:+5.1f}"
        rumbo = f"{loc.direction_deg:03.0f}° {cardinal_direction(loc.direction_deg):<3s}"
    else:
        lat, lon, alt, agl, vh, vv, rumbo = (
            "--".rjust(10), "--".rjust(11), "--".rjust(7), "--".rjust(5), "--".rjust(5), "--".rjust(5), "--".ljust(7)
        )

    if fp is not None and fp.time_unix_usec and recent:
        time_txt = format_utc_us(fp.time_unix_usec)
    else:
        time_txt = "--".ljust(23)

    hz = f"{tr.hz:4.1f}" if recent else " 0.0"
    marker = ">" if selected else " "
    prefix = REVERSE if selected else ""
    suffix = RESET_COLOR if selected else ""
    state_cell = f"{state_col}{state_txt:<8s}{FG_RESET}"
    return (
        f"{prefix}{marker}{tr.uav_idx:>3d} {tr.uas_id[:16]:<16s} {state_cell} {lat} {lon} {alt} {agl} "
        f"{vh} {vv} {rumbo:<7s} {time_txt:<23s} {hz}{suffix}"
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
            f" Telemetría     : {FG_GREY}[Esperando datagramas de {tr.host} (puerto local {tr.tx_port}). "
            f"Pulsa PLAY en Isaac Sim]{FG_RESET}"
        )
        lines.append(f" Enlace         : Isaac Sim {tr.host}  TX telemetría {tr.tx_port}  RX comandos {tr.rx_port}")
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
        lines.append(
            f" Posición       : Lat {loc.lat_deg:+.7f}°   Lon {loc.lon_deg:+.7f}°   Alt HAE {loc.altitude_geodetic:+.2f} m"
        )
        lines.append(f" Alturas        : AGL {loc.height:.2f} m   Baro {loc.altitude_barometric:+.2f} m   Ref: {href}")
        lines.append(
            f" Velocidad      : Horiz {v_h:.2f} m/s ({v_h * 3.6:.1f} km/h)   Vert {v_v:+.2f} m/s ({trend})   "
            f"Rumbo {loc.direction_deg:.1f}° ({cardinal_direction(loc.direction_deg)})"
        )

    if fp is not None and fp.time_unix_usec:
        utc_txt = f"{format_utc_us(fp.time_unix_usec)} UTC"
    else:
        utc_txt = "-- (sin NAVSIM_FLIGHT_PLAN_STATUS)"
    odid_txt = f"{loc.timestamp:.1f} s tras la hora en punto" if loc is not None else "--"
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
        f" Enlace         : Isaac Sim {tr.host}  TX telemetría {tr.tx_port}  RX comandos {tr.rx_port}  "
        f"Tasa {tr.hz:.1f} Hz"
    )
    return lines


def render_dashboard(
    trackers: List[UAVTelemetryTracker],
    now: float,
    timeout: float = 2.0,
    selected: int = 0,
) -> str:
    lines: List[str] = []
    active_count = len([t for t in trackers if t.is_active(now, timeout=timeout)])
    pc_utc = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
    lines.append(f"{BOLD}{CYAN}NavSim Telemetry Monitor — MAVLink v2 · ASTM F3411 / ED-282{RESET_COLOR}")
    sub = f"Hora del PC (UTC): {pc_utc}   Drones activos: {active_count}/{len(trackers)}"
    if active_count == 0:
        sub += "   (En espera de telemetría: pulsa PLAY en Isaac Sim)"
    lines.append(sub)

    # Alerta de identificadores duplicados (ASTM F3411 prohíbe IDs duplicados en el aire)
    ids: Dict[str, List[int]] = {}
    for tr in trackers:
        if tr.is_active(now, timeout):
            ids.setdefault(tr.uas_id.strip().upper(), []).append(tr.uav_idx)
    for uid, lst in ids.items():
        if len(lst) > 1:
            lines.append(f"{FG_RED}{BOLD}!! CONFLICTO: identificador '{uid}' duplicado en UAVs {lst}{RESET_COLOR}")

    lines.append("═" * TABLE_WIDTH)
    lines.append(
        f"{BOLD} {'#':>3s} {'UAS ID':<16s} {'Estado':<8s} {'Lat [°]':>10s} {'Lon [°]':>11s} {'Alt HAE':>7s} "
        f"{'AGL':>5s} {'Vh':>5s} {'Vv':>5s} {'Rumbo':<7s} {'Hora UTC (dron)':<23s} {'Hz':>4s}{RESET_COLOR}"
    )
    lines.append("─" * TABLE_WIDTH)

    selected = max(0, min(selected, len(trackers) - 1))
    for i, tr in enumerate(trackers):
        lines.append(render_row(tr, now, timeout, selected=(i == selected)))
    lines.append("═" * TABLE_WIDTH)

    lines.extend(render_detail(trackers[selected], now, timeout))
    lines.append("─" * TABLE_WIDTH)
    lines.append(f"{FG_GREY}↑/↓ seleccionar dron · q o Ctrl+C salir · --help para ver la ayuda{FG_RESET}")

    return "\n".join(f"{line}{ERA_LINE}" for line in lines) + ERA_DISPLAY


# ==============================================================================
# Bucle principal
# ==============================================================================
def run_monitor(
    trackers: List[UAVTelemetryTracker],
    stream_mode: bool,
    timeout: float = 2.0,
    bind_host: str = "0.0.0.0",
    subscribe: bool = True,
) -> int:
    try:
        for tr in trackers:
            tr.open_socket(bind_host)
    except OSError as e:
        print(f"Error: no se pudo abrir el puerto local de escucha ({e}). ¿Hay otro monitor ejecutándose?")
        for tr in trackers:
            tr.close()
        return 1

    if not stream_mode:
        os.system("cls" if os.name == "nt" else "clear")
    else:
        targets = ", ".join(f"{t.host}:{t.tx_port}" for t in trackers)
        print(f"Monitor escuchando {len(trackers)} UAV(s): {targets}")

    selected = 0
    last_render_time = 0.0
    try:
        with KeyReader() as keys:
            while True:
                socks = [t.sock for t in trackers if t.sock is not None]
                readable, _, _ = select.select(socks, [], [], 0.05)

                now = time.time()
                if subscribe:
                    for tr in trackers:
                        tr.send_subscribe(now)

                new_msgs = False
                for tr in trackers:
                    if tr.sock in readable:
                        events = tr.process_incoming()
                        if events:
                            new_msgs = True
                            if stream_mode:
                                for ev_type, m in events:
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
                                                f"[{t_txt}] UAV #{tr.uav_idx:02d} ({tr.uas_id} | {tr.host}:{tr.tx_port}) "
                                                f"Lat={m.lat_deg:+.7f}° Lon={m.lon_deg:+.7f}° "
                                                f"Alt={m.altitude_geodetic:.1f}m AGL={m.height:.1f}m "
                                                f"Vh={m.speed_h_ms:.1f}m/s Vz={m.speed_v_ms:+.1f}m/s "
                                                f"Dir={m.direction_deg:.1f}°({cardinal_direction(m.direction_deg)}) "
                                                f"Status={st_name}"
                                            )

                # Limpiar estado de trackers que hayan superado el timeout de inactividad
                for tr in trackers:
                    if tr.last_recv_time > 0 and (now - tr.last_recv_time) >= timeout and not tr.is_stopped:
                        tr.reset_state()

                quit_requested = False
                dirty = False
                if not stream_mode:
                    for key in keys.read_keys():
                        if key == "up":
                            selected = (selected - 1) % len(trackers)
                            dirty = True
                        elif key == "down":
                            selected = (selected + 1) % len(trackers)
                            dirty = True
                        elif key == "q":
                            quit_requested = True
                if quit_requested:
                    break

                if not stream_mode and (new_msgs or dirty or (now - last_render_time) >= 0.1):
                    output = render_dashboard(trackers, now, timeout=timeout, selected=selected)
                    sys.stdout.write("\033[H" + output)
                    sys.stdout.flush()
                    last_render_time = now
    except KeyboardInterrupt:
        pass
    finally:
        for tr in trackers:
            tr.close()
        print("\nCerrando monitor de telemetría...")
    return 0


HELP_EPILOG = """\
CÓMO FUNCIONA
  El monitor escucha la telemetría MAVLink v2 que emite NavSim (Isaac Sim) por UDP y muestra una
  tabla con una fila por dron (estado, posición WGS-84, alturas, velocidades, rumbo y hora UTC).
  Debajo aparece el detalle completo del dron seleccionado con las flechas ↑/↓. Salir: q o Ctrl+C.

LISTA DE DRONES (IP:puerto)
  Cada dron emite su telemetría por un puerto propio: 16000 + índice (el RX de comandos es 15000 + índice).
  Indica los drones a monitorizar como argumentos, separados por espacios:

    16003                         un dron, en la IP por defecto (--host, 127.0.0.1)
    16000-16009                   rango de puertos
    192.168.1.20                  IP de otra máquina con Isaac Sim, puerto 16000
    192.168.1.20:16002            IP y puerto
    192.168.1.20:16000-16003      IP y rango de puertos

  Sin argumentos se monitoriza un único dron en 127.0.0.1:16000.

EJECUCIÓN EN OTRA MÁQUINA
  El monitor envía periódicamente un datagrama de presencia/suscripción al puerto RX del dron
  (IP:15000+índice). Isaac Sim registra su dirección y le devuelve la telemetría; basta con
  indicar la IP de la máquina de Isaac Sim. Hay que permitir UDP en el cortafuegos de ambas máquinas
  (puertos 15000+ en Isaac Sim y 16000+ en el monitor).

EJEMPLOS
  python telemetry_monitor.py
  python telemetry_monitor.py 16000 16005 16010
  python telemetry_monitor.py 16000-16009
  python telemetry_monitor.py 192.168.1.20:16000-16003
  python telemetry_monitor.py 16000 --stream > registro.log
"""


def main() -> int:
    # Habilitar secuencias de escape ANSI en consolas Windows (cmd.exe y PowerShell)
    if os.name == "nt":
        os.system("")

    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    parser = argparse.ArgumentParser(
        prog="telemetry_monitor.py",
        description="Monitor de telemetría MAVLink v2 (ASTM F3411 / EUROCAE ED-282) para NavSim (Isaac Sim).",
        epilog=HELP_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "targets",
        nargs="*",
        metavar="DRON",
        help="Drones a monitorizar: PUERTO, PUERTO-PUERTO, IP o IP:PUERTO[-PUERTO]. "
        "Sin argumentos: un dron en 127.0.0.1:16000.",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="IP de Isaac Sim para los elementos que no indican IP (por defecto: 127.0.0.1).",
    )
    parser.add_argument(
        "--bind",
        default="0.0.0.0",
        help="IP local donde enlazar la escucha UDP (por defecto: 0.0.0.0).",
    )
    parser.add_argument(
        "--base-tx-port",
        type=int,
        default=DEFAULT_TX_PORT,
        help="Puerto base de telemetría de Isaac Sim (por defecto: 16000).",
    )
    parser.add_argument(
        "--base-rx-port",
        type=int,
        default=DEFAULT_RX_PORT,
        help="Puerto base de comandos de Isaac Sim (por defecto: 15000).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Segundos sin datos para considerar un dron inactivo (por defecto: 2.0).",
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="Salida línea a línea (log continuo) en lugar de la tabla interactiva.",
    )
    parser.add_argument(
        "--no-subscribe",
        action="store_true",
        help="No enviar datagrama de presencia/suscripción (solo escucha pasiva; válido en la misma máquina).",
    )

    args = parser.parse_args()

    try:
        targets = parse_targets(args.targets, args.host, args.base_tx_port, args.base_rx_port)
    except ValueError as e:
        parser.error(str(e))

    trackers = [
        UAVTelemetryTracker(uav_idx=uav_idx, tx_port=tx_port, rx_port=rx_port, host=host)
        for host, tx_port, rx_port, uav_idx in targets
    ]

    return run_monitor(
        trackers=trackers,
        stream_mode=args.stream,
        timeout=args.timeout,
        bind_host=args.bind,
        subscribe=not args.no_subscribe,
    )


if __name__ == "__main__":
    sys.exit(main())
