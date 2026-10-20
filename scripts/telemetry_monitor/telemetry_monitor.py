#!/usr/bin/env python3
"""
Monitor de Telemetría MAVLink v2 ASTM F3411 / EUROCAE ED-282 para NavSim (Isaac Sim).

Escucha dinámicamente los puertos UDP de telemetría (14650, 14651, ...) y muestra
en tiempo real ÚNICAMENTE los drones que están activos y transmitiendo:
  - Detección y filtrado dinámico de flota (se refresca y limpia automáticamente al parar o recargar escenas).
  - Coordenadas Geodésicas WGS-84 (Latitud, Longitud, Altitud HAE, Altura AGL).
  - Vector Polar ASTM F3411 (Velocidad Horizontal, Velocidad Vertical, Dirección/Track).
  - Estado Operativo ASTM F3411 (GROUND, AIRBORNE, EMERGENCY, FAILURE).
  - Telemetría Cartesiana ENU y Dinámica de Rotores de Isaac Sim.

Uso:
    python scripts/telemetry_monitor/telemetry_monitor.py
    python scripts/telemetry_monitor/telemetry_monitor.py --stream
    python scripts/telemetry_monitor/telemetry_monitor.py --show-all
"""

import argparse
from datetime import datetime
import math
import os
from pathlib import Path
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
    MavOdidBasicIdMsg,
    MavOdidHeightRef,
    MavOdidHorAcc,
    MavOdidLocationMsg,
    MavOdidOperatorIdMsg,
    MavOdidSelfIdMsg,
    MavOdidSpeedAcc,
    MavOdidStatus,
    MavOdidVerAcc,
    MSG_ID_OPEN_DRONE_ID_BASIC_ID,
    MSG_ID_OPEN_DRONE_ID_LOCATION,
    MSG_ID_OPEN_DRONE_ID_OPERATOR_ID,
    MSG_ID_OPEN_DRONE_ID_SELF_ID,
)

ODID_STATUS_NAMES = {
    int(MavOdidStatus.UNDECLARED): "UNDECLARED",
    int(MavOdidStatus.GROUND): "GROUND (En tierra)",
    int(MavOdidStatus.AIRBORNE): "AIRBORNE (En vuelo)",
    int(MavOdidStatus.EMERGENCY): "EMERGENCY (Emergencia)",
    int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE): "FAILURE (Fallo RID)",
}

ODID_STATUS_COLORS = {
    int(MavOdidStatus.UNDECLARED): "\033[90m",  # Gris
    int(MavOdidStatus.GROUND): "\033[93m",      # Amarillo
    int(MavOdidStatus.AIRBORNE): "\033[92m",    # Verde
    int(MavOdidStatus.EMERGENCY): "\033[91m",   # Rojo
    int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE): "\033[95m",  # Magenta
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
ERA_LINE = "\033[K"
ERA_DISPLAY = "\033[J"


def cardinal_direction(deg: float) -> str:
    """Devuelve el punto cardinal aproximado (16 rumbos) para un rumbo en grados."""
    cardinals = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    idx = int(round((deg % 360.0) / 22.5)) % 16
    return cardinals[idx]


class UAVTelemetryTracker:
    def __init__(self, uav_idx: int, rx_port: int, tx_drone_port: int):
        self.uav_idx = uav_idx
        self.rx_port = rx_port
        self.tx_drone_port = tx_drone_port
        self.codec = MAVLinkV2Codec(sysid=255, compid=1)
        self.sock: Optional[socket.socket] = None

        self.last_loc_msg: Optional[MavOdidLocationMsg] = None
        self.last_basic_msg: Optional[MavOdidBasicIdMsg] = None
        self.last_op_msg: Optional[MavOdidOperatorIdMsg] = None
        self.last_self_msg: Optional[MavOdidSelfIdMsg] = None

        self.last_recv_time: float = 0.0
        self.msg_count: int = 0
        self.hz: float = 0.0
        self._hz_timer: float = time.time()
        self._hz_counter: int = 0
        self.is_stopped: bool = False

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
        self.last_recv_time = 0.0
        self.hz = 0.0
        self.is_stopped = True

    def open_socket(self, host: str = "0.0.0.0") -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, self.rx_port))
        sock.setblocking(False)
        self.sock = sock

    def close(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def send_handshake_ping(self, target_host: str = "127.0.0.1") -> None:
        """
        Envía un pequeño paquete UDP al puerto RX de Isaac Sim (14550 + idx)
        para forzar la vinculación remota del simulador hacia este monitor.
        """
        if self.sock:
            try:
                self.sock.sendto(b"PING", (target_host, self.tx_drone_port))
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
        return received_events


def render_dashboard(
    trackers: List[UAVTelemetryTracker],
    now: float,
    timeout: float = 2.0,
    show_all: bool = False,
) -> str:
    lines = []
    lines.append(f"{BOLD}{CYAN}=== MONITOR DE TELEMETRÍA MAVLink v2 ASTM F3411 / EUROCAE ED-282 ==={RESET_COLOR}")

    if show_all:
        display_trackers = trackers
    else:
        # Filtrar ÚNICAMENTE los drones que están transmitiendo activamente
        display_trackers = [t for t in trackers if t.is_active(now, timeout=timeout)]

    active_count = len([t for t in trackers if t.is_active(now, timeout=timeout)])
    sub_title = (
        f"Fecha/Hora: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | "
        f"Drones activos: {active_count}"
    )
    if active_count == 0:
        sub_title += " (Simulación detenida / En espera)"
    lines.append(sub_title)
    lines.append("-" * 96)

    col_w = 26

    # Si no hay ningún dron activo transmitiendo, mostrar pantalla de espera limpia
    if not display_trackers:
        lines.append("")
        lines.append(f"  {BOLD}\033[93m[En espera de telemetría activa desde Isaac Sim en puertos UDP 14650+ ...]{RESET_COLOR}")
        lines.append(f"  \033[90m(Pulsa PLAY en Isaac Sim para iniciar la simulación y ver los drones activos)\033[0m")
        lines.append("")
        lines.append("-" * 96)
        lines.append("Presiona Ctrl+C para salir.")
        cleared_lines = [f"{line}{ERA_LINE}" for line in lines]
        return "\n".join(cleared_lines) + ERA_DISPLAY

    # Detectar duplicidad de identificadores en el aire (Conflicto ASTM F3411)
    active_ids: Dict[str, List[int]] = {}
    for tr in display_trackers:
        uid = (
            tr.last_basic_msg.uas_id
            if (tr.last_basic_msg and tr.last_basic_msg.uas_id)
            else f"UAV_{tr.uav_idx:02d}"
        ).strip().upper()
        active_ids.setdefault(uid, []).append(tr.uav_idx + 1)

    has_conflict = False
    for uid, uav_list in active_ids.items():
        if len(uav_list) > 1:
            has_conflict = True
            lines.append(
                f"  \033[91m{BOLD}🚨 CONFLICTO CRÍTICO EN EL AIRE: Identificador '{uid}' duplicado en UAVs {uav_list}! "
                f"(ASTM F3411 prohíbe IDs duplicados){RESET_COLOR}"
            )
    if has_conflict:
        lines.append("-" * 96)

    for tr in display_trackers:
        is_recent = tr.is_active(now, timeout=timeout)
        hz_str = f"{tr.hz:.1f} Hz" if is_recent else "0.0 Hz"

        uas_id = (
            tr.last_basic_msg.uas_id
            if (tr.last_basic_msg and tr.last_basic_msg.uas_id)
            else f"UAV_{tr.uav_idx:02d}"
        )

        loc = tr.last_loc_msg
        basic = tr.last_basic_msg

        if not is_recent:
            state_str = "TIMEOUT (>2s)" if tr.last_recv_time > 0 else "SIN DATOS"
            state_col = "\033[91m" if tr.last_recv_time > 0 else "\033[90m"
        elif loc is not None:
            st = loc.status
            state_str = ODID_STATUS_NAMES.get(st, f"STATUS({st})")
            state_col = ODID_STATUS_COLORS.get(st, "\033[97m")
        else:
            state_str = "ESPERANDO"
            state_col = "\033[90m"

        state_tag = f"[{state_str}]"
        time_str = f"{loc.timestamp:.1f} s UTC" if loc else "--"

        header = (
            f"{BOLD}UAV #{tr.uav_idx + 1:02d} ({uas_id} | Puerto :{tr.rx_port}){RESET_COLOR} -> "
            f"{state_col}{BOLD}{state_tag:<22s}{RESET_COLOR}  "
            f"(Tiempo: {time_str:>8s} | Tasa: {hz_str:>7s})"
        )
        lines.append(header)

        # Identificación UAS según estándar ASTM F3411 OPEN_DRONE_ID_BASIC_ID
        if basic is not None:
            ua_type_str = UA_TYPE_NAMES.get(basic.ua_type, f"Tipo({basic.ua_type})")
            id_type_str = ID_TYPE_NAMES.get(basic.id_type, f"TipoID({basic.id_type})")
            c_id = f"ID: {uas_id}".ljust(col_w)
            c_typ = f"Tipo: {ua_type_str}".ljust(col_w + 5)
            c_idt = f"Tipo ID: {id_type_str}"
            lines.append(f"  • Identificación UAS: {c_id}  {c_typ}  {c_idt}")

        # Registro oficial de operador CAA y Self ID (OPEN_DRONE_ID_OPERATOR_ID / SELF_ID)
        if tr.last_op_msg is not None or tr.last_self_msg is not None:
            op_text = f"Operador: {tr.last_op_msg.operator_id}" if (tr.last_op_msg and tr.last_op_msg.operator_id) else "Operador: --"
            desc_text = f"Misión: {tr.last_self_msg.description}" if (tr.last_self_msg and tr.last_self_msg.description) else ""
            c_op = op_text.ljust(col_w + 5)
            lines.append(f"  • Registro Operador: {c_op}  {desc_text}")

        if loc is not None:
            # 1. Posición WGS-84 (Latitud, Longitud, Altitud HAE recibidas)
            c_lat = f"Lat: {loc.lat_deg:+11.7f}°".ljust(col_w)
            c_lon = f"Lon: {loc.lon_deg:+11.7f}°".ljust(col_w)
            c_alt = f"Alt HAE: {loc.altitude_geodetic:+7.2f} m"
            lines.append(f"  • Posición WGS-84:  {c_lat}  {c_lon}  {c_alt}")

            # 2. Alturas ASTM F3411
            c_h = f"Altura (AGL): {loc.height:6.2f} m".ljust(col_w)
            c_baro = f"Alt Baro: {loc.altitude_barometric:+7.2f} m".ljust(col_w)
            href_name = "Suelo (AGL)" if loc.height_reference == 1 else "Despegue"
            c_href = f"Ref: {href_name}"
            lines.append(f"  • Alturas ASTM:     {c_h}  {c_baro}  {c_href}")

            # 3. Vector Polar de Velocidad ASTM F3411
            v_h = loc.speed_h_ms
            v_v = loc.speed_v_ms
            c_vh = f"V. Horiz: {v_h:5.2f} m/s ({v_h * 3.6:5.1f} km/h)".ljust(col_w + 5)
            vert_trend = "Ascenso" if v_v > 0.05 else ("Descenso" if v_v < -0.05 else "Nivelado")
            c_vv = f"V. Vert: {v_v:+5.2f} m/s ({vert_trend})"
            lines.append(f"  • Velocidad ASTM:   {c_vh}  {c_vv}")

            # 4. Rumbo / Track sobre el terreno
            dir_deg = loc.direction_deg
            card_dir = cardinal_direction(dir_deg)
            c_dir = f"Rumbo/Track: {dir_deg:5.1f}° ({card_dir:<3s})".ljust(col_w)
            c_acc = "Exactitud: Horiz <1m | Vert <1m"
            lines.append(f"  • Rumbo / Track:    {c_dir}  {c_acc}")
        else:
            lines.append(f"  • Posición WGS-84:  \033[91m[NO TRANSMITIDA - Falta trama ASTM F3411 OPEN_DRONE_ID_LOCATION]\033[0m")
            lines.append(f"  • Alturas ASTM:     Altura (AGL): --      Alt Baro: --           Ref: --")
            lines.append(f"  • Velocidad ASTM:   V. Horiz: --          V. Vert: --")
            lines.append(f"  • Rumbo / Track:    Rumbo: --")

        lines.append("-" * 96)

    lines.append("Presiona Ctrl+C para salir.")
    cleared_lines = [f"{line}{ERA_LINE}" for line in lines]
    return "\n".join(cleared_lines) + ERA_DISPLAY


def run_monitor(
    ports: List[int],
    base_rx_port: int,
    stream_mode: bool,
    target_host: str,
    timeout: float = 2.0,
    show_all: bool = False,
) -> None:

    trackers: List[UAVTelemetryTracker] = []
    for idx, tx_port in enumerate(ports):
        tr = UAVTelemetryTracker(
            uav_idx=idx,
            rx_port=tx_port,
            tx_drone_port=base_rx_port + idx,
        )
        tr.open_socket()
        tr.send_handshake_ping(target_host=target_host)
        trackers.append(tr)

    print(f"Monitor escuchando en {len(ports)} puertos UDP ({ports[0]}..{ports[-1]})...")
    if not stream_mode:
        os.system("cls" if os.name == "nt" else "clear")

    last_render_time = 0.0
    try:
        while True:
            socks = [t.sock for t in trackers if t.sock is not None]
            readable, _, _ = select.select(socks, [], [], 0.05)

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
                                        st_name = ODID_STATUS_NAMES.get(m.status, str(m.status))
                                        card = cardinal_direction(m.direction_deg)
                                        print(
                                            f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] "
                                            f"UAV #{tr.uav_idx + 1:02d} [ASTM F3411] "
                                            f"Lat={m.lat_deg:+.7f}° Lon={m.lon_deg:+.7f}° "
                                            f"Alt={m.altitude_geodetic:.1f}m AGL={m.height:.1f}m "
                                            f"Vh={m.speed_h_ms:.1f}m/s Vz={m.speed_v_ms:+.1f}m/s "
                                             f"Dir={m.direction_deg:.1f}°({card}) Status={st_name}"
                                        )

            now = time.time()
            # Limpiar estado de trackers que hayan superado el timeout de inactividad
            for tr in trackers:
                if tr.last_recv_time > 0 and (now - tr.last_recv_time) >= timeout and not tr.is_stopped:
                    tr.reset_state()

            if not stream_mode and (new_msgs or (now - last_render_time) >= 0.1):
                output = render_dashboard(trackers, now, timeout=timeout, show_all=show_all)
                sys.stdout.write("\033[H" + output)
                sys.stdout.flush()
                last_render_time = now

    except KeyboardInterrupt:
        print("\nCerrando monitor de telemetría...")
    finally:
        for tr in trackers:
            tr.close()


def main():
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass

    parser = argparse.ArgumentParser(
        description="Monitor UDP MAVLink v2 ASTM F3411 / EUROCAE ED-282 para NavSim (Isaac Sim)."
    )
    parser.add_argument(
        "--base-tx-port",
        type=int,
        default=14650,
        help="Puerto base de telemetría transmitida por Isaac Sim (por defecto: 14650).",
    )
    parser.add_argument(
        "--base-rx-port",
        type=int,
        default=14550,
        help="Puerto base de recepción de planes de vuelo en Isaac Sim (por defecto: 14550).",
    )
    parser.add_argument(
        "--max-uavs",
        "-n",
        type=int,
        default=16,
        help="Número máximo de puertos consecutivos a sondear para detección automática (por defecto: 16).",
    )
    parser.add_argument(
        "--ports",
        nargs="+",
        type=int,
        help="Lista explícita de puertos de telemetría a escuchar (ejemplo: --ports 14650 14651).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Tiempo en segundos sin datos para considerar un dron inactivo y ocultarlo (por defecto: 2.0 s).",
    )
    parser.add_argument(
        "--show-all",
        action="store_true",
        help="Mostrar todos los puertos sondados, incluso si están inactivos o sin datos.",
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="Modo línea a línea continuo (stream) en lugar de tabla interactiva.",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="IP del host donde corre Isaac Sim (por defecto: 127.0.0.1).",
    )

    args = parser.parse_args()

    if args.ports:
        target_ports = args.ports
    else:
        target_ports = [args.base_tx_port + i for i in range(args.max_uavs)]

    run_monitor(
        ports=target_ports,
        base_rx_port=args.base_rx_port,
        stream_mode=args.stream,
        target_host=args.host,
        timeout=args.timeout,
        show_all=args.show_all,
    )


if __name__ == "__main__":
    main()
