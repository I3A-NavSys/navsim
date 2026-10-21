#!/usr/bin/env python3
"""
Monitor de Telemetría MAVLink v2 ASTM F3411 / EUROCAE ED-282 + Rutas 4D Taylor para NavSim (Isaac Sim).

Escucha dinámicamente los puertos UDP de telemetría (16000, 16001, ...) para 4 drones consecutivos
y muestra en tiempo real su estado de navegación y Remote ID:
  - Visualización continua de flota (4 drones por pantalla sin desbordamiento vertical).
  - Coordenadas Geodésicas WGS-84 (Latitud, Longitud, Altitud HAE, Altura AGL).
  - Vector Polar ASTM F3411 (Velocidad Horizontal, Velocidad Vertical, Dirección/Track).
  - Estado Operativo ASTM F3411 (GROUND, AIRBORNE, EMERGENCY, FAILURE).
  - Estado de Ejecución de Ruta 4D Taylor (`NAVSIM_FLIGHT_PLAN_STATUS`, ID 12003):
    Plan ID, Waypoint actual (`seq/total` + `label`), ventana temporal y error de seguimiento 4D.

Uso:
    python scripts/telemetry_monitor/telemetry_monitor.py              # Drones 0..3 (TX: 16000..16003, RX: 15000..15003)
    python scripts/telemetry_monitor/telemetry_monitor.py 16004        # Drones 4..7 (TX: 16004..16007, RX: 15004..15007)
    python scripts/telemetry_monitor/telemetry_monitor.py 4            # Drones 4..7 (por índice directo de UAV)
    python scripts/telemetry_monitor/telemetry_monitor.py --stream
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
    MavFlightPlanStatusMsg,
    MavOdidBasicIdMsg,
    MavOdidHeightRef,
    MavOdidHorAcc,
    MavOdidLocationMsg,
    MavOdidOperatorIdMsg,
    MavOdidSelfIdMsg,
    MavOdidSpeedAcc,
    MavOdidStatus,
    MavOdidVerAcc,
    NavSimUAVState,
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
ERA_LINE = "\033[K"
ERA_DISPLAY = "\033[J"


def cardinal_direction(deg: float) -> str:
    """Devuelve el punto cardinal aproximado (16 rumbos) para un rumbo en grados."""
    cardinals = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    idx = int(round((deg % 360.0) / 22.5)) % 16
    return cardinals[idx]


class UAVTelemetryTracker:
    def __init__(self, uav_idx: int, tx_port: int, rx_port: int):
        self.uav_idx = uav_idx
        self.tx_port = tx_port  # Puerto TX de telemetría de Isaac Sim donde escucha el monitor
        self.rx_port = rx_port  # Puerto RX de comandos de Isaac Sim (informativo)
        self.codec = MAVLinkV2Codec(sysid=255, compid=1)
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

    def open_socket(self, host: str = "0.0.0.0") -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, self.tx_port))
        sock.setblocking(False)
        self.sock = sock

    def close(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

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
                elif isinstance(msg, MavFlightPlanStatusMsg):
                    self.last_fp_status_msg = msg
                    received_events.append(("fp_status", msg))
        return received_events


def render_dashboard(
    trackers: List[UAVTelemetryTracker],
    now: float,
    timeout: float = 2.0,
) -> str:
    lines = []
    lines.append(
        f"{BOLD}{CYAN}=== MONITOR DE TELEMETRÍA MAVLink v2 (ASTM F3411 + RUTAS 4D TAYLOR) ==={RESET_COLOR}"
    )

    display_trackers = trackers
    active_count = len([t for t in trackers if t.is_active(now, timeout=timeout)])
    sub_title = (
        f"Fecha/Hora: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | "
        f"Drones activos: {active_count}/{len(trackers)}"
    )
    if active_count == 0:
        sub_title += " (En espera de telemetría - Pulsa PLAY en Isaac Sim)"
    lines.append(sub_title)
    lines.append("-" * 96)

    col_w = 26

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
        fp_st = tr.last_fp_status_msg

        if tr.is_stopped:
            state_str = "DETENIDO / DISARMED"
            state_col = "\033[91m"
        elif not is_recent:
            state_str = "TIMEOUT (>2s)" if tr.last_recv_time > 0 else "ESPERANDO CONEXIÓN"
            state_col = "\033[91m" if tr.last_recv_time > 0 else "\033[90m"
        elif loc is not None:
            st = loc.status
            state_str = ODID_STATUS_NAMES.get(st, f"STATUS({st})")
            state_col = ODID_STATUS_COLORS.get(st, "\033[97m")
        else:
            state_str = "ESPERANDO"
            state_col = "\033[90m"

        state_tag = f"[{state_str}]"
        time_str = f"{fp_st.sim_time:.2f} s sim" if fp_st else (f"{loc.timestamp:.1f} s UTC" if loc else "--")

        header = (
            f"{BOLD}UAV #{tr.uav_idx:02d} ({uas_id} | TX:{tr.tx_port} RX:{tr.rx_port}){RESET_COLOR} -> "
            f"{state_col}{BOLD}{state_tag:<22s}{RESET_COLOR}  "
            f"(Tiempo: {time_str:>10s} | Tasa: {hz_str:>7s})"
        )
        lines.append(header)

        # Si aún no se ha recibido telemetría de este dron, mostrar aviso de espera limpio
        if not is_recent and loc is None:
            lines.append(
                f"  • Telemetría:        \033[90m[Esperando datagramas UDP en puerto TX:{tr.tx_port} (Pulsa PLAY en Isaac Sim)]\033[0m"
            )
            lines.append("-" * 96)
            continue

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
            op_text = (
                f"Operador: {tr.last_op_msg.operator_id}"
                if (tr.last_op_msg and tr.last_op_msg.operator_id)
                else "Operador: --"
            )
            desc_text = (
                f"Misión: {tr.last_self_msg.description}"
                if (tr.last_self_msg and tr.last_self_msg.description)
                else ""
            )
            c_op = op_text.ljust(col_w + 5)
            lines.append(f"  • Registro Operador: {c_op}  {desc_text}")

        if loc is not None:
            # 1. Posición WGS-84 (Latitud, Longitud, Altitud HAE recibidas)
            c_lat = f"Lat: {loc.lat_deg:+11.7f}°".ljust(col_w)
            c_lon = f"Lon: {loc.lon_deg:+11.7f}°".ljust(col_w)
            c_alt = f"Alt HAE: {loc.altitude_geodetic:+7.2f} m"
            lines.append(f"  • Posición WGS-84:   {c_lat}  {c_lon}  {c_alt}")

            # 2. Alturas ASTM F3411
            c_h = f"Altura (AGL): {loc.height:6.2f} m".ljust(col_w)
            c_baro = f"Alt Baro: {loc.altitude_barometric:+7.2f} m".ljust(col_w)
            href_name = "Suelo (AGL)" if loc.height_reference == 1 else "Despegue"
            c_href = f"Ref: {href_name}"
            lines.append(f"  • Alturas ASTM:      {c_h}  {c_baro}  {c_href}")

            # 3. Vector Polar de Velocidad ASTM F3411
            v_h = loc.speed_h_ms
            v_v = loc.speed_v_ms
            c_vh = f"V. Horiz: {v_h:5.2f} m/s ({v_h * 3.6:5.1f} km/h)".ljust(col_w + 5)
            vert_trend = "Ascenso" if v_v > 0.05 else ("Descenso" if v_v < -0.05 else "Nivelado")
            c_vv = f"V. Vert: {v_v:+5.2f} m/s ({vert_trend})"
            lines.append(f"  • Velocidad ASTM:    {c_vh}  {c_vv}")

            # 4. Rumbo / Track sobre el terreno
            dir_deg = loc.direction_deg
            card_dir = cardinal_direction(dir_deg)
            c_dir = f"Rumbo/Track: {dir_deg:5.1f}° ({card_dir:<3s})".ljust(col_w)
            c_acc = f"Exactitud: Horiz <1m | Vert <1m | ODID t: {loc.timestamp:6.1f}s"
            lines.append(f"  • Rumbo / Track:     {c_dir}  {c_acc}")
        else:
            lines.append(
                "  • Posición WGS-84:   \033[91m[NO TRANSMITIDA - Falta trama ASTM F3411 OPEN_DRONE_ID_LOCATION]\033[0m"
            )

        # 5. Estado de Ejecución de Ruta 4D (Polinomio de Taylor - NAVSIM_FLIGHT_PLAN_STATUS ID 12003)
        if fp_st is not None:
            uav_mode_str = UAV_STATE_NAMES.get(fp_st.uav_state, f"STATE({fp_st.uav_state})")
            if fp_st.uav_state == int(NavSimUAVState.BUSY) and fp_st.plan_id:
                wp_idx_disp = fp_st.current_wp + 1 if fp_st.current_wp >= 0 else 0
                lbl_str = f" '{fp_st.wp_label}'" if fp_st.wp_label else ""
                wp_str = f"WP: {wp_idx_disp}/{fp_st.total_wps}{lbl_str} (t_k={fp_st.wp_time:.1f}s)"
                dur = max(fp_st.fp_finish_time - fp_st.fp_init_time, 1e-6)
                pct = min(100.0, max(0.0, (fp_st.sim_time - fp_st.fp_init_time) * 100.0 / dur))
                prog_str = f"Ventana: [{fp_st.fp_init_time:.1f}..{fp_st.fp_finish_time:.1f}s] ({pct:5.1f}%)"
                ep_norm = math.sqrt(sum(c * c for c in fp_st.pos_error))
                ev_norm = math.sqrt(sum(c * c for c in fp_st.vel_error))
                c_plan = f"Plan: {fp_st.plan_id} ({uav_mode_str})".ljust(col_w + 5)
                c_wp = wp_str.ljust(col_w + 5)
                lines.append(f"  • Ruta 4D (Taylor):  {c_plan}  {c_wp}  {prog_str}")
                c_des = (
                    f"Ref Taylor: [{fp_st.desired_pos[0]:+.1f}, {fp_st.desired_pos[1]:+.1f}, {fp_st.desired_pos[2]:+.1f}] m"
                ).ljust(col_w + 12)
                c_err = f"Error 4D: |e_pos|={ep_norm:.3f} m | |e_vel|={ev_norm:.3f} m/s"
                lines.append(f"  • Seguimiento 4D:    {c_des}  {c_err}")
            else:
                c_mode = f"Modo: {uav_mode_str}".ljust(col_w + 5)
                lines.append(f"  • Ruta 4D (Taylor):  {c_mode}  Plan activo: Ninguno")

        lines.append("-" * 96)

    lines.append("Presiona Ctrl+C para salir.")
    cleared_lines = [f"{line}{ERA_LINE}" for line in lines]
    return "\n".join(cleared_lines) + ERA_DISPLAY


def run_monitor(
    trackers: List[UAVTelemetryTracker],
    stream_mode: bool,
    timeout: float = 2.0,
) -> None:
    for tr in trackers:
        tr.open_socket()

    uav_range_str = f"#{trackers[0].uav_idx:02d}..#{trackers[-1].uav_idx:02d}"
    tx_range_str = f"{trackers[0].tx_port}..{trackers[-1].tx_port}"
    rx_range_str = f"{trackers[0].rx_port}..{trackers[-1].rx_port}"
    print(f"Monitor escuchando {len(trackers)} UAVs ({uav_range_str}) | TX:{tx_range_str} | RX:{rx_range_str}...")
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
                                        fp_info = ""
                                        if tr.last_fp_status_msg is not None:
                                            fp = tr.last_fp_status_msg
                                            if fp.uav_state == int(NavSimUAVState.BUSY) and fp.plan_id:
                                                ep = math.sqrt(sum(c * c for c in fp.pos_error))
                                                fp_info = (
                                                    f" | 4D Plan={fp.plan_id} "
                                                    f"WP={fp.current_wp + 1}/{fp.total_wps}({fp.wp_label}) "
                                                    f"|e_pos|={ep:.2f}m"
                                                )
                                            else:
                                                fp_info = f" | Mode={UAV_STATE_NAMES.get(fp.uav_state, str(fp.uav_state))}"
                                        uas_id = (
                                            tr.last_basic_msg.uas_id
                                            if (tr.last_basic_msg and tr.last_basic_msg.uas_id)
                                            else f"UAV_{tr.uav_idx:02d}"
                                        )
                                        print(
                                            f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] "
                                            f"UAV #{tr.uav_idx:02d} ({uas_id} | TX:{tr.tx_port} RX:{tr.rx_port}) [ASTM F3411] "
                                            f"Lat={m.lat_deg:+.7f}° Lon={m.lon_deg:+.7f}° "
                                            f"Alt={m.altitude_geodetic:.1f}m AGL={m.height:.1f}m "
                                            f"Vh={m.speed_h_ms:.1f}m/s Vz={m.speed_v_ms:+.1f}m/s "
                                            f"Dir={m.direction_deg:.1f}°({card}) Status={st_name}{fp_info}"
                                        )

            now = time.time()
            # Limpiar estado de trackers que hayan superado el timeout de inactividad
            for tr in trackers:
                if tr.last_recv_time > 0 and (now - tr.last_recv_time) >= timeout and not tr.is_stopped:
                    tr.reset_state()

            if not stream_mode and (new_msgs or (now - last_render_time) >= 0.1):
                output = render_dashboard(trackers, now, timeout=timeout)
                sys.stdout.write("\033[H" + output)
                sys.stdout.flush()
                last_render_time = now

    except KeyboardInterrupt:
        print("\nCerrando monitor de telemetría...")
    finally:
        for tr in trackers:
            tr.close()


def main():
    # Habilitar secuencias de escape ANSI en consolas Windows (cmd.exe y PowerShell)
    if os.name == "nt":
        os.system("")

    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    parser = argparse.ArgumentParser(
        description="Monitor UDP MAVLink v2 ASTM F3411 / EUROCAE ED-282 + Rutas 4D Taylor para NavSim (Isaac Sim)."
    )
    parser.add_argument(
        "port",
        nargs="?",
        type=int,
        default=16000,
        help="Puerto TX inicial de telemetría (por defecto: 16000) o índice de UAV para monitorizar 4 drones.",
    )
    parser.add_argument(
        "--port",
        "-p",
        dest="opt_port",
        type=int,
        default=None,
        help="Puerto inicial TX o índice de UAV (alternativa con flag).",
    )
    parser.add_argument(
        "--base-tx-port",
        type=int,
        default=16000,
        help="Puerto base de telemetría transmitida por Isaac Sim (por defecto: 16000).",
    )
    parser.add_argument(
        "--base-rx-port",
        type=int,
        default=15000,
        help="Puerto base de comandos en Isaac Sim (por defecto: 15000).",
    )
    parser.add_argument(
        "--count",
        "-n",
        dest="max_uavs",
        type=int,
        default=4,
        help="Número de drones consecutivos a monitorizar (por defecto: 4).",
    )
    parser.add_argument(
        "--ports",
        nargs="+",
        type=int,
        help="Lista explícita de puertos de telemetría a escuchar (ejemplo: --ports 16000 16001).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Tiempo en segundos sin datos para considerar un dron inactivo (por defecto: 2.0 s).",
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="Modo línea a línea continuo (stream) en lugar de tabla interactiva.",
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="IP local donde enlazar la escucha UDP (por defecto: 0.0.0.0).",
    )

    args = parser.parse_args()

    trackers: List[UAVTelemetryTracker] = []

    if args.ports:
        for idx, tx_port in enumerate(args.ports):
            u_idx = tx_port - args.base_tx_port if tx_port >= args.base_tx_port else idx
            tr = UAVTelemetryTracker(
                uav_idx=u_idx,
                tx_port=tx_port,
                rx_port=args.base_rx_port + u_idx,
            )
            trackers.append(tr)
    else:
        # Resolver puerto inicial o índice base (por defecto: 16000)
        raw_val = args.opt_port if args.opt_port is not None else args.port
        if raw_val is None:
            raw_val = 16000

        if raw_val >= args.base_tx_port:
            base_uav_idx = raw_val - args.base_tx_port
        elif raw_val >= args.base_rx_port:
            base_uav_idx = raw_val - args.base_rx_port
        else:
            base_uav_idx = max(0, raw_val)

        for i in range(args.max_uavs):
            u_idx = base_uav_idx + i
            rx_drone_p = args.base_rx_port + u_idx
            tx_drone_p = args.base_tx_port + u_idx
            tr = UAVTelemetryTracker(
                uav_idx=u_idx,
                tx_port=tx_drone_p, # El monitor escucha telemetría en el puerto TX del dron (16000+)
                rx_port=rx_drone_p, # Puerto RX del dron (15000+) para información en pantalla
            )
            trackers.append(tr)

    run_monitor(
        trackers=trackers,
        stream_mode=args.stream,
        timeout=args.timeout,
    )


if __name__ == "__main__":
    main()
