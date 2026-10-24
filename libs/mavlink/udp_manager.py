import logging
import math
import socket
import time
from typing import Any, Dict, List, Optional, Set, Tuple
import numpy as np

from .dialect import (
    FlightPlanData,
    MAVLinkV2Codec,
    MAVLINK_STX_V2,
    MavFlightPlanAckMsg,
    MavFlightPlanHeaderMsg,
    MavFlightPlanStatusMsg,
    MavFlightPlanWaypointMsg,
    MavManualControlMsg,
    MavOdidBasicIdMsg,
    MavOdidHeightRef,
    MavOdidHorAcc,
    MavOdidLocationMsg,
    MavOdidOperatorIdMsg,
    MavOdidSelfIdMsg,
    MavOdidSpeedAcc,
    MavOdidStatus,
    MavOdidTimeAcc,
    MavOdidVerAcc,
    NavSimFPAckResult,
    NavSimUAVState,
    WaypointData,
    _encode_str,
)


class _PendingFlightPlan:
    """Buffer temporal para ensamblar los waypoints de un FlightPlan recibido por MAVLink."""

    def __init__(self, plan_id: str, num_waypoints: int):
        self.plan_id = plan_id
        self.priority: int = 0
        self.radius: float = 1.0
        self.max_var_lin_vel: float = 5.0
        self.max_var_ang_vel: float = 1.0
        self.target_yaw: Optional[float] = None
        self.num_waypoints: int = num_waypoints
        self.waypoints_by_seq: Dict[int, WaypointData] = {}

    def apply_header(self, header: MavFlightPlanHeaderMsg) -> None:
        self.priority = header.priority
        self.radius = header.radius
        self.max_var_lin_vel = header.max_var_lin_vel
        self.max_var_ang_vel = header.max_var_ang_vel
        self.target_yaw = header.target_yaw
        self.num_waypoints = header.num_waypoints

    def add_waypoint(self, msg: MavFlightPlanWaypointMsg) -> bool:
        if msg.num_waypoints > 0:
            self.num_waypoints = msg.num_waypoints
        self.waypoints_by_seq[msg.seq] = msg.to_waypoint_data()
        return self.is_complete()

    def is_complete(self) -> bool:
        return self.num_waypoints > 0 and len(self.waypoints_by_seq) >= self.num_waypoints

    def build(self) -> FlightPlanData:
        ordered_wps = [
            self.waypoints_by_seq[i]
            for i in sorted(self.waypoints_by_seq.keys())
        ]
        return FlightPlanData(
            id=self.plan_id,
            priority=self.priority,
            radius=self.radius,
            max_var_lin_vel=self.max_var_lin_vel,
            max_var_ang_vel=self.max_var_ang_vel,
            target_yaw=self.target_yaw,
            waypoints=ordered_wps,
        )


class UAVMavlinkEndpoint:
    """
    Endpoint lógico MAVLink v2 para un único dron simulado.
    Contiene la lógica de aviónica, identificación y serialización de telemetría.
    El transporte de red es gestionado centralizadamente por FleetMavlinkManager
    a través de un único socket UDP bidireccional.
    """

    def __init__(
        self,
        uav_index: int,
        uav_id: str,
        identity: Optional[Any] = None,
        sim_port: int = 14550,
    ):
        self.uav_index = uav_index
        self.uav_id = uav_id
        self.identity = identity
        self.sysid = (uav_index % 254) + 1
        self.sim_port = sim_port
        self.rx_port = sim_port
        self.tx_port = sim_port

        self.codec = MAVLinkV2Codec(sysid=self.sysid, compid=1)
        self.sock: Optional[socket.socket] = None
        self._pending_plans: Dict[str, _PendingFlightPlan] = {}
        self.active_flight_plan: Optional[FlightPlanData] = None
        self.latest_completed_fp: Optional[FlightPlanData] = None
        self.latest_manual_cmd: Optional[MavManualControlMsg] = None
        self.last_fp_sender_addr: Optional[Tuple[str, int]] = None
        self.last_cmd_addr: Optional[Tuple[str, int]] = None

    def open(self) -> None:
        """Compatibilidad: el socket real es administrado por FleetMavlinkManager."""
        pass

    def close(self) -> None:
        self._pending_plans.clear()
        self.latest_completed_fp = None
        self.latest_manual_cmd = None
        self.last_fp_sender_addr = None
        self.last_cmd_addr = None

    def drain_socket(self) -> None:
        """Compatibilidad: el drenaje se realiza centralizadamente en FleetMavlinkManager."""
        pass

    def poll_incoming(self) -> Optional[FlightPlanData]:
        fp = self.latest_completed_fp
        self.latest_completed_fp = None
        return fp

    def build_stop_notification(self) -> bytes:
        """Emite notificación estándar de fin de simulación para limpiar receptores externos."""
        id_bytes = _encode_str(self.uav_id, 20)
        return self.codec.pack_odid_location(
            MavOdidLocationMsg(
                status=int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE),
                speed_horizontal=0,
                speed_vertical=0,
                id_or_mac=id_bytes,
            )
        )

    def build_telemetry_frames(
        self,
        sim_time: float,
        uav_state: int,
        plan_id: str,
        current_wp: int,
        pos: np.ndarray,
        euler: np.ndarray,
        quat: np.ndarray,
        lin_vel: np.ndarray,
        ang_vel: np.ndarray,
        rotor_speeds: np.ndarray,
        georeference=None,
        total_wps: int = 0,
        wp_label: str = "",
        fp_init_time: float = 0.0,
        fp_finish_time: float = 0.0,
        wp_time: float = 0.0,
        desired_pos: Optional[np.ndarray] = None,
        desired_vel: Optional[np.ndarray] = None,
        pos_error: Optional[np.ndarray] = None,
        vel_error: Optional[np.ndarray] = None,
        time_reference: Optional[Any] = None,
    ) -> bytes:
        """
        Construye el conjunto de tramas MAVLink v2 (ASTM F3411 OpenDroneID y NavSim Status)
        para este UAV y devuelve el paquete binario completo listo para enviar.
        """
        # Posición geográfica geodésica WGS-84
        if georeference is None:
            try:
                from extensions.navsim.navsim_python.georeference import WGS84Georeference
                georeference = WGS84Georeference()
            except Exception:
                georeference = None

        if georeference is not None:
            geo = georeference.enu_to_geodetic(pos)
            lat_deg, lon_deg, alt_hae = float(geo[0]), float(geo[1]), float(geo[2])
        else:
            lat_deg, lon_deg, alt_hae = 38.981800, -1.857000, 686.0 + float(pos[2])

        lat_e7 = int(round(lat_deg * 1e7))
        lon_e7 = int(round(lon_deg * 1e7))
        height_m = float(pos[2])  # Isaac Sim Z en ENU = altura AGL sobre el suelo/vertipuerto

        # Velocidades cartesianas en ENU: +X=Este, +Y=Norte, +Z=Arriba
        vx = float(lin_vel[0])
        vy = float(lin_vel[1])
        vz = float(lin_vel[2])

        # Velocidad horizontal en cm/s (módulo >= 0)
        v_h = math.sqrt(vx * vx + vy * vy)
        speed_h_cm = int(round(v_h * 100.0))
        speed_horizontal = min(25425, max(0, speed_h_cm))

        # Velocidad vertical en cm/s (positivo hacia arriba)
        speed_v_cm = int(round(vz * 100.0))
        speed_vertical = min(6200, max(-6200, speed_v_cm))

        # Dirección sobre el terreno (ASTM F3411 track angle en centígrados 0..35999)
        if v_h >= 0.1:
            track_rad = math.atan2(vx, vy)
            track_deg = (math.degrees(track_rad) + 360.0) % 360.0
            direction_cdeg = int(round(track_deg * 100.0)) % 36000
        else:
            # En reposo, usar el rumbo (heading)
            yaw_deg = math.degrees(float(euler[2]))
            heading_deg = (90.0 - yaw_deg) % 360.0
            direction_cdeg = int(round(heading_deg * 100.0)) % 36000

        # Estado operativo ASTM F3411
        if uav_state == int(NavSimUAVState.DEAD):
            odid_status = int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE)
        elif height_m < 0.15 and v_h < 0.2:
            odid_status = int(MavOdidStatus.GROUND)
        else:
            odid_status = int(MavOdidStatus.AIRBORNE)

        # Timestamps
        if time_reference is not None:
            odid_timestamp = float(time_reference.get_odid_timestamp(sim_time))
            time_unix_usec = int(time_reference.get_unix_usec(sim_time))
        else:
            odid_timestamp = float(time.time() % 3600.0)
            time_unix_usec = int(time.time() * 1e6)

        id_bytes = _encode_str(self.uav_id, 20)

        # Trama 1: OPEN_DRONE_ID_LOCATION (ID 12901)
        frame_loc = self.codec.pack_odid_location(
            MavOdidLocationMsg(
                latitude=lat_e7,
                longitude=lon_e7,
                altitude_barometric=alt_hae,
                altitude_geodetic=alt_hae,
                height=height_m,
                timestamp=odid_timestamp,
                direction=direction_cdeg,
                speed_horizontal=speed_horizontal,
                speed_vertical=speed_vertical,
                target_system=0,
                target_component=0,
                id_or_mac=id_bytes,
                status=odid_status,
                height_reference=int(MavOdidHeightRef.OVER_GROUND),
                horizontal_accuracy=int(MavOdidHorAcc.ACC_1M),
                vertical_accuracy=int(MavOdidVerAcc.ACC_1M),
                barometer_accuracy=int(MavOdidVerAcc.ACC_1M),
                speed_accuracy=int(MavOdidSpeedAcc.ACC_0_3MS),
                timestamp_accuracy=int(MavOdidTimeAcc.ACC_0_1S),
            )
        )

        # Trama 2: OPEN_DRONE_ID_BASIC_ID (ID 12900)
        id_type = int(getattr(self.identity, "id_type", 1)) if self.identity else 1
        ua_type = int(getattr(self.identity, "ua_type", 2)) if self.identity else 2
        uas_id = str(getattr(self.identity, "uas_id", self.uav_id)) if self.identity else self.uav_id

        frame_basic = self.codec.pack_odid_basic_id(
            MavOdidBasicIdMsg(
                target_system=0,
                target_component=0,
                id_or_mac=id_bytes,
                id_type=id_type,
                ua_type=ua_type,
                uas_id=uas_id,
            )
        )

        frames_to_send = bytearray(frame_loc + frame_basic)

        # Trama 3: OPEN_DRONE_ID_OPERATOR_ID (ID 12905)
        op_id = getattr(self.identity, "operator_id", "") if self.identity else ""
        if op_id:
            op_type = int(getattr(self.identity, "operator_id_type", 0))
            frame_op = self.codec.pack_odid_operator_id(
                MavOdidOperatorIdMsg(
                    target_system=0,
                    target_component=0,
                    id_or_mac=id_bytes,
                    operator_id_type=op_type,
                    operator_id=op_id,
                )
            )
            frames_to_send.extend(frame_op)

        # Trama 4: OPEN_DRONE_ID_SELF_ID (ID 12903)
        self_desc_fallback = getattr(self.identity, "self_id_desc", "") if self.identity else ""
        if uav_state == int(NavSimUAVState.DEAD):
            desc_type = 1
            self_desc = "EMERGENCY / FAILURE"
        elif uav_state == int(NavSimUAVState.BUSY) and plan_id:
            desc_type = 0
            clean_plan = str(plan_id).strip()
            self_desc = f"Mision: {clean_plan}" if len(clean_plan) <= 15 else clean_plan
        elif uav_state == int(NavSimUAVState.MANUAL):
            desc_type = 0
            self_desc = "Control Manual"
        else:
            desc_type = 0
            self_desc = self_desc_fallback or ""

        self_desc = str(self_desc)[:23]
        frame_self = self.codec.pack_odid_self_id(
            MavOdidSelfIdMsg(
                target_system=0,
                target_component=0,
                id_or_mac=id_bytes,
                description_type=desc_type,
                description=self_desc,
            )
        )
        frames_to_send.extend(frame_self)

        # Trama 5: NAVSIM_FLIGHT_PLAN_STATUS (ID 12003)
        t_init_clean = float(fp_init_time) if math.isfinite(float(fp_init_time)) else 0.0
        t_fin_clean = float(fp_finish_time) if math.isfinite(float(fp_finish_time)) else 0.0
        t_wp_clean = float(wp_time) if math.isfinite(float(wp_time)) else 0.0
        d_pos_tuple = (
            (float(desired_pos[0]), float(desired_pos[1]), float(desired_pos[2]))
            if desired_pos is not None
            else (0.0, 0.0, 0.0)
        )
        d_vel_tuple = (
            (float(desired_vel[0]), float(desired_vel[1]), float(desired_vel[2]))
            if desired_vel is not None
            else (0.0, 0.0, 0.0)
        )
        e_pos_tuple = (
            (float(pos_error[0]), float(pos_error[1]), float(pos_error[2]))
            if pos_error is not None
            else (0.0, 0.0, 0.0)
        )
        e_vel_error = (
            (float(vel_error[0]), float(vel_error[1]), float(vel_error[2]))
            if vel_error is not None
            else (0.0, 0.0, 0.0)
        )

        frame_fp_status = self.codec.pack_flight_plan_status(
            MavFlightPlanStatusMsg(
                time_unix_usec=time_unix_usec,
                sim_time=float(sim_time),
                uav_state=int(uav_state),
                plan_id=str(plan_id),
                current_wp=int(current_wp),
                total_wps=int(total_wps),
                wp_label=str(wp_label),
                fp_init_time=t_init_clean,
                fp_finish_time=t_fin_clean,
                wp_time=t_wp_clean,
                desired_pos=d_pos_tuple,
                desired_vel=d_vel_tuple,
                pos_error=e_pos_tuple,
                vel_error=e_vel_error,
            )
        )
        frames_to_send.extend(frame_fp_status)
        return bytes(frames_to_send)

    def send_telemetry(self, *args, **kwargs) -> None:
        """Compatibilidad con llamadas directas por endpoint."""
        pass


class FleetMavlinkManager:
    """
    Gestor central de comunicaciones MAVLink v2 para toda la flota en escena.
    Mantiene un único socket UDP bidireccional (por defecto en el puerto 14550)
    para recibir órdenes de joystick, planes de vuelo y solicitudes de suscripción,
    y emitir telemetría multiplexada bajo demanda.
    """

    def __init__(
        self,
        uav_ids: List[str],
        sim_port: int = 14550,
        base_rx_port: Optional[int] = None,
        base_tx_port: Optional[int] = None,
        bind_host: str = "0.0.0.0",
        telemetry_rate_hz: float = 1.0,
        georeference=None,
        identities: Optional[List[Any]] = None,
        time_reference=None,
    ):
        self.logger = logging.getLogger("NavSim.MAVLink")
        self.uav_ids = list(uav_ids)
        self.identities = list(identities) if identities else []
        self.sim_port = int(base_rx_port if base_rx_port is not None else sim_port)
        self.bind_host = bind_host
        self.base_rx_port = self.sim_port
        self.base_tx_port = self.sim_port
        self.telemetry_rate_hz = max(0.1, float(telemetry_rate_hz))
        self.telemetry_period = 1.0 / self.telemetry_rate_hz

        if georeference is None:
            try:
                from extensions.navsim.navsim_python.georeference import WGS84Georeference
                georeference = WGS84Georeference()
            except Exception:
                georeference = None
        self.georeference = georeference

        if time_reference is None:
            try:
                from extensions.navsim.navsim_python.time_reference import NavSimTimeReference
                time_reference = NavSimTimeReference()
            except Exception:
                time_reference = None
        self.time_reference = time_reference
        self._last_telemetry_time: float = -1.0

        self.sock: Optional[socket.socket] = None
        # Suscriptores: (ip, port) -> {"targets": Optional[Set[int]], "expire_time": float}
        # targets == None indica suscripción a ALL UAVs
        self.subscribers: Dict[Tuple[str, int], Dict[str, Any]] = {}

        # Mapeos de enrutamiento
        self._plan_to_uav: Dict[str, int] = {}
        self._plan_sender_addr: Dict[str, Tuple[str, int]] = {}

        self.endpoints: List[UAVMavlinkEndpoint] = [
            UAVMavlinkEndpoint(
                uav_index=i,
                uav_id=uid,
                identity=self.identities[i] if i < len(self.identities) else None,
                sim_port=self.sim_port,
            )
            for i, uid in enumerate(self.uav_ids)
        ]
        self._generic_codec = MAVLinkV2Codec(sysid=255, compid=1)

    def set_telemetry_rate(self, rate_hz: float) -> None:
        self.telemetry_rate_hz = max(0.1, float(rate_hz))
        self.telemetry_period = 1.0 / self.telemetry_rate_hz

    def set_georeference(self, georeference) -> None:
        self.georeference = georeference

    def set_time_reference(self, time_reference) -> None:
        self.time_reference = time_reference

    def start(self) -> None:
        if self.sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_host, self.sim_port))
        sock.setblocking(False)
        self.sock = sock

        for ep in self.endpoints:
            ep.sock = sock

        self.logger.info(
            f"[NavSim] Socket UDP bidireccional activo en {self.bind_host}:{self.sim_port} "
            f"para {len(self.endpoints)} aeronaves."
        )

    def stop(self) -> None:
        now = time.time()
        active_clients = [k for k, v in self.subscribers.items() if v["expire_time"] >= now]
        if self.sock is not None and active_clients:
            for ep in self.endpoints:
                stop_bytes = ep.build_stop_notification()
                for dest in active_clients:
                    try:
                        self.sock.sendto(stop_bytes, dest)
                    except OSError:
                        pass

        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

        for ep in self.endpoints:
            ep.close()
        self.subscribers.clear()
        self._last_telemetry_time = -1.0

    def drain_socket(self) -> None:
        """Lee y despacha todos los datagramas pendientes en el socket central UDP."""
        if self.sock is None:
            return

        now = time.time()
        while True:
            try:
                data, addr = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except ConnectionResetError:
                # Windows WSAECONNRESET tras ICMP port unreachable
                continue
            except OSError:
                break

            if not data or not addr:
                continue

            # 1. Comandos de infraestructura del simulador (Texto ASCII)
            # Formatos admitidos:
            #   SIM:SUB <client_port> [target] [ttl]
            #   SIM:UNSUB <client_port> [target]
            #   NAVSIM_SUB
            if data.startswith(b"SIM:SUB"):
                self._handle_subscription(data.decode("utf-8", errors="ignore").strip(), addr, now)
                continue
            elif data.startswith(b"SIM:UNSUB"):
                self._handle_unsubscription(data.decode("utf-8", errors="ignore").strip(), addr)
                continue
            elif data.startswith(b"NAVSIM_SUB"):
                # Compatibilidad con clientes simples
                self._register_subscriber(addr[0], 16000, targets=None, ttl=3.0, now=now)
                continue

            # 2. Tramas MAVLink v2 (Inician con 0xFD)
            if data[0] == MAVLINK_STX_V2:
                self._handle_mavlink(data, addr)

    def _handle_subscription(self, cmd_text: str, addr: Tuple[str, int], now: float) -> None:
        parts = cmd_text.split()
        client_port = addr[1]
        target = "ALL"
        ttl = 3.0

        if len(parts) > 1:
            try:
                client_port = int(parts[1])
            except ValueError:
                client_port = addr[1]
        if len(parts) > 2:
            target = parts[2].strip()
        if len(parts) > 3:
            try:
                ttl = max(0.5, float(parts[3]))
            except ValueError:
                ttl = 3.0

        targets_set: Optional[Set[int]] = None
        if target.upper() == "FIRST":
            targets_set = {0} if self.endpoints else set()
        elif target.upper() != "ALL":
            targets_set = set()
            for t_item in target.split(","):
                t_item = t_item.strip()
                if not t_item:
                    continue
                if t_item.upper() == "FIRST":
                    if self.endpoints:
                        targets_set.add(0)
                    continue

                # 1. Rango numérico simple "0-4" (solo dígitos y guion)
                if "-" in t_item and not any(c.isalpha() for c in t_item):
                    try:
                        s_str, e_str = t_item.split("-", 1)
                        for u_idx in range(int(s_str), int(e_str) + 1):
                            if 0 <= u_idx < len(self.endpoints):
                                targets_set.add(u_idx)
                        continue
                    except ValueError:
                        pass

                # 2. Número de índice directo (ej. 23)
                try:
                    u_idx = int(t_item)
                    if 0 <= u_idx < len(self.endpoints):
                        targets_set.add(u_idx)
                        continue
                except ValueError:
                    pass

                # 3. Identificador de aeronave (ej. ESP-ABJ-2026-23)
                matched = False
                for u_idx, uid in enumerate(self.uav_ids):
                    if uid.upper() == t_item.upper():
                        targets_set.add(u_idx)
                        matched = True

                # 4. Coincidencia parcial (ej. "2026-23")
                if not matched:
                    for u_idx, uid in enumerate(self.uav_ids):
                        if t_item.upper() in uid.upper():
                            targets_set.add(u_idx)

        self._register_subscriber(addr[0], client_port, targets_set, ttl, now)

    def _handle_unsubscription(self, cmd_text: str, addr: Tuple[str, int]) -> None:
        parts = cmd_text.split()
        client_port = addr[1]
        if len(parts) > 1:
            try:
                client_port = int(parts[1])
            except ValueError:
                pass
        target = parts[2].strip() if len(parts) > 2 else "ALL"
        sub_key = (addr[0], client_port)

        if target.upper() == "ALL":
            self.subscribers.pop(sub_key, None)
        else:
            sub = self.subscribers.get(sub_key)
            if sub and sub["targets"] is not None:
                try:
                    u_idx = int(target)
                    sub["targets"].discard(u_idx)
                    if not sub["targets"]:
                        self.subscribers.pop(sub_key, None)
                except ValueError:
                    pass

    def _register_subscriber(
        self,
        ip: str,
        port: int,
        targets: Optional[Set[int]],
        ttl: float,
        now: float,
    ) -> None:
        sub_key = (ip, port)
        existing = self.subscribers.get(sub_key)
        if existing is not None:
            existing["expire_time"] = now + ttl
            existing["targets"] = set(targets) if targets is not None else None
        else:
            self.subscribers[sub_key] = {
                "targets": set(targets) if targets is not None else None,
                "expire_time": now + ttl,
            }

    def _handle_mavlink(self, data: bytes, addr: Tuple[str, int]) -> None:
        messages = self._generic_codec.parse_bytes(data)
        for msg in messages:
            if isinstance(msg, MavManualControlMsg):
                target_idx = msg.uav_id_num
                if 0 <= target_idx < len(self.endpoints):
                    self.endpoints[target_idx].latest_manual_cmd = msg

            elif isinstance(msg, MavFlightPlanHeaderMsg):
                target_idx = msg.uav_id_num
                if 0 <= target_idx < len(self.endpoints):
                    ep = self.endpoints[target_idx]
                    ep.last_fp_sender_addr = addr
                    self._plan_to_uav[msg.plan_id] = target_idx
                    self._plan_sender_addr[msg.plan_id] = addr
                    pending = _PendingFlightPlan(msg.plan_id, msg.num_waypoints)
                    pending.apply_header(msg)
                    ep._pending_plans[msg.plan_id] = pending

            elif isinstance(msg, MavFlightPlanWaypointMsg):
                target_idx = self._plan_to_uav.get(msg.plan_id)
                if target_idx is None:
                    # Fallback al sysid de cabecera si está disponible
                    target_idx = getattr(msg, "sysid", 1) - 1

                if 0 <= target_idx < len(self.endpoints):
                    ep = self.endpoints[target_idx]
                    pending = ep._pending_plans.get(msg.plan_id)
                    if pending is None:
                        pending = _PendingFlightPlan(msg.plan_id, msg.num_waypoints)
                        ep._pending_plans[msg.plan_id] = pending

                    if pending.add_waypoint(msg):
                        completed_fp = pending.build()
                        ep.active_flight_plan = completed_fp
                        ep.latest_completed_fp = completed_fp
                        del ep._pending_plans[msg.plan_id]
                        sender = self._plan_sender_addr.get(msg.plan_id, addr)
                        self._send_ack(
                            target_addr=sender,
                            plan_id=msg.plan_id,
                            received=len(completed_fp.waypoints),
                            total=len(completed_fp.waypoints),
                            result=NavSimFPAckResult.ACCEPTED,
                            sysid=ep.sysid,
                        )

    def _send_ack(
        self,
        target_addr: Tuple[str, int],
        plan_id: str,
        received: int,
        total: int,
        result: NavSimFPAckResult,
        sysid: int = 1,
    ) -> None:
        if self.sock is None:
            return
        codec = MAVLinkV2Codec(sysid=sysid, compid=1)
        frame = codec.pack_ack(
            MavFlightPlanAckMsg(
                plan_id=plan_id,
                received_waypoints=received,
                num_waypoints=total,
                result=int(result),
            )
        )
        try:
            self.sock.sendto(frame, target_addr)
        except OSError:
            pass

    def poll_new_flight_plans(self) -> Dict[int, FlightPlanData]:
        self.drain_socket()
        updated: Dict[int, FlightPlanData] = {}
        for idx, ep in enumerate(self.endpoints):
            fp = ep.latest_completed_fp
            if fp is not None:
                updated[idx] = fp
                ep.latest_completed_fp = None
        return updated

    def poll_manual_controls(self) -> Dict[int, MavManualControlMsg]:
        self.drain_socket()
        cmds: Dict[int, MavManualControlMsg] = {}
        for idx, ep in enumerate(self.endpoints):
            if ep.latest_manual_cmd is not None:
                cmds[idx] = ep.latest_manual_cmd
                ep.latest_manual_cmd = None
        return cmds

    def maybe_send_telemetry(
        self,
        sim_time: float,
        uav_states: np.ndarray,
        plan_ids: List[str],
        current_wps: np.ndarray,
        positions: np.ndarray,
        eulers: np.ndarray,
        orientations: np.ndarray,
        world_lin_vels: np.ndarray,
        local_ang_vels: np.ndarray,
        rotors_vel: np.ndarray,
        total_wps: Optional[np.ndarray] = None,
        wp_labels: Optional[List[str]] = None,
        fp_init_times: Optional[np.ndarray] = None,
        fp_finish_times: Optional[np.ndarray] = None,
        wp_times: Optional[np.ndarray] = None,
        desired_positions: Optional[np.ndarray] = None,
        desired_velocities: Optional[np.ndarray] = None,
        pos_errors: Optional[np.ndarray] = None,
        vel_errors: Optional[np.ndarray] = None,
    ) -> None:
        last_time = getattr(self, "_last_telemetry_time", -1.0)
        if last_time >= 0.0 and (sim_time - last_time) < self.telemetry_period:
            return

        if self.sock is None:
            return

        now = time.time()
        # 1. Purgar suscriptores expirados (TTL)
        expired = [k for k, v in self.subscribers.items() if v["expire_time"] < now]
        for k in expired:
            del self.subscribers[k]

        # 2. Si no hay suscriptores vivos: cero trabajo de empaquetado ni envíos
        if not self.subscribers:
            self._last_telemetry_time = sim_time
            return

        self._last_telemetry_time = sim_time

        # 3. Emitir telemetría bajo demanda UAV por UAV
        for idx, ep in enumerate(self.endpoints):
            active_recipients = [
                k for k, v in self.subscribers.items()
                if v["targets"] is None or idx in v["targets"]
            ]
            if not active_recipients:
                continue

            raw_bytes = ep.build_telemetry_frames(
                sim_time=sim_time,
                uav_state=int(uav_states[idx]),
                plan_id=plan_ids[idx],
                current_wp=int(current_wps[idx]),
                pos=positions[idx],
                euler=eulers[idx],
                quat=orientations[idx],
                lin_vel=world_lin_vels[idx],
                ang_vel=local_ang_vels[idx],
                rotor_speeds=rotors_vel[idx],
                georeference=self.georeference,
                total_wps=int(total_wps[idx]) if total_wps is not None else 0,
                wp_label=str(wp_labels[idx]) if wp_labels is not None else "",
                fp_init_time=float(fp_init_times[idx]) if fp_init_times is not None else 0.0,
                fp_finish_time=float(fp_finish_times[idx]) if fp_finish_times is not None else 0.0,
                wp_time=float(wp_times[idx]) if wp_times is not None else 0.0,
                desired_pos=desired_positions[idx] if desired_positions is not None else None,
                desired_vel=desired_velocities[idx] if desired_velocities is not None else None,
                pos_error=pos_errors[idx] if pos_errors is not None else None,
                vel_error=vel_errors[idx] if vel_errors is not None else None,
                time_reference=self.time_reference,
            )

            for dest in active_recipients:
                try:
                    self.sock.sendto(raw_bytes, dest)
                except OSError:
                    pass
