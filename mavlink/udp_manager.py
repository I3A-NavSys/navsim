import logging
import math
import socket
import time
from typing import Any, Dict, List, Optional, Set, Tuple
import numpy as np

from .dialect import (
    FlightPlanData,
    MAVLinkV2Codec,
    MavFlightPlanAckMsg,
    MavFlightPlanHeaderMsg,
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
    Endpoint MAVLink v2 sobre UDP para un único dron simulado.
    Escucha en `rx_port` (p. ej. 14550 + idx) y envía telemetría/ACKs al último
    remitente conocido o a `(tx_host, tx_port)`.
    """

    def __init__(
        self,
        uav_index: int,
        uav_id: str,
        rx_host: str = "0.0.0.0",
        rx_port: int = 14550,
        tx_host: str = "127.0.0.1",
        tx_port: int = 14650,
        identity: Optional[Any] = None,
    ):
        self.uav_index = uav_index
        self.uav_id = uav_id
        self.identity = identity
        self.sysid = (uav_index % 254) + 1
        self.rx_host = rx_host
        self.rx_port = rx_port
        self.tx_host = tx_host
        self.tx_port = tx_port
        # Destinos fijos de telemetría: siempre emite a (tx_host, tx_port) y a suscriptores reconocidos
        self.telemetry_destinations: Set[Tuple[str, int]] = {(tx_host, tx_port)}
        self.last_fp_sender_addr: Optional[Tuple[str, int]] = None
        self.last_cmd_addr: Optional[Tuple[str, int]] = None

        self.codec = MAVLinkV2Codec(sysid=self.sysid, compid=1)
        self.sock: Optional[socket.socket] = None
        self._pending_plans: Dict[str, _PendingFlightPlan] = {}
        self.active_flight_plan: Optional[FlightPlanData] = None
        self.latest_completed_fp: Optional[FlightPlanData] = None
        self.latest_manual_cmd: Optional[MavManualControlMsg] = None

    @property
    def remote_addr(self) -> Tuple[str, int]:
        return (self.tx_host, self.tx_port)

    @remote_addr.setter
    def remote_addr(self, addr: Tuple[str, int]) -> None:
        self.last_cmd_addr = addr

    def open(self) -> None:
        if self.sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.rx_host, self.rx_port))
        sock.setblocking(False)
        self.sock = sock

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        self._pending_plans.clear()
        self.latest_completed_fp = None
        self.latest_manual_cmd = None
        self.last_fp_sender_addr = None
        self.last_cmd_addr = None

    def drain_socket(self) -> None:
        """Lee y desempaqueta todos los datagramas pendientes en el socket UDP."""
        if self.sock is None:
            return

        while True:
            try:
                data, addr = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except ConnectionResetError:
                # En Windows, si un sendto previo recibió ICMP Port Unreachable,
                # el siguiente recvfrom lanza WSAECONNRESET (10054). Continuamos leyendo.
                continue
            except OSError:
                break

            self.last_cmd_addr = addr

            # Si es un ping de sincronización o suscripción de monitor (p. ej. scripts/telemetry_monitor)
            if data.strip().upper() == b"PING":
                self.telemetry_destinations.add(addr)
                continue

            for msg in self.codec.parse_bytes(data):
                if isinstance(msg, MavFlightPlanHeaderMsg):
                    self.last_fp_sender_addr = addr
                    pending = _PendingFlightPlan(msg.plan_id, msg.num_waypoints)
                    pending.apply_header(msg)
                    self._pending_plans[msg.plan_id] = pending

                elif isinstance(msg, MavFlightPlanWaypointMsg):
                    self.last_fp_sender_addr = addr
                    pending = self._pending_plans.get(msg.plan_id)
                    if pending is None:
                        pending = _PendingFlightPlan(msg.plan_id, msg.num_waypoints)
                        self._pending_plans[msg.plan_id] = pending

                    if pending.add_waypoint(msg):
                        completed_fp = pending.build()
                        self.active_flight_plan = completed_fp
                        self.latest_completed_fp = completed_fp
                        del self._pending_plans[msg.plan_id]
                        self._send_ack(
                            plan_id=msg.plan_id,
                            received=len(completed_fp.waypoints),
                            total=completed_fp.waypoints.__len__(),
                            result=NavSimFPAckResult.ACCEPTED,
                        )

                elif isinstance(msg, MavManualControlMsg):
                    self.latest_manual_cmd = msg

    def poll_incoming(self) -> Optional[FlightPlanData]:
        """
        Drena el socket UDP y devuelve un nuevo `FlightPlanData` si se ha completado uno.
        """
        self.drain_socket()
        fp = self.latest_completed_fp
        self.latest_completed_fp = None
        return fp

    def _send_ack(self, plan_id: str, received: int, total: int, result: NavSimFPAckResult) -> None:
        if self.sock is None:
            return
        frame = self.codec.pack_ack(
            MavFlightPlanAckMsg(
                plan_id=plan_id,
                received_waypoints=received,
                num_waypoints=total,
                result=int(result),
            )
        )
        target = self.last_fp_sender_addr or (self.tx_host, self.tx_port)
        try:
            self.sock.sendto(frame, target)
        except OSError:
            pass

    def send_stop_notification(self) -> None:
        """Emite notificación estándar de fin de simulación para limpiar receptores externos."""
        if self.sock is None:
            return
        try:
            id_bytes = _encode_str(self.uav_id, 20)
            frame_loc = self.codec.pack_odid_location(
                MavOdidLocationMsg(
                    status=int(MavOdidStatus.REMOTE_ID_SYSTEM_FAILURE),
                    speed_horizontal=0,
                    speed_vertical=0,
                    id_or_mac=id_bytes,
                )
            )
            for dest in self.telemetry_destinations:
                try:
                    self.sock.sendto(frame_loc, dest)
                except OSError:
                    pass
        except OSError:
            pass

    def send_telemetry(
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
    ) -> None:
        if self.sock is None:
            return

        # ------------------------------------------------------------------
        # 1. Transformación Cinemática Estándar ASTM F3411 / EUROCAE ED-282
        # ------------------------------------------------------------------
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

        # Dirección sobre el terreno (ASTM F3411 track angle en centígrados 0..35999 horario desde Norte)
        if v_h >= 0.1:
            track_rad = math.atan2(vx, vy)
            track_deg = (math.degrees(track_rad) + 360.0) % 360.0
            direction_cdeg = int(round(track_deg * 100.0)) % 36000
        else:
            # En reposo / estacionario, usar el rumbo (heading) de la aeronave
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

        # Timestamp: segundos transcurridos tras la hora UTC actual
        odid_timestamp = float(time.time() % 3600.0)
        id_bytes = _encode_str(self.uav_id, 20)

        # ------------------------------------------------------------------
        # 2. Empaquetar y enviar tramas MAVLink v2
        # ------------------------------------------------------------------
        # Trama 1: OPEN_DRONE_ID_LOCATION (ID 12901) - Estándar ASTM F3411 / EUROCAE ED-282
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

        # Trama 2: OPEN_DRONE_ID_BASIC_ID (ID 12900) - Identificación oficial UAS
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

        # Trama 3: OPEN_DRONE_ID_OPERATOR_ID (ID 12905) - Registro oficial CAA de operador
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

        # Trama 4: OPEN_DRONE_ID_SELF_ID (ID 12903) - Descripción de propósito de vuelo
        self_desc = getattr(self.identity, "self_id_desc", "") if self.identity else ""
        if self_desc:
            frame_self = self.codec.pack_odid_self_id(
                MavOdidSelfIdMsg(
                    target_system=0,
                    target_component=0,
                    id_or_mac=id_bytes,
                    description_type=0,
                    description=self_desc,
                )
            )
            frames_to_send.extend(frame_self)

        raw_bytes = bytes(frames_to_send)
        for dest in self.telemetry_destinations:
            try:
                # Enviamos el datagrama UDP conteniendo ÚNICAMENTE tramas estándar ASTM F3411 / EUROCAE ED-282
                self.sock.sendto(raw_bytes, dest)
            except OSError:
                pass


class FleetMavlinkManager:
    """
    Gestor central de comunicaciones MAVLink v2 para toda la flota en escena.
    Abre un puerto UDP dedicado por dron (`base_rx_port + idx`) y mantiene los
    planes de vuelo recibidos sin bloquear el bucle de física.
    """

    def __init__(
        self,
        uav_ids: List[str],
        base_rx_port: int = 14550,
        base_tx_port: int = 14650,
        telemetry_rate_hz: float = 20.0,
        georeference=None,
        identities: Optional[List[Any]] = None,
    ):
        self.logger = logging.getLogger("NavSim.MAVLink")
        self.uav_ids = list(uav_ids)
        self.identities = list(identities) if identities else []
        self.base_rx_port = base_rx_port
        self.base_tx_port = base_tx_port
        self.telemetry_period = 1.0 / max(telemetry_rate_hz, 0.1)
        if georeference is None:
            try:
                from extensions.navsim.navsim_python.georeference import WGS84Georeference
                georeference = WGS84Georeference()
            except Exception:
                georeference = None
        self.georeference = georeference
        self._last_telemetry_time: float = -1.0

        self.endpoints: List[UAVMavlinkEndpoint] = [
            UAVMavlinkEndpoint(
                uav_index=i,
                uav_id=uid,
                rx_port=base_rx_port + i,
                tx_port=base_tx_port + i,
                identity=self.identities[i] if i < len(self.identities) else None,
            )
            for i, uid in enumerate(self.uav_ids)
        ]

    def set_georeference(self, georeference) -> None:
        """Actualiza la referencia geodésica WGS-84 en tiempo real."""
        self.georeference = georeference

    def start(self) -> None:
        for ep in self.endpoints:
            ep.open()
        if self.endpoints:
            self.logger.info(
                f"MAVLink UDP activo para {len(self.endpoints)} UAVs "
                f"(puertos RX {self.endpoints[0].rx_port}..{self.endpoints[-1].rx_port})"
            )

    def stop(self) -> None:
        for ep in self.endpoints:
            ep.send_stop_notification()
            ep.close()
        self._last_telemetry_time = -1.0

    def poll_new_flight_plans(self) -> Dict[int, FlightPlanData]:
        """
        Sondea todos los sockets UDP de la flota.
        Devuelve `{uav_physics_idx: FlightPlanData}` únicamente para aquellos UAVs
        que acaban de recibir un FlightPlan nuevo en este paso.
        """
        updated: Dict[int, FlightPlanData] = {}
        for idx, ep in enumerate(self.endpoints):
            fp = ep.poll_incoming()
            if fp is not None:
                updated[idx] = fp
        return updated

    def poll_manual_controls(self) -> Dict[int, MavManualControlMsg]:
        """
        Sondea comandos de control manual pendientes recibidos en los endpoints UDP.
        Devuelve `{uav_physics_idx: MavManualControlMsg}` para aquellos UAVs con consignas manuales.
        """
        cmds: Dict[int, MavManualControlMsg] = {}
        for idx, ep in enumerate(self.endpoints):
            ep.drain_socket()
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
    ) -> None:
        last_time = getattr(self, "_last_telemetry_time", -1.0)
        if (
            last_time >= 0.0
            and (sim_time - last_time) < self.telemetry_period
        ):
            return

        self._last_telemetry_time = sim_time
        for idx, ep in enumerate(self.endpoints):
            ep.send_telemetry(
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
            )

