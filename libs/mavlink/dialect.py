from dataclasses import dataclass, field
from enum import IntEnum
import math
import struct
from typing import Dict, List, Optional, Sequence, Tuple, Union
import numpy as np


# ==============================================================================
# Constantes y Enumerados del Dialecto MAVLink v2 NavSim
# ==============================================================================
MAVLINK_STX_V2 = 0xFD
MAVLINK_HEADER_LEN = 10  # STX(1)+LEN(1)+INC(1)+CMP(1)+SEQ(1)+SYS(1)+COMP(1)+MSGID(3)

MSG_ID_FLIGHT_PLAN_HEADER = 12000
MSG_ID_FLIGHT_PLAN_WAYPOINT = 12001
MSG_ID_FLIGHT_PLAN_ACK = 12002
MSG_ID_FLIGHT_PLAN_STATUS = 12003
MSG_ID_TELEMETRY = MSG_ID_FLIGHT_PLAN_STATUS
MSG_ID_MANUAL_CONTROL = 12004
MSG_ID_OPEN_DRONE_ID_BASIC_ID = 12900
MSG_ID_OPEN_DRONE_ID_LOCATION = 12901
MSG_ID_OPEN_DRONE_ID_SELF_ID = 12903
MSG_ID_OPEN_DRONE_ID_OPERATOR_ID = 12905


class NavSimUAVState(IntEnum):
    IDLE = 0
    BUSY = 1
    DEAD = 2
    MANUAL = 3


class NavSimFPAckResult(IntEnum):
    ACCEPTED = 0
    WAYPOINT_RECEIVED = 1
    INVALID = 2


# ==============================================================================
# Enumerados Estándar ASTM F3411 / EUROCAE ED-282 (OpenDroneID / Remote ID)
# ==============================================================================
class MavOdidStatus(IntEnum):
    UNDECLARED = 0
    GROUND = 1
    AIRBORNE = 2
    EMERGENCY = 3
    REMOTE_ID_SYSTEM_FAILURE = 4


class MavOdidHeightRef(IntEnum):
    OVER_TAKEOFF = 0
    OVER_GROUND = 1


class MavOdidHorAcc(IntEnum):
    UNKNOWN = 0
    ACC_10NM = 1
    ACC_4NM = 2
    ACC_2NM = 3
    ACC_1NM = 4
    ACC_0_5NM = 5
    ACC_0_3NM = 6
    ACC_0_1NM = 7
    ACC_0_05NM = 8
    ACC_30M = 9
    ACC_10M = 10
    ACC_3M = 11
    ACC_1M = 12


class MavOdidVerAcc(IntEnum):
    UNKNOWN = 0
    ACC_150M = 1
    ACC_45M = 2
    ACC_25M = 3
    ACC_10M = 4
    ACC_3M = 5
    ACC_1M = 6


class MavOdidSpeedAcc(IntEnum):
    UNKNOWN = 0
    ACC_10MS = 1
    ACC_3MS = 2
    ACC_1MS = 3
    ACC_0_3MS = 4


class MavOdidTimeAcc(IntEnum):
    UNKNOWN = 0
    ACC_0_1S = 1
    ACC_0_2S = 2
    ACC_0_3S = 3
    ACC_0_4S = 4
    ACC_0_5S = 5
    ACC_0_6S = 6
    ACC_0_7S = 7
    ACC_0_8S = 8
    ACC_0_9S = 9
    ACC_1_0S = 10


# ==============================================================================
# Estructuras de Datos equivalentes a FlightPlan y Waypoint
# ==============================================================================
@dataclass
class WaypointData:
    """
    Estructura de un tramo polinómico de Taylor de orden 5, idéntica a Waypoint.
    """
    label: str = ""
    t: float = 0.0
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    acel: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    jerk: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    snap: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    crakle: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    heading: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=np.float64))
    fly_over: bool = False

    def __post_init__(self):
        self.t = float(self.t)
        self.pos = np.asarray(self.pos, dtype=np.float64).reshape(3)
        self.vel = np.asarray(self.vel, dtype=np.float64).reshape(3)
        self.acel = np.asarray(self.acel, dtype=np.float64).reshape(3)
        self.jerk = np.asarray(self.jerk, dtype=np.float64).reshape(3)
        self.snap = np.asarray(self.snap, dtype=np.float64).reshape(3)
        self.crakle = np.asarray(self.crakle, dtype=np.float64).reshape(3)
        if self.heading is None:
            self.heading = np.zeros(2, dtype=np.float64)
        else:
            self.heading = np.asarray(self.heading, dtype=np.float64).reshape(2)
        self.fly_over = bool(self.fly_over)


@dataclass
class FlightPlanData:
    """
    Estructura completa de un Plan de Vuelo con sus tramos polinómicos de Taylor.
    Espejo directo de la clase FlightPlan de Python.
    """
    id: str = "0"
    priority: int = 0
    radius: float = 1.0
    max_var_lin_vel: float = 5.0
    max_var_ang_vel: float = 1.0
    target_yaw: Optional[float] = None
    waypoints: List[WaypointData] = field(default_factory=list)

    def init_time(self) -> float:
        return self.waypoints[0].t if self.waypoints else 0.0

    def finish_time(self) -> float:
        return self.waypoints[-1].t if self.waypoints else 0.0

    def to_arrays(self) -> Tuple[
        np.ndarray, np.ndarray, np.ndarray, np.ndarray,
        np.ndarray, np.ndarray, np.ndarray, np.ndarray
    ]:
        """
        Convierte la lista de waypoints en 8 arrays NumPy contiguos listos para
        insertarse en las matrices 2D/3D del controlador vectorizado:
          (times, positions, velocities, accelerations, jerks, snaps, crackles, headings)
        """
        if not self.waypoints:
            empty_1d = np.empty((0,), dtype=np.float64)
            empty_3d = np.empty((0, 3), dtype=np.float64)
            empty_2d = np.empty((0, 2), dtype=np.float64)
            return empty_1d, empty_3d, empty_3d, empty_3d, empty_3d, empty_3d, empty_3d, empty_2d

        times = np.array([wp.t for wp in self.waypoints], dtype=np.float64)
        positions = np.stack([wp.pos for wp in self.waypoints], axis=0)
        velocities = np.stack([wp.vel for wp in self.waypoints], axis=0)
        accelerations = np.stack([wp.acel for wp in self.waypoints], axis=0)
        jerks = np.stack([wp.jerk for wp in self.waypoints], axis=0)
        snaps = np.stack([wp.snap for wp in self.waypoints], axis=0)
        crackles = np.stack([wp.crakle for wp in self.waypoints], axis=0)
        headings = np.stack([wp.heading for wp in self.waypoints], axis=0)
        return times, positions, velocities, accelerations, jerks, snaps, crackles, headings

    def to_dict(self) -> dict:
        return {
            "id": str(self.id),
            "priority": int(self.priority),
            "radius": float(self.radius),
            "max_var_lin_vel": float(self.max_var_lin_vel),
            "max_var_ang_vel": float(self.max_var_ang_vel),
            "target_yaw": self.target_yaw,
            "waypoints": [
                {
                    "label": wp.label,
                    "t": wp.t,
                    "pos": wp.pos.tolist(),
                    "vel": wp.vel.tolist(),
                    "acel": wp.acel.tolist(),
                    "jerk": wp.jerk.tolist(),
                    "snap": wp.snap.tolist(),
                    "crakle": wp.crakle.tolist(),
                    "fly_over": wp.fly_over,
                    "heading": wp.heading.tolist(),
                }
                for wp in self.waypoints
            ],
        }

    @classmethod
    def from_dict(cls, data: dict) -> "FlightPlanData":
        wps = [
            WaypointData(
                label=str(w.get("label", w.get("id", ""))),
                t=float(w.get("t", w.get("time", 0.0))),
                pos=w.get("pos", [0.0, 0.0, 0.0]),
                vel=w.get("vel", [0.0, 0.0, 0.0]),
                acel=w.get("acel", [0.0, 0.0, 0.0]),
                jerk=w.get("jerk", [0.0, 0.0, 0.0]),
                snap=w.get("snap", [0.0, 0.0, 0.0]),
                crakle=w.get("crakle", [0.0, 0.0, 0.0]),
                heading=w.get("heading", [0.0, 0.0]) or [0.0, 0.0],
                fly_over=bool(w.get("fly_over", False)),
            )
            for w in data.get("waypoints", [])
        ]
        return cls(
            id=str(data.get("id", "0")),
            priority=int(data.get("priority", 0)),
            radius=float(data.get("radius", 1.0)),
            max_var_lin_vel=float(data.get("max_var_lin_vel", 5.0)),
            max_var_ang_vel=float(data.get("max_var_ang_vel", 1.0)),
            target_yaw=data.get("target_yaw", None),
            waypoints=wps,
        )


# ==============================================================================
# Mensajes MAVLink v2 Tipados
# ==============================================================================
@dataclass
class MavFlightPlanHeaderMsg:
    plan_id: str
    priority: int
    radius: float
    max_var_lin_vel: float
    max_var_ang_vel: float
    target_yaw: float  # math.nan si es None
    num_waypoints: int
    uav_id_num: int = 0  # UAV destinatario (0..65535)
    msg_id: int = MSG_ID_FLIGHT_PLAN_HEADER


@dataclass
class MavFlightPlanWaypointMsg:
    plan_id: str
    seq: int
    num_waypoints: int
    label: str
    t: float
    pos: Tuple[float, float, float]
    vel: Tuple[float, float, float]
    acel: Tuple[float, float, float]
    jerk: Tuple[float, float, float]
    snap: Tuple[float, float, float]
    crakle: Tuple[float, float, float]
    heading: Tuple[float, float]
    fly_over: int
    msg_id: int = MSG_ID_FLIGHT_PLAN_WAYPOINT

    def to_waypoint_data(self) -> WaypointData:
        return WaypointData(
            label=self.label,
            t=self.t,
            pos=np.array(self.pos, dtype=np.float64),
            vel=np.array(self.vel, dtype=np.float64),
            acel=np.array(self.acel, dtype=np.float64),
            jerk=np.array(self.jerk, dtype=np.float64),
            snap=np.array(self.snap, dtype=np.float64),
            crakle=np.array(self.crakle, dtype=np.float64),
            heading=np.array(self.heading, dtype=np.float64),
            fly_over=bool(self.fly_over),
        )


@dataclass
class MavFlightPlanAckMsg:
    plan_id: str
    received_waypoints: int
    num_waypoints: int
    result: int
    msg_id: int = MSG_ID_FLIGHT_PLAN_ACK


@dataclass
class MavFlightPlanStatusMsg:
    """
    Mensaje NAVSIM_FLIGHT_PLAN_STATUS (ID: 12003).
    Telemetría de retorno del UAV indicando el estado de ejecución de su ruta 4D
    (polinomio de Taylor de orden 5):
      - time_unix_usec: instante UTC actual de simulación [µs desde 1970-01-01 00:00:00 UTC]
      - sim_time: tiempo actual de simulación [s]
      - uav_state: estado operativo (IDLE=0, BUSY=1, DEAD=2, MANUAL=3)
      - plan_id: identificador del FlightPlan activo (vacío si no hay plan)
      - current_wp: índice del waypoint actual (0..total_wps-1, o -1 si no activo)
      - total_wps: número total de waypoints del plan activo
      - wp_label: etiqueta identificativa del waypoint actual (Waypoint.label)
      - fp_init_time: instante de inicio del FlightPlan activo [s]
      - fp_finish_time: instante de finalización del FlightPlan activo [s]
      - wp_time: instante de referencia t_k del tramo polinómico activo [s]
      - desired_pos: posición 3D deseada evaluada por el polinomio de Taylor [m]
      - desired_vel: velocidad 3D deseada evaluada por el polinomio de Taylor [m/s]
      - pos_error: error instantáneo 3D de seguimiento de posición [m]
      - vel_error: error instantáneo 3D de seguimiento de velocidad [m/s]
    """
    time_unix_usec: int = 0
    sim_time: float = 0.0
    uav_state: int = int(NavSimUAVState.IDLE)
    plan_id: str = ""
    current_wp: int = -1
    total_wps: int = 0
    wp_label: str = ""
    fp_init_time: float = 0.0
    fp_finish_time: float = 0.0
    wp_time: float = 0.0
    desired_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    desired_vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    pos_error: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    vel_error: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    msg_id: int = MSG_ID_FLIGHT_PLAN_STATUS


# Alias para compatibilidad
MavTelemetryMsg = MavFlightPlanStatusMsg


@dataclass
class MavOdidBasicIdMsg:
    """
    Mensaje OPEN_DRONE_ID_BASIC_ID (ID: 12900).
    Identificación del UAS según estándares ASTM F3411 / EUROCAE ED-282 (Remote ID).
    """
    target_system: int = 0
    target_component: int = 0
    id_or_mac: bytes = field(default_factory=lambda: b"\x00" * 20)
    id_type: int = 1  # 1 = Número de serie (ANSI/CTA-2063-A)
    ua_type: int = 2  # 2 = Aeroplano / Multirrotor
    uas_id: str = ""
    msg_id: int = MSG_ID_OPEN_DRONE_ID_BASIC_ID


@dataclass
class MavOdidLocationMsg:
    """
    Mensaje OPEN_DRONE_ID_LOCATION (ID: 12901).
    Cinemática estándar 100% geodésica WGS-84 y vector polar según ASTM F3411 / EUROCAE ED-282:
      - latitude / longitude en degE7 (grados * 1e7)
      - altitude_geodetic en metros (HAE sobre elipsoide WGS-84)
      - altitude_barometric en metros
      - height en metros (AGL sobre el suelo o punto de despegue)
      - speed_horizontal en cm/s (módulo >= 0)
      - speed_vertical en cm/s (+ arriba, - abajo)
      - direction en centígrados 0..35999 horario desde Norte Verdadero (0=N, 9000=E)
      - status: estado de vuelo (0=Undeclared, 1=Ground, 2=Airborne, 3=Emergency, 4=Failure)
    """
    latitude: int = 0
    longitude: int = 0
    altitude_barometric: float = -1000.0
    altitude_geodetic: float = -1000.0
    height: float = -1000.0
    timestamp: float = 0xFFFF
    direction: int = 36100
    speed_horizontal: int = 0
    speed_vertical: int = 0
    target_system: int = 0
    target_component: int = 0
    id_or_mac: bytes = field(default_factory=lambda: b"\x00" * 20)
    status: int = int(MavOdidStatus.GROUND)
    height_reference: int = int(MavOdidHeightRef.OVER_GROUND)
    horizontal_accuracy: int = int(MavOdidHorAcc.ACC_1M)
    vertical_accuracy: int = int(MavOdidVerAcc.ACC_1M)
    barometer_accuracy: int = int(MavOdidVerAcc.ACC_1M)
    speed_accuracy: int = int(MavOdidSpeedAcc.ACC_0_3MS)
    timestamp_accuracy: int = int(MavOdidTimeAcc.ACC_0_1S)
    msg_id: int = MSG_ID_OPEN_DRONE_ID_LOCATION

    @property
    def lat_deg(self) -> float:
        return self.latitude * 1e-7

    @property
    def lon_deg(self) -> float:
        return self.longitude * 1e-7

    @property
    def direction_deg(self) -> float:
        return self.direction * 0.01

    @property
    def speed_h_ms(self) -> float:
        return self.speed_horizontal * 0.01

    @property
    def speed_v_ms(self) -> float:
        return self.speed_vertical * 0.01


@dataclass
class MavOdidOperatorIdMsg:
    """
    Mensaje OPEN_DRONE_ID_OPERATOR_ID (ID: 12905).
    Registro oficial de operador civil (CAA/AESA/EASA/FAA) según ASTM F3411 / ED-282.
    """
    target_system: int = 0
    target_component: int = 0
    id_or_mac: bytes = field(default_factory=lambda: b"\x00" * 20)
    operator_id_type: int = 0  # 0 = CAA_OPERATOR_ID
    operator_id: str = ""
    msg_id: int = MSG_ID_OPEN_DRONE_ID_OPERATOR_ID


@dataclass
class MavOdidSelfIdMsg:
    """
    Mensaje OPEN_DRONE_ID_SELF_ID (ID: 12903).
    Descripción de misión o propósito de vuelo según ASTM F3411 / ED-282 (máx 23 chars).
    """
    target_system: int = 0
    target_component: int = 0
    id_or_mac: bytes = field(default_factory=lambda: b"\x00" * 20)
    description_type: int = 0  # 0 = TEXT
    description: str = ""
    msg_id: int = MSG_ID_OPEN_DRONE_ID_SELF_ID


@dataclass
class MavManualControlMsg:
    """
    Mensaje NAVSIM_MANUAL_CONTROL (ID: 12004).
    Comando directo de velocidades en ejes cuerpo y rotación yaw para control manual.
      - vel_x: velocidad lineal en X (adelante/atrás en cuerpo) [m/s]
      - vel_y: velocidad lineal en Y (derecha/izquierda en cuerpo) [m/s]
      - vel_z: velocidad lineal en Z (arriba/abajo) [m/s]
      - rot_z: velocidad angular de guiñada (yaw rate) [rad/s]
      - buttons: máscara de botones/flags (p. ej. bit 0 = armado/on)
      - uav_id_num: índice o ID numérico del UAV objetivo (0-indexed)
      - mode: modo deseado (0=DISARM/IDLE, 1=MANUAL, 2=AUTO, 3=HOVER)
    """
    vel_x: float = 0.0
    vel_y: float = 0.0
    vel_z: float = 0.0
    rot_z: float = 0.0
    buttons: int = 0
    uav_id_num: int = 0
    mode: int = 1
    msg_id: int = MSG_ID_MANUAL_CONTROL


MavMessage = Union[
    MavFlightPlanHeaderMsg,
    MavFlightPlanWaypointMsg,
    MavFlightPlanAckMsg,
    MavFlightPlanStatusMsg,
    MavOdidBasicIdMsg,
    MavOdidLocationMsg,
    MavOdidOperatorIdMsg,
    MavOdidSelfIdMsg,
    MavManualControlMsg,
]


# ==============================================================================
# Codificador / Decodificador Binario MAVLink v2 (Zero-Dependency)
# ==============================================================================
# En MAVLink v2 los campos del payload se ordenan de mayor a menor tamaño de tipo
# primitivo (4 bytes -> 2 bytes -> 1 byte / char[]) para evitar alineamiento roto.
_STRUCT_HEADER = struct.Struct("<iffffHH32s")                     # 56 bytes
_STRUCT_WAYPOINT = struct.Struct("<f3f3f3f3f3f3f2fHH32s16sB")     # 137 bytes
_STRUCT_ACK = struct.Struct("<HH32sB")                            # 37 bytes
_STRUCT_FLIGHT_PLAN_STATUS = struct.Struct("<Qffff3f3f3f3fhH32s16sB")  # 125 bytes
_STRUCT_TELEMETRY = _STRUCT_FLIGHT_PLAN_STATUS
_STRUCT_ODID_BASIC_ID = struct.Struct("<BB20sBB20s")              # 44 bytes
_STRUCT_ODID_LOCATION = struct.Struct("<iiffffHHhBB20s7B")        # 59 bytes
_STRUCT_ODID_OPERATOR_ID = struct.Struct("<BB20sB20s")            # 43 bytes
_STRUCT_ODID_SELF_ID = struct.Struct("<BB20sB23s")                # 46 bytes
_STRUCT_MANUAL_CONTROL = struct.Struct("<ffffIHB")                # 23 bytes

_CRC_EXTRA = {
    MSG_ID_FLIGHT_PLAN_HEADER: 173,
    MSG_ID_FLIGHT_PLAN_WAYPOINT: 89,
    MSG_ID_FLIGHT_PLAN_ACK: 214,
    MSG_ID_FLIGHT_PLAN_STATUS: 221,
    MSG_ID_OPEN_DRONE_ID_BASIC_ID: 114,
    MSG_ID_OPEN_DRONE_ID_LOCATION: 16,
    MSG_ID_OPEN_DRONE_ID_SELF_ID: 249,
    MSG_ID_OPEN_DRONE_ID_OPERATOR_ID: 49,
    MSG_ID_MANUAL_CONTROL: 55,
}


def _x25_crc(buffer: bytes, crc_extra: int) -> int:
    """Calcula el CRC-16/MCRF4XX estándar de MAVLink v2 incluyendo el byte CRC_EXTRA."""
    crc = 0xFFFF
    for b in buffer:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    # Añadir CRC_EXTRA
    tmp = crc_extra ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def _encode_str(value: str, length: int) -> bytes:
    raw = str(value).encode("utf-8")[:length]
    return raw.ljust(length, b"\x00")


def _decode_str(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")


class MAVLinkV2Codec:
    """
    Empaquetador y parser de tramas MAVLink v2 para el dialecto NavSim.
    Soporta fragmentación / múltiples paquetes en un mismo datagrama UDP.
    """

    def __init__(self, sysid: int = 1, compid: int = 1):
        self.sysid = sysid & 0xFF
        self.compid = compid & 0xFF
        self._seq = 0
        self._rx_buf = bytearray()

    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return s

    def _wrap_frame(self, msg_id: int, payload: bytes) -> bytes:
        header_without_stx = struct.pack(
            "<BBBBBBB",
            len(payload),
            0,  # incompat_flags
            0,  # compat_flags
            self._next_seq(),
            self.sysid,
            self.compid,
            msg_id & 0xFF,
        ) + struct.pack("<H", (msg_id >> 8) & 0xFFFF)
        crc = _x25_crc(header_without_stx + payload, _CRC_EXTRA[msg_id])
        return bytes([MAVLINK_STX_V2]) + header_without_stx + payload + struct.pack("<H", crc)

    # ------------------------------------------------------------------
    # Empaquetado de mensajes (Encode)
    # ------------------------------------------------------------------
    def pack_flight_plan_header(self, msg: MavFlightPlanHeaderMsg) -> bytes:
        target_yaw = float("nan") if msg.target_yaw is None else float(msg.target_yaw)
        payload = _STRUCT_HEADER.pack(
            int(msg.priority),
            float(msg.radius),
            float(msg.max_var_lin_vel),
            float(msg.max_var_ang_vel),
            target_yaw,
            int(msg.num_waypoints) & 0xFFFF,
            int(msg.uav_id_num) & 0xFFFF,
            _encode_str(msg.plan_id, 32),
        )
        return self._wrap_frame(MSG_ID_FLIGHT_PLAN_HEADER, payload)

    def pack_flight_plan_waypoint(self, msg: MavFlightPlanWaypointMsg) -> bytes:
        payload = _STRUCT_WAYPOINT.pack(
            float(msg.t),
            *map(float, msg.pos),
            *map(float, msg.vel),
            *map(float, msg.acel),
            *map(float, msg.jerk),
            *map(float, msg.snap),
            *map(float, msg.crakle),
            *map(float, msg.heading),
            int(msg.seq),
            int(msg.num_waypoints),
            _encode_str(msg.plan_id, 32),
            _encode_str(msg.label, 16),
            int(msg.fly_over),
        )
        return self._wrap_frame(MSG_ID_FLIGHT_PLAN_WAYPOINT, payload)

    def pack_flight_plan(self, fp: FlightPlanData, uav_id_num: int = 0) -> List[bytes]:
        """
        Convierte un objeto FlightPlanData completo en la secuencia de paquetes
        MAVLink v2: 1 x NAVSIM_FLIGHT_PLAN_HEADER + N x NAVSIM_FLIGHT_PLAN_WAYPOINT.
        """
        num_wps = len(fp.waypoints)
        packets = [
            self.pack_flight_plan_header(
                MavFlightPlanHeaderMsg(
                    plan_id=str(fp.id),
                    priority=int(fp.priority),
                    radius=float(fp.radius),
                    max_var_lin_vel=float(fp.max_var_lin_vel),
                    max_var_ang_vel=float(fp.max_var_ang_vel),
                    target_yaw=float("nan") if fp.target_yaw is None else float(fp.target_yaw),
                    num_waypoints=num_wps,
                    uav_id_num=int(uav_id_num),
                )
            )
        ]
        for idx, wp in enumerate(fp.waypoints):
            packets.append(
                self.pack_flight_plan_waypoint(
                    MavFlightPlanWaypointMsg(
                        plan_id=str(fp.id),
                        seq=idx,
                        num_waypoints=num_wps,
                        label=wp.label,
                        t=float(wp.t),
                        pos=tuple(wp.pos.tolist()),
                        vel=tuple(wp.vel.tolist()),
                        acel=tuple(wp.acel.tolist()),
                        jerk=tuple(wp.jerk.tolist()),
                        snap=tuple(wp.snap.tolist()),
                        crakle=tuple(wp.crakle.tolist()),
                        heading=tuple(wp.heading.tolist()),
                        fly_over=1 if wp.fly_over else 0,
                    )
                )
            )
        return packets

    def pack_ack(self, msg: MavFlightPlanAckMsg) -> bytes:
        payload = _STRUCT_ACK.pack(
            int(msg.received_waypoints),
            int(msg.num_waypoints),
            _encode_str(msg.plan_id, 32),
            int(msg.result),
        )
        return self._wrap_frame(MSG_ID_FLIGHT_PLAN_ACK, payload)

    def pack_flight_plan_status(self, msg: MavFlightPlanStatusMsg) -> bytes:
        """Empaqueta trama MAVLink v2 NAVSIM_FLIGHT_PLAN_STATUS (ID 12003)."""
        payload = _STRUCT_FLIGHT_PLAN_STATUS.pack(
            int(msg.time_unix_usec) & 0xFFFFFFFFFFFFFFFF,
            float(msg.sim_time),
            float(msg.fp_init_time),
            float(msg.fp_finish_time),
            float(msg.wp_time),
            *map(float, msg.desired_pos),
            *map(float, msg.desired_vel),
            *map(float, msg.pos_error),
            *map(float, msg.vel_error),
            int(msg.current_wp),
            int(msg.total_wps) & 0xFFFF,
            _encode_str(msg.plan_id, 32),
            _encode_str(msg.wp_label, 16),
            int(msg.uav_state) & 0xFF,
        )
        return self._wrap_frame(MSG_ID_FLIGHT_PLAN_STATUS, payload)

    pack_telemetry = pack_flight_plan_status

    def pack_odid_basic_id(self, msg: MavOdidBasicIdMsg) -> bytes:
        """Empaqueta trama MAVLink v2 OPEN_DRONE_ID_BASIC_ID (ID 12900)."""
        id_or_mac = msg.id_or_mac if isinstance(msg.id_or_mac, bytes) else str(msg.id_or_mac).encode("utf-8")
        id_or_mac = id_or_mac[:20].ljust(20, b"\x00")
        uas_id = _encode_str(msg.uas_id, 20)
        payload = _STRUCT_ODID_BASIC_ID.pack(
            int(msg.target_system) & 0xFF,
            int(msg.target_component) & 0xFF,
            id_or_mac,
            int(msg.id_type) & 0xFF,
            int(msg.ua_type) & 0xFF,
            uas_id,
        )
        return self._wrap_frame(MSG_ID_OPEN_DRONE_ID_BASIC_ID, payload)

    def pack_odid_location(self, msg: MavOdidLocationMsg) -> bytes:
        """Empaqueta trama MAVLink v2 OPEN_DRONE_ID_LOCATION (ID 12901)."""
        id_or_mac = msg.id_or_mac if isinstance(msg.id_or_mac, bytes) else str(msg.id_or_mac).encode("utf-8")
        id_or_mac = id_or_mac[:20].ljust(20, b"\x00")
        payload = _STRUCT_ODID_LOCATION.pack(
            int(msg.latitude),
            int(msg.longitude),
            float(msg.altitude_barometric),
            float(msg.altitude_geodetic),
            float(msg.height),
            float(msg.timestamp),
            int(msg.direction) & 0xFFFF,
            int(msg.speed_horizontal) & 0xFFFF,
            int(msg.speed_vertical),
            int(msg.target_system) & 0xFF,
            int(msg.target_component) & 0xFF,
            id_or_mac,
            int(msg.status) & 0xFF,
            int(msg.height_reference) & 0xFF,
            int(msg.horizontal_accuracy) & 0xFF,
            int(msg.vertical_accuracy) & 0xFF,
            int(msg.barometer_accuracy) & 0xFF,
            int(msg.speed_accuracy) & 0xFF,
            int(msg.timestamp_accuracy) & 0xFF,
        )
        return self._wrap_frame(MSG_ID_OPEN_DRONE_ID_LOCATION, payload)

    def pack_odid_operator_id(self, msg: MavOdidOperatorIdMsg) -> bytes:
        """Empaqueta trama MAVLink v2 OPEN_DRONE_ID_OPERATOR_ID (ID 12905)."""
        id_or_mac = msg.id_or_mac if isinstance(msg.id_or_mac, bytes) else str(msg.id_or_mac).encode("utf-8")
        id_or_mac = id_or_mac[:20].ljust(20, b"\x00")
        op_id = _encode_str(msg.operator_id, 20)
        payload = _STRUCT_ODID_OPERATOR_ID.pack(
            int(msg.target_system) & 0xFF,
            int(msg.target_component) & 0xFF,
            id_or_mac,
            int(msg.operator_id_type) & 0xFF,
            op_id,
        )
        return self._wrap_frame(MSG_ID_OPEN_DRONE_ID_OPERATOR_ID, payload)

    def pack_odid_self_id(self, msg: MavOdidSelfIdMsg) -> bytes:
        """Empaqueta trama MAVLink v2 OPEN_DRONE_ID_SELF_ID (ID 12903)."""
        id_or_mac = msg.id_or_mac if isinstance(msg.id_or_mac, bytes) else str(msg.id_or_mac).encode("utf-8")
        id_or_mac = id_or_mac[:20].ljust(20, b"\x00")
        desc = _encode_str(msg.description, 23)
        payload = _STRUCT_ODID_SELF_ID.pack(
            int(msg.target_system) & 0xFF,
            int(msg.target_component) & 0xFF,
            id_or_mac,
            int(msg.description_type) & 0xFF,
            desc,
        )
        return self._wrap_frame(MSG_ID_OPEN_DRONE_ID_SELF_ID, payload)

    def pack_manual_control(self, msg: MavManualControlMsg) -> bytes:
        """Empaqueta trama MAVLink v2 NAVSIM_MANUAL_CONTROL (ID 12004)."""
        payload = _STRUCT_MANUAL_CONTROL.pack(
            float(msg.vel_x),
            float(msg.vel_y),
            float(msg.vel_z),
            float(msg.rot_z),
            int(msg.buttons) & 0xFFFFFFFF,
            int(msg.uav_id_num) & 0xFFFF,
            int(msg.mode) & 0xFF,
        )
        return self._wrap_frame(MSG_ID_MANUAL_CONTROL, payload)

    # ------------------------------------------------------------------
    # Desempaquetado de mensajes (Decode)
    # ------------------------------------------------------------------
    def parse_bytes(self, data: bytes) -> List[MavMessage]:
        self._rx_buf.extend(data)
        messages: List[MavMessage] = []

        while len(self._rx_buf) >= MAVLINK_HEADER_LEN + 2:
            # Buscar byte de inicio STX (0xFD)
            stx_idx = self._rx_buf.find(MAVLINK_STX_V2)
            if stx_idx < 0:
                self._rx_buf.clear()
                break
            if stx_idx > 0:
                del self._rx_buf[:stx_idx]
            if len(self._rx_buf) < MAVLINK_HEADER_LEN + 2:
                break

            payload_len = self._rx_buf[1]
            frame_len = MAVLINK_HEADER_LEN + payload_len + 2
            if len(self._rx_buf) < frame_len:
                break

            frame = bytes(self._rx_buf[:frame_len])
            del self._rx_buf[:frame_len]

            msg_id = frame[7] | (frame[8] << 8) | (frame[9] << 16)
            if msg_id not in _CRC_EXTRA:
                continue

            expected_crc = _x25_crc(frame[1:-2], _CRC_EXTRA[msg_id])
            (received_crc,) = struct.unpack("<H", frame[-2:])
            if expected_crc != received_crc:
                continue

            payload = frame[MAVLINK_HEADER_LEN:-2]
            parsed = self._decode_payload(msg_id, payload)
            if parsed is not None:
                parsed.sysid = frame[5]
                parsed.compid = frame[6]
                messages.append(parsed)

        return messages

    def _decode_payload(self, msg_id: int, payload: bytes) -> Optional[MavMessage]:
        if msg_id == MSG_ID_FLIGHT_PLAN_HEADER and len(payload) == _STRUCT_HEADER.size:
            prio, rad, max_lin, max_ang, tyaw, nwps, uav_num, pid = _STRUCT_HEADER.unpack(payload)
            return MavFlightPlanHeaderMsg(
                plan_id=_decode_str(pid),
                priority=prio,
                radius=rad,
                max_var_lin_vel=max_lin,
                max_var_ang_vel=max_ang,
                target_yaw=None if math.isnan(tyaw) else tyaw,
                num_waypoints=nwps,
                uav_id_num=uav_num,
            )

        if msg_id == MSG_ID_FLIGHT_PLAN_WAYPOINT and len(payload) == _STRUCT_WAYPOINT.size:
            unpacked = _STRUCT_WAYPOINT.unpack(payload)
            t = unpacked[0]
            pos = unpacked[1:4]
            vel = unpacked[4:7]
            acel = unpacked[7:10]
            jerk = unpacked[10:13]
            snap = unpacked[13:16]
            crakle = unpacked[16:19]
            heading = unpacked[19:21]
            seq = unpacked[21]
            num_wps = unpacked[22]
            plan_id = _decode_str(unpacked[23])
            label = _decode_str(unpacked[24])
            fly_over = unpacked[25]
            return MavFlightPlanWaypointMsg(
                plan_id=plan_id,
                seq=seq,
                num_waypoints=num_wps,
                label=label,
                t=t,
                pos=pos,
                vel=vel,
                acel=acel,
                jerk=jerk,
                snap=snap,
                crakle=crakle,
                heading=heading,
                fly_over=fly_over,
            )

        if msg_id == MSG_ID_FLIGHT_PLAN_ACK and len(payload) == _STRUCT_ACK.size:
            rec_wps, num_wps, pid, res = _STRUCT_ACK.unpack(payload)
            return MavFlightPlanAckMsg(
                plan_id=_decode_str(pid),
                received_waypoints=rec_wps,
                num_waypoints=num_wps,
                result=res,
            )

        if msg_id == MSG_ID_FLIGHT_PLAN_STATUS and len(payload) == _STRUCT_FLIGHT_PLAN_STATUS.size:
            u = _STRUCT_FLIGHT_PLAN_STATUS.unpack(payload)
            return MavFlightPlanStatusMsg(
                time_unix_usec=u[0],
                sim_time=u[1],
                fp_init_time=u[2],
                fp_finish_time=u[3],
                wp_time=u[4],
                desired_pos=u[5:8],
                desired_vel=u[8:11],
                pos_error=u[11:14],
                vel_error=u[14:17],
                current_wp=u[17],
                total_wps=u[18],
                plan_id=_decode_str(u[19]),
                wp_label=_decode_str(u[20]),
                uav_state=u[21],
            )

        if msg_id == MSG_ID_OPEN_DRONE_ID_BASIC_ID and len(payload) == _STRUCT_ODID_BASIC_ID.size:
            tsys, tcomp, id_mac, id_t, ua_t, uas_id_raw = _STRUCT_ODID_BASIC_ID.unpack(payload)
            return MavOdidBasicIdMsg(
                target_system=tsys,
                target_component=tcomp,
                id_or_mac=id_mac,
                id_type=id_t,
                ua_type=ua_t,
                uas_id=_decode_str(uas_id_raw),
            )

        if msg_id == MSG_ID_OPEN_DRONE_ID_LOCATION and len(payload) == _STRUCT_ODID_LOCATION.size:
            (
                lat, lon, alt_baro, alt_geo, h, ts,
                direction, spd_h, spd_v,
                tsys, tcomp, id_mac,
                status, href, hacc, vacc, bacc, sacc, tacc
            ) = _STRUCT_ODID_LOCATION.unpack(payload)
            return MavOdidLocationMsg(
                latitude=lat,
                longitude=lon,
                altitude_barometric=alt_baro,
                altitude_geodetic=alt_geo,
                height=h,
                timestamp=ts,
                direction=direction,
                speed_horizontal=spd_h,
                speed_vertical=spd_v,
                target_system=tsys,
                target_component=tcomp,
                id_or_mac=id_mac,
                status=status,
                height_reference=href,
                horizontal_accuracy=hacc,
                vertical_accuracy=vacc,
                barometer_accuracy=bacc,
                speed_accuracy=sacc,
                timestamp_accuracy=tacc,
            )

        if msg_id == MSG_ID_OPEN_DRONE_ID_OPERATOR_ID and len(payload) == _STRUCT_ODID_OPERATOR_ID.size:
            tsys, tcomp, id_mac, op_type, op_id_raw = _STRUCT_ODID_OPERATOR_ID.unpack(payload)
            return MavOdidOperatorIdMsg(
                target_system=tsys,
                target_component=tcomp,
                id_or_mac=id_mac,
                operator_id_type=op_type,
                operator_id=_decode_str(op_id_raw),
            )

        if msg_id == MSG_ID_OPEN_DRONE_ID_SELF_ID and len(payload) == _STRUCT_ODID_SELF_ID.size:
            tsys, tcomp, id_mac, desc_t, desc_raw = _STRUCT_ODID_SELF_ID.unpack(payload)
            return MavOdidSelfIdMsg(
                target_system=tsys,
                target_component=tcomp,
                id_or_mac=id_mac,
                description_type=desc_t,
                description=_decode_str(desc_raw),
            )

        if msg_id == MSG_ID_MANUAL_CONTROL and len(payload) == _STRUCT_MANUAL_CONTROL.size:
            vx, vy, vz, wz, btns, uav_num, mode = _STRUCT_MANUAL_CONTROL.unpack(payload)
            return MavManualControlMsg(
                vel_x=vx,
                vel_y=vy,
                vel_z=vz,
                rot_z=wz,
                buttons=btns,
                uav_id_num=uav_num,
                mode=mode,
            )

        return None
