from typing import Dict, List, Optional
import numpy as np

from .drone_config import FleetDynamicsTensors
from mavlink.dialect import FlightPlanData, NavSimUAVState


class UAVControl:
    """
    Motor vectorizado de física, inercia y control lineal de bajo nivel para flotas
    heterogéneas de UAVs en Isaac Sim.
    """

    def __init__(
        self,
        rigid_prim_view,
        fleet_tensors: FleetDynamicsTensors,
        uav_ids: List[str],
    ):
        self.rigid_prim_view = rigid_prim_view
        self.fleet = fleet_tensors
        self.uav_ids = list(uav_ids)
        self.amount_uavs = self.fleet.count
        self.all_indices = np.arange(self.amount_uavs, dtype=np.int64)

        # Estado cinemático (actualizado en bloque desde RigidPrim)
        self.positions = np.zeros((self.amount_uavs, 3), dtype=np.float64)
        self.orientations = np.zeros((self.amount_uavs, 4), dtype=np.float64)
        self.orientations[:, 0] = 1.0  # Cuaternión identidad [w, x, y, z]
        self.eulers = np.zeros((self.amount_uavs, 3), dtype=np.float64)
        self.world_linear_vels = np.zeros((self.amount_uavs, 3), dtype=np.float64)
        self.world_angular_vels = np.zeros((self.amount_uavs, 3), dtype=np.float64)
        self.local_linear_vels = np.zeros((self.amount_uavs, 3), dtype=np.float64)
        self.local_angular_vels = np.zeros((self.amount_uavs, 3), dtype=np.float64)

        # Variables de estado del controlador lineal: u = Hs - Kx @ x - Ky @ E
        self.r = np.zeros((self.amount_uavs, 4, 1), dtype=np.float64)
        self.x = np.zeros((self.amount_uavs, 8, 1), dtype=np.float64)
        self.y = np.zeros((self.amount_uavs, 4, 1), dtype=np.float64)
        self.e = np.zeros((self.amount_uavs, 4, 1), dtype=np.float64)
        self.E = np.zeros((self.amount_uavs, 4, 1), dtype=np.float64)
        self.u = np.zeros((self.amount_uavs, 4, 1), dtype=np.float64)
        self.rotors_vel = np.zeros((self.amount_uavs, 4), dtype=np.float64)

        # Buffers de fuerzas y pares resultantes en el Centro de Masas
        self.bodies_forces = np.zeros((self.amount_uavs, 5, 3), dtype=np.float64)
        self.forces_to_apply = np.zeros((self.amount_uavs, 3), dtype=np.float64)
        self.torques_to_apply = np.zeros((self.amount_uavs, 3), dtype=np.float64)

        # Estado de planes de vuelo 4D (almacenamiento pre-alojado por evento, sin recrear a 100 Hz)
        self.uav_states = np.full(self.amount_uavs, int(NavSimUAVState.IDLE), dtype=np.int32)
        # Modos operativos 1:1 con la máquina de estados:
        # 0=DISARMED, 1=MANUAL, 2=AUTO, 3=HOVER, 4=SYSTEM_FAILURE
        self.uav_modes = np.zeros(self.amount_uavs, dtype=np.int32)
        self.active_plan_ids: List[str] = ["" for _ in range(self.amount_uavs)]
        self.current_wp_indices = np.full(self.amount_uavs, -1, dtype=np.int32)
        self.fp_num_wps = np.zeros(self.amount_uavs, dtype=np.int32)
        self.fp_wp_labels: List[List[str]] = [[] for _ in range(self.amount_uavs)]
        self.current_wp_labels: List[str] = ["" for _ in range(self.amount_uavs)]
        self.current_wp_times = np.zeros(self.amount_uavs, dtype=np.float64)

        # Referencias instantáneas y error de seguimiento del polinomio de Taylor 4D
        self.desired_positions = np.zeros((self.amount_uavs, 3), dtype=np.float64)
        self.desired_velocities = np.zeros((self.amount_uavs, 3), dtype=np.float64)
        self.pos_errors = np.zeros((self.amount_uavs, 3), dtype=np.float64)
        self.vel_errors = np.zeros((self.amount_uavs, 3), dtype=np.float64)

        self._max_wps = 1
        self.has_plan = np.zeros(self.amount_uavs, dtype=bool)
        self.fp_init_times = np.full(self.amount_uavs, np.inf, dtype=np.float64)
        self.fp_finish_times = np.full(self.amount_uavs, -np.inf, dtype=np.float64)
        self.fp_times = np.full((self.amount_uavs, self._max_wps), np.inf, dtype=np.float64)
        self.fp_positions = np.full((self.amount_uavs, self._max_wps, 3), np.inf, dtype=np.float64)
        self.fp_velocities = np.full((self.amount_uavs, self._max_wps, 3), np.inf, dtype=np.float64)
        self.fp_accelerations = np.full((self.amount_uavs, self._max_wps, 3), np.inf, dtype=np.float64)
        self.fp_jerks = np.full((self.amount_uavs, self._max_wps, 3), np.inf, dtype=np.float64)
        self.fp_snaps = np.full((self.amount_uavs, self._max_wps, 3), np.inf, dtype=np.float64)
        self.fp_crackels = np.full((self.amount_uavs, self._max_wps, 3), np.inf, dtype=np.float64)
        self.fp_headings = np.full((self.amount_uavs, self._max_wps, 2), np.inf, dtype=np.float64)

        # Estado de control manual por UAV (conmutación entre AUTO y MANUAL)
        self.manual_active = np.zeros(self.amount_uavs, dtype=bool)
        self.manual_linear_vel = np.zeros((self.amount_uavs, 3), dtype=np.float64)
        self.manual_yaw_vel = np.zeros(self.amount_uavs, dtype=np.float64)
        self.manual_cmd_times = np.full(self.amount_uavs, -np.inf, dtype=np.float64)

        # Política de finalización de rutas 4D:
        # True = Auto-Disarm al aterrizar en vertipuerto (0 RPM, DISARMED)
        # False = Mantenerse en HOVER sobre el último waypoint
        self.auto_disarm_on_finish: bool = True

    # ==========================================================================
    # Configuración de Inercia y Masa en los Prims USD / PhysX
    # ==========================================================================
    def apply_inertial_properties(self) -> None:
        """
        Aplica la masa y la matriz diagonal de inercia configuradas en `dynamics.yaml`
        directamente a cada UAV de la vista en el escenario.
        """
        if self.rigid_prim_view is None or self.amount_uavs == 0:
            return

        # 1. Intentar vía API vectorizada de RigidPrim
        try:
            self.rigid_prim_view.set_masses(self.fleet.mass.astype(np.float32))
        except Exception:
            pass

        try:
            self.rigid_prim_view.set_inertias(self.fleet.inertia.astype(np.float32))
        except Exception:
            pass

        # 2. Garantizar atributos USD en cada Prim individual
        prims = getattr(self.rigid_prim_view, "prims", None) or []
        if prims:
            from pxr import Gf
            for idx, prim in enumerate(prims):
                mass_attr = prim.GetAttribute("physics:mass")
                if mass_attr and mass_attr.IsValid():
                    mass_attr.Set(float(self.fleet.mass[idx]))
                inertia_attr = prim.GetAttribute("physics:diagonalInertia")
                if inertia_attr and inertia_attr.IsValid():
                    ix, iy, iz = self.fleet.inertia[idx]
                    inertia_attr.Set(Gf.Vec3f(float(ix), float(iy), float(iz)))

    # ==========================================================================
    # Gestión de Planes de Vuelo (Actualización solo bajo evento MAVLink)
    # ==========================================================================
    def _ensure_waypoint_capacity(self, required_wps: int) -> None:
        if required_wps <= self._max_wps:
            return
        new_max = max(required_wps, self._max_wps * 2)
        pad_len = new_max - self._max_wps

        self.fp_times = np.pad(
            self.fp_times, ((0, 0), (0, pad_len)), constant_values=np.inf
        )
        for attr in (
            "fp_positions",
            "fp_velocities",
            "fp_accelerations",
            "fp_jerks",
            "fp_snaps",
            "fp_crackels",
        ):
            arr = getattr(self, attr)
            setattr(
                self,
                attr,
                np.pad(arr, ((0, 0), (0, pad_len), (0, 0)), constant_values=np.inf),
            )
        self.fp_headings = np.pad(
            self.fp_headings, ((0, 0), (0, pad_len), (0, 0)), constant_values=np.inf
        )
        self._max_wps = new_max

    def set_flight_plan(self, uav_idx: int, fp: FlightPlanData) -> None:
        """
        Inserta o actualiza los coeficientes polinómicos de Taylor de un UAV
        en los tensores pre-alojados sin tocar el resto de la flota.
        """
        num_wps = len(fp.waypoints)
        if num_wps == 0:
            return

        self._ensure_waypoint_capacity(num_wps)
        times, pos, vel, acel, jerk, snap, crakle, heading = fp.to_arrays()

        # Limpiar fila previa con np.inf
        self.fp_times[uav_idx, :] = np.inf
        self.fp_positions[uav_idx, :, :] = np.inf
        self.fp_velocities[uav_idx, :, :] = np.inf
        self.fp_accelerations[uav_idx, :, :] = np.inf
        self.fp_jerks[uav_idx, :, :] = np.inf
        self.fp_snaps[uav_idx, :, :] = np.inf
        self.fp_crackels[uav_idx, :, :] = np.inf
        self.fp_headings[uav_idx, :, :] = np.inf

        # Copiar nuevos tramos polinómicos
        self.fp_times[uav_idx, :num_wps] = times
        self.fp_positions[uav_idx, :num_wps, :] = pos
        self.fp_velocities[uav_idx, :num_wps, :] = vel
        self.fp_accelerations[uav_idx, :num_wps, :] = acel
        self.fp_jerks[uav_idx, :num_wps, :] = jerk
        self.fp_snaps[uav_idx, :num_wps, :] = snap
        self.fp_crackels[uav_idx, :num_wps, :] = crakle
        self.fp_headings[uav_idx, :num_wps, :] = heading

        self.fp_init_times[uav_idx] = float(times[0])
        self.fp_finish_times[uav_idx] = float(times[-1])
        self.has_plan[uav_idx] = True
        self.uav_modes[uav_idx] = 2  # AUTO
        self.active_plan_ids[uav_idx] = str(fp.id)
        self.fp_num_wps[uav_idx] = num_wps
        self.fp_wp_labels[uav_idx] = [str(wp.label) for wp in fp.waypoints]
        self.current_wp_indices[uav_idx] = 0
        self.current_wp_labels[uav_idx] = self.fp_wp_labels[uav_idx][0]
        self.current_wp_times[uav_idx] = float(times[0])
        self.E[uav_idx, :, 0] = 0.0

        # Al recibir una ruta nueva pasa a modo AUTO
        self.manual_active[uav_idx] = False
        self.uav_states[uav_idx] = int(NavSimUAVState.BUSY)

    def _clear_fp_telemetry_state(self, uav_idx: int) -> None:
        self.active_plan_ids[uav_idx] = ""
        self.current_wp_indices[uav_idx] = -1
        self.fp_num_wps[uav_idx] = 0
        self.fp_wp_labels[uav_idx] = []
        self.current_wp_labels[uav_idx] = ""
        self.current_wp_times[uav_idx] = 0.0
        self.desired_positions[uav_idx, :] = 0.0
        self.desired_velocities[uav_idx, :] = 0.0
        self.pos_errors[uav_idx, :] = 0.0
        self.vel_errors[uav_idx, :] = 0.0

    def set_manual_control(
        self,
        uav_idx: int,
        vel_x: float,
        vel_y: float,
        vel_z: float,
        rot_z: float,
        mode: int,
        sim_time: float,
    ) -> None:
        """
        Aplica consigna de control manual para un UAV o conmuta su modo operativo:
          mode: 0=DISARM/IDLE, 1=MANUAL, 2=AUTO, 3=HOVER
        """
        if uav_idx < 0 or uav_idx >= self.amount_uavs:
            return

        if mode == 0:  # DISARM / IDLE (Corte total de motores)
            self.manual_active[uav_idx] = False
            self.has_plan[uav_idx] = False
            self._clear_fp_telemetry_state(uav_idx)
            self.uav_states[uav_idx] = int(NavSimUAVState.IDLE)
            self.uav_modes[uav_idx] = 0  # DISARMED
            self.rotors_vel[uav_idx, :] = 0.0
            self.forces_to_apply[uav_idx] = 0.0
            self.torques_to_apply[uav_idx] = 0.0
            self.E[uav_idx, :, 0] = 0.0
            self.u[uav_idx] = 0.0
            self.manual_linear_vel[uav_idx, :] = 0.0
            self.manual_yaw_vel[uav_idx] = 0.0

        elif mode == 1:  # MANUAL
            self.manual_active[uav_idx] = True
            self.uav_states[uav_idx] = int(NavSimUAVState.MANUAL)
            self.uav_modes[uav_idx] = 1  # MANUAL
            self.manual_linear_vel[uav_idx, 0] = float(vel_x)
            self.manual_linear_vel[uav_idx, 1] = float(vel_y)
            self.manual_linear_vel[uav_idx, 2] = float(vel_z)
            self.manual_yaw_vel[uav_idx] = float(rot_z)
            self.manual_cmd_times[uav_idx] = sim_time
            self.pos_errors[uav_idx, :] = 0.0
            self.vel_errors[uav_idx, :] = 0.0

        elif mode == 2:  # AUTO
            self.manual_active[uav_idx] = False
            if self.has_plan[uav_idx] and sim_time <= self.fp_finish_times[uav_idx]:
                self.uav_states[uav_idx] = int(NavSimUAVState.BUSY)
                self.uav_modes[uav_idx] = 2  # AUTO
            else:
                self.uav_states[uav_idx] = int(NavSimUAVState.IDLE)
                # Si se solicita AUTO sin un plan de vuelo válido:
                # Failsafe a HOVER si estaba en vuelo, o permanece DISARMED si estaba en reposo
                prev_mode = int(self.uav_modes[uav_idx])
                self.uav_modes[uav_idx] = 3 if prev_mode in (1, 3) else 0

        elif mode == 3:  # HOVER
            self.manual_active[uav_idx] = True
            self.uav_states[uav_idx] = int(NavSimUAVState.MANUAL)
            self.uav_modes[uav_idx] = 3  # HOVER
            self.manual_linear_vel[uav_idx, :] = 0.0
            self.manual_yaw_vel[uav_idx] = 0.0
            self.manual_cmd_times[uav_idx] = sim_time
            self.pos_errors[uav_idx, :] = 0.0
            self.vel_errors[uav_idx, :] = 0.0

    def update_flight_plans_batch(self, new_plans: Dict[int, FlightPlanData]) -> None:
        for uav_idx, fp in new_plans.items():
            self.set_flight_plan(uav_idx, fp)

    # ==========================================================================
    # Bucle Principal de Control y Física Vectorizada
    # ==========================================================================
    def step(self, current_time: float, step_size: float) -> None:
        if self.amount_uavs == 0:
            return

        # 1. Leer estado cinemático de todos los UAVs desde Isaac Sim (para control y telemetría)
        self.imu()

        # 2. Expiración de planes de vuelo para UAVs en modo AUTO
        auto_expired_mask = (
            self.has_plan
            & (~self.manual_active)
            & (current_time >= self.fp_finish_times)
        )
        if np.any(auto_expired_mask):
            for idx in np.nonzero(auto_expired_mask)[0]:
                self.has_plan[idx] = False
                self._clear_fp_telemetry_state(idx)
                if getattr(self, "auto_disarm_on_finish", True):
                    # Auto-Disarm al completar la ruta 4D:
                    # Las trayectorias depositan la aeronave en el vertipuerto (v=0, a=0).
                    # Corte automático de motores a 0 RPM y paso a estado DISARMED.
                    self.manual_active[idx] = False
                    self.uav_states[idx] = int(NavSimUAVState.IDLE)
                    self.uav_modes[idx] = 0  # DISARMED
                    self.rotors_vel[idx, :] = 0.0
                    self.forces_to_apply[idx, :] = 0.0
                    self.torques_to_apply[idx, :] = 0.0
                    self.E[idx, :, 0] = 0.0
                    self.u[idx] = 0.0
                    self.manual_linear_vel[idx, :] = 0.0
                    self.manual_yaw_vel[idx] = 0.0
                else:
                    # Fallback a HOVER si se desactiva el Auto-Disarm
                    self.manual_active[idx] = True
                    self.manual_linear_vel[idx, :] = 0.0
                    self.manual_yaw_vel[idx] = 0.0
                    self.manual_cmd_times[idx] = current_time
                    self.uav_states[idx] = int(NavSimUAVState.MANUAL)
                    self.uav_modes[idx] = 3  # HOVER activo

        # 3. Identificar UAVs activos en modo AUTO
        auto_mask = (
            self.has_plan
            & (~self.manual_active)
            & (current_time >= self.fp_init_times)
            & (current_time <= self.fp_finish_times)
        )
        auto_indices = np.nonzero(auto_mask)[0]
        if auto_indices.size > 0:
            self.uav_states[auto_indices] = int(NavSimUAVState.BUSY)
            auto_lin_vel, auto_yaw_rot = self.navigation(auto_indices, current_time)
        else:
            auto_lin_vel = np.empty((0, 3), dtype=np.float64)
            auto_yaw_rot = np.empty((0,), dtype=np.float64)

        # 4. Identificar UAVs activos en modo MANUAL
        manual_indices = np.nonzero(self.manual_active)[0]
        if manual_indices.size > 0:
            self.uav_states[manual_indices] = int(NavSimUAVState.MANUAL)
            manual_lin_vel = np.zeros((manual_indices.size, 3), dtype=np.float64)
            manual_yaw_rot = np.zeros((manual_indices.size,), dtype=np.float64)

            # Timeout de seguridad: si no hay consigna en más de 1.0s, hacer hover (0 vel)
            for m_i, idx in enumerate(manual_indices):
                if (current_time - self.manual_cmd_times[idx]) > 1.0:
                    manual_lin_vel[m_i, :] = 0.0
                    manual_yaw_rot[m_i] = 0.0
                    if self.uav_modes[idx] == 1:
                        self.uav_modes[idx] = 3  # Failsafe de enlace: pasar a HOVER
                else:
                    manual_lin_vel[m_i, :] = self.manual_linear_vel[idx]
                    manual_yaw_rot[m_i] = self.manual_yaw_vel[idx]
        else:
            manual_lin_vel = np.empty((0, 3), dtype=np.float64)
            manual_yaw_rot = np.empty((0,), dtype=np.float64)

        # 5. Unificar todos los UAVs activos (AUTO + MANUAL)
        if auto_indices.size == 0 and manual_indices.size == 0:
            self.forces_to_apply[:] = 0.0
            self.torques_to_apply[:] = 0.0
            self.rotors_vel[:] = 0.0
            self.apply_dynamics(self.all_indices)
            return

        if auto_indices.size > 0 and manual_indices.size > 0:
            active_indices = np.concatenate([auto_indices, manual_indices])
            cmd_local_linear_vel = np.vstack([auto_lin_vel, manual_lin_vel])
            cmd_yaw_rotation = np.concatenate([auto_yaw_rot, manual_yaw_rot])
        elif auto_indices.size > 0:
            active_indices = auto_indices
            cmd_local_linear_vel = auto_lin_vel
            cmd_yaw_rotation = auto_yaw_rot
        else:
            active_indices = manual_indices
            cmd_local_linear_vel = manual_lin_vel
            cmd_yaw_rotation = manual_yaw_rot

        # 6. Controlador lineal en espacio de estados para todos los activos
        self.servo_control(
            active_indices, cmd_local_linear_vel, cmd_yaw_rotation, step_size
        )

        # 7. Dinámica y fuerzas aerodinámicas en CoM
        self.compute_dynamics(active_indices)
        self.reset_dynamics(active_indices)
        self.apply_dynamics(self.all_indices)

    def imu(self) -> None:
        self.positions, self.orientations = self.rigid_prim_view.get_world_poses()
        self.eulers = self.quaternion_to_euler(self.orientations)
        self.world_linear_vels = self.rigid_prim_view.get_linear_velocities(clone=True)
        self.world_angular_vels = self.rigid_prim_view.get_angular_velocities(clone=True)
        self.local_linear_vels = self.global_to_local_velocity(
            self.world_linear_vels, self.orientations
        )
        self.local_angular_vels = self.global_to_local_velocity(
            self.world_angular_vels, self.orientations
        )

    def navigation(self, uav_indices: np.ndarray, current_time: float):
        times_sub = self.fp_times[uav_indices]
        target_indices = self.get_running_idx(times_sub, current_time)
        self.current_wp_indices[uav_indices] = target_indices

        running_times = self.fp_times[uav_indices, target_indices]
        self.current_wp_times[uav_indices] = running_times
        for i_sub, uav_i in enumerate(uav_indices):
            wp_i = int(target_indices[i_sub])
            labels = self.fp_wp_labels[uav_i]
            self.current_wp_labels[uav_i] = labels[wp_i] if 0 <= wp_i < len(labels) else ""

        running_pos = self.fp_positions[uav_indices, target_indices]
        running_vel = self.fp_velocities[uav_indices, target_indices]
        running_accel = self.fp_accelerations[uav_indices, target_indices]
        running_jerk = self.fp_jerks[uav_indices, target_indices]
        running_snap = self.fp_snaps[uav_indices, target_indices]
        running_crackel = self.fp_crackels[uav_indices, target_indices]
        running_headings = self.fp_headings[uav_indices, target_indices].copy()

        expected_pos, expected_vel = self.status_at_time(
            current_time,
            running_times,
            running_pos,
            running_vel,
            running_accel,
            running_jerk,
            running_snap,
            running_crackel,
        )

        # Guardar consigna 4D evaluada y errores de seguimiento para telemetría MAVLink (ID 12003)
        self.desired_positions[uav_indices] = expected_pos
        self.desired_velocities[uav_indices] = expected_vel
        self.pos_errors[uav_indices] = self.positions[uav_indices] - expected_pos
        self.vel_errors[uav_indices] = self.world_linear_vels[uav_indices] - expected_vel

        cmd_world_linear_vel, cmd_yaw_rotation = self.get_command(
            expected_pos,
            expected_vel,
            self.positions[uav_indices],
            self.world_linear_vels[uav_indices],
            self.eulers[uav_indices, 2],
            running_headings,
            self.fleet.t_to_solve[uav_indices],
        )

        cmd_local_linear_vel = self.global_to_local_velocity(
            cmd_world_linear_vel, self.orientations[uav_indices]
        )

        return cmd_local_linear_vel, cmd_yaw_rotation

    def servo_control(
        self,
        uav_indices: np.ndarray,
        command_linear_vel: np.ndarray,
        command_angular_vel: np.ndarray,
        step_size: float,
    ) -> None:
        # Vector de referencia r (4x1): [vx_body, vy_body, vz_body, wz_body]
        self.r[uav_indices, 0, 0] = command_linear_vel[:, 0]
        self.r[uav_indices, 1, 0] = command_linear_vel[:, 1]
        self.r[uav_indices, 2, 0] = command_linear_vel[:, 2]
        self.r[uav_indices, 3, 0] = command_angular_vel

        # Vector de estado x (8x1): [roll, pitch, wx, wy, wz, vx_body, vy_body, vz_body]
        self.x[uav_indices, :2, 0] = self.eulers[uav_indices, :2]
        self.x[uav_indices, 2:5, 0] = self.local_angular_vels[uav_indices]
        self.x[uav_indices, 5:8, 0] = self.local_linear_vels[uav_indices]

        # Vector de salida y (4x1): [vx_body, vy_body, vz_body, wz_body]
        self.y[uav_indices, 0, 0] = self.x[uav_indices, 5, 0]
        self.y[uav_indices, 1, 0] = self.x[uav_indices, 6, 0]
        self.y[uav_indices, 2, 0] = self.x[uav_indices, 7, 0]
        self.y[uav_indices, 3, 0] = self.x[uav_indices, 4, 0]

        # Error instantáneo e integral saturada por tipo de dron
        self.e[uav_indices] = self.y[uav_indices] - self.r[uav_indices]
        self.E[uav_indices] += self.e[uav_indices] * step_size
        self.E[uav_indices] = np.clip(
            self.E[uav_indices],
            -self.fleet.E_max[uav_indices],
            self.fleet.E_max[uav_indices],
        )

        # Ley de control lineal heterogénea: u = Hs - Kx @ x - Ky @ E
        self.u[uav_indices] = (
            self.fleet.Hs[uav_indices]
            - (self.fleet.Kx[uav_indices] @ self.x[uav_indices])
            - (self.fleet.Ky[uav_indices] @ self.E[uav_indices])
        )

        # Saturación de velocidades de rotor según límites de cada modelo
        self.u[uav_indices] = np.clip(
            self.u[uav_indices],
            self.fleet.w_min[uav_indices],
            self.fleet.w_max[uav_indices],
        )

        # Orden canónico de rotores: [NE, NW, SE, SW]
        self.rotors_vel[uav_indices] = self.u[uav_indices, :, 0]

    def compute_dynamics(self, uav_indices: np.ndarray) -> None:
        w_sq = self.rotors_vel[uav_indices] ** 2  # (K, 4)

        # 1. Empuje vertical de los 4 rotores: FT_i = kFT_i * w_i²
        thrust_z = self.fleet.thrust_coeffs[uav_indices] * w_sq  # (K, 4)

        # 2. Rozamiento aerodinámico lineal del fuselaje: FD = -kFD * |v| * v
        v_loc = self.local_linear_vels[uav_indices]
        fd = self.fleet.drag_force_coeffs[uav_indices] * np.abs(v_loc) * v_loc  # (K, 3)

        # 3. Par de arrastre de rotores en eje Z (signos incluidos en kMDR)
        mdr_z = np.sum(self.fleet.drag_moment_coeffs[uav_indices] * w_sq, axis=1)  # (K,)

        # 4. Momento de rozamiento angular del fuselaje: MD = -kMD * |w| * w
        w_loc = self.local_angular_vels[uav_indices]
        md = self.fleet.friction_moment_coeffs[uav_indices] * np.abs(w_loc) * w_loc  # (K, 3)

        # 5. Ensamblaje de fuerzas y cálculo de pares respecto al Centro de Masas
        self.bodies_forces[uav_indices] = 0.0
        self.bodies_forces[uav_indices, 0, :] = fd
        self.bodies_forces[uav_indices, 1:, 2] = thrust_z

        self.forces_to_apply[uav_indices] = np.sum(
            self.bodies_forces[uav_indices], axis=1
        )

        torques = np.sum(
            np.cross(
                self.fleet.bodies_positions[uav_indices],
                self.bodies_forces[uav_indices],
            ),
            axis=1,
        )

        self.torques_to_apply[uav_indices] = md + torques
        self.torques_to_apply[uav_indices, 2] += mdr_z

    def apply_dynamics(self, uav_indices: np.ndarray) -> None:
        self.rigid_prim_view.apply_forces_and_torques_at_pos(
            forces=self.forces_to_apply[uav_indices],
            torques=self.torques_to_apply[uav_indices],
            is_global=False,
            indices=self.all_indices[uav_indices],
        )

    def reset_dynamics(self, active_indices: np.ndarray) -> None:
        mask = np.ones(self.amount_uavs, dtype=bool)
        if active_indices.size > 0:
            mask[active_indices] = False
        if np.any(mask):
            self.forces_to_apply[mask] = 0.0
            self.torques_to_apply[mask] = 0.0
            self.E[mask, :, 0] = 0.0
            self.rotors_vel[mask, :] = 0.0

    # ==========================================================================
    # Funciones Matemáticas Vectorizadas
    # ==========================================================================
    @staticmethod
    def rotate_vector_by_quaternion(
        vector: np.ndarray, quaternion: np.ndarray, conjugate: bool
    ) -> np.ndarray:
        is_1d = vector.ndim == 1
        if is_1d:
            vector = vector[np.newaxis, :]
        if quaternion.ndim == 1:
            quaternion = quaternion[np.newaxis, :]

        w = quaternion[:, 0:1]
        u = -quaternion[:, 1:4] if conjugate else quaternion[:, 1:4]

        t = 2.0 * np.cross(u, vector)
        rotated = vector + w * t + np.cross(u, t)
        return rotated[0] if is_1d else rotated

    def local_to_global_velocity(self, vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
        return self.rotate_vector_by_quaternion(vector, quaternion, conjugate=False)

    def global_to_local_velocity(self, vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
        return self.rotate_vector_by_quaternion(vector, quaternion, conjugate=True)

    @staticmethod
    def quaternion_to_euler(quaternion: np.ndarray) -> np.ndarray:
        is_1d = quaternion.ndim == 1
        if is_1d:
            quaternion = quaternion[np.newaxis, :]

        w, x, y, z = quaternion[:, 0], quaternion[:, 1], quaternion[:, 2], quaternion[:, 3]

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        euler = np.column_stack((roll, pitch, yaw))
        return euler[0] if is_1d else euler

    @staticmethod
    def get_running_idx(padded_times: np.ndarray, current_time: float) -> np.ndarray:
        indices = np.sum(padded_times <= current_time, axis=1) - 1
        return np.clip(indices, 0, None)

    @staticmethod
    def status_at_time(
        current_time: float,
        running_times: np.ndarray,
        running_pos: np.ndarray,
        running_vel: np.ndarray,
        running_accel: np.ndarray,
        running_jerk: np.ndarray,
        running_snap: np.ndarray,
        running_crackel: np.ndarray,
    ):
        dt = np.clip(current_time - running_times, 0.0, None)[:, np.newaxis]
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        dt5 = dt4 * dt

        pos = (
            running_pos
            + running_vel * dt
            + 0.5 * running_accel * dt2
            + (1.0 / 6.0) * running_jerk * dt3
            + (1.0 / 24.0) * running_snap * dt4
            + (1.0 / 120.0) * running_crackel * dt5
        )
        vel = (
            running_vel
            + running_accel * dt
            + 0.5 * running_jerk * dt2
            + (1.0 / 6.0) * running_snap * dt3
            + (1.0 / 24.0) * running_crackel * dt4
        )
        return pos, vel

    @staticmethod
    def get_command(
        expected_pos: np.ndarray,
        expected_vel: np.ndarray,
        current_pos: np.ndarray,
        current_lin_vel: np.ndarray,
        current_yaw: np.ndarray,
        headings: np.ndarray,
        t_to_solve: np.ndarray,
    ):
        correction_vel = (expected_pos - current_pos) / t_to_solve
        command_linear_vel = expected_vel + correction_vel

        mask = (headings[:, 0] == 0.0) & (headings[:, 1] == 0.0)
        headings[mask] = expected_vel[mask, :2]
        mask = (headings[:, 0] == 0.0) & (headings[:, 1] == 0.0)

        target_yaw = np.arctan2(headings[:, 1], headings[:, 0])
        target_yaw[mask] = current_yaw[mask]

        yaw_error = (target_yaw - current_yaw + np.pi) % (2.0 * np.pi) - np.pi
        command_yaw_rotation = yaw_error / t_to_solve[:, 0]

        return command_linear_vel, command_yaw_rotation

    def get_uav_mode(self, uav_idx: int) -> int:
        """
        Devuelve el modo operativo actual del UAV según la máquina de estados:
          0=DISARMED, 1=MANUAL, 2=AUTO, 3=HOVER, 4=SYSTEM_FAILURE
        """
        if hasattr(self, "uav_modes") and 0 <= uav_idx < len(self.uav_modes):
            return int(self.uav_modes[uav_idx])
        return 0

