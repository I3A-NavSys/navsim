import asyncio
import importlib
import logging
import math
import socket
from typing import Optional

import numpy as np
import omni.usd
from pxr import Gf, UsdGeom, Usd

import mavlink.dialect
if not hasattr(mavlink.dialect, "MavManualControlMsg"):
    importlib.reload(mavlink.dialect)

from mavlink.dialect import MAVLinkV2Codec, MavManualControlMsg
from .joystick_input import JoystickInput
from .keyboard_input import KeyboardInput


class ControllerLogic:
    """
    Lógica de control cinemático manual para UAVs en Isaac Sim:
      1. Lee entradas de joystick/teclado.
      2. Empaqueta tramas MAVLink v2 NAVSIM_MANUAL_CONTROL.
      3. Las transmite por UDP directamente al puerto RX del dron seleccionado
         (14550 + uav_idx).
      4. Maneja una cámara de persecución (ChaseCamera) en un Viewport SECUNDARIO exclusivo,
         dejando intacto el Viewport principal (Viewport 1) del usuario.
    """

    def __init__(self, base_rx_port: int = 14550):
        self.logger = logging.getLogger("ManualControllerLogic")
        self.base_rx_port = base_rx_port
        self._stop = True

        self.uav_idx = 0
        self.uav_path = ""
        self.linear_vel_limit = 2.0
        self.ang_vel_limit = 1.0
        self.current_mode = 1  # 1=MANUAL, 2=AUTO, 3=HOVER, 0=DISARM
        self.current_on = False
        self.invert_camera_movement = False
        self.invert_yaw = False

        # Configuración del Viewport Secundario
        self.viewport_window_title = "NavSim — Cámara de Seguimiento"
        self.viewport_window = None

        # Parámetros locales de ChaseCamera
        self.camera_path = ""
        self.camera_prim = None
        self.cam_distance = 14.0
        self.cam_height = 3.5
        self.cam_target_z = 1.2
        self.cam_yaw_deg = 0.0
        self.cam_pitch_deg = 0.0

        self.joystick = JoystickInput()
        self.keyboard = KeyboardInput()
        self.use_keyboard = False

        # MAVLink Codec & UDP Socket
        self.codec = MAVLinkV2Codec(sysid=253, compid=1)
        self.sock: Optional[socket.socket] = None

        # Telemetría instantánea para gráficas
        self.latest_command_vel = np.zeros(3, dtype=np.float32)
        self.latest_command_yaw = 0.0
        self._changed_trigger = False

    def start(self, uav_path: str, uav_idx: int) -> None:
        if not self._stop:
            self.stop()

        self._stop = False
        self.uav_path = uav_path
        self.uav_idx = uav_idx

        if self.sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 1. Configurar la cámara hija montada en el UAV
        self.setup_camera_for_uav(self.uav_path)

        # 2. Abrir o enfocar el viewport secundario independiente
        # NOTA: El viewport principal (Viewport 1) permanece intacto.
        self.open_or_update_viewport()

        self.joystick.start()
        if self.use_keyboard:
            self.keyboard.start()

        self.current_on = True
        self.current_mode = 1  # Iniciar en modo MANUAL
        self.logger.info(f"Control manual activado para UAV {uav_idx} ({uav_path})")

        asyncio.ensure_future(self.control_loop())

    def stop(self) -> None:
        if not self._stop:
            self._stop = True
            # Enviar comando de hover/disarm al detener
            self.send_command(0.0, 0.0, 0.0, 0.0, mode=3)
            self.joystick.stop()
            if self.use_keyboard:
                self.keyboard.stop()

            if self.sock is not None:
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None

            self.current_on = False
            self.latest_command_vel[:] = 0.0
            self.latest_command_yaw = 0.0
            self.logger.info("Control manual detenido")

    def set_uav(self, uav_path: str, uav_idx: int) -> None:
        """Cambia el UAV objetivo y conmuta la cámara del viewport secundario."""
        self.uav_path = uav_path
        self.uav_idx = uav_idx
        cam_path = self.setup_camera_for_uav(uav_path)
        if self.viewport_window is not None:
            self.open_or_update_viewport()
        self.logger.info(f"UAV objetivo cambiado a {uav_path} (idx {uav_idx}, cam: {cam_path})")

    def set_mode(self, mode: int) -> None:
        """Cambia el modo operativo del UAV: 0=DISARM, 1=MANUAL, 2=AUTO, 3=HOVER."""
        self.current_mode = mode
        if mode == 0:
            self.current_on = False
            self.send_command(0.0, 0.0, 0.0, 0.0, mode=0)
        elif mode == 3:
            self.send_command(0.0, 0.0, 0.0, 0.0, mode=3)
        elif mode == 2:
            self.send_command(0.0, 0.0, 0.0, 0.0, mode=2)
        elif mode == 1:
            self.current_on = True
            self.send_command(0.0, 0.0, 0.0, 0.0, mode=1)

    def send_command(
        self, vel_x: float, vel_y: float, vel_z: float, rot_z: float, mode: int = 1
    ) -> None:
        """Transmite una trama MAVLink NAVSIM_MANUAL_CONTROL al puerto RX del dron."""
        if self.sock is None:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except OSError:
                return

        target_port = self.base_rx_port + self.uav_idx
        msg = MavManualControlMsg(
            uav_id_num=self.uav_idx,
            mode=mode,
            vel_x=float(vel_x),
            vel_y=float(vel_y),
            vel_z=float(vel_z),
            rot_z=float(rot_z),
            buttons=1 if self.current_on else 0,
        )
        packet = self.codec.pack_manual_control(msg)
        try:
            self.sock.sendto(packet, ("127.0.0.1", target_port))
        except OSError:
            pass

    async def control_loop(self) -> None:
        while not self._stop:
            inputs = self.get_inputs()

            # Escalar por límites configurados
            vx = inputs[0] * self.linear_vel_limit
            vy = inputs[1] * self.linear_vel_limit
            vz = inputs[2] * self.linear_vel_limit
            wz = inputs[3] * self.ang_vel_limit
            if self.invert_yaw:
                wz = -wz

            # Botón 0: Gatillo (toggle on/off)
            trigger = inputs[4]
            if trigger == 1.0 and not self._changed_trigger:
                self._changed_trigger = True
                self.current_on = not self.current_on
                if not self.current_on:
                    self.set_mode(0)
                else:
                    self.set_mode(1)
            elif trigger == 0.0:
                self._changed_trigger = False

            # Si estamos en modo MANUAL y armados, enviar velocidades
            if self.current_mode == 1 and self.current_on:
                self.send_command(vx, vy, vz, wz, mode=1)
                self.latest_command_vel[0] = vx
                self.latest_command_vel[1] = vy
                self.latest_command_vel[2] = vz
                self.latest_command_yaw = wz
            elif self.current_mode == 3:  # HOVER
                self.latest_command_vel[:] = 0.0
                self.latest_command_yaw = 0.0
                self.send_command(0.0, 0.0, 0.0, 0.0, mode=3)
            else:
                self.latest_command_vel[:] = 0.0
                self.latest_command_yaw = 0.0

            # Control de cámara con hat y botones de zoom
            zoom_in = inputs[5]
            zoom_out = inputs[6]
            cam_yaw = inputs[7]
            cam_pitch = inputs[8]
            if (
                zoom_in != 0.0
                or zoom_out != 0.0
                or cam_yaw != 0.0
                or cam_pitch != 0.0
            ):
                self.move_camera(zoom_in, zoom_out, cam_yaw, cam_pitch)

            await asyncio.sleep(0.05)  # 20 Hz

    def get_inputs(self) -> np.ndarray:
        self.joystick.ask_input()
        joy_inputs = np.array(self.joystick.inputs, dtype=np.float32)

        # Si hay joystick conectado y con entradas distintas de cero, tiene prioridad
        if np.any(joy_inputs[:4] != 0):
            return joy_inputs

        if self.use_keyboard:
            key_inputs = np.array(self.keyboard.inputs, dtype=np.float32)
            if np.any(key_inputs != 0):
                return key_inputs

        return joy_inputs

    # ==========================================================================
    # Gestión de Cámara de Persecución y Viewport Secundario
    # ==========================================================================
    def setup_camera_for_uav(self, uav_path: str) -> str:
        """Crea o reutiliza la cámara de persecución como prim hijo del UAV."""
        stage = omni.usd.get_context().get_stage()
        if not stage or not uav_path:
            return ""

        target_prim = stage.GetPrimAtPath(uav_path)
        if not target_prim or not target_prim.IsValid():
            return ""

        # Calibrar dimensiones según el modelo del dron
        try:
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
            bbox = bbox_cache.ComputeWorldBound(target_prim)
            box_size = bbox.ComputeAlignedBox().GetSize()
            max_dim = max(float(box_size[0]), float(box_size[1]), float(box_size[2]))

            if max_dim > 3.0:  # Aerotaxi (~8-10m)
                self.cam_distance = max(14.0, max_dim * 1.6)
                self.cam_height = 3.8
                self.cam_target_z = 1.2
            elif max_dim < 1.0:  # Minidrones (~0.25-0.5m)
                self.cam_distance = 2.0
                self.cam_height = 0.6
                self.cam_target_z = 0.15
            else:  # Drones medianos
                self.cam_distance = max_dim * 2.5
                self.cam_height = float(box_size[2]) * 0.7
                self.cam_target_z = float(box_size[2]) * 0.3
        except Exception:
            self.cam_distance = 14.0
            self.cam_height = 3.5
            self.cam_target_z = 1.2

        cam_path = f"{uav_path}/ChaseCamera"
        cam_prim = stage.GetPrimAtPath(cam_path)
        if not cam_prim.IsValid():
            usd_cam = UsdGeom.Camera.Define(stage, cam_path)
            cam_prim = usd_cam.GetPrim()
            usd_cam.GetFocalLengthAttr().Set(28.0)  # Lente 28mm gran angular
            usd_cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 100000.0))

        self.camera_path = cam_path
        self.camera_prim = cam_prim
        self._apply_camera_matrix(cam_prim)

        return cam_path

    def _apply_camera_matrix(self, cam_prim) -> None:
        """Aplica la matriz de transformación local de la cámara respecto al UAV."""
        if not cam_prim or not cam_prim.IsValid():
            return

        yaw_rad = math.radians(self.cam_yaw_deg)
        pitch_rad = math.radians(self.cam_pitch_deg)

        eye_x = -self.cam_distance * math.cos(yaw_rad) * math.cos(pitch_rad)
        eye_y = -self.cam_distance * math.sin(yaw_rad) * math.cos(pitch_rad)
        eye_z = self.cam_height + self.cam_distance * math.sin(pitch_rad)

        eye = Gf.Vec3d(eye_x, eye_y, eye_z)
        target = Gf.Vec3d(0.0, 0.0, self.cam_target_z)

        f = (target - eye).GetNormalized()
        up_vec = Gf.Vec3d(0.0, 0.0, 1.0)
        r = Gf.Cross(f, up_vec).GetNormalized()
        u = Gf.Cross(r, f).GetNormalized()

        # En convención USD la cámara mira hacia -Z local
        m = Gf.Matrix4d(
            r[0], r[1], r[2], 0.0,
            u[0], u[1], u[2], 0.0,
            -f[0], -f[1], -f[2], 0.0,
            eye[0], eye[1], eye[2], 1.0,
        )

        xform = UsdGeom.Xformable(cam_prim)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(m)

    def open_or_update_viewport(self) -> None:
        """
        Abre o actualiza un Viewport SECUNDARIO exclusivo mostrando la ChaseCamera del UAV.
        IMPORTANTE: NUNCA modifica el Viewport 1 (visor principal) del usuario.
        """
        if not self.camera_path:
            return

        win = None
        try:
            from omni.kit.viewport.window import get_viewport_window_instances
            for instance in get_viewport_window_instances(None):
                t = getattr(instance, "title", "") or getattr(instance, "name", "")
                if t == self.viewport_window_title:
                    win = instance
                    break
        except Exception:
            pass

        if win is not None:
            self.viewport_window = win
            try:
                self.viewport_window.visible = True
            except Exception:
                pass
        else:
            try:
                from omni.kit.viewport.utility import create_viewport_window
                self.viewport_window = create_viewport_window(
                    name=self.viewport_window_title,
                    width=640,
                    height=480,
                    camera_path=self.camera_path,
                )
            except Exception as e:
                self.logger.warning(f"Error al instanciar create_viewport_window: {e}")
                self.viewport_window = None

        if self.viewport_window and hasattr(self.viewport_window, "viewport_api"):
            try:
                self.viewport_window.viewport_api.set_active_camera(self.camera_path)
            except Exception:
                try:
                    self.viewport_window.viewport_api.camera_path = self.camera_path
                except Exception:
                    pass

    def destroy_camera(self) -> None:
        """
        Cierra el Viewport secundario y desvincula la cámara sin tocar el Viewport principal.
        """
        if self.viewport_window:
            try:
                self.viewport_window.destroy()
            except Exception:
                pass
            self.viewport_window = None

        self.camera_prim = None
        self.camera_path = ""

    def move_camera(
        self, zoom_in: float, zoom_out: float, yaw: float, pitch: float
    ) -> None:
        """Ajusta zoom y orientación angular de la cámara de persecución."""
        if not self.camera_prim or not self.camera_prim.IsValid():
            return

        if self.invert_camera_movement:
            yaw *= -1
            pitch *= -1

        dist_delta = zoom_in + zoom_out
        self.cam_distance = max(3.0, self.cam_distance + dist_delta * 0.4)
        self.cam_yaw_deg += yaw * 3.0
        self.cam_pitch_deg = float(np.clip(self.cam_pitch_deg + pitch * 2.0, -30.0, 60.0))

        self._apply_camera_matrix(self.camera_prim)