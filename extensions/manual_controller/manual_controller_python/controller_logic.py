import asyncio
import importlib
import logging
import math
import socket
from typing import Any, Callable, Optional, Tuple

import numpy as np
import omni.usd
from pxr import Gf, UsdGeom, Usd

import libs.mavlink.dialect as mavlink_dialect
if not hasattr(mavlink_dialect, "MavManualControlMsg"):
    importlib.reload(mavlink_dialect)

from libs.mavlink.dialect import MAVLinkV2Codec, MavManualControlMsg
from .joystick_input import JoystickInput
from .keyboard_input import KeyboardInput


class ControllerLogic:
    """
    Lógica de control cinemático manual para UAVs en Isaac Sim:
      1. Lee entradas de joystick/teclado.
      2. Empaqueta tramas MAVLink v2 NAVSIM_MANUAL_CONTROL.
      3. Las transmite por UDP directamente al puerto RX del dron seleccionado
         (15000 + uav_idx).
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
        self.invert_camera_vertical = True
        self.invert_camera_horizontal = False
        self.invert_drone_yaw = False

        # Configuración del Viewport Secundario
        self.WINDOW_ID = "NavSim_ChaseCamera_Viewport"
        self.uas_id = ""
        self.viewport_window_title = f"NavSim###{self.WINDOW_ID}"
        self.viewport_window = None

        # Parámetros locales de ChaseCamera
        self.camera_path = ""
        self.camera_prim = None
        self.cam_distance = 10.0
        self.initial_cam_distance = 10.0
        self.initial_cam_height = 3.5
        self.initial_cam_target_z = 1.2
        self.min_cam_distance = 0.4
        self.max_cam_distance = 80.0
        self.cam_height = 3.5
        self.cam_target_z = 1.2
        self.cam_yaw_deg = 0.0
        self.cam_pitch_deg = 0.0
        self._changed_trigger = False
        self._changed_reset_cam = False
        self.on_mode_change_requested: Optional[Callable[[int], None]] = None

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

    def get_uas_id(self, uav_path: Optional[str] = None) -> str:
        """Obtiene el identificador uas_id del UAV (desde el atributo USD 'NavSim:id' o el nombre del Prim)."""
        target_path = uav_path or self.uav_path
        if not target_path:
            return ""
        try:
            stage = omni.usd.get_context().get_stage()
            if stage:
                prim = stage.GetPrimAtPath(target_path)
                if prim and prim.IsValid():
                    id_attr = prim.GetAttribute("NavSim:id")
                    if id_attr and id_attr.IsValid():
                        raw_id = id_attr.Get()
                        if raw_id is not None and str(raw_id).strip():
                            return str(raw_id).strip()
                    return str(prim.GetName())
        except Exception:
            pass
        return target_path.split("/")[-1] if target_path else ""

    def find_existing_viewport_window(self) -> Optional[Any]:
        """Localiza la ventana del viewport secundario de la cámara si ya existe en Kit."""
        if self.viewport_window is not None:
            try:
                if hasattr(self.viewport_window, "viewport_api") and self.viewport_window.viewport_api is not None:
                    return self.viewport_window
            except Exception:
                pass
            self.viewport_window = None

        try:
            from omni.kit.viewport.window import get_viewport_window_instances
            for instance in get_viewport_window_instances(None):
                try:
                    t = getattr(instance, "title", "") or ""
                    n = getattr(instance, "name", "") or ""
                    if (
                        self.WINDOW_ID in t
                        or self.WINDOW_ID in n
                        or t.startswith("NavSim -")
                        or t.startswith("NavSim —")
                        or n.startswith("NavSim -")
                        or n.startswith("NavSim —")
                        or (hasattr(instance, "name") and str(instance.name).startswith("NavSim"))
                    ):
                        self.viewport_window = instance
                        return instance
                except Exception:
                    continue
        except Exception:
            pass
        return None

    def is_viewport_open(self) -> bool:
        """Indica si el visor secundario está actualmente instanciado y visible en pantalla."""
        win = self.find_existing_viewport_window()
        if win is None:
            return False
        try:
            if hasattr(win, "visible") and not win.visible:
                return False
            return True
        except Exception:
            return False

    def toggle_viewport(self) -> None:
        """Muestra u oculta (toggle) la ventana del Viewport secundario de la cámara."""
        win = self.find_existing_viewport_window()
        if win is not None:
            try:
                cur_visible = bool(getattr(win, "visible", True))
                win.visible = not cur_visible
                if win.visible:
                    try:
                        win.focus()
                    except Exception:
                        pass
                return
            except Exception:
                pass
        # Si no existe o no se pudo alternar, lo abre
        self.open_or_update_viewport()

    def start(self, uav_path: str, uav_idx: int) -> None:
        if not self._stop:
            self.stop()

        self._stop = False
        self.uav_path = uav_path
        self.uav_idx = uav_idx
        self.uas_id = self.get_uas_id(uav_path)
        display_title = f"NavSim - {self.uas_id}" if self.uas_id else "NavSim"
        self.viewport_window_title = f"{display_title}###{self.WINDOW_ID}"

        if self.sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 1. Configurar la cámara hija montada en el UAV
        self.setup_camera_for_uav(self.uav_path)

        # 2. Abrir o enfocar el viewport secundario independiente con el título "NavSim - <uas_id>"
        # NOTA: El viewport principal (Viewport 1) permanece intacto.
        self.open_or_update_viewport()

        self.joystick.start()
        if self.use_keyboard:
            self.keyboard.start()

        self.current_on = True
        self.current_mode = 1  # Iniciar en modo MANUAL
        self.logger.info(f"Control manual activado para UAV {uav_idx} ({self.uas_id}, {uav_path})")

        asyncio.ensure_future(self.control_loop())

    def stop(self) -> None:
        self._stop = True
        self.current_mode = 0
        self.current_on = False
        # Enviar comando de DISARM (mode=0) al dron para apagar motores
        self.send_command(0.0, 0.0, 0.0, 0.0, mode=0)
        self.joystick.stop()
        if self.use_keyboard:
            self.keyboard.stop()

        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

        self.latest_command_vel[:] = 0.0
        self.latest_command_yaw = 0.0
        self.logger.info("Control manual: UAV desarmado (DISARM, mode=0)")

    def set_uav(self, uav_path: str, uav_idx: int) -> None:
        """Cambia el UAV objetivo y actualiza dinámicamente la cámara y el título del visor secundario."""
        self.uav_path = uav_path
        self.uav_idx = uav_idx
        self.uas_id = self.get_uas_id(uav_path)
        display_title = f"NavSim - {self.uas_id}" if self.uas_id else "NavSim"
        self.viewport_window_title = f"{display_title}###{self.WINDOW_ID}"

        cam_path = self.setup_camera_for_uav(uav_path)
        # Si el visor de cámara ya está abierto, actualizar dinámicamente cámara y título
        if self.is_viewport_open():
            self.open_or_update_viewport()

        self.logger.info(f"UAV objetivo cambiado a {uav_path} ({self.uas_id}, idx {uav_idx}, cam: {cam_path})")

    def set_mode(self, mode: int) -> None:
        """Cambia el modo operativo del UAV: 0=DISARM, 1=MANUAL, 2=AUTO, 3=HOVER."""
        self.current_mode = mode
        if mode == 0:
            self.current_on = False
            self.send_command(0.0, 0.0, 0.0, 0.0, mode=0)
        elif mode == 3:
            self.current_on = True
            self.send_command(0.0, 0.0, 0.0, 0.0, mode=3)
        elif mode == 2:
            self.current_on = True
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

        target_port = self.base_rx_port
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
            if self.invert_drone_yaw:
                wz = -wz

            # Teclado Espacio: Alternar entre HOVER (3) y MANUAL (1)
            if self.use_keyboard and not np.any(self.joystick.inputs[:4] != 0):
                trigger = inputs[4]
                if trigger == 1.0 and not self._changed_trigger:
                    self._changed_trigger = True
                    new_mode = 3 if self.current_mode == 1 else 1
                    if hasattr(self, "on_mode_change_requested") and callable(self.on_mode_change_requested):
                        self.on_mode_change_requested(new_mode)
                    else:
                        self.set_mode(new_mode)
                elif trigger == 0.0:
                    self._changed_trigger = False

            # Reset de cámara a posición inicial (inputs[9])
            reset_cam = inputs[9] if len(inputs) > 9 else 0.0
            if reset_cam == 1.0 and not self._changed_reset_cam:
                self._changed_reset_cam = True
                self.reset_camera()
            elif reset_cam == 0.0:
                self._changed_reset_cam = False

            self.latest_command_vel[0] = vx
            self.latest_command_vel[1] = vy
            self.latest_command_vel[2] = vz
            self.latest_command_yaw = wz

            # Si estamos en modo MANUAL y armados, enviar velocidades
            if self.current_mode == 1 and self.current_on:
                self.send_command(vx, vy, vz, wz, mode=1)
            elif self.current_mode == 3:  # HOVER
                self.send_command(0.0, 0.0, 0.0, 0.0, mode=3)

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

            await asyncio.sleep(0.1)  # 10 Hz (100 ms por consigna manual)

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

    def get_current_velocities(self) -> Tuple[float, float, float, float]:
        """
        Devuelve las consignas (vx, vy, vz, wz) calculadas a partir de las entradas
        actuales (joystick o teclado) y los límites cinemáticos configurados.
        Disponible en todo momento para alimentar las gráficas del monitor.
        """
        inputs = self.get_inputs()
        vx = float(inputs[0] * self.linear_vel_limit)
        vy = float(inputs[1] * self.linear_vel_limit)
        vz = float(inputs[2] * self.linear_vel_limit)
        wz = float(inputs[3] * self.ang_vel_limit)
        if self.invert_drone_yaw:
            wz = -wz
        return vx, vy, vz, wz

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
                self.min_cam_distance = 3.0
                self.max_cam_distance = 80.0
                self.cam_height = 3.8
                self.cam_target_z = 1.2
            elif max_dim < 1.0:  # Minidrones (~0.25-0.5m)
                self.cam_distance = 2.0
                self.min_cam_distance = 0.4
                self.max_cam_distance = 15.0
                self.cam_height = 0.6
                self.cam_target_z = 0.15
            else:  # Drones medianos (~1-3m)
                self.cam_distance = max(3.0, max_dim * 2.5)
                self.min_cam_distance = max(0.8, max_dim * 0.8)
                self.max_cam_distance = 40.0
                self.cam_height = float(box_size[2]) * 0.7
                self.cam_target_z = float(box_size[2]) * 0.3
        except Exception:
            self.cam_distance = 14.0
            self.min_cam_distance = 1.0
            self.max_cam_distance = 80.0
            self.cam_height = 3.5
            self.cam_target_z = 1.2

        self.initial_cam_distance = self.cam_distance
        self.initial_cam_height = self.cam_height
        self.initial_cam_target_z = self.cam_target_z

        cam_path = f"{uav_path}/ChaseCamera"
        if self.camera_path != cam_path:
            self.cam_yaw_deg = 0.0
            self.cam_pitch_deg = 0.0

        cam_prim = stage.GetPrimAtPath(cam_path)
        usd_cam = None
        if not cam_prim.IsValid():
            usd_cam = UsdGeom.Camera.Define(stage, cam_path)
            cam_prim = usd_cam.GetPrim()
            usd_cam.GetFocalLengthAttr().Set(28.0)  # Lente 28mm gran angular
            usd_cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 100000.0))
        else:
            usd_cam = UsdGeom.Camera(cam_prim)

        if usd_cam:
            focus_attr = usd_cam.GetFocusDistanceAttr()
            if not focus_attr.IsValid():
                focus_attr = usd_cam.CreateFocusDistanceAttr()
            focus_attr.Set(float(self.cam_distance))

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

        # Sincronizar focusDistance con la distancia real al target del dron en el visor Kit
        try:
            usd_cam = UsdGeom.Camera(cam_prim)
            if usd_cam:
                focus_attr = usd_cam.GetFocusDistanceAttr()
                if not focus_attr.IsValid():
                    focus_attr = usd_cam.CreateFocusDistanceAttr()
                focal_dist = float((target - eye).GetLength())
                focus_attr.Set(focal_dist)
        except Exception:
            pass

    def reset_camera(self) -> None:
        """Restablece la cámara de persecución a su posición y distancia inicial por defecto."""
        if not self.camera_prim or not self.camera_prim.IsValid():
            return
        self.cam_distance = getattr(self, "initial_cam_distance", 10.0)
        self.cam_height = getattr(self, "initial_cam_height", 3.5)
        self.cam_target_z = getattr(self, "initial_cam_target_z", 1.2)
        self.cam_yaw_deg = 0.0
        self.cam_pitch_deg = 0.0
        self._apply_camera_matrix(self.camera_prim)
        self.logger.info(
            f"Cámara restablecida a posición inicial: dist={self.cam_distance:.2f}m, yaw=0°, pitch=0°"
        )

    def open_or_update_viewport(self) -> None:
        """
        Abre o actualiza un Viewport SECUNDARIO exclusivo mostrando la ChaseCamera del UAV.
        IMPORTANTE: NUNCA modifica el Viewport 1 (visor principal) del usuario.
        Actualiza dinámicamente el título en la parte superior a 'NavSim - <uas_id>'
        manteniendo un ID de ventana persistente (Dear ImGui ###) para preservar
        el acople (docking) y evitar que la ventana se colapse o minimice a cero.
        """
        if not self.camera_path:
            return

        if not self.uas_id and self.uav_path:
            self.uas_id = self.get_uas_id(self.uav_path)
        display_title = f"NavSim - {self.uas_id}" if self.uas_id else "NavSim"
        self.viewport_window_title = f"{display_title}###{self.WINDOW_ID}"

        win = self.find_existing_viewport_window()

        if win is not None:
            self.viewport_window = win
            try:
                self.viewport_window.visible = True
            except Exception:
                pass

            # Preservar dimensiones válidas (>100px) o restaurar tamaño por defecto (640x360) si se colapsó
            w, h = 640.0, 360.0
            try:
                cur_w = float(self.viewport_window.width)
                cur_h = float(self.viewport_window.height)
                if cur_w >= 100.0:
                    w = cur_w
                if cur_h >= 100.0:
                    h = cur_h
            except Exception:
                pass

            try:
                self.viewport_window.title = self.viewport_window_title
            except Exception:
                pass

            try:
                self.viewport_window.width = w
                self.viewport_window.height = h
            except Exception:
                pass

            try:
                self.viewport_window.focus()
            except Exception:
                pass
        else:
            try:
                from omni.kit.viewport.utility import create_viewport_window
                self.viewport_window = create_viewport_window(
                    name=self.viewport_window_title,
                    width=640,
                    height=360,
                    camera_path=self.camera_path,
                )
                if self.viewport_window:
                    try:
                        self.viewport_window.width = 640.0
                        self.viewport_window.height = 360.0
                    except Exception:
                        pass
            except Exception as e:
                self.logger.warning(f"Error al instanciar create_viewport_window: {e}")
                self.viewport_window = None

        if self.viewport_window and hasattr(self.viewport_window, "viewport_api") and self.viewport_window.viewport_api:
            try:
                self.viewport_window.viewport_api.camera_path = self.camera_path
            except Exception:
                try:
                    self.viewport_window.viewport_api.set_active_camera(self.camera_path)
                except Exception:
                    pass

        # Aplicar limpieza estética sobre el viewport secundario (sin alterar dimensiones)
        if self.viewport_window:
            self._clean_viewport_window(self.viewport_window)

    def _clean_viewport_window(self, win) -> None:
        """
        Limpia el Viewport secundario de la cámara de seguimiento:
          - Quita la barra superior de herramientas/menús (Menubar).
          - Quita el cuadro de estadísticas HUD (FPS, tiempo de frame, memoria GPU, memoria de proceso, resolución).
          - Quita el eje de coordenadas XYZ y el indicador de unidades en metros [m] (Axis y Grid).
          - Activa fill_frame para que la escena 3D ocupe todo el marco de la ventana sin bandas negras (letterboxing).
        """
        if not win:
            return

        # 1. Ajustar el render al tamaño exacto de la ventana (sin bandas negras)
        try:
            if hasattr(win, "viewport_widget") and win.viewport_widget:
                win.viewport_widget.fill_frame = True
        except Exception:
            pass
        try:
            if hasattr(win, "viewport_api") and win.viewport_api:
                win.viewport_api.fill_frame = True
        except Exception:
            pass

        # 2. Configurar carb.settings específico para este Viewport
        try:
            import carb.settings
            settings = carb.settings.get_settings()
            vp_id = str(win.viewport_api.id) if hasattr(win, "viewport_api") and win.viewport_api else ""

            if vp_id:
                # Ocultar HUD y estadísticas (FPS, resolución, memoria, etc.)
                for hud_key in [
                    "visible",
                    "renderFPS/visible",
                    "renderResolution/visible",
                    "processMemory/visible",
                    "deviceMemory/visible",
                    "hostMemory/visible",
                    "cameraSpeed/visible",
                ]:
                    settings.set(f"/persistent/app/viewport/{vp_id}/hud/{hud_key}", False)
                    settings.set(f"/app/viewport/{vp_id}/hud/{hud_key}", False)

                # Ocultar Ejes, cuadrícula, origen y guías
                for guide_key in [
                    "axis/visible",
                    "grid/visible",
                    "origin/visible",
                    "boundingBox/visible",
                    "selection/visible",
                ]:
                    settings.set(f"/persistent/app/viewport/{vp_id}/guide/{guide_key}", False)
                    settings.set(f"/app/viewport/{vp_id}/guide/{guide_key}", False)

                # Desactivar la barra flotante de línea de tiempo (minibar) en este Viewport
                settings.set(f"/app/viewport/{vp_id}/TimelineMinibar", False)
                settings.set(f"/persistent/app/viewport/{vp_id}/TimelineMinibar", False)

                # Desactivar el indicador de unidades del escenario [m] (MetricsAssemblerUnitsView)
                for ma_key in [
                    "hud/metrics/assembler/visible",
                    "metricsAssembler/visible",
                ]:
                    settings.set(f"/persistent/app/viewport/{vp_id}/{ma_key}", False)
                    settings.set(f"/app/viewport/{vp_id}/{ma_key}", False)
                # Restablecer altura de capa CameraAxis
                settings.set(f"/app/viewport/{vp_id}/guide/axis/size/1", 60.0)
        except Exception as e:
            self.logger.debug(f"Error configurando carb.settings de viewport: {e}")

        # 3. Acciones de viewport de omni.kit para axis y grid
        try:
            from omni.kit.viewport.actions.actions import toggle_axis_visibility, toggle_grid_visibility
            if hasattr(win, "viewport_api") and win.viewport_api:
                toggle_axis_visibility(win.viewport_api, False)
                toggle_grid_visibility(win.viewport_api, False)
        except Exception:
            pass

        # 4. Ocultar capas directamente por ID / categoría
        layer_targets = [
            ("Menubar", "menubar"),
            ("Menubar", "menu"),
            ("Menubar", None),
            ("All Stats", "stats"),
            ("All Stats", None),
            ("Viewport HUD", "stats"),
            ("Viewport HUD", None),
            ("Axis", "guide"),
            ("Axis", None),
            ("Grid", "guide"),
            ("Grid", None),
            ("Origin", "guide"),
            ("Origin", None),
            ("Timeline", None),
            ("Timeline", "timeline"),
            ("TimelineMinibar", None),
            ("TimelineMinibarScene", None),
            ("timeline_minibar_frame", None),
            ("MetricsAssembler", "hud"),
            ("MetricsAssembler", None),
            ("omni.usd.metrics.assembler.ui.MetricsAssemblerUnitsView", None),
        ]
        for name, cat in layer_targets:
            try:
                layer = win._find_viewport_layer(name, cat)
                if layer and hasattr(layer, "visible"):
                    layer.visible = False
            except Exception:
                pass

        # 5. Ocultar capas de ViewportLayers inspeccionando instancias directas (incluyendo TimelineMinibarScene y MetricsAssemblerUnitsView)
        try:
            vp_layers_obj = getattr(win, "_ViewportWindow__viewport_layers", None)
            if vp_layers_obj is not None:
                layers_dict = getattr(vp_layers_obj, "_ViewportLayers__viewport_layers", {})
                for factory, instance in layers_dict.items():
                    inst_type = str(type(instance))
                    fact_str = str(factory)
                    suppress_terms = ["Timeline", "minibar", "MetricsAssembler", "metrics"]
                    if any(term in inst_type or term in fact_str for term in suppress_terms):
                        if hasattr(instance, "visible"):
                            instance.visible = False
        except Exception:
            pass

        # 6. Ocultar frames si están registrados
        for frame_name in [
            "menubar",
            "menu",
            "timeline",
            "minibar",
            "timeline_minibar_frame",
            "omni.usd.metrics.assembler.ui.MetricsAssemblerUnitsView",
            "MetricsAssembler",
        ]:
            try:
                f = win.get_frame(frame_name)
                if f:
                    f.visible = False
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

    def destroy(self) -> None:
        """Cierra el control, el viewport secundario y el subsistema de joystick."""
        self.stop()
        self.destroy_camera()
        if hasattr(self, "joystick") and self.joystick:
            self.joystick.destroy()

    @property
    def invert_camera_movement(self) -> bool:
        return self.invert_camera_vertical

    @invert_camera_movement.setter
    def invert_camera_movement(self, val: bool):
        self.invert_camera_vertical = val

    @property
    def invert_yaw(self) -> bool:
        return self.invert_camera_horizontal

    @invert_yaw.setter
    def invert_yaw(self, val: bool):
        self.invert_camera_horizontal = val

    def move_camera(
        self, zoom_in: float, zoom_out: float, yaw: float, pitch: float
    ) -> None:
        """Ajusta zoom y orientación angular de la cámara de persecución."""
        if not self.camera_prim or not self.camera_prim.IsValid():
            return

        # Sincronizar si el usuario navegó / hizo scroll con el ratón directamente en el viewport
        try:
            xf = UsdGeom.Xformable(self.camera_prim)
            local_trans = xf.GetLocalTransformation(Usd.TimeCode.Default()).ExtractTranslation()
            yaw_rad = math.radians(self.cam_yaw_deg)
            pitch_rad = math.radians(self.cam_pitch_deg)
            expected_eye_x = -self.cam_distance * math.cos(yaw_rad) * math.cos(pitch_rad)
            expected_eye_y = -self.cam_distance * math.sin(yaw_rad) * math.cos(pitch_rad)
            expected_eye_z = self.cam_height + self.cam_distance * math.sin(pitch_rad)
            drift = math.sqrt(
                (local_trans[0] - expected_eye_x) ** 2
                + (local_trans[1] - expected_eye_y) ** 2
                + (local_trans[2] - expected_eye_z) ** 2
            )
            if drift > 0.15:
                cos_p = math.cos(pitch_rad)
                if cos_p > 0.1:
                    horiz_dist = math.sqrt(local_trans[0] ** 2 + local_trans[1] ** 2)
                    self.cam_distance = horiz_dist / cos_p
                else:
                    target_z = getattr(self, "cam_target_z", 0.0)
                    self.cam_distance = math.sqrt(
                        local_trans[0] ** 2 + local_trans[1] ** 2 + (local_trans[2] - target_z) ** 2
                    )
        except Exception:
            pass

        # Control independiente de inversión vertical (pitch) y horizontal (yaw)
        if not self.invert_camera_vertical:
            pitch *= -1

        if not self.invert_camera_horizontal:
            yaw *= -1

        dist_delta = zoom_in + zoom_out
        min_dist = getattr(self, "min_cam_distance", 0.4)
        max_dist = getattr(self, "max_cam_distance", 100.0)
        self.cam_distance = float(np.clip(self.cam_distance + dist_delta * 0.4, min_dist, max_dist))
        self.cam_yaw_deg += yaw * 3.0
        self.cam_pitch_deg = float(np.clip(self.cam_pitch_deg + pitch * 2.0, -30.0, 60.0))

        self._apply_camera_matrix(self.camera_prim)