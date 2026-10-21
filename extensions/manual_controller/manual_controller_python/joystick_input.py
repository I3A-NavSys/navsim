import asyncio
import logging
from typing import List

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    pygame = None
    PYGAME_AVAILABLE = False


class JoystickInput:
    """
    Gestión de lectura de entradas analógicas (ejes), botones y cruceta (hat)
    de un joystick conectado mediante Pygame.
    """

    def __init__(self, deadzone: float = 0.05):
        # Formato de inputs:
        # [0]: Pitch (velX)  [-1.0 .. 1.0]
        # [1]: Roll  (velY)  [-1.0 .. 1.0]
        # [2]: Throttle (velZ) [-1.0 .. 1.0]
        # [3]: Yaw rotZ      [-1.0 .. 1.0]
        # [4]: Botón 0 (Armado / Toggle manual)
        # [5]: Botón 4 (Zoom cámara - disminuir distancia)
        # [6]: Botón 5 (Zoom cámara - aumentar distancia)
        # [7]: Hat X (Rotación Yaw cámara)
        # [8]: Hat Y (Rotación Pitch cámara)
        self.inputs = [0.0] * 9
        self.deadzone = deadzone
        self.event = asyncio.Event()
        self._stop = True
        self.joystick = None
        self.joystick_name = "Desconectado"
        self.button_states: List[bool] = [False] * 12
        self.logger = logging.getLogger("JoystickInput")

    def _ensure_init(self) -> None:
        """Inicializa pygame y el subsistema de joystick si aún no se ha hecho."""
        if not PYGAME_AVAILABLE or pygame is None:
            return
        try:
            if not pygame.get_init():
                pygame.init()
            if not pygame.joystick.get_init():
                pygame.joystick.init()
            if self.joystick is None:
                self._detect_joystick()
        except Exception as e:
            self.logger.warning(f"Error al inicializar pygame.joystick: {e}")

    def poll(self) -> None:
        """
        Sondea el joystick (ejes analógicos, botones y cruceta/hat) mediante pygame.
        Actualiza self.inputs y self.button_states en tiempo real, incluso cuando
        el bucle de vuelo asíncrono no está activo.
        """
        if not PYGAME_AVAILABLE or pygame is None:
            return

        self._ensure_init()
        if self.joystick is None:
            try:
                if pygame.joystick.get_count() > 0:
                    self._detect_joystick()
            except Exception:
                pass

        if self.joystick is None:
            return

        try:
            pygame.event.pump()
            num_axes = self.joystick.get_numaxes()
            num_buttons = self.joystick.get_numbuttons()
            num_hats = self.joystick.get_numhats()

            self.button_states = [
                bool(self.joystick.get_button(i)) if i < num_buttons else False
                for i in range(12)
            ]

            # Axis 1: Pitch (eje cuerpo hacia adelante)
            axis_pitch = self.joystick.get_axis(1) if num_axes > 1 else 0.0
            self.inputs[0] = self._apply_deadzone(-axis_pitch)

            # Axis 0: Roll (eje cuerpo lateral izq/dcha - invertido para que inclinación a la izquierda sea +Y/izquierda)
            axis_roll = self.joystick.get_axis(0) if num_axes > 0 else 0.0
            self.inputs[1] = self._apply_deadzone(-axis_roll)

            # Throttle / Slider Z (subir / bajar)
            if num_axes > 3:
                axis_z = self.joystick.get_axis(3)
            elif num_axes > 2:
                axis_z = self.joystick.get_axis(2)
            else:
                axis_z = 0.0
            self.inputs[2] = self._apply_deadzone(-axis_z)

            # Yaw (rotación guiñada - invertido: giro a la izquierda rota a la izquierda)
            axis_yaw = self.joystick.get_axis(2) if num_axes > 3 else 0.0
            self.inputs[3] = self._apply_deadzone(-axis_yaw)

            # Botones de control
            self.inputs[4] = float(self.joystick.get_button(0)) if num_buttons > 0 else 0.0
            self.inputs[5] = -1.0 if (num_buttons > 4 and self.joystick.get_button(4)) else 0.0
            self.inputs[6] = 1.0 if (num_buttons > 5 and self.joystick.get_button(5)) else 0.0

            # Cruceta / Hat para órbita de cámara
            if num_hats > 0:
                hat = self.joystick.get_hat(0)
                self.inputs[7] = float(hat[0])
                self.inputs[8] = float(hat[1])
            else:
                self.inputs[7] = 0.0
                self.inputs[8] = 0.0
        except Exception:
            pass

    def get_buttons_state(self, count: int = 12) -> List[bool]:
        """Obtiene el estado booleano de los primeros `count` botones del joystick en tiempo real."""
        self.poll()
        if len(self.button_states) >= count:
            return self.button_states[:count]
        return self.button_states + [False] * (count - len(self.button_states))

    get_button_states = get_buttons_state

    def start(self) -> None:
        self._stop = False
        if not PYGAME_AVAILABLE or pygame is None:
            self.logger.warning("pygame no está disponible en este entorno. El control por joystick está deshabilitado; use teclado.")
            self.joystick_name = "Pygame no disponible"
            return

        self._ensure_init()
        asyncio.ensure_future(self.input_loop())

    def stop(self) -> None:
        """Detiene el bucle de consignas pero mantiene el dispositivo activo para test de botones y monitor."""
        self._stop = True
        self.event.set()

    def destroy(self) -> None:
        """Cierra y libera por completo el subsistema de joystick."""
        self.stop()
        try:
            if PYGAME_AVAILABLE and pygame is not None and pygame.joystick.get_init():
                pygame.joystick.quit()
        except Exception:
            pass
        self.joystick = None
        self.joystick_name = "Desconectado"

    def _detect_joystick(self) -> None:
        count = pygame.joystick.get_count()
        if count > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.joystick_name = self.joystick.get_name()
            self.logger.info(f"Joystick detectado: {self.joystick_name}")
        else:
            self.joystick = None
            self.joystick_name = "Ninguno detectado"

    def ask_input(self) -> None:
        self.poll()
        self.event.set()

    def _apply_deadzone(self, val: float) -> float:
        return 0.0 if abs(val) < self.deadzone else round(float(val), 2)

    async def input_loop(self) -> None:
        while not self._stop:
            await self.event.wait()
            if self._stop:
                break
            self.poll()
            self.event.clear()