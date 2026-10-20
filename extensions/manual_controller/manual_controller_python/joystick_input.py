import asyncio
import logging

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
        self.logger = logging.getLogger("JoystickInput")

    def start(self) -> None:
        self._stop = False
        if not PYGAME_AVAILABLE or pygame is None:
            self.logger.warning("pygame no está disponible en este entorno. El control por joystick está deshabilitado; use teclado.")
            self.joystick_name = "Pygame no disponible"
            return

        try:
            if not pygame.get_init():
                pygame.init()
            if not pygame.joystick.get_init():
                pygame.joystick.init()
            self._detect_joystick()
        except Exception as e:
            self.logger.warning(f"Error al inicializar pygame.joystick: {e}")

        asyncio.ensure_future(self.input_loop())

    def stop(self) -> None:
        self._stop = True
        self.event.set()
        try:
            if pygame.joystick.get_init():
                pygame.joystick.quit()
        except Exception:
            pass
        self.joystick = None
        self.joystick_name = "Desconectado"
        self.inputs = [0.0] * 9

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
        self.event.set()

    def _apply_deadzone(self, val: float) -> float:
        return 0.0 if abs(val) < self.deadzone else round(float(val), 2)

    async def input_loop(self) -> None:
        while not self._stop:
            await self.event.wait()
            if self._stop:
                break

            if pygame is not None:
                try:
                    for event in pygame.event.get():
                        if event.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
                            self._detect_joystick()
                except Exception:
                    pass

            if self.joystick is not None:
                try:
                    num_axes = self.joystick.get_numaxes()
                    num_buttons = self.joystick.get_numbuttons()
                    num_hats = self.joystick.get_numhats()

                    # Axis 1: Pitch (eje cuerpo hacia adelante)
                    axis_pitch = self.joystick.get_axis(1) if num_axes > 1 else 0.0
                    self.inputs[0] = self._apply_deadzone(-axis_pitch)

                    # Axis 0: Roll (eje cuerpo lateral izq/dcha)
                    axis_roll = self.joystick.get_axis(0) if num_axes > 0 else 0.0
                    self.inputs[1] = self._apply_deadzone(axis_roll)

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

            self.event.clear()