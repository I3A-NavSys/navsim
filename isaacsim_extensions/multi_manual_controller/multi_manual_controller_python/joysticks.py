import warnings
import numpy as np
import logging
import carb

# Suppress setuptools/pkg_resources deprecation warning from pygame internals
warnings.filterwarnings("ignore", message=".*pkg_resources is deprecated.*")

try:
    import pygame
except ImportError:
    pygame = None


# Deadband threshold to ignore micro-drifts and avoid expensive rounding calls
JOYSTICK_DEADBAND = 0.04


class Joysticks:
    """Manages physical joysticks using pygame with low-overhead event handling and polling."""

    def __init__(self):
        self.joysticks_inputs: dict[int, np.ndarray] = {}
        self.joysticks: list = []
        self.joysticks_ids: list[int] = []
        self.logger = logging.getLogger("A_joy")
        self.initialized = False

    def start(self):
        """Initializes pygame joystick subsystem and enumerates already connected devices."""
        if pygame is None:
            carb.log_error("[Joysticks] pygame module is not installed or available.")
            return

        if not self.initialized:
            self.initialized = True
            pygame.init()
            pygame.joystick.init()
            self._enumerate_existing_joysticks()

    def _enumerate_existing_joysticks(self):
        """Enumerate any joystick hardware already connected when the extension starts."""
        self.joysticks.clear()
        self.joysticks_ids.clear()
        self.joysticks_inputs.clear()

        count = pygame.joystick.get_count()
        for device_idx in range(count):
            try:
                joy = pygame.joystick.Joystick(device_idx)
                joy.init()
                joy_id = joy.get_id()
                self.joysticks.append(joy)
                self.joysticks_ids.append(joy_id)
                self.joysticks_inputs[joy_id] = np.zeros(6, dtype=np.float32)
            except Exception as e:
                carb.log_warn(f"[Joysticks] Failed to initialize joystick device {device_idx}: {e}")

    def stop(self):
        """Shuts down joystick subsystem and frees all device references."""
        if self.initialized:
            self.initialized = False
            if pygame is not None:
                try:
                    for joy in self.joysticks:
                        if joy.get_init():
                            joy.quit()
                    pygame.joystick.quit()
                    pygame.quit()
                except Exception as e:
                    carb.log_warn(f"[Joysticks] Error stopping pygame joysticks: {e}")

        self.joysticks.clear()
        self.joysticks_ids.clear()
        self.joysticks_inputs.clear()

    def get_inputs(self) -> tuple[list[int], dict[int, np.ndarray]]:
        """
        Polls joystick events and updates inputs.
        
        Returns:
            Tuple of (list of connected joystick IDs, dictionary of joystick ID to input array).
            The array contains [axis_pitch, axis_roll, axis_altitude, axis_yaw, button_boost, button_reset].
        """
        if not self.initialized or pygame is None:
            return self.joysticks_ids, self.joysticks_inputs

        # Process hardware connection/disconnection events
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                try:
                    joy = pygame.joystick.Joystick(event.device_index)
                    joy.init()
                    joy_id = joy.get_id()
                    if joy_id not in self.joysticks_ids:
                        self.joysticks.append(joy)
                        self.joysticks_ids.append(joy_id)
                        self.joysticks_inputs[joy_id] = np.zeros(6, dtype=np.float32)
                except Exception as e:
                    carb.log_warn(f"[Joysticks] Failed to add joystick: {e}")

            elif event.type == pygame.JOYDEVICEREMOVED:
                # Handle disconnection cleanly to prevent accessing invalid device pointers
                removed_instance_id = event.instance_id
                matched_idx = -1
                for idx, joy in enumerate(self.joysticks):
                    try:
                        if joy.get_instance_id() == removed_instance_id:
                            matched_idx = idx
                            break
                    except Exception:
                        pass

                if matched_idx >= 0:
                    joy = self.joysticks.pop(matched_idx)
                    joy_id = self.joysticks_ids.pop(matched_idx)
                    self.joysticks_inputs.pop(joy_id, None)
                    try:
                        joy.quit()
                    except Exception:
                        pass

        # Poll axis and button states into pre-allocated NumPy buffers
        for joy, joy_id in zip(self.joysticks, self.joysticks_ids):
            buf = self.joysticks_inputs.get(joy_id)
            if buf is None:
                buf = np.zeros(6, dtype=np.float32)
                self.joysticks_inputs[joy_id] = buf

            num_axes = joy.get_numaxes()
            num_buttons = joy.get_numbuttons()

            # Axis reading with deadband filter to eliminate noise without expensive round() calls
            a1 = joy.get_axis(1) if num_axes > 1 else 0.0
            a0 = joy.get_axis(0) if num_axes > 0 else 0.0
            a3 = joy.get_axis(3) if num_axes > 3 else 0.0
            a2 = joy.get_axis(2) if num_axes > 2 else 0.0

            buf[0] = a1 if abs(a1) >= JOYSTICK_DEADBAND else 0.0
            buf[1] = a0 if abs(a0) >= JOYSTICK_DEADBAND else 0.0
            buf[2] = a3 if abs(a3) >= JOYSTICK_DEADBAND else 0.0
            buf[3] = a2 if abs(a2) >= JOYSTICK_DEADBAND else 0.0
            buf[4] = float(joy.get_button(0)) if num_buttons > 0 else 0.0
            buf[5] = float(joy.get_button(1)) if num_buttons > 1 else 0.0

        return self.joysticks_ids, self.joysticks_inputs
