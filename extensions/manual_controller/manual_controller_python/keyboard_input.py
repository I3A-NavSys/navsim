import omni.appwindow
import carb.input
from carb.input import KeyboardEventType

class KeyboardInput:
    def __init__(self):
        self.app_window = omni.appwindow.get_default_app_window()
        self.keyboard = self.app_window.get_keyboard() if self.app_window else None
        self.input = carb.input.acquire_input_interface()
        self.keyboard_sub_id = None
        self.inputs = [0.0] * 9
        self.acc = 0.05

    def start(self):
        if self.keyboard_sub_id is not None:
            return
        if self.keyboard and self.input:
            self.keyboard_sub_id = self.input.subscribe_to_keyboard_events(
                self.keyboard, self.on_keyboard_input
            )
        self.inputs = [0.0] * 9
        self.acc = 0.05

    def stop(self):
        if getattr(self, "keyboard_sub_id", None) is not None:
            try:
                self.input.unsubscribe_to_keyboard_events(self.keyboard, self.keyboard_sub_id)
            except Exception:
                pass
            self.keyboard_sub_id = None
        self.inputs = [0.0] * 9

    def on_keyboard_input(self, e: carb.input.KeyboardEvent):
        # Forward (W -> +X vel)
        if e.input == carb.input.KeyboardInput.W:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[0] = 0.2
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[0] < 1.0:
                    self.inputs[0] += self.acc
            else:
                self.inputs[0] = 0.0

        # Backwards (S -> -X vel)
        if e.input == carb.input.KeyboardInput.S:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[0] = -0.2
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[0] > -1.0:
                    self.inputs[0] -= self.acc
            else:
                self.inputs[0] = 0.0

        # Left (A -> +Y vel)
        if e.input == carb.input.KeyboardInput.A:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[1] = 0.2
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[1] < 1.0:
                    self.inputs[1] += self.acc
            else:
                self.inputs[1] = 0.0

        # Right (D -> -Y vel)
        if e.input == carb.input.KeyboardInput.D:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[1] = -0.2
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[1] > -1.0:
                    self.inputs[1] -= self.acc
            else:
                self.inputs[1] = 0.0

        # Up (UP -> +Z vel)
        if e.input == carb.input.KeyboardInput.UP:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[2] = 0.2
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[2] < 1.0:
                    self.inputs[2] += self.acc
            else:
                self.inputs[2] = 0.0

        # Down (DOWN -> -Z vel)
        if e.input == carb.input.KeyboardInput.DOWN:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[2] = -0.2
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[2] > -1.0:
                    self.inputs[2] -= self.acc
            else:
                self.inputs[2] = 0.0

        # Turn left (LEFT -> +wz)
        if e.input == carb.input.KeyboardInput.LEFT:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[3] = 0.3
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[3] < 1.0:
                    self.inputs[3] += self.acc
            else:
                self.inputs[3] = 0.0

        # Turn right (RIGHT -> -wz)
        if e.input == carb.input.KeyboardInput.RIGHT:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[3] = -0.3
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[3] > -1.0:
                    self.inputs[3] -= self.acc
            else:
                self.inputs[3] = 0.0

        # ON/OFF (SPACE -> Arm/Disarm)
        if e.input == carb.input.KeyboardInput.SPACE:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[4] = 1.0
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[4] = 0.0
            

        # CAMERA NUM_PAD
        # Zoom in
        if e.input == carb.input.KeyboardInput.NUMPAD_SUBTRACT:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[5] = -1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[5] = 0

        # Zoom out
        if e.input == carb.input.KeyboardInput.NUMPAD_ADD:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[6] = 1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[6] = 0

        # Negative yaw
        if e.input == carb.input.KeyboardInput.NUMPAD_4:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[7] = -1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[7] = 0

        # Positive yaw
        if e.input == carb.input.KeyboardInput.NUMPAD_6:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[7] = 1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[7] = 0

        # Positive pitch
        if e.input == carb.input.KeyboardInput.NUMPAD_8:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[8] = 1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[8] = 0

        # Negative pitch
        if e.input == carb.input.KeyboardInput.NUMPAD_2:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[8] = -1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[8] = 0

        # CAMERA NOT NUM_PAD
        # Zoom in
        if e.input == carb.input.KeyboardInput.LEFT_CONTROL:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[5] = -1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[5] = 0

        # Zoom out
        if e.input == carb.input.KeyboardInput.RIGHT_CONTROL:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[6] = 1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[6] = 0

        # Negative yaw
        if e.input == carb.input.KeyboardInput.J:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[7] = -1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[7] = 0

        # Positive yaw
        if e.input == carb.input.KeyboardInput.L:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[7] = 1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[7] = 0

        # Positive pitch
        if e.input == carb.input.KeyboardInput.I:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[8] = 1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[8] = 0

        # Negative pitch
        if e.input == carb.input.KeyboardInput.K:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[8] = -1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[8] = 0