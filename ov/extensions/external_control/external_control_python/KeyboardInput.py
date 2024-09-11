import omni.appwindow
import carb.input
from carb.input import KeyboardEventType

class KeyboardInput:
    def __init__(self):
        #Get Keyboard
        self.app_window = omni.appwindow.get_default_app_window()
        self.keyboard = self.app_window.get_keyboard()
        self.input = carb.input.acquire_input_interface()

    def start(self):
        self.keyboard_sub_id = self.input.subscribe_to_keyboard_events(self.keyboard, self.on_keyboard_input)

        self.inputs = [0,0,0,0,0,0,0,0,0]
        self.acc = 0.03
    

    def stop(self):
        self.input.unsubscribe_to_keyboard_events(self.keyboard, self.keyboard_sub_id)


    def on_keyboard_input(self, e: carb.input.KeyboardEvent):
        # Fordward
        if e.input == carb.input.KeyboardInput.W:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[0] = -0.1
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[0] > -1:
                    self.inputs[0] -= self.acc
            else:
                self.inputs[0] = 0.0

        # Backwards
        if e.input == carb.input.KeyboardInput.S:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[0] = 0.1
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[0] < 1:
                    self.inputs[0] += self.acc
            else:
                self.inputs[0] = 0.0

        # Leftside
        if e.input == carb.input.KeyboardInput.A:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[1] = -0.1
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[1] > -1:
                    self.inputs[1] -= self.acc
            else:
                self.inputs[1] = 0.0

        # Rightside
        if e.input == carb.input.KeyboardInput.D:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[1] = 0.1
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[1] < 1:
                    self.inputs[1] += self.acc
            else:
                self.inputs[1] = 0.0

        # Up
        if e.input == carb.input.KeyboardInput.UP:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[2] = -0.1
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[2] > -1:
                    self.inputs[2] -= self.acc
            else:
                self.inputs[2] = 0.0

        # Down
        if e.input == carb.input.KeyboardInput.DOWN:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[2] = 0.1
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[2] < 1:
                    self.inputs[2] += self.acc
            else:
                self.inputs[2] = 0.0

        # Turn left
        if e.input == carb.input.KeyboardInput.LEFT:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[3] = -0.25
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[3] > -1:
                    self.inputs[3] -= self.acc
            else:
                self.inputs[3] = 0.0

        # Turn right
        if e.input == carb.input.KeyboardInput.RIGHT:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[3] = 0.25
            elif e.type == KeyboardEventType.KEY_REPEAT:
                if self.inputs[3] < 1:
                    self.inputs[3] += self.acc
            else:
                self.inputs[3] = 0.0

        # ON/OFF
        if e.input == carb.input.KeyboardInput.SPACE:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[4] = 1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[4] = 0
            

        # CAMERA
        # -1 active camera
        if e.input == carb.input.KeyboardInput.NUMPAD_SUBTRACT:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[5] = -1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[5] = 0

        # +1 active camera
        if e.input == carb.input.KeyboardInput.NUMPAD_ADD:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[6] = 1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[6] = 0

        # Follow distance
        if e.input == carb.input.KeyboardInput.NUMPAD_4:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[7] = -1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[7] = 0

        # Follow distance
        if e.input == carb.input.KeyboardInput.NUMPAD_6:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[7] = 1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[7] = 0

        # Follow height
        if e.input == carb.input.KeyboardInput.NUMPAD_8:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[8] = 1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[8] = 0

        # Follow height
        if e.input == carb.input.KeyboardInput.NUMPAD_2:
            if e.type == KeyboardEventType.KEY_PRESS:
                self.inputs[8] = -1
            elif e.type == KeyboardEventType.KEY_RELEASE:
                self.inputs[8] = 0